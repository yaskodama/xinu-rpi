/* apps/aipl_remote.c — AIPL の remote("host:port","actor") を Pi 3 で実現する。
 *
 * Pi 4 / Pi 5 は TCP が受け側しか無いのでフレームを自作したが、この板には
 * 本物の Xinu ネットワーク層（UDP デバイス）がある。素直にそれを使う。
 * 電文は 3 台で共通の ASCII 一行:
 *
 *   要求  Q <reqid> <actor> <method> <arg...>\n
 *   応答  R <reqid> <値>\n
 *
 * 受け口は UDP/9010 に常駐する番人プロセス。送り側は別のポートを取って
 * 出す（同じ局所ポートに 2 本開くと振り分けが曖昧になるため）。相手は
 * 要求の送信元ポートへ返すので、これで噛み合う。
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <device.h>
#include <network.h>
#include <udp.h>
#include <thread.h>
#include <ether.h>

#define AIPL_PORT 9010

extern int vm_remote_call(const char *path, const char *meth, const char *arg,
                          char *out, int cap);
extern syscall sleep(unsigned);

/* ---- 電文の組み立て・取り出し -------------------------------------------- */
static int put(char *d, int at, int cap, const char *s)
{ while (*s && at < cap - 1) d[at++] = *s++; d[at] = 0; return at; }

static int build_q(char *out, int cap, int id, const char *actor,
                   const char *meth, const char *arg)
{
    int at = 0;
    at = put(out, at, cap, "Q ");
    { char n[16]; sprintf(n, "%d", id); at = put(out, at, cap, n); }
    at = put(out, at, cap, " ");
    at = put(out, at, cap, actor);
    at = put(out, at, cap, " ");
    at = put(out, at, cap, meth);
    at = put(out, at, cap, " ");
    at = put(out, at, cap, arg ? arg : "");
    at = put(out, at, cap, "\n");
    return at;
}

/* "192.168.3.101" / "192.168.3.101:9010" を解く */
static int parse_hostport(const char *h, struct netaddr *ip, ushort *port)
{
    char host[32]; int i = 0, p = 0;
    *port = AIPL_PORT;
    while (h[i] && h[i] != ':' && i < (int)sizeof host - 1) { host[i] = h[i]; i++; }
    host[i] = 0;
    if (h[i] == ':') { i++; while (h[i] >= '0' && h[i] <= '9') { p = p*10 + (h[i]-'0'); i++; }
                       if (p > 0 && p < 65536) *port = (ushort)p; }
    return (SYSERR == dot2ipv4(host, ip)) ? -1 : 0;
}

/* ---- 応答の控え（at-most-once） ------------------------------------------
   要求は再送する（UDP なので落ちる）。素朴に作ると相手のメソッドが二度走る。
   (送り主 IP, reqid) で最後の答えを覚え、同じものが来たら実行せずに返す。 */
#define ANS_MAX 4
static struct { uchar ip[4]; int id; char val[192]; uchar used; } g_ans[ANS_MAX];
static int g_ans_next;

static const char *ans_lookup(const uchar *ip, int id)
{
    int i;
    for (i = 0; i < ANS_MAX; i++)
        if (g_ans[i].used && g_ans[i].id == id
         && g_ans[i].ip[0]==ip[0] && g_ans[i].ip[1]==ip[1]
         && g_ans[i].ip[2]==ip[2] && g_ans[i].ip[3]==ip[3])
            return g_ans[i].val;
    return 0;
}
static void ans_store(const uchar *ip, int id, const char *val)
{
    int i = g_ans_next, k;
    g_ans_next = (g_ans_next + 1) % ANS_MAX;
    for (k = 0; k < 4; k++) g_ans[i].ip[k] = ip[k];
    g_ans[i].id = id;
    for (k = 0; val[k] && k < (int)sizeof g_ans[0].val - 1; k++) g_ans[i].val[k] = val[k];
    g_ans[i].val[k] = 0;
    g_ans[i].used = 1;
}

/* ---- 計器 ----------------------------------------------------------------
   番人が黙っているとき、どこまで進んだのかを外から読む。
   「板が生きている」ことと「電文が通っている」ことは別なので、
   推測でコードを直す前にここを見る。GET /version が出す。 */
static long g_state = 0;   /* 0=未起動 1=udpAlloc失敗 2=open失敗 3=待ち受け中 */
static unsigned long g_n_read, g_n_short, g_n_notq, g_n_q, g_n_reply, g_n_tx_q;
static long g_last_n = -1, g_last_plen = -1, g_last_c0 = -1;
void aipl_remote_stats(long *o)
{ o[0]=g_state; o[1]=(long)g_n_read; o[2]=(long)g_n_short; o[3]=(long)g_n_notq;
  o[4]=(long)g_n_q; o[5]=(long)g_n_reply; o[6]=(long)g_n_tx_q;
  o[7]=g_last_n; o[8]=g_last_plen; o[9]=g_last_c0; }

/* ---- 送り側 -------------------------------------------------------------- */
static int g_next_id = 1;

static int remote_xfer(const char *hostport, const char *actor, const char *meth,
                       const char *arg, int timeout_ms, char *out, int cap)
{
    struct netaddr rip;
    ushort rport;
    int dev, id, n, waited = 0, rc = -1;
    char q[288];
    static char rbuf[512];

    if (out && cap > 0) out[0] = 0;
    if (parse_hostport(hostport, &rip, &rport) != 0) return -1;

    dev = udpAlloc();
    if (SYSERR == dev) return -1;
    /* 局所ポート 0 = 任意。9010 を取らないのは、受け口の番人と食い合うため。 */
    if (SYSERR == open(dev, &netiftab[0].ip, &rip, 0, rport)) {
        udptab[dev - UDP0].state = UDP_FREE; return -1;
    }

    id = g_next_id++;
    if (g_next_id > 1000000) g_next_id = 1;
    n = build_q(q, sizeof q, id, actor, meth, arg);
    if (SYSERR == write(dev, q, n)) goto out_close;
    g_n_tx_q++;

    if (timeout_ms <= 0) { rc = 0; goto out_close; }   /* 返事を待たない送信 */

    /* ★ UDP の要求応答なので再送する。落ちた一発をそのまま待てば期限切れに
       なる。相手は (送り主, reqid) の控えを持つので二度は走らない。 */
    control(dev, UDP_CTRL_SETFLAG, UDP_FLAG_NOBLOCK, 0);
    { int since_tx = 0;
    while (waited < timeout_ms) {
        if (since_tx >= 200) { write(dev, q, n); since_tx = 0; }
        n = read(dev, rbuf, sizeof rbuf - 1);
        if (n > 0) {
            rbuf[n] = 0;
            { int i = 0; while (rbuf[i]) { if (rbuf[i]=='\n'||rbuf[i]=='\r') { rbuf[i]=0; break; } i++; } }
            if (rbuf[0] == 'R' && rbuf[1] == ' ') {
                int p = 2, gotid = 0;
                while (rbuf[p] >= '0' && rbuf[p] <= '9') { gotid = gotid*10 + (rbuf[p]-'0'); p++; }
                if (rbuf[p] == ' ') p++;
                if (gotid == id) {
                    int k = 0; while (rbuf[p] && k < cap - 1) out[k++] = rbuf[p++];
                    out[k] = 0; rc = 0; goto out_close;
                }
            }
            continue;                    /* 自分宛でない返事。読み飛ばす */
        }
        sleep(5); waited += 5; since_tx += 5;
    } }
    rc = -2;                             /* 期限切れ */
out_close:
    close(dev);
    return rc;
}

int aipl_remote_send(const char *hostport, const char *actor,
                     const char *meth, const char *arg)
{ return remote_xfer(hostport, actor, meth, arg, 0, NULL, 0); }

int aipl_remote_call(const char *hostport, const char *actor, const char *meth,
                     const char *arg, int timeout_ms, char *out, int cap)
{ return remote_xfer(hostport, actor, meth, arg, timeout_ms > 0 ? timeout_ms : 2000, out, cap); }

/* ===== メッシュの三つ（送り手） ============================================
 * 宛先表を持たない。自網の同報へ撒いて、返ってきた分だけを拾う。
 * 経路が変わっても、誰が消えても、ここは変わらない。
 *
 * ★ 受けと送りで装置を分ける。同報宛に結んだ装置で待つと、返事は相手の
 *   実 IP から来るので udpDemux が噛み合わない（remoteip が一致しない）。
 *   受けは「局所ポートだけ決めて相手は決めない」装置にする（DEST_MATCH）。
 */
#define MESH_RX_PORT 9011

static int mesh_open_tx(void)
{
    int dev = udpAlloc();
    if (SYSERR == dev) return -1;
    /* ★ 局所ポートを MESH_RX_PORT に固定する。以前は 0（任意）で開いていた。
       相手は「要求が来た送信元ポート」へ返すので、任意ポートで出すと返事は
       その場限りのポートに届き、9011 で待っている受信装置は永遠に見ない。
       計器がそう言った ―― Pi 4 の rx_q は Pi 3 の同報を 8 通数えているのに、
       Pi 3 の gather は 0 のままだった（＝出てはいる、返りが噛み合わない）。 */
    if (SYSERR == open(dev, &netiftab[0].ip, &netiftab[0].ipbrc,
                       MESH_RX_PORT, AIPL_PORT)) {
        udptab[dev - UDP0].state = UDP_FREE; return -1;
    }
    return dev;
}

/* 撒いて、期限まで拾う。kind の行だけを、送り主ごとに一つ集める。
 * 戻りは集まった数。out は stride 幅の並び。 */
static int mesh_run(const char *line, int len, int id, char kind, int ms,
                    char *out, int stride, int max)
{
    int txdev, rxdev, n, waited = 0, got = 0, since_tx = 0;
    uchar seen[8][4];
    static char rbuf[1024];

    rxdev = udpAlloc();
    if (SYSERR == rxdev) return 0;
    if (SYSERR == open(rxdev, &netiftab[0].ip, NULL, MESH_RX_PORT, 0)) {
        udptab[rxdev - UDP0].state = UDP_FREE; return 0;
    }
    control(rxdev, UDP_CTRL_SETFLAG, UDP_FLAG_PASSIVE, 0);
    control(rxdev, UDP_CTRL_SETFLAG, UDP_FLAG_NOBLOCK, 0);

    txdev = mesh_open_tx();
    if (txdev < 0) { close(rxdev); return 0; }
    write(txdev, (void *)line, len);

    if (ms <= 0) ms = 1;
    while (waited < ms && got < max) {
        if (since_tx >= 200) { write(txdev, (void *)line, len); since_tx = 0; }
        n = read(rxdev, rbuf, sizeof rbuf - 1);
        if (n > (int)sizeof(struct udpPseudoHdr)) {
            struct udpPseudoHdr *ph = (struct udpPseudoHdr *)rbuf;
            struct udpPkt *pkt = (struct udpPkt *)(rbuf + sizeof(struct udpPseudoHdr));
            char *p = (char *)pkt->data;
            int plen = (int)pkt->len - UDP_HDR_LEN;
            if (plen > 2 && plen < (int)sizeof rbuf - 64) {
                int k = 0; p[plen] = 0;
                while (p[k]) { if (p[k]=='\n'||p[k]=='\r') { p[k]=0; break; } k++; }
                if (p[0] == kind && p[1] == ' ') {
                    int q = 2, gotid = 0, dup = 0, i;
                    while (p[q] >= '0' && p[q] <= '9') { gotid = gotid*10 + (p[q]-'0'); q++; }
                    if (p[q] == ' ') q++;
                    for (i = 0; i < got; i++)
                        if (seen[i][0]==(uchar)ph->srcIp[0] && seen[i][1]==(uchar)ph->srcIp[1]
                         && seen[i][2]==(uchar)ph->srcIp[2] && seen[i][3]==(uchar)ph->srcIp[3])
                            dup = 1;                       /* 同じ相手は一つだけ */
                    if (gotid == id && !dup) {
                        char *d = out + got * stride;
                        if (kind == 'A') {                 /* 答は送り主の IP そのもの */
                            int at = 0;
                            for (i = 0; i < 4; i++) {
                                char nb[8]; sprintf(nb, "%d", (int)(uchar)ph->srcIp[i]);
                                at = put(d, at, stride, nb);
                                if (i < 3) at = put(d, at, stride, ".");
                            }
                        } else {                           /* R : 値の文字 */
                            int t = 0; while (p[q] && t < stride - 1) d[t++] = p[q++];
                            d[t] = 0;
                        }
                        for (i = 0; i < 4; i++) seen[got][i] = (uchar)ph->srcIp[i];
                        got++;
                    }
                }
            }
            continue;                                      /* 続けて読む */
        }
        sleep(5); waited += 5; since_tx += 5;
    }
    close(txdev); close(rxdev);
    return got;
}

/* neighbors() — 今つながっている相手の IP。過去に見た相手ではない。 */
int aipl_mesh_probe(char *out, int stride, int max, int ms)
{
    char q[32]; int n = 0, id = g_next_id++;
    if (g_next_id > 1000000) g_next_id = 1;
    n = put(q, n, sizeof q, "H ");
    { char nb[16]; sprintf(nb, "%d", id); n = put(q, n, sizeof q, nb); }
    n = put(q, n, sizeof q, "\n");
    return mesh_run(q, n, id, 'A', ms > 0 ? ms : 300, out, stride, max);
}

/* broadcast(役, メソッド, 引数) — 撒くだけ。返事は求めない。 */
int aipl_mesh_bcast(const char *actor, const char *meth, const char *arg)
{
    int dev, n = 0, id = g_next_id++;
    char q[288];
    if (g_next_id > 1000000) g_next_id = 1;
    n = put(q, n, sizeof q, "B ");
    { char nb[16]; sprintf(nb, "%d", id); n = put(q, n, sizeof q, nb); }
    n = put(q, n, sizeof q, " ");
    n = put(q, n, sizeof q, actor);
    n = put(q, n, sizeof q, " ");
    n = put(q, n, sizeof q, meth);
    n = put(q, n, sizeof q, " ");
    n = put(q, n, sizeof q, arg ? arg : "");
    n = put(q, n, sizeof q, "\n");
    dev = mesh_open_tx();
    if (dev < 0) return -1;
    write(dev, q, n);
    close(dev);
    return 0;
}

/* gather(役, メソッド, 引数, ms) — 同報で問い、期限までに届いた分だけ返す。
 * 全員から返る保証は無い。だから戻りが配列なのである。 */
int aipl_mesh_gather(const char *actor, const char *meth, const char *arg,
                     int ms, char *out, int stride, int max)
{
    char q[288]; int n, id = g_next_id++;
    if (g_next_id > 1000000) g_next_id = 1;
    n = build_q(q, sizeof q, id, actor, meth, arg);
    return mesh_run(q, n, id, 'R', ms, out, stride, max);
}

/* 応答を返す。
   ★ 番人の装置は UDP_FLAG_PASSIVE で開いている。その装置の udpWrite は
     「擬似ヘッダ＋UDP ヘッダ＋本文」を要求し、本文だけ渡すと長さ検査で
     SYSERR になる（実機で reply=0 のまま応答が出ていなかったのはこれ）。
     返信は素の装置を1本開いて出す。局所ポートは任意でよい ―― 相手は
     reqid で突き合わせるので、返りの送信元ポートは見ていない。 */
static int reply_to(const struct netaddr *dst, ushort dstpt, const char *msg, int len)
{
    int dev, rc = -1;
    dev = udpAlloc();
    if (SYSERR == dev) return -1;
    if (SYSERR == open(dev, &netiftab[0].ip, (struct netaddr *)dst, 0, dstpt)) {
        udptab[dev - UDP0].state = UDP_FREE;
        return -1;
    }
    if (SYSERR != write(dev, (void *)msg, len)) rc = 0;
    close(dev);
    return rc;
}

/* ---- 受け口の番人 --------------------------------------------------------
 * UDP/9010 に常駐して、来た要求をこの板の公開アクターへ渡す。
 * PASSIVE で開くと、読んだ塊の先頭に送り主の擬似ヘッダが付いてくるので、
 * ARP も相手表も要らずに返せる。 */
/* ---- 同報の受け口 --------------------------------------------------------
   udpDemux は「装置の局所 IP が宛先 IP と一致すること」を求める（device/udp/
   udpDemux.c）。同報の宛先は 192.168.3.255 なので、自分の IP に結んだ装置には
   一致せず、黙って落ちる ―― メッシュの三つが Pi 3 だけ届かない理由がこれ。
   Xinu の振り分けには手を入れず、同報の宛て先に結んだ装置をもう 1 本開いて
   同じ番人を回す。受け口が二つになるだけで、扱いは何も変わらない。 */
static void remote_serve(int dev)
{
    int n;
    static char buf[1024];
    for (;;) {
        n = read(dev, buf, sizeof buf - 1);
        g_n_read++; g_last_n = n;
        if (n <= (int)sizeof(struct udpPseudoHdr)) { g_n_short++; continue; }
        buf[n] = 0;

        { struct udpPseudoHdr *ph = (struct udpPseudoHdr *)buf;
          struct udpPkt *pkt = (struct udpPkt *)(buf + sizeof(struct udpPseudoHdr));
          struct netaddr src;
          /* ★ udpRecv が srcPort / dstPort / len を「受け取った時点で」ホスト順へ
                直してから積んでいる（network/udp/udpRecv.c）。ここで net2hs を
                かけると二度入れ替わって長さが化け、番人は黙って読み捨てる ――
                実機で Pi 3 だけが応答しなかったのはこれ。 */
          ushort srcpt = pkt->srcPort;
          char *p = (char *)pkt->data;
          int plen = (int)pkt->len - UDP_HDR_LEN;
          g_last_plen = plen;
          if (plen < 2) { g_n_short++; continue; }
          p[(plen < (int)(sizeof buf - sizeof(struct udpPseudoHdr) - UDP_HDR_LEN - 1))
            ? plen : 0] = 0;
          { int k = 0; while (p[k]) { if (p[k]=='\n'||p[k]=='\r') { p[k]=0; break; } k++; } }

          src.type = NETADDR_IPv4; src.len = IPv4_ADDR_LEN;
          memcpy(src.addr, ph->srcIp, IPv4_ADDR_LEN);

          g_last_c0 = (long)(unsigned char)p[0];

          /* ---- H <id> : 誰かいますか（同報）。居ることだけを単送で返す ----
             メッシュの neighbors() の呼びかけである。 */
          if (p[0] == 'H' && p[1] == ' ') {
              int q = 2, id = 0, at = 0;
              char reply[32];
              while (p[q] >= '0' && p[q] <= '9') { id = id*10 + (p[q]-'0'); q++; }
              at = put(reply, at, sizeof reply, "A ");
              { char nb[16]; sprintf(nb, "%d", id); at = put(reply, at, sizeof reply, nb); }
              at = put(reply, at, sizeof reply, "\n");
              reply_to(&src, srcpt, reply, at);
              continue;
          }

          /* ---- B <id> <役> <メソッド> <引数> : broadcast(...)。返事は出さない ----
             返すと、撒いた一通に対して全員が返して嵐になる。控え（at-most-once）は
             Q と同じものを使うので、再送で二度走ることはない。 */
          if (p[0] == 'B' && p[1] == ' ') {
              int q = 2, id = 0, k;
              char actor[40], meth[40], val[192];
              while (p[q] >= '0' && p[q] <= '9') { id = id*10 + (p[q]-'0'); q++; }
              if (p[q] == ' ') q++;
              k = 0; while (p[q] && p[q] != ' ' && k < (int)sizeof actor - 1) actor[k++] = p[q++];
              actor[k] = 0;
              if (p[q] == ' ') q++;
              k = 0; while (p[q] && p[q] != ' ' && k < (int)sizeof meth - 1) meth[k++] = p[q++];
              meth[k] = 0;
              if (p[q] == ' ') q++;
              if (ans_lookup((const uchar *)src.addr, id)) continue;   /* 再送 */
              { char withslash[42]; withslash[0] = '/';
                { int t = 0; while (actor[t] && t < 40) { withslash[t+1] = actor[t]; t++; }
                  withslash[t+1] = 0; }
                if (!vm_remote_call(actor, meth, p + q, val, sizeof val))
                    vm_remote_call(withslash, meth, p + q, val, sizeof val); }
              ans_store((const uchar *)src.addr, id, "ok");
              continue;
          }

          if (!(p[0] == 'Q' && p[1] == ' ')) { g_n_notq++; continue; }
          g_n_q++;

          { int q = 2, id = 0, k;
            char actor[40], meth[40], val[192], reply[224];
            while (p[q] >= '0' && p[q] <= '9') { id = id*10 + (p[q]-'0'); q++; }
            if (p[q] == ' ') q++;
            k = 0; while (p[q] && p[q] != ' ' && k < (int)sizeof actor - 1) actor[k++] = p[q++];
            actor[k] = 0;
            if (p[q] == ' ') q++;
            k = 0; while (p[q] && p[q] != ' ' && k < (int)sizeof meth - 1) meth[k++] = p[q++];
            meth[k] = 0;
            if (p[q] == ' ') q++;

            /* 同じ要求の再送なら、走らせずに前の答えを返す */
            { const char *prev = ans_lookup((const uchar *)src.addr, id);
              if (prev) {
                  int at = 0;
                  at = put(reply, at, sizeof reply, "R ");
                  { char nb[16]; sprintf(nb, "%d", id); at = put(reply, at, sizeof reply, nb); }
                  at = put(reply, at, sizeof reply, " ");
                  at = put(reply, at, sizeof reply, prev);
                  at = put(reply, at, sizeof reply, "\n");
                  reply_to(&src, srcpt, reply, at);
                  continue;
              } }

            /* 公開名は "/echo" のように '/' 始まりで登録されている。
               電文では '/' を書かせないので、両方の書き方を試す。 */
            { char withslash[42]; withslash[0] = '/';
              { int t = 0; while (actor[t] && t < 40) { withslash[t+1] = actor[t]; t++; }
                withslash[t+1] = 0; }
              if (!vm_remote_call(actor, meth, p + q, val, sizeof val)
               && !vm_remote_call(withslash, meth, p + q, val, sizeof val))
                  put(val, 0, sizeof val, "err"); }

            ans_store((const uchar *)src.addr, id, val);

            { int at = 0;
              at = put(reply, at, sizeof reply, "R ");
              { char nb[16]; sprintf(nb, "%d", id); at = put(reply, at, sizeof reply, nb); }
              at = put(reply, at, sizeof reply, " ");
              at = put(reply, at, sizeof reply, val);
              at = put(reply, at, sizeof reply, "\n");
              if (0 == reply_to(&src, srcpt, reply, at)) g_n_reply++; } }
        }
    }
}

/* 同報の受け口だけを回す番人（自網.255 に結ぶ） */
thread aipl_remote_bcast_daemon(void)
{
    int dev, i;
    for (i = 0; i < 60; i++) {
        if (ethertab[0].state == ETH_STATE_UP) break;
        sleep(500);
    }
    dev = udpAlloc();
    if (SYSERR == dev) return SYSERR;
    if (SYSERR == open(dev, &netiftab[0].ipbrc, NULL, AIPL_PORT, 0)) {
        udptab[dev - UDP0].state = UDP_FREE; return SYSERR;
    }
    control(dev, UDP_CTRL_SETFLAG, UDP_FLAG_PASSIVE, 0);
    kprintf("[remote] AIPL mesh listening on UDP %d (broadcast)\r\n", AIPL_PORT);
    remote_serve(dev);
    return OK;
}

thread aipl_remote_daemon(void)
{
    int dev, i;
    for (i = 0; i < 60; i++) {
        if (ethertab[0].state == ETH_STATE_UP) break;
        sleep(500);
    }
    dev = udpAlloc();
    if (SYSERR == dev) { g_state = 1; kprintf("[remote] udpAlloc failed\r\n"); return SYSERR; }
    if (SYSERR == open(dev, &netiftab[0].ip, NULL, AIPL_PORT, 0)) {
        g_state = 2;
        kprintf("[remote] open(:%d) failed\r\n", AIPL_PORT);
        udptab[dev - UDP0].state = UDP_FREE; return SYSERR;
    }
    control(dev, UDP_CTRL_SETFLAG, UDP_FLAG_PASSIVE, 0);
    g_state = 3;
    kprintf("[remote] AIPL remote listening on UDP %d\r\n", AIPL_PORT);
    /* 同報の受け口を別プロセスで立てる（読みは塞ぐので 1 本では兼ねられない） */
    ready(create((void *)aipl_remote_bcast_daemon, 4096, 20,
                 "aipl-mesh", 0), RESCHED_NO);
    remote_serve(dev);
    return OK;
}

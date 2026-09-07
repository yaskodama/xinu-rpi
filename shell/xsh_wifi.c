/**
 * @file     xsh_wifi.c
 *
 * Shell command `wifi`:
 *   wifi status  — show the current WiFi connection (SSID + IP)
 *   wifi on      — if not connected, scan, list candidate APs and let the
 *                  user pick one by number, then prompt for a password
 *                  and connect (join + DHCP)
 *   wifi off     — disconnect (bring the BSS down)
 */
/* Embedded Xinu, arm-rpi3 WiFi (BCM43455) add-on. */

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fat.h>

#ifdef _XINU_PLATFORM_ARM_RPI3_

/* Accumulate the (small) /microsd/wifi.txt into a buffer. */
struct wifi_cfgbuf { char *buf; int len, max; };
static int wifi_cfg_emit(const unsigned char *b, int n, void *ctx)
{
    struct wifi_cfgbuf *c = (struct wifi_cfgbuf *)ctx;
    int i;
    for (i = 0; i < n && c->len < c->max - 1; i++) c->buf[c->len++] = (char)b[i];
    c->buf[c->len] = 0;
    return 0;
}
static void wifi_rstrip(char *s)
{
    int n = (int)strlen(s);
    while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t')) s[--n] = 0;
}
/* Read /microsd/wifi.txt and parse `ssid=` / `pass=` (with '#' comments).
 * Returns 0 if at least an SSID was found, -1 otherwise. */
int wifi_load_cfg(char *ssid, int sl, char *pass, int pl)
{
    /* SD and WiFi share the ONE Arasan EMMC controller.  Reading the SD card
     * (fat_mount) re-muxes the controller to the card's pins — which STEALS the
     * bus from the WiFi SDIO link.  On the first connect that is fine (we read
     * wifi.txt, THEN wifi_bringup() muxes the controller to WiFi).  But on a
     * reconnect (wifi off -> wifi on) bringup is skipped, so a second SD read
     * would leave the controller on the card and kill WiFi — exactly the
     * observed failure (iovars rc=-1, events=0, join failed).
     *
     * Fix: cache the credentials on the first good read and, once cached, hand
     * them back WITHOUT touching the SD at all, so the EMMC stays with WiFi. */
    static char c_ssid[40], c_pass[68];
    static int  c_have = 0;
    struct fat_dirent e;
    char raw[640];
    struct wifi_cfgbuf cb;
    char *p;
    int got = 0, i;

    if (c_have) {                                    /* cached: never re-touch SD */
        for (i = 0; i < sl - 1 && c_ssid[i]; i++) ssid[i] = c_ssid[i];
        ssid[i] = 0;
        for (i = 0; i < pl - 1 && c_pass[i]; i++) pass[i] = c_pass[i];
        pass[i] = 0;
        return 0;
    }

    cb.buf = raw; cb.len = 0; cb.max = (int)sizeof(raw);
    ssid[0] = 0; pass[0] = 0;

    if (fat_mount() == 0 &&
        0 == fat_find_root("wifi.txt", &e) && !e.is_dir &&
        0 == fat_read_file(e.cluster, e.size, wifi_cfg_emit, &cb))
    {
        for (p = raw; *p; )
        {
            char *d; int n;
            while (*p == ' ' || *p == '\t') p++;
            if (0 == strncmp(p, "ssid=", 5)) { d = ssid; n = sl; p += 5; }
            else if (0 == strncmp(p, "pass=", 5)) { d = pass; n = pl; p += 5; }
            else d = NULL;
            if (d != NULL) {
                int k = 0;
                while (*p && *p != '\n' && *p != '\r' && k < n - 1) d[k++] = *p++;
                d[k] = 0;
                wifi_rstrip(d);
            }
            while (*p && *p != '\n') p++;            /* to end of line */
            if (*p == '\n') p++;
        }
        got = ssid[0] ? 1 : 0;
    }

    if (got) {                                       /* cache the good read */
        for (i = 0; i < (int)sizeof(c_ssid) - 1 && ssid[i]; i++) c_ssid[i] = ssid[i];
        c_ssid[i] = 0;
        for (i = 0; i < (int)sizeof(c_pass) - 1 && pass[i]; i++) c_pass[i] = pass[i];
        c_pass[i] = 0;
        c_have = 1;
        return 0;
    }
    return -1;                                        /* no wifi.txt, nothing cached */
}

extern int         wifi_connected(void);
extern const char *wifi_ssid(void);
extern void        wifi_dhcp_diag(unsigned char *ip, unsigned char *gw, int *have);
extern int         wifi_scan_ssids(char ssids[][40], int max);
extern int         wifi_join(const char *ssid, const char *pass);
extern int         wifi_dhcp(void);
extern void        wifi_disconnect(void);
extern void        wifi_serve_start(void);   /* start ARP/ICMP responder -> pingable */

/* Read a line from `fd` into buf (echoing to stdout).  `mask` prints '*'
 * instead of the typed character (for passwords).  Stops on CR/LF. */
static void wifi_readline(int fd, char *buf, int max, int mask)
{
    int i = 0, c;
    while ((c = fgetc(fd)) != EOF && c != '\n' && c != '\r') {
        if (c == '\b' || c == 0x7F) {            /* backspace */
            if (i > 0) { i--; printf("\b \b"); }
            continue;
        }
        if (c < 0x20) continue;                  /* drop other controls */
        if (i < max - 1) {
            buf[i++] = (char)c;
            printf("%c", mask ? '*' : c);        /* local echo */
        }
    }
    buf[i] = '\0';
    printf("\n");
}

static void wifi_show_status(void)
{
    /* ★ まずファームウェアに「いまどのセルに居るか」を聞く。局所変数だけを
       見ていると、ad-hoc で IP を代入しただけの状態を「接続済み」と表示して
       しまう（実際そうなっていて、合流の切り分けを誤らせた）。 */
    { unsigned char bss[6];
      extern int wifi_live_bssid(unsigned char *);
      if (wifi_live_bssid(bss))
          printf("  BSSID %02x:%02x:%02x:%02x:%02x:%02x （ファームから取得）\n",
                 bss[0],bss[1],bss[2],bss[3],bss[4],bss[5]);
      else
          printf("  BSSID なし ―― 無線はどのセルにも入っていない\n"); }
    if (wifi_connected()) {
        unsigned char ip[4], gw[4]; int have;
        wifi_dhcp_diag(ip, gw, &have);
        printf("WiFi: connected to \"%s\"\n", wifi_ssid());
        printf("  ip %u.%u.%u.%u  gw %u.%u.%u.%u\n",
               ip[0], ip[1], ip[2], ip[3], gw[0], gw[1], gw[2], gw[3]);
    } else {
        printf("WiFi: not connected\n");
    }
}

shellcmd xsh_wifi(int nargs, char *args[])
{
    if (nargs < 2) {
        printf("Usage: %s on [ssid pass]|off|status\n", args[0]);
        return 0;
    }

    if (strcmp(args[1], "status") == 0) {
        wifi_show_status();
        return 0;
    }

    /* `wifi diag` — dump the LAST join attempt's firmware diagnostics so a
     * failed (re)connect can be diagnosed from the on-screen shell window. */
    if (strcmp(args[1], "diag") == 0) {
        extern void wifi_diag(int *, int *, int *, int *, int *, int *, int *);
        extern int  wifi_diag_seq(int *, int);
        extern int  wifi_tgt_diag(int *, int *);
        int sup, pmk, nev, eapol, link, lastev, laststat;
        int seq[16], n, i, found = 0, chanspec = 0;
        wifi_diag(&sup, &pmk, &nev, &eapol, &link, &lastev, &laststat);
        wifi_tgt_diag(&found, &chanspec);
        printf("WiFi join diag (last attempt):\n");
        printf("  tgt_found=%d chanspec=0x%04x  (directed scan saw the AP?)\n", found, chanspec);
        printf("  sup_wpa rc=%d  PMK rc=%d  (0 = ok)\n", sup, pmk);
        printf("  events=%d EAPOL=%d link=%d lastev=%d laststat=%d\n",
               nev, eapol, link, lastev, laststat);
        n = wifi_diag_seq(seq, 16);
        printf("  event seq:");
        for (i = 0; i < n; i++) printf(" %d", seq[i]);
        printf("\n  (0=SET_SSID 3=AUTH 7=ASSOC 16=LINK 46=PSK_SUP)\n");
        return 0;
    }

    /* `wifi trace` — dump the tail of the WiFi driver log for the LAST connect
     * attempt (wifi_join resets the buffer each try).  Shows the bring-up
     * stages so a failed reconnect's stopping point is visible on-screen. */
    if (strcmp(args[1], "trace") == 0) {
        extern const char *wifi_trace(void);
        const char *t = wifi_trace();
        int len = 0, start;
        while (t[len]) len++;
        start = (len > 2000) ? len - 2000 : 0;     /* last ~2000 chars */
        while (start > 0 && t[start - 1] != '\n') start--;  /* line boundary */
        printf("---- wifi log tail (%d bytes total) ----\n", len);
        printf("%s", t + start);
        printf("---- end ----\n");
        return 0;
    }

    if (strcmp(args[1], "off") == 0) {
        wifi_disconnect();
        printf("WiFi: disconnected\n");
        return 0;
    }

    if (strcmp(args[1], "adhoc") == 0) {        /* MANET ad-hoc (IBSS) node */
        extern int wifi_adhoc(const char *ssid, int channel, int n);
        int ch = (nargs >= 4) ? atoi(args[3]) : 6;
        int node = (nargs >= 5) ? atoi(args[4]) : 2;
        if (nargs < 3) { printf("Usage: %s adhoc <ssid> [ch] [node]\n", args[0]); return 0; }
        printf("Joining ad-hoc cell \"%s\" ch=%d as 10.0.0.%d ...\n", args[2], ch, node);
        if (wifi_adhoc(args[2], ch, node) != 0) { printf("WiFi: adhoc failed\n"); return 0; }
        printf("WiFi: ad-hoc up, ip 10.0.0.%d (responder running)\n", node);
        return 0;
    }

    if (strcmp(args[1], "on") == 0) {
        static char ssids[24][40];
        char sel[8], pass[68];
        int n, num;

        if (wifi_connected()) {
            printf("WiFi: already connected to \"%s\"\n", wifi_ssid());
            printf("(run 'wifi off' first to switch networks)\n");
            return 0;
        }

        /* Non-interactive form: `wifi on <ssid> [pass]`.  Skips the scan +
         * number/password prompts so the command can be driven from the
         * remote-login /shell route (which has no stdin).  Blank/omitted
         * pass = open network. */
        if (nargs >= 3) {
            const char *jssid = args[2];
            const char *jpass = (nargs >= 4) ? args[3] : "";
            printf("Connecting to \"%s\"...\n", jssid);
            if (wifi_join(jssid, jpass) != 0) {
                printf("WiFi: join failed.\n");
                return 0;
            }
            if (wifi_dhcp() != 0 || !wifi_connected()) {
                printf("WiFi: DHCP failed (no IP).\n");
                return 0;
            }
            wifi_serve_start();   /* ARP/ICMP responder -> the AP IP is pingable */
            wifi_show_status();
            return 0;
        }

        /* No SSID on the command line: try the saved /microsd/wifi.txt so a
         * plain `wifi on` connects to the usual network with no scan/prompt.
         * (The SD card is read first, then wifi_join takes over the shared
         * EMMC controller for the WiFi SDIO bus.) */
        {
            char cssid[40], cpass[68];
            if (0 == wifi_load_cfg(cssid, sizeof(cssid), cpass, sizeof(cpass))) {
                printf("Using /microsd/wifi.txt -> \"%s\"\n", cssid);
                printf("Connecting...\n");
                if (wifi_join(cssid, cpass) != 0) { printf("WiFi: join failed.\n"); return 0; }
                if (wifi_dhcp() != 0 || !wifi_connected()) { printf("WiFi: DHCP failed (no IP).\n"); return 0; }
                wifi_serve_start();
                wifi_show_status();
                return 0;
            }
            printf("(no /microsd/wifi.txt — falling back to scan)\n");
        }

        printf("Scanning for access points (~30s; the radio drops briefly)...\n");
        n = wifi_scan_ssids(ssids, 24);
        if (n <= 0) { printf("No access points found.\n"); return 0; }

        printf("Available networks:\n");
        for (num = 0; num < n; num++)
            printf("  %d: %s\n", num + 1, ssids[num]);

        printf("Select AP number (0 = cancel): ");
        wifi_readline(stdin, sel, sizeof(sel), 0);
        num = atoi(sel);
        if (num < 1 || num > n) { printf("Cancelled.\n"); return 0; }

        printf("Password for \"%s\" (blank = open): ", ssids[num - 1]);
        wifi_readline(stdin, pass, sizeof(pass), 1);

        printf("Connecting to \"%s\"...\n", ssids[num - 1]);
        if (wifi_join(ssids[num - 1], pass) != 0) {
            printf("WiFi: join failed.\n");
            return 0;
        }
        if (wifi_dhcp() != 0 || !wifi_connected()) {
            printf("WiFi: DHCP failed (no IP).\n");
            return 0;
        }
        wifi_serve_start();   /* ARP/ICMP responder -> the AP IP is pingable */
        wifi_show_status();
        return 0;
    }

    printf("Usage: %s on [ssid pass]|off|status\n", args[0]);
    return 0;
}

#else  /* not arm-rpi3 */

shellcmd xsh_wifi(int nargs, char *args[])
{
    (void)nargs; (void)args;
    printf("wifi: only available on the arm-rpi3 (BCM43455) build\n");
    return 0;
}

#endif

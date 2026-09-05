#include <stddef.h>
#include <stdint.h>
#include <kernel.h>
#include <thread.h>
#include <semaphore.h>
#include <clock.h>     /* P3: clkticks for throughput markers */

/* Monotonic milliseconds.
 *
 * `clkticks` is NOT a millisecond counter and is NOT monotonic: clkhandler()
 * resets it to 0 once it reaches CLKTICKS_PER_SEC and bumps `clktime`
 * (system/clkhandler.c), so it only ever counts within the current second.
 * This file used to read it raw and multiply by 10 on the belief that the
 * clock ran at 100 Hz.  It runs at 1000 Hz (include/clock.h), so every
 * elapsed time computed here was wrong by 10x AND wrapped to a negative or
 * absurd value at each second boundary.  Every benchmark number taken from
 * this runtime inherited both faults. */
unsigned long abcl_now_ms(void)
{
    return (unsigned long)clktime * 1000UL + (unsigned long)clkticks;
}
#include <stdio.h>
#include <string.h>
#include <rcu.h>       /* RCU: lock-free actor-registry reads (concurrency_safety=rcu) */

/* P3: bumped from 16 to 64 so the lock-free MPSC ring can sustain
   higher producer fan-in without back-pressure dropping messages.
   P4: bumped 64 -> 1024 so a 3-D-data actor can stream a high-resolution
   mesh (~700 triangles per chunk) to the "Blender display" actor without
   the receiver's mailbox overflowing.  ~296 KB/object * 128 obj ~= 38 MB
   .bss — fine on the Pi 3's 1 GB. */
#define MAX_MAILBOX 1024
/* MAX_OBJECTS bumped 16 -> 32 -> 128.  alloc_obj() never recycles slots
 * (it just does n_objects++), and DiningBench_run() spawns a fresh batch of
 * 5 Forks + 5 Philosophers on every run without reaping the old batch (the
 * GC actor only reaps actors idle past threshold_ms, so livelocking
 * philosophers are never collected).  With the old cap of 32 a benchmark of
 * even two dining runs overflowed objects[] and corrupted entry 32+, wedging
 * the Pi.  128 gives headroom for a full sweep — modes 1/2/3 x repeat 3 =
 * 9 runs x 11 = 99 actors plus infrastructure (~7).  Cost is ~18.5 KB of
 * .bss per object (mailbox dominates) => ~2.4 MB total, trivial on the Pi 3's
 * 1 GB.  A proper fix (slot recycling in alloc_obj) is still TODO. */
#define MAX_OBJECTS 128
#define MAX_FIELDS  16
#define MAX_ARGS    8

static int max_messages       = 20;
static int messages_processed = 0;
volatile int global_shutdown  = 0;
static semaphore counter_mu;
static semaphore print_mu;

typedef enum { V_NIL, V_INT, V_FLOAT, V_STR, V_OBJ } vtag_t;
typedef struct {
  vtag_t      tag;
  long        i;
  double      f;
  const char *s;
  int         obj_id;
} value_t;

static value_t mk_int(long n)        { value_t v; v.tag=V_INT;   v.i=n;   v.f=0; v.s=0; v.obj_id=0; return v; }
static value_t mk_float(double n)    { value_t v; v.tag=V_FLOAT; v.f=n;   v.i=0; v.s=0; v.obj_id=0; return v; }
static value_t mk_str(const char *s) { value_t v; v.tag=V_STR;   v.s=s;   v.i=0; v.f=0; v.obj_id=0; return v; }
static value_t mk_obj(int id)        { value_t v; v.tag=V_OBJ;   v.obj_id=id; v.i=0; v.f=0; v.s=0; return v; }

static int truthy(value_t v) {
  switch (v.tag) {
  case V_INT:   return v.i != 0;
  case V_FLOAT: return v.f != 0.0;
  case V_STR:   return v.s != NULL && v.s[0] != '\0';
  case V_OBJ:   return v.obj_id >= 0;
  default:      return 0;
  }
}

static value_t v_binop(const char *op, value_t a, value_t b) {
  long ai = (a.tag == V_INT) ? a.i : (a.tag == V_FLOAT ? (long)a.f : 0);
  long bi = (b.tag == V_INT) ? b.i : (b.tag == V_FLOAT ? (long)b.f : 0);
  if (op[0] == '+' && op[1] == '\0') return mk_int(ai + bi);
  if (op[0] == '-' && op[1] == '\0') return mk_int(ai - bi);
  if (op[0] == '*' && op[1] == '\0') return mk_int(ai * bi);
  if (op[0] == '/' && op[1] == '\0') return mk_int(bi != 0 ? ai / bi : 0);
  if (op[0] == '=' && op[1] == '=')  return mk_int(ai == bi);
  if (op[0] == '!' && op[1] == '=')  return mk_int(ai != bi);
  if (op[0] == '<' && op[1] == '=')  return mk_int(ai <= bi);
  if (op[0] == '>' && op[1] == '=')  return mk_int(ai >= bi);
  if (op[0] == '<' && op[1] == '\0') return mk_int(ai <  bi);
  if (op[0] == '>' && op[1] == '\0') return mk_int(ai >  bi);
  return mk_int(0);
}

#ifdef _XINU_PLATFORM_ARM_RPI3_
extern void aipl_console_puts(const char *s);  /* HDMI console (apps/gwm.c) */
#endif

static void v_int_to_buf(char *b, int *p, long n) {
  char t[20];
  int  i = 0, j, neg = (n < 0);
  if (neg) n = -n;
  if (n == 0) t[i++] = '0';
  while (n > 0) { t[i++] = (char)('0' + (n % 10)); n /= 10; }
  if (neg) b[(*p)++] = '-';
  for (j = i - 1; j >= 0; j--) b[(*p)++] = t[j];
}

/* Print a value to the serial console AND (on the Pi3) the on-screen
 * AIPL print window. */
static void v_print(value_t v) {
  char line[80];
  int  p = 0, i;
  switch (v.tag) {
  case V_STR: { const char *s = v.s ? v.s : "";
                for (i = 0; s[i] && p < 79; i++) line[p++] = s[i]; break; }
  case V_INT: v_int_to_buf(line, &p, v.i); break;
  case V_OBJ: { const char *pre = "<obj ";
                for (i = 0; pre[i] && p < 79; i++) line[p++] = pre[i];
                v_int_to_buf(line, &p, v.obj_id);
                if (p < 79) line[p++] = '>'; break; }
  default:    { const char *s = "<nil>";
                for (i = 0; s[i] && p < 79; i++) line[p++] = s[i]; break; }
  }
  line[p] = '\0';
  wait(print_mu);
  kprintf("%s\r\n", line);
  signal(print_mu);
#ifdef _XINU_PLATFORM_ARM_RPI3_
  aipl_console_puts(line);
#endif
}

/* Narrate a Xinu philosopher's state into the AIPL print stream (serial +
 * HDMI console): formats "P<pid> <action>" and prints it. */
void abcl_phil_say(int pid, const char *action) {
  static char pool[8][48];
  static int  slot = 0;
  char *b = pool[slot & 7];
  int   p = 0, i = 0;
  slot++;
  b[p++] = 'P';
  if (pid >= 10) b[p++] = (char)('0' + (pid / 10));
  b[p++] = (char)('0' + (pid % 10));
  b[p++] = ' ';
  while (action[i] && p < 46) b[p++] = action[i++];
  b[p] = '\0';
  v_print(mk_str(b));
}

typedef struct {
  int         sender;
  int         receiver;
  const char *method;
  int         n_args;
  value_t     args[MAX_ARGS];
} message_t;

/* P3: lock-free MPSC bounded ring buffer (Vyukov style, single consumer).
   - `enq` is bumped via CAS by multiple producers, no critical section.
   - `deq` is touched only by the owning actor's thread, so a plain
     atomic store suffices.
   - `slot_seq[i]` is a per-slot sequence stamp; producer sees its slot
     ready when seq==pos, consumer sees ready payload when seq==pos+1,
     and the consumer recycles the slot for the producer at pos+CAP by
     storing seq=pos+CAP.  Initialized so slot_seq[i]=i.
   - `items` is a counting semaphore kept solely so the receiver can
     block on empty.  Lock-free bookkeeping eliminates wait()/signal()
     on the producer path's critical section (only the kernel signal()
     call into `items` remains, which is ISR-safe in Xinu).
*/
typedef struct {
  message_t msgs[MAX_MAILBOX];
  volatile uint32_t slot_seq[MAX_MAILBOX];
  volatile uint32_t enq;
  volatile uint32_t deq;
  volatile uint32_t drops;   /* producer-side drop counter (full mailbox) */
  semaphore items;
} mailbox_t;

typedef struct {
  int       class_id;
  value_t   fields[MAX_FIELDS];
  mailbox_t mbox;
  tid_typ   tid;
  int       started;
  volatile int dead;     /* set by abcl_actor_suicide(): the actor's worker
                            thread exits after the current dispatch returns */
  /* Activity timestamps for the GC sweep — both stamped on each enq/deq
   * so a "zombie" actor (no recent mailbox activity AND no progress
   * processing what's queued) shows up as old by either metric. */
  volatile unsigned long last_enq_ms;
  volatile unsigned long last_deq_ms;
  unsigned long          birth_ticks;   /* abcl_now_ms() at spawn */
  volatile int           protected_from_gc;
} object_t;

static object_t objects[MAX_OBJECTS];
static int      n_objects = 0;
static semaphore objects_mu;

/* RCU reader: ordered load of the published actor count.  Pairs with the
 * rcu_assign_pointer(n_objects, ...) in alloc_obj() — a reader that bounds its
 * scan with this value is guaranteed to observe only fully-initialised slots,
 * with no lock and no interrupt disable. */
static int rcu_n_objects(void) { return rcu_dereference(n_objects); }

static void mailbox_init(mailbox_t *mb) {
  int i;
  mb->enq   = 0;
  mb->deq   = 0;
  mb->drops = 0;
  for (i = 0; i < MAX_MAILBOX; i++) mb->slot_seq[i] = (uint32_t)i;
  mb->items = semcreate(0);
}

void wake_all_actors(void) {
  int i, total = rcu_n_objects();   /* ordered load: see only published slots */
  for (i = 0; i < total; i++) {
    /* items を 1 増やすことで wait() しているアクターを起こす */
    signal(objects[i].mbox.items);
  }
}

void abcl_shutdown(void) {
  global_shutdown = 1;
  wake_all_actors();
}

/* F1: public accessors over the static `objects[]` table.  Used by
   abcl_xinu_chkpt.c to snapshot / restore actor fields without
   re-declaring the object_t layout in two files.  Returns 1 on
   success, 0 on a bad slot or field index. */
int abcl_object_field_count(void) { return MAX_FIELDS; }

int abcl_object_field_get(int obj_id, int field_idx, value_t *out) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return 0;
  if (field_idx < 0 || field_idx >= MAX_FIELDS) return 0;
  *out = objects[obj_id].fields[field_idx];
  return 1;
}

int abcl_object_field_set(int obj_id, int field_idx, value_t v) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return 0;
  if (field_idx < 0 || field_idx >= MAX_FIELDS) return 0;
  objects[obj_id].fields[field_idx] = v;
  return 1;
}

int abcl_object_class_id(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return objects[obj_id].class_id;
}

/* Render actor field as plain text into the caller's buf.  Returns
 * bytes written.  Caller must pass cap >= ~80 (Xinu's libxc has no
 * snprintf so we sprintf and trust the cap).  Used by webactor's
 * /api/object-field since value_t is file-scope here. */
int abcl_object_field_render(int obj_id, int field_idx, char *buf, int cap) {
  value_t v;
  if (cap < 80) { if (cap > 0) buf[0] = 0; return 0; }
  if (abcl_object_field_get(obj_id, field_idx, &v) == 0)
    return sprintf(buf, "out_of_range");
  switch (v.tag) {
    case V_NIL:   return sprintf(buf, "nil");
    case V_INT:   return sprintf(buf, "int=%ld", v.i);
    case V_FLOAT: return sprintf(buf, "float=%f", v.f);
    case V_STR: {
      /* Truncate the string to keep us under cap; sprintf is bounds-less. */
      char tmp[48];
      int i = 0;
      const char *s = v.s ? v.s : "(null)";
      while (s[i] && i < 47) { tmp[i] = s[i]; i++; }
      tmp[i] = 0;
      return sprintf(buf, "str=%s", tmp);
    }
    case V_OBJ:   return sprintf(buf, "obj_id=%d", v.obj_id);
    default:      return sprintf(buf, "tag=%d", (int)v.tag);
  }
}

/* H3 RPC: expose total live actor count so the dispatcher LIST command
   can answer without walking the table. */
int abcl_n_objects(void) { return rcu_n_objects(); }

/* S3 DeadlineHints: expose the Xinu tid_typ that backs an AIPL actor,
   so the set_deadline builtin can translate an obj_id into the
   actual thread id that the kernel's setdeadline() takes. */
int abcl_object_tid(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return (int)objects[obj_id].tid;
}

/* Mailbox telemetry for the on-screen actor monitor (apps/gwm.c).
 * enq = total messages received, deq = total processed, so (enq - deq)
 * is the current backlog; drops = messages lost to a full mailbox;
 * started = 1 once the actor's consumer thread has been spawned. */
int abcl_object_enq(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return (int)objects[obj_id].mbox.enq;
}
int abcl_object_deq(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return (int)objects[obj_id].mbox.deq;
}
int abcl_object_drops(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return (int)objects[obj_id].mbox.drops;
}
int abcl_object_started(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return objects[obj_id].started;
}
int abcl_object_dead(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  return objects[obj_id].dead;
}

/* GC support: age = ms since the more recent of last_enq / last_deq.
 * Returns -1 for an invalid id. */
long abcl_object_age_ms(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return -1;
  unsigned long now = abcl_now_ms();
  unsigned long e = objects[obj_id].last_enq_ms;
  unsigned long d = objects[obj_id].last_deq_ms;
  unsigned long last = (e > d) ? e : d;
  return (long)(now - last);
}

void abcl_object_protect(int obj_id, int on) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return;
  objects[obj_id].protected_from_gc = on ? 1 : 0;
}

int abcl_object_protected(int obj_id) {
  if (obj_id < 0 || obj_id >= rcu_n_objects()) return 0;
  return objects[obj_id].protected_from_gc;
}

/* GC sweep: iterate live actors, force-kill those older than threshold
 * (unless protected).  dry_run=1 reports without killing.  Returns
 * count of kills. */
int abcl_gc_sweep(long threshold_ms, int dry_run, int *out_scanned) {
  int killed = 0, scanned = 0;
  int i, total = rcu_n_objects();   /* ordered load: see only published slots */
  for (i = 0; i < total; i++) {
    if (objects[i].dead) continue;
    if (!objects[i].started) continue;
    scanned++;
    if (objects[i].protected_from_gc) continue;
    long age = abcl_object_age_ms(i);
    if (age >= 0 && age >= threshold_ms) {
      if (!dry_run) {
        tid_typ t = objects[i].tid;
        if (t > 0) {
          kill(t);
          objects[i].dead = 1;
        }
      }
      killed++;
    }
  }
  if (out_scanned) *out_scanned = scanned;
  return killed;
}

/* Reachability garbage collector (mark & sweep).  Roots are: protected actors,
 * actors with pending mail (enq != deq), and actors whose worker thread is
 * doing work (any state other than THRWAIT — i.e. not blocked waiting on its
 * empty mailbox; a sleeping actor in wait() is THRSLEEP and stays a root).
 * From the roots we follow actor-id references stored in fields — V_OBJ
 * precisely, and V_INT conservatively (bytecode-VM actors keep peer ids as
 * plain ints, so any int that is a valid live actor id is treated as a possible
 * reference; this never frees something still reachable).  Every actor not
 * marked is unreferenced garbage and is killed.  dry_run=1 only counts.
 * Returns the number of actors collected. */
int abcl_gc_mark_sweep(int dry_run, int *out_scanned) {
  static char mark[MAX_OBJECTS];
  static int  stack[MAX_OBJECTS];
  int i, fi, sp = 0, scanned = 0, collected = 0;
  int total = rcu_n_objects();
  for (i = 0; i < total; i++) mark[i] = 0;
  /* --- seed roots --- */
  for (i = 0; i < total; i++) {
    if (objects[i].dead || !objects[i].started) continue;
    scanned++;
    int pending = (objects[i].mbox.enq != objects[i].mbox.deq);
    tid_typ t = objects[i].tid;
    int active = (!isbadtid(t) && thrtab[t].state != THRWAIT);
    if (objects[i].protected_from_gc || pending || active) {
      if (!mark[i]) { mark[i] = 1; stack[sp++] = i; }
    }
  }
  /* --- transitive mark over field references --- */
  while (sp > 0) {
    int a = stack[--sp];
    for (fi = 0; fi < MAX_FIELDS; fi++) {
      value_t v = objects[a].fields[fi];
      int ref = -1;
      if (v.tag == V_OBJ) ref = v.obj_id;
      else if (v.tag == V_INT && v.i >= 0 && v.i < total) ref = (int)v.i;  /* conservative */
      if (ref >= 0 && ref < total && !objects[ref].dead && !mark[ref]) {
        mark[ref] = 1; stack[sp++] = ref;
      }
    }
  }
  /* --- sweep: kill every live, unprotected, unmarked actor --- */
  for (i = 0; i < total; i++) {
    if (objects[i].dead || !objects[i].started) continue;
    if (objects[i].protected_from_gc || mark[i]) continue;
    collected++;
    if (!dry_run) {
      tid_typ t = objects[i].tid;
      objects[i].dead = 1;
      if (!isbadtid(t)) { signal(objects[i].mbox.items); kill(t); }  /* wake if blocked, then reap */
    }
  }
  if (!dry_run && collected > 0)
    kprintf("[aipl-gc] mark&sweep: scanned %d, collected %d unreferenced actor(s)\r\n",
            scanned, collected);
  if (out_scanned) *out_scanned = scanned;
  return collected;
}

/* 自殺関数: the actor terminates itself.  Sets the dead flag so its worker
 * thread (abcl_actor_main) leaves its dispatch loop after the current
 * message and exits.  Callable from a generated method via self_id. */
void abcl_actor_suicide(int self_id) {
  if (self_id < 0 || self_id >= n_objects) return;
  objects[self_id].dead = 1;
}

/* Clear the whole actor table so a host program can run again from a clean
 * slate (the next SPAWN starts at id 0).  Without this, re-running a program
 * spawns new actors at shifted ids while leftover actors from the previous
 * run persist, and the table fills toward MAX_OBJECTS.  Stops every actor
 * thread cooperatively (mark dead + wake it so it leaves its loop), frees the
 * per-mailbox semaphores, then resets n_objects.  Call when the actors are
 * idle / between runs (the RESET RPC opcode invokes this). */
void abcl_rt_reset(void) {
  int i, total;
  wait(objects_mu);
  total = n_objects;
  signal(objects_mu);
  for (i = 0; i < total; i++) objects[i].dead = 1;
  for (i = 0; i < total; i++) signal(objects[i].mbox.items); /* wake blocked */
  sleep(100);                       /* let the worker threads exit their loops */
  /* RCU reclaim: unpublish the registry FIRST so no new lock-free reader can
   * start scanning the slots, then wait out a grace period so any reader still
   * mid-scan has drained — only then is it safe to free the mailbox
   * semaphores it might have been touching. */
  wait(objects_mu);
  rcu_assign_pointer(n_objects, 0);
  signal(objects_mu);
  synchronize_rcu();
  for (i = 0; i < total; i++) {
    semfree(objects[i].mbox.items); /* avoid a semaphore leak across resets */
    objects[i].started = 0;
    objects[i].dead    = 0;
  }
  kprintf("[aipl] runtime reset — %d actors cleared, next id starts at 0\r\n",
          total);
}

/* Xinu の queue.h にある enqueue() と名前が衝突するのでリネーム。
   以降 abcl 側のコードでは enqueue マクロで本関数を呼ぶ。 */
/* R1 smoke markers: print the FIRST send + FIRST recv to the serial
   console so a -nographic QEMU run can verify the AIPL actor system
   was reached via grep.  Subsequent sends/recvs are silent to keep
   the log readable. */
static volatile int abcl_first_send_logged = 0;
static volatile int abcl_first_recv_logged = 0;
extern semaphore print_mu;
void abcl_log_first_send(int receiver, const char *method) {
  if (abcl_first_send_logged) return;
  abcl_first_send_logged = 1;
  wait(print_mu);
  kprintf("[aipl] first-send to=%d method=%s\r\n",
          receiver, method ? method : "?");
  signal(print_mu);
}
void abcl_log_first_recv(int self_id, const char *method) {
  if (abcl_first_recv_logged) return;
  abcl_first_recv_logged = 1;
  wait(print_mu);
  kprintf("[aipl] first-recv on=%d method=%s\r\n",
          self_id, method ? method : "?");
  signal(print_mu);
}

/* P3 lock-free MPSC enqueue.
   The hot path has zero kernel disable()/restore() — only LDREX/STREX
   pairs (GCC __atomic primitives on ARMv6+).  The trailing signal() on
   `items` is the only kernel call; it is safe to invoke from an ISR
   because Xinu's signal() handles its own irq mask. */
/* web_expose 用（実体は下の VM 節。ここでは宣言だけ） */
#define VM_WEB_SINK (-2)
static char vm_web_reply[160];
static int  vm_web_have = 0;
static void vm_fmt_val(char *out, int cap, long v);

void abcl_enqueue(int sender, int receiver, const char *method,
                  int n_args, value_t *args) {
  if (receiver == VM_WEB_SINK) {
    /* HTTP から呼んだアクタの返信。値を捕まえて待っている側へ渡す */
    if (method && method[0]=='r' && method[1]=='e' && method[2]=='p'
        && method[3]=='l' && method[4]=='y' && method[5]==0 && n_args > 0) {
      vm_fmt_val(vm_web_reply, sizeof vm_web_reply, args[0].i);
      vm_web_have = 1;
    }
    return;
  }
  if (receiver < 0 || receiver >= n_objects) return;
  abcl_log_first_send(receiver, method);
  /* Stamp activity timestamp early (under the same critical section is
   * not strictly needed since both clkticks read + the field are
   * single-word writes — the GC sweep tolerates a one-tick fuzz). */
  objects[receiver].last_enq_ms = abcl_now_ms();
  mailbox_t *mb = &objects[receiver].mbox;
#ifdef _XINU_PLATFORM_ARM_RPI3_
  /* Pi3 (Cortex-A53 with MMU / L1 D-cache OFF => strongly-ordered memory):
     LDREX/STREX, which the __atomic lock-free path below relies on, are
     UNPREDICTABLE / fault on strongly-ordered memory and abort here.  Use a
     short interrupt-disabled critical section instead — Xinu is
     non-preemptive while interrupts are disabled, so plain accesses are
     race-free across producers and the single consumer. */
  {
    irqmask im = disable();
    uint32_t pos = mb->enq;
    uint32_t idx;
    int i;
    if (pos - mb->deq >= (uint32_t)MAX_MAILBOX) {
      mb->drops++;
      restore(im);
      return;
    }
    idx = pos % (uint32_t)MAX_MAILBOX;
    mb->msgs[idx].sender   = sender;
    mb->msgs[idx].receiver = receiver;
    mb->msgs[idx].method   = method;
    mb->msgs[idx].n_args   = n_args;
    for (i = 0; i < n_args && i < MAX_ARGS; i++)
      mb->msgs[idx].args[i] = args[i];
    mb->slot_seq[idx] = pos + 1;
    mb->enq = pos + 1;
    restore(im);
  }
  signal(mb->items);
#else
  {
  uint32_t pos;
  uint32_t idx;
  uint32_t cur;
  int retries;

  /* (1) Reserve a slot.  Loop: load enq, verify slot is free
     (slot_seq==pos), CAS-bump enq.  Bounded retry. */
  for (retries = 0; retries < 256; retries++) {
    pos = __atomic_load_n(&mb->enq, __ATOMIC_ACQUIRE);
    if (pos - __atomic_load_n(&mb->deq, __ATOMIC_ACQUIRE) >= (uint32_t)MAX_MAILBOX) {
      /* Bounded ring is full — back off briefly and retry.  After
         several yields, give up so a stuck consumer can't deadlock
         a producer thread. */
      if (retries > 16) {
        __atomic_fetch_add(&mb->drops, 1, __ATOMIC_RELAXED);
        return;
      }
      yield();
      continue;
    }
    idx = pos % (uint32_t)MAX_MAILBOX;
    cur = __atomic_load_n(&mb->slot_seq[idx], __ATOMIC_ACQUIRE);
    if (cur != pos) {
      /* Slot not yet recycled by consumer — retry (rare under MPSC). */
      continue;
    }
    if (__atomic_compare_exchange_n(&mb->enq, &pos, pos + 1,
                                    /*weak=*/0,
                                    __ATOMIC_ACQ_REL,
                                    __ATOMIC_ACQUIRE)) {
      break;  /* won reservation */
    }
    /* CAS lost — another producer grabbed pos; retry. */
  }
  if (retries >= 256) {
    __atomic_fetch_add(&mb->drops, 1, __ATOMIC_RELAXED);
    return;
  }

  /* (2) Write payload — we have exclusive ownership of slot[idx]
     because slot_seq[idx]==pos blocked any other producer. */
  mb->msgs[idx].sender   = sender;
  mb->msgs[idx].receiver = receiver;
  mb->msgs[idx].method   = method;
  mb->msgs[idx].n_args   = n_args;
  {
    int i;
    for (i = 0; i < n_args && i < MAX_ARGS; i++)
      mb->msgs[idx].args[i] = args[i];
  }

  /* (3) Publish — releasing store on slot_seq makes the payload
     visible to the consumer. */
  __atomic_store_n(&mb->slot_seq[idx], pos + 1, __ATOMIC_RELEASE);

  /* (4) Wake the consumer if blocked. */
  signal(mb->items);
  }
#endif
}

/* 以降の生成コードでは abcl_enqueue を enqueue として書く */
#define enqueue abcl_enqueue

/* P1 heartbeat thread: sleep + print, sleep + print, ...  Six ticks
   over ~6 sec.  Always lands as long as Xinu's scheduler keeps
   running — i.e. as long as the actor pool isn't busy-looping.
   Each tick also reports the live actor count, which grows past the
   global-declaration count as constructors spawn nested actors. */
thread abcl_heartbeat(void) {
  int i;
  for (i = 0; i < 6; i++) {
    sleep(1000);
    if (global_shutdown) break;
    wait(print_mu);
    kprintf("[aipl] heartbeat tick=%d actors=%d msgs=%d\r\n",
            i, n_objects, messages_processed);
    signal(print_mu);
  }
  return OK;
}

/* runtime cap override */
static int _abcl_cap = 0;

#define CLASS_Fork 0
#define CLASS_Philosopher 1
#define CLASS_WebReceiver 2   /* web-bridge demo actor (apps/webactor.c) */
#define CLASS_Dispatcher 3    /* load-balancer router */
#define CLASS_Worker 4        /* load-balancer worker (compute target) */
#define CLASS_Collector 5     /* periodic actor GC */
#define CLASS_DiningBench 6   /* 5 philosophers + 5 forks benchmark orchestrator */
#define CLASS_ForkCM 7        /* Chandy-Misra hygienic fork (clean/dirty) */
#define CLASS_PhilCM 8        /* Chandy-Misra philosopher (request/release) */
#define CLASS_Spinner 9       /* Rotate4Lines: a rotating line-segment actor   */
#define CLASS_Controller 10   /* Rotate4Lines: spawns 4 spinners + Start/Stop  */
#define CLASS_Pinger 11       /* PingPong: prints + bounces a message to Ponger */
#define CLASS_Ponger 12       /* PingPong: bounces it back to Pinger            */

/* P2: AIPL class -> Xinu priority */
#define ABCL_PRIO_Fork 20
#define ABCL_PRIO_Philosopher 20
#define ABCL_PRIO_WebReceiver 20
#define ABCL_PRIO_Dispatcher 25   /* routing is fast — keep ahead of workers */
#define ABCL_PRIO_Worker 22       /* one above philosophers so submit() lands */
#define ABCL_PRIO_Collector 26    /* GC runs above dispatcher so sweeps land */
#define ABCL_PRIO_DiningBench 24  /* between Dispatcher (25) and Worker (22) */
#define ABCL_PRIO_ForkCM 20       /* same tier as old Fork */
#define ABCL_PRIO_PhilCM 20       /* same tier as old Philosopher */
#define ABCL_PRIO_Spinner 20      /* Rotate4Lines spinner actor */
#define ABCL_PRIO_Controller 21   /* one above the spinners it drives */
static int abcl_class_prio(int class_id) {
  switch (class_id) {
  case CLASS_Fork: return ABCL_PRIO_Fork;
  case CLASS_Philosopher: return ABCL_PRIO_Philosopher;
  case CLASS_WebReceiver: return ABCL_PRIO_WebReceiver;
  case CLASS_Dispatcher: return ABCL_PRIO_Dispatcher;
  case CLASS_Worker:     return ABCL_PRIO_Worker;
  case CLASS_Collector:  return ABCL_PRIO_Collector;
  case CLASS_DiningBench: return ABCL_PRIO_DiningBench;
  case CLASS_ForkCM:     return ABCL_PRIO_ForkCM;
  case CLASS_PhilCM:     return ABCL_PRIO_PhilCM;
  case CLASS_Spinner:    return ABCL_PRIO_Spinner;
  case CLASS_Controller: return ABCL_PRIO_Controller;
  case CLASS_Pinger:     return ABCL_PRIO_Spinner;
  case CLASS_Ponger:     return ABCL_PRIO_Spinner;
  default: return INITPRIO;
  }
}
const char* abcl_class_name(int class_id) {
  switch (class_id) {
  case CLASS_Fork: return "Fork";
  case CLASS_Philosopher: return "Philosopher";
  case CLASS_WebReceiver: return "WebReceiver";
  case CLASS_Dispatcher: return "Dispatcher";
  case CLASS_Worker:     return "Worker";
  case CLASS_Collector:  return "Collector";
  case CLASS_DiningBench: return "DiningBench";
  case CLASS_ForkCM:     return "ForkCM";
  case CLASS_PhilCM:     return "PhilCM";
  case CLASS_Spinner:    return "Spinner";
  case CLASS_Controller: return "Controller";
  case CLASS_Pinger:     return "Pinger";
  case CLASS_Ponger:     return "Ponger";
  default: return class_id >= 200 ? "VM" : "?";   /* 200 = VM_CLASS_BASE (dynamic) */
  }
}

static void dispatch_Fork(int, int, const char*, value_t*, int);
static void dispatch_Philosopher(int, int, const char*, value_t*, int);
static void dispatch_Dispatcher(int, int, const char*, value_t*, int);
static void dispatch_Worker(int, int, const char*, value_t*, int);
static void dispatch_Collector(int, int, const char*, value_t*, int);
static void dispatch_DiningBench(int, int, const char*, value_t*, int);
static void dispatch_ForkCM(int, int, const char*, value_t*, int);
static void dispatch_PhilCM(int, int, const char*, value_t*, int);
static void dispatch_Spinner(int, int, const char*, value_t*, int);
static void dispatch_Controller(int, int, const char*, value_t*, int);
static void dispatch_Pinger(int, int, const char*, value_t*, int);
static void dispatch_Ponger(int, int, const char*, value_t*, int);
static void dispatch(int, int, const char*, value_t*, int);
static int  alloc_obj(int class_id, int n_args, value_t* args);
static void spawn_actor(int id);
int         create_obj(int class_id, int n_args, value_t* args);
int         create_obj_noinit(int class_id);
thread      abcl_actor_main(int self_id);

static int g_f0 = -1;
static int g_f1 = -1;
static int g_f2 = -1;
static int g_f3 = -1;
static int g_f4 = -1;
static int g_p4 = -1;
static int g_p5 = -1;
/* load-balancer globals (set in aipl_main, read by webactor via accessors) */
#define LB_N_WORKERS 4
static int g_loadbal_disp = -1;
static int g_loadbal_w[LB_N_WORKERS] = { -1, -1, -1, -1 };

/* Garbage-collector actor globals.  g_gc_actor is the Collector obj_id;
 * the heartbeat thread sends "tick" to it every GC_HEARTBEAT_MS as long
 * as the actor's F_GC_enabled field is non-zero. */
static int g_gc_actor = -1;
#define GC_HEARTBEAT_MS 1000   /* heartbeat poll granularity (ms) */

/* Rate-limiter (token bucket) for HTTP /submit + /jit gating.  Kept as
 * file-scope state because the Dispatcher object is at the F_Disp__N=13
 * fields budget already (MAX_FIELDS=16 leaves room for 3 more, all
 * earmarked for future Dispatcher-internal use).  Race on the counters
 * is benign — worst case we miss a refill tick or count a throttle
 * twice; not worth a mutex on the hot path. */
static long g_rate_tokens       = 100;   /* current bucket level */
static long g_rate_capacity     = 100;   /* burst cap */
static long g_rate_per_sec      = 50;    /* refill rate */
static long g_rate_last_refill  = 0;     /* abcl_now_ms() at last refill */
static long g_rate_throttled    = 0;     /* lifetime 429-from-rate count */

/* Server-side task timeout.  Collector.tick scans the task table and
 * marks any PENDING entry older than this CANCELLED.  Mac can detect
 * via /api/loadbal/task?id=N (state=CANCELLED + done_ms still 0). */
static long g_task_timeout_ms   = 30000;
static long g_task_timed_out    = 0;     /* lifetime auto-expire count */

enum { F_Fork_holder, F_Fork__N };

static void init_fields_Fork(int self_id) {
  (void)self_id;
  objects[self_id].fields[F_Fork_holder] = mk_int((long)(0L));
}

static void Fork_acquire(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  value_t p_pid = (n_args > 0) ? args[0] : mk_int(0L);
  if (truthy(mk_int((long)(((long)((((objects[self_id].fields[F_Fork_holder]).tag == V_INT ? (objects[self_id].fields[F_Fork_holder]).i : (long)((objects[self_id].fields[F_Fork_holder]).f))) == (0L))))))) {
    objects[self_id].fields[F_Fork_holder] = p_pid;
    enqueue(self_id, sender_id, "fork_granted", 1, (value_t[]){p_pid});
  } else {
    enqueue(self_id, sender_id, "fork_denied", 1, (value_t[]){p_pid});
  }
}

static void Fork_release(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  value_t p_pid = (n_args > 0) ? args[0] : mk_int(0L);
  if (truthy(v_binop("==", mk_int((long)(((objects[self_id].fields[F_Fork_holder]).tag == V_INT ? (objects[self_id].fields[F_Fork_holder]).i : (long)((objects[self_id].fields[F_Fork_holder]).f)))), p_pid))) {
    objects[self_id].fields[F_Fork_holder] = mk_int((long)(0L));
  } else {
  }
}

static void dispatch_Fork(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (strcmp(method, "acquire") == 0) { Fork_acquire(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "release") == 0) { Fork_release(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init") == 0) return; /* default no-op init */
  fprintf(stderr, "unknown method %s on Fork\n", method);
}

enum { F_Philosopher_my_id, F_Philosopher_f_low, F_Philosopher_f_high,
       F_Philosopher_meals, F_Philosopher_meal_idx, F_Philosopher_state,
       F_Philosopher_done_to,    /* DiningBench orch obj_id (0 = no-op) */
       F_Philosopher_t_start,    /* ms when init_phil was processed */
       F_Philosopher__N };

static void init_fields_Philosopher(int self_id) {
  (void)self_id;
  objects[self_id].fields[F_Philosopher_my_id] = mk_int((long)(0L));
  objects[self_id].fields[F_Philosopher_f_low] = mk_int(0L);
  objects[self_id].fields[F_Philosopher_f_high] = mk_int(0L);
  objects[self_id].fields[F_Philosopher_meals] = mk_int((long)(50L));
  objects[self_id].fields[F_Philosopher_meal_idx] = mk_int((long)(0L));
  objects[self_id].fields[F_Philosopher_state] = mk_int((long)(0L));
}

/* dining5 bench 用に拡張: init_phil(id, low, high, meals, done_to) を新規
 * メソッドとして dispatch_Philosopher で受ける.  既存の `init` は 3-arg の
 * ままで dining 哲学者本来の startup シーケンスとして残る。 */
static void Philosopher_init(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  long p_id_arg = (n_args > 0) ? ((args[0]).tag == V_INT ? (args[0]).i : (long)((args[0]).f)) : (long)(0L);
  int p_lo = (n_args > 1) ? ((args[1]).obj_id) : (int)(-1);
  int p_hi = (n_args > 2) ? ((args[2]).obj_id) : (int)(-1);
  objects[self_id].fields[F_Philosopher_my_id] = mk_int((long)(p_id_arg));
  objects[self_id].fields[F_Philosopher_f_low] = mk_obj((int)(p_lo));
  objects[self_id].fields[F_Philosopher_f_high] = mk_obj((int)(p_hi));
  objects[self_id].fields[F_Philosopher_meals] = mk_int((long)(50L));
  objects[self_id].fields[F_Philosopher_meal_idx] = mk_int((long)(0L));
  objects[self_id].fields[F_Philosopher_state] = mk_int((long)(0L));
  objects[self_id].fields[F_Philosopher_done_to] = mk_obj(0);
  objects[self_id].fields[F_Philosopher_t_start] = mk_int((long)abcl_now_ms());
  enqueue(self_id, self_id, "try_eat", 0, NULL);
}

/* 5-arg 版: (id, low, high, meals, done_to_orch).  meals と done_to を上書きする
 * 以外は init() と同じ。dining5 bench はこれを使う。 */
static void Philosopher_init_phil(int self_id, int sender_id,
                                  value_t* args, int n_args) {
  (void)sender_id;
  long p_id     = (n_args > 0) ? args[0].i      : 0L;
  int  p_lo     = (n_args > 1) ? args[1].obj_id : -1;
  int  p_hi     = (n_args > 2) ? args[2].obj_id : -1;
  long meals    = (n_args > 3) ? args[3].i      : 50L;
  int  done_to  = (n_args > 4) ? args[4].obj_id : 0;
  objects[self_id].fields[F_Philosopher_my_id]    = mk_int(p_id);
  objects[self_id].fields[F_Philosopher_f_low]    = mk_obj(p_lo);
  objects[self_id].fields[F_Philosopher_f_high]   = mk_obj(p_hi);
  objects[self_id].fields[F_Philosopher_meals]    = mk_int(meals);
  objects[self_id].fields[F_Philosopher_meal_idx] = mk_int(0L);
  objects[self_id].fields[F_Philosopher_state]    = mk_int(0L);
  objects[self_id].fields[F_Philosopher_done_to]  = mk_obj(done_to);
  objects[self_id].fields[F_Philosopher_t_start]  = mk_int((long)abcl_now_ms());
  enqueue(self_id, self_id, "try_eat", 0, NULL);
}

static void Philosopher_try_eat(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  int _pid = (int)((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f));
  long meals_left = (objects[self_id].fields[F_Philosopher_meals]).i;
  if (meals_left == 0L) {
    int orch = objects[self_id].fields[F_Philosopher_done_to].obj_id;
    if (orch > 0) {
      long t_start = objects[self_id].fields[F_Philosopher_t_start].i;
      long elapsed = (long)abcl_now_ms() - t_start;
      enqueue(self_id, orch, "phil_done", 2,
              (value_t[]){ mk_int((long)_pid), mk_int(elapsed) });
    } else {
      abcl_phil_say(_pid, "finished — terminating (suicide)");
    }
    abcl_actor_suicide(self_id);
  } else {
    /* dining5 bench 用に narrative + 450ms sleep を抑制
     * (orch 設定時 = bench 中はサイレント+高速)。 */
    if (objects[self_id].fields[F_Philosopher_done_to].obj_id == 0) {
      abcl_phil_say(_pid, "thinking");
      sleep(450);
    }
    enqueue(self_id, objects[self_id].fields[F_Philosopher_f_low].obj_id, "acquire", 1, (value_t[]){mk_int((long)(((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f))))});
  }
}

static void Philosopher_fork_granted(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  value_t p_pid = (n_args > 0) ? args[0] : mk_int(0L);
  int _pid = (int)((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f));
  int bench_mode = (objects[self_id].fields[F_Philosopher_done_to].obj_id != 0);
  if (truthy(mk_int((long)(((long)((((objects[self_id].fields[F_Philosopher_state]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_state]).i : (long)((objects[self_id].fields[F_Philosopher_state]).f))) == (0L))))))) {
    /* got the first (low) fork — go for the second (high) one. */
    objects[self_id].fields[F_Philosopher_state] = mk_int((long)(1L));
    if (!bench_mode) abcl_phil_say(_pid, "took a fork");
    enqueue(self_id, objects[self_id].fields[F_Philosopher_f_high].obj_id, "acquire", 1, (value_t[]){mk_int((long)(((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f))))});
  } else {
    /* got the second fork — both held, so eat, then put them down. */
    if (!bench_mode) {
      abcl_phil_say(_pid, "took a fork");
      abcl_phil_say(_pid, "eating");
      sleep(450);   /* eating (holds both forks) — narration pace only */
    }
    objects[self_id].fields[F_Philosopher_meal_idx] = mk_int((long)(((((objects[self_id].fields[F_Philosopher_meal_idx]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_meal_idx]).i : (long)((objects[self_id].fields[F_Philosopher_meal_idx]).f))) + (1L))));
    objects[self_id].fields[F_Philosopher_meals] = mk_int((long)(((((objects[self_id].fields[F_Philosopher_meals]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_meals]).i : (long)((objects[self_id].fields[F_Philosopher_meals]).f))) - (1L))));
    enqueue(self_id, objects[self_id].fields[F_Philosopher_f_high].obj_id, "release", 1, (value_t[]){mk_int((long)(((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f))))});
    enqueue(self_id, objects[self_id].fields[F_Philosopher_f_low].obj_id, "release", 1, (value_t[]){mk_int((long)(((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f))))});
    if (!bench_mode) abcl_phil_say(_pid, "put down forks");
    objects[self_id].fields[F_Philosopher_state] = mk_int((long)(0L));
    enqueue(self_id, self_id, "try_eat", 0, NULL);
  }
}

static void Philosopher_fork_denied(int self_id, int sender_id, value_t* args, int n_args) {
  (void)args; (void)n_args; (void)sender_id;
  value_t p_pid = (n_args > 0) ? args[0] : mk_int(0L);
  int bench_mode = (objects[self_id].fields[F_Philosopher_done_to].obj_id != 0);
  if (truthy(mk_int((long)(((long)((((objects[self_id].fields[F_Philosopher_state]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_state]).i : (long)((objects[self_id].fields[F_Philosopher_state]).f))) == (1L))))))) {
    enqueue(self_id, objects[self_id].fields[F_Philosopher_f_low].obj_id, "release", 1, (value_t[]){mk_int((long)(((objects[self_id].fields[F_Philosopher_my_id]).tag == V_INT ? (objects[self_id].fields[F_Philosopher_my_id]).i : (long)((objects[self_id].fields[F_Philosopher_my_id]).f))))});
    objects[self_id].fields[F_Philosopher_state] = mk_int((long)(0L));
  } else {
  }
  /* Backoff so we don't livelock with the philosopher who has our fork.
   * Stagger by my_id so adjacent philosophers don't lock-step retry —
   * P_i sleeps 10 + i*8 ms (= 10, 18, 26, 34, 42).  Empirically a
   * uniform 5 ms backoff still livelocks the P2/P3 pair on the shared
   * F3 fork; staggering breaks the symmetry. */
  if (bench_mode) {
    int my_id_int = (int)objects[self_id].fields[F_Philosopher_my_id].i;
    sleep(10 + my_id_int * 8);
  }
  enqueue(self_id, self_id, "try_eat", 0, NULL);
}

static void dispatch_Philosopher(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (strcmp(method, "init") == 0) { Philosopher_init(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init_phil") == 0) { Philosopher_init_phil(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "try_eat") == 0) { Philosopher_try_eat(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "fork_granted") == 0) { Philosopher_fork_granted(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "fork_denied") == 0) { Philosopher_fork_denied(self_id, sender_id, args, n_args); return; }
  fprintf(stderr, "unknown method %s on Philosopher\n", method);
}

/* WebReceiver: a minimal demo actor reachable from the web bridge.  Its
 * `recv` method just prints the message it was sent, demonstrating a message
 * flowing  Mac actor -> HTTP -> Xinu web server -> AIPL actor mailbox. */
static void dispatch_WebReceiver(int self_id, int sender_id, const char* method,
                                 value_t* args, int n_args) {
  if (strcmp(method, "recv") == 0) {
    wait(print_mu);
    kprintf("[webactor] actor %d got message from %d: ", self_id, sender_id);
    signal(print_mu);
    if (n_args > 0) v_print(args[0]);   /* v_print takes print_mu itself */
    else { wait(print_mu); kprintf("(empty)\r\n"); signal(print_mu); }
    return;
  }
  /* "init" and anything else: no-op */
}

/* ============================================================
 * Load-balancer: Dispatcher + Worker classes.
 *
 *   Dispatcher.register_workers(w0, w1, w2, w3)
 *     Tell the dispatcher which Worker objects it owns.  Sent once
 *     at boot.  Up to LB_N_WORKERS (4) — extras are ignored.
 *
 *   Dispatcher.submit(work_ms, task_id)
 *     Pick the worker with the LEAST outstanding load, increment its
 *     load counter, and forward compute(work_ms, task_id) to it.
 *     Round-robin would be simpler but doesn't react to existing
 *     backlog; least-loaded keeps the slowest worker from piling up.
 *
 *   Dispatcher.done(worker_idx, task_id, result)
 *     Called by a Worker when it finishes a task.  Decrements that
 *     worker's load counter, bumps the dispatcher's completed counter,
 *     and logs the completion to serial.
 *
 *   Worker.bind(my_idx, dispatcher_id)
 *     Tell the worker its own slot index AND who to send `done` to.
 *
 *   Worker.compute(work_ms, task_id)
 *     Simulate work: sleep work_ms, then `result = work_ms*7 + task_id`
 *     (deterministic, easy to verify Mac-side).  Updates own counters,
 *     then enqueues done(my_idx, task_id, result) to the dispatcher.
 *
 * Priorities: Dispatcher=25 (highest) so routing decisions don't get
 * starved by Workers (22) or Philosophers (20).  Workers run above
 * Philosophers so a /api/loadbal/submit burst gets serviced even when
 * the dining demo is at full chatter.
 * ============================================================ */

enum { F_Disp_n, F_Disp_w0, F_Disp_w1, F_Disp_w2, F_Disp_w3,
       F_Disp_load0, F_Disp_load1, F_Disp_load2, F_Disp_load3,
       F_Disp_submitted, F_Disp_completed,
       F_Disp_rr,                     /* round-robin cursor for submit_jit */
       F_Disp_enabled,                /* bitmask: bit i = worker i enabled */
       F_Disp__N };

/* === Task tracking table ===========================================
 * Production load-balancers let callers fire-and-forget then ask
 * later "what happened to task #N?".  We keep a small circular
 * buffer of the most recent LB_TASKS_MAX submissions plus their
 * outcomes (submit-time, done-time, worker, result).
 *
 * Lookup is a linear scan.  256 entries × ~32 B = ~8 KB .bss; the
 * walk fits comfortably in L1 and the cost is well under the
 * HTTP-side round-trip noise.  Bumped from 64 to 256 so the
 * N-Queens benchmark's REPEAT * sizes * N submission count doesn't
 * scroll completed results out of the buffer before Mac collects
 * them via /api/loadbal/task?id=N. */
#define LB_TASKS_MAX 256

/* Per-task state for cancellation tracking.
 *   P = pending / running (worker should run it)
 *   D = done (worker finished and reported back)
 *   C = cancelled (worker should abort if still in chunked sleep) */
#define LB_STATE_PENDING   'P'
#define LB_STATE_DONE      'D'
#define LB_STATE_CANCELLED 'C'

typedef struct {
  int  task_id;        /* monotonic from dispatcher; 0 = empty slot */
  int  worker_obj;     /* dispatched-to worker obj_id, -1 if pending */
  long submit_ms;      /* abcl_now_ms() when dispatched */
  long done_ms;        /* 0 while pending */
  long result;
  char kind;           /* 's' = sleep compute, 'j' = JIT compute */
  char state;          /* LB_STATE_* */
  int  param;          /* work_ms or prog_id */
} lb_task_t;

/* Per-worker queue cap.  Used by the HTTP layer (via
 * abcl_loadbal_can_accept) to return 429 instead of letting the
 * dispatcher mailbox absorb arbitrary backlog. */
#define LB_WORKER_QUEUE_MAX 8

/* Forward decl — lb_latency_bucket uses F_Worker_hist_K constants
 * which aren't declared until the Worker class definition further
 * down, but Dispatcher_done (earlier in the file) needs to call it. */
static int lb_latency_bucket(long elapsed_ms);

static lb_task_t lb_tasks[LB_TASKS_MAX];
static int       lb_tasks_next = 0;     /* round-robin write cursor */

/* N-Queens push-aggregation: workers push `done` to the Dispatcher, which
 * sums the per-first-column counts here as they arrive — so the Mac reads
 * one running total instead of polling every task. */
static volatile int  g_nq_expected = 0;
static volatile int  g_nq_received = 0;
static volatile long g_nq_sum      = 0;

static lb_task_t* lb_task_alloc(int task_id, char kind, int param,
                                int worker_obj) {
  lb_task_t *t = &lb_tasks[lb_tasks_next];
  lb_tasks_next = (lb_tasks_next + 1) % LB_TASKS_MAX;
  t->task_id    = task_id;
  t->worker_obj = worker_obj;
  t->submit_ms  = (long)abcl_now_ms();
  t->done_ms    = 0;
  t->result     = 0;
  t->kind       = kind;
  t->state      = LB_STATE_PENDING;
  t->param      = param;
  return t;
}

static lb_task_t* lb_task_lookup(int task_id) {
  int i;
  for (i = 0; i < LB_TASKS_MAX; i++) {
    if (lb_tasks[i].task_id == task_id) return &lb_tasks[i];
  }
  return NULL;
}

static void init_fields_Dispatcher(int self_id) {
  int i;
  for (i = 0; i < F_Disp__N; i++)
    objects[self_id].fields[i] = mk_int(0L);
  /* Default: all workers enabled (bit 0..3 set). */
  objects[self_id].fields[F_Disp_enabled] = mk_int(0xFL);
}

static void Dispatcher_register_workers(int self_id, int sender_id,
                                        value_t* args, int n_args) {
  (void)sender_id;
  int count = 0;
  if (n_args > 0) { objects[self_id].fields[F_Disp_w0] = mk_obj(args[0].obj_id); count++; }
  if (n_args > 1) { objects[self_id].fields[F_Disp_w1] = mk_obj(args[1].obj_id); count++; }
  if (n_args > 2) { objects[self_id].fields[F_Disp_w2] = mk_obj(args[2].obj_id); count++; }
  if (n_args > 3) { objects[self_id].fields[F_Disp_w3] = mk_obj(args[3].obj_id); count++; }
  objects[self_id].fields[F_Disp_n] = mk_int((long)count);
  wait(print_mu);
  kprintf("[loadbal] dispatcher %d: registered %d workers\r\n", self_id, count);
  signal(print_mu);
}

/* Pick the lowest-load worker that is also enabled in F_Disp_enabled.
 * Returns slot index (0..n-1), or -1 if no worker is enabled. */
static int dispatcher_pick_least_loaded(int self_id) {
  int n = (int)objects[self_id].fields[F_Disp_n].i;
  long mask = objects[self_id].fields[F_Disp_enabled].i;
  int  best = -1;
  long best_load = 0;
  int  i;
  long loads[4] = {
    objects[self_id].fields[F_Disp_load0].i,
    objects[self_id].fields[F_Disp_load1].i,
    objects[self_id].fields[F_Disp_load2].i,
    objects[self_id].fields[F_Disp_load3].i,
  };
  for (i = 0; i < n && i < 4; i++) {
    if (!((mask >> i) & 1L)) continue;            /* paused — skip */
    if (best < 0 || loads[i] < best_load) {
      best = i; best_load = loads[i];
    }
  }
  return best;
}

static int dispatcher_worker_obj(int self_id, int slot) {
  switch (slot) {
  case 0: return objects[self_id].fields[F_Disp_w0].obj_id;
  case 1: return objects[self_id].fields[F_Disp_w1].obj_id;
  case 2: return objects[self_id].fields[F_Disp_w2].obj_id;
  case 3: return objects[self_id].fields[F_Disp_w3].obj_id;
  }
  return -1;
}

static void dispatcher_bump_load(int self_id, int slot, long delta) {
  switch (slot) {
  case 0: objects[self_id].fields[F_Disp_load0] =
            mk_int(objects[self_id].fields[F_Disp_load0].i + delta); break;
  case 1: objects[self_id].fields[F_Disp_load1] =
            mk_int(objects[self_id].fields[F_Disp_load1].i + delta); break;
  case 2: objects[self_id].fields[F_Disp_load2] =
            mk_int(objects[self_id].fields[F_Disp_load2].i + delta); break;
  case 3: objects[self_id].fields[F_Disp_load3] =
            mk_int(objects[self_id].fields[F_Disp_load3].i + delta); break;
  }
}

static void Dispatcher_submit(int self_id, int sender_id,
                              value_t* args, int n_args) {
  (void)sender_id;
  long work_ms = (n_args > 0) ? args[0].i : 100L;
  long task_id = (n_args > 1) ? args[1].i : 0L;
  if ((int)objects[self_id].fields[F_Disp_n].i <= 0) {
    wait(print_mu);
    kprintf("[loadbal] submit dropped: no workers registered\r\n");
    signal(print_mu);
    return;
  }
  int best = dispatcher_pick_least_loaded(self_id);
  if (best < 0) {
    wait(print_mu);
    kprintf("[loadbal] submit dropped task=%ld: all workers paused\r\n",
            task_id);
    signal(print_mu);
    return;
  }
  int worker_obj = dispatcher_worker_obj(self_id, best);
  long old_load = (best == 0) ? objects[self_id].fields[F_Disp_load0].i
                : (best == 1) ? objects[self_id].fields[F_Disp_load1].i
                : (best == 2) ? objects[self_id].fields[F_Disp_load2].i
                              : objects[self_id].fields[F_Disp_load3].i;
  dispatcher_bump_load(self_id, best, +1);
  objects[self_id].fields[F_Disp_submitted] =
    mk_int(objects[self_id].fields[F_Disp_submitted].i + 1);

  /* Record task for /api/loadbal/task?id= later. */
  lb_task_alloc((int)task_id, 's', (int)work_ms, worker_obj);

  wait(print_mu);
  kprintf("[loadbal] dispatch task=%ld -> worker idx=%d obj=%d (load %ld->%ld)\r\n",
          task_id, best, worker_obj, old_load, old_load + 1);
  signal(print_mu);

  enqueue(self_id, worker_obj, "compute", 2,
          (value_t[]){ mk_int(work_ms), mk_int(task_id) });
}

static void Dispatcher_done(int self_id, int sender_id,
                            value_t* args, int n_args) {
  (void)sender_id;
  long widx    = (n_args > 0) ? args[0].i : -1L;
  long task_id = (n_args > 1) ? args[1].i : -1L;
  long result  = (n_args > 2) ? args[2].i : 0L;
  dispatcher_bump_load(self_id, (int)widx, -1);
  objects[self_id].fields[F_Disp_completed] =
    mk_int(objects[self_id].fields[F_Disp_completed].i + 1);
  /* Stamp the task table + bump that worker's latency-histogram
   * bucket — visible via /api/loadbal/stats. */
  lb_task_t *t = lb_task_lookup((int)task_id);
  long elapsed = 0;
  if (NULL != t) {
    t->done_ms = (long)abcl_now_ms();
    t->result  = result;
    /* Don't overwrite a CANCELLED state — the worker explicitly
     * acknowledged the cancel.  Otherwise mark DONE. */
    if (t->state != LB_STATE_CANCELLED) t->state = LB_STATE_DONE;
    /* N-Queens push-aggregation: accumulate the partial count here as the
     * worker reports it (no Mac-side per-task polling needed). */
    if (t->kind == 'n' && t->state == LB_STATE_DONE) {
      g_nq_sum += result;
      g_nq_received++;
    }
    elapsed = t->done_ms - t->submit_ms;
    if (elapsed < 0) elapsed = 0;
    int worker_obj = dispatcher_worker_obj(self_id, (int)widx);
    if (worker_obj >= 0) {
      int bkt = lb_latency_bucket(elapsed);
      objects[worker_obj].fields[bkt] =
        mk_int(objects[worker_obj].fields[bkt].i + 1);
    }
  }
  wait(print_mu);
  kprintf("[loadbal] complete task=%ld worker=%ld result=%ld elapsed=%ld (done=%ld/%ld)\r\n",
          task_id, widx, result, elapsed,
          objects[self_id].fields[F_Disp_completed].i,
          objects[self_id].fields[F_Disp_submitted].i);
  signal(print_mu);
}

/* Mark a task CANCELLED.  The worker's compute loop polls task state
 * during its chunked sleep — see Worker_compute.  For JIT tasks
 * cancel only catches the BEFORE-compile window because the JIT call
 * itself is synchronous.  HTTP: POST /api/loadbal/cancel?id=N */
static void Dispatcher_cancel_task(int self_id, int sender_id,
                                   value_t* args, int n_args) {
  (void)self_id; (void)sender_id;
  if (n_args < 1) return;
  long task_id = args[0].i;
  lb_task_t *t = lb_task_lookup((int)task_id);
  if (NULL == t || t->task_id == 0) {
    wait(print_mu);
    kprintf("[loadbal] cancel: task %ld not in table\r\n", task_id);
    signal(print_mu);
    return;
  }
  if (t->state == LB_STATE_DONE) {
    wait(print_mu);
    kprintf("[loadbal] cancel: task %ld already DONE — ignored\r\n", task_id);
    signal(print_mu);
    return;
  }
  t->state = LB_STATE_CANCELLED;
  wait(print_mu);
  kprintf("[loadbal] cancel: task %ld marked CANCELLED\r\n", task_id);
  signal(print_mu);
}

/* Toggle a worker's enabled bit.  args[0] = slot, args[1] = on (0/1).
 * Paused workers receive no new dispatches; in-flight tasks complete
 * normally (load counter still decrements via done). */
static void Dispatcher_set_enabled(int self_id, int sender_id,
                                   value_t* args, int n_args) {
  (void)sender_id;
  if (n_args < 2) return;
  long slot = args[0].i;
  long on   = args[1].i;
  if (slot < 0 || slot >= LB_N_WORKERS) return;
  long mask = objects[self_id].fields[F_Disp_enabled].i;
  if (on) mask |=  (1L << slot);
  else    mask &= ~(1L << slot);
  objects[self_id].fields[F_Disp_enabled] = mk_int(mask);
  wait(print_mu);
  kprintf("[loadbal] worker idx=%ld %s (mask=0x%lx)\r\n",
          slot, on ? "RESUMED" : "PAUSED", mask);
  signal(print_mu);
}

/* Sticky routing: caller passes a precomputed hash, dispatcher maps
 * hash%n_workers to a slot.  Same hash -> same slot (so a cluster of
 * tasks sharing a key all land on the same worker, useful for cache
 * locality or stateful workers).  If that slot is paused, walk
 * forward to the next enabled one — preserves sticky semantics when
 * the target is alive, gracefully degrades when not. */
static void Dispatcher_submit_sticky(int self_id, int sender_id,
                                     value_t* args, int n_args) {
  (void)sender_id;
  long work_ms  = (n_args > 0) ? args[0].i : 100L;
  long task_id  = (n_args > 1) ? args[1].i : 0L;
  long key_hash = (n_args > 2) ? args[2].i : 0L;
  int n = (int)objects[self_id].fields[F_Disp_n].i;
  if (n <= 0) return;
  long mask = objects[self_id].fields[F_Disp_enabled].i;
  unsigned long h = (unsigned long)(key_hash < 0 ? -key_hash : key_hash);
  int slot = (int)(h % (unsigned long)n);
  int tries;
  for (tries = 0; tries < n; tries++) {
    if ((mask >> slot) & 1L) break;
    slot = (slot + 1) % n;
  }
  if (!((mask >> slot) & 1L)) {
    wait(print_mu);
    kprintf("[loadbal] submit_sticky dropped task=%ld key_hash=%ld: all paused\r\n",
            task_id, key_hash);
    signal(print_mu);
    return;
  }
  int worker_obj = dispatcher_worker_obj(self_id, slot);
  dispatcher_bump_load(self_id, slot, +1);
  objects[self_id].fields[F_Disp_submitted] =
    mk_int(objects[self_id].fields[F_Disp_submitted].i + 1);
  lb_task_alloc((int)task_id, 's', (int)work_ms, worker_obj);
  wait(print_mu);
  kprintf("[loadbal] sticky task=%ld key_hash=%ld -> worker idx=%d obj=%d\r\n",
          task_id, key_hash, slot, worker_obj);
  signal(print_mu);
  enqueue(self_id, worker_obj, "compute", 2,
          (value_t[]){ mk_int(work_ms), mk_int(task_id) });
}

/* N-Queens dispatch — round-robin first_col across workers.  Same
 * routing as submit_jit (uses F_Disp_rr cursor).  Each task receives
 * (n, first_col, task_id), worker calls abcl_nq_count_partial. */
static void Dispatcher_submit_nq(int self_id, int sender_id,
                                 value_t* args, int n_args) {
  (void)sender_id;
  long n         = (n_args > 0) ? args[0].i : 8L;
  long first_col = (n_args > 1) ? args[1].i : 0L;
  long task_id   = (n_args > 2) ? args[2].i : 0L;
  int n_workers = (int)objects[self_id].fields[F_Disp_n].i;
  if (n_workers <= 0) return;
  long mask = objects[self_id].fields[F_Disp_enabled].i;
  int slot = -1, tries;
  for (tries = 0; tries < n_workers; tries++) {
    int s = (int)(objects[self_id].fields[F_Disp_rr].i % n_workers);
    objects[self_id].fields[F_Disp_rr] =
      mk_int(objects[self_id].fields[F_Disp_rr].i + 1);
    if ((mask >> s) & 1L) { slot = s; break; }
  }
  if (slot < 0) return;
  int worker_obj = dispatcher_worker_obj(self_id, slot);
  dispatcher_bump_load(self_id, slot, +1);
  objects[self_id].fields[F_Disp_submitted] =
    mk_int(objects[self_id].fields[F_Disp_submitted].i + 1);
  lb_task_alloc((int)task_id, 'n', (int)first_col, worker_obj);
  enqueue(self_id, worker_obj, "compute_nq", 3,
          (value_t[]){ mk_int(n), mk_int(first_col), mk_int(task_id) });
}

/* Worker restart — kill old thread, allocate fresh actor, swap into
 * the dispatcher's worker slot.  Uses a NEW obj_id (n_objects bumps),
 * which is permanent — restarting all 4 workers consumes 4 of the 9
 * post-boot MAX_OBJECTS=16 budget.  Acceptable for occasional ops
 * use; a long-running restart loop would exhaust the table.
 *
 * Pending tasks in the old worker's mailbox are LOST (still counted
 * as submitted-not-completed forever).  Done-callbacks from the old
 * worker after kill are also lost.  The load counter is reset to 0
 * for the slot to reflect the fresh start. */
static void Dispatcher_submit_nq(int self_id, int sender_id,
                                 value_t* args, int n_args);
int abcl_nq_count_partial(int n, int first_col);

static void Dispatcher_restart_worker(int self_id, int sender_id,
                                      value_t* args, int n_args) {
  (void)sender_id;
  if (n_args < 1) return;
  long slot = args[0].i;
  if (slot < 0 || slot >= LB_N_WORKERS) return;
  if (n_objects >= MAX_OBJECTS) {
    wait(print_mu);
    kprintf("[loadbal] restart_worker idx=%ld FAILED: actor table full (%d/%d)\r\n",
            slot, n_objects, MAX_OBJECTS);
    signal(print_mu);
    return;
  }
  int old_obj = dispatcher_worker_obj(self_id, (int)slot);
  if (old_obj > 0 && objects[old_obj].tid > 0 && !objects[old_obj].dead) {
    kill(objects[old_obj].tid);
    objects[old_obj].dead = 1;
  }
  int new_obj = alloc_obj(CLASS_Worker, 0, NULL);
  abcl_object_protect(new_obj, 1);
  spawn_actor(new_obj);
  /* Swap into dispatcher's slot + reset load counter. */
  switch ((int)slot) {
  case 0: objects[self_id].fields[F_Disp_w0] = mk_obj(new_obj);
          objects[self_id].fields[F_Disp_load0] = mk_int(0L); break;
  case 1: objects[self_id].fields[F_Disp_w1] = mk_obj(new_obj);
          objects[self_id].fields[F_Disp_load1] = mk_int(0L); break;
  case 2: objects[self_id].fields[F_Disp_w2] = mk_obj(new_obj);
          objects[self_id].fields[F_Disp_load2] = mk_int(0L); break;
  case 3: objects[self_id].fields[F_Disp_w3] = mk_obj(new_obj);
          objects[self_id].fields[F_Disp_load3] = mk_int(0L); break;
  }
  g_loadbal_w[(int)slot] = new_obj;
  /* Bind the fresh worker so it knows its slot index + dispatcher. */
  abcl_enqueue(-1, new_obj, "bind", 2,
               (value_t[]){ mk_int(slot), mk_obj(self_id) });
  wait(print_mu);
  kprintf("[loadbal] worker idx=%ld restarted: old_obj=%d -> new_obj=%d\r\n",
          slot, old_obj, new_obj);
  signal(print_mu);
}

/* Routes a JIT job using round-robin instead of least-loaded.
 *
 * For sleep-based `submit` the work is slower than dispatch, so by the
 * time the second submit arrives worker 0 still has load=1 and worker 1
 * wins — distribution looks even.
 *
 * For JIT, each task is fast enough that the worker often finishes (and
 * sends `done` decrementing its load) BEFORE the next submit arrives.
 * Least-loaded then breaks ties in favour of slot 0, so every JIT task
 * lands on the same worker.  Round-robin sidesteps this and gives an
 * honest demonstration of 4 worker threads JIT-compiling in parallel. */
static void Dispatcher_submit_jit(int self_id, int sender_id,
                                  value_t* args, int n_args) {
  (void)sender_id;
  long prog_id = (n_args > 0) ? args[0].i : 0L;
  long task_id = (n_args > 1) ? args[1].i : 0L;
  int n = (int)objects[self_id].fields[F_Disp_n].i;
  if (n <= 0) return;
  long mask = objects[self_id].fields[F_Disp_enabled].i;
  /* Round-robin among ENABLED workers only — paused workers are
   * skipped so a /api/loadbal/pause during a JIT burst still drains
   * cleanly to the survivors. */
  int slot = -1, tries;
  for (tries = 0; tries < n; tries++) {
    int s = (int)(objects[self_id].fields[F_Disp_rr].i % n);
    objects[self_id].fields[F_Disp_rr] =
      mk_int(objects[self_id].fields[F_Disp_rr].i + 1);
    if ((mask >> s) & 1L) { slot = s; break; }
  }
  if (slot < 0) {
    wait(print_mu);
    kprintf("[loadbal] submit_jit dropped task=%ld: all workers paused\r\n",
            task_id);
    signal(print_mu);
    return;
  }
  int worker_obj = dispatcher_worker_obj(self_id, slot);
  dispatcher_bump_load(self_id, slot, +1);
  objects[self_id].fields[F_Disp_submitted] =
    mk_int(objects[self_id].fields[F_Disp_submitted].i + 1);

  lb_task_alloc((int)task_id, 'j', (int)prog_id, worker_obj);

  enqueue(self_id, worker_obj, "compute_jit", 2,
          (value_t[]){ mk_int(prog_id), mk_int(task_id) });
}

static void dispatch_Dispatcher(int self_id, int sender_id, const char* method,
                                value_t* args, int n_args) {
  if (strcmp(method, "register_workers") == 0)
  { Dispatcher_register_workers(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "submit") == 0)
  { Dispatcher_submit(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "submit_jit") == 0)
  { Dispatcher_submit_jit(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "submit_sticky") == 0)
  { Dispatcher_submit_sticky(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "done") == 0)
  { Dispatcher_done(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "set_enabled") == 0)
  { Dispatcher_set_enabled(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "cancel_task") == 0)
  { Dispatcher_cancel_task(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "restart_worker") == 0)
  { Dispatcher_restart_worker(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "submit_nq") == 0)
  { Dispatcher_submit_nq(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init") == 0) return;
}

enum { F_Worker_my_idx, F_Worker_dispatcher, F_Worker_done,
       F_Worker_busy_ms, F_Worker_last_n, F_Worker_last_result,
       /* Latency histogram: 7 buckets.  Caller (Dispatcher_done)
        * computes elapsed_ms = done_ms - submit_ms and bumps the
        * right bucket on the worker.  These give a coarse p50/p95
        * read without a full sorted list. */
       F_Worker_hist_0,   /* <50 ms */
       F_Worker_hist_1,   /* 50-99 ms */
       F_Worker_hist_2,   /* 100-199 ms */
       F_Worker_hist_3,   /* 200-499 ms */
       F_Worker_hist_4,   /* 500-999 ms */
       F_Worker_hist_5,   /* 1000-1999 ms */
       F_Worker_hist_6,   /* >=2000 ms */
       F_Worker__N };

/* Map elapsed_ms to a histogram bucket field index (F_Worker_hist_0..6). */
static int lb_latency_bucket(long elapsed_ms) {
  if (elapsed_ms < 50)   return F_Worker_hist_0;
  if (elapsed_ms < 100)  return F_Worker_hist_1;
  if (elapsed_ms < 200)  return F_Worker_hist_2;
  if (elapsed_ms < 500)  return F_Worker_hist_3;
  if (elapsed_ms < 1000) return F_Worker_hist_4;
  if (elapsed_ms < 2000) return F_Worker_hist_5;
  return F_Worker_hist_6;
}

static void init_fields_Worker(int self_id) {
  int i;
  for (i = 0; i < F_Worker__N; i++)
    objects[self_id].fields[i] = mk_int(0L);
}

static void Worker_bind(int self_id, int sender_id,
                        value_t* args, int n_args) {
  (void)sender_id;
  if (n_args < 2) return;
  objects[self_id].fields[F_Worker_my_idx]     = mk_int(args[0].i);
  objects[self_id].fields[F_Worker_dispatcher] = mk_obj(args[1].obj_id);
  wait(print_mu);
  kprintf("[loadbal] worker %d bound idx=%ld dispatcher=%d\r\n",
          self_id, args[0].i, args[1].obj_id);
  signal(print_mu);
}

static void Worker_compute(int self_id, int sender_id,
                           value_t* args, int n_args) {
  (void)sender_id;
  long work_ms = (n_args > 0) ? args[0].i : 0L;
  long task_id = (n_args > 1) ? args[1].i : 0L;
  long idx     = objects[self_id].fields[F_Worker_my_idx].i;
  int  disp    = objects[self_id].fields[F_Worker_dispatcher].obj_id;

  wait(print_mu);
  kprintf("[loadbal] worker idx=%ld obj=%d task=%ld start (%ld ms)\r\n",
          idx, self_id, task_id, work_ms);
  signal(print_mu);

  /* Chunked sleep so /api/loadbal/cancel?id=N can interrupt mid-task.
   * 50 ms chunks give cancel a worst-case ~50 ms reaction time, which
   * is fine alongside our HTTP RTT of ~5 ms.  task_aborted flips to 1
   * if the task table entry is marked CANCELLED. */
  int  task_aborted = 0;
  long slept = 0;
  while (slept < work_ms) {
    lb_task_t *t = lb_task_lookup((int)task_id);
    if (NULL != t && t->state == LB_STATE_CANCELLED) {
      task_aborted = 1;
      break;
    }
    long chunk = (work_ms - slept) > 50 ? 50 : (work_ms - slept);
    sleep((uint)chunk);
    slept += chunk;
  }

  long result = task_aborted ? -1L : (work_ms * 7L + task_id);
  objects[self_id].fields[F_Worker_last_n]      = mk_int(work_ms);
  objects[self_id].fields[F_Worker_last_result] = mk_int(result);
  objects[self_id].fields[F_Worker_done] =
    mk_int(objects[self_id].fields[F_Worker_done].i + 1);
  objects[self_id].fields[F_Worker_busy_ms] =
    mk_int(objects[self_id].fields[F_Worker_busy_ms].i + slept);

  wait(print_mu);
  if (task_aborted) {
    kprintf("[loadbal] worker idx=%ld task=%ld CANCELLED at %ld/%ld ms\r\n",
            idx, task_id, slept, work_ms);
  } else {
    kprintf("[loadbal] worker idx=%ld task=%ld done result=%ld\r\n",
            idx, task_id, result);
  }
  signal(print_mu);

  enqueue(self_id, disp, "done", 3,
          (value_t[]){ mk_int(idx), mk_int(task_id), mk_int(result) });
}

/* Static JIT-test programs.  cc_mvp doesn't have local vars yet so the
 * source must be self-contained (literals + builtin calls only).
 * Each worker JIT-compiles+executes one of these per task, exercising
 * memget/memfree, codegen, and the builtin-call ABI under load. */
static const char *jit_progs[] = {
  "int main() { return 42; }",                              /* trivial */
  "int main() { return 100 + 23; }",                        /* binop  */
  "int main() { return now_ms(); }",                        /* builtin */
  "int main() { return actor_count() + 1000; }",            /* builtin+binop */
  "int main() { return now_ms() + actor_count(); }",        /* 2 builtins */
};
#define N_JIT_PROGS ((int)(sizeof(jit_progs) / sizeof(jit_progs[0])))

static void Worker_compute_jit(int self_id, int sender_id,
                               value_t* args, int n_args) {
  (void)sender_id;
  long prog_id = (n_args > 0) ? args[0].i : 0L;
  long task_id = (n_args > 1) ? args[1].i : 0L;
  if (prog_id < 0 || prog_id >= N_JIT_PROGS) prog_id = 0;
  long idx  = objects[self_id].fields[F_Worker_my_idx].i;
  int  disp = objects[self_id].fields[F_Worker_dispatcher].obj_id;

  /* Cancellation window: only effective BEFORE the JIT call.  The
   * compile+execute itself is synchronous and cannot be interrupted
   * from the dispatcher side. */
  {
    lb_task_t *t = lb_task_lookup((int)task_id);
    if (NULL != t && t->state == LB_STATE_CANCELLED) {
      wait(print_mu);
      kprintf("[loadbal/jit] worker idx=%ld task=%ld CANCELLED before run\r\n",
              idx, task_id);
      signal(print_mu);
      enqueue(self_id, disp, "done", 3,
              (value_t[]){ mk_int(idx), mk_int(task_id), mk_int(-1L) });
      return;
    }
  }

  extern int cc_mvp_compile_and_run(const char *, long *, int *);
  long result = 0;
  int  code_size = 0;
  int  rc = cc_mvp_compile_and_run(jit_progs[(int)prog_id],
                                   &result, &code_size);

  wait(print_mu);
  if (rc == 0) {
    kprintf("[loadbal/jit] worker idx=%ld task=%ld prog=%ld result=%ld bytes=%d\r\n",
            idx, task_id, prog_id, result, code_size);
  } else {
    kprintf("[loadbal/jit] worker idx=%ld task=%ld prog=%ld FAILED rc=%d\r\n",
            idx, task_id, prog_id, rc);
    result = -1;
  }
  signal(print_mu);

  objects[self_id].fields[F_Worker_last_n]      = mk_int(prog_id);
  objects[self_id].fields[F_Worker_last_result] = mk_int(result);
  objects[self_id].fields[F_Worker_done] =
    mk_int(objects[self_id].fields[F_Worker_done].i + 1);
  /* For JIT tasks, busy_ms re-purposed as cumulative code bytes emitted. */
  objects[self_id].fields[F_Worker_busy_ms] =
    mk_int(objects[self_id].fields[F_Worker_busy_ms].i + code_size);

  enqueue(self_id, disp, "done", 3,
          (value_t[]){ mk_int(idx), mk_int(task_id), mk_int(result) });
}

/* N-Queens partial work — computes nqueens_count_partial(n, first_col)
 * on this worker thread, reports the solution count as `result` via
 * done.  Used by the /api/loadbal/nqueens demo / benchmark. */
static void Worker_compute_nq(int self_id, int sender_id,
                              value_t* args, int n_args) {
  (void)sender_id;
  long n         = (n_args > 0) ? args[0].i : 8L;
  long first_col = (n_args > 1) ? args[1].i : 0L;
  long task_id   = (n_args > 2) ? args[2].i : 0L;
  long idx       = objects[self_id].fields[F_Worker_my_idx].i;
  int  disp      = objects[self_id].fields[F_Worker_dispatcher].obj_id;

  long t0 = (long)abcl_now_ms();
  long result = (long)abcl_nq_count_partial((int)n, (int)first_col);
  long elapsed = (long)abcl_now_ms() - t0;

  objects[self_id].fields[F_Worker_last_n]      = mk_int(n);
  objects[self_id].fields[F_Worker_last_result] = mk_int(result);
  objects[self_id].fields[F_Worker_done] =
    mk_int(objects[self_id].fields[F_Worker_done].i + 1);
  objects[self_id].fields[F_Worker_busy_ms] =
    mk_int(objects[self_id].fields[F_Worker_busy_ms].i + elapsed);

  wait(print_mu);
  kprintf("[loadbal/nq] worker idx=%ld task=%ld nq(%ld,%ld)=%ld in %ldms\r\n",
          idx, task_id, n, first_col, result, elapsed);
  signal(print_mu);

  enqueue(self_id, disp, "done", 3,
          (value_t[]){ mk_int(idx), mk_int(task_id), mk_int(result) });
}

static void dispatch_Worker(int self_id, int sender_id, const char* method,
                            value_t* args, int n_args) {
  if (strcmp(method, "bind") == 0)
  { Worker_bind(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "compute") == 0)
  { Worker_compute(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "compute_jit") == 0)
  { Worker_compute_jit(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "compute_nq") == 0)
  { Worker_compute_nq(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init") == 0) return;
}

/* === accessors used by apps/webactor.c =================================== */
int abcl_loadbal_dispatcher_id(void) { return g_loadbal_disp; }
int abcl_loadbal_worker_count(void)  { return LB_N_WORKERS; }
int abcl_loadbal_worker_id(int idx) {
  if (idx < 0 || idx >= LB_N_WORKERS) return -1;
  return g_loadbal_w[idx];
}
void abcl_loadbal_submit(int work_ms, int task_id) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "submit", 2,
               (value_t[]){ mk_int((long)work_ms), mk_int((long)task_id) });
}

void abcl_loadbal_submit_jit(int prog_id, int task_id) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "submit_jit", 2,
               (value_t[]){ mk_int((long)prog_id), mk_int((long)task_id) });
}
int abcl_loadbal_n_progs(void) { return N_JIT_PROGS; }

void abcl_loadbal_set_enabled(int worker_idx, int on) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "set_enabled", 2,
               (value_t[]){ mk_int((long)worker_idx), mk_int((long)on) });
}

void abcl_loadbal_cancel(int task_id) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "cancel_task", 1,
               (value_t[]){ mk_int((long)task_id) });
}

void abcl_loadbal_submit_sticky(int work_ms, int task_id, int key_hash) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "submit_sticky", 3,
               (value_t[]){ mk_int((long)work_ms),
                            mk_int((long)task_id),
                            mk_int((long)key_hash) });
}

void abcl_loadbal_restart_worker(int slot) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "restart_worker", 1,
               (value_t[]){ mk_int((long)slot) });
}

void abcl_loadbal_submit_nq(int n, int first_col, int task_id) {
  if (g_loadbal_disp < 0) return;
  abcl_enqueue(-1, g_loadbal_disp, "submit_nq", 3,
               (value_t[]){ mk_int((long)n), mk_int((long)first_col),
                            mk_int((long)task_id) });
}

/* N-Queens push-aggregation control (called from the HTTP layer):
 *  _begin()  reset the accumulator before submitting a job's columns
 *  _expect() record how many partial results to wait for
 *  _result() read {expected, received, total}; returns 1 when complete */
void abcl_loadbal_nq_begin(void) {
  g_nq_sum = 0; g_nq_received = 0; g_nq_expected = 0;
}
void abcl_loadbal_nq_expect(int n) { g_nq_expected = n; }
int abcl_loadbal_nq_result(int *expected, int *received, long *total) {
  if (expected) *expected = g_nq_expected;
  if (received) *received = g_nq_received;
  if (total)    *total    = g_nq_sum;
  return (g_nq_expected > 0 && g_nq_received >= g_nq_expected) ? 1 : 0;
}

/* N-Queens partial counter — counts solutions where the FIRST row's
 * queen is placed at column `first_col`.  Caller sums over
 * first_col=0..n-1 to get the total count.  Embarrassingly parallel:
 * each first_col job is independent and they're roughly equal in
 * cost (modulo board symmetry), making it a clean load-balance demo.
 *
 * O(n!) worst case, but the branching prune cuts it sharply.  For
 * n=8 the total is 92 solutions and the work fits in < 100 ms even
 * on a single Pi 3 worker.  For n=11 it's 2680 solutions and starts
 * to be visible (~seconds). */
static int nq_recurse(int n, int row, int *cols) {
  if (row == n) return 1;
  int count = 0, c;
  for (c = 0; c < n; c++) {
    int ok = 1;
    int r;
    for (r = 0; r < row; r++) {
      int dc = cols[r] - c;
      if (dc < 0) dc = -dc;
      if (cols[r] == c || dc == row - r) { ok = 0; break; }
    }
    if (ok) {
      cols[row] = c;
      count += nq_recurse(n, row + 1, cols);
    }
  }
  return count;
}

int abcl_nq_count_partial(int n, int first_col) {
  int cols[16];
  if (n < 1 || n > 16) return 0;
  if (first_col < 0 || first_col >= n) return 0;
  cols[0] = first_col;
  return nq_recurse(n, 1, cols);
}

/* djb2 — small portable string hash for sticky routing keys.
 * Caller passes the resulting int to abcl_loadbal_submit_sticky. */
int abcl_loadbal_hash_key(const char *s, int len) {
  unsigned long h = 5381;
  int i;
  for (i = 0; i < len; i++)
    h = (h * 33u) + (unsigned char)s[i];
  return (int)(h & 0x7FFFFFFFu);
}

/* Token-bucket rate limit.  Refills based on elapsed milliseconds
 * since the last call; bucket caps at capacity.  Returns 1 if a token
 * was deducted (caller may proceed) or 0 if throttled.  Bypassed
 * entirely when caller passes priority="high" — see webactor's /submit
 * route.  All state in file-scope statics above. */
int abcl_loadbal_rate_check(void) {
  long now = (long)abcl_now_ms();
  long elapsed = now - g_rate_last_refill;
  if (elapsed > 0) {
    /* refill = (elapsed_ms * per_sec) / 1000.  Was /100, from the same
     * 100 Hz belief, so the bucket refilled 10x too fast. */
    long refill = (elapsed * g_rate_per_sec) / 1000;
    if (refill > 0) {
      g_rate_tokens += refill;
      if (g_rate_tokens > g_rate_capacity) g_rate_tokens = g_rate_capacity;
      g_rate_last_refill = now;
    }
  }
  if (g_rate_tokens >= 1) {
    g_rate_tokens--;
    return 1;
  }
  g_rate_throttled++;
  return 0;
}

void abcl_loadbal_rate_set(int per_sec, int capacity) {
  if (per_sec  > 0) g_rate_per_sec  = (long)per_sec;
  if (capacity > 0) g_rate_capacity = (long)capacity;
  if (g_rate_tokens > g_rate_capacity) g_rate_tokens = g_rate_capacity;
}

int abcl_loadbal_rate_stats(char *buf, int cap) {
  if (cap < 200) return 0;
  return sprintf(buf,
    "rate_limit: per_sec=%ld capacity=%ld tokens=%ld throttled=%ld\n"
    "task_timeout: ms=%ld auto_cancelled=%ld\n",
    g_rate_per_sec, g_rate_capacity, g_rate_tokens, g_rate_throttled,
    g_task_timeout_ms, g_task_timed_out);
}

void abcl_loadbal_timeout_set(int ms) {
  if (ms >= 0) g_task_timeout_ms = (long)ms;
}

/* Walk the task table; any PENDING task whose age exceeds
 * g_task_timeout_ms gets marked CANCELLED.  Called from Collector.tick
 * so it shares the GC actor's heartbeat (no extra thread).  Returns
 * the number freshly expired this call (for kprintf in caller). */
int abcl_loadbal_timeout_scan(void) {
  if (g_task_timeout_ms <= 0) return 0;
  long now_ms = (long)abcl_now_ms();
  int  expired = 0;
  int  i;
  for (i = 0; i < LB_TASKS_MAX; i++) {
    lb_task_t *t = &lb_tasks[i];
    if (t->task_id == 0) continue;
    if (t->state != LB_STATE_PENDING) continue;
    long age = now_ms - t->submit_ms;
    if (age < g_task_timeout_ms) continue;
    t->state = LB_STATE_CANCELLED;
    expired++;
    g_task_timed_out++;
  }
  return expired;
}

/* HTTP backpressure check.  Returns 1 if at least one enabled worker
 * has load < LB_WORKER_QUEUE_MAX, else 0.  Inherently racey (load may
 * change between this check and the subsequent submit) but good
 * enough to refuse runaway submitters at the HTTP boundary. */
int abcl_loadbal_can_accept(void) {
  if (g_loadbal_disp < 0) return 0;
  int  d = g_loadbal_disp;
  int  n = (int)objects[d].fields[F_Disp_n].i;
  long mask = objects[d].fields[F_Disp_enabled].i;
  long loads[4] = {
    objects[d].fields[F_Disp_load0].i,
    objects[d].fields[F_Disp_load1].i,
    objects[d].fields[F_Disp_load2].i,
    objects[d].fields[F_Disp_load3].i,
  };
  int i;
  for (i = 0; i < n && i < 4; i++) {
    if (!((mask >> i) & 1L)) continue;
    if (loads[i] < LB_WORKER_QUEUE_MAX) return 1;
  }
  return 0;
}

/* Fill `buf` with one task row for /api/loadbal/task?id=N.
 * Returns bytes written, or 0 if no such task. */
int abcl_loadbal_task_info(int task_id, char *buf, int cap) {
  if (cap < 200) return 0;
  lb_task_t *t = lb_task_lookup(task_id);
  if (NULL == t || t->task_id == 0) {
    return sprintf(buf, "task_id=%d not_found\n", task_id);
  }
  long elapsed = (t->done_ms > 0) ? (t->done_ms - t->submit_ms) : -1L;
  const char *state_str =
    (t->state == LB_STATE_CANCELLED) ? "CANCELLED" :
    (t->state == LB_STATE_DONE     ) ? "DONE"      :
                                       "PENDING";
  return sprintf(buf,
    "task_id=%d worker_obj=%d kind=%c param=%d\n"
    "submit_ms=%ld done_ms=%ld elapsed_ms=%ld\n"
    "state=%s result=%ld\n",
    t->task_id, t->worker_obj, t->kind, t->param,
    t->submit_ms, t->done_ms, elapsed,
    state_str, t->result);
}

/* One-shot bootstrap of the load-balancer actors.  Safe to call multiple
 * times — only the first call creates the actors.  Called from
 * webactor_autostart so the load-balancer exists even when aipl_main
 * isn't running (the default boot path runs only the AIPL-RPC + HTTP
 * servers, not the full dining demo). */
static int g_loadbal_inited = 0;
void abcl_loadbal_init(void) {
  if (g_loadbal_inited) return;
  /* Runtime mutexes must already be up — abcl_web_init() / abcl_rt_init()
   * sets them.  We don't call abcl_rt_init_once() here so the caller
   * controls ordering. */
  int i;
  g_loadbal_disp = alloc_obj(CLASS_Dispatcher, 0, NULL);
  for (i = 0; i < LB_N_WORKERS; i++)
    g_loadbal_w[i] = alloc_obj(CLASS_Worker, 0, NULL);
  /* Spawn the threads BEFORE sending bind/register so the messages
   * have a thread to consume them. */
  spawn_actor(g_loadbal_disp);
  for (i = 0; i < LB_N_WORKERS; i++) spawn_actor(g_loadbal_w[i]);
  /* Bind each worker to its slot + dispatcher (queued) */
  for (i = 0; i < LB_N_WORKERS; i++) {
    abcl_enqueue(-1, g_loadbal_w[i], "bind", 2,
                 (value_t[]){ mk_int((long)i), mk_obj(g_loadbal_disp) });
  }
  /* Register all workers in dispatcher */
  abcl_enqueue(-1, g_loadbal_disp, "register_workers", LB_N_WORKERS,
               (value_t[]){ mk_obj(g_loadbal_w[0]), mk_obj(g_loadbal_w[1]),
                            mk_obj(g_loadbal_w[2]), mk_obj(g_loadbal_w[3]) });
  g_loadbal_inited = 1;
  wait(print_mu);
  kprintf("[loadbal] init done: dispatcher=%d workers=%d,%d,%d,%d\r\n",
          g_loadbal_disp,
          g_loadbal_w[0], g_loadbal_w[1], g_loadbal_w[2], g_loadbal_w[3]);
  signal(print_mu);
}

/* Fills `buf` with a one-line-per-row snapshot of dispatcher + workers.
 * Caller-allocated; cap must be >= 512.  Returns bytes written. */
int abcl_loadbal_stats(char *buf, int cap) {
  if (cap < 512 || g_loadbal_disp < 0) {
    return sprintf(buf, "loadbal not initialized\n");
  }
  int d = g_loadbal_disp;
  int n = (int)objects[d].fields[F_Disp_n].i;
  long mask = objects[d].fields[F_Disp_enabled].i;
  int blen = sprintf(buf,
    "dispatcher obj=%d workers=%d submitted=%ld completed=%ld\n"
    "enabled_mask=0x%lx (1=on per worker bit; 0xF = all four enabled)\n"
    "loads: w0=%ld w1=%ld w2=%ld w3=%ld\n",
    d, n,
    objects[d].fields[F_Disp_submitted].i,
    objects[d].fields[F_Disp_completed].i,
    mask,
    objects[d].fields[F_Disp_load0].i,
    objects[d].fields[F_Disp_load1].i,
    objects[d].fields[F_Disp_load2].i,
    objects[d].fields[F_Disp_load3].i);
  int i;
  blen += sprintf(buf + blen,
                  "idx obj done busy_ms last_n last_result\n");
  for (i = 0; i < LB_N_WORKERS; i++) {
    int wid = g_loadbal_w[i];
    if (wid < 0) continue;
    blen += sprintf(buf + blen, "%d %d %ld %ld %ld %ld\n",
      i, wid,
      objects[wid].fields[F_Worker_done].i,
      objects[wid].fields[F_Worker_busy_ms].i,
      objects[wid].fields[F_Worker_last_n].i,
      objects[wid].fields[F_Worker_last_result].i);
  }
  /* Latency histograms — bucket counts per worker.  Buckets:
   *   b0:<50  b1:50-99  b2:100-199  b3:200-499  b4:500-999
   *   b5:1000-1999  b6:>=2000  (all ms) */
  blen += sprintf(buf + blen,
    "idx hist_lt50 50-99 100-199 200-499 500-999 1k-2k ge2k\n");
  for (i = 0; i < LB_N_WORKERS; i++) {
    int wid = g_loadbal_w[i];
    if (wid < 0) continue;
    blen += sprintf(buf + blen, "%d %ld %ld %ld %ld %ld %ld %ld\n", i,
      objects[wid].fields[F_Worker_hist_0].i,
      objects[wid].fields[F_Worker_hist_1].i,
      objects[wid].fields[F_Worker_hist_2].i,
      objects[wid].fields[F_Worker_hist_3].i,
      objects[wid].fields[F_Worker_hist_4].i,
      objects[wid].fields[F_Worker_hist_5].i,
      objects[wid].fields[F_Worker_hist_6].i);
  }
  return blen;
}

/* ============================================================
 * Collector — periodic actor garbage-collector, implemented as
 * an AIPL actor (not a kernel thread).  It owns a mailbox; an
 * external heartbeat thread (abcl_gc_heartbeat) sends it "tick"
 * messages at GC_HEARTBEAT_MS granularity, and tick handler
 * decides whether `period_ms` has elapsed since the last sweep.
 * When it does, it calls the existing abcl_gc_sweep() (which
 * was already used by the manual /gc HTTP route).
 *
 * Configuration: tweak via Dispatcher-style messages —
 *   configure(period_ms, threshold_ms)
 *   enable(on)
 *   sweep_now
 * Reachable from Mac via /api/gc-actor/* HTTP routes.
 *
 * Critical actors (WebReceiver, Dispatcher, Workers, Collector)
 * are explicitly protected_from_gc=1 right after creation so a
 * mis-configured threshold can't reap the runtime itself.
 * ============================================================ */

enum { F_GC_period_ms, F_GC_threshold_ms, F_GC_enabled,
       F_GC_last_sweep_ticks, F_GC_sweep_count, F_GC_swept_total,
       F_GC_last_swept_n, F_GC_last_scanned_n,
       F_GC__N };

static void init_fields_Collector(int self_id) {
  int i;
  for (i = 0; i < F_GC__N; i++)
    objects[self_id].fields[i] = mk_int(0L);
  /* Reasonable defaults — Mac can override via /api/gc-actor/configure. */
  objects[self_id].fields[F_GC_period_ms]    = mk_int(5000L);   /* 5 s */
  objects[self_id].fields[F_GC_threshold_ms] = mk_int(30000L);  /* 30 s idle */
  objects[self_id].fields[F_GC_enabled]      = mk_int(1L);
}

static void Collector_do_sweep(int self_id) {
  long threshold = objects[self_id].fields[F_GC_threshold_ms].i;
  if (threshold < 1000L) threshold = 1000L;     /* sanity floor */
  int scanned = 0;
  int killed  = abcl_gc_sweep(threshold, 0, &scanned);
  objects[self_id].fields[F_GC_last_sweep_ticks] = mk_int((long)abcl_now_ms());
  objects[self_id].fields[F_GC_sweep_count] =
    mk_int(objects[self_id].fields[F_GC_sweep_count].i + 1);
  objects[self_id].fields[F_GC_last_swept_n]  = mk_int((long)killed);
  objects[self_id].fields[F_GC_last_scanned_n] = mk_int((long)scanned);
  objects[self_id].fields[F_GC_swept_total] =
    mk_int(objects[self_id].fields[F_GC_swept_total].i + killed);
  wait(print_mu);
  kprintf("[gc-actor] sweep killed=%d/%d threshold=%ldms (sweeps=%ld, total_killed=%ld)\r\n",
          killed, scanned, threshold,
          objects[self_id].fields[F_GC_sweep_count].i,
          objects[self_id].fields[F_GC_swept_total].i);
  signal(print_mu);
}

static void Collector_tick(int self_id, int sender_id,
                           value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  if (!objects[self_id].fields[F_GC_enabled].i) return;
  /* Task-timeout scan runs EVERY tick (1 s), independent of the
   * actor-GC period — Mac wants prompt deadline enforcement even
   * if actor GC is set to a long interval. */
  {
    int expired = abcl_loadbal_timeout_scan();
    if (expired > 0) {
      wait(print_mu);
      kprintf("[gc-actor] timeout: %d pending task(s) auto-cancelled\r\n", expired);
      signal(print_mu);
    }
  }
  /* All three quantities are milliseconds now.  The period used to be
   * divided by 10 to convert ms -> "100 Hz ticks", which combined with the
   * non-monotonic clkticks made the sweep fire either constantly or never
   * depending on where in the second it was called. */
  long now_ms = (long)abcl_now_ms();
  long last   = objects[self_id].fields[F_GC_last_sweep_ticks].i;
  long period_ms = objects[self_id].fields[F_GC_period_ms].i;
  if (period_ms <= 0) period_ms = 5000;          /* 5 s floor */
  if (now_ms - last < period_ms) return;         /* not yet */
  Collector_do_sweep(self_id);
}

static void Collector_configure(int self_id, int sender_id,
                                value_t* args, int n_args) {
  (void)sender_id;
  if (n_args >= 1 && args[0].i > 0)
    objects[self_id].fields[F_GC_period_ms] = mk_int(args[0].i);
  if (n_args >= 2 && args[1].i > 0)
    objects[self_id].fields[F_GC_threshold_ms] = mk_int(args[1].i);
  wait(print_mu);
  kprintf("[gc-actor] configured period=%ldms threshold=%ldms\r\n",
          objects[self_id].fields[F_GC_period_ms].i,
          objects[self_id].fields[F_GC_threshold_ms].i);
  signal(print_mu);
}

static void Collector_enable(int self_id, int sender_id,
                             value_t* args, int n_args) {
  (void)sender_id;
  if (n_args < 1) return;
  objects[self_id].fields[F_GC_enabled] = mk_int(args[0].i ? 1L : 0L);
  wait(print_mu);
  kprintf("[gc-actor] %s\r\n", args[0].i ? "ENABLED" : "PAUSED");
  signal(print_mu);
}

static void Collector_sweep_now(int self_id, int sender_id,
                                value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  Collector_do_sweep(self_id);
}

static void dispatch_Collector(int self_id, int sender_id, const char* method,
                               value_t* args, int n_args) {
  if (strcmp(method, "tick") == 0)
  { Collector_tick(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "configure") == 0)
  { Collector_configure(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "enable") == 0)
  { Collector_enable(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "sweep_now") == 0)
  { Collector_sweep_now(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init") == 0) return;
}

/* GC heartbeat thread — sends "tick" to the Collector at
 * GC_HEARTBEAT_MS intervals.  The actor itself decides whether
 * to actually sweep, based on F_GC_period_ms vs time since last
 * sweep.  Splitting heartbeat-frequency from sweep-frequency
 * means re-configure() takes effect at the next heartbeat
 * (no thread restart needed). */
thread abcl_gc_heartbeat(void) {
  while (!global_shutdown) {
    sleep(GC_HEARTBEAT_MS);
    if (g_gc_actor < 0) continue;
    abcl_enqueue(-1, g_gc_actor, "tick", 0, NULL);
  }
  return OK;
}

/* === accessors for /api/gc-actor/* HTTP routes ========================= */
int abcl_gc_actor_id(void) { return g_gc_actor; }

void abcl_gc_actor_configure(int period_ms, int threshold_ms) {
  if (g_gc_actor < 0) return;
  abcl_enqueue(-1, g_gc_actor, "configure", 2,
               (value_t[]){ mk_int((long)period_ms),
                            mk_int((long)threshold_ms) });
}

void abcl_gc_actor_enable(int on) {
  if (g_gc_actor < 0) return;
  abcl_enqueue(-1, g_gc_actor, "enable", 1,
               (value_t[]){ mk_int((long)on) });
}

void abcl_gc_actor_sweep_now(void) {
  if (g_gc_actor < 0) return;
  abcl_enqueue(-1, g_gc_actor, "sweep_now", 0, NULL);
}

int abcl_gc_actor_stats(char *buf, int cap) {
  if (cap < 300) return 0;
  if (g_gc_actor < 0) return sprintf(buf, "gc-actor not initialized\n");
  int g = g_gc_actor;
  long now_ms  = (long)abcl_now_ms();
  long last_ms = objects[g].fields[F_GC_last_sweep_ticks].i * 10;
  long since   = last_ms > 0 ? now_ms - last_ms : -1L;
  return sprintf(buf,
    "gc-actor obj=%d enabled=%ld\n"
    "period_ms=%ld threshold_ms=%ld\n"
    "sweep_count=%ld swept_total=%ld\n"
    "last_swept=%ld last_scanned=%ld\n"
    "now_ms=%ld last_sweep_ms=%ld since_last_ms=%ld\n",
    g,
    objects[g].fields[F_GC_enabled].i,
    objects[g].fields[F_GC_period_ms].i,
    objects[g].fields[F_GC_threshold_ms].i,
    objects[g].fields[F_GC_sweep_count].i,
    objects[g].fields[F_GC_swept_total].i,
    objects[g].fields[F_GC_last_swept_n].i,
    objects[g].fields[F_GC_last_scanned_n].i,
    now_ms, last_ms, since);
}

/* Bootstrap.  Called from webactor_autostart alongside abcl_loadbal_init.
 * Creates the Collector actor, protects all critical actors from being
 * reaped, and spawns the heartbeat thread. */
static int g_gc_inited = 0;
void abcl_gc_actor_init(void) {
  if (g_gc_inited) return;
  g_gc_actor = alloc_obj(CLASS_Collector, 0, NULL);
  spawn_actor(g_gc_actor);
  /* Protect every infrastructure actor so a mis-set threshold cannot
   * sweep the runtime itself.  We protect the Collector too — recursive
   * suicide would be funny but unhelpful. */
  abcl_object_protect(g_gc_actor, 1);
  {
    int i;
    extern int abcl_loadbal_dispatcher_id(void);
    extern int abcl_loadbal_worker_id(int);
    int d = abcl_loadbal_dispatcher_id();
    if (d >= 0) abcl_object_protect(d, 1);
    for (i = 0; i < LB_N_WORKERS; i++) {
      int w = abcl_loadbal_worker_id(i);
      if (w >= 0) abcl_object_protect(w, 1);
    }
    /* WebReceiver is allocated by abcl_web_init in webactor.  It's
     * always object 0 in the boot path so a hardcoded protect is fine. */
    if (n_objects > 0 && objects[0].class_id == CLASS_WebReceiver)
      abcl_object_protect(0, 1);
  }
  /* Spawn the heartbeat thread */
  tid_typ htid = create((void*)abcl_gc_heartbeat, 4096, INITPRIO,
                        "aipl-gc-hb", 0);
  if (htid != SYSERR) ready(htid, RESCHED_NO);
  g_gc_inited = 1;
  wait(print_mu);
  kprintf("[gc-actor] init done: obj=%d heartbeat tid=%d period=5000ms threshold=30000ms\r\n",
          g_gc_actor, (int)htid);
  signal(print_mu);
}

/* ============================================================
 * DiningBench — 5 哲学者問題 (5 forks + 5 philosophers) のベンチ
 * Orchestrator として 3 つの分散戦略を比較.
 *
 * mode 0 : 全 5 並列 (古典)
 * mode 1 : 3+2 段階    (P0-P2 完了後に P3,P4 起動)
 * mode 2 : 順次 1 人ずつ
 *
 * HTTP:
 *   POST /api/dining/init        — DiningBench actor を spawn (idempotent)
 *   POST /api/dining/start?mode=N&meals=M
 *   GET  /api/dining/status      — n_done / elapsed_ms / max_phil_ms
 * ============================================================ */
enum { F_Dining_mode, F_Dining_meals,
       F_Dining_t_start, F_Dining_t_end,
       F_Dining_n_done, F_Dining_max_phil,
       F_Dining_cursor,
       F_Dining_f0, F_Dining_f1, F_Dining_f2, F_Dining_f3, F_Dining_f4,
       F_Dining_p0, F_Dining_p1, F_Dining_p2, F_Dining_p3, F_Dining_p4,
       F_Dining__N };
/* F_Dining__N = 17 — EXCEEDS MAX_FIELDS=16 !
 * Fix: keep philosophers in a small file-static array indexed by
 * DiningBench obj_id.  Workable because we only ever have 1
 * DiningBench actor live at a time. */
static int  g_dining_obj   = -1;        /* the DiningBench's obj_id */
static int  g_dining_forks[5] = { -1, -1, -1, -1, -1 };
static int  g_dining_phils[5] = { -1, -1, -1, -1, -1 };

/* Re-defined enum without the per-actor f/p slot fields (now in globals). */
#undef F_Dining_f0
#undef F_Dining_f1
#undef F_Dining_f2
#undef F_Dining_f3
#undef F_Dining_f4
#undef F_Dining_p0
#undef F_Dining_p1
#undef F_Dining_p2
#undef F_Dining_p3
#undef F_Dining_p4
#undef F_Dining__N
enum { F_DB_mode, F_DB_meals,
       F_DB_t_start, F_DB_t_end,
       F_DB_n_done, F_DB_max_phil,
       F_DB_cursor,
       F_DB__N };

static void init_fields_DiningBench(int self_id) {
  int i;
  for (i = 0; i < F_DB__N; i++)
    objects[self_id].fields[i] = mk_int(0L);
  g_dining_obj = self_id;
  for (i = 0; i < 5; i++) { g_dining_forks[i] = -1; g_dining_phils[i] = -1; }
}

/* ============================================================
 * Chandy-Misra hygienic philosophers
 *
 *   Each Fork holds: holder (philosopher obj_id), dirty flag,
 *   single pending-request slot (only one neighbor competes per fork).
 *
 *   Rules:
 *   - request from non-holder: dirty -> transfer immediately and clean
 *                              clean -> queue (will transfer when dirtied)
 *                              holder == requester -> reconfirm via fork_arrived
 *   - release(p) by current holder p: mark dirty; if pending, transfer.
 *
 *   Each Philosopher tracks has_lo / has_hi (mirroring holder state) and
 *   req_lo / req_hi (have I already asked).  On both forks present, eat,
 *   then release; fork_lost messages re-sync the mirror if transferred.
 *   Initial fork distribution per CM precedence: lower philosopher id
 *   holds the fork between any pair (P0 starts with F0+F1, P1 with F2,
 *   P2 with F3, P3 with F4, P4 with none).  All forks initially dirty
 *   so the first request transfers.
 * ============================================================ */

/* ---- ForkCM ---- */
enum { F_ForkCM_holder, F_ForkCM_dirty, F_ForkCM_request_id, F_ForkCM__N };

static void init_fields_ForkCM(int self_id) {
  objects[self_id].fields[F_ForkCM_holder]     = mk_int(-1L);
  objects[self_id].fields[F_ForkCM_dirty]      = mk_int(1L);
  objects[self_id].fields[F_ForkCM_request_id] = mk_int(-1L);
}

static void ForkCM_set_holder(int self_id, int sender_id,
                              value_t* args, int n_args) {
  (void)sender_id;
  long h = (n_args > 0) ? args[0].i : -1L;
  objects[self_id].fields[F_ForkCM_holder]     = mk_int(h);
  objects[self_id].fields[F_ForkCM_dirty]      = mk_int(1L);
  objects[self_id].fields[F_ForkCM_request_id] = mk_int(-1L);
}

static void ForkCM_request(int self_id, int sender_id,
                           value_t* args, int n_args) {
  (void)sender_id;
  long requester = (n_args > 0) ? args[0].i : -1L;
  long holder = objects[self_id].fields[F_ForkCM_holder].i;
  long dirty  = objects[self_id].fields[F_ForkCM_dirty].i;
  if (requester < 0) return;
  if (holder == requester) {
    /* Already owns it — re-confirm (covers post-release self-request). */
    abcl_enqueue(self_id, (int)requester, "fork_arrived", 1,
                 (value_t[]){ mk_int((long)self_id) });
    return;
  }
  if (holder < 0) {
    /* In transit — queue (rare). */
    objects[self_id].fields[F_ForkCM_request_id] = mk_int(requester);
    return;
  }
  if (dirty) {
    /* Transfer now; mark clean for new holder. */
    objects[self_id].fields[F_ForkCM_holder]     = mk_int(requester);
    objects[self_id].fields[F_ForkCM_dirty]      = mk_int(0L);
    objects[self_id].fields[F_ForkCM_request_id] = mk_int(-1L);
    abcl_enqueue(self_id, (int)holder, "fork_lost", 1,
                 (value_t[]){ mk_int((long)self_id) });
    abcl_enqueue(self_id, (int)requester, "fork_arrived", 1,
                 (value_t[]){ mk_int((long)self_id) });
  } else {
    /* Clean — queue request; transferred on next release. */
    objects[self_id].fields[F_ForkCM_request_id] = mk_int(requester);
  }
}

static void ForkCM_release(int self_id, int sender_id,
                           value_t* args, int n_args) {
  (void)sender_id;
  long p_id = (n_args > 0) ? args[0].i : -1L;
  long holder = objects[self_id].fields[F_ForkCM_holder].i;
  if (holder != p_id) return;             /* stale / spurious release */
  objects[self_id].fields[F_ForkCM_dirty] = mk_int(1L);
  long pending = objects[self_id].fields[F_ForkCM_request_id].i;
  if (pending >= 0L) {
    objects[self_id].fields[F_ForkCM_holder]     = mk_int(pending);
    objects[self_id].fields[F_ForkCM_dirty]      = mk_int(0L);
    objects[self_id].fields[F_ForkCM_request_id] = mk_int(-1L);
    abcl_enqueue(self_id, (int)holder, "fork_lost", 1,
                 (value_t[]){ mk_int((long)self_id) });
    abcl_enqueue(self_id, (int)pending, "fork_arrived", 1,
                 (value_t[]){ mk_int((long)self_id) });
  }
}

static void dispatch_ForkCM(int self_id, int sender_id, const char* method,
                            value_t* args, int n_args) {
  if (strcmp(method, "init") == 0) return;
  if (strcmp(method, "set_holder") == 0)
  { ForkCM_set_holder(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "request") == 0)
  { ForkCM_request(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "release") == 0)
  { ForkCM_release(self_id, sender_id, args, n_args); return; }
}

/* ---- PhilCM ---- */
enum { F_PhilCM_my_id, F_PhilCM_f_lo, F_PhilCM_f_hi,
       F_PhilCM_has_lo, F_PhilCM_has_hi,
       F_PhilCM_meals, F_PhilCM_meal_idx,
       F_PhilCM_done_to, F_PhilCM_t_start,
       F_PhilCM_req_lo, F_PhilCM_req_hi,
       F_PhilCM__N };

static void init_fields_PhilCM(int self_id) {
  int i;
  for (i = 0; i < F_PhilCM__N; i++)
    objects[self_id].fields[i] = mk_int(0L);
}

static void PhilCM_init_phil(int self_id, int sender_id,
                             value_t* args, int n_args) {
  (void)sender_id; (void)n_args;
  /* args: (id, f_lo, f_hi, has_lo, has_hi, meals, done_to) */
  long id      = args[0].i;
  int  f_lo    = args[1].obj_id;
  int  f_hi    = args[2].obj_id;
  long has_lo  = args[3].i;
  long has_hi  = args[4].i;
  long meals   = args[5].i;
  int  done_to = args[6].obj_id;
  objects[self_id].fields[F_PhilCM_my_id]    = mk_int(id);
  objects[self_id].fields[F_PhilCM_f_lo]     = mk_obj(f_lo);
  objects[self_id].fields[F_PhilCM_f_hi]     = mk_obj(f_hi);
  objects[self_id].fields[F_PhilCM_has_lo]   = mk_int(has_lo);
  objects[self_id].fields[F_PhilCM_has_hi]   = mk_int(has_hi);
  objects[self_id].fields[F_PhilCM_meals]    = mk_int(meals);
  objects[self_id].fields[F_PhilCM_meal_idx] = mk_int(0L);
  objects[self_id].fields[F_PhilCM_done_to]  = mk_obj(done_to);
  objects[self_id].fields[F_PhilCM_t_start]  = mk_int((long)abcl_now_ms());
  objects[self_id].fields[F_PhilCM_req_lo]   = mk_int(0L);
  objects[self_id].fields[F_PhilCM_req_hi]   = mk_int(0L);
  abcl_enqueue(self_id, self_id, "try_eat", 0, NULL);
}

static void PhilCM_try_eat(int self_id, int sender_id,
                           value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  long meals  = objects[self_id].fields[F_PhilCM_meals].i;
  long has_lo = objects[self_id].fields[F_PhilCM_has_lo].i;
  long has_hi = objects[self_id].fields[F_PhilCM_has_hi].i;
  if (meals <= 0L) {
    int orch     = objects[self_id].fields[F_PhilCM_done_to].obj_id;
    long t_start = objects[self_id].fields[F_PhilCM_t_start].i;
    long elapsed = (long)abcl_now_ms() - t_start;
    long my_id   = objects[self_id].fields[F_PhilCM_my_id].i;
    if (elapsed < 0) elapsed = 0;
    if (orch > 0) {
      abcl_enqueue(self_id, orch, "phil_done", 2,
                   (value_t[]){ mk_int(my_id), mk_int(elapsed) });
    }
    abcl_actor_suicide(self_id);
    return;
  }
  if (has_lo && has_hi) {
    /* EAT */
    objects[self_id].fields[F_PhilCM_meals]    = mk_int(meals - 1L);
    objects[self_id].fields[F_PhilCM_meal_idx] = mk_int(
        objects[self_id].fields[F_PhilCM_meal_idx].i + 1L);
    /* Optimistically zero has_lo/has_hi: a pending request may transfer
     * either fork during release.  Real ownership is reconfirmed by
     * fork_arrived (post-release self-request) or fork_lost (transfer). */
    objects[self_id].fields[F_PhilCM_has_lo] = mk_int(0L);
    objects[self_id].fields[F_PhilCM_has_hi] = mk_int(0L);
    objects[self_id].fields[F_PhilCM_req_lo] = mk_int(0L);
    objects[self_id].fields[F_PhilCM_req_hi] = mk_int(0L);
    int f_lo = objects[self_id].fields[F_PhilCM_f_lo].obj_id;
    int f_hi = objects[self_id].fields[F_PhilCM_f_hi].obj_id;
    abcl_enqueue(self_id, f_lo, "release", 1,
                 (value_t[]){ mk_int((long)self_id) });
    abcl_enqueue(self_id, f_hi, "release", 1,
                 (value_t[]){ mk_int((long)self_id) });
    abcl_enqueue(self_id, self_id, "try_eat", 0, NULL);
    return;
  }
  /* Need at least one fork — request missing (idempotent via req_lo/hi). */
  long req_lo = objects[self_id].fields[F_PhilCM_req_lo].i;
  long req_hi = objects[self_id].fields[F_PhilCM_req_hi].i;
  if (!has_lo && !req_lo) {
    int f_lo = objects[self_id].fields[F_PhilCM_f_lo].obj_id;
    abcl_enqueue(self_id, f_lo, "request", 1,
                 (value_t[]){ mk_int((long)self_id) });
    objects[self_id].fields[F_PhilCM_req_lo] = mk_int(1L);
  }
  if (!has_hi && !req_hi) {
    int f_hi = objects[self_id].fields[F_PhilCM_f_hi].obj_id;
    abcl_enqueue(self_id, f_hi, "request", 1,
                 (value_t[]){ mk_int((long)self_id) });
    objects[self_id].fields[F_PhilCM_req_hi] = mk_int(1L);
  }
}

static void PhilCM_fork_arrived(int self_id, int sender_id,
                                value_t* args, int n_args) {
  (void)sender_id;
  int fork = (n_args > 0) ? (int)args[0].i : -1;
  int f_lo = objects[self_id].fields[F_PhilCM_f_lo].obj_id;
  int f_hi = objects[self_id].fields[F_PhilCM_f_hi].obj_id;
  if (fork == f_lo) {
    objects[self_id].fields[F_PhilCM_has_lo] = mk_int(1L);
    objects[self_id].fields[F_PhilCM_req_lo] = mk_int(0L);
  } else if (fork == f_hi) {
    objects[self_id].fields[F_PhilCM_has_hi] = mk_int(1L);
    objects[self_id].fields[F_PhilCM_req_hi] = mk_int(0L);
  }
  abcl_enqueue(self_id, self_id, "try_eat", 0, NULL);
}

static void PhilCM_fork_lost(int self_id, int sender_id,
                             value_t* args, int n_args) {
  (void)sender_id;
  int fork = (n_args > 0) ? (int)args[0].i : -1;
  int f_lo = objects[self_id].fields[F_PhilCM_f_lo].obj_id;
  int f_hi = objects[self_id].fields[F_PhilCM_f_hi].obj_id;
  if (fork == f_lo) objects[self_id].fields[F_PhilCM_has_lo] = mk_int(0L);
  else if (fork == f_hi) objects[self_id].fields[F_PhilCM_has_hi] = mk_int(0L);
}

static void dispatch_PhilCM(int self_id, int sender_id, const char* method,
                            value_t* args, int n_args) {
  if (strcmp(method, "init") == 0) return;
  if (strcmp(method, "init_phil") == 0)
  { PhilCM_init_phil(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "try_eat") == 0)
  { PhilCM_try_eat(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "fork_arrived") == 0)
  { PhilCM_fork_arrived(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "fork_lost") == 0)
  { PhilCM_fork_lost(self_id, sender_id, args, n_args); return; }
}

static void dining_start_phil(int self_id, int i) {
  long meals = objects[self_id].fields[F_DB_meals].i;
  int pid = g_dining_phils[i];
  /* fork pairs: P_i uses F_i and F_(i+1)%5.  Pick (low, high) by id so
   * deadlock-free (canonical asymmetric).  P4 uses F0 (low) and F4 (high). */
  int lo_idx, hi_idx;
  if (i == 4) { lo_idx = 0; hi_idx = 4; }
  else        { lo_idx = i; hi_idx = i + 1; }
  int lo = g_dining_forks[lo_idx];
  int hi = g_dining_forks[hi_idx];
  abcl_enqueue(self_id, pid, "init_phil", 5,
               (value_t[]){ mk_int((long)i),
                            mk_obj(lo), mk_obj(hi),
                            mk_int(meals),
                            mk_obj(self_id) });
}

static void DiningBench_run(int self_id, int sender_id,
                            value_t* args, int n_args) {
  (void)sender_id;
  long mode  = (n_args > 0) ? args[0].i : 0L;
  long meals = (n_args > 1) ? args[1].i : 50L;
  int i;
  /* Allocate 5 Forks + 5 Philosophers fresh per run.
   * mode 0..2: legacy Fork + Philosopher (acquire/release with staggered
   *            backoff — has the 3/5 livelock we're working around).
   * mode 3:    ForkCM + PhilCM (Chandy-Misra hygiene — deadlock-free, fair).
   * Caller must space repeated run_bench calls; old slots get reaped by
   * the GC actor. */
  int fork_class = (mode == 3L) ? CLASS_ForkCM : CLASS_Fork;
  int phil_class = (mode == 3L) ? CLASS_PhilCM : CLASS_Philosopher;
  for (i = 0; i < 5; i++) {
    g_dining_forks[i] = alloc_obj(fork_class, 0, NULL);
    spawn_actor(g_dining_forks[i]);
  }
  for (i = 0; i < 5; i++) {
    g_dining_phils[i] = alloc_obj(phil_class, 0, NULL);
    spawn_actor(g_dining_phils[i]);
  }
  objects[self_id].fields[F_DB_mode]     = mk_int(mode);
  objects[self_id].fields[F_DB_meals]    = mk_int(meals);
  objects[self_id].fields[F_DB_n_done]   = mk_int(0L);
  objects[self_id].fields[F_DB_max_phil] = mk_int(0L);
  objects[self_id].fields[F_DB_t_start]  = mk_int((long)abcl_now_ms());
  objects[self_id].fields[F_DB_t_end]    = mk_int(0L);

  wait(print_mu);
  kprintf("[dining] start mode=%ld meals=%ld phils=%d..%d forks=%d..%d\r\n",
          mode, meals,
          g_dining_phils[0], g_dining_phils[4],
          g_dining_forks[0], g_dining_forks[4]);
  signal(print_mu);

  if (mode == 0L) {
    /* All 5 in parallel */
    objects[self_id].fields[F_DB_cursor] = mk_int(5L);
    for (i = 0; i < 5; i++) dining_start_phil(self_id, i);
  } else if (mode == 1L) {
    /* 3 + 2 staggered: launch P0..P2 now; the rest on phil_done */
    objects[self_id].fields[F_DB_cursor] = mk_int(3L);
    for (i = 0; i < 3; i++) dining_start_phil(self_id, i);
  } else if (mode == 3L) {
    /* CM mode — initial fork distribution per Chandy-Misra precedence.
     * Pairs: F0(P0,P4) F1(P0,P1) F2(P1,P2) F3(P2,P3) F4(P3,P4)
     * Lower-id philosopher holds each fork initially (and dirty so the
     * first request transfers):
     *   F0 -> P0, F1 -> P0, F2 -> P1, F3 -> P2, F4 -> P3
     *   P0 holds {F0,F1};  P1 holds {F2};  P2 holds {F3};  P3 holds {F4};  P4 nothing.
     */
    int holders[5];
    holders[0] = g_dining_phils[0];
    holders[1] = g_dining_phils[0];
    holders[2] = g_dining_phils[1];
    holders[3] = g_dining_phils[2];
    holders[4] = g_dining_phils[3];
    for (i = 0; i < 5; i++) {
      abcl_enqueue(self_id, g_dining_forks[i], "set_holder", 1,
                   (value_t[]){ mk_int((long)holders[i]) });
    }
    for (i = 0; i < 5; i++) {
      int lo_idx, hi_idx;
      if (i == 4) { lo_idx = 0; hi_idx = 4; }
      else        { lo_idx = i; hi_idx = i + 1; }
      int pid = g_dining_phils[i];
      int has_lo = (holders[lo_idx] == pid) ? 1 : 0;
      int has_hi = (holders[hi_idx] == pid) ? 1 : 0;
      abcl_enqueue(self_id, pid, "init_phil", 7,
                   (value_t[]){ mk_int((long)i),
                                mk_obj(g_dining_forks[lo_idx]),
                                mk_obj(g_dining_forks[hi_idx]),
                                mk_int((long)has_lo),
                                mk_int((long)has_hi),
                                mk_int(meals),
                                mk_obj(self_id) });
    }
    objects[self_id].fields[F_DB_cursor] = mk_int(5L);
  } else {
    /* sequential: launch P0; subsequent on phil_done */
    objects[self_id].fields[F_DB_cursor] = mk_int(1L);
    dining_start_phil(self_id, 0);
  }
}

static void DiningBench_phil_done(int self_id, int sender_id,
                                  value_t* args, int n_args) {
  (void)sender_id;
  long phil_id    = (n_args > 0) ? args[0].i : -1L;
  long elapsed_ms = (n_args > 1) ? args[1].i : 0L;
  long n_done = objects[self_id].fields[F_DB_n_done].i + 1;
  objects[self_id].fields[F_DB_n_done] = mk_int(n_done);
  if (elapsed_ms > objects[self_id].fields[F_DB_max_phil].i) {
    objects[self_id].fields[F_DB_max_phil] = mk_int(elapsed_ms);
  }
  long mode = objects[self_id].fields[F_DB_mode].i;
  long cur  = objects[self_id].fields[F_DB_cursor].i;
  if (mode == 1L && n_done == 3L) {
    /* 3+2 staggered: launch P3 + P4 */
    objects[self_id].fields[F_DB_cursor] = mk_int(5L);
    dining_start_phil(self_id, 3);
    dining_start_phil(self_id, 4);
  } else if (mode == 2L && cur < 5L) {
    /* sequential: launch next */
    objects[self_id].fields[F_DB_cursor] = mk_int(cur + 1L);
    dining_start_phil(self_id, (int)cur);
  }
  if (n_done == 5L) {
    long t_end = (long)abcl_now_ms();
    objects[self_id].fields[F_DB_t_end] = mk_int(t_end);
    wait(print_mu);
    kprintf("[dining] mode=%ld all done in %ld ms (max_phil=%ld)\r\n",
            mode, t_end - objects[self_id].fields[F_DB_t_start].i,
            objects[self_id].fields[F_DB_max_phil].i);
    signal(print_mu);
  } else {
    wait(print_mu);
    kprintf("[dining] phil=%ld done elapsed=%ld (n_done=%ld/5)\r\n",
            phil_id, elapsed_ms, n_done);
    signal(print_mu);
  }
}

static void dispatch_DiningBench(int self_id, int sender_id, const char* method,
                                 value_t* args, int n_args) {
  if (strcmp(method, "run_bench") == 0)
  { DiningBench_run(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "phil_done") == 0)
  { DiningBench_phil_done(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "init") == 0) return;
}

/* HTTP-facing helpers */
int abcl_dining_actor_id(void) { return g_dining_obj; }

void abcl_dining_init(void) {
  if (g_dining_obj >= 0) return;
  int id = alloc_obj(CLASS_DiningBench, 0, NULL);
  abcl_object_protect(id, 1);
  spawn_actor(id);
  /* g_dining_obj is set inside init_fields_DiningBench */
}

void abcl_dining_start(int mode, int meals) {
  if (g_dining_obj < 0) return;
  abcl_enqueue(-1, g_dining_obj, "run_bench", 2,
               (value_t[]){ mk_int((long)mode), mk_int((long)meals) });
}

int abcl_dining_status(char *buf, int cap) {
  if (cap < 256) return 0;
  if (g_dining_obj < 0) return sprintf(buf, "dining not initialized\n");
  int d = g_dining_obj;
  long t_start  = objects[d].fields[F_DB_t_start].i;
  long t_end    = objects[d].fields[F_DB_t_end].i;
  long elapsed  = (t_end > 0) ? (t_end - t_start)
                               : ((long)abcl_now_ms() - t_start);
  /* Race guard: a concurrent run_bench could update t_start AFTER we
   * read clkticks; clamp to 0 instead of confusing the Mac poller
   * with negative numbers. */
  if (elapsed < 0) elapsed = 0;
  return sprintf(buf,
    "obj=%d mode=%ld meals=%ld n_done=%ld/5\n"
    "elapsed_ms=%ld (final=%s) max_phil_ms=%ld\n",
    d,
    objects[d].fields[F_DB_mode].i,
    objects[d].fields[F_DB_meals].i,
    objects[d].fields[F_DB_n_done].i,
    elapsed,
    (objects[d].fields[F_DB_n_done].i == 5L) ? "yes" : "no",
    objects[d].fields[F_DB_max_phil].i);
}

/* ============================================================ *
 *  Rotate4Lines — Spinner + Controller actor classes.          *
 *  Translated from abclc/Rotate4LinesXinu.abcl by the host      *
 *  abcl2c (--xinu) and hand-integrated here (the file is no     *
 *  longer machine-regenerated — it carries RCU/GC changes).     *
 *  GUI hooks (b_cos/b_sin/xinu_gui_*) live in abcl_xinu_gui.c.  *
 * ============================================================ */
extern value_t b_cos(int n_args, value_t *args);             /* deg -> cos*1024 */
extern value_t b_sin(int n_args, value_t *args);             /* deg -> sin*1024 */
extern value_t xinu_gui_set_line(int n_args, value_t *args); /* (idx,x1,y1,x2,y2[,r,g,b]) */
extern value_t xinu_gui_register_ticker(int n_args, value_t *args); /* (actor) */
extern value_t xinu_gui_add_button(int n_args, value_t *args);     /* (label,x,y,w,h,obj,method) */

enum { F_Spinner_idx, F_Spinner_angle, F_Spinner_speed, F_Spinner_cx, F_Spinner_cy,
       F_Spinner_radius, F_Spinner_running, F_Spinner_rr, F_Spinner_gg, F_Spinner_bb,
       F_Spinner_turns };

static void init_fields_Spinner(int self_id) {
  objects[self_id].fields[F_Spinner_idx]     = mk_int(0L);
  objects[self_id].fields[F_Spinner_angle]   = mk_float(0.0);
  objects[self_id].fields[F_Spinner_speed]   = mk_int(5L);
  objects[self_id].fields[F_Spinner_cx]      = mk_int(0L);
  objects[self_id].fields[F_Spinner_cy]      = mk_int(0L);
  objects[self_id].fields[F_Spinner_radius]  = mk_int(60L);
  objects[self_id].fields[F_Spinner_running] = mk_int(1L);
  objects[self_id].fields[F_Spinner_rr]      = mk_int(200L);
  objects[self_id].fields[F_Spinner_gg]      = mk_int(220L);
  objects[self_id].fields[F_Spinner_bb]      = mk_int(255L);
  objects[self_id].fields[F_Spinner_turns]   = mk_int(0L);
}
static long fi(int self_id, int f) {     /* field-as-long helper (int or float) */
  value_t v = objects[self_id].fields[f];
  return (v.tag == V_INT) ? v.i : (long)v.f;
}
static double ff(int self_id, int f) {   /* field-as-double helper */
  value_t v = objects[self_id].fields[f];
  return (v.tag == V_FLOAT) ? v.f : (double)v.i;
}
static void Spinner_init(int self_id, int sender_id, value_t* args, int n_args) {
  (void)sender_id;
  objects[self_id].fields[F_Spinner_idx]     = mk_int((n_args > 0) ? (long)((args[0].tag==V_INT)?args[0].i:(long)args[0].f) : 0L);
  objects[self_id].fields[F_Spinner_speed]   = mk_int((n_args > 1) ? (long)((args[1].tag==V_INT)?args[1].i:(long)args[1].f) : 0L);
  objects[self_id].fields[F_Spinner_cx]      = mk_int((n_args > 2) ? (long)((args[2].tag==V_INT)?args[2].i:(long)args[2].f) : 0L);
  objects[self_id].fields[F_Spinner_cy]      = mk_int((n_args > 3) ? (long)((args[3].tag==V_INT)?args[3].i:(long)args[3].f) : 0L);
  objects[self_id].fields[F_Spinner_rr]      = mk_int((n_args > 4) ? (long)((args[4].tag==V_INT)?args[4].i:(long)args[4].f) : 0L);
  objects[self_id].fields[F_Spinner_gg]      = mk_int((n_args > 5) ? (long)((args[5].tag==V_INT)?args[5].i:(long)args[5].f) : 0L);
  objects[self_id].fields[F_Spinner_bb]      = mk_int((n_args > 6) ? (long)((args[6].tag==V_INT)?args[6].i:(long)args[6].f) : 0L);
  objects[self_id].fields[F_Spinner_running] = mk_int(1L);
  objects[self_id].fields[F_Spinner_turns]   = mk_int(0L);
  enqueue(self_id, self_id, "tick", 0, NULL);
}
static void Spinner_start(int self_id, int s, value_t* a, int n) { (void)s;(void)a;(void)n;
  objects[self_id].fields[F_Spinner_turns]   = mk_int(0L);    /* restart the 10-turn count */
  objects[self_id].fields[F_Spinner_running] = mk_int(1L); }
static void Spinner_stop(int self_id, int s, value_t* a, int n)  { (void)s;(void)a;(void)n;
  objects[self_id].fields[F_Spinner_running] = mk_int(0L); }
#define SPINNER_MAX_TURNS 10
static void Spinner_tick(int self_id, int sender_id, value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  if (fi(self_id, F_Spinner_running) == 1L) {
    double a = ff(self_id, F_Spinner_angle) + (double)fi(self_id, F_Spinner_speed);
    if (a >= 360.0) {
      a -= 360.0;
      long turns = fi(self_id, F_Spinner_turns) + 1;          /* one full rotation */
      objects[self_id].fields[F_Spinner_turns] = mk_int(turns);
      if (turns >= SPINNER_MAX_TURNS)                         /* stop after 10 turns */
        objects[self_id].fields[F_Spinner_running] = mk_int(0L);
    }
    objects[self_id].fields[F_Spinner_angle] = mk_float(a);
  }
  double ang = ff(self_id, F_Spinner_angle);
  long c = b_cos(1, (value_t[]){ mk_float(ang) }).i;   /* cos*1024 */
  long s = b_sin(1, (value_t[]){ mk_float(ang) }).i;   /* sin*1024 */
  long radius = fi(self_id, F_Spinner_radius);
  long dx = c * radius / 1024;
  long dy = s * radius / 1024;
  long cx = fi(self_id, F_Spinner_cx), cy = fi(self_id, F_Spinner_cy);
  xinu_gui_set_line(8, (value_t[]){
      mk_int(fi(self_id, F_Spinner_idx)),
      mk_int(cx + dx), mk_int(cy + dy), mk_int(cx - dx), mk_int(cy - dy),
      mk_int(fi(self_id, F_Spinner_rr)), mk_int(fi(self_id, F_Spinner_gg)), mk_int(fi(self_id, F_Spinner_bb)) });
}
static void dispatch_Spinner(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (strcmp(method, "init") == 0)  {Spinner_init(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "start") == 0) {Spinner_start(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "stop") == 0)  {Spinner_stop(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "tick") == 0)  { Spinner_tick(self_id, sender_id, args, n_args); return; }
}

enum { F_Controller_s1, F_Controller_s2, F_Controller_s3, F_Controller_s4 };

static void init_fields_Controller(int self_id) {
  objects[self_id].fields[F_Controller_s1] = mk_int(0L);
  objects[self_id].fields[F_Controller_s2] = mk_int(0L);
  objects[self_id].fields[F_Controller_s3] = mk_int(0L);
  objects[self_id].fields[F_Controller_s4] = mk_int(0L);
}
static void Controller_init(int self_id, int sender_id, value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  objects[self_id].fields[F_Controller_s1] = mk_obj(create_obj(CLASS_Spinner, 7, (value_t[]){mk_int(0L),mk_int(3L),mk_int(200L),mk_int(160L),mk_int(255L),mk_int(100L),mk_int(100L)}));
  objects[self_id].fields[F_Controller_s2] = mk_obj(create_obj(CLASS_Spinner, 7, (value_t[]){mk_int(1L),mk_int(5L),mk_int(440L),mk_int(160L),mk_int(100L),mk_int(220L),mk_int(130L)}));
  objects[self_id].fields[F_Controller_s3] = mk_obj(create_obj(CLASS_Spinner, 7, (value_t[]){mk_int(2L),mk_int(7L),mk_int(200L),mk_int(320L),mk_int(100L),mk_int(150L),mk_int(255L)}));
  objects[self_id].fields[F_Controller_s4] = mk_obj(create_obj(CLASS_Spinner, 7, (value_t[]){mk_int(3L),mk_int(9L),mk_int(440L),mk_int(320L),mk_int(240L),mk_int(200L),mk_int(100L)}));
  xinu_gui_register_ticker(1, (value_t[]){objects[self_id].fields[F_Controller_s1]});
  xinu_gui_register_ticker(1, (value_t[]){objects[self_id].fields[F_Controller_s2]});
  xinu_gui_register_ticker(1, (value_t[]){objects[self_id].fields[F_Controller_s3]});
  xinu_gui_register_ticker(1, (value_t[]){objects[self_id].fields[F_Controller_s4]});
  xinu_gui_add_button(7, (value_t[]){mk_str("Start"), mk_int(40L), mk_int(26L), mk_int(90L), mk_int(30L), mk_obj(self_id), mk_str("start")});
  xinu_gui_add_button(7, (value_t[]){mk_str("Stop"),  mk_int(510L),mk_int(26L), mk_int(90L), mk_int(30L), mk_obj(self_id), mk_str("stop")});
}
static void Controller_bcast(int self_id, const char *m) {
  enqueue(self_id, objects[self_id].fields[F_Controller_s1].obj_id, m, 0, NULL);
  enqueue(self_id, objects[self_id].fields[F_Controller_s2].obj_id, m, 0, NULL);
  enqueue(self_id, objects[self_id].fields[F_Controller_s3].obj_id, m, 0, NULL);
  enqueue(self_id, objects[self_id].fields[F_Controller_s4].obj_id, m, 0, NULL);
}
static void dispatch_Controller(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  (void)sender_id; (void)args; (void)n_args;
  if (strcmp(method, "init") == 0)  {Controller_init(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "start") == 0) {Controller_bcast(self_id, "start"); return; }
  if (strcmp(method, "stop") == 0)  {Controller_bcast(self_id, "stop"); return; }
}

/* Bootstrap: spawn the Controller (its init spawns 4 spinners + Start/Stop
 * buttons + registers tickers).  Called on demand from the AIPL window's
 * `run "Rotate4Lines.abcl"` (apps/gwm.c).  Idempotent-ish guard via g_rotate4. */
static int g_rotate4 = -1;
int abcl_rotate4_init(void) {
  if (g_rotate4 >= 0) return g_rotate4;    /* already running */
  g_rotate4 = create_obj(CLASS_Controller, 0, NULL);
  return g_rotate4;
}
void abcl_rotate4_start(void) { if (g_rotate4 >= 0) enqueue(-1, g_rotate4, "start", 0, NULL); }
void abcl_rotate4_stop(void)  { if (g_rotate4 >= 0) enqueue(-1, g_rotate4, "stop",  0, NULL); }

/* Fully end the Rotate4Lines demo: reap the Controller + 4 Spinner actors,
 * drop the GUI lines/tickers/buttons, and allow a fresh `run` afterwards. */
static void rotate4_reap(int oid) {
  if (oid < 0 || objects[oid].dead) return;
  tid_typ t = objects[oid].tid;
  objects[oid].dead = 1;
  if (!isbadtid(t)) { signal(objects[oid].mbox.items); kill(t); }
}
void abcl_rotate4_end(void) {
  extern void abcl_xinu_gui_clear(void);
  int cid = g_rotate4, f;
  if (cid < 0) return;
  for (f = F_Controller_s1; f <= F_Controller_s4; f++) {
    value_t sv = objects[cid].fields[f];
    if (sv.tag == V_OBJ) rotate4_reap(sv.obj_id);
  }
  rotate4_reap(cid);
  abcl_xinu_gui_clear();
  g_rotate4 = -1;
}

/* ============================================================ *
 *  PingPong — two actors bounce a message back and forth,      *
 *  printing each hop.  Translated from abclc/PingPong.abcl by   *
 *  abcl2c.  print() routes to v_print -> the AIPL console window.*
 *  Bounded by PP_MAX so the (otherwise infinite) loop stops.    *
 * ============================================================ */
#define PP_MAX 40
static int g_pinger = -1, g_ponger = -1, g_pp_count = 0;
static void init_fields_Pinger(int self_id) { (void)self_id; }
static void init_fields_Ponger(int self_id) { (void)self_id; }
static void Pinger_init(int self_id, int s, value_t* a, int n) { (void)s;(void)a;(void)n;
  v_print(mk_str("Pinger starting"));
  if (g_pp_count++ < PP_MAX) enqueue(self_id, g_ponger, "ping", 0, NULL);
}
static void Pinger_pong(int self_id, int sender_id, value_t* a, int n) { (void)a;(void)n;
  v_print(mk_str("Pinger got pong"));
  if (g_pp_count++ < PP_MAX) enqueue(self_id, sender_id, "ping", 0, NULL);
}
static void dispatch_Pinger(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (strcmp(method, "init") == 0) { Pinger_init(self_id, sender_id, args, n_args); return; }
  if (strcmp(method, "pong") == 0) { Pinger_pong(self_id, sender_id, args, n_args); return; }
}
static void Ponger_ping(int self_id, int sender_id, value_t* a, int n) { (void)a;(void)n;
  v_print(mk_str("Ponger got ping"));
  if (g_pp_count++ < PP_MAX) enqueue(self_id, sender_id, "pong", 0, NULL);
}
static void dispatch_Ponger(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (strcmp(method, "ping") == 0) { Ponger_ping(self_id, sender_id, args, n_args); return; }
}
/* Bootstrap: create Ponger first (Pinger.init sends to g_ponger), then Pinger.
 * Re-runnable: resets the message counter each time. */
void abcl_pingpong_init(void) {
  g_pp_count = 0;
  g_ponger = create_obj(CLASS_Ponger, 0, NULL);
  g_pinger = create_obj(CLASS_Pinger, 0, NULL);
}

/* ====================================================================
 * Dynamic actor bytecode VM.  An "actor binary" (.avm module) is sent to the
 * running kernel over HTTP (/actor/loadvm), parsed into a dynamic class table,
 * created with create_obj() and run on the EXISTING actor runtime (mailboxes,
 * scheduler, GC, monitor) — no kernel recompile.  VM class ids start at
 * VM_CLASS_BASE so dispatch()'s default case routes them to abcl_vm_dispatch().
 *
 * .avm module layout (little-endian):
 *   "AVM1" | u16 nstr | nstr*( u16 len, len bytes ) |
 *   u16 nclass | nclass*( u16 nameIdx, u16 nfields, u16 nmethods |
 *      nmethods*( u16 nameIdx, u8 nparams, u16 codeLen, codeLen bytes ) )
 * Opcodes: 01 PUSHI(i32) 02 LDF(u8) 03 STF(u8) 04 LDA(u8) 05 SELF
 *   10..14 ADD SUB MUL DIV MOD   20..25 LT LE GT GE EQ NE
 *   30 JMP(u16) 31 JZ(u16)   40 SEND(u16 mIdx,u8 nargs) 41 SPAWN(u16 cIdx)
 *   42 PRINT   43 RET
 * ==================================================================== */
#define VM_CLASS_BASE  200
#define VM_MAX_CLASSES 16
#define VM_MAX_METHODS 16
#define VM_STR_MAX     64
/* Normal actor modules stay small; large "image/media" modules (3-D wireframes,
 * and — coming — drone photo/video frames sent as actors) use the big buffer.
 * The webactor separates the two: /actor/loadvm caps at VM_ACTOR_MAX, while
 * /actor/loadimg allows up to VM_MODULE_MAX. */
#define VM_MODULE_MAX  (1024*1024)   /* 1 MB — image/media modules */
#define VM_ACTOR_MAX   65536         /* 64 KB — ordinary actors */
#define VM_STRBUF_MAX  2048
#define VM_VSTACK      64
typedef struct { int name, n_params, code_off, code_len; } vmmethod_t;
typedef struct { int name, n_fields, n_methods; vmmethod_t m[VM_MAX_METHODS]; } vmclass_t;
static unsigned char vm_mod[VM_MODULE_MAX]; static int vm_mod_len = 0;
static char          vm_strbuf[VM_STRBUF_MAX];
static const char   *vm_str[VM_STR_MAX]; static int vm_n_str = 0;

/* 文字列の値は「0x40000000 | 文字列表の添字」というタグ付き整数で流れてくる。
 * 見るのは印字のときだけで、フィールド・引数・送信・演算は整数のまま素通りする。
 * ホスト VM (aice-avm の avm.ml vm_show) と同じ約束。 */
#define VM_STR_TAG  0x40000000   /* 文字列の値であることを示す */
#define VM_STR_HEAP 0x00800000   /* 立っていれば実行時ヒープ、落ちていればモジュールの文字列表 */
#define VM_STR_MASK 0x007fffff

/* 真偽値。0x20000000 のビットが立っていれば真偽値で、値は最下位ビット。
 * 文字列と同じ「印字のときだけタグを見る」やり方だが、真偽値は
 * ★条件判定でもタグを見なければならない★ ―― false = 0x20000000 は 0 ではないので、
 * 素の JZ では偽と判定されない。必ず vm_falsy を通すこと。
 * ホスト VM (aice-avm の avm.ml) と同じ約束。 */
#define VM_BOOL_TAG 0x20000000
#define vm_bool(b)     ((long)(VM_BOOL_TAG | ((b) ? 1 : 0)))

/* result<tau> の失敗を表す予約値。`else` を書かない期限がこれを返す。
   compile.ml の vm_err と同じ値でなければならない。 */
#define VM_ERR_TAG  0x10000000

/* --- 値のタグ体系 ---------------------------------------------------------
   負の整数は「そのまま整数」。非負のときだけ上位 3 ビット（0x70000000）を
   タグとして読む。もとは「ビットが立っているか」で見ていたが、種類が増えると
   組み合わせが衝突するので、パターン一致に改めた。

     000  整数（0 .. 0x0FFFFFFF）
     001  err（result の失敗）
     010  真偽値（値は最下位ビット）
     011  浮動小数（浮動小数表の添字）
     100  文字列（文字列表／実行時ヒープの添字）
     101  配列（配列表の添字）

   ★ タグ判定はどれも「非負であること」を先に確かめる。負の整数は上位ビットが
     すべて立っているので、この番人が無いと -1 が文字列だと誤認される。 */
#define VM_TAG_MASK  0x70000000
#define VM_FLT_TAG   0x30000000
#define VM_LST_TAG   0x50000000
#define VM_PAYLOAD   0x0FFFFFFF

#define vm_tagged(v)   ((v) >= 0)
#define vm_tag(v)      ((long)((v) & VM_TAG_MASK))
#define vm_is_bool(v)  (vm_tagged(v) && vm_tag(v) == VM_BOOL_TAG)
#define vm_is_err(v)   (vm_tagged(v) && vm_tag(v) == VM_ERR_TAG)
#define vm_is_str(v)   (vm_tagged(v) && vm_tag(v) == VM_STR_TAG)
#define vm_is_flt(v)   (vm_tagged(v) && vm_tag(v) == VM_FLT_TAG)
#define vm_is_lst(v)   (vm_tagged(v) && vm_tag(v) == VM_LST_TAG)
#define vm_is_num(v)   (vm_is_flt(v) || (!vm_tagged(v)) || vm_tag(v) == 0)
#define vm_falsy(v)    (vm_is_bool(v) ? (((v) & 1) == 0) : ((v) == 0))

/* --- 浮動小数の置き場 ------------------------------------------------------
   値そのものは 32 ビットに収まらないので、表に置いて添字を配る。
   回収は無い（この VM に GC は無い）ので、上限に達したら最後の枠を使い回す
   ―― 文字列ヒープと同じ約束である。 */
#define VM_FLT_MAX 128
static double vm_fheap[VM_FLT_MAX];
static int    vm_fheap_n = 0;

static long vm_mkflt(double d)
{
    int i;
    if (vm_fheap_n >= VM_FLT_MAX) i = VM_FLT_MAX - 1;
    else i = vm_fheap_n++;
    vm_fheap[i] = d;
    return (long)(VM_FLT_TAG | i);
}
static double vm_fltval(long v)
{
    int i = (int)(v & VM_PAYLOAD);
    if (vm_is_flt(v) && i >= 0 && i < VM_FLT_MAX) return vm_fheap[i];
    return (double)v;                 /* 整数はそのまま実数として読む */
}

/* --- 数学組込み ------------------------------------------------------------
   この板に libm は無い。Pi 4・Pi 5 で libm と突き合わせて誤差 1e-11 以下を
   確かめた実装を、そのまま移してある（fsqrt だけは AArch64 命令なので置換）。
   三台で同じ式を使うので、同じ入力に対して同じ出力になる。 */
#define VM_PI  3.14159265358979323846
#define VM_PI2 1.57079632679489661923

/* ★ Pi 4・Pi 5 は AArch64 の fsqrt 命令を使っているが、この板は ARM32 で
   浮動小数はソフトウェア実装なので、その命令が無い。ニュートン法で置く
   （初期値を指数で合わせてあるので 5 回で倍精度の桁が埋まる）。 */
static double m_sqrt(double x)
{
    double r; int i;
    if (x <= 0.0) return 0.0;
    r = x; if (r > 1.0) { r = x / 2.0; } else { r = (x + 1.0) / 2.0; }
    for (i = 0; i < 40; i++) { double n = 0.5 * (r + x / r); if (n == r) break; r = n; }
    return r;
}
static double m_abs(double x)   { return x < 0 ? -x : x; }
static double m_floor(double x) { double t = (double)(long)x; return (x < 0 && t != x) ? t - 1 : t; }
static double m_ceil(double x)  { double t = (double)(long)x; return (x > 0 && t != x) ? t + 1 : t; }
static double m_round(double x) { return x < 0 ? -m_floor(-x + 0.5) : m_floor(x + 0.5); }

/* exp: x = k*ln2 + r,  |r| <= ln2/2。 e^r を 12 次のテイラーで、2^k は指数を直に置く。 */
static double m_exp(double x)
{
    if (x >  709.0) x =  709.0;
    if (x < -745.0) return 0.0;
    const double LN2 = 0.69314718055994530942;
    double kf = m_floor(x / LN2 + 0.5);
    double r  = x - kf * LN2;
    double s = 1.0, t = 1.0;
    for (int i = 1; i <= 12; i++) { t = t * r / (double)i; s += t; }
    /* ★ この板は ARM32 で long が 32 ビットである。Pi 4・Pi 5 から移した原文は
       `unsigned long` の共用体で指数を組み立てていたが、それでは << 52 が
       意味を失い、exp が 0 を返していた（実機で判明）。幅を明示する。 */
    long k = (long)kf;
    union { double d; unsigned long long u; } p;
    p.u = ((unsigned long long)(1023 + k)) << 52;
    return s * p.d;
}
/* log: x = m * 2^e （m in [1,2)）。ln m は atanh 級数で。 */
static double m_log(double x)
{
    if (x <= 0) return 0.0;
    /* ★ 同上。>> 52 と定数 0x800FFFFFFFFFFFFF は 64 ビットでなければならない。
       32 ビットのまま動かすと log が -704 付近の値を返す。 */
    union { double d; unsigned long long u; } v; v.d = x;
    long e = (long)((v.u >> 52) & 0x7FF) - 1023;
    v.u = (v.u & 0x800FFFFFFFFFFFFFULL) | (1023ULL << 52);   /* m in [1,2) */
    double m = v.d;
    if (m > 1.4142135623730951) { m *= 0.5; e += 1; }
    double z = (m - 1.0) / (m + 1.0), z2 = z * z, s = 0.0, t = z;
    for (int i = 1; i <= 25; i += 2) { s += t / (double)i; t *= z2; }
    return 2.0 * s + (double)e * 0.69314718055994530942;
}
static double m_log10(double x) { return m_log(x) * 0.43429448190325182765; }

/* sin/cos: 2π で畳み、さらに象限で [-π/4, π/4] まで縮めてから級数。
   縮めないと |x| が π 近くで桁落ちする（実測で 2.4e-4 の誤差が出た）。 */
static double sc_poly_sin(double x)
{
    double s = x, t = x, x2 = x * x;
    for (int i = 1; i <= 7; i++) { t = -t * x2 / (double)((2*i) * (2*i + 1)); s += t; }
    return s;
}
static double sc_poly_cos(double x)
{
    double s = 1.0, t = 1.0, x2 = x * x;
    for (int i = 1; i <= 7; i++) { t = -t * x2 / (double)((2*i - 1) * (2*i)); s += t; }
    return s;
}
static double m_sin(double x)
{
    double n = m_floor(x / VM_PI2 + 0.5);          /* π/2 単位に畳む */
    double r = x - n * VM_PI2;
    long q = ((long)n) & 3; if (q < 0) q += 4;
    switch (q) {
      case 0: return  sc_poly_sin(r);
      case 1: return  sc_poly_cos(r);
      case 2: return -sc_poly_sin(r);
      default:return -sc_poly_cos(r);
    }
}
static double m_cos(double x)
{
    double n = m_floor(x / VM_PI2 + 0.5);
    double r = x - n * VM_PI2;
    long q = ((long)n) & 3; if (q < 0) q += 4;
    switch (q) {
      case 0: return  sc_poly_cos(r);
      case 1: return -sc_poly_sin(r);
      case 2: return -sc_poly_cos(r);
      default:return  sc_poly_sin(r);
    }
}
static double m_tan(double x) { double c = m_cos(x); return c == 0.0 ? 0.0 : m_sin(x) / c; }

/* atan: |x|<=1 に落としたあと、さらに tan(π/12) 以下まで縮めてから級数。
   縮めないと x が 1 に近いところで級数が収束せず 1e-2 も外れる（実測）。 */
static double m_atan(double x)
{
    int neg = x < 0; if (neg) x = -x;
    int inv = x > 1.0; if (inv) x = 1.0 / x;
    double add = 0.0;
    if (x > 0.26794919243112270647) {              /* tan(π/12) */
        const double S3 = 1.73205080756887729353;  /* √3 */
        x = (x * S3 - 1.0) / (x + S3);
        add = VM_PI / 6.0;
    }
    double s = 0.0, t = x, x2 = x * x;
    for (int i = 1; i <= 31; i += 2) { s += ((i % 4 == 1) ? t : -t) / (double)i; t *= x2; }
    s += add;
    if (inv) s = VM_PI2 - s;
    return neg ? -s : s;
}
static double m_asin(double x)
{
    if (x >=  1.0) return  VM_PI2;
    if (x <= -1.0) return -VM_PI2;
    return m_atan(x / m_sqrt(1.0 - x * x));
}
static double m_acos(double x) { return VM_PI2 - m_asin(x); }

/* 値ランタイムへの入口。引数も戻り値も value_t（整数でも浮動小数でも受ける）。 */

/* --- 配列の置き場 ----------------------------------------------------------
   要素は値（タグ付き 32 ビット）。長さは可変だが、確保したら伸ばさない
   ―― array_push は「新しい配列を作って返す」正典の意味なので、それで足りる。 */
#define VM_LST_MAX   32
#define VM_LST_POOL  512
static long vm_lpool[VM_LST_POOL];
static int  vm_lpool_n = 0;
static struct { int off, len; } vm_lists[VM_LST_MAX];
static int  vm_lists_n = 0;

static long vm_mklst(const long *src, int n)
{
    int i, li;
    if (n < 0) n = 0;
    if (vm_lists_n >= VM_LST_MAX || vm_lpool_n + n > VM_LST_POOL) {
        /* 溢れたら空の配列を返す。黙って壊れた添字を配るより良い。 */
        if (vm_lists_n == 0) return (long)VM_LST_TAG;
        li = vm_lists_n - 1; vm_lists[li].len = 0; return (long)(VM_LST_TAG | li);
    }
    li = vm_lists_n++;
    vm_lists[li].off = vm_lpool_n;
    vm_lists[li].len = n;
    for (i = 0; i < n; i++) vm_lpool[vm_lpool_n + i] = src ? src[i] : 0;
    vm_lpool_n += n;
    return (long)(VM_LST_TAG | li);
}
static int vm_lstlen(long v)
{
    int i = (int)(v & VM_PAYLOAD);
    if (vm_is_lst(v) && i >= 0 && i < vm_lists_n) return vm_lists[i].len;
    return 0;
}
static long *vm_lstptr(long v)
{
    int i = (int)(v & VM_PAYLOAD);
    if (vm_is_lst(v) && i >= 0 && i < vm_lists_n) return &vm_lpool[vm_lists[i].off];
    return 0;
}

/* 実行時に作られた文字列の置き場。連結（CONCAT）だけが使う。
 * 回収は無い（この VM に GC は無い）ので、上限に達したら最後の枠を使い回す。
 * ホスト VM (aice-avm の avm.ml) と同じ約束。 */
#define VM_HEAP_STR_MAX 64
#define VM_HEAP_BUF_MAX 2048
static char vm_heapbuf[VM_HEAP_BUF_MAX];
static int  vm_heap_off[VM_HEAP_STR_MAX];
static int  vm_heap_n = 0, vm_heap_used = 0;

static void vm_fmt_val(char *out, int cap, long v)
{
    if (vm_is_bool(v)) {
        const char *s = ((v & 1) != 0) ? "true" : "false";
        int n = 0;
        while (s[n] && n < cap - 1) { out[n] = s[n]; n++; }
        out[n] = 0; return;
    }
    if (vm_is_err(v)) {
        const char *s = "err";
        int n = 0;
        while (s[n] && n < cap - 1) { out[n] = s[n]; n++; }
        out[n] = 0; return;
    }
    if (vm_is_flt(v)) {
        /* 小数点以下 6 桁。Pi 4・Pi 5 の v_render と同じ見た目にする
           （三台で同じ出力になっていないと、正典との照合が板ごとに割れる）。 */
        double d = vm_fltval(v);
        int neg = (d < 0.0), n = 0;
        long ip; double fr;
        if (neg) d = -d;
        ip = (long)d; fr = d - (double)ip;
        if (neg && n < cap - 1) out[n++] = '-';
        { char t[16]; int k = 0;
          if (ip == 0) t[k++] = '0';
          while (ip > 0 && k < 15) { t[k++] = (char)('0' + (ip % 10)); ip /= 10; }
          while (k > 0 && n < cap - 1) out[n++] = t[--k]; }
        if (n < cap - 1) out[n++] = '.';
        { int k; for (k = 0; k < 6 && n < cap - 1; k++) {
            fr *= 10.0; { int dig = (int)fr; if (dig > 9) dig = 9; if (dig < 0) dig = 0;
                          out[n++] = (char)('0' + dig); fr -= (double)dig; } } }
        out[n] = 0; return;
    }
    if (vm_is_lst(v)) {
        long *b = vm_lstptr(v); int ln = vm_lstlen(v), i, n = 0;
        if (n < cap - 1) out[n++] = '[';
        for (i = 0; i < ln && n < cap - 2; i++) {
            char t[24];
            if (i) { if (n < cap - 1) out[n++] = ','; if (n < cap - 1) out[n++] = ' '; }
            vm_fmt_val(t, sizeof t, b ? b[i] : 0);
            { int k = 0; while (t[k] && n < cap - 2) out[n++] = t[k++]; }
        }
        if (n < cap - 1) out[n++] = ']';
        out[n] = 0; return;
    }
    if (vm_is_str(v)) {
        int i = (int)(v & VM_STR_MASK);
        const char *s = 0;
        if ((v & VM_STR_HEAP) != 0) {
            if (i >= 0 && i < vm_heap_n) s = &vm_heapbuf[vm_heap_off[i]];
        } else if (i >= 0 && i < vm_n_str) s = vm_str[i];
        if (s) { int n = 0;
            while (s[n] && n < cap - 1) { out[n] = s[n]; n++; }
            out[n] = 0; return; }
    }
    sprintf(out, "%ld", v);
}


/* ===== web_listen / web_expose =====
 * HTTP のパスをアクタに結びつける。呼び出しの送り主を VM_WEB_SINK(-2) に
 * しておき、そこへ来た reply を横取りして HTTP の応答にする。
 * ホスト VM (aice-avm の avm.ml) と同じ約束。 */
#define VM_WEB_MAX  8
static char vm_web_path[VM_WEB_MAX][40];
static int  vm_web_actor[VM_WEB_MAX];
static int  vm_web_n = 0, vm_web_port = 0;

/* ===== プログラムの出力を HTTP から読む =====
 * Pi 3 の print はシリアルと画面へ出るだけで、外からは読めなかった。
 * Pi 5 は POST /cc の応答に出力がそのまま返るので、ここが揃っていなかった
 * ―― 同じプログラムを両機で走らせて突き合わせることができない。
 * 直近の出力を輪バッファに控え、GET /api/console で返す。
 * 形はホスト VM (aice-avm server.ml) と同じ {"total":N,"lines":[...]}。 */
#define VM_CON_MAX   24
#define VM_CON_LEN   96
static char vm_con[VM_CON_MAX][VM_CON_LEN];
static int  vm_con_head  = 0;      /* 次に書く位置                     */
static int  vm_con_n     = 0;      /* 保持している本数（<= VM_CON_MAX）*/
static int  vm_con_total = 0;      /* 起動からの通算（切り捨て分も含む）*/

void vm_console_push(int self, const char *s)
{
    char *d = vm_con[vm_con_head];
    int p = 0, i, v = self, q = 0;
    char t[12];
    d[p++] = 'a';
    if (v < 0) { d[p++] = '-'; v = -v; }
    if (0 == v) t[q++] = '0';
    while (v > 0 && q < 11) { t[q++] = (char)('0' + (v % 10)); v /= 10; }
    while (q > 0 && p < VM_CON_LEN - 3) d[p++] = t[--q];
    d[p++] = ':'; d[p++] = ' ';
    for (i = 0; s[i] && p < VM_CON_LEN - 1; i++) d[p++] = s[i];
    d[p] = 0;
    vm_con_head = (vm_con_head + 1) % VM_CON_MAX;
    if (vm_con_n < VM_CON_MAX) vm_con_n++;
    vm_con_total++;
}

int vm_console_json(char *out, int cap, int clear)
{
    int p = 0, i, k, first;
    first = vm_con_head - vm_con_n;
    if (first < 0) first += VM_CON_MAX;
    p += sprintf(out + p, "{\"total\":%d,\"lines\":[", vm_con_total);
    for (i = 0; i < vm_con_n; i++)
    {
        const char *s = vm_con[(first + i) % VM_CON_MAX];
        if (p > cap - 140) break;          /* 入り切らない分は落とす */
        if (i > 0) out[p++] = ',';
        out[p++] = '"';
        for (k = 0; s[k] && p < cap - 8; k++)
        {
            unsigned char ch = (unsigned char)s[k];
            if ('"' == ch || '\\' == ch) { out[p++] = '\\'; out[p++] = (char)ch; }
            else if (ch < 0x20)          { out[p++] = ' '; }
            else                         { out[p++] = (char)ch; }
        }
        out[p++] = '"';
    }
    p += sprintf(out + p, "]}\r\n");
    if (clear) { vm_con_head = 0; vm_con_n = 0; }
    out[p] = 0;
    return p;
}

/* HTTP から来た文字列を VM の値にする（ヒープに積む） */
static long vm_intern(const char *s)
{
    int n = 0, off, i;
    while (s[n]) n++;
    if (vm_heap_n >= VM_HEAP_STR_MAX || vm_heap_used + n + 1 > VM_HEAP_BUF_MAX)
        return VM_STR_TAG | VM_STR_HEAP | (vm_heap_n > 0 ? vm_heap_n - 1 : 0);
    off = vm_heap_used;
    for (i = 0; i < n; i++) vm_heapbuf[off + i] = s[i];
    vm_heapbuf[off + n] = 0;
    vm_heap_off[vm_heap_n] = off;
    vm_heap_used += n + 1;
    return VM_STR_TAG | VM_STR_HEAP | (vm_heap_n++);
}

/* 公開したパスへ HTTP から 1 回呼ぶ。返信が来るまで少し待つ。
 * webactor.c から呼ぶ。戻り値: 1=経路あり / 0=経路なし */
int vm_web_call(const char *path, const char *meth, const char *arg, char *out, int cap)
{
    extern syscall sleep(unsigned);
    int i, k = -1, n;
    for (i = 0; i < vm_web_n; i++) {
        const char *a = vm_web_path[i], *b = path; int same = 1;
        while (*a || *b) { if (*a != *b) { same = 0; break; } a++; b++; }
        if (same) { k = i; break; }
    }
    if (k < 0) return 0;
    vm_web_have = 0; vm_web_reply[0] = 0;
    { value_t va[1]; int na = 0;
      if (arg && arg[0]) { va[0].tag = V_INT; va[0].i = vm_intern(arg);
                           va[0].f = 0; va[0].s = 0; va[0].obj_id = 0; na = 1; }
      abcl_enqueue(VM_WEB_SINK, vm_web_actor[k], meth, na, va); }
    /* 2 秒だと ai_call（実機のLLM推論）が終わる前に打ち切ってしまい、
       空の応答が返る。推論を含む呼び出しを通すため 90 秒まで待つ。
       返信が来た時点で抜けるので、普通のメソッドは従来どおり即座に返る。 */
    for (n = 0; n < 1800 && !vm_web_have; n++) sleep(50);  /* 最大 90 秒 */
    { int p = 0; const char *s = vm_web_reply;
      while (s[p] && p < cap - 1) { out[p] = s[p]; p++; }
      out[p] = 0; }
    return 1;
}

/* remote(...) から呼ばれる版。vm_web_call との違いは引数の扱いだけで、
 * 数字だけの文字列は整数値として渡す（正典の `now remote(...).step(3)` は
 * 整数を取る。文字列のまま渡すと相手側の x * 2 が意味をなさない）。 */
int vm_remote_call(const char *path, const char *meth, const char *arg,
                   char *out, int cap)
{
    extern syscall sleep(unsigned);
    int i, k = -1, n;
    for (i = 0; i < vm_web_n; i++) {
        const char *a = vm_web_path[i], *b = path; int same = 1;
        while (*a || *b) { if (*a != *b) { same = 0; break; } a++; b++; }
        if (same) { k = i; break; }
    }
    if (k < 0) return 0;
    vm_web_have = 0; vm_web_reply[0] = 0;
    { value_t va[1]; int na = 0;
      if (arg && arg[0]) {
          int neg = (arg[0] == '-'), j = neg ? 1 : 0, digits = 0;
          long v = 0;
          for (; arg[j]; j++) {
              if (arg[j] < '0' || arg[j] > '9') { digits = -1; break; }
              v = v * 10 + (arg[j] - '0'); digits++;
          }
          va[0].tag = V_INT; va[0].f = 0; va[0].s = 0; va[0].obj_id = 0;
          va[0].i = (digits > 0) ? (neg ? -v : v) : vm_intern(arg);
          na = 1;
      }
      abcl_enqueue(VM_WEB_SINK, vm_web_actor[k], meth, na, va); }
    for (n = 0; n < 600 && !vm_web_have; n++) sleep(50);   /* 最大 30 秒 */
    { int p = 0; const char *s = vm_web_reply;
      while (s[p] && p < cap - 1) { out[p] = s[p]; p++; }
      out[p] = 0; }
    return 1;
}

/* 公開済みの経路の一覧 */
int vm_web_list(char *out, int cap)
{
    int i, p = 0;
    for (i = 0; i < vm_web_n && p < cap - 60; i++)
        p += sprintf(out + p, "%s -> a%d\r\n", vm_web_path[i], vm_web_actor[i]);
    if (p == 0 && cap > 20) p = sprintf(out, "(no routes)\r\n");
    out[p] = 0; return p;
}

/* a と b を文字列として並べ、ヒープに積んでタグ付き値を返す */
static long vm_concat(long a, long b)
{
    char ta[96], tb[96];
    int la, lb, need, off, i;
    vm_fmt_val(ta, sizeof ta, a);
    vm_fmt_val(tb, sizeof tb, b);
    for (la = 0; ta[la]; la++) ;
    for (lb = 0; tb[lb]; lb++) ;
    need = la + lb + 1;
    if (vm_heap_n >= VM_HEAP_STR_MAX || vm_heap_used + need > VM_HEAP_BUF_MAX) {
        /* 溢れたら最後の枠を使い回す。黙って壊れないよう印を残す */
        kprintf("[vm] string heap full\r\n");
        return VM_STR_TAG | VM_STR_HEAP | (vm_heap_n > 0 ? vm_heap_n - 1 : 0);
    }
    off = vm_heap_used;
    for (i = 0; i < la; i++) vm_heapbuf[off + i] = ta[i];
    for (i = 0; i < lb; i++) vm_heapbuf[off + la + i] = tb[i];
    vm_heapbuf[off + la + lb] = 0;
    vm_heap_off[vm_heap_n] = off;
    vm_heap_used += need;
    return VM_STR_TAG | VM_STR_HEAP | (vm_heap_n++);
}
static vmclass_t     vm_class[VM_MAX_CLASSES]; static int vm_n_class = 0;
static int  vm_u16(const unsigned char *p) { return p[0] | (p[1] << 8); }
/* --- 資源の名前つき錠 ------------------------------------------------------
   名前は文字列リテラルなので数は少ない。線形探索で足りる。
   同じアクタが二度取っても止まらないよう、持ち主を覚えて再入を許す
   （正典の検査器が二重取得を静的に弾くので、ここは保険）。 */
#define VM_RES_MAX 16
static struct { char name[24]; semaphore sem; tid_typ owner; int depth; } vm_res[VM_RES_MAX];
static int vm_res_n = 0;

static int vm_res_find(const char *nm)
{
    int i;
    for (i = 0; i < vm_res_n; i++)
        if (strncmp(vm_res[i].name, nm, sizeof vm_res[0].name - 1) == 0) return i;
    if (vm_res_n >= VM_RES_MAX) return -1;
    i = vm_res_n++;
    { int k = 0; while (nm[k] && k < (int)sizeof vm_res[0].name - 1) { vm_res[i].name[k] = nm[k]; k++; }
      vm_res[i].name[k] = 0; }
    vm_res[i].sem = semcreate(1);
    vm_res[i].owner = BADTID;
    vm_res[i].depth = 0;
    return i;
}

static void vm_res_acquire(long nv)
{
    char nm[24];
    int i;
    vm_fmt_val(nm, sizeof nm, nv);
    i = vm_res_find(nm);
    if (i < 0) return;                      /* 表が溢れた: 錠なしで通す */
    if (vm_res[i].owner == gettid()) { vm_res[i].depth++; return; }
    if (vm_res[i].sem != SYSERR) wait(vm_res[i].sem);
    vm_res[i].owner = gettid();
    vm_res[i].depth = 1;
}

static void vm_res_release(long nv)
{
    char nm[24];
    int i;
    vm_fmt_val(nm, sizeof nm, nv);
    i = vm_res_find(nm);
    if (i < 0) return;
    if (vm_res[i].owner != gettid()) return;   /* 取っていない資源の release */
    if (--vm_res[i].depth > 0) return;
    vm_res[i].owner = BADTID;
    if (vm_res[i].sem != SYSERR) signal(vm_res[i].sem);
}

/* 相手が描いた文字列を値に読み直す。型は運んでいない ―― 正典の型検査器が
   両側の型を合わせている前提で成り立つ約束である。 */
extern int  aipl_remote_send(const char *, const char *, const char *, const char *);
extern int  aipl_remote_call(const char *, const char *, const char *, const char *,
                             int, char *, int);
static long vm_remote_value(const char *t)
{
    int i = 0, neg = 0, digits = 0; long v = 0;
    if (!t || !t[0]) return vm_intern("");
    if (t[0]=='t'&&t[1]=='r'&&t[2]=='u'&&t[3]=='e'&&t[4]==0)  return vm_bool(1);
    if (t[0]=='f'&&t[1]=='a'&&t[2]=='l'&&t[3]=='s'&&t[4]=='e'&&t[5]==0) return vm_bool(0);
    if (t[0]=='e'&&t[1]=='r'&&t[2]=='r'&&t[3]==0) return (long)VM_ERR_TAG;
    if (t[0] == '-') { neg = 1; i = 1; }
    for (; t[i]; i++) {
        if (t[i] < '0' || t[i] > '9') { digits = -1; break; }
        v = v * 10 + (t[i] - '0'); digits++;
    }
    if (digits > 0) return neg ? -v : v;
    return vm_intern(t);
}

static long vm_i32(const unsigned char *p) {
  return (long)(int)((unsigned)p[0] | ((unsigned)p[1]<<8) | ((unsigned)p[2]<<16) | ((unsigned)p[3]<<24)); }

int abcl_vm_load(const unsigned char *buf, int len) {
  if (len < 6 || len > VM_MODULE_MAX) return -1;
  if (buf[0]!='A'||buf[1]!='V'||buf[2]!='M'||buf[3]!='1') return -1;
  memcpy(vm_mod, buf, len); vm_mod_len = len;
  const unsigned char *p = vm_mod + 4, *end = vm_mod + len;
  int ns = vm_u16(p); p += 2, vm_n_str = 0; int sb = 0, i;
  for (i = 0; i < ns && i < VM_STR_MAX; i++) {
    if (p + 2 > end) return -1;
    int l = vm_u16(p); p += 2;
    if (p + l > end || sb + l + 1 > VM_STRBUF_MAX) return -1;
    vm_str[i] = &vm_strbuf[sb]; memcpy(&vm_strbuf[sb], p, l); vm_strbuf[sb+l] = 0; sb += l+1; p += l;
  }
  vm_n_str = i;
  if (p + 2 > end) return -1;
  int nc = vm_u16(p); p += 2; vm_n_class = 0;
  int c, mi;
  for (c = 0; c < nc && c < VM_MAX_CLASSES; c++) {
    if (p + 6 > end) return -1;
    vmclass_t *cl = &vm_class[c];
    cl->name = vm_u16(p); p += 2; cl->n_fields = vm_u16(p); p += 2; cl->n_methods = vm_u16(p); p += 2;
    for (mi = 0; mi < cl->n_methods && mi < VM_MAX_METHODS; mi++) {
      if (p + 5 > end) return -1;
      cl->m[mi].name = vm_u16(p); p += 2; cl->m[mi].n_params = *p++;
      cl->m[mi].code_len = vm_u16(p); p += 2; cl->m[mi].code_off = (int)(p - vm_mod);
      if (p + cl->m[mi].code_len > end) return -1;
      p += cl->m[mi].code_len;
    }
    vm_n_class++;
  }
  return vm_n_class > 0 ? VM_CLASS_BASE : -1;
}
int abcl_vm_class_count(void) { return vm_n_class; }
const char *abcl_vm_class_name(int ci) {
  if (ci < 0 || ci >= vm_n_class) return "?";
  int s = vm_class[ci].name; return (s >= 0 && s < vm_n_str) ? vm_str[s] : "?";
}
int abcl_vm_spawn(int ci) {
  if (ci < 0 || ci >= vm_n_class) return -1;
  int id = create_obj_noinit(VM_CLASS_BASE + ci), i;
  if (id >= 0) for (i = 0; i < MAX_FIELDS; i++) { objects[id].fields[i].tag = V_INT; objects[id].fields[i].i = 0; }
  return id;
}
void abcl_vm_dispatch(int self, int sender, const char *method, value_t *args, int n_args) {
  int ci = objects[self].class_id - VM_CLASS_BASE, k;
  if (ci < 0 || ci >= vm_n_class) return;
  vmclass_t *cl = &vm_class[ci]; vmmethod_t *mt = NULL;
  for (k = 0; k < cl->n_methods; k++) {
    const char *mn = vm_str[cl->m[k].name];
    if (mn && 0 == strcmp(mn, method)) { mt = &cl->m[k]; break; }
  }
  if (!mt) return;
  const unsigned char *code = vm_mod + mt->code_off; int clen = mt->code_len;
  long stk[VM_VSTACK]; int sp = 0, pc = 0; long guard = 0;
#define VPUSH(v) do { if (sp < VM_VSTACK) stk[sp++] = (long)(v); } while (0)
#define VPOP()   (sp > 0 ? stk[--sp] : 0)
  while (pc < clen) {
    if (++guard > 2000000L) break;
    unsigned char op = code[pc++];
    switch (op) {
    case 0x01: VPUSH(vm_i32(code + pc)); pc += 4; break;
    case 0x5E: {                                                            /* PUSHF */
        /* 浮動小数リテラル。次の 8 バイトが IEEE754 のビット列（下位から）。
           32 ビットの値には収まらないので、表に置いて添字を積む。 */
        union { double d; unsigned char b[8]; } u; int i;
        for (i = 0; i < 8; i++) u.b[i] = code[pc + i];
        pc += 8;
        VPUSH(vm_mkflt(u.d)); } break;
    case 0x02: { int f = code[pc++]; VPUSH(f < MAX_FIELDS ? objects[self].fields[f].i : 0); } break;
    case 0x03: { int f = code[pc++]; long v = VPOP();
                 if (f < MAX_FIELDS) { objects[self].fields[f].tag = V_INT; objects[self].fields[f].i = v; } } break;
    case 0x04: { int a = code[pc++]; VPUSH(a < n_args ? args[a].i : 0); } break;
    case 0x05: VPUSH(self); break;
    /* 算術と比較。どちらかが実数なら実数で計算する（正典に暗黙変換は無いが、
       前段が int と float を混ぜないので、ここは「実数が来たら実数」で足りる）。 */
    case 0x10: { long b = VPOP(), a = VPOP();
        if (vm_is_flt(a) || vm_is_flt(b)) VPUSH(vm_mkflt(vm_fltval(a) + vm_fltval(b)));
        else VPUSH(a + b); } break;
    case 0x11: { long b = VPOP(), a = VPOP();
        if (vm_is_flt(a) || vm_is_flt(b)) VPUSH(vm_mkflt(vm_fltval(a) - vm_fltval(b)));
        else VPUSH(a - b); } break;
    case 0x12: { long b = VPOP(), a = VPOP();
        if (vm_is_flt(a) || vm_is_flt(b)) VPUSH(vm_mkflt(vm_fltval(a) * vm_fltval(b)));
        else VPUSH(a * b); } break;
    case 0x13: { long b = VPOP(), a = VPOP();
        if (vm_is_flt(a) || vm_is_flt(b)) { double d = vm_fltval(b);
            VPUSH(vm_mkflt(d != 0.0 ? vm_fltval(a) / d : 0.0)); }
        else VPUSH(b ? a / b : 0); } break;
    case 0x14: { long b = VPOP(), a = VPOP(); VPUSH(b ? a % b : 0); } break;
#define VM_CMP(op) { long b = VPOP(), a = VPOP(); \
        if (vm_is_flt(a) || vm_is_flt(b)) VPUSH(vm_bool(vm_fltval(a) op vm_fltval(b))); \
        else VPUSH(vm_bool(a op b)); } break;
    case 0x20: VM_CMP(<)
    case 0x21: VM_CMP(<=)
    case 0x22: VM_CMP(>)
    case 0x23: VM_CMP(>=)
    case 0x24: VM_CMP(==)
    case 0x25: VM_CMP(!=)
#undef VM_CMP
    case 0x30: pc = vm_u16(code + pc); break;
    case 0x31: { int t = vm_u16(code + pc); pc += 2; long c = VPOP(); if (vm_falsy(c)) pc = t; } break;
    case 0x40: { int mn = vm_u16(code + pc); pc += 2; int na = code[pc++], i;
                 value_t va[8]; if (na > 8) na = 8;
                 for (i = na - 1; i >= 0; i--) { va[i].tag = V_INT; va[i].i = VPOP(); }
                 int recv = (int)VPOP();
                 if (mn >= 0 && mn < vm_n_str) abcl_enqueue(self, recv, vm_str[mn], na, va); } break;
    case 0x41: { int ci2 = vm_u16(code + pc); pc += 2; VPUSH(abcl_vm_spawn(ci2)); } break;
    case 0x06: VPUSH(sender); break;                          /* SENDER */
    case 0x07: { extern syscall sleep(unsigned); long ms = VPOP(); if (ms > 0) sleep((unsigned)ms); } break; /* WAIT */
    case 0x08: { long v = sp > 0 ? stk[sp-1] : 0; VPUSH(v); } break; /* DUP */
    case 0x15: { long b = VPOP(), a = VPOP(); VPUSH(vm_concat(a, b)); } break;  /* CONCAT */
    case 0x52: {                                                               /* AI_CALL */
                 /* カーネルに焼き込んだ小型 LLM (stories260K) で推論する。
                  * D-cache OFF・ソフト浮動小数点なので遅い。呼んだアクタは
                  * その間止まる（他のアクタは動く）。 */
                 extern int llm_run(const char *, int, char *, int, int);
                 static char aip[128], aio[256];
                 long pv = VPOP();
                 vm_fmt_val(aip, sizeof aip, pv);
                 kprintf("[vm] ai_call: %s\r\n", aip);
                 aio[0] = 0;
                 llm_run(aip, 24, aio, (int)sizeof aio - 1, 0);
                 VPUSH(vm_intern(aio));
               } break;
    /* ACQUIRE name / RELEASE name — 資源の名前つき錠。
       対と取得順序の検査は正典の型検査器が担う。ここが持つのは実行時の
       排他だけ。Pi3 ではアクタが本物の Xinu プロセスなので、名前ごとに
       セマフォを張らないと acquire の意味が無い。 */
    /* REMOTE_SEND host actor meth arg / REMOTE_CALL host actor meth arg ms
       ―― 他ノードのアクターへ。運びかたは apps/aipl_remote.c（UDP/9010）。
       押す順は前段（compile.ml）と揃えること。 */
    case 0x55: {                                                            /* REMOTE_SEND */
        char h[48], a[40], m[40], g[96];
        long ag = VPOP(), me = VPOP(), ac = VPOP(), ho = VPOP();
        vm_fmt_val(h, sizeof h, ho); vm_fmt_val(a, sizeof a, ac);
        vm_fmt_val(m, sizeof m, me); vm_fmt_val(g, sizeof g, ag);
        aipl_remote_send(h, a, m, g);
    } break;
    case 0x56: {                                                            /* REMOTE_CALL */
        /* 既定値も命令が受け取る。else を書かなければ前段が err を積むので、
           そのまま result<tau> になる ―― 期限切れの扱いが一箇所に閉じる。
           押した順は host, actor, meth, arg, ms, else なので逆に取る。 */
        char h[48], a[40], m[40], g[96], r[192];
        long dflt = VPOP(), ms = VPOP(), ag = VPOP(), me = VPOP(), ac = VPOP(), ho = VPOP();
        vm_fmt_val(h, sizeof h, ho); vm_fmt_val(a, sizeof a, ac);
        vm_fmt_val(m, sizeof m, me); vm_fmt_val(g, sizeof g, ag);
        if (aipl_remote_call(h, a, m, g, (int)ms, r, sizeof r) != 0) VPUSH(dflt);
        else {
            /* 相手が "err" を返した場合も失敗として扱う（宛先が無い・メソッドが
               無い）。期限切れと同じ扱いにしないと else が効かない穴が残る。 */
            long v = vm_remote_value(r);
            VPUSH(v == (long)VM_ERR_TAG ? dflt : v);
        }
    } break;
    /* --- 配列（正典 array_*）------------------------------------------------
       正典では array_push / array_set は「新しい配列を返す」ので、ここでも
       作り直す。回収は無いが、板に投げるプログラムの寿命は短いので足りる。 */
    case 0x57: { VPUSH(vm_mklst(0, 0)); } break;                            /* ARR_NEW */
    case 0x58: { long e = VPOP(), a = VPOP();                               /* ARR_PUSH */
        long tmp[64]; int n = vm_lstlen(a), i; long *b = vm_lstptr(a);
        if (n > 63) n = 63;
        for (i = 0; i < n; i++) tmp[i] = b[i];
        tmp[n] = e; VPUSH(vm_mklst(tmp, n + 1)); } break;
    case 0x59: { long ix = VPOP(), a = VPOP();                              /* ARR_GET */
        int n = vm_lstlen(a); long *b = vm_lstptr(a);
        long i = vm_is_flt(ix) ? (long)vm_fltval(ix) : ix;
        VPUSH((b && i >= 0 && i < n) ? b[i] : 0); } break;
    case 0x5A: { long e = VPOP(), ix = VPOP(), a = VPOP();                  /* ARR_SET */
        long tmp[64]; int n = vm_lstlen(a), k; long *b = vm_lstptr(a);
        long i = vm_is_flt(ix) ? (long)vm_fltval(ix) : ix;
        if (n > 64) n = 64;
        for (k = 0; k < n; k++) tmp[k] = b[k];
        if (i >= 0 && i < n) tmp[i] = e;
        VPUSH(vm_mklst(tmp, n)); } break;
    case 0x5B: { long a = VPOP(); VPUSH((long)vm_lstlen(a)); } break;       /* ARR_LEN */
    case 0x5C: { long nv = VPOP();                                          /* ARR_ZEROS */
        long tmp[64]; int n = (int)(vm_is_flt(nv) ? (long)vm_fltval(nv) : nv), i;
        if (n < 0) n = 0; if (n > 64) n = 64;
        for (i = 0; i < n; i++) tmp[i] = 0;
        VPUSH(vm_mklst(tmp, n)); } break;
    /* --- 数学組込み。番号は compile.ml と揃えること --------------------- */
    case 0x5D: { int k = code[pc++]; long a = VPOP(); double x = vm_fltval(a); double r;
        switch (k) {
        case 0:  r = m_sqrt(x);  break;   case 1:  r = m_exp(x);   break;
        case 2:  r = m_log(x);   break;   case 3:  r = m_log10(x); break;
        case 4:  r = m_sin(x);   break;   case 5:  r = m_cos(x);   break;
        case 6:  r = m_tan(x);   break;   case 7:  r = m_asin(x);  break;
        case 8:  r = m_acos(x);  break;   case 9:  r = m_atan(x);  break;
        case 10: r = m_floor(x); break;   case 11: r = m_ceil(x);  break;
        case 12: r = m_round(x); break;   case 13: r = m_abs(x);   break;
        case 14: r = -x;         break;
        default: r = 0.0;        break; }
        VPUSH(vm_mkflt(r)); } break;
    case 0x53: { long nv = VPOP(); vm_res_acquire(nv); } break;              /* ACQUIRE */
    case 0x54: { long nv = VPOP(); vm_res_release(nv); } break;              /* RELEASE */
    case 0x50: { long p = VPOP(); vm_web_port = (int)p; } break;               /* WEBLISTEN */
    case 0x51: { long aid = VPOP(); long pv = VPOP();                          /* WEBEXPOSE */
                 if (vm_web_n < VM_WEB_MAX) {
                     vm_fmt_val(vm_web_path[vm_web_n], 40, pv);
                     vm_web_actor[vm_web_n] = (int)aid; vm_web_n++;
                     kprintf("[vm] expose %s -> a%d\r\n",
                             vm_web_path[vm_web_n-1], vm_web_actor[vm_web_n-1]); } } break;
    case 0x42: { extern void vm_print(int, const char *); char ln[80]; long v = VPOP();
                 vm_fmt_val(ln, sizeof ln, v);
                 kprintf("[vm] a%d: %s\r\n", self, ln); vm_print(self, ln);
                 vm_console_push(self, ln); } break;
    case 0x44: { extern void vm_print(int, const char *);                       /* PRINTF fmt,nargs */
                 int fi = vm_u16(code + pc); pc += 2; int na = code[pc++], i, ai = 0;
                 long va[8]; if (na > 8) na = 8; for (i = na - 1; i >= 0; i--) va[i] = VPOP();
                 const char *f = (fi >= 0 && fi < vm_n_str) ? vm_str[fi] : "";
                 char ln[80]; int p = 0;
                 while (*f && p < 76) { if (f[0] == '%' && f[1] == 'd') {
                                  if (ai < na) { char tmp[64]; int q = 0;
                                                 vm_fmt_val(tmp, sizeof tmp, va[ai++]);
                                                 while (tmp[q] && p < 76) ln[p++] = tmp[q++]; }
                                  f += 2; }
                              else { ln[p++] = *f++; } }
                 ln[p] = 0; kprintf("[vm] a%d: %s\r\n", self, ln); vm_print(self, ln);
                 vm_console_push(self, ln); } break;
    case 0x45: { extern void vm_line(int, int, int, int, int);                  /* LINE x1,y1,x2,y2,col */
                 long col = VPOP(), y2 = VPOP(), x2 = VPOP(), y1 = VPOP(), x1 = VPOP();
                 vm_line((int)x1, (int)y1, (int)x2, (int)y2, (int)col); } break;
    case 0x46: { extern void vm_cls(void); vm_cls(); } break;                   /* CLS */
    case 0x47: { extern void vm_fill_triangle(int,int,int,int,int,int,int);     /* TRI x1,y1,x2,y2,x3,y3,col */
                 long col = VPOP(), y3 = VPOP(), x3 = VPOP(), y2 = VPOP(), x2 = VPOP(), y1 = VPOP(), x1 = VPOP();
                 vm_fill_triangle((int)x1,(int)y1,(int)x2,(int)y2,(int)x3,(int)y3,(int)col); } break;
    case 0x43: pc = clen; break;
    default:   pc = clen; break;
    }
  }
#undef VPUSH
#undef VPOP
}
/* HTTP entry: load a .avm, spawn its first class, kick it with "tick". */
/* 新しいモジュールを載せる前に、前のモジュールのアクタを畳む。
 *
 * Pi 5 は /cc を投げるたびに前のアクター世界が置き換わる。Pi 3 は
 * 古いアクタが Xinu スレッドのまま残るので、
 *   ・プロセス表を食いつぶす（正典ガイド 10 本を順に流すと 9 本目の g9 で
 *     新しいアクタが走らなくなり、そのあと HTTP ごと詰まった。実測）
 *   ・mailbox の msgs[].method が、置き換えられた vm_mod を指したまま残る
 * という二つの形で壊れる。同じ約束に揃える。
 * 畳むのは VM のアクタ（class_id >= VM_CLASS_BASE）だけで、webactor など
 * 常駐の C アクタには触らない。 */
static void abcl_vm_kill_prev(void) {
  int i, total = rcu_n_objects(), n = 0;
  /* ★ kill() は使わない。アクタ・スレッドは spawn 命令の中で alloc_obj を
   * 呼ぶ ―― そこで objects_mu を握っている最中に kill すると、以後の生成が
   * 全部そこで止まり、webactor まで巻き込んで HTTP が詰まる（実測）。
   * 印をつけて起こすだけにして、abcl_actor_main に自分でループを抜けさせる
   * （abcl_rt_reset と同じ作法）。 */
  for (i = 0; i < total; i++) {
    if (objects[i].class_id < VM_CLASS_BASE) continue;
    if (objects[i].dead || !objects[i].started) continue;
    objects[i].dead = 1; n++;
  }
  for (i = 0; i < total; i++) {
    if (objects[i].class_id < VM_CLASS_BASE) continue;
    signal(objects[i].mbox.items);   /* 待っているものを起こす */
  }
  if (n > 0) {
    sleep(100);                      /* ループを抜けるのを待つ */
    kprintf("[vm] 前のモジュールのアクタ %d 体を畳んだ\r\n", n);
  }
  vm_web_n = 0;                      /* 公開ルートもプログラムと寿命を共にする */
}

int abcl_vm_loadrun(const unsigned char *buf, int len) {
  extern void vm_gfx_unsuppress(void);
  abcl_vm_kill_prev();
  int base = abcl_vm_load(buf, len);
  if (base < 0) return -1;
  vm_gfx_unsuppress();                 /* a fresh send re-enables closed VM windows */
  int id = abcl_vm_spawn(0);
  if (id >= 0) abcl_enqueue(-1, id, "tick", 0, NULL);
  return id;
}

static void dispatch(int self_id, int sender_id, const char* method, value_t* args, int n_args) {
  if (objects[self_id].class_id >= VM_CLASS_BASE) {     /* dynamically-loaded VM actor */
    abcl_vm_dispatch(self_id, sender_id, method, args, n_args);
    return;
  }
  switch (objects[self_id].class_id) {
  case CLASS_Fork: dispatch_Fork(self_id, sender_id, method, args, n_args); break;
  case CLASS_Philosopher: dispatch_Philosopher(self_id, sender_id, method, args, n_args); break;
  case CLASS_WebReceiver: dispatch_WebReceiver(self_id, sender_id, method, args, n_args); break;
  case CLASS_Dispatcher: dispatch_Dispatcher(self_id, sender_id, method, args, n_args); break;
  case CLASS_Worker:     dispatch_Worker(self_id, sender_id, method, args, n_args); break;
  case CLASS_Collector:  dispatch_Collector(self_id, sender_id, method, args, n_args); break;
  case CLASS_DiningBench: dispatch_DiningBench(self_id, sender_id, method, args, n_args); break;
  case CLASS_ForkCM:     dispatch_ForkCM(self_id, sender_id, method, args, n_args); break;
  case CLASS_PhilCM:     dispatch_PhilCM(self_id, sender_id, method, args, n_args); break;
  case CLASS_Spinner:    dispatch_Spinner(self_id, sender_id, method, args, n_args); break;
  case CLASS_Controller: dispatch_Controller(self_id, sender_id, method, args, n_args); break;
  case CLASS_Pinger:     dispatch_Pinger(self_id, sender_id, method, args, n_args); break;
  case CLASS_Ponger:     dispatch_Ponger(self_id, sender_id, method, args, n_args); break;
  default: kprintf("unknown class %d\r\n", objects[self_id].class_id);
  }
}

static void init_fields(int class_id, int self_id) {
  switch (class_id) {
  case CLASS_Fork: init_fields_Fork(self_id); break;
  case CLASS_Philosopher: init_fields_Philosopher(self_id); break;
  case CLASS_WebReceiver: break;   /* no fields */
  case CLASS_Dispatcher: init_fields_Dispatcher(self_id); break;
  case CLASS_Worker:     init_fields_Worker(self_id);     break;
  case CLASS_Collector:  init_fields_Collector(self_id);  break;
  case CLASS_DiningBench: init_fields_DiningBench(self_id); break;
  case CLASS_ForkCM:     init_fields_ForkCM(self_id);     break;
  case CLASS_PhilCM:     init_fields_PhilCM(self_id);     break;
  case CLASS_Spinner:    init_fields_Spinner(self_id);    break;
  case CLASS_Controller: init_fields_Controller(self_id); break;
  case CLASS_Pinger:     init_fields_Pinger(self_id);     break;
  case CLASS_Ponger:     init_fields_Ponger(self_id);     break;
  default: break;
  }
}

static int alloc_obj_ex(int class_id, int n_args, value_t* args, int auto_init) {
  int id;
  int i;
  /* RCU writer: reserve a slot index but DO NOT publish it yet.  Lock-free
   * readers iterate `i < n_objects`, so the slot must be fully initialised
   * BEFORE n_objects exposes it — otherwise a reader could see a half-built
   * actor (garbage class_id / an uninitialised mailbox semaphore).  We hold
   * objects_mu only to serialise concurrent registrations. */
  wait(objects_mu);
  id = n_objects;
  if (id >= MAX_OBJECTS) {            /* table full — fail instead of overflowing */
    signal(objects_mu);
    kprintf("[aipl] alloc_obj: registry full (%d), actor dropped\r\n", MAX_OBJECTS);
    return -1;
  }
  objects[id].class_id = class_id;
  for (i = 0; i < MAX_FIELDS; i++) objects[id].fields[i] = mk_int(0L);
  init_fields(class_id, id);
  mailbox_init(&objects[id].mbox);
  objects[id].started = 0;
  objects[id].dead = 0;
  objects[id].birth_ticks    = abcl_now_ms();
  objects[id].last_enq_ms = abcl_now_ms();
  objects[id].last_deq_ms = abcl_now_ms();
  objects[id].protected_from_gc = 0;
  /* PUBLISH: the barrier guarantees every store above is visible before the
   * new count, so a reader that observes id+1 also sees a complete slot. */
  rcu_assign_pointer(n_objects, id + 1);
  signal(objects_mu);
  if (auto_init) enqueue(-1, id, "init", n_args, args);
  return id;
}

static int alloc_obj(int class_id, int n_args, value_t* args) {
  return alloc_obj_ex(class_id, n_args, args, 1);
}

static void spawn_actor(int id) {
  int prio;
  int actual;
  if (objects[id].started) return;
  objects[id].started = 1;
  prio = abcl_class_prio(objects[id].class_id);
  objects[id].tid = create((void*)abcl_actor_main, 4096, prio,
                            "abcl-actor", 1, id);
  actual = getprio(objects[id].tid);
  wait(print_mu);
  kprintf("[aipl] prio class=%s id=%d tid=%d want=%d got=%d\r\n",
          abcl_class_name(objects[id].class_id), id,
          (int)objects[id].tid, prio, actual);
  signal(print_mu);
  ready(objects[id].tid, RESCHED_NO);
}

int create_obj(int class_id, int n_args, value_t* args) {
  int id = alloc_obj(class_id, n_args, args);
  spawn_actor(id);
  return id;
}

/* 自動 init を送らない生成。VM（.avm）のアクタ用。
 * alloc_obj は生成のたびに引数 0 個の "init" を投げていた。組み込みの C
 * アクタはそれを前提にしているが、VM のアクタでは前段（compile_avm）が
 * init を明示的に送るので二重になる ―― g1 が `hello, AIPL` の前に
 * `hello, 0` を出していたのがこれで、引数が 0 個の側が先に走っていた。
 * 前段が「init があれば必ず一度送る」ようになったので、ここは送らない。 */
int create_obj_noinit(int class_id) {
  int id = alloc_obj_ex(class_id, 0, NULL, 0);
  spawn_actor(id);
  return id;
}

static int _abcl_streq(const char *a, const char *b) {
  while (*a && *b && *a == *b) { a++; b++; }
  return (*a == 0 && *b == 0) ? 1 : 0;
}
int abcl_lookup_class_id(const char *name) {
  if (_abcl_streq(name, "Fork")) return CLASS_Fork;
  if (_abcl_streq(name, "Philosopher")) return CLASS_Philosopher;
  return -1;
}

thread abcl_actor_main(int self_id) {
  mailbox_t* mb = &objects[self_id].mbox;
  for (;;) {
    message_t m;
    int idx;
    uint32_t pos;
    uint32_t slot;
    int spin;
    if (global_shutdown) break;
    wait(mb->items);
    if (global_shutdown) break;
    if (objects[self_id].dead) break;   /* woken by suicide / abcl_rt_reset */
#ifdef _XINU_PLATFORM_ARM_RPI3_
    /* Pi3: strongly-ordered memory, no LDREX/STREX — dequeue under a short
     * interrupt-disabled critical section (matches the producer above). */
    {
      irqmask im = disable();
      (void)spin;
      pos  = mb->deq;
      slot = pos % (uint32_t)MAX_MAILBOX;
      if (mb->slot_seq[slot] != pos + 1) {
        restore(im);
        continue;   /* spurious wake (e.g. shutdown) */
      }
      m = mb->msgs[slot];
      mb->slot_seq[slot] = pos + (uint32_t)MAX_MAILBOX;
      mb->deq = pos + 1;
      restore(im);
    }
#else
    /* P3: lock-free MPSC consumer.  We are the only consumer for
       this mailbox, so deq does not need CAS.  Spin on slot_seq
       until the producer's release-store of pos+1 is visible. */
    pos  = __atomic_load_n(&mb->deq, __ATOMIC_ACQUIRE);
    slot = pos % (uint32_t)MAX_MAILBOX;
    for (spin = 0; spin < 1024; spin++) {
      if (__atomic_load_n(&mb->slot_seq[slot], __ATOMIC_ACQUIRE) == pos + 1) break;
    }
    if (__atomic_load_n(&mb->slot_seq[slot], __ATOMIC_ACQUIRE) != pos + 1) {
      /* Spurious wake (e.g. wake_all_actors during shutdown) — retry. */
      continue;
    }
    m = mb->msgs[slot];
    /* Recycle slot for producer at pos+MAX_MAILBOX. */
    __atomic_store_n(&mb->slot_seq[slot], pos + (uint32_t)MAX_MAILBOX, __ATOMIC_RELEASE);
    __atomic_store_n(&mb->deq, pos + 1, __ATOMIC_RELEASE);
#endif
    /* Stamp dequeue activity (GC sweep "age" = now - max(last_enq, last_deq)). */
    objects[self_id].last_deq_ms = abcl_now_ms();
    abcl_log_first_recv(self_id, m.method);
    wait(counter_mu);
    idx = ++messages_processed;
    signal(counter_mu);
    /* P1: every 25 dispatches print one liveness marker.
       P3: also include clkticks so a smoke can compute throughput. */
    if (idx % 25 == 0) {
      wait(print_mu);
      kprintf("[aipl] alive msg=%d tick=%d\r\n", idx, (int)clkticks);
      signal(print_mu);
    }
    if (_abcl_cap > 0 && idx > _abcl_cap) {
      wait(print_mu);
      kprintf("[abcl] message cap reached (%d)\r\n", _abcl_cap);
      signal(print_mu);
      abcl_shutdown();
      break;
    }
    dispatch(self_id, m.sender, m.method, m.args, m.n_args);
    if (objects[self_id].dead) {
      wait(print_mu);
      kprintf("[aipl] actor %d terminated (suicide)\r\n", self_id);
      signal(print_mu);
      break;            /* self-terminate: leave the loop, thread exits */
    }
  }
  return OK;
}

/* === Web bridge ============================================================
 * Lets an external (HTTP) message be delivered into the AIPL actor system,
 * used by apps/webactor.c so a Mac-side actor can message a Xinu AIPL actor.
 * abcl_web_init() registers a WebReceiver actor (idempotent) and returns its
 * object id; abcl_web_deliver() enqueues a string message to it. */
int abcl_rt_ready = 0;

static void abcl_rt_init_once(void) {
  if (abcl_rt_ready) return;
  counter_mu = semcreate(1);
  print_mu   = semcreate(1);
  objects_mu = semcreate(1);
  abcl_rt_ready = 1;
}

int abcl_web_init(void) {
  static int web_id = -1;
  abcl_rt_init_once();
  if (web_id < 0) {
    web_id = create_obj(CLASS_WebReceiver, 0, NULL);
    /* The web bridge actor is idle (blocked on its mailbox) and referenced by
     * nobody, so the reachability GC would collect it and cut off HTTP — pin
     * it as a permanent root. */
    if (web_id >= 0) abcl_object_protect(web_id, 1);
  }
  return web_id;
}

/* Initialise the AIPL runtime (mutexes) WITHOUT instantiating any actors.
 * Used for the "dynamic" dining demo where Xinu boots with zero dining
 * actors and the Mac spawns the Forks/Philosophers at runtime via SPAWN RPC,
 * so the first SPAWN'd Fork deterministically becomes actor id 0. */
void abcl_rt_init(void) {
  abcl_rt_init_once();
}

void abcl_web_deliver(int receiver, const char *method, const char *str) {
  /* Copy into a small rotating pool so the value_t string survives until the
   * actor thread consumes it.  `method` must be a string literal / static. */
  static char pool[8][240];
  static int  slot = 0;
  int s = slot;
  int i = 0;
  value_t a;
  slot = (slot + 1) & 7;
  while (str[i] && i < 239) { pool[s][i] = str[i]; i++; }
  pool[s][i] = '\0';
  a = mk_str(pool[s]);
  abcl_enqueue(-1, receiver, method, 1, &a);
}

thread aipl_main(void) {
  abcl_rt_init_once();
  kprintf("\r\n[abcl] starting...\r\n");
  kprintf("[aipl] start tick=%d\r\n", (int)clkticks);
  /* phase 1: alloc all globals */
  g_f0 = alloc_obj(CLASS_Fork, 0, NULL);
  g_f1 = alloc_obj(CLASS_Fork, 0, NULL);
  g_f2 = alloc_obj(CLASS_Fork, 0, NULL);
  g_f3 = alloc_obj(CLASS_Fork, 0, NULL);
  g_f4 = alloc_obj(CLASS_Fork, 0, NULL);
  g_p4 = alloc_obj(CLASS_Philosopher, 3, (value_t[]){mk_int((long)(4L)), mk_obj((int)(g_f2)), mk_obj((int)(g_f3))});
  g_p5 = alloc_obj(CLASS_Philosopher, 3, (value_t[]){mk_int((long)(5L)), mk_obj((int)(g_f3)), mk_obj((int)(g_f4))});
  /* Load-balancer (Dispatcher + Workers) is bootstrapped separately
   * by abcl_loadbal_init() — invoked from webactor_autostart so it
   * exists even when aipl_main isn't part of the boot path. */
  /* phase 2: spawn actors */
  {
    int i, total;
    wait(objects_mu); total = n_objects; signal(objects_mu);
    for (i = 0; i < total; i++) spawn_actor(i);
    /* P1: stable thread-count marker. */
    kprintf("[aipl] spawned=%d\r\n", total);
  }
  /* P1: heartbeat thread — proves the Xinu scheduler isn't
     starved by busy-looping actors.  Five ticks over ~5 sec
     should always land if mailboxes use blocking semaphores. */
  {
    extern thread abcl_heartbeat(void);
    tid_typ htid = create((void*)abcl_heartbeat, 4096, INITPRIO,
                          "aipl-hb", 0);
    if (htid != SYSERR) ready(htid, RESCHED_NO);
  }
  /* phase 3: any non-VarDecl top-level */
  /* wait for shutdown */
  while (!global_shutdown) sleep(50);
  /* P3: aggregate mailbox-drop counters across all objects so a
     smoke can verify lock-free MPSC didn't lose messages. */
  {
    /* RCU read side: walk the registry lock-free for a consistent snapshot of
     * the per-actor drop counters, concurrent with actor registration. */
    int i, total;
    uint32_t total_drops = 0;
    rcu_read_lock();
    total = rcu_n_objects();
    for (i = 0; i < total; i++)
      total_drops += objects[i].mbox.drops;
    rcu_read_unlock();
    kprintf("[abcl] done; messages=%d drops=%u tick=%d\r\n",
            messages_processed, (unsigned)total_drops, (int)clkticks);
  }
  return OK;
}

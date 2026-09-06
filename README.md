# xinu-rpi3 — Embedded Xinu on Raspberry Pi 3 B+

Fork of Embedded Xinu (Marquette University) targeting the
**Raspberry Pi 3 B+ (BCM2837, ARMv7-A Cortex-A53 in AArch32 mode)**
on real hardware.  Branch: `arm-rpi3-port`.

> Japanese-language developer log: see [`NEXT_SESSION.md`](NEXT_SESSION.md).
> The upstream Embedded Xinu README starts at section "Embedded Xinu" below.

![Xinu Pi3 window system running on a Raspberry Pi 3 B+](docs/boot-screen.jpg)

*Embedded Xinu booted on a real Raspberry Pi 3 B+: HDMI window system with live
actor table, AIPL console, shell (`xsh`), soft keyboard, and a rotating 3D
wine-glass wireframe in the graphics window.*

## Multi-core SMP & self-forming Wi-Fi mesh (2026 experiments)

This fork is one of three sibling ports — **xinu-rpi3 (A53)**, **xinu-rpi4
(A72)**, **xinu-rpi5 (A76)** — that were wired together into a single
self-forming cluster and benchmarked. The results are written up in the
companion `smp_report` (per-board SMP + D-cache) and `mesh_report` (the mesh +
distributed benchmarks) PDFs.

**4-core worker-pool SMP (newly ported here).** Upstream Xinu and this fork
originally ran on core 0 only, parking cores 1–3 in `wfe`. This branch adds an
**AArch32 worker pool** so all four Cortex-A53 cores compute:

- `loader/platforms/arm-rpi3/start.S` releases the parked secondaries (they poll
  `smp_release[]`, with an ARM-local mailbox-3 fallback) into a new `_smp_start`
  trampoline (defensive HYP→SVC drop, per-core stack).
- `system/platforms/arm-rpi3/mmu.c` `mmu_enable_secondary()` brings each worker's
  MMU up on core 0's L1 table — **MMU + I-cache on, D-cache off**. Every core in
  the same cache mode ⇒ the lock-free job mailbox is coherent with *no* cache
  maintenance, and 1-core vs 4-core timings are fair.
- `system/smp.c` / `include/smp.h`: online flags, a per-core job mailbox, a
  bounded bring-up wait, and `smp_parallel_sum()` (core 0 runs chunk 0 inline and
  takes over a stuck worker). `smp_init()` is lazy (first benchmark), so boot is
  untouched — and it degrades safely to `cores_online = 1` if a core never comes up.

Measured on real hardware (`agree=yes`, i.e. 1-core and 4-core totals match):

| Benchmark            | 1-core | 4-core | Speedup |
|----------------------|-------:|-------:|:-------:|
| dining (philosophers)| 119627 µs | 29913 µs | **3.99×** |
| primes (count)       | 274011 µs | 90826 µs | **3.01×** |
| n-queens (n=12, block split) | 111313 µs | 52487 µs | 2.12× |

```mermaid
xychart-beta
    title "Pi 3 (A53) — 4-core SMP speedup (x over 1 core)"
    x-axis [dining, primes, "n-queens"]
    y-axis "speedup" 0 --> 4.2
    bar [3.99, 3.01, 2.12]
```

dining scales almost linearly (4 cores → 3.99×); n-queens is limited by the
uneven per-column subtrees of a block split, the same effect the A72/A76 boards
show.

**Self-forming Wi-Fi ad-hoc mesh.** All three boards join one IBSS cell with no
access point: SSID `MANET`, channel 6, fixed BSSID `02:4d:41:4e:45:54`, static
`10.0.0.n/24` (this board is **node 1 = 10.0.0.1**). A periodic **HELLO beacon**
(UDP/5000, every 2 s) announces the node id; on receipt each board records the
sender, so neighbour tables fill automatically — power-on and join is enough for
all three to discover each other. Inspect convergence with `GET /manet`
(`node`, `rx`, `hello_tx`, `peers ids=`).

**Distributed benchmark routes.**
- `GET /bench?kind=nqueens|dining|primes[&n=N][&cores=K]` — the SMP benchmark
  above (1-core vs 4-core, µs, speedup, `agree=`).
- `GET /nqpart?n=N&c0=A&c1=B` — count N-Queens solutions for first-queen columns
  `[c0,c1)`, split across this board's 4 cores. A Mac orchestrator hands each
  board a disjoint column range and sums the partials, so the whole mesh solves
  one problem as a **12-core (3×4) distributed computer**. Best 3-board result:
  N=14 (365 596 solutions) in **1 458 ms — 1.87× the fastest single board**, sum
  verified. See `mesh_report` for the capacity/granularity efficiency analysis.

```mermaid
xychart-beta
    title "Distributed N-Queens (N=14, 365596 solutions) — wall-clock ms"
    x-axis ["Pi4 4c", "Pi5 4c", "8-core", "9-core", "12-core"]
    y-axis "ms (lower is better)" 0 --> 5400
    bar [5216, 2722, 2088, 1585, 1458]
```

More cores → shorter wall-clock: 12 cores across the 3 boards solve N=14 in
1 458 ms, 1.87× the fastest single board (heterogeneous ideal ≈ 1 172 ms, so
~80% of the capacity ceiling; the rest is column-granularity loss).

## What works on this fork

Hardware bring-up:
- **HDMI framebuffer** + console
- **USB OTG** (DWC2) — keyboard, mouse
- **USB ethernet** (LAN78xx / LAN7515 onboard NIC) — ping ~3 ms, TCP bidirectional
- **WiFi** (BCM43455 over the Arasan SDIO) — scan, WPA2 join, DHCP, ARP/ICMP
- **HDMI window manager** (ported from Pi 5; interactive `xsh` + WiFi UI)
- **MMU** — ARMv7-A short-descriptor identity map, RAM vs MMIO region attributes
  enforced (Normal cacheable RAM, Device strongly-ordered MMIO, MMIO no-execute)

Runtime + observability:
- **AIPL actor runtime** — `Fork`, `Philosopher` (dining demo), `WebReceiver`,
  plus a `Dispatcher` + 4 `Worker` load balancer
- **Lock-free MPSC mailbox** — stress-tested with 256 in-flight tasks, 0 drops
- **HTTP server on port 8080** with 20+ routes for introspection and control
  (`/api/actors`, `/api/threads`, `/api/memstat`, `/api/mmu`, `/api/loadbal/*`,
  `/gc`, `/mmio-read`, `/compile`, `/upload`, `/kexec`, `/reboot`, …)
- **AIPL-RPC server on port 5555** — Mac-side tooling can `SPAWN` / `COMPILE`
  AIPL programs at runtime

C JIT (`apps/cc_mvp.c`):
- Compiles a Turing-complete C subset to native **ARM32 (AArch32) machine code**
  and executes it in-place out of an identity-mapped RAM page
- Supports: locals, `+ - == != < > <= >= = if / else while return`, function
  calls into a builtin symbol table (`print_int`, `actor_count`, `actor_age`,
  `now_ms`, …)
- Verified on hardware: `int main() { int s=0; int i=1; while (i<=10) { s=s+i; i=i+1; } return s; }`
  → returns 55 from a 140-byte ARM32 code blob the kernel emitted at runtime

Network-only kernel update (no SD swap):
- `POST /upload` — receive a kernel image into a 512 KB slot (chunked HTTP
  reader, ~11 s for the current 305 KB build at 26 KB/s)
- `POST /kexec` — disable MMU, copy uploaded image to `0x8000`, jump to it; the
  new kernel boots fresh and re-enables the MMU
- Volatile (a power cycle still reboots from SD), but the dev loop no longer
  needs the user to physically swap the SD card for each iteration

Performance work documented in commit history:
- `TCP_GRACIOUSACK` enabled + 4 KB chunked body reader → kills the O(n²)
  byte-at-a-time strstr pattern in the HTTP request parser
- `TCP_IBLEN` 16 KB → 65528 (set just under the `ushort` window cap — `65536`
  bricks because `tcpSendWindow` overflows to advertising window=0)
- End-to-end upload: 91 s → 11 s (~8× on 305 KB)

## HDMI desktop, WiFi & GC-backed N-Queens (this branch)

On top of the bring-up above, this branch adds an interactive on-screen
desktop, a WiFi stack with shell commands, and a GC-backed distributed
benchmark. Current build tag: `xinu-gwm-b69` (see `WIFI_BUILD_ID` in
`apps/wifi.c`).

**gwm window manager (HDMI, 1280×800)** — `apps/gwm.c` + `apps/gvideo.c`,
drawn over the framebuffer that `screenInit` set up. Windows:
- **Shell** — a *real* `xsh` bound to the `GWINCON0` software console:
  command output, `ls`, `ps`, `cat`, `wifi`, … all run and print here.
  stdin is injected over HTTP (`/api/wifi/key`); stdout/stderr render via
  the window's text ring.
- **graphics** — a wire-frame 3-D wine glass that tumbles on the X/Y/Z
  axes; started on demand by the `wine` shell command (30 turns, then
  idle so it does not steal CPU from the network responder).
- **AIPL actors / AIPL console / Soft keyboard / Info** — live monitors.
- **WiFi indicator** (bottom-right): white = not connected, green = up,
  with the joined SSID printed underneath.

**WiFi (BCM43455 over SDIO)** — scan / WPA2 join / DHCP / ARP+ICMP. Shell
commands:
- `wifi status` — connection state (SSID + IP/GW)
- `wifi on` — scan, list candidate APs, pick one by number, enter the
  password, connect (join + DHCP)
- `wifi off` — disconnect
- `wifi-invest` — when "connected but ping fails", restart the responder,
  send gratuitous ARP, and retry the gateway probe until it works (then
  auto-finish), or print the most likely reason

**Distributed N-Queens with actor GC** — the `Dispatcher` + 4 `Worker`
load balancer solves N-Queens split by first column (one `compute_nq`
task per column). A `Collector` actor sweeps idle actors periodically
(`/api/gc-actor/*`). Results are **push-aggregated**: workers push `done`
to the Dispatcher, which sums the partial counts, so the client reads one
running total from `/api/loadbal/nqueens-result` instead of polling every
task. Verified: n=8 → 92, n=9 → 352.

**Mac dashboard** — `src/python-aipl/aipl_dashboard.py` (`--dashboard 8899`)
serves `/pi3`, a browser mirror of the gwm desktop: the Shell window
mirrors the real `xsh` ring, keystrokes are proxied to the Pi 3, and the
WiFi widget scans / connects / shows status. See the Japanese user guide
in `docs/USERS-GUIDE-JA.md`.

> Note: `webactor` is single-threaded, so heavy on-CPU work (a WiFi scan,
> a continuous animation, or a big N-Queens run) briefly starves the HTTP
> control plane. Animations are therefore on-demand and bounded.

## Quick start

Build:

    cd compile
    make PLATFORM=arm-rpi3
    # produces ./xinu.boot (~305 KB)

    # If you changed a shared header (e.g. include/thread.h), libxc is NOT
    # rebuilt by `make clean` — force it or printf-to-device silently breaks:
    #   rm -f lib/libxc/*.o lib/libxc.a && make PLATFORM=arm-rpi3

Deploy A — over the network (Pi 3 must already be running a `/kexec`-capable
build):

    curl --data-binary @xinu.boot \
        "http://192.168.3.50:8080/upload?dst=xinu.boot"
    curl -X POST http://192.168.3.50:8080/kexec
    sleep 25
    curl http://192.168.3.50:8080/api/mmu   # should respond if the new
                                             # kernel booted cleanly

Deploy B — physical SD swap (initial bootstrap, or recovery after a bricked
`/kexec`):

    # eject SD from Pi 3, plug into Mac, /Volumes/XINU appears
    cp xinu.boot /Volumes/XINU/kernel.img
    sync && diskutil eject /Volumes/XINU
    # back into Pi 3, power on

Backup kernels (in case a deploy bricks):

    /Users/kodamay/tftpboot/kernel.img.pre-mmu-bak    # MMU off, oldest safe
    /Users/kodamay/tftpboot/kernel.img.pre-kexec-bak  # MMU on, no /kexec
    /Users/kodamay/tftpboot/kernel.img                # current build

## Running AIPL on this board

AIPL programs run here as `.avm` bytecode interpreted by an in-kernel VM.
Unlike the Pi 4 and Pi 5 (which carry the front end on the board and accept
raw AIPL over `POST /cc`), the Pi 3 is fed a **pre-compiled module** from the
Mac. **No kernel reflash is needed** — the module loads into the running kernel.

    # 1. AIPL -> .avm  (Mac side; ~/projects/aice-avm)
    ./_build/default/compile_avm.exe prog.aipl prog.avm

    # 2. check the values on the host VM first (it has /api/console)
    ./_build/default/server.exe 8099 --no-open
    curl --data-binary @prog.avm "http://127.0.0.1:8099/actor/loadvm?ask=0"

    # 3. send it to the board (port 8080; ?ask=0 is REQUIRED)
    curl --data-binary @prog.avm "http://192.168.3.50:8080/actor/loadvm?ask=0"
    loadvm: body=168 accepted=1 spawned actor id=1

    # 4. read what it printed
    curl "http://192.168.3.50:8080/api/console"
    {"total":3,"lines":["a2: hello, AIPL","a2: tick 1","a2: tick 2"]}

Notes:

- **`?ask=0` is required.** Without it `loadvm` never answers and the board
  looks dead.
- **One module at a time.** Loading a new one folds away the previous
  program's actors and clears its exposed routes.
- **Leave ~8 s between requests.** Back-to-back HTTP can wedge the server
  (the kernel itself stays alive).
- `web_expose` publishes an actor: `GET /api/x/<path>?method=&args=`.
- `ai_call(prompt)` runs a small model baked into the kernel (Karpathy
  `stories260K`), 2–6 s per call, producing the same text as the Pi 4, Pi 5
  and the Mac reference implementation. Nothing leaves the board.
- Nine of the ten canonical guide samples match the canonical implementation
  exactly; g9's ordering can differ because actors here are real processes.
  See chapter 39 of the AIPL user's guide (`~/aios/abclcp/docs/`).


### How much of AIPL runs here

Checked against the **sources** of the canonical language — the keywords in
`src/lexer.mll`, the builtin table in `src/typing_env.ml`, and the effect table
in the user's guide. The front end is on the Mac (`compile.ml`), so a change
there needs no reflash; only the rows that need a VM instruction do.

> This used to say "25 of 25". That 25 was a split **made up here**, not read off
> the sources; counting again turned up `sender`, `timed_out`, `typeof` and
> `float` field declarations. `sender` happened to work here already — it broke
> only on the boards that carry their own front end. Count from the sources.

| Feature | Runs | Notes |
|---|---|---|
| `class` / `method` / `var` / fields | yes | |
| `float x = 1.5;` (a field declaration) | yes | `float` is a reserved word, distinct from `var` |
| `sender` | yes | who sent the message being handled; `send sender.m();` |
| `timed_out(r)` | yes | the third observer of a `result` (with `is_ok` / `value`) |
| `typeof(x)` | yes | VM instruction `0x5F` |
| `neg(x)` | yes | canonically both `int -> int` and `float -> float`; it is not one of the maths builtins |
| `new` / `init` / `send` / `send!` | yes | `new` always calls `init` |
| `now` / `future` / `await` / deadlines | yes | by **continuation splitting** |
| `select` / `case` / `timeout` | yes | `case` names must be real methods |
| Booleans and comparisons | yes | printed as `true` / `false` |
| Strings and `++` | yes | |
| `wait(ms)` | yes | |
| `web_listen` / `web_expose` | yes | |
| `ai_call` | yes | on-board model; nothing leaves the box |
| Deadline without `else` (`result<t>`) | yes | failure is a reserved value; no new instruction |
| `is_ok` / `value` | yes | lowered to a comparison and a branch |
| `replyto` / `answer` / parameter type `reply` | yes | a reply destination *is* an actor id (`__snd`) |
| `acquire` / `release` | yes | VM instructions `0x53`/`0x54`; a named semaphore per resource |
| Type / effect / level annotations | accepted, discarded | checked on the canonical side (`tc`) |
| `float` and float literals | yes | boxed in a table; the tag is 3 bits wide (see below) |
| Arrays (`array_*`) | yes | boxed in a table; `push`/`set` return a new array, as canonically |
| Math builtins (`sqrt` etc.) | yes | the same series the Pi 4 and Pi 5 use; `sqrt` by Newton (no `fsqrt` on ARM32) |
| The `call` statement | yes | a builtin called as a statement inside a method |
| `spawn("Class","name")` | yes | the new actor id goes into the published table under that name |
| `remote("host:port","actor")` | yes | over UDP/9010, using the Xinu UDP device layer |

`remote(...)` is carried over UDP/9010 with the same one-line ASCII datagram
the Pi 4, the Pi 5 and the Mac host VM speak, so a program here can call an
actor another node published with `web_expose`:

```
var d = now remote("192.168.3.101", "echo").say(21) timeout 2000 else -1;
send remote("192.168.3.50", "echo").say(5);
```

Unlike the Pi 4 and the Pi 5 — whose TCP is listen-only, so they build their
frames by hand — this board has the real Xinu network stack, so it just uses
`udpAlloc` / `open` / `read` / `write`. The receiving end is a daemon process
on UDP 9010, opened `PASSIVE` so the sender's pseudo-header comes with each
read and the reply needs no ARP and no peer table. The sending end takes a
different local port: two devices on the same local port make the demux
ambiguous, and the peer answers to the request's source port anyway.
VM instructions `0x55` (REMOTE_SEND) and `0x56` (REMOTE_CALL); `0x56` also
takes the default value, so a deadline without `else` yields `err` and the
timeout case stays in one place.

Two things cost a day here and are worth writing down. `udpRecv` converts
`srcPort` / `dstPort` / `len` to host order **before** it buffers the packet,
so a second `net2hs` in the daemon corrupts the length and the request is
dropped in silence. And a device opened `PASSIVE` cannot be written with a
bare payload: its `udpWrite` wants pseudo-header + UDP header + payload and
rejects anything shorter than 20 bytes. The reply therefore goes out on a
separate, ordinary device (`reply_to`); the peer matches on the request id, so
the source port of the reply does not matter.

Measured, all four implementations calling each other (`say(21)` → `42`):
the Pi 3, the Pi 4, the Pi 5 and the Mac host VM each reach the other three,
and an unreachable host falls to the `else` value.

`acquire` here is a **real named semaphore**, not just bookkeeping: actors on
this board are real Xinu processes, so two of them can genuinely contend. Same
owner re-entry is allowed (the canonical checker rejects a double acquire
statically anyway), and a `release` of a resource one does not hold is ignored.

Tag discipline, for anyone touching `vm_fmt_val`. A negative value is simply an
integer; for a non-negative one the top three bits (`0x70000000`) are a tag read
as a **pattern**, not as independent bits:

    000 int (0..0x0FFFFFFF)   001 err   010 bool
    011 float (index)         100 string (index)   101 array (index)

**Every tag test first checks that the value is non-negative** — without that
guard `-1` matches every tag at once. Testing single bits instead of the pattern
is what breaks once a fourth or fifth kind of value appears; that is why floats
are not simply "one more bit".

Two things about the maths, since they were both learned the hard way on this
board. `sqrt` cannot be the `fsqrt` instruction the Pi 4 and Pi 5 use — this is
ARM32 with software floating point — so it is Newton's method. And the exponent
manipulation inside `exp` and `log` must use `unsigned long long`: `long` is
**32 bits** here, so the 64-bit shifts silently do nothing and you get
`exp(1) = 0` while `sqrt`, `sin`, `cos` and `atan` all still look right.
Measure the maths builtins **one function at a time**.

## A few useful HTTP routes

    GET  /api/mmu                          MMU SCTLR / TTBR0
    GET  /api/console[?clear=1]            program output (print) as JSON
    GET  /api/routes                       paths published with web_expose
    GET  /api/x/<path>?method=&args=       call an exposed actor
    POST /actor/loadvm?ask=0               body = .avm module; load + run
    GET  /api/actors                       AIPL actor inventory
    GET  /api/threads                      kernel thread table
    GET  /api/memstat                      free memory
    GET  /api/loadbal/stats                dispatcher + worker counters
    POST /api/loadbal/submit?n=K&ms=M      K sleep tasks of M ms each
    POST /api/loadbal/jit?n=K&prog=P       K JIT tasks (compile + run)
    POST /compile                          body = C source; runs it via JIT
    POST /upload?dst=NAME                  body = bytes; lands in upload slot
    POST /kexec                            jump to last upload as new kernel
    POST /reboot                           watchdog reset (re-boot from SD)

    WiFi + desktop:
    GET  /api/wifi/scan                    scan APs (JSON; ~30 s, drops link)
    GET  /api/wifi/join?ssid=S&pass=P      WPA2 join
    GET  /api/wifi/dhcp                    DHCP lease
    GET  /api/wifi/key?c=N                 inject a keystroke into the Shell
    GET  /api/wifi/shellring               dump the Shell window text ring

    N-Queens + actor GC:
    GET  /api/loadbal/init                 spawn Dispatcher + 4 Workers
    GET  /api/gc-actor/init                spawn Collector
    GET  /api/gc-actor/enable?on=1         enable periodic sweeps
    GET  /api/gc-actor/stats               GC sweep counters
    GET  /api/loadbal/nqueens?n=N          submit an N-Queens job (push-agg)
    GET  /api/loadbal/nqueens-result       one-poll aggregated total

## Repository layout (fork-specific additions)

    apps/cc_mvp.c                            C JIT (Stage 5)
    apps/abcl_program.c                      AIPL runtime + Dispatcher/Worker/Collector
                                             classes + N-Queens push-aggregation
    apps/webactor.c                          HTTP server with the routes above
    apps/wifi.c                              BCM43455 WiFi driver + investigate
    apps/gwm.c / apps/gvideo.c               HDMI window manager + drawing prims
    device/gwincon/                          windowed-shell console (xsh I/O)
    shell/xsh_wifi.c / xsh_wifi_invest.c     wifi / wifi-invest commands
    shell/xsh_wine.c                         wine (3-D wire-frame) command
    docs/USERS-GUIDE-JA.md                   Japanese user guide
    docs/nqueens_bench_report.tex/.pdf       N-Queens + GC benchmark report
    apps/sd_block.c                          SD driver stub (doesn't yet work —
                                             needs SDHOST port; see Known limits)
    system/platforms/arm-rpi3/mmu.c          MMU enable/disable
    include/tcp.h                            TCP_IBLEN=65528, GRACIOUSACK enabled

Driver fixes documented in commit history (see also `NEXT_SESSION.md`):

    device/smsc9512/lan78xx.c / lan78xx.h    Pi 3 onboard NIC driver
    system/platforms/arm-rpi3/usb_dwc_hcd.c  fast bulk-NAK defer for ping RTT

## Known limits

- `/kexec` is volatile — a real power cycle still re-loads from SD
- D-cache and I-cache are intentionally disabled (`SCTLR.C=0`, `SCTLR.I=0`);
  Cortex-A53 needs a set/way invalidate sweep before enabling them and that
  is the next planned commit
- SD writeback (for a persistent network kernel update) needs an SDHOST port
  — BCM2837's default SD controller is SDHOST at 0x3F202000, not the EMMC at
  0x3F300000 that `apps/sd_block.c` currently targets
- Pi 3 firmware TFTP fallback boot is not enabled (would need OTP bit set
  on the SoC), so kernel deploys without an existing `/kexec`-capable kernel
  on SD still need a physical SD swap
- `telnetd` on port 23 has an unsolved bug where child shell commands' output
  is intermittently lost (the shell thread's own output works); see
  `NEXT_SESSION.md` for the trace evidence

## License

Same as upstream Embedded Xinu — see `LICENSE`.

---

# Embedded Xinu #

Embedded Xinu, Copyright (C) 2008, 2009, 2010.  All rights reserved.

Version: 2.01

 1. What is Embedded Xinu?
 2. Directory Structure
 3. Prerequisites
 4. Installation Instructions
    1. Build Embedded Xinu
    2. Make serial and network connections
    3. Enter Common Firmware Environment prompt
    4. Set IP address
    5. Load image over TFTP
 5. Links


## 1. What is Embedded Xinu? ##

Xinu ("Xinu is not unix", a recursive acronym) is a UNIX-like operating
system originally developed by Douglas Comer for instructional purposes at
Purdue University in the 1980s.

Embedded Xinu is a reimplementation of the original Xinu operating system
on the MIPS processor which is able to run on inexpensive wireless routers
and is suitable for courses and research in the areas of Operating Systems,
Hardware Systems, Embedded Systems, Networking, and Compilers.

## 2. Directory Structure ##

Once you have downloaded and extracted the xinu tarball, you will see a
basic directory structure:

	AUTHORS   device/   lib/     mem/      loader/   README  system/
	compile/  include/  LICENSE  mailbox/  network/  shell/  test/

 * `AUTHORS`  a brief history of contributors to the Xinu operating system
              in it's varying iterations.
 * `compile/` contains the Makefile and other necessities for building the
              Xinu system once you have a cross-compiler.
 * `device/`  contains directories with source for all device drivers in Xinu.
 * `include/` contains all the header files used by Xinu.
 * `lib/`     contains library folders (e.g., `libxc/`) with a Makefile and 
              source for the library
 * `LICENSE`  the license under which this project falls.
 * `loader/`  contains assembly files and is where the bootloader will begin
              execution of O/S code.
 * `mailbox/` contains source for the mailbox message queuing system.
 * `mem/`     contains source for page-based memory protection.
 * `network/` contains code for the TCP/IP networking stack.
 * `README`   this document.
 * `RELEASE`  release notes for the current version.
 * `shell/`   contains the source for all Xinu shell related functions.
 * `system/`  contains the source for all Xinu system functions such as the
              nulluser process (`initialize.c`) as well as code to set up a C
              environment (`startup.S`).
 * `test/`    contains a number of testcases (which can be run using the shell
              command `testsuite`).

## 3. Prerequisites ##

### 3.1 Supported platform with hardware modification ###

To run Embedded Xinu you need a supported router or virtual machine.
Currently, Embedded Xinu supports Linksys WRT54GL, Linksys WRT160NL,
and the Qemu-mipsel virtual machine.  For an updated list
of supported platforms, visit:

https://xinu.cs.mu.edu/index.php/List_of_supported_platforms

In order to communicate with the router, you need to perform a hardware
modification that will expose the serial port that exists on the PCB.  For
more information on this process, see:

https://xinu.cs.mu.edu/index.php/Modify_the_Linksys_hardware

### 3.2 Cross-compiler ###

To build Embedded Xinu you will need a cross-compiler from your host
computer's architecture to MIPSEL (little endian MIPS for the 54GL router)
or MIPS (big endian for the 160NL router).  Instructions on how to do this
can be found here:

https://xinu.cs.mu.edu/index.php/Build_Xinu#Cross-Compiler

### 3.3 Serial communication software ###

Any serial communication software will do. The Xinu Console Tools include
a program called tty-connect which can serve the purpose for a UNIX 
environment.  More information about the Xinu Console Tools can be found 
at:

https://xinu.cs.mu.edu/index.php/Console_Tools#Xinu_Console_Tools

### 3.4 TFTP server software ###

A TFTP server will provide the router with the ability to download and run
the compiled Embedded Xinu image.  

## 4. Installation Instructions ##

### 4.1 Build Embedded Xinu ###

Update the `MIPS_ROOT` and `MIPS_PREFIX` variables in compile/mipsVars to 
correctly point to the cross-compiler on your machine.

Then, from the compile directory, simply run make, which should leave you
with a xinu.boot file.  This is the binary image you need to transfer to
your router for it to run Embedded Xinu.  The default build is for the
WRT54GL router; change the compile/Makefile `PLATFORM` variable for other
builds.  See the compile/platforms directory for supported configurations.

### 4.2 Make serial and network connections ###

After creating the `xinu.boot` image you can connect the router's serial
port to your computer and open up a connection using the following
settings:

 - 8 data bits, no parity bit, and 1 stop bit (8N1)
 - 115200 bps

### 4.3 Enter Common Firmware Environment prompt ###

With the serial connection open, power on the router and immediately start
sending breaks (Control-C) to the device, if your luck holds you will be
greeted with a CFE prompt.

    CFE>

If the router seems to start booting up, you can powercycle and try again.

### 4.4 Set IP address ###

By default, the router will have a static IP address of 192.168.1.1.  If you
need to set a different address for your network, run one of the following
commands:

    ifconfig eth0 -auto                      if you are using a DHCP server 
    ifconfig eth0 -addr=*.*.*.*              for a static IP address

### 4.5 Load image over TFTP ###

On a computer that is network accessible from the router, start your TFTP
server and place the xinu.boot image in the root directory that the server
makes available.

Then, on the router type the command:

    CFE> boot -elf [TFTP server IP]:xinu.boot

If all has gone correctly the router you will be greeted with the Xinu Shell
(`xsh$ `), which means you are now running Embedded Xinu!

## 5. Links ##

### The Embedded Xinu Wiki ###

The home of the Embedded Xinu project

    https://xinu.cs.mu.edu/index.php/

### Dr. Brylow's Embedded Xinu Lab Infrastructure Page ###

More information about the Embedded Xinu Lab at Marquette University

    http://www.mscs.mu.edu/~brylow/xinu/


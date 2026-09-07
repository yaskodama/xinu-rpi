/**
 * @file wifi.c
 *
 * Raspberry Pi 3 B+ on-board WiFi (Cypress/Broadcom BCM43455) bring-up.
 *
 * STAGE 1+2 ONLY — SDIO host controller + chip power-on + SDIO enumeration +
 * backplane chip-ID readback.  This does NOT download firmware (Stage 3) and
 * does NOT join any network.  The goal of this stage is a single, verifiable
 * milestone: power the WiFi chip, talk to it over SDIO, and read back its
 * silicon chip-ID (expected 0x4345 for the BCM43455 on the Pi 3 B+).
 *
 * Hardware facts (Pi 3 / BCM2837), confirmed against raspberrypi/linux:
 *   - The Pi 3 has TWO SD controllers.  The microSD CARD is on the custom
 *     "sdhost" controller (0x3F202000, GPIO48-53).  The Arasan SDHCI-compatible
 *     "emmc" controller (0x3F300000) is wired to the WiFi chip's SDIO bus on
 *     GPIO34-39 (ALT3).  So the EMMC register block we drive here is the WiFi
 *     SDIO host, NOT the card. (apps/sd_block.c targeted the same block but
 *     wrongly assumed the card was on it — that is why it never worked.)
 *   - WiFi power (WL_REG_ON) is NOT a SoC GPIO; it is expander GPIO 1 on the
 *     VideoCore firmware GPIO expander.  dts: wifi_pwrseq reset-gpios =
 *     <&expgpio 1 ACTIVE_LOW>, and expgpio names pin 1 "WL_ON".  The expander
 *     base in the firmware mailbox numbering is 128, so WL_ON = firmware GPIO
 *     129.  We assert it via the property mailbox SET_GPIO_STATE tag.
 *
 * Reference: raspberrypi/linux drivers/net/wireless/.../brcmfmac/{sdio,chip,
 * bcmsdh}.c and drivers/mmc/host/sdhci*.c (cloned to ~/projects/rpi-linux-ref).
 *
 * ★ Many constants below need on-hardware confirmation; every step emits a
 * kprintf marker "[wifi] ..." so the serial console shows exactly how far the
 * bring-up gets.  This is deliberate — the deploy loop (SD swap + power cycle)
 * is slow, so the first boot must be maximally diagnostic.
 */

#include <kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <clock.h>
#include <thread.h>
#include <semaphore.h>
#include <stdarg.h>

/* Build id — bump on every flashed build so /api/wifi/probe (and the serial
 * trace) unambiguously report WHICH kernel is actually running.  The slow
 * SD-swap + power-cycle deploy loop kept leaving a stale kernel resident in
 * RAM; this removes the "is the new code even running?" guesswork. */
#define WIFI_BUILD_ID "xinu-gwm-b69 (N-Queens push-aggregation: Dispatcher sums partials; /api/loadbal/nqueens-result one-poll total)"

extern int kprintf(const char *, ...);
extern int _doprnt(const char *fmt, va_list ap, int (*putc)(int, int), int arg);

/* ------------------------------------------------------------------ *
 *  Trace buffer — every [wifi] line is captured here AND echoed to    *
 *  the serial console, so the HTTP reply can carry the whole trace.   *
 * ------------------------------------------------------------------ */
static char wifi_tbuf[8000];
static int  wifi_tn;

static int wifi_tputc(int c, int arg)
{
    (void)arg;
    if (wifi_tn < (int)sizeof(wifi_tbuf) - 1)
        wifi_tbuf[wifi_tn++] = (char)c;
    return c;
}

static void wifi_log(const char *fmt, ...)
{
    va_list ap;
    int start = wifi_tn;
    va_start(ap, fmt);
    _doprnt(fmt, ap, wifi_tputc, 0);
    va_end(ap);
    wifi_tbuf[wifi_tn] = '\0';
    kprintf("%s", &wifi_tbuf[start]);   /* mirror to serial console */
}

/* Public: the captured trace from the last wifi_probe() run. */
const char *wifi_trace(void) { return wifi_tbuf; }

/* Last-join diagnostics, surfaced via HTTP response headers because the long
 * (~150 s) join leaves the TCP connection unable to deliver the large trace
 * body reliably, while the small header block always arrives. */
static int wifi_d_sup = -999, wifi_d_pmk = -999, wifi_d_nev = 0,
           wifi_d_eapol = 0, wifi_d_link = -1, wifi_d_lastev = -1,
           wifi_d_laststat = 0;
static int wifi_d_seq[16];     /* first 16 event codes, in order */
void wifi_diag(int *sup, int *pmk, int *nev, int *eapol, int *link,
               int *lastev, int *laststat)
{
    *sup = wifi_d_sup; *pmk = wifi_d_pmk; *nev = wifi_d_nev;
    *eapol = wifi_d_eapol; *link = wifi_d_link;
    *lastev = wifi_d_lastev; *laststat = wifi_d_laststat;
}
/* Copy the recorded event-code sequence; returns how many (<=16). */
int wifi_diag_seq(int *out, int cap)
{
    int i, n = wifi_d_nev; if (n > 16) n = 16; if (n > cap) n = cap;
    for (i = 0; i < n; i++) out[i] = wifi_d_seq[i];
    return n;
}

/* Directed-join target: when wifi_tgt_set, wifi_scan() records the BSSID and
 * the real firmware chanspec of the AP whose SSID matches wifi_tgt_ssid, so
 * the join can go straight to that BSSID/channel instead of broadcast + scan-
 * all (which reaches PROBREQ but never associates on this firmware). */
static char     wifi_tgt_ssid[33];
static int      wifi_tgt_slen = 0, wifi_tgt_set = 0, wifi_tgt_found = 0;
static uint8_t  wifi_tgt_bssid[6];
static uint16_t wifi_tgt_chanspec = 0;
int wifi_tgt_diag(int *found, int *chanspec) { *found = wifi_tgt_found; *chanspec = wifi_tgt_chanspec; return wifi_tgt_found; }

/* ------------------------------------------------------------------ *
 *  MMIO bases (BCM2837, peripheral base 0x3F000000)                  *
 * ------------------------------------------------------------------ */
#define WIFI_GPIO_BASE     0x3F200000UL
#define WIFI_EMMC_BASE     0x3F300000UL   /* Arasan SDHCI = WiFi SDIO host */
#define WIFI_MBOX_BASE     0x3F00B880UL   /* VideoCore mailbox             */

/* GPIO function-select registers (each GPFSELn covers 10 pins, 3 bits each) */
#define GPFSEL3            (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x0C)) /* GPIO30-39 */
#define GPPUD              (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x94))
#define GPPUDCLK0          (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x98))
#define GPPUDCLK1          (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x9C)) /* GPIO32-53 */
#define GPFSEL4            (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x10)) /* GPIO40-49 */
#define GPFSEL5            (*(volatile uint32_t *)(WIFI_GPIO_BASE + 0x14)) /* GPIO50-59 */

/* Clock manager (GPCLK2 = WIFI_CLK on GPIO43, dts gpclk2_gpio43). */
#define WIFI_CM_BASE       0x3F101000UL
#define CM_GP2CTL          (*(volatile uint32_t *)(WIFI_CM_BASE + 0x80))
#define CM_GP2DIV          (*(volatile uint32_t *)(WIFI_CM_BASE + 0x84))
#define CM_PASSWD          0x5A000000u
#define CM_CTL_ENAB        (1u << 4)
#define CM_CTL_BUSY        (1u << 7)
#define CM_SRC_OSC         1          /* 19.2 MHz crystal oscillator */

/* ------------------------------------------------------------------ *
 *  Arasan SDHCI register block (SDHCI v3 layout)                     *
 * ------------------------------------------------------------------ */
#define EMMC_ARG2          (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x00))
#define EMMC_BLKSIZECNT    (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x04))
#define EMMC_ARG1          (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x08))
#define EMMC_CMDTM         (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x0C))
#define EMMC_RESP0         (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x10))
#define EMMC_RESP1         (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x14))
#define EMMC_RESP2         (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x18))
#define EMMC_RESP3         (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x1C))
#define EMMC_DATA          (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x20))
#define EMMC_STATUS        (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x24))
#define EMMC_CONTROL0      (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x28))
#define EMMC_CONTROL1      (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x2C))
#define EMMC_INTERRUPT     (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x30))
#define EMMC_IRPT_MASK     (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x34))
#define EMMC_IRPT_EN       (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x38))
#define EMMC_CONTROL2      (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0x3C))
#define EMMC_SLOTISR_VER   (*(volatile uint32_t *)(WIFI_EMMC_BASE + 0xFC))

/* STATUS bits */
#define SR_CMD_INHIBIT     (1u << 0)
#define SR_DAT_INHIBIT     (1u << 1)

/* CONTROL1 bits */
#define C1_CLK_INTLEN      (1u << 0)   /* internal clock enable             */
#define C1_CLK_STABLE      (1u << 1)
#define C1_CLK_EN          (1u << 2)   /* clock enable to the card/chip     */
#define C1_TOUNIT_MAX      (0xeu << 16)
#define C1_SRST_HC         (1u << 24)  /* reset complete host circuit       */
#define C1_SRST_CMD        (1u << 25)
#define C1_SRST_DATA       (1u << 26)

/* INTERRUPT bits (also used as IRPT_EN/MASK) */
#define INT_CMD_DONE       (1u << 0)
#define INT_DATA_DONE      (1u << 1)
#define INT_WRITE_RDY      (1u << 4)
#define INT_READ_RDY       (1u << 5)
#define INT_ERR            0xFFFF8000u

/* CMDTM response-type encodings (bits 16-17) + flags */
#define CMD_RSPNS_NONE     (0u << 16)
#define CMD_RSPNS_136      (1u << 16)
#define CMD_RSPNS_48       (2u << 16)
#define CMD_RSPNS_48BUSY   (3u << 16)
#define CMD_CRCCHK_EN      (1u << 19)
#define CMD_IXCHK_EN       (1u << 20)
#define CMD_ISDATA         (1u << 21)
#define TM_DAT_DIR_CH      (1u << 4)   /* card -> host (read)               */

/* ------------------------------------------------------------------ *
 *  SDIO command set + register addresses (SDIO simplified spec)      *
 * ------------------------------------------------------------------ */
#define SD_CMD0_GO_IDLE         0
#define SD_CMD3_SEND_RCA        3
#define SD_CMD5_IO_OP_COND      5
#define SD_CMD7_SELECT          7
#define SD_CMD52_IO_RW_DIRECT   52
#define SD_CMD53_IO_RW_EXT      53

/* CCCR (function 0) registers */
#define SDIO_CCCR_IOEx          0x02   /* I/O function enable               */
#define SDIO_CCCR_IORx          0x03   /* I/O function ready                */
#define SDIO_CCCR_INT_ENABLE    0x04   /* interrupt enable                  */
#define SDIO_CCCR_BUS_IF        0x07   /* bus width                         */

/* Broadcom F1 backplane window registers (brcmfmac sdio.h) */
#define SBSDIO_FUNC1_SBADDRLOW  0x1000A
#define SBSDIO_FUNC1_SBADDRMID  0x1000B
#define SBSDIO_FUNC1_SBADDRHIGH 0x1000C
#define SBSDIO_SB_OFT_ADDR_MASK 0x07FFF
#define SBSDIO_SB_ACCESS_2_4B   0x08000  /* 32-bit access flag in the offset */
#define SBSDIO_SBWINDOW_MASK    0xFFFF8000u

/* Backplane: chipcommon enumeration base + chip-id register (offset 0) */
#define SI_ENUM_BASE            0x18000000u
#define CID_ID_MASK             0x0000FFFFu
#define BCM43455_CHIP_ID        0x4345     /* what we hope to read back      */

#define WIFI_GPIO_WL_ON_FW      129        /* expander pin 1 + base 128      */

/* ------------------------------------------------------------------ *
 *  tiny busy-wait (Xinu sleep() is ms-granular; we need sub-ms too)  *
 * ------------------------------------------------------------------ */
static void wifi_udelay(volatile unsigned long n)
{
    while (n--) { asm volatile("nop"); }
}

/* Precise microsecond delay via the BCM free-running system timer (1 MHz). */
#define SYSTIMER_CLO (*(volatile uint32_t *)(0x3F003000UL + 0x04))
static void wifi_delay_us(uint32_t us)
{
    uint32_t start = SYSTIMER_CLO;
    while ((SYSTIMER_CLO - start) < us) { /* spin */ }
}

/* Write an EMMC/SDHCI register, then honour the BCM2835 Arasan erratum: at the
 * <=400 kHz init clock, successive register writes within ~2 SD clocks can be
 * LOST (clock-domain crossing).  sdhci-iproc delays ~4 SD clocks (~10us @
 * 400kHz) after each write; bcm2835-mmc ~6us.  Without this the command/clock
 * register writes silently vanish and CMD5 never actually issues -> the CMD5
 * timeout we were chasing.  The DATA register is exempt (per the erratum). */
static void ew(volatile uint32_t *reg, uint32_t val)
{
    *reg = val;
    wifi_delay_us(12);   /* generous: > 4 SD clocks at 400 kHz */
}

/* ================================================================== *
 *  VideoCore property mailbox (channel 8) — used to assert WL_REG_ON *
 * ================================================================== */
#define MBOX_FULL   0x80000000u
#define MBOX_EMPTY  0x40000000u
#define MBOX_CHAN_PROP 8

static volatile uint32_t *const mbox = (volatile uint32_t *)WIFI_MBOX_BASE;
/* register word indices within the 0x3F00B880 block (see bcm2835_power.c) */
#define MBOX_READ   0
#define MBOX_STATUS 6
#define MBOX_WRITE  8

/* 16-byte-aligned property buffer (mailbox requires the low 4 bits clear). */
static volatile uint32_t mbox_buf[36] __attribute__((aligned(16)));

static uint32_t wifi_phys_to_bus(volatile void *p)
{
    /* Xinu runs with D-cache off; hand the GPU the uncached bus alias so it
     * reads our writes from RAM directly (same rule as screenInit.c). */
    return ((uint32_t)(unsigned long)p & 0x3FFFFFFFu) | 0xC0000000u;
}

/* Generic property-mailbox request.  `tag` is the property tag; `in`/`n_in`
 * are the request value words (copied into the value buffer); on return up to
 * `n_out` response value words are copied back into `out`.  vbuf_bytes is the
 * value buffer size (max of request/response value sizes).  Returns 0 on a
 * success response code. */
static int wifi_mbox_prop(uint32_t tag, const uint32_t *in, int n_in,
                          uint32_t *out, int n_out, int vbuf_bytes)
{
    volatile uint32_t *b = mbox_buf;
    uint32_t r;
    int i, hdr = 5;     /* words [0..4] are the header, value starts at [5] */

    b[0] = (hdr + (vbuf_bytes / 4) + 1) * 4;   /* total buffer size (bytes) */
    b[1] = 0;                                  /* request                   */
    b[2] = tag;
    b[3] = vbuf_bytes;                         /* value buffer size (bytes) */
    b[4] = 0;                                  /* req/resp code             */
    for (i = 0; i < n_in; i++)  b[hdr + i] = in[i];
    for (i = n_in; i < vbuf_bytes / 4; i++) b[hdr + i] = 0;
    b[hdr + vbuf_bytes / 4] = 0;               /* end tag                   */

    while (mbox[MBOX_STATUS] & MBOX_FULL) { }
    mbox[MBOX_WRITE] = (wifi_phys_to_bus(b) & ~0xFu) | MBOX_CHAN_PROP;
    do {
        while (mbox[MBOX_STATUS] & MBOX_EMPTY) { }
        r = mbox[MBOX_READ];
    } while ((r & 0xF) != MBOX_CHAN_PROP);

    for (i = 0; i < n_out; i++) out[i] = b[hdr + i];
    return (b[1] == 0x80000000u) ? 0 : -1;     /* response success code */
}

/* Set a VideoCore-expander GPIO state (SET_GPIO_STATE).  state 1 = on. */
static int wifi_fw_set_gpio(uint32_t gpio, uint32_t state)
{
    uint32_t in[2] = { gpio, state };
    return wifi_mbox_prop(0x00038041, in, 2, NULL, 0, 8);
}

/* Read back a VideoCore-expander GPIO state (GET_GPIO_STATE). */
static int wifi_fw_get_gpio(uint32_t gpio, uint32_t *state)
{
    uint32_t in[2] = { gpio, 0 }, out[2] = { 0, 0 };
    int rc = wifi_mbox_prop(0x00030041, in, 1, out, 2, 8);
    if (state) *state = out[1];
    return rc;
}

/* Enable/disable a firmware-managed clock (SET_CLOCK_STATE). */
static int wifi_fw_set_clock_state(uint32_t clk_id, uint32_t on)
{
    uint32_t in[2] = { clk_id, on & 1 }, out[2] = { 0, 0 };
    return wifi_mbox_prop(0x00038001, in, 2, out, 2, 8);
}

/* Query a firmware-managed clock's current rate in Hz (GET_CLOCK_RATE). */
static uint32_t wifi_fw_get_clock_rate(uint32_t clk_id)
{
    uint32_t in[1] = { clk_id }, out[2] = { 0, 0 };
    if (wifi_mbox_prop(0x00030002, in, 1, out, 2, 8) != 0)
        return 0;
    return out[1];   /* out[0] = clock id echoed, out[1] = rate in Hz */
}

#define RPI_FW_EMMC_CLK_ID  1

/* ================================================================== *
 *  WIFI_CLK — GPCLK2 on GPIO43 (the chip's reference clock).         *
 *  The BCM43455 needs a slow reference clock to come out of reset and *
 *  respond on SDIO; on the Pi 3 this is GPCLK2 routed to GPIO43       *
 *  (dts: gpclk2_gpio43).  Target ~32.768 kHz from the 19.2 MHz osc.   *
 * ================================================================== */
static void wifi_clk_setup(void)
{
    uint32_t sel = GPFSEL4;
    int alt43 = (sel >> 9) & 7;

    /* The firmware already routes GPIO43 -> ALT0 (GPCLK2) AND runs WIFI_CLK at
     * whatever frequency the chip expects (observed: GPIO43 fsel=4, GP2CTL
     * BUSY).  Reprogramming GPCLK2 ourselves risks feeding the chip the wrong
     * reference clock, so b5 TRUSTS the firmware: ensure GPIO43 is ALT0 and
     * leave the clock manager untouched. */
    if (alt43 != 4) {
        sel &= ~(7u << 9);
        sel |=  (4u << 9);
        GPFSEL4 = sel;
        wifi_log("[wifi] WIFI_CLK: set GPIO43 -> ALT0 (was fsel=%d)\r\n", alt43);
    }
    wifi_log("[wifi] WIFI_CLK: trusting firmware GPCLK2 (GP2CTL=0x%08x, GP2DIV=0x%08x)\r\n",
             CM_GP2CTL, CM_GP2DIV);
}

/* ================================================================== *
 *  GPIO 34-39 -> ALT3 (SDIO bus to the WiFi chip)                    *
 * ================================================================== */
static void wifi_gpio_sdio(void)
{
    uint32_t sel;
    int pin;

    /* ★ Disconnect the Arasan EMMC from the SD-card pins so it is free to
     * drive the WiFi SDIO bus.  The Arasan can mux to EITHER GPIO48-53 (SD
     * card) or GPIO34-39 (WiFi) via ALT3.  If the firmware left GPIO48-53 at
     * ALT3 (Arasan on the card — the Pi1/2 legacy default), the controller
     * stays on the card and our GPIO34-39 ALT3 never reaches the chip ->
     * CMD5 timeout.  plan9/Circle's Pi3 path sets GPIO48-53 to ALT0 (routes
     * the card to the separate SDHOST controller) precisely for this reason.
     * GPIO48-49 are in GPFSEL4 [24:29]; GPIO50-53 in GPFSEL5 [0:11].  ALT0=4.
     * (We do not use the SD card at runtime, so moving it is safe.) */
    sel = GPFSEL4;
    sel &= ~((7u << 24) | (7u << 27));
    sel |=  ((4u << 24) | (4u << 27));         /* GPIO48,49 -> ALT0 */
    GPFSEL4 = sel;
    sel = GPFSEL5;
    sel &= ~((7u << 0) | (7u << 3) | (7u << 6) | (7u << 9));
    sel |=  ((4u << 0) | (4u << 3) | (4u << 6) | (4u << 9)); /* GPIO50-53 -> ALT0 */
    GPFSEL5 = sel;

    sel = GPFSEL3;
    /* GPIO34-39 live in GPFSEL3; pin p uses bits [3*(p-30) .. +2].
     * ALT3 = 0b111. */
    for (pin = 34; pin <= 39; pin++) {
        int sh = 3 * (pin - 30);
        sel &= ~(7u << sh);
        sel |=  (7u << sh);   /* ALT3 == 7 */
    }
    GPFSEL3 = sel;

    /* SDIO line pulls (dts sdio_pins brcm,pull = <0 2 2 2 2 2>):
     *   GPIO34 = SDIO CLK   -> no pull
     *   GPIO35 = SDIO CMD   -> pull-up
     *   GPIO36-39 = DAT0-3  -> pull-up
     * Without the CMD/DAT pull-ups the lines float and CMD5 never sees a
     * response (the bring-up's first CMD5 timeout). */
    /* pull-UP (GPPUD=2) on GPIO35-39 = GPPUDCLK1 bits 3..7 */
    GPPUD = 2;
    wifi_udelay(300);
    GPPUDCLK1 = (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7);
    wifi_udelay(300);
    GPPUD = 0;
    GPPUDCLK1 = 0;
    /* no pull (GPPUD=0) on GPIO34 = GPPUDCLK1 bit 2 */
    GPPUD = 0;
    wifi_udelay(300);
    GPPUDCLK1 = (1u << 2);
    wifi_udelay(300);
    GPPUD = 0;
    GPPUDCLK1 = 0;
}

/* ================================================================== *
 *  SDHCI host controller bring-up                                    *
 * ================================================================== */
/* Compute the SDHCI 10-bit "divided clock" divisor that yields <= target Hz
 * from the given base clock: SDCLK = base / (2 * divisor). */
static uint32_t emmc_divisor(uint32_t base_hz, uint32_t target_hz)
{
    uint32_t d = 1;
    if (target_hz == 0) target_hz = 400000;
    while ((base_hz / (2 * d)) > target_hz && d < 1023)
        d++;
    return d;
}

static int emmc_reset_clock(uint32_t divisor)
{
    uint32_t c1;
    int t;

    /* full host-circuit reset */
    ew(&EMMC_CONTROL1, EMMC_CONTROL1 | C1_SRST_HC);
    for (t = 0; t < 100000; t++) {
        if (!(EMMC_CONTROL1 & C1_SRST_HC)) break;
        wifi_delay_us(100);
    }
    if (EMMC_CONTROL1 & C1_SRST_HC) {
        wifi_log("[wifi]   SRST_HC did not clear (controller dead?)\r\n");
        return -1;
    }

    /* SD bus power + voltage (standard SDHCI Power Control, byte at 0x29 =
     * CONTROL0 bits 8-11): bit8 = SD_BUS_POWER, bits9-11 = voltage select
     * (0b111 = 3.3V).  May be a no-op on the BCM EMMC (which powers the bus
     * via firmware/GPIO) but is harmless and required by spec on real SDHCI. */
    ew(&EMMC_CONTROL0, (EMMC_CONTROL0 & ~0xF00u) | (1u << 8) | (7u << 9));

    /* program the SD clock: 8-bit divided-clock mode (SDHCI v3 uses the
     * low 8 bits of the divisor in [15:8] plus the upper 2 bits in [7:6]). */
    c1 = EMMC_CONTROL1;
    c1 &= ~0xFFE0u;                       /* clear CLK_EN | DIV | GENSEL */
    c1 |= C1_CLK_INTLEN | C1_TOUNIT_MAX;
    c1 |= (divisor & 0xFF) << 8;
    c1 |= ((divisor >> 8) & 0x3) << 6;
    ew(&EMMC_CONTROL1, c1);

    for (t = 0; t < 100000; t++) {
        if (EMMC_CONTROL1 & C1_CLK_STABLE) break;
        wifi_delay_us(100);
    }
    if (!(EMMC_CONTROL1 & C1_CLK_STABLE)) {
        wifi_log("[wifi]   internal clock never stabilised\r\n");
        return -1;
    }
    ew(&EMMC_CONTROL1, EMMC_CONTROL1 | C1_CLK_EN);   /* gate clock to the chip */
    wifi_delay_us(2000);
    return 0;
}

static int emmc_host_init(void)
{
    uint32_t ver, base, div;

    /* The Arasan EMMC needs its base clock running before its registers are
     * meaningful.  Ask the firmware to power the EMMC clock on and tell us its
     * rate, then derive the SDHCI divisor for a ~400 kHz SDIO init clock.
     * (This was the likely cause of the first CMD5 timeout — no bus clock.) */
    wifi_fw_set_clock_state(RPI_FW_EMMC_CLK_ID, 1);
    base = wifi_fw_get_clock_rate(RPI_FW_EMMC_CLK_ID);
    wifi_log("[wifi] EMMC base clock = %u Hz\r\n", base);
    if (base == 0) {
        base = 250000000u;   /* fallback to a common Pi default */
        wifi_log("[wifi]   (base=0 from firmware, assuming %u)\r\n", base);
    }

    ver = EMMC_SLOTISR_VER;
    wifi_log("[wifi] SDHCI HC version=0x%02x (SLOTISR_VER=0x%08x)\r\n",
            (ver >> 16) & 0xFF, ver);

    div = emmc_divisor(base, 400000);
    wifi_log("[wifi] SDIO init clock divisor=%u (~%u Hz)\r\n",
            div, base / (2 * div));
    if (emmc_reset_clock(div) != 0)
        return -1;

    /* enable all normal-interrupt status bits (polled, not IRQ) */
    ew(&EMMC_IRPT_EN,   0xFFFFFFFFu);
    ew(&EMMC_IRPT_MASK, 0xFFFFFFFFu);
    ew(&EMMC_INTERRUPT, 0xFFFFFFFFu);      /* clear stale */
    wifi_log("[wifi] SDHCI init done: CONTROL0=0x%08x CONTROL1=0x%08x STATUS=0x%08x\r\n",
             EMMC_CONTROL0, EMMC_CONTROL1, EMMC_STATUS);
    return 0;
}

/* Issue one SDIO command.  resp_flags selects the response encoding; data!=0
 * means a 4-byte CMD53 read into *data.  Returns 0 on success. */
static int emmc_cmd(uint32_t cmd_idx, uint32_t arg, uint32_t resp_flags,
                    uint32_t *resp_out, int read_4byte, uint32_t *data)
{
    uint32_t cmd, intr, t;

    /* wait for the command line to be free */
    for (t = 0; t < 1000000; t++) {
        if (!(EMMC_STATUS & SR_CMD_INHIBIT)) break;
    }
    ew(&EMMC_INTERRUPT, 0xFFFFFFFFu);

    if (read_4byte) {
        ew(&EMMC_BLKSIZECNT, (1u << 16) | 4);   /* 1 block of 4 bytes */
    }

    ew(&EMMC_ARG1, arg);
    cmd = (cmd_idx << 24) | resp_flags;
    if (read_4byte)
        cmd |= CMD_ISDATA | TM_DAT_DIR_CH;
    ew(&EMMC_CMDTM, cmd);   /* erratum-safe: spaced write so the command issues */

    /* wait for CMD_DONE or error */
    for (t = 0; t < 1000000; t++) {
        intr = EMMC_INTERRUPT;
        if (intr & INT_ERR)      { wifi_log("[wifi]   cmd%d err intr=0x%08x\r\n", cmd_idx, intr); return -1; }
        if (intr & INT_CMD_DONE) { EMMC_INTERRUPT = INT_CMD_DONE; break; }
    }
    if (t >= 1000000) { wifi_log("[wifi]   cmd%d timeout (no CMD_DONE)\r\n", cmd_idx); return -1; }

    if (resp_out) *resp_out = EMMC_RESP0;

    if (read_4byte) {
        for (t = 0; t < 1000000; t++) {
            intr = EMMC_INTERRUPT;
            if (intr & INT_ERR)       { wifi_log("[wifi]   cmd%d data err 0x%08x\r\n", cmd_idx, intr); return -1; }
            if (intr & INT_READ_RDY)  { EMMC_INTERRUPT = INT_READ_RDY; break; }
        }
        if (t >= 1000000) { wifi_log("[wifi]   cmd%d no READ_RDY\r\n", cmd_idx); return -1; }
        if (data) *data = EMMC_DATA;
        for (t = 0; t < 1000000; t++) {
            if (EMMC_INTERRUPT & INT_DATA_DONE) { EMMC_INTERRUPT = INT_DATA_DONE; break; }
        }
    }
    return 0;
}

/* CMD52: single-byte direct register access.  write!=0 -> write `val`.
 * Returns the read/echoed byte (0..255) or -1 on error. */
static int sdio_cmd52(int func, uint32_t addr, int write, uint32_t val)
{
    uint32_t arg, resp;
    arg = ((uint32_t)(write & 1) << 31) |
          ((uint32_t)(func & 7) << 28)  |
          (write ? (1u << 27) : 0)      |   /* RAW: read-after-write */
          ((addr & 0x1FFFF) << 9)       |
          (val & 0xFF);
    if (emmc_cmd(SD_CMD52_IO_RW_DIRECT, arg, CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN,
                 &resp, 0, NULL) != 0)
        return -1;
    return (int)(resp & 0xFF);
}

/* CMD53 4-byte read at backplane (function-1) offset `f1off`. */
static int sdio_cmd53_read32(uint32_t f1off, uint32_t *out)
{
    uint32_t arg;
    /* byte mode, function 1, increment, count=4 */
    arg = (0u << 31) |              /* read                 */
          (1u << 28) |              /* function 1           */
          (0u << 27) |              /* byte (not block) mode*/
          (1u << 26) |              /* increment address    */
          ((f1off & 0x1FFFF) << 9) |
          (4u & 0x1FF);             /* 4 bytes              */
    return emmc_cmd(SD_CMD53_IO_RW_EXT, arg,
                    CMD_RSPNS_48 | CMD_CRCCHK_EN | CMD_IXCHK_EN, NULL, 1, out);
}

/* Point the F1 backplane window at `addr` (brcmf_sdiod_set_backplane_window). */
static int sdio_set_window(uint32_t addr)
{
    uint32_t bar0 = addr & SBSDIO_SBWINDOW_MASK;
    uint32_t v = bar0 >> 8;
    int i;
    for (i = 0; i < 3; i++, v >>= 8) {
        if (sdio_cmd52(1, SBSDIO_FUNC1_SBADDRLOW + i, 1, v & 0xFF) < 0)
            return -1;
    }
    return 0;
}

/* Read a 32-bit backplane register at absolute address `addr`. */
static int wifi_backplane_read32(uint32_t addr, uint32_t *out)
{
    uint32_t off;
    if (sdio_set_window(addr) != 0)
        return -1;
    off = (addr & SBSDIO_SB_OFT_ADDR_MASK) | SBSDIO_SB_ACCESS_2_4B;
    return sdio_cmd53_read32(off, out);
}

/* ================================================================== *
 *  Stage 3 — backplane bulk access + chip core/RAM enumeration       *
 *  (port of plan9 ether4330.c sbmem/sbrw/corescan/ramscan)           *
 * ================================================================== */
#define SB_WSIZE        0x8000u          /* backplane window size      */
#define SB_32BIT        0x8000u          /* 32-bit access flag in off  */
#define SB_ENUMBASE     0x18000000u
#define CORE_ARMCR4     0x83E
#define CORE_ARMCM3     0x82A
#define CORE_ARM7       0x825
#define CORE_CHIPCOMMON 0x800
#define CORE_SOCRAM     0x80E
#define CORE_SDIODEV    0x829
#define CORE_D11        0x812
#define REG_IOCTRL      0x408
#define REG_RESETCTRL   0x800
#define CR4_CAP         0x04
#define CR4_BANKIDX     0x40
#define CR4_BANKINFO    0x44
#define CR4_CPUHALT     0x20

/* discovered chip layout (filled by wifi_corescan / wifi_ramscan) */
static uint32_t chip_chipcommon, chip_armctl, chip_armregs, chip_armcore;
static uint32_t chip_d11ctl, chip_socramctl, chip_socramregs, chip_sdregs;
static uint32_t chip_rambase, chip_socramsize;

/* General CMD53 (IO_RW_EXTENDED) with PIO data transfer.  `off` is the F1
 * register offset (already including SB_32BIT when addressing the backplane).
 * Transfers up to 511*bsize bytes (bsize=64 for Fn1).  Returns 0 on success. */
static int wifi_cmd53_pio(int write, int fn, uint32_t off, uint8_t *buf,
                          int len, int incr)
{
    uint32_t arg, intr, t, total, w, words;
    uint32_t bsize = (fn == 2) ? 512u : 64u;
    int blkmode, blkcnt;

    if (len <= 0) return 0;
    if ((uint32_t)len > bsize) { blkmode = 1; blkcnt = len / bsize; total = blkcnt * bsize; }
    else                       { blkmode = 0; blkcnt = len;         total = len; }

    for (t = 0; t < 1000000; t++) if (!(EMMC_STATUS & SR_CMD_INHIBIT)) break;
    ew(&EMMC_INTERRUPT, 0xFFFFFFFFu);
    if (blkmode) ew(&EMMC_BLKSIZECNT, ((uint32_t)blkcnt << 16) | bsize);
    else         ew(&EMMC_BLKSIZECNT, (1u << 16) | (uint32_t)total);

    arg = ((uint32_t)(write & 1) << 31) | ((uint32_t)(fn & 7) << 28) |
          ((uint32_t)(blkmode & 1) << 27) | ((uint32_t)(incr & 1) << 26) |
          ((off & 0x1FFFF) << 9) |
          ((blkmode ? (uint32_t)blkcnt : (uint32_t)total) & 0x1FF);
    ew(&EMMC_ARG1, arg);
    {
        uint32_t cmd = (SD_CMD53_IO_RW_EXT << 24) | CMD_RSPNS_48 |
                       CMD_CRCCHK_EN | CMD_IXCHK_EN | CMD_ISDATA;
        if (!write)  cmd |= TM_DAT_DIR_CH;       /* card -> host */
        if (blkmode) cmd |= (1u << 5) | (1u << 1); /* multiblock + blkcnt-en */
        ew(&EMMC_CMDTM, cmd);
    }
    for (t = 0; t < 1000000; t++) {
        intr = EMMC_INTERRUPT;
        if (intr & INT_ERR)      { wifi_log("[wifi]   cmd53 err 0x%08x\r\n", intr); return -1; }
        if (intr & INT_CMD_DONE) { ew(&EMMC_INTERRUPT, INT_CMD_DONE); break; }
    }
    if (t >= 1000000) return -1;   /* no CMD_DONE (e.g. empty FIFO poll) — quiet */

    /* PIO: the controller raises READ_RDY/WRITE_RDY before each block-sized
     * buffer; transfer it word-by-word via the DATA register. */
    words = (total + 3) / 4;
    {
        uint32_t done = 0;
        while (done < words) {
            uint32_t flag = write ? INT_WRITE_RDY : INT_READ_RDY;
            uint32_t chunk = (total > bsize && (words - done) > (bsize / 4))
                             ? (bsize / 4) : (words - done);
            for (t = 0; t < 1000000; t++) {
                intr = EMMC_INTERRUPT;
                if (intr & INT_ERR)  { wifi_log("[wifi]   cmd53 data err 0x%08x\r\n", intr); return -1; }
                if (intr & flag)     { EMMC_INTERRUPT = flag; break; }
            }
            if (t >= 1000000) return -1;   /* no data ready (empty FIFO poll) — quiet */
            for (w = 0; w < chunk; w++) {
                uint32_t idx = (done + w) * 4;
                if (write) {
                    uint32_t v = (uint32_t)buf[idx] | ((uint32_t)buf[idx+1] << 8) |
                                 ((uint32_t)buf[idx+2] << 16) | ((uint32_t)buf[idx+3] << 24);
                    EMMC_DATA = v;
                } else {
                    uint32_t v = EMMC_DATA;
                    buf[idx] = v; buf[idx+1] = v >> 8; buf[idx+2] = v >> 16; buf[idx+3] = v >> 24;
                }
            }
            done += chunk;
        }
    }
    for (t = 0; t < 1000000; t++)
        if (EMMC_INTERRUPT & INT_DATA_DONE) { EMMC_INTERRUPT = INT_DATA_DONE; break; }
    return 0;
}

/* Bulk backplane read/write across 0x8000 windows (plan9 sbmem+sbrw).
 * Each CMD53 is capped at SB_XFER (2048) so it stays well under the 511-block
 * limit (32 blocks of 64) and within a single 0x8000 window. */
#define SB_XFER 2048
static int wifi_sbmem(int write, uint8_t *buf, int len, uint32_t addr)
{
    while (len > 0) {
        uint32_t woff;
        int wrem = (int)(SB_WSIZE - (addr & (SB_WSIZE - 1)));   /* to window end */
        int chunk = len;
        if (chunk > SB_XFER) chunk = SB_XFER;
        if (chunk > wrem)    chunk = wrem;
        if (sdio_set_window(addr) != 0) return -1;
        woff = (addr & (SB_WSIZE - 1)) | SB_32BIT;
        if (wifi_cmd53_pio(write, 1, woff, buf, (chunk + 3) & ~3, 1) != 0) return -1;
        addr += chunk; buf += chunk; len -= chunk;
    }
    return 0;
}

/* 32-bit backplane write at absolute address. */
static int wifi_backplane_write32(uint32_t addr, uint32_t val)
{
    uint8_t b[4];
    b[0] = val; b[1] = val >> 8; b[2] = val >> 16; b[3] = val >> 24;
    return wifi_sbmem(1, b, 4, addr);
}

/* Scan the SonicsSiliconBackplane EROM to locate the chip's cores. */
static int wifi_corescan(void)
{
    static uint8_t erom[512];
    uint32_t eromptr, i;
    int coreid = 0;

    /* EROM pointer lives at chipcommon enum base + 63*4 */
    if (wifi_backplane_read32(SB_ENUMBASE + 63 * 4, &eromptr) != 0) return -1;
    chip_chipcommon = chip_armctl = chip_armregs = chip_armcore = 0;
    chip_d11ctl = chip_socramctl = chip_socramregs = chip_sdregs = 0;
    if (wifi_sbmem(0, erom, sizeof(erom), eromptr) != 0) return -1;

    for (i = 0; i < sizeof(erom); i += 4) {
        uint32_t addr;
        switch (erom[i] & 0xF) {
        case 0xF:                                  /* end */
            return 0;
        case 0x1:                                  /* core info */
            if ((erom[i + 4] & 0xF) != 0x1) break;
            coreid = (erom[i + 1] | (erom[i + 2] << 8)) & 0xFFF;
            i += 4;
            break;
        case 0x5:                                  /* address descriptor */
            addr = (erom[i + 1] << 8) | (erom[i + 2] << 16) | (erom[i + 3] << 24);
            addr &= ~0xFFFu;
            switch (coreid) {
            case CORE_CHIPCOMMON:
                if ((erom[i] & 0xC0) == 0) chip_chipcommon = addr;
                break;
            case CORE_ARMCM3: case CORE_ARM7: case CORE_ARMCR4:
                chip_armcore = coreid;
                if (erom[i] & 0xC0) { if (!chip_armctl) chip_armctl = addr; }
                else                { if (!chip_armregs) chip_armregs = addr; }
                break;
            case CORE_SOCRAM:
                if (erom[i] & 0xC0) chip_socramctl = addr;
                else if (!chip_socramregs) chip_socramregs = addr;
                break;
            case CORE_SDIODEV:
                if ((erom[i] & 0xC0) == 0) chip_sdregs = addr;
                break;
            case CORE_D11:
                if (erom[i] & 0xC0) chip_d11ctl = addr;
                break;
            }
            break;
        }
    }
    return 0;
}

/* Determine on-chip RAM size (CR4 path for the BCM43455). */
static int wifi_ramscan(void)
{
    uint32_t r, n, size = 0;
    int banks, i;

    if (chip_armcore != CORE_ARMCR4) {
        wifi_log("[wifi]   ramscan: non-CR4 core 0x%x not handled here\r\n", chip_armcore);
        return -1;
    }
    r = chip_armregs;
    if (wifi_backplane_read32(r + CR4_CAP, &n) != 0) return -1;
    banks = ((n >> 4) & 0xF) + (n & 0xF);
    for (i = 0; i < banks; i++) {
        if (wifi_backplane_write32(r + CR4_BANKIDX, i) != 0) return -1;
        if (wifi_backplane_read32(r + CR4_BANKINFO, &n) != 0) return -1;
        size += 8192 * ((n & 0x3F) + 1);
    }
    chip_socramsize = size;
    chip_rambase = 0x198000;       /* BCM43455 TCM base */
    return 0;
}

/* ================================================================== *
 *  Stage 3b — firmware download + ARM core start                     *
 *  (port of plan9 ether4330.c sbinit/fwload/sbenable)                *
 * ================================================================== */
/* Clkcsr (F1 byte register 0x1000e) bits */
#define REG_CLKCSR   0x1000E
#define CLK_FORCEALP 0x01
#define CLK_FORCEHT  0x02
#define CLK_REQALP   0x08
#define CLK_REQHT    0x10
#define CLK_NOHWREQ  0x20
#define CLK_ALPAVAIL 0x40
#define CLK_HTAVAIL  0x80
#define REG_PULLUPS  0x1000F
/* sdio core (0x829) registers */
#define SDR_INTSTATUS 0x20
#define SDR_INTMASK   0x24
#define SDR_MBOXDATA  0x48

extern uint8_t wifi_fw_bin[],   wifi_fw_bin_end[];
extern uint8_t wifi_nvram_txt[], wifi_nvram_txt_end[];
extern uint8_t wifi_clm_blob[],  wifi_clm_blob_end[];

/* single-byte F1 register access (plan9 cfgr/cfgw) */
static int  cfgr(uint32_t off) { return sdio_cmd52(1, off, 0, 0); }
static void cfgw(uint32_t off, int v) { sdio_cmd52(1, off, 1, v); }

/* core enable/reset via the SonicsSiliconBackplane wrapper (Ioctrl/Resetctrl) */
static void sb_disable(uint32_t regs, int pre, int ioctl)
{
    uint32_t v;
    if (wifi_backplane_read32(regs + REG_RESETCTRL, &v) == 0 && (v & 1)) {
        wifi_backplane_write32(regs + REG_IOCTRL, 3 | ioctl);
        wifi_backplane_read32(regs + REG_IOCTRL, &v);
        return;
    }
    wifi_backplane_write32(regs + REG_IOCTRL, 3 | pre);
    wifi_backplane_read32(regs + REG_IOCTRL, &v);
    wifi_backplane_write32(regs + REG_RESETCTRL, 1);
    wifi_delay_us(10);
    do { wifi_backplane_read32(regs + REG_RESETCTRL, &v); } while (!(v & 1));
    wifi_backplane_write32(regs + REG_IOCTRL, 3 | ioctl);
    wifi_backplane_read32(regs + REG_IOCTRL, &v);
}

static void sb_reset(uint32_t regs, int pre, int ioctl)
{
    uint32_t v;
    sb_disable(regs, pre, ioctl);
    do {
        wifi_backplane_read32(regs + REG_RESETCTRL, &v);
        if (v & 1) { wifi_backplane_write32(regs + REG_RESETCTRL, 0); wifi_delay_us(40); }
    } while (v & 1);
    wifi_backplane_write32(regs + REG_IOCTRL, 1 | ioctl);
    wifi_backplane_read32(regs + REG_IOCTRL, &v);
}

/* condense nvram text to a 'var=value\0...\0' list (plan9 condense). */
static int wifi_condense(uint8_t *buf, int n)
{
    uint8_t *p, *ep = buf + n, *op = buf, *lp = buf;
    int c, skipping = 0;
    for (p = buf; p < ep; p++) {
        c = *p;
        if (c == '#') skipping = 1;
        else if (c == '\0' || c == '\n') {
            skipping = 0;
            if (op != lp) { *op++ = '\0'; lp = op; }
        } else if (c == '\r') { /* drop */ }
        else if (!skipping) *op++ = (uint8_t)c;
    }
    if (!skipping && op != lp) *op++ = '\0';
    *op++ = '\0';
    for (n = op - buf; n & 3; n++) *op++ = '\0';
    return n;
}

/* Download firmware + NVRAM into chip RAM and start the ARM CR4 core. */
static int wifi_fwload(void)
{
    static uint8_t nvbuf[3072];
    uint8_t trailer[4];
    uint32_t resetvec, t, fwlen, nvlen, off;
    int i;

    fwlen = (uint32_t)(wifi_fw_bin_end - wifi_fw_bin);
    nvlen = (uint32_t)(wifi_nvram_txt_end - wifi_nvram_txt);
    wifi_log("[wifi] fwload: fw=%u B nvram=%u B -> rambase 0x%x (ramsize %u)\r\n",
             fwlen, nvlen, chip_rambase, chip_socramsize);

    /* request ALP clock */
    cfgw(REG_CLKCSR, CLK_REQALP);
    for (i = 0; i < 2000 && !(cfgr(REG_CLKCSR) & CLK_ALPAVAIL); i++) wifi_delay_us(100);

    /* clear the nvram trailer word */
    trailer[0] = trailer[1] = trailer[2] = trailer[3] = 0;
    if (wifi_sbmem(1, trailer, 4, chip_rambase + chip_socramsize - 4) != 0) return -1;

    /* write the firmware to RAM base; first 4 bytes are the reset vector */
    resetvec = (uint32_t)wifi_fw_bin[0] | ((uint32_t)wifi_fw_bin[1] << 8) |
               ((uint32_t)wifi_fw_bin[2] << 16) | ((uint32_t)wifi_fw_bin[3] << 24);
    if (wifi_sbmem(1, wifi_fw_bin, (int)fwlen, chip_rambase) != 0) {
        wifi_log("[wifi] fwload: firmware write failed\r\n");
        return -1;
    }
    wifi_log("[wifi] fwload: firmware written\r\n");

    /* condense nvram, write near the top of RAM, then the length token */
    if (nvlen > sizeof(nvbuf)) nvlen = sizeof(nvbuf);
    for (i = 0; i < (int)nvlen; i++) nvbuf[i] = wifi_nvram_txt[i];
    nvlen = wifi_condense(nvbuf, (int)nvlen);
    off = chip_socramsize - nvlen - 4;
    if (wifi_sbmem(1, nvbuf, (int)nvlen, chip_rambase + off) != 0) return -1;
    t = nvlen / 4;
    t = (t & 0xFFFF) | (~t << 16);
    trailer[0] = t; trailer[1] = t >> 8; trailer[2] = t >> 16; trailer[3] = t >> 24;
    if (wifi_sbmem(1, trailer, 4, chip_rambase + chip_socramsize - 4) != 0) return -1;
    wifi_log("[wifi] fwload: nvram %u B + trailer 0x%x written\r\n", nvlen, t);

    /* start the CR4: clear sdio intstatus, plant reset vector at 0, run */
    wifi_backplane_write32(chip_sdregs + SDR_INTSTATUS, 0xFFFFFFFFu);
    if (resetvec != 0) {
        trailer[0] = resetvec; trailer[1] = resetvec >> 8;
        trailer[2] = resetvec >> 16; trailer[3] = resetvec >> 24;
        wifi_sbmem(1, trailer, 4, 0);
    }
    sb_reset(chip_armctl, CR4_CPUHALT, 0);
    wifi_log("[wifi] fwload: CR4 released (resetvec=0x%x) — firmware running\r\n", resetvec);
    return 0;
}

/* After fw is running: bring up the HT clock and enable WLAN data (Fn2). */
static int wifi_sbenable(void)
{
    int i, fn2;
    cfgw(REG_CLKCSR, 0);
    wifi_delay_us(1000);
    cfgw(REG_CLKCSR, CLK_REQHT);
    for (i = 0; i < 500 && !(cfgr(REG_CLKCSR) & CLK_HTAVAIL); i++) wifi_delay_us(2000);
    if (!(cfgr(REG_CLKCSR) & CLK_HTAVAIL)) {
        wifi_log("[wifi] sbenable: HT clock never came up (csr=0x%02x)\r\n", cfgr(REG_CLKCSR));
        return -1;
    }
    cfgw(REG_CLKCSR, cfgr(REG_CLKCSR) | CLK_FORCEHT);
    wifi_delay_us(10000);
    wifi_log("[wifi] sbenable: HT clock up (csr=0x%02x)\r\n", cfgr(REG_CLKCSR));

    /* sdio core: protocol version + interrupt mask */
    wifi_backplane_write32(chip_sdregs + SDR_MBOXDATA, 4 << 16);
    wifi_backplane_write32(chip_sdregs + SDR_INTMASK, (1u<<7)|(1u<<6)|(1u<<5));

    /* enable WLAN data function 2 */
    sdio_cmd52(0, SDIO_CCCR_IOEx, 1, (1 << 1) | (1 << 2));   /* enable F1+F2 */
    for (i = 0, fn2 = 0; i < 50; i++) {
        fn2 = sdio_cmd52(0, SDIO_CCCR_IORx, 0, 0);
        if (fn2 >= 0 && (fn2 & (1 << 2))) break;
        wifi_delay_us(2000);
    }
    if (!(fn2 & (1 << 2))) {
        wifi_log("[wifi] sbenable: Fn2 not ready (IOR=0x%02x)\r\n", fn2);
        return -1;
    }
    sdio_cmd52(0, SDIO_CCCR_INT_ENABLE, 1, (1<<1)|(1<<2)|1);
    wifi_log("[wifi] sbenable: WLAN function 2 enabled+ready\r\n");
    return 0;
}

/* ================================================================== *
 *  Stage 4 — SDPCM control channel (ioctl / iovar over Fn2)          *
 *  (synchronous, polled port of plan9 ether4330.c wlcmd/packetrw)    *
 * ================================================================== */
#define SDPCM_HDR   12          /* len[2] lenck[2] seq chan nextlen doff fc win ver pad */
#define BCDC_HDR    16          /* cmd[4] len[4] flags[2] id[2] status[4] */
#define SD_INT_FRAME  (1u << 6)
#define SD_INT_MBOX   (1u << 7)
#define SDR_SBMBOX    0x40
#define SDR_HOSTMBOX  0x4c
#define WLC_GET_VAR   262
#define WLC_SET_VAR   263

static uint8_t  wl_txseq;
/* SDPCM tx flow control: the fw advertises a credit window (max_seq, byte 9)
 * and a per-channel flow-control mask (byte 8) in EVERY frame it sends.  We
 * must not transmit when txseq == txwindow or when data-channel FC is set,
 * else the fw silently drops the frame.  Updated from every RX SDPCM header. */
static uint8_t  wl_txwindow = 1;
static uint8_t  wl_fcmask = 0;
/* Serialize SDIO FIFO access between the net-service thread and client ops
 * (ping/NTP), so a frame read+reply never interleaves with a client's send/
 * recv on the same controller.  Created in wifi_bringup(). */
static semaphore wl_io_sem = 0;
static int       wl_io_ready = 0;
static uint16_t wl_reqid;

/* Fn2 bulk transfer to/from the fixed FIFO address (incr=0).  Split into
 * <=512-byte byte-mode CMD53s: block-mode truncates non-block-aligned lengths
 * and SDPCM frames are arbitrary lengths (e.g. the ~1.4 KB clmload frame). */
static int wifi_packetrw(int write, uint8_t *buf, int len)
{
    while (len > 0) {
        int chunk = (len > 512) ? 512 : len;
        if (wifi_cmd53_pio(write, 2, 0, buf, (chunk + 3) & ~3, 0) != 0) return -1;
        buf += chunk; len -= chunk;
    }
    return 0;
}

/* Issue a firmware ioctl over the SDPCM control channel and return its reply.
 * write!=0 sends `data` (dlen) as the ioctl payload; otherwise `data` is the
 * request (e.g. an iovar name) and the reply is copied into res/rlen.
 * Returns 0 on success. */
static int wifi_wlcmd(int write, int op, const uint8_t *data, int dlen,
                      uint8_t *res, int rlen)
{
    static uint8_t pkt[2048];
    int tlen = write ? (dlen + rlen) : (dlen > rlen ? dlen : rlen);
    int len = SDPCM_HDR + BCDC_HDR + tlen;
    int i, tries;

    if (len > (int)sizeof(pkt)) { wifi_log("[wifi] wlcmd: pkt too big %d\r\n", len); return -1; }
    for (i = 0; i < len; i++) pkt[i] = 0;

    /* SDPCM header (control channel = 0) */
    pkt[0] = len & 0xFF; pkt[1] = (len >> 8) & 0xFF;
    pkt[2] = ~len & 0xFF; pkt[3] = (~len >> 8) & 0xFF;
    pkt[4] = wl_txseq;          /* seq */
    pkt[5] = 0;                 /* chanflg: channel 0 = control */
    pkt[7] = SDPCM_HDR;         /* doffset */

    /* BCDC command header */
    {
        uint8_t *q = pkt + SDPCM_HDR;
        q[0] = op; q[1] = op >> 8; q[2] = op >> 16; q[3] = op >> 24;
        q[4] = tlen; q[5] = tlen >> 8; q[6] = tlen >> 16; q[7] = tlen >> 24;
        q[8] = write ? 2 : 0;   /* flags */
        wl_reqid++;
        q[10] = wl_reqid; q[11] = wl_reqid >> 8;
    }
    if (dlen > 0) for (i = 0; i < dlen; i++) pkt[SDPCM_HDR + BCDC_HDR + i] = data[i];

    if (wifi_packetrw(1, pkt, len) != 0) { wifi_log("[wifi] wlcmd: tx failed\r\n"); return -1; }
    wl_txseq++;

    /* wait for the control-channel response by polling the Fn2 FIFO directly:
     * a read of the 12-byte SDPCM header returns len==0 when no frame is
     * queued (plan9 wlreadpkt), so we don't depend on the interrupt status. */
    for (tries = 0; tries < 400; tries++) {
        uint32_t ints;
        int plen, lenck, chan, doff;

        /* best-effort: ack any sdio-core interrupt + firmware mailbox */
        if (wifi_backplane_read32(chip_sdregs + SDR_INTSTATUS, &ints) == 0 && ints) {
            wifi_backplane_write32(chip_sdregs + SDR_INTSTATUS, ints);
            if (ints & SD_INT_MBOX) {
                uint32_t mb;
                wifi_backplane_read32(chip_sdregs + SDR_HOSTMBOX, &mb);
                wifi_backplane_write32(chip_sdregs + SDR_SBMBOX, 2);   /* ack */
            }
        }

        /* read the SDPCM header from the FIFO; len==0 => nothing queued yet */
        if (wifi_packetrw(0, pkt, SDPCM_HDR) != 0) { wifi_delay_us(2000); continue; }
        plen = pkt[0] | (pkt[1] << 8);
        lenck = pkt[2] | (pkt[3] << 8);
        if (tries < 3)
            wifi_log("[wifi] wlcmd rx try%d: len=0x%x lenck=0x%x ints=0x%x\r\n",
                     tries, plen, lenck, ints);
        if (plen == 0) { wifi_delay_us(2000); continue; }
        if (lenck != ((plen ^ 0xFFFF) & 0xFFFF) || plen < SDPCM_HDR || plen > (int)sizeof(pkt)) {
            wifi_log("[wifi] wlcmd: bad frame len=0x%x lenck=0x%x\r\n", plen, lenck);
            wifi_delay_us(2000);
            continue;
        }
        chan = pkt[5] & 0xF;
        doff = pkt[7];
        wl_fcmask = pkt[8]; wl_txwindow = pkt[9];   /* tx credit window + flow ctl */
        if (plen > SDPCM_HDR)
            if (wifi_packetrw(0, pkt + SDPCM_HDR, plen - SDPCM_HDR) != 0) return -1;
        if (chan != 0) { wifi_log("[wifi] wlcmd: drained chan %d frame\r\n", chan); continue; }

        /* BCDC reply at doff */
        {
            uint8_t *q = pkt + doff;
            uint32_t st = q[12] | (q[13] << 8) | (q[14] << 16) | (q[15] << 24);
            if (st != 0) { wifi_log("[wifi] wlcmd: op %d status %u\r\n", op, st); return -1; }
            if (!write && rlen > 0) {
                uint8_t *d = q + BCDC_HDR;
                for (i = 0; i < rlen; i++) res[i] = d[i];
            }
            return 0;
        }
    }
    wifi_log("[wifi] wlcmd: op %d timed out (no control response)\r\n", op);
    return -1;
}

/* iovar GET (e.g. "ver"). */
static int wifi_get_iovar(const char *name, uint8_t *val, int len)
{
    int n = 0;
    while (name[n]) n++;
    n++;                         /* include NUL */
    return wifi_wlcmd(0, WLC_GET_VAR, (const uint8_t *)name, n, val, len);
}

/* WLC ioctl taking a single 4-byte int (e.g. PASSIVE_SCAN). */
static int wifi_cmd_int(int op, uint32_t v)
{
    uint8_t b[4];
    b[0] = v; b[1] = v >> 8; b[2] = v >> 16; b[3] = v >> 24;
    return wifi_wlcmd(1, op, b, 4, NULL, 0);
}

/* iovar SET: name\0 + value as one payload. */
static int wifi_set_iovar(const char *name, const uint8_t *val, int vlen)
{
    static uint8_t b[1600];
    int n = 0, i;
    while (name[n]) { b[n] = name[n]; n++; }
    b[n++] = '\0';
    for (i = 0; i < vlen && (n + i) < (int)sizeof(b); i++) b[n + i] = val[i];
    return wifi_wlcmd(1, WLC_SET_VAR, b, n + vlen, NULL, 0);
}

/* iovar SET with a 4-byte int value. */
static int wifi_set_iovar_int(const char *name, uint32_t v)
{
    uint8_t b[4];
    b[0] = v; b[1] = v >> 8; b[2] = v >> 16; b[3] = v >> 24;
    return wifi_set_iovar(name, b, 4);
}

/* Upload the regulatory (CLM) blob via the "clmload" iovar.  Without valid
 * regulatory data the firmware refuses "country" (-2) and the radio won't
 * come up for scanning (-4).  Packet = flag[2] type[2] len[4] crc[4] data
 * (plan9 reguload); chunked at 1400 bytes. */
#define CLM_CHUNK   1400
#define CLM_FLAG    0x1000          /* DL_BEGIN-ish: clm data flag */
#define CLM_FIRST   0x0002
#define CLM_LAST    0x0004
static int wifi_clmload(void)
{
    static uint8_t buf[12 + CLM_CHUNK];
    uint32_t total = (uint32_t)(wifi_clm_blob_end - wifi_clm_blob);
    uint32_t off = 0;
    int flag = CLM_FLAG | CLM_FIRST;

    wifi_log("[wifi] clmload: %u bytes regulatory\r\n", total);
    while (off < total || flag == (CLM_FLAG | CLM_FIRST)) {
        uint32_t n = total - off;
        int i, pad;
        if (n > CLM_CHUNK) n = CLM_CHUNK;
        else flag |= CLM_LAST;
        for (i = 0; i < (int)n; i++) buf[12 + i] = wifi_clm_blob[off + i];
        pad = 0;
        if (flag & CLM_LAST) while ((n + pad) & 7) buf[12 + n + pad++] = 0;
        buf[0] = flag; buf[1] = flag >> 8;        /* flag */
        buf[2] = 2; buf[3] = 0;                   /* type = 2 */
        buf[4] = n; buf[5] = n>>8; buf[6] = n>>16; buf[7] = n>>24;   /* len */
        buf[8] = buf[9] = buf[10] = buf[11] = 0;  /* crc (unused) */
        if (wifi_set_iovar("clmload", buf, 12 + n + pad) != 0) {
            wifi_log("[wifi] clmload: chunk at %u failed\r\n", off);
            return -1;
        }
        off += n;
        flag &= ~CLM_FIRST;
        if (off >= total) break;
    }
    wifi_log("[wifi] clmload: done\r\n");
    return 0;
}

/* ================================================================== *
 *  Stage 5 — scan for access points (escan + event parsing)         *
 * ================================================================== */
#define WLC_PASSIVE_SCAN 49
#define WLC_E_ESCAN_RESULT 69
#define WLC_E_SCAN_COMPLETE 26

/* escan_params template (little-endian; plan9 ether4330.c wlscanstart):
 * version=1 action=START(1) sync_id=0x1234, ssid(any), bssid=ff..ff,
 * bss_type=any(2), defaults, 14 2.4GHz channels, 1 (any) ssid. */
static const uint8_t escan_params[] = {
    1,0,0,0,            /* version */
    1,0,                /* action = WL_SCAN_ACTION_START */
    0x34,0x12,          /* sync_id */
    0,0,0,0,            /* ssid len */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, /* ssid[32] */
    0xff,0xff,0xff,0xff,0xff,0xff,   /* bssid = broadcast (any) */
    2,                  /* bss_type = any */
    0,                  /* scan_type = active */
    0xff,0xff,0xff,0xff, /* nprobes (default) */
    0xff,0xff,0xff,0xff, /* active_time */
    0xff,0xff,0xff,0xff, /* passive_time */
    0xff,0xff,0xff,0xff, /* home_time */
    14,0,               /* nchannels */
    0,0,                /* nssids = 0 (broadcast scan; no directed-SSID entries
                           — avoids the per-ssid 36-byte buffer requirement
                           that made fw 7.45.265 return BCME_BUFTOOSHORT -14) */
    0x01,0x2b,0x02,0x2b,0x03,0x2b,0x04,0x2b,0x05,0x2e,0x06,0x2e,0x07,0x2e,
    0x08,0x2b,0x09,0x2b,0x0a,0x2b,0x0b,0x2b,0x0c,0x2b,0x0d,0x2b,0x0e,0x2b,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  /* ssid */
};

/* Read one SDPCM frame from the Fn2 FIFO into `buf` (cap bytes).
 * Returns the frame length (>=12), 0 if nothing queued, -1 on error.
 * On success *chan = channel, *doff = data offset. */
static int wifi_read_frame(uint8_t *buf, int cap, int *chan, int *doff)
{
    int plen, lenck;
    if (wifi_packetrw(0, buf, SDPCM_HDR) != 0) return -1;
    plen = buf[0] | (buf[1] << 8);
    lenck = buf[2] | (buf[3] << 8);
    if (plen == 0) return 0;
    if (lenck != ((plen ^ 0xFFFF) & 0xFFFF) || plen < SDPCM_HDR || plen > cap)
        return 0;                              /* junk; treat as empty */
    *chan = buf[5] & 0xF;
    *doff = buf[7];
    wl_fcmask = buf[8]; wl_txwindow = buf[9];   /* tx credit window + flow ctl */
    if (plen > SDPCM_HDR)
        if (wifi_packetrw(0, buf + SDPCM_HDR, plen - SDPCM_HDR) != 0) return -1;
    return plen;
}

/* Run a scan and append the discovered APs to the trace.  out_count gets the
 * number of unique APs found.  Returns 0 on success. */
/* One-time radio bring-up needed before scan/join: enable fw events, load
 * regulatory (CLM), set country + power management.  Idempotent. */
static int wifi_radio_done = 0;
static void wifi_radio_up(void)
{
    int i;
    if (wifi_radio_done) return;
    {
        uint8_t em[16];                           /* enable all fw events */
        for (i = 0; i < 16; i++) em[i] = 0xFF;
        if (wifi_set_iovar("event_msgs", em, 16) != 0)
            wifi_log("[wifi] radio: event_msgs failed (continuing)\r\n");
        else
            wifi_log("[wifi] radio: event_msgs enabled\r\n");
    }
    wifi_cmd_int(0x56, 0);                         /* WLC_SET_PM = 0 */
    if (wifi_clmload() != 0)                       /* regulatory firmware */
        wifi_log("[wifi] radio: clmload failed (continuing)\r\n");
    {
        uint8_t cc[12];
        for (i = 0; i < 12; i++) cc[i] = 0;
        cc[0] = 'U'; cc[1] = 'S';
        cc[4] = cc[5] = cc[6] = cc[7] = 0xFF;      /* revision = -1 */
        cc[8] = 'U'; cc[9] = 'S';
        if (wifi_set_iovar("country", cc, 12) != 0)
            wifi_log("[wifi] radio: set country failed (continuing)\r\n");
    }
    if (wifi_set_iovar_int("mpc", 0) != 0)
        wifi_log("[wifi] radio: set mpc=0 failed (continuing)\r\n");
    wifi_radio_done = 1;
}

static int wifi_scan(int *out_count)
{
    static uint8_t fr[2048];
    static uint8_t seen[32][6];     /* dedup by BSSID */
    int nseen = 0, tries, chan, doff, i;

    *out_count = 0;
    wifi_radio_up();
    if (wifi_cmd_int(2, 1) != 0)                  /* WLC_UP */
        wifi_log("[wifi] scan: WLC_UP returned error (continuing)\r\n");
    else
        wifi_log("[wifi] scan: WLC_UP ok\r\n");
    wifi_delay_us(150000);
    wifi_cmd_int(WLC_PASSIVE_SCAN, 0);
    if (wifi_set_iovar("escan", escan_params, sizeof(escan_params)) != 0) {
        wifi_log("[wifi] scan: escan start failed\r\n");
        return -1;
    }
    wifi_log("[wifi] scan: escan started, collecting results...\r\n");

    {
    int nframes = 0, nchan1 = 0, nev = 0;
    for (tries = 0; tries < 2000; tries++) {     /* ~up to several seconds */
        uint8_t *evp, *escan, *bss;
        int len, event, nbss, bdc;

        len = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (len < 0) break;
        if (len == 0) { wifi_delay_us(5000); continue; }
        nframes++;
        if (chan != 1) {
            if (nframes <= 12) wifi_log("[wifi] scan rx: chan=%d len=%d (skip)\r\n", chan, len);
            continue;
        }
        nchan1++;
        if (len < doff + 4) continue;
        bdc = 4 + (fr[doff + 3] << 2);
        evp = fr + doff + bdc;                    /* 802.3 event frame */
        if ((int)(evp - fr) + 24 + 64 > len) continue;
        event = (evp[24 + 6] << 8) | evp[24 + 7]; /* big-endian event number */
        if (nev < 12) { wifi_log("[wifi] scan rx: chan1 len=%d doff=%d bdc=%d event=%d\r\n",
                                 len, doff, bdc, event); nev++; }
        if (event == WLC_E_SCAN_COMPLETE) { wifi_log("[wifi] scan: complete event\r\n"); break; }
        if (event != WLC_E_ESCAN_RESULT) continue;
        escan = evp + 24 + 48;
        if ((int)(escan - fr) + 12 > len) continue;
        nbss = escan[10] | (escan[11] << 8);
        if (nbss == 0) { wifi_log("[wifi] scan: escan done (nbss=0)\r\n"); break; }
        bss = escan + 12;
        if ((int)(bss - fr) + 82 > len) continue;
        {
            uint8_t *bssid = bss + 8;
            int ssidlen = bss[18]; if (ssidlen > 32) ssidlen = 32;
            short rssi = (short)(bss[78] | (bss[79] << 8));
            uint16_t chanspec = (uint16_t)(bss[72] | (bss[73] << 8));
            int ch = chanspec & 0xFF;
            int dup = 0;
            /* directed-join: record this AP's BSSID + real chanspec if its SSID
             * matches the one we're trying to join. */
            if (wifi_tgt_set && !wifi_tgt_found && ssidlen == wifi_tgt_slen) {
                int m = 1;
                for (i = 0; i < ssidlen; i++)
                    if (bss[19 + i] != (uint8_t)wifi_tgt_ssid[i]) { m = 0; break; }
                if (m) {
                    for (i = 0; i < 6; i++) wifi_tgt_bssid[i] = bssid[i];
                    wifi_tgt_chanspec = chanspec; wifi_tgt_found = 1;
                    wifi_log("[wifi] join: target found bssid=%02x:%02x:%02x:%02x:%02x:%02x chanspec=0x%04x ch=%d\r\n",
                             bssid[0],bssid[1],bssid[2],bssid[3],bssid[4],bssid[5], chanspec, ch);
                }
            }
            for (i = 0; i < nseen; i++)
                if (seen[i][0]==bssid[0]&&seen[i][1]==bssid[1]&&seen[i][2]==bssid[2]&&
                    seen[i][3]==bssid[3]&&seen[i][4]==bssid[4]&&seen[i][5]==bssid[5]) { dup=1; break; }
            if (dup) continue;
            if (nseen < 32) { for (i=0;i<6;i++) seen[nseen][i]=bssid[i]; nseen++; }
            {
                char ssid[33];
                for (i = 0; i < ssidlen; i++) {
                    uint8_t c = bss[19 + i];
                    ssid[i] = (c >= 0x20 && c < 0x7f) ? c : '?';
                }
                ssid[ssidlen] = '\0';
                wifi_log("[wifi] AP %2d: \"%s\" %02x:%02x:%02x:%02x:%02x:%02x ch=%d rssi=%d\r\n",
                         nseen, ssidlen ? ssid : "(hidden)",
                         bssid[0],bssid[1],bssid[2],bssid[3],bssid[4],bssid[5], ch, rssi);
            }
        }
        /* Directed join (`wifi on` with a known SSID): once the target AP's
         * beacon arrives we already have its BSSID + channel, so stop the
         * ~10 s collect-everything scan immediately and go to the join. */
        if (wifi_tgt_set && wifi_tgt_found) {
            wifi_log("[wifi] scan: target found early, stopping scan\r\n");
            break;
        }
    }
    wifi_log("[wifi] scan: frames=%d chan1=%d (collected %d AP)\r\n",
             nframes, nchan1, nseen);
    }
    *out_count = nseen;
    return 0;
}

/* ================================================================== *
 *  Bring the chip fully up (Stages 1-4): SDIO host, chip detect,     *
 *  firmware download, SDPCM ioctl.  Idempotent per boot (wifi_ready). *
 * ================================================================== */
static int wifi_ready = 0;
static int wifi_bringup(void)
{
    uint32_t ocr = 0, rca = 0, chipid = 0;
    int r, ioe;

    if (!wl_io_ready) { wl_io_sem = semcreate(1); wl_io_ready = 1; }  /* SDIO mutex */
    if (wifi_ready) { wifi_log("[wifi] (chip already up)\r\n"); return 0; }
    wifi_log("[wifi] === BCM43455 SDIO bring-up ===\r\n");
    wifi_log("[wifi] build: %s\r\n", WIFI_BUILD_ID);

    /* 0a. ROUTING CHECK: read the firmware's GPIO function-select for
     *     GPIO34-39 BEFORE we touch them.  If the firmware already has them
     *     as ALT3 (=7), the Arasan EMMC (0x3F300000) is wired to the WiFi
     *     SDIO bus and our controller choice is correct.  If they read as
     *     input(0)/something-else, the WiFi SDIO may instead be on the SDHOST
     *     controller (0x3F202000) and we are driving the wrong block. */
    {
        uint32_t f3 = GPFSEL3, f4 = GPFSEL4;
        int p;
        wifi_log("[wifi] firmware GPFSEL3=0x%08x GPFSEL4=0x%08x\r\n", f3, f4);
        for (p = 34; p <= 39; p++) {
            int alt = (f3 >> (3 * (p - 30))) & 7;
            wifi_log("[wifi]   GPIO%d fsel=%d (%s)\r\n", p, alt,
                     (alt == 7) ? "ALT3=SD1/Arasan" :
                     (alt == 0) ? "input" : "other");
        }
        wifi_log("[wifi]   GPIO43 (WIFI_CLK) fsel=%d\r\n", (f4 >> 9) & 7);
        /* SD-card pins 48-53: if these read ALT3(=7) the Arasan is on the
         * card (the conflict b7 fixes by moving them to ALT0). */
        {
            uint32_t f5 = GPFSEL5;
            wifi_log("[wifi]   GPIO48=%d 49=%d 50=%d 51=%d 52=%d 53=%d (card pins; 7=Arasan-on-card)\r\n",
                     (f4 >> 24) & 7, (f4 >> 27) & 7,
                     (f5 >> 0) & 7, (f5 >> 3) & 7, (f5 >> 6) & 7, (f5 >> 9) & 7);
        }
    }

    /* 0b. start the chip's reference clock (GPCLK2/WIFI_CLK) BEFORE releasing
     *    reset, so the chip has a clock as it boots. */
    wifi_clk_setup();

    /* 1. power-cycle the chip via WL_REG_ON (firmware expander gpio 129):
     *    drive reset asserted (0), settle, then deasserted (1), and read it
     *    back to confirm the firmware actually drove the expander pin. */
    {
        uint32_t st = 0xDEAD;
        wifi_log("[wifi] WL_REG_ON reset cycle (fw gpio %d)...\r\n", WIFI_GPIO_WL_ON_FW);
        wifi_fw_set_gpio(WIFI_GPIO_WL_ON_FW, 0);   /* hold in reset */
        sleep(50);
        if (wifi_fw_set_gpio(WIFI_GPIO_WL_ON_FW, 1) != 0)  /* release */
            wifi_log("[wifi]   warning: SET_GPIO_STATE response not OK\r\n");
        sleep(150);                                 /* regulators + LPO settle */
        if (wifi_fw_get_gpio(WIFI_GPIO_WL_ON_FW, &st) == 0)
            wifi_log("[wifi] WL_REG_ON readback state=%u (expect 1)\r\n", st);
        else
            wifi_log("[wifi]   warning: GET_GPIO_STATE failed\r\n");
    }

    /* 2. route GPIO34-39 to ALT3 (SDIO bus). */
    wifi_gpio_sdio();
    wifi_log("[wifi] GPIO34-39 -> ALT3 (GPFSEL3=0x%08x)\r\n", GPFSEL3);

    /* 3. bring up the Arasan SDHCI host. */
    if (emmc_host_init() != 0) {
        wifi_log("[wifi] FAILED: host controller init\r\n");
        return -1;
    }

    /* 4. CMD0 (go idle) then CMD5 loop to read the SDIO OCR. */
    emmc_cmd(SD_CMD0_GO_IDLE, 0, CMD_RSPNS_NONE, NULL, 0, NULL);
    sleep(2);

    /* first CMD5 with arg 0 reads the OCR; then repeat with the voltage
     * window until the chip reports ready (OCR bit 31). */
    r = emmc_cmd(SD_CMD5_IO_OP_COND, 0, CMD_RSPNS_48, &ocr, 0, NULL);
    if (r != 0) {
        wifi_log("[wifi] FAILED: CMD5 (no SDIO device responding)\r\n");
        return -1;
    }
    wifi_log("[wifi] CMD5 OCR=0x%08x (numfn=%d mempresent=%d)\r\n",
            ocr, (ocr >> 28) & 7, (ocr >> 27) & 1);

    {
        int tries;
        uint32_t voltwin = ocr & 0x00FFFF00u;   /* the chip's supported window */
        for (tries = 0; tries < 50; tries++) {
            if (emmc_cmd(SD_CMD5_IO_OP_COND, voltwin, CMD_RSPNS_48, &ocr, 0, NULL) != 0)
                break;
            if (ocr & 0x80000000u) break;        /* ready */
            sleep(10);
        }
    }
    if (!(ocr & 0x80000000u)) {
        wifi_log("[wifi] FAILED: SDIO OCR never ready (0x%08x)\r\n", ocr);
        return -1;
    }
    wifi_log("[wifi] SDIO power-up ready, OCR=0x%08x\r\n", ocr);

    /* 5. CMD3 -> relative card address, CMD7 -> select. */
    if (emmc_cmd(SD_CMD3_SEND_RCA, 0, CMD_RSPNS_48, &rca, 0, NULL) != 0) {
        wifi_log("[wifi] FAILED: CMD3 (no RCA)\r\n");
        return -1;
    }
    rca &= 0xFFFF0000u;
    wifi_log("[wifi] RCA=0x%08x\r\n", rca);
    if (emmc_cmd(SD_CMD7_SELECT, rca, CMD_RSPNS_48BUSY, NULL, 0, NULL) != 0) {
        wifi_log("[wifi] FAILED: CMD7 (select)\r\n");
        return -1;
    }
    wifi_log("[wifi] chip selected (SDIO command state)\r\n");

    /* 6. enable backplane function 1 (CCCR IOEx bit 1) and wait IOR1. */
    if (sdio_cmd52(0, SDIO_CCCR_IOEx, 1, 0x02) < 0) {
        wifi_log("[wifi] FAILED: enable F1 (CMD52 IOEx)\r\n");
        return -1;
    }
    {
        int tries;
        for (tries = 0; tries < 50; tries++) {
            ioe = sdio_cmd52(0, SDIO_CCCR_IORx, 0, 0);
            if (ioe >= 0 && (ioe & 0x02)) break;
            sleep(2);
        }
    }
    if (!(ioe & 0x02)) {
        wifi_log("[wifi] FAILED: F1 not ready (IOR1=0x%02x)\r\n", ioe);
        return -1;
    }
    wifi_log("[wifi] backplane function 1 enabled+ready\r\n");

    /* 6b. set SDIO function block sizes.  Block-mode CMD53 (used by the bulk
     * backplane transfers / firmware download) frames data into fixed-size
     * blocks each with its own CRC16; without programming the function block
     * size the device CRC-mismatches (the b8 DCRC error 0x208000).  plan9
     * uses 64 for F1 (backplane) and 512 for F2 (WLAN data).
     * FBR1 blksize @ CCCR 0x110, FBR2 @ 0x210. */
    sdio_cmd52(0, 0x110, 1, 64);
    sdio_cmd52(0, 0x111, 1, 0);
    sdio_cmd52(0, 0x210, 1, 512 & 0xFF);
    sdio_cmd52(0, 0x211, 1, (512 >> 8) & 0xFF);
    wifi_log("[wifi] F1 blksize=64, F2 blksize=512\r\n");

    /* 7. read the silicon chip-id over the backplane (chipcommon offset 0). */
    if (wifi_backplane_read32(SI_ENUM_BASE, &chipid) != 0) {
        wifi_log("[wifi] FAILED: backplane chip-id read\r\n");
        return -1;
    }
    wifi_log("[wifi] chipcommon[0]=0x%08x  chip-id=0x%04x  rev=%d\r\n",
            chipid, chipid & CID_ID_MASK, (chipid >> 16) & 0xF);

    if ((chipid & CID_ID_MASK) != BCM43455_CHIP_ID) {
        wifi_log("[wifi] chip-id 0x%04x is not the expected 0x4345 "
                "(check window/access path)\r\n", chipid & CID_ID_MASK);
        return -1;
    }
    wifi_log("[wifi] BCM43455 (0x4345) detected over SDIO\r\n");

    /* ---- Stage 3a: enumerate the chip's cores + on-chip RAM ----
     * Validates the bulk backplane path (CMD53 PIO + sbmem) by reading the
     * 512-byte EROM and the CR4 RAM banks.  This is the foundation for the
     * firmware download (Stage 3b). */
    if (wifi_corescan() != 0) {
        wifi_log("[wifi] FAILED: corescan (EROM bulk read)\r\n");
        return -1;
    }
    wifi_log("[wifi] cores: chipcommon=0x%x armcore=0x%x armctl=0x%x armregs=0x%x\r\n",
             chip_chipcommon, chip_armcore, chip_armctl, chip_armregs);
    wifi_log("[wifi]        d11ctl=0x%x socramctl=0x%x sdregs=0x%x\r\n",
             chip_d11ctl, chip_socramctl, chip_sdregs);
    if (chip_armctl == 0 || chip_d11ctl == 0) {
        wifi_log("[wifi] FAILED: corescan missing essential cores\r\n");
        return -1;
    }
    if (wifi_ramscan() != 0) {
        wifi_log("[wifi] FAILED: ramscan\r\n");
        return -1;
    }
    wifi_log("[wifi] RAM: base=0x%x size=%u bytes (%u KB)\r\n",
             chip_rambase, chip_socramsize, chip_socramsize / 1024);

    /* ---- Stage 3b: download firmware + start the ARM CR4 ----
     * sbinit-equivalent: halt the ARM core, reset the d11 (MAC) core, then
     * force the ALP clock; then fwload writes fw+nvram and releases the CR4;
     * sbenable brings up the HT clock and enables WLAN function 2. */
    sb_reset(chip_armctl, CR4_CPUHALT, CR4_CPUHALT);   /* halt CR4 */
    sb_reset(chip_d11ctl, 8 | 4, 4);                   /* reset d11 MAC */
    cfgw(REG_CLKCSR, 0);
    wifi_delay_us(10);
    cfgw(REG_CLKCSR, CLK_NOHWREQ | CLK_REQALP);
    for (r = 0; r < 2000 && !(cfgr(REG_CLKCSR) & (CLK_HTAVAIL | CLK_ALPAVAIL)); r++)
        wifi_delay_us(10);
    cfgw(REG_CLKCSR, CLK_NOHWREQ | CLK_FORCEALP);
    wifi_delay_us(65);
    cfgw(REG_PULLUPS, 0);
    wifi_backplane_write32(chip_chipcommon + 0x58, 0);   /* gpio pullup   */
    wifi_backplane_write32(chip_chipcommon + 0x5c, 0);   /* gpio pulldown */
    wifi_log("[wifi] chip halted + ALP clock forced (csr=0x%02x)\r\n", cfgr(REG_CLKCSR));

    if (wifi_fwload() != 0) {
        wifi_log("[wifi] FAILED: firmware download\r\n");
        return -1;
    }
    if (wifi_sbenable() != 0) {
        wifi_log("[wifi] FAILED: sbenable (HT clock / Fn2)\r\n");
        return -1;
    }
    wifi_log("[wifi] Stage 3 — firmware loaded, chip running\r\n");

    /* ---- Stage 4: SDPCM control channel — ask the firmware its version ----
     * A successful "ver" iovar GET proves the host<->firmware ioctl path
     * (SDPCM framing + BCDC control header over Fn2) works end to end. */
    {
        static uint8_t ver[128];
        int i;
        for (i = 0; i < (int)sizeof(ver); i++) ver[i] = 0;
        wifi_delay_us(50000);       /* let the firmware settle after boot */
        if (wifi_get_iovar("ver", ver, sizeof(ver) - 1) != 0) {
            wifi_log("[wifi] FAILED: SDPCM ioctl (get 'ver')\r\n");
            return -1;
        }
        /* trim the version string to its first line for the trace */
        for (i = 0; i < (int)sizeof(ver) - 1; i++)
            if (ver[i] == '\r' || ver[i] == '\n') { ver[i] = '\0'; break; }
        wifi_log("[wifi] firmware version: %s\r\n", (char *)ver);
        wifi_log("[wifi] Stage 4 — SDPCM ioctl works (fw responds)\r\n");
    }
    wifi_ready = 1;
    return 0;
}

/* ================================================================== *
 *  SHA1 / HMAC-SHA1 / PBKDF2 — to derive the WPA2 PMK from the        *
 *  passphrase (PMK = PBKDF2-SHA1(passphrase, ssid, 4096, 32)).        *
 * ================================================================== */
struct sha1 { uint32_t h[5]; uint64_t len; uint8_t buf[64]; int n; };

static uint32_t sha1_rol(uint32_t v, int s) { return (v << s) | (v >> (32 - s)); }

static void sha1_block(struct sha1 *c, const uint8_t *p)
{
    uint32_t w[80], a, b, d, e, f, k, t; int i;
    for (i = 0; i < 16; i++)
        w[i] = (p[i*4]<<24)|(p[i*4+1]<<16)|(p[i*4+2]<<8)|p[i*4+3];
    for (i = 16; i < 80; i++) w[i] = sha1_rol(w[i-3]^w[i-8]^w[i-14]^w[i-16], 1);
    a=c->h[0]; b=c->h[1]; d=c->h[2]; e=c->h[3]; f=c->h[4];
    for (i = 0; i < 80; i++) {
        uint32_t fn;
        if (i < 20)      { fn = (b & d) | (~b & e); k = 0x5A827999; }
        else if (i < 40) { fn = b ^ d ^ e;          k = 0x6ED9EBA1; }
        else if (i < 60) { fn = (b&d)|(b&e)|(d&e);  k = 0x8F1BBCDC; }
        else             { fn = b ^ d ^ e;          k = 0xCA62C1D6; }
        t = sha1_rol(a,5) + fn + f + k + w[i];
        f = e; e = d; d = sha1_rol(b,30); b = a; a = t;
    }
    c->h[0]+=a; c->h[1]+=b; c->h[2]+=d; c->h[3]+=e; c->h[4]+=f;
}

static void sha1_init(struct sha1 *c)
{
    c->h[0]=0x67452301; c->h[1]=0xEFCDAB89; c->h[2]=0x98BADCFE;
    c->h[3]=0x10325476; c->h[4]=0xC3D2E1F0; c->len=0; c->n=0;
}

static void sha1_update(struct sha1 *c, const uint8_t *p, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        c->buf[c->n++] = p[i];
        if (c->n == 64) { sha1_block(c, c->buf); c->n = 0; }
    }
    c->len += len;
}

static void sha1_final(struct sha1 *c, uint8_t out[20])
{
    uint64_t bits = c->len * 8; int i;
    uint8_t pad = 0x80;
    sha1_update(c, &pad, 1);
    pad = 0;
    while (c->n != 56) sha1_update(c, &pad, 1);
    for (i = 7; i >= 0; i--) { uint8_t b = (bits >> (i*8)) & 0xFF; sha1_update(c, &b, 1); }
    for (i = 0; i < 5; i++) {
        out[i*4]   = c->h[i] >> 24; out[i*4+1] = c->h[i] >> 16;
        out[i*4+2] = c->h[i] >> 8;  out[i*4+3] = c->h[i];
    }
}

/* HMAC-SHA1(key, msg) -> 20-byte mac */
static void hmac_sha1(const uint8_t *key, int klen, const uint8_t *msg, int mlen,
                      uint8_t mac[20])
{
    uint8_t k[64], ipad[64], opad[64], inner[20];
    struct sha1 c; int i;
    for (i = 0; i < 64; i++) k[i] = 0;
    if (klen > 64) { sha1_init(&c); sha1_update(&c, key, klen); sha1_final(&c, k); }
    else for (i = 0; i < klen; i++) k[i] = key[i];
    for (i = 0; i < 64; i++) { ipad[i] = k[i] ^ 0x36; opad[i] = k[i] ^ 0x5C; }
    sha1_init(&c); sha1_update(&c, ipad, 64); sha1_update(&c, msg, mlen); sha1_final(&c, inner);
    sha1_init(&c); sha1_update(&c, opad, 64); sha1_update(&c, inner, 20); sha1_final(&c, mac);
}

/* PBKDF2-SHA1: derive `dklen` bytes from passphrase + salt (ssid), 4096 iters. */
static void pbkdf2_sha1(const uint8_t *pass, int plen, const uint8_t *salt, int slen,
                        int iter, uint8_t *dk, int dklen)
{
    int block = 1, off = 0;
    while (off < dklen) {
        uint8_t salt_i[40], u[20], t[20]; int i, j, cpy;
        for (i = 0; i < slen && i < 36; i++) salt_i[i] = salt[i];
        salt_i[slen]   = block >> 24; salt_i[slen+1] = block >> 16;
        salt_i[slen+2] = block >> 8;  salt_i[slen+3] = block;
        hmac_sha1(pass, plen, salt_i, slen + 4, u);
        for (i = 0; i < 20; i++) t[i] = u[i];
        for (j = 1; j < iter; j++) {
            hmac_sha1(pass, plen, u, 20, u);
            for (i = 0; i < 20; i++) t[i] ^= u[i];
        }
        cpy = (dklen - off < 20) ? (dklen - off) : 20;
        for (i = 0; i < cpy; i++) dk[off + i] = t[i];
        off += cpy; block++;
    }
}

/* ================================================================== *
 *  Stage 6 — join a (WPA2-PSK) access point                          *
 * ================================================================== */
#define WLC_DOWN        3
#define WLC_GET_INFRA   19
#define WLC_SET_INFRA   20
#define WLC_SET_WSEC    134
#define WLC_SET_WPA_AUTH 165
#define WLC_SET_WSEC_PMK 268
#define WLC_SET_SSID    26
#define WLC_GET_BSSID   23
#define WLC_SET_CHANNEL 30

/* Derive the WPA2 PMK (PBKDF2-SHA1 of passphrase+ssid) and hand it to the
 * firmware's in-dongle supplicant via WLC_SET_WSEC_PMK.
 *
 * ★ The struct is brcmf_wsec_pmk_le { __le16 key_len; __le16 flags; u8 key[128] }
 *   = a FIXED 132 bytes (key[] is BRCMF_WSEC_MAX_SAE_PASSWORD_LEN=128), sent in
 *   full regardless of key_len.  Our earlier attempt sent only 36 bytes
 *   (key[32]) which the firmware rejected (-2 BADARG) — the FWSUP firmware
 *   wants the whole 132-byte struct.  key_len=32, flags=0 for a raw PMK. */
static int wifi_set_pmk(const char *ssid, int slen, const char *pass, int plen)
{
    uint8_t pmk32[32];
    uint8_t msg[2 + 2 + 128];             /* brcmf_wsec_pmk_le: 132 B fixed */
    int i;
    pbkdf2_sha1((const uint8_t *)pass, plen, (const uint8_t *)ssid, slen, 4096, pmk32, 32);
    wifi_log("[wifi] join: PMK %02x%02x%02x%02x...%02x%02x derived\r\n",
             pmk32[0], pmk32[1], pmk32[2], pmk32[3], pmk32[30], pmk32[31]);
    for (i = 0; i < (int)sizeof(msg); i++) msg[i] = 0;
    msg[0] = 32; msg[1] = 0;              /* key_len = 32 (raw PMK) */
    msg[2] = 0;  msg[3] = 0;              /* flags = 0 (key material, not passphrase) */
    for (i = 0; i < 32; i++) msg[4 + i] = pmk32[i];
    return wifi_wlcmd(1, WLC_SET_WSEC_PMK, msg, sizeof(msg), NULL, 0);
}

/* Join an AP.  pass==NULL/"" => open network; otherwise WPA2-PSK. */
static int wifi_do_join(const char *ssid, const char *pass)
{
    uint8_t jp[114];
    int sl = 0, i, ev, secured = (pass && pass[0]), pl = 0, jsz;

    while (ssid[sl] && sl < 32) sl++;
    if (pass) while (pass[pl] && pl < 63) pl++;
    wifi_log("[wifi] join: ssid=\"%s\" %s\r\n", ssid, secured ? "WPA2-PSK" : "open");
    wifi_d_sup = wifi_d_pmk = -999; wifi_d_nev = wifi_d_eapol = 0;
    wifi_d_link = -1; wifi_d_lastev = -1; wifi_d_laststat = 0;

    wifi_radio_up();                             /* event_msgs/clm/country/pm */

    /* Establish infra (managed STA) mode + UP ONCE, before scanning/joining.
     * brcmf_cfg80211_connect runs with the interface already up in infra mode
     * and never toggles DOWN afterwards — toggling DOWN after the scan (as we
     * did) left the fw mid-scan / cleared state so the join never associated. */
    wifi_cmd_int(WLC_DOWN, 1);
    wifi_cmd_int(WLC_SET_INFRA, 1);
    wifi_cmd_int(2, 1);                          /* WLC_UP */
    /* Re-assert "minimize power consumption" OFF on every join.  wifi_radio_up()
     * sets mpc=0 only once (it's wifi_radio_done-guarded), but a prior
     * `wifi off` (WLC_DOWN) can let the firmware power the radio back down — so
     * on a RECONNECT the directed scan below finds no AP ("No access points").
     * Forcing mpc=0 here keeps the radio awake for the scan + association. */
    wifi_set_iovar_int("mpc", 0);
    wifi_delay_us(50000);

    /* A prior scan populates the firmware's BSS cache (brcmfmac always connects
     * after a cfg80211 scan).  Keep it directed by SSID for the header diag,
     * but DON'T toggle DOWN/UP afterwards — go straight to the iovars + join. */
    {
        int n = 0, j;
        wifi_tgt_slen = sl; wifi_tgt_found = 0; wifi_tgt_set = 1;
        for (j = 0; j < sl && j < 32; j++) wifi_tgt_ssid[j] = ssid[j];
        wifi_scan(&n);
        wifi_tgt_set = 0;
        wifi_tn = 0;                             /* trace shows only join phase */
        wifi_log("[wifi] === JOIN PHASE === ssid=\"%s\" tgt=%d chanspec=0x%04x\r\n",
                 ssid, wifi_tgt_found, wifi_tgt_chanspec);
    }

    if (secured) {
        /* Match brcmf_cfg80211_connect exactly — every security parameter is a
         * (bsscfg, idx 0 => plain) IOVAR, not a WLC_SET_* command.  Order:
         *   set_wpa_version : wpa_auth = WPA2_AUTH_PSK|WPA2_AUTH_UNSPECIFIED
         *   set_auth_type   : auth     = 0 (open)
         *   set_wsec_mode   : wsec     = AES_ENABLED (4)
         *   set_key_mgmt    : wpa_auth = WPA2_AUTH_PSK (refine)
         *   FWSUP           : sup_wpa  = 1
         *   set_pmk         : WLC_SET_WSEC_PMK (the only command)
         * Setting wsec/auth via commands (134/165/22) earlier left the bsscfg
         * iovars the join/FWSUP path reads at 0 -> probed but never associated. */
        int rc, r1, r2, r3, r4;
        r1 = wifi_set_iovar_int("wpa_auth", 0xc0); /* WPA2_AUTH_PSK | UNSPECIFIED */
        r2 = wifi_set_iovar_int("auth", 0);        /* open auth */
        r3 = wifi_set_iovar_int("wsec", 4);        /* AES_ENABLED (CCMP) */
        r4 = wifi_set_iovar_int("wpa_auth", 0x80); /* WPA2_AUTH_PSK */
        wifi_log("[wifi] join: iovar rc wpa_auth(c0)=%d auth=%d wsec=%d wpa_auth(80)=%d\r\n",
                 r1, r2, r3, r4);
        rc = wifi_set_iovar_int("sup_wpa", 1);   /* enable in-dongle supplicant */
        wifi_d_sup = rc;
        wifi_log("[wifi] join: sup_wpa=1 rc=%d %s\r\n", rc,
                 rc ? "(FWSUP NOT available — wrong fw variant?)" : "(FWSUP on)");
        rc = wifi_set_pmk(ssid, sl, pass, pl);   /* 132-B WSEC_PMK with PMK */
        wifi_d_pmk = rc;
        wifi_log("[wifi] join: WSEC_PMK rc=%d %s\r\n", rc,
                 rc ? "(fw rejected PMK)" : "(PMK accepted)");
    } else {
        wifi_set_iovar_int("wsec", 0);
        wifi_set_iovar_int("wpa_auth", 0);       /* WPA_AUTH_DISABLED */
        wifi_set_iovar_int("auth", 0);
    }

    /* Build the "join" iovar.  ★ KEY: brcmf_ext_join_params_le is NOT __packed
     * (only one __packed in all of fwil_types.h, not these), so it's NATURALLY
     * ALIGNED — scan_type(1)+3pad+4*le32 = 20, assoc bssid(6)+2pad+chanspec_num
     * (4)+chanspec(2).  Real join_params_size = offsetof(assoc_le)=56 + offsetof
     * (chanspec_list)=12 + 2 = 70 B (with channel), 68 (without).  That is why
     * a 65-B packed guess got -14 BUFTOOSHORT — and the aligned layout is
     * byte-identical to the proven ether4330 wl_extjoin_params_t.  Layout
     * (70 B with 1 chanspec):
     *   [0..3]   ssid.SSID_len        [4..35]  SSID[32]
     *   --- wl_join_scan_params_t (20 B) ---
     *   [36..39] scan_type(u8)+pad = 0xff   [40..43] nprobes
     *   [44..47] active_time  [48..51] passive_time  [52..55] home_time
     *   --- wl_join_assoc_params_t (16 B) ---
     *   [56..61] bssid   [62..63] bssid_cnt/pad
     *   [64..67] chanspec_num   [68..69] chanspec_list[0]   [70..71] pad */
    for (i = 0; i < (int)sizeof(jp); i++) jp[i] = 0;
    jp[0] = sl;                                   /* ssid.SSID_len */
    for (i = 0; i < sl; i++) jp[4 + i] = ssid[i];
    jp[36] = 0xFF;                                /* scan_type = 0xff (ether4330) */
    if (wifi_tgt_found) {
        /* directed: known BSSID + chanspec, finite probes/dwell */
        jp[40] = 2;                               /* nprobes = 2 */
        jp[44] = 120;                             /* active_time = 120 ms */
        jp[48] = 0x86; jp[49] = 0x01;             /* passive_time = 390 ms */
        jp[52]=0xFF; jp[53]=0xFF; jp[54]=0xFF; jp[55]=0xFF;  /* home_time = -1 */
        for (i = 0; i < 6; i++) jp[56 + i] = wifi_tgt_bssid[i];  /* assoc bssid */
        jp[64] = 1;                               /* chanspec_num = 1 */
        jp[68] = wifi_tgt_chanspec & 0xFF;
        jp[69] = (wifi_tgt_chanspec >> 8) & 0xFF;
        jsz = 70;   /* EXACT brcmfmac size: offsetof(assoc_le)56 + 12 + 2 */
    } else {
        for (i = 40; i < 56; i++) jp[i] = 0xFF;   /* nprobes/active/passive/home = -1 */
        for (i = 56; i < 62; i++) jp[i] = 0xFF;   /* assoc bssid = broadcast */
        /* chanspec_num = 0; omit chanspec_list+pad */
        jsz = 68;
    }
    /* Send the join + wait for association, retrying a few times: the assoc
     * can stall at AUTH and fall back to scanning (PROBREQ), and a fresh join
     * iovar then usually completes.  Success path: E_SET_SSID(0) -> fw 4-way ->
     * E_PSK_SUP(46) status 6 (WLC_SUP_KEYED) -> E_LINK(16) up.  GET_BSSID is the
     * ground-truth "are we on?" poll (a fast directed join can beat the FIFO). */
    {
        static uint8_t fr[2048];
        int attempt, chan, doff, tries, nev, ndata, eapol;
        for (attempt = 0; attempt < 4; attempt++) {
            if (wifi_set_iovar("join", jp, jsz) != 0) {
                wifi_log("[wifi] join: 'join' iovar failed\r\n");
                return -1;
            }
            wifi_log("[wifi] join: request sent (attempt %d), waiting...\r\n", attempt + 1);
            nev = ndata = eapol = 0;
            for (tries = 0; tries < 600; tries++) {     /* ~6 s per attempt */
                int len;
                if ((tries % 80) == 40) {
                    uint8_t bss[6]; int k, nz = 0;
                    if (wifi_wlcmd(0, WLC_GET_BSSID, NULL, 0, bss, 6) == 0)
                        for (k = 0; k < 6; k++) if (bss[k] && bss[k] != 0xFF) nz = 1;
                    if (nz) {
                        wifi_d_link = 1;
                        wifi_log("[wifi] join: GET_BSSID %02x:%02x:%02x:%02x:%02x:%02x -> *** associated ***\r\n",
                                 bss[0],bss[1],bss[2],bss[3],bss[4],bss[5]);
                        return 0;
                    }
                }
                len = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
                if (len < 0) break;
                if (len == 0) { wifi_delay_us(10000); continue; }
                if (len < doff + 4) continue;
                {
                    int bdc = 4 + (fr[doff + 3] << 2);
                    uint8_t *p = fr + doff + bdc;     /* 802.3 frame */
                    if ((int)(p - fr) + 16 > len) continue;
                    if (chan == 1) {                  /* event */
                        long status = (p[24+8]<<24)|(p[24+9]<<16)|(p[24+10]<<8)|p[24+11];
                        ev = (p[24 + 6] << 8) | p[24 + 7];
                        nev++; wifi_d_nev = nev; wifi_d_lastev = ev; wifi_d_laststat = (int)status;
                        if (nev <= 16) wifi_d_seq[nev - 1] = ev;
                        if (nev <= 16) wifi_log("[wifi] join: event %d status %ld\r\n", ev, status);
                        if (ev == 16) {
                            int up = (p[24+2] << 8 | p[24+3]) & 1;
                            wifi_d_link = up ? 1 : 0;
                            wifi_log("[wifi] join: E_LINK %s\r\n", up ? "UP" : "down");
                            if (up) { wifi_log("[wifi] *** associated (link up) ***\r\n"); return 0; }
                        } else if (ev == 5 || ev == 6 || ev == 12 || ev == 24) {
                            wifi_log("[wifi] join: deauth/disassoc event %d\r\n", ev);
                        }
                    } else if (chan == 2) {           /* data frame */
                        int et = (p[12] << 8) | p[13];
                        ndata++;
                        if (et == 0x888E) { eapol++; wifi_d_eapol = eapol; }
                    }
                }
            }
            wifi_log("[wifi] join: attempt %d timeout (events=%d)\r\n", attempt + 1, nev);
        }
    }
    return -1;
}

/* ================================================================== *
 *  Data path (SDPCM channel 2) + a minimal DHCP client                *
 * ================================================================== */

/* Transmit one 802.3 Ethernet frame over the WLAN data channel.
 * Framing (matches ether4330 txstart): SDPCM hdr (12, chanflg=2, doffset=12)
 * + BDC hdr (4 bytes, flags byte 0x20 = BDC proto ver 2) + the ethernet frame. */
static int wifi_data_tx(const uint8_t *eth, int ethlen)
{
    static uint8_t pkt[1600];
    int len = SDPCM_HDR + 4 + ethlen, i, spin;
    if (len > (int)sizeof(pkt)) return -1;
    /* Respect the fw tx credit window + data-channel flow control; if closed,
     * drain RX frames (which carry fresh window/fcmask) until it reopens. */
    for (spin = 0; spin < 60; spin++) {
        if (!(wl_fcmask & (1 << 2)) && wl_txseq != wl_txwindow) break;
        { static uint8_t t[2048]; int c, d; if (wifi_read_frame(t, sizeof(t), &c, &d) <= 0) wifi_delay_us(2000); }
    }
    for (i = 0; i < len; i++) pkt[i] = 0;
    pkt[0] = len & 0xFF; pkt[1] = (len >> 8) & 0xFF;
    pkt[2] = ~len & 0xFF; pkt[3] = (~len >> 8) & 0xFF;
    pkt[4] = wl_txseq;
    pkt[5] = 2;                 /* channel 2 = data */
    pkt[7] = SDPCM_HDR;         /* doffset -> BDC header */
    pkt[SDPCM_HDR + 0] = 0x20;  /* BDC: proto ver 2, prio 0, data_offset 0 */
    for (i = 0; i < ethlen; i++) pkt[SDPCM_HDR + 4 + i] = eth[i];
    if (wifi_packetrw(1, pkt, len) != 0) return -1;
    wl_txseq++;
    return 0;
}

/* 16-bit one's-complement checksum (IP header / UDP). */
static uint16_t ip_cksum(const uint8_t *p, int n, uint32_t sum)
{
    int i;
    for (i = 0; i + 1 < n; i += 2) sum += (p[i] << 8) | p[i + 1];
    if (i < n) sum += p[i] << 8;
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum & 0xFFFF);
}

/* DHCP results (also surfaced via headers). */
static uint8_t wifi_ip[4], wifi_mask[4], wifi_gw[4], wifi_dns[4], wifi_mac[6];
static int     wifi_have_ip = 0;
void wifi_dhcp_diag(uint8_t *ip, uint8_t *gw, int *have) {
    int i; for (i=0;i<4;i++){ ip[i]=wifi_ip[i]; gw[i]=wifi_gw[i]; } *have = wifi_have_ip;
}

/* Connection state for the on-screen (gwm) WiFi indicator: nonzero once
 * DHCP has leased an IP, zeroed when a (re)join starts. */
int wifi_connected(void) { return wifi_have_ip; }

/* SSID of the network we (last) joined — shown under the gwm WiFi mark. */
static char wifi_cur_ssid[40] = "";
const char *wifi_ssid(void) { return wifi_cur_ssid; }

/* Build a DHCP packet (DISCOVER if reqip==NULL, else REQUEST) into `out`.
 * Returns total ethernet frame length.  xid identifies our exchange. */
static int dhcp_build(uint8_t *out, const uint8_t *mac, uint32_t xid,
                      const uint8_t *reqip, const uint8_t *srvid)
{
    uint8_t *e = out, *ip, *udp, *bootp, *opt;
    int dhcplen, udplen, iplen, i;

    /* Ethernet: dst broadcast, src mac, type IPv4 */
    for (i = 0; i < 6; i++) e[i] = 0xFF;
    for (i = 0; i < 6; i++) e[6 + i] = mac[i];
    e[12] = 0x08; e[13] = 0x00;
    ip = e + 14;
    udp = ip + 20;
    bootp = udp + 8;

    /* BOOTP/DHCP fixed 236 bytes */
    for (i = 0; i < 236; i++) bootp[i] = 0;
    bootp[0] = 1; bootp[1] = 1; bootp[2] = 6;          /* op=REQUEST htype=ETH hlen=6 */
    bootp[4] = xid >> 24; bootp[5] = xid >> 16; bootp[6] = xid >> 8; bootp[7] = xid;
    bootp[10] = 0x80;                                   /* flags = broadcast */
    for (i = 0; i < 6; i++) bootp[28 + i] = mac[i];     /* chaddr */
    opt = bootp + 236;
    opt[0] = 0x63; opt[1] = 0x82; opt[2] = 0x53; opt[3] = 0x63;  /* magic cookie */
    opt += 4;
    *opt++ = 53; *opt++ = 1; *opt++ = reqip ? 3 : 1;    /* msg type: REQUEST/DISCOVER */
    if (reqip) {
        *opt++ = 50; *opt++ = 4; for (i=0;i<4;i++) *opt++ = reqip[i];   /* requested IP */
        *opt++ = 54; *opt++ = 4; for (i=0;i<4;i++) *opt++ = srvid[i];   /* server id */
    }
    *opt++ = 55; *opt++ = 4; *opt++ = 1; *opt++ = 3; *opt++ = 6; *opt++ = 51; /* params */
    *opt++ = 255;                                        /* end */
    dhcplen = (int)(opt - bootp);

    /* UDP: sport 68, dport 67 */
    udplen = 8 + dhcplen;
    udp[0]=0; udp[1]=68; udp[2]=0; udp[3]=67;
    udp[4]=udplen>>8; udp[5]=udplen&0xFF; udp[6]=0; udp[7]=0;  /* csum 0 (optional) */

    /* IPv4 header */
    iplen = 20 + udplen;
    for (i = 0; i < 20; i++) ip[i] = 0;
    ip[0]=0x45; ip[2]=iplen>>8; ip[3]=iplen&0xFF; ip[8]=64; ip[9]=17;  /* ttl, proto UDP */
    /* src 0.0.0.0, dst 255.255.255.255 */
    for (i = 0; i < 4; i++) ip[16 + i] = 0xFF;
    { uint16_t c = ip_cksum(ip, 20, 0); ip[10]=c>>8; ip[11]=c&0xFF; }

    return 14 + iplen;
}

/* Parse a received DHCP reply.  Returns the DHCP msg type (2=OFFER,5=ACK) or
 * 0, fills yiaddr + server id + mask/gw/dns from options.  `eth`/`elen` is the
 * 802.3 frame (after SDPCM+BDC). */
static int dhcp_parse(const uint8_t *eth, int elen, uint32_t xid,
                      uint8_t *yiaddr, uint8_t *srvid,
                      uint8_t *mask, uint8_t *gw, uint8_t *dns)
{
    const uint8_t *ip, *udp, *bootp, *o, *end;
    int ihl, mtype = 0, i;
    if (elen < 14 + 20 + 8 + 240) return 0;
    if (eth[12] != 0x08 || eth[13] != 0x00) return 0;     /* IPv4 */
    ip = eth + 14;
    ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 17) return 0;                            /* UDP */
    udp = ip + ihl;
    if (udp[2] != 0 || udp[3] != 68) return 0;            /* dport 68 (to client) */
    bootp = udp + 8;
    if (bootp[0] != 2) return 0;                          /* BOOTREPLY */
    if (((uint32_t)bootp[4]<<24|bootp[5]<<16|bootp[6]<<8|bootp[7]) != xid) return 0;
    for (i = 0; i < 4; i++) yiaddr[i] = bootp[16 + i];    /* offered IP */
    o = bootp + 236;
    if (o[0]!=0x63||o[1]!=0x82||o[2]!=0x53||o[3]!=0x63) return 0;
    o += 4; end = eth + elen;
    while (o < end && *o != 255) {
        int t = *o++, l; if (o >= end) break; l = *o++;
        if (o + l > end) break;
        if (t == 53 && l >= 1) mtype = o[0];
        else if (t == 54 && l >= 4) for (i=0;i<4;i++) srvid[i]=o[i];
        else if (t == 1  && l >= 4) for (i=0;i<4;i++) mask[i]=o[i];
        else if (t == 3  && l >= 4) for (i=0;i<4;i++) gw[i]=o[i];
        else if (t == 6  && l >= 4) for (i=0;i<4;i++) dns[i]=o[i];
        o += l;
    }
    return mtype;
}

/* Wait for a DHCP reply of the wanted msg type (2 or 5).  Returns msg type. */
static int dhcp_wait(uint32_t xid, int want, uint8_t *yiaddr, uint8_t *srvid,
                     uint8_t *mask, uint8_t *gw, uint8_t *dns)
{
    static uint8_t fr[2048];
    int chan, doff, tries;
    for (tries = 0; tries < 800; tries++) {
        int len = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (len < 0) break;
        if (len == 0) { wifi_delay_us(10000); continue; }
        if (chan != 2 || len < doff + 4) continue;
        {
            int bdc = 4 + (fr[doff + 3] << 2);
            const uint8_t *eth = fr + doff + bdc;
            int elen = len - (doff + bdc);
            int t = dhcp_parse(eth, elen, xid, yiaddr, srvid, mask, gw, dns);
            if (t == want) return t;
        }
    }
    return 0;
}

/* Run a DHCP DISCOVER/REQUEST exchange over the (already associated) link. */
int wifi_dhcp(void)
{
    static uint8_t pkt[600];
    uint8_t yi[4]={0}, srv[4]={0}, mask[4]={0}, gw[4]={0}, dns[4]={0};
    uint32_t xid;
    int n, t;

    wifi_have_ip = 0;
    wifi_tn = 0;                /* trace shows only the DHCP exchange */
    wifi_log("[wifi] === DHCP ===\r\n");
    if (wifi_get_iovar("cur_etheraddr", wifi_mac, 6) != 0) {
        wifi_log("[wifi] dhcp: cur_etheraddr failed\r\n"); return -1;
    }
    wifi_log("[wifi] dhcp: mac %02x:%02x:%02x:%02x:%02x:%02x\r\n",
             wifi_mac[0],wifi_mac[1],wifi_mac[2],wifi_mac[3],wifi_mac[4],wifi_mac[5]);
    xid = 0x52610000u | (wifi_mac[4] << 8) | wifi_mac[5];  /* 'Ra' + mac tail */

    n = dhcp_build(pkt, wifi_mac, xid, NULL, NULL);         /* DISCOVER */
    if (wifi_data_tx(pkt, n) != 0) { wifi_log("[wifi] dhcp: DISCOVER tx failed\r\n"); return -1; }
    wifi_log("[wifi] dhcp: DISCOVER sent (%d B), waiting for OFFER...\r\n", n);
    t = dhcp_wait(xid, 2, yi, srv, mask, gw, dns);
    if (t != 2) { wifi_log("[wifi] dhcp: no OFFER (timeout)\r\n"); return -1; }
    wifi_log("[wifi] dhcp: OFFER ip=%d.%d.%d.%d server=%d.%d.%d.%d\r\n",
             yi[0],yi[1],yi[2],yi[3], srv[0],srv[1],srv[2],srv[3]);

    n = dhcp_build(pkt, wifi_mac, xid, yi, srv);            /* REQUEST */
    if (wifi_data_tx(pkt, n) != 0) { wifi_log("[wifi] dhcp: REQUEST tx failed\r\n"); return -1; }
    wifi_log("[wifi] dhcp: REQUEST sent, waiting for ACK...\r\n");
    t = dhcp_wait(xid, 5, yi, srv, mask, gw, dns);
    if (t != 5) { wifi_log("[wifi] dhcp: no ACK (timeout)\r\n"); return -1; }

    for (n = 0; n < 4; n++) { wifi_ip[n]=yi[n]; wifi_mask[n]=mask[n]; wifi_gw[n]=gw[n]; wifi_dns[n]=dns[n]; }
    wifi_have_ip = 1;
    wifi_log("[wifi] *** DHCP ACK: ip=%d.%d.%d.%d mask=%d.%d.%d.%d gw=%d.%d.%d.%d dns=%d.%d.%d.%d ***\r\n",
             wifi_ip[0],wifi_ip[1],wifi_ip[2],wifi_ip[3],
             wifi_mask[0],wifi_mask[1],wifi_mask[2],wifi_mask[3],
             wifi_gw[0],wifi_gw[1],wifi_gw[2],wifi_gw[3],
             wifi_dns[0],wifi_dns[1],wifi_dns[2],wifi_dns[3]);
    return 0;
}

/* ------------------------------------------------------------------ *
 *  Minimal IP responder: ARP + ICMP echo, so the host can ping us.    *
 * ------------------------------------------------------------------ */
static int wifi_ip_eq(const uint8_t *p) {
    return p[0]==wifi_ip[0] && p[1]==wifi_ip[1] && p[2]==wifi_ip[2] && p[3]==wifi_ip[3];
}

/* ================================================================== *
 *  MANET HELLO (M14, rpi3 port) — periodic self-announce for auto      *
 *  neighbour discovery on UDP/5000.  Mirrors the rpi4/rpi5 port so the  *
 *  three boards populate their peer tables on the shared fixed-BSSID    *
 *  cell without any driven traffic.                                     *
 * ================================================================== */
#define MN_PORT    5000
#define MN_HELLO   6
#define MN_HDR     16
#define MN_MAXNBR  64
static uint8_t   g_mn_node = 0;
static uint8_t   g_mn_peers[MN_MAXNBR];
static int       g_mn_peersn = 0;
static uint32_t  g_mn_hello_tx = 0, g_mn_hello_last = 0, g_mn_rx = 0;
static uint16_t  g_mn_seq = 0;

static void mn_peer_seen(uint8_t src)
{
    int i;
    if (!src || src == g_mn_node) return;
    for (i = 0; i < g_mn_peersn; i++) if (g_mn_peers[i] == src) return;
    if (g_mn_peersn < MN_MAXNBR) g_mn_peers[g_mn_peersn++] = src;
}

static void mn_bcast_hello(void)
{
    static uint8_t f[64];
    int i, plen = MN_HDR, tot = 14 + 20 + 8 + plen;
    uint8_t *ip, *udp, *p; uint16_t c;
    for (i = 0; i < tot; i++) f[i] = 0;
    for (i = 0; i < 6; i++) f[i] = 0xff;
    for (i = 0; i < 6; i++) f[6 + i] = wifi_mac[i];
    f[12] = 0x08; f[13] = 0x00;
    ip = f + 14;
    ip[0] = 0x45; ip[2] = (uint8_t)((20 + 8 + plen) >> 8); ip[3] = (uint8_t)(20 + 8 + plen);
    ip[8] = 64; ip[9] = 17;
    for (i = 0; i < 4; i++) ip[12 + i] = wifi_ip[i];
    for (i = 0; i < 4; i++) ip[16 + i] = 255;
    c = ip_cksum(ip, 20, 0); ip[10] = c >> 8; ip[11] = c & 0xFF;
    udp = ip + 20;
    udp[0] = (uint8_t)(MN_PORT >> 8); udp[1] = (uint8_t)MN_PORT;
    udp[2] = (uint8_t)(MN_PORT >> 8); udp[3] = (uint8_t)MN_PORT;
    udp[4] = (uint8_t)((8 + plen) >> 8); udp[5] = (uint8_t)(8 + plen);
    p = udp + 8;
    p[0] = MN_HELLO; p[1] = g_mn_node;
    p[2] = (uint8_t)g_mn_seq; p[3] = (uint8_t)(g_mn_seq >> 8); g_mn_seq++;
    wifi_data_tx(f, tot);
    g_mn_hello_tx++;
}

/* Called from the wifi_net_service responder loop; self-rate-limits to ~2s. */
static void mn_hello_tick(void)
{
    uint32_t now;
    if (!wifi_have_ip) return;
    if (!g_mn_node) g_mn_node = wifi_ip[3];
    now = SYSTIMER_CLO;
    if (g_mn_hello_last && (uint32_t)(now - g_mn_hello_last) < 2000000u) return;
    g_mn_hello_last = now;
    mn_bcast_hello();
}

void wifi_manet_get(int *node, unsigned *rx, unsigned *htx, int *np, unsigned char *peers)
{
    int i;
    if (!g_mn_node) g_mn_node = wifi_ip[3];
    if (node) *node = g_mn_node;
    if (rx)   *rx   = g_mn_rx;
    if (htx)  *htx  = g_mn_hello_tx;
    if (np)   *np   = g_mn_peersn;
    if (peers) for (i = 0; i < g_mn_peersn && i < MN_MAXNBR; i++) peers[i] = g_mn_peers[i];
}

/* Process one received 802.3 frame; reply to ARP-who-has-us and ICMP echo. */
static int aodv_ip_in(uint8_t *e, int elen);   /* M13: AODV control / relay */

static void wifi_handle_frame(uint8_t *fr, int len, int doff)
{
    int bdc = 4 + (fr[doff + 3] << 2);
    uint8_t *e = fr + doff + bdc;            /* 802.3 frame */
    int elen = len - (doff + bdc), et, i;
    static int dbgn = 0;
    if (elen < 14) return;
    et = (e[12] << 8) | e[13];
    if (dbgn < 40) {
        dbgn++;
        wifi_log("[wifi] rx et=0x%04x dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x len=%d\r\n",
                 et, e[0],e[1],e[2],e[3],e[4],e[5], e[6],e[7],e[8],e[9],e[10],e[11], elen);
    }

    if (et == 0x0806 && elen >= 42) {        /* ARP */
        uint8_t *a = e + 14;
        int op = (a[6] << 8) | a[7];
        if (dbgn < 40) wifi_log("[wifi] rx ARP op=%d tgtip=%d.%d.%d.%d (ourip=%d.%d.%d.%d)\r\n",
                                op, a[24],a[25],a[26],a[27], wifi_ip[0],wifi_ip[1],wifi_ip[2],wifi_ip[3]);
        if (op == 1 && wifi_ip_eq(a + 24)) { /* request for our IP */
            static uint8_t tx[42];
            for (i = 0; i < 6; i++) tx[i]     = e[6 + i];   /* dst = requester */
            for (i = 0; i < 6; i++) tx[6 + i] = wifi_mac[i];/* src = us */
            tx[12] = 0x08; tx[13] = 0x06;
            { uint8_t *r = tx + 14;
              r[0]=0;r[1]=1; r[2]=0x08;r[3]=0; r[4]=6;r[5]=4; r[6]=0;r[7]=2; /* reply */
              for (i=0;i<6;i++) r[8+i]  = wifi_mac[i];     /* sender mac = us */
              for (i=0;i<4;i++) r[14+i] = wifi_ip[i];      /* sender ip  = us */
              for (i=0;i<6;i++) r[18+i] = a[8+i];          /* target mac = requester */
              for (i=0;i<4;i++) r[24+i] = a[14+i];         /* target ip  = requester */
            }
            wifi_data_tx(tx, 42);
            wifi_log("[wifi] -> ARP reply to %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                     e[6],e[7],e[8],e[9],e[10],e[11]);
        }
    } else if (et == 0x0800 && elen >= 14 + 20 + 8) {   /* IPv4 */
        uint8_t *ip = e + 14;
        int ihl = (ip[0] & 0x0F) * 4;
        if (aodv_ip_in(e, elen)) return;                /* AODV control / relay */
        if (ip[9] == 17) {                              /* UDP */
            uint8_t *udp = ip + ihl;
            int dport = (udp[2] << 8) | udp[3];
            if (dport == MN_PORT && 14 + ihl + 8 + 2 <= elen) {  /* MANET HELLO */
                uint8_t src = (udp + 8)[1];
                if (src && src != g_mn_node) { g_mn_rx++; mn_peer_seen(src); }
                return;
            }
        }
        if (ip[9] == 1 && wifi_ip_eq(ip + 16)) {        /* ICMP to us */
            uint8_t *ic = ip + ihl;
            int iptot = (ip[2] << 8) | ip[3];
            int iclen = iptot - ihl;
            if (ic[0] == 8 && iclen >= 8 && 14 + iptot <= elen) {  /* echo request */
                uint8_t m[6]; uint16_t c;
                for (i=0;i<6;i++){ m[i]=e[i]; e[i]=e[6+i]; e[6+i]=m[i]; }   /* swap eth */
                for (i=0;i<4;i++){ uint8_t t=ip[12+i]; ip[12+i]=ip[16+i]; ip[16+i]=t; } /* swap IP */
                ic[0] = 0;                               /* echo reply */
                ic[2] = ic[3] = 0; c = ip_cksum(ic, iclen, 0); ic[2]=c>>8; ic[3]=c&0xFF;
                ip[10]=ip[11]=0; c = ip_cksum(ip, ihl, 0); ip[10]=c>>8; ip[11]=c&0xFF;
                wifi_data_tx(e, 14 + iptot);
                wifi_log("[wifi] -> ICMP echo reply\r\n");
            }
        }
    }
}

/* Network service loop (a thread): poll the WLAN RX FIFO and answer ARP/ICMP.
 * Started once after DHCP succeeds; sole consumer of chan-2 frames thereafter.
 * wifi_net_pause lets a client op (ping/NTP) take over the RX FIFO briefly. */
static int wifi_net_running = 0;
static volatile int wifi_net_pause = 0;
void wifi_net_service(void)
{
    static uint8_t fr[2048];
    int chan, doff;
    wifi_net_running = 1;
    wifi_log("[wifi] net service: ARP/ICMP responder up on %d.%d.%d.%d\r\n",
             wifi_ip[0], wifi_ip[1], wifi_ip[2], wifi_ip[3]);
    for (;;) {
        int n;
        if (wifi_net_pause) { wifi_delay_us(5000); continue; }  /* yield RX to a client op */
        wait(wl_io_sem);                       /* atomic read+reply vs client ops */
        n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (n > 0 && chan == 2 && n > doff + 4) wifi_handle_frame(fr, n, doff);
        mn_hello_tick();                       /* periodic MANET HELLO (2s throttle) */
        signal(wl_io_sem);
        if (n <= 0) wifi_delay_us(2000);
    }
}
int wifi_net_active(void) { return wifi_net_running; }

/* Spawn the persistent ARP/ICMP responder thread (idempotent) so this node
 * answers pings on its current wifi_ip.  Same pattern wifi_adhoc() uses; call
 * it after an AP join+DHCP so an infra-mode WiFi IP becomes reachable too. */
void wifi_serve_start(void)
{
    if (!wifi_net_active()) {
        tid_typ tid = create((void *)wifi_net_service, INITSTK, INITPRIO,
                             "wifi-net", 0);
        if (tid != SYSERR) {
            ready(tid, RESCHED_NO);
            wifi_log("[wifi] responder thread up on %d.%d.%d.%d\r\n",
                     wifi_ip[0], wifi_ip[1], wifi_ip[2], wifi_ip[3]);
        }
    }
}

/* ================================================================== *
 *  M12 — MANET ad-hoc (IBSS): create/join a cell, static IP, respond *
 * ================================================================== */
/* Port of the Pi4 wifi_adhoc(): WLC_SET_INFRA 0 (IBSS) + SET_CHANNEL +
 * SET_SSID (join_params, chanspec_num=0), static IP 10.0.0.<n>/24, then
 * spawn the ARP/ICMP responder thread so peers can reach this node. */
/* いまファームウェアが実際に居るセルの BSSID を聞く。
   ★ 局所変数（wifi_have_ip / wifi_cur_ssid）は「代入したかどうか」しか表さない。
     ad-hoc では IP を静的に置くだけで status が「接続済み」になってしまい、
     無線が associate していなくても嘘をつく。実物を聞くこと。
   戻り: 1=セルに居る（bss に BSSID）/ 0=居ない */
int wifi_live_bssid(uint8_t *bss)
{
    int k, nz = 0;
    if (!wifi_ready) return 0;
    if (wifi_wlcmd(0, WLC_GET_BSSID, NULL, 0, bss, 6) != 0) return 0;
    for (k = 0; k < 6; k++) if (bss[k] && bss[k] != 0xFF) nz = 1;
    return nz;
}

/* ad-hoc 各段の結果。★ これまで WLC_SET_INFRA / WLC_SET_CHANNEL /
   WLC_SET_SSID の戻り値を誰も見ておらず、黙って失敗しても「ad-hoc up」と
   表示していた。どこで落ちているかを外から読めるようにする。 */
static int g_ah_infra_rc = -99, g_ah_infra_rb = -99, g_ah_chan_rc = -99;
static int g_ah_ssid_rc  = -99, g_ah_up = -99, g_ah_up_tries = -1;
void wifi_adhoc_diag(int *infra_rc, int *infra_rb, int *chan_rc,
                     int *ssid_rc, int *up, int *tries)
{
    if (infra_rc) *infra_rc = g_ah_infra_rc;
    if (infra_rb) *infra_rb = g_ah_infra_rb;
    if (chan_rc)  *chan_rc  = g_ah_chan_rc;
    if (ssid_rc)  *ssid_rc  = g_ah_ssid_rc;
    if (up)       *up       = g_ah_up;
    if (tries)    *tries    = g_ah_up_tries;
}

int wifi_adhoc(const char *ssid, int channel, int n)
{
    /* ★ jp は static（.bss）に置く ―― Pi 4 の実装がそうなっている。
       この配列は SDIO の転送バッファとしてそのまま渡る。スタック上に置くと
       整列や置き場所の前提が崩れうる。Pi 4 は ad-hoc セルを作れて Pi 3 は
       作れない、という差の候補はここだけだった（他の差は zero 埋め済みの
       重複と、bringup の呼び分けのみ）。 */
    static uint8_t jp[64];
    uint8_t bss[6];
    int sl = 0, i, t, up = 0;
    while (ssid[sl] && sl < 32) sl++;
    wifi_log("[wifi] === ADHOC/IBSS \"%s\" ch=%d ip=10.0.0.%d ===\r\n", ssid, channel, n);
    /* 既に上がっていれば bring-up をやり直さない（Pi 4 と同じ）。毎回やり直すと
       直前に整えた状態を落としうる。 */
    if (!wifi_ready) { if (wifi_bringup() != 0) { wifi_log("[wifi] adhoc: bringup failed\r\n"); return -1; } }
    wifi_radio_up();
    wifi_cmd_int(WLC_DOWN, 1);
    wifi_set_iovar_int("wsec", 0);
    wifi_set_iovar_int("wpa_auth", 0);
    wifi_set_iovar_int("auth", 0);
    g_ah_infra_rc = wifi_cmd_int(WLC_SET_INFRA, 0);   /* IBSS (ad-hoc) mode */
    wifi_cmd_int(2, 1);                          /* WLC_UP */
    wifi_delay_us(100000);
    g_ah_chan_rc = wifi_cmd_int(WLC_SET_CHANNEL, (uint32_t)channel);
    /* 設定が本当に入ったか読み返す（1 なら infra のまま＝IBSS になっていない） */
    { uint32_t v = 0xdeadbeef;
      if (wifi_wlcmd(0, WLC_GET_INFRA, NULL, 0, (uint8_t *)&v, 4) == 0)
          g_ah_infra_rb = (int)v;
      else g_ah_infra_rb = -1; }
    for (i = 0; i < (int)sizeof(jp); i++) jp[i] = 0;
    jp[0] = sl; for (i = 0; i < sl; i++) jp[4+i] = ssid[i];
    /* ★ BSSID はブロードキャストにする ―― 「この SSID のセルがあれば参加、
       無ければ作る」の意味。以前は 02:4d:41:4e:45:54 ("MANET") を指定していたが、
       あれは「ちょうどこの BSSID のセルに参加せよ」であり、そんなセルは誰も
       作らない（IBSS を新規作成するとき、ファームウェアは要求された BSSID を
       使わず、ローカル管理ビットの立った値を自分で作る）。結果、全ノードが
       自分だけのセルを作って永久に合流しなかった。実測でも毎回別の値が返った:
       8a:58:5a:… / 32:e8:8f:… / 9e:5a:71:… */
    for (i = 36; i < 42; i++) jp[i] = 0xFF;
    jp[44]=0;                                    /* chanspec_num = 0 */
    g_ah_ssid_rc = wifi_wlcmd(1, WLC_SET_SSID, jp, 48, NULL, 0);
    if (g_ah_ssid_rc != 0) { wifi_log("[wifi] adhoc: SET_SSID failed\r\n"); return -1; }
    wifi_delay_us(400000);
    wifi_get_iovar("cur_etheraddr", wifi_mac, 6);
    wifi_ip[0]=10; wifi_ip[1]=0; wifi_ip[2]=0; wifi_ip[3]=(uint8_t)n;
    wifi_mask[0]=255; wifi_mask[1]=255; wifi_mask[2]=255; wifi_mask[3]=0;
    wifi_gw[0]=10; wifi_gw[1]=0; wifi_gw[2]=0; wifi_gw[3]=1;
    for (i = 0; i < 4; i++) wifi_dns[i] = 0;
    for (i = 0; i < sl && i < 38; i++) wifi_cur_ssid[i] = ssid[i]; wifi_cur_ssid[i] = 0;
    wifi_have_ip = 1;
    g_ah_up = 0; g_ah_up_tries = 0;
    for (t = 0; t < 24; t++) {
        g_ah_up_tries = t + 1;
        if (wifi_wlcmd(0, WLC_GET_BSSID, NULL, 0, bss, 6) == 0) {
            int k, nz = 0; for (k = 0; k < 6; k++) if (bss[k] && bss[k] != 0xFF) nz = 1;
            if (nz) { up = 1; g_ah_up = 1;
                wifi_log("[wifi] adhoc: *** IBSS cell up, BSSID %02x:%02x:%02x:%02x:%02x:%02x ***\r\n",
                         bss[0],bss[1],bss[2],bss[3],bss[4],bss[5]); break; }
        }
        wifi_delay_us(500000);
    }
    if (!up) wifi_log("[wifi] adhoc: IBSS not associated yet (no peer / still forming)\r\n");
    /* ★ associate できていないなら have_ip を立てない。立てると status が
       「connected」と嘘をつき、切り分けを誤らせる（実際そうなった）。 */
    if (!up) wifi_have_ip = 0;
    wifi_log("[wifi] mac %02x:%02x:%02x:%02x:%02x:%02x  ip=10.0.0.%d\r\n",
             wifi_mac[0],wifi_mac[1],wifi_mac[2],wifi_mac[3],wifi_mac[4],wifi_mac[5], n);
    if (!wifi_net_active()) {                     /* spawn ARP/ICMP responder thread */
        tid_typ tid = create((void *)wifi_net_service, INITSTK, INITPRIO, "wifi-net", 0);
        if (tid != SYSERR) { ready(tid, RESCHED_NO); wifi_log("[wifi] adhoc: responder thread up\r\n"); }
    }
    wifi_log("[wifi] *** ADHOC up: \"%s\" ch=%d ip=10.0.0.%d (peer-to-peer, no AP) ***\r\n", ssid, channel, n);
    return 0;
}

/* Resolve an IP to a MAC via ARP.  PRECONDITION: the net thread is paused
 * (wifi_net_pause=1) so we own the RX FIFO.  Returns 0 + mac on success. */
static int wifi_arp_resolve(const uint8_t *ip, uint8_t *mac)
{
    static uint8_t fr[2048], tx[42];
    int i, chan, doff, tries, w;
    for (tries = 0; tries < 12; tries++) {
        for (i = 0; i < 6; i++) tx[i] = 0xFF;          /* dst bcast */
        for (i = 0; i < 6; i++) tx[6 + i] = wifi_mac[i];
        tx[12] = 0x08; tx[13] = 0x06;
        { uint8_t *a = tx + 14;
          a[0]=0;a[1]=1;a[2]=0x08;a[3]=0;a[4]=6;a[5]=4;a[6]=0;a[7]=1;   /* request */
          for (i=0;i<6;i++) a[8+i]  = wifi_mac[i];     /* sender mac */
          for (i=0;i<4;i++) a[14+i] = wifi_ip[i];      /* sender ip */
          for (i=0;i<6;i++) a[18+i] = 0;               /* target mac unknown */
          for (i=0;i<4;i++) a[24+i] = ip[i];           /* target ip */
        }
        wifi_data_tx(tx, 42);
        for (w = 0; w < 40; w++) {
            int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
            if (n <= 0) { wifi_delay_us(5000); continue; }
            if (chan != 2 || n < doff + 4) continue;
            { int bdc = 4 + (fr[doff+3]<<2); uint8_t *e = fr+doff+bdc; int el = n-(doff+bdc);
              if (el >= 42 && e[12]==0x08 && e[13]==0x06) {
                uint8_t *a = e + 14; int op = (a[6]<<8)|a[7];
                if (op == 2 && a[14]==ip[0]&&a[15]==ip[1]&&a[16]==ip[2]&&a[17]==ip[3]) {
                    for (i=0;i<6;i++) mac[i] = a[8+i];
                    return 0;
                }
              }
            }
        }
    }
    return -1;
}

/* Decide the next-hop MAC for a destination IP: ARP the host if it is on our
 * subnet, otherwise ARP the default gateway (off-subnet -> route via gw). */
static int wifi_nexthop_mac(const uint8_t *dst, uint8_t *mac)
{
    int i, onsub = 1;
    for (i = 0; i < 4; i++)
        if ((dst[i] & wifi_mask[i]) != (wifi_ip[i] & wifi_mask[i])) onsub = 0;
    return wifi_arp_resolve(onsub ? dst : wifi_gw, mac);
}

/* ================================================================== *
 *  M13 — minimal AODV multi-hop routing (port of the Pi4 module)    *
 * ================================================================== */
static void wifi_udp_tx(const uint8_t *nh, const uint8_t *dip, int sport, int dport, const uint8_t *p, int plen)
{
    static uint8_t tx[1600];
    int i, udplen = 8 + plen, iptot = 20 + udplen, framelen = 14 + iptot;
    uint8_t *ip4 = tx + 14, *udp;
    if (framelen > (int)sizeof(tx)) return;
    for (i=0;i<6;i++) tx[i]=nh[i];
    for (i=0;i<6;i++) tx[6+i]=wifi_mac[i];
    tx[12]=0x08; tx[13]=0x00;
    for (i=0;i<20;i++) ip4[i]=0;
    ip4[0]=0x45; ip4[2]=iptot>>8; ip4[3]=iptot&0xFF; ip4[5]=1; ip4[8]=64; ip4[9]=17;
    for (i=0;i<4;i++) ip4[12+i]=wifi_ip[i];
    for (i=0;i<4;i++) ip4[16+i]=dip[i];
    { uint16_t c=ip_cksum(ip4,20,0); ip4[10]=c>>8; ip4[11]=c&0xFF; }
    udp = ip4+20;
    udp[0]=sport>>8; udp[1]=sport&0xFF; udp[2]=dport>>8; udp[3]=dport&0xFF;
    udp[4]=udplen>>8; udp[5]=udplen&0xFF; udp[6]=0; udp[7]=0;
    for (i=0;i<plen;i++) udp[8+i]=p[i];
    wifi_data_tx(tx, framelen);
}
#define AODV_PORT  654
#define AODV_RREQ  1
#define AODV_RREP  2
#define AODV_NRT   16
struct aodv_rt { uint8_t dst[4], nh[4]; uint8_t hops; uint32_t seq; int valid; };
static struct aodv_rt g_rt[AODV_NRT];
static uint16_t g_rreqid = 0; static uint32_t g_myseq = 0;
static uint8_t g_seen_o[8][4]; static uint16_t g_seen_id[8]; static int g_seen_n = 0;
static int ip4eq(const uint8_t *a, const uint8_t *b){ return a[0]==b[0]&&a[1]==b[1]&&a[2]==b[2]&&a[3]==b[3]; }
static struct aodv_rt *aodv_find(const uint8_t *d){ int i; for(i=0;i<AODV_NRT;i++) if(g_rt[i].valid&&ip4eq(g_rt[i].dst,d)) return &g_rt[i]; return 0; }
static void aodv_add(const uint8_t *d, const uint8_t *nh, uint8_t hops, uint32_t seq){
    struct aodv_rt *r = aodv_find(d); int i;
    if(!r){ for(i=0;i<AODV_NRT;i++) if(!g_rt[i].valid){ r=&g_rt[i]; break; } }
    if(!r) r=&g_rt[0];
    if(r->valid && ip4eq(r->dst,d) && r->hops <= hops) return;
    for(i=0;i<4;i++){ r->dst[i]=d[i]; r->nh[i]=nh[i]; } r->hops=hops; r->seq=seq; r->valid=1;
}
static int aodv_seen(const uint8_t *o, uint16_t id){ int i; for(i=0;i<g_seen_n&&i<8;i++) if(g_seen_id[i]==id&&ip4eq(g_seen_o[i],o)) return 1; return 0; }
static void aodv_remember(const uint8_t *o, uint16_t id){ int s=g_seen_n%8,i; for(i=0;i<4;i++) g_seen_o[s][i]=o[i]; g_seen_id[s]=id; g_seen_n++; }
static void aodv_bcast(const uint8_t *p, int n){ uint8_t bc[6]={0xff,0xff,0xff,0xff,0xff,0xff}, bip[4]={255,255,255,255}; wifi_udp_tx(bc, bip, AODV_PORT, AODV_PORT, p, n); }
static void aodv_send_rreq(const uint8_t *dst){
    uint8_t p[16]; int i;
    g_rreqid++; g_myseq++;
    p[0]=AODV_RREQ; p[1]=g_rreqid; p[2]=g_rreqid>>8;
    for(i=0;i<4;i++) p[3+i]=wifi_ip[i];
    for(i=0;i<4;i++) p[7+i]=dst[i];
    p[11]=g_myseq; p[12]=g_myseq>>8; p[13]=g_myseq>>16; p[14]=g_myseq>>24; p[15]=0;
    aodv_remember(wifi_ip, g_rreqid);
    wifi_log("[wifi] aodv: RREQ id=%d for %d.%d.%d.%d\r\n", g_rreqid, dst[0],dst[1],dst[2],dst[3]);
    aodv_bcast(p, 16);
}
static int aodv_ip_in(uint8_t *e, int elen)
{
    uint8_t *ip = e+14; int ihl=(ip[0]&0xF)*4, i; uint8_t ipsrc[4], ipdst[4], *l2src = e+6;
    for(i=0;i<4;i++){ ipsrc[i]=ip[12+i]; ipdst[i]=ip[16+i]; }
    if (ip[9]==17) {
        uint8_t *udp = ip+ihl; int dport=(udp[2]<<8)|udp[3];
        if (dport != AODV_PORT) return 0;
        { uint8_t *p = udp+8; int type=p[0]; uint8_t orig[4], dst[4], hop=p[15]; uint32_t sq;
          for(i=0;i<4;i++){ orig[i]=p[3+i]; dst[i]=p[7+i]; }
          sq = p[11]|(p[12]<<8)|(p[13]<<16)|((uint32_t)p[14]<<24);
          if (type==AODV_RREQ) {
              uint16_t rid = p[1]|(p[2]<<8);
              if (aodv_seen(orig, rid)) return 1;
              aodv_remember(orig, rid);
              aodv_add(orig, ipsrc, hop+1, sq);
              if (ip4eq(dst, wifi_ip)) {
                  uint8_t r[16]; g_myseq++;
                  r[0]=AODV_RREP; for(i=0;i<4;i++){ r[3+i]=orig[i]; r[7+i]=wifi_ip[i]; }
                  r[11]=g_myseq; r[12]=g_myseq>>8; r[13]=g_myseq>>16; r[14]=g_myseq>>24; r[15]=0;
                  wifi_udp_tx(l2src, ipsrc, AODV_PORT, AODV_PORT, r, 16);
                  wifi_log("[wifi] aodv: RREQ for me from %d.%d.%d.%d -> RREP\r\n", orig[0],orig[1],orig[2],orig[3]);
              } else { p[15]=hop+1; aodv_bcast(p, 16); }
              return 1;
          } else if (type==AODV_RREP) {
              aodv_add(dst, ipsrc, hop+1, sq);
              if (ip4eq(orig, wifi_ip)) {
                  wifi_log("[wifi] aodv: *** route to %d.%d.%d.%d via %d.%d.%d.%d (%d hop) ***\r\n",
                           dst[0],dst[1],dst[2],dst[3], ipsrc[0],ipsrc[1],ipsrc[2],ipsrc[3], hop+1);
              } else {
                  struct aodv_rt *rr = aodv_find(orig); uint8_t mac[6];
                  if (rr && wifi_arp_resolve(rr->nh, mac)==0) { p[15]=hop+1; wifi_udp_tx(mac, rr->nh, AODV_PORT, AODV_PORT, p, 16); }
              }
              return 1;
          } }
        return 1;
    }
    if (!ip4eq(ipdst, wifi_ip) && ipdst[0] != 255) {
        struct aodv_rt *r = aodv_find(ipdst); uint8_t mac[6];
        if (r && wifi_arp_resolve(r->nh, mac)==0) {
            for(i=0;i<6;i++) e[i]=mac[i];
            for(i=0;i<6;i++) e[6+i]=wifi_mac[i];
            wifi_data_tx(e, elen);
            wifi_log("[wifi] aodv: relay %d.%d.%d.%d -> nh %d.%d.%d.%d\r\n",
                     ipdst[0],ipdst[1],ipdst[2],ipdst[3], r->nh[0],r->nh[1],r->nh[2],r->nh[3]);
        }
        return 1;
    }
    return 0;
}
/* Originator: discover a route (RREQ + wait for RREP, reading frames). */
int wifi_aodv(const uint8_t *dst)
{
    static uint8_t fr[2048]; int chan, doff, t, r2;
    wifi_log("[wifi] === AODV discover %d.%d.%d.%d ===\r\n", dst[0],dst[1],dst[2],dst[3]);
    if (!wifi_have_ip) { wifi_log("[wifi] aodv: no IP (run wifi adhoc first)\r\n"); return -1; }
    if (aodv_find(dst)) { wifi_log("[wifi] aodv: route already known\r\n"); return 0; }
    for (r2 = 0; r2 < 4; r2++) {
        aodv_send_rreq(dst);
        for (t = 0; t < 80; t++) {
            int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
            if (n > 0 && chan == 2 && n > doff + 4) wifi_handle_frame(fr, n, doff);
            else wifi_delay_us(5000);
            if (aodv_find(dst)) { struct aodv_rt *r = aodv_find(dst);
                wifi_log("[wifi] *** AODV route: %d.%d.%d.%d via %d.%d.%d.%d, %d hop ***\r\n",
                         dst[0],dst[1],dst[2],dst[3], r->nh[0],r->nh[1],r->nh[2],r->nh[3], r->hops);
                return 0; }
        }
    }
    wifi_log("[wifi] aodv: no route (RREP timeout)\r\n");
    return -1;
}

/* ICMP echo client: ping `ip` `count` times.  Returns # replies. */
int wifi_ping(const uint8_t *ip, int count)
{
    static uint8_t fr[2048], tx[128];
    uint8_t nh[6];
    int i, chan, doff, seq, w, replies = 0, paylen = 32;
    uint16_t c;
    wifi_tn = 0;
    wifi_log("[wifi] === PING %d.%d.%d.%d (%d) ===\r\n", ip[0],ip[1],ip[2],ip[3], count);
    if (!wifi_have_ip) { wifi_log("[wifi] ping: no IP yet\r\n"); return -1; }
    wifi_net_pause = 1; wifi_delay_us(40000); wait(wl_io_sem);  /* own the RX FIFO */
    if (wifi_nexthop_mac(ip, nh) != 0) {
        wifi_log("[wifi] ping: ARP/next-hop failed\r\n");
        signal(wl_io_sem); wifi_net_pause = 0; return -1;
    }
    wifi_log("[wifi] ping: next-hop %02x:%02x:%02x:%02x:%02x:%02x\r\n",
             nh[0],nh[1],nh[2],nh[3],nh[4],nh[5]);
    for (seq = 0; seq < count; seq++) {
        int icmplen = 8 + paylen, iptot = 20 + icmplen, framelen = 14 + iptot, rcvd = 0;
        for (i=0;i<6;i++) tx[i] = nh[i];
        for (i=0;i<6;i++) tx[6+i] = wifi_mac[i];
        tx[12]=0x08; tx[13]=0x00;
        { uint8_t *ip4 = tx + 14, *ic;
          for (i=0;i<20;i++) ip4[i]=0;
          ip4[0]=0x45; ip4[2]=iptot>>8; ip4[3]=iptot&0xFF; ip4[5]=seq+1; ip4[8]=64; ip4[9]=1;
          for (i=0;i<4;i++) ip4[12+i]=wifi_ip[i];
          for (i=0;i<4;i++) ip4[16+i]=ip[i];
          c=ip_cksum(ip4,20,0); ip4[10]=c>>8; ip4[11]=c&0xFF;
          ic = ip4 + 20;
          for (i=0;i<icmplen;i++) ic[i]=0;
          ic[0]=8; ic[4]=0xBE; ic[5]=0xEF; ic[6]=seq>>8; ic[7]=seq&0xFF;
          for (i=0;i<paylen;i++) ic[8+i]=(uint8_t)(0x61+(i%26));
          c=ip_cksum(ic,icmplen,0); ic[2]=c>>8; ic[3]=c&0xFF;
        }
        wifi_data_tx(tx, framelen);
        for (w = 0; w < 60 && !rcvd; w++) {
            int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
            if (n <= 0) { wifi_delay_us(5000); continue; }
            if (chan != 2 || n < doff + 4) continue;
            { int bdc = 4 + (fr[doff+3]<<2); uint8_t *e = fr+doff+bdc; int el = n-(doff+bdc);
              if (el >= 14+20+8 && e[12]==0x08 && e[13]==0x00) {
                uint8_t *ip4 = e+14; int ihl = (ip4[0]&0xF)*4; uint8_t *ic = ip4+ihl;
                if (ip4[9]==1 && ip4[12]==ip[0]&&ip4[13]==ip[1]&&ip4[14]==ip[2]&&ip4[15]==ip[3]
                    && ic[0]==0 && (((ic[6]<<8)|ic[7])==seq)) rcvd = 1;
              }
            }
        }
        if (rcvd) { replies++; wifi_log("[wifi] ping: reply seq=%d\r\n", seq); }
        else        wifi_log("[wifi] ping: timeout seq=%d\r\n", seq);
        wifi_delay_us(300000);
    }
    signal(wl_io_sem); wifi_net_pause = 0;
    wifi_log("[wifi] *** ping %d.%d.%d.%d: %d/%d ***\r\n", ip[0],ip[1],ip[2],ip[3], replies, count);
    return replies;
}

/* Broadcast a few gratuitous ARPs announcing our IP->MAC so the peer /
 * switch refreshes a stuck "incomplete" ARP entry and can reach us. */
static void wifi_grat_arp(void)
{
    uint8_t tx[42];
    int i, rep;
    wifi_net_pause = 1; wifi_delay_us(40000); wait(wl_io_sem);
    for (rep = 0; rep < 3; rep++) {
        for (i = 0; i < 6; i++) tx[i]   = 0xFF;            /* dest broadcast */
        for (i = 0; i < 6; i++) tx[6+i] = wifi_mac[i];     /* src = us       */
        tx[12] = 0x08; tx[13] = 0x06;                      /* ethertype ARP  */
        tx[14] = 0x00; tx[15] = 0x01;                      /* htype ethernet */
        tx[16] = 0x08; tx[17] = 0x00;                      /* ptype IPv4     */
        tx[18] = 6;    tx[19] = 4;                          /* hlen / plen    */
        tx[20] = 0x00; tx[21] = 0x02;                      /* op = reply     */
        for (i = 0; i < 6; i++) tx[22+i] = wifi_mac[i];    /* sender mac     */
        for (i = 0; i < 4; i++) tx[28+i] = wifi_ip[i];     /* sender ip      */
        for (i = 0; i < 6; i++) tx[32+i] = 0xFF;           /* target mac     */
        for (i = 0; i < 4; i++) tx[38+i] = wifi_ip[i];     /* target ip = us */
        wifi_data_tx(tx, 42);
        wifi_delay_us(50000);
    }
    signal(wl_io_sem); wifi_net_pause = 0;
}

/* Investigate "connected but ping fails": make sure the responder is up,
 * refresh the peer's ARP (gratuitous ARP), then probe the gateway from the
 * Pi3.  Writes a human-readable report (+ remedy / reason) into `out`. */
int wifi_investigate(char *out, int cap)
{
    int len = 0, gw;
    (void)cap;
    if (!wifi_have_ip)
        return sprintf(out, "WiFi not connected -- run 'wifi on' first.\n");

    len += sprintf(out + len,
        "ip %u.%u.%u.%u  gw %u.%u.%u.%u  mac %02x:%02x:%02x:%02x:%02x:%02x\n",
        wifi_ip[0], wifi_ip[1], wifi_ip[2], wifi_ip[3],
        wifi_gw[0], wifi_gw[1], wifi_gw[2], wifi_gw[3],
        wifi_mac[0], wifi_mac[1], wifi_mac[2],
        wifi_mac[3], wifi_mac[4], wifi_mac[5]);

    if (!wifi_net_active()) {
        wifi_net_service();
        len += sprintf(out + len, "- responder was down -> started it\n");
    } else {
        len += sprintf(out + len, "- ARP/ICMP responder: running\n");
    }

    /* Keep refreshing the peer's ARP and probing the gateway until the
     * data path comes up, then stop by ourselves.  Bounded (≈20s max) so
     * it always terminates even when it cannot be fixed. */
    {
        const int MAXATT = 8;
        int att;
        for (att = 1; att <= MAXATT; att++) {
            wifi_grat_arp();
            gw = wifi_ping(wifi_gw, 2);
            len += sprintf(out + len, "- try %d/%d: grat-ARP + gw ping = %d/2\n",
                           att, MAXATT, gw > 0 ? gw : 0);
            if (gw > 0) break;          /* data path is up -> finished */
        }
    }

    if (gw > 0) {
        len += sprintf(out + len,
            "RESULT: WiFi data path is UP (gateway reachable); the peer\n"
            "ARP was refreshed -- ping from your Mac should work now.\n"
            "(auto-finished on success)\n");
    } else {
        len += sprintf(out + len,
            "RESULT: still no gateway after retries.  Either the BCM43455\n"
            "data path is stalled ('wifi off' then 'wifi on', or power-\n"
            "cycle), or the router isolates WiFi clients from the wired Mac.\n");
    }
    return len;
}

/* NTP time client: query an NTP server (UDP/123) through the gateway and
 * return the Unix time (seconds since 1970), or 0 on failure.  `srv` is the
 * NTP server IPv4 (off-subnet -> routed via the default gateway). */
unsigned long wifi_ntp(const uint8_t *srv)
{
    static uint8_t fr[2048], tx[90];
    uint8_t nh[6];
    int i, chan, doff, w;
    uint16_t c;
    unsigned long secs = 0;
    wifi_tn = 0;
    wifi_log("[wifi] === NTP %d.%d.%d.%d ===\r\n", srv[0],srv[1],srv[2],srv[3]);
    if (!wifi_have_ip) { wifi_log("[wifi] ntp: no IP yet\r\n"); return 0; }
    wifi_net_pause = 1; wifi_delay_us(40000); wait(wl_io_sem);
    if (wifi_nexthop_mac(srv, nh) != 0) {
        wifi_log("[wifi] ntp: next-hop ARP failed\r\n");
        signal(wl_io_sem); wifi_net_pause = 0; return 0;
    }
    /* eth + IP(20) + UDP(8) + NTP(48) */
    { int udplen = 8 + 48, iptot = 20 + udplen, framelen = 14 + iptot;
      uint8_t *ip4 = tx + 14, *udp, *ntp;
      for (i=0;i<6;i++) tx[i] = nh[i];
      for (i=0;i<6;i++) tx[6+i] = wifi_mac[i];
      tx[12]=0x08; tx[13]=0x00;
      for (i=0;i<framelen-14;i++) ip4[i]=0;
      ip4[0]=0x45; ip4[2]=iptot>>8; ip4[3]=iptot&0xFF; ip4[5]=1; ip4[8]=64; ip4[9]=17;
      for (i=0;i<4;i++) ip4[12+i]=wifi_ip[i];
      for (i=0;i<4;i++) ip4[16+i]=srv[i];
      c=ip_cksum(ip4,20,0); ip4[10]=c>>8; ip4[11]=c&0xFF;
      udp = ip4 + 20;
      udp[0]=0x00; udp[1]=0x7b;          /* sport 123 */
      udp[2]=0x00; udp[3]=0x7b;          /* dport 123 */
      udp[4]=udplen>>8; udp[5]=udplen&0xFF; udp[6]=0; udp[7]=0;   /* csum 0 */
      ntp = udp + 8;
      ntp[0] = 0x1b;                     /* LI=0 VN=3 Mode=3 (client) */
      wifi_data_tx(tx, framelen);
    }
    for (w = 0; w < 120; w++) {
        int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (n <= 0) { wifi_delay_us(5000); continue; }
        if (chan != 2 || n < doff + 4) continue;
        { int bdc = 4 + (fr[doff+3]<<2); uint8_t *e = fr+doff+bdc; int el = n-(doff+bdc);
          if (el >= 14+20+8+48 && e[12]==0x08 && e[13]==0x00) {
            uint8_t *ip4 = e+14; int ihl = (ip4[0]&0xF)*4; uint8_t *udp = ip4+ihl, *ntp = udp+8;
            if (ip4[9]==17 && udp[2]==0x00 && udp[3]==0x7b
                && ip4[12]==srv[0]&&ip4[13]==srv[1]&&ip4[14]==srv[2]&&ip4[15]==srv[3]) {
                /* NTP transmit timestamp: bytes 40..43 = seconds since 1900 */
                unsigned long ntp_secs = ((unsigned long)ntp[40]<<24)|((unsigned long)ntp[41]<<16)
                                       | ((unsigned long)ntp[42]<<8)|ntp[43];
                secs = ntp_secs - 2208988800UL;   /* 1900 -> 1970 epoch */
                break;
            }
          }
        }
    }
    signal(wl_io_sem); wifi_net_pause = 0;
    if (secs) {
        /* format the JST (UTC+9) civil date/time (Howard Hinnant's algorithm) */
        unsigned long jst = secs + 9UL*3600;
        long z = (long)(jst / 86400) + 719468;
        long era = (z >= 0 ? z : z - 146096) / 146097;
        unsigned doe = (unsigned)(z - era*146097);
        unsigned yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;
        long yy = (long)yoe + era*400;
        unsigned doy = doe - (365*yoe + yoe/4 - yoe/100);
        unsigned mp = (5*doy + 2)/153;
        unsigned dd = doy - (153*mp+2)/5 + 1;
        unsigned mm = mp < 10 ? mp+3 : mp-9;
        int Y = (int)(yy + (mm <= 2)), tod = (int)(jst % 86400);
        wifi_log("[wifi] *** NTP unix=%lu  JST=%04d-%02u-%02u %02d:%02d:%02d ***\r\n",
                 secs, Y, mm, dd, tod/3600, (tod%3600)/60, tod%60);
    } else {
        wifi_log("[wifi] ntp: no reply (timeout)\r\n");
    }
    return secs;
}

/* ------------------------------------------------------------------ *
 *  Minimal TCP + HTTP/1.0 client (text "browser") over the WLAN.      *
 * ------------------------------------------------------------------ */
static char     wifi_http_buf[12288];  /* fetched response (headers+body) */
static int      wifi_http_len = 0;
int wifi_http_get_buf(char **p) { *p = wifi_http_buf; return wifi_http_len; }

/* Send one TCP segment (eth+IP+TCP[+data]) to dip via next-hop nh. */
static void wifi_tcp_send(const uint8_t *nh, const uint8_t *dip, int sport, int dport,
                          uint32_t seq, uint32_t ack, int flags,
                          const uint8_t *data, int dlen)
{
    static uint8_t tx[700];
    int i, tcplen = 20 + dlen, iptot = 20 + tcplen, framelen = 14 + iptot;
    uint8_t *ip4, *tcp; uint32_t psum; uint16_t c;
    if (framelen > (int)sizeof(tx)) return;
    for (i = 0; i < 6; i++) tx[i] = nh[i];
    for (i = 0; i < 6; i++) tx[6 + i] = wifi_mac[i];
    tx[12] = 0x08; tx[13] = 0x00;
    ip4 = tx + 14;
    for (i = 0; i < 20; i++) ip4[i] = 0;
    ip4[0]=0x45; ip4[2]=iptot>>8; ip4[3]=iptot&0xFF; ip4[5]=1; ip4[8]=64; ip4[9]=6; /* TCP */
    for (i = 0; i < 4; i++) ip4[12+i] = wifi_ip[i];
    for (i = 0; i < 4; i++) ip4[16+i] = dip[i];
    c = ip_cksum(ip4, 20, 0); ip4[10]=c>>8; ip4[11]=c&0xFF;
    tcp = ip4 + 20;
    for (i = 0; i < 20; i++) tcp[i] = 0;
    tcp[0]=sport>>8; tcp[1]=sport&0xFF; tcp[2]=dport>>8; tcp[3]=dport&0xFF;
    tcp[4]=seq>>24; tcp[5]=seq>>16; tcp[6]=seq>>8; tcp[7]=seq;
    tcp[8]=ack>>24; tcp[9]=ack>>16; tcp[10]=ack>>8; tcp[11]=ack;
    tcp[12]=5<<4; tcp[13]=flags; tcp[14]=0x20; tcp[15]=0x00;  /* data-off 5, window 8192 */
    for (i = 0; i < dlen; i++) tcp[20+i] = data[i];
    psum = ((uint32_t)wifi_ip[0]<<8|wifi_ip[1]) + ((uint32_t)wifi_ip[2]<<8|wifi_ip[3])
         + ((uint32_t)dip[0]<<8|dip[1]) + ((uint32_t)dip[2]<<8|dip[3]) + 6 + tcplen;
    c = ip_cksum(tcp, tcplen, psum); tcp[16]=c>>8; tcp[17]=c&0xFF;
    wifi_data_tx(tx, framelen);
}

/* HTTP/1.0 GET http://<host>/ at `ip`:80.  Stores the response (capped) in
 * wifi_http_buf and mirrors it to the serial console (Xinu's screen). */
int wifi_http(const uint8_t *ip, const char *host)
{
    static uint8_t fr[2048];
    static char req[256];
    uint8_t nh[6];
    uint32_t iss = 0x015A0000, our_seq, our_ack = 0, their_seq;
    int chan, doff, w, sport = 0xC0DE, reqn, got_synack = 0, fin = 0, idle = 0;
    wifi_http_len = 0;
    wifi_tn = 0;
    wifi_log("[wifi] === HTTP GET http://%s/  (%d.%d.%d.%d:80) ===\r\n",
             host, ip[0],ip[1],ip[2],ip[3]);
    if (!wifi_have_ip) { wifi_log("[wifi] http: no IP yet\r\n"); return -1; }
    wifi_net_pause = 1; wifi_delay_us(40000); wait(wl_io_sem);
    if (wifi_nexthop_mac(ip, nh) != 0) {
        wifi_log("[wifi] http: next-hop ARP failed\r\n");
        signal(wl_io_sem); wifi_net_pause = 0; return -1;
    }

    /* --- TCP 3-way handshake: SYN -> SYN/ACK -> ACK --- */
    wifi_tcp_send(nh, ip, sport, 80, iss, 0, 0x02, NULL, 0);   /* SYN */
    for (w = 0; w < 200 && !got_synack; w++) {
        int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (n <= 0) { wifi_delay_us(5000); continue; }
        if (chan != 2 || n < doff + 4) continue;
        { int bdc = 4 + (fr[doff+3]<<2); uint8_t *e = fr+doff+bdc; int el = n-(doff+bdc);
          if (el >= 14+20+20 && e[12]==0x08 && e[13]==0x00) {
            uint8_t *ip4 = e+14; int ihl=(ip4[0]&0xF)*4; uint8_t *tcp = ip4+ihl;
            int dport = (tcp[2]<<8)|tcp[3];
            if (ip4[9]==6 && dport==sport && (tcp[13] & 0x12)==0x12) {  /* SYN+ACK */
                their_seq = ((uint32_t)tcp[4]<<24)|((uint32_t)tcp[5]<<16)|((uint32_t)tcp[6]<<8)|tcp[7];
                got_synack = 1;
            }
          }
        }
    }
    if (!got_synack) { wifi_log("[wifi] http: no SYN-ACK (timeout)\r\n");
        signal(wl_io_sem); wifi_net_pause = 0; return -1; }
    our_seq = iss + 1; our_ack = their_seq + 1;
    wifi_tcp_send(nh, ip, sport, 80, our_seq, our_ack, 0x10, NULL, 0);  /* ACK */
    wifi_log("[wifi] http: connected, sending GET\r\n");

    /* --- send the request --- */
    reqn = sprintf(req, "GET / HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", host);
    wifi_tcp_send(nh, ip, sport, 80, our_seq, our_ack, 0x18, (uint8_t*)req, reqn); /* PSH|ACK */
    our_seq += reqn;

    /* --- receive loop: ACK in-order data, accumulate, stop on FIN --- */
    for (w = 0; w < 2000 && !fin; w++) {
        int n = wifi_read_frame(fr, sizeof(fr), &chan, &doff);
        if (n <= 0) { wifi_delay_us(3000); if (++idle > 400) break; continue; }
        idle = 0;
        if (chan != 2 || n < doff + 4) continue;
        { int bdc = 4 + (fr[doff+3]<<2); uint8_t *e = fr+doff+bdc; int el = n-(doff+bdc);
          if (el < 14+20+20 || e[12]!=0x08 || e[13]!=0x00) continue;
          { uint8_t *ip4 = e+14; int ihl=(ip4[0]&0xF)*4;
            int iptot = (ip4[2]<<8)|ip4[3];          /* IP total length */
            uint8_t *tcp = ip4+ihl; int thl = (tcp[12]>>4)*4;
            int dport = (tcp[2]<<8)|tcp[3];
            uint32_t sseq;
            int dlen;
            if (ip4[9]!=6 || dport!=sport) continue;
            sseq = ((uint32_t)tcp[4]<<24)|((uint32_t)tcp[5]<<16)|((uint32_t)tcp[6]<<8)|tcp[7];
            dlen = iptot - ihl - thl;
            if (dlen < 0) dlen = 0;
            if (dlen > 0 && sseq == our_ack) {       /* in-order data */
                uint8_t *payload = tcp + thl; int k;
                for (k = 0; k < dlen && wifi_http_len < (int)sizeof(wifi_http_buf)-1; k++)
                    wifi_http_buf[wifi_http_len++] = payload[k];
                our_ack += dlen;
                wifi_tcp_send(nh, ip, sport, 80, our_seq, our_ack, 0x10, NULL, 0); /* ACK */
            } else if (dlen > 0) {
                /* out-of-order / retransmit: re-ACK our current position */
                wifi_tcp_send(nh, ip, sport, 80, our_seq, our_ack, 0x10, NULL, 0);
            }
            if (tcp[13] & 0x01) {                    /* FIN */
                our_ack += 1;
                wifi_tcp_send(nh, ip, sport, 80, our_seq, our_ack, 0x11, NULL, 0); /* FIN|ACK */
                fin = 1;
            }
            if (wifi_http_len >= (int)sizeof(wifi_http_buf)-1) break;
          }
        }
    }
    wifi_http_buf[wifi_http_len] = '\0';
    signal(wl_io_sem); wifi_net_pause = 0;
    wifi_log("[wifi] *** HTTP got %d bytes from %s (fin=%d) ***\r\n", wifi_http_len, host, fin);
    /* mirror the page to the serial console = Xinu's screen */
    kprintf("\r\n---- http://%s/ ----\r\n%s\r\n---- end (%d bytes) ----\r\n",
            host, wifi_http_buf, wifi_http_len);
    return wifi_http_len;
}

/* ------------------------------------------------------------------ *
 *  Simple framebuffer "browser": draw a window + render fetched HTML.  *
 * ------------------------------------------------------------------ */
extern void screenClear(unsigned long);
extern void fillRect(int, int, int, int, unsigned long, int);
extern void drawRect(int, int, int, int, unsigned long);
extern void drawLine(int, int, int, int, unsigned long);
extern void drawChar(char, int, int, unsigned long);
#define CHAR_WIDTH_  8     /* matches framebuffer_rpi font cell */
#define CHAR_HEIGHT_ 12

/* Draw a titled window frame: drop shadow + body fill + title bar + 2px border
 * + title-bar separator + white title text.  Corners (x,y)-(x2,y2). */
static void draw_win_frame(int x, int y, int x2, int y2, const char *title,
                           unsigned long body, unsigned long titlebar)
{
    const int TBH = 22; int i;
    fillRect(x+8, y+8, x2+8, y2+8, 0xFF202020, 0);   /* drop shadow */
    fillRect(x, y, x2, y2, body, 0);
    fillRect(x, y, x2, y+TBH, titlebar, 0);
    drawRect(x, y, x2, y2, 0xFF000000);
    drawRect(x-1, y-1, x2+1, y2+1, 0xFF000000);
    drawLine(x, y+TBH, x2, y+TBH, 0xFF000000);
    for (i = 0; title[i] && x+8+i*CHAR_WIDTH_ < x2-4; i++)
        drawChar(title[i], x+8+i*CHAR_WIDTH_, y+5, 0xFFFFFFFF);
}

/* A non-functional on-screen "soft keyboard" panel (key cells + labels). */
static void draw_softkbd(int x, int y, int x2, int y2)
{
    static const char *rows[5] = {
        "1234567890-=", "QWERTYUIOP[]", "ASDFGHJKL;", "ZXCVBNM,./", "  SPACE   Enter" };
    int r, c, kx, ky;
    draw_win_frame(x, y, x2, y2, "Soft keyboard", 0xFFE8E8E8, 0xFF504030);
    ky = y + 30;
    for (r = 0; r < 5; r++) {
        const char *row = rows[r]; kx = x + 8;
        for (c = 0; row[c]; c++) {
            if (kx + 18 > x2 - 4) break;
            drawRect(kx, ky, kx+18, ky+18, 0xFF808080);
            if (row[c] != ' ') drawChar(row[c], kx+5, ky+4, 0xFF000000);
            kx += 22;
        }
        ky += 24;
        if (ky + 18 > y2 - 4) break;
    }
}

/* Shell window state: text typed from the Mac browser (via /api/wifi/key). */
static char wifi_shell_buf[512];
static int  wifi_shell_len = 0;
static int  wifi_shell_gx = 720, wifi_shell_gy = 20, wifi_shell_gx2 = 1260, wifi_shell_gy2 = 320;

/* A "Shell (UART)" panel: green prompt + the typed text on black.  Remembers
 * its geometry so a keystroke can redraw just this window. */
static void draw_shell_win(int x, int y, int x2, int y2)
{
    int cx = x+8, cy = y+30, i;
    wifi_shell_gx=x; wifi_shell_gy=y; wifi_shell_gx2=x2; wifi_shell_gy2=y2;
    draw_win_frame(x, y, x2, y2, "Shell (UART)", 0xFF000000, 0xFF705030);
    { const char *pr = "xsh $ "; for (i = 0; pr[i]; i++) { drawChar(pr[i], cx, cy, 0xFF00FF00); cx += CHAR_WIDTH_; } }
    for (i = 0; i < wifi_shell_len; i++) {
        char c = wifi_shell_buf[i];
        if (c == '\n') { cx = x+8; cy += CHAR_HEIGHT_ + 2; }
        else { if (cx+CHAR_WIDTH_ > x2-4) { cx = x+8; cy += CHAR_HEIGHT_ + 2; }
               if (cy > y2-12) break;
               drawChar(c, cx, cy, 0xFF00FF00); cx += CHAR_WIDTH_; }
    }
    if (cy <= y2-12) drawChar('_', cx, cy, 0xFF00FF00);   /* cursor */
}

/* Append a key (char code) typed in the Mac browser to the shell buffer and
 * redraw just the Shell window.  8/127=backspace, 13/10=newline. */
void wifi_shell_key(int c)
{
    if (c == 8 || c == 127) { if (wifi_shell_len > 0) wifi_shell_len--; }
    else if (c == 13 || c == 10) { if (wifi_shell_len < (int)sizeof(wifi_shell_buf)-1) wifi_shell_buf[wifi_shell_len++] = '\n'; }
    else if (c >= 0x20 && c < 0x7f) { if (wifi_shell_len < (int)sizeof(wifi_shell_buf)-1) wifi_shell_buf[wifi_shell_len++] = (char)c; }
    wifi_shell_buf[wifi_shell_len] = '\0';
    draw_shell_win(wifi_shell_gx, wifi_shell_gy, wifi_shell_gx2, wifi_shell_gy2);
}

/* draw a clipped text line at (x,y) up to xmax. */
static void draw_text_line(int x, int y, const char *s, unsigned long col, int xmax)
{
    int i;
    for (i = 0; s[i] && x+i*CHAR_WIDTH_ < xmax; i++) drawChar(s[i], x+i*CHAR_WIDTH_, y, col);
}

/* "Xinu Pi3 Window System" info window. */
static void draw_winsys(int x, int y, int x2, int y2)
{
    char line[96];
    draw_win_frame(x, y, x2, y2, "Xinu Pi3 Window System", 0xFFF0F0F0, 0xFF206040);
    extern int abcl_n_objects(void);
    sprintf(line, "Build: %s", WIFI_BUILD_ID);
    draw_text_line(x+8, y+30, line, 0xFF000000, x2-4);
    sprintf(line, "IP: %d.%d.%d.%d   Screen: 1280x800x32",
            wifi_ip[0], wifi_ip[1], wifi_ip[2], wifi_ip[3]);
    draw_text_line(x+8, y+46, line, 0xFF000000, x2-4);
    draw_text_line(x+8, y+62, "Windows: Browser/Soft kbd/Shell/Actors/WinSys", 0xFF000000, x2-4);
    sprintf(line, "Actors loaded: %d", abcl_n_objects());
    draw_text_line(x+8, y+78, line, 0xFF000000, x2-4);
}

/* "Actors" window: the AIPL actor inventory (id, class, state, mbox backlog). */
static void draw_actors(int x, int y, int x2, int y2)
{
    extern int abcl_n_objects(void); extern int abcl_object_class_id(int);
    extern int abcl_object_started(int); extern int abcl_object_dead(int);
    extern int abcl_object_enq(int); extern int abcl_object_deq(int);
    int n = abcl_n_objects(), i, ly = y + 30;
    char line[64];
    draw_win_frame(x, y, x2, y2, "Actors", 0xFF101018, 0xFF603040);
    draw_text_line(x+8, ly, "id cls state mbox", 0xFF80C0FF, x2-4); ly += 16;
    for (i = 0; i < n && ly+12 < y2-4; i++) {
        const char *st = abcl_object_dead(i) ? "dead" : (abcl_object_started(i) ? "run " : "idle");
        sprintf(line, "%2d  %2d  %s  %d", i, abcl_object_class_id(i), st,
                abcl_object_enq(i) - abcl_object_deq(i));
        draw_text_line(x+8, ly, line, 0xFF00FF80, x2-4); ly += 14;
    }
    if (n == 0) draw_text_line(x+8, ly, "(no actors loaded)", 0xFF808080, x2-4);
}

/* Crudely convert HTML -> plain text: drop <script>/<style> blocks and all
 * tags, collapse whitespace.  ASCII only. */
static int html_to_text(const char *s, char *d, int cap)
{
    int n = 0, intag = 0, lastsp = 1;
    while (*s && n < cap - 1) {
        if (!intag && s[0] == '<') {
            if (!strncmp(s,"<script",7) || !strncmp(s,"<SCRIPT",7)) {
                const char *e = strstr(s, "</script>"); if (!e) e = strstr(s, "</SCRIPT>");
                if (e) { s = e + 9; continue; } else break;
            }
            if (!strncmp(s,"<style",6) || !strncmp(s,"<STYLE",6)) {
                const char *e = strstr(s, "</style>"); if (!e) e = strstr(s, "</STYLE>");
                if (e) { s = e + 8; continue; } else break;
            }
            intag = 1; s++; continue;
        }
        if (intag) { if (*s == '>') intag = 0; s++; continue; }
        { char c = *s++;
          if (c==' '||c=='\t'||c=='\r'||c=='\n') { if (!lastsp) { d[n++]=' '; lastsp=1; } }
          else if ((unsigned char)c >= 0x20 && (unsigned char)c < 0x7f) { d[n++]=c; lastsp=0; }
        }
    }
    d[n] = '\0';
    return n;
}

/* HTML "source view": keep tags + line structure but drop <script>/<style>
 * blocks.  Used as a fallback when the visible text is empty (e.g. only the
 * <head> was fetched), so the window always shows real page content. */
static int html_to_source(const char *s, char *d, int cap)
{
    int n = 0;
    while (*s && n < cap - 1) {
        if (!strncmp(s,"<script",7) || !strncmp(s,"<SCRIPT",7)) {
            const char *e = strstr(s, "</script>"); if (!e) e = strstr(s, "</SCRIPT>");
            if (e) { s = e + 9; continue; } else break;
        }
        if (!strncmp(s,"<style",6) || !strncmp(s,"<STYLE",6)) {
            const char *e = strstr(s, "</style>"); if (!e) e = strstr(s, "</STYLE>");
            if (e) { s = e + 8; continue; } else break;
        }
        { char c = *s++;
          if (c == '\n') d[n++] = '\n';
          else if (c == '\t') d[n++] = ' ';
          else if ((unsigned char)c >= 0x20 && (unsigned char)c < 0x7f) d[n++] = c;
        }
    }
    d[n] = '\0';
    return n;
}

int wifi_browse_xyf(const uint8_t *ip, const char *host, int wx, int wy, int ww, int wh, int fetch);

/* Fetch http://<host>/ and render it as a window on the HDMI framebuffer.
 * wx,wy = top-left; ww,wh = window size (<=0 => default centred 704x480).  The
 * geometry lets the Py-I "screen design" page place the window. */
int wifi_browse_xy(const uint8_t *ip, const char *host, int wx, int wy, int ww, int wh)
{
    return wifi_browse_xyf(ip, host, wx, wy, ww, wh, 1);   /* fetch */
}

/* As wifi_browse_xy, but `fetch`==0 re-renders the CACHED page (wifi_http_buf)
 * without a network round-trip — used for live drag/resize so moving the window
 * is instant (no re-fetch). */
int wifi_browse_xyf(const uint8_t *ip, const char *host, int wx, int wy, int ww, int wh, int fetch)
{
    static char text[8192];
    const char *body;
    int n, tn, i, col, x, y;
    int WX, WY, WX2, WY2; const int TBH=22;
    int TX0, TY0, TXMAX, TYMAX;
    /* clamp geometry to the 1280x800 screen; fall back to a centred window */
    if (ww <= 40 || wh <= 40) { wx=200; wy=140; ww=700; wh=440; }
    if (wx < 0) wx = 0; if (wy < 0) wy = 0;
    if (wx + ww > 1272) ww = 1272 - wx;
    if (wy + wh > 792)  wh = 792 - wy;
    WX=wx; WY=wy; WX2=wx+ww; WY2=wy+wh;
    TX0=WX+8; TY0=WY+TBH+6; TXMAX=WX2-8; TYMAX=WY2-10;

    if (fetch) { n = wifi_http(ip, host); }   /* fills wifi_http_buf */
    else       { n = wifi_http_len; }         /* reuse the cached page */
    if (n <= 0) return -1;

    /* skip HTTP headers (to after the blank line) before converting to text */
    body = strstr(wifi_http_buf, "\r\n\r\n");
    body = body ? body + 4 : wifi_http_buf;
    tn = html_to_text(body, text, sizeof(text));
    if (tn < 40)   /* little/no visible text (only head fetched) -> show source */
        tn = html_to_source(body, text, sizeof(text));

    /* --- draw the window (no full-screen clear: it floats over the console) --- */
    fillRect(WX+8, WY+8, WX2+8, WY2+8, 0xFF202020, 0); /* drop shadow */
    fillRect(WX, WY, WX2, WY2, 0xFFFFFFFF, 0);         /* page area: white */
    fillRect(WX, WY, WX2, WY+TBH, 0xFFC05000, 0);      /* title bar: blue */
    drawRect(WX,   WY,   WX2,   WY2,   0xFF000000);    /* window border (2px) */
    drawRect(WX-1, WY-1, WX2+1, WY2+1, 0xFF000000);
    drawLine(WX, WY+TBH, WX2, WY+TBH, 0xFF000000);     /* title separator */
    { char title[80]; int tl;
      tl = sprintf(title, "Xinu Browser   http://%s/", host);
      for (i = 0; i < tl; i++) drawChar(title[i], WX+8+i*CHAR_WIDTH_, WY+5, 0xFFFFFFFF);
    }

    /* --- render the text, word-wrapped, clipped to the window --- */
    x = TX0; y = TY0; col = 0; (void)col;
    for (i = 0; i < tn && y <= TYMAX; i++) {
        char c = text[i];
        if (c == '\n') { x = TX0; y += CHAR_HEIGHT_ + 2; continue; }   /* line break */
        if (x + CHAR_WIDTH_ > TXMAX) { x = TX0; y += CHAR_HEIGHT_ + 2; if (y > TYMAX) break; }
        if (c == ' ' && x == TX0) continue;           /* no leading space */
        drawChar(c, x, y, 0xFF000000);
        x += CHAR_WIDTH_;
    }
    wifi_log("[wifi] browse: rendered %d text bytes of %s to framebuffer\r\n", tn, host);
    return n;
}

/* Default-geometry wrapper (centred window). */
int wifi_browse(const uint8_t *ip, const char *host)
{
    return wifi_browse_xy(ip, host, 160, 140, 704, 480);
}

/* Multi-window "desktop": draw a Shell panel + a Soft-keyboard panel, then the
 * Browser window (which fetches `host` and renders on top).  Each window's
 * geometry comes from the Py-I screen-design page. */
int wifi_desktop(const uint8_t *ip, const char *host,
                 int bx, int by, int bw, int bh,
                 int kx, int ky, int kw, int kh,
                 int sx, int sy, int sw, int sh,
                 int px, int py, int pw, int ph,    /* Window System */
                 int ax, int ay, int aw, int ah,    /* Actors */
                 int fetch)
{
    /* Disabled: the gwm window manager (apps/gwm.c) now owns the HDMI
     * framebuffer and repaints it every frame.  Hand-drawing a desktop
     * here would fight gwm, so this is a no-op kept only so the legacy
     * /api/wifi/desktop endpoint still links.  The interactive Shell now
     * lives in gwm's "Shell" window, fed via /api/wifi/key. */
    (void)ip; (void)host;
    (void)bx; (void)by; (void)bw; (void)bh;
    (void)kx; (void)ky; (void)kw; (void)kh;
    (void)sx; (void)sy; (void)sw; (void)sh;
    (void)px; (void)py; (void)pw; (void)ph;
    (void)ax; (void)ay; (void)aw; (void)ah;
    (void)fetch;
    return 0;
}

/* Draw the default 3-window layout as the boot-time "initial screen".  The
 * Shell + Soft-keyboard panels need no network; the Browser shows a frame +
 * placeholder until a page is loaded via the design page.  Geometry matches
 * the /api/wifi/desktop defaults. */
void wifi_desktop_initial(void)
{
    /* Disabled — gwm (apps/gwm.c) owns the HDMI framebuffer now.  Kept as
     * a no-op so existing references still link. */
}

/* ================================================================== *
 *  Public entry points (called from the HTTP routes in webactor.c)   *
 * ================================================================== */
int wifi_probe(void)
{
    int n = 0;
    wifi_tn = 0;
    if (wifi_bringup() != 0) return -1;
    if (wifi_scan(&n) != 0) { wifi_log("[wifi] FAILED: scan\r\n"); return -1; }
    wifi_log("[wifi] *** SUCCESS: scan found %d access point(s) ***\r\n", n);
    return 0;
}

/* Scan and emit the AP list as JSON into out (for the selection UI). */
int wifi_scan_json(char *out, int cap)
{
    /* run a fresh scan (bring-up is cached), then format wifi_tbuf's AP lines */
    int n = 0, len = 0, i;
    wifi_tn = 0;
    if (wifi_bringup() != 0) return sprintf(out, "{\"error\":\"bringup failed\"}");
    wifi_scan(&n);
    /* parse the "[wifi] AP  N: \"ssid\" bssid ch=.. rssi=.." lines from the trace */
    len += sprintf(out + len, "{\"aps\":[");
    {
        const char *t = wifi_tbuf;
        int first = 1;
        for (i = 0; t[i] && len < cap - 200; i++) {
            if (t[i]=='A'&&t[i+1]=='P'&&t[i+2]==' ') {
                const char *q = &t[i];
                int j = 0; char line[160];
                while (q[j] && q[j] != '\r' && q[j] != '\n' && j < 159) { line[j]=q[j]; j++; }
                line[j] = '\0';
                len += sprintf(out + len, "%s\"%s\"", first ? "" : ",", line);
                first = 0;
                i += j;
            }
        }
    }
    len += sprintf(out + len, "],\"count\":%d}", n);
    return len;
}

/* Scan and extract up to `max` SSIDs into ssids[][40] (first quoted field
 * of each "AP N: \"ssid\" ..." trace line).  Returns the count.  Used by
 * the `wifi on` xsh command to present a numbered pick-list. */
int wifi_scan_ssids(char ssids[][40], int max)
{
    int n = 0, count = 0, i;
    wifi_tn = 0;
    if (wifi_bringup() != 0) return 0;
    wifi_scan(&n);
    const char *t = wifi_tbuf;
    for (i = 0; t[i] && count < max; i++) {
        if (t[i] == 'A' && t[i+1] == 'P' && t[i+2] == ' ') {
            int j = i;
            while (t[j] && t[j] != '"' && t[j] != '\n') j++;
            if (t[j] == '"') {
                int k = 0; j++;
                while (t[j] && t[j] != '"' && k < 39) ssids[count][k++] = t[j++];
                ssids[count][k] = 0;
                if (k > 0) count++;
            }
            while (t[i] && t[i] != '\n') i++;
        }
    }
    return count;
}

/* Disconnect: bring the BSS down and clear the connected state so the
 * indicator / `wifi status` show "not connected". */
void wifi_disconnect(void)
{
    /* Bare WLC_DOWN: simple + reliable (a WLC_DISASSOC here wedged the SDIO
     * control path on real hardware). */
    wifi_cmd_int(WLC_DOWN, 1);
    wifi_have_ip = 0;
    wifi_cur_ssid[0] = 0;
    /* Force a FULL chip re-bring-up on the next wifi_join().  The shared Arasan
     * EMMC controller + SDIO backplane/clock state do not survive a disconnect
     * intact, so a bare re-join finds the control path dead (every ioctl rc=-1,
     * zero firmware events — see `wifi diag`).  wifi_bringup() replays the exact
     * sequence that worked at first connect: WL_REG_ON reset, GPIO34-39->ALT3,
     * Arasan host init, F1 enable, firmware download (RAM-embedded — no SD), and
     * the SDPCM 'ver' check.  Re-running wifi_radio_up() too (event_msgs / clm /
     * country / mpc=0). */
    wifi_ready = 0;
    wifi_radio_done = 0;
}

/* Join (connect to) an access point. */
int wifi_join(const char *ssid, const char *pass)
{
    /* Remember the SSID for the on-screen indicator label. */
    int i; for (i = 0; i < 38 && ssid[i]; i++) wifi_cur_ssid[i] = ssid[i];
    wifi_cur_ssid[i] = 0;
    wifi_tn = 0;
    if (wifi_bringup() != 0) return -1;
    return wifi_do_join(ssid, pass);
}

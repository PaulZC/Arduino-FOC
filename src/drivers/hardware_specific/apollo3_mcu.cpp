/**
 * Support for the Ambiq Apollo3 Blue (SparkFun Artemis boards) using the CTIMERs
 * for 6-PWM BLDC driving.
 *
 * Why this exists:
 *   The generic fall-back uses analogWrite(). On the SparkFun Apollo3 core, analogWrite()
 *   is ~183 Hz / 8 bit (12 MHz / 65536) and every call can block while it waits for the
 *   timer to roll over. The six pins are also driven by six unsynchronised timers.
 *
 * How it works:
 *   - One CTIMER segment (A or B half of a timer) per pin, 6 in total, all on the 12 MHz
 *     HFRC clock with the same period P (default 20 kHz = 600 counts).
 *   - A segment in repeated-PWM mode is low from count 0 until its compare value c0, then
 *     high until the end of the period. So every segment has one edge we can move (c0) and
 *     one edge fixed at the period wrap.
 *   - High side: active-high pulse from c0H to the wrap.
 *   - Low side: polarity inverted, so it is on from its wrap until its c0L.
 *   - The low side counters are started 'lag' counts after the high side counters. That puts
 *     a gap of 'lag' counts between "high side off" (high side wrap) and "low side on"
 *     (low side wrap). The other gap, "low side off" -> "high side on", is set by c0L.
 *     Both edges therefore get dead time: lag ~= dead = dead_zone * P / 2 counts per edge.
 *   - The lag is measured after start-up and used in all later calculations.
 *
 *     count (high side frame):  0       lag            c0L+lag    c0H           P
 *     high side:                ______________________________________|'''''''''''|
 *     low side:                 ________|'''''''''''''''''''|______________________
 *                                 gap = lag             gap = dead
 *
 *   - The Apollo3 compare registers are not double-buffered: a new value takes effect
 *     immediately. Writing while the counter is between a phase's edges could make the
 *     low side miss its turn-off while the high side still turns on, i.e. both on at once.
 *     Each phase update therefore waits (with interrupts enabled) until the counter is
 *     before all, or after all, of that phase's old and new edges, then writes both registers
 *     inside a short critical section. Big jumps in duty cycle go via 50% in two steps.
 *     The safety margins are sized from the measured cost of a counter read and of a
 *     register write (printed at start-up). Typical wait: a few us per phase; worst case
 *     about one PWM period.
 *   - The low side lag is set with a fixed ROM delay between the two GLOBEN writes, which is
 *     measured and corrected at start-up (printed as "low side lag").
 *
 * Phase states (SimpleFOC semantics):
 *   PHASE_ON  - both sides PWM (complementary, with dead time)
 *   PHASE_HI  - high side PWM, low side held off
 *   PHASE_LO  - low side PWM, high side held off
 *   PHASE_OFF - both sides held off
 *   A side is "held off" by switching its pad to a plain GPIO output at the off level.
 *
 * Notes:
 *   - PWM is edge aligned (the Apollo3 counters only count up).
 *   - CTIMER 7 is never used, because mbed uses it for the low power ticker.
 *   - Pads are mapped to CTIMER segments automatically (with backtracking) from the same pad
 *     table the SparkFun core uses; each pin gets its own segment. Several 6PWM drivers can
 *     be used: segments are tracked across drivers (14 available = 2 drivers), and segments
 *     that few pads can use are taken first, to leave the flexible ones for later drivers.
 *     Pads 37 and 39 are supported via their normal timer outputs (the core's analogWrite
 *     treats them specially, which does not apply here).
 *   - The duty cycle is clamped near the ends: at 20 kHz with the default 2% dead zone,
 *     duty values below ~1.3% or above ~97.5% are clamped (the high side pulse never
 *     gets shorter than 2 counts, the low side pulse never shorter than ~4 counts).
 *   - The low side polarity inversion (TMRxPOL) is essential. Check H and L of each phase
 *     with a scope or logic analyser, motor supply off, before driving the motor.
 */

#include "../hardware_api.h"

#if defined(ARDUINO_ARCH_APOLLO3)

#include "am_mcu_apollo.h"

#define AP3_PWM_CLK            AM_HAL_CTIMER_HFRC_12MHZ
#define AP3_PWM_CLK_HZ         12000000UL
#define AP3_PWM_DEFAULT_FREQ   20000UL
#define AP3_PWM_MIN_PERIOD     100UL       // counts - room for dead time, margins and resolution
#define AP3_PWM_MAX_PERIOD     65535UL     // 16 bit counter
#define AP3_PWM_DRIVE          AM_HAL_GPIO_PIN_DRIVESTRENGTH_4MA
#define AP3_MIN_DEAD           2           // counts per edge (167 ns)
#define AP3_EDGE_MIN           2           // compare values are kept in [AP3_EDGE_MIN, P-1-AP3_EDGE_MIN]
#define AP3_LAG_ATTEMPTS       8           // start-up attempts to set the low side counter lag
#define AP3_CNT_PER_DELAY_IT   0.75f       // am_hal_flash_delay(): 3 CPU cycles @ 48 MHz = 0.75 timer counts @ 12 MHz

// ---------------------------------------------------------------------------------------
// Pad -> CTIMER output tables.
// Copied from the SparkFun core (CommonAnalog.cpp / am_hal_ctimer.c), where they are static.
// ---------------------------------------------------------------------------------------
#define AP3_CTX_PADNUM(ctx) ((ap3_ctx_tbl[ctx] >> 0) & 0x3f)
#define AP3_CTX(pad, fn)    ((fn << 8) | (pad << 0))
static const uint16_t ap3_ctx_tbl[32] = {
  AP3_CTX(12, 2), AP3_CTX(25, 2), AP3_CTX(13, 2), AP3_CTX(26, 2), AP3_CTX(18, 2),
  AP3_CTX(27, 2), AP3_CTX(19, 2), AP3_CTX(28, 2), AP3_CTX(5, 7),  AP3_CTX(29, 2),
  AP3_CTX(6, 5),  AP3_CTX(30, 2), AP3_CTX(22, 2), AP3_CTX(31, 2), AP3_CTX(23, 2),
  AP3_CTX(32, 2), AP3_CTX(42, 2), AP3_CTX(4, 6),  AP3_CTX(43, 2), AP3_CTX(7, 7),
  AP3_CTX(44, 2), AP3_CTX(24, 5), AP3_CTX(45, 2), AP3_CTX(33, 6), AP3_CTX(46, 2),
  AP3_CTX(39, 2), AP3_CTX(47, 2), AP3_CTX(35, 5), AP3_CTX(48, 2), AP3_CTX(37, 7),
  AP3_CTX(49, 2), AP3_CTX(11, 2)
};

// each entry: bit4 = OUT2, bit3 = segment B, bits2:0 = timer number
#define AP3_OUTC(timB, timN, N2) ((N2 << 4) | (timB << 3) | (timN << 0))
static const uint8_t ap3_outcfg_tbl[32][4] = {
  {AP3_OUTC(0,0,0), AP3_OUTC(1,2,1), AP3_OUTC(0,5,1), AP3_OUTC(0,6,0)}, // CTX0
  {AP3_OUTC(0,0,1), AP3_OUTC(0,0,0), AP3_OUTC(0,5,0), AP3_OUTC(1,7,1)}, // CTX1
  {AP3_OUTC(1,0,0), AP3_OUTC(1,1,1), AP3_OUTC(1,6,1), AP3_OUTC(0,7,0)}, // CTX2
  {AP3_OUTC(1,0,1), AP3_OUTC(1,0,0), AP3_OUTC(0,1,0), AP3_OUTC(0,6,0)}, // CTX3
  {AP3_OUTC(0,1,0), AP3_OUTC(0,2,1), AP3_OUTC(0,5,1), AP3_OUTC(1,5,0)}, // CTX4
  {AP3_OUTC(0,1,1), AP3_OUTC(0,1,0), AP3_OUTC(1,6,0), AP3_OUTC(0,7,0)}, // CTX5
  {AP3_OUTC(1,1,0), AP3_OUTC(0,1,0), AP3_OUTC(1,5,1), AP3_OUTC(1,7,0)}, // CTX6
  {AP3_OUTC(1,1,1), AP3_OUTC(1,1,0), AP3_OUTC(1,5,0), AP3_OUTC(0,7,0)}, // CTX7
  {AP3_OUTC(0,2,0), AP3_OUTC(0,3,1), AP3_OUTC(0,4,1), AP3_OUTC(1,6,0)}, // CTX8
  {AP3_OUTC(0,2,1), AP3_OUTC(0,2,0), AP3_OUTC(0,4,0), AP3_OUTC(1,0,0)}, // CTX9
  {AP3_OUTC(1,2,0), AP3_OUTC(1,3,1), AP3_OUTC(1,4,1), AP3_OUTC(0,6,0)}, // CTX10
  {AP3_OUTC(1,2,1), AP3_OUTC(1,2,0), AP3_OUTC(1,4,0), AP3_OUTC(1,5,1)}, // CTX11
  {AP3_OUTC(0,3,0), AP3_OUTC(1,1,0), AP3_OUTC(1,0,1), AP3_OUTC(1,6,1)}, // CTX12
  {AP3_OUTC(0,3,1), AP3_OUTC(0,3,0), AP3_OUTC(0,6,0), AP3_OUTC(1,4,1)}, // CTX13
  {AP3_OUTC(1,3,0), AP3_OUTC(1,1,0), AP3_OUTC(1,7,1), AP3_OUTC(0,7,0)}, // CTX14
  {AP3_OUTC(1,3,1), AP3_OUTC(1,3,0), AP3_OUTC(0,7,0), AP3_OUTC(0,4,1)}, // CTX15
  {AP3_OUTC(0,4,0), AP3_OUTC(0,0,0), AP3_OUTC(0,0,1), AP3_OUTC(1,3,1)}, // CTX16
  {AP3_OUTC(0,4,1), AP3_OUTC(1,7,0), AP3_OUTC(0,4,0), AP3_OUTC(0,1,1)}, // CTX17
  {AP3_OUTC(1,4,0), AP3_OUTC(1,0,0), AP3_OUTC(0,0,0), AP3_OUTC(0,3,1)}, // CTX18
  {AP3_OUTC(1,4,1), AP3_OUTC(0,2,0), AP3_OUTC(1,4,0), AP3_OUTC(1,1,1)}, // CTX19
  {AP3_OUTC(0,5,0), AP3_OUTC(0,1,0), AP3_OUTC(0,1,1), AP3_OUTC(1,2,1)}, // CTX20
  {AP3_OUTC(0,5,1), AP3_OUTC(0,1,0), AP3_OUTC(1,5,0), AP3_OUTC(0,0,1)}, // CTX21
  {AP3_OUTC(1,5,0), AP3_OUTC(0,6,0), AP3_OUTC(0,1,0), AP3_OUTC(0,2,1)}, // CTX22
  {AP3_OUTC(1,5,1), AP3_OUTC(0,7,0), AP3_OUTC(0,5,0), AP3_OUTC(1,0,1)}, // CTX23
  {AP3_OUTC(0,6,0), AP3_OUTC(0,2,0), AP3_OUTC(0,1,0), AP3_OUTC(1,1,1)}, // CTX24
  {AP3_OUTC(1,4,1), AP3_OUTC(1,2,0), AP3_OUTC(0,6,0), AP3_OUTC(0,2,1)}, // CTX25
  {AP3_OUTC(1,6,0), AP3_OUTC(1,2,0), AP3_OUTC(0,5,0), AP3_OUTC(0,1,1)}, // CTX26
  {AP3_OUTC(1,6,1), AP3_OUTC(0,1,0), AP3_OUTC(1,6,0), AP3_OUTC(1,2,1)}, // CTX27
  {AP3_OUTC(0,7,0), AP3_OUTC(0,3,0), AP3_OUTC(0,5,1), AP3_OUTC(1,0,1)}, // CTX28
  {AP3_OUTC(1,5,1), AP3_OUTC(0,1,0), AP3_OUTC(0,7,0), AP3_OUTC(0,3,1)}, // CTX29
  {AP3_OUTC(1,7,0), AP3_OUTC(1,3,0), AP3_OUTC(0,4,1), AP3_OUTC(0,0,1)}, // CTX30
  {AP3_OUTC(1,7,1), AP3_OUTC(0,6,0), AP3_OUTC(1,7,0), AP3_OUTC(1,3,1)}, // CTX31
};

// ---------------------------------------------------------------------------------------
// Driver state
// ---------------------------------------------------------------------------------------
typedef struct {
  uint8_t  pad;
  uint8_t  timer;
  uint32_t segment;      // AM_HAL_CTIMER_TIMERA / TIMERB
  bool     invert;       // segment output polarity inverted
  bool     active_high;  // level on the pad that switches the MOSFET on
} Ap3Channel;

typedef struct {
  Ap3Channel ch[6];      // [0]=A high, [1]=A low, [2]=B high, [3]=B low, [4]=C high, [5]=C low
  int32_t    period;     // P, PWM period in timer counts
  int32_t    dead;       // dead time in counts at the "low side off -> high side on" edge
  int32_t    lag[3];     // measured low side counter lag per phase = dead time at the wrap edge
  int32_t    c0H[3];     // compare values currently in the high side registers
  int32_t    c0L[3];     // compare values currently in the low side registers (low side frame)
  int32_t    midH[3];    // compare values for 50% duty, used as a stepping stone for big jumps
  int32_t    midL[3];
  int32_t    margin;     // counts from a counter read until both compare registers are written (measured)
  int32_t    slot;       // shortest counter range the polling loop is sure to see (measured)
  uint32_t   guard;      // polling iterations after which the counter is considered stopped
  bool       pwm[6];     // pad connected to its timer (true) or held off as GPIO (false)
} Ap3DriverParams;

static inline int32_t ap3_min(int32_t a, int32_t b) { return a < b ? a : b; }
static inline int32_t ap3_max(int32_t a, int32_t b) { return a > b ? a : b; }

// bit position of a segment in CTIMER->GLOBEN (ENA0=bit0, ENB0=bit1, ENA1=bit2, ...)
static inline uint32_t ap3_globen_bit(const Ap3Channel& c) {
  return 1UL << (c.timer * 2 + (c.segment == AM_HAL_CTIMER_TIMERB ? 1 : 0));
}

static int ap3_find_ctx(int pad) {
  if (pad < 0) return -1;
  for (int ctx = 0; ctx < 32; ctx++) {
    if (AP3_CTX_PADNUM(ctx) == pad) return ctx;
  }
  return -1;
}

// CTIMER segments in use by all 6PWM drivers (bit = timer*2 + B). Timer 7 is reserved:
// mbed uses it for the low power ticker.
static uint16_t ap3_used_segments = (1u << 14) | (1u << 15);

// Segments a pad can drive on its primary OUT (never OUT2, never timer 7), as GLOBEN bit
// numbers, in table order. Returns the count.
static int ap3_pad_options(int ctx, uint8_t* bits) {
  int n = 0;
  for (int k = 0; k < 4; k++) {
    uint8_t v = ap3_outcfg_tbl[ctx][k];
    if (v & 0x10) continue;                    // OUT2 - not used
    uint8_t timer = v & 0x7;
    if (timer == 7) continue;
    uint8_t bit = (uint8_t)(timer * 2 + ((v & 0x8) ? 1 : 0));
    bool dup = false;
    for (int j = 0; j < n; j++) dup |= (bits[j] == bit);
    if (!dup) bits[n++] = bit;
  }
  return n;
}

// How many pads can drive a segment. Segments that few pads can use are taken first, which
// leaves the widely usable ones (e.g. A1, the only option for pad 37) for later drivers.
static int ap3_segment_popularity(uint8_t bit) {
  int count = 0;
  for (int ctx = 0; ctx < 32; ctx++) {
    uint8_t bits[4];
    int n = ap3_pad_options(ctx, bits);
    for (int j = 0; j < n; j++) count += (bits[j] == bit);
  }
  return count;
}

// Give each pin its own CTIMER segment, avoiding segments other drivers use. Backtracking.
static bool ap3_allocate(const int* pins, int idx, uint16_t used, Ap3Channel* ch) {
  if (idx == 6) return true;
  int ctx = ap3_find_ctx(pins[idx]);
  if (ctx < 0) return false;
  uint8_t bits[4];
  int pop[4];
  int n = ap3_pad_options(ctx, bits);
  for (int j = 0; j < n; j++) pop[j] = ap3_segment_popularity(bits[j]);
  // stable sort by popularity, least popular first
  for (int a = 1; a < n; a++) {
    for (int b = a; b > 0 && pop[b] < pop[b - 1]; b--) {
      uint8_t tb = bits[b]; bits[b] = bits[b - 1]; bits[b - 1] = tb;
      int tp = pop[b]; pop[b] = pop[b - 1]; pop[b - 1] = tp;
    }
  }
  for (int j = 0; j < n; j++) {
    uint16_t mask = (uint16_t)(1u << bits[j]);
    if (used & mask) continue;
    ch[idx].pad = (uint8_t)pins[idx];
    ch[idx].timer = bits[j] >> 1;
    ch[idx].segment = (bits[j] & 1) ? AM_HAL_CTIMER_TIMERB : AM_HAL_CTIMER_TIMERA;
    if (ap3_allocate(pins, idx + 1, used | mask, ch)) return true;
  }
  return false;
}

// Counter value of a segment. Same method as am_hal_ctimer_read() (triple read, to
// reject a read torn by the clock domain crossing), without the table lookups.
static inline int32_t ap3_read_count(const Ap3Channel& c) {
  uint32_t v[3];
  am_hal_triple_read((uint32_t)CTIMERADDRn(CTIMER, c.timer, TMR0), v);
  uint32_t sh = (c.segment == AM_HAL_CTIMER_TIMERB) ? 16 : 0;
  uint32_t a = (v[0] >> sh) & 0xFFFF;
  uint32_t b = (v[1] >> sh) & 0xFFFF;
  uint32_t d = (v[2] >> sh) & 0xFFFF;
  return (int32_t)(((a & ~0x1Fu) == (b & ~0x1Fu)) ? b : d);
}

// Lag of the low side counter behind the high side counter, in counts (0..P-1).
// The low side read is bracketed by two high side reads, so the read time cancels out.
static int32_t ap3_measure_lag(const Ap3Channel& H, const Ap3Channel& L, int32_t P) {
  for (int attempt = 0; attempt < 4; attempt++) {
    const int32_t h1 = ap3_read_count(H);
    const int32_t l  = ap3_read_count(L);
    const int32_t h2 = ap3_read_count(H);
    if (h2 < h1) continue;                     // a wrap happened in between - read again
    int32_t d = ((h1 + h2 + 1) / 2 - l) % P;
    if (d < 0) d += P;
    return d;
  }
  return -1;
}

// Write a segment's compare value: output goes high at count c0, low at the wrap (count P-1).
// One 32 bit write, so CMPR0/CMPR1 always change together.
static inline void ap3_set_c0(const Ap3Channel& c, int32_t period, int32_t c0) {
  volatile uint32_t* reg = (c.segment == AM_HAL_CTIMER_TIMERA)
      ? (volatile uint32_t*)CTIMERADDRn(CTIMER, c.timer, CMPRA0)
      : (volatile uint32_t*)CTIMERADDRn(CTIMER, c.timer, CMPRB0);
  *reg = _VAL2FLD(CTIMER_CMPRA0_CMPR0A0, (uint32_t)c0) | _VAL2FLD(CTIMER_CMPRA0_CMPR1A0, (uint32_t)(period - 1));
}

// Hold a pad at its off level as a GPIO output. The level is written before the pad is
// switched to GPIO, so it can never briefly drive a stale (possibly "on") level.
static inline void ap3_pad_off(const Ap3Channel& c) {
  am_hal_gpio_write_type_e lvl = c.active_high ? AM_HAL_GPIO_OUTPUT_CLEAR : AM_HAL_GPIO_OUTPUT_SET;
  am_hal_gpio_state_write(c.pad, lvl);
  am_hal_gpio_pincfg_t cfg = g_AM_HAL_GPIO_OUTPUT;
  cfg.eDriveStrength = AP3_PWM_DRIVE;
  am_hal_gpio_pinconfig(c.pad, cfg);
  am_hal_gpio_state_write(c.pad, lvl);
}

// Connect a pad to its CTIMER segment output.
static inline void ap3_pad_pwm(const Ap3Channel& c) {
  am_hal_ctimer_output_config(c.timer, c.segment, c.pad, AM_HAL_CTIMER_OUTPUT_NORMAL, AP3_PWM_DRIVE);
}

// Duty cycle -> compare values for phase i.
// High side on-time h = duty*P - dead, low side on-time = (1-duty)*P - dead (approx.)
static inline void ap3_compute(const Ap3DriverParams* p, int i, float duty, int32_t& c0H, int32_t& c0L) {
  const int32_t P = p->period;
  const int32_t Q = P - 1;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  float hf = duty * (float)P - (float)p->dead;
  int32_t h = (hf <= 0.0f) ? 0 : (int32_t)(hf + 0.5f);
  int32_t hmax = Q - 2 * AP3_EDGE_MIN - p->dead - p->lag[i];
  if (h < AP3_EDGE_MIN) h = AP3_EDGE_MIN;
  if (h > hmax) h = hmax;
  c0H = Q - h;                          // high side on from c0H to the wrap
  c0L = c0H - p->dead - p->lag[i];      // low side off at c0L (+lag in the high side frame)
}

// Counter ranges (high side frame) in which the registers of phase i may be changed from the
// current values to (nH, nL): the write is allowed if t < lowEnd, or upStart < t <= upEnd.
//
//  - "low" region:  before the earliest old/new edge of this phase in this period. Nothing has
//    switched yet (high side off since its wrap, low side on since its wrap - or, for t < lag,
//    still in the off part at the end of its previous period), so the new values simply apply
//    to the whole period.
//  - "up" region:   after the latest old/new edge and far enough from the wrap. Everything has
//    switched already (high side on, low side off), the new values apply from the next period.
// The margin (time from reading the counter until both registers are written) and the minimum
// region length (so the polling loop cannot step over a region) are measured at start-up.
// Returns false if neither region is usable.
static bool ap3_regions(const Ap3DriverParams* p, int i, int32_t nH, int32_t nL,
                        int32_t& lowEnd, int32_t& upStart, int32_t& upEnd) {
  const int32_t Q = p->period - 1;
  const int32_t eMin = ap3_min(p->c0L[i], nL) + p->lag[i];   // earliest "low side off"
  const int32_t eMax = ap3_max(p->c0H[i], nH);                // latest "high side on"
  lowEnd  = eMin - p->margin;
  upStart = eMax + 2;
  upEnd   = Q - p->margin;                                    // never write across the wrap
  const bool okLow = lowEnd >= p->slot;
  const bool okUp  = (upEnd - upStart) >= p->slot;
  if (!okLow) lowEnd = 0;   // "t < 0" is never true
  if (!okUp)  upEnd = -1;   // "t <= -1" is never true
  return okLow || okUp;
}

// Update the compare registers of phase i without ever allowing high and low side on together.
// The Apollo3 compare registers are not buffered, so the write is placed in a part of the
// period where it cannot affect an edge that is in progress (see ap3_regions). For a big jump
// in duty cycle there may be no such part; the update is then done in two steps via the 50%
// values, which always leaves a usable region. Interrupts are only disabled for each single
// read-check-write, never for the whole wait (typically < 1 us, at most about one period).
static void ap3_write_pair(Ap3DriverParams* p, int i, int32_t nH, int32_t nL) {
  const int32_t P = p->period;
  const Ap3Channel& H = p->ch[2 * i];
  const Ap3Channel& L = p->ch[2 * i + 1];

  uint32_t guard = 0;
  while (p->c0H[i] != nH || p->c0L[i] != nL) {
    int32_t wH = nH, wL = nL, lowEnd, upStart, upEnd;
    if (!ap3_regions(p, i, wH, wL, lowEnd, upStart, upEnd)) {
      wH = p->midH[i];
      wL = p->midL[i];
      if (!ap3_regions(p, i, wH, wL, lowEnd, upStart, upEnd)) {
        // cannot happen with the period checks in _configure6PWM - write directly
        wH = nH; wL = nL; guard = p->guard;
      }
    }
    bool written = false;
    while (!written) {
      uint32_t primask = am_hal_interrupt_master_disable();
      const int32_t t = ap3_read_count(H);
      if (guard >= p->guard || t < lowEnd || (t > upStart && t <= upEnd)) {
        ap3_set_c0(L, P, wL);
        ap3_set_c0(H, P, wH);
        p->c0H[i] = wH;
        p->c0L[i] = wL;
        written = true;
      }
      am_hal_interrupt_master_set(primask);
      guard++;   // a counter that does not run (guard reached) has no edges to protect
    }
  }
}

// ---------------------------------------------------------------------------------------
// 6PWM
// ---------------------------------------------------------------------------------------
// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l, const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  if (!pwm_frequency || !_isset(pwm_frequency)) pwm_frequency = AP3_PWM_DEFAULT_FREQ;
  if (!_isset(dead_zone) || dead_zone < 0.0f) dead_zone = 0.0f;

  if (pwm_frequency <= 0 || (uint32_t)pwm_frequency > AP3_PWM_CLK_HZ / AP3_PWM_MIN_PERIOD) {
    SIMPLEFOC_DEBUG("Apollo3-DRV: unsupported PWM frequency (184 Hz .. 120 kHz): ", (int)pwm_frequency);
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  int32_t P = (int32_t)((AP3_PWM_CLK_HZ + (uint32_t)pwm_frequency / 2) / (uint32_t)pwm_frequency);
  if (P < (int32_t)AP3_PWM_MIN_PERIOD || P > (int32_t)AP3_PWM_MAX_PERIOD) {
    SIMPLEFOC_DEBUG("Apollo3-DRV: unsupported PWM frequency (184 Hz .. 120 kHz): ", (int)pwm_frequency);
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  // SimpleFOC dead_zone is the fraction of the period spent in dead time; it is split
  // between the two switching edges of each half bridge
  int32_t dead = (int32_t)(0.5f * dead_zone * (float)P + 0.5f);
  if (dead < AP3_MIN_DEAD) dead = AP3_MIN_DEAD;
  if (dead > P / 8) dead = P / 8;

  Ap3DriverParams* p = new Ap3DriverParams();
  p->period = P;
  p->dead = dead;

  const int pins[6] = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l };
  for (int i = 0; i < 6; i++) {
    if (ap3_find_ctx(pins[i]) < 0) {
      SIMPLEFOC_DEBUG("Apollo3-DRV: pin has no CTIMER output: ", pins[i]);
      delete p;
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
  }
  if (!ap3_allocate(pins, 0, ap3_used_segments, p->ch)) {
    SIMPLEFOC_DEBUG("Apollo3-DRV: these 6 pins cannot each get their own free CTIMER segment");
    delete p;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  uint32_t maskH = 0, maskL = 0;
  for (int i = 0; i < 6; i++) {
    bool low_side = (i & 1);
    Ap3Channel& c = p->ch[i];
    c.active_high = low_side ? SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH : SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH;
    // high side: raw pulse = on time  -> inverted only if active low
    // low side:  raw pulse = off time -> inverted if active high
    c.invert = low_side ? c.active_high : !c.active_high;
    if (low_side) maskL |= ap3_globen_bit(c);
    else          maskH |= ap3_globen_bit(c);
  }
  // reserve the segments for this driver (GLOBEN bit numbers = segment numbers)
  ap3_used_segments |= (uint16_t)(maskH | maskL);

  // all pads held off while the timers are set up
  for (int i = 0; i < 6; i++) {
    ap3_pad_off(p->ch[i]);
    p->pwm[i] = false;
  }

  // ---- Start-up ----------------------------------------------------------------------
  // All six segments are configured while held in GLOBEN. Then the high side segments are
  // released and, after a fixed ROM delay (am_hal_flash_delay, deterministic with interrupts
  // off), the low side segments. The resulting lag is measured and the delay corrected until
  // the lag is within a count or two of 'dead'. (Polling the counter is too coarse for this:
  // on the Apollo3 a single counter read takes several timer counts.)
  const uint32_t globen = CTIMER->GLOBEN & ~(maskH | maskL);
  CTIMER->GLOBEN = globen;
  for (int i = 0; i < 6; i++) {
    const Ap3Channel& c = p->ch[i];
    uint32_t cfg = AP3_PWM_CLK | AM_HAL_CTIMER_FN_PWM_REPEAT | (c.invert ? AM_HAL_CTIMER_PIN_INVERT : 0);
    am_hal_ctimer_config_single(c.timer, c.segment, cfg);
    ap3_set_c0(c, P, P / 2);                                 // placeholder, pads are off
  }

  int32_t iters = 0, offset0 = -1, desired = dead;
  int attempts = 0;
  bool lag_ok = false;
  while (attempts < AP3_LAG_ATTEMPTS && !lag_ok) {
    attempts++;
    uint32_t primask = am_hal_interrupt_master_disable();
    CTIMER->GLOBEN = globen;                                   // hold all six
    for (int i = 0; i < 6; i++) {
      am_hal_ctimer_clear(p->ch[i].timer, p->ch[i].segment);   // counter to 0
      am_hal_ctimer_start(p->ch[i].timer, p->ch[i].segment);   // enabled, still held by GLOBEN
    }
    CTIMER->GLOBEN = globen | maskH;                           // high side counters run
    if (iters > 0) am_hal_flash_delay((uint32_t)iters);
    CTIMER->GLOBEN = globen | maskH | maskL;                   // low side counters run
    for (int i = 0; i < 3; i++) p->lag[i] = ap3_measure_lag(p->ch[2 * i], p->ch[2 * i + 1], P);
    am_hal_interrupt_master_set(primask);

    lag_ok = true;
    for (int i = 0; i < 3; i++) {
      if (p->lag[i] < ap3_max(AP3_MIN_DEAD, dead - 1) || p->lag[i] > dead + 2) lag_ok = false;
    }
    const int32_t m = p->lag[0];
    if (lag_ok || m < 0) continue;

    // correct the delay: lag = offset0 + 0.75 * iters (mod P)
    if (offset0 < 0) {
      offset0 = m;   // lag caused by the instructions between the two GLOBEN writes alone
      // am_hal_flash_delay() has a minimum length, so keep the delay well above zero:
      // if the overhead alone is already close to or above 'dead', aim one period further
      desired = (dead >= offset0 + 24) ? dead : dead + P;
      iters = (int32_t)floorf((float)(desired - offset0) / AP3_CNT_PER_DELAY_IT + 0.5f);
    } else {
      // unwrap the measured lag to the value closest to the one aimed for
      const int32_t abs_lag = m + P * (int32_t)floorf((float)(desired - m) / (float)P + 0.5f);
      iters += (int32_t)floorf((float)(desired - abs_lag) / AP3_CNT_PER_DELAY_IT + 0.5f);
      if (iters < 0) iters = 0;
    }
  }
  if (!lag_ok) {
    // accept a slightly less precise result rather than failing
    lag_ok = true;
    for (int i = 0; i < 3; i++) {
      if (p->lag[i] < ap3_max(AP3_MIN_DEAD, dead - 2) || p->lag[i] > dead + 6) lag_ok = false;
    }
  }
  SIMPLEFOC_DEBUG("Apollo3-DRV: low side lag A [counts]: ", (int)p->lag[0]);
  SIMPLEFOC_DEBUG("Apollo3-DRV: low side lag B [counts]: ", (int)p->lag[1]);
  SIMPLEFOC_DEBUG("Apollo3-DRV: low side lag C [counts]: ", (int)p->lag[2]);
  SIMPLEFOC_DEBUG("Apollo3-DRV: lag without delay [counts]: ", (int)offset0);
  SIMPLEFOC_DEBUG("Apollo3-DRV: lag delay [iterations]: ", (int)iters);
  SIMPLEFOC_DEBUG("Apollo3-DRV: lag attempts: ", attempts);
  if (!lag_ok) {
    SIMPLEFOC_DEBUG("Apollo3-DRV: could not set the low side timer lag");
    CTIMER->GLOBEN = globen;
    ap3_used_segments &= (uint16_t)~(maskH | maskL);
    delete p;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  // ---- Measure how long a counter read and a register write take -----------------------
  int32_t read_cost, write_cost;
  {
    const Ap3Channel& H = p->ch[0];
    const Ap3Channel& L = p->ch[1];
    uint32_t primask = am_hal_interrupt_master_disable();
    int32_t t0 = ap3_read_count(H);
    ap3_read_count(H); ap3_read_count(H); ap3_read_count(H);
    int32_t t1 = ap3_read_count(H);
    read_cost = ((((t1 - t0) % P) + P) % P + 2) / 4;          // 4 reads between the two samples
    t0 = ap3_read_count(H);
    ap3_set_c0(L, P, P / 2);                                   // placeholder values, pads are off
    ap3_set_c0(H, P, P / 2);
    t1 = ap3_read_count(H);
    am_hal_interrupt_master_set(primask);
    write_cost = (((t1 - t0) % P) + P) % P - read_cost;
    if (write_cost < 0) write_cost = 0;
  }
  // margin: the counter value may be up to one read old, then the check, then two writes
  p->margin = 2 * read_cost + write_cost + 4;
  // a region must be longer than one turn of the polling loop
  p->slot   = 2 * read_cost + 4;
  p->guard  = (uint32_t)(4 * P / ap3_max(read_cost, 1)) + 1000;
  SIMPLEFOC_DEBUG("Apollo3-DRV: counter read [counts]: ", (int)read_cost);
  SIMPLEFOC_DEBUG("Apollo3-DRV: 2 register writes [counts]: ", (int)write_cost);

  const int32_t lag_max = ap3_max(p->lag[0], ap3_max(p->lag[1], p->lag[2]));
  const int32_t p_min = 2 * (p->margin + p->slot + dead + lag_max + 8);
  if (P < p_min) {
    SIMPLEFOC_DEBUG("Apollo3-DRV: PWM frequency too high, max [Hz]: ", (int)(AP3_PWM_CLK_HZ / (uint32_t)p_min));
    CTIMER->GLOBEN = globen;
    ap3_used_segments &= (uint16_t)~(maskH | maskL);
    delete p;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  // initial compare values for 0 duty (pads are still off, so no timing needed)
  for (int i = 0; i < 3; i++) {
    ap3_compute(p, i, 0.5f, p->midH[i], p->midL[i]);
    int32_t c0H, c0L;
    ap3_compute(p, i, 0.0f, c0H, c0L);
    ap3_set_c0(p->ch[2 * i], P, c0H);
    ap3_set_c0(p->ch[2 * i + 1], P, c0L);
    p->c0H[i] = c0H;
    p->c0L[i] = c0L;
  }

  SIMPLEFOC_DEBUG("Apollo3-DRV: CTIMER 6PWM, period [counts]: ", (int)P);
  SIMPLEFOC_DEBUG("Apollo3-DRV: dead time [counts]: ", (int)dead);
  SIMPLEFOC_DEBUG("Apollo3-DRV: wrap edge dead time [counts]: ", (int)ap3_min(p->lag[0], ap3_min(p->lag[1], p->lag[2])));
  return p;
}

// Set one half bridge.
static void _ap3SetPwmPair(Ap3DriverParams* p, int i, float duty, PhaseState state) {
  const bool wantH = (state == PhaseState::PHASE_ON || state == PhaseState::PHASE_HI);
  const bool wantL = (state == PhaseState::PHASE_ON || state == PhaseState::PHASE_LO);
  const int iH = 2 * i, iL = 2 * i + 1;

  // 1. switch off whatever must be off
  if (!wantH && p->pwm[iH]) { ap3_pad_off(p->ch[iH]); p->pwm[iH] = false; }
  if (!wantL && p->pwm[iL]) { ap3_pad_off(p->ch[iL]); p->pwm[iL] = false; }
  if (!wantH && !wantL) return;

  // 2. new compare values, written at a safe moment
  int32_t c0H, c0L;
  ap3_compute(p, i, duty, c0H, c0L);
  ap3_write_pair(p, i, c0H, c0L);

  // 3. connect the sides that run PWM
  if (wantH && !p->pwm[iH]) { ap3_pad_pwm(p->ch[iH]); p->pwm[iH] = true; }
  if (wantL && !p->pwm[iL]) { ap3_pad_pwm(p->ch[iL]); p->pwm[iL] = true; }
}

// Function setting the duty cycle to the pwm pins
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState* phase_state, void* params) {
  Ap3DriverParams* p = (Ap3DriverParams*)params;
  _ap3SetPwmPair(p, 0, dc_a, phase_state[0]);
  _ap3SetPwmPair(p, 1, dc_b, phase_state[1]);
  _ap3SetPwmPair(p, 2, dc_c, phase_state[2]);
}

#endif // ARDUINO_ARCH_APOLLO3

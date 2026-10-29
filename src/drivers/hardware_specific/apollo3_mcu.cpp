/**
 * Support for the Ambiq Apollo3 Blue (SparkFun Artemis boards) using the CTIMERs
 * for 6-PWM BLDC driving.
 *
 * Why this exists:
 *   The generic fall-back uses analogWrite(). On the SparkFun Apollo3 core, analogWrite()
 *   is ~183 Hz / 8 bit (12 MHz / 65536) and every call can block while it waits for the
 *   timer to roll over. The six pins are also driven by six unsynchronised timers.
 *
 * What this does:
 *   - one CTIMER segment (A or B half of a timer) per pin, 6 in total
 *   - all segments use the 12 MHz HFRC clock and the same period (default 20 kHz = 600 counts)
 *   - all segments are held by the global enable register (GLOBEN) while they are configured,
 *     then released together, so the six counters start in phase
 *   - the low-side segments have their output polarity inverted (TMRxPOL), so the low side
 *     is the exact complement of the high side, with a dead zone of 'dead_zone' * period
 *     created by shortening the high-side pulse and lengthening the low-side one
 *   - a duty cycle update is a single 32 bit register write per pin (no blocking)
 *
 * Notes:
 *   - PWM is edge aligned (the Apollo3 counters only count up).
 *   - CTIMER 7 is skipped, because mbed uses it for the low power ticker.
 *   - Pads are mapped to CTIMER segments automatically from the same pad table the SparkFun
 *     core uses; each pin gets its own segment, so no two pins ever share a counter.
 *   - Phase states (OFF / HI / LO) are done by switching the pads back to plain GPIO.
 */

#include "../hardware_api.h"

#if defined(ARDUINO_ARCH_APOLLO3)

#include "am_mcu_apollo.h"

#define AP3_PWM_CLK            AM_HAL_CTIMER_HFRC_12MHZ
#define AP3_PWM_CLK_HZ         12000000UL
#define AP3_PWM_DEFAULT_FREQ   20000UL
#define AP3_PWM_MIN_PERIOD     20UL        // counts - below this the resolution is pointless
#define AP3_PWM_MAX_PERIOD     65535UL     // 16 bit counter
#define AP3_PWM_DRIVE          AM_HAL_GPIO_PIN_DRIVESTRENGTH_4MA

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

enum Ap3PairMode : uint8_t { AP3_MODE_UNKNOWN = 0, AP3_MODE_PWM, AP3_MODE_OFF, AP3_MODE_HI, AP3_MODE_LO };

typedef struct {
  Ap3Channel ch[6];      // [0]=A high, [1]=A low, [2]=B high, [3]=B low, [4]=C high, [5]=C low
  uint32_t   period;     // PWM period in timer counts
  float      dead_zone;  // fraction of the period
  Ap3PairMode mode[3];
} Ap3DriverParams;

// bit position of a segment in CTIMER->GLOBEN (ENA0=bit0, ENB0=bit1, ENA1=bit2, ...)
static inline uint32_t ap3_globen_bit(const Ap3Channel& c) {
  return 1UL << (c.timer * 2 + (c.segment == AM_HAL_CTIMER_TIMERB ? 1 : 0));
}

// Choose a free CTIMER segment that can output on 'pad' (primary OUT only, not OUT2).
static bool ap3_pick_segment(uint8_t pad, uint16_t& used, Ap3Channel& c) {
  uint8_t ctx = 0;
  for (; ctx < 32; ctx++) {
    if (AP3_CTX_PADNUM(ctx) == pad) break;
  }
  if (ctx >= 32) return false;
  if (pad == 37 || pad == 39) return false;   // special-cased in the core, not supported here
  for (uint8_t n = 0; n < 4; n++) {
    uint8_t v = ap3_outcfg_tbl[ctx][n];
    if (v & 0x10) continue;                    // OUT2 - skip
    uint8_t timer = v & 0x7;
    bool isB = (v & 0x8) != 0;
    if (timer == 7) continue;                  // used by mbed low power ticker
    uint16_t bit = 1u << (timer * 2 + (isB ? 1 : 0));
    if (used & bit) continue;
    used |= bit;
    c.pad = pad;
    c.timer = timer;
    c.segment = isB ? AM_HAL_CTIMER_TIMERB : AM_HAL_CTIMER_TIMERA;
    return true;
  }
  return false;
}

// Set the on-time (in counts) of a segment. One 32 bit write -> atomic w.r.t. the counter.
// Rising edge at (period-1 - on), falling edge / wrap at (period-1): the pulse sits at the end of the period.
static inline void ap3_set_on_time(const Ap3Channel& c, uint32_t period, uint32_t on) {
  volatile uint32_t* reg = (c.segment == AM_HAL_CTIMER_TIMERA)
      ? (volatile uint32_t*)CTIMERADDRn(CTIMER, c.timer, CMPRA0)
      : (volatile uint32_t*)CTIMERADDRn(CTIMER, c.timer, CMPRB0);
  uint32_t q = period - 1;
  *reg = _VAL2FLD(CTIMER_CMPRA0_CMPR0A0, q - on) | _VAL2FLD(CTIMER_CMPRA0_CMPR1A0, q);
}

static inline void ap3_pad_gpio(const Ap3Channel& c, bool level_high) {
  am_hal_gpio_pincfg_t cfg = g_AM_HAL_GPIO_OUTPUT;
  cfg.eDriveStrength = AP3_PWM_DRIVE;
  am_hal_gpio_pinconfig(c.pad, cfg);
  am_hal_gpio_state_write(c.pad, level_high ? AM_HAL_GPIO_OUTPUT_SET : AM_HAL_GPIO_OUTPUT_CLEAR);
}

static inline void ap3_pad_ctimer(const Ap3Channel& c) {
  am_hal_ctimer_output_config(c.timer, c.segment, c.pad, AM_HAL_CTIMER_OUTPUT_NORMAL, AP3_PWM_DRIVE);
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
  if (dead_zone > 0.5f) dead_zone = 0.5f;

  uint32_t period = (AP3_PWM_CLK_HZ + (uint32_t)pwm_frequency / 2) / (uint32_t)pwm_frequency;
  if (period < AP3_PWM_MIN_PERIOD || period > AP3_PWM_MAX_PERIOD) {
    SIMPLEFOC_DEBUG("Apollo3: unsupported PWM frequency (valid ~184 Hz .. 600 kHz): ", (int)pwm_frequency);
    return SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  Ap3DriverParams* p = new Ap3DriverParams();
  p->period = period;
  p->dead_zone = dead_zone;

  const int pins[6] = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l };
  uint16_t used = 0;
  uint32_t mask = 0;
  for (int i = 0; i < 6; i++) {
    bool low_side = (i & 1);
    if (!ap3_pick_segment((uint8_t)pins[i], used, p->ch[i])) {
      SIMPLEFOC_DEBUG("Apollo3: no free CTIMER output for pin ", pins[i]);
      delete p;
      return SIMPLEFOC_DRIVER_INIT_FAILED;
    }
    p->ch[i].active_high = low_side ? SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH : SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH;
    // High side: pulse is the "on" time -> not inverted when active high.
    // Low side:  complement of the high side -> inverted when active high.
    p->ch[i].invert = low_side ? p->ch[i].active_high : !p->ch[i].active_high;
    mask |= ap3_globen_bit(p->ch[i]);
  }

  // Hold all six segments while they are configured, then release them together
  uint32_t globen_prev = CTIMER->GLOBEN;
  am_hal_ctimer_globen(globen_prev & ~mask);

  for (int i = 0; i < 6; i++) {
    const Ap3Channel& c = p->ch[i];
    uint32_t cfg = AP3_PWM_CLK | AM_HAL_CTIMER_FN_PWM_REPEAT | (c.invert ? AM_HAL_CTIMER_PIN_INVERT : 0);
    am_hal_ctimer_config_single(c.timer, c.segment, cfg);   // also clears the counter
    ap3_set_on_time(c, period, 2);
    am_hal_ctimer_start(c.timer, c.segment);
  }

  // all pads start as GPIO outputs in the OFF level, until the first PHASE_ON write
  for (int i = 0; i < 6; i++) {
    ap3_pad_gpio(p->ch[i], !p->ch[i].active_high);
  }
  for (int i = 0; i < 3; i++) p->mode[i] = AP3_MODE_OFF;

  am_hal_ctimer_globen(globen_prev | mask);

  SIMPLEFOC_DEBUG("Apollo3: CTIMER 6PWM, period [counts]: ", (int)period);
  return p;
}

static inline void ap3_set_pair_mode(Ap3DriverParams* p, int i, Ap3PairMode m) {
  const Ap3Channel& H = p->ch[2 * i];
  const Ap3Channel& L = p->ch[2 * i + 1];
  switch (m) {
    case AP3_MODE_OFF: ap3_pad_gpio(H, !H.active_high); ap3_pad_gpio(L, !L.active_high); break;
    case AP3_MODE_HI:  ap3_pad_gpio(H,  H.active_high); ap3_pad_gpio(L, !L.active_high); break;
    case AP3_MODE_LO:  ap3_pad_gpio(H, !H.active_high); ap3_pad_gpio(L,  L.active_high); break;
    default:           ap3_pad_ctimer(H); ap3_pad_ctimer(L); break;
  }
  p->mode[i] = m;
}

// Set one half bridge: complementary H/L pair with dead zone.
// duty in [0,1] is the fraction of the period for which the high side is (nominally) on.
//
// High side on-time h = duty*P - dead/2     -> pulse at the end of the period
// Low side  raw  r    = duty*P + dead/2     -> output inverted: low side is on for the first (P - r) counts
// Gap between them    = r - h = dead counts (at least 2)
static inline void _ap3SetPwmPair(Ap3DriverParams* p, int i, float duty, PhaseState state) {
  Ap3PairMode want;
  switch (state) {
    case PhaseState::PHASE_ON: want = AP3_MODE_PWM; break;
    case PhaseState::PHASE_HI: want = AP3_MODE_HI;  break;
    case PhaseState::PHASE_LO: want = AP3_MODE_LO;  break;
    default:                   want = AP3_MODE_OFF; break;
  }
  if (want != AP3_MODE_PWM) {
    if (p->mode[i] != want) ap3_set_pair_mode(p, i, want);
    return;
  }

  const uint32_t P = p->period;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  float on   = duty * (float)P;
  float half = 0.5f * p->dead_zone * (float)P;
  if (half < 1.0f) half = 1.0f;                 // never less than 2 counts (167 ns) of gap

  int32_t h = (int32_t)(on - half + 0.5f);
  int32_t r = (int32_t)(on + half + 0.5f);
  // hardware limits: pulse >= 1 count and <= period-3 counts
  int32_t rmax = (int32_t)P - 3;
  if (h < 2) h = 2;
  if (h > rmax - 2) h = rmax - 2;
  if (r < h + 2) r = h + 2;
  if (r > rmax) r = rmax;

  ap3_set_on_time(p->ch[2 * i],     P, (uint32_t)h);
  ap3_set_on_time(p->ch[2 * i + 1], P, (uint32_t)r);

  if (p->mode[i] != AP3_MODE_PWM) ap3_set_pair_mode(p, i, AP3_MODE_PWM);
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

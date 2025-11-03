/*
  MARS — Modular Autonomous Robotic System
  Hexapod v2.0 Controller (Teensy 4.1)

  Change log (top N entries)
  - 2025-11-03: TEST: Added runtime command `TEST OVERLAP <pct>` (0..25) with persistence to /config.txt; STATUS shows overlap_pct; version bump to 0.1.58. (author: copilot)
  - 2025-11-03: TEST: Added overlap_pct (config) to yield brief both-tripods stance; version bump to 0.1.56. (author: copilot)
  - 2025-11-03: STATUS: Uptime is now wrap-safe (64-bit accumulator across millis rollover) and printed as D/H/M/S/ms; version bump to 0.1.55. (author: copilot)
  - 2025-11-03: STATUS: Uptime changed to D/H/M/S/ms format; version bump to 0.1.54. (author: copilot)
  - 2025-11-03: STATUS: Added uptime_ms line; TEST: Added LIFT parameter and STATUS lift_y; version bump to 0.1.53. (author: copilot)
  - 2025-11-02: TEST mode UX: Added TEST CYCLE/HEIGHT/BASEX/STEPLEN commands; STATUS shows test parameters; version bump to 0.1.52. (author: copilot)
  - 2025-11-02: STATUS: Added tprobe_us=serial/send/fb/tick when MARS_TIMING_PROBES is enabled; version bump to 0.1.51. (author: copilot)
  - 2025-11-02: UX: Added single-letter command 'I' as a shortcut for MODE IDLE; version bump to 0.1.50. (author: copilot)
  - 2025-11-01: Command behavior: STAND now uses IK to move each leg to a neutral stance at x=BASE_X, y=BASE_Y, z=0 (instead of copying home_cd); version bump to 0.1.49. (author: copilot)
  - 2025-11-01: Commands: Added HOME and SAVEHOME (persist home_cd to SD for enabled, in-service servos); version bump to 0.1.48. (author: copilot)
  - 2025-11-01: STATUS: Added home_cd grid to STATUS output; version bump to 0.1.47. (author: copilot)
  - 2025-11-01: Reliability: OOS logic now counts a failure only if BOTH vin and temp are invalid; added 750 ms startup grace; version bump to 0.1.46. (author: copilot)
  - 2025-11-01: Fix: Closed misplaced brace in loopTick(); restored per-tick logic outside MODE_TEST; version bump to 0.1.45. (author: copilot)
  - 2025-10-31: Config: Added oos.fail_threshold to set OOS failure threshold; version bump to 0.1.44. (author: copilot)
  - 2025-10-31: Reliability: RR/OOS feedback fixes; default joint mask enabled (all 18); bus init uses OE pins. (author: copilot)
  - 2025-10-31: UX: Restored startup splash with ASCII banner, UART mapping summary, and config status; prints briefly and non-blocking. (author: copilot)
  - 2025-10-28: Feature: Tripod test gait mode now moves legs in TEST mode (writes IK results to g_cmd_cd for each leg if IK succeeds). Diagnostics retained. (author: copilot)
  - 2025-10-28: Feature: Added tripod test gait mode (diagnostics only, no movement). Implements time-based tripod phase switching, correct 45° rotation for corner legs, COXA_OFFSET, and full IK/foot target debug output. (author: copilot)
  - 2025-10-28: Memory: narrow feedback/state types (vin→mV u16, temp→0.1C i16, pos i16), probes→u16, rate_limit→cdeg/s u16; STATUS/safety updated. (author: copilot)
  - 2025-10-28: Reliability: add OOS report command and include OOS in STATUS; version bump. (author: copilot)
  - 2025-10-28: Memory: compress OOS flags to a single 18-bit bitmask instead of a bool array. (author: copilot)
  - 2025-10-27: Reliability: auto mark servo out-of-service (OOS) after consecutive feedback errors; ignore until reboot. (author: copilot)
  - 2025-10-27: Config: add home_cd.<LEG>.<joint> and use to init home and startup positions; add basic IK and wire FOOT/FEET. (author: copilot)
  - 2025-10-27: Safety: add soft joint limits (from config) and per-tick rate limiting of joint commands; clamp prior to send. (author: copilot)
  - 2025-10-27: Honor safety overrides during checks (e.g., TEMP override prevents re-lockout). (author: copilot)
  - 2025-10-27: STATUS now shows safety=OK|LOCKOUT|OVERRIDDEN to reflect overrides. (author: copilot)
  - 2025-10-27: SAFETY command: add LIST and OVERRIDE <ALL|TEMP|NONE>; track lockout causes and allow overrides. (author: copilot)
  - 2025-10-27: Add LOCKOUT mode; STATUS first line appends 'SAFETY LOCKOUT' when tripped. (author: copilot)
  - 2025-10-27: Safety: lockout if any servo temp >= 80°C; immediate torque-off; requires SAFETY CLEAR. (author: copilot)
  - 2025-10-27: Use direct bus write for MOVE_TIME (custom helper) instead of LX16AServo::move_time; wired into loopTick. (author: copilot)
  - 2025-10-27: Add compile-time timing probes (serial/send/feedback/tick) and show Robot::SPLASH_BANNER at startup. (author: copilot)
  - 2025-10-26: Optimize loop: remove per-tick torque processing; apply torque immediately in commands. (author: copilot)
  - 2025-10-26: Adaptive rate: -1 Hz after 100 overruns; +1 Hz after 100 on-time ticks (ceiling=configured loop_hz). (author: copilot)
  - 2025-10-26: Overrun handling: decrement loop_hz by 1 Hz on each overrun; adjusts timer at runtime. (author: copilot)
     - 2025-10-26: Loop frequency default set to 100 Hz for overrun testing; splash/STATUS reflect new rate. (author: copilot)
   - 2025-10-26: Round-robin position reads (pos_cd) via LX16AServo::pos_read(); STATUS prints pos grid. (author: copilot)
   - 2025-10-26: Round-robin reads (vin/temp); vin interpreted in mV with wider clamp; STATUS grids show values. (author: copilot)
   - 2025-10-26: Safety: default startup servo targets to 12000 centideg (mid-range) to avoid jumps on enable. (author: copilot)
  - 2025-10-26: Refactor: moved config helpers to functions.ino; geometry to robot_config.h. (author: copilot)
  - 2025-10-26: STATUS now reports legs_active bitstring (leg-enabled OR any joint enabled). (author: copilot)
  - 2025-10-26: RAW/RAW3 gating: allow when globally enabled and either the leg is enabled or the relevant joint(s) are enabled. (author: copilot)
  - 2025-10-25: Servo IO routed through lx16a-servo adapter (enable with MARS_USE_LX16A); raw frames remain as fallback. (author: copilot)
  - 2025-10-25: SD config loader parses /config.txt (loop_hz, servo_id.<LEG>.<joint>); splash shows config status. (author: copilot)
  - 2025-10-25: Global DISABLE queues torque-off for all servos; SERVO torque-on gated by global ENABLE. (author: copilot)
  - 2025-10-25: SERVO command now sends torque load/unload to hardware on enable/disable. (author: copilot)
  - 2025-10-25: STATUS/HELP output now CR/LF-delimited multi-line for readability. (author: copilot)
  - 2025-10-25: Added per-servo (joint) enable gating and SERVO/SERVOS commands; STATUS includes joint mask. (author: copilot)
  - 2025-10-25: Corrected UART mapping: LF→Serial8, LR→Serial5. (author: copilot)
  - 2025-10-25: Default all legs disabled at startup for safety; per-leg enables required. (author: copilot)
  - 2025-10-25: Wired 6 UART servo buses and per-leg half-duplex send wrappers; placeholders for lx16a-servo sends. (author: copilot)
  - 2025-10-25: Fixed signed/unsigned comparison warning in serial buffer check. (author: copilot)
  - 2025-10-25: Simplified leg arrays to canonical order (LF,LM,LR,RF,RM,RR); removed mapping helpers. (author: copilot)
  - 2025-10-25: Added COXA_OFFSET geometry (mm) mapped from user order RF,RM,RR,LR,LM,LF to canonical LF,LM,LR,RF,RM,RR. (author: copilot)
  - 2025-10-25: Added 74HC126 half-duplex buffer control scaffolding with user pin order (RF,RM,RR,LR,LM,LF) and canonical leg mapping. (author: copilot)
  - 2025-10-25: Added firmware version string and surfaced in splash and STATUS. (author: copilot)
  - 2025-10-25: REBOOT command now uses direct AIRCR write (SCB_AIRCR). (author: copilot)
  - 2025-10-25: Added REBOOT serial command (OK then reset). (author: copilot)
  - 2025-10-25: Initial skeleton. Splash, 166 Hz loop shell, ASCII parser with HELP/STATUS and stubs for core commands. (author: copilot)

  Summary
  - Toolchain: Arduino 1.8.19 + Teensyduino (RPi 5)
  - Loop: fixed 166 Hz (≈6.024 ms). Each tick we: (1) compute IK targets (stub), (2) command all servos (stub), (3) read one servo (stub), (4) estimate others (stub), (5) safety checks (stub), (6) optional logging (stub).
  - Serial protocol: ASCII line commands with OK/ERR replies. Implemented: HELP, STATUS. Stubs: ENABLE, DISABLE, STAND, FOOT, FEET, MODE, SAFETY CLEAR.
  - Notes: Keep memory footprint small. Avoid String; use fixed buffers. Non-blocking splash.
*/

// -----------------------------------------------------------------------------
// Top-level includes and feature flags
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ctype.h>
#include "LoopTimer.hpp"
#if MARS_ENABLE_SD
#include <SD.h>
#endif
#include <lx16a-servo.h>
#include <new> // for placement new; avoids heap allocations for servo objects
#include "robot_config.h"

// Prototypes for helpers defined in functions.ino
bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]);
void splash();
void processSerial();
int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen);
void printOK();
void printERR(uint8_t code, const char* msg);
void printSTATUS();
void printHELP();
void rebootNow();
void handleLine(const char* line);
// Mode setters (called from functions.ino)
void modeSetTest();
void modeSetIdle();
// Synchronous position read for SAVEHOME (centidegrees); returns -1 on failure
int16_t readServoPosCdSync(uint8_t leg, uint8_t joint);

// Feature flags (compile-time)
#ifndef MARS_ENABLE_LOGGING
#define MARS_ENABLE_LOGGING 0
#endif
#ifndef MARS_ENABLE_SD
#define MARS_ENABLE_SD 1
#endif
#ifndef MARS_TIMING_PROBES
#define MARS_TIMING_PROBES 1
#endif

// Firmware version (override at compile time with -DFW_VERSION="x.y.z")
#ifndef FW_VERSION
#define FW_VERSION "0.1.58"
#endif

// -----------------------------------------------------------------------------
// Topology and robot state
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Loop timing and control variables
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Safety and lockout state
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Joint command, home, and limit state
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Enable/disable bitmask helpers
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Feedback, telemetry, and OOS tracking
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Hardware pin mappings (see robot_config.h)
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Main setup() and loop() functions
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Core loopTick() logic (runs every control tick)
// -----------------------------------------------------------------------------
#ifndef NUM_LEGS
#define NUM_LEGS 6
#endif

// Canonical leg ordering used across firmware and serial protocol:
// 0: LF, 1: LM, 2: LR, 3: RF, 4: RM, 5: RR
enum LegIdx : uint8_t { LEG_LF = 0, LEG_LM = 1, LEG_LR = 2, LEG_RF = 3, LEG_RM = 4, LEG_RR = 5 };
// Joints per leg
static constexpr uint8_t LEG_SERVOS = 3;
enum JointIdx : uint8_t { JOINT_COXA = 0, JOINT_FEMUR = 1, JOINT_TIBIA = 2 };

// Robot geometry now provided by robot_config.h

// Loop timing
static const uint16_t LOOP_HZ_DEFAULT = 100; // default set lower for overrun testing (spec target is 166)
static volatile uint16_t g_loop_hz = LOOP_HZ_DEFAULT;
static const float TICK_MS_DEFAULT = 1000.0f / LOOP_HZ_DEFAULT; // ~6.024 ms

// ISR-based tick trigger
// Loop timer (polling in loop)
static LoopTimerClass g_loopTimer(LOOP_HZ_DEFAULT);

// ----------------------------------------------------------------------------
// TEST gait runtime parameters (mutable via serial commands)
// ----------------------------------------------------------------------------
// Defaults chosen to match prior constants used in tripod test gait
float    g_test_base_y_mm   = -110.0f;  // ground height (negative is down)
float    g_test_base_x_mm   = 130.0f;   // lateral offset
float    g_test_step_len_mm = 40.0f;    // forward/back amplitude (|z|)
uint32_t g_test_cycle_ms    = 3000;     // ms per tripod phase
float    g_test_lift_y_mm   = 40.0f;    // step height (lift amount on swing)
float    g_test_overlap_pct = 5.0f;     // percent of per-phase time reserved as overlap (both tripods stance)

// System state
enum MarsMode : uint8_t { MODE_IDLE = 0, MODE_TEST = 1, MODE_LOCKOUT = 2 };
static volatile bool g_enabled = false;
static volatile bool g_lockout = false; // safety lockout flag

// Collision detection removed; no externs or forward-declarations here.
static volatile MarsMode g_mode = MODE_IDLE;
static volatile uint8_t g_last_err = 0; // 0 = OK
static volatile uint32_t g_overrun_count = 0; // counts ticks exceeding period
static volatile uint16_t g_overrun_since_adjust = 0; // consecutive overruns since last rate change
static volatile uint16_t g_ok_since_adjust = 0;       // consecutive on-time ticks since last rate change
// Config status (SD)
bool g_config_loaded = false;
uint16_t g_config_keys_applied = 0;
uint16_t g_config_loop_hz = LOOP_HZ_DEFAULT;

// Timing probes (compile-time optional)
#if MARS_TIMING_PROBES
static volatile uint16_t g_probe_serial_us = 0;  // time spent in processSerial() this loop (us)
static volatile uint16_t g_probe_send_us   = 0;  // time to send all enabled servo commands (us)
static volatile uint16_t g_probe_fb_us     = 0;  // time to read one servo feedback (us)
static volatile uint16_t g_probe_tick_us   = 0;  // total loopTick() duration (us)
#endif
// Startup time (for OOS grace window)
static uint32_t g_boot_ms = 0;

// Safety: lockout causes and overrides
enum LockoutCauseBits : uint16_t {
  LOCKOUT_CAUSE_NONE = 0,
  LOCKOUT_CAUSE_TEMP = 1u << 0,
  // Reserved for future causes (soft limit, collision, estop, etc.)
};
static volatile uint16_t g_lockout_causes = LOCKOUT_CAUSE_NONE; // snapshot at lockout
static volatile uint16_t g_override_mask  = LOCKOUT_CAUSE_NONE; // user overrides
// Snapshot of temp trips at lockout time (true if that joint exceeded threshold) and value in 0.1 C
static volatile bool g_lockout_temp_trip[NUM_LEGS][LEG_SERVOS] = { { false } };
static volatile int16_t g_lockout_temp_c10[NUM_LEGS][LEG_SERVOS] = { { 0 } };

// Commanded joint targets (centidegrees) per leg/joint. IK/commands will write here.
static volatile int16_t g_cmd_cd[NUM_LEGS][3] = { {0} };
// Home positions (centidegrees) per leg/joint; loaded from SD config; default 12000
static volatile int16_t g_home_cd[NUM_LEGS][LEG_SERVOS] = {
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000},
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000}
};
// Soft joint limits (absolute centidegrees). Defaults to full range; load from SD config.
static volatile int16_t g_limit_min_cd[NUM_LEGS][LEG_SERVOS] = {
  {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
};
static volatile int16_t g_limit_max_cd[NUM_LEGS][LEG_SERVOS] = {
  {24000, 24000, 24000}, {24000, 24000, 24000}, {24000, 24000, 24000},
  {24000, 24000, 24000}, {24000, 24000, 24000}, {24000, 24000, 24000}
};
// Rate limiting config: max joint speed in centideg/s (applied as per-tick cd delta clamp). Safe default 360.00 deg/s => 36000 cdeg/s
static volatile uint16_t g_rate_limit_cdeg_per_s = 36000u;
// Last sent command (centidegrees) for per-tick rate limiting
static volatile int16_t g_last_sent_cd[NUM_LEGS][LEG_SERVOS] = {
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000},
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000}
};
// Torque control is handled synchronously in command handlers (no per-tick processing)
// Enable masks (bit-packed) to reduce RAM and simplify checks
// Bits 0..5 correspond to legs LF..RR
static volatile uint8_t g_leg_enabled_mask = 0; // 6-bit mask
// Bits 0..17 correspond to (leg*3 + joint)
static volatile uint32_t g_joint_enabled_mask = 0x3FFFFUL; // 18-bit mask: default all joints enabled for bring-up
// Forward declaration (defined later in OOS helpers)
static inline uint8_t servoFlatIndex(uint8_t leg, uint8_t joint);
static inline bool legEnabled(uint8_t leg) {
  return ((g_leg_enabled_mask >> leg) & 1u) != 0;
}
static inline void setLegEnabled(uint8_t leg, bool en) {
  if (leg >= NUM_LEGS) return; if (en) g_leg_enabled_mask |= (1u << leg); else g_leg_enabled_mask &= ~(1u << leg);

}
static inline bool jointEnabled(uint8_t leg, uint8_t joint) {
  uint8_t idx = servoFlatIndex(leg, joint); return ((g_joint_enabled_mask >> idx) & 1u) != 0;
}
static inline void setJointEnabled(uint8_t leg, uint8_t joint, bool en) {
  uint8_t idx = servoFlatIndex(leg, joint); if (en) g_joint_enabled_mask |= (1u << idx); else g_joint_enabled_mask &= ~(1u << idx);
}

// Round-robin index 0..17
static volatile uint8_t g_rr_index = 0;

// Last measured values from sparse feedback (updated round-robin)
static uint16_t g_meas_vin_mV[NUM_LEGS][LEG_SERVOS] = { {0} }; // millivolts
static uint8_t  g_meas_temp_C[NUM_LEGS][LEG_SERVOS] = { {0} }; // temperature in whole deg C
static int16_t  g_meas_pos_cd[NUM_LEGS][LEG_SERVOS] = { {0} }; // 0..24000 centidegrees
// Feedback reliability tracking and out-of-service masking
volatile uint8_t g_fb_fail_count[NUM_LEGS][LEG_SERVOS] = { {0} };
volatile uint32_t g_servo_oos_mask = 0; // 18-bit mask: bit index (leg*3 + joint)
static volatile uint8_t g_servo_fb_fail_threshold = 3; // configurable via /config.txt oos.fail_threshold
// OOS helpers
static inline uint8_t servoFlatIndex(uint8_t leg, uint8_t joint) {
  return (uint8_t)(leg * LEG_SERVOS + joint);
}
static inline bool servoIsOOS(uint8_t leg, uint8_t joint) {
  uint8_t idx = servoFlatIndex(leg, joint);
  return ((g_servo_oos_mask >> idx) & 1u) != 0;
}
static inline void servoMarkOOS(uint8_t leg, uint8_t joint) {
  uint8_t idx = servoFlatIndex(leg, joint);
  g_servo_oos_mask |= (1UL << idx);
}

// Hardware pin mappings moved to robot_config.h (Robot::SERIAL_TX_PINS_CAN, Robot::BUFFER_ENABLE_PINS)

// Error codes (subset per spec)
#define E_OK            0
#define E_PARSE         1  // E01
#define E_UNKNOWN_CMD   2  // E02
#define E_BAD_ARG       3  // E03
#define E_DISABLED      10 // E10
#define E_UNREACHABLE   20 // E20
#define E_SOFT_LIMIT    30 // E30
#define E_COLLISION     40 // E40 (reserved)
#define E_BUS_IO        50 // E50
#define E_OVERRUN       60 // E60
#define E_LOCKOUT       90 // E90
#define E_NOT_LOCKED    91 // E91
// Serial line buffer (externally visible so helpers in functions.ino can use them)
char lineBuf[160];
uint8_t lineLen = 0;

// Servo buses (canonical order: LF, LM, LR, RF, RM, RR)
// Default bus baudrate moved to robot_config.h (Robot::SERVO_BAUD_DEFAULT)
HardwareSerial* SERVO_BUS[NUM_LEGS] = {
  &Serial8, // LF
  &Serial3, // LM
  &Serial5, // LR
  &Serial7, // RF
  &Serial6, // RM
  &Serial2  // RR
};

// Per-servo ID mapping (default: coxa=1, femur=2, tibia=3 for every leg)
static uint8_t SERVO_ID[NUM_LEGS][LEG_SERVOS] = {
  {1, 2, 3}, // LF
  {1, 2, 3}, // LM
  {1, 2, 3}, // LR
  {1, 2, 3}, // RF
  {1, 2, 3}, // RM
  {1, 2, 3} // RR
};

inline uint8_t servoId(uint8_t leg, uint8_t joint) {
  if (leg >= NUM_LEGS || joint >= LEG_SERVOS) return 0;

  return SERVO_ID[leg][joint];
}

void setServoId(uint8_t leg, uint8_t joint, uint8_t id) {
  if (leg < NUM_LEGS && joint < LEG_SERVOS) SERVO_ID[leg][joint] = id;

}

static LX16ABus g_bus[NUM_LEGS];
// servoBusesInit moved to functions.ino

// Pre-allocated LX16AServo objects per leg/joint to avoid per-tick construction overhead
static LX16AServo* g_servo[NUM_LEGS][LEG_SERVOS] = { { nullptr } };
alignas(LX16AServo) static uint8_t g_servo_mem[NUM_LEGS][LEG_SERVOS][sizeof(LX16AServo)];
static void servoObjectsInit() {
  for (uint8_t L = 0; L < NUM_LEGS; ++L)

  {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J)

    {
      g_servo[L][J] = nullptr;
      uint8_t id = servoId(L, J);
      if (id >= 1 && id <= 253 && SERVO_BUS[L])

      {
        void* slot = static_cast<void*>(&g_servo_mem[L][J][0]);
        g_servo[L][J] = new (slot) LX16AServo(&g_bus[L], id);
      }
    }
  }
}

// Immediate torque control helper using cached servo objects
void setServoTorqueNow(uint8_t leg, uint8_t joint, bool on) {
  if (leg >= NUM_LEGS || joint >= LEG_SERVOS) return;

  LX16AServo* s = g_servo[leg][joint];
  if (!s) return;

  if (on) s->enable(); else s->disable();

}

// --- Safety helpers ---
static constexpr int16_t SERVO_TEMP_LOCKOUT_C10 = 800; // 80.0 C in 0.1 C units
static inline void safetyLockoutTemp(uint8_t leg, uint8_t joint, int16_t temp_c10) {
  if (g_lockout) return; // already locked out

  g_lockout = true;
  g_enabled = false;
  g_last_err = E_LOCKOUT;
  g_mode = MODE_LOCKOUT;
  g_lockout_causes |= LOCKOUT_CAUSE_TEMP;
  // Snapshot all servos that are currently above threshold
  for (uint8_t L = 0; L < NUM_LEGS; ++L)

  {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J)

    {
      uint8_t tC = g_meas_temp_C[L][J];
      bool trip = (tC >= (SERVO_TEMP_LOCKOUT_C10 / 10));
      g_lockout_temp_trip[L][J] = trip;
      g_lockout_temp_c10[L][J] = tC * 10; // still store in c10 for lockout logic
    }
  }
  // Immediately torque off all servos
  for (uint8_t L = 0; L < NUM_LEGS; ++L)

  {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J)

    {
      setServoTorqueNow(L, J, false);
    }
  }
}

// Placeholder: integrate lx16a-servo library here to send this leg's 3 joint targets
// Minimal LX-16A compatible frame sender for MOVE_TIME_WRITE (CMD=1)
static inline uint16_t cd_to_lx_units(int cd) {
  if (cd < 0) cd = 0; if (cd > 24000) cd = 24000;

  // LX-16A: 0..1000 units map to ~0..240 degrees → 1 unit ≈ 0.24° = 24 centideg
  return (uint16_t)(cd / 24);
}

// Using lx16a-servo library via LX16ABus::write; helpers keep our units mapping
static inline void sendLegPositions(uint8_t leg) {
  // Skip if bus not available
  if (!SERVO_BUS[leg]) return;

  // Default move time; tune via config later
  uint32_t period_ms = (g_loop_hz > 0) ? (1000UL / (uint32_t)g_loop_hz) : 6UL;
  if (period_ms == 0) period_ms = 1;

  uint16_t move_time_ms = (uint16_t)period_ms;
  if (move_time_ms < 30) move_time_ms = 30; // floor to ensure visible motion on bring-up

  // Send coxa, femur, tibia in order
  for (uint8_t j = 0; j < LEG_SERVOS; ++j)

  {
    if (!jointEnabled(leg, j)) continue; // per-servo gating

    uint8_t id = servoId(leg, j);
    if (id == 0x00 || id == 0xFF) continue; // invalid

    int cd = g_cmd_cd[leg][j];
    uint16_t units = cd_to_lx_units(cd);
    uint8_t params[] = { (uint8_t)(units & 0xFF), (uint8_t)(units >> 8), (uint8_t)(move_time_ms & 0xFF), (uint8_t)(move_time_ms >> 8) };
    (void)g_bus[leg].write(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, id);
  }
}

// Direct bus MOVE_TIME helper for a single servo (avoids LX16AServo::move_time overhead)
static inline void busMoveTimeWrite(uint8_t leg, uint8_t joint, int16_t cd, uint16_t move_time_ms) {
  if (leg >= NUM_LEGS || joint >= LEG_SERVOS) return;

  if (!SERVO_BUS[leg]) return;

  uint8_t id = servoId(leg, joint);
  if (id == 0x00 || id == 0xFF) return;

  if (move_time_ms < 1) move_time_ms = 1;

  uint16_t units = cd_to_lx_units(cd);
  uint8_t params[4] = {
    (uint8_t)(units & 0xFF), (uint8_t)(units >> 8),
    (uint8_t)(move_time_ms & 0xFF), (uint8_t)(move_time_ms >> 8)
  };
  (void)g_bus[leg].write(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, id);
}

// Forward decls (only those local to this TU)
static void loopTick();
// Config (implemented in functions.ino)
void configLoad();
// Hardware helpers; buffersInit implemented in functions.ino; servoBusesInit defined here
void buffersInit();
static void servoBusesInit(uint32_t baud = Robot::SERVO_BAUD_DEFAULT);
// no geometryInit needed with canonical-order arrays

// Morphology offsets available as Robot::COXA_OFFSET (see robot_config.h)

// Mode setters (exposed so command handler can change modes)
void modeSetTest() { g_mode = MODE_TEST; }
void modeSetIdle() { g_mode = MODE_IDLE; }

int16_t readServoPosCdSync(uint8_t leg, uint8_t joint) {
  if (leg >= NUM_LEGS || joint >= LEG_SERVOS) return -1;
  LX16AServo* s = g_servo[leg][joint];
  if (!s) return -1;
  int32_t pos = s->pos_read();
  if (pos < 0) return -1;
  if (pos > 24000) pos = 24000;
  return (int16_t)pos;
}

void setup() {
  // Optional brief settle to avoid early contention before USB enumeration and IO setup
  delay(500);

  Serial.begin(115200);
  delay(1000); // allow time for Serial to initialize

  // Load config (SD) before splash so status reflects applied values
  configLoad();
  // Do not wait for Serial; print if available
  splash();

  // Safety: initialize commanded joint targets to mid-range (12000 centidegrees ≈ 120.00°)
  for (uint8_t L = 0; L < NUM_LEGS; ++L)

  {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J)

    {
      g_cmd_cd[L][J] = g_home_cd[L][J];
      g_last_sent_cd[L][J] = g_home_cd[L][J];
    }
  }

  buffersInit();
  servoBusesInit();
  servoObjectsInit();
  g_boot_ms = millis();
  // Configure loop timer from loop_hz (default now 100 Hz, spec target 166 Hz)
  g_loopTimer.SetFrequency(g_loop_hz);
}

static void servoBusesInit(uint32_t baud) {
  (void)baud;
  for (uint8_t i = 0; i < NUM_LEGS; ++i)

  {
    if (SERVO_BUS[i])

    {
      // Initialize underlying HardwareSerial and half-duplex control pins
      SERVO_BUS[i]->begin(Robot::SERVO_BAUD_DEFAULT);
      g_bus[i].begin(SERVO_BUS[i], Robot::SERIAL_TX_PINS_CAN[i], Robot::BUFFER_ENABLE_PINS[i]);
    }
  }
}

void loop() {
  // Serial processing is cheap; handle every loop iteration
#if MARS_TIMING_PROBES
  elapsedMicros serial_us = 0;
#endif
  processSerial();
#if MARS_TIMING_PROBES
  g_probe_serial_us = (uint32_t)serial_us;
#endif

  if (g_loopTimer.Update()) {
    loopTick();
  }
}

static void loopTick() {
  // Measure tick execution time for overrun guard (start at the very top)
  uint32_t tick_start_us = micros();
  // --- [TRIPOD TEST GAIT: DIAGNOSTICS ONLY, WITH CORNER LEG ROTATION] ---
  // Implements a simple tripod gait for diagnostics. No movement until math is verified.
  // - Y position is negative (down)
  // - Step size <100mm
  // - Corner legs (LF, LR, RF, RR) are rotated 45° outwards
  // - Middle legs (LM, RM) are aligned with body axes
  // - COXA_OFFSET is applied per leg
  // - Uses IK for all legs
  // - Extensive Serial debug output
  static uint32_t test_last_ms = 0;
  static uint8_t test_phase = 0;
  static uint32_t phase_start_ms;
  // static const uint8_t tripodB[3] = {LEG_LF, LEG_RM, LEG_LR}; // unused
  static const uint8_t tripodA[3] = {LEG_RF, LEG_LM, LEG_RR};
  // Axis-aligned tripod gait parameters (IK: X=lateral, Y=up, Z=forward)
  // Values are taken from mutable globals to allow runtime tuning via serial commands
  const float    BASE_Y  = g_test_base_y_mm;   // mm, ground (negative is down)
  const float    BASE_X  = g_test_base_x_mm;   // mm, lateral offset
  const float    STEP_Z  = g_test_step_len_mm; // mm, forward/back amplitude
  // static const float STEP_Z_INC = 0.0f;     // mm, lateral (unused)
  const float    LIFT_Y  = g_test_lift_y_mm;   // mm, up for swing
  const uint32_t PHASE_MS = g_test_cycle_ms;   // ms per tripod phase
  // Removed unused foot_targets array to silence compiler warning
  //10968/11688/14784
  // Only run in MODE_TEST
  if (g_mode == MODE_TEST)
  {
    if (test_last_ms == 0)
    {
      test_last_ms = millis();
      phase_start_ms = test_last_ms;
    }
    uint32_t now = millis();
    if (now - test_last_ms > PHASE_MS)
    {

      test_last_ms = now;
      test_phase ^= 1; // alternate phase
      phase_start_ms = test_last_ms;
    }

  // determine step progression with single overlap window at end-of-phase
  // overlap_pct is defined as percent of TOTAL CYCLE; per transition window = r * PHASE_MS
  uint32_t elapsed = now - phase_start_ms;
  uint32_t overlap_ms = (uint32_t)(g_test_overlap_pct * 0.01f * (float)PHASE_MS);
  if (overlap_ms > PHASE_MS) overlap_ms = PHASE_MS; // cap to phase length
  uint32_t active_ms = PHASE_MS - overlap_ms; // active swing progression runs from 0..active_ms
  float t_active = (elapsed >= active_ms) ? 1.0f : ((active_ms > 0) ? ((float)elapsed / (float)active_ms) : 0.0f);
  float step_offset = 2.0f * STEP_Z * t_active;

    // Set foot targets for each leg
    // removed unused temp_out_cd and ik_ok_all
    for (uint8_t L = 0; L < NUM_LEGS; ++L)
    {

      // --- Tripod gait with correct IK axes: X=lateral, Y=up (neg=down), Z=forward ---
      // Step is along Z (forward/back), lift is along Y (up/down), lateral is X
      float x = BASE_X; // lateral
      float y = BASE_Y; // up/down (negative is down)
      float z = 0; // forward/back
      float xprime = x; // for rotation adjustment
      float zprime = z; // for rotation adjustment

      // Determine which tripod and phase
      bool isA = (L == tripodA[0] || L == tripodA[1] || L == tripodA[2]);
      //Serial << L << F(" isA=") << isA << F(" phase=") << (int)test_phase << F(" step_offset=") << step_offset << F("\n");
      bool assignedSwing = ((isA && test_phase == 0) || (!isA && test_phase == 1));
      bool inOverlap = (elapsed >= active_ms); // only one window at end-of-phase
      // Z progression is continuous regardless of overlap; Y (lift) is gated off during overlap
      if (assignedSwing) {
        z = -STEP_Z + step_offset; // swing path from -STEP_Z to +STEP_Z
        y = inOverlap ? BASE_Y : (BASE_Y + LIFT_Y);
      } else {
        z = +STEP_Z - step_offset; // stance path from +STEP_Z to -STEP_Z
        y = BASE_Y; // always on ground in stance
      }

      // --- Apply 45° rotation for corner legs in X/Z plane ---
      // Canonical leg order: 0:LF, 1:LM, 2:LR, 3:RF, 4:RM, 5:RR
      // removed unused x_rot/z_rot
      // if (L == LEG_RF || L == LEG_LR)
      // {
      //   // Left corners: rotate +45° in X/Z
      //   float c = 0.70710678f, s = 0.70710678f; // cos(45°), sin(45°)
      //   float len = BASE_X - c * BASE_X;
      //   xprime = BASE_X - c * len;
      //   zprime = s * len;
      //   zprime += s * z;
      //   xprime += -c * z;
      // } else if (false || L == LEG_LF || L == LEG_RR) {
      switch (L)
      {

        case LEG_RR:
        case LEG_LR:  // good
          // Left corners: rotate -45° in X/Z
          {
            float c = 0.70710678f, s = -0.70710678f; // cos(45°), -sin(45°)
            xprime = -c * z + (c + 0.5f) * g_test_base_x_mm;
            zprime =  c * z + 0.5f       * g_test_base_x_mm;
          }
          break;
        case LEG_LF:  // good
        case LEG_RF:
          // Right corners: rotate +45° in X/Z
          {
            float c = 0.70710678f, s = 0.70710678f; // cos(45°), sin(45°)
            xprime =  c * z + (c + 0.5f) * g_test_base_x_mm;
            zprime =  c * z - 0.5f       * g_test_base_x_mm;
          }
          break;
      default:
        // Middle legs: no rotation
        xprime = x;
        zprime = z;
        break;
      }
      //if (L == LEG_LF || L == LEG_RR) {

      //   // Right corners: rotate -45° in X/Z
      //   float c = 0.70710678f, s = -0.70710678f; // cos(45°), -sin(45°)
      //   if (L == LEG_LF)
      //   {
      //     Serial << _FLOAT(x,4) << F(",") << _FLOAT(y,4) << F(",") << _FLOAT(z,4) << F(",");
      //   }
      //   xprime = c * z + (c + 0.5) * BASE_X;
      //   zprime = c * z - 0.5 * BASE_X;
      //   if (L == LEG_LF)
      //   {
      //     Serial << _FLOAT(xprime,4) << F(",") << _FLOAT(y,4) << F(",") << _FLOAT(zprime,4) << F("\n");
      //   }
      // }

      // Compute IK for this leg
      int16_t out_cd[3] = {0};
      bool ik_ok = calculateIK(L, xprime, y, zprime, out_cd);
      if (ik_ok)
      {
        if (L == LEG_RF || L == LEG_RM || L == LEG_RR)
        {
          g_cmd_cd[L][0] = out_cd[0];
          g_cmd_cd[L][1] = out_cd[1];
          g_cmd_cd[L][2] = out_cd[2];
        }
        else
        {
          g_cmd_cd[L][0] = (int16_t)(2 * g_home_cd[L][0] - out_cd[0]);
          g_cmd_cd[L][1] = (int16_t)(2 * g_home_cd[L][1] - out_cd[1]);
          g_cmd_cd[L][2] = (int16_t)(2 * g_home_cd[L][2] - out_cd[2]);
        }

        // debug output
        // Serial.print(F("[TRIGAIT] IK OK  leg=")); Serial.print((int)L);
        // Serial.print(F(" gcmd[0]=")); Serial.print(g_cmd_cd[L][0],1);
        // Serial.print(F(" gcmd[1]=")); Serial.print(g_cmd_cd[L][1],1);
        // Serial.println(F(" gcmd[2]=")); Serial.print(g_cmd_cd[L][2],1);
      } else {
        // message about IK failure
        Serial.print(F("[TRIGAIT] IK FAIL leg=")); Serial.print((int)L);
        Serial.print(F(" x=")); Serial.print(xprime,1);
        Serial.print(F(" y=")); Serial.print(y,1);
        Serial.println(F(" z=")); Serial.print(zprime,1);
      }
      // IK and diagnostics
      // Serial.print(F("[TRIGAIT] phase=")); Serial.print((int)test_phase);
      // Serial.print(F(" t=")); Serial.print((unsigned long)millis());
      // Serial.print(F(" loop_us="));
#if MARS_TIMING_PROBES
      //Serial.print((unsigned long)g_probe_tick_us);
#else
      Serial.print("NA");
#endif
      //Serial.print(F("\r\n"));
      //for (uint8_t L = 0; L < NUM_LEGS; ++L) {
      // int16_t out_cd[3] = {0};
      // bool ik_ok = calculateIK(L, foot_targets[L][0], foot_targets[L][1], foot_targets[L][2], out_cd);
      // Serial.print(F("  LEG "));
      // const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
      // Serial.print(names[L]);
      // Serial.print(F(": x=")); Serial.print(foot_targets[L][0],1);
      // Serial.print(F(" y=")); Serial.print(foot_targets[L][1],1);
      // Serial.print(F(" z=")); Serial.print(foot_targets[L][2],1);
      // Serial.print(F(" | IK: "));
      // if (ik_ok) {
      //   Serial.print(F("OK "));
      //   Serial.print(F("c=")); Serial.print(out_cd[0]);
      //   Serial.print(F(" f=")); Serial.print(out_cd[1]);
      //   Serial.print(F(" t=")); Serial.print(out_cd[2]);
      //   // Enable movement: write joint commands

      // } else {
      //   Serial.print(F("FAIL"));
      // }
      // Serial.print(F("\r\n"));
      //}
      // Now legs will move in test mode if IK is valid
      //return;
    }
  }
  // tick_start_us already captured at function entry
  // 166 Hz budget guide (~6.024 ms):
  //  - 0.5 ms: read serial and parse (best-effort)
  //  - 0.8 ms: compute IK for active targets (stub)
  //  - 1.5 ms: send commands to all servos (stub)
  //  - 1.0 ms: read one servo feedback (stub)
  //  - 0.3 ms: estimator update (stub)
  //  - 0.4 ms: safety checks (stub)
  //  - 0.4 ms: optional logging (stub)
  //  - 0.6 ms: slack/jitter margin

  // Advance round-robin index 0..17 without modulo to avoid overflow concerns
  if (++g_rr_index >= 18) g_rr_index = 0;

  // Example half-duplex buffer control for each leg (skeleton):
  // 1) Enable TX, send commands on that bus; 2) Return to RX immediately after.
  {
#if MARS_TIMING_PROBES
    elapsedMicros send_us = 0;
#endif
    if (g_enabled)

    {
      // TODO(perf): Optimize servo command send path. Current measurements suggest ~1.3 ms per command in some cases.
      //  - Investigate per-call overhead in lx16a bus write and serial driver.
      //  - Consider prebuilding frames, minimizing copies, batching writes, increasing baud, or DMA/queue techniques.
      //  - Target: << 1.3 ms per command (aim for < 300 µs at 115200) to comfortably hit 166 Hz with headroom.
      // Derive a small move time from current loop rate; floor to 1 ms
      // Bitmask logic: each bit in g_leg_enabled_mask/g_joint_enabled_mask represents enable state for a leg/joint
      // Clamp commanded joint to soft joint limits (from config)
      // Rate limit: prevent joint from moving too fast per tick
      // Update last sent value for rate limiting
      // (busMoveTimeWrite would send the command to the servo here)
      // End of per-servo command loop
      // End of servo command send block
      // Read one servo's feedback (round-robin): position (cd), voltage (mV), and temperature (C)
      // Only update feedback if servo is not out-of-service (OOS)
      // Convert voltage to mV, clamp to uint16_t range
      // Convert temperature to 0.1C, clamp to int16_t range
      // Reset feedback failure count on valid read
      // If feedback is invalid, increment failure count and mark OOS if threshold reached
      // Print OOS event to serial for diagnostics
      // End of feedback read block
      // Safety check: lockout on over-temperature (unless TEMP cause is overridden)
      // Overrun guard and adaptive loop rate adjustment
      // End of loopTick()
      // -----------------------------------------------------------------------------
      // Configuration loading (SD /config.txt)
      // -----------------------------------------------------------------------------
      uint32_t period_ms = (g_loop_hz > 0) ? (1000UL / (uint32_t)g_loop_hz) : 6UL;
      if (period_ms == 0) period_ms = 1;

      uint16_t move_time_ms = (uint16_t)period_ms;
      if (move_time_ms < 1) move_time_ms = 1;

      // Compute per-tick delta clamp in centidegrees from deg/s and current loop rate
      int32_t max_delta_cd = 0;
      if (g_loop_hz > 0 && g_rate_limit_cdeg_per_s > 0)

      {
        uint32_t cd_per_tick = (uint32_t)g_rate_limit_cdeg_per_s / (uint32_t)g_loop_hz;
        if (cd_per_tick < 1u) cd_per_tick = 1u; // at least 1 cd/tick

        if (cd_per_tick > 10000u) cd_per_tick = 10000u; // sanity cap

        max_delta_cd = (int32_t)cd_per_tick;
      }
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg)

      {
        if (!legEnabled(leg)) continue; // per-leg gating

        for (uint8_t j = 0; j < LEG_SERVOS; ++j)

        {
          if (!jointEnabled(leg, j)) continue; // per-joint gating

          if (servoIsOOS(leg, j)) continue; // skip OOS servos

          //if (j == 0) g_bus[leg].debug(TRNG_DEFAULT_FREQUENCY_MINIMUM);
          // 1) Soft-limit clamp (absolute cd bounds from config)
          int32_t target_cd = g_cmd_cd[leg][j];
          int16_t min_cd = g_limit_min_cd[leg][j];
          int16_t max_cd = g_limit_max_cd[leg][j];
          if (target_cd < min_cd) target_cd = min_cd;

          if (target_cd > max_cd) target_cd = max_cd;

          // 2) Rate limit per tick around last sent value
          int32_t prev_cd = g_last_sent_cd[leg][j];
          int32_t delta = target_cd - prev_cd;
          if (max_delta_cd > 0)

          {
            if (delta > max_delta_cd) delta = max_delta_cd;

            else if (delta < -max_delta_cd) delta = -max_delta_cd;

          }
          int16_t out_cd = (int16_t)(prev_cd + delta);
          g_last_sent_cd[leg][j] = out_cd;
          //Serial << "<" << leg << "," << j << "> " << out_cd << " -> " << target_cd << " (" << move_time_ms << " ms)\r\n";
          busMoveTimeWrite(leg, j, target_cd, 0 ); //move_time_ms);
          //if (j == 0) g_bus[leg].debug(false);
        }
      }
    }
#if MARS_TIMING_PROBES
    g_probe_send_us = (uint16_t)send_us;
#endif
  }

  // Read one servo's feedback (round-robin): position (cd), voltage (mV), and temperature (C)
  {
    uint32_t fb_start_us = 0;
#if MARS_TIMING_PROBES
    fb_start_us = micros();
#endif
    uint8_t leg = g_rr_index / LEG_SERVOS;
    uint8_t joint = g_rr_index % LEG_SERVOS;
    if (leg < NUM_LEGS && joint < LEG_SERVOS) {
      if (!servoIsOOS(leg, joint)) {
        LX16AServo* s = g_servo[leg][joint];
        if (s) {
          // Position read is optional and does not drive OOS
          int32_t pos_cd = s->pos_read();
          if (pos_cd >= 0) {
            if (pos_cd > 24000) pos_cd = 24000;
            g_meas_pos_cd[leg][joint] = (int16_t)pos_cd;
          }

          // vin/temp read drive OOS detection
          uint16_t vin_mV = s->vin();
          uint8_t  temp_C = s->temp();
          g_meas_vin_mV[leg][joint] = vin_mV;
          g_meas_temp_C[leg][joint] = temp_C;

          // Validity: accept either reading being sane as a 'success'
          bool validVin  = (vin_mV > 0); // lenient: any non-zero
          bool validTemp = (temp_C > 0 && temp_C < 400); // 1..119 C

          // Startup grace window: don't count failures too early after boot
          const uint16_t OOS_GRACE_MS = 750; // ignore failures for first 0.75s after setup
          bool inGrace = (millis() - g_boot_ms) < OOS_GRACE_MS;

          if (validVin || validTemp || inGrace) {
            g_fb_fail_count[leg][joint] = 0; // success or grace period
          } else {
            uint8_t c = g_fb_fail_count[leg][joint];
            if (c < 255) ++c;
            g_fb_fail_count[leg][joint] = c;
            if (c >= g_servo_fb_fail_threshold && !servoIsOOS(leg, joint)) {
              servoMarkOOS(leg, joint);
              g_last_err = E_BUS_IO;
              if (Serial) {
                Serial.print(F("OOS "));
                const char* legNames[6]  = {"LF", "LM", "LR", "RF", "RM", "RR"};
                const char* jointNames[3] = {"COXA", "FEMUR", "TIBIA"};
                Serial.print(legNames[leg]); Serial.print("/"); Serial.print(jointNames[joint]);
                Serial.print(F(" after ")); Serial.print((int)c); Serial.print(F(" vin/temp failures\r\n"));
              }
            }
          }
        }
      }
    }
#if MARS_TIMING_PROBES
    g_probe_fb_us = (uint16_t)(micros() - fb_start_us);
#endif
  }

  // Safety check: lockout on over-temperature (unless TEMP cause is overridden)
  if (!g_lockout)

  {
    bool temp_overridden = (g_override_mask & LOCKOUT_CAUSE_TEMP) != 0;
    if (!temp_overridden)

    {
      for (uint8_t L = 0; L < NUM_LEGS && !g_lockout; ++L)

      {
        for (uint8_t J = 0; J < LEG_SERVOS; ++J)

        {
          if (g_meas_temp_C[L][J] >= (SERVO_TEMP_LOCKOUT_C10 / 10))

          {
            safetyLockoutTemp(L, J, g_meas_temp_C[L][J] * 10);
            break;
          }
        }
      }
    }
  }

  // Overrun guard and adaptive loop rate adjustment
  uint32_t period_us = (g_loop_hz > 0) ? (1000000UL / (uint32_t)g_loop_hz) : 0;
  uint32_t tick_elapsed_us = micros() - tick_start_us;
  if (period_us > 0 && tick_elapsed_us > period_us)

  {
    g_last_err = E_OVERRUN;
    ++g_overrun_count;
    // Count consecutive overruns; step down by 1 Hz only after 100 misses
    if (++g_overrun_since_adjust >= 100)

    {
      g_overrun_since_adjust = 0;
      g_ok_since_adjust = 0;
      if (g_loop_hz > 30)

      {
        --g_loop_hz;
        g_loopTimer.SetFrequency(g_loop_hz);
      }
    }
  } else {
    // On-time tick: count consecutive OKs; step up by 1 Hz after 100 OKs (toward configured target)
    if (++g_ok_since_adjust >= 100)

    {
      g_ok_since_adjust = 0;
      g_overrun_since_adjust = 0;
      uint16_t ceiling = g_config_loop_hz ? g_config_loop_hz : 166;
      if (ceiling > 500) ceiling = 500;

      if (g_loop_hz < ceiling)

      {
        ++g_loop_hz;
        g_loopTimer.SetFrequency(g_loop_hz);
      }
    }
  }
#if MARS_TIMING_PROBES
  g_probe_tick_us = (uint16_t)tick_elapsed_us;
#endif
}
// moved helpers to functions.ino

// --- Configuration loading (SD /config.txt) ---
// Config helpers moved to functions.ino

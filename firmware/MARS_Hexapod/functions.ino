// =============================================================
// functions.ino — Helper and Non-Core Functions
// Minimal post-refactor content: serial dispatch shim + buffer init.
// All command implementations live in commandprocessor.ino.
// =============================================================

#include <Arduino.h>
#include <ctype.h>
#include "command_helpers.h"
#include "command_types.h"
#include "robot_config.h"
// Servo library header provides hardware angle offset helpers
#include <lx16a-servo.h>
// For rebootNow() register access (ARM SCB AIRCR)
#include <stdint.h>
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#include <SD.h>
#endif

// -----------------------------------------------------------------------------
// Forward Kinematics (FK) implementations (body + leg frame)
// -----------------------------------------------------------------------------
// These definitions were previously removed during refactor; reintroduced to
// satisfy linker references from updateFootBodyEstimates() and telemetry.
// Lightweight approximation consistent with current IK conventions:
//  - Joint angles are absolute centidegrees; converted relative to g_home_cd.
//  - Coxa yaw rotates around +Y (up) axis.
//  - Femur & tibia pitch in sagittal plane; simple 2-link chain.
//  - Leg frame origin: hip yaw axis; +x lateral, +y up, +z forward.
//  - Body frame translation by Robot::COXA_OFFSET[leg] (x,z).
// Accuracy: Adequate for collision and telemetry; refine later if needed.

extern volatile int16_t g_home_cd[6][3];
extern volatile uint16_t g_loop_hz;
extern bool     g_config_loaded;
extern uint16_t g_config_keys_applied;
extern uint16_t g_config_loop_hz;
extern volatile uint8_t  g_fk_stream_mask;
// Test mode parameters
extern float    g_test_base_y_mm;
extern float    g_test_base_x_mm;
extern float    g_test_step_len_mm;
extern uint32_t g_test_cycle_ms;
extern float    g_test_lift_y_mm;
extern float    g_test_overlap_pct;
// Safety parameters
extern volatile bool    g_safety_soft_limits_enabled;
extern volatile bool    g_safety_collision_enabled;
extern volatile int16_t g_safety_temp_lockout_c10;
extern float            g_safety_clearance_mm;
// OOS and offsets
extern volatile uint32_t g_servo_oos_mask;
extern int16_t g_offset_cd[6][3];
// Last measured values from sparse feedback (updated round-robin)
extern uint16_t g_meas_vin_mV[6][3]; // millivolts
extern uint8_t  g_meas_temp_C[6][3]; // temperature in whole deg C
extern int16_t  g_meas_pos_cd[6][3]; // 0..24000 centidegrees
extern volatile uint8_t g_meas_pos_valid[6][3];
extern volatile uint8_t g_tuck_active;
extern volatile uint8_t g_tuck_mask;
extern volatile uint8_t g_tuck_done_mask;
extern volatile int16_t g_last_sent_cd[6][3];
extern volatile int16_t  g_tuck_tibia_cd;
extern volatile int16_t  g_tuck_femur_cd;
extern volatile int16_t  g_tuck_coxa_cd;
extern volatile int16_t  g_tuck_tol_tibia_cd;
extern volatile int16_t  g_tuck_tol_other_cd;
extern volatile uint16_t g_tuck_timeout_ms;
// Globals from main sketch
extern volatile bool g_enabled;
extern volatile bool g_lockout;
extern volatile uint16_t g_lockout_causes;
extern volatile uint16_t g_override_mask;
extern volatile uint64_t g_uptime_ms64;
extern volatile uint8_t g_last_err;
extern volatile uint32_t g_overrun_count;
extern volatile uint8_t g_rr_index;
extern volatile uint8_t g_servo_fb_fail_threshold;
extern volatile int16_t g_limit_min_cd[6][3];
extern volatile int16_t g_limit_max_cd[6][3];
extern volatile uint16_t g_rate_limit_cdeg_per_s;
extern void setServoId(uint8_t leg, uint8_t joint, uint8_t id);
extern uint8_t servoId(uint8_t leg, uint8_t joint);
// PID globals from main sketch
extern volatile bool     g_pid_enabled;
extern volatile uint16_t g_pid_kp_milli[3];
extern volatile uint16_t g_pid_ki_milli[3];
extern volatile uint16_t g_pid_kd_milli[3];
extern volatile uint16_t g_pid_kd_alpha_milli[3];
extern volatile uint8_t  g_pid_mode; // 0=active,1=shadow
extern volatile uint16_t g_pid_shadow_report_hz;
// Estimator globals
extern volatile uint16_t g_est_cmd_alpha_milli;
extern volatile uint16_t g_est_meas_alpha_milli;
extern volatile uint16_t g_est_meas_vel_alpha_milli;
// Impedance globals (access to MarsImpedance instance)
#include "MarsImpedance.hpp"
extern MarsImpedance g_impedance;
// Command handlers provided by commandprocessor.ino
void processCmdLIMITS(const char* line, int s, int len);

// Joint compliance globals (defined in MARS_Hexapod.ino)
extern bool     g_comp_enabled;
extern uint16_t g_comp_kp_milli[3];
extern uint16_t g_comp_range_cd[3];
extern uint16_t g_comp_deadband_cd[3];
extern uint16_t g_comp_leak_milli;

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
// Logging globals (defined in MARS_Hexapod.ino)
extern volatile bool     g_log_enabled;
extern volatile uint16_t g_log_rate_hz;
extern volatile uint8_t  g_log_sample_div;
extern volatile uint32_t g_log_tick_counter;
extern volatile uint8_t  g_log_mode;
extern volatile bool     g_log_header;
#endif

  // Forward declaration for loop Hz apply helper in main sketch
void configApplyLoopHz(uint16_t hz);

// Local helpers
static inline void rtrim(char* s)
{
  if (!s) return;

  int n = (int)strlen(s);
  while (n > 0)

  {
    char c = s[n - 1];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n')

    {
      s[n - 1] = 0;
      --n;
    }
    else

    {
      break;
    }
  }
}

static inline char* ltrim(char* s)
{
  if (!s) return s;

  while (*s == ' ' || *s == '\t')

  {
    ++s;
  }

  return s;
}

// Parse common boolean forms: true/false, on/off, 1/0 (case-insensitive)
static inline bool parse_bool(const char* v) {
  if (!v) return false;
  // Skip leading spaces
  while (*v==' '||*v=='\t') ++v;
  char c0 = (char)tolower(*v);
  if (c0=='1') return true;
  if (c0=='0') return false;
  // Case-insensitive literal checks without strncasecmp (Arduino-safe)
  // true
  if ((tolower(v[0])=='t') && (tolower(v[1])=='r') && (tolower(v[2])=='u') && (tolower(v[3])=='e')) return true;
  // on
  if ((tolower(v[0])=='o') && (tolower(v[1])=='n') && (v[2]==0 || v[2]=='\n' || v[2]=='\r')) return true;
  // false
  if ((tolower(v[0])=='f') && (tolower(v[1])=='a') && (tolower(v[2])=='l') && (tolower(v[3])=='s') && (tolower(v[4])=='e')) return false;
  // off
  if ((tolower(v[0])=='o') && (tolower(v[1])=='f') && (tolower(v[2])=='f')) return false;
  // Fallback: non-zero numeric
  long n = atol(v);
  return n != 0;
}

// Centidegrees → radians helper
static inline float cd_to_rad(int16_t cd_rel) {
  const float PI_F = 3.14159265358979323846f;
  return (float)cd_rel * (0.01f * PI_F / 180.0f); // deg = cd*0.01; rad = deg*(pi/180)
}

static inline int16_t rad_to_cd(float rad) {
  const float PI_F = 3.14159265358979323846f;
  float deg = rad * (180.0f / PI_F);
  long cd = lroundf(deg * 100.0f);
  if (cd < 0) cd = 0; if (cd > 24000) cd = 24000;
  return (int16_t)cd;
}

// Clamp centidegree value to valid servo range [0, 24000]
static inline int16_t clamp_cd(int32_t v) {
  if (v < 0) v = 0;
  else if (v > 24000) v = 24000;
  return (int16_t)v;
}

bool fk_leg_body(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                 float* out_x_mm, float* out_y_mm, float* out_z_mm) {
  if (leg >= 6 || !out_x_mm || !out_y_mm || !out_z_mm) return false;
  // Relative angles (centideg from home) → radians
  int16_t coxa_rel  = (int16_t)(coxa_cd  - g_home_cd[leg][0]);
  int16_t femur_rel = (int16_t)(femur_cd - g_home_cd[leg][1]);
  int16_t tibia_rel = (int16_t)(tibia_cd - g_home_cd[leg][2]);
  float yaw   = cd_to_rad(coxa_rel);
  float alpha = cd_to_rad(femur_rel);      // femur pitch
  float beta  = cd_to_rad(tibia_rel);      // tibia relative pitch

  // Link lengths
  const float a = FEMUR_LENGTH_MM;
  const float b = TIBIA_LENGTH_MM;

  // Planar projection (simplified elbow-down assumption)
  float y = -(a * cosf(alpha) + b * cosf(alpha + beta)); // up is +Y; robot foot down → negative
  float R =  (a * sinf(alpha) + b * sinf(alpha + beta)); // horizontal (sagittal plane) radius from femur base

  // Add coxa standoff
  float rproj = COXA_LENGTH_MM + R;

  // Rotate by yaw and translate to body frame using chassis offsets
  float x_local = rproj * sinf(yaw);
  float z_local = rproj * cosf(yaw);
  float bx = Robot::COXA_OFFSET[leg][0];
  float bz = Robot::COXA_OFFSET[leg][1];

  *out_x_mm = bx + x_local;
  *out_y_mm = y;            // body frame y (vertical)
  *out_z_mm = bz + z_local;
  return true;
}

bool fk_leg_both(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                 float* out_body_x_mm, float* out_body_y_mm, float* out_body_z_mm,
                 float* out_leg_x_mm,  float* out_leg_y_mm,  float* out_leg_z_mm) {
  if (!out_body_x_mm || !out_body_y_mm || !out_body_z_mm || !out_leg_x_mm || !out_leg_y_mm || !out_leg_z_mm)
    return false;
  float bx, by, bz;
  if (!fk_leg_body(leg, coxa_cd, femur_cd, tibia_cd, &bx, &by, &bz)) return false;
  // Leg-frame components: remove chassis translation
  float lx = bx - Robot::COXA_OFFSET[leg][0];
  float lz = bz - Robot::COXA_OFFSET[leg][1];
  float ly = by; // same vertical component
  *out_body_x_mm = bx; *out_body_y_mm = by; *out_body_z_mm = bz;
  *out_leg_x_mm  = lx; *out_leg_y_mm  = ly; *out_leg_z_mm  = lz;
  return true;
}

// -----------------------------------------------------------------------------
// IK — compute servo centidegrees for a BODY-frame foot target
// Returns absolute centidegrees for a right-side leg convention; left legs will
// be mirrored by the caller around g_home_cd to maintain symmetry.
// -----------------------------------------------------------------------------
bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]) {
  if (leg >= 6 || !out_cd) return false;

  // In this phase, assume body frame aligned with coxa frame
  float dx = x_mm;
  float dy = y_mm;
  float dz = z_mm;

  float coxa_cdeg = (degrees(atan2f(dx, dz)) * 100.0f) + (float)g_home_cd[leg][0] - 9000.0f;
  float R = sqrtf(dx * dx + dz * dz) - COXA_LENGTH;
  if (R < 0.0f) R = 0.0f;

  float D = sqrtf(R * R + dy * dy);
  float a = FEMUR_LENGTH, b = TIBIA_LENGTH;
  if (D > (a + b) || D < fabsf(a - b)) return false;

  float alpha1 = asinf(R / D);
  float alpha2 = acosf((D * D + a * a - b * b) / (2.0f * a * D));
  float alpha = alpha1 + alpha2;
  float femur_cdeg = degrees(radians((g_home_cd[leg][1] / 100.0f) - 90.0f) + alpha) * 100.0f;

  float beta = acosf((b * b + a * a - D * D) / (2.0f * a * b));
  float gamma = PI - beta;
  float tibia_cdeg = degrees(radians(g_home_cd[leg][2] / 100.0f) + gamma) * 100.0f;

  out_cd[0] = clamp_cd((int32_t)coxa_cdeg);
  out_cd[1] = clamp_cd((int32_t)femur_cdeg);
  out_cd[2] = clamp_cd((int32_t)tibia_cdeg);
  return true;
}

// Externs provided by other compilation units
extern int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen);
// printERR is defined later in this file; only declare its prototype here to
// avoid multiple-definition/linker issues when functions.ino and
// commandprocessor.ino are linked together.
void printERR(uint8_t code, const char* msg);
extern CommandType parseCommandType(const char* cmd);
extern void processCmdHELP(const char* line,int s,int len);
extern void processCmdSTATUS(const char* line,int s,int len);
extern void processCmdREBOOT(const char* line,int s,int len);
extern void processCmdENABLE(const char* line,int s,int len);
extern void processCmdDISABLE(const char* line,int s,int len);
extern void processCmdFK(const char* line,int s,int len);
extern void processCmdLEGS(const char* line,int s,int len);
extern void processCmdSERVOS(const char* line,int s,int len);
extern void processCmdLEG(const char* line,int s,int len);
extern void processCmdSERVO(const char* line,int s,int len);
extern void processCmdRAW(const char* line,int s,int len);
extern void processCmdRAW3(const char* line,int s,int len);
extern void processCmdFOOT(const char* line,int s,int len);
extern void processCmdFEET(const char* line,int s,int len);
extern void processCmdMODE(const char* line,int s,int len);
extern void processCmdI(const char* line,int s,int len);
extern void processCmdT(const char* line,int s,int len);
extern void processCmdTEST(const char* line,int s,int len);
extern void processCmdSTAND(const char* line,int s,int len);
extern void processCmdTUCK(const char* line,int s,int len);
extern void processCmdSAFETY(const char* line,int s,int len);
extern void processCmdHOME(const char* line,int s,int len);
extern void processCmdSAVEHOME(const char* line,int s,int len);
extern void processCmdOFFSET(const char* line,int s,int len);
extern void processCmdLOG(const char* line,int s,int len);
extern void processCmdPID(const char* line,int s,int len);
extern void processCmdIMP(const char* line,int s,int len);
extern void processCmdCOMP(const char* line,int s,int len);
extern void processCmdEST(const char* line,int s,int len);
extern void processCmdLOOP(const char* line,int s,int len);
extern void processCmdLIMITS(const char* line,int s,int len);
extern void processCmdCONFIG(const char* line,int s,int len);

extern char    lineBuf[160];
extern uint8_t lineLen;
// Centralized OK/ERR printing lives below with unified dispatch.

void printOK() { Serial.print(F("OK")); }

void printERR(uint8_t code, const char* msg) {
  g_last_err = code;
  Serial.print(F("ERR "));
  Serial.print((unsigned int)code);
  Serial.print(F(" "));
  if (msg) Serial.print(msg);
}

// handleLine — tokenizes first word and dispatches via CommandType enum.
// Centralizes OK emission (printed once per successful command) and defers reboot.
void handleLine(const char* line) {
  if (!line) return;
  int len = (int)strlen(line);
  int s = 0, ts = 0, tl = 0;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) return; // empty line

  char cmd[16];
  int n = (tl < (int)sizeof(cmd)-1) ? tl : (int)sizeof(cmd)-1;
  memcpy(cmd, line + ts, n); cmd[n] = 0;
  for (int i = 0; cmd[i]; ++i) cmd[i] = (char)toupper(cmd[i]);

  g_last_err = 0; // reset before dispatch
  bool reboot_pending = false;
  CommandType ct = parseCommandType(cmd);
  switch (ct) {
    case CMD_HELP:      processCmdHELP(line, s, len); break;
    case CMD_STATUS:    processCmdSTATUS(line, s, len); break;
    case CMD_REBOOT:    reboot_pending = true; break; // defer reboot
    case CMD_ENABLE:    processCmdENABLE(line, s, len); break;
    case CMD_DISABLE:   processCmdDISABLE(line, s, len); break;
    case CMD_FK:        processCmdFK(line, s, len); break;
    case CMD_LEGS:      processCmdLEGS(line, s, len); break;
    case CMD_SERVOS:    processCmdSERVOS(line, s, len); break;
    case CMD_LEG:       processCmdLEG(line, s, len); break;
    case CMD_SERVO:     processCmdSERVO(line, s, len); break;
    case CMD_RAW:       processCmdRAW(line, s, len); break;
    case CMD_RAW3:      processCmdRAW3(line, s, len); break;
    case CMD_FOOT:      processCmdFOOT(line, s, len); break;
    case CMD_FEET:      processCmdFEET(line, s, len); break;
    case CMD_MODE:      processCmdMODE(line, s, len); break;
    case CMD_I:         processCmdI(line, s, len); break;
    case CMD_T:         processCmdT(line, s, len); break;
    case CMD_TEST:      processCmdTEST(line, s, len); break;
    case CMD_STAND:     processCmdSTAND(line, s, len); break;
  case CMD_TUCK:      processCmdTUCK(line, s, len); break;
    case CMD_SAFETY:    processCmdSAFETY(line, s, len); break;
    case CMD_HOME:      processCmdHOME(line, s, len); break;
    case CMD_SAVEHOME:  processCmdSAVEHOME(line, s, len); break;
    case CMD_OFFSET:    processCmdOFFSET(line, s, len); break;
  case CMD_LOG:       processCmdLOG(line, s, len); break;
  case CMD_LOOP:      processCmdLOOP(line, s, len); break;
  case CMD_PID:       processCmdPID(line, s, len); break;
  case CMD_IMP:       processCmdIMP(line, s, len); break;
  case CMD_COMP:      processCmdCOMP(line, s, len); break;
  case CMD_EST:       processCmdEST(line, s, len); break;
  case CMD_LIMITS:    processCmdLIMITS(line, s, len); break;
  case CMD_CONFIG:    processCmdCONFIG(line, s, len); break;
    default:            printERR(1, "UNKNOWN_CMD"); break;
  }

  if (g_last_err == 0) { printOK(); }

  // Echo original line (after response for consistency with prior behavior)
  Serial.print(F(" \u003e "));
  Serial.print(line);
  Serial.print(F("\r\n"));

  if (reboot_pending) {
    Serial.flush();
    delay(10);
    rebootNow();
  }
}

// -----------------------------------------------------------------------------
// buffersInit — initialize serial parsing state + tri-state buffers.
// -----------------------------------------------------------------------------
void buffersInit() {
  lineLen = 0; lineBuf[0] = '\0';
  for (uint8_t i = 0; i < 6; ++i) {
    int pin = Robot::BUFFER_ENABLE_PINS[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // LOW -> RX (driver high-Z)
  }
}

// -----------------------------------------------------------------------------
// Minimal splash — brief non-blocking banner and config summary
// -----------------------------------------------------------------------------
void splash() {
  if (!Serial) return;
  Serial.print(Robot::SPLASH_BANNER);
  Serial.print(F("MARS — Modular Autonomous Robotic System\r\n"));

  // Firmware version/build line
  Serial.print(F("FW "));
#ifdef FW_VERSION
  Serial.print(FW_VERSION);
#else
  Serial.print(F("unknown"));
#endif
#ifdef FW_BUILD
  Serial.print(F(" b=")); Serial.print((unsigned long)FW_BUILD);
#endif
  Serial.print(F("\r\n"));

  // Build metadata line
  Serial.print(F("BUILD "));
  Serial.print(__DATE__); Serial.print(F(" ")); Serial.print(__TIME__);
  Serial.print(F(" loop_hz=")); Serial.print((int)g_loop_hz);
  Serial.print(F(" cfg_loop_hz=")); Serial.print((int)g_config_loop_hz);
  Serial.print(F("\r\n"));

  // UART mapping / config summary line
  Serial.print(F("UARTs: ")); Serial.print(Robot::UART_MAP_SUMMARY);
  // If uart.* keys were present in /config.txt, show a brief sanity note (mapping is compile-time fixed)
  extern volatile bool g_uart_cfg_present; extern volatile bool g_uart_cfg_match;
  if (g_uart_cfg_present) {
    Serial.print(F(" (cfg: "));
    Serial.print(g_uart_cfg_match ? F("match") : F("mismatch"));
    Serial.print(F(")"));
  }
  Serial.print(F(" cfg=")); Serial.print(g_config_loaded ? 1 : 0);
  Serial.print(F(" keys=")); Serial.print((int)g_config_keys_applied);
  Serial.print(F("\r\n"));
}

// -----------------------------------------------------------------------------
// Serial line reader — fills lineBuf and dispatches on newline
// -----------------------------------------------------------------------------
void processSerial() {
  while (Serial && Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else {
      // Reserve one byte for null-terminator when dispatching
      if (lineLen < (uint8_t)(sizeof(lineBuf) - 1)) {
        lineBuf[lineLen++] = (char)ch;
      } else {
        // overflow: reset buffer to avoid partial commands lingering
        lineLen = 0;
      }
    }
  }
}

// Parse a key=value pair from config
static void configParseKV(char* key, char* val)
{
  if (!key || !val) return;

  // Capture but do not apply: uart.<LEG>=SerialX (compile-time mapping is fixed). Used for splash sanity.
  if (!strncmp(key, "uart.", 5))
  {
    char* p = key + 5;
    char legtok[4] = {0}; int li = 0;
    while (*p && *p != '.' && li < 3) legtok[li++] = *p++;
    legtok[li] = 0;
    int leg = legIndexFromToken(legtok);
    if (leg >= 0 && leg < 6) {
      extern volatile uint8_t g_uart_cfg_seen_mask; // declared in main TU
      extern volatile bool g_uart_cfg_present;
      extern volatile bool g_uart_cfg_match;
      g_uart_cfg_present = true;
      g_uart_cfg_seen_mask |= (uint8_t)(1u << leg);
      // Expected compile-time mapping names
      const char* expected[6] = {"Serial8","Serial3","Serial5","Serial7","Serial6","Serial2"};
      if (val && *val) {
        if (strcmp(val, expected[leg]) != 0) {
          g_uart_cfg_match = false;
        }
      } else {
        // Empty value counts as mismatch
        g_uart_cfg_match = false;
      }
    }
    // Do not count toward applied keys; mapping is not changed at runtime in Phase 1
    return;
  }

  if (!strcmp(key, "loop_hz"))

  {
    long hz = atol(val);
    if (hz > 0) configApplyLoopHz((uint16_t)hz);

    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "oos.fail_threshold"))

  {
    long n = atol(val);
    if (n < 1) n = 1; if (n > 20) n = 20;

    g_servo_fb_fail_threshold = (uint8_t)n;
    ++g_config_keys_applied;
    return;
  }

  // --- Safety keys ---
  if (!strcmp(key, "safety.soft_limits"))

  {
    bool on = parse_bool(val);
    g_safety_soft_limits_enabled = on;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "safety.collision"))

  {
    bool on = parse_bool(val);
    g_safety_collision_enabled = on;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "safety.temp_lockout_c"))

  {
    long c = atol(val);
    if (c < 30) c = 30; if (c > 120) c = 120;
    g_safety_temp_lockout_c10 = (int16_t)(c * 10);
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "safety.clearance_mm"))

  {
    long mm = atol(val);
    if (mm < 0) mm = 0; if (mm > 500) mm = 500;
    g_safety_clearance_mm = (float)mm;
    ++g_config_keys_applied;
    return;
  }

  if (!strncmp(key, "servo_id.", 9))

  {
    // servo_id.<LEG>.<JOINT>
    char* p = key + 9;
    char legtok[4] = {0}; int li = 0;
    while (*p && *p != '.' && li < 3) legtok[li++] = *p++;

    legtok[li] = 0;
    if (*p != '.') return; ++p;

    char jtok[8] = {0}; int ji = 0;
    while (*p && *p != '.' && ji < 7) jtok[ji++] = *p++;

    jtok[ji] = 0;

    int leg = legIndexFromToken(legtok);
    int j = jointIndexFromToken(jtok);
    if (leg < 0 || j < 0) return;

    long id = atol(val);
    if (id >= 1 && id <= 253) setServoId((uint8_t)leg, (uint8_t)j, (uint8_t)id);

    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "home_cdeg"))

  {
    // 18 comma-separated centideg values in LF..RR x (COXA,FEMUR,TIBIA)
    int idx = 0;
    long v = 0;
    bool inNum = false, neg = false;
    const char* p = val;
    while (*p && idx < 18)

    {
      if (*p == '-' && !inNum)

      {
        neg = true;
      }
      else if (*p >= '0' && *p <= '9')

      {
        v = inNum ? (v * 10 + (*p - '0')) : (*p - '0');
        inNum = true;
      }
      else if (*p == ',' || *p == ' ')

      {
        if (inNum)

        {
          if (neg) v = -v;

          if (v < 0) v = 0; if (v > 24000) v = 24000;

          int L = idx / 3, J = idx % 3;
          g_home_cd[L][J] = (int16_t)v;
          ++idx; v = 0; inNum = false; neg = false;
        }
      }

      ++p;
    }

    if (inNum && idx < 18)

    {
      if (neg) v = -v;

      if (v < 0) v = 0; if (v > 24000) v = 24000;

      int L = idx / 3, J = idx % 3; g_home_cd[L][J] = (int16_t)v; ++idx;
    }

    ++g_config_keys_applied;
    return;
  }

  if (!strncmp(key, "home_cd.", 8))

  {
    // home_cd.<LEG>.<coxa|femur|tibia>
    char* p = key + 8;
    char legtok[4] = {0}; int li = 0;
    while (*p && *p != '.' && li < 3) legtok[li++] = *p++;

    legtok[li] = 0;
    if (*p != '.') return; ++p;

    char jtok[8] = {0}; int ji = 0;
    while (*p && *p != '.' && ji < 7) jtok[ji++] = *p++;

    jtok[ji] = 0;

    int leg = legIndexFromToken(legtok);
    int j = jointIndexFromToken(jtok);
    if (leg < 0 || j < 0) return;

    long cd = atol(val);
    if (cd < 0) cd = 0; if (cd > 24000) cd = 24000;

    g_home_cd[leg][j] = (int16_t)cd;
    ++g_config_keys_applied;
    return;
  }

  if (!strncmp(key, "offset_cd.", 10))

  {
    // offset_cd.<LEG>.<coxa|femur|tibia> (centideg, typically within ±3000)
    char* p = key + 10;
    char legtok[4] = {0}; int li = 0;
    while (*p && *p != '.' && li < 3) legtok[li++] = *p++;

    legtok[li] = 0;
    if (*p != '.') return; ++p;

    char jtok[8] = {0}; int ji = 0;
    while (*p && *p != '.' && ji < 7) jtok[ji++] = *p++;

    jtok[ji] = 0;

    int leg = legIndexFromToken(legtok);
    int j = jointIndexFromToken(jtok);
    if (leg < 0 || j < 0) return;

    long cd = atol(val);
    if (cd < -3000) cd = -3000; if (cd > 3000) cd = 3000;

    g_offset_cd[leg][j] = (int16_t)cd; // seed from config; hardware refresh will overwrite
    ++g_config_keys_applied;
    return;
  }

  if (!strncmp(key, "joint_limits.", 14))

  {
    // joint_limits.<LEG>.<coxa|femur|tibia>.<min_deg|max_deg>
    char* p = key + 14;
    char legtok[4] = {0}; int li = 0;
    while (*p && *p != '.' && li < 3) legtok[li++] = *p++;

    legtok[li] = 0;
    if (*p != '.') return; ++p;

    char jtok[8] = {0}; int ji = 0;
    while (*p && *p != '.' && ji < 7) jtok[ji++] = *p++;

    jtok[ji] = 0;
    if (*p != '.') return; ++p;

    char prop[16] = {0}; int pi = 0;
    while (*p && *p != '.' && pi < 15) prop[pi++] = *p++;

    prop[pi] = 0;

    int leg = legIndexFromToken(legtok);
    int j = jointIndexFromToken(jtok);
    if (leg < 0 || j < 0) return;

    long deg = atol(val);
    long base = g_home_cd[leg][j];
    long cd = base + deg * 100L; // centideg around home
    if (cd < 0) cd = 0; if (cd > 24000) cd = 24000;

    if (!strcmp(prop, "min_deg"))

    {
      g_limit_min_cd[leg][j] = (int16_t)cd;
      if (g_limit_min_cd[leg][j] > g_limit_max_cd[leg][j])

      {
        g_limit_max_cd[leg][j] = g_limit_min_cd[leg][j];
      }
    }
    else if (!strcmp(prop, "max_deg"))

    {
      g_limit_max_cd[leg][j] = (int16_t)cd;
      if (g_limit_max_cd[leg][j] < g_limit_min_cd[leg][j])

      {
        g_limit_min_cd[leg][j] = g_limit_max_cd[leg][j];
      }
    }

    ++g_config_keys_applied;
    return;
  }

  // --- PID keys (Phase 2) ---
  // pid.enabled=true|false
  if (!strcmp(key, "pid.enabled"))
  {
    bool on = parse_bool(val);
    g_pid_enabled = on;
    ++g_config_keys_applied;
    return;
  }
  // pid.kp_milli.<coxa|femur|tibia>
  if (!strncmp(key, "pid.kp_milli.", 13))
  {
    const char* jtok = key + 13;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_pid_kp_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  // pid.ki_milli.<coxa|femur|tibia>
  if (!strncmp(key, "pid.ki_milli.", 13))
  {
    const char* jtok = key + 13;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_pid_ki_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  // pid.kd_milli.<coxa|femur|tibia>
  if (!strncmp(key, "pid.kd_milli.", 13))
  {
    const char* jtok = key + 13;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_pid_kd_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }

  // pid.kd_alpha_milli.<coxa|femur|tibia> (0..1000)
  if (!strncmp(key, "pid.kd_alpha_milli.", 19))
  {
    const char* jtok = key + 19;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 1000) v = 1000;
      g_pid_kd_alpha_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }

  // pid.mode=active|shadow
  if (!strcmp(key, "pid.mode"))
  {
    // normalize first char
    char c0 = tolower(val[0]);
    if (c0 == 's') g_pid_mode = 1; else g_pid_mode = 0; // anything not starting with 's' treated as active
    ++g_config_keys_applied; return;
  }
  // pid.shadow_report_hz=<1..50>
  if (!strcmp(key, "pid.shadow_report_hz"))
  {
    long hz = atol(val);
    if (hz < 1) hz = 1; if (hz > 50) hz = 50; // cap to avoid serial spam
    g_pid_shadow_report_hz = (uint16_t)hz;
    ++g_config_keys_applied; return;
  }

  if (!strcmp(key, "rate_limit.deg_per_s"))

  {
    float v = atof(val);
    if (v <= 0.0f)

    {
      g_rate_limit_cdeg_per_s = 0u;
    }
    else

    {
      long cdeg = lroundf(v * 100.0f);
      if (cdeg < 0) cdeg = 0; if (cdeg > 65535) cdeg = 65535;

      g_rate_limit_cdeg_per_s = (uint16_t)cdeg;
    }

    ++g_config_keys_applied;
    return;
  }

  // Estimator configuration
  if (!strcmp(key, "est.cmd_alpha_milli"))
  {
    long v = atol(val); if (v < 0) v = 0; if (v > 1000) v = 1000;
    g_est_cmd_alpha_milli = (uint16_t)v;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "est.meas_alpha_milli"))
  {
    long v = atol(val); if (v < 0) v = 0; if (v > 1000) v = 1000;
    g_est_meas_alpha_milli = (uint16_t)v;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "est.meas_vel_alpha_milli"))
  {
    long v = atol(val); if (v < 0) v = 0; if (v > 2000) v = 2000; // allow slightly higher gain for velocity
    g_est_meas_vel_alpha_milli = (uint16_t)v;
    ++g_config_keys_applied; return;
  }

  // --- Impedance keys ---
  if (!strcmp(key, "imp.enabled"))
  {
    bool on = parse_bool(val);
    g_impedance.config().enabled = on;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "imp.mode"))
  {
    // off|joint|cart (case-insensitive, check first char)
    char c0 = tolower(val[0]);
    if (c0 == 'j') g_impedance.config().mode = IMP_MODE_JOINT;
    else if (c0 == 'c') g_impedance.config().mode = IMP_MODE_CART;
    else g_impedance.config().mode = IMP_MODE_OFF;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "imp.joint_deadband_cd"))
  {
    long v = atol(val);
    if (v < 0) v = 0; if (v > 5000) v = 5000; // up to 50 deg
    g_impedance.config().joint_deadband_cd = (uint16_t)v;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "imp.cart_deadband_mm"))
  {
    float v = atof(val);
    if (v < 0.0f) v = 0.0f; if (v > 100.0f) v = 100.0f;
    g_impedance.config().cart_deadband_mm = v;
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "imp.output_scale_milli"))
  {
    long v = atol(val);
    if (v < 0) v = 0; if (v > 1000) v = 1000; // 0..1000 scaling
    g_impedance.config().output_scale_milli = (uint16_t)v;
    ++g_config_keys_applied; return;
  }
  // Joint compliance keys: comp.enabled, comp.kp_milli.<coxa|femur|tibia>,
  // comp.range_cd.<coxa|femur|tibia>, comp.deadband_cd.<coxa|femur|tibia>, comp.leak_milli
  if (!strcmp(key, "comp.enabled"))
  {
    g_comp_enabled = parse_bool(val);
    ++g_config_keys_applied; return;
  }
  if (!strcmp(key, "comp.leak_milli"))
  {
    long v = atol(val);
    if (v < 0) v = 0; if (v > 1000) v = 1000; // 0..1.0 per tick
    g_comp_leak_milli = (uint16_t)v;
    ++g_config_keys_applied; return;
  }
  if (!strncmp(key, "comp.kp_milli.", 14))
  {
    const char* jtok = key + 14;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_comp_kp_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  if (!strncmp(key, "comp.range_cd.", 14))
  {
    const char* jtok = key + 14;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 5000) v = 5000; // up to 50 deg of compliance
      g_comp_range_cd[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  if (!strncmp(key, "comp.deadband_cd.", 18))
  {
    const char* jtok = key + 18;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 2000) v = 2000; // up to 20 deg deadband
      g_comp_deadband_cd[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  // Joint-space impedance gains: imp.joint.k_spring_milli.<coxa|femur|tibia>
  if (!strncmp(key, "imp.joint.k_spring_milli.", 25))
  {
    const char* jtok = key + 25;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_impedance.config().joint.k_spring_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  // Joint-space damping: imp.joint.k_damp_milli.<coxa|femur|tibia>
  if (!strncmp(key, "imp.joint.k_damp_milli.", 23))
  {
    const char* jtok = key + 23;
    int j = jointIndexFromToken(jtok);
    if (j >= 0 && j < 3) {
      long v = atol(val);
      if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_impedance.config().joint.k_damp_milli[j] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  // Cartesian impedance gains (body frame): imp.cart.k_spring_milli.<x|y|z> / imp.cart.k_damp_milli.<x|y|z>
  if (!strncmp(key, "imp.cart.k_spring_milli.", 25))
  {
    const char* atok = key + 25;
    int axis = -1;
    if      (!strcmp(atok, "x")) axis = 0;
    else if (!strcmp(atok, "y")) axis = 1;
    else if (!strcmp(atok, "z")) axis = 2;
    if (axis >= 0 && axis < 3) {
      long v = atol(val); if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_impedance.config().cart.k_spring_milli[axis] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }
  if (!strncmp(key, "imp.cart.k_damp_milli.", 23))
  {
    const char* atok = key + 23;
    int axis = -1;
    if      (!strcmp(atok, "x")) axis = 0;
    else if (!strcmp(atok, "y")) axis = 1;
    else if (!strcmp(atok, "z")) axis = 2;
    if (axis >= 0 && axis < 3) {
      long v = atol(val); if (v < 0) v = 0; if (v > 65535) v = 65535;
      g_impedance.config().cart.k_damp_milli[axis] = (uint16_t)v;
      ++g_config_keys_applied;
    }
    return;
  }

  if (!strcmp(key, "test.trigait.overlap_pct"))

  {
    float v = atof(val);
    if (v < 0.0f) v = 0.0f; if (v > 25.0f) v = 25.0f; // clamp to sane range
    g_test_overlap_pct = v;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "test.trigait.cycle_ms"))

  {
    long ms = atol(val);
    if (ms < 750) ms = 750; if (ms > 10000) ms = 10000;
    g_test_cycle_ms = (uint32_t)ms;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "test.trigait.height_mm"))

  {
    float v = atof(val);
    if (v > 0.0f) v = 0.0f; if (v < -300.0f) v = -300.0f; // Y: negative is down
    g_test_base_y_mm = v;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "test.trigait.basex_mm"))

  {
    float v = atof(val);
    if (v < -300.0f) v = -300.0f; if (v > 300.0f) v = 300.0f;
    g_test_base_x_mm = v;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "test.trigait.steplen_mm"))

  {
    float v = atof(val);
    if (v < 0.0f) v = 0.0f; if (v > 200.0f) v = 200.0f;
    g_test_step_len_mm = v;
    ++g_config_keys_applied;
    return;
  }

  if (!strcmp(key, "test.trigait.lift_mm"))

  {
    float v = atof(val);
    if (v < 0.0f) v = 0.0f; if (v > 200.0f) v = 200.0f;
    g_test_lift_y_mm = v;
    ++g_config_keys_applied;
    return;
  }

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // --- Logging keys (Phase 1) ---
  if (!strcmp(key, "logging.enabled"))
  {
    long v = atol(val);
    g_log_enabled = (v != 0);
    ++g_config_keys_applied;
    return;
  }
  if (!strcmp(key, "logging.rate_hz"))
  {
    long hz = atol(val);
    if (hz < 1) hz = 1; if (hz > 500) hz = 500; // align with LOG RATE bounds
    g_log_rate_hz = (uint16_t)hz;
    ++g_config_keys_applied;
    return;
  }
  if (!strcmp(key, "logging.mode"))
  {
    long m = atol(val);
    if (m < 0) m = 0; if (m > 1) m = 1; // 0=compact,1=full (full deferred)
    g_log_mode = (uint8_t)m;
    ++g_config_keys_applied;
    return;
  }
  if (!strcmp(key, "logging.header"))
  {
    long h = atol(val);
    g_log_header = (h != 0);
    ++g_config_keys_applied;
    return;
  }
  if (!strcmp(key, "logging.rotate"))
  {
    long v = atol(val);
    g_log_rotate = (v != 0);
    ++g_config_keys_applied;
    return;
  }
  if (!strcmp(key, "logging.max_kb"))
  {
    long kb = atol(val);
    if (kb < 100) kb = 100; // minimum 100KB
    long maxKBClamp = 1024L * 1024L; // 1GB in KB
    if (kb > maxKBClamp) kb = maxKBClamp;
    g_log_max_bytes = (uint32_t)kb * 1024UL;
    ++g_config_keys_applied;
    return;
  }
#endif

  // --- TUCK parameters ---
  if (!strcmp(key, "tuck.tibia_cd")) { long v = atol(val); if (v<0) v=0; if (v>24000) v=24000; g_tuck_tibia_cd = (int16_t)v; ++g_config_keys_applied; return; }
  if (!strcmp(key, "tuck.femur_cd")) { long v = atol(val); if (v<0) v=0; if (v>24000) v=24000; g_tuck_femur_cd = (int16_t)v; ++g_config_keys_applied; return; }
  if (!strcmp(key, "tuck.coxa_cd"))  { long v = atol(val); if (v!=12000) v=12000; g_tuck_coxa_cd = (int16_t)v; ++g_config_keys_applied; return; }
  if (!strcmp(key, "tuck.tol_tibia_cd")) { long v = atol(val); if (v<10) v=10; if (v>5000) v=5000; g_tuck_tol_tibia_cd = (int16_t)v; ++g_config_keys_applied; return; }
  if (!strcmp(key, "tuck.tol_other_cd")) { long v = atol(val); if (v<10) v=10; if (v>5000) v=5000; g_tuck_tol_other_cd = (int16_t)v; ++g_config_keys_applied; return; }
  if (!strcmp(key, "tuck.timeout_ms")) { long v = atol(val); if (v<250) v=250; if (v>10000) v=10000; g_tuck_timeout_ms = (uint16_t)v; ++g_config_keys_applied; return; }
}

// -----------------------------------------------------------------------------
// Config loader — minimal placeholder that marks config as not loaded.
// Extend later to parse /config.txt keys as needed.
// -----------------------------------------------------------------------------
void configLoad()
{
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  if (!SD.begin(BUILTIN_SDCARD))

  {
    g_config_loaded = false;
    g_config_keys_applied = 0;
    g_config_loop_hz = 166;
    return;
  }

  File f = SD.open("/config.txt", FILE_READ);
  if (!f)

  {
    g_config_loaded = false;
    g_config_keys_applied = 0;
    g_config_loop_hz = g_loop_hz;
    return;
  }

  g_config_loaded = true;
  g_config_keys_applied = 0;
  static char buf[128];
  int len = 0;
  while (f.available())

  {
    int ch = f.read();
    if (ch < 0) break;

    if (ch == '\r') continue;

    if (ch == '\n')

    {
      buf[len] = 0;
      char* line = ltrim(buf);
      rtrim(line);
      if (*line && *line != '#')

      {
        char* eq = strchr(line, '=');
        if (eq)

        {
          *eq = 0;
          char* key = ltrim(line); rtrim(key);
          char* val = ltrim(eq + 1); rtrim(val);
          configParseKV(key, val);
        }
      }

      len = 0;
    }
    else if (len + 1 < (int)sizeof(buf))

    {
      buf[len++] = (char)ch;
    }
    else

    {
      // overflow: reset line
      len = 0;
    }
  }

  f.close();
#else
  g_config_loaded = false;
  g_config_keys_applied = 0;
  g_config_loop_hz = g_loop_hz;
#endif
}

// -----------------------------------------------------------------------------
// Refresh hardware angle offsets into g_offset_cd at startup
// Reads each in-service servo's stored angle offset (cd) and populates g_offset_cd.
// Safe on host: fake lx16a-servo stubs return 0.
// -----------------------------------------------------------------------------
void refreshOffsetsAtStartup() {
  for (uint8_t L = 0; L < 6; ++L) {
    for (uint8_t J = 0; J < 3; ++J) {
      int16_t cd = 0;
      uint8_t id = servoId(L, J);
      if (id >= 1 && id <= 253) {
        cd = (int16_t)angle_offset_read(L, id);
      }
      g_offset_cd[L][J] = cd;
    }
  }
}

// -----------------------------------------------------------------------------
// Missing helpers required by commandprocessor.ino
//  - Tokenizer
//  - STATUS/HELP printers
//  - Reboot
//  - Angle offset helpers are provided by lx16a-servo library
// -----------------------------------------------------------------------------

// Tokenizer used by dispatcher and command handlers
int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen) {
  if (!s || start < 0 || len < 0) {
    if (tokStart) *tokStart = 0;
    if (tokLen) *tokLen = 0;
    return 0;
  }
  int i = start;
  while (i < len && (s[i] == ' ' || s[i] == '\t')) ++i; // skip spaces
  if (i >= len) {
    if (tokStart) *tokStart = 0;
    if (tokLen) *tokLen = 0;
    return len;
  }
  int j = i;
  while (j < len && s[j] != ' ' && s[j] != '\t') ++j;
  if (tokStart) *tokStart = i;
  if (tokLen) *tokLen = (j - i);
  return j;
}

void printSTATUS()
{
  Serial.print(F("STATUS\r\n"));

  // [SYSTEM]
  Serial.print(F("[SYSTEM]\r\n"));
  {
    uint64_t ms_total = g_uptime_ms64;
    uint64_t days  = ms_total / 86400000ULL; ms_total %= 86400000ULL;
    uint64_t hours = ms_total / 3600000ULL;  ms_total %= 3600000ULL;
    uint64_t mins  = ms_total / 60000ULL;    ms_total %= 60000ULL;
    uint64_t secs  = ms_total / 1000ULL;     ms_total %= 1000ULL;
    uint64_t ms    = ms_total;
    Serial.print(F("  uptime="));
    Serial.print((unsigned long)days);  Serial.print(F("d"));
    Serial.print((unsigned long)hours); Serial.print(F("h"));
    Serial.print((unsigned long)mins);  Serial.print(F("m"));
    Serial.print((unsigned long)secs);  Serial.print(F("s"));
    Serial.print((unsigned long)ms);    Serial.print(F("ms\r\n"));
    Serial.print(F("  enabled=")); Serial.print(g_enabled ? 1 : 0); Serial.print(F("\r\n"));
  // Firmware version and build (monotonic)
  Serial.print(F("  fw="));
#ifdef FW_VERSION
  Serial.print(FW_VERSION);
#else
  Serial.print(F("unknown"));
#endif
#ifdef FW_BUILD
  Serial.print(F(" b=")); Serial.print((unsigned long)FW_BUILD);
#endif
  Serial.print(F("\r\n"));
    Serial.print(F("  loop_hz=")); Serial.print(g_loop_hz); Serial.print(F("\r\n"));
    Serial.print(F("  rr_idx=")); Serial.print(g_rr_index); Serial.print(F("\r\n"));
    Serial.print(F("  overruns=")); Serial.print((unsigned long)g_overrun_count); Serial.print(F("\r\n"));
    Serial.print(F("  last_err=")); Serial.print((int)g_last_err); Serial.print(F("\r\n"));
    const char* safety_state = g_lockout ? ((g_lockout_causes & ~g_override_mask) == 0 ? "OVERRIDDEN" : "LOCKOUT") : "OK";
    Serial.print(F("  safety=")); Serial.print(safety_state);
    Serial.print(F(" cause=0x")); Serial.print((unsigned int)g_lockout_causes, HEX);
    Serial.print(F(" override=0x")); Serial.print((unsigned int)g_override_mask, HEX);
    Serial.print(F("\r\n"));
  }

  // [ENABLES]
  Serial.print(F("[ENABLES]\r\n"));
  {
    Serial.print(F("  legs="));
    for (int i = 0; i < 6; ++i) { Serial.print(leg_enabled_mask_get((uint8_t)i) ? '1' : '0'); }
    Serial.print(F("\r\n"));
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("  jmask="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(","));
      Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) Serial.print(joint_enabled_mask_get((uint8_t)i,(uint8_t)j)?1:0);
    }
    Serial.print(F("\r\n"));
  }

  // [TELEMETRY]
  Serial.print(F("[TELEMETRY]\r\n"));
  {
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("  vin_V="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(","));
      Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) {
        if (j) Serial.print(F("/"));
        uint16_t mV = g_meas_vin_mV[i][j];
        Serial.print(mV / 1000); Serial.print(F(".")); Serial.print((mV % 1000) / 100);
      }
    }
    Serial.print(F("\r\n"));
    Serial.print(F("  temp_C="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(","));
      Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print((int)g_meas_temp_C[i][j]); }
    }
    Serial.print(F("\r\n"));
    Serial.print(F("  pos_cd="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(","));
      Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print((int)g_meas_pos_cd[i][j]); }
    }
    Serial.print(F("\r\n"));
  }

  // [CONTROL MODES]
  Serial.print(F("[CONTROL]\r\n"));
  {
    // PID summary
    Serial.print(F("  pid.enabled=")); Serial.print(g_pid_enabled ? 1 : 0);
    Serial.print(F(" mode=")); Serial.print((g_pid_mode==0)?F("active"):F("shadow"));
    Serial.print(F(" kp_milli="));
    Serial.print((unsigned)g_pid_kp_milli[0]); Serial.print('/');
    Serial.print((unsigned)g_pid_kp_milli[1]); Serial.print('/');
    Serial.print((unsigned)g_pid_kp_milli[2]);
    Serial.print(F(" ki_milli="));
    Serial.print((unsigned)g_pid_ki_milli[0]); Serial.print('/');
    Serial.print((unsigned)g_pid_ki_milli[1]); Serial.print('/');
    Serial.print((unsigned)g_pid_ki_milli[2]);
    Serial.print(F(" kd_milli="));
    Serial.print((unsigned)g_pid_kd_milli[0]); Serial.print('/');
    Serial.print((unsigned)g_pid_kd_milli[1]); Serial.print('/');
    Serial.print((unsigned)g_pid_kd_milli[2]);
    Serial.print(F("\r\n"));

    // Impedance summary (JOINT/CART outer loop around effective joint commands)
    const ImpedanceConfig& icfg = g_impedance.config();
    Serial.print(F("  imp.enabled=")); Serial.print(icfg.enabled ? 1 : 0);
    Serial.print(F(" mode="));
    switch (icfg.mode) {
      case IMP_MODE_JOINT: Serial.print(F("joint")); break;
      case IMP_MODE_CART:  Serial.print(F("cart"));  break;
      default:             Serial.print(F("off"));   break;
    }
    Serial.print(F(" jspring="));
    Serial.print((unsigned)icfg.joint.k_spring_milli[0]); Serial.print('/');
    Serial.print((unsigned)icfg.joint.k_spring_milli[1]); Serial.print('/');
    Serial.print((unsigned)icfg.joint.k_spring_milli[2]);
    Serial.print(F(" jdamp="));
    Serial.print((unsigned)icfg.joint.k_damp_milli[0]);   Serial.print('/');
    Serial.print((unsigned)icfg.joint.k_damp_milli[1]);   Serial.print('/');
    Serial.print((unsigned)icfg.joint.k_damp_milli[2]);
    Serial.print(F(" cspring="));
    Serial.print((unsigned)icfg.cart.k_spring_milli[0]);  Serial.print('/');
    Serial.print((unsigned)icfg.cart.k_spring_milli[1]);  Serial.print('/');
    Serial.print((unsigned)icfg.cart.k_spring_milli[2]);
    Serial.print(F(" cdamp="));
    Serial.print((unsigned)icfg.cart.k_damp_milli[0]);    Serial.print('/');
    Serial.print((unsigned)icfg.cart.k_damp_milli[1]);    Serial.print('/');
    Serial.print((unsigned)icfg.cart.k_damp_milli[2]);
    Serial.print(F(" scale="));
    Serial.print((unsigned)icfg.output_scale_milli);
    Serial.print(F(" jdb_cd="));
    Serial.print((unsigned)icfg.joint_deadband_cd);
    Serial.print(F(" cdb_mm="));
    Serial.print(icfg.cart_deadband_mm, 2);
    Serial.print(F("\r\n"));

    // Joint compliance summary (setpoint adaptation around estimator)
    Serial.print(F("  comp.enabled=")); Serial.print(g_comp_enabled ? 1 : 0);
    Serial.print(F(" kp_milli="));
    Serial.print((unsigned)g_comp_kp_milli[0]); Serial.print('/');
    Serial.print((unsigned)g_comp_kp_milli[1]); Serial.print('/');
    Serial.print((unsigned)g_comp_kp_milli[2]);
    Serial.print(F(" range_cd="));
    Serial.print((unsigned)g_comp_range_cd[0]); Serial.print('/');
    Serial.print((unsigned)g_comp_range_cd[1]); Serial.print('/');
    Serial.print((unsigned)g_comp_range_cd[2]);
    Serial.print(F(" deadband_cd="));
    Serial.print((unsigned)g_comp_deadband_cd[0]); Serial.print('/');
    Serial.print((unsigned)g_comp_deadband_cd[1]); Serial.print('/');
    Serial.print((unsigned)g_comp_deadband_cd[2]);
    Serial.print(F(" leak_milli="));
    Serial.print((unsigned)g_comp_leak_milli);
    Serial.print(F("\r\n"));
  }

  // [HOME]
  Serial.print(F("[HOME]\r\n"));
  {
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("  home_cd="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(",")); Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print((int)g_home_cd[i][j]); }
    }
    Serial.print(F("\r\n"));
    Serial.print(F("  offset_cd="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(",")); Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print((int)g_offset_cd[i][j]); }
    }
    Serial.print(F("\r\n"));
  }

  // [TEST]
  Serial.print(F("[TEST]\r\n"));
  {
    Serial.print(F("  cycle_ms=")); Serial.print((unsigned int)g_test_cycle_ms); Serial.print(F("\r\n"));
    Serial.print(F("  base_x="));  Serial.print((int)lroundf(g_test_base_x_mm)); Serial.print(F("\r\n"));
    Serial.print(F("  base_y="));  Serial.print((int)lroundf(g_test_base_y_mm)); Serial.print(F("\r\n"));
    Serial.print(F("  step_z="));  Serial.print((int)lroundf(g_test_step_len_mm)); Serial.print(F("\r\n"));
    Serial.print(F("  lift_y="));  Serial.print((int)lroundf(g_test_lift_y_mm)); Serial.print(F("\r\n"));
    Serial.print(F("  overlap_pct=")); Serial.print((int)lroundf(g_test_overlap_pct)); Serial.print(F("\r\n"));
  }

  // [TIMING]
#if MARS_TIMING_PROBES
  Serial.print(F("[TIMING]\r\n"));
  {
    Serial.print(F("  serial_us=")); Serial.print((unsigned int)g_probe_serial_us); Serial.print(F("\r\n"));
    Serial.print(F("  send_us="));   Serial.print((unsigned int)g_probe_send_us);   Serial.print(F("\r\n"));
    Serial.print(F("  fb_us="));     Serial.print((unsigned int)g_probe_fb_us);     Serial.print(F("\r\n"));
    Serial.print(F("  log_us="));    Serial.print((unsigned int)g_probe_log_us);    Serial.print(F("\r\n"));
    Serial.print(F("  tick_us="));   Serial.print((unsigned int)g_probe_tick_us);   Serial.print(F("\r\n"));
    // Jitter summary (rolling window)
    unsigned int jmin = (unsigned int)g_jitter_min_us;
    unsigned int jmax = (unsigned int)g_jitter_max_us;
    unsigned int javg = (g_jitter_count > 0) ? (unsigned int)(g_jitter_sum_us / g_jitter_count) : 0u;
    Serial.print(F("  jitter_us=min/avg/max="));
    Serial.print(jmin); Serial.print(F("/")); Serial.print(javg); Serial.print(F("/")); Serial.print(jmax);
    Serial.print(F("\r\n"));
#if MARS_PROFILE_SEND
    // Send-path profiling summary (since boot)
    unsigned long pcnt = (unsigned long)g_prof_send_call_count;
    unsigned long tcnt = (unsigned long)g_prof_send_tick_count;
    unsigned int pmin = (g_prof_send_call_us_min==0xFFFFu)?0u:(unsigned int)g_prof_send_call_us_min;
    unsigned int pmax = (unsigned int)g_prof_send_call_us_max;
    unsigned int pavg = (pcnt>0)?(unsigned int)(g_prof_send_call_us_sum / pcnt):0u;
    unsigned int tmin = (g_prof_send_tick_us_min==0xFFFFu)?0u:(unsigned int)g_prof_send_tick_us_min;
    unsigned int tmax = (unsigned int)g_prof_send_tick_us_max;
    unsigned int tavg = (tcnt>0)?(unsigned int)(g_prof_send_tick_us_sum / tcnt):0u;
    Serial.print(F("  send_profile.per_call_us=min/avg/max="));
    Serial.print(pmin); Serial.print(F("/")); Serial.print(pavg); Serial.print(F("/")); Serial.print(pmax);
    Serial.print(F(" calls=")); Serial.print(pcnt); Serial.print(F("\r\n"));
    Serial.print(F("  send_profile.total_us=min/avg/max="));
    Serial.print(tmin); Serial.print(F("/")); Serial.print(tavg); Serial.print(F("/")); Serial.print(tmax);
    Serial.print(F(" ticks=")); Serial.print(tcnt); Serial.print(F("\r\n"));
#endif

  // [PID]
  Serial.print(F("[PID]\r\n"));
  {
    Serial.print(F("  enabled=")); Serial.print(g_pid_enabled ? 1 : 0); Serial.print(F("\r\n"));
    Serial.print(F("  mode=")); Serial.print(g_pid_mode==0?F("active"):F("shadow")); Serial.print(F("\r\n"));
    const char* jn[3] = {"coxa","femur","tibia"};
    Serial.print(F("  kp_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)g_pid_kp_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  ki_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)g_pid_ki_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  kd_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)g_pid_kd_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  kd_alpha_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)g_pid_kd_alpha_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  shadow_report_hz=")); Serial.print((unsigned int)g_pid_shadow_report_hz); Serial.print(F("\r\n"));
  }

  // [EST]
  Serial.print(F("[EST]\r\n"));
  {
    Serial.print(F("  cmd_alpha_milli=")); Serial.print((unsigned int)g_est_cmd_alpha_milli); Serial.print(F("\r\n"));
    Serial.print(F("  meas_alpha_milli=")); Serial.print((unsigned int)g_est_meas_alpha_milli); Serial.print(F("\r\n"));
    Serial.print(F("  meas_vel_alpha_milli=")); Serial.print((unsigned int)g_est_meas_vel_alpha_milli); Serial.print(F("\r\n"));
  }
  
  // [IMP]
  Serial.print(F("[IMP]\r\n"));
  {
    const ImpedanceConfig& cfg = g_impedance.config();
    Serial.print(F("  enabled=")); Serial.print(cfg.enabled ? 1 : 0); Serial.print(F("\r\n"));
    Serial.print(F("  mode="));
    switch (cfg.mode) {
      case IMP_MODE_JOINT: Serial.print(F("joint")); break;
      case IMP_MODE_CART:  Serial.print(F("cart"));  break;
      default:             Serial.print(F("off"));   break;
    }
    Serial.print(F("\r\n"));
    const char* jn[3] = {"coxa","femur","tibia"};
    Serial.print(F("  joint_k_spring_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)cfg.joint.k_spring_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  joint_k_damp_milli="));
    for (int j = 0; j < 3; ++j) { if (j) Serial.print(F("/")); Serial.print(jn[j]); Serial.print(F("=")); Serial.print((unsigned int)cfg.joint.k_damp_milli[j]); }
    Serial.print(F("\r\n"));
    Serial.print(F("  cart_k_spring_milli="));
    Serial.print(F("x=")); Serial.print((unsigned int)cfg.cart.k_spring_milli[0]); Serial.print(F("/"));
    Serial.print(F("y=")); Serial.print((unsigned int)cfg.cart.k_spring_milli[1]); Serial.print(F("/"));
    Serial.print(F("z=")); Serial.print((unsigned int)cfg.cart.k_spring_milli[2]); Serial.print(F("\r\n"));
    Serial.print(F("  cart_k_damp_milli="));
    Serial.print(F("x=")); Serial.print((unsigned int)cfg.cart.k_damp_milli[0]); Serial.print(F("/"));
    Serial.print(F("y=")); Serial.print((unsigned int)cfg.cart.k_damp_milli[1]); Serial.print(F("/"));
    Serial.print(F("z=")); Serial.print((unsigned int)cfg.cart.k_damp_milli[2]); Serial.print(F("\r\n"));
  }
  }
#endif

  // [OOS]
  Serial.print(F("[OOS]\r\n"));
  {
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("  mask="));
    for (int i = 0; i < 6; ++i) {
      if (i) Serial.print(F(","));
      Serial.print(names[i]); Serial.print(F(":"));
      for (int j = 0; j < 3; ++j) { int idx = i*3+j; Serial.print(((g_servo_oos_mask>>idx)&1u)?1:0); }
    }
    Serial.print(F("\r\n"));
  }
}

// Minimal HELP — authoritative list here to avoid duplication.
void printHELP() {
  // Header with firmware version
  Serial.print(F("HELP FW "));
#ifdef FW_VERSION
  Serial.print(FW_VERSION);
#else
  Serial.print(F("unknown"));
#endif
  Serial.print(F("\r\n"));

  // System / meta
  Serial.print(F("[SYSTEM]\r\n"));
  Serial.print(F("  HELP                      Show this help\r\n"));
  Serial.print(F("  STATUS                    Show system status summary\r\n"));
  Serial.print(F("  REBOOT                    Reboot controller\r\n"));
  Serial.print(F("  CONFIG                    Dump /config.txt contents (when SD present)\r\n"));
  Serial.print(F("  LOOP <hz>                 Set control loop frequency (50..500 Hz, persists loop_hz when SD enabled)\r\n"));

  // Enable / disable
  Serial.print(F("[ENABLE]\r\n"));
  Serial.print(F("  ENABLE                    Global enable (torque allowed)\r\n"));
  Serial.print(F("  DISABLE                   Global disable (torque off)\r\n"));
  Serial.print(F("  LEGS                      List per-leg enable mask\r\n"));
  Serial.print(F("  LEG <LEG|ALL> <ENABLE|DISABLE>    Enable/disable leg(s)\r\n"));
  Serial.print(F("  SERVOS                    List per-servo enable mask\r\n"));
  Serial.print(F("  SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>  Enable/disable joint(s)\r\n"));

  // Motion / command targets
  Serial.print(F("[MOTION]\r\n"));
  Serial.print(F("  STAND                     Move enabled legs to neutral IK stance\r\n"));
  Serial.print(F("  TUCK [LEG|ALL]            Start tuck sequence (tibia-first then femur+coxa) using tuck.* params\r\n"));
  Serial.print(F("  TUCK SET <PARAM> <VAL>    Update+persist param TIBIA|FEMUR|COXA|TOL_TIBIA|TOL_OTHER|TIMEOUT (angles/tols=cd, timeout=ms)\r\n"));
  Serial.print(F("  TUCK PARAMS               Show current tuck parameter values (tuck.* config)\r\n"));
  Serial.print(F("  FOOT <LEG> <x y z>        IK move single leg (mm, body frame)\r\n"));
  Serial.print(F("  FEET <x1 y1 z1 ... x6 y6 z6>  IK move all 6 legs (LF..RR)\r\n"));
  Serial.print(F("  RAW <LEG> <JOINT> <cd>    Direct joint target (centideg)\r\n"));
  Serial.print(F("  RAW3 <LEG> <c f t>        Direct 3-joint targets (centideg)\r\n"));

  // Geometry / calibration
  Serial.print(F("[GEOMETRY]\r\n"));
  Serial.print(F("  FK <LEG|ALL> <ON|OFF>     Toggle FK body-frame stream per leg\r\n"));
  Serial.print(F("  HOME                      Move enabled in-service servos to home_cd\r\n"));
  Serial.print(F("  SAVEHOME                  Capture current positions as new home_cd (enabled/in-service)\r\n"));
  Serial.print(F("  OFFSET LIST               List hardware angle offsets (centideg)\r\n"));
  Serial.print(F("  OFFSET CLEAR <LEG|ALL> <JOINT|ALL>  Clear hardware angle offsets\r\n"));
  Serial.print(F("  LIMITS LIST               Show shared joint workspace limits (per joint type, all legs)\r\n"));
  Serial.print(F("  LIMITS <COXA|FEMUR|TIBIA> <MIN_DEG> <MAX_DEG>  Set shared joint workspace (deg, applied around home_cd)\r\n"));

  // Mode / test / shortcuts
  Serial.print(F("[MODE]\r\n"));
  Serial.print(F("  MODE <TEST|IDLE>          Set operating mode\r\n"));
  Serial.print(F("  T                         Shortcut: MODE TEST\r\n"));
  Serial.print(F("  I                         Shortcut: MODE IDLE\r\n"));
  Serial.print(F("  TEST CYCLE <ms>           Set tripod phase duration (750..10000)\r\n"));
  Serial.print(F("  TEST HEIGHT <mm>          Set ground height (Y, negative down)\r\n"));
  Serial.print(F("  TEST BASEx <mm>           Set lateral X base offset\r\n"));
  Serial.print(F("  TEST STEPLEN <mm>         Set forward/back amplitude (|Z|)\r\n"));
  Serial.print(F("  TEST LIFT <mm>            Set swing lift height (Y)\r\n"));
  Serial.print(F("  TEST OVERLAP <pct>        Set overlap percent (0..25)\r\n"));

  // Safety
  Serial.print(F("[SAFETY]\r\n"));
  Serial.print(F("  SAFETY LIST               Show safety state/config\r\n"));
  Serial.print(F("  SAFETY SOFTLIMITS <ON|OFF>  Toggle soft joint limits\r\n"));
  Serial.print(F("  SAFETY COLLISION <ON|OFF>   Toggle foot keep-out check\r\n"));
  Serial.print(F("  SAFETY TEMPLOCK <C>       Set over-temp lockout threshold (Celsius)\r\n"));
  Serial.print(F("  SAFETY CLEARANCE <mm>     Set foot-to-foot clearance (X/Z plane)\r\n"));
  Serial.print(F("  SAFETY OVERRIDE <ALL|TEMP|COLLISION|NONE>  Override lockout causes\r\n"));
  Serial.print(F("  SAFETY CLEAR              Attempt to clear safety lockout (requires no active non-overridden causes)\r\n"));

  // Logging
  Serial.print(F("[LOGGING]\r\n"));
  Serial.print(F("  LOG ENABLE                 Enable CSV logging (compat: ENABLE ON/OFF)\r\n"));
  Serial.print(F("  LOG DISABLE                Disable CSV logging\r\n"));
  Serial.print(F("  LOG RATE <hz>             Set logging rate (Hz, <= loop_hz)\r\n"));
  Serial.print(F("  LOG MODE <COMPACT|FULL>   Set logging mode\r\n"));
  Serial.print(F("  LOG HEADER <ON|OFF>       Toggle header line on next file open\r\n"));
  Serial.print(F("  LOG FLUSH                 Flush buffer to SD (if open)\r\n"));
  Serial.print(F("  LOG STATUS                Report logging configuration/state\r\n"));
  Serial.print(F("  LOG TAIL <N>              Show last N data rows (prints header as 0)\r\n"));
  Serial.print(F("  LOG ROTATE <ON|OFF>       Toggle size-based log rotation\r\n"));
  Serial.print(F("  LOG MAXKB <KB>            Set rotation threshold in KB (clamped <= 1048576KB)\r\n"));
  Serial.print(F("  LOG CLEAR                 Delete current log file (next sample reopens)\r\n"));

  // Notes
  Serial.print(F("[NOTES]\r\n"));
  Serial.print(F("  Units: angles=centideg (0..24000); positions=mm (body frame). Left legs mirror IK angles.\r\n"));
  Serial.print(F("  Config: loop_hz, logging.rate_hz (<= loop_hz), test.trigait.{cycle_ms,height_mm,basex_mm,steplen_mm,lift_mm,overlap_pct}\r\n"));
  Serial.print(F("  PID config keys (Phase 2): pid.enabled, pid.kp_milli.<coxa|femur|tibia>, pid.ki_milli.<...>, pid.kd_milli.<...>\r\n"));
  Serial.print(F("  PID shadow mode keys: pid.mode=active|shadow, pid.shadow_report_hz=<1..50>\r\n"));
  Serial.print(F("[PID COMMANDS]\r\n"));
  Serial.print(F("  PID LIST\r\n"));
  Serial.print(F("  PID ENABLE | DISABLE\r\n"));
  Serial.print(F("  PID MODE <ACTIVE|SHADOW>\r\n"));
  Serial.print(F("  PID KP|KI|KD <COXA|FEMUR|TIBIA|ALL> <milli>\r\n"));
  Serial.print(F("  PID KDALPHA <COXA|FEMUR|TIBIA|ALL> <milli 0..1000>\r\n"));
  Serial.print(F("  PID SHADOW_RATE <hz 1..50>\r\n"));
  Serial.print(F("[IMP COMMANDS]\r\n"));
  Serial.print(F("  IMP LIST\r\n"));
  Serial.print(F("  IMP ENABLE | DISABLE\r\n"));
  Serial.print(F("  IMP MODE <OFF|JOINT|CART>\r\n"));
  Serial.print(F("  IMP SCALE <milli 0..1000>\r\n"));
  Serial.print(F("  IMP JSPRING|JDAMP <COXA|FEMUR|TIBIA|ALL> <milli>\r\n"));
  Serial.print(F("  IMP CSPRING|CDAMP <X|Y|Z|ALL> <milli>\r\n"));
  Serial.print(F("  IMP JDB <cd> (joint deadband)\r\n"));
  Serial.print(F("  IMP CDB <mm> (Cartesian deadband radius)\r\n"));
  Serial.print(F("[COMP (JOINT COMPLIANCE) COMMANDS]\r\n"));
  Serial.print(F("  COMP LIST\r\n"));
  Serial.print(F("  COMP ENABLE | DISABLE\r\n"));
  Serial.print(F("  COMP KP <COXA|FEMUR|TIBIA|ALL> <milli>\r\n"));
  Serial.print(F("  COMP RANGE <COXA|FEMUR|TIBIA|ALL> <cd>\r\n"));
  Serial.print(F("  COMP DEADBAND <COXA|FEMUR|TIBIA|ALL> <cd>\r\n"));
  Serial.print(F("  COMP LEAK <milli 0..1000>\r\n"));
  Serial.print(F("[EST (ESTIMATOR) COMMANDS]\r\n"));
  Serial.print(F("  EST LIST\r\n"));
  Serial.print(F("  EST CMD_ALPHA <milli 0..1000>\r\n"));
  Serial.print(F("  EST MEAS_ALPHA <milli 0..1000>\r\n"));
  Serial.print(F("  EST MEAS_VEL_ALPHA <milli 0..2000>\r\n"));
}

void rebootNow() {
  // Trigger ARM system reset via Application Interrupt and Reset Control Register
  volatile uint32_t* AIRCR = (volatile uint32_t*)0xE000ED0C;
  const uint32_t VECTKEY = 0x5FA << 16;
  *AIRCR = VECTKEY | (1u << 2); // SYSRESETREQ bit
  while (1) { /* wait for reset */ }
}

// Hardware angle offset wrappers — convert between centidegrees and LX-16A "tick" units
// Contract (cd = centidegrees; units = cd/24):
//   angle_offset_read(leg,id)          -> returns current hardware offset in cd
//   angle_offset_adjust(leg,id, cd)    -> sets absolute hardware offset (cd), converted to units internally
//   angle_offset_write(leg,id)         -> persists current offset to servo flash
// Notes:
//   - The underlying library stores angle offsets in ticks (units ≈ 0.24° = 24 cd).
//   - We expose a centidegree API here and convert using helpers in command_helpers.h.
//   - Clamp: firmware uses ±3000 cd (≈ ±30°), which corresponds to ±125 units.

extern LX16AServo* g_servo[6][3]; // from main sketch

// Search only within the specified leg row to avoid cross-leg ID collisions
int angle_offset_read(uint8_t leg, uint8_t id) {
  if (leg >= 6) return 0;
  LX16AServo* s = nullptr;
  for (uint8_t J = 0; J < 3; ++J) {
    LX16AServo* cur = g_servo[leg][J];
    if (cur && cur->_id == id) { s = cur; break; }
  }
  if (!s) return 0;
  // Library returns offset in units (ticks); convert to centidegrees
  int16_t units = s->read_angle_offset();
  int16_t cd = units_to_cd((int)units);
  // Clamp to firmware’s safety bounds
  if (cd < -3000) cd = -3000; if (cd > 3000) cd = 3000;
  return (int)cd;
}

bool angle_offset_adjust(uint8_t leg, uint8_t id, int16_t offset_cd) {
  if (leg >= 6) return false;
  LX16AServo* s = nullptr;
  for (uint8_t J = 0; J < 3; ++J) {
    LX16AServo* cur = g_servo[leg][J];
    if (cur && cur->_id == id) { s = cur; break; }
  }
  if (!s) return false;
  // Convert centidegrees to units (ticks) and clamp to ±125
  int16_t units = cd_to_units_round((int)offset_cd);
  if (units < -125) units = -125; if (units > 125) units = 125;
  s->angle_offset_adjust(units);
  return true;
}

bool angle_offset_write(uint8_t leg, uint8_t id) {
  if (leg >= 6) return false;
  LX16AServo* s = nullptr;
  for (uint8_t J = 0; J < 3; ++J) {
    LX16AServo* cur = g_servo[leg][J];
    if (cur && cur->_id == id) { s = cur; break; }
  }
  if (!s) return false;
  s->angle_offset_save();
  return true;
}


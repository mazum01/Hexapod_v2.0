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
// For rebootNow() register access (ARM SCB AIRCR)
#include <stdint.h>

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
// Globals from main sketch
extern volatile bool g_enabled;
extern volatile bool g_lockout;
extern volatile uint8_t g_last_err;
extern volatile uint32_t g_overrun_count;
extern volatile uint8_t g_rr_index;
extern volatile uint8_t g_servo_fb_fail_threshold;
extern volatile int16_t g_limit_min_cd[6][3];
extern volatile int16_t g_limit_max_cd[6][3];
extern volatile uint16_t g_rate_limit_cdeg_per_s;
extern void setServoId(uint8_t leg, uint8_t joint, uint8_t id);

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
extern void printERR(uint8_t code, const char* msg);
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
extern void processCmdSAFETY(const char* line,int s,int len);
extern void processCmdHOME(const char* line,int s,int len);
extern void processCmdSAVEHOME(const char* line,int s,int len);
extern void processCmdOFFSET(const char* line,int s,int len);

extern char    lineBuf[160];
extern uint8_t lineLen;

// -----------------------------------------------------------------------------
// handleLine — tokenizes first word and dispatches via CommandType enum.
// -----------------------------------------------------------------------------
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

  CommandType ct = parseCommandType(cmd);
  switch (ct) {
    case CMD_HELP:      processCmdHELP(line, s, len); break;
    case CMD_STATUS:    processCmdSTATUS(line, s, len); break;
    case CMD_REBOOT:    processCmdREBOOT(line, s, len); break;
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
    case CMD_SAFETY:    processCmdSAFETY(line, s, len); break;
    case CMD_HOME:      processCmdHOME(line, s, len); break;
    case CMD_SAVEHOME:  processCmdSAVEHOME(line, s, len); break;
    case CMD_OFFSET:    processCmdOFFSET(line, s, len); break;
    default:            printERR(1, "UNKNOWN_CMD"); break;
  }

  // Reflect the processed command back to the host
  Serial.print(F(" > "));
  Serial.print(line);
  Serial.print(F("\r\n"));
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
  Serial.print(F("FW "));
#ifdef FW_VERSION
  Serial.print(FW_VERSION);
#else
  Serial.print(F("unknown"));
#endif
  Serial.print(F(" build=")); Serial.print(__DATE__); Serial.print(F(" ")); Serial.print(__TIME__);
  Serial.print(F(" loop_hz=")); Serial.print((int)g_loop_hz);
  Serial.print(F(" UARTs: ")); Serial.print(Robot::UART_MAP_SUMMARY);
  Serial.print(F(" cfg=")); Serial.print(g_config_loaded ? 1 : 0);
  Serial.print(F(" keys=")); Serial.print((int)g_config_keys_applied);
  Serial.print(F(" cfg_loop_hz=")); Serial.print((int)g_config_loop_hz);
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

  if (!strcmp(key, "test.trigait.overlap_pct"))

  {
    float v = atof(val);
    if (v < 0.0f) v = 0.0f; if (v > 25.0f) v = 25.0f; // clamp to sane range
    g_test_overlap_pct = v;
    ++g_config_keys_applied;
    return;
  }
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
// Missing helpers required by commandprocessor.ino
//  - Tokenizer
//  - OK/ERR/HELP/STATUS printers
//  - Reboot
//  - Angle offset read/adjust/write (stubbed with in-memory table)
// -----------------------------------------------------------------------------

int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen) {
  if (!s || start < 0 || len < 0) { if (tokStart) *tokStart = 0; if (tokLen) *tokLen = 0; return 0; }
  int i = start;
  // Skip leading spaces/tabs
  while (i < len && (s[i] == ' ' || s[i] == '\t')) ++i;
  if (i >= len) { if (tokStart) *tokStart = 0; if (tokLen) *tokLen = 0; return len; }
  int j = i;
  while (j < len && s[j] != ' ' && s[j] != '\t') ++j;
  if (tokStart) *tokStart = i; if (tokLen) *tokLen = (j - i);
  return j;
}

void printOK() {
  Serial.print(F("OK"));
}

void printERR(uint8_t code, const char* msg) {
  Serial.print(F("ERR "));
  Serial.print((unsigned int)code);
  Serial.print(F(" "));
  if (msg) Serial.print(msg);
}

void printSTATUS()
{
  Serial.print(F("STATUS\r\n"));

  // Uptime line (days, hours, minutes, seconds, milliseconds), wrap-safe via 64-bit accumulator
  {
    static bool     s_up_inited = false;
    static uint32_t s_up_prev_ms = 0;
    static uint64_t s_up_total_ms = 0;
    uint32_t now_ms = millis();
    if (!s_up_inited) {
      s_up_inited = true;
      s_up_prev_ms = now_ms;
    }
    uint32_t delta = now_ms - s_up_prev_ms; // wrap-safe unsigned diff
    s_up_prev_ms = now_ms;
    s_up_total_ms += (uint64_t)delta;

    uint64_t ms_total = s_up_total_ms;
    uint64_t days  = ms_total / 86400000ULL; ms_total %= 86400000ULL;
    uint64_t hours = ms_total / 3600000ULL;  ms_total %= 3600000ULL;
    uint64_t mins  = ms_total / 60000ULL;    ms_total %= 60000ULL;
    uint64_t secs  = ms_total / 1000ULL;     ms_total %= 1000ULL;
    uint64_t ms    = ms_total;

    Serial.print(F("uptime="));
    Serial.print((unsigned long)days);  Serial.print(F("d "));
    Serial.print((unsigned long)hours); Serial.print(F("h "));
    Serial.print((unsigned long)mins);  Serial.print(F("m "));
    Serial.print((unsigned long)secs);  Serial.print(F("s "));
    Serial.print((unsigned long)ms);    Serial.print(F("ms\r\n"));
  }

  Serial.print(F("enabled=")); Serial.print(g_enabled ? 1 : 0); Serial.print(F("\r\n"));
  Serial.print(F("loop_hz=")); Serial.print(g_loop_hz); Serial.print(F("\r\n"));
  Serial.print(F("rr_idx=")); Serial.print(g_rr_index); Serial.print(F("\r\n"));
  Serial.print(F("last_err=")); Serial.print((int)g_last_err); Serial.print(F("\r\n"));
  Serial.print(F("overruns=")); Serial.print((unsigned long)g_overrun_count); Serial.print(F("\r\n"));

  Serial.print(F("legs="));
  for (int i = 0; i < 6; ++i)

  {
    Serial.print(leg_enabled_mask_get((uint8_t)i) ? 1 : 0);
  }
  Serial.print(F("\r\n"));

  Serial.print(F("jmask="));
  const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      Serial.print(joint_enabled_mask_get((uint8_t)i, (uint8_t)j) ? 1 : 0);
    }
  }
  Serial.print(F("\r\n"));

  Serial.print(F("vin_V="));
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      if (j) Serial.print(F("/"));

      uint16_t mV = g_meas_vin_mV[i][j];
      Serial.print(mV / 1000);
      Serial.print(F("."));
      Serial.print((mV % 1000) / 100);
    }
  }
  Serial.print(F("\r\n"));

  Serial.print(F("temp="));
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      if (j) Serial.print(F("/"));

      Serial.print((int)g_meas_temp_C[i][j]);
    }
  }
  Serial.print(F("\r\n"));

  Serial.print(F("pos_cd="));
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      if (j) Serial.print(F("/"));

      Serial.print((int)g_meas_pos_cd[i][j]);
    }
  }
  Serial.print(F("\r\n"));

  // Home positions (centidegrees)
  Serial.print(F("home_cd="));
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      if (j) Serial.print(F("/"));

      Serial.print((int)g_home_cd[i][j]);
    }
  }
  Serial.print(F("\r\n"));

  // TEST gait parameters (runtime-adjustable)
  Serial.print(F("test="));
  Serial.print(F("cycle_ms=")); Serial.print((unsigned int)g_test_cycle_ms);
  Serial.print(F(" base_x="));  Serial.print((int)lroundf(g_test_base_x_mm));
  Serial.print(F(" base_y="));  Serial.print((int)lroundf(g_test_base_y_mm));
  Serial.print(F(" step_z="));  Serial.print((int)lroundf(g_test_step_len_mm));
  Serial.print(F(" lift_y="));  Serial.print((int)lroundf(g_test_lift_y_mm));
  Serial.print(F(" overlap_pct=")); Serial.print((int)lroundf(g_test_overlap_pct));
  Serial.print(F("\r\n"));

  // Optional timing probes (microseconds)
#if MARS_TIMING_PROBES
  Serial.print(F("tprobe_us="));
  Serial.print((unsigned int)g_probe_serial_us);
  Serial.print(F("/"));
  Serial.print((unsigned int)g_probe_send_us);
  Serial.print(F("/"));
  Serial.print((unsigned int)g_probe_fb_us);
  Serial.print(F("/"));
  Serial.print((unsigned int)g_probe_tick_us);
  Serial.print(F("\r\n"));
#endif

  Serial.print(F("oos="));
  for (int i = 0; i < 6; ++i)

  {
    if (i) Serial.print(F(","));

    Serial.print(names[i]); Serial.print(F(":"));
    for (int j = 0; j < 3; ++j)

    {
      int idx = i * 3 + j;
      Serial.print(((g_servo_oos_mask >> idx) & 1u) ? 1 : 0);
    }
  }
  Serial.print(F("\r\n"));
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

  // Notes
  Serial.print(F("[NOTES]\r\n"));
  Serial.print(F("  Units: angles=centideg (0..24000); positions=mm (body frame). Left legs mirror IK angles.\r\n"));
}

void rebootNow() {
  // Trigger ARM system reset via Application Interrupt and Reset Control Register
  volatile uint32_t* AIRCR = (volatile uint32_t*)0xE000ED0C;
  const uint32_t VECTKEY = 0x5FA << 16;
  *AIRCR = VECTKEY | (1u << 2); // SYSRESETREQ bit
  while (1) { /* wait for reset */ }
}

// In-memory angle offset cache per-servo ID (units; ±125)
static int16_t s_angle_offset_units[256] = {0};

int angle_offset_read(uint8_t id) {
  return (int)s_angle_offset_units[id];
}

bool angle_offset_adjust(uint8_t id, int8_t delta) {
  int v = (int)s_angle_offset_units[id] + (int)delta;
  if (v < -125) v = -125; if (v > 125) v = 125;
  s_angle_offset_units[id] = (int16_t)v;
  return true;
}

bool angle_offset_write(uint8_t id) {
  (void)id; // Persist to hardware not yet implemented
  return true;
}


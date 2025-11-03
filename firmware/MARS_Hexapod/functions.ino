// =============================================================
//  functions.ino — Helper and Non-Core Functions (cleaned)
//  Parsing, printing, IK, config, and serial helpers for MARS_Hexapod
// =============================================================

#include <Arduino.h>
#include <ctype.h>
#include "robot_config.h"

// Optional SD config support guarded for Arduino concat order
#ifdef MARS_ENABLE_SD
#if MARS_ENABLE_SD
#include <SD.h>
#endif
#endif

// Error codes (ensure defined)
#ifndef E_OK
#define E_OK 0
#endif
#ifndef E_PARSE
#define E_PARSE 1
#endif
#ifndef E_BUS_IO
#define E_BUS_IO 2
#endif

// Shared globals from the main TU (only what's used here)
extern volatile uint16_t g_loop_hz;
extern volatile uint8_t  g_last_err;
extern volatile bool     g_enabled;
extern volatile bool     g_lockout;
extern volatile uint8_t  g_rr_index;
extern volatile uint32_t g_overrun_count;
extern volatile uint8_t  g_leg_enabled_mask;   // bit per leg
extern volatile uint32_t g_joint_enabled_mask; // bit per joint (leg*3 + joint)
extern volatile uint32_t g_servo_oos_mask;     // bit per joint (leg*3 + joint)
extern volatile uint8_t  g_servo_fb_fail_threshold; // OOS threshold (consecutive pos_read failures)
extern volatile int16_t  g_cmd_cd[6][3];
extern volatile int16_t  g_home_cd[6][3];
extern volatile int16_t  g_limit_min_cd[6][3];
extern volatile int16_t  g_limit_max_cd[6][3];
extern volatile uint16_t g_rate_limit_cdeg_per_s;
extern uint16_t          g_meas_vin_mV[6][3];
extern uint8_t           g_meas_temp_C[6][3];
extern int16_t           g_meas_pos_cd[6][3];
// TEST gait parameters (from main TU)
extern float    g_test_base_y_mm;
extern float    g_test_base_x_mm;
extern float    g_test_step_len_mm;
extern uint32_t g_test_cycle_ms;
extern float    g_test_lift_y_mm;
extern float    g_test_overlap_pct;

// Timing probes from main TU (enabled when MARS_TIMING_PROBES)
#if MARS_TIMING_PROBES
extern volatile uint16_t g_probe_serial_us;
extern volatile uint16_t g_probe_send_us;
extern volatile uint16_t g_probe_fb_us;
extern volatile uint16_t g_probe_tick_us;
#endif

// Mode change helpers implemented in main .ino
extern void modeSetTest();
extern void modeSetIdle();
// Synchronous position read helper (centidegrees); returns -1 on failure
extern int16_t readServoPosCdSync(uint8_t leg, uint8_t joint);

// Config status
extern bool     g_config_loaded;
extern uint16_t g_config_keys_applied;
extern uint16_t g_config_loop_hz;

// Serial input buffer (lives in main .ino)
extern char    lineBuf[160];
extern uint8_t lineLen;

// Servo helpers
extern uint8_t servoId(uint8_t leg, uint8_t joint);
extern void    setServoId(uint8_t leg, uint8_t joint, uint8_t id);
extern void    setServoTorqueNow(uint8_t leg, uint8_t joint, bool on);

#ifndef FW_VERSION
#define FW_VERSION "0.0.0"
#endif

// Track last command for echoing
static char g_last_cmd[160] __attribute__((used)) = {0};

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

static inline int legIndexFromToken(const char* t)
{
  if (!t) return -1;

  if (!strcmp(t, "LF")) return 0;

  if (!strcmp(t, "LM")) return 1;

  if (!strcmp(t, "LR")) return 2;

  if (!strcmp(t, "RF")) return 3;

  if (!strcmp(t, "RM")) return 4;

  if (!strcmp(t, "RR")) return 5;

  return -1;
}

static inline int jointIndexFromToken(const char* t)
{
  if (!t) return -1;

  if (!strcmp(t, "COXA") || !strcmp(t, "coxa")) return 0;

  if (!strcmp(t, "FEMUR") || !strcmp(t, "femur")) return 1;

  if (!strcmp(t, "TIBIA") || !strcmp(t, "tibia")) return 2;

  return -1;
}

static inline bool leg_enabled_mask_get(uint8_t leg)
{
  return (g_leg_enabled_mask >> leg) & 0x1;
}

static inline bool joint_enabled_mask_get(uint8_t leg, uint8_t joint)
{
  uint8_t idx = (uint8_t)(leg * 3u + joint);
  return (g_joint_enabled_mask >> idx) & 0x1u;
}

// Clamp and apply loop frequency from config
static void configApplyLoopHz(uint16_t hz)
{
  if (hz < 50) hz = 50;

  if (hz > 500) hz = 500;

  g_loop_hz = hz;
  g_config_loop_hz = hz;
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

// Splash banner
void splash()
{
  if (!Serial) return;

  // ASCII banner (kept short) and concise runtime summary per spec
  Serial.print(R"RAW(
 +----------------------------------+
 |  __  __    _    ____  ____       |
 | |  \/  |  / \  |  _ \/ ___|      |
 | | |\/| | / _ \ | |_) \___ \      |
 | | |  | |/ ___ \|  _ < ___) |     |
 | |_|  |_/_/   \_\_| \_\____/      |
 |                                  |
 +----------------------------------+
)RAW");

  Serial.print(F("MARS — Modular Autonomous Robotic System\r\n"));
  Serial.print(F("Build: ")); Serial.print(__DATE__); Serial.print(F(" ")); Serial.print(__TIME__);
  Serial.print(F(" | fw=")); Serial.print(FW_VERSION);
  Serial.print(F(" | loop_hz=")); Serial.print(g_loop_hz);
  Serial.print(F("\r\n"));

  // UART mapping summary (from robot_config.h)
  Serial.print(F("UART: ")); Serial.print(Robot::UART_MAP_SUMMARY); Serial.print(F("\r\n"));

  // Config status
  Serial.print(F("config: sd=")); Serial.print(g_config_loaded ? 1 : 0);
  Serial.print(F(" keys=")); Serial.print((unsigned int)g_config_keys_applied);
  Serial.print(F(" cfg_loop_hz=")); Serial.print(g_config_loop_hz);
  Serial.print(F("\r\n"));
}

// IK helpers
static inline int16_t clamp_cd(int32_t cd)
{
  if (cd < 0) cd = 0;

  if (cd > 24000) cd = 24000;

  return (int16_t)cd;
}

bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3])
{
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

// Serial helpers
void processSerial()
{
  while (Serial && Serial.available() > 0)

  {
    int c = Serial.read();
    if (c == '\r')

    {
      continue;
    }

    if (c == '\n')

    {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0)

      {
        size_t n = strlen(lineBuf);
        if (n >= sizeof(g_last_cmd)) n = sizeof(g_last_cmd) - 1;

        memcpy(g_last_cmd, lineBuf, n);
        g_last_cmd[n] = 0;
        handleLine(lineBuf);
      }

      lineLen = 0;
    }
    else if ((size_t)lineLen + 1 < sizeof(lineBuf))

    {
      lineBuf[lineLen++] = (char)c;
    }
    else

    {
      lineLen = 0; // overflow -> reset
    }
  }
}

int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen)
{
  int i = start;
  while (i < len && isspace((int)s[i]))

  {
    ++i;
  }

  int b = i;
  while (i < len && !isspace((int)s[i]))

  {
    ++i;
  }

  int e = i;
  *tokStart = b;
  *tokLen = (e > b) ? (e - b) : 0;
  return i;
}

void printOK()
{
  Serial.print(F("OK "));
  Serial.print(g_last_cmd);
  Serial.print(F("\r\n"));
}

void printERR(uint8_t code, const char* msg)
{
  g_last_err = code;
  Serial.print(F("ERR "));
  Serial.print((int)code);
  Serial.print(F(" "));
  Serial.print(msg);
  Serial.print(F(" "));
  Serial.print(g_last_cmd);
  Serial.print(F("\r\n"));
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

// REMINDER: When commands change, update this HELP output and docs/PROJECT_SPEC.md.
void printHELP()
{
  Serial.print(F("CMDS:\r\n"));
  Serial.print(F("  HELP\r\n"));
  Serial.print(F("  STATUS\r\n"));
  Serial.print(F("  ENABLE | DISABLE\r\n"));
  Serial.print(F("  LEG <LEG|ALL> <ENABLE|DISABLE>\r\n"));
  Serial.print(F("  SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>\r\n"));
  Serial.print(F("  MODE <TEST|IDLE>\r\n"));
  Serial.print(F("  I  (shortcut for MODE IDLE)\r\n"));
  Serial.print(F("  T  (shortcut for MODE TEST)\r\n"));
  Serial.print(F("  TEST CYCLE <ms>     (750..10000)\r\n"));
  Serial.print(F("  TEST HEIGHT <mm>    (ground height; negative is down)\r\n"));
  Serial.print(F("  TEST BASEX <mm>     (lateral offset)\r\n"));
  Serial.print(F("  TEST STEPLEN <mm>   (forward/back amplitude)\r\n"));
  Serial.print(F("  LEGS\r\n"));
  Serial.print(F("  SERVOS\r\n"));
  Serial.print(F("  REBOOT\r\n"));
  Serial.print(F("  STAND\r\n"));
  Serial.print(F("  HOME\r\n"));
  Serial.print(F("  SAVEHOME\r\n"));
  Serial.print(F("  FOOT <LEG x y z>\r\n"));
  Serial.print(F("  FEET <x1 y1 z1 ... x6 y6 z6>  (order: LF,LM,LR,RF,RM,RR)\r\n"));
  Serial.print(F("  RAW <LEG> <JOINT> <centideg>\r\n"));
  Serial.print(F("  RAW3 <LEG> <c_cd f_cd t_cd>\r\n"));
    Serial.print(F("  TEST CYCLE <ms>     (750..10000)\r\n"));
    Serial.print(F("  TEST HEIGHT <mm>    (ground height; negative is down)\r\n"));
    Serial.print(F("  TEST BASEX <mm>     (lateral offset)\r\n"));
    Serial.print(F("  TEST STEPLEN <mm>   (forward/back amplitude)\r\n"));
    Serial.print(F("  TEST LIFT <mm>      (step height / lift amount)\r\n"));
  Serial.print(F("  TEST OVERLAP <pct>  (0..25, percent of total cycle for overlap; persisted to /config.txt)\r\n"));
}

void rebootNow()
{
  __disable_irq();
  SCB_AIRCR = 0x05FA0004; // request system reset
  while (1)

  {
  }
}

void handleLine(const char* line)
{
  int len = (int)strlen(line);
  int s = 0, ts = 0, tl = 0;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) return;

  char cmd[16];
  int n = min(tl, (int)sizeof(cmd) - 1);
  memcpy(cmd, line + ts, n); cmd[n] = 0;
  for (int i = 0; cmd[i]; ++i) cmd[i] = toupper(cmd[i]);

  if (!strcmp(cmd, "HELP"))

  {
    printHELP();
    printOK();
    return;
  }

  if (!strcmp(cmd, "STATUS"))

  {
    printSTATUS();
    printOK();
    return;
  }

  if (!strcmp(cmd, "REBOOT"))

  {
    printOK();
    Serial.flush();
    delay(10);
    rebootNow();
    return;
  }

  if (!strcmp(cmd, "ENABLE"))

  {
    g_enabled = true; g_last_err = E_OK;
    printOK();
    return;
  }

  if (!strcmp(cmd, "DISABLE"))

  {
    g_enabled = false; g_last_err = E_OK;
    for (int L = 0; L < 6; ++L)

    {
      for (int J = 0; J < 3; ++J)

      {
        setServoTorqueNow((uint8_t)L, (uint8_t)J, false);
      }
    }

    printOK();
    return;
  }

  if (!strcmp(cmd, "LEGS"))

  {
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("LEGS "));
    for (int i = 0; i < 6; ++i)

    {
      if (i) Serial.print(F(" "));

      Serial.print(names[i]); Serial.print(F("="));
      Serial.print(leg_enabled_mask_get((uint8_t)i) ? 1 : 0);
    }
    Serial.print(F("\r\n"));
    printOK();
    return;
  }

  if (!strcmp(cmd, "SERVOS"))

  {
    const char* names[6] = {"LF","LM","LR","RF","RM","RR"};
    Serial.print(F("SERVOS "));
    for (int i = 0; i < 6; ++i)

    {
      if (i) Serial.print(F(" "));

      Serial.print(names[i]); Serial.print(F("="));
      for (int j = 0; j < 3; ++j)

      {
        Serial.print(joint_enabled_mask_get((uint8_t)i, (uint8_t)j) ? 1 : 0);
      }
    }
    Serial.print(F("\r\n"));
    printOK();
    return;
  }

  if (!strcmp(cmd, "LEG"))
  {
    // LEG <LEG|ALL> <ENABLE|DISABLE>
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);

    bool doEnable = !strcmp(act, "ENABLE");
    bool doDisable = !strcmp(act, "DISABLE");
    if (!doEnable && !doDisable) { printERR(E_PARSE, "BAD_ARG"); return; }

    auto apply_leg = [&](int L)
    {
      if (L < 0 || L >= 6) return;
      if (doEnable) {
        g_leg_enabled_mask |= (1u << L);
        if (g_enabled) {
          for (uint8_t J = 0; J < 3; ++J) {
            uint8_t idx = (uint8_t)(L * 3u + J);
            if ((g_joint_enabled_mask >> idx) & 1u) {
              setServoTorqueNow((uint8_t)L, J, true);
            }
          }
        }
      } else {
        g_leg_enabled_mask &= ~(1u << L);
        for (uint8_t J = 0; J < 3; ++J) {
          setServoTorqueNow((uint8_t)L, J, false);
        }
      }
    };

    if (!strcmp(legt, "ALL")) {
      for (int L = 0; L < 6; ++L) apply_leg(L);
    } else {
      int leg = legIndexFromToken(legt);
      if (leg < 0) { printERR(E_PARSE, "BAD_LEG"); return; }
      apply_leg(leg);
    }

    printOK();
    return;
  }

  if (!strcmp(cmd, "SERVO"))
  {
    // SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);

    bool doEnable = !strcmp(act, "ENABLE");
    bool doDisable = !strcmp(act, "DISABLE");
    if (!doEnable && !doDisable) { printERR(E_PARSE, "BAD_ARG"); return; }

    auto apply_servo = [&](int L, int J)
    {
      if (L < 0 || L >= 6 || J < 0 || J >= 3) return;
      uint8_t idx = (uint8_t)(L * 3u + J);
      if (doEnable) {
        g_joint_enabled_mask |= (1u << idx);
        if (g_enabled && ((g_leg_enabled_mask >> L) & 1u)) {
          setServoTorqueNow((uint8_t)L, (uint8_t)J, true);
        }
      } else {
        g_joint_enabled_mask &= ~(1u << idx);
        setServoTorqueNow((uint8_t)L, (uint8_t)J, false);
      }
    };

    if (!strcmp(legt, "ALL")) {
      if (!strcmp(jtok, "ALL")) {
        for (int L = 0; L < 6; ++L) {
          for (int J = 0; J < 3; ++J) apply_servo(L, J);
        }
      } else {
        int J = jointIndexFromToken(jtok);
        if (J < 0) { printERR(E_PARSE, "BAD_JOINT"); return; }
        for (int L = 0; L < 6; ++L) apply_servo(L, J);
      }
    } else {
      int leg = legIndexFromToken(legt);
      if (leg < 0) { printERR(E_PARSE, "BAD_LEG"); return; }
      if (!strcmp(jtok, "ALL")) {
        for (int J = 0; J < 3; ++J) apply_servo(leg, J);
      } else {
        int J = jointIndexFromToken(jtok);
        if (J < 0) { printERR(E_PARSE, "BAD_JOINT"); return; }
        apply_servo(leg, J);
      }
    }

    printOK();
    return;
  }

  if (!strcmp(cmd, "RAW"))

  {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
    int leg = legIndexFromToken(legt); if (leg < 0) { printERR(E_PARSE, "BAD_LEG"); return; }

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char jt[8] = {0}; int nj = min(tl, 7); memcpy(jt, line + ts, nj); for (int i = 0; i < nj; ++i) jt[i] = toupper(jt[i]);
    int joint = jointIndexFromToken(jt); if (joint < 0) { printERR(E_PARSE, "BAD_JOINT"); return; }

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long v = atol(nb);
    if (v < 0) v = 0; if (v > 24000) v = 24000;

    g_cmd_cd[leg][joint] = (int16_t)v;
    printOK();
    return;
  }

  if (!strcmp(cmd, "RAW3"))

  {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
    int leg = legIndexFromToken(legt); if (leg < 0) { printERR(E_PARSE, "BAD_LEG"); return; }

    long vals[3];
    for (int j = 0; j < 3; ++j)

    {
      s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
      char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long v = atol(nb);
      if (v < 0) v = 0; if (v > 24000) v = 24000; vals[j] = v;

    }

    g_cmd_cd[leg][0] = (int16_t)vals[0];
    g_cmd_cd[leg][1] = (int16_t)vals[1];
    g_cmd_cd[leg][2] = (int16_t)vals[2];
    printOK();
    return;
  }

  if (!strcmp(cmd, "FOOT"))

  {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
    int leg = legIndexFromToken(legt); if (leg < 0) { printERR(E_PARSE, "BAD_LEG"); return; }

    float vals[3];
    for (int j = 0; j < 3; ++j)

    {
      s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
      char nb[24] = {0}; int nn = min(tl, 23); memcpy(nb, line + ts, nn); nb[nn] = 0; vals[j] = atof(nb);
    }

    int16_t out[3];
    if (!calculateIK((uint8_t)leg, vals[0], vals[1], vals[2], out))

    {
      printERR(E_PARSE, "IK_FAIL");
      return;
    }

    // Mirror left legs on coxa around home for symmetry
    bool isRight = (legt[0] == 'R');
    if (isRight)

    {
      g_cmd_cd[leg][0] = out[0];
      g_cmd_cd[leg][1] = out[1];
      g_cmd_cd[leg][2] = out[2];
    }
    else

    {
      g_cmd_cd[leg][0] = 2 * g_home_cd[leg][0] - out[0];
      g_cmd_cd[leg][1] = 2 * g_home_cd[leg][1] - out[1];
      g_cmd_cd[leg][2] = 2 * g_home_cd[leg][2] - out[2];
    }

    printOK();
    return;
  }

  if (!strcmp(cmd, "FEET"))

  {
    float p[6][3];
    for (int i = 0; i < 6; ++i)

    {
      for (int j = 0; j < 3; ++j)

      {
        s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
        char nb[24] = {0}; int nn = min(tl, 23); memcpy(nb, line + ts, nn); nb[nn] = 0; p[i][j] = atof(nb);
      }
    }

    for (int L = 0; L < 6; ++L)

    {
      int16_t out[3];
      if (!calculateIK((uint8_t)L, p[L][0], p[L][1], p[L][2], out))

      {
        printERR(E_PARSE, "IK_FAIL");
        return;
      }

      // Mirror left legs around home on coxa
      bool isRight = (L >= 3);
      if (isRight)

      {
        g_cmd_cd[L][0] = out[0];
        g_cmd_cd[L][1] = out[1];
        g_cmd_cd[L][2] = out[2];
      }
      else

      {
        g_cmd_cd[L][0] = 2 * g_home_cd[L][0] - out[0];
        g_cmd_cd[L][1] = 2 * g_home_cd[L][1] - out[1];
        g_cmd_cd[L][2] = 2 * g_home_cd[L][2] - out[2];
      }
    }

    printOK();
    return;
  }

  // MODE handler (TEST/IDLE)
  if (!strcmp(cmd, "MODE"))
  {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char mtok[8] = {0}; int nm = min(tl, 7); memcpy(mtok, line + ts, nm); for (int i = 0; i < nm; ++i) mtok[i] = toupper(mtok[i]);
    if (!strcmp(mtok, "TEST")) {
      modeSetTest();
      printOK();
      return;
    } else if (!strcmp(mtok, "IDLE")) {
      modeSetIdle();
      printOK();
      return;
    }
    printERR(E_PARSE, "BAD_ARG");
    return;
  }

  // 'I' — shortcut for MODE IDLE
  if (!strcmp(cmd, "I"))
  {
    modeSetIdle();
    printOK();
    return;
  }

  // 'T' — shortcut for MODE TEST
  if (!strcmp(cmd, "T"))
  {
    modeSetTest();
    printOK();
    return;
  }

  // TEST parameter commands
  if (!strcmp(cmd, "TEST"))
  {
    // TEST <CYCLE|HEIGHT|BASEX|STEPLEN|LIFT> <value>
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char sub[12] = {0}; int ns = min(tl, 11); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(E_PARSE, "BAD_ARG"); return; }
    char vb[24] = {0}; int nv = min(tl, 23); memcpy(vb, line + ts, nv); vb[nv] = 0;

    if (!strcmp(sub, "CYCLE")) {
      long ms = atol(vb);
      if (ms < 750) ms = 750; if (ms > 10000) ms = 10000;
      g_test_cycle_ms = (uint32_t)ms;
      printOK();
      return;
    } else if (!strcmp(sub, "HEIGHT")) {
      g_test_base_y_mm = atof(vb);
      printOK();
      return;
    } else if (!strcmp(sub, "BASEX")) {
      g_test_base_x_mm = atof(vb);
      printOK();
      return;
    } else if (!strcmp(sub, "STEPLEN")) {
      float v = atof(vb);
      if (v < 0.0f) v = 0.0f;
      g_test_step_len_mm = v;
      printOK();
      return;
    } else if (!strcmp(sub, "LIFT")) {
      float v = atof(vb);
      if (v < 0.0f) v = 0.0f;
      g_test_lift_y_mm = v;
      printOK();
      return;
    } else if (!strcmp(sub, "OVERLAP")) {
      // Persisted runtime parameter: percent of total cycle reserved for overlap (0..25)
      float v = atof(vb);
      if (v < 0.0f) v = 0.0f; if (v > 25.0f) v = 25.0f;
      g_test_overlap_pct = v;

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      if (!SD.begin(BUILTIN_SDCARD)) { printERR(E_BUS_IO, "NO_SD"); return; }

      // Stream copy config with replacement of test.trigait.overlap_pct, append if missing
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); printERR(E_BUS_IO, "SD_WRITE_FAIL"); return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break;
          if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0;
            const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "test.trigait.overlap_pct=";
            size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              // Replace with new value (integer percent)
              fout.print(key);
              long iv = lroundf(g_test_overlap_pct);
              fout.print((int)iv);
              fout.print('\n');
              replaced = true;
            } else {
              fout.print(linebuf); fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          } else {
            // overflow: flush partial as-is
            linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("test.trigait.overlap_pct=");
        long iv = lroundf(g_test_overlap_pct);
        fout.print((int)iv);
        fout.print('\n');
      }
      fout.close();
      SD.remove("/config.txt");
      SD.rename("/config.tmp", "/config.txt");
      printOK();
      return;
#else
      printERR(E_BUS_IO, "NO_SD");
      return;
#endif
    }
    printERR(E_PARSE, "BAD_ARG");
    return;
  }

  if (!strcmp(cmd, "STAND"))
  {
    // IK a neutral stance at base X/Y with Z=0 for all legs.
    // Use the same base values as TEST gait to keep behavior consistent.
    const float BASE_Y = g_test_base_y_mm; // mm (negative is down)
    const float BASE_X = g_test_base_x_mm;  // mm (lateral)
    for (int L = 0; L < 6; ++L) {
      int16_t out[3];
      bool ok = calculateIK((uint8_t)L, BASE_X, BASE_Y, 0.0f, out);
      if (!ok) { printERR(E_PARSE, "IK_FAIL"); return; }
      bool isRight = (L >= 3);
      if (isRight) {
        g_cmd_cd[L][0] = out[0];
        g_cmd_cd[L][1] = out[1];
        g_cmd_cd[L][2] = out[2];
      } else {
        // Mirror around home for left legs to match current convention
        g_cmd_cd[L][0] = (int16_t)(2 * g_home_cd[L][0] - out[0]);
        g_cmd_cd[L][1] = (int16_t)(2 * g_home_cd[L][1] - out[1]);
        g_cmd_cd[L][2] = (int16_t)(2 * g_home_cd[L][2] - out[2]);
      }
    }
    printOK();
    return;
  }

  // HOME — move enabled, in-service servos to their home positions
  if (!strcmp(cmd, "HOME"))
  {
    for (int L = 0; L < 6; ++L) {
      if (!leg_enabled_mask_get((uint8_t)L)) continue;
      for (int J = 0; J < 3; ++J) {
        int idx = L * 3 + J;
        bool jointEn = joint_enabled_mask_get((uint8_t)L, (uint8_t)J);
        bool inService = ((g_servo_oos_mask >> idx) & 1u) == 0;
        if (jointEn && inService) {
          g_cmd_cd[L][J] = g_home_cd[L][J];
        }
      }
    }
    printOK();
    return;
  }

  // SAVEHOME — read positions for enabled, in-service servos and persist to /config.txt as home_cd.<LEG>.<joint>=<cd>
  if (!strcmp(cmd, "SAVEHOME"))
  {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(E_BUS_IO, "NO_SD"); return; }

    // Build updates and track which keys we replace
    struct Update { uint8_t L; uint8_t J; int16_t cd; bool have; bool replaced; };
    Update ups[18]; int upCount = 0;
    for (uint8_t L = 0; L < 6; ++L) {
      if (!leg_enabled_mask_get(L)) continue;
      for (uint8_t J = 0; J < 3; ++J) {
        int idx = L * 3 + J;
        bool jointEn = joint_enabled_mask_get(L, J);
        bool inService = ((g_servo_oos_mask >> idx) & 1u) == 0;
        if (!jointEn || !inService) continue;
        int16_t cd = readServoPosCdSync(L, J);
        if (cd >= 0) {
          ups[upCount++] = {L, J, cd, true, false};
          g_home_cd[L][J] = cd; // update runtime home as well
        }
      }
    }
    // Open existing config for read; temp for write
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(E_BUS_IO, "SD_WRITE_FAIL"); return; }
    const char* legNames[6] = {"LF","LM","LR","RF","RM","RR"};
    const char* jointNames[3] = {"coxa","femur","tibia"};
    if (fin) {
      // Stream copy with replacements
      static char line[256]; int len = 0;
      while (fin.available()) {
        int ch = fin.read(); if (ch < 0) break;
        if (ch == '\r') continue;
        if (ch == '\n') {
          line[len] = 0;
          bool didReplace = false;
          // Try to match any key
          for (int i = 0; i < upCount; ++i) {
            char key[40];
            snprintf(key, sizeof(key), "home_cd.%s.%s=", legNames[ups[i].L], jointNames[ups[i].J]);
            const char* p = line; while (*p == ' ' || *p == '\t') ++p;
            if (strncmp(p, key, strlen(key)) == 0) {
              // Write updated key
              fout.print(key);
              fout.print((int)ups[i].cd);
              fout.print('\n');
              ups[i].replaced = true;
              didReplace = true;
              break;
            }
          }
          if (!didReplace) {
            fout.print(line);
            fout.print('\n');
          }
          len = 0;
        } else if (len + 1 < (int)sizeof(line)) {
          line[len++] = (char)ch;
        } else {
          // overflow: flush partial line as-is
          line[len] = 0; fout.print(line); fout.print('\n'); len = 0;
        }
      }
      fin.close();
    }
    // Append any keys not replaced
    for (int i = 0; i < upCount; ++i) {
      if (!ups[i].replaced) {
        fout.print("home_cd."); fout.print(legNames[ups[i].L]); fout.print("."); fout.print(jointNames[ups[i].J]); fout.print("="); fout.print((int)ups[i].cd); fout.print('\n');
      }
    }
    fout.close();
    // Replace original
    SD.remove("/config.txt");
    SD.rename("/config.tmp", "/config.txt");
    printOK();
    return;
#else
    printERR(E_BUS_IO, "NO_SD");
    return;
#endif
  }

  printERR(2, "UNKNOWN_CMD");
}

void buffersInit()
{
  // Reset serial parsing buffers
  lineLen = 0;
  lineBuf[0] = '\0';
  g_last_cmd[0] = '\0';

  // Initialize 74HC126 buffer enable pins: LOW = RX (high-Z), HIGH = TX drive
  for (uint8_t i = 0; i < 6; ++i)

  {

    int pin = Robot::BUFFER_ENABLE_PINS[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // default to RX (high-Z driver)
  }
}

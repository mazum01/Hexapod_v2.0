// commandprocessor.ino — Modular command handling (refactored)
// Each top-level serial command now handled by a dedicated routine.
// Keeps core parser minimal and enables switch-based dispatch.

#include <Arduino.h>
#include <ctype.h>
#include <lx16a-servo.h>
#include "robot_config.h"
#include "command_types.h"
#include "command_helpers.h"
#include "globals.h"
#include "MarsImpedance.hpp"

// Ensure feature flags have sane defaults in this TU as well
#ifndef MARS_ENABLE_SD
#define MARS_ENABLE_SD 1
#endif

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#include <SD.h>
#endif

// NOTE: All extern declarations are now centralized in globals.h.

// -------------------------------------------------------------------------------------------------
// Command enumeration & mapping (enum defined in command_types.h)
// -------------------------------------------------------------------------------------------------

// Helpers provided by command_helpers.h

CommandType parseCommandType(const char* cmd) {
  if (!cmd || !*cmd) return CMD_UNKNOWN;
  // Single-letter shortcuts first
  if (cmd[1] == 0) {
    if (cmd[0] == 'I') return CMD_I;
    if (cmd[0] == 'T') return CMD_T;
    if (cmd[0] == 'Y') return CMD_Y;
  }
  if (!strcmp(cmd, "HELP")) return CMD_HELP;
  if (!strcmp(cmd, "STATUS")) return CMD_STATUS;
  if (!strcmp(cmd, "REBOOT")) return CMD_REBOOT;
  if (!strcmp(cmd, "ENABLE")) return CMD_ENABLE;
  if (!strcmp(cmd, "DISABLE")) return CMD_DISABLE;
  if (!strcmp(cmd, "FK")) return CMD_FK;
  if (!strcmp(cmd, "LEGS")) return CMD_LEGS;
  if (!strcmp(cmd, "SERVOS")) return CMD_SERVOS;
  if (!strcmp(cmd, "LEG")) return CMD_LEG;
  if (!strcmp(cmd, "SERVO")) return CMD_SERVO;
  if (!strcmp(cmd, "RAW")) return CMD_RAW;
  if (!strcmp(cmd, "RAW3")) return CMD_RAW3;
  if (!strcmp(cmd, "FOOT")) return CMD_FOOT;
  if (!strcmp(cmd, "FEET")) return CMD_FEET;
  if (!strcmp(cmd, "MODE")) return CMD_MODE;
  if (!strcmp(cmd, "TEST")) return CMD_TEST;
  if (!strcmp(cmd, "STAND")) return CMD_STAND;
  if (!strcmp(cmd, "SAFETY")) return CMD_SAFETY;
  if (!strcmp(cmd, "HOME")) return CMD_HOME;
  if (!strcmp(cmd, "SAVEHOME")) return CMD_SAVEHOME;
  if (!strcmp(cmd, "OFFSET")) return CMD_OFFSET;
  if (!strcmp(cmd, "LOG")) return CMD_LOG;
  if (!strcmp(cmd, "LOOP")) return CMD_LOOP;
  if (!strcmp(cmd, "TUCK")) return CMD_TUCK;
  if (!strcmp(cmd, "PID")) return CMD_PID;
  if (!strcmp(cmd, "IMP")) return CMD_IMP;
  if (!strcmp(cmd, "COMP")) return CMD_COMP;
  if (!strcmp(cmd, "EST")) return CMD_EST;
  if (!strcmp(cmd, "LIMITS")) return CMD_LIMITS;
  if (!strcmp(cmd, "CONFIG")) return CMD_CONFIG;
  if (!strcmp(cmd, "TELEM")) return CMD_TELEM;
  return CMD_UNKNOWN;
}

// -------------------------------------------------------------------------------------------------
// Per-command routines (minimal wrappers preserving original semantics)
// Each function receives: original line, current parse index (start after command token), and total length.
// -------------------------------------------------------------------------------------------------

void processCmdHELP(const char* line, int s, int len)
{
  (void)line;
  (void)s;
  (void)len;
  printHELP();
}
void processCmdSTATUS(const char* line, int s, int len)
{
  (void)line;
  (void)s;
  (void)len;
  printSTATUS();
}
void processCmdREBOOT(const char* line, int s, int len)
{
  (void)line;
  (void)s;
  (void)len; /* centralized layer will handle OK+reboot */
}

// Y <0|1|ON|OFF|TRUE|FALSE> — Telemetry (S1/S2/S3) master toggle (single-char command)
void processCmdY(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char vb[8] = {0}; int nv = tl < 7 ? tl : 7; memcpy(vb, line + ts, nv);
  for (int i = 0; i < nv; ++i) vb[i] = (char)toupper(vb[i]);
  bool on = (!strcmp(vb, "1") || !strcmp(vb, "ON") || !strcmp(vb, "TRUE"));
  g_telem_enabled = on ? 1 : 0;
}

// TELEM <ASCII|BIN> [0|1|ON|OFF|TRUE|FALSE]
// Select telemetry stream format. Master enable remains controlled by Y 1/Y 0.
// - TELEM ASCII          -> force ASCII segments (S1..S5)
// - TELEM BIN 1|0        -> enable/disable binary framed telemetry
void processCmdTELEM(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) { printERR(1, "BAD_ARG"); return; }

  char modeb[8] = {0};
  int nm = (tl < 7) ? tl : 7;
  memcpy(modeb, line + ts, nm);
  for (int i = 0; i < nm; ++i) modeb[i] = (char)toupper(modeb[i]);

  if (!strcmp(modeb, "ASCII") || !strcmp(modeb, "TEXT")) {
    g_telem_bin_enabled = 0;
    return;
  }

  if (!strcmp(modeb, "BIN") || !strcmp(modeb, "BINARY")) {
    int vs, vl;
    s = nextToken(line, s, len, &vs, &vl);
    if (vl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[8] = {0};
    int nv = (vl < 7) ? vl : 7;
    memcpy(vb, line + vs, nv);
    for (int i = 0; i < nv; ++i) vb[i] = (char)toupper(vb[i]);
    bool on = (!strcmp(vb, "1") || !strcmp(vb, "ON") || !strcmp(vb, "TRUE"));
    g_telem_bin_enabled = on ? 1 : 0;
    return;
  }

  printERR(1, "BAD_ARG");
}

// CONFIG DUMP — stream /config.txt contents to the console (read-only)
void processCmdCONFIG(const char* line, int s, int len)
{
  (void)line; (void)s; (void)len;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  if (!SD.begin(BUILTIN_SDCARD)) {
    printERR(1, "NO_SD");
    return;
  }
  File f = SD.open("/config.txt", FILE_READ);
  if (!f) {
    printERR(2, "NO_CONFIG");
    return;
  }
  Serial.print(F("# BEGIN CONFIG\r\n"));
  while (f.available()) {
    int ch = f.read();
    if (ch < 0) break;
    Serial.write((char)ch);
  }
  f.close();
  Serial.print(F("\r\n# END CONFIG\r\n"));
#else
  printERR(1, "NO_SD");
#endif
}
// LOOP <hz> — Set control loop frequency and persist loop_hz in config (when SD enabled)
void processCmdLOOP(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char nb[12] = {0};
  int nn = min(tl, 11);
  memcpy(nb, line + ts, nn);
  nb[nn] = 0;
  long hz = atol(nb);
  if (hz <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  if (hz > 500) hz = 500; // hard cap for safety
  // Apply loop Hz immediately
  configApplyLoopHz((uint16_t)hz);

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // Persist loop_hz in /config.txt as loop_hz=<hz>
  if (SD.begin(BUILTIN_SDCARD)) {
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (fout) {
      bool replaced = false;
      if (fin) {
        static char linebuf[256];
        int llen = 0;
        while (fin.available()) {
          int ch = fin.read();
          if (ch < 0) break;
          if (ch == '\r') continue;
          if (ch == '\n') {
            linebuf[llen] = 0;
            const char* p = linebuf;
            while (*p == ' ' || *p == '\t') ++p;
            const char* key = "loop_hz=";
            size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print((unsigned int)hz);
              fout.print('\n');
              replaced = true;
            } else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch;
          } else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("loop_hz=");
        fout.print((unsigned int)hz);
        fout.print('\n');
      }
      fout.close();
      SD.remove("/config.txt");
      SD.rename("/config.tmp", "/config.txt");
    } else if (fin) {
      fin.close();
    }
  }
#endif
}
void processCmdENABLE(const char* line, int s, int len)
{
  (void)line;
  (void)s;
  (void)len;
  g_enabled = true;
}
void processCmdDISABLE(const char* line, int s, int len)
{
  (void)line;
  (void)s;
  (void)len;
  g_enabled = false;
  for (uint8_t L = 0; L < 6; ++L) {
    for (uint8_t J = 0; J < 3; ++J) setServoTorqueNow(L, J, false);
  }
}

void processCmdFK(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  } char legt[4] = {0};
  int nl = min(tl, 3);
  memcpy(legt, line + ts, nl);
  for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  } char vb[8] = {0};
  int nv = min(tl, 7);
  memcpy(vb, line + ts, nv);
  for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
  bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
  if (!strcmp(legt, "ALL")) {
    if (on) g_fk_stream_mask |= 0x3Fu;
    else g_fk_stream_mask &= ~0x3Fu;
    return;
  } int leg = legIndexFromToken(legt);
  if (leg < 0) {
    printERR(1, "BAD_LEG");
    return;
  } if (on) g_fk_stream_mask |= (uint8_t)(1u << leg);
  else g_fk_stream_mask &= (uint8_t)~(1u << leg);
}

void processCmdLEGS(const char* line, int s, int len) {
  (void)line;
  (void)s;
  (void)len;
  const char* names[6] = {"LF", "LM", "LR", "RF", "RM", "RR"};
  Serial.print(F("LEGS "));
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(F(" "));
    Serial.print(names[i]);
    Serial.print(F("="));
    Serial.print(leg_enabled_mask_get((uint8_t)i) ? 1 : 0);
  } Serial.print(F("\r\n"));
}

void processCmdSERVOS(const char* line, int s, int len) {
  (void)line;
  (void)s;
  (void)len;
  const char* names[6] = {"LF", "LM", "LR", "RF", "RM", "RR"};
  Serial.print(F("SERVOS "));
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(F(" "));
    Serial.print(names[i]);
    Serial.print(F("="));
    for (int j = 0; j < 3; ++j) Serial.print(joint_enabled_mask_get((uint8_t)i, (uint8_t)j) ? 1 : 0);
  } Serial.print(F("\r\n"));
}

// LEG <LEG|ALL> <ENABLE|DISABLE>
void processCmdLEG(const char* line, int s, int len)
{
  int ts, tl;
  // Parse leg token
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  // Parse action token
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);
  bool doEnable = !strcmp(act, "ENABLE");
  bool doDisable = !strcmp(act, "DISABLE");
  if (!doEnable && !doDisable) {
    printERR(1, "BAD_ARG");
    return;
  }

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
    if (leg < 0) {
      printERR(1, "BAD_LEG");
      return;
    }
    apply_leg(leg);
  }

  /* success */
}

// SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>
void processCmdSERVO(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);
  bool doEnable = !strcmp(act, "ENABLE");
  bool doDisable = !strcmp(act, "DISABLE");
  if (!doEnable && !doDisable) {
    printERR(1, "BAD_ARG");
    return;
  }

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
      int J = jointIndexFromToken(jtok); if (J < 0) {
        printERR(1, "BAD_JOINT");
        return;
      }
      for (int L = 0; L < 6; ++L) apply_servo(L, J);
    }
  } else {
    int leg = legIndexFromToken(legt); if (leg < 0) {
      printERR(1, "BAD_LEG");
      return;
    }
    if (!strcmp(jtok, "ALL")) {
      for (int J = 0; J < 3; ++J) apply_servo(leg, J);
    } else {
      int J = jointIndexFromToken(jtok); if (J < 0) {
        printERR(1, "BAD_JOINT");
        return;
      }
      apply_servo(leg, J);
    }
  }
  /* success */
}

// RAW <LEG> <JOINT> <centideg>
void processCmdRAW(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  int leg = legIndexFromToken(legt); if (leg < 0) {
    printERR(1, "BAD_LEG");
    return;
  }
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char jt[8] = {0}; int nj = min(tl, 7); memcpy(jt, line + ts, nj); for (int i = 0; i < nj; ++i) jt[i] = toupper(jt[i]);
  int joint = jointIndexFromToken(jt); if (joint < 0) {
    printERR(1, "BAD_JOINT");
    return;
  }
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long v = atol(nb); if (v < 0) v = 0; if (v > 24000) v = 24000;
  g_cmd_cd[leg][joint] = (int16_t)v;
  /* success */
}

// RAW3 <LEG> <c_cd> <f_cd> <t_cd>
void processCmdRAW3(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  int leg = legIndexFromToken(legt); if (leg < 0) {
    printERR(1, "BAD_LEG");
    return;
  }
  long vals[3];
  for (int j = 0; j < 3; ++j) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long v = atol(nb); if (v < 0) v = 0; if (v > 24000) v = 24000; vals[j] = v;
  }
  g_cmd_cd[leg][0] = (int16_t)vals[0];
  g_cmd_cd[leg][1] = (int16_t)vals[1];
  g_cmd_cd[leg][2] = (int16_t)vals[2];
  /* success */
}

// FOOT <LEG> <x_mm> <y_mm> <z_mm>
// Sets a single leg Cartesian target (body frame) by running IK and
// translating the resulting joint angles into centidegree servo commands.
// Left legs mirror the right leg IK solution about their calibrated home
// angles (legacy convention preserved).
void processCmdFOOT(const char* line, int s, int len)
{
  int ts, tl;

  // Parse leg token
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char legt[4] = {0};
  int nl = min(tl, 3);
  memcpy(legt, line + ts, nl);
  for (int i = 0; i < nl; ++i) legt[i] = (char)toupper(legt[i]);
  int leg = legIndexFromToken(legt);
  if (leg < 0) {
    printERR(1, "BAD_LEG");
    return;
  }

  // Parse 3 numeric Cartesian components: x y z (mm)
  float xyz[3];
  for (int j = 0; j < 3; ++j) {
    s = nextToken(line, s, len, &ts, &tl);
    if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char nb[24] = {0};
    int nn = min(tl, 23);
    memcpy(nb, line + ts, nn);
    nb[nn] = 0;
    xyz[j] = atof(nb);
  }

  // Inverse kinematics: returns joint centidegrees in out_cd[]
  int16_t out_cd[3];
  if (!calculateIK((uint8_t)leg, xyz[0], xyz[1], xyz[2], out_cd)) {
    printERR(1, "IK_FAIL");
    return;
  }

  Serial.print(F("FOOTDBG L=")); Serial.print(leg);
  Serial.print(F(" x=")); Serial.print(xyz[0], 2);
  Serial.print(F(" y=")); Serial.print(xyz[1], 2);
  Serial.print(F(" z=")); Serial.print(xyz[2], 2);
  Serial.print(F(" out_cd="));
  Serial.print(out_cd[0]); Serial.print('/');
  Serial.print(out_cd[1]); Serial.print('/');
  Serial.print(out_cd[2]);
  Serial.print(F("\r\n"));

  // Side-specific mapping:
  // Right legs (indices 3..5) use IK angles directly.
  // Left legs (0..2) mirror around their home calibration to maintain
  // consistent global pose conventions.
  bool isRight = (leg >= 3);
  if (isRight) {
    g_cmd_cd[leg][0] = out_cd[0];
    g_cmd_cd[leg][1] = out_cd[1];
    g_cmd_cd[leg][2] = out_cd[2];
  } else {
    g_cmd_cd[leg][0] = (int16_t)(2 * g_home_cd[leg][0] - out_cd[0]);
    g_cmd_cd[leg][1] = (int16_t)(2 * g_home_cd[leg][1] - out_cd[1]);
    g_cmd_cd[leg][2] = (int16_t)(2 * g_home_cd[leg][2] - out_cd[2]);
  }

  // (Collision / soft-limit checks are deferred to the main safety pass.)
  /* success */
}

void processCmdFEET(const char* line, int s, int len) {
  int ts, tl;
  float p[6][3];
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      s = nextToken(line, s, len, &ts, &tl);
      if (tl <= 0) {
        printERR(1, "BAD_ARG");
        return;
      } char nb[24] = {0};
      int nn = min(tl, 23);
      memcpy(nb, line + ts, nn);
      nb[nn] = 0;
      p[i][j] = atof(nb);
    }
  } for (int L = 0; L < 6; ++L) {
    int16_t out[3];
    if (!calculateIK((uint8_t)L, p[L][0], p[L][1], p[L][2], out)) {
      printERR(1, "IK_FAIL");
      return;
    } bool isRight = (L >= 3);
    if (isRight) {
      g_cmd_cd[L][0] = out[0];
      g_cmd_cd[L][1] = out[1];
      g_cmd_cd[L][2] = out[2];
    } else {
      g_cmd_cd[L][0] = (int16_t)(2 * g_home_cd[L][0] - out[0]);
      g_cmd_cd[L][1] = (int16_t)(2 * g_home_cd[L][1] - out[1]);
      g_cmd_cd[L][2] = (int16_t)(2 * g_home_cd[L][2] - out[2]);
    }
  } /* success */
}

void processCmdMODE(const char* line, int s, int len) {
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  } char mtok[8] = {0};
  int nm = min(tl, 7);
  memcpy(mtok, line + ts, nm);
  for (int i = 0; i < nm; ++i) mtok[i] = toupper(mtok[i]);
  if (!strcmp(mtok, "TEST")) {
    modeSetTest();
    return;
  } if (!strcmp(mtok, "IDLE")) {
    modeSetIdle();
    return;
  } printERR(1, "BAD_ARG");
}
void processCmdI(const char* line, int s, int len) {
  (void)line;
  (void)s;
  (void)len;
  modeSetIdle();
}
void processCmdT(const char* line, int s, int len) {
  (void)line;
  (void)s;
  (void)len;
  modeSetTest();
}

static void persistTrigaitIntKey(const char* key, long value)
{
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  char buf[24];
  snprintf(buf, sizeof(buf), "%ld", value);
  configSetKeyValue(key, buf);
#else
  (void)key; (void)value;
#endif
}

void processCmdTEST(const char* line, int s, int len)
{
  int ts, tl;
  // TEST <CYCLE|HEIGHT|BASEX|STEPLEN|LIFT|OVERLAP> <value>
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[12] = {0}; int ns = min(tl, 11); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char vb[24] = {0}; int nv = min(tl, 23); memcpy(vb, line + ts, nv); vb[nv] = 0;

  void processCmdIMP(const char* line, int s, int len);
  if (!strcmp(sub, "CYCLE")) {
    long ms = atol(vb); if (ms < 750) ms = 750; if (ms > 10000) ms = 10000;
    g_test_cycle_ms = (uint32_t)ms;
    persistTrigaitIntKey("test.trigait.cycle_ms", ms);
    return;
  }
  if (!strcmp(sub, "HEIGHT")) {
    g_test_base_y_mm = atof(vb);
    persistTrigaitIntKey("test.trigait.height_mm", lroundf(g_test_base_y_mm));
    return;
  }
  if (!strcmp(sub, "BASEX"))  {
    g_test_base_x_mm = atof(vb);
    persistTrigaitIntKey("test.trigait.basex_mm", lroundf(g_test_base_x_mm));
    return;
  }
  if (!strcmp(sub, "STEPLEN")) {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_step_len_mm = v;
    persistTrigaitIntKey("test.trigait.steplen_mm", lroundf(g_test_step_len_mm));
    return;
  }
  if (!strcmp(sub, "LIFT"))    {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_lift_y_mm = v;
    persistTrigaitIntKey("test.trigait.lift_mm", lroundf(g_test_lift_y_mm));
    return;
  }
  if (!strcmp(sub, "OVERLAP")) {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; if (v > 25.0f) v = 25.0f; g_test_overlap_pct = v;
    persistTrigaitIntKey("test.trigait.overlap_pct", lroundf(g_test_overlap_pct));
    return;
  }

  printERR(1, "BAD_ARG");
}

void processCmdSTAND(const char* line, int s, int len)
{
  (void)line; (void)s; (void)len;
  // STAND should always be executed from a safe, non-test state.
  // Force MODE IDLE semantics before updating foot targets / IK.
  modeSetIdle();

  // NOTE: Historically STAND reused the TEST gait base parameters (g_test_base_*).
  // If a user tunes TEST width/height aggressively, that can make the (x=BASE_X,z=0)
  // stance unreachable and STAND would fail with IK_FAIL. Clamp to a reachable
  // workspace and fall back to a conservative default if needed.
  float base_y = g_test_base_y_mm;
  float base_x = g_test_base_x_mm;

  // Enforce the sign convention used by our IK: negative y is down.
  if (base_y > 0.0f) base_y = -base_y;
  if (base_y > -1.0f) base_y = -120.0f;

  // Ensure y is within the 2-link reach envelope.
  const float reach = (FEMUR_LENGTH + TIBIA_LENGTH) - 1.0f; // 1mm margin
  if (fabsf(base_y) > reach) base_y = -reach;

  // For z=0, clamp x so D <= (a+b). Derived from:
  //   D^2 = (max(0, x-COXA))^2 + y^2 <= reach^2
  float max_x = COXA_LENGTH;
  float under = (reach * reach) - (base_y * base_y);
  if (under > 0.0f) max_x += sqrtf(under);
  if (base_x > max_x) base_x = max_x;
  if (base_x < 0.0f) base_x = 0.0f;

  bool any_fail = false;
  for (int L = 0; L < 6; ++L) {
    int16_t out[3];
    bool ok = calculateIK((uint8_t)L, base_x, base_y, 0.0f, out);
    if (!ok) { any_fail = true; break; }
    bool isRight = (L >= 3);
    if (isRight) {
      g_cmd_cd[L][0] = out[0];
      g_cmd_cd[L][1] = out[1];
      g_cmd_cd[L][2] = out[2];
    } else {
      g_cmd_cd[L][0] = (int16_t)(2 * g_home_cd[L][0] - out[0]);
      g_cmd_cd[L][1] = (int16_t)(2 * g_home_cd[L][1] - out[1]);
      g_cmd_cd[L][2] = (int16_t)(2 * g_home_cd[L][2] - out[2]);
    }
    g_foot_target_x_mm[L] = base_x;
    g_foot_target_z_mm[L] = 0.0f;
  }

  if (any_fail) {
    // Conservative fallback stance (known-good across typical geometry).
    base_x = 130.0f;
    base_y = -120.0f;
    for (int L = 0; L < 6; ++L) {
      int16_t out[3];
      bool ok = calculateIK((uint8_t)L, base_x, base_y, 0.0f, out);
      if (!ok) {
        printERR(1, "IK_FAIL");
        return;
      }
      bool isRight = (L >= 3);
      if (isRight) {
        g_cmd_cd[L][0] = out[0];
        g_cmd_cd[L][1] = out[1];
        g_cmd_cd[L][2] = out[2];
      } else {
        g_cmd_cd[L][0] = (int16_t)(2 * g_home_cd[L][0] - out[0]);
        g_cmd_cd[L][1] = (int16_t)(2 * g_home_cd[L][1] - out[1]);
        g_cmd_cd[L][2] = (int16_t)(2 * g_home_cd[L][2] - out[2]);
      }
      g_foot_target_x_mm[L] = base_x;
      g_foot_target_z_mm[L] = 0.0f;
    }
  }
}

// PID command — runtime configuration and persistence
// Syntax:
//   PID LIST
//   PID ENABLE | PID DISABLE
//   PID MODE <ACTIVE|SHADOW>
//   PID KP|KI|KD <COXA|FEMUR|TIBIA|ALL> <milli>
//   PID KDALPHA <COXA|FEMUR|TIBIA|ALL> <milli 0..1000>
//   PID SHADOW_RATE <hz 1..50>
static void pid_print_list() {
  Serial.print(F("PID "));
  Serial.print(F("enabled=")); Serial.print(g_pid_enabled ? 1 : 0);
  Serial.print(F(" mode=")); Serial.print(g_pid_mode == 0 ? F("active") : F("shadow"));
  Serial.print(F(" kp=")); Serial.print((unsigned)g_pid_kp_milli[0]); Serial.print('/'); Serial.print((unsigned)g_pid_kp_milli[1]); Serial.print('/'); Serial.print((unsigned)g_pid_kp_milli[2]);
  Serial.print(F(" ki=")); Serial.print((unsigned)g_pid_ki_milli[0]); Serial.print('/'); Serial.print((unsigned)g_pid_ki_milli[1]); Serial.print('/'); Serial.print((unsigned)g_pid_ki_milli[2]);
  Serial.print(F(" kd=")); Serial.print((unsigned)g_pid_kd_milli[0]); Serial.print('/'); Serial.print((unsigned)g_pid_kd_milli[1]); Serial.print('/'); Serial.print((unsigned)g_pid_kd_milli[2]);
  Serial.print(F(" kdalph=")); Serial.print((unsigned)g_pid_kd_alpha_milli[0]); Serial.print('/'); Serial.print((unsigned)g_pid_kd_alpha_milli[1]); Serial.print('/'); Serial.print((unsigned)g_pid_kd_alpha_milli[2]);
  Serial.print(F(" shadow_hz=")); Serial.print((unsigned)g_pid_shadow_report_hz);
  Serial.print(F("\r\n"));
}

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
static bool config_update_single_key(const char* key, const String& value) {
  if (!SD.begin(BUILTIN_SDCARD)) return false;
  File fin = SD.open("/config.txt", FILE_READ);
  File fout = SD.open("/config.tmp", FILE_WRITE);
  if (!fout) {
    if (fin) fin.close();
    return false;
  }
  bool replaced = false;
  if (fin) {
    static char linebuf[256]; int llen = 0;
    while (fin.available()) {
      int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
      if (ch2 == '\n') {
        linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
        size_t klen = strlen(key);
        if (strncmp(p, key, klen) == 0) {
          fout.print(key); fout.print(value); fout.print('\n');
          replaced = true;
        } else {
          fout.print(linebuf);
          fout.print('\n');
        }
        llen = 0;
      } else if (llen + 1 < (int)sizeof(linebuf)) {
        linebuf[llen++] = (char)ch2;
      }
      else {
        linebuf[llen] = 0;
        fout.print(linebuf);
        fout.print('\n');
        llen = 0;
      }
    }
    fin.close();
  }
  if (!replaced) {
    fout.print(key);
    fout.print(value);
    fout.print('\n');
  }
  fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  return true;
}
#endif

void processCmdPID(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "LIST")) {
    pid_print_list();
    return;
  }
  if (!strcmp(sub, "ENABLE")) {
    g_pid_enabled = true;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("pid.enabled=", String(F("true")));
#endif
    return;
  }
  if (!strcmp(sub, "DISABLE")) {
    g_pid_enabled = false;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("pid.enabled=", String(F("false")));
#endif
    return;
  }

  if (!strcmp(sub, "MODE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char mt[12] = {0}; int nm = min(tl, 11); memcpy(mt, line + ts, nm); for (int i = 0; i < nm; ++i) mt[i] = toupper(mt[i]);
    if (!strcmp(mt, "ACTIVE")) g_pid_mode = 0; else if (!strcmp(mt, "SHADOW")) g_pid_mode = 1; else {
      printERR(1, "BAD_ARG");
      return;
    }
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("pid.mode=", (g_pid_mode == 0) ? String(F("active")) : String(F("shadow")));
#endif
    return;
  }

  if (!strcmp(sub, "KP") || !strcmp(sub, "KI") || !strcmp(sub, "KD") || !strcmp(sub, "KDALPHA")) {
    bool isAlpha = !strcmp(sub, "KDALPHA"); bool isKP = !strcmp(sub, "KP"), isKI = !strcmp(sub, "KI"), isKD = !strcmp(sub, "KD");
    // joint token
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);
    // value token
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (isAlpha && v > 1000) v = 1000; if (!isAlpha && v > 65535) v = 65535;

    auto apply_one = [&](int J) {
      if (J < 0 || J > 2) return;
      if (isKP) g_pid_kp_milli[J] = (uint16_t)v;
      else if (isKI) g_pid_ki_milli[J] = (uint16_t)v;
      else if (isKD) g_pid_kd_milli[J] = (uint16_t)v;
      else g_pid_kd_alpha_milli[J] = (uint16_t)v;
    };
    auto persist_one = [&](int J) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      const char* names[3] = {"coxa", "femur", "tibia"};
      String key;
      if (isKP) key = String(F("pid.kp_milli.")) + names[J] + "=";
      else if (isKI) key = String(F("pid.ki_milli.")) + names[J] + "=";
      else if (isKD) key = String(F("pid.kd_milli.")) + names[J] + "=";
      else key = String(F("pid.kd_alpha_milli.")) + names[J] + "=";
      (void)config_update_single_key(key.c_str(), String((unsigned long)v));
#endif
    };

    if (!strcmp(jtok, "ALL")) {
      for (int J = 0; J < 3; ++J) {
        apply_one(J);
        persist_one(J);
      }
    }
    else {
      int J = jointIndexFromToken(jtok);
      if (J < 0) {
        printERR(1, "BAD_JOINT");
        return;
      } apply_one(J);
      persist_one(J);
    }
    return;
  }

  if (!strcmp(sub, "SHADOW_RATE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[12] = {0}; int nv = min(tl, 11); memcpy(vb, line + ts, nv); vb[nv] = 0; long hz = atol(vb);
    if (hz < 1) hz = 1; if (hz > 50) hz = 50; g_pid_shadow_report_hz = (uint16_t)hz;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("pid.shadow_report_hz=", String((unsigned long)g_pid_shadow_report_hz));
#endif
    return;
  }

  printERR(1, "BAD_ARG");
}

// LIMITS command — shared joint workspace tuning
// Syntax:
//   LIMITS LIST
//   LIMITS <COXA|FEMUR|TIBIA> <MIN_DEG> <MAX_DEG>
//
// Behavior:
//   - Limits are applied identically to all legs for the selected joint type.
//   - Values are interpreted in degrees and converted to absolute centidegrees
//     around each leg's g_home_cd[leg][joint].
//   - On change, corresponding joint_limits.* keys in /config.txt are updated
//     for all legs.
void processCmdLIMITS(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { // LIST
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "LIST")) {
    const char* jnames[3] = {"COXA", "FEMUR", "TIBIA"};
    Serial.print(F("LIMITS "));
    for (int J = 0; J < 3; ++J) {
      if (J) Serial.print(F(" "));
      long min_cd = g_limit_min_cd[0][J];
      long max_cd = g_limit_max_cd[0][J];
      Serial.print(jnames[J]);
      Serial.print('=');
      Serial.print(min_cd);
      Serial.print('/');
      Serial.print(max_cd);
    }
    Serial.print(F("\r\n"));
    return;
  }

  // Joint token already consumed into sub
  int J = jointIndexFromToken(sub);
  if (J < 0 || J > 2) {
    printERR(1, "BAD_JOINT");
    return;
  }

  // MIN_CD
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char vb1[16] = {0}; int n1 = min(tl, 15); memcpy(vb1, line + ts, n1); vb1[n1] = 0;
  // MAX_CD
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char vb2[16] = {0}; int n2 = min(tl, 15); memcpy(vb2, line + ts, n2); vb2[n2] = 0;

  long min_cd = atol(vb1);
  long max_cd = atol(vb2);
  if (min_cd > max_cd) {
    long tmp = min_cd; min_cd = max_cd; max_cd = tmp;
  }
  if (min_cd < 0) min_cd = 0;
  if (max_cd > 24000) max_cd = 24000;

  for (int L = 0; L < 6; ++L) {
    g_limit_min_cd[L][J] = (int16_t)min_cd;
    g_limit_max_cd[L][J] = (int16_t)max_cd;
  }

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // Persist shared centidegree limits for all legs into /config.txt (stored as degrees)
  const char* legNames[6] = {"LF", "LM", "LR", "RF", "RM", "RR"};
  const char* jointNames[3] = {"coxa", "femur", "tibia"};

  // Convert centidegrees to degrees once for persistence
  float min_deg = (float)min_cd * 0.01f;
  float max_deg = (float)max_cd * 0.01f;

  for (int L = 0; L < 6; ++L) {
    const char* leg = legNames[L];

    char keyMin[64];
    char keyMax[64];

    snprintf(keyMin, sizeof(keyMin), "joint_limits.%s.%s.min_deg=", leg, jointNames[J]);
    snprintf(keyMax, sizeof(keyMax), "joint_limits.%s.%s.max_deg=", leg, jointNames[J]);

    (void)config_update_single_key(keyMin, String(min_deg, 3));
    (void)config_update_single_key(keyMax, String(max_deg, 3));
  }
#endif

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // Persist shared degree limits for all legs into /config.txt
  //const char* legNames[6] = {"LF","LM","LR","RF","RM","RR"};
  //const char* jointNames[3] = {"coxa","femur","tibia"};

  for (int L = 0; L < 6; ++L) {
    const char* leg = legNames[L];

    char keyMin[64];
    char keyMax[64];

    snprintf(keyMin, sizeof(keyMin), "joint_limits.%s.%s.min_deg=", leg, jointNames[J]);
    snprintf(keyMax, sizeof(keyMax), "joint_limits.%s.%s.max_deg=", leg, jointNames[J]);

    (void)config_update_single_key(keyMin, String(min_deg, 3));
    (void)config_update_single_key(keyMax, String(max_deg, 3));
  }
#endif
}

// IMP command — runtime impedance configuration and (where SD present) persistence
// Syntax:
//   IMP LIST
//   IMP ENABLE | DISABLE
//   IMP MODE <OFF|JOINT|CART>
//   IMP SCALE <milli>
//   IMP JSPRING|JDAMP <COXA|FEMUR|TIBIA|ALL> <milli>
//   IMP CSPRING|CDAMP <X|Y|Z|ALL> <milli>
static void imp_print_list() {
  const ImpedanceConfig& cfg = g_impedance.config();
  Serial.print(F("IMP "));
  Serial.print(F("enabled=")); Serial.print(cfg.enabled ? 1 : 0);
  Serial.print(F(" mode="));
  switch (cfg.mode) {
    case IMP_MODE_JOINT: Serial.print(F("joint")); break;
    case IMP_MODE_CART:  Serial.print(F("cart"));  break;
    default:             Serial.print(F("off"));   break;
  }
  Serial.print(F(" jspring="));
  Serial.print((unsigned)cfg.joint.k_spring_milli[0]); Serial.print('/');
  Serial.print((unsigned)cfg.joint.k_spring_milli[1]); Serial.print('/');
  Serial.print((unsigned)cfg.joint.k_spring_milli[2]);
  Serial.print(F(" jdamp="));
  Serial.print((unsigned)cfg.joint.k_damp_milli[0]);   Serial.print('/');
  Serial.print((unsigned)cfg.joint.k_damp_milli[1]);   Serial.print('/');
  Serial.print((unsigned)cfg.joint.k_damp_milli[2]);
  Serial.print(F(" cspring="));
  Serial.print((unsigned)cfg.cart.k_spring_milli[0]); Serial.print('/');
  Serial.print((unsigned)cfg.cart.k_spring_milli[1]); Serial.print('/');
  Serial.print((unsigned)cfg.cart.k_spring_milli[2]);
  Serial.print(F(" cdamp="));
  Serial.print((unsigned)cfg.cart.k_damp_milli[0]);   Serial.print('/');
  Serial.print((unsigned)cfg.cart.k_damp_milli[1]);   Serial.print('/');
  Serial.print((unsigned)cfg.cart.k_damp_milli[2]);
  Serial.print(F(" scale="));
  Serial.print((unsigned)cfg.output_scale_milli);
  Serial.print(F(" jdb_cd="));
  Serial.print((unsigned)cfg.joint_deadband_cd);
  Serial.print(F(" cdb_mm="));
  Serial.print(cfg.cart_deadband_mm, 2);
  Serial.print(F("\r\n"));
}

// COMP command — joint compliance configuration (setpoint adaptation)
// Syntax:
//   COMP LIST
//   COMP ENABLE | DISABLE
//   COMP KP <COXA|FEMUR|TIBIA|ALL> <milli>
//   COMP RANGE <COXA|FEMUR|TIBIA|ALL> <cd>
//   COMP DEADBAND <COXA|FEMUR|TIBIA|ALL> <cd>
//   COMP LEAK <milli>
// (Compliance globals declared in globals.h)

static void comp_print_list() {
  Serial.print(F("COMP enabled=")); Serial.print(g_comp_enabled ? 1 : 0);
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

void processCmdCOMP(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "LIST")) {
    comp_print_list();
    return;
  }

  if (!strcmp(sub, "ENABLE")) {
    g_comp_enabled = true;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("comp.enabled=", String(F("true")));
#endif
    return;
  }
  if (!strcmp(sub, "DISABLE")) {
    g_comp_enabled = false;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("comp.enabled=", String(F("false")));
#endif
    return;
  }

  if (!strcmp(sub, "LEAK")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[12] = {0}; int nv = min(tl, 11); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (v > 1000) v = 1000; // 0..1.0 per tick
    g_comp_leak_milli = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("comp.leak_milli=", String((unsigned long)v));
#endif
    return;
  }

  if (!strcmp(sub, "KP") || !strcmp(sub, "RANGE") || !strcmp(sub, "DEADBAND")) {
    bool isKP = !strcmp(sub, "KP");
    bool isRange = !strcmp(sub, "RANGE");
    bool isDB = !strcmp(sub, "DEADBAND");

    // joint token
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); jtok[nj] = 0; for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);

    // value token
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);

    if (isKP) {
      if (v < 0) v = 0; if (v > 65535) v = 65535;
    } else if (isRange) {
      if (v < 0) v = 0; if (v > 5000) v = 5000; // up to 50 deg
    } else if (isDB) {
      if (v < 0) v = 0; if (v > 2000) v = 2000; // up to 20 deg
    }

    auto apply_one = [&](int J) {
      if (J < 0 || J > 2) return;
      if (isKP)      g_comp_kp_milli[J]    = (uint16_t)v;
      else if (isRange)   g_comp_range_cd[J]    = (uint16_t)v;
      else if (isDB)      g_comp_deadband_cd[J] = (uint16_t)v;
    };
    auto persist_one = [&](int J) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      const char* names[3] = {"coxa", "femur", "tibia"};
      String key;
      if (isKP)      key = String(F("comp.kp_milli."))      + names[J] + "=";
      else if (isRange) key = String(F("comp.range_cd."))      + names[J] + "=";
      else if (isDB)    key = String(F("comp.deadband_cd."))  + names[J] + "=";
      (void)config_update_single_key(key.c_str(), String((unsigned long)v));
#else
      (void)J;
#endif
    };

    if (!strcmp(jtok, "ALL")) {
      for (int J = 0; J < 3; ++J) {
        apply_one(J);
        persist_one(J);
      }
    } else {
      int J = jointIndexFromToken(jtok); if (J < 0) {
        printERR(1, "BAD_JOINT");
        return;
      }
      apply_one(J); persist_one(J);
    }
    return;
  }

  printERR(1, "BAD_ARG");
}

void processCmdIMP(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[12] = {0}; int ns = min(tl, 11); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "LIST")) {
    imp_print_list();
    return;
  }

  if (!strcmp(sub, "ENABLE")) {
    g_impedance.config().enabled = true;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("imp.enabled=", String(F("true")));
#endif
    return;
  }
  if (!strcmp(sub, "DISABLE")) {
    g_impedance.config().enabled = false;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("imp.enabled=", String(F("false")));
#endif
    return;
  }

  if (!strcmp(sub, "MODE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char mt[8] = {0}; int nm = min(tl, 7); memcpy(mt, line + ts, nm); for (int i = 0; i < nm; ++i) mt[i] = toupper(mt[i]);
    if (!strcmp(mt, "JOINT")) g_impedance.config().mode = IMP_MODE_JOINT;
    else if (!strcmp(mt, "CART")) g_impedance.config().mode = IMP_MODE_CART;
    else if (!strcmp(mt, "OFF"))  g_impedance.config().mode = IMP_MODE_OFF;
    else {
      printERR(1, "BAD_ARG");
      return;
    }
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    const char* val = (g_impedance.config().mode == IMP_MODE_JOINT) ? "joint" : (g_impedance.config().mode == IMP_MODE_CART ? "cart" : "off");
    (void)config_update_single_key("imp.mode=", String(val));
#endif
    return;
  }

  if (!strcmp(sub, "SCALE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[12] = {0}; int nv = min(tl, 11); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (v > 1000) v = 1000; // 0..1000
    g_impedance.config().output_scale_milli = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("imp.output_scale_milli=", String((unsigned long)v));
#endif
    return;
  }

  if (!strcmp(sub, "JDB")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[12] = {0}; int nv = min(tl, 11); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (v > 5000) v = 5000; // 0..50 deg
    g_impedance.config().joint_deadband_cd = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("imp.joint_deadband_cd=", String((unsigned long)v));
#endif
    return;
  }

  if (!strcmp(sub, "CDB")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; float v = atof(vb);
    if (v < 0.0f) v = 0.0f; if (v > 100.0f) v = 100.0f;
    g_impedance.config().cart_deadband_mm = v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    // Store with limited precision; sprintf via String is fine here.
    (void)config_update_single_key("imp.cart_deadband_mm=", String(v, 3));
#endif
    return;
  }

  if (!strcmp(sub, "JSPRING") || !strcmp(sub, "JDAMP")) {
    bool isSpring = !strcmp(sub, "JSPRING");
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); jtok[nj] = 0; for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (v > 65535) v = 65535;

    auto apply_one = [&](int J) {
      if (J < 0 || J > 2) return;
      if (isSpring) g_impedance.config().joint.k_spring_milli[J] = (uint16_t)v;
      else g_impedance.config().joint.k_damp_milli[J] = (uint16_t)v;
    };
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    auto persist_one = [&](int J) {
      const char* names[3] = {"coxa", "femur", "tibia"}; String key;
      if (isSpring) key = String(F("imp.joint.k_spring_milli.")) + names[J] + "=";
      else          key = String(F("imp.joint.k_damp_milli."))   + names[J] + "=";
      (void)config_update_single_key(key.c_str(), String((unsigned long)v));
    };
#endif

    if (!strcmp(jtok, "ALL")) {
      for (int J = 0; J < 3; ++J) {
        apply_one(J);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
        persist_one(J);
#endif
      }
    } else {
      int J = jointIndexFromToken(jtok); if (J < 0) {
        printERR(1, "BAD_JOINT");
        return;
      }
      apply_one(J);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      persist_one(J);
#endif
    }
    return;
  }

  if (!strcmp(sub, "CSPRING") || !strcmp(sub, "CDAMP")) {
    bool isSpring = !strcmp(sub, "CSPRING");
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char atok[4] = {0}; int na = min(tl, 3); memcpy(atok, line + ts, na); atok[na] = 0; for (int i = 0; i < na; ++i) atok[i] = toupper(atok[i]);
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);
    if (v < 0) v = 0; if (v > 65535) v = 65535;

    auto apply_axis = [&](int A) {
      if (A < 0 || A > 2) return;
      if (isSpring) g_impedance.config().cart.k_spring_milli[A] = (uint16_t)v;
      else          g_impedance.config().cart.k_damp_milli[A] = (uint16_t)v;
    };
    auto persist_axis = [&](int A) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      const char* names[3] = {"x", "y", "z"}; String key;
      if (isSpring) key = String(F("imp.cart.k_spring_milli.")) + names[A] + "=";
      else          key = String(F("imp.cart.k_damp_milli."))   + names[A] + "=";
      (void)config_update_single_key(key.c_str(), String((unsigned long)v));
#else
      (void)A; // unused
#endif
    };

    if (!strcmp(atok, "ALL")) {
      for (int A = 0; A < 3; ++A) {
        apply_axis(A);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
        persist_axis(A);
#endif
      }
    } else if (!strcmp(atok, "X")) {
      apply_axis(0);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      persist_axis(0);
#endif
    } else if (!strcmp(atok, "Y")) {
      apply_axis(1);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      persist_axis(1);
#endif
    } else if (!strcmp(atok, "Z")) {
      apply_axis(2);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      persist_axis(2);
#endif
    } else {
      printERR(1, "BAD_AXIS"); return;
    }
    return;
  }

  printERR(1, "BAD_ARG");
}

void processCmdSAFETY(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "CLEAR")) {
    if (!g_lockout) {
      printERR(91, "NOT_LOCKED");
      return;
    }
    if ((g_lockout_causes & ~g_override_mask) != 0) {
      printERR(90, "VIOLATION");
      return;
    }
    g_lockout = false; g_lockout_causes = 0; return;
  }

  if (!strcmp(sub, "LIST")) {
    Serial.print(F("SAFETY "));
    Serial.print(F("lockout=")); Serial.print(g_lockout ? 1 : 0);
    Serial.print(F(" cause=0x")); Serial.print((unsigned int)g_lockout_causes, HEX);
    Serial.print(F(" override=0x")); Serial.print((unsigned int)g_override_mask, HEX);
    Serial.print(F(" clearance_mm=")); Serial.print((int)lroundf(g_safety_clearance_mm));
    Serial.print(F(" soft_limits=")); Serial.print(g_safety_soft_limits_enabled ? 1 : 0);
    Serial.print(F(" collision=")); Serial.print(g_safety_collision_enabled ? 1 : 0);
    Serial.print(F(" temp_C=")); Serial.print((int)(g_safety_temp_lockout_c10 / 10));
    Serial.print(F("\r\n"));
    return;
  }

  if (!strcmp(sub, "OVERRIDE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char which[16] = {0}; int nw = min(tl, 15); memcpy(which, line + ts, nw); for (int i = 0; i < nw; ++i) which[i] = toupper(which[i]);
    if      (!strcmp(which, "ALL"))       {
      g_override_mask = 0xFFFFu;
    }
    else if (!strcmp(which, "NONE"))      {
      g_override_mask = 0u;
    }
    else if (!strcmp(which, "TEMP"))      {
      g_override_mask |= (1u << 0);
    }
    else if (!strcmp(which, "COLLISION")) {
      g_override_mask |= (1u << 1);
    }
    else {
      printERR(1, "BAD_ARG");
      return;
    }
    if (g_lockout && ((g_lockout_causes & ~g_override_mask) == 0)) g_lockout = false;
    return;
  }

  if (!strcmp(sub, "SOFTLIMITS")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_safety_soft_limits_enabled = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      printERR(2, "NO_SD");
      return;
    }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) {
      if (fin) fin.close();
      printERR(2, "SD_WRITE_FAIL");
      return;
    }
    bool replaced = false;
    if (fin) {
      static char linebuf[256]; int llen = 0;
      while (fin.available()) {
        int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
        if (ch2 == '\n') {
          linebuf[llen] = 0;
          const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
          const char* key = "safety.soft_limits="; size_t klen = strlen(key);
          if (strncmp(p, key, klen) == 0) {
            fout.print(key); fout.print(g_safety_soft_limits_enabled ? F("true") : F("false")); fout.print('\n');
            replaced = true;
          } else {
            fout.print(linebuf);
            fout.print('\n');
          }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) {
          linebuf[llen++] = (char)ch2;
        }
        else {
          linebuf[llen] = 0;
          fout.print(linebuf);
          fout.print('\n');
          llen = 0;
        }
      }
      fin.close();
    }
    if (!replaced) {
      fout.print("safety.soft_limits=");
      fout.print(g_safety_soft_limits_enabled ? F("true") : F("false"));
      fout.print('\n');
    }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "COLLISION")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_safety_collision_enabled = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      printERR(2, "NO_SD");
      return;
    }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) {
      if (fin) fin.close();
      printERR(2, "SD_WRITE_FAIL");
      return;
    }
    bool replaced = false;
    if (fin) {
      static char linebuf[256]; int llen = 0;
      while (fin.available()) {
        int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
        if (ch2 == '\n') {
          linebuf[llen] = 0;
          const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
          const char* key = "safety.collision="; size_t klen = strlen(key);
          if (strncmp(p, key, klen) == 0) {
            fout.print(key); fout.print(g_safety_collision_enabled ? F("true") : F("false")); fout.print('\n');
            replaced = true;
          } else {
            fout.print(linebuf);
            fout.print('\n');
          }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) {
          linebuf[llen++] = (char)ch2;
        }
        else {
          linebuf[llen] = 0;
          fout.print(linebuf);
          fout.print('\n');
          llen = 0;
        }
      }
      fin.close();
    }
    if (!replaced) {
      fout.print("safety.collision=");
      fout.print(g_safety_collision_enabled ? F("true") : F("false"));
      fout.print('\n');
    }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "TEMPLOCK")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0;
    float c = atof(vb); if (c < 30.0f) c = 30.0f; if (c > 120.0f) c = 120.0f;
    int v = (int)lroundf(c * 10.0f); if (v < 0) v = 0; if (v > 32767) v = 32767;
    g_safety_temp_lockout_c10 = (int16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      printERR(2, "NO_SD");
      return;
    }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) {
      if (fin) fin.close();
      printERR(2, "SD_WRITE_FAIL");
      return;
    }
    bool replaced = false;
    if (fin) {
      static char linebuf[256]; int llen = 0;
      while (fin.available()) {
        int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
        if (ch2 == '\n') {
          linebuf[llen] = 0;
          const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
          const char* key = "safety.temp_lockout_c="; size_t klen = strlen(key);
          if (strncmp(p, key, klen) == 0) {
            fout.print(key); fout.print((int)(g_safety_temp_lockout_c10 / 10)); fout.print('\n');
            replaced = true;
          } else {
            fout.print(linebuf);
            fout.print('\n');
          }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) {
          linebuf[llen++] = (char)ch2;
        }
        else {
          linebuf[llen] = 0;
          fout.print(linebuf);
          fout.print('\n');
          llen = 0;
        }
      }
      fin.close();
    }
    if (!replaced) {
      fout.print("safety.temp_lockout_c=");
      fout.print((int)(g_safety_temp_lockout_c10 / 10));
      fout.print('\n');
    }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "CLEARANCE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[24] = {0}; int nv = min(tl, 23); memcpy(vb, line + ts, nv); vb[nv] = 0;
    float v = atof(vb); if (v < 0.0f) v = 0.0f; if (v > 500.0f) v = 500.0f;
    g_safety_clearance_mm = v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      printERR(2, "NO_SD");
      return;
    }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) {
      if (fin) fin.close();
      printERR(2, "SD_WRITE_FAIL");
      return;
    }
    bool replaced = false;
    if (fin) {
      static char linebuf[256]; int llen = 0;
      while (fin.available()) {
        int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
        if (ch2 == '\n') {
          linebuf[llen] = 0;
          const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
          const char* key = "safety.clearance_mm="; size_t klen = strlen(key);
          if (strncmp(p, key, klen) == 0) {
            fout.print(key); long iv = lroundf(g_safety_clearance_mm); fout.print((int)iv); fout.print('\n');
            replaced = true;
          } else {
            fout.print(linebuf);
            fout.print('\n');
          }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) {
          linebuf[llen++] = (char)ch2;
        }
        else {
          linebuf[llen] = 0;
          fout.print(linebuf);
          fout.print('\n');
          llen = 0;
        }
      }
      fin.close();
    }
    if (!replaced) {
      fout.print("safety.clearance_mm=");
      long iv = lroundf(g_safety_clearance_mm);
      fout.print((int)iv);
      fout.print('\n');
    }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  printERR(1, "BAD_ARG");
}

// TUCK <SET|CLEAR|PARAMS> [LEG]
//   SET            – arm the tuck sequence for one leg or all enabled legs
//   CLEAR          – cancel any active tuck sequence
//   SET <PARAM> <VAL> – update tuck.* params (TIBIA|FEMUR|COXA|TOL_TIBIA|TOL_OTHER|TIMEOUT) and persist to /config.txt
//   PARAMS         – print current tuck.* parameter values
void processCmdTUCK(const char* line, int s, int len)
{
  // (TUCK globals declared in globals.h)

  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);

  // If no subcommand is provided, treat plain "TUCK" as "TUCK SET" for all enabled legs.
  char sub[8] = {0};
  if (tl <= 0) {
    strcpy(sub, "SET");
  } else {
    int ns = min(tl, 7);
    memcpy(sub, line + ts, ns);
    for (int i = 0; i < ns; ++i) sub[i] = (char)toupper(sub[i]);
  }

  // TUCK CLEAR: cancel tuck state
  if (!strcmp(sub, "CLEAR")) {
    g_tuck_active     = 0;
    g_tuck_mask       = 0;
    g_tuck_done_mask  = 0;
    g_tuck_start_ms   = 0;
    return;
  }

  // TUCK PARAMS: print current tuck parameter values
  if (!strcmp(sub, "PARAMS")) {
    Serial.print(F("tuck.tibia_cd="));      Serial.println(g_tuck_tibia_cd);
    Serial.print(F("tuck.femur_cd="));      Serial.println(g_tuck_femur_cd);
    Serial.print(F("tuck.coxa_cd="));       Serial.println(g_tuck_coxa_cd);
    Serial.print(F("tuck.tol_tibia_cd="));  Serial.println(g_tuck_tol_tibia_cd);
    Serial.print(F("tuck.tol_other_cd="));  Serial.println(g_tuck_tol_other_cd);
    Serial.print(F("tuck.timeout_ms="));    Serial.println(g_tuck_timeout_ms);
    return;
  }

  // TUCK SET [LEG]   or   TUCK SET <PARAM> <VAL>
  if (!strcmp(sub, "SET")) {
    // Peek next token to decide if this is a LEG or PARAM form
    int ts2, tl2;
    int s_after_sub = nextToken(line, s, len, &ts2, &tl2);

    // LEG form: TUCK SET [LEG]
    bool isLegForm = false;
    if (tl2 <= 0) {
      isLegForm = true;  // no extra token -> all enabled legs
    } else {
      char tbuf[8] = {0};
      int n = min(tl2, 7);
      memcpy(tbuf, line + ts2, n);
      for (int i = 0; i < n; ++i) tbuf[i] = (char)toupper(tbuf[i]);
      // Recognize LEG tokens (LF/LM/LR/RF/RM/RR/ALL) by mapping via legIndexFromToken
      int leg_test = legIndexFromToken(tbuf);
      if (leg_test >= 0 || !strcmp(tbuf, "ALL")) {
        isLegForm = true;
      }
    }

    if (!isLegForm) {
      // PARAM form: TUCK SET <PARAM> <VAL>
      if (tl2 <= 0) {
        printERR(1, "BAD_ARG");
        return;
      }

      char pbuf[16] = {0};
      int pn = min(tl2, 15);
      memcpy(pbuf, line + ts2, pn);
      for (int i = 0; i < pn; ++i) pbuf[i] = (char)toupper(pbuf[i]);

      // Read value token
      int ts3, tl3;
      nextToken(line, s_after_sub, len, &ts3, &tl3);
      if (tl3 <= 0) {
        printERR(1, "BAD_ARG");
        return;
      }
      char vbuf[16] = {0};
      int vn = min(tl3, 15);
      memcpy(vbuf, line + ts3, vn);
      vbuf[vn] = '\0';
      long v = atol(vbuf);

      // Clamp and apply based on PARAM
      if (!strcmp(pbuf, "TIBIA")) {
        if (v < 0) v = 0; if (v > 24000) v = 24000;
        g_tuck_tibia_cd = (int16_t)v;
        configSetKeyValue("tuck.tibia_cd", vbuf);
      } else if (!strcmp(pbuf, "FEMUR")) {
        if (v < 0) v = 0; if (v > 24000) v = 24000;
        g_tuck_femur_cd = (int16_t)v;
        configSetKeyValue("tuck.femur_cd", vbuf);
      } else if (!strcmp(pbuf, "COXA")) {
        // coxa is logical center; force 12000
        v = 12000; strcpy(vbuf, "12000");
        g_tuck_coxa_cd = (int16_t)v;
        configSetKeyValue("tuck.coxa_cd", vbuf);
      } else if (!strcmp(pbuf, "TOL_TIBIA")) {
        if (v < 10) v = 10; if (v > 5000) v = 5000;
        g_tuck_tol_tibia_cd = (int16_t)v;
        configSetKeyValue("tuck.tol_tibia_cd", vbuf);
      } else if (!strcmp(pbuf, "TOL_OTHER")) {
        if (v < 10) v = 10; if (v > 5000) v = 5000;
        g_tuck_tol_other_cd = (int16_t)v;
        configSetKeyValue("tuck.tol_other_cd", vbuf);
      } else if (!strcmp(pbuf, "TIMEOUT")) {
        if (v < 250) v = 250; if (v > 10000) v = 10000;
        g_tuck_timeout_ms = (uint16_t)v;
        configSetKeyValue("tuck.timeout_ms", vbuf);
      } else {
        printERR(1, "BAD_ARG");
        return;
      }

      return;
    }

    // --- Original TUCK SET [LEG] behavior ---
    // TUCK is a supervisory motion; ensure we are in MODE IDLE
    // so that test gait or other planners are not concurrently writing targets.
    modeSetIdle();
    // TUCK is a supervisory motion; ensure we are in MODE IDLE
    // so that test gait or other planners are not concurrently writing targets.
    modeSetIdle();

    // Peek optional leg token (ignore returned index; we only care about ts/tl)
    nextToken(line, s, len, &ts, &tl);
    uint8_t mask = 0;

    if (tl <= 0) {
      // No leg specified -> all enabled legs
      for (uint8_t L = 0; L < 6; ++L) {
        if (leg_enabled_mask_get(L)) mask |= (uint8_t)(1u << L);
      }
    } else {
      char legt[4] = {0};
      int nl = min(tl, 3);
      memcpy(legt, line + ts, nl);
      for (int i = 0; i < nl; ++i) legt[i] = (char)toupper(legt[i]);
      int leg = legIndexFromToken(legt);
      if (leg < 0) {
        printERR(1, "BAD_LEG");
        return;
      }
      if (leg_enabled_mask_get((uint8_t)leg)) mask |= (uint8_t)(1u << leg);
    }

    if (mask == 0) {
      printERR(92, "NO_LEGS");
      return;
    }

    g_tuck_active    = 1;
    g_tuck_mask      = mask;
    g_tuck_done_mask = 0;
    g_tuck_start_ms  = millis();

    // Kick off tibia motion for selected legs; rest of sequence is handled in loopTick()
    for (uint8_t L = 0; L < 6; ++L) {
      if ((mask >> L) & 1u) {
        if (joint_enabled_mask_get(L, 2) && (((g_servo_oos_mask >> (L * 3 + 2)) & 1u) == 0)) {
          bool isRight = (L >= 3);
          int16_t tibia_target = g_tuck_tibia_cd;
          if (isRight) {
            tibia_target = (int16_t)(2 * g_home_cd[L][2] - tibia_target);
          }
          g_cmd_cd[L][2] = tibia_target;
        }
      }
    }
    return;
  }

  printERR(1, "BAD_ARG");
}

void processCmdHOME(const char* line, int s, int len) {
  (void)line;
  (void)s;
  (void)len;
  for (int L = 0; L < 6; ++L) {
    if (!leg_enabled_mask_get((uint8_t)L)) continue;
    for (int J = 0; J < 3; ++J) {
      int idx = L * 3 + J;
      bool jointEn = joint_enabled_mask_get((uint8_t)L, (uint8_t)J);
      bool inService = ((g_servo_oos_mask >> idx) & 1u) == 0;
      if (jointEn && inService) g_cmd_cd[L][J] = g_home_cd[L][J];
    }
  }
}

/**
   @brief Calibrate and persistently save per-joint home pose and angle offsets to SD card.

   This command handler computes and writes a consistent set of per-servo calibration values
   so that the robot’s current physical pose becomes the new logical “home” pose while
   simultaneously re-centering each servo’s internal offset around a target raw-center.

   High-level behavior:
   - Reads the current position (in centidegrees) of each enabled, in-service joint.
   - Adjusts the servo’s persistent angle offset so the underlying raw mechanical center
     aligns with a fixed target (TARGET_CD = 12000 cd), within ±30° limits.
   - Records the current physical joint position as the “home” pose without moving hardware.
   - Updates in-memory calibration arrays and rewrites/merges corresponding entries in
     /config.txt on the SD card as lines of the form:
       home_cd.<LEG>.<JOINT>=<centidegrees>
   - Emits a per-joint status line on the serial console.

   Preconditions:
   - SD support must be compiled in and enabled (MARS_ENABLE_SD).
   - The SD interface must initialize (SD.begin(BUILTIN_SDCARD) succeeds).
   - The robot may be physically positioned into the desired home pose prior to invocation.
   - Joint/leg enablement masks must reflect which servos are allowed to be processed:
     - leg_enabled_mask_get(L) identifies which legs are considered.
     - joint_enabled_mask_get(L, J) identifies which joints are considered.
     - g_servo_oos_mask marks joints out-of-service; those are skipped.

   Input parameters:
   - line: Original command line (unused).
   - s:    Start index (unused).
   - len:  Command length (unused).
     All three are ignored in this handler and only present to conform to the command
     processor’s interface. They are explicitly silenced to avoid compiler warnings.

   Units and limits:
   - Positions are handled as centidegrees (cd). 1 degree = 100 cd.
   - Target raw center is TARGET_CD = 12000 cd (i.e., 120.00°).
   - Persistent offset is stored in “units” where 1 unit = 24 cd = 0.24°.
   - The servo offset is clamped to MAX_OFFSET_UNITS = ±125 (±3000 cd ≈ ±30.00°).

   Detailed algorithm:
   1) Servo iteration and filtering:
      - Enumerate all legs L in [0..5] and joints J in [0..2] mapping to:
        legs:  LF, LM, LR, RF, RM, RR
        joints: coxa, femur, tibia
      - Skip any leg or joint that is disabled or flagged out-of-service.

   2) Position capture and offset computation for each processed joint:
      - Read the joint’s current position pos_cd in centidegrees; if the read fails (pos_cd < 0), skip.
      - Retrieve the servo’s current persistent offset in “units” and clamp to ±125.
      - Compute the current raw (un-offset) position:
          raw_cd = pos_cd - (current_units * 24)
      - Compute a rounded delta (in units) required to bring raw_cd to TARGET_CD:
          delta_units = cd_to_units_round(TARGET_CD - raw_cd)
      - Propose new persistent units:
          new_units = clamp(current_units + delta_units, -125, +125)
      - If the offset changed, apply the delta immediately and persist it:
          angle_offset_adjust(sid, applied_delta)
          angle_offset_write(sid)
      - Read back the stored value (final_units) and clamp for safety, then compute:
          final_offset_cd = final_units * 24

   3) Home pose selection and in-memory state:
      - The “home” pose is defined as the robot’s current exact physical pose, not the
        computed target. Thus:
          residual_home_cd = pos_cd
      - Update globals per joint:
          g_home_cd[L][J]   = residual_home_cd
          g_offset_cd[L][J] = final_offset_cd
      - Collect a SaveInfo record to drive file rewriting and logging.

   4) SD card configuration file update (/config.txt):
      - Open /config.txt for read if it exists; always open /config.tmp for write.
      - Stream the old file line-by-line with a 256-byte buffer:
        - Normalize CRLF to LF (ignore '\r').
        - Preserve unrelated lines verbatim.
        - For each processed joint, match keys of the form (leading whitespace ignored):
            "home_cd.<LEG>.<JOINT>="
          If matched, overwrite the line with the new value:
            home_cd.<LEG>.<JOINT>=<residual_home_cd>
          and mark it as replaced.
        - Lines longer than the buffer are flushed in chunks to avoid overflow; this preserves
          content but may split extremely long lines across writes (not typical for config keys).
      - After streaming, append any missing keys for joints that were not found:
          home_cd.<LEG>.<JOINT>=<residual_home_cd>
      - Close files, then replace the original by:
          close(tmp); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt")
        Note: This is a best-effort replacement; power loss between remove and rename can
        transiently leave only the .tmp or no file. Ensure robust power during calibration.

   5) Logging:
      - For each processed joint, print a single-line status record to Serial:
          HOMESSET <LEG>.<JOINT> pos_cd=<pos> new_offset_cd=<final_offset_cd> home_cd=<residual_home_cd>

   Data format in /config.txt:
   - Key names are stable and human-editable:
       home_cd.LF.coxa=<int>
       home_cd.LF.femur=<int>
       home_cd.LF.tibia=<int>
       ...
       home_cd.RR.tibia=<int>
   - Values are int16 centidegrees representing the saved physical home pose, not the
     servo offset or target center.

   Side effects:
   - Persists updated servo offset values to non-volatile storage per servo.
   - Updates in-memory arrays g_home_cd and g_offset_cd used elsewhere at runtime.
   - Rewrites /config.txt entries for all handled joints; preserves unrelated keys and comments.
   - Emits Serial logs for visibility and diagnostics.

   Error handling and early exits:
   - If SD is not enabled at compile time: printERR(2, "NO_SD") and return.
   - If SD.begin() fails: printERR(2, "NO_SD") and return.
   - If the temp output file cannot be created: printERR(2, "SD_WRITE_FAIL") and return.
   - If /config.txt does not exist or cannot be read: the process still succeeds by creating
     a new /config.txt that includes only the needed home_cd.* lines (others are absent).
   - Per-joint read failures (pos_cd < 0) silently skip that joint; no file updates for it.

   Safety and constraints:
   - Offset clamping ensures servos never store offsets beyond ±30° equivalent (±125 units).
   - The robot is not commanded to move; only persistent offsets and saved home values change.
   - This routine is not re-entrant and assumes single-threaded invocation typical of MCU firmware.

   Complexity and resource usage:
   - Processes up to 18 joints (6 legs × 3 joints).
   - Uses a fixed 256-byte line buffer for file rewriting.
   - SaveInfo array stores up to 18 entries with small POD records.

   Typical use:
   - Manually place the robot into the desired neutral/home pose.
   - Invoke this command to:
     1) Save the current pose as “home” per joint.
     2) Re-center servo offsets toward a known mechanical target center.
     3) Persist both the pose (to SD) and the offsets (to servo/EEPROM).

   @param line Unused command buffer pointer (ignored).
   @param s    Unused command substring start index (ignored).
   @param len  Unused command substring length (ignored).

   @return void

   @note The function treats leading spaces/tabs as insignificant when matching keys.
   @note Keys are matched case-sensitively and must follow: home_cd.<LEG>.<JOINT>=
   @note The target center (12000 cd) corresponds to 120.00 degrees by convention in this firmware.
   @warning Power loss during file replacement may leave configuration in a transient state.
   @warning Very long lines in /config.txt are handled conservatively; they are forwarded but may be
            chunked, which can be undesirable if such lines are not expected. Keep config lines short.
*/
void processCmdSAVEHOME(const char* line, int s, int len)
{
  (void)line; (void)s; (void)len; // unused
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  if (!SD.begin(BUILTIN_SDCARD)) {
    printERR(2, "NO_SD");
    return;
  }

  // Centidegree calibration: center mechanical neutral at TARGET_CD by storing absolute hardware offsets.
  const int TARGET_CD     = 12000; // 120.00° logical center
  const int MAX_OFFSET_CD = 3000;  // ±30° clamp (centidegrees)

  struct SaveInfo {
    uint8_t L; uint8_t J;
    int16_t pos_before_cd;   // position read before any offset changes
    int16_t final_offset_cd; // hardware offset after calibration (clamped)
    int16_t home_cd;         // post-calibration measured logical position (persisted)
    bool replaced_home;      // existing home_cd line replaced
    bool replaced_offset;    // existing offset_cd line replaced
  };
  SaveInfo infos[18]; int infoCount = 0;

  const char* legNames[6]   = { "LF", "LM", "LR", "RF", "RM", "RR" };
  const char* jointNames[3] = { "coxa", "femur", "tibia" };

  // Iterate enabled, in-service servos
  for (uint8_t L = 0; L < 6; ++L) {
    if (!leg_enabled_mask_get(L)) continue;
    for (uint8_t J = 0; J < 3; ++J) {
      int idx = L * 3 + J;
      if (!joint_enabled_mask_get(L, J)) continue;
      if (((g_servo_oos_mask >> idx) & 1u) != 0) continue; // out-of-service

      int16_t pos1 = readServoPosCdSync(L, J); // initial logical reading (with whatever offset currently stored)
      if (pos1 < 0) continue; // skip on read failure
      uint8_t sid = servoId(L, J);

      // 1) Clear existing offset to get a true RAW reading baseline (absolute centidegrees)
      angle_offset_adjust(L, sid, 0);
      angle_offset_write(L, sid);
      delay(5); // allow EEPROM/flash write to settle

      // 2) Re-read position with zero offset: this is the raw mechanical angle
      int16_t raw_cd = readServoPosCdSync(L, J);
      if (raw_cd < 0) raw_cd = pos1; // fallback to previous if read transiently fails

      // 3) Compute new ABSOLUTE offset in centidegrees so (raw_cd - offset_cd) == TARGET_CD
      //    i.e., offset_cd = raw_cd - TARGET_CD (clamped to ±3000 cd)
      int32_t new_offset_cd = (int32_t)raw_cd - (int32_t)TARGET_CD;
      if (new_offset_cd < -MAX_OFFSET_CD) new_offset_cd = -MAX_OFFSET_CD;
      if (new_offset_cd >  MAX_OFFSET_CD) new_offset_cd =  MAX_OFFSET_CD;

      // 4) Apply and persist absolute offset in centidegrees
      angle_offset_adjust(L, sid, (int16_t)new_offset_cd);
      angle_offset_write(L, sid);
      delay(5); // allow EEPROM/flash write to settle

      // 5) Read back final stored offset (should already be in cd) and post-calibration logical position
      int final_offset_cd = angle_offset_read(L, sid);
      int16_t pos_after = readServoPosCdSync(L, J);
      Serial << F("DEBUG: ") << legNames[L] << "." << jointNames[J]
             << F(" pos1=") << pos1
             << F(" raw=") << raw_cd
             << F(" new_offset_cd=") << (int)new_offset_cd
             << F(" final_offset_cd=") << final_offset_cd
             << F(" pos_after=") << pos_after << F("\r\n");
      // Update runtime mirrors
      g_offset_cd[L][J] = (int16_t)final_offset_cd;
      g_home_cd[L][J]   = pos_after; // logical home pose captured after offset applied
      g_cmd_cd[L][J]   = pos_after; // update command target to match new home

      infos[infoCount++] = { L, J, pos1, (int16_t)final_offset_cd, pos_after, false, false };
    }
  }

  // Rewrite /config.txt updating/adding home_cd.* and offset_cd.* lines.
  File fin  = SD.open("/config.txt", FILE_READ);
  File fout = SD.open("/config.tmp", FILE_WRITE);
  if (!fout) {
    if (fin) fin.close();
    printERR(2, "SD_WRITE_FAIL");
    return;
  }

  if (fin) {
    static char linebuf[256]; int llen = 0;
    while (fin.available()) {
      int ch = fin.read(); if (ch < 0) break;
      if (ch == '\r') continue;
      if (ch == '\n') {
        linebuf[llen] = 0;
        const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
        bool replaced = false;
        for (int i = 0; i < infoCount; ++i) {
          char keyH[56]; char keyO[56];
          snprintf(keyH, sizeof(keyH), "home_cd.%s.%s=", legNames[infos[i].L], jointNames[infos[i].J]);
          snprintf(keyO, sizeof(keyO), "offset_cd.%s.%s=", legNames[infos[i].L], jointNames[infos[i].J]);
          size_t kHlen = strlen(keyH); size_t kOlen = strlen(keyO);
          if (strncmp(p, keyH, kHlen) == 0) {
            fout.print(keyH); fout.print((int)infos[i].home_cd); fout.print('\n');
            infos[i].replaced_home = true; replaced = true; break;
          }
          if (strncmp(p, keyO, kOlen) == 0) {
            fout.print(keyO); fout.print((int)infos[i].final_offset_cd); fout.print('\n');
            infos[i].replaced_offset = true; replaced = true; break;
          }
        }
        if (!replaced) {
          fout.print(linebuf);
          fout.print('\n');
        }
        llen = 0;
      } else if (llen + 1 < (int)sizeof(linebuf)) {
        linebuf[llen++] = (char)ch;
      } else { // flush overly long line
        linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0;
      }
    }
    fin.close();
  }

  // Append any missing keys
  for (int i = 0; i < infoCount; ++i) {
    if (!infos[i].replaced_home) {
      fout.print("home_cd."); fout.print(legNames[infos[i].L]); fout.print("."); fout.print(jointNames[infos[i].J]); fout.print("="); fout.print((int)infos[i].home_cd); fout.print('\n');
    }
    if (!infos[i].replaced_offset) {
      fout.print("offset_cd."); fout.print(legNames[infos[i].L]); fout.print("."); fout.print(jointNames[infos[i].J]); fout.print("="); fout.print((int)infos[i].final_offset_cd); fout.print('\n');
    }
  }

  fout.close();
  SD.remove("/config.txt");
  SD.rename("/config.tmp", "/config.txt");

  // Logging summary
  for (int i = 0; i < infoCount; ++i) {
    Serial.print(F("HOMESSET "));
    Serial.print(legNames[infos[i].L]); Serial.print(F(".")); Serial.print(jointNames[infos[i].J]);
    Serial.print(F(" pos_cd=")); Serial.print((int)infos[i].pos_before_cd);
    Serial.print(F(" new_offset_cd=")); Serial.print((int)infos[i].final_offset_cd);
    Serial.print(F(" home_cd=")); Serial.print((int)infos[i].home_cd);
    Serial.print(F("\r\n"));
  }
#else
  printERR(2, "NO_SD");
#endif
}

void processCmdOFFSET(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl);
  if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }

  char sub[12] = {0};
  int ns = min(tl, 11);
  memcpy(sub, line + ts, ns);
  for (int i = 0; i < ns; ++i) sub[i] = (char)toupper(sub[i]);

  // OFFSET LIST
  if (!strcmp(sub, "LIST")) {
    const char* legNames[6] = { "LF", "LM", "LR", "RF", "RM", "RR" };
    for (int L = 0; L < 6; ++L) {
      for (int J = 0; J < 3; ++J) {
        uint8_t sid = servoId((uint8_t)L, (uint8_t)J);
        int cd = angle_offset_read((uint8_t)L, sid);
        if (cd < -3000) cd = -3000;
        if (cd > 3000)  cd = 3000;
        g_offset_cd[L][J] = (int16_t)cd;
      }
    }
    Serial.print(F("OFFSET "));
    for (int L = 0; L < 6; ++L) {
      if (L) Serial.print(F(" "));
      Serial.print(legNames[L]);
      Serial.print(F("="));
      for (int J = 0; J < 3; ++J) {
        if (J) Serial.print(F("/"));
        Serial.print((int)g_offset_cd[L][J]);
      }
    }
    Serial.print(F("\r\n"));
    return;
  }

  // OFFSET CLEAR <LEG|ALL> <JOINT|ALL>
  if (!strcmp(sub, "CLEAR")) {
    s = nextToken(line, s, len, &ts, &tl);
    if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char legt[4] = {0};
    int nl = min(tl, 3);
    memcpy(legt, line + ts, nl);
    for (int i = 0; i < nl; ++i) legt[i] = (char)toupper(legt[i]);

    s = nextToken(line, s, len, &ts, &tl);
    if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char jtok[8] = {0};
    int nj = min(tl, 7);
    memcpy(jtok, line + ts, nj);
    for (int i = 0; i < nj; ++i) jtok[i] = (char)toupper(jtok[i]);

    const char* legNames[6]   = { "LF", "LM", "LR", "RF", "RM", "RR" };
    const char* jointNames[3] = { "COXA", "FEMUR", "TIBIA" };

    auto clear_one = [&](int L, int J)
    {
      if (L < 0 || L >= 6 || J < 0 || J >= 3) return;
      uint8_t sid = servoId((uint8_t)L, (uint8_t)J);
      int cur_cd = angle_offset_read((uint8_t)L, sid);
      if (cur_cd != 0) {
        angle_offset_adjust((uint8_t)L, sid, 0);
        angle_offset_write((uint8_t)L, sid);
      }
      g_offset_cd[L][J] = 0;
      int16_t pos_cd = readServoPosCdSync((uint8_t)L, (uint8_t)J);
      if (pos_cd >= 0) g_home_cd[L][J] = pos_cd;
      Serial.print(F("OFFSETCLR "));
      Serial.print(legNames[L]);
      Serial.print(F("."));
      Serial.print(jointNames[J]);
      Serial.print(F(" home_cd="));
      Serial.print((int)g_home_cd[L][J]);
      Serial.print(F("\r\n"));
    };

    if (!strcmp(legt, "ALL")) {
      if (!strcmp(jtok, "ALL")) {
        for (int L = 0; L < 6; ++L)
          for (int J = 0; J < 3; ++J)
            clear_one(L, J);
      } else {
        int J = jointIndexFromToken(jtok);
        if (J < 0) {
          printERR(1, "BAD_JOINT");
          return;
        }
        for (int L = 0; L < 6; ++L) clear_one(L, J);
      }
    } else {
      int leg = legIndexFromToken(legt);
      if (leg < 0) {
        printERR(1, "BAD_LEG");
        return;
      }
      if (!strcmp(jtok, "ALL")) {
        for (int J = 0; J < 3; ++J) clear_one(leg, J);
      } else {
        int J = jointIndexFromToken(jtok);
        if (J < 0) {
          printERR(1, "BAD_JOINT");
          return;
        }
        clear_one(leg, J);
      }
    }
    return;
  }

  printERR(1, "BAD_ARG");
}

// LOG <ENABLE|RATE|MODE|HEADER|FLUSH|STATUS> [...]
void processCmdLOG(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[10] = {0}; int ns = min(tl, 9); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  // New syntax: LOG ENABLE (no argument) turns logging on; LOG DISABLE turns it off.
  // Backwards compatibility: LOG ENABLE <ON|OFF> still supported.
  if (!strcmp(sub, "ENABLE") || !strcmp(sub, "DISABLE")) {
    bool request_enable = !strcmp(sub, "ENABLE");
    // Peek optional token if ENABLE form used
    int ts2, tl2; int s2 = nextToken(line, s, len, &ts2, &tl2);
    if (request_enable && tl2 > 0) {
      char vb[8] = {0}; int nv = min(tl2, 7); memcpy(vb, line + ts2, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
      if (!strcmp(vb, "OFF") || !strcmp(vb, "FALSE") || !strcmp(vb, "0")) {
        request_enable = false;
      }
      else if (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1")) {
        request_enable = true;
      }
      // If token was recognized as ON/OFF consume it
      s = s2;
    }
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    bool prev = g_log_enabled;
    if (request_enable) {
      g_log_enabled = true;
    } else {
      g_log_enabled = false;
      if (prev) {
        if (g_log_buf_used && g_log_file) {
          g_log_file.write(g_log_buf, g_log_buf_used);
          g_log_buf_used = 0;
        }
        if (g_log_file) {
          g_log_file.flush();
          g_log_file.close();
        }
      }
    }
#else
    if (request_enable) {
      printERR(2, "NO_SD");
      return;
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "RATE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char nb[12] = {0}; int nn = min(tl, 11); memcpy(nb, line + ts, nn); nb[nn] = 0; long hz = atol(nb);
    if (hz < 1) hz = 1; if (hz > 500) hz = 500;
    g_log_rate_hz = (uint16_t)hz;
    // recompute divisor (integer)
    if (g_loop_hz > 0 && g_log_rate_hz > 0 && g_loop_hz >= g_log_rate_hz) {
      uint16_t div = g_loop_hz / g_log_rate_hz; if (div < 1) div = 1; if (div > 255) div = 255; g_log_sample_div = (uint8_t)div;
    } else {
      g_log_sample_div = 1;
    }
    // persist: logging.rate_hz
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      /* ignore persist error */ return;
    }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) {
        if (fin) fin.close(); /* ignore persist error */ return;
      }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.rate_hz="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print((unsigned int)g_log_rate_hz);
              fout.print('\n');
              replaced = true;
            }
            else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          }
          else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("logging.rate_hz=");
        fout.print((unsigned int)g_log_rate_hz);
        fout.print('\n');
      }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "MODE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char mtok[10] = {0}; int nm = min(tl, 9); memcpy(mtok, line + ts, nm); for (int i = 0; i < nm; ++i) mtok[i] = toupper(mtok[i]);
    uint8_t m = 0; if (!strcmp(mtok, "FULL") || !strcmp(mtok, "1")) m = 1; else m = 0; g_log_mode = m;
    // persist: logging.mode (numeric 0|1)
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      /* ignore persist error */ return;
    }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) {
        if (fin) fin.close();
        return;
      }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.mode="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print((unsigned int)g_log_mode);
              fout.print('\n');
              replaced = true;
            }
            else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          }
          else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("logging.mode=");
        fout.print((unsigned int)g_log_mode);
        fout.print('\n');
      }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "HEADER")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_log_header = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      return;
    }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) {
        if (fin) fin.close();
        return;
      }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.header="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print(g_log_header ? 1 : 0);
              fout.print('\n');
              replaced = true;
            }
            else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          }
          else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("logging.header=");
        fout.print(g_log_header ? 1 : 0);
        fout.print('\n');
      }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "FLUSH")) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (g_log_file) {
      if (g_log_buf_used) {
        g_log_file.write(g_log_buf, g_log_buf_used);
        g_log_buf_used = 0;
      }
      g_log_file.flush();
    } else {
      // No file open; nothing to flush
    }
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "CLEAR")) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    // Flush and close then delete current file; next sample will reopen
    if (g_log_file) {
      if (g_log_buf_used) {
        g_log_file.write(g_log_buf, g_log_buf_used);
        g_log_buf_used = 0;
      }
      g_log_file.flush();
      // Capture name before close (File doesn't expose path directly; rotation used seq; deletion best-effort via scanning)
      g_log_file.close();
      // Best-effort: iterate /logs and delete the most recent file matching seq index
      if (SD.begin(BUILTIN_SDCARD)) {
        File dir = SD.open("/logs");
        if (dir) {
          char newestName[64] = {0};
          uint32_t newestTime = 0;
          while (true) {
            File f = dir.openNextFile();
            if (!f) break;
            if (!f.isDirectory()) {
              // Match pattern _seq<seq>.csv
              const char* n = f.name();
              if (strstr(n, "_seq")) {
                // Heuristic: later modification time or larger size -> choose
                uint32_t sz = f.size();
                if (sz >= newestTime) {
                  strncpy(newestName, n, sizeof(newestName) - 1);
                  newestTime = sz;
                }
              }
            }
            f.close();
          }
          dir.close();
          if (newestName[0]) {
            SD.remove(newestName);
          }
        }
      }
    }
    // Reset file byte counters so next sample opens a new file cleanly
    g_log_file_bytes = 0; g_log_buf_used = 0; // g_log_file already closed
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "STATUS") || !strcmp(sub, "LIST")) {
    Serial.print(F("LOG "));
    Serial.print(F("enabled=")); Serial.print(g_log_enabled ? 1 : 0);
    Serial.print(F(" rate_hz=")); Serial.print((unsigned int)g_log_rate_hz);
    Serial.print(F(" mode=")); Serial.print((unsigned int)g_log_mode);
    Serial.print(F(" header=")); Serial.print(g_log_header ? 1 : 0);
    Serial.print(F(" div=")); Serial.print((unsigned int)g_log_sample_div);
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    Serial.print(F(" buf_used=")); Serial.print((unsigned int)g_log_buf_used);
    Serial.print(F(" open=")); Serial.print(g_log_file ? 1 : 0);
    Serial.print(F(" rotate=")); Serial.print(g_log_rotate ? 1 : 0);
    Serial.print(F(" max_kb=")); Serial.print((unsigned long)(g_log_max_bytes / 1024UL));
    Serial.print(F(" size_kb=")); Serial.print((unsigned long)(g_log_file_bytes / 1024UL));
    Serial.print(F(" seq=")); Serial.print((unsigned long)g_log_seq);
#endif
    Serial.print(F("\r\n"));
    return;
  }
  if (!strcmp(sub, "ROTATE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_log_rotate = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      return;
    }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) {
        if (fin) fin.close();
        return;
      }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.rotate="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print(g_log_rotate ? 1 : 0);
              fout.print('\n');
              replaced = true;
            }
            else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          }
          else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("logging.rotate=");
        fout.print(g_log_rotate ? 1 : 0);
        fout.print('\n');
      }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    return;
  }

  if (!strcmp(sub, "MAXKB")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long kb = atol(nb);
    if (kb < 100) kb = 100; // minimum 100KB
    long maxKBClamp = 1024L * 1024L; // 1GB in KB
    if (kb > maxKBClamp) kb = maxKBClamp; // hard clamp to 1GB
    g_log_max_bytes = (uint32_t)kb * 1024UL;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) {
      return;
    }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) {
        if (fin) fin.close();
        return;
      }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.max_kb="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) {
              fout.print(key);
              fout.print((unsigned long)(g_log_max_bytes / 1024UL));
              fout.print('\n');
              replaced = true;
            }
            else {
              fout.print(linebuf);
              fout.print('\n');
            }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) {
            linebuf[llen++] = (char)ch2;
          }
          else {
            linebuf[llen] = 0;
            fout.print(linebuf);
            fout.print('\n');
            llen = 0;
          }
        }
        fin.close();
      }
      if (!replaced) {
        fout.print("logging.max_kb=");
        fout.print((unsigned long)(g_log_max_bytes / 1024UL));
        fout.print('\n');
      }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    return;
  }

  if (!strcmp(sub, "TAIL")) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    // Parse N
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
      printERR(1, "BAD_ARG");
      return;
    }
    char nb[12] = {0}; int nn = min(tl, 11); memcpy(nb, line + ts, nn); nb[nn] = 0; long want = atol(nb);
    if (want < 1) want = 1; if (want > 100) want = 100; // bound to avoid excessive prints

    // Ensure we have a file
    if (!g_log_file) {
      printERR(3, "NO_FILE");
      return;
    }
    // Flush any staged buffer
    if (g_log_buf_used) {
      g_log_file.write(g_log_buf, g_log_buf_used);
      g_log_buf_used = 0;
    }
    g_log_file.flush();

    uint32_t fsize = g_log_file.size();
    if (fsize == 0) {
      // Print header only (fallback)
      Serial.print(F("0: time_ms,leg,joint,cmd_cd,meas_cd,vin_V,temp_C,err\r\n"));
      return;
    }

    // Collect start offsets of last N data rows (skip comment lines starting with '#'
    // and skip any header line starting with "time_ms,")
    uint32_t starts[100];
    uint16_t found = 0;
    uint32_t pos = fsize;
    // Handle a possible trailing line without newline by treating end as a break
    while (pos > 0 && found < (uint16_t)want) {
      pos--;
      g_log_file.seek(pos);
      int c = g_log_file.read();
      if (c == '\n') {
        uint32_t line_start = pos + 1;
        // Peek first few bytes of the line to filter
        g_log_file.seek(line_start);
        int c0 = g_log_file.read();
        if (c0 <= 0 || c0 == '\n' || c0 == '\r' || c0 == '#') {
          continue;
        }
        if (c0 == 't') {
          // Possible header line
          char hb[9] = {0}; hb[0] = 't';
          for (int i = 1; i < 9; ++i) {
            int ci = g_log_file.read();
            if (ci < 0) {
              hb[i] = 0;
              break;
            } hb[i] = (char)ci;
          }
          if (!strncmp(hb, "time_ms,", 8)) {
            continue;
          }
        }
        starts[found++] = line_start;
      }
    }
    // Also consider the first line if we haven't yet reached N
    if (found < (uint16_t)want && pos == 0) {
      // Evaluate line starting at 0
      g_log_file.seek(0);
      int c0 = g_log_file.read();
      if (c0 > 0 && c0 != '\n' && c0 != '\r' && c0 != '#') {
        if (c0 != 't') {
          starts[found++] = 0;
        }
        else {
          char hb[9] = {0}; hb[0] = 't'; for (int i = 1; i < 9; ++i) {
            int ci = g_log_file.read();
            if (ci < 0) {
              hb[i] = 0;
              break;
            } hb[i] = (char)ci;
          }
          if (strncmp(hb, "time_ms,", 8) != 0) {
            starts[found++] = 0;
          }
        }
      }
    }

    // Print actual header line from file (first non-comment line starting with time_ms)
    {
      g_log_file.seek(0);
      char headerBuf[120] = {0};
      int hbUsed = 0; bool headerPrinted = false;
      while (g_log_file.available()) {
        int ch = g_log_file.read(); if (ch < 0) break;
        if (ch == '\r') continue;
        if (ch == '\n') {
          if (hbUsed > 0) {
            headerBuf[hbUsed] = 0;
            // Skip comments
            if (headerBuf[0] != '#') {
              // Expect starts with time_ms
              if (!strncmp(headerBuf, "time_ms,", 8)) {
                Serial.print(F("0: ")); Serial.print(headerBuf); Serial.print(F("\r\n"));
                headerPrinted = true;
                break;
              }
            }
          }
          hbUsed = 0; continue;
        }
        if (hbUsed + 1 < (int)sizeof(headerBuf)) headerBuf[hbUsed++] = (char)ch;
      }
      if (!headerPrinted) {
        // Fallback: legacy header without oos
        Serial.print(F("0: time_ms,leg,joint,cmd_cd,meas_cd,vin_V,temp_C,err\r\n"));
      }
    }
    // Print collected lines in chronological order
    for (int i = (int)found - 1, lineNo = 1; i >= 0; --i, ++lineNo) {
      uint32_t ls = starts[i];
      g_log_file.seek(ls);
      Serial.print(lineNo); Serial.print(F(": "));
      // Stream until newline or EOF
      while (true) {
        int ch = g_log_file.read();
        if (ch < 0 || ch == '\n') break;
        if (ch == '\r') continue;
        Serial.write((char)ch);
      }
      Serial.print(F("\r\n"));
    }
    return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }
}

// -----------------------------------------------------------------------------
// EST (Estimator) command handler
// -----------------------------------------------------------------------------

static void est_print_list() {
  Serial.print(F("EST cmd_alpha_milli=")); Serial.print((unsigned int)g_est_cmd_alpha_milli);
  Serial.print(F(" meas_alpha_milli=")); Serial.print((unsigned int)g_est_meas_alpha_milli);
  Serial.print(F(" meas_vel_alpha_milli=")); Serial.print((unsigned int)g_est_meas_vel_alpha_milli);
  Serial.print(F("\r\n"));
}

void processCmdEST(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char sub[20] = {0}; int ns = min(tl, 19); memcpy(sub, line + ts, ns);
  for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "LIST")) {
    est_print_list();
    return;
  }

  // EST <FIELD> <VAL>
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) {
    printERR(1, "BAD_ARG");
    return;
  }
  char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0; long v = atol(vb);

  if (!strcmp(sub, "CMD_ALPHA")) {
    if (v < 0) v = 0; if (v > 1000) v = 1000;
    g_est_cmd_alpha_milli = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("est.cmd_alpha_milli=", String((unsigned long)g_est_cmd_alpha_milli));
#endif
    return;
  }

  if (!strcmp(sub, "MEAS_ALPHA")) {
    if (v < 0) v = 0; if (v > 1000) v = 1000;
    g_est_meas_alpha_milli = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("est.meas_alpha_milli=", String((unsigned long)g_est_meas_alpha_milli));
#endif
    return;
  }

  if (!strcmp(sub, "MEAS_VEL_ALPHA")) {
    if (v < 0) v = 0; if (v > 2000) v = 2000;
    g_est_meas_vel_alpha_milli = (uint16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    (void)config_update_single_key("est.meas_vel_alpha_milli=", String((unsigned long)g_est_meas_vel_alpha_milli));
#endif
    return;
  }

  printERR(1, "BAD_ARG");
}

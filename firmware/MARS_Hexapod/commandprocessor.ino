// commandprocessor.ino — Modular command handling (refactored)
// Each top-level serial command now handled by a dedicated routine.
// Keeps core parser minimal and enables switch-based dispatch.

#include <Arduino.h>
#include <ctype.h>
#include <lx16a-servo.h>
#include "robot_config.h"
#include "command_types.h"
#include "command_helpers.h"

// Ensure feature flags have sane defaults in this TU as well
#ifndef MARS_ENABLE_SD
#define MARS_ENABLE_SD 1
#endif

// Externs from other compilation units (defined in MARS_Hexapod.ino / functions.ino)
extern void printHELP();
extern void printSTATUS();
extern void rebootNow();
extern void modeSetTest();
extern void modeSetIdle();
extern bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]);
extern int16_t readServoPosCdSync(uint8_t leg, uint8_t joint);
extern int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen);
extern void printERR(uint8_t code, const char* msg);
extern uint8_t servoId(uint8_t leg, uint8_t joint);
extern void setServoTorqueNow(uint8_t leg, uint8_t joint, bool on);
extern bool fk_leg_body(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                        float* out_x_mm, float* out_y_mm, float* out_z_mm);

// Shared state (extern)
extern volatile uint8_t  g_last_err;
extern volatile bool     g_enabled;
extern volatile bool     g_lockout;
extern volatile uint16_t g_lockout_causes;
extern volatile uint16_t g_override_mask;
extern volatile uint8_t  g_leg_enabled_mask;
extern volatile uint32_t g_joint_enabled_mask;
extern volatile uint32_t g_servo_oos_mask;
extern volatile uint8_t  g_rr_index;
extern volatile uint16_t g_rate_limit_cdeg_per_s;
extern volatile int16_t  g_cmd_cd[6][3];
extern volatile int16_t  g_home_cd[6][3];
extern volatile int16_t  g_limit_min_cd[6][3];
extern volatile int16_t  g_limit_max_cd[6][3];
extern int16_t           g_offset_cd[6][3];
extern uint16_t          g_meas_vin_mV[6][3];
extern uint8_t           g_meas_temp_C[6][3];
extern int16_t           g_meas_pos_cd[6][3];
extern float             g_test_base_y_mm;
extern float             g_test_base_x_mm;
extern float             g_test_step_len_mm;
extern uint32_t          g_test_cycle_ms;
extern float             g_test_lift_y_mm;
extern float             g_test_overlap_pct;
extern float             g_safety_clearance_mm;
extern float             g_foot_target_x_mm[6];
extern float             g_foot_target_z_mm[6];
extern volatile bool     g_safety_soft_limits_enabled;
extern volatile bool     g_safety_collision_enabled;
extern volatile int16_t  g_safety_temp_lockout_c10;
extern volatile uint8_t  g_fk_stream_mask;

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
// Logging globals (from main)
extern volatile bool     g_log_enabled;
extern volatile uint16_t g_log_rate_hz;
extern volatile uint8_t  g_log_sample_div;
extern volatile uint32_t g_log_tick_counter;
extern volatile uint8_t  g_log_mode;
extern volatile bool     g_log_header;
extern volatile bool     g_log_rotate;
extern volatile uint32_t g_log_max_bytes;
extern volatile uint32_t g_log_file_bytes;
extern volatile uint32_t g_log_total_bytes;
extern volatile uint32_t g_log_seq;
extern char              g_log_buf[8192];
extern uint16_t          g_log_buf_used;
extern File              g_log_file;
#endif
extern volatile uint16_t g_loop_hz;


// Hardware angle offset helpers (provided by lx16a-servo library or stubs)
extern int  angle_offset_read(uint8_t leg, uint8_t id); // returns cd
extern bool angle_offset_adjust(uint8_t leg, uint8_t id, int16_t offset_cd); // sets absolute cd
extern bool angle_offset_write(uint8_t leg, uint8_t id);

// SD enable flag
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#include <SD.h>
#endif

// -------------------------------------------------------------------------------------------------
// Command enumeration & mapping (enum defined in command_types.h)
// -------------------------------------------------------------------------------------------------

// Helpers provided by command_helpers.h

CommandType parseCommandType(const char* cmd) {
  if (!cmd || !*cmd) return CMD_UNKNOWN;
  // Single-letter shortcuts first
  if (cmd[1]==0) { if (cmd[0]=='I') return CMD_I; if (cmd[0]=='T') return CMD_T; }
  if (!strcmp(cmd,"HELP")) return CMD_HELP;
  if (!strcmp(cmd,"STATUS")) return CMD_STATUS;
  if (!strcmp(cmd,"REBOOT")) return CMD_REBOOT;
  if (!strcmp(cmd,"ENABLE")) return CMD_ENABLE;
  if (!strcmp(cmd,"DISABLE")) return CMD_DISABLE;
  if (!strcmp(cmd,"FK")) return CMD_FK;
  if (!strcmp(cmd,"LEGS")) return CMD_LEGS;
  if (!strcmp(cmd,"SERVOS")) return CMD_SERVOS;
  if (!strcmp(cmd,"LEG")) return CMD_LEG;
  if (!strcmp(cmd,"SERVO")) return CMD_SERVO;
  if (!strcmp(cmd,"RAW")) return CMD_RAW;
  if (!strcmp(cmd,"RAW3")) return CMD_RAW3;
  if (!strcmp(cmd,"FOOT")) return CMD_FOOT;
  if (!strcmp(cmd,"FEET")) return CMD_FEET;
  if (!strcmp(cmd,"MODE")) return CMD_MODE;
  if (!strcmp(cmd,"TEST")) return CMD_TEST;
  if (!strcmp(cmd,"STAND")) return CMD_STAND;
  if (!strcmp(cmd,"SAFETY")) return CMD_SAFETY;
  if (!strcmp(cmd,"HOME")) return CMD_HOME;
  if (!strcmp(cmd,"SAVEHOME")) return CMD_SAVEHOME;
  if (!strcmp(cmd,"OFFSET")) return CMD_OFFSET;
  if (!strcmp(cmd,"LOG")) return CMD_LOG;
  if (!strcmp(cmd,"TUCK")) return CMD_TUCK;
  return CMD_UNKNOWN;
}

// -------------------------------------------------------------------------------------------------
// Per-command routines (minimal wrappers preserving original semantics)
// Each function receives: original line, current parse index (start after command token), and total length.
// -------------------------------------------------------------------------------------------------

void processCmdHELP(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; printHELP(); }
void processCmdSTATUS(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; printSTATUS(); }
void processCmdREBOOT(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; /* centralized layer will handle OK+reboot */ }
void processCmdENABLE(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; g_enabled=true; }
void processCmdDISABLE(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; g_enabled=false; for(uint8_t L=0;L<6;++L){ for(uint8_t J=0;J<3;++J) setServoTorqueNow(L,J,false);} }

void processCmdFK(const char* line,int s,int len)
{ int ts,tl; s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char legt[4]={0}; int nl=min(tl,3); memcpy(legt,line+ts,nl); for(int i=0;i<nl;++i) legt[i]=toupper(legt[i]); s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char vb[8]={0}; int nv=min(tl,7); memcpy(vb,line+ts,nv); for(int i=0;i<nv;++i) vb[i]=toupper(vb[i]); bool on = (!strcmp(vb,"ON")||!strcmp(vb,"TRUE")||!strcmp(vb,"1")); if(!strcmp(legt,"ALL")){ if(on) g_fk_stream_mask|=0x3Fu; else g_fk_stream_mask&=~0x3Fu; return;} int leg=legIndexFromToken(legt); if(leg<0){printERR(1,"BAD_LEG");return;} if(on) g_fk_stream_mask|=(uint8_t)(1u<<leg); else g_fk_stream_mask&=(uint8_t)~(1u<<leg); }

void processCmdLEGS(const char* line,int s,int len){ (void)line;(void)s;(void)len; const char* names[6]={"LF","LM","LR","RF","RM","RR"}; Serial.print(F("LEGS ")); for(int i=0;i<6;++i){ if(i) Serial.print(F(" ")); Serial.print(names[i]); Serial.print(F("=")); Serial.print(leg_enabled_mask_get((uint8_t)i)?1:0);} Serial.print(F("\r\n")); }

void processCmdSERVOS(const char* line,int s,int len){ (void)line;(void)s;(void)len; const char* names[6]={"LF","LM","LR","RF","RM","RR"}; Serial.print(F("SERVOS ")); for(int i=0;i<6;++i){ if(i) Serial.print(F(" ")); Serial.print(names[i]); Serial.print(F("=")); for(int j=0;j<3;++j) Serial.print(joint_enabled_mask_get((uint8_t)i,(uint8_t)j)?1:0);} Serial.print(F("\r\n")); }

// LEG <LEG|ALL> <ENABLE|DISABLE>
void processCmdLEG(const char* line, int s, int len)
{
  int ts, tl;
  // Parse leg token
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  // Parse action token
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);
  bool doEnable = !strcmp(act, "ENABLE");
  bool doDisable = !strcmp(act, "DISABLE");
  if (!doEnable && !doDisable) { printERR(1, "BAD_ARG"); return; }

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
    if (leg < 0) { printERR(1, "BAD_LEG"); return; }
    apply_leg(leg);
  }

  /* success */
}

// SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>
void processCmdSERVO(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char jtok[8] = {0}; int nj = min(tl, 7); memcpy(jtok, line + ts, nj); for (int i = 0; i < nj; ++i) jtok[i] = toupper(jtok[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char act[10] = {0}; int na = min(tl, 9); memcpy(act, line + ts, na); for (int i = 0; i < na; ++i) act[i] = toupper(act[i]);
  bool doEnable = !strcmp(act, "ENABLE");
  bool doDisable = !strcmp(act, "DISABLE");
  if (!doEnable && !doDisable) { printERR(1, "BAD_ARG"); return; }

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
      int J = jointIndexFromToken(jtok); if (J < 0) { printERR(1, "BAD_JOINT"); return; }
      for (int L = 0; L < 6; ++L) apply_servo(L, J);
    }
  } else {
    int leg = legIndexFromToken(legt); if (leg < 0) { printERR(1, "BAD_LEG"); return; }
    if (!strcmp(jtok, "ALL")) {
      for (int J = 0; J < 3; ++J) apply_servo(leg, J);
    } else {
      int J = jointIndexFromToken(jtok); if (J < 0) { printERR(1, "BAD_JOINT"); return; }
      apply_servo(leg, J);
    }
  }
  /* success */
}

// RAW <LEG> <JOINT> <centideg>
void processCmdRAW(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  int leg = legIndexFromToken(legt); if (leg < 0) { printERR(1, "BAD_LEG"); return; }
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char jt[8] = {0}; int nj = min(tl, 7); memcpy(jt, line + ts, nj); for (int i = 0; i < nj; ++i) jt[i] = toupper(jt[i]);
  int joint = jointIndexFromToken(jt); if (joint < 0) { printERR(1, "BAD_JOINT"); return; }
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char nb[16] = {0}; int nn = min(tl, 15); memcpy(nb, line + ts, nn); nb[nn] = 0; long v = atol(nb); if (v < 0) v = 0; if (v > 24000) v = 24000;
  g_cmd_cd[leg][joint] = (int16_t)v;
  /* success */
}

// RAW3 <LEG> <c_cd> <f_cd> <t_cd>
void processCmdRAW3(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char legt[4] = {0}; int nl = min(tl, 3); memcpy(legt, line + ts, nl); for (int i = 0; i < nl; ++i) legt[i] = toupper(legt[i]);
  int leg = legIndexFromToken(legt); if (leg < 0) { printERR(1, "BAD_LEG"); return; }
  long vals[3];
  for (int j = 0; j < 3; ++j) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
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
  if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char legt[4] = {0};
  int nl = min(tl, 3);
  memcpy(legt, line + ts, nl);
  for (int i = 0; i < nl; ++i) legt[i] = (char)toupper(legt[i]);
  int leg = legIndexFromToken(legt);
  if (leg < 0) { printERR(1, "BAD_LEG"); return; }

  // Parse 3 numeric Cartesian components: x y z (mm)
  float xyz[3];
  for (int j = 0; j < 3; ++j) {
    s = nextToken(line, s, len, &ts, &tl);
    if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
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

void processCmdFEET(const char* line,int s,int len){ int ts,tl; float p[6][3]; for(int i=0;i<6;++i){ for(int j=0;j<3;++j){ s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char nb[24]={0}; int nn=min(tl,23); memcpy(nb,line+ts,nn); nb[nn]=0; p[i][j]=atof(nb);} } for(int L=0;L<6;++L){ int16_t out[3]; if(!calculateIK((uint8_t)L,p[L][0],p[L][1],p[L][2],out)){ printERR(1,"IK_FAIL"); return;} bool isRight=(L>=3); if(isRight){ g_cmd_cd[L][0]=out[0]; g_cmd_cd[L][1]=out[1]; g_cmd_cd[L][2]=out[2]; } else { g_cmd_cd[L][0]=(int16_t)(2*g_home_cd[L][0]-out[0]); g_cmd_cd[L][1]=(int16_t)(2*g_home_cd[L][1]-out[1]); g_cmd_cd[L][2]=(int16_t)(2*g_home_cd[L][2]-out[2]); } } /* success */ }

void processCmdMODE(const char* line,int s,int len){ int ts,tl; s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char mtok[8]={0}; int nm=min(tl,7); memcpy(mtok,line+ts,nm); for(int i=0;i<nm;++i) mtok[i]=toupper(mtok[i]); if(!strcmp(mtok,"TEST")){ modeSetTest(); return;} if(!strcmp(mtok,"IDLE")){ modeSetIdle(); return;} printERR(1,"BAD_ARG"); }
void processCmdI(const char* line,int s,int len){ (void)line;(void)s;(void)len; modeSetIdle(); }
void processCmdT(const char* line,int s,int len){ (void)line;(void)s;(void)len; modeSetTest(); }

void processCmdTEST(const char* line, int s, int len)
{
  int ts, tl;
  // TEST <CYCLE|HEIGHT|BASEX|STEPLEN|LIFT|OVERLAP> <value>
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char sub[12] = {0}; int ns = min(tl, 11); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char vb[24] = {0}; int nv = min(tl, 23); memcpy(vb, line + ts, nv); vb[nv] = 0;

  if (!strcmp(sub, "CYCLE")) {
    long ms = atol(vb); if (ms < 750) ms = 750; if (ms > 10000) ms = 10000;
    g_test_cycle_ms = (uint32_t)ms; return;
  }
  if (!strcmp(sub, "HEIGHT")) { g_test_base_y_mm = atof(vb); return; }
  if (!strcmp(sub, "BASEX"))  { g_test_base_x_mm = atof(vb); return; }
  if (!strcmp(sub, "STEPLEN")) {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_step_len_mm = v; return;
  }
  if (!strcmp(sub, "LIFT"))    {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_lift_y_mm = v; return;
  }
  if (!strcmp(sub, "OVERLAP")) {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; if (v > 25.0f) v = 25.0f; g_test_overlap_pct = v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
    bool replaced = false;
    if (fin) {
      static char linebuf[256]; int llen = 0;
      while (fin.available()) {
        int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
        if (ch2 == '\n') {
          linebuf[llen] = 0;
          const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
          const char* key = "test.trigait.overlap_pct="; size_t klen = strlen(key);
          if (strncmp(p, key, klen) == 0) {
            fout.print(key); long iv = lroundf(g_test_overlap_pct); fout.print((int)iv); fout.print('\n');
            replaced = true;
          } else {
            fout.print(linebuf); fout.print('\n');
          }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) {
          linebuf[llen++] = (char)ch2;
        } else {
          linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0;
        }
      }
      fin.close();
    }
    if (!replaced) {
      fout.print("test.trigait.overlap_pct="); long iv = lroundf(g_test_overlap_pct); fout.print((int)iv); fout.print('\n');
    }
  fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt"); return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  printERR(1, "BAD_ARG");
}

void processCmdSTAND(const char* line,int s,int len){ (void)line;(void)s;(void)len; const float BASE_Y=g_test_base_y_mm; const float BASE_X=g_test_base_x_mm; for(int L=0;L<6;++L){ int16_t out[3]; bool ok=calculateIK((uint8_t)L,BASE_X,BASE_Y,0.0f,out); if(!ok){ printERR(1,"IK_FAIL"); return;} bool isRight=(L>=3); if(isRight){ g_cmd_cd[L][0]=out[0]; g_cmd_cd[L][1]=out[1]; g_cmd_cd[L][2]=out[2]; } else { g_cmd_cd[L][0]=(int16_t)(2*g_home_cd[L][0]-out[0]); g_cmd_cd[L][1]=(int16_t)(2*g_home_cd[L][1]-out[1]); g_cmd_cd[L][2]=(int16_t)(2*g_home_cd[L][2]-out[2]); } g_foot_target_x_mm[L]=BASE_X; g_foot_target_z_mm[L]=0.0f; } }

// TUCK command — optional leg argument
// Syntax:
//   TUCK            -> all enabled legs
//   TUCK <LEG>      -> single leg (LF,LM,LR,RF,RM,RR)
// Behavior:
//   Initiates a non-blocking sequence (tibia first → then femur/coxa) managed in loopTick.
extern volatile uint8_t  g_tuck_active;      // defined in MARS_Hexapod.ino
extern volatile uint8_t  g_tuck_mask;        // legs participating
extern volatile uint8_t  g_tuck_done_mask;   // legs with femur/coxa issued
extern volatile uint32_t g_tuck_start_ms;    // start time
extern volatile int16_t  g_tuck_tibia_cd;
extern volatile int16_t  g_tuck_femur_cd;
extern volatile int16_t  g_tuck_coxa_cd;
extern volatile int16_t  g_tuck_tol_tibia_cd;
extern volatile int16_t  g_tuck_tol_other_cd;
extern volatile uint16_t g_tuck_timeout_ms;
void processCmdTUCK(const char* line,int s,int len)
{
  int ts, tl;
  // Syntax extensions:
  //   TUCK                -> all enabled legs (use current parameters)
  //   TUCK <LEG>          -> single leg
  //   TUCK SET <param> <value>  -> update a parameter and persist to /config.txt
  // Parameters: TIBIA, FEMUR, COXA, TOL_TIBIA, TOL_OTHER, TIMEOUT
  // Values are integers (centideg except TIMEOUT in ms). COXA must remain 12000 (center) for safety.

  // Check for SET subcommand first
  int peekStart = s; int ts_set, tl_set; int afterFirst = nextToken(line, peekStart, len, &ts_set, &tl_set);
  if (tl_set > 0) {
    char firstTok[12]={0}; int n1=min(tl_set,11); memcpy(firstTok,line+ts_set,n1); firstTok[n1]=0; for(int i=0;i<n1;++i) firstTok[i]=(char)toupper(firstTok[i]);
    if (!strcmp(firstTok,"SET")) {
      // Parse param
      int ts_p, tl_p; afterFirst = nextToken(line, afterFirst, len, &ts_p, &tl_p); if (tl_p <= 0) { printERR(1,"BAD_ARG"); return; }
      char param[16]={0}; int np=min(tl_p,15); memcpy(param,line+ts_p,np); param[np]=0; for(int i=0;i<np;++i) param[i]=(char)toupper(param[i]);
      // Parse value
      int ts_v, tl_v; afterFirst = nextToken(line, afterFirst, len, &ts_v, &tl_v); if (tl_v <= 0) { printERR(1,"BAD_ARG"); return; }
      char valbuf[16]={0}; int nv=min(tl_v,15); memcpy(valbuf,line+ts_v,nv); valbuf[nv]=0; long v = atol(valbuf);
      bool changed=false; if (!strcmp(param,"TIBIA")) { if (v<0) v=0; if (v>24000) v=24000; g_tuck_tibia_cd=(int16_t)v; changed=true; }
      else if (!strcmp(param,"FEMUR")) { if (v<0) v=0; if (v>24000) v=24000; g_tuck_femur_cd=(int16_t)v; changed=true; }
      else if (!strcmp(param,"COXA")) { if (v!=12000) { printERR(1,"COXA_CENTER_ONLY"); return; } g_tuck_coxa_cd=12000; changed=true; }
      else if (!strcmp(param,"TOL_TIBIA")) { if (v<10) v=10; if (v>5000) v=5000; g_tuck_tol_tibia_cd=(int16_t)v; changed=true; }
      else if (!strcmp(param,"TOL_OTHER")) { if (v<10) v=10; if (v>5000) v=5000; g_tuck_tol_other_cd=(int16_t)v; changed=true; }
      else if (!strcmp(param,"TIMEOUT")) { if (v<250) v=250; if (v>10000) v=10000; g_tuck_timeout_ms=(uint16_t)v; changed=true; }
      else { printERR(1,"BAD_PARAM"); return; }
      if (!changed) return; // should not happen
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
      if (!SD.begin(BUILTIN_SDCARD)) return; // silent if no SD
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); printERR(2,"SD_WRITE_FAIL"); return; }
      const struct KeyVal { const char* key; } keys[6] = {
        {"tuck.tibia_cd="},{"tuck.femur_cd="},{"tuck.coxa_cd="},{"tuck.tol_tibia_cd="},{"tuck.tol_other_cd="},{"tuck.timeout_ms="}
      };
      bool replaced[6]={false,false,false,false,false,false};
      if (fin) {
        static char linebuf[256]; int llen=0; while (fin.available()) { int ch2=fin.read(); if (ch2<0) break; if (ch2=='\r') continue; if (ch2=='\n') { linebuf[llen]=0; const char* p=linebuf; while(*p==' '||*p=='\t')++p; bool any=false; for(int k=0;k<6;++k){ size_t klen=strlen(keys[k].key); if (strncmp(p,keys[k].key,klen)==0){ any=true; break; }} if (any) {
              // Rewrite all tuck keys once when first encountered to keep grouping
              if (!replaced[0]) { fout.print("tuck.tibia_cd="); fout.print((int)g_tuck_tibia_cd); fout.print('\n'); replaced[0]=true; }
              if (!replaced[1]) { fout.print("tuck.femur_cd="); fout.print((int)g_tuck_femur_cd); fout.print('\n'); replaced[1]=true; }
              if (!replaced[2]) { fout.print("tuck.coxa_cd="); fout.print((int)g_tuck_coxa_cd); fout.print('\n'); replaced[2]=true; }
              if (!replaced[3]) { fout.print("tuck.tol_tibia_cd="); fout.print((int)g_tuck_tol_tibia_cd); fout.print('\n'); replaced[3]=true; }
              if (!replaced[4]) { fout.print("tuck.tol_other_cd="); fout.print((int)g_tuck_tol_other_cd); fout.print('\n'); replaced[4]=true; }
              if (!replaced[5]) { fout.print("tuck.timeout_ms="); fout.print((int)g_tuck_timeout_ms); fout.print('\n'); replaced[5]=true; }
            } else {
              fout.print(linebuf); fout.print('\n');
            }
            llen=0; }
          else if (llen+1 < (int)sizeof(linebuf)) { linebuf[llen++]=(char)ch2; } else { llen=0; }
        }
      }
      // Append missing tuck keys
      if (!replaced[0]) { fout.print("tuck.tibia_cd="); fout.print((int)g_tuck_tibia_cd); fout.print('\n'); }
      if (!replaced[1]) { fout.print("tuck.femur_cd="); fout.print((int)g_tuck_femur_cd); fout.print('\n'); }
      if (!replaced[2]) { fout.print("tuck.coxa_cd="); fout.print((int)g_tuck_coxa_cd); fout.print('\n'); }
      if (!replaced[3]) { fout.print("tuck.tol_tibia_cd="); fout.print((int)g_tuck_tol_tibia_cd); fout.print('\n'); }
      if (!replaced[4]) { fout.print("tuck.tol_other_cd="); fout.print((int)g_tuck_tol_other_cd); fout.print('\n'); }
      if (!replaced[5]) { fout.print("tuck.timeout_ms="); fout.print((int)g_tuck_timeout_ms); fout.print('\n'); }
      if (fin) fin.close(); fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp","/config.txt");
#endif
      return; // SET path does not start sequence
    }
  }

  // Peek next token; if absent, all legs
  int s2 = nextToken(line, s, len, &ts, &tl);
  uint8_t mask = 0;
  if (tl <= 0) {
    // All legs
    for (uint8_t L=0; L<6; ++L) if (leg_enabled_mask_get(L)) mask |= (1u<<L);
  } else {
    char legt[4]={0}; int nl=min(tl,3); memcpy(legt,line+ts,nl); for(int i=0;i<nl;++i) legt[i]=toupper(legt[i]);
    int leg = legIndexFromToken(legt);
    if (leg < 0) { printERR(1,"BAD_LEG"); return; }
    if (leg_enabled_mask_get((uint8_t)leg)) mask |= (uint8_t)(1u<<leg);
    s = s2; // consume
  }
  if (mask == 0) { printERR(92, "NO_LEGS"); return; }
  g_tuck_active = 1;
  g_tuck_mask = mask;
  g_tuck_done_mask = 0;
  g_tuck_start_ms = millis();
  // Initial tibia commands (kick off sequence); femur/coxa set later on convergence
  for (uint8_t L=0; L<6; ++L) {
    if ((mask>>L)&1u) {
      if (joint_enabled_mask_get(L,2) && (((g_servo_oos_mask>>(L*3+2))&1u)==0)) {
        // Compute mirrored tibia target if right side
        bool isRight = (L>=3);
        int16_t tibia_target = g_tuck_tibia_cd;
        if (isRight) tibia_target = (int16_t)(2 * g_home_cd[L][2] - tibia_target);
        g_cmd_cd[L][2] = tibia_target;
      }
    }
  }
}

void processCmdSAFETY(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "CLEAR")) {
    if (!g_lockout) { printERR(91, "NOT_LOCKED"); return; }
    if ((g_lockout_causes & ~g_override_mask) != 0) { printERR(90, "VIOLATION"); return; }
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
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char which[16] = {0}; int nw = min(tl, 15); memcpy(which, line + ts, nw); for (int i = 0; i < nw; ++i) which[i] = toupper(which[i]);
    if      (!strcmp(which, "ALL"))       { g_override_mask = 0xFFFFu; }
    else if (!strcmp(which, "NONE"))      { g_override_mask = 0u; }
    else if (!strcmp(which, "TEMP"))      { g_override_mask |= (1u << 0); }
    else if (!strcmp(which, "COLLISION")) { g_override_mask |= (1u << 1); }
    else { printERR(1, "BAD_ARG"); return; }
    if (g_lockout && ((g_lockout_causes & ~g_override_mask) == 0)) g_lockout = false;
  return;
  }

  if (!strcmp(sub, "SOFTLIMITS")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_safety_soft_limits_enabled = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
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
          } else { fout.print(linebuf); fout.print('\n'); }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
        else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
      }
      fin.close();
    }
    if (!replaced) { fout.print("safety.soft_limits="); fout.print(g_safety_soft_limits_enabled ? F("true") : F("false")); fout.print('\n'); }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "COLLISION")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_safety_collision_enabled = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
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
          } else { fout.print(linebuf); fout.print('\n'); }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
        else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
      }
      fin.close();
    }
    if (!replaced) { fout.print("safety.collision="); fout.print(g_safety_collision_enabled ? F("true") : F("false")); fout.print('\n'); }
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "TEMPLOCK")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[16] = {0}; int nv = min(tl, 15); memcpy(vb, line + ts, nv); vb[nv] = 0;
    float c = atof(vb); if (c < 30.0f) c = 30.0f; if (c > 120.0f) c = 120.0f;
    int v = (int)lroundf(c * 10.0f); if (v < 0) v = 0; if (v > 32767) v = 32767;
    g_safety_temp_lockout_c10 = (int16_t)v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
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
          } else { fout.print(linebuf); fout.print('\n'); }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
        else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
      }
      fin.close();
    }
    if (!replaced) { fout.print("safety.temp_lockout_c="); fout.print((int)(g_safety_temp_lockout_c10 / 10)); fout.print('\n'); }
  fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  if (!strcmp(sub, "CLEARANCE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[24] = {0}; int nv = min(tl, 23); memcpy(vb, line + ts, nv); vb[nv] = 0;
    float v = atof(vb); if (v < 0.0f) v = 0.0f; if (v > 500.0f) v = 500.0f;
    g_safety_clearance_mm = v;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
    File fin = SD.open("/config.txt", FILE_READ);
    File fout = SD.open("/config.tmp", FILE_WRITE);
    if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
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
          } else { fout.print(linebuf); fout.print('\n'); }
          llen = 0;
        } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
        else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
      }
      fin.close();
    }
    if (!replaced) { fout.print("safety.clearance_mm="); long iv = lroundf(g_safety_clearance_mm); fout.print((int)iv); fout.print('\n'); }
  fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  printERR(1, "BAD_ARG");
}

void processCmdHOME(const char* line,int s,int len){ (void)line;(void)s;(void)len; for(int L=0;L<6;++L){ if(!leg_enabled_mask_get((uint8_t)L)) continue; for(int J=0;J<3;++J){ int idx=L*3+J; bool jointEn=joint_enabled_mask_get((uint8_t)L,(uint8_t)J); bool inService=((g_servo_oos_mask>>idx)&1u)==0; if(jointEn && inService) g_cmd_cd[L][J]=g_home_cd[L][J]; }} }

/**
 * @brief Calibrate and persistently save per-joint home pose and angle offsets to SD card.
 *
 * This command handler computes and writes a consistent set of per-servo calibration values
 * so that the robot’s current physical pose becomes the new logical “home” pose while
 * simultaneously re-centering each servo’s internal offset around a target raw-center.
 *
 * High-level behavior:
 * - Reads the current position (in centidegrees) of each enabled, in-service joint.
 * - Adjusts the servo’s persistent angle offset so the underlying raw mechanical center
 *   aligns with a fixed target (TARGET_CD = 12000 cd), within ±30° limits.
 * - Records the current physical joint position as the “home” pose without moving hardware.
 * - Updates in-memory calibration arrays and rewrites/merges corresponding entries in
 *   /config.txt on the SD card as lines of the form:
 *     home_cd.<LEG>.<JOINT>=<centidegrees>
 * - Emits a per-joint status line on the serial console.
 *
 * Preconditions:
 * - SD support must be compiled in and enabled (MARS_ENABLE_SD).
 * - The SD interface must initialize (SD.begin(BUILTIN_SDCARD) succeeds).
 * - The robot may be physically positioned into the desired home pose prior to invocation.
 * - Joint/leg enablement masks must reflect which servos are allowed to be processed:
 *   - leg_enabled_mask_get(L) identifies which legs are considered.
 *   - joint_enabled_mask_get(L, J) identifies which joints are considered.
 *   - g_servo_oos_mask marks joints out-of-service; those are skipped.
 *
 * Input parameters:
 * - line: Original command line (unused).
 * - s:    Start index (unused).
 * - len:  Command length (unused).
 *   All three are ignored in this handler and only present to conform to the command
 *   processor’s interface. They are explicitly silenced to avoid compiler warnings.
 *
 * Units and limits:
 * - Positions are handled as centidegrees (cd). 1 degree = 100 cd.
 * - Target raw center is TARGET_CD = 12000 cd (i.e., 120.00°).
 * - Persistent offset is stored in “units” where 1 unit = 24 cd = 0.24°.
 * - The servo offset is clamped to MAX_OFFSET_UNITS = ±125 (±3000 cd ≈ ±30.00°).
 *
 * Detailed algorithm:
 * 1) Servo iteration and filtering:
 *    - Enumerate all legs L in [0..5] and joints J in [0..2] mapping to:
 *      legs:  LF, LM, LR, RF, RM, RR
 *      joints: coxa, femur, tibia
 *    - Skip any leg or joint that is disabled or flagged out-of-service.
 *
 * 2) Position capture and offset computation for each processed joint:
 *    - Read the joint’s current position pos_cd in centidegrees; if the read fails (pos_cd < 0), skip.
 *    - Retrieve the servo’s current persistent offset in “units” and clamp to ±125.
 *    - Compute the current raw (un-offset) position:
 *        raw_cd = pos_cd - (current_units * 24)
 *    - Compute a rounded delta (in units) required to bring raw_cd to TARGET_CD:
 *        delta_units = cd_to_units_round(TARGET_CD - raw_cd)
 *    - Propose new persistent units:
 *        new_units = clamp(current_units + delta_units, -125, +125)
 *    - If the offset changed, apply the delta immediately and persist it:
 *        angle_offset_adjust(sid, applied_delta)
 *        angle_offset_write(sid)
 *    - Read back the stored value (final_units) and clamp for safety, then compute:
 *        final_offset_cd = final_units * 24
 *
 * 3) Home pose selection and in-memory state:
 *    - The “home” pose is defined as the robot’s current exact physical pose, not the
 *      computed target. Thus:
 *        residual_home_cd = pos_cd
 *    - Update globals per joint:
 *        g_home_cd[L][J]   = residual_home_cd
 *        g_offset_cd[L][J] = final_offset_cd
 *    - Collect a SaveInfo record to drive file rewriting and logging.
 *
 * 4) SD card configuration file update (/config.txt):
 *    - Open /config.txt for read if it exists; always open /config.tmp for write.
 *    - Stream the old file line-by-line with a 256-byte buffer:
 *      - Normalize CRLF to LF (ignore '\r').
 *      - Preserve unrelated lines verbatim.
 *      - For each processed joint, match keys of the form (leading whitespace ignored):
 *          "home_cd.<LEG>.<JOINT>="
 *        If matched, overwrite the line with the new value:
 *          home_cd.<LEG>.<JOINT>=<residual_home_cd>
 *        and mark it as replaced.
 *      - Lines longer than the buffer are flushed in chunks to avoid overflow; this preserves
 *        content but may split extremely long lines across writes (not typical for config keys).
 *    - After streaming, append any missing keys for joints that were not found:
 *        home_cd.<LEG>.<JOINT>=<residual_home_cd>
 *    - Close files, then replace the original by:
 *        close(tmp); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt")
 *      Note: This is a best-effort replacement; power loss between remove and rename can
 *      transiently leave only the .tmp or no file. Ensure robust power during calibration.
 *
 * 5) Logging:
 *    - For each processed joint, print a single-line status record to Serial:
 *        HOMESSET <LEG>.<JOINT> pos_cd=<pos> new_offset_cd=<final_offset_cd> home_cd=<residual_home_cd>
 *
 * Data format in /config.txt:
 * - Key names are stable and human-editable:
 *     home_cd.LF.coxa=<int>
 *     home_cd.LF.femur=<int>
 *     home_cd.LF.tibia=<int>
 *     ...
 *     home_cd.RR.tibia=<int>
 * - Values are int16 centidegrees representing the saved physical home pose, not the
 *   servo offset or target center.
 *
 * Side effects:
 * - Persists updated servo offset values to non-volatile storage per servo.
 * - Updates in-memory arrays g_home_cd and g_offset_cd used elsewhere at runtime.
 * - Rewrites /config.txt entries for all handled joints; preserves unrelated keys and comments.
 * - Emits Serial logs for visibility and diagnostics.
 *
 * Error handling and early exits:
 * - If SD is not enabled at compile time: printERR(2, "NO_SD") and return.
 * - If SD.begin() fails: printERR(2, "NO_SD") and return.
 * - If the temp output file cannot be created: printERR(2, "SD_WRITE_FAIL") and return.
 * - If /config.txt does not exist or cannot be read: the process still succeeds by creating
 *   a new /config.txt that includes only the needed home_cd.* lines (others are absent).
 * - Per-joint read failures (pos_cd < 0) silently skip that joint; no file updates for it.
 *
 * Safety and constraints:
 * - Offset clamping ensures servos never store offsets beyond ±30° equivalent (±125 units).
 * - The robot is not commanded to move; only persistent offsets and saved home values change.
 * - This routine is not re-entrant and assumes single-threaded invocation typical of MCU firmware.
 *
 * Complexity and resource usage:
 * - Processes up to 18 joints (6 legs × 3 joints).
 * - Uses a fixed 256-byte line buffer for file rewriting.
 * - SaveInfo array stores up to 18 entries with small POD records.
 *
 * Typical use:
 * - Manually place the robot into the desired neutral/home pose.
 * - Invoke this command to:
 *   1) Save the current pose as “home” per joint.
 *   2) Re-center servo offsets toward a known mechanical target center.
 *   3) Persist both the pose (to SD) and the offsets (to servo/EEPROM).
 *
 * @param line Unused command buffer pointer (ignored).
 * @param s    Unused command substring start index (ignored).
 * @param len  Unused command substring length (ignored).
 *
 * @return void
 *
 * @note The function treats leading spaces/tabs as insignificant when matching keys.
 * @note Keys are matched case-sensitively and must follow: home_cd.<LEG>.<JOINT>=
 * @note The target center (12000 cd) corresponds to 120.00 degrees by convention in this firmware.
 * @warning Power loss during file replacement may leave configuration in a transient state.
 * @warning Very long lines in /config.txt are handled conservatively; they are forwarded but may be
 *          chunked, which can be undesirable if such lines are not expected. Keep config lines short.
 */
void processCmdSAVEHOME(const char* line, int s, int len)
{
  (void)line; (void)s; (void)len; // unused
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }

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

  const char* legNames[6]   = { "LF","LM","LR","RF","RM","RR" };
  const char* jointNames[3] = { "coxa","femur","tibia" };

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
  if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }

  if (fin) {
    static char linebuf[256]; int llen = 0;
    while (fin.available()) {
      int ch = fin.read(); if (ch < 0) break;
      if (ch == '\r') continue;
      if (ch == '\n') {
        linebuf[llen] = 0;
        const char* p = linebuf; while (*p==' '||*p=='\t') ++p;
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
        if (!replaced) { fout.print(linebuf); fout.print('\n'); }
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
  if (tl <= 0) { printERR(1, "BAD_ARG"); return; }

  char sub[12] = {0};
  int ns = min(tl, 11);
  memcpy(sub, line + ts, ns);
  for (int i = 0; i < ns; ++i) sub[i] = (char)toupper(sub[i]);

  // OFFSET LIST
  if (!strcmp(sub, "LIST")) {
    const char* legNames[6] = { "LF","LM","LR","RF","RM","RR" };
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
    if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char legt[4] = {0};
    int nl = min(tl, 3);
    memcpy(legt, line + ts, nl);
    for (int i = 0; i < nl; ++i) legt[i] = (char)toupper(legt[i]);

    s = nextToken(line, s, len, &ts, &tl);
    if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char jtok[8] = {0};
    int nj = min(tl, 7);
    memcpy(jtok, line + ts, nj);
    for (int i = 0; i < nj; ++i) jtok[i] = (char)toupper(jtok[i]);

    const char* legNames[6]   = { "LF","LM","LR","RF","RM","RR" };
    const char* jointNames[3] = { "COXA","FEMUR","TIBIA" };

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
        if (J < 0) { printERR(1, "BAD_JOINT"); return; }
        for (int L = 0; L < 6; ++L) clear_one(L, J);
      }
    } else {
      int leg = legIndexFromToken(legt);
      if (leg < 0) { printERR(1, "BAD_LEG"); return; }
      if (!strcmp(jtok, "ALL")) {
        for (int J = 0; J < 3; ++J) clear_one(leg, J);
      } else {
        int J = jointIndexFromToken(jtok);
        if (J < 0) { printERR(1, "BAD_JOINT"); return; }
        clear_one(leg, J);
      }
    }
    return;
  }

  printERR(1, "BAD_ARG");
}

// LOG <ENABLE|RATE|MODE|HEADER|FLUSH|STATUS> [...]
void processCmdLOG(const char* line,int s,int len)
{
#if !(defined(MARS_ENABLE_SD) && MARS_ENABLE_SD)
  (void)line; (void)s; (void)len; printERR(2, "NO_SD"); return;
#else
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char sub[10] = {0}; int ns = min(tl, 9); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  // New syntax: LOG ENABLE (no argument) turns logging on; LOG DISABLE turns it off.
  // Backwards compatibility: LOG ENABLE <ON|OFF> still supported.
  if (!strcmp(sub, "ENABLE") || !strcmp(sub, "DISABLE")) {
  bool request_enable = !strcmp(sub, "ENABLE");
  // Peek optional token if ENABLE form used
    int ts2, tl2; int s2 = nextToken(line, s, len, &ts2, &tl2);
    if (request_enable && tl2 > 0) {
      char vb[8] = {0}; int nv = min(tl2, 7); memcpy(vb, line + ts2, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
      if (!strcmp(vb, "OFF") || !strcmp(vb, "FALSE") || !strcmp(vb, "0")) { request_enable = false; }
      else if (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1")) { request_enable = true; }
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
        if (g_log_buf_used && g_log_file) { g_log_file.write(g_log_buf, g_log_buf_used); g_log_buf_used = 0; }
        if (g_log_file) { g_log_file.flush(); g_log_file.close(); }
      }
    }
#else
    if (request_enable) { printERR(2, "NO_SD"); return; }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "RATE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char nb[12] = {0}; int nn = min(tl, 11); memcpy(nb, line + ts, nn); nb[nn] = 0; long hz = atol(nb);
    if (hz < 1) hz = 1; if (hz > 500) hz = 500;
    g_log_rate_hz = (uint16_t)hz;
    // recompute divisor (integer)
    if (g_loop_hz > 0 && g_log_rate_hz > 0 && g_loop_hz >= g_log_rate_hz) {
      uint16_t div = g_loop_hz / g_log_rate_hz; if (div < 1) div = 1; if (div > 255) div = 255; g_log_sample_div = (uint8_t)div;
    } else { g_log_sample_div = 1; }
    // persist: logging.rate_hz
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { /* ignore persist error */ return; }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); /* ignore persist error */ return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.rate_hz="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) { fout.print(key); fout.print((unsigned int)g_log_rate_hz); fout.print('\n'); replaced = true; }
            else { fout.print(linebuf); fout.print('\n'); }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
          else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
        }
        fin.close();
      }
      if (!replaced) { fout.print("logging.rate_hz="); fout.print((unsigned int)g_log_rate_hz); fout.print('\n'); }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "MODE")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char mtok[10] = {0}; int nm = min(tl, 9); memcpy(mtok, line + ts, nm); for (int i = 0; i < nm; ++i) mtok[i] = toupper(mtok[i]);
    uint8_t m = 0; if (!strcmp(mtok, "FULL") || !strcmp(mtok, "1")) m = 1; else m = 0; g_log_mode = m;
    // persist: logging.mode (numeric 0|1)
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { /* ignore persist error */ return; }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.mode="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) { fout.print(key); fout.print((unsigned int)g_log_mode); fout.print('\n'); replaced = true; }
            else { fout.print(linebuf); fout.print('\n'); }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
          else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
        }
        fin.close();
      }
      if (!replaced) { fout.print("logging.mode="); fout.print((unsigned int)g_log_mode); fout.print('\n'); }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "HEADER")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[8] = {0}; int nv = min(tl, 7); memcpy(vb, line + ts, nv); for (int i = 0; i < nv; ++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb, "ON") || !strcmp(vb, "TRUE") || !strcmp(vb, "1"));
    g_log_header = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { return; }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.header="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) { fout.print(key); fout.print(g_log_header ? 1 : 0); fout.print('\n'); replaced = true; }
            else { fout.print(linebuf); fout.print('\n'); }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
          else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
        }
        fin.close();
      }
      if (!replaced) { fout.print("logging.header="); fout.print(g_log_header ? 1 : 0); fout.print('\n'); }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    /* success */ return;
  }

  if (!strcmp(sub, "FLUSH")) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (g_log_file) {
      if (g_log_buf_used) { g_log_file.write(g_log_buf, g_log_buf_used); g_log_buf_used = 0; }
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
      if (g_log_buf_used) { g_log_file.write(g_log_buf, g_log_buf_used); g_log_buf_used = 0; }
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
                if (sz >= newestTime) { strncpy(newestName, n, sizeof(newestName)-1); newestTime = sz; }
              }
            }
            f.close();
          }
          dir.close();
          if (newestName[0]) { SD.remove(newestName); }
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
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char vb[8] = {0}; int nv = min(tl,7); memcpy(vb, line+ts, nv); for(int i=0;i<nv;++i) vb[i] = toupper(vb[i]);
    bool on = (!strcmp(vb,"ON")||!strcmp(vb,"TRUE")||!strcmp(vb,"1"));
    g_log_rotate = on;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { return; }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.rotate="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) { fout.print(key); fout.print(g_log_rotate ? 1 : 0); fout.print('\n'); replaced = true; }
            else { fout.print(linebuf); fout.print('\n'); }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
          else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
        }
        fin.close();
      }
      if (!replaced) { fout.print("logging.rotate="); fout.print(g_log_rotate ? 1 : 0); fout.print('\n'); }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    return;
  }

  if (!strcmp(sub, "MAXKB")) {
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char nb[16] = {0}; int nn = min(tl,15); memcpy(nb, line+ts, nn); nb[nn]=0; long kb = atol(nb);
    if (kb < 100) kb = 100; // minimum 100KB
    long maxKBClamp = 1024L * 1024L; // 1GB in KB
    if (kb > maxKBClamp) kb = maxKBClamp; // hard clamp to 1GB
    g_log_max_bytes = (uint32_t)kb * 1024UL;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    if (!SD.begin(BUILTIN_SDCARD)) { return; }
    {
      File fin = SD.open("/config.txt", FILE_READ);
      File fout = SD.open("/config.tmp", FILE_WRITE);
      if (!fout) { if (fin) fin.close(); return; }
      bool replaced = false;
      if (fin) {
        static char linebuf[256]; int llen = 0;
        while (fin.available()) {
          int ch2 = fin.read(); if (ch2 < 0) break; if (ch2 == '\r') continue;
          if (ch2 == '\n') {
            linebuf[llen] = 0; const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
            const char* key = "logging.max_kb="; size_t klen = strlen(key);
            if (strncmp(p, key, klen) == 0) { fout.print(key); fout.print((unsigned long)(g_log_max_bytes / 1024UL)); fout.print('\n'); replaced = true; }
            else { fout.print(linebuf); fout.print('\n'); }
            llen = 0;
          } else if (llen + 1 < (int)sizeof(linebuf)) { linebuf[llen++] = (char)ch2; }
          else { linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0; }
        }
        fin.close();
      }
      if (!replaced) { fout.print("logging.max_kb="); fout.print((unsigned long)(g_log_max_bytes / 1024UL)); fout.print('\n'); }
      fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
    }
#endif
    return;
  }

  if (!strcmp(sub, "TAIL")) {
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
    // Parse N
    s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
    char nb[12] = {0}; int nn = min(tl, 11); memcpy(nb, line + ts, nn); nb[nn] = 0; long want = atol(nb);
    if (want < 1) want = 1; if (want > 100) want = 100; // bound to avoid excessive prints

    // Ensure we have a file
    if (!g_log_file) { printERR(3, "NO_FILE"); return; }
    // Flush any staged buffer
    if (g_log_buf_used) { g_log_file.write(g_log_buf, g_log_buf_used); g_log_buf_used = 0; }
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
        if (c0 <= 0 || c0 == '\n' || c0 == '\r' || c0 == '#') { continue; }
        if (c0 == 't') {
          // Possible header line
          char hb[9] = {0}; hb[0] = 't';
          for (int i = 1; i < 9; ++i) { int ci = g_log_file.read(); if (ci < 0) { hb[i] = 0; break; } hb[i] = (char)ci; }
          if (!strncmp(hb, "time_ms,", 8)) { continue; }
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
        if (c0 != 't') { starts[found++] = 0; }
        else {
          char hb[9] = {0}; hb[0] = 't'; for (int i = 1; i < 9; ++i) { int ci = g_log_file.read(); if (ci < 0) { hb[i] = 0; break; } hb[i] = (char)ci; }
          if (strncmp(hb, "time_ms,", 8) != 0) { starts[found++] = 0; }
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

  printERR(1, "BAD_ARG");
#endif
}

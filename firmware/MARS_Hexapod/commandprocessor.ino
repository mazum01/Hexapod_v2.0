// commandprocessor.ino — Modular command handling (refactored)
// Each top-level serial command now handled by a dedicated routine.
// Keeps core parser minimal and enables switch-based dispatch.

#include <Arduino.h>
#include <ctype.h>
#include <lx16a-servo.h>
#include "robot_config.h"
#include "command_types.h"
#include "command_helpers.h"

// Externs from other compilation units (defined in MARS_Hexapod.ino / functions.ino)
extern void printHELP();
extern void printSTATUS();
extern void rebootNow();
extern void modeSetTest();
extern void modeSetIdle();
extern bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]);
extern int16_t readServoPosCdSync(uint8_t leg, uint8_t joint);
extern int nextToken(const char* s, int start, int len, int* tokStart, int* tokLen);
extern void printOK();
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


// Hardware angle offset helpers (provided by lx16a-servo library or stubs)
extern int  angle_offset_read(uint8_t id);
extern bool angle_offset_adjust(uint8_t id, int8_t delta);
extern bool angle_offset_write(uint8_t id);

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
  return CMD_UNKNOWN;
}

// -------------------------------------------------------------------------------------------------
// Per-command routines (minimal wrappers preserving original semantics)
// Each function receives: original line, current parse index (start after command token), and total length.
// -------------------------------------------------------------------------------------------------

void processCmdHELP(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; printHELP(); printOK(); }
void processCmdSTATUS(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; printSTATUS(); printOK(); }
void processCmdREBOOT(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; printOK(); Serial.flush(); delay(10); rebootNow(); }
void processCmdENABLE(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; g_enabled=true; g_last_err=0; printOK(); }
void processCmdDISABLE(const char* line,int s,int len)
{ (void)line;(void)s;(void)len; g_enabled=false; g_last_err=0; for(uint8_t L=0;L<6;++L){ for(uint8_t J=0;J<3;++J) setServoTorqueNow(L,J,false);} printOK(); }

void processCmdFK(const char* line,int s,int len)
{ int ts,tl; s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char legt[4]={0}; int nl=min(tl,3); memcpy(legt,line+ts,nl); for(int i=0;i<nl;++i) legt[i]=toupper(legt[i]); s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char vb[8]={0}; int nv=min(tl,7); memcpy(vb,line+ts,nv); for(int i=0;i<nv;++i) vb[i]=toupper(vb[i]); bool on = (!strcmp(vb,"ON")||!strcmp(vb,"TRUE")||!strcmp(vb,"1")); if(!strcmp(legt,"ALL")){ if(on) g_fk_stream_mask|=0x3Fu; else g_fk_stream_mask&=~0x3Fu; printOK(); return;} int leg=legIndexFromToken(legt); if(leg<0){printERR(1,"BAD_LEG");return;} if(on) g_fk_stream_mask|=(uint8_t)(1u<<leg); else g_fk_stream_mask&=(uint8_t)~(1u<<leg); printOK(); }

void processCmdLEGS(const char* line,int s,int len){ (void)line;(void)s;(void)len; const char* names[6]={"LF","LM","LR","RF","RM","RR"}; Serial.print(F("LEGS ")); for(int i=0;i<6;++i){ if(i) Serial.print(F(" ")); Serial.print(names[i]); Serial.print(F("=")); Serial.print(leg_enabled_mask_get((uint8_t)i)?1:0);} Serial.print(F("\r\n")); printOK(); }

void processCmdSERVOS(const char* line,int s,int len){ (void)line;(void)s;(void)len; const char* names[6]={"LF","LM","LR","RF","RM","RR"}; Serial.print(F("SERVOS ")); for(int i=0;i<6;++i){ if(i) Serial.print(F(" ")); Serial.print(names[i]); Serial.print(F("=")); for(int j=0;j<3;++j) Serial.print(joint_enabled_mask_get((uint8_t)i,(uint8_t)j)?1:0);} Serial.print(F("\r\n")); printOK(); }

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

  printOK();
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
  printOK();
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
  printOK();
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
  printOK();
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
  printOK();
}

void processCmdFEET(const char* line,int s,int len){ int ts,tl; float p[6][3]; for(int i=0;i<6;++i){ for(int j=0;j<3;++j){ s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char nb[24]={0}; int nn=min(tl,23); memcpy(nb,line+ts,nn); nb[nn]=0; p[i][j]=atof(nb);} } for(int L=0;L<6;++L){ int16_t out[3]; if(!calculateIK((uint8_t)L,p[L][0],p[L][1],p[L][2],out)){ printERR(1,"IK_FAIL"); return;} bool isRight=(L>=3); if(isRight){ g_cmd_cd[L][0]=out[0]; g_cmd_cd[L][1]=out[1]; g_cmd_cd[L][2]=out[2]; } else { g_cmd_cd[L][0]=(int16_t)(2*g_home_cd[L][0]-out[0]); g_cmd_cd[L][1]=(int16_t)(2*g_home_cd[L][1]-out[1]); g_cmd_cd[L][2]=(int16_t)(2*g_home_cd[L][2]-out[2]); } } printOK(); }

void processCmdMODE(const char* line,int s,int len){ int ts,tl; s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char mtok[8]={0}; int nm=min(tl,7); memcpy(mtok,line+ts,nm); for(int i=0;i<nm;++i) mtok[i]=toupper(mtok[i]); if(!strcmp(mtok,"TEST")){ modeSetTest(); printOK(); return;} if(!strcmp(mtok,"IDLE")){ modeSetIdle(); printOK(); return;} printERR(1,"BAD_ARG"); }
void processCmdI(const char* line,int s,int len){ (void)line;(void)s;(void)len; modeSetIdle(); printOK(); }
void processCmdT(const char* line,int s,int len){ (void)line;(void)s;(void)len; modeSetTest(); printOK(); }

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
    g_test_cycle_ms = (uint32_t)ms; printOK(); return;
  }
  if (!strcmp(sub, "HEIGHT")) { g_test_base_y_mm = atof(vb); printOK(); return; }
  if (!strcmp(sub, "BASEX"))  { g_test_base_x_mm = atof(vb); printOK(); return; }
  if (!strcmp(sub, "STEPLEN")) {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_step_len_mm = v; printOK(); return;
  }
  if (!strcmp(sub, "LIFT"))    {
    float v = atof(vb); if (v < 0.0f) v = 0.0f; g_test_lift_y_mm = v; printOK(); return;
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
    fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt"); printOK(); return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  printERR(1, "BAD_ARG");
}

void processCmdSTAND(const char* line,int s,int len){ (void)line;(void)s;(void)len; const float BASE_Y=g_test_base_y_mm; const float BASE_X=g_test_base_x_mm; for(int L=0;L<6;++L){ int16_t out[3]; bool ok=calculateIK((uint8_t)L,BASE_X,BASE_Y,0.0f,out); if(!ok){ printERR(1,"IK_FAIL"); return;} bool isRight=(L>=3); if(isRight){ g_cmd_cd[L][0]=out[0]; g_cmd_cd[L][1]=out[1]; g_cmd_cd[L][2]=out[2]; } else { g_cmd_cd[L][0]=(int16_t)(2*g_home_cd[L][0]-out[0]); g_cmd_cd[L][1]=(int16_t)(2*g_home_cd[L][1]-out[1]); g_cmd_cd[L][2]=(int16_t)(2*g_home_cd[L][2]-out[2]); } g_foot_target_x_mm[L]=BASE_X; g_foot_target_z_mm[L]=0.0f; } printOK(); }

void processCmdSAFETY(const char* line, int s, int len)
{
  int ts, tl;
  s = nextToken(line, s, len, &ts, &tl); if (tl <= 0) { printERR(1, "BAD_ARG"); return; }
  char sub[16] = {0}; int ns = min(tl, 15); memcpy(sub, line + ts, ns); for (int i = 0; i < ns; ++i) sub[i] = toupper(sub[i]);

  if (!strcmp(sub, "CLEAR")) {
    if (!g_lockout) { printERR(91, "NOT_LOCKED"); return; }
    if ((g_lockout_causes & ~g_override_mask) != 0) { printERR(90, "VIOLATION"); return; }
    g_lockout = false; g_lockout_causes = 0; printOK(); return;
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
    printOK(); return;
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
    printOK(); return;
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
    printOK(); return;
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
    printOK(); return;
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
    printOK(); return;
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
    printOK(); return;
#else
    printERR(2, "NO_SD"); return;
#endif
  }

  printERR(1, "BAD_ARG");
}

void processCmdHOME(const char* line,int s,int len){ (void)line;(void)s;(void)len; for(int L=0;L<6;++L){ if(!leg_enabled_mask_get((uint8_t)L)) continue; for(int J=0;J<3;++J){ int idx=L*3+J; bool jointEn=joint_enabled_mask_get((uint8_t)L,(uint8_t)J); bool inService=((g_servo_oos_mask>>idx)&1u)==0; if(jointEn && inService) g_cmd_cd[L][J]=g_home_cd[L][J]; }} printOK(); }

void processCmdSAVEHOME(const char* line, int s, int len)
{
  (void)line; (void)s; (void)len;
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  if (!SD.begin(BUILTIN_SDCARD)) { printERR(2, "NO_SD"); return; }
  const int TARGET_CD = 12000; // center
  const int MAX_OFFSET_UNITS = 125; // ±30°
  struct SaveInfo { uint8_t L; uint8_t J; int16_t pos_cd; int16_t new_units; int16_t final_offset_cd; int16_t residual_home_cd; bool replaced; };
  SaveInfo infos[18]; int infoCount = 0;
  const char* legNames[6]   = { "LF","LM","LR","RF","RM","RR" };
  const char* jointNames[3] = { "coxa","femur","tibia" };

  for (uint8_t L = 0; L < 6; ++L) {
    if (!leg_enabled_mask_get(L)) continue;
    for (uint8_t J = 0; J < 3; ++J) {
      int idx = L * 3 + J;
      bool jointEn = joint_enabled_mask_get(L, J);
      bool inService = ((g_servo_oos_mask >> idx) & 1u) == 0;
      if (!jointEn || !inService) continue;
      int16_t pos_cd = readServoPosCdSync(L, J); if (pos_cd < 0) continue;
      uint8_t sid = servoId(L, J);
      int current_units = angle_offset_read(sid);
      if (current_units < -MAX_OFFSET_UNITS) current_units = -MAX_OFFSET_UNITS;
      if (current_units >  MAX_OFFSET_UNITS) current_units =  MAX_OFFSET_UNITS;
      int raw_cd = pos_cd - current_units * 24;
      int delta_units = cd_to_units_round(TARGET_CD - raw_cd);
      int new_units = current_units + delta_units;
      if (new_units < -MAX_OFFSET_UNITS) new_units = -MAX_OFFSET_UNITS;
      if (new_units >  MAX_OFFSET_UNITS) new_units =  MAX_OFFSET_UNITS;
      int applied_delta = new_units - current_units;
      if (applied_delta != 0) { angle_offset_adjust(sid, (int8_t)applied_delta); angle_offset_write(sid); }
      int final_units = angle_offset_read(sid);
      if (final_units < -MAX_OFFSET_UNITS) final_units = -MAX_OFFSET_UNITS;
      if (final_units >  MAX_OFFSET_UNITS) final_units =  MAX_OFFSET_UNITS;
      int final_offset_cd = final_units * 24;
      int residual_home_cd = pos_cd; // preserve exact pose
      g_home_cd[L][J] = (int16_t)residual_home_cd;
      g_offset_cd[L][J] = (int16_t)final_offset_cd;
      infos[infoCount++] = { L, J, pos_cd, (int16_t)new_units, (int16_t)final_offset_cd, (int16_t)residual_home_cd, false };
    }
  }

  File fin = SD.open("/config.txt", FILE_READ);
  File fout = SD.open("/config.tmp", FILE_WRITE);
  if (!fout) { if (fin) fin.close(); printERR(2, "SD_WRITE_FAIL"); return; }
  if (fin) {
    static char linebuf[256]; int llen = 0;
    while (fin.available()) {
      int ch = fin.read(); if (ch < 0) break; if (ch == '\r') continue;
      if (ch == '\n') {
        linebuf[llen] = 0; bool didReplace = false;
        const char* p = linebuf; while (*p == ' ' || *p == '\t') ++p;
        for (int i = 0; i < infoCount; ++i) {
          char key[48]; snprintf(key, sizeof(key), "home_cd.%s.%s=", legNames[infos[i].L], jointNames[infos[i].J]);
          if (strncmp(p, key, strlen(key)) == 0) {
            fout.print(key); fout.print((int)infos[i].residual_home_cd); fout.print('\n'); infos[i].replaced = true; didReplace = true; break;
          }
        }
        if (!didReplace) { fout.print(linebuf); fout.print('\n'); }
        llen = 0;
      } else if (llen + 1 < (int)sizeof(linebuf)) {
        linebuf[llen++] = (char)ch;
      } else {
        linebuf[llen] = 0; fout.print(linebuf); fout.print('\n'); llen = 0;
      }
    }
    fin.close();
  }
  for (int i = 0; i < infoCount; ++i) {
    if (!infos[i].replaced) {
      fout.print("home_cd."); fout.print(legNames[infos[i].L]); fout.print("."); fout.print(jointNames[infos[i].J]); fout.print("="); fout.print((int)infos[i].residual_home_cd); fout.print('\n');
    }
  }
  fout.close(); SD.remove("/config.txt"); SD.rename("/config.tmp", "/config.txt");
  for (int i = 0; i < infoCount; ++i) {
    Serial.print(F("HOMESSET ")); Serial.print(legNames[infos[i].L]); Serial.print(F(".")); Serial.print(jointNames[infos[i].J]);
    Serial.print(F(" pos_cd=")); Serial.print((int)infos[i].pos_cd);
    Serial.print(F(" new_offset_cd=")); Serial.print((int)infos[i].final_offset_cd);
    Serial.print(F(" home_cd=")); Serial.print((int)infos[i].residual_home_cd);
    Serial.print(F("\r\n"));
  }
  printOK();
#else
  printERR(2, "NO_SD");
#endif
}

void processCmdOFFSET(const char* line,int s,int len){ int ts,tl; s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char sub[12]={0}; int ns=min(tl,11); memcpy(sub,line+ts,ns); for(int i=0;i<ns;++i) sub[i]=toupper(sub[i]); if(!strcmp(sub,"LIST")){ const char* legNames[6]={"LF","LM","LR","RF","RM","RR"}; for(int L=0;L<6;++L){ for(int J=0;J<3;++J){ uint8_t sid=servoId((uint8_t)L,(uint8_t)J); int units=angle_offset_read(sid); if(units<-125) units=-125; if(units>125) units=125; g_offset_cd[L][J]=units_to_cd(units);} } Serial.print(F("OFFSET ")); for(int L=0;L<6;++L){ if(L) Serial.print(F(" ")); Serial.print(legNames[L]); Serial.print(F("=")); for(int J=0;J<3;++J){ if(J) Serial.print(F("/")); Serial.print((int)g_offset_cd[L][J]); } } Serial.print(F("\r\n")); printOK(); return; }
 if(!strcmp(sub,"CLEAR")){ s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char legt[4]={0}; int nl=min(tl,3); memcpy(legt,line+ts,nl); for(int i=0;i<nl;++i) legt[i]=toupper(legt[i]); s=nextToken(line,s,len,&ts,&tl); if(tl<=0){printERR(1,"BAD_ARG");return;} char jtok[8]={0}; int nj=min(tl,7); memcpy(jtok,line+ts,nj); for(int i=0;i<nj;++i) jtok[i]=toupper(jtok[i]); const char* legNames[6]={"LF","LM","LR","RF","RM","RR"}; const char* jointNames[3]={"COXA","FEMUR","TIBIA"}; auto clear_one=[&](int L,int J){ if(L<0||L>=6||J<0||J>=3) return; uint8_t sid=servoId((uint8_t)L,(uint8_t)J); int cu=angle_offset_read(sid); if(cu!=0){ angle_offset_adjust(sid,(int8_t)(-cu)); angle_offset_write(sid); } g_offset_cd[L][J]=0; int16_t pos_cd=readServoPosCdSync((uint8_t)L,(uint8_t)J); if(pos_cd>=0) g_home_cd[L][J]=pos_cd; Serial.print(F("OFFSETCLR ")); Serial.print(legNames[L]); Serial.print(F(".")); Serial.print(jointNames[J]); Serial.print(F(" home_cd=")); Serial.print((int)g_home_cd[L][J]); Serial.print(F("\r\n")); }; if(!strcmp(legt,"ALL")){ if(!strcmp(jtok,"ALL")){ for(int L=0;L<6;++L) for(int J=0;J<3;++J) clear_one(L,J);} else { int J=jointIndexFromToken(jtok); if(J<0){printERR(1,"BAD_JOINT");return;} for(int L=0;L<6;++L) clear_one(L,J);} } else { int leg=legIndexFromToken(legt); if(leg<0){printERR(1,"BAD_LEG");return;} if(!strcmp(jtok,"ALL")){ for(int J=0;J<3;++J) clear_one(leg,J);} else { int J=jointIndexFromToken(jtok); if(J<0){printERR(1,"BAD_JOINT");return;} clear_one(leg,J);} } printOK(); return; }
 printERR(1,"BAD_ARG"); }

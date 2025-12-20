// =============================================================================
// globals.h — Centralized extern declarations for MARS Hexapod firmware
// Consolidates ~100+ scattered extern declarations from functions.ino and
// commandprocessor.ino into a single authoritative header.
// FW 0.2.43 — Created 2025-12-18
// =============================================================================
#ifndef MARS_GLOBALS_H
#define MARS_GLOBALS_H

#include <Arduino.h>
#include <stdint.h>
#include "robot_config.h"

// Forward declare MarsImpedance to avoid full header include here
class MarsImpedance;
class LX16AServo;  // Forward declare for g_servo array

// SD support conditional
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#include <SD.h>
#endif

// -----------------------------------------------------------------------------
// Compile-time configuration constants (exposed for shared use)
// -----------------------------------------------------------------------------
#ifndef NUM_LEGS
#define NUM_LEGS 6
#endif
#ifndef LEG_SERVOS
#define LEG_SERVOS 3
#endif

// -----------------------------------------------------------------------------
// System state globals (defined in MARS_Hexapod.ino)
// -----------------------------------------------------------------------------
extern volatile bool     g_enabled;
extern volatile bool     g_lockout;
extern volatile uint16_t g_lockout_causes;
extern volatile uint16_t g_override_mask;
extern volatile uint8_t  g_last_err;
extern volatile uint32_t g_overrun_count;
extern volatile uint64_t g_uptime_ms64;
extern volatile uint16_t g_loop_hz;
extern uint16_t          g_config_loop_hz;
extern bool              g_config_loaded;
extern uint16_t          g_config_keys_applied;
extern volatile uint8_t  g_uart_cfg_seen_mask;
extern volatile bool     g_uart_cfg_present;
extern volatile bool     g_uart_cfg_match;

// -----------------------------------------------------------------------------
// Servo hardware pointers
// -----------------------------------------------------------------------------
extern LX16AServo*       g_servo[NUM_LEGS][LEG_SERVOS];

// -----------------------------------------------------------------------------
// Enable/disable masks (leg and joint)
// -----------------------------------------------------------------------------
extern volatile uint8_t  g_leg_enabled_mask;
extern volatile uint32_t g_joint_enabled_mask;
extern volatile uint32_t g_servo_oos_mask;
extern volatile uint8_t  g_rr_index;
extern volatile uint8_t  g_servo_fb_fail_threshold;

// -----------------------------------------------------------------------------
// Joint command, home, offset, and limit state
// -----------------------------------------------------------------------------
extern volatile int16_t  g_cmd_cd[NUM_LEGS][LEG_SERVOS];
extern int16_t           g_eff_cmd_cd[NUM_LEGS][LEG_SERVOS];
extern volatile int16_t  g_home_cd[NUM_LEGS][LEG_SERVOS];
extern volatile int16_t  g_last_sent_cd[NUM_LEGS][LEG_SERVOS];
extern volatile int16_t  g_limit_min_cd[NUM_LEGS][LEG_SERVOS];
extern volatile int16_t  g_limit_max_cd[NUM_LEGS][LEG_SERVOS];
extern int16_t           g_offset_cd[NUM_LEGS][LEG_SERVOS];
extern volatile uint16_t g_rate_limit_cdeg_per_s;

// -----------------------------------------------------------------------------
// Measurement state (updated round-robin)
// -----------------------------------------------------------------------------
extern uint16_t          g_meas_vin_mV[NUM_LEGS][LEG_SERVOS];
extern uint8_t           g_meas_temp_C[NUM_LEGS][LEG_SERVOS];
extern int16_t           g_meas_pos_cd[NUM_LEGS][LEG_SERVOS];
extern volatile uint8_t  g_meas_pos_valid[NUM_LEGS][LEG_SERVOS];

// -----------------------------------------------------------------------------
// TUCK command state
// -----------------------------------------------------------------------------
extern volatile uint8_t  g_tuck_active;
extern volatile uint8_t  g_tuck_mask;
extern volatile uint8_t  g_tuck_done_mask;
extern volatile int16_t  g_tuck_tibia_cd;
extern volatile int16_t  g_tuck_femur_cd;
extern volatile int16_t  g_tuck_coxa_cd;
extern volatile int16_t  g_tuck_tol_tibia_cd;
extern volatile int16_t  g_tuck_tol_other_cd;
extern volatile uint16_t g_tuck_timeout_ms;

// -----------------------------------------------------------------------------
// Test mode gait parameters
// -----------------------------------------------------------------------------
extern float             g_test_base_y_mm;
extern float             g_test_base_x_mm;
extern float             g_test_step_len_mm;
extern uint32_t          g_test_cycle_ms;
extern float             g_test_lift_y_mm;
extern float             g_test_overlap_pct;

// -----------------------------------------------------------------------------
// Safety state
// -----------------------------------------------------------------------------
extern volatile bool     g_safety_soft_limits_enabled;
extern volatile bool     g_safety_collision_enabled;
extern volatile int16_t  g_safety_temp_lockout_c10;
extern float             g_safety_clearance_mm;

// -----------------------------------------------------------------------------
// FK/foot state
// -----------------------------------------------------------------------------
extern volatile uint8_t  g_fk_stream_mask;
extern float             g_foot_target_x_mm[NUM_LEGS];
extern float             g_foot_target_z_mm[NUM_LEGS];
extern float             g_foot_body_x_mm[NUM_LEGS];
extern float             g_foot_body_y_mm[NUM_LEGS];
extern float             g_foot_body_z_mm[NUM_LEGS];

// -----------------------------------------------------------------------------
// PID controller state
// -----------------------------------------------------------------------------
extern volatile bool     g_pid_enabled;
extern volatile uint16_t g_pid_kp_milli[LEG_SERVOS];
extern volatile uint16_t g_pid_ki_milli[LEG_SERVOS];
extern volatile uint16_t g_pid_kd_milli[LEG_SERVOS];
extern volatile uint16_t g_pid_kd_alpha_milli[LEG_SERVOS];
extern volatile uint8_t  g_pid_mode;           // 0=active, 1=shadow
extern volatile uint16_t g_pid_shadow_report_hz;

// -----------------------------------------------------------------------------
// Estimator state
// -----------------------------------------------------------------------------
extern volatile uint16_t g_est_cmd_alpha_milli;
extern volatile uint16_t g_est_meas_alpha_milli;
extern volatile uint16_t g_est_meas_vel_alpha_milli;

// -----------------------------------------------------------------------------
// Impedance controller (instance lives in MARS_Hexapod.ino)
// -----------------------------------------------------------------------------
extern MarsImpedance     g_impedance;

// -----------------------------------------------------------------------------
// Joint compliance state
// -----------------------------------------------------------------------------
extern bool              g_comp_enabled;
extern uint16_t          g_comp_kp_milli[LEG_SERVOS];
extern uint16_t          g_comp_range_cd[LEG_SERVOS];
extern uint16_t          g_comp_deadband_cd[LEG_SERVOS];
extern uint16_t          g_comp_leak_milli;
extern int16_t           g_comp_offset_cd[NUM_LEGS][LEG_SERVOS];

// -----------------------------------------------------------------------------
// Telemetry state
// -----------------------------------------------------------------------------
extern volatile uint8_t  g_telem_enabled;
extern volatile uint8_t  g_telem_bin_enabled;

// -----------------------------------------------------------------------------
// SD logging state (conditionally compiled)
// -----------------------------------------------------------------------------
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
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

// -----------------------------------------------------------------------------
// Timing probes (conditionally compiled)
// -----------------------------------------------------------------------------
#if MARS_TIMING_PROBES
extern volatile uint16_t g_probe_serial_us;
extern volatile uint16_t g_probe_send_us;
extern volatile uint16_t g_probe_fb_us;
extern volatile uint16_t g_probe_log_us;
extern volatile uint16_t g_probe_tick_us;
extern volatile uint16_t g_jitter_min_us;
extern volatile uint16_t g_jitter_max_us;
extern volatile uint32_t g_jitter_sum_us;
extern volatile uint16_t g_jitter_count;
#endif

// -----------------------------------------------------------------------------
// Serial command buffer
// -----------------------------------------------------------------------------
extern char              lineBuf[160];
extern uint8_t           lineLen;

// -----------------------------------------------------------------------------
// Function externs — implemented in MARS_Hexapod.ino
// -----------------------------------------------------------------------------
extern void     modeSetTest();
extern void     modeSetIdle();
extern bool     calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]);
extern bool     fk_leg_body(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                            float* out_x_mm, float* out_y_mm, float* out_z_mm);
extern bool     fk_leg_both(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                            float* out_body_x_mm, float* out_body_y_mm, float* out_body_z_mm,
                            float* out_leg_x_mm, float* out_leg_y_mm, float* out_leg_z_mm);
extern void     configApplyLoopHz(uint16_t hz);
extern bool     legEnabled(uint8_t leg);
extern bool     jointEnabled(uint8_t leg, uint8_t joint);
extern uint8_t  footContactState(uint8_t leg);
extern uint8_t  servoId(uint8_t leg, uint8_t joint);
extern void     setServoId(uint8_t leg, uint8_t joint, uint8_t id);
extern bool     servoIsOOS(uint8_t leg, uint8_t joint);
extern void     servoMarkOOS(uint8_t leg, uint8_t joint);
extern void     safetyLockoutCollision(uint8_t legA, uint8_t legB);
extern void     safetyLockoutTemp(uint8_t leg, uint8_t joint, int16_t temp_c10);
extern void     safetyCollisionStandDisable();
extern void     updateFootBodyEstimates();

// -----------------------------------------------------------------------------
// Function externs — implemented in functions.ino
// -----------------------------------------------------------------------------
extern void     printHELP();
extern void     printSTATUS();
extern void     rebootNow();
extern void     printOK();
extern void     printERR(uint8_t code, const char* msg);
extern void     handleLine(const char* line);
extern int      nextToken(const char* s, int start, int len, int* tokStart, int* tokLen);
extern void     buffersInit();
extern void     configLoad();
extern void     configWriteDefaults();
extern void     configSetKeyValue(const char* key, const char* val);
extern int16_t  readServoPosCdSync(uint8_t leg, uint8_t joint);
extern void     setServoTorqueNow(uint8_t leg, uint8_t joint, bool on);
extern int      angle_offset_read(uint8_t leg, uint8_t id);
extern bool     angle_offset_adjust(uint8_t leg, uint8_t id, int16_t offset_cd);
extern bool     angle_offset_write(uint8_t leg, uint8_t id);
extern void     telemetryPrintS1(uint8_t leg, uint16_t tick_us, uint8_t lockout,
                                 uint8_t test_mode, uint8_t test_phase, uint8_t rr, uint8_t enabled);
extern void     telemetryPrintS2();
extern void     telemetryPrintS3();
extern void     telemetryPrintS4();
extern void     telemetryPrintS5();
extern void     telemetryBinS1(uint16_t tick_us, uint8_t lockout, uint8_t test_mode,
                               uint8_t test_phase, uint8_t rr, uint8_t enabled);
extern void     telemetryBinS2();
extern void     telemetryBinS3();
extern void     telemetryBinS4();
extern void     telemetryBinS5();

// -----------------------------------------------------------------------------
// Command handler externs — implemented in commandprocessor.ino
// -----------------------------------------------------------------------------
extern CommandType parseCommandType(const char* cmd);
extern void     processCmdHELP(const char* line, int s, int len);
extern void     processCmdSTATUS(const char* line, int s, int len);
extern void     processCmdREBOOT(const char* line, int s, int len);
extern void     processCmdENABLE(const char* line, int s, int len);
extern void     processCmdDISABLE(const char* line, int s, int len);
extern void     processCmdFK(const char* line, int s, int len);
extern void     processCmdLEGS(const char* line, int s, int len);
extern void     processCmdSERVOS(const char* line, int s, int len);
extern void     processCmdLEG(const char* line, int s, int len);
extern void     processCmdSERVO(const char* line, int s, int len);
extern void     processCmdRAW(const char* line, int s, int len);
extern void     processCmdRAW3(const char* line, int s, int len);
extern void     processCmdFOOT(const char* line, int s, int len);
extern void     processCmdFEET(const char* line, int s, int len);
extern void     processCmdMODE(const char* line, int s, int len);
extern void     processCmdI(const char* line, int s, int len);
extern void     processCmdT(const char* line, int s, int len);
extern void     processCmdY(const char* line, int s, int len);
extern void     processCmdTELEM(const char* line, int s, int len);
extern void     processCmdTEST(const char* line, int s, int len);
extern void     processCmdSTAND(const char* line, int s, int len);
extern void     processCmdTUCK(const char* line, int s, int len);
extern void     processCmdSAFETY(const char* line, int s, int len);
extern void     processCmdHOME(const char* line, int s, int len);
extern void     processCmdSAVEHOME(const char* line, int s, int len);
extern void     processCmdOFFSET(const char* line, int s, int len);
extern void     processCmdLOG(const char* line, int s, int len);
extern void     processCmdPID(const char* line, int s, int len);
extern void     processCmdIMP(const char* line, int s, int len);
extern void     processCmdCOMP(const char* line, int s, int len);
extern void     processCmdEST(const char* line, int s, int len);
extern void     processCmdLOOP(const char* line, int s, int len);
extern void     processCmdLIMITS(const char* line, int s, int len);
extern void     processCmdCONFIG(const char* line, int s, int len);

#endif // MARS_GLOBALS_H

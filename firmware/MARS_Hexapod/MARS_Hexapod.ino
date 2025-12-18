/*
  MARS — Modular Autonomous Robotic System
  Hexapod v2.0 Controller (Teensy 4.1)

  Change log (top N entries)
  DIRECTIVE: Every code change (any file) must update BOTH this header list (add a concise dated bullet with FW version) AND `CHANGELOG.md`, bump `FW_VERSION` (patch) and `FW_BUILD`.
  - 2025-12-18: Bugfix: Removed duplicate jitter metrics calculation block in loopTick() that was wasting cycles and double-counting jitter stats. FW 0.2.42. (author: copilot)
  - 2025-12-17: Telemetry: Re-introduced S4 contact segment (6 per-leg contact flags). Currently stubbed to 0 for all legs until foot contact sensing is implemented. FW 0.2.41. (author: copilot)
  - 2025-12-17: TEST mode: Persist tripod test-gait parameters changed via TEST (cycle/height/basex/steplen/lift/overlap) into `/config.txt` (test.trigait.*) so CONFIG reflects updates and values survive reboot. FW 0.2.40. (author: copilot)
  - 2025-12-17: STAND: Clamp neutral IK stance to a reachable workspace and fall back to a conservative default when test-gait base params are out-of-range, preventing ERR 1 IK_FAIL during bring-up. FW 0.2.39. (author: copilot)
  - 2025-12-16: Telemetry: Extended binary S1 payload to include battery/current/IMU fields (with safe fallbacks when not instrumented) so controller INFO can show real values in pure-binary mode. FW 0.2.38. (author: copilot)
  - 2025-12-16: Telemetry: Added TELEM command and binary framed telemetry replacement mode (TELEM BIN 1) while keeping Y 1/Y 0 as the master telemetry enable for backward compatibility. FW 0.2.37. (author: copilot)
  - 2025-12-12: Safety telemetry S5: Added S5 segment streaming detailed safety state (lockout, causes, overrides, clearance, toggles, temp threshold) for Python controller UI and safety handling. FW 0.2.36. (author: copilot)
  - 2025-12-09: Float formatting fix: Replaced snprintf %f with dtostrf() for float config values (test gait params, clearance, rate limit, cart deadband). Teensy snprintf doesn't support %f; values were written as empty strings. FW 0.2.35. (author: copilot)
  - 2025-12-09: Missing extern fix: Added missing extern declarations for g_log_rotate and g_log_max_bytes in functions.ino; configWriteDefaults() now compiles correctly and persists all config keys including test params and logging settings. FW 0.2.34. (author: copilot)
  - 2025-12-09: Auto-generate missing config keys on boot: Added configWriteDefaults() called after configLoad() in setup() to ensure all known config keys exist in /config.txt with their code-default values. FW 0.2.33. (author: copilot)
  - 2025-12-09: Config persistence fix v2: Rewrote configSetKeyValue to use streaming temp-file pattern (like other config writers) instead of fixed 32-line buffer. No line limit, no data loss. FW 0.2.32. (author: copilot)
  - 2025-12-09: Config persistence fix: configSetKeyValue no longer corrupts non-matching lines when searching for key; added SD.remove before rewrite to ensure truncation. Fixes TUCK SET params not persisting across reboot. FW 0.2.31. (author: copilot)
  - 2025-11-21: Telemetry S3 format change: S3 now lists 18 voltages then 18 temperatures (v0..v17,t0..t17) to align with Python controller expectation. FW 0.2.29. (author: copilot)
  - 2025-11-21: Telemetry decoupled from FK: S1/S2/S3 output now controlled solely by `Y` (on/off) and no longer gated by the FK mask. RR_FK remains gated by `FK` mask. FW 0.2.28. (author: copilot)
  - 2025-11-21: Telemetry toggle + RR_FK: Added single-letter `Y` command (`Y1`/`Y0`) to enable/disable S1/S2/S3 streams; re-enabled legacy `RR_FK` body/leg-frame prints gated by `FK` mask. FW 0.2.27. (author: copilot)
  - 2025-11-21: Telemetry S2/S3 split: S2 now streams only enable flags for all 18 servos; new S3 carries volt/temp for all 18. Order remains LF,LM,LR,RF,RM,RR × C,F,T. FW 0.2.26. (author: copilot)
  - 2025-11-21: Telemetry S2 format: Now a single line includes all 18 servos (LF,LM,LR,RF,RM,RR × C,F,T) as `vinV,tempC,en` triplets; removed leg tag. `en` is the logical AND of leg and joint enables. FW 0.2.25. (author: copilot)
  - 2025-11-21: Telemetry refactor: Moved S1/S2 streaming into helpers `telemetryPrintS1`/`telemetryPrintS2` (functions.ino) preserving format; internal maintainability improvement. FW 0.2.24. (author: copilot)
  - 2025-11-17: LOOP command: Added `LOOP <hz>` console command to set control loop frequency at runtime (50..500 Hz), applying via configApplyLoopHz and persisting `loop_hz` to `/config.txt` when SD is enabled. HELP/NOTES updated to document LOOP and interaction with logging.rate_hz. FW 0.2.14. (author: copilot)
  - 2025-11-13: PID dt-aware: PID now accounts for variable loop timing. D-term uses (de/dt) with dt from LoopTimer; I-term integrates e*dt (cd·ms) with appropriate scaling. Keeps derivative smoothing. FW 0.2.11. (author: copilot)
  - 2025-11-13: PID shadow telemetry: Enriched PID_SHADOW lines to include per-leg joint error (err_cd=des-est), read/estimate (est_cd), and base target (tgt_cd) alongside diffs (pid-base). FW 0.2.10. (author: copilot)
  - 2025-11-13: PID shadow mode: Added parallel (non-driving) PID computation mode `pid.mode=shadow` that computes hypothetical PID-corrected targets without applying them, streams per-leg diffs at `pid.shadow_report_hz` (default 2 Hz) via `PID_SHADOW` lines. Active mode remains `pid.mode=active` (default). FW 0.2.9. (author: copilot)
  - 2025-11-13: Estimator + filtered D: Added simple position estimator (exp. smoothing toward last command with measurement correction on RR updates) and low-pass filtered derivative to reduce noise spikes. New config keys: est.cmd_alpha_milli, est.meas_alpha_milli, pid.kd_alpha_milli.<joint>. FW 0.2.8. (author: copilot)
  - 2025-11-13: Joint PID integration (sparse-feedback): compute P/PI/PD correction per joint (default disabled). Added /config.txt keys pid.enabled and pid.{kp,ki,kd}_milli.<joint>. STATUS shows [PID] section. FW 0.2.7. (author: copilot)
  - 2025-11-13: Wire-in optimizations: Fast send and staggered feedback are now always enabled (flags removed). Loop rate remains elastic via adaptive logic. FW 0.2.5. (author: copilot)
  - 2025-11-13: Feedback IO: Added MARS_FB_STAGGER (default ON). Always read position; alternate vin and temp per RR visit to the same servo. Preserves OOS rule (any valid vin OR temp resets). Reduces fb_us per tick. FW 0.2.4. (author: copilot)
  - 2025-11-13: Fast send path (Option A): Added MARS_FAST_SEND (default OFF). Batched per-leg MOVE_TIME frames with single OE window; flush/deassert on RR leg before feedback. Observed send_us≈1.7 ms in TEST; fb_us now dominant. FW 0.2.3. (author: copilot)
  - 2025-11-13: Versioning policy: Added SemVer + FW_BUILD policy to docs/PROJECT_SPEC.md (major/minor require explicit confirmation; patch/build may auto-increment). FW 0.2.2. (author: copilot)
  - 2025-11-18: Agent policy: For any assistant-made code change, bump FW_VERSION patch + FW_BUILD and add concise entries to BOTH this header and CHANGELOG.md summarizing behavior/config changes. (author: copilot)
  - 2025-11-13: Build metadata: Introduced FW_BUILD (monotonic build number) and print in splash/STATUS. Minor bump to 0.2.1 to surface this UI change. FW 0.2.1. (author: copilot)
  - 2025-11-13: Phase 2 kickoff: Minor version bump to 0.2.0. Added send-path profiling (MARS_PROFILE_SEND) capturing per-call and per-tick send durations for lx16a writes; no behavior changes. Adopt Phase 2 workflow policies: defer commits until instructed; auto-bump FW patch on assistant edits (CHANGELOG only on TODO completion/behavior change). FW 0.2.0. (author: copilot)
  - 2025-11-12: Loop timing validation: Jitter instrumentation (min/avg/max) confirms <10% timing error at 166 Hz under idle and tripod test gait (no sustained overruns). Acceptance criterion met; no functional changes beyond earlier probes. FW 0.1.116. (author: copilot)
  - 2025-11-12: Safety config on boot: Config loader now parses safety.* keys (soft_limits, collision, temp_lockout_c, clearance_mm) at startup to restore persisted settings. Added tiny parse_bool helper. STATUS [TIMING] now includes jitter_us min/avg/max. FW 0.1.115. (author: copilot)
  - 2025-11-12: Config keys + offsets: Added boot-time refresh of hardware angle offsets into g_offset_cd and parsing of offset_cd.<LEG>.<joint> from /config.txt (seed, hardware wins). Expanded config keys to load tripod gait params at boot: test.trigait.{cycle_ms,height_mm,basex_mm,steplen_mm,lift_mm,overlap_pct}. HELP notes new keys; PROJECT_SPEC updated. FW 0.1.114. (author: copilot)
  - 2025-11-12: TUCK PARAMS: Added TUCK PARAMS subcommand and HELP line to display current tuck.* parameter values. FW 0.1.112. (author: copilot)
  - 2025-11-12: HELP update: TUCK help lines now describe runtime-configurable tuck.* params and TUCK SET command (persist TIBIA/FEMUR/COXA/TOL_TIBIA/TOL_OTHER/TIMEOUT). FW 0.1.111. (author: copilot)
  - 2025-11-12: STATUS trim: Removed temporary [TUCK] debug section from STATUS to reduce print cost and keep output focused. No behavior changes to TUCK controller. FW 0.1.110. (author: copilot)
  - 2025-11-12: TUCK mirror: Right-side legs now mirror femur/tibia targets (coxa unchanged) for symmetric tuck; added STATUS [TUCK] debug section. FW 0.1.108. (author: copilot)
  - 2025-11-12: Feedback fix: Treat 0 cd as a valid measurement using explicit validity flags; prevents fallback to last-sent (e.g., 24000) that stalled TUCK tibia convergence. Restored TUCK tibia target to 0 cd. FW 0.1.107. (author: copilot)
  - 2025-11-12: TUCK tweak: Set coxa target to 12000 cd (logical center) during TUCK; femur=19000, tibia=0 unchanged. FW 0.1.106. (author: copilot)
  - 2025-11-12: TUCK fix: Continuously drive femur/coxa after tibia convergence and complete only on convergence or timeout; resolves case where only tibia moved. FW 0.1.105. (author: copilot)
  - 2025-11-12: TUCK sequencing: Added leg-selectable TUCK [LEG|ALL] with non-blocking tibia-first convergence then femur/coxa (targets c=0,f=19000,t=0). FW 0.1.104. (author: copilot)
  - 2025-11-12: Command: Added TUCK to move enabled, in-service joints to c=0/f=20000/t=0 cd respecting enable and OOS flags. FW 0.1.103. (author: copilot)
  - 2025-11-12: STATUS formatting: Now uses HELP-like section headers ([SYSTEM], [ENABLES], [TELEMETRY], [HOME], [TEST], [TIMING], [OOS]) with one key per line indented under each header. No semantic changes; improves readability for CLI and logs. FW 0.1.102. (author: copilot)
  - 2025-11-12: Offset unit fix: hardware angle offsets are stored as ticks (cd/24). Wrapper now converts units↔centideg (read: units*24; write: round(cd/24) clamp ±125). SAVEHOME and OFFSET commands surface true centidegree values. FW 0.1.101. (author: copilot)
  - 2025-11-12: Calibration fix: SAVEHOME now clears existing offsets, reads raw position, and applies absolute centidegree offsets (offset_cd=raw-12000, clamp ±3000). Removed legacy /24 units conversion. RR pos grid may take a few ticks to reflect new logical values. FW 0.1.100. (author: copilot)
  - 2025-11-11: Offset bug fix: angle_offset_* wrappers now scope lookup to the specified leg (previously global ID scan returned same joint offset across legs). Re-run SAVEHOME for accurate per-leg offsets. FW 0.1.99. (author: copilot)
  - 2025-11-11: Calibration: SAVEHOME now uses cd-only clear→read→compute flow (offset_cd=pos-12000) and persists both home_cd and offset_cd; OFFSET LIST/CLEAR aligned to cd semantics. FW 0.1.98. (author: copilot)
  - 2025-11-11: Offsets: Wired hardware angle offset helpers to servo library; removed in-RAM stubs so SAVEHOME/OFFSET use real device I/O. FW 0.1.97. (author: copilot)
  - 2025-11-10: Loop timing: defaulted LOOP_HZ_DEFAULT and ceiling to 166 Hz; adaptive windowed logic allows ramp-up under headroom. FW 0.1.95. (author: copilot)
  - 2025-11-10: Uptime: Added continuous wrap-safe uptime accumulator (g_uptime_ms64) updated each loop; STATUS now reads without advancing it (decoupled from call frequency). FW 0.1.90. (author: copilot)
  - 2025-11-10: Logging: LOG settings persistence (RATE, MODE, HEADER, ROTATE, MAXKB) to /config.txt (excluding enabled). Config parser now supports logging.rotate and logging.max_kb. FW 0.1.89. (author: copilot)
  - 2025-11-10: Logging: Rows now aggregate an entire leg per line (cmd/meas/vin/temp/oos for 3 servos). Compact: 1 line (RR leg); Full: 6 lines (all legs). Updated headers/schema. FW 0.1.88. (author: copilot)
  - 2025-11-10: Logging: FULL mode now appends per-servo OOS flag column (oos). Added LOG CLEAR to close & delete current file; next sample reopens. FW 0.1.87. (author: copilot)
  - 2025-11-10: Logging: Added size-based rotation (LOG ROTATE, LOG MAXKB) with KB thresholds (min 100KB, clamp 1GB). Filenames now use _seq<index>. FW 0.1.86. (author: copilot)
  - 2025-11-09: Logging: Phase 1 compact CSV implemented (config: logging.enabled, logging.rate_hz, logging.mode, logging.header). 8KB buffer, /logs/<millis>.csv, sample-divisor, header optional. Added LOG command suite. FW 0.1.81. (author: copilot)
  - 2025-11-09: Build: Unified SD feature guard to `#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD`; removed obsolete extern in commandprocessor. FW 0.1.78. (author: copilot)
  - 2025-11-09: Protocol: Centralized OK responses — removed residual per-handler printOK() calls across command handlers; dispatcher now emits a single OK on success. FW 0.1.77. (author: copilot)
  - 2025-11-08: HELP: Restored multi-line categorized HELP (System, Enable/Disable, Motion, Geometry/Calibration, Mode/Test, Safety) with CRLF and F() strings. FW 0.1.76. (author: copilot)
  - 2025-11-08: STATUS: Restored multi-line STATUS with system/config, test params, safety toggles, leg/joint enables, offset grid, OOS mask, and FK mask. FW 0.1.75. (author: copilot)
  - 2025-11-08: Startup splash restored with build date/time and compact summary (non-blocking). FW 0.1.74. (author: copilot)
  - 2025-11-08: UX: Echo the processed serial command back to host ("> <line>") after handlers run to aid logging/debug. FW 0.1.73. (author: copilot)
  - 2025-11-08: Restore/stub helper functions required by modular command processor (printOK/ERR, HELP/STATUS, nextToken, rebootNow, angle_offset_*); confirm FK helpers present. Addresses undefined references at link time after refactor. FW 0.1.72. (author: copilot)
  - 2025-11-08: Remove legacy if/else command chain from functions.ino; handleLine now delegates exclusively to the modular dispatcher in commandprocessor.ino. FW 0.1.70. (author: copilot)
  - 2025-11-08: Wire modular dispatcher into handleLine; replace legacy if/else chain with enum-based switch; all commands now route through commandprocessor.ino (OFFSET/SAVEHOME/SAFETY etc.). FW 0.1.69. (author: copilot)
  - 2025-11-07: Refactor: modular command processor with per-command handlers and enum-based dispatcher moved to commandprocessor.ino; readability pass and preprocessor fixes. FW 0.1.68. (author: copilot)
  - 2025-11-07: Calibration: SAVEHOME now adjusts and saves servo hardware angle offsets to center at 12000 cd (±30° clamp); persists residual homes; added OFFSET LIST/CLEAR and STATUS offset_cd grid. FW 0.1.67. (author: copilot)
  - 2025-11-05: Telemetry: RR_FK now includes leg-frame position (lx/ly/lz, mm, origin at hip) and raw joint angles (c/f/t in centideg). FW 0.1.66. (author: copilot)
  - 2025-11-04: Telemetry: Added `FK <LEG|ALL> <ON|OFF>` command to toggle FK body-frame stream per leg; RR_FK now gated by mask (defaults to LM only to preserve prior behavior). FW 0.1.65. (author: copilot)
  - 2025-11-04: Docs: Updated PROJECT_SPEC with full SAFETY subcommands (LIST/OVERRIDE/SOFTLIMITS/COLLISION/TEMPLOCK/CLEARANCE), grouped STATUS description, and added implemented config keys (safety.*, test.trigait.overlap_pct). FW 0.1.64. (author: copilot)
  - 2025-11-03: UX: Synced HELP with implemented commands (added SAFETY SOFTLIMITS/COLLISION/TEMPLOCK; removed duplicates) and reformatted STATUS into grouped sections for readability. FW 0.1.63. (author: copilot)
  - 2025-11-03: Telemetry: Print FK body-frame foot position (x/y/z) for the leg served in the round-robin feedback each tick to aid bring-up. FW 0.1.62. (author: copilot)
  - 2025-11-03: Safety config: Added toggles for soft limits and collision checks, and a configurable over-temp lockout threshold. New config keys: safety.soft_limits, safety.collision, safety.temp_lockout_c. New serial: SAFETY SOFTLIMITS, SAFETY COLLISION, SAFETY TEMPLOCK. STATUS/SAFETY LIST updated. FW 0.1.61. (author: copilot)
  - 2025-11-03: Safety: FOOT/FEET collision checks now use FK-estimated body-frame foot positions (X/Z) to predict post-command clearance; avoids startup false positives and aligns with loop safety; version bump to 0.1.60. (author: copilot)
  - 2025-11-03: Safety: Added simple foot-to-foot keep-out check in X/Z plane with configurable clearance; new serial cmd 'SAFETY CLEARANCE <mm>' and config key 'safety.clearance_mm'; SAFETY LIST/OVERRIDE and STATUS safety summary; version bump to 0.1.59. (author: copilot)
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
// Feature flags (compile-time) — define early so includes can honor guards
// -----------------------------------------------------------------------------
#ifndef MARS_ENABLE_LOGGING
#define MARS_ENABLE_LOGGING 0
#endif
#ifndef MARS_ENABLE_SD
#define MARS_ENABLE_SD 1
#endif
#ifndef MARS_TIMING_PROBES
#define MARS_TIMING_PROBES 1
#endif
// Enable fine-grained send-path profiling (per-call and per-tick totals)
#ifndef MARS_PROFILE_SEND
#define MARS_PROFILE_SEND 1
#endif
// Optimizations (fast send + staggered feedback) are now permanently enabled; no flags.

// -----------------------------------------------------------------------------
// Top-level includes (order matters for SD feature guard)
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include "robot_config.h"
#include "LoopTimer.hpp"
#include "command_types.h"
#include "command_helpers.h"
#include "MarsImpedance.hpp"

// Telemetry helpers (defined in functions.ino)
void telemetryPrintS1(uint8_t leg, uint16_t loop_us, uint8_t lockout, uint8_t mode, uint8_t test_phase, uint8_t rr_index, uint8_t enabled);
void telemetryPrintS2();
void telemetryPrintS3();
void telemetryPrintS4();
void telemetryPrintS5();
void telemetryBinS1(uint16_t loop_us, uint8_t lockout, uint8_t mode, uint8_t test_phase, uint8_t rr_index, uint8_t enabled);
void telemetryBinS2();
void telemetryBinS3();
void telemetryBinS4();
void telemetryBinS5();
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#include <FS.h>
#include <SD.h>
#endif
#include <lx16a-servo.h>

// -----------------------------------------------------------------------------
// Firmware version (surfaced in splash/STATUS and logs)
// -----------------------------------------------------------------------------
// NOTE: Per Phase 2 policy, bump PATCH and FW_BUILD on each assistant edit that
// changes behavior or completes a TODO item. Keep MAJOR/MINOR stable unless
// explicitly requested.
#ifndef FW_VERSION
#define FW_VERSION "0.2.42"
#endif
// Monotonic build number (never resets across minor/major). Increment every code edit.
#ifndef FW_BUILD
#define FW_BUILD 158
#endif

// -----------------------------------------------------------------------------
// Forward declarations for cross-TU helpers implemented in functions.ino
// -----------------------------------------------------------------------------
void splash();
void processSerial();
bool calculateIK(uint8_t leg, float x_mm, float y_mm, float z_mm, int16_t out_cd[3]);
void refreshOffsetsAtStartup();
//void refreshOffsetsAtStartup();

// -----------------------------------------------------------------------------
// Foot contact sensing (future enhancement)
// -----------------------------------------------------------------------------
// Telemetry S4 streams one contact flag per leg (LF,LM,LR,RF,RM,RR).
// Hardware contact sensing is not implemented yet, so this currently returns 0.
uint8_t footContactState(uint8_t leg)
{
  (void)leg;
  return 0;
}
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
// FK and body-frame foot estimates
extern bool    fk_leg_body(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                           float* out_x_mm, float* out_y_mm, float* out_z_mm);
extern bool    fk_leg_both(uint8_t leg, int16_t coxa_cd, int16_t femur_cd, int16_t tibia_cd,
                           float* out_body_x_mm, float* out_body_y_mm, float* out_body_z_mm,
                           float* out_leg_x_mm,  float* out_leg_y_mm,  float* out_leg_z_mm);

// Impedance controller instance (joint-space and simple Cartesian modes)
static MarsImpedance g_impedance;

// Loop timing
static const uint16_t LOOP_HZ_DEFAULT = 166; // spec target; adaptive logic can step down if overruns occur
volatile uint16_t g_loop_hz = LOOP_HZ_DEFAULT;
static const float TICK_MS_DEFAULT = 1000.0f / LOOP_HZ_DEFAULT; // ~6.024 ms

// ISR-based tick trigger
// Loop timer (polling in loop)
static LoopTimerClass g_loopTimer(LOOP_HZ_DEFAULT);

// -----------------------------------------------------------------------------
// TUCK command state (non-blocking sequencing): tibia first, then femur+coxa
// -----------------------------------------------------------------------------
// Contract:
//  - processCmdTUCK sets g_tuck_active=1 and selects legs in g_tuck_mask (bit per leg).
//  - Stage 1: loopTick keeps commanding tibia to target (0 cd) until within tolerance.
//  - Stage 2: for each leg whose tibia reached target, command femur/coxa (19000/0 cd) once.
//  - Completion: when all selected legs have been issued femur/coxa (or timeout), clear active.
static volatile uint8_t  g_tuck_active     = 0;
static volatile uint8_t  g_tuck_mask       = 0; // bits 0..5 for LF..RR
static volatile uint8_t  g_tuck_done_mask  = 0; // femur+coxa within tolerance (stage 2 complete per leg)
static volatile uint32_t g_tuck_start_ms   = 0;

// TUCK parameters (runtime-configurable; persisted to /config.txt)
volatile int16_t  g_tuck_tibia_cd       = 200;    // tibia target (cd)
volatile int16_t  g_tuck_femur_cd       = 19000;  // femur target (cd)
volatile int16_t  g_tuck_coxa_cd        = 12000;  // coxa target (cd, absolute center)
volatile int16_t  g_tuck_tol_tibia_cd   = 750;    // tolerance for tibia (cd)
volatile int16_t  g_tuck_tol_other_cd   = 500;    // tolerance for femur/coxa (cd)
volatile uint16_t g_tuck_timeout_ms     = 5000;   // timeout (ms)

// ----------------------------------------------------------------------------
// Logging (compact CSV) globals (Phase 1 + rotation)
// ----------------------------------------------------------------------------
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
volatile bool     g_log_enabled = false;        // set true by config key logging.enabled
volatile uint16_t g_log_rate_hz = 166;          // target sampling rate (<= loop_hz)
volatile uint8_t  g_log_sample_div = 1;         // loop_hz / log_rate_hz (integer)
volatile uint32_t g_log_tick_counter = 0;       // total ticks since boot (used for sampling)
volatile uint8_t  g_log_mode = 0;               // 0=compact (RR leg only) ; 1=full
volatile bool     g_log_header = true;          // emit header line at file open
volatile bool     g_log_rotate = true;          // enable size-based rotation
volatile uint32_t g_log_max_bytes = 10UL * 1024UL * 1024UL; // rotation threshold (default 10MB)
volatile uint32_t g_log_file_bytes = 0;         // bytes written into current file
volatile uint32_t g_log_total_bytes = 0;        // cumulative bytes across rotations
volatile uint32_t g_log_seq = 0;                // sequence index for rotated files
char              g_log_buf[8192];              // staging buffer
uint16_t          g_log_buf_used = 0;           // bytes currently staged
File              g_log_file;                   // active log file handle
#endif

// ----------------------------------------------------------------------------
// TEST gait runtime parameters (mutable via serial commands)
// ----------------------------------------------------------------------------
// Defaults chosen to match prior constants used in tripod test gait
float    g_test_base_y_mm   = -150.0f;  // ground height (negative is down)
float    g_test_base_x_mm   = 130.0f;   // lateral offset
float    g_test_step_len_mm = 40.0f;    // forward/back amplitude (|z|)
uint32_t g_test_cycle_ms    = 2000;     // ms per tripod phase
float    g_test_lift_y_mm   = 40.0f;    // step height (lift amount on swing)
float    g_test_overlap_pct = 5.0f;     // percent of per-phase time reserved as overlap (both tripods stance)

// System state
enum MarsMode : uint8_t { MODE_IDLE = 0, MODE_TEST = 1, MODE_LOCKOUT = 2 };
volatile bool g_enabled = false;
volatile bool g_lockout = false; // safety lockout flag

// Collision detection removed; no externs or forward-declarations here.
static volatile MarsMode g_mode = MODE_IDLE;
volatile uint8_t g_last_err = 0; // 0 = OK
volatile uint32_t g_overrun_count = 0; // counts ticks exceeding period
static volatile uint16_t g_overrun_since_adjust = 0; // consecutive overruns since last rate change
static volatile uint16_t g_ok_since_adjust = 0;       // consecutive on-time ticks since last rate change
// Adaptive window counters (used for new loop rate adjustment logic)
static volatile uint16_t g_tick_window_count = 0;      // ticks accumulated in current window
static volatile uint16_t g_tick_window_overruns = 0;   // overruns in current window
// Config status (SD)
bool g_config_loaded = false;
uint16_t g_config_keys_applied = 0;
// Default ceiling should reflect spec target (166 Hz); start rate can be lower but ceiling shouldn't block ramp-up.
uint16_t g_config_loop_hz = 166;

// FK stream control: 6-bit mask (LF..RR). Default to LM only to preserve prior behavior.
// FK stream mask (bit per leg LF..RR). Disabled at startup; can be enabled via FK command.
volatile uint8_t g_fk_stream_mask = 0u;
// Telemetry master toggle for S1/S2/S3 streams (default ON for backward compatibility)
volatile uint8_t g_telem_enabled = 1;
// Telemetry format select (0=ASCII S1..S5, 1=binary framed packets). Master enable remains Y 1/Y 0.
volatile uint8_t g_telem_bin_enabled = 0;

// Timing probes (compile-time optional)
#if MARS_TIMING_PROBES
static volatile uint16_t g_probe_serial_us = 0;  // time spent in processSerial() this loop (us)
static volatile uint16_t g_probe_send_us   = 0;  // time to send all enabled servo commands (us)
static volatile uint16_t g_probe_fb_us     = 0;  // time to read one servo feedback (us)
static volatile uint16_t g_probe_tick_us   = 0;  // total loopTick() duration (us)
static volatile uint16_t g_probe_log_us    = 0;  // time spent in logging write/rotate segment (us)
// Jitter metrics: absolute error |tick_elapsed_us - period_us| over a rolling window
static volatile uint16_t g_jitter_min_us   = 0xFFFF;  // min abs error in current window
static volatile uint16_t g_jitter_max_us   = 0;       // max abs error in current window
static volatile uint32_t g_jitter_sum_us   = 0;       // sum for avg in current window
static volatile uint16_t g_jitter_count    = 0;       // samples in current window
#endif

// Optional send-path profiling (aggregated stats; gated at compile-time)
#if MARS_PROFILE_SEND
// Per-call (single servo command) duration stats
static volatile uint32_t g_prof_send_call_us_sum = 0;  // sum of per-call durations (us)
static volatile uint16_t g_prof_send_call_us_min = 0xFFFF; // min duration observed (us)
static volatile uint16_t g_prof_send_call_us_max = 0;      // max duration observed (us)
static volatile uint32_t g_prof_send_call_count  = 0;      // number of calls measured
// Per-tick total send duration stats
static volatile uint32_t g_prof_send_tick_us_sum = 0;   // sum of per-tick total send durations (us)
static volatile uint16_t g_prof_send_tick_us_min = 0xFFFF; // min total per-tick send time (us)
static volatile uint16_t g_prof_send_tick_us_max = 0;      // max total per-tick send time (us)
static volatile uint32_t g_prof_send_tick_count  = 0;      // number of ticks with sends measured
#endif
// Startup time (for OOS grace window)
static uint32_t g_boot_ms = 0;
// Monotonic uptime tracker (wrap-safe, updated continuously in loop)
volatile uint64_t g_uptime_ms64 = 0;
static volatile uint32_t g_uptime_prev_ms = 0;

// Safety: lockout causes and overrides
enum LockoutCauseBits : uint16_t {
  LOCKOUT_CAUSE_NONE = 0,
  LOCKOUT_CAUSE_TEMP = 1u << 0,
  LOCKOUT_CAUSE_COLLISION = 1u << 1,
  // Reserved for future causes (soft limit, collision, estop, etc.)
};
static volatile uint16_t g_lockout_causes = LOCKOUT_CAUSE_NONE; // snapshot at lockout
static volatile uint16_t g_override_mask  = LOCKOUT_CAUSE_NONE; // user overrides
// Snapshot of temp trips at lockout time (true if that joint exceeded threshold) and value in 0.1 C
static volatile bool g_lockout_temp_trip[NUM_LEGS][LEG_SERVOS] = { { false } };
static volatile int16_t g_lockout_temp_c10[NUM_LEGS][LEG_SERVOS] = { { 0 } };

// UART config presence/match flags from /config.txt (uart.<LEG>=SerialX); mapping is compile-time fixed.
volatile uint8_t g_uart_cfg_seen_mask = 0;  // bit per leg LF..RR when a uart.<LEG> key is present
volatile bool    g_uart_cfg_present   = false; // any uart.* key present
volatile bool    g_uart_cfg_match     = true;  // assumes match until a mismatch is seen

// Safety: configurable foot-to-foot keep-out clearance (mm) in X/Z plane
float g_safety_clearance_mm = 60.0f;

// Safety toggles and thresholds (runtime-configurable; loaded from /config.txt)
// - Soft joint limits enable: clamp commands to configured min/max
// - Collision check enable: enable/disable foot-to-foot keep-out test
// - Temperature lockout threshold in 0.1 C units (default 80.0 C)
volatile bool   g_safety_soft_limits_enabled = true;
volatile bool   g_safety_collision_enabled   = true;
volatile int16_t g_safety_temp_lockout_c10   = 800; // 80.0 C

// Track last commanded foot targets (body frame, mm) to evaluate keep-out
float g_foot_target_x_mm[NUM_LEGS] = {0};
float g_foot_target_z_mm[NUM_LEGS] = {0};

// FK-based foot position estimates in BODY frame (mm)
float g_foot_body_x_mm[NUM_LEGS] = {0};
float g_foot_body_y_mm[NUM_LEGS] = {0};
float g_foot_body_z_mm[NUM_LEGS] = {0};

// FK helpers declared below once joint/home arrays are defined

// Commanded joint targets (centidegrees) per leg/joint. IK/commands will write here.
static volatile int16_t g_cmd_cd[NUM_LEGS][3] = { {0} };
// Joint compliance: per-leg/joint offset applied around g_cmd_cd to form an effective
// setpoint. This is adapted using a simple P-like controller around the estimated
// joint position to provide "soft" behavior while keeping the planner target as a
// moving reference.
static int16_t g_comp_offset_cd[NUM_LEGS][LEG_SERVOS] = { {0} };
// Effective joint command after compliance; this is what downstream PID/impedance
// and safety logic will see as the base target each tick.
static int16_t g_eff_cmd_cd[NUM_LEGS][LEG_SERVOS] = { {0} };
// Home positions (centidegrees) per leg/joint; loaded from SD config; default 12000
volatile int16_t g_home_cd[NUM_LEGS][LEG_SERVOS] = {
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000},
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000}
};
// Soft joint limits (absolute centidegrees). Defaults to full range; load from SD config.
volatile int16_t g_limit_min_cd[NUM_LEGS][LEG_SERVOS] = {
  {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
};
volatile int16_t g_limit_max_cd[NUM_LEGS][LEG_SERVOS] = {
  {24000, 24000, 24000}, {24000, 24000, 24000}, {24000, 24000, 24000},
  {24000, 24000, 24000}, {24000, 24000, 24000}, {24000, 24000, 24000}
};
// Persistent hardware offset values (centideg) per servo (derived from angle_offset_* units*24)
int16_t g_offset_cd[NUM_LEGS][LEG_SERVOS] = { {0} };
// Rate limiting config: max joint speed in centideg/s (applied as per-tick cd delta clamp). Safe default 360.00 deg/s => 36000 cdeg/s
volatile uint16_t g_rate_limit_cdeg_per_s = 36000u;
// Last sent command (centidegrees) for per-tick rate limiting
static volatile int16_t g_last_sent_cd[NUM_LEGS][LEG_SERVOS] = {
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000},
  {12000, 12000, 12000}, {12000, 12000, 12000}, {12000, 12000, 12000}
};
// Torque control is handled synchronously in command handlers (no per-tick processing)
// Enable masks (bit-packed) to reduce RAM and simplify checks
// Bits 0..5 correspond to legs LF..RR
volatile uint8_t g_leg_enabled_mask = 0; // 6-bit mask
// Bits 0..17 correspond to (leg*3 + joint)
volatile uint32_t g_joint_enabled_mask = 0x3FFFFUL; // 18-bit mask: default all joints enabled for bring-up
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
volatile uint8_t g_rr_index = 0;

// Last measured values from sparse feedback (updated round-robin)
uint16_t g_meas_vin_mV[NUM_LEGS][LEG_SERVOS] = { {0} }; // millivolts
uint8_t  g_meas_temp_C[NUM_LEGS][LEG_SERVOS] = { {0} }; // temperature in whole deg C
int16_t  g_meas_pos_cd[NUM_LEGS][LEG_SERVOS] = { {0} }; // 0..24000 centidegrees
// Position validity flags: true when the last RR read for that joint succeeded
volatile uint8_t g_meas_pos_valid[NUM_LEGS][LEG_SERVOS] = { {0} };
// Feedback staggering toggle per servo: 0 -> read vin this visit; 1 -> read temp this visit
static uint8_t g_fb_rr_toggle[NUM_LEGS][LEG_SERVOS] = { {0} };

// -----------------------------------------------------------------------------
// Position & velocity estimator (per joint)
//
// We maintain a tiny motion model per joint with position (cd) and velocity
// (cd/s). Each tick we:
//   1) Predict: pos += vel * dt_s, vel decays slightly toward 0.
//   2) Nudge pos toward the effective command (compliance-adjusted) using a
//      small command gain (keeps estimate from drifting too far off plan when
//      feedback is sparse).
//   3) On RR measurement for that joint we apply a simple correction:
//        pos += Kx * (meas - pos)
//        vel += Kv * (meas - pos) / dt
// Gains are expressed in milli-units (0..1000) for cheap integer math.
// -----------------------------------------------------------------------------
struct JointEstimator {
  int16_t pos_cd;   // estimated position (centideg)
  int16_t vel_cds;  // estimated velocity (centideg per second)
};

static JointEstimator g_est_state[NUM_LEGS][LEG_SERVOS] = { { {0, 0} } };

// Estimator tuning (milli-gains shared across joints for now)
// - g_est_cmd_alpha_milli: small pull toward effective command each tick.
// - g_est_meas_alpha_milli: how hard we snap position toward measurement on RR.
// - g_est_meas_vel_alpha_milli: how aggressively we update velocity from residual.
volatile uint16_t g_est_cmd_alpha_milli      = 150; // 0.15 toward command per tick
volatile uint16_t g_est_meas_alpha_milli     = 700; // 0.70 toward measurement on RR update
volatile uint16_t g_est_meas_vel_alpha_milli = 400; // 0.40 of (residual/dt) blended into vel

// -----------------------------------------------------------------------------
// Joint compliance — simple P-like setpoint adaptation around estimated position
// per joint group. This layer adjusts g_comp_offset_cd[leg][joint] such that the
// effective command g_eff_cmd_cd = g_cmd_cd + offset can move within a limited
// window toward the actual joint angle when external forces are applied.
// Defaults are conservative; configuration and commands will be layered on later.
// -----------------------------------------------------------------------------
bool     g_comp_enabled = false; // runtime toggle; initially off
// Per joint group (coxa/femur/tibia) gains and bounds
uint16_t g_comp_kp_milli[LEG_SERVOS]    = {100, 100, 100};    // 0.1 * err_cd per tick
uint16_t g_comp_range_cd[LEG_SERVOS]    = {1000, 1500, 1500}; // +/-10..15 deg
uint16_t g_comp_deadband_cd[LEG_SERVOS] = {50, 50, 50};       // 0.5 deg deadband
uint16_t g_comp_leak_milli = 50;                               // leak toward 0 when quiet

// -----------------------------------------------------------------------------
// Joint PID (sparse-feedback) — P/PI/PD scaffolding (defaults to disabled)
// Gains are in milli-units; correction in centidegrees: corr_cd = (Kp_milli * err_cd)/1000 + ...
// Defaults: pid.enabled=false; only P-term applied when enabled and Kp>0.
// -----------------------------------------------------------------------------
volatile bool     g_pid_enabled = true;
volatile uint16_t g_pid_kp_milli[LEG_SERVOS] = {0, 0, 0}; // per joint group (coxa/femur/tibia)
volatile uint16_t g_pid_ki_milli[LEG_SERVOS] = {0, 0, 0};
volatile uint16_t g_pid_kd_milli[LEG_SERVOS] = {0, 0, 0};
// State per joint
static int32_t    g_pid_i_accum[NUM_LEGS][LEG_SERVOS] = { {0} }; // integral accumulator in cd·ms (centideg-milliseconds)
static int16_t    g_pid_prev_err[NUM_LEGS][LEG_SERVOS] = { {0} };
// Filtered derivative state per joint
static int16_t    g_pid_d_filt[NUM_LEGS][LEG_SERVOS] = { {0} };
// Derivative smoothing factor (0..1000 milli) per joint group
volatile uint16_t g_pid_kd_alpha_milli[LEG_SERVOS] = {200, 200, 200};
// Clamp for correction contribution to avoid extreme nudges
static const int16_t PID_CORR_CLAMP_CD = 2000; // ±20.00°
// PID mode: 0=active (corrections applied), 1=shadow (compute only, do not drive; stream diffs)
volatile uint8_t  g_pid_mode = 1; // default shadow
volatile uint16_t g_pid_shadow_report_hz = 2; // streaming frequency for PID_SHADOW lines (Hz)
static uint32_t   g_pid_shadow_last_report_ms = 0;
// Forward kinematics: derive BODY-frame foot position from absolute joint centidegrees
// ----------------------------------------------------------------------------
// Forward kinematics (FK): Compute BODY-frame foot position (x,y,z) from joint angles
// ----------------------------------------------------------------------------
// Purpose
//   Given measured/estimated joint angles for one leg — Coxa (hip yaw), Femur (hip pitch),
//   and Tibia (knee pitch) — this routine computes the foot position in the robot BODY frame
//   in millimeters.
//
// Frames and conventions
//   BODY frame axes:
//     x: lateral (left +, right - or vice-versa depending on leg, see offset usage)
//     y: vertical (up positive)
//     z: forward (front positive)
//   Joint angles:
//     - All joint angles are provided as absolute centidegrees (cd) from the servos.
//     - We convert them to degrees/radians relative to each joint’s configured home
//       g_home_cd[leg][joint]. The home convention matches calculateIK().
//     - Coxa (yaw) rotates around the vertical (BODY y) axis and orients the leg’s sagittal plane.
//     - Femur and Tibia rotate in that sagittal plane (pitch).
//
// Geometry
//   - COXA_LENGTH: horizontal standoff from hip yaw axis to the femur joint axis (mm).
//   - FEMUR_LENGTH (a): femur link length (mm).
//   - TIBIA_LENGTH (b): tibia link length (mm).
//   - Robot::COXA_OFFSET[leg] = (bx, bz): BODY-frame horizontal location (x,z) of the hip yaw axis
//     for each leg, i.e., the leg’s mounting point on the chassis.
//
// Algorithm overview
//   1) Yaw orientation:
//      - The Coxa yaw angle yaw rotates the leg’s sagittal plane around BODY y.
//      - We first solve 2D forward kinematics in that sagittal plane, then rotate the resulting
//        horizontal projection by yaw into (x,z), and finally translate by (bx,bz).
//   2) Planar 2-link FK for femur/tibia in sagittal plane:
//      - Let a = FEMUR_LENGTH, b = TIBIA_LENGTH.
//      - Define femur pitch alpha (relative to vertical) and tibia relative angle gamma (knee).
//        These angles are derived from the absolute servo centidegrees and home angles to match
//        the IK convention used elsewhere in this codebase.
//      - The straight-line distance from femur base to foot is
//            D = sqrt(a^2 + b^2 + 2 a b cos(gamma))
//        (law of cosines; gamma = interior knee angle between femur and tibia).
//      - The elbow angle at the femur base that places the foot at distance D is
//            alpha2 = arccos((D^2 + a^2 - b^2) / (2 a D))
//        and the absolute femur inclination from vertical to foot is
//            alpha1 = alpha - alpha2.
//      - Then the vertical and horizontal (in-plane) components from the femur base are
//            y = D cos(alpha1)
//            R = D sin(alpha1)
//   3) Add Coxa radial and rotate by yaw:
//      - Add COXA_LENGTH in the horizontal (in-plane) direction, so the horizontal radius
//        from the yaw axis to the foot projection is rproj = COXA_LENGTH + R.
//      - Convert this radial to BODY-frame x/z using yaw:
//            x_local = rproj sin(yaw)
//            z_local = rproj cos(yaw)
//   4) Translate by chassis mount offset to BODY frame:
//            out_x = bx + x_local
//            out_y = y
//            out_z = bz + z_local
//
// References
//   - Planar 2-DOF arm forward kinematics (law of cosines form):
//       https://en.wikipedia.org/wiki/Forward_kinematics#Example:_Planar_2-DOF_robot
//   - Law of cosines and elbow-up/down solutions for 2-link manipulators:
//       https://en.wikipedia.org/wiki/Law_of_cosines
//   - Craig, J. J., Introduction to Robotics: Mechanics and Control, 3rd ed.,
//       Chapter 2 (planar manipulators) — standard treatment of 2R kinematics.
//   - Murray, Li, Sastry, "A Mathematical Introduction to Robotic Manipulation",
//       Section 2.2 (planar kinematics) — freely available online.
//
// Notes
//   - Angle offsets: The +90° (for femur) and +90°/home centidegree adjustments below align
//     this FK with calculateIK()’s convention so that FK(IK(target)) ≈ target.
//   - Sign conventions differ across robots; the formulas are invariant but the offset/signs
//     here are chosen to match this hardware’s mechanical zeroes and mounting.
// ----------------------------------------------------------------------------
// fk_leg_body moved to functions.ino

static inline void updateFootBodyEstimates() {
  for (uint8_t L = 0; L < NUM_LEGS; ++L) {
    int16_t c = g_meas_pos_valid[L][0] ? g_meas_pos_cd[L][0] : g_last_sent_cd[L][0];
    int16_t f = g_meas_pos_valid[L][1] ? g_meas_pos_cd[L][1] : g_last_sent_cd[L][1];
    int16_t t = g_meas_pos_valid[L][2] ? g_meas_pos_cd[L][2] : g_last_sent_cd[L][2];
    (void)fk_leg_body(L, c, f, t, &g_foot_body_x_mm[L], &g_foot_body_y_mm[L], &g_foot_body_z_mm[L]);
  }
}
// Feedback reliability tracking and out-of-service masking
volatile uint8_t g_fb_fail_count[NUM_LEGS][LEG_SERVOS] = { {0} };
volatile uint32_t g_servo_oos_mask = 0; // 18-bit mask: bit index (leg*3 + joint)
volatile uint8_t g_servo_fb_fail_threshold = 3; // configurable via /config.txt oos.fail_threshold
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

// Forward declaration so collision helpers can invoke a common STAND+DISABLE path
void safetyCollisionStandDisable();

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
LX16AServo* g_servo[NUM_LEGS][LEG_SERVOS] = { { nullptr } };
alignas(LX16AServo) uint8_t g_servo_mem[NUM_LEGS][LEG_SERVOS][sizeof(LX16AServo)];
static void servoObjectsInit() {
  for (uint8_t L = 0; L < NUM_LEGS; ++L) {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J) {
      g_servo[L][J] = nullptr;
      uint8_t id = servoId(L, J);
      if (id >= 1 && id <= 253 && SERVO_BUS[L]) {
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
      bool trip = (tC >= (g_safety_temp_lockout_c10 / 10));
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

// Collision safety helper: transition to STAND pose then enter lockout/disable.
// This is used by all collision-style violations (joint workspace, foot keep-out, etc.).
void safetyCollisionStandDisable() {
  if (g_lockout) return;

  // Request neutral stance first if we are not already locked out; this keeps
  // behavior consistent for all collision classes and avoids leaving legs in
  // extreme poses. MODE is forced to IDLE inside processCmdSTAND.
  extern void processCmdSTAND(const char* line, int s, int len);
  const char* cmd = "STAND";
  processCmdSTAND(cmd, 5, 5);

  g_lockout = true;
  g_enabled = false;
  g_last_err = E_LOCKOUT;
  g_mode = MODE_LOCKOUT;
  g_lockout_causes |= LOCKOUT_CAUSE_COLLISION;

  // Immediately torque off all servos after commanding STAND
  for (uint8_t L2 = 0; L2 < NUM_LEGS; ++L2) {
    for (uint8_t J2 = 0; J2 < LEG_SERVOS; ++J2) {
      setServoTorqueNow(L2, J2, false);
    }
  }
}

// Collision lockout helper for legacy callers (e.g., foot-to-foot keep-out).
// Now delegates to the common STAND+DISABLE collision path.
static inline void safetyLockoutCollision(uint8_t legA, uint8_t legB) {
  (void)legA;
  (void)legB;
  safetyCollisionStandDisable();
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

// --- FAST SEND SUPPORT (Option A) ---
static inline void oeSetTX(uint8_t leg, bool tx) {
  if (leg >= NUM_LEGS) return;
  int pin = Robot::BUFFER_ENABLE_PINS[leg];
  digitalWrite(pin, tx ? HIGH : LOW); // HIGH -> TX drive, LOW -> RX/high-Z
}

// Build a LewanSoul LX-16A MOVE_TIME frame into out[]; returns frame length (always 10)
static inline uint8_t build_move_time_frame(uint8_t id, uint16_t pos_units, uint16_t time_ms, uint8_t out[10]) {
  // Protocol: 0x55 0x55 ID LEN CMD P_L P_H T_L T_H CHECKSUM
  // LEN = number of bytes after LEN (CMD+PARAMS+CHECKSUM) => 7 for MOVE_TIME (4 params)
  // CHECKSUM = ~(ID + LEN + CMD + P_L + P_H + T_L + T_H) & 0xFF
  uint8_t idx = 0;
  out[idx++] = 0x55; out[idx++] = 0x55;
  out[idx++] = id;
  out[idx++] = 7;
  out[idx++] = LX16A_SERVO_MOVE_TIME_WRITE;
  out[idx++] = (uint8_t)(pos_units & 0xFF);
  out[idx++] = (uint8_t)(pos_units >> 8);
  out[idx++] = (uint8_t)(time_ms & 0xFF);
  out[idx++] = (uint8_t)(time_ms >> 8);
  uint16_t sum = (uint16_t)id + 7 + LX16A_SERVO_MOVE_TIME_WRITE + (pos_units & 0xFF) + (pos_units >> 8) + (time_ms & 0xFF) + (time_ms >> 8);
  out[idx++] = (uint8_t)(~(sum & 0xFF));
  return idx;
}

// Send up to 3 frames for a leg in a single OE window; optionally wait/return to RX for rr_leg
static inline void fastSendLeg(uint8_t leg, const int16_t out_cd[3], const bool joint_ok[3], uint16_t move_time_ms, bool is_rr_leg) {
  if (leg >= NUM_LEGS || !SERVO_BUS[leg]) return;
  // Assert TX enable once
  oeSetTX(leg, true);

  // Build and write frames back-to-back; skip disabled/OOS joints
  uint8_t frame[10];
  for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
    if (!joint_ok[j]) continue;
    uint8_t id = servoId(leg, j);
    if (id == 0 || id == 0xFF) continue;
    uint16_t units = cd_to_lx_units(out_cd[j]);
    uint8_t n = build_move_time_frame(id, units, move_time_ms, frame);
    // Teensy HardwareSerial::write supports buffer writes
    SERVO_BUS[leg]->write(frame, n);
    delayMicroseconds(25); // inter-frame gap; adjust as needed for bus timing
  }

  // Only on the RR leg we wait for TX complete and drop to RX, so reads are safe
  if (is_rr_leg) {
    SERVO_BUS[leg]->flush(); // wait for transmit complete
    oeSetTX(leg, false);     // return to RX
  }
  // For non-RR legs, we leave OE asserted; we'll drop to RX when that leg becomes RR
}
// end fast send helpers

// Forward decls (only those local to this TU)
static void loopTick();
// Config (implemented in functions.ino)
void configLoad();
void configWriteDefaults();
#if !(defined(MARS_ENABLE_SD) && MARS_ENABLE_SD)
static inline void configWriteDefaults() {}
#endif
// Hardware helpers; buffersInit implemented in functions.ino; servoBusesInit defined here
void buffersInit();
static void servoBusesInit(uint32_t baud = Robot::SERVO_BAUD_DEFAULT);
// no geometryInit needed with canonical-order arrays

// Morphology offsets available as Robot::COXA_OFFSET (see robot_config.h)

// Mode setters (exposed so command handler can change modes)
void modeSetTest() {
  g_mode = MODE_TEST;
}
void modeSetIdle() {
  g_mode = MODE_IDLE;
}

// Apply a new loop frequency from config or runtime request.
// Adjusts loop timer and mirrors into g_config_loop_hz for STATUS/reporting.
void configApplyLoopHz(uint16_t hz) {
  if (hz < 30) hz = 30;          // enforce a safe minimum to avoid overrun storms
  if (hz > 500) hz = 500;        // conservative upper bound for Teensy 4.1 + IO
  g_loop_hz = hz;
  g_config_loop_hz = hz;
  g_loopTimer.SetFrequency(g_loop_hz);
}

int16_t readServoPosCdSync(uint8_t leg, uint8_t joint) {
  if (leg >= NUM_LEGS || joint >= LEG_SERVOS) return -1;
  LX16AServo* s = g_servo[leg][joint];
  if (!s) return -1;
  int32_t pos = s->pos_read();
  // if (pos < 0) return -1;
  // if (pos > 24000) pos = 24000;
  return (int16_t)pos;
}

void setup() {
  // Optional brief settle to avoid early contention before USB enumeration and IO setup
  delay(2000);

  Serial.begin(115200);
  delay(1000); // allow time for Serial to initialize

  // Load config (SD) before splash so status reflects applied values
  configLoad();
  // Ensure all known config keys exist with defaults (creates missing keys)
  configWriteDefaults();
  delay(1000); // allow time for config load

  // Do not wait for Serial; print if available
  splash();

  // Safety: initialize commanded joint targets to mid-range (12000 centidegrees ≈ 120.00°)
  for (uint8_t L = 0; L < NUM_LEGS; ++L)

  {
    for (uint8_t J = 0; J < LEG_SERVOS; ++J)

    {
      g_cmd_cd[L][J] = g_home_cd[L][J];
      g_last_sent_cd[L][J] = g_home_cd[L][J];
      g_est_state[L][J].pos_cd = g_home_cd[L][J];
      g_est_state[L][J].vel_cds = 0;
    }
  }
  // Initialize foot estimates from home positions to avoid false startup collisions
  updateFootBodyEstimates();
  for (uint8_t L = 0; L < NUM_LEGS; ++L) {
    g_foot_target_x_mm[L] = g_foot_body_x_mm[L];
    g_foot_target_z_mm[L] = g_foot_body_z_mm[L];
  }

  buffersInit();
  servoBusesInit();
  servoObjectsInit();
  // Populate hardware angle offsets before any STATUS is printed/used
  refreshOffsetsAtStartup();
  g_boot_ms = millis();
  // Initialize uptime tracker
  g_uptime_prev_ms = g_boot_ms;
  g_uptime_ms64 = 0;
  // Configure loop timer from loop_hz (default now 100 Hz, spec target 166 Hz)
  g_loopTimer.SetFrequency(g_loop_hz);

#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // Derive initial sample divisor after config load (configLoad happens later if moved)
  if (g_log_rate_hz > 0 && g_loop_hz >= g_log_rate_hz) {
    uint16_t div = g_loop_hz / g_log_rate_hz;
    if (div < 1) div = 1; if (div > 255) div = 255;
    g_log_sample_div = (uint8_t)div;
  } else {
    g_log_sample_div = 1;
  }
  g_log_tick_counter = 0;
#endif
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
  // Update uptime accumulator (wrap-safe)
  {
    uint32_t now_ms = millis();
    uint32_t delta = now_ms - g_uptime_prev_ms; // unsigned wrap-safe
    g_uptime_prev_ms = now_ms;
    g_uptime_ms64 += (uint64_t)delta;
  }
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

      switch (L)
      {
        case LEG_RR:
        case LEG_LR:  // good
          // Left corners: rotate -45° in X/Z
          {
            float c = 0.70710678f; // cos(45°)
            xprime = -c * z + (c + 0.5f) * g_test_base_x_mm;
            zprime =  c * z + 0.5f       * g_test_base_x_mm;
          }
          break;
        case LEG_LF:  // good
        case LEG_RF:
          // Right corners: rotate +45° in X/Z
          {
            float c = 0.70710678f; // cos(45°)
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

      // Compute IK for this leg
      int16_t out_cd[3] = {0};

      // Update last foot targets (for safety keep-out)
      g_foot_target_x_mm[L] = xprime;
      g_foot_target_z_mm[L] = zprime;
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
        Serial.print(F(" x=")); Serial.print(xprime, 1);
        Serial.print(F(" y=")); Serial.print(y, 1);
        Serial.println(F(" z=")); Serial.print(zprime, 1);
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

  // TUCK controller: overrides gait/other commands while active.
  // Strategy:
  //  - Continuously drive tibia toward 0 cd for selected legs.
  //  - Once tibia within tolerance, also drive femur=19000 cd and coxa=0 cd each tick (not just once).
  //  - Completion when all selected legs have tibia within tolerance AND femur/coxa within tolerance (or timeout).
  if (g_tuck_active && !g_lockout) {
    const int16_t TIBIA_TARGET_CD = g_tuck_tibia_cd;      // base (left-side convention)
    const int16_t FEMUR_TARGET_CD = g_tuck_femur_cd;      // base (left-side convention)
    const int16_t COXA_TARGET_CD  = g_tuck_coxa_cd;       // absolute logical center
    const int16_t TOL_TIBIA_CD    = g_tuck_tol_tibia_cd;  // tolerance
    const int16_t TOL_OTHER_CD    = g_tuck_tol_other_cd;  // tolerance
    const uint32_t TIMEOUT_MS     = g_tuck_timeout_ms;    // safety timeout
    uint32_t now_ms = millis();
    uint8_t all_done = 1;
    for (uint8_t L = 0; L < NUM_LEGS; ++L) {
      if (((g_tuck_mask >> L) & 1u) == 0) continue;
      if (!legEnabled(L)) continue; // ignore disabled legs (treated as done)

      // Mirror femur/tibia targets for right-side legs so motion direction matches left
      bool isRight = (L == LEG_RF) || (L == LEG_RM) || (L == LEG_RR);
      int16_t tibia_target  = TIBIA_TARGET_CD;
      int16_t femur_target  = FEMUR_TARGET_CD;
      const int16_t coxa_target   = COXA_TARGET_CD; // unchanged by mirroring (center)
      if (isRight) {
        tibia_target = (int16_t)(2 * g_home_cd[L][JOINT_TIBIA] - tibia_target);
        femur_target = (int16_t)(2 * g_home_cd[L][JOINT_FEMUR] - femur_target);
      }

      // --- Tibia control ---
      if (jointEnabled(L, 2) && !servoIsOOS(L, 2)) {
        g_cmd_cd[L][2] = tibia_target;
      }
      int16_t t_meas = g_meas_pos_cd[L][2];
      int16_t t_eff  = g_meas_pos_valid[L][2] ? t_meas : g_last_sent_cd[L][2];
      int16_t t_err  = (int16_t)abs(tibia_target - t_eff);
      bool tibia_ok  = (t_err <= TOL_TIBIA_CD) || servoIsOOS(L, 2) || !jointEnabled(L, 2);

      // --- Femur & Coxa control (only after tibia acceptable) ---
      bool fem_coxa_ok = true; // assume ok if joints disabled/OOS
      if (tibia_ok) {
        if (jointEnabled(L, 0) && !servoIsOOS(L, 0)) g_cmd_cd[L][0] = coxa_target;
        if (jointEnabled(L, 1) && !servoIsOOS(L, 1)) g_cmd_cd[L][1] = femur_target;
        // Check their convergence
        int16_t f_meas = g_meas_pos_cd[L][1];
        int16_t f_eff  = g_meas_pos_valid[L][1] ? f_meas : g_last_sent_cd[L][1];
        int16_t f_err  = (int16_t)(femur_target - f_eff); if (f_err < 0) f_err = (int16_t) - f_err;
        if (!(servoIsOOS(L, 1) || !jointEnabled(L, 1))) fem_coxa_ok &= (f_err <= TOL_OTHER_CD);
        int16_t c_meas = g_meas_pos_cd[L][0];
        int16_t c_eff  = g_meas_pos_valid[L][0] ? c_meas : g_last_sent_cd[L][0];
        int16_t c_err  = (int16_t)(coxa_target - c_eff); if (c_err < 0) c_err = (int16_t) - c_err;
        if (!(servoIsOOS(L, 0) || !jointEnabled(L, 0))) fem_coxa_ok &= (c_err <= TOL_OTHER_CD);
        if (tibia_ok && fem_coxa_ok && ((g_tuck_done_mask >> L) & 1u) == 0) {
          g_tuck_done_mask |= (1u << L);
        }
      } else {
        fem_coxa_ok = false; // not yet driving
      }
      if (((g_tuck_done_mask >> L) & 1u) == 0) all_done = 0;
    }
    if (all_done || (now_ms - g_tuck_start_ms) > TIMEOUT_MS) {
      g_tuck_active = 0; g_tuck_mask = 0; g_tuck_done_mask = 0; g_tuck_start_ms = 0;
    }
  }

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
      // Update estimator prediction for all servos (position + velocity) toward
      // the last effective command (before PID). Effective command includes
      // compliance offsets so the estimator tracks what the joints are actually
      // being asked to do. Measurement correction is applied later in the RR
      // feedback block when fresh data is available.
      {
        uint16_t a_cmd = g_est_cmd_alpha_milli; if (a_cmd > 1000) a_cmd = 1000;
        float dt_s = g_loopTimer.DeltaTseconds();
        if (!(dt_s > 0.0f) || dt_s > 0.100f) {
          dt_s = (g_loop_hz > 0) ? (1.0f / (float)g_loop_hz) : 0.006f;
        }
        for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
          for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
            JointEstimator &st = g_est_state[leg][j];
            // 1) Predict using current velocity
            int32_t pos = st.pos_cd + (int32_t)(st.vel_cds * dt_s);
            if (pos < 0) pos = 0; else if (pos > 24000) pos = 24000;
            // 2) Small pull toward effective command to keep estimate near plan
            int32_t cmd = (int32_t)g_eff_cmd_cd[leg][j];
            int32_t diff_cmd = cmd - pos;
            pos += (a_cmd * diff_cmd) / 1000;
            if (pos < 0) pos = 0; else if (pos > 24000) pos = 24000;
            // 3) Simple velocity decay toward 0 to avoid drift when idle
            st.vel_cds = (int16_t)((int32_t)st.vel_cds * 9 / 10); // ~0.9 per tick
            st.pos_cd = (int16_t)pos;
          }
        }
      }

      // Build both base (no PID) and PID-corrected desired targets; apply only PID in active mode.
      int16_t base_target_cd[NUM_LEGS][LEG_SERVOS];
      int16_t pid_target_cd[NUM_LEGS][LEG_SERVOS];
      // dt-aware PID: derive elapsed time since last tick
      float dt_s = g_loopTimer.DeltaTseconds();
      if (!(dt_s > 0.0f) || dt_s > 0.100f) {
        // Fallback to nominal period if dt is invalid or absurdly large (>100 ms)
        dt_s = (g_loop_hz > 0) ? (1.0f / (float)g_loop_hz) : 0.006f;
      }
      // Convert to milliseconds for fixed-point arithmetic in I/D terms
      uint16_t dt_ms = (uint16_t)(dt_s * 1000.0f + 0.5f);
      if (dt_ms < 1) dt_ms = 1;

      // Joint compliance update: adapt per-leg/joint offsets around estimated position.
      // This uses the previous tick's effective command as the reference for error.
      if (g_comp_enabled) {
        for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
          for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
            // Skip joints that are disabled or out-of-service
            if (!jointEnabled(leg, j) || servoIsOOS(leg, j)) {
              // Passive decay toward zero when compliance is enabled but joint not active
              int16_t off = g_comp_offset_cd[leg][j];
              if (off != 0) {
                int32_t leak = ((int32_t)g_comp_leak_milli * (int32_t)off) / 1000;
                off -= (int16_t)leak;
                g_comp_offset_cd[leg][j] = off;
              }
              continue;
            }

            int16_t eff_prev = g_eff_cmd_cd[leg][j];
            int16_t est      = g_est_state[leg][j].pos_cd;
            int32_t e_cd     = (int32_t)est - (int32_t)eff_prev;

            // Deadband around zero error
            int32_t db = (int32_t)g_comp_deadband_cd[j];
            if (e_cd > -db && e_cd < db) {
              // Leak compliance offset toward zero when quiet
              int16_t off = g_comp_offset_cd[leg][j];
              if (off != 0) {
                int32_t leak = ((int32_t)g_comp_leak_milli * (int32_t)off) / 1000;
                off -= (int16_t)leak;
                g_comp_offset_cd[leg][j] = off;
              }
              continue;
            }

            // P-like update on compliance offset
            int32_t off = g_comp_offset_cd[leg][j];
            int32_t kp  = (int32_t)g_comp_kp_milli[j];
            off += (kp * e_cd) / 1000;

            // Clamp offset to configured range
            int32_t r = (int32_t)g_comp_range_cd[j];
            if (off >  r) off =  r;
            if (off < -r) off = -r;
            if (off >  32767) off =  32767;
            if (off < -32768) off = -32768;
            g_comp_offset_cd[leg][j] = (int16_t)off;
          }
        }
      } else {
        // Ensure offsets are neutral when compliance is globally disabled
        for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
          for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
            g_comp_offset_cd[leg][j] = 0;
          }
        }
      }

      // Build effective commands from planner outputs plus compliance offsets.
      // When collision safety is enabled, also enforce the configured joint
      // workspace (joint_limits.*). If an effective command attempts to leave
      // this workspace, treat it as a collision: trigger a STAND+DISABLE
      // sequence via safetyCollisionStandDisable().
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
          int16_t base = g_cmd_cd[leg][j];
          int16_t off  = g_comp_offset_cd[leg][j];
          int32_t eff  = (int32_t)base + (int32_t)off;

          // Servo hardware range clamp
          if (eff < 0) eff = 0;
          if (eff > 24000) eff = 24000;

          // Joint workspace enforcement when collision safety is enabled.
          if (g_safety_collision_enabled && !g_lockout) {
            int16_t min_cd = g_limit_min_cd[leg][j];
            int16_t max_cd = g_limit_max_cd[leg][j];
            if (min_cd < 0) min_cd = 0;
            if (max_cd > 24000) max_cd = 24000;
            if (min_cd > max_cd) {
              int16_t tmp = min_cd;
              min_cd = max_cd;
              max_cd = tmp;
            }
            if (eff < (int32_t)min_cd || eff > (int32_t)max_cd) {
              // Record collision-style error and transition to STAND+DISABLE.
              g_last_err = E_COLLISION;
              safetyCollisionStandDisable();
              // After lockout, stop updating further effective commands this tick.
              return;
            }
          }

          g_eff_cmd_cd[leg][j] = (int16_t)eff;
        }
      }

      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
          int16_t des = g_eff_cmd_cd[leg][j];
          base_target_cd[leg][j] = des; // raw desired before PID (after compliance)
          // Compute PID corrected target if enabled
          if (g_pid_enabled) {
            int16_t meas = g_est_state[leg][j].pos_cd;
            int16_t err = (int16_t)(des - meas);
            int32_t p = ((int32_t)g_pid_kp_milli[j] * (int32_t)err) / 1000;
            int32_t i = 0;
            if (g_pid_ki_milli[j] != 0) {
              // integrate error in cd·ms to be robust to variable dt
              int32_t acc = g_pid_i_accum[leg][j] + (int32_t)((int32_t)err * (int32_t)dt_ms);
              // clamp integral accumulator (cd·ms)
              if (acc > 2000000L) acc = 2000000L; else if (acc < -2000000L) acc = -2000000L;
              g_pid_i_accum[leg][j] = acc;
              // Ki[milli] * (cd·ms) / (1000[milli]*1000[ms/s]) -> cd
              int64_t i64 = ((int64_t)g_pid_ki_milli[j] * (int64_t)acc);
              i = (int32_t)(i64 / 1000000LL);
            }
            int32_t d = 0;
            if (g_pid_kd_milli[j] != 0) {
              int16_t prev = g_pid_prev_err[leg][j];
              int16_t derr = (int16_t)(err - prev);
              g_pid_prev_err[leg][j] = err;
              // Convert per-tick error delta to per-second using dt (fixed-point: *1000/dt_ms)
              int32_t der_per_s = (int32_t)((int32_t)derr * 1000) / (int32_t)dt_ms; // cd/s
              int32_t df = g_pid_d_filt[leg][j];
              uint16_t a = g_pid_kd_alpha_milli[j]; if (a > 1000) a = 1000;
              // Low-pass filter the derivative signal toward der_per_s
              df = df + (int32_t)((a * (der_per_s - df)) / 1000);
              if (df > 32767)
                df = 32767;
              else if (df < -32768)
                df = -32768;
              g_pid_d_filt[leg][j] = (int16_t)df;
              d = ((int32_t)g_pid_kd_milli[j] * (int32_t)g_pid_d_filt[leg][j]) / 1000;
            }
            int32_t corr = p + i + d;
            if (corr > PID_CORR_CLAMP_CD)
              corr = PID_CORR_CLAMP_CD;
            else if (corr < -PID_CORR_CLAMP_CD)
              corr = -PID_CORR_CLAMP_CD;
            int32_t des_corr = (int32_t)des + corr;
            if (des_corr < 0)
              des_corr = 0;
            else if (des_corr > 24000)
              des_corr = 24000;
            pid_target_cd[leg][j] = (int16_t)des_corr;
          } else {
            pid_target_cd[leg][j] = des; // same as base when PID disabled
          }
        }
      }

      // Apply safety + rate limit separately for base and pid targets (shadow comparison uses both)
      int16_t out_base_cd[NUM_LEGS][LEG_SERVOS];
      int16_t out_pid_cd[NUM_LEGS][LEG_SERVOS];
      bool    ok_grid[NUM_LEGS][LEG_SERVOS];
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        bool leg_on = legEnabled(leg);
        for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
          bool ok = leg_on && jointEnabled(leg, j) && !servoIsOOS(leg, j);
          ok_grid[leg][j] = ok;
          int16_t prev_cd = g_last_sent_cd[leg][j];
          int32_t base_t = base_target_cd[leg][j];
          int32_t pid_t  = pid_target_cd[leg][j];
          if (ok) {
            if (g_safety_soft_limits_enabled) {
              int16_t min_cd = g_limit_min_cd[leg][j];
              int16_t max_cd = g_limit_max_cd[leg][j];
              if (base_t < min_cd) base_t = min_cd; if (base_t > max_cd) base_t = max_cd;
              if (pid_t  < min_cd) pid_t  = min_cd;  if (pid_t  > max_cd) pid_t  = max_cd;
            }
            int32_t base_delta = base_t - prev_cd;
            int32_t pid_delta  = pid_t  - prev_cd;
            if (max_delta_cd > 0) {
              if (base_delta > max_delta_cd) base_delta = max_delta_cd; else if (base_delta < -max_delta_cd) base_delta = -max_delta_cd;
              if (pid_delta  > max_delta_cd) pid_delta  = max_delta_cd;  else if (pid_delta  < -max_delta_cd)  pid_delta  = -max_delta_cd;
            }
            out_base_cd[leg][j] = (int16_t)(prev_cd + base_delta);
            out_pid_cd[leg][j]  = (int16_t)(prev_cd + pid_delta);
          } else {
            out_base_cd[leg][j] = prev_cd;
            out_pid_cd[leg][j]  = prev_cd;
          }
        }
      }

      // Select which outputs to actually send based on PID mode, then apply optional impedance
      int16_t out_drive_cd[NUM_LEGS][LEG_SERVOS];
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
          out_drive_cd[leg][j] = (g_pid_enabled && g_pid_mode == 0) ? out_pid_cd[leg][j] : out_base_cd[leg][j];
        }
      }

      // TEMP: LF coxa debug stream — command, PID, last measurement, estimator (centideg), CSV per tick
      // Guarded by MARS_DEBUG_LF_COXA_EST so it can be compiled out for normal runs.
#ifdef MARS_DEBUG_LF_COXA_EST
      {
        const uint8_t leg = 0;      // LF
        const uint8_t joint = 0;    // COXA
        int16_t base_cmd = base_target_cd[leg][joint];
        int16_t pid_cmd  = out_pid_cd[leg][joint];
        int16_t est_cd   = g_est_state[leg][joint].pos_cd;
        int16_t meas_cd  = g_meas_pos_valid[leg][joint] ? g_meas_pos_cd[leg][joint] : -1;
        // Format (no labels): base_cmd_cd,pid_cmd_cd,meas_cd,est_cd
        Serial.print(base_cmd);
        Serial.print(',');
        Serial.print(pid_cmd);
        Serial.print(',');
        Serial.print(meas_cd);
        Serial.print(',');
        Serial.print(est_cd);
        Serial.print('\n');
      }
#endif

      // Impedance layer: compute per-leg corrections and blend into out_drive_cd
      if (g_impedance.config().enabled && g_impedance.config().mode != IMP_MODE_OFF) {
        for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
          // Skip legs with no active joints
          bool has_any = ok_grid[leg][0] || ok_grid[leg][1] || ok_grid[leg][2];
          if (!has_any) continue;

          int16_t est_cd[3];
          int16_t cmd_cd[3];
          for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
            est_cd[j] = g_est_state[leg][j].pos_cd;
            cmd_cd[j] = out_drive_cd[leg][j];
          }

          float bodyFoot[3];
          float bodyRef[3];
          const float* pFoot = nullptr;
          const float* pRef  = nullptr;
          if (g_impedance.config().mode == IMP_MODE_CART) {
            // Use BODY-frame FK estimate as current foot position
            bodyFoot[0] = g_foot_body_x_mm[leg];
            bodyFoot[1] = g_foot_body_y_mm[leg];
            bodyFoot[2] = g_foot_body_z_mm[leg];
            // Use last commanded foot target in X/Z and current BASE_Y as reference
            bodyRef[0] = g_foot_target_x_mm[leg];
            bodyRef[1] = g_test_base_y_mm;
            bodyRef[2] = g_foot_target_z_mm[leg];
            pFoot = bodyFoot;
            pRef  = bodyRef;
          }

          int16_t corr_cd[3] = {0, 0, 0};
          g_impedance.computeLegCorrection(leg, est_cd, cmd_cd, dt_s, pFoot, pRef, corr_cd);

          for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
            if (!ok_grid[leg][j]) continue;
            int32_t v = (int32_t)out_drive_cd[leg][j] + (int32_t)corr_cd[j];
            if (v < 0) v = 0; else if (v > 24000) v = 24000;
            out_drive_cd[leg][j] = (int16_t)v;
          }
        }
      }

      // Update last_sent to what we will drive (estimator & rate-limit reference)
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        for (uint8_t j = 0; j < LEG_SERVOS; ++j) {
          g_last_sent_cd[leg][j] = out_drive_cd[leg][j];
        }
      }

      // Parallelize across UARTs by batching per leg and avoiding per-frame waits
      uint8_t rr_leg = g_rr_index / LEG_SERVOS;
      for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
        bool has_any = ok_grid[leg][0] || ok_grid[leg][1] || ok_grid[leg][2];
        if (!has_any) continue;
        bool is_rr = (leg == rr_leg);
        // Use a small non-zero move time so each incremental step is treated
        // as a short ramp by the servo. Approximate one control tick.
        uint16_t move_ms = (g_loop_hz > 0) ? (uint16_t)(1000U / g_loop_hz) : 0U;
        if (move_ms == 0) move_ms = 1; // floor to 1 ms when loop_hz is high or zero
        fastSendLeg(leg, out_drive_cd[leg], ok_grid[leg], move_ms, is_rr);
      }
      // Shadow mode streaming: periodically print per-leg summary without driving PID corrections.
      // Also surface estimator details (actual vs estimate vs velocity) for debugging.
      if (g_pid_enabled && g_pid_mode == 1 && g_pid_shadow_report_hz > 0) {
        uint32_t now_ms = millis();
        uint32_t interval_ms = (uint32_t)(1000UL / (uint32_t)g_pid_shadow_report_hz);
        if (interval_ms == 0) interval_ms = 1;
        if (now_ms - g_pid_shadow_last_report_ms >= interval_ms) {
          g_pid_shadow_last_report_ms = now_ms;
          if (Serial) {
            const char* legNames[6]  = {"LF", "LM", "LR", "RF", "RM", "RR"};
            Serial.print(F("PID_SHADOW t_ms=")); Serial.print((unsigned long)now_ms);
            for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
              Serial.print(F(" ")); Serial.print(legNames[leg]); Serial.print(F(":"));
              // diffs (pid - base) in centidegrees per joint
              int16_t d_c = (int16_t)(out_pid_cd[leg][0] - out_base_cd[leg][0]);
              int16_t d_f = (int16_t)(out_pid_cd[leg][1] - out_base_cd[leg][1]);
              int16_t d_t = (int16_t)(out_pid_cd[leg][2] - out_base_cd[leg][2]);
              // error used by PID (des - est), using estimate (meas-corrected on RR)
              int16_t e_c = (int16_t)(base_target_cd[leg][0] - g_est_state[leg][0].pos_cd);
              int16_t e_f = (int16_t)(base_target_cd[leg][1] - g_est_state[leg][1].pos_cd);
              int16_t e_t = (int16_t)(base_target_cd[leg][2] - g_est_state[leg][2].pos_cd);
              // estimate/read (centidegrees)
              int16_t m_c = g_est_state[leg][0].pos_cd;
              int16_t m_f = g_est_state[leg][1].pos_cd;
              int16_t m_t = g_est_state[leg][2].pos_cd;
              // Measured positions (last RR read, may be stale) for comparison
              int16_t z_c = g_meas_pos_valid[leg][0] ? g_meas_pos_cd[leg][0] : -1;
              int16_t z_f = g_meas_pos_valid[leg][1] ? g_meas_pos_cd[leg][1] : -1;
              int16_t z_t = g_meas_pos_valid[leg][2] ? g_meas_pos_cd[leg][2] : -1;
              // Estimated velocities (cd/s)
              int16_t v_c = g_est_state[leg][0].vel_cds;
              int16_t v_f = g_est_state[leg][1].vel_cds;
              int16_t v_t = g_est_state[leg][2].vel_cds;
              // base target (pre-PID) in centidegrees
              int16_t t_c = base_target_cd[leg][0];
              int16_t t_f = base_target_cd[leg][1];
              int16_t t_t = base_target_cd[leg][2];
              Serial.print(F(" diff_cd=")); Serial.print((int)d_c); Serial.print(F("/")); Serial.print((int)d_f); Serial.print(F("/")); Serial.print((int)d_t);
              Serial.print(F(" err_cd="));  Serial.print((int)e_c); Serial.print(F("/")); Serial.print((int)e_f); Serial.print(F("/")); Serial.print((int)e_t);
              Serial.print(F(" est_cd="));  Serial.print((int)m_c); Serial.print(F("/")); Serial.print((int)m_f); Serial.print(F("/")); Serial.print((int)m_t);
              Serial.print(F(" meas_cd=")); Serial.print((int)z_c); Serial.print(F("/")); Serial.print((int)z_f); Serial.print(F("/")); Serial.print((int)z_t);
              Serial.print(F(" est_vel_cds=")); Serial.print((int)v_c); Serial.print(F("/")); Serial.print((int)v_f); Serial.print(F("/")); Serial.print((int)v_t);
              Serial.print(F(" tgt_cd="));  Serial.print((int)t_c); Serial.print(F("/")); Serial.print((int)t_f); Serial.print(F("/")); Serial.print((int)t_t);
            }
            Serial.print(F("\r\n"));
          }
        }
      }
      // Ensure RR leg is in RX before feedback even if it had nothing to send this tick
      SERVO_BUS[rr_leg]->flush();
      oeSetTX(rr_leg, false);
      // end fast send path
    }
#if MARS_TIMING_PROBES
    g_probe_send_us = (uint16_t)send_us;
#endif
#if MARS_PROFILE_SEND
    // Update per-tick aggregation when any sends potentially occurred
    uint16_t __tick_send = (uint16_t)send_us;
    g_prof_send_tick_us_sum += (uint32_t)__tick_send;
    if (__tick_send < g_prof_send_tick_us_min) g_prof_send_tick_us_min = __tick_send;
    if (__tick_send > g_prof_send_tick_us_max) g_prof_send_tick_us_max = __tick_send;
    ++g_prof_send_tick_count;
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
            g_meas_pos_valid[leg][joint] = 1;
            // Measurement correction for estimator: update position and velocity
            {
              JointEstimator &st = g_est_state[leg][joint];
              int32_t pos = st.pos_cd;
              int32_t meas = g_meas_pos_cd[leg][joint];
              uint16_t a_pos = g_est_meas_alpha_milli; if (a_pos > 1000) a_pos = 1000;
              // Position update toward measurement
              int32_t resid = meas - pos;
              pos = pos + (int32_t)((a_pos * resid) / 1000);
              if (pos < 0) pos = 0; else if (pos > 24000) pos = 24000;
              st.pos_cd = (int16_t)pos;
              // Velocity update based on residual / dt
              float dt_s = g_loopTimer.DeltaTseconds();
              if (!(dt_s > 0.0f) || dt_s > 0.100f) {
                dt_s = (g_loop_hz > 0) ? (1.0f / (float)g_loop_hz) : 0.006f;
              }
              if (dt_s > 0.0f) {
                int32_t dv = (int32_t)((g_est_meas_vel_alpha_milli * (resid / dt_s)) / 1000);
                int32_t vel_est = (int32_t)st.vel_cds + dv;
                if (vel_est < -240000) vel_est = -240000; else if (vel_est > 240000) vel_est = 240000;
                st.vel_cds = (int16_t)vel_est;
              }
            }
          } else {
            g_meas_pos_valid[leg][joint] = 0;
          }

          // vin/temp read drive OOS detection
          // Alternate vin and temp across visits; always retain last known values
          uint8_t tog = (g_fb_rr_toggle[leg][joint] ^= 1); // flip 0<->1
          if (tog == 0) {
            uint16_t vin_mV = s->vin();
            g_meas_vin_mV[leg][joint] = vin_mV;
          } else {
            uint8_t temp_C = s->temp();
            g_meas_temp_C[leg][joint] = temp_C;
          }

          // Validity: accept either stored reading being sane as a 'success'
          bool validVin  = (g_meas_vin_mV[leg][joint] > 0); // lenient: any non-zero
          bool validTemp = (g_meas_temp_C[leg][joint] > 0 && g_meas_temp_C[leg][joint] < 400); // 1..119 C

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

      // Print FK and S1/S2-style telemetry for the leg served this tick
      {
        int16_t c = g_meas_pos_valid[leg][0] ? g_meas_pos_cd[leg][0] : g_last_sent_cd[leg][0];
        int16_t f = g_meas_pos_valid[leg][1] ? g_meas_pos_cd[leg][1] : g_last_sent_cd[leg][1];
        int16_t t = g_meas_pos_valid[leg][2] ? g_meas_pos_cd[leg][2] : g_last_sent_cd[leg][2];
        float bx, by, bz, lx, ly, lz;
        if (fk_leg_both(leg, c, f, t, &bx, &by, &bz, &lx, &ly, &lz)) {
          if (Serial && ((g_fk_stream_mask >> leg) & 1u)) {
            const char* legNames[6]  = {"LF", "LM", "LR", "RF", "RM", "RR"};
            // Existing RR_FK stream (unchanged format)
            Serial.print(F("RR_FK "));
            Serial.print(legNames[leg]);
            Serial.print(F(" x=")); Serial.print(bx, 1);
            Serial.print(F(" y=")); Serial.print(by, 1);
            Serial.print(F(" z=")); Serial.print(bz, 1);
            Serial.print(F(" lx=")); Serial.print(lx, 1);
            Serial.print(F(" ly=")); Serial.print(ly, 1);
            Serial.print(F(" lz=")); Serial.print(lz, 1);
            Serial.print(F(" c=")); Serial.print((int)c);
            Serial.print(F(" f=")); Serial.print((int)f);
            Serial.print(F(" t=")); Serial.print((int)t);
            Serial.print(F("\r\n"));
          }
        }
      }
    }
#if 1
    // New compact segments for Pi display pipeline — independent of FK mask
    if (Serial && g_telem_enabled) {
      if (g_telem_bin_enabled) {
        telemetryBinS1((uint16_t)g_probe_tick_us,
                       g_lockout ? 1 : 0,
                       (uint8_t)(g_mode == MODE_TEST ? 1 : 0),
                       (uint8_t)(g_mode == MODE_TEST ? test_phase : 0),
                       g_rr_index,
                       g_enabled ? 1 : 0);
        telemetryBinS2();
        telemetryBinS3();
        telemetryBinS4();
        telemetryBinS5();
      } else {
        telemetryPrintS1((uint8_t)leg,
                         (uint16_t)g_probe_tick_us,
                         g_lockout ? 1 : 0,
                         (uint8_t)(g_mode == MODE_TEST ? 1 : 0),
                         (uint8_t)(g_mode == MODE_TEST ? test_phase : 0),
                         g_rr_index,
                         g_enabled ? 1 : 0);

        // S2: enable flags (all 18); S3: voltage/temperature (all 18);
        // S4: per-leg contact flags (currently stubbed to 0);
        // S5: detailed safety state/config snapshot.
        telemetryPrintS2();
        telemetryPrintS3();
        telemetryPrintS4();
        telemetryPrintS5();
      }
    }
#endif
#if MARS_TIMING_PROBES
    g_probe_fb_us = (uint16_t)(micros() - fb_start_us);
#endif
  }

  // Update BODY-frame foot position estimates before collision checks
  updateFootBodyEstimates();

  // Safety check: keep-out collision between feet in X/Z plane (applies in all modes)
  if (!g_lockout)
  {
    float clr = g_safety_clearance_mm;
    if (g_safety_collision_enabled && clr > 0.0f) {
      float r2 = clr * clr;
      const uint8_t pairs[][2] = {
        // Same-side adjacent pairs
        {LEG_LF, LEG_LM}, {LEG_LM, LEG_LR},
        {LEG_RF, LEG_RM}, {LEG_RM, LEG_RR},
        // Opposite-side facing pairs
        {LEG_LF, LEG_RF}, {LEG_LM, LEG_RM}, {LEG_LR, LEG_RR}
      };
      const char* legNames[6]  = {"LF", "LM", "LR", "RF", "RM", "RR"};

      const uint8_t numPairs = (uint8_t)(sizeof(pairs) / sizeof(pairs[0]));
      for (uint8_t idx = 0; idx < numPairs && !g_lockout; ++idx) {
        uint8_t i = pairs[idx][0];
        uint8_t j = pairs[idx][1];
        float dx = g_foot_body_x_mm[i] - g_foot_body_x_mm[j];
        float dz = g_foot_body_z_mm[i] - g_foot_body_z_mm[j];
        float d2 = dx * dx + dz * dz;
        if (d2 < r2) {
          if (Serial) {
            Serial.print(F("COLLISION "));
            Serial.print(legNames[i]); Serial.print(F("-")); Serial.print(legNames[j]);
            Serial.print(F(" d_mm=")); Serial.print(sqrtf(d2), 1);
            Serial.print(F(" < clr=")); Serial.print(clr, 1);
            Serial.print(F("\r\n"));
          }
          bool coll_overridden = (g_override_mask & LOCKOUT_CAUSE_COLLISION) != 0;
          if (!coll_overridden) {
            safetyLockoutCollision(i, j);
          }
          break;
        }
      }
    }
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
          if (g_meas_temp_C[L][J] >= (g_safety_temp_lockout_c10 / 10))
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
  bool overrun = (period_us > 0 && tick_elapsed_us > period_us);
  if (overrun) {
    g_last_err = E_OVERRUN;
    ++g_overrun_count;
  }
  // Windowed adaptive logic over 128 ticks:
  // - Zero overruns: increase loop_hz by 1 toward ceiling.
  // - >=8 overruns: decrease loop_hz by 1 (floor 30 Hz).
  ++g_tick_window_count; if (overrun) ++g_tick_window_overruns;
  if (g_tick_window_count >= 128) {
    uint16_t ceiling = g_config_loop_hz ? g_config_loop_hz : 166; if (ceiling > 500) ceiling = 500;
    const uint8_t DOWN_THRESHOLD = 8;
    if (g_tick_window_overruns == 0) {
      if (g_loop_hz < ceiling) {
        ++g_loop_hz;
        g_loopTimer.SetFrequency(g_loop_hz);
      }
    } else if (g_tick_window_overruns >= DOWN_THRESHOLD) {
      if (g_loop_hz > 30) {
        --g_loop_hz;
        g_loopTimer.SetFrequency(g_loop_hz);
      }
    }
    g_tick_window_count = 0; g_tick_window_overruns = 0;
    g_ok_since_adjust = 0; g_overrun_since_adjust = 0; // reset legacy counters
  }
#if MARS_TIMING_PROBES
  g_probe_tick_us = (uint16_t)tick_elapsed_us;
  // Update jitter metrics (absolute timing error)
  if (period_us > 0) {
    uint32_t diff = (tick_elapsed_us > period_us) ? (tick_elapsed_us - period_us) : (period_us - tick_elapsed_us);
    uint16_t jd = (diff > 0xFFFFu) ? 0xFFFFu : (uint16_t)diff;
    if (g_jitter_min_us == 0xFFFFu || jd < g_jitter_min_us) g_jitter_min_us = jd;
    if (jd > g_jitter_max_us) g_jitter_max_us = jd;
    g_jitter_sum_us += (uint32_t)jd;
    if (++g_jitter_count >= 128) {
      // Reset window each 128 samples; STATUS will read current snapshot
      g_jitter_min_us = 0xFFFFu; g_jitter_max_us = 0; g_jitter_sum_us = 0; g_jitter_count = 0;
    }
  }
#endif

  //#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD
  // --- Buffered CSV logging (compact/full modes) ---
  if (g_log_enabled) {
    ++g_log_tick_counter;
    if (g_log_sample_div == 0) g_log_sample_div = 1;
    bool sample_now = ((g_log_tick_counter % g_log_sample_div) == 0);
    if (sample_now) {
      uint32_t log_start_us = micros();
      // Ensure file is open
      if (!g_log_file) {
        if (SD.begin(BUILTIN_SDCARD)) {
          SD.mkdir("/logs");
          char fname[64];
          uint32_t ms = millis();
          snprintf(fname, sizeof(fname), "/logs/%lu_seq%lu.csv", (unsigned long)ms, (unsigned long)g_log_seq);
          g_log_file = SD.open(fname, FILE_WRITE);
          if (g_log_file && g_log_header) {
            g_log_file.print(F("# MARS — Modular Autonomous Robotic System\r\n"));
            g_log_file.print(F("# FW ")); g_log_file.print(FW_VERSION);
            g_log_file.print(F(" build=")); g_log_file.print(__DATE__); g_log_file.print(F(" ")); g_log_file.print(__TIME__);
            g_log_file.print(F("\r\n# loop_hz=")); g_log_file.print((unsigned int)g_loop_hz);
            g_log_file.print(F(" log_rate_hz=")); g_log_file.print((unsigned int)g_log_rate_hz);
            g_log_file.print(F(" mode=")); g_log_file.print((unsigned int)g_log_mode);
            g_log_file.print(F(" div=")); g_log_file.print((unsigned int)g_log_sample_div);
            g_log_file.print(F(" rotate=")); g_log_file.print(g_log_rotate ? 1 : 0);
            g_log_file.print(F(" max_bytes=")); g_log_file.print((unsigned long)g_log_max_bytes);
            g_log_file.print(F(" seq=")); g_log_file.print((unsigned long)g_log_seq);
            g_log_file.print(F("\r\n# columns: time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err\r\n"));
            g_log_file.print(F("time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err\r\n"));
            g_log_file_bytes = g_log_file.size();
          }
          g_log_buf_used = 0;
        } else {
          g_log_enabled = false; // SD init failure
        }
      }

      if (g_log_file) {
        uint32_t t_ms = millis();
        uint8_t err = g_last_err;
        if (g_log_mode == 0) {
          uint8_t leg = g_rr_index / LEG_SERVOS;
          char row[192];
          int16_t cmd0 = g_last_sent_cd[leg][0];
          int16_t cmd1 = g_last_sent_cd[leg][1];
          int16_t cmd2 = g_last_sent_cd[leg][2];
          int16_t meas0 = g_meas_pos_cd[leg][0];
          int16_t meas1 = g_meas_pos_cd[leg][1];
          int16_t meas2 = g_meas_pos_cd[leg][2];
          uint16_t vin0_mV = g_meas_vin_mV[leg][0];
          uint16_t vin1_mV = g_meas_vin_mV[leg][1];
          uint16_t vin2_mV = g_meas_vin_mV[leg][2];
          uint8_t temp0C = g_meas_temp_C[leg][0];
          uint8_t temp1C = g_meas_temp_C[leg][1];
          uint8_t temp2C = g_meas_temp_C[leg][2];
          uint8_t oos0 = servoIsOOS(leg, 0) ? 1 : 0;
          uint8_t oos1 = servoIsOOS(leg, 1) ? 1 : 0;
          uint8_t oos2 = servoIsOOS(leg, 2) ? 1 : 0;
          int vin0_whole = vin0_mV / 1000; int vin0_tenth = (vin0_mV % 1000) / 100;
          int vin1_whole = vin1_mV / 1000; int vin1_tenth = (vin1_mV % 1000) / 100;
          int vin2_whole = vin2_mV / 1000; int vin2_tenth = (vin2_mV % 1000) / 100;
          int n = snprintf(row, sizeof(row), "%lu,%u,%d,%d,%d.%d,%u,%u,%d,%d,%d.%d,%u,%u,%d,%d,%d.%d,%u,%u,%u\r\n",
                           (unsigned long)t_ms, (unsigned int)leg,
                           (int)cmd0, (int)meas0, vin0_whole, vin0_tenth, (unsigned int)temp0C, (unsigned int)oos0,
                           (int)cmd1, (int)meas1, vin1_whole, vin1_tenth, (unsigned int)temp1C, (unsigned int)oos1,
                           (int)cmd2, (int)meas2, vin2_whole, vin2_tenth, (unsigned int)temp2C, (unsigned int)oos2,
                           (unsigned int)err);
          if (n > 0 && (g_log_buf_used + (uint16_t)n) < sizeof(g_log_buf)) {
            memcpy(g_log_buf + g_log_buf_used, row, (size_t)n);
            g_log_buf_used += (uint16_t)n;
          }
          else {
            g_log_file.write(g_log_buf, g_log_buf_used); g_log_file_bytes += g_log_buf_used; g_log_total_bytes += g_log_buf_used; g_log_buf_used = 0;
            if (n > 0 && (uint16_t)n < sizeof(g_log_buf)) {
              memcpy(g_log_buf, row, (size_t)n);
              g_log_buf_used = (uint16_t)n;
            }
          }
        } else {
          for (uint8_t leg = 0; leg < NUM_LEGS; ++leg) {
            char row[192];
            int16_t cmd0 = g_last_sent_cd[leg][0];
            int16_t cmd1 = g_last_sent_cd[leg][1];
            int16_t cmd2 = g_last_sent_cd[leg][2];
            int16_t meas0 = g_meas_pos_cd[leg][0];
            int16_t meas1 = g_meas_pos_cd[leg][1];
            int16_t meas2 = g_meas_pos_cd[leg][2];
            uint16_t vin0_mV = g_meas_vin_mV[leg][0];
            uint16_t vin1_mV = g_meas_vin_mV[leg][1];
            uint16_t vin2_mV = g_meas_vin_mV[leg][2];
            uint8_t temp0C = g_meas_temp_C[leg][0];
            uint8_t temp1C = g_meas_temp_C[leg][1];
            uint8_t temp2C = g_meas_temp_C[leg][2];
            uint8_t oos0 = servoIsOOS(leg, 0) ? 1 : 0;
            uint8_t oos1 = servoIsOOS(leg, 1) ? 1 : 0;
            uint8_t oos2 = servoIsOOS(leg, 2) ? 1 : 0;
            int vin0_whole = vin0_mV / 1000; int vin0_tenth = (vin0_mV % 1000) / 100;
            int vin1_whole = vin1_mV / 1000; int vin1_tenth = (vin1_mV % 1000) / 100;
            int vin2_whole = vin2_mV / 1000; int vin2_tenth = (vin2_mV % 1000) / 100;
            int n = snprintf(row, sizeof(row), "%lu,%u,%d,%d,%d.%d,%u,%u,%d,%d,%d.%d,%u,%u,%d,%d,%d.%d,%u,%u,%u\r\n",
                             (unsigned long)t_ms, (unsigned int)leg,
                             (int)cmd0, (int)meas0, vin0_whole, vin0_tenth, (unsigned int)temp0C, (unsigned int)oos0,
                             (int)cmd1, (int)meas1, vin1_whole, vin1_tenth, (unsigned int)temp1C, (unsigned int)oos1,
                             (int)cmd2, (int)meas2, vin2_whole, vin2_tenth, (unsigned int)temp2C, (unsigned int)oos2,
                             (unsigned int)err);
            if (n > 0 && (g_log_buf_used + (uint16_t)n) < sizeof(g_log_buf)) {
              memcpy(g_log_buf + g_log_buf_used, row, (size_t)n);
              g_log_buf_used += (uint16_t)n;
            }
            else {
              g_log_file.write(g_log_buf, g_log_buf_used); g_log_file_bytes += g_log_buf_used; g_log_total_bytes += g_log_buf_used; g_log_buf_used = 0;
              if (n > 0 && (uint16_t)n < sizeof(g_log_buf)) {
                memcpy(g_log_buf, row, (size_t)n);
                g_log_buf_used = (uint16_t)n;
              }
            }
          }
        }

        // Flush/rotate inside file-opened scope
        if (g_log_buf_used > (sizeof(g_log_buf) * 3 / 4)) {
          g_log_file.write(g_log_buf, g_log_buf_used);
          g_log_file_bytes += g_log_buf_used;
          g_log_total_bytes += g_log_buf_used;
          g_log_buf_used = 0;
        }
        if (g_log_rotate) {
          const uint32_t HARD_CLAMP = (1024UL * 1024UL * 1024UL);
          if (g_log_max_bytes > HARD_CLAMP) g_log_max_bytes = HARD_CLAMP;
          if ((g_log_file_bytes + g_log_buf_used) >= g_log_max_bytes) {
            if (g_log_buf_used) {
              g_log_file.write(g_log_buf, g_log_buf_used);
              g_log_file_bytes += g_log_buf_used;
              g_log_total_bytes += g_log_buf_used;
              g_log_buf_used = 0;
            }
            g_log_file.flush();
            g_log_file.close();
            ++g_log_seq;
            char fname2[64];
            uint32_t ms2 = millis();
            snprintf(fname2, sizeof(fname2), "/logs/%lu_seq%lu.csv", (unsigned long)ms2, (unsigned long)g_log_seq);
            g_log_file = SD.open(fname2, FILE_WRITE);
            g_log_file_bytes = 0;
            if (g_log_file && g_log_header) {
              g_log_file.print(F("# MARS — Modular Autonomous Robotic System\r\n"));
              g_log_file.print(F("# FW ")); g_log_file.print(FW_VERSION);
              g_log_file.print(F(" build=")); g_log_file.print(__DATE__); g_log_file.print(F(" ")); g_log_file.print(__TIME__);
              g_log_file.print(F("\r\n# loop_hz=")); g_log_file.print((unsigned int)g_loop_hz);
              g_log_file.print(F(" log_rate_hz=")); g_log_file.print((unsigned int)g_log_rate_hz);
              g_log_file.print(F(" mode=")); g_log_file.print((unsigned int)g_log_mode);
              g_log_file.print(F(" div=")); g_log_file.print((unsigned int)g_log_sample_div);
              g_log_file.print(F(" rotate=")); g_log_file.print(g_log_rotate ? 1 : 0);
              g_log_file.print(F(" max_bytes=")); g_log_file.print((unsigned long)g_log_max_bytes);
              g_log_file.print(F(" seq=")); g_log_file.print((unsigned long)g_log_seq);
              g_log_file.print(F("\r\n# columns: time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err\r\n"));
              g_log_file.print(F("time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err\r\n"));
              g_log_file_bytes = g_log_file.size();
            }
          }
        }
        // End of logging operations for this sampled tick: capture duration
#if MARS_TIMING_PROBES
        g_probe_log_us = (uint16_t)(micros() - log_start_us);
#endif
      }
    } else {
#if MARS_TIMING_PROBES
      g_probe_log_us = 0;
#endif
    }
  }
  else {
#if MARS_TIMING_PROBES
    // When logging is disabled entirely, ensure probe reads 0 each tick.
    g_probe_log_us = 0;
#endif
  }
#endif
}
// moved helpers to functions.ino

// --- Configuration loading (SD /config.txt) ---
// Config helpers moved to functions.ino

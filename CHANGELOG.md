# Changelog

## 2025-11-03
- TEST mode UX: Added `TEST OVERLAP <pct>` (0..25) to adjust the overlap window at runtime. The value is persisted to `/config.txt` as `test.trigait.overlap_pct`.
- STATUS: Now also prints `overlap_pct=<..>` in the test parameters summary.
- Version: Bumped FW_VERSION to 0.1.58.

## 2025-11-03
- TEST mode: Overlap is now interpreted as percent of total cycle; implemented a single window per phase end and removed Z jumps by keeping continuous progression (Y lift disabled in overlap). Config key: `test.trigait.overlap_pct`. Default 5%.
- Version: Bumped FW_VERSION to 0.1.57.
- STATUS: Uptime is now wrap-safe using a 64-bit accumulator (handles millis() rollover) and printed as `uptime=<D>d <H>h <M>m <S>s <ms>ms`.
- TEST mode UX: Added `TEST LIFT <mm>` (step height); STATUS shows `lift_y`.
- Version: Bumped FW_VERSION to 0.1.55.

## 2025-11-02
- TEST mode UX: Added serial commands to tune tripod test gait at runtime:
	- `TEST CYCLE <ms>` sets per-tripod phase duration (bounded 750..10000 ms)
	- `TEST HEIGHT <mm>` sets ground height (Y, negative is down)
	- `TEST BASEX <mm>` sets lateral offset (X)
	- `TEST STEPLEN <mm>` sets forward/back amplitude (|Z|)
	- STATUS now prints `test=cycle_ms=<..> base_x=<..> base_y=<..> step_z=<..>`
- Version: Bumped FW_VERSION to 0.1.52.
- UX: Added single-letter command `I` as a shortcut for `MODE IDLE`.
- Version: Bumped FW_VERSION to 0.1.50.
 - UX: Added single-letter command `T` as a shortcut for `MODE TEST` (no version bump).
 - STATUS: Added `tprobe_us=serial/send/fb/tick` line when `MARS_TIMING_PROBES=1` to expose per-tick timing (microseconds).
 - Version: Bumped FW_VERSION to 0.1.51.

## 2025-11-01
- Command behavior: `STAND` now computes a neutral stance via IK at `(x=BASE_X, y=BASE_Y, z=0)` for each leg, aligning with TEST gait base parameters. Previous behavior (copying `home_cd`) is replaced to ensure geometry-consistent standing poses.
- Version: Bumped FW_VERSION to 0.1.49.

## 2025-11-01
- Commands: Added `HOME` (move enabled, in-service servos to their configured home positions) and `SAVEHOME` (read positions of enabled, in-service servos and persist as `home_cd.<LEG>.<joint>=<cd>` in `/config.txt`; updates keys in place or appends if missing). Runtime `g_home_cd` is updated too.
- Fix: Corrected a missing/misplaced brace in `loopTick()` that caused a cascade of compile errors and unintentionally gated common per-tick logic under `MODE TEST`.
- Behavior: Common send/feedback/safety/overrun logic now executes every tick as intended (independent of mode).
- Reliability: Reduced false OOS trips — treat vin OR temp being valid as success and only increment failures when both are invalid; added a brief startup grace window (750 ms) before counting failures.
- IO: Restored proper LX16A bus init: set SerialX baud and provide TX and OE pins to `LX16ABus::begin(...)` so the library can control half-duplex direction; this prevents universal OOS due to failed reads.
- STATUS: Added `home_cd=` grid with per-leg/joint home positions (centidegrees).
- Version: Bumped FW_VERSION to 0.1.48.

## 2025-10-31
- UX: Restored startup splash on USB Serial: ASCII banner, firmware version, loop_hz, UART mapping summary, and config status (sd/keys/cfg_loop_hz). Non-blocking and brief to avoid impacting the first tick.
 - Reliability: Fixed round-robin feedback reads and OOS detection. Now we:
  - Run RR every tick (independent of mode), and only count failures when vin/temp reads are invalid (e.g., return 0).
  - Reset failure counter on successful vin/temp reads; position reads are independent and do not affect OOS.
	- Initialize LX16A buses with proper OE pins for half-duplex direction control.
- Defaults: Joint enable mask now defaults to all 18 joints enabled so leg enables gate motion during bring-up.
 - Config: Added `/config.txt` key `oos.fail_threshold=<1..20>` to set the consecutive vin/temp failure threshold before marking OOS; default 3.
- Version: Bumped FW_VERSION to 0.1.44.

All notable changes to this project will be documented here. Also include a concise change log block at the top of the main `.ino` file for device‑level traceability.

Format: Keep entries short and operationally useful. Use UTC dates.


## 2025-10-28
- Feature: Added tripod test gait mode (diagnostics only, no movement). Implements time-based tripod phase switching, correct 45° rotation for corner legs, COXA_OFFSET, and full IK/foot target debug output.

## [Unreleased]
- Initial scaffolding pending.

## [2025-10-25] Added project docs and AI rules
- Added docs/PROJECT_SPEC.md outlining Phase 1 architecture, safety, and interfaces.
- Added .github/copilot-instructions.md with actionable guidance for AI coding agents.
## 2025-10-27
- Mode: Introduced LOCKOUT mode that is set automatically on safety lockout; STATUS first line appends `SAFETY LOCKOUT` when active.
- Version: Bumped FW_VERSION to 0.1.27.

## 2025-10-27
- Safety UX: Extended `SAFETY` command with `LIST` and `OVERRIDE <ALL|TEMP|NONE>`.
	- Tracks lockout causes and captures a snapshot (per-leg/joint details for temperature trips) at the moment of lockout.
	- Listing shows cause bitmask and specific joints with over-temp values.
	- Overrides can suppress specific causes; if all active causes are overridden, lockout auto-clears (system stays disabled until `ENABLE`).
- Version: Bumped FW_VERSION to 0.1.29.

## 2025-10-27
- STATUS now includes a concise safety summary: `safety=OK|LOCKOUT|OVERRIDDEN` to reflect current state and any active overrides.
- Version: Bumped FW_VERSION to 0.1.30.

## 2025-10-27
- Safety: Overrides now suppress re-lockout by their cause (e.g., TEMP override prevents temperature checks from triggering lockout).
- Version: Bumped FW_VERSION to 0.1.31.

## 2025-10-27
- Config: Added `home_cd.<LEG>.<coxa|femur|tibia>` keys; startup commanded positions now initialize from home values.
- IK: Implemented a basic kinematics routine (coxa yaw + planar femur/tibia) using Robot geometry and wired it into `FOOT` and `FEET`.
	- On unreachable targets, commands return `ERR E20 UNREACHABLE`.
- Version: Bumped FW_VERSION to 0.1.32.
	- 166 Hz control loop shell using elapsedMicros and fixed tick budget comments.
	- ASCII serial parser implementing `HELP` and `STATUS`; stubs for `ENABLE`, `DISABLE`, `STAND`, `FOOT`, `FEET`, `MODE`, and `SAFETY CLEAR`.
	- Fixed-size buffers, no dynamic allocation, and small footprint aligned with repo guidance.

## 2025-10-28
- Reliability: Added `OOS` serial command to report out-of-service status.
	- STATUS also includes an `oos=` line per leg/joint.
- Memory: Compressed OOS flags into a single 18-bit mask to reduce RAM.
- Version: Bumped FW_VERSION to 0.1.36.

## 2025-10-28
- Memory: Reduced RAM and float usage in feedback/state paths.
	- Telemetry now stored compactly: vin in millivolts (uint16), temperature in 0.1°C (int16), position in centidegrees (int16).
	- Timing probes narrowed to uint16 (microseconds), and rate limit stored as centideg/sec (uint16).
	- Updated STATUS formatting and safety lockout checks to use integer-scaled values.
- Version: Bumped FW_VERSION to 0.1.37.

## 2025-10-28
- Memory/UX: Switched leg and joint enable states to compact bitmasks (6-bit legs, 18-bit joints) with helpers; updated gating and STATUS/LEGS/SERVOS output.
- Flash/RAM: Wrapped constant strings in F() to keep them in program memory (splash, STATUS labels, HELP, errors, and command responses).
- Version: Bumped FW_VERSION to 0.1.38.

## 2025-10-27
- Safety: Implemented soft joint limits and rate limiting of commanded joint motions.
	- New SD config keys: `joint_limits.<LEG>.<coxa|femur|tibia>.{min_deg|max_deg}` interpreted around `home_cd` and clamped to 0..24000 cd.
	- New SD config key: `rate_limit.deg_per_s` controls per-tick delta clamp; `<=0` disables the limiter.
	- Enforcement: clamp to limits and apply rate limit before sending each command; no lockout on soft limit clamp.
- Version: Bumped FW_VERSION to 0.1.33.

## 2025-10-27
- Reliability: Automatically mark a servo out-of-service (OOS) after N consecutive feedback read failures (default N=3).
	- OOS servos are ignored: no position commands sent and no further feedback reads attempted until reboot.
	- STATUS now includes an `oos=` line showing per-leg/joint OOS masks.
- Version: Bumped FW_VERSION to 0.1.34.

## 2025-10-25
- Firmware: Added `REBOOT` serial command. Behavior: reply `OK`, flush briefly, then reset MCU. Included in `HELP` output.
- Firmware: Switched reset mechanism to direct AIRCR write on Teensy (`SCB_AIRCR = 0x05FA0004`).

## 2025-10-25
- Firmware: Added 74HC126 (74126) buffer control scaffolding for half-duplex servo buses.
	- User-provided pin order: RF, RM, RR(RB), LR(LB), LM, LF mapped to canonical [LF, LM, LR, RF, RM, RR].
	- Pins default to RX (driver high-Z); enable TX only during sends per leg.

## 2025-10-25
- Firmware: Resolved signed/unsigned comparison warning in serial input buffer check to keep clean builds on Teensy.

## 2025-10-25
- Firmware: Added firmware version string (FW_VERSION) and surfaced in startup splash and STATUS response.

## 2025-10-25
- Firmware: Switched control loop to ISR-triggered tick using IntervalTimer (flag-only ISR; main loop runs body).
- Firmware: Updated round-robin index increment to bounded increment without modulo.
- Firmware: Split non-core helpers (splash, serial parsing/printing, reboot, etc.) into `firmware/MARS_Hexapod/functions.ino` to keep the main sketch small.
- Docs: Added coding guideline to prefer keeping main .ino minimal and moving helpers to `functions.ino`.

## 2025-10-25
- Firmware: Cleaned up `functions.ino` to rely on shared globals without conflicting `extern` declarations; aligned `MODE` command handling to use `MODE_IDLE`/`MODE_TEST` enums.
- Docs: Updated `docs/PROJECT_SPEC.md` to note ISR-driven tick via IntervalTimer flag and the helper organization guideline (`functions.ino`).
 - Docs: Updated `.github/copilot-instructions.md` to add a guideline requiring a firmware version (FW_VERSION) bump after completing any todo item; ensure splash/STATUS reflect the new version and update changelogs in the same commit.

## 2025-10-25
- Firmware: Wired 6 UART servo buses with default 115200 baud (LF→Serial8, LM→Serial3, LR→Serial5, RF→Serial7, RM→Serial6, RR→Serial2) and added per-leg half‑duplex send wrappers (enable TX, flush, return to RX).
- Firmware: Added `servoBusesInit()`, `busSendBegin()/busSendEnd()`, and `sendLegPositions()` placeholder to integrate lx16a‑servo later while keeping timing structure intact.
- Version: Bumped FW_VERSION to 0.1.1 to reflect this todo completion; surfaced in splash/STATUS.
 - Firmware: Added per‑servo ID mapping array `SERVO_ID[leg][joint]` with defaults {1,2,3} per leg, plus `servoId()/setServoId()` helpers; documented SD config keys `servo_id.<LEG>.<joint>`.

## 2025-10-25
- Firmware: Added per‑tick timing guard using `elapsedMicros`; flags `E60 OVERRUN` and increments an overrun counter if execution exceeds the configured period.
- Firmware: STATUS now reports `overruns=<count>` to aid runtime diagnostics; spec updated accordingly.

## 2025-10-25
- Firmware: Default all legs disabled at startup for safety; use `LEG <LEG> ENABLE` to arm individual legs after `ENABLE`.

## 2025-10-25
- Firmware: Corrected UART mapping to match wiring: LF→Serial8 and LR→Serial5; updated splash/status summary.

## 2025-10-25
- Firmware: Added per-servo (joint) enable gating with new `SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>` command and `SERVOS` status listing.
- Firmware: STATUS now includes per-leg joint mask `jmask=LF:xyz,...` and RAW/RAW3 now respect per-joint enable gating.
- Version: Bumped FW_VERSION to 0.1.4; reflected in splash and STATUS.

## 2025-10-25
- Firmware: Switched STATUS and HELP outputs to CR/LF-delimited multi-line format for readability.
- Docs: Spec updated to reflect multi-line STATUS and list form HELP.
- Version: Bumped FW_VERSION to 0.1.5.

## 2025-10-25
- Firmware: SERVO <...> <ENABLE|DISABLE> now also sends torque load/unload to the addressed servo(s) over the bus (assumes LOAD_OR_UNLOAD_WRITE CMD=0x1F).
- Note: If HTS-35S variant uses a different torque command ID, adjust CMD constant in `lx_loadUnloadWrite`.

## 2025-10-25
- Firmware: Global `DISABLE` now queues torque-off for all servos; processed at the start of the next control tick.
- Firmware: `SERVO ... ENABLE` only torques on when the global state is enabled; when globally disabled, it updates masks but defers torque-on (no auto-torque on `ENABLE`).
- Version: Bumped FW_VERSION to 0.1.6.

## 2025-10-25
- Firmware: Hardened setup for Teensy 4.1 startup — added a brief 50 ms settle before initializing buses and starting the ISR tick; use unsigned int for IntervalTimer period to avoid type mismatch.
- Version: Bumped FW_VERSION to 0.1.7.

## 2025-10-25
- Firmware: Switched control tick from IntervalTimer ISR to Metro (polling) timer. The main loop calls loopTick() when tick elapses.
- Version: Bumped FW_VERSION to 0.1.8.

## 2025-10-25
- Firmware: Increased MOVE_TIME_WRITE duration to a minimum of 30 ms to ensure visible motion when using RAW/RAW3 at 6 ms tick.
- Note: This is a temporary floor for bring-up; later make it configurable via SD.

## 2025-10-25
- Firmware: Added SD configuration loader (guarded by MARS_ENABLE_SD) that parses `/config.txt` on boot.
	- Recognized keys (Phase 1): `loop_hz=<50..500>` and `servo_id.<LEG>.{coxa|femur|tibia}=<1..253>`.
	- Applied before splash; splash now reports `config: sd=<0|1> keys=<N>`.
- Version: Bumped FW_VERSION to 0.1.10.

## 2025-10-25
- Firmware: Echo the original serial command on the result line.
	- OK replies now include the command: `OK <original line>`.
	- ERR replies now include the command: `ERR <code> <msg> <original line>`.
	- LEGS/SERVOS/STATUS also append an OK line with the echoed command for consistency.
- Version: Bumped FW_VERSION to 0.1.11.

## 2025-10-25
- Firmware: Servo routines now route through an lx16a-servo adapter layer.
	- New flag `MARS_USE_LX16A` (default 0) selects library-backed commands.
	- When enabled, replace shim calls with your library API (see comments) and include the proper header.
	- Fallback: raw frame writers retained to keep builds working without the library.
-- Version: Bumped FW_VERSION to 0.1.12.

## 2025-10-26
- Firmware: Hardwired integration with madhephaestus/lx16a-servo — adapter flag removed; library included directly.
	- One LX16ABus per leg initialized via begin(HardwareSerial*, txPin, TXFlagGPIO=OE).
	- Replaced raw MOVE_TIME_WRITE and LOAD_OR_UNLOAD_WRITE with LX16ABus::write calls.
	- Removed manual bus TX gating (busSendBegin/End are no-ops now). Library manages OE.
	- TX pin mapping (canonical): {34,14,21,28,25,8} for {LF,LM,LR,RF,RM,RR} derived from prior mapping.
- Version: Bumped FW_VERSION to 0.1.13.

## 2025-10-26
- Firmware: RAW/RAW3 gating semantics adjusted.
	- Now require global ENABLE and EITHER: (a) the leg is enabled OR (b) the relevant joint(s) are enabled.
	- RAW gates on the specific joint; RAW3 gates on all three joints (unless the leg is enabled).
	- Note: loopTick still sends motion commands only for leg-enabled legs; RAW may return OK without motion until the leg is enabled.
- Dev: VS Code IntelliSense include paths updated to resolve `lx16a-servo` and `Streaming` headers without stubs; removed local placeholder header.
- Version: Bumped FW_VERSION to 0.1.14.

## 2025-10-26
- Firmware: STATUS now reports `legs_active` bitstring where a leg is active if leg-enabled OR any joint on that leg is enabled.
- Version: Bumped FW_VERSION to 0.1.15.

## 2025-10-26
- Refactor: moved configuration helpers (SD parser and key application) from `MARS_Hexapod.ino` to `functions.ino` to keep the main loop file minimal.
- Refactor: introduced `firmware/MARS_Hexapod/robot_config.h` to hold robot attributes (link lengths and coxa offsets) for a clean separation of morphology.
- Version: Bumped FW_VERSION to 0.1.16.

## 2025-10-26
- Safety: default startup servo targets to 12000 centidegrees (~120° mid-range) to avoid sudden jumps when enabling motion.
- Version: Bumped FW_VERSION to 0.1.17.

## 2025-10-26
- Firmware: Round-robin sparse feedback reads implemented for vin/temp per leg/joint using lx16a-servo servo helpers.
	- vin interpreted in millivolts (e.g., 12000 mV) with broader clamp.
	- STATUS now reports per-leg/joint grids: vin_mV=..., temp=...
- Version: Bumped FW_VERSION to 0.1.18.

## 2025-10-26
- Firmware: Added position reads in the round-robin feedback cycle using `LX16AServo::pos_read()`.
	- STATUS now reports `pos_cd=LF:c/f/t,...` grid with centidegrees (0..24000).
- Version: Bumped FW_VERSION to 0.1.19.

## 2025-10-26
- Firmware: Temporarily reduced default loop frequency to 100 Hz to investigate loop overruns.
	- Splash and STATUS reflect current `loop_hz`; SD `loop_hz` config key still overrides.
- Version: Bumped FW_VERSION to 0.1.20.

## 2025-10-26
- Firmware: Replaced Metro timer with a lightweight `LoopTimerClass` based on `micros()`.
	- Fixed a bug in `LoopTimer.hpp` where `_loopdT` was shadowed and never updated.
	- Added `SetFrequency()` to allow runtime loop rate updates after config load.
- Version: Bumped FW_VERSION to 0.1.21.

## 2025-10-26
- Firmware: Overrun mitigation — automatically reduce loop frequency by 1 Hz on each detected overrun (floor 50 Hz) and apply immediately.
- Version: Bumped FW_VERSION to 0.1.22.

## 2025-10-26
- Firmware: Adaptive loop rate — step down by 1 Hz only after 100 consecutive overruns, and step up by 1 Hz after 100 consecutive on-time ticks.
	- Upper ceiling is the configured `loop_hz` (from SD or default), lower floor 50 Hz.
- Version: Bumped FW_VERSION to 0.1.23.

## 2025-10-26
- Performance: Removed per-tick torque processing; torque enable/disable now applied immediately in command handlers (SERVO/DISABLE).
- Version: Bumped FW_VERSION to 0.1.24.

## 2025-10-27
- Firmware: Added compile-time timing probes to diagnose loop timing.
	- Captures microseconds for: serial parsing, bulk send to servos, single feedback read, and total loopTick.
	- Guarded by MARS_TIMING_PROBES (default 0). When enabled, STATUS reports tprobe_us line.
- Firmware: Startup splash now prints Robot::SPLASH_BANNER and a concise system summary per spec.
- Version: Bumped FW_VERSION to 0.1.25.

## 2025-10-27
- Firmware: Switched servo motion command path to direct bus writes for MOVE_TIME.
	- Implemented busMoveTimeWrite(leg,joint,cd,time_ms) using LX16ABus::write(LX16A_SERVO_MOVE_TIME_WRITE,...).
	- loopTick now uses this helper with per-leg and per-joint gating; move_time_ms derived from loop_hz.
- Version: Bumped FW_VERSION to 0.1.26.

## 2025-10-27
- Safety: Added temperature lockout — if any servo temperature >= 80°C, immediately torque-off all servos, set lockout, and require `SAFETY CLEAR` to resume.
- Mode: Introduced LOCKOUT mode that is set automatically on safety lockout; STATUS first line appends `SAFETY LOCKOUT` when active.
- Version: Bumped FW_VERSION to 0.1.27.

## [2025-10-25] Spec updates: geometry and UART mapping
- Recorded link lengths in kinematics section: COXA=41.70 mm, FEMUR=80.00 mm, TIBIA=133.78 mm.
- Added explicit leg-to-Serial UART mapping: LF→Serial7, LM→Serial6, LR→Serial2, RF→Serial5, RM→Serial3, RR→Serial8.

## [2025-10-25] Spec updates: serial protocol and logging schema
- Expanded Serial command interface (ASCII) with `ENABLE/DISABLE/STAND/FOOT/FEET/MODE/STATUS`, replies, and error codes.
- Defined CSV logging schema and modes (`read`/`all`), default rate, and key fields.

## [2025-10-25] Spec updates: safety clear command
- Added `SAFETY CLEAR` command to explicitly clear safety lockout (returns to disabled state; use `ENABLE` to resume motion).
- Added error `E91 NOT_LOCKED` for clearing when not in lockout.

## [2025-10-25] Spec updates: IK interface and units
- Documented `calculateIK(leg, target, angles, homeAngles)` interface, coordinate conventions, and centidegree outputs.
- Noted macro alignment (`COXA_LENGTH/FEMUR_LENGTH/TIBIA_LENGTH`) and aliasing to `_MM` constants.

## [2025-10-25] Spec correction: UART mapping flipped L/R
- Corrected leg-to-Serial mapping to flip left/right: LF→Serial5, LM→Serial3, LR→Serial8, RF→Serial7, RM→Serial6, RR→Serial2.

## [2025-10-25] Naming
- Adopted project codename: MARS — Modular Autonomous Robotic System. Reflected in docs and AI instructions.

## [2025-10-25] Spec updates: startup splash
- Added startup splash/banner requirements: print MARS name, build time, loop_hz, UART mapping, config/logging status; non-blocking at boot.

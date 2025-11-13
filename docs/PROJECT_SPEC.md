# Code commenting requirements

All code must be clearly commented:
- Use section headers (block comments) to mark general sections (e.g., setup, loop, helpers, config, safety, etc.).
- Use inline or end-of-line comments for important, non-intuitive, or tricky code (e.g., math, bitmasks, hardware quirks, safety logic).
- Comments should explain intent, not just restate code.

# Hexapod v2.0 Controller — Project Specification (Phase 1)
Project codename: MARS — Modular Autonomous Robotic System

Last updated: 2025-11-12
Target MCU: Teensy 4.1
Servo type: Hiwonder HTS-35S (serial bus)
Servo library: lx16a-servo
Toolchain: Arduino IDE/CLI 1.8.19 + Teensyduino (develop on Raspberry Pi 5, 8GB)

## 1. Goals and scope

- Provide a reliable low-level controller for a 6‑leg hexapod using HTS‑35S serial bus servos.
- Run a deterministic control loop at 166 Hz to command all servos and read limited feedback.
- Accept foot targets in the body frame (mm), compute joint angles via IK, and drive servos.
- Use SD card for runtime configuration (config.txt) and engineering data logging.
- Offer a simple USB serial command interface for enable/disable, stand still, and foot position commands.
- Enforce safety: soft limits, leg/body collision lockout, and motion enable gating.
- Include a “test” mode tripod gait for bring‑up.

Out of scope (Phase 1): ROS integration, advanced terrain adaptation, high‑level planning.

## 2. Hardware architecture

- Controller: Teensy 4.1
- Actuators: 18x HTS‑35S serial bus servos (3 per leg)
- Bus topology: one independent serial bus per leg (6 buses total), each shared by its 3 servos.
  - UART mapping (leg → SerialX):
    - LF → Serial8
    - LM → Serial3
    - LR → Serial5
    - RF → Serial7
    - RM → Serial6
    - RR → Serial2
    - Note: ensure this mapping matches the firmware's `SERVO_PORTS` order; update either the spec or code if they diverge.
    - Teensy pin numbers for each SerialX are board-dependent; exact pins/wiring remain TBD and should be recorded in config and wiring docs.
- Power & grounding: shared ground between MCU and servo power rails; power details TBD.

Leg/servo naming convention:
- Legs: LF, LM, LR, RF, RM, RR (Left/Right x Front/Middle/Rear)
- Joints per leg: Coxa (hip yaw), Femur (hip pitch), Tibia (knee pitch)

## 3. Software overview

- Language/toolchain: Arduino 1.8.19 + Teensyduino; servo IO via lx16a-servo.
- Main loop: fixed‑rate 166 Hz tick using a Metro timer (polling) in loop; on check() true, the loop consumes one tick. ISR kept unused to minimize contention.
- Per tick:
  1) Compute desired joint angles from current foot targets (IK provided externally).
  2) Send motion commands to ALL 18 servos (position targets via lx16a‑servo).
  3) Read feedback from ONLY ONE servo (round‑robin across all 18 over time).
  4) Update estimator for non‑read servos (simple hold‑last/prediction).
  5) Safety checks (soft limits, collision, enable gate).
  6) Optional logging to SD (rate/configurable).

Phase 2 preview (not in Phase 1): outer‑loop PID per joint; virtual spring‑damper per leg.

## 4. Control loop details

- Frequency: Target 166 Hz (period ≈ 6.024 ms). Default temporarily set to 100 Hz for overrun testing; SD config `loop_hz` may override.
- Scheduling:
  - Command broadcast: all servos each tick.
  - Feedback read: one servo per tick (round‑robin order: LF→…→RR, coxa→femur→tibia within leg; exact order TBD).
- Estimation between reads: simple hold‑last (or linear prediction from last commanded delta); upgrade path to a lightweight filter.
- Timing budget: keep IO non‑blocking where possible; per‑tick guard to avoid overrun.

Implementation notes
- Keep the ISR feather‑light: only set a boolean flag. Do all work in the main loop when consuming a tick.
- Keep `MARS_Hexapod.ino` minimal (setup/loop and core tick). Place non‑core helpers (printing, serial parsing, reboot, utilities) in a sibling `functions.ino` for readability and to minimize flash.
 - Design criteria:
   - All robot characteristics (geometry, offsets, UART mapping summary, TX pins, buffer OE pins, default baud) live in `firmware/MARS_Hexapod/robot_config.h`.
   - All helper functions (printing, serial parsing, reboot, config parsing, hardware init helpers like buffers/buses) live in `firmware/MARS_Hexapod/functions.ino`.

## 5. Kinematics

- Input: desired foot pose in body frame, units mm.
- Output: joint angles (deg) for coxa/femur/tibia per leg.
- IK: routine will be provided and integrated; interface contract:
  - Inputs: leg ID, foot target (x, y, z) in body frame; robot geometry parameters.
  - Outputs: joint angles {coxa_deg, femur_deg, tibia_deg}; error code on unreachable.
- Joint limits: per‑joint min/max degrees loaded from config.txt; violations trigger safety.

Link lengths (body frame, mm):

```
#define COXA_LENGTH_MM   41.70f
#define FEMUR_LENGTH_MM  80.00f
#define TIBIA_LENGTH_MM 133.78f
```

These constants inform the IK function and collision geometry. Store definitive values in code and mirror in config if you wish to override per build.

IK implementation notes (as provided)

- Function signature:
  - `bool calculateIK(int leg, Vector3 target, int* angles, long* homeAngles)`
- Coordinate conventions (body frame):
  - `target.x` lateral (left/right), `target.y` vertical (up/down), `target.z` forward/back.
  - Coxa angle uses `atan2(x, z)`; 0° is forward, positive is to the right (viewed from above).
- Geometry constants (mm): code expects `COXA_LENGTH`, `FEMUR_LENGTH`, `TIBIA_LENGTH` macros.
  - To align with this spec’s `_MM` naming, define aliases in code:
    - `#ifndef COXA_LENGTH\n#define COXA_LENGTH COXA_LENGTH_MM\n#endif` (same for FEMUR/TIBIA)
- Outputs and units:
  - `angles[0..2]` are joint commands in centidegrees (0..24000), order: COXA, FEMUR, TIBIA.
  - `homeAngles[servoIdxBase + j]` provide centidegree offsets for each joint (12000 = 120° etc.).
  - Internal helper values may be in radians/deg; final clamped outputs are centidegrees.
- Reachability check:
  - Compute `L = sqrt(x^2 + z^2)`; `D = sqrt((L - COXA_LENGTH)^2 + y^2)`.
  - Unreachable if `D > FEMUR_LENGTH + TIBIA_LENGTH` or `D < |FEMUR_LENGTH - TIBIA_LENGTH|` → return false.
- Indexing and joint order:
  - `servoIdxBase = leg * LEG_SERVOS` with `LEG_SERVOS = 3` implies angles and homeAngles are linearized per leg as COXA,FEMUR,TIBIA.
- Clamping:
  - Final outputs constrained to `0..24000` centidegrees prior to send.

## 6. Safety and collision

- Motion enable gate: commands to servos are suppressed unless enabled.
- Soft limits: joint angle bounds from config; rate limits on commanded steps. [impl]
- Leg/body collision detection: kinematic check against body footprint/keep‑out; on violation → lockout and require re‑enable. [impl]
  - Keep-out model (Phase 1): simple foot-to-foot minimum clearance in the X/Z plane. If any pair of feet are closer than `safety.clearance_mm`, trigger a lockout (E40 COLLISION). Configurable at runtime via `SAFETY CLEARANCE <mm>` and persisted to `/config.txt`.
- E‑stop concept: serial command to DISABLE; clears motion and optionally torque‑off (as supported).

## 7. Configuration (SD card)

- File: /config.txt on SD (root), loaded at boot; key=value per line.
- Format rules:
  - Ignore blank lines; allow comments starting with #; trim whitespace.
  - Values parsed as int/float/bool/strings; invalid lines logged and ignored.
- Example keys (Phase 1 implemented subset marked [impl]):
  - loop_hz=166 [impl]
  - uart.LF=Serial8 [not-impl]
  - uart.LM=Serial3 [not-impl]
  - uart.LR=Serial5 [not-impl]
  - uart.RF=Serial7 [not-impl]
  - uart.RM=Serial6 [not-impl]
  - uart.RR=Serial2 [not-impl]
  - servo_id.LF.coxa=1 [impl]
  - servo_id.LF.femur=2 [impl]
  - servo_id.LF.tibia=3 [impl]
  - servo_id.<LEG>.{coxa|femur|tibia}=<1..253> for all legs (LM,LR,RF,RM,RR) [impl]
  - joint_limits.LF.coxa.min_deg=-45 [impl]
  - joint_limits.LF.coxa.max_deg=45 [impl]
  - body.dimensions.width_mm=…
  - body.dimensions.length_mm=…
  - safety.soft_limits=true [impl]
  - safety.collision=true [impl]
  - safety.temp_lockout_c=80 [impl]
  - safety.clearance_mm=10 [impl]
  - rate_limit.deg_per_s=360 [impl]
  - oos.fail_threshold=3 [impl] — number of consecutive vin/temp failures before marking a servo out-of-service (range 1..20, default 3). At runtime, a failure is counted only when BOTH vin and temp reads are invalid; any valid vin OR temp resets the counter. A brief startup grace window (~750 ms) ignores failures to avoid false OOS during boot.
  - logging.enabled=true
  - logging.rate_hz=166
  - logging.rotate=true            # enable size-based rotation (new)
  - logging.max_kb=10240           # rotation threshold in KB (min 100, clamp 1048576)
  - test.trigait.cycle_ms=3000 [impl]
  - test.trigait.height_mm=-110 [impl]
  - test.trigait.basex_mm=130 [impl]
  - test.trigait.steplen_mm=40 [impl]
  - test.trigait.lift_mm=40 [impl]
  - test.trigait.overlap_pct=5 [impl]
- Reload: Phase 1 loads once at boot; live reload TBD. If SD is disabled at build time (`MARS_ENABLE_SD=0`) or missing, compiled-in defaults are used.
- Defaults: compiled‑in defaults used when keys are missing.

## 8. Runtime logging (SD)

- Purpose: store engineering data for analysis.
- Format: CSV (human‑readable). One header row + data rows.
- Filename pattern: `logs/YYYYMMDD_hhmmss.csv`.
- Modes (config `logging.mode`):
  - `read` (default) — log only the servo that was read on this tick (compact).
  - `all` — log all servos every tick (large; use only for short traces).
  - `off` — disable logging regardless of `logging.enabled`.
- Default rate: `logging.rate_hz=166` (downsample allowed).
- Rotation: optional size-based rotation when `logging.rotate=true`; threshold set by `logging.max_kb` (KB units, minimum 100KB, hard clamp 1GB). Filenames include `_seq<index>` suffix to indicate sequence.
- Buffering: 8KB staging buffer with opportunistic flush (on buffer full, LOG FLUSH, rotation event).

CSV schema (leg-aggregated)
```
time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err
```
- Rows represent one entire leg (3 servos aggregated).
- Compact mode: one row per sample for the current round‑robin leg.
- Full mode: six rows per sample (LF,LM,LR,RF,RM,RR).
- Persistence: Runtime changes via `LOG RATE|MODE|HEADER|ROTATE|MAXKB` are written back to `/config.txt` (except `logging.enabled`).
- Units: cmd/meas in centidegrees; `vin*_V` rendered as X.Y; `temp*_C` in whole °C; `oos*` is 0|1.

## 9. Serial command interface (USB)

- Transport: USB Serial.
- Encoding: ASCII line protocol. One command per line, fields space‑separated, terminated by `\n` (`\r\n` accepted). Keywords are UPPERCASE; numbers are base‑10. Extra spaces are ignored.
- Replies: `OK` or `ERR <code> <msg>`. Unknown commands and bad args produce `ERR`.
- Safety interlocks: motion commands ignored when disabled; soft limits and collision checks applied prior to sending to servos; on violation return `ERR` and enter lockout where specified.
 - Gating: motion output is subject to global enable, per-leg enables, and per-joint enables; commands like `RAW/RAW3` are rejected when the corresponding leg/joint is disabled.

### Telemetry: FK stream

- Controlled via `FK <LEG|ALL> <ON|OFF>` runtime command.
- When enabled, each tick emits one line for the leg that had feedback read that tick:

```
RR_FK <LEG> x=<bx> y=<by> z=<bz> lx=<lx> ly=<ly> lz=<lz> c=<c_cd> f=<f_cd> t=<t_cd>
```

- Units: positions in mm with one decimal; joint angles are raw centidegrees (0..24000).
- Frames:
  - `x/y/z` are BODY-frame coordinates at the foot.
  - `lx/ly/lz` are coordinates relative to the leg’s hip origin (LEG-frame), expressed in BODY axes (no COXA_OFFSET translation).
 - Torque policy: `DISABLE` torques off all servos at the next control tick; `ENABLE` does not torque-on any servo automatically. `SERVO ... ENABLE` torques on only when global is enabled; otherwise, it updates masks without sending torque-on.

Core commands (Phase 1)
- `HELP` — prints the command list.
- `ENABLE` — enable motion output (clears lockout only after power‑on self‑check).
- `DISABLE` — immediate stop/hold; motion output suppressed until `ENABLE`.
- `STAND` — command a neutral standing pose (heights/widths from config).
- `FOOT <LEG> <x_mm> <y_mm> <z_mm>` — set a single foot target in body frame (mm). LEG ∈ {LF,LM,LR,RF,RM,RR}.
- `FEET <x1> <y1> <z1> … <x6> <y6> <z6>` — set all six foot targets in order LF,LM,LR,RF,RM,RR (body frame, mm).
- `MODE TEST` — enter tripod gait test mode.
- `MODE IDLE` — leave test mode; stand/idle.
- `I` — shortcut for `MODE IDLE`.
- `T` — shortcut for `MODE TEST`.
- `STATUS` — returns a multi-line summary (CR/LF-delimited): first line `STATUS`, then key=value lines: `enabled`, `loop_hz`, `rr_idx`, `last_err`, `overruns`, `legs`, `jmask`, telemetry grids `vin_V`, `temp`, `pos_cd`, and `home_cd`.
 - `STATUS` — returns a multi-line summary (CR/LF-delimited): first line `STATUS`, then key=value lines: `enabled`, `loop_hz`, `rr_idx`, `last_err`, `overruns`, `legs`, `jmask`, telemetry grids `vin_V`, `temp`, `pos_cd`, and `home_cd`. When `MARS_TIMING_PROBES` is enabled, includes `tprobe_us=serial/send/fb/tick` (microseconds).
  - STATUS also includes grouped sections: a compact safety summary `safety=<OK|LOCKOUT|OVERRIDDEN> cause=0x.. override=0x..`, test parameters including `overlap_pct=...`, `safety_clearance_mm=<mm>`, enables, telemetry grids, home grid, timing probes (when enabled), and OOS masks.
- `HOME` — move enabled, in-service servos to their configured home positions (gated by leg/joint enables and OOS).
- `SAVEHOME` — read current positions of enabled, in-service servos and persist to SD `/config.txt` as `home_cd.<LEG>.<coxa|femur|tibia>=<centideg>`. Updates existing keys in place or appends new ones; also updates runtime `g_home_cd`.
 - `SAVEHOME` — adjust and save hardware angle offsets so the present neutral maps to 12000 cd (±30° clamp), then persist residual centidegree homes as `home_cd.<LEG>.<coxa|femur|tibia>=<centideg>`; updates runtime `g_home_cd`.
 - `OFFSET LIST` — report per-servo hardware angle offsets (centidegrees) after scaling.
 - `OFFSET CLEAR <LEG|ALL> <JOINT|ALL>` — clear hardware angle offsets for selected servos and update `home_cd` to preserve logical pose.
- `LEGS` — prints per-leg enable mask in human-friendly form: `LEGS LF=1 LM=0 ...` (CR/LF-terminated).
- `SERVOS` — prints per-leg 3-bit joint enable masks: `SERVOS LF=111 LM=101 ...` (CR/LF-terminated).
- `LEG <LEG|ALL> <ENABLE|DISABLE>` — toggle per-leg motion gating.
- `SERVO <LEG|ALL> <JOINT|ALL> <ENABLE|DISABLE>` — toggle per-joint (servo) motion gating.
- `FK <LEG|ALL> <ON|OFF>` — enable/disable FK body-frame stream for selected leg(s); output occurs once per tick for the leg currently served by the round-robin feedback.
- `SAFETY` — safety inspection and controls:
  - `SAFETY LIST` — print current safety state: `lockout`, `cause`, `override`, `clearance_mm`, `soft_limits`, `collision`, `temp_C`.
  - `SAFETY OVERRIDE <ALL|TEMP|COLLISION|NONE>` — set override mask to suppress specific lockout causes. If all active causes are overridden, lockout auto-clears (system remains disabled until `ENABLE`).
  - `SAFETY SOFTLIMITS <ON|OFF>` — enable/disable soft joint limits; persisted to `/config.txt` as `safety.soft_limits=true|false`.
  - `SAFETY COLLISION <ON|OFF>` — enable/disable collision checks; persisted to `/config.txt` as `safety.collision=true|false`.
  - `SAFETY TEMPLOCK <C>` — set over-temperature lockout threshold in Celsius (30..120); persisted as `safety.temp_lockout_c=<C>`.
  - `SAFETY CLEARANCE <mm>` — set the foot-to-foot keep-out clearance (X/Z plane), 0..500 mm; persisted as `safety.clearance_mm=<mm>`.
  - `SAFETY CLEAR` — attempt to clear safety lockout and return to disabled state (requires no active, un-overridden violations). Use `ENABLE` to re‑arm motion.

Notes
- Position commands are clamped to soft limits. If IK is unreachable, return `ERR E20 UNREACHABLE` and ignore the command.
- While disabled, `FOOT/FEET/STAND` are parsed but not applied; reply `ERR E10 DISABLED`.

Error codes (non‑exhaustive)
- `E01 PARSE` — malformed line or wrong arg count
- `E02 UNKNOWN_CMD` — command not recognized
- `E03 BAD_ARG` — out‑of‑range or invalid argument
- `E10 DISABLED` — command not allowed while disabled
- `E20 UNREACHABLE` — IK failed to reach target
- `E30 SOFT_LIMIT` — violates joint soft limits after clamp
- `E40 COLLISION` — violates collision/keep‑out model (lockout engaged)
- `E50 BUS_IO` — serial bus IO error
- `E60 OVERRUN` — per‑tick execution time exceeded the configured period (timing guard)
- `E90 LOCKOUT` — system in safety lockout; requires `SAFETY CLEAR` (and then `ENABLE` to resume motion)
- `E91 NOT_LOCKED` — `SAFETY CLEAR` requested but system is not in lockout

Examples
```
ENABLE                 -> OK
STAND                  -> OK
FOOT LF 120 40 -60     -> OK
FEET 120 40 -60  120 0 -60  120 -40 -60  120 40 -60  120 0 -60  120 -40 -60 -> OK
MODE TEST              -> OK
STATUS                 -> STATUS mode=TEST enabled=1 loop_hz=166 rr_idx=7 last_err=OK
SAFETY CLEAR           -> OK
DISABLE                -> OK
FOOT RF 150 0 -80      -> ERR E10 DISABLED
SAFETY CLEAR           -> ERR E91 NOT_LOCKED
```

## 10. Test mode (tripod gait)

- Purpose: simple validation of bus wiring, IK, timing, and stability.
- Tripod sets: A={LF, RR, RM}, B={RF, LR, LM} (exact grouping TBD; conventional is A={LF, RR, RM} / B={RF, LR, LM}).
- Parameters (runtime adjustable via serial, defaults persisted in firmware; config keys optional later):
  - TEST CYCLE <ms>     — per-tripod phase time; bounded 750..10000 ms
  - TEST HEIGHT <mm>    — ground height (Y, negative is down)
  - TEST BASEX <mm>     — lateral offset (X)
  - TEST STEPLEN <mm>   — forward/back amplitude (|Z|)
  - TEST LIFT <mm>      — step height (lift amount on swing)
  - TEST OVERLAP <pct>  — overlap window percent of total cycle reserved for both-tripods stance (0..25); persisted to `/config.txt` as `test.trigait.overlap_pct`
  - Config: `test.trigait.{cycle_ms,height_mm,basex_mm,steplen_mm,lift_mm,overlap_pct}` — tripod gait parameters loaded at boot. `overlap_pct` also updated at runtime when changed via TEST command. Defaults: cycle_ms=3000, height_mm=-110, basex_mm=130, steplen_mm=40, lift_mm=40, overlap_pct=5.
  - STATUS prints a compact summary: `uptime=<D>d <H>h <M>m <S>s <ms>ms` (monotonic across millis() rollover) and `test=cycle_ms=... base_x=... base_y=... step_z=... lift_y=... overlap_pct=...`
- Foot trajectory: simple trapezoidal or cycloidal swing; linear stance.
- Body stabilization: none in Phase 1 (keep level); optional later.

## 11. Integration with lx16a‑servo

- Use library to set position targets per servo each tick.
- Feedback: read position, voltage, temperature for exactly one servo/tick. STATUS exposes recent values as grids (`vin_mV`, `temp`, `pos_cd`).
- Error handling: comm errors counted per bus/servo; threshold triggers degrade/lockout.

## 12. Timing and scheduling

- Tick source: LoopTimerClass (micros()-based polling) with elapsedMicros guard; jitter kept below ~10% of tick.
- Round-robin feedback index advances each tick; wraps after 18 ticks.
- Overrun handling: step-down after hysteresis — reduce `loop_hz` by 1 Hz only after 100 consecutive overruns (min 50 Hz).
- Recovery: step-up after stability — increase `loop_hz` by 1 Hz after 100 consecutive on-time ticks, up to the configured `loop_hz` target.
- Watchdog: optional tick overrun counter, logging when tick exceeds budget.

## 16. Startup splash/banner

- Purpose: human-friendly confirmation at boot without impacting real-time behavior.
- Transport: USB Serial; print once during setup before entering the control loop.
- Content (example; single or few short lines):
  - `MARS — Modular Autonomous Robotic System | Hexapod v2.0 Controller`
  - `Build: __DATE__ __TIME__ | loop_hz=166 | uart: LF=Serial8 LM=Serial3 LR=Serial5 RF=Serial7 RM=Serial6 RR=Serial2`
  - `config: OK | logging: mode=read rate=166`
- Constraints: keep output brief and non-blocking; do not delay the first control tick beyond budget.
- Optional: re-print a one-line summary on `SAFETY CLEAR` or `ENABLE` if helpful for logs.

## 13. Phase 2 preview (non‑blocking to Phase 1)

- Joint PID: outer‑loop PID around measured/estimated joint angles; map to adjusted position targets.
- Virtual spring‑damper: Cartesian/joint impedance per leg for compliant behavior.

## 14. Acceptance criteria (Phase 1)

- Build: compiles under Arduino 1.8.19 + Teensyduino on RPi 5.
- Timing: verified 166 Hz loop with <10% jitter; no sustained overruns during idle/tri‑gait.
- IO: commands sent to all servos each tick; feedback successfully read round‑robin.
- IK path: foot targets accepted over serial; joints commanded; soft limits enforced.
- Safety: DISABLE halts motion; soft limits and collision lockout tested with synthetic cases.
- Logging: CSV generated on SD with headers and valid samples at configured rate.
- Config: /config.txt parsed with defaults when missing; keys reflected in runtime behavior.
- Boot splash: prints MARS banner and runtime summary within 1s of boot without impacting the 166 Hz loop startup.

## 15. Open TBDs to resolve

- Exact pin wiring for each SerialX per leg; signal levels and bus wiring notes.
- Finalize command list details and full error code table.
- Geometry parameters (offsets, footprints) and joint limits.
- Collision model details and clearance margins.
- Logging rotation and buffering policy specifics.
- Config key list freeze and defaults.
- Where to persist `homeAngles` (SD vs compiled defaults) and calibration workflow.

### Open performance TODOs

- Optimize servo command send path (bus writes) — measured ~1.3 ms per command is too high for 166 Hz budget.
  - Goals: reduce per-command latency to < 300 µs at 115200 baud (or evaluate higher baud), minimize per-call overhead.
  - Ideas: prebuild frames, minimize copies, batch/burst writes per leg, review library overhead, consider DMA/queue where feasible.

---

Feedback welcome: annotate TBDs, propose values (e.g., joint limits, gait params), or suggest additions. Once agreed, we’ll freeze Phase 1 spec and start implementation.

### Servo angle offset calibration (added 2025-11-07)

Workflow to center physical neutral at logical 12000 cd using LX16A hardware offset:
1. Mechanically place each leg in desired neutral stance; enable relevant legs/joints.
2. Issue `SAVEHOME`. For each enabled, in-service servo:
  - Read current position and existing offset.
  - Compute required total offset so (raw_physical_angle − total_offset) = 12000 cd.
  - Clamp to ±30° (±3000 cd ≈ ±125 ticks) and apply via `angle_offset_adjust` + `angle_offset_save`.
  - Re-read position; store residual as `home_cd.<LEG>.<joint>`.
3. STATUS then shows both `home_cd` and `offset_cd` grids. As of FW 0.1.98 the calibration routine also persists `offset_cd.<LEG>.<joint>=<centideg>` alongside `home_cd.<LEG>.<joint>` in `/config.txt` for post-reboot inspection. Startup reads hardware offsets and also accepts `offset_cd.<LEG>.<joint>` keys; parsed values are seeded then overwritten by hardware reads.

Clearing offsets: `OFFSET CLEAR <LEG|ALL> <JOINT|ALL>` restores offset(s) to zero while updating `home_cd` so logical pose stays stable.

Rationale: hardware offsets reduce need for per-servo home tuning and keep IK/FK consistent around a standard center (12000 cd) while residual homes capture sub-degree differences.

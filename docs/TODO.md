# Project TODOs (persistent)

Last updated: 2025-11-14 (dt-aware PID FW 0.2.11)
Owner: Hexapod v2.0 firmware (MARS)

Conventions
- [ ] not-started | [~] in-progress | [x] completed
- Keep items short and actionable; mirror high-level plan in docs/PROJECT_SPEC.md when relevant.
- Copilot will update this file when TODOs change, and reference it in commits.

## Open tasks

- [ ] Optimize servo send path — moved to Phase 2 backlog
  - Reduce per-command latency (<300 µs at 115200). Investigate prebuilt frames, batching per leg, minimize copies, evaluate higher baud or DMA/queue. [Deferred]
- [ ] Stabilize loop at 166 Hz
  - Tune and verify adaptive hysteresis to maintain 166 Hz under typical load; measure with timing probes; ensure no sustained overruns.
 
 - [x] CSV logging to SD (2025-11-09 → 2025-11-10)
  - Phase 1 + FULL mode implemented; size rotation; tail/clear; leg-aggregated rows; per-servo OOS; settings persistence (RATE/MODE/HEADER/ROTATE/MAXKB) excluding enabled. Remaining future (deferred): Bresenham sampling, column mask/filter, compression/binary format.
 - [x] Expand config keys (2025-11-12)
  - Added logging.rotate, logging.max_kb, test.trigait.{cycle_ms,height_mm,basex_mm,steplen_mm,lift_mm,overlap_pct}. Joint limits parser already supports all legs.
 - [x] Tripod test mode (2025-11-09)
  - Implemented deterministic tripod gait with runtime tuning (CYCLE/HEIGHT/BASEX/STEPLEN/LIFT/OVERLAP) and persistence of overlap pct; STATUS reflects parameters.
 
 - [x] Configurable move_time (2025-11-09)
  - Implemented via dynamic derivation from loop rate each tick; future persistence optional.
- [ ] STATUS readability
  - [x] Reformat STATUS output for better readability (grouped sections, aligned grids, compact summary lines). (2025-11-12 — section headers like HELP; one key per line)

  - [~] Complete servo home offset update
    - [x] Wire hardware angle offset I/O via lx16a-servo helpers (remove RAM stubs; Teensy uses real device I/O; host uses fake include).
    - [x] Auto-refresh offsets at startup (populate g_offset_cd from hardware for STATUS before any commands).
    - [x] SAVEHOME cd-only clear→read→compute flow; persist both home_cd.* and offset_cd.* (0.1.98).
    - [ ] Reboot validation: ensure startup loads offset_cd (future key parser update) or recomputes g_offset_cd from hardware so STATUS matches after power cycle.

  - Re-implemented multi-line grouped STATUS (system/config, test, safety, enables, offsets, oos, fk mask). (2025-11-08)
  - [x] Restore HELP formatting (2025-11-08) — multi-line categorized sections; single authoritative implementation in printHELP().

## Phase 1 completion tasks (active)
 - [x] Phase1: expand config keys (joint_limits all legs; test.trigait.* persistence)
 - [x] Phase1: loop timing validation (<10% jitter idle + test gait) (2025-11-12)
 - [x] Phase1: UART mapping consistency (spec vs robot_config.h) (2025-11-12)
 - [x] Phase1: TUCK stage debug instrumentation (FW 0.1.113)

## Phase 2 backlog (deferred)
- [x] Phase2: optimize send path (<300us)
  - Implemented Option A: fast per-leg batched send (MARS_FAST_SEND, default OFF). Batches 3 frames per leg with single OE window and waits only on the RR feedback leg. STATUS shows send_us ≈ 1.7 ms in TEST mode; objective met. Further work: feedback path dominates (fb_us ≈ 4.9 ms) — consider staggering vin/temp reads.
 - [x] Phase2: stagger feedback reads
  - Added MARS_FB_STAGGER (default ON). Always read position; alternate vin and temp across RR visits; maintain OOS semantics using stored values. Expect fb_us reduction by ~2–3×.
 - [x] Phase2: joint PID controller (FW 0.2.11)
  - Integrated dt-aware PID compute into loop (between estimator and send) with P/PI/PD, anti-windup clamp, and filtered derivative. Default disabled with zero gains. Config keys `pid.enabled` and `pid.{kp,ki,kd}_milli.<coxa|femur|tibia>` parsed at boot; STATUS [PID] shows enabled flag, mode, gains, kd_alpha, and shadow report rate. Shadow mode (`pid.mode=shadow`) computes PID without driving servos and streams enriched `PID_SHADOW` lines (diff_cd, err_cd, est_cd, tgt_cd) at `pid.shadow_report_hz`.
 - [ ] Phase2: virtual impedance (spring-damper)
   - Cartesian/joint spring-damper per leg; config keys `impedance.{kx,ky,kz,cx,cy,cz}`; maps desired foot pose deltas to joint target adjustments; deterministic update in tick.
 - [~] Phase2: estimator upgrade
   - Implemented simple exponential smoothing toward last command with measurement correction on RR updates; PID now uses estimate between sparse reads. Next: validate timing impact and tune alphas.
 - [ ] Phase2: collision model refinement
   - Extend from foot clearance to include body footprint, vertical constraints, and joint self-interference checks; add config toggles for refined modes.
 - [ ] Phase2: FK/IK test harness
   - Add `IKTEST` / `FKTEST` serial commands for quick validation; emit diff vs expected; optional stats accumulation.
 - [ ] Phase2: config live reload
   - Implement `CONFIG RELOAD` (safe mid-run parse) with timing guard; only apply non-critical keys (limits, gait, safety, logging); reject loop_hz if would cause instability.
 - [ ] Phase2: pin wiring documentation
   - Document SerialX TX/RX/enable pins & power wiring; mirror in `robot_config.h`; splash sanity print of pin set completeness.
 - [ ] Phase2: logging enhancements
   - Add `logging.mode=profile` (per-tick timing, rr index, overruns); jitter histogram snapshot command; optional servo read distribution metrics.
 - [ ] Phase2: homeAngles calibration workflow
   - Add `CALIBRATE <LEG|ALL>` automation; unify offset/home persistence strategy; clearly separate hardware offset vs logical home.
 - [ ] Phase2: loop_hz strategy review
   - Re-assess adaptive up/down hysteresis; consider fixed 166 Hz with overrun counter & downgrade warning only.
 - [ ] Phase2: memory/footprint audit
   - Generate RAM/flash map; identify large symbols (float printf, lx16a unused); plan trims before adding PID/impedance.

## Workflow policies (Phase 2)

- [~] Defer commits until instructed
  - Keep changes in the working tree; do not commit or push without an explicit request. Provide git status/diff on demand.
- [~] Auto-bump FW_VERSION per edit
  - Increment FW_VERSION patch with every assistant-made code change. Update main .ino header and CHANGELOG entries when a TODO is completed or behavior changes; skip verbose changelog spam for pure profiling/formatting edits.
 - [~] Confirm major/minor version bumps
   - Major/minor version increments require explicit user confirmation. Patch and FW_BUILD monotonic can auto-increment with edits.

## Completed
 - [x] Keep STATUS slim (2025-11-12)
  - Verified STATUS remains compact with grouped sections; no `[TUCK]` debug block or verbose fields present.
 - [x] Startup offset refresh (2025-11-12)
  - On boot, read hardware angle offsets and populate g_offset_cd before first STATUS. Also parse offset_cd.<LEG>.<joint> from config (seed only; hardware read overwrites).
 - [x] HELP TUCK update (2025-11-12)
  - Added TUCK help lines documenting runtime tuck.* params and TUCK SET subcommand (persist TIBIA/FEMUR/COXA/TOL_TIBIA/TOL_OTHER/TIMEOUT). (FW 0.1.111)
 - [x] TUCK PARAMS command (2025-11-12)
  - Added TUCK PARAMS subcommand to print current tuck parameter values (tibia/femur/coxa, tolerances, timeout) and corresponding HELP line. (FW 0.1.112)
 - [x] STATUS [TUCK] debug removal (2025-11-12)
  - Removed temporary STATUS `[TUCK]` debug section (active/masks/params and tibia meas/eff). Keeps STATUS leaner and reduces print cost; TUCK controller remains unchanged. (FW 0.1.110)
 - [x] Position validity & TUCK tibia convergence (2025-11-12)
 - [x] Loop timing validation (2025-11-12)
  - Jitter metrics added (FW 0.1.115) and observed min/avg/max within <10% of 6.024 ms budget under idle + test gait; no sustained overruns.
 - Confirms Phase 1 timing acceptance criteria.
  - Added `g_meas_pos_valid` flags so a measured 0 cd is treated as valid instead of falling back to last-sent (which appeared as 24000). Restored TUCK tibia target to 0 cd; prevents sequencing stall.
 - [x] Logging rotation & size cap (2025-11-10)
  - Implemented size-based rotation with `LOG ROTATE` and `LOG MAXKB` (KB units). 1GB hard clamp; default max 10MB; sequence-based filenames and status reporting.
 - [x] Leg-aggregated CSV rows (2025-11-10)
  - Changed logging schema to emit one line per leg (3 servos aggregated) reducing rows (compact: 1→1 unchanged semantics; full: 18→6). Updated headers, CHANGELOG, FW_VERSION to 0.1.88.
 - [x] Tripod test mode (2025-11-09)
  - Deterministic tripod gait with runtime tuning (CYCLE/HEIGHT/BASEX/STEPLEN/LIFT/OVERLAP) and persistence of overlap pct; STATUS reflects parameters.
 - [x] Move time derived from loop period (2025-11-09)
  - Per-tick move_time computed from current loop_hz; eliminates static constant and ensures sync with adaptive loop rate.
 - [x] Unify SD guard macros (2025-11-09)
   - Standardized feature guards to `#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD` across sources; avoids ambiguous truthiness and keeps guards consistent.
 - [x] Centralize OK responses (2025-11-09)
   - Moved OK printing to a centralized dispatcher epilogue; removed per-handler printOK() calls; handlers only print ERR on failure.
- [x] Echo processed command lines (2025-11-08)
  - After each command is handled, the original line is reflected back as `> <line>` to aid logging/CLI.
- [x] Keep docs in sync with commands (2025-11-04)
  - Updated Serial command list in `docs/PROJECT_SPEC.md` to include SAFETY LIST/OVERRIDE/SOFTLIMITS/COLLISION/TEMPLOCK/CLEARANCE and expanded STATUS grouping; added implemented config keys (safety.* and test.trigait.overlap_pct).
- [x] STATUS readability (2025-11-03)
  - Reformatted STATUS output into grouped sections: system, safety, test, enables, telemetry, home, timing, oos.
- [x] HELP in sync (2025-11-03)
  - Updated HELP to list all implemented commands, including SAFETY SOFTLIMITS/COLLISION/TEMPLOCK; removed duplicates.
- [x] RR FK round-robin telemetry (2025-11-03)
  - Print FK body-frame x/y/z for the leg read during round-robin feedback to aid bring-up.
- [x] Safety config toggles (2025-11-03)
  - Added safety.soft_limits, safety.collision, and safety.temp_lockout_c keys; runtime `SAFETY` subcommands to update/persist; STATUS/SAFETY LIST show current values.
- [x] Collision lockout (2025-11-03)
  - Implemented simple foot-to-foot keep-out in X/Z plane with configurable clearance. Added `SAFETY CLEARANCE <mm>` command and `safety.clearance_mm` config key; STATUS prints `safety_clearance_mm`. Lockout triggers E40 and requires re-enable.
- [x] TEST OVERLAP runtime command (2025-11-03)
  - Added `TEST OVERLAP <pct>` to adjust overlap at runtime (0..25). Value is persisted to `/config.txt` as `test.trigait.overlap_pct`. STATUS includes `overlap_pct`.
- [x] STATUS safety details (2025-11-03)
  - STATUS now includes `safety=<state> cause=0x.. override=0x..`. Added `SAFETY LIST` and `SAFETY OVERRIDE` commands.

  - [x] Defer commits until instructed (2025-11-12)
    - Phase 1 policy enforced (batched edits, explicit user-driven commit points). Closed after merge/tag of Phase 1 completion.

- [x] TEST gait params commands (2025-11-02)
  - Added TEST subcommands to set cycle time, ground height, base X, and step length at runtime; STATUS shows current values.

- [x] Timing probes
  - Compile-time timing probes for serial/send/feedback/tick and STATUS reporting.
- [x] Direct bus MOVE_TIME writes
  - Replace LX16AServo::move_time with custom busMoveTimeWrite helper in loopTick.
- [x] Temperature safety lockout and UX
  - Lockout at >=80°C with torque-off; LOCKOUT mode; SAFETY LIST and OVERRIDE support; STATUS safety summary and override handling.
 - [x] STAND neutral via IK (2025-11-01)
  - STAND computes IK to a neutral stance at (x=BASE_X, y=BASE_Y, z=0) per leg; replaces prior home_cd copy behavior.
- [x] Startup splash banner
  - Show Robot::SPLASH_BANNER and concise runtime summary at boot.
- [x] Integrate IK function
  - Implemented calculateIK and wired FOOT/FEET; unreachable returns E20.
- [x] Soft joint limits
  - Parsed joint_limits.* from /config.txt; clamp commanded centidegrees prior to send.
- [x] Rate limit angle deltas
  - Per-joint delta clamp per tick derived from rate_limit.deg_per_s; configurable via SD.

- [x] Width downsizing of globals
  - Store telemetry as integers: vin in mV (u16), temp in 0.1°C (i16), pos in centideg (i16); timing probes as u16; rate limit in cdeg/s.

- [x] Bitmask leg/joint enable flags
  - Replace arrays with compact 6-bit and 18-bit masks; update gating, LEGS/SERVOS, and STATUS rendering.

- [x] Move constant strings to flash
  - Wrap constant prints with F() across splash, STATUS labels, HELP, and command responses to keep strings in PROGMEM.

- [x] RR/OOS feedback robustness
  - Round-robin feedback runs every tick; position reads are optional and do not drive OOS.
  - OOS is based on vin/temp: a failure is counted only when BOTH vin and temp are invalid; any valid vin OR temp resets the counter.
  - Added a brief startup grace window (~750 ms) before counting failures to avoid false OOS at boot.
  - Threshold configurable via `/config.txt` key `oos.fail_threshold` (1..20, default 3).
  - LX16A bus init uses proper OE pins; joint mask defaults to all-enabled to simplify bring-up gating.


# Master TODO List (organized by area)

## Memory & Code Footprint
- [x] Audit globals for width downsizing
  - Identify arrays/vars that can be int16_t/uint16_t/uint8_t (e.g., cmd/home/est/limits in centideg, temperatures, voltages, failure counters).
- [x] Bitmask leg/joint enable flags
  - Replace g_leg_enabled[6] and g_joint_enabled[6][3] with 6- and 18-bit masks (mirroring OOS) to save RAM and simplify code paths.
- [x] Move all constant strings to flash
  - Use F() and PROGMEM for Serial prints (splash, STATUS, HELP, errors).
- [x] Eliminate double usage
  - Ensure all math/IK and printing uses float (sinf/cosf/atan2f, suffix f, -fsingle-precision-constant).
- [ ] Slim feedback/state types
  - Compact types/scaling: temp uint8_t C, voltage uint16_t mV/cV, positions int16_t cd, timestamps uint32_t. List fields to change.
- [ ] Library surface minimization
  - Check lx16a-servo usage; mark unused functions static or isolate a tiny send/read shim to let the linker drop dead code.

## Safety & Robustness
- [ ] Stack usage review
  - Identify large locals in hot paths (loopTick, IK, STATUS); make small/static or reduce scope; propose a tiny stack watermark utility (later).

## Performance & Output
- [ ] STATUS print cost reduction
  - Fewer/lighter STATUS fields, integer rendering (avoid float prints), single snprintf into static buffer; estimate flash/RAM impact.

## Config & Logging
- [ ] Config parser buffers review
  - Review line/token buffers, propose minimal safe sizes (e.g., 128B line, 32B key, 64B value), static reuse, avoid hidden heap use.
- [ ] SdFat/logging footprint trim
  - Lazy init only when logging enabled; SdFat config to disable LFN/reduce buffers; small ring buffer with fixed-size records.

## Build, Linker, & Optimization
- [ ] Linker map + section GC check
  - Enable/inspect map output, confirm -ffunction/data-sections and --gc-sections, consider LTO; identify big symbols (float printf, trig).

## Optional Features

- [ ] Centralize OK responses (consolidate duplicate entry)
  - (Planned) After confirming link success under FW 0.1.72, evaluate moving printOK() emission to a single dispatcher epilogue unless handler already printed error or multi-line output; update handlers accordingly.

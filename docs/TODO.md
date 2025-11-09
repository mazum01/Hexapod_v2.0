# Project TODOs (persistent)

Last updated: 2025-11-08 (post 0.1.72 helper restore)
Owner: Hexapod v2.0 firmware (MARS)

Conventions
- [ ] not-started | [~] in-progress | [x] completed
- Keep items short and actionable; mirror high-level plan in docs/PROJECT_SPEC.md when relevant.
- Copilot will update this file when TODOs change, and reference it in commits.

## Open tasks

- [ ] Optimize servo send path
  - Reduce per-command latency (<300 µs at 115200). Investigate prebuilt frames, batching per leg, minimize copies, evaluate higher baud or DMA/queue.
- [ ] Stabilize loop at 166 Hz
  - Tune and verify adaptive hysteresis to maintain 166 Hz under typical load; measure with timing probes; ensure no sustained overruns.
 
- [ ] CSV logging to SD
  - Implement buffered CSV logging with modes (read/all/off), rate_hz; file naming logs/YYYYMMDD_hhmmss.csv; minimal schema.
- [ ] Expand config keys
  - Add logging.* (enabled, rate_hz, mode), additional test.trigait.* and joint_limits.* parsing.
- [ ] Tripod test mode
  - Implement simple tripod gait (tri-gait) with configurable step height/length, cycle time, duty factor.
  - [x] Expose runtime tuning via serial: TEST CYCLE/HEIGHT/BASEX/STEPLEN; reflect in STATUS. (2025-11-02)
 
- [ ] Configurable move_time
  - Expose move_time_ms via /config.txt and echo in STATUS; default derived from loop rate.
  - Add compact indicator of which leg/joint was read this tick (e.g., rr_meas=LF/TIBIA).
- [ ] STATUS readability
  - Reformat STATUS output for better readability (grouped sections, aligned grids, compact summary lines).

  - Re-implemented multi-line grouped STATUS (system/config, test, safety, enables, offsets, oos, fk mask). (2025-11-08)
  - [x] Restore HELP formatting (2025-11-08) — multi-line categorized sections; single authoritative implementation in printHELP().

## Completed
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

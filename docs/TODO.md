# Project TODOs (persistent)

Last updated: 2025-11-03
Owner: Hexapod v2.0 firmware (MARS)

Conventions
- [ ] not-started | [~] in-progress | [x] completed
- Keep items short and actionable; mirror high-level plan in docs/PROJECT_SPEC.md when relevant.
- Copilot will update this file when TODOs change, and reference it in commits.

## Open tasks

- [ ] Keep HELP in sync with commands
  - Whenever commands are added/removed/changed, update `printHELP()` in `firmware/MARS_Hexapod/functions.ino` and the Serial command list in `docs/PROJECT_SPEC.md`.

- [ ] Optimize servo send path
  - Reduce per-command latency (<300 µs at 115200). Investigate prebuilt frames, batching per leg, minimize copies, evaluate higher baud or DMA/queue.
- [ ] Stabilize loop at 166 Hz
  - Tune and verify adaptive hysteresis to maintain 166 Hz under typical load; measure with timing probes; ensure no sustained overruns.
- [ ] Collision lockout
  - Implement simple keep-out model; detect leg/body collisions and enter safety lockout; config clearance_mm.
- [ ] CSV logging to SD
  - Implement buffered CSV logging with modes (read/all/off), rate_hz; file naming logs/YYYYMMDD_hhmmss.csv; minimal schema.
- [ ] Expand config keys
  - Add safety.* (soft_limits, clearance_mm), logging.* (enabled, rate_hz, mode), test.trigait.* and joint_limits.* parsing.
- [ ] Tripod test mode
  - Implement simple tripod gait (tri-gait) with configurable step height/length, cycle time, duty factor.
  - [x] Expose runtime tuning via serial: TEST CYCLE/HEIGHT/BASEX/STEPLEN; reflect in STATUS. (2025-11-02)
- [ ] STATUS safety details (optional)
  - Include cause/override bitmasks (hex) in STATUS for quick triage.
- [ ] Configurable move_time
  - Expose move_time_ms via /config.txt and echo in STATUS; default derived from loop rate.
- [ ] STATUS rr_read identity (optional)
  - Add compact indicator of which leg/joint was read this tick (e.g., rr_meas=LF/TIBIA).
- [ ] STATUS readability
  - Reformat STATUS output for better readability (grouped sections, aligned grids, compact summary lines).

## Completed
- [x] TEST OVERLAP runtime command (2025-11-03)
  - Added `TEST OVERLAP <pct>` to adjust overlap at runtime (0..25). Value is persisted to `/config.txt` as `test.trigait.overlap_pct`. STATUS includes `overlap_pct`.

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
- [ ] Optional feature flags
  - Plan compile-time flags to strip TEST gait, verbose IK debug, timing probes, extended STATUS for a “slim build” for troubleshooting freezes.
  - Plan compile-time flags to strip TEST gait, verbose IK debug, timing probes, and extended STATUS to create a “slim build” for troubleshooting freezes.

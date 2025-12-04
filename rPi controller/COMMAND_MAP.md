# Python ↔ Firmware Command Alignment Audit

Date: 2025-11-22
Scope: rPi controller `controller.py` usage vs Teensy firmware `commandprocessor.ino` parser.

## Firmware Supported Top-Level Commands
From `parseCommandType` enumeration:
`HELP, STATUS, REBOOT, ENABLE, DISABLE, FK, LEGS, SERVOS, LEG, SERVO, RAW, RAW3, FOOT, FEET, MODE, TEST, STAND, SAFETY, HOME, SAVEHOME, OFFSET, LOG, LOOP, TUCK, PID, IMP, COMP, EST, LIMITS, CONFIG`
Single-letter shortcuts: `I (IDLE), T (TEST MODE), Y (Telemetry toggle)`.

## Current Python Usage (Observed in `controller.py`)
- Startup / Posture: `LEG ALL ENABLE`, `ENABLE`, `DISABLE`, `TUCK`, `STAND`, `T` (test mode), `b1`, `b0` (custom steering – not a firmware command, likely legacy/gait internal)
- Gamepad toggles: repeat of above enable/disable + steering `b1/b0`
- Gait/Test: `T` (sets TEST), no direct `MODE TEST` / `MODE IDLE` invocation (instead using shortcut `T`).
- Missing but beneficial introspection: `STATUS`, `LEGS`, `SERVOS`, `PID LIST`, `IMP LIST`, `COMP LIST`, `LIMITS LIST`, `LOG`, `CONFIG DUMP`, `FK` stream toggling, `EST`, `SAFETY`.
- Not used: `RAW`, `RAW3`, `FOOT`, `FEET` (critical for future gait engine), `LEG <LEG> DISABLE`, `SERVO <...>` granular torque control, `LOOP <hz>` runtime frequency tuning, `HOME`, `SAVEHOME`, `OFFSET` calibration, impedance (`IMP`), compliance (`COMP`), estimator (`EST`).

## Redundant / Legacy / Non-Firmware
- `b1`, `b0` steering commands: not recognized by firmware parser; if still handled by Teensy (perhaps legacy branch), formalize into a new command (`STEER ACKERMAN|OMNI`) or migrate to existing `MODE` semantics.
- Use of repeated `LEG ALL ENABLE` before every posture even when legs already enabled.

## Proposed High-Value Additions
1. Introspection Poll Cycle:
   - On 1 Hz interval: send `STATUS`, `LEGS`, `SERVOS`, `LIMITS LIST`, `PID LIST`, `IMP LIST`, `COMP LIST` (staggered) to populate richer UI overlays.
2. Gait Engine Interface (future task):
   - Adopt `FEET` for bulk Cartesian targets when generating trajectories; fallback to `FOOT` for per-leg manual tweaks.
3. Safety & Telemetry Control:
   - Expose `Y 1|0` toggle via a UI menu; add `SAFETY` subcommands (if implemented in firmware) once spec is known.
4. Configuration / Persistence:
   - Integrate `LOOP <hz>` for runtime loop tuning (with guard rails); add `CONFIG DUMP` to a menu item for quick inspection.
5. Calibration Hooks:
   - Provide hidden developer menu entries for `HOME`, `SAVEHOME`, `OFFSET` operations.
6. Control Abstraction:
   - Replace raw strings with `send_cmd('DISABLE')`, `send_cmd('LEG ALL ENABLE')` etc., enabling rate limiting & dedup.

## Suggested Deprecations / Changes
- Replace `T` direct usage with `MODE TEST`; use `MODE IDLE` for clarity (keep single-letter shortcuts optionally via wrapper).
- Remove or rework `b1/b0`; unify steering mode selection under a planned `STEER` or an extension of `MODE`.
- Eliminate posture duplication: unify `TUCK` and `STAND` sequences into `posture('TUCK')` / `posture('STAND')` using shared enable logic.

## Wrapper Strategy
Introduce `send_cmd(cmd: str, *, force=False, throttle_ms=50)`:
- Auto-appends `\n`.
- Maintains `last_sent_time[cmd]` to suppress spam within `throttle_ms` unless `force=True`.
- Tracks enabled state locally: if `g_enabled_local` true, skip redundant `LEG ALL ENABLE` + `ENABLE` pair.
- Optional queue for multi-line sequences (e.g., posture) ensuring ordering.

Example pseudo-code:
```python
_cmd_last = {}
_enabled_local = False

def send_cmd(cmd, force=False, throttle_ms=50):
    now = time.time() * 1000.0
    if not force and cmd in _cmd_last and (now - _cmd_last[cmd]) < throttle_ms:
        return False
    _teensy.write((cmd + "\n").encode('ascii'))
    _cmd_last[cmd] = now
    if cmd == 'ENABLE':
        _enabled_local = True
    elif cmd == 'DISABLE':
        _enabled_local = False
    return True
```

## Sequencing Templates
- Posture: `if not _enabled_local: send_cmd('LEG ALL ENABLE'); send_cmd('ENABLE'); send_cmd('TUCK'); schedule_auto_disable()`
- Mode change: `send_cmd('MODE TEST')` / `send_cmd('MODE IDLE')`
- Gait engine frame: `send_cmd(f"FEET {x1} {y1} {z1} ... {x6} {y6} {z6}")`

## Prioritized Implementation Steps
1. Add `COMMAND_SET` list & `send_cmd` helper (quick win).
2. Replace hard-coded writes; implement local enable tracking.
3. Add polling for `STATUS`, `LEGS`, `SERVOS` to enrich display.
4. Introduce `MODE TEST/IDLE` usage; deprecate direct `T` send in UI.
5. Prepare for gait engine by adding manual `FEET` test command path.
6. Developer calibration menu (HOME/SAVEHOME/OFFSET) gated behind flag.

## Acceptance Criteria (Align Commands Task)
- Mapping documented (this file).
- Clear list of additions & deprecations + wrapper design.
- No code changes required yet (deferred to helper task) — this completes the audit.

## Follow-On TODO References
Links to existing backlog items: send_cmd helper (#6), posture abstraction (#7), telemetry schema validation (#8), command rate throttling (#10), Controller refactor (#11), split display rendering (#28), suppress redundant enable sequences (#24).

---
End of audit.

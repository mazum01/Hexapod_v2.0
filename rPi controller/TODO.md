# Python Controller Master TODO

Single source of truth for rPi controller (Python) tasks.
Keep entries concise; update status inline and mirror in-file header changelog when behavior changes.

Legend: [ ] not-started | [~] in-progress | [x] completed

## Active Backlog

0. [x] Apply serial config settings
   - Goal: Honor port/baud settings from controller.ini for Teensy connection instead of hardcoded values.
   - Deliverable: Controller.connect_teensy() method with config override support.
   - Acceptance: controller.ini port/baud settings applied; connection logic consolidated.
   - Completed: 2025-11-25 (integrated with Controller refactor phase 1e).

1. [x] Review controller code
   - Goal: Structural/code quality assessment (duplication, cohesion, error handling, timing discipline).
   - Deliverable: Markdown section with prioritized recommendations (low/med/high impact) and estimated effort.
   - Acceptance: List delivered; no runtime regressions.
   - Completed: 2025-12-04 (v0.4.8 b73). See "Code Review Recommendations" section below.

2. [ ] Align commands with firmware
   - Goal: Diff Python command usage vs current Teensy HELP output; identify missing abstractions (e.g., STATUS polling, SAFETY toggles).
   - Deliverable: Proposed command map + deprecation list + wrapper strategy.
   - Acceptance: Document approved; ready for incremental implementation tasks.

3. [x] Design gait engine
   - Goal: Generic gait engine producing per-leg foot targets (phase, duty factor, trajectory) using FEET/FOOT and telemetry S1/S2/S3.
   - Deliverable: Minimal architecture doc + prototype module (no GUI) with pluggable gait parameters.
   - Acceptance: Engine can command a stationary test pattern and a tripod gait; timing loop stable (<5% jitter target).
   - Completed: 2025-11-26 (v0.3.0 b33) - gait_engine.py with TripodGait, StationaryPattern; keyboard controls w/h/p.

4. [ ] Touchscreen config menu
   - Goal: Usable LCD touchscreen menu for adjusting Python-level settings (verbosity, mirror, auto-disable timeout, selected gait).
   - Deliverable: Menu screens + navigation + persistence hooks.
   - Acceptance: All listed settings adjustable; feedback immediate; no blocking main loop > 20 ms.

5. [x] Python config persistence & centralize constants
   - Goal: Use `controller.ini` for persistent settings and move magic numbers (timings, thresholds) into config sections.
   - Deliverable: Loader with safe defaults; [timing], [display], [gait] sections with all key constants.
   - Acceptance: Start-up loads config; code references config values instead of literals; corruption handled gracefully.
   - Completed: 2025-12-04 (merged with item 15). Added [timing] (loop_target_ms, teensy_loop_us, telem_sync_fallback_ms, teensy_reconnect_backoff_s), [display] (screen_refresh_ms, brightness, auto_disable_s), [gait] (width/lift ranges).

6. [x] Implement send_cmd helper
   - Goal: Centralize Teensy command emission (newline termination, optional dedup + rate limiting).
   - Deliverable: Helper function/module with simple queue or last-command timestamp.
   - Acceptance: All command writes routed through helper; duplicate rapid-fire commands suppressed.

7. [x] Posture abstraction helper
   - Goal: Unify enable + posture (TUCK/STAND) + auto-disable scheduling.
   - Deliverable: `set_posture(name, auto_disable_ms)` function; internal posture mapping.
   - Acceptance: Existing tuck/stand paths replaced; behavior identical; code duplication reduced.

8. [x] Telemetry schema validation
   - Goal: Assert S1/S2/S3/S4 lengths & indices at startup; warn if mismatch.
   - Deliverable: Validation routine with non-blocking warnings.
   - Acceptance: Mismatch clearly logged; runtime guards prevent index errors.

9. [x] Font caching optimization
   - Goal: Reduce per-loop Pillow/OpenCV rendering cost.
   - Deliverable: Glyph/string cache (e.g., dict of Image objects) with reuse strategy.
   - Acceptance: Measurable loop time reduction (>10% in rendering section) with identical visuals.
   - Note: Implemented font object caching (2025-11-24). Text image caching optional.

10. [x] Command rate throttling / dedup
    - Goal: Prevent spamming identical commands (e.g., ENABLE) within short intervals.
    - Deliverable: Timestamp-based throttle integrated into send_cmd helper.
    - Acceptance: No behavioral regressions; command spam reduced during rapid input.

11. [x] Controller class refactor
    - Goal: Encapsulate globals (state, timers, rendering) for clarity and testability.
    - Deliverable: Lightweight `Controller` class without adding heavy abstraction.
   - Acceptance: File size stable; external interface unchanged; easier unit isolation.
   - Notes:
     - Skeleton `Controller` added with factory `from_current_globals` (2025-11-24).
     - Phase 1b: Mirroring of verbose/mirror/telemetry timers into class with per-loop sync; behavior unchanged (2025-11-24).
     - Phase 1d: Migrated Teensy polling, gamepad event handling, and display update into Controller methods (2025-11-24).
     - Phase 1e: Completed migration of all global state into Controller class with serial config support (2025-11-25).

12. [x] Auto-disable configurability
    - Goal: Make posture auto-disable duration adjustable via config.
    - Deliverable: Config key parsed; default preserved; integrated into abstraction helper.
    - Acceptance: Changing value affects next posture sequence; validated at startup.
    - Completed: 2025-12-04 (v0.3.31 b64). Added `[display] auto_disable_s` to controller.ini; apply_posture() uses config value.

13. [ ] Logging enhancements
    - Goal: Optional structured event log (enable, posture, errors) with timestamps.
    - Deliverable: Simple logger writing CSV or JSON lines; toggle via config.
    - Acceptance: Log captures listed events; disabled mode has zero measurable overhead.

14. [x] Modular loop phases refactor
   - Goal: Split giant loop into distinct phases (poll_teensy, poll_gamepad, update_display, process_keyboard, housekeeping).
   - Deliverable: Refactored main loop preserving behavior; timing comments added.
   - Acceptance: No regression; readability improved; loop jitter unchanged.
   - Completed: 2025-12-04 (v0.4.3 b68). Created phase functions: phase_timing_update, phase_teensy_connection, phase_gamepad_connection, phase_display_update, phase_touch_input, phase_keyboard_input, phase_gait_tick, phase_auto_disable, phase_loop_sleep, sync_globals_to_ctrl, sync_ctrl_to_globals.

15. [x] Display/eye rendering thread
   - Goal: Move screen updates (LCD, eye animations) to a dedicated background thread to decouple from gait timing.
   - Deliverable: Thread owning SimpleEyes + SPI display; runs at ~15 Hz; uses lock/queue for cross-thread state (mood changes).
   - Acceptance: No visual regression; gait loop jitter reduced; display updates independent of main loop.
   - Completed: 2025-12-04 (v0.4.0 b65). DisplayThread class runs at configurable Hz; thread-safe state updates via lock; proper shutdown.

15a. [x] Eye tracking with joystick input
   - Goal: Eyes look in direction of joystick input (strafe/heading → X, speed → Y) at display thread refresh rate.
   - Deliverable: DisplayThread applies look offsets to eye_center_offset/eye_vertical_offset before render; rate limited by thread Hz.
   - Acceptance: Eyes move smoothly in response to joystick; no main loop impact; configurable look_range_x/y in [eyes] section.
   - Completed: 2025-12-04 (v0.4.1 b66). Added eye_vertical_offset to SimpleEyes; DisplayThread manages look direction with change detection.

16. [ ] Telemetry data structures upgrade
   - Goal: Use namedtuple/dataclass for servo/system telemetry for clarity.
   - Deliverable: Definitions + parser returning structured objects.
   - Acceptance: Parsing performance acceptable; code readability improved.

17. [x] Loop timing drift correction
   - Goal: Replace fixed sleep with monotonic next-tick scheduling.
   - Deliverable: Timing helper tracking accumulated drift.
   - Acceptance: Average loop period stability improved; no starvation.
   - Completed: 2025-11-26 (Session 2 - v0.2.0)

18. [x] Keyboard input optimization
   - Goal: Remove per-loop curses wrapper overhead.
   - Deliverable: Persistent stdscr + non-blocking getch approach.
   - Acceptance: Reduced CPU usage; identical input responsiveness.
   - Completed: 2025-11-26 (Session 2 - v0.2.0)

19. [x] Display redraw optimization
   - Goal: Avoid full-frame redraw & rotate every loop.
   - Deliverable: Partial redraw system or layered caching.
   - Acceptance: Measured frame time reduction (>10%).
   - Completed: 2025-06-10 (v0.4.10 b75) - Frame hash detection in show_image_rotated() skips identical frames.

20. [x] Display rotation optimization
   - Goal: Minimize expensive rotate operations.
   - Deliverable: Pre-rotated static assets or final composite rotate only.
   - Acceptance: Rotation CPU cost reduced without visual regression.
   - Completed: 2025-06-10 (v0.4.10 b75) - show_image_rotated() combines rotation + RGB565 in single pass.

21. [x] Serial read batching & sanitation
   - Goal: Batch reads (read(in_waiting)) and correct CRLF handling; robust line splitting.
   - Deliverable: Updated readTeensy function with error guards.
   - Acceptance: Throughput improved; malformed lines safely skipped.
   - Completed: 2025-11-26 (Session 2 - v0.2.0). Added incomplete line buffering.
   - Note: CR/CRLF sanitation fixed (2025-11-24); batching pending.

22. [x] Teensy disconnect overlay
   - Goal: Show 'NO TEENSY' watermark when telemetry absent beyond threshold.
   - Deliverable: Overlay rendering + timeout logic.
   - Acceptance: Clear user feedback; disappears on reconnect.
   - Completed: 2025-12-04 (v0.4.4 b69). Red 'NO TEENSY' watermark with shadow drawn centered on display when ctrl.teensy is None.

23. [x] Dynamic posture messaging
   - Goal: Display correct posture name in auto-disable countdown.
   - Deliverable: last_posture tracking & message update.
   - Acceptance: Message reflects actual posture each time.

24. [x] Suppress redundant enable sequences
   - Goal: Skip sending enable commands when already enabled.
   - Deliverable: Local enable state tracking.
   - Acceptance: Fewer serial writes; no missed enable.

25. [ ] Joystick command tolerance filtering
   - Goal: Reduce spam by skipping tiny deltas.
   - Deliverable: Threshold logic before send_cmd.
   - Acceptance: Serial traffic decreased; control feel unchanged.

26. [ ] Document telemetry field mapping
   - Goal: Clarify S1/S2/S3/S4 schemas and indices.
   - Deliverable: Inline comments + doc snippet.
   - Acceptance: New contributors can interpret telemetry quickly.

27. [ ] Configurable voltage/temp thresholds
   - Goal: Move color palette thresholds to config.
   - Deliverable: Config keys + usage in display rendering.
   - Acceptance: Adjusting file changes display scaling.

28. [ ] Split display rendering
   - Goal: Break UpdateDisplay into pure sub-functions.
   - Deliverable: Functions (render_status_text, render_servo_bars, apply_overlays, mirror_output).
   - Acceptance: Top-level function shorter & clearer.

29. [x] Enhanced status overlays
   - Goal: Overlays for no gamepad, stale telemetry, safety states.
   - Deliverable: Overlay manager.
   - Acceptance: Accurate, non-flickering overlays.
   - Completed: 2025-12-04 (v0.4.7 b72). Full overlay system with:
     - 'NO TEENSY' (red) - Teensy disconnected
     - 'NO CONTROLLER' (red) - Gamepad disconnected
     - 'NO TELEMETRY' (orange) - Connected but no data received yet
     - 'DISABLED' (yellow) - Robot in disabled state
     - Overlays stack vertically, render immediately on status change.

30. [x] Replace getGait with dict
   - Goal: Remove chained if/elif.
   - Deliverable: Dict lookup with default.
   - Acceptance: Function simplified; identical output.
   - Completed: 2025-12-04 (v0.4.2 b67). Replaced 10-branch if/elif chain with _GAIT_NAMES dict and .get() lookup.

31. [x] Remove duplicate numpy import
   - Goal: Eliminate redundant import to tidy header.
   - Deliverable: Single import statement.
   - Acceptance: Duplicate removed; no side effects.

32. [x] Fix newline parsing typo
   - Goal: Correct '/r/n' to '\r\n'.
   - Deliverable: Adjusted replace/split logic.
   - Acceptance: Proper line termination handling.

33. [x] Rename _menuVisable variable
   - Goal: Correct spelling to _menuVisible.
   - Deliverable: Variable renamed everywhere.
   - Acceptance: No reference errors; improved readability.
   - Completed: 2025-12-04 (v0.4.2 b67). Fixed all 9 occurrences.

34. [x] Telemetry stall retry
   - Goal: If telemetry is silent after auto-start, re-send `Y 1` once after backoff.
   - Deliverable: Single retry with 2s backoff; cancels on first S1/S2/S3/S4.
   - Acceptance: Retry triggers only once; no spam; behavior logged. (2025-11-24)

35. [x] Teensy reconnect loop
   - Goal: Detect disconnects, close port, and rescan with backoff; reset telemetry state.
   - Deliverable: Drop after 3 read failures; 1.5s rescan; on reconnect, re-arm telemetry.
   - Acceptance: Auto-recovers on cable unplug/replug; no crashes. (2025-11-24)

36. [ ] Keyboard input during screen mirror
   - Goal: Handle keyboard commands while screen mirroring is enabled (cv2 window active).
   - Deliverable: Route keystrokes through cv2.waitKey() or alternate input path when mirror active.
   - Acceptance: 's', 'k', 't', 'q', 'd', 'g', 'x' keys work whether mirror on or off; no blocking.

## Future / Parking Lot
- Gamepad abstraction layer refactor (events → state snapshot).
- Telemetry rate adaptation (dynamic downsample when idle).
- Modular plugin system (e.g., vision, behavior modules).
- PID/impedance experimental hooks (Phase 2 alignment).

---

## Code Review Recommendations (v0.4.8 b73)

Structural/code quality assessment completed 2025-12-04.

### Summary Statistics
- **File:** `controller.py` — 2277 lines
- **Global variables:** 50+ (with underscores)
- **Global statements:** 20+ occurrences across functions
- **Exception handlers:** 18 bare `except Exception` blocks
- **Duplicated enable patterns:** 9× `send_cmd(b'LEG ALL ENABLE', ...)` 

---

### HIGH Impact Recommendations

| # | Issue | Description | Effort |
|---|-------|-------------|--------|
| H1 | **Global/Controller state duplication** | Controller class mirrors ~30 globals via `sync_globals_to_ctrl` + `sync_ctrl_to_globals`. Maintaining parity is error-prone and adds 80+ LOC of boilerplate. | Med |
| | *Recommendation:* Complete migration to Controller-only state. Remove global state variables; reference `ctrl.*` throughout. The phase functions already accept `ctrl` — extend pattern. | |
| H2 | **Enable sequence duplication** | 9× repetitions of `send_cmd(b'LEG ALL ENABLE', force=True); send_cmd(b'ENABLE', force=True)` across keyboard/gamepad handlers. | Low |
| | *Recommendation:* Create `ensure_enabled()` helper. Pattern: `if not _enabledLocal: ensure_enabled()`. Already have `_enabledLocal` tracking. | |
| H3 | **Magic index 9 for robot enabled** | 9 usages of `state[9]` or `ctrl.state[9]` for "robot enabled" flag. No named constant or property accessor. | Low |
| | *Recommendation:* Add `IDX_ROBOT_ENABLED = 9` constant and/or `@property is_enabled` in Controller. Improves readability & guards against schema drift. | |

---

### MEDIUM Impact Recommendations

| # | Issue | Description | Effort |
|---|-------|-------------|--------|
| M1 | **Bare exception handlers (×18)** | Generic `except Exception:` with silent pass. Masks bugs; complicates debugging. Examples: lines 165, 274, 318, 347, 783, 1285. | Low-Med |
| | *Recommendation:* Narrow to expected types (SerialException, OSError). Log unexpected errors to console or file for post-mortem. | |
| M2 | **DisplayThread + direct UpdateDisplay overlap** | `phase_display_update` calls either thread method or direct `ctrl.update_display()`. Thread path updates same globals; unclear ownership. | Med |
| | *Recommendation:* When thread enabled, all display state flows through thread. Remove direct path branch; simplify phase function. | |
| M3 | **Config load in module-level try/except** | 150+ lines of config parsing wrapped in single try/except (lines 1459-1545). Any parse failure silently uses defaults. | Low |
| | *Recommendation:* Per-section try/except with logging. Bad value logged; rest of config still loads. | |
| M4 | **forceDisplayUpdate in 3 places** | Flag set by Controller instance, globals, and DisplayThread. Coordination unclear. | Low |
| | *Recommendation:* Consolidate into single source (Controller instance); thread reads via update_state(). | |

---

### LOW Impact Recommendations (Quick Wins)

| # | Issue | Description | Effort |
|---|-------|-------------|--------|
| L1 | **Commented-out imports/code** | ~15 lines: `#import os`, `#import random`, `#time.sleep(1.1)`, dead rectangles. Adds visual noise. | Trivial |
| | *Recommendation:* Remove or document why preserved. | |
| L2 | **Inconsistent docstrings** | Some functions have detailed docstrings (`UpdateDisplay`, `bezier_point`); others (`processTelemS1`) have minimal or none. | Low |
| | *Recommendation:* Add one-liner docstrings to all public functions; use Args/Returns format consistently. | |
| L3 | **Repeated `print(..., end="\r\n")`** | ~60 print statements all end with `end="\r\n"`. Verbose; easy to miss during edits. | Trivial |
| | *Recommendation:* Create `log(msg, ...)` wrapper that appends `\r\n`. Optionally integrate with proper logging. | |
| L4 | **hardcoded sleep values** | `time.sleep(0.02)`, `time.sleep(0.05)` scattered in shutdown. No config keys. | Trivial |
| | *Recommendation:* Minor, but could add to [timing] section for consistency. | |

---

### Timing Discipline Assessment

| Check | Status | Notes |
|-------|--------|-------|
| Fixed sleep avoidance | ✅ Pass | `phase_loop_sleep` uses monotonic scheduling |
| Blocking I/O guards | ⚠️ Partial | Serial reads are non-blocking (`in_waiting`), but cv2.imshow may stall on resize |
| Thread priority | ✅ Pass | DisplayThread uses `os.nice(5)` to lower priority |
| Gait tick budget | ✅ Pass | `_gaitSendDivisor` rate-limits FEET commands to ~55 Hz |
| Telemetry fallback | ✅ Pass | Falls back to 166 Hz if S1 stalls > 50 ms |

**Conclusion:** Timing discipline is solid. Main risks are cv2 mirror mode on slow displays and potential GIL contention during SPI writes (mitigated by `show_image_fast()`).

---

### Error Handling Assessment

| Category | Status | Notes |
|----------|--------|-------|
| Serial disconnect recovery | ✅ Good | 3-strike policy → rescan with backoff |
| Telemetry retry | ✅ Good | Single retry after 2 s stall |
| Gamepad loss | ✅ Good | Retry counter + periodic rescan |
| Config parse errors | ⚠️ Weak | Entire section skipped silently |
| Telemetry parse errors | ✅ Good | Per-line skip with verbose logging |

---

### Cohesion Assessment

**Controller class:** Medium cohesion. Contains Teensy state, gamepad state, display state, gait input state. Consider splitting into:
- `TeensyConnection` (serial, telemetry)
- `DisplayManager` (eyes, overlays, thread)
- `InputManager` (gamepad, keyboard)

**Gait engine:** Good cohesion. Separate module with clear interface (`tick()`, `get_feet_bytes()`).

**Phase functions:** Good cohesion. Each handles single responsibility. Could be further grouped into a `LoopPhases` class for namespace clarity.

---

### Recommended Prioritization

1. **H2 — `ensure_enabled()` helper** (Quick, high value)
2. **H3 — Named constants for telemetry indices** (Quick, reduces bugs)
3. **M1 — Narrow exception handlers** (Medium, improves debuggability)
4. **H1 — Complete Controller migration** (Larger, but eliminates sync bugs)
5. **M3 — Per-section config parsing** (Medium, improves startup robustness)

---

## Conventions
- **Version Policy**: MUST bump `CONTROLLER_VERSION` (patch increment) with every session that modifies code behavior, adds features, or fixes bugs.
- **Build Policy**: MUST increment `CONTROLLER_BUILD` on every code edit. This number is monotonic and never resets across version changes.
- One commit per completed backlog item; update this file and `rPi controller/CHANGELOG.md` together.
- Avoid large rewrites; prefer incremental, testable patches.
- Keep loop responsiveness: avoid blocking calls > 2 ms except intentional sleeps.

## Last Updated
2025-12-04

## Python Controller TODO (moved)

This file is now a pointer. The authoritative, project-wide TODO list
for both firmware and the Python controller lives at:

- `docs/TODO.md`

Please add, update, and complete controller tasks in the
"Python controller TODOs" section of `docs/TODO.md`.
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

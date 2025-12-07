## Python Controller 0.4.42 (b108) — 2025-12-07
### Fixed
- **First-tap ignore v2**: Simplified to counter-based approach
  - `show()` sets `_ignore_touches = 2`
  - Counter decrements each time finger is lifted
  - Touches ignored while counter > 0
  - More reliable than state-machine approach

## Python Controller 0.4.41 (b107) — 2025-12-07
### Fixed
- **First-tap ignore fix**: Now properly tracks finger lift after wake
  - Uses two-phase state: `_just_woke` and `_wake_finger_lifted`
  - Wake tap is ignored until finger is lifted AND a new touch occurs
  - Fixes issue where all taps were being ignored after wake

## Python Controller 0.4.40 (b106) — 2025-12-07
### Changed
- **LCARS color variety**: Frame now uses distinct colors for top sweep, bottom sweep, and vertical bar
  - Each palette defines: `frame_top`, `frame_bottom`, `frame_bar`, `accent_alt`
  - Tabs use full 5-color palette with more variety
  - Item values and selection indicators alternate colors for visual interest
  - Right-edge accent bar uses alternate accent color

### Fixed
- **First-tap ignore**: Tapping the screen to wake up the menu now ignores that first tap
  - Prevents accidental menu actions when bringing up the display
  - Menu shows but first tap is consumed; subsequent taps work normally
  - Flag clears when finger is lifted, ready for next intentional tap

## Python Controller 0.4.39 (b105) — 2025-12-07
### Fixed
- **LCARS tab touch alignment**: Touch zones now match actual LCARS tab positions
  - Tabs start at y=32 (after top frame sweep)
  - Each tab is 22px tall with 3px gap
  - Touch ignores gaps between tabs
  - MARS theme still uses even height division

## Python Controller 0.4.38 (b104) — 2025-12-07
### Improved
- **LCARS frame structure**: Added bottom-left swept corner matching top-left
- **Vertical bar placement**: Now spans only between the two swept corners (top sweep bottom to bottom sweep top)
- **Proper L-bracket**: Both corners curve outward with thick-to-thin transitions

## Python Controller 0.4.37 (b103) — 2025-12-07
### Fixed
- **LCARS tab/frame overlap**: Tabs now start at x=14 (after 12px frame bar + gap)
- **Continuous left frame**: Vertical bar extends full height alongside tabs
- **Bottom bar alignment**: Starts at x=12 to connect with vertical frame bar

## Python Controller 0.4.36 (b102) — 2025-12-07
### Improved
- **LCARS design guidelines applied** (based on lcars-terminal.de tutorial):
  - Thick-to-thin frame transitions at swept corners (never same thickness)
  - Proper LCARS swept corner element (quarter-circle with inner cutout)
  - Rounded caps used correctly as terminators and buttons
  - Consistent spacing grid throughout interface
  - 3 font sizes only: Main Title, Sub Header, Normal Data
  - Selection indicator bars instead of filled backgrounds
  - Improved scroll bar with proper rounded caps

## Python Controller 0.4.35 (b101) — 2025-12-07
### Added
- **Eye V Center**: New menu option to adjust vertical eye position (±30px), saved to config
- **Config persistence**: Menu theme, LCARS palette, and eye V center now saved to `controller.ini`
- **Save functions**: `save_menu_settings()`, `save_eye_center_offset()` for persistent storage

## Python Controller 0.4.34 (b100) — 2025-12-07
### Fixed
- **LCARS L-bracket**: Curve now faces outward (bottom-right) as in authentic LCARS design
- **Tab sizing**: Reduced to 24px height with 3px gaps to fit all 5 tabs on 170px display

### Added
- **LCARS color palettes** (from TheLCARS.com):
  - **Classic**: TNG/DS9 orange, peach, violet, gold
  - **Nemesis**: Cool blues, ghost, midnight
  - **LwrDecks**: Warm oranges, harvest gold, butter
  - **PADD**: Arctic blues, radioactive cyan
- **Palette selector**: System menu > Palette option to switch between palettes

## Python Controller 0.4.33 (b99) — 2025-12-07
### Added
- **LCARS theme**: Star Trek inspired menu visual style with:
  - Pill-shaped tabs with rounded ends
  - Orange/peach/lavender/blue color palette on black
  - L-shaped corner bracket decorations
  - Selection indicator bars instead of filled backgrounds
- **Theme selector**: System menu now has Theme option to switch between MARS (default) and LCARS

## Python Controller 0.4.32 (b98) — 2025-12-07
### Added
- **Touch close button**: X button in top-right corner of menu to close via touchscreen

## Python Controller 0.4.31 (b97) — 2025-12-07
### Improved
- **Functional scroll bar**: Tap above/below the thumb to scroll up/down by one page
- **Wider scroll bar**: Increased from 20px to 30px track width for easier touch interaction
- **Arrow placement**: Moved value arrows further left (x=265) to avoid scroll bar overlap

## Python Controller 0.4.30 (b96) — 2025-12-07
### Fixed
- **Touch handling reworked**: `touched()` now returns current touch state; debouncing moved inside `handle_touch()` so value adjustments are debounced but touch detection works continuously
- **Scroll bar overlap**: Moved value arrows and action indicators left (from x=305 to x=285) to avoid overlapping with 20px scroll bar

## Python Controller 0.4.29 (b95) — 2025-12-07
### Fixed
- **Touch debouncing**: Now uses `_marsMenu.touched()` for proper debouncing (was incorrectly using legacy menu's touch detection, causing rapid value changes)
- **Wider scroll bar**: Increased from 5px to 20px track width, 25px minimum thumb height for easier touch interaction

## Python Controller 0.4.28 (b94) — 2025-12-07
### Improved
- **Menu UX enhancements**:
  - Touch debouncing: single value change per touch (must lift finger to change again)
  - Visual scroll bar for menus with more items than fit on screen
  - Larger fonts (11→14px) and taller items (22→34px) for easier touch
  - Fixed touch coordinate rotation for 270° rotated display
  - Immediate display refresh after any input (moved display update after input phases)
  - Replaced Unicode arrows with ASCII for font compatibility

## Python Controller 0.4.27 (b93) — 2025-12-06
### Added
- **MARS menu system**: Complete rebuild of on-screen menu:
  - New `MarsMenu.py` replaces legacy GUIMenu with tabbed interface
  - Categories: Eyes, Gait, Posture, Info, System
  - Controller support: DPAD navigates items, LB/RB switch tabs, A selects, B closes, Start toggles
  - Touch support: Tap tabs, tap items, left/right zones adjust values
  - Safety gate: Menu only opens when robot is disabled and not in motion
  - Touch E-STOP still active: touch during gait immediately stops robot
  - Eye settings: Style (9 types), Human Color, Size, Spacing, CRT Effect
  - System settings: Brightness, Verbose mode, Mirror Display, Save All, Shutdown
  - Info display: Firmware version, Controller version, Battery, Status
- New `handle_button()` method in MarsMenu for clean controller integration

## Python Controller 0.4.13 (b79) — 2025-12-04
### Added
- **Walking turn**: Yaw while walking for arc motion:
  - Right stick Y controls turn rate: up = CCW (left turn), down = CW (right turn)
  - Maximum turn rate: ±60 deg/s (configurable)
  - Differential stride: legs on outside of turn take longer strides, inside legs shorter
  - Works with all gait types (Tripod, Wave, Ripple)
  - Pure rotation (speed=0, turn≠0) rotates in place
  - Combined with strafe (right stick X) for full omni-directional control
- Added `LEG_HIP_X` and `LEG_HIP_Z` constants for leg position geometry

## Python Controller 0.4.12 (b78) — 2025-12-04
### Added
- **Phase-locked gait transitions**: Smooth switching between gait types:
  - RB button now triggers phase-locked transition instead of instant switch
  - Waits for current gait's phase boundary before transitioning
  - Blends foot positions between old and new gait over 500ms
  - Cosine interpolation for smooth acceleration/deceleration at blend boundaries
  - New `GaitTransition` class in gait_engine.py manages the state machine
  - States: IDLE → WAITING (for phase) → BLENDING → COMPLETE

## Python Controller 0.4.11 (b77) — 2025-12-04
### Added
- **Wave and Ripple gaits**: Two new gait patterns for different terrain/speed needs:
  - WaveGait: One leg swings at a time (6 phases) — maximum stability, slow speed
  - RippleGait: Two diagonal legs swing together (3 phases) — good speed/stability balance
  - RB button now cycles: Tripod → Wave → Ripple → Stationary → Tripod
- Removed STRAFE_DEBUG_LOG file writing

## Python Controller 0.4.0 (b65) — 2025-12-04
### Added
- **Display thread**: Background thread for eye animation and LCD updates:
  - Runs at configurable Hz (default 15Hz, via `[display] thread_hz`)
  - Enable/disable via `[display] thread_enabled` in controller.ini
  - Decouples rendering from main loop timing — eyes now animate during gait!
  - Thread-safe state updates from main loop
  - Proper shutdown on exit
- Minor version bump (0.3 → 0.4) for significant architectural change

## Python Controller 0.3.31 (b64) — 2025-12-04
### Added
- **Complete config centralization**: All magic numbers now in `controller.ini`:
  - `[serial]`: error_threshold (disconnect after N read errors)
  - `[gait]`: cycle_ms, step_len_mm, base_y_mm, max_step_len_mm, overlap_pct, smoothing_alpha, send_divisor, cmd_throttle_ms
  - `[gait]`: Bezier curve shape: bezier_p1_height, bezier_p1_overshoot, bezier_p2_height, bezier_p3_height, bezier_p3_overshoot
  - `[eyes]`: spacing_offset, center_offset, eyelid_angle, blink_percent_step, rotation, size_x, size_y
- GaitParams dataclass extended with Bezier control point parameters
- send_cmd() and apply_posture() now use config values for throttle_ms and auto_disable_s

## Python Controller 0.3.30 (b63) — 2025-12-04
### Added
- **Centralized configuration**: Magic numbers moved to `controller.ini` sections:
  - `[timing]`: loop_target_ms, teensy_loop_us, telem_sync_fallback_ms, teensy_reconnect_backoff_s
  - `[display]`: screen_refresh_ms, brightness, auto_disable_s
  - `[gait]`: Added width_min_mm, width_max_mm, lift_min_mm, lift_max_mm for configurable ranges
- All timing and display constants now loaded from config with safe fallback defaults

## Python Controller 0.3.29 (b62) — 2025-12-03
### Added
- **Persistent gait settings**: Gait width and lift height are now saved to `controller.ini` [gait] section when left stick button is released.
  - Settings persist across controller restarts
  - Loaded on startup and applied to new gait sessions
  - Fallback defaults: width=100mm, lift=60mm

## Python Controller 0.3.28 (b61) — 2025-12-03
### Added
- **Lift height adjustment**: Hold left stick button + right trigger to adjust lift height (20-100mm).
  - Right trigger (code 9) maps 0-1023 to 20-100mm range
  - Saved lift height used when starting new gait
  - Real-time adjustment during active gait supported

## Python Controller 0.3.27 (b60) — 2025-12-03
### Changed
- **Gait width range**: Expanded from 60-140mm to 50-175mm for more adjustment range.

## Python Controller 0.3.26 (b59) — 2025-12-03
### Fixed
- **Left trigger event code**: Changed from code 2 to code 10 for Xbox controller left trigger detection.
- **Analog debug output**: Added debug print for all analog events when left stick button is held (for controller mapping).

## Python Controller 0.3.25 (b58) — 2025-12-03
### Added
- **Gait width adjustment**: Hold left stick button + left trigger to adjust gait width (lateral foot offset).
  - Left trigger maps 0-1023 to 60-140mm width range
  - Width is saved when left stick button is released
  - Saved width is used when starting new gait (LB button)
  - Real-time adjustment during active gait supported

## Python Controller 0.3.24 (b57) — 2025-12-03
### Changed
- **Bezier lift height tuning**: Increased lift_mm to 60mm and Bezier peak control point to 150% to compensate for curve attenuation. Actual peak now reaches target height.
- **Bezier control points adjusted**: P1 (15% height), P2 (150% height), P3 (35% height) for proper arc shape.

## Python Controller 0.3.23 (b56) — 2025-12-03
### Added
- **Bezier curve foot trajectories**: Swing phase now uses a 5-point Bezier curve for smooth, natural foot motion:
  - P0: Start (back position, on ground)
  - P1: Initial lift with slight backward overshoot (10% height, -110% stride)
  - P2: Peak height at center (-10% stride position)
  - P3: Descending with forward overshoot (25% height, +110% stride)
  - P4: End (front position, on ground)
- **Bezier utilities**: Added `bezier_point()`, `binomial_coefficient()`, and `map_range()` functions to gait_engine.py

### Changed
- **Stance phase**: Simplified to linear interpolation (equivalent to 2-point Bezier)

## Python Controller 0.3.22 (b55) — 2025-12-03
### Fixed
- **Safe shutdown on exit**: Script now sends `LEG ALL DISABLE` + `DISABLE` before stopping telemetry when exiting.

### Changed
- **Strafe (crab walk) fully working**: Rewrote `_apply_leg_rotation` with proper 2D rotation math:
  - Base foot position computed from `LEG_BASE_SIN/COS` arrays
  - Full rotation angle = leg base rotation + side-dependent heading offset
  - Left legs (LF,LM,LR) add heading, right legs (RF,RM,RR) subtract for coordinated body-frame motion
  - Stride vector rotated then translated back to leg base position

## Python Controller 0.3.21 (b54) — 2025-11-30
### Fixed
- **Middle leg strafe motion**: LM and RM legs now properly participate in crab walk. Strafe component routed to their local Z axis (body lateral direction) with correct sign per side (LM positive, RM negative).
- **Print statement line endings**: All print() calls now use `end="\r\n"` for proper terminal display.
- **MODE IDLE on gait start**: Python gait (LB button) now sends MODE IDLE to prevent Teensy TEST mode from overriding FEET commands.

## Python Controller 0.2.1 (b28) — 2025-11-26
### Added
- **Monotonic build number**: Introduced `CONTROLLER_BUILD` (starts at 28, never resets across version changes). Increment on every code edit for precise tracking.
- Bumped `CONTROLLER_VERSION` to 0.2.1.

## Python Controller 0.2.0 (b27) — 2025-11-26
### Changed (Session 2 Performance Improvements)
- **Serial read batching**: Replaced inefficient byte-by-byte reads with batch `read(in_waiting)` and incomplete line buffering. Partial lines are now preserved across reads, preventing data corruption on line boundaries.
- **Loop timing drift correction**: Replaced fixed `time.sleep(0.005)` with monotonic next-tick scheduling. Sleep time is now calculated to hit the target tick time, preventing accumulated drift. Includes catch-up logic to reset if >50ms behind.
- **Keyboard input optimization**: Replaced per-loop `curses.wrapper(getkey)` with persistent `init_keyboard()` + `poll_keyboard()`. Curses is now initialized once and cleaned up on exit, eliminating significant per-loop overhead.
- **Removed debug prints**: Cleaned up temporary X/Y button debug print statements.
- Bumped `CONTROLLER_VERSION` to 0.2.0.

## Python Controller 0.1.24 — 2025-11-25
### Changed
- **X button (306)**: Now toggles between `MODE TEST` and `MODE IDLE` with internal state tracking.
- **Y button (307)**: Changed from `MODE TEST` to `TUCK` posture command with 4-second auto-disable.
- Bumped `CONTROLLER_VERSION` to 0.1.24.

## Python Controller 0.1.23 — 2025-11-25
### Changed
- **Xbox controller button remapping**: Updated face buttons and right joystick to use modern Teensy firmware commands:
  - **Y button (307)**: `MODE TEST` - enters tripod gait test mode
  - **X button (306)**: `MODE IDLE` - exits test mode to idle
  - **A button (304)**: `STAND` - stand posture with 4-second auto-disable
  - **B button (305)**: `HOME` - home position with 4-second auto-disable
  - **Right joystick press (318)**: `DISABLE` - immediate disable/stop
- **Removed obsolete commands**: Eliminated legacy single-letter commands (`r`, `w`, `t`, `m`, `f`) and steering mode (`b0`/`b1`) that are not supported by current firmware.
- **Left joystick press (317)**: Reserved for future functionality.
- Bumped `CONTROLLER_VERSION` to 0.1.23.

## Python Controller 0.1.22 — 2025-11-25
### Fixed
- **Enhanced power button detection**: Added support for multiple Xbox guide button event codes (139, 158, 172) and debug logging for unhandled button events.
- Bumped `CONTROLLER_VERSION` to 0.1.22.

## Python Controller 0.1.21 — 2025-11-25
### Fixed
- **Power button application exit**: Fixed Xbox controller power button (event code 172) not terminating the application. Added `requestExit` flag to Controller class; power button handler now sets this flag, and main loop checks it to set `_run = False` and break execution.
- Bumped `CONTROLLER_VERSION` to 0.1.21.

## Python Controller 0.1.20 — 2025-11-25
### Fixed
- **Menu display bug**: Menu toggle ('n' key) now properly syncs between global state and Controller instance, fixing menu visibility in mirror display mode.
- **Stand auto-disable bug**: Stand command ('s' key) now properly schedules auto-disable by syncing `autoDisableAt` to Controller instance.
- **Tuck auto-disable bug**: Tuck command ('k' key) now properly schedules auto-disable by syncing `autoDisableAt` to Controller instance.
- **Auto-disable execution**: Auto-disable logic now uses Controller state consistently for reliable execution.
- Bumped `CONTROLLER_VERSION` to 0.1.20.

## Python Controller 0.1.19 — 2025-11-25
### Fixed
- **Critical bugfix**: Added missing `connect_teensy()` and `handle_teensy_disconnect()` methods to Controller class that were referenced but not implemented, causing runtime crashes.
- Bumped `CONTROLLER_VERSION` to 0.1.19.

## Python Controller 0.1.18 — 2025-11-25
### Changed
- Controller refactor (phase 1e): completed migration of all global state into `Controller` class. All polling, display, and connection management now encapsulated in instance methods.
- Added Teensy connection management with `controller.ini` serial config support (port override and baud rate from config).
- Bumped `CONTROLLER_VERSION` to 0.1.18.

## Python Controller 0.1.17 — 2025-11-24
### Changed
- Controller refactor (phase 1d): migrated Teensy telemetry polling, gamepad event handling, and display update into `Controller.poll_teensy()`, `Controller.poll_gamepad()`, and `Controller.update_display()` methods. Behavior unchanged (structural consolidation only).
- Bumped `CONTROLLER_VERSION` to 0.1.17.

## Python Controller 0.1.16 — 2025-11-24
### Fixed
- Mirror ('m') and verbose ('v') toggles now persist correctly after Controller state mirroring (sync into class before global write-back).
### Changed
- Bumped `CONTROLLER_VERSION` to 0.1.16.

## Python Controller 0.1.15 — 2025-11-24
### Changed
- Controller refactor (phase 1c): moved telemetry auto-start/retry logic into `Controller.housekeeping()`; behavior unchanged.
- Bumped `CONTROLLER_VERSION` to 0.1.15.

## Python Controller 0.1.14 — 2025-11-24
### Changed
- Controller refactor (phase 1b): added `Controller` state mirroring for `verbose`, `mirror`, and telemetry timers; no behavior change.
- Bumped `CONTROLLER_VERSION` to 0.1.14.

## Python Controller 0.1.13 — 2025-11-24
### Optimized
- Font caching: cache PIL truetype fonts to avoid reloading each frame (reduces per-loop overhead in UpdateDisplay and logo rendering).
### Changed
- Bumped `CONTROLLER_VERSION` to 0.1.13.

## Python Controller 0.1.12 — 2025-11-24
### Added
- Teensy reconnect loop: detects repeated read failures, drops port after 3 errors, rescans with 1.5s backoff, and resets telemetry auto-start state.
### Changed
- Bumped `CONTROLLER_VERSION` to 0.1.12.

## Python Controller 0.1.11 — 2025-11-24
### Added
- Telemetry stall retry: after auto-start, if still no S1/S2, resend `Y 1` once after a 2s backoff.
### Fixed
- Bumped `CONTROLLER_VERSION` to 0.1.11.

## Python Controller 0.1.10 — 2025-11-24
### Fixed
- Corrected newline normalization in `readTeensy`: now replaces CRLF/CR to LF instead of the incorrect `/r/n` literal.
- Bumped `CONTROLLER_VERSION` to 0.1.10.

## Python Controller 0.1.9 — 2025-11-24
### Changed
- 'q' key now performs full shutdown sequence: idle ('I'), then `LEG ALL DISABLE`, then master `DISABLE` to ensure leg torque drop before global disable.
- Bumped `CONTROLLER_VERSION` to 0.1.9.

## Python Controller 0.1.8 — 2025-11-24
### Changed
- Unified command emission as bytes; `send_cmd` now normalizes str/bytes safely and prevents prior mixed-type suppression issues.
- Posture helper (`apply_posture`) now enforces bytes posture tokens and fixes comparison bug from mixed str/bytes.
- Bumped `CONTROLLER_VERSION` to 0.1.8.

## Python Controller 0.1.7 — 2025-11-23
### Added
- send_cmd debug instrumentation: prints suppressed and sent commands (throttle reasons) when verbose and for gait/motion commands, or globally with 'g' toggle.
- S2 telemetry debug parity ensured (raw + parsed enable flag list) alongside S1.
### Changed
- Bumped `CONTROLLER_VERSION` to 0.1.7.

## Python Controller 0.1.6 — 2025-11-23
### Added
- S2 telemetry debug: raw S2 segment and parsed enable flag list printed alongside S1 when debug ('d') toggle active.
### Changed
- Bumped `CONTROLLER_VERSION` to 0.1.6.

## Python Controller 0.1.5 — 2025-11-23
### Added / Changed
- Posture abstraction helper (`apply_posture`) consolidating TUCK/STAND enable + auto-disable flow.
- Centralized send path (`send_cmd`) with throttling and local enable tracking.
- Telemetry schema validation: strict length checks for S1/S2/S3/S4; short packets skipped, long truncated with warning.
- Telemetry auto-start after 750 ms grace using spaced command `Y 1`; stop with `Y 0` on exit.
- Corrected telemetry command spacing (firmware expects `Y 1` / `Y 0`).
- Raw + parsed S1 debug view (`d` key toggles) to aid bring-up of controller-side parsing.
### Notes
- Controller version introduced as `CONTROLLER_VERSION` in `controller.py` for disciplined patch bumps.
- Earlier (2025-11-22) host-side protocol normalizations (LF only, posture keys, A button enable toggle) captured in `controller.py` inline change log; consolidated here for repository history.

## [0.2.13] - 2025-11-14
### Added
- Virtual impedance layer (`MarsImpedance`) with joint-space and simple Cartesian (Z-axis focused) modes. Applied on top of PID/base commands with bounded centidegree corrections per joint.
- Config keys parsed from `/config.txt`: `imp.enabled`, `imp.mode=off|joint|cart`, `imp.joint.k_spring_milli.<coxa|femur|tibia>`, `imp.joint.k_damp_milli.<coxa|femur|tibia>`, `imp.cart.k_spring_milli.z`, and `imp.cart.k_damp_milli.z`.
- IMP command family for runtime tuning and persistence: `IMP LIST`, `IMP ENABLE|DISABLE`, `IMP MODE <OFF|JOINT|CART>`, `IMP JSPRING|JDAMP <COXA|FEMUR|TIBIA|ALL> <milli>`, `IMP CSPRING|CDAMP <X|Y|Z|ALL> <milli>`. When SD is present, changes are written back to `/config.txt` via the existing single-key update helper.
- STATUS now includes an `[IMP]` section summarizing impedance enabled flag, mode, joint-space gains, and Cartesian gains (x/y/z) to support on-robot tuning.

## 0.2.23 — 2025-11-19
## 0.2.24 — 2025-11-21

- Telemetry: Refactored S1 (aggregate state) and S2 (per-leg vin/temp/enable) streaming into helpers `telemetryPrintS1`/`telemetryPrintS2` in `functions.ino`. No format change; behavior unchanged. Internal maintainability improvement only.

## 0.2.25 — 2025-11-21

- Telemetry: S2 now emits one line per tick with all 18 servos in canonical order (LF,LM,LR,RF,RM,RR × C,F,T). Removed per-line leg tag; each triple is `vinV, tempC, en`. The `en` flag is now `legEnabled AND jointEnabled` to reflect effective availability in the UI. S1 format unchanged.

## 0.2.26 — 2025-11-21

- Telemetry: Split S2/S3 responsibilities. S2 now carries only enable flags for all 18 servos: `S2:<en0>,...,<en17>` where each is `(legEnabled AND jointEnabled)`. New S3 streams volt/temperature for all 18: `S3:<v0>,<t0>,...,<v17>,<t17>` with voltage as `X.Y` (V) and temperature in °C.


- UX: Reformatted startup splash so firmware version/build, build metadata+loop_hz, and UART/config summary each print on their own line for better readability, keeping content and timing behavior unchanged.

## 0.2.27 — 2025-11-21

- Telemetry: Added single-letter `Y` command as a master toggle for the compact streams: `Y1`/`Y0` turns S1/S2/S3 on/off without changing FK mask. Streams default to ON for backward compatibility.
- Telemetry: Re-enabled legacy `RR_FK` body-frame and leg-frame stream for the current RR leg, gated by the existing `FK <LEG|ALL> <ON|OFF>` mask. Format unchanged.

## 0.2.28 — 2025-11-21

- Telemetry: Decoupled compact telemetry from FK gating. S1/S2/S3 now emit based solely on `Y` (on/off) and serial availability; `FK` mask no longer affects them. `RR_FK` remains controlled by the `FK` mask only.

## 0.2.29 — 2025-11-21

- Telemetry: Updated S3 format to match controller expectations — now outputs all 18 voltages followed by all 18 temperatures: `S3:<v0>,...,<v17>,<t0>,...,<t17>` (canonical leg/joint order). Previous interleaved `v,t` pairs format deprecated.

## 0.2.22 — 2025-11-18

- Safety/collision: Refined foot-to-foot XZ clearance check to consider only same-side adjacent (LF–LM, LM–LR, RF–RM, RM–RR) and opposite-side facing (LF–RF, LM–RM, LR–RR) leg pairs using BODY-frame FK foot positions when `safety.collision` is enabled, still comparing against `safety.clearance_mm` and triggering the unified STAND+DISABLE collision path.

## 0.2.21 — 2025-11-18

- Commands/UX: Added `CONFIG` command with `CONFIG` dumping `/config.txt` contents over USB serial when SD is present, wrapped with simple `NO_SD`/`NO_CONFIG` errors and documented in HELP under [SYSTEM].

## 0.2.20 — 2025-11-18

- Commands/UX: LIMITS command now takes absolute centidegree limits (`LIMITS <COXA|FEMUR|TIBIA> <MIN_CD> <MAX_CD>`), reports them in raw centidegrees via `LIMITS LIST`, and persists them to `/config.txt` as degrees (`joint_limits.<LEG>.<joint>.min_deg/max_deg`).

## 0.2.19 — 2025-11-18

- Commands/UX: LIMITS command now persists updated shared joint workspace values back to `/config.txt` for all legs as `joint_limits.<LEG>.<joint>.{min_deg,max_deg}` using the existing single-key update helper.

## 0.2.18 — 2025-11-18

- HELP: Added LIMITS command documentation under [GEOMETRY], describing `LIMITS LIST` and `LIMITS <COXA|FEMUR|TIBIA> <MIN_DEG> <MAX_DEG>` as shared per-joint workspace controls across all legs.

## 0.2.17 — 2025-11-18

- Policy: Documented an explicit assistant/automation rule in `MARS_Hexapod.ino` and `docs/PROJECT_SPEC.md` that every assistant-made code change must bump `FW_VERSION` (patch) and `FW_BUILD` and add a concise `CHANGELOG.md` entry describing the behavior or configuration changes.

## 0.2.16 — 2025-11-18

- Safety/collision: Reworked collision handling to use a common STAND+DISABLE path. Any collision-style violation (including foot-to-foot keep-out and the new joint workspace enforcement) now commands STAND, then immediately disables torque and enters LOCKOUT with `LOCKOUT_CAUSE_COLLISION` set.
- Joint workspace: Effective joint commands now respect the configured `joint_limits.*` workspace when `safety.collision` is enabled. Attempts to command joints outside their workspace trigger the STAND+DISABLE collision sequence instead of silently clamping.
- Commands/UX: Added `LIMITS` command family with `LIMITS LIST` and `LIMITS <COXA|FEMUR|TIBIA> <MIN_DEG> <MAX_DEG>` to tune shared per-joint workspace limits (degrees) applied consistently across all legs and persisted to `/config.txt` as `joint_limits.<LEG>.<joint>.{min_deg,max_deg}`.

## 0.2.15 — 2025-11-18

- Estimator: Finalized tuning rubric and defaults based on hardware testing. Recommended starting values are now `est.cmd_alpha_milli=100`, `est.meas_alpha_milli=150`, and `est.meas_vel_alpha_milli=100`, with updated guidance in `docs/ESTIMATOR_TUNING.md` and `docs/PROJECT_SPEC.md`.
- Commands/UX: STAND and TUCK now force MODE IDLE semantics before applying motion (via `modeSetIdle()`), ensuring tripod test gait or other planners are not concurrently writing to joint targets when these supervisory poses are requested.
- Control smoothing: Servo MOVE_TIME for the fast send path is now set to approximately one control tick (`≈1000/g_loop_hz` ms, floored to 1 ms), turning each incremental command into a short ramp at the device level and slightly reducing visible jitter, especially on the coxa joints.

## 0.2.14 — 2025-11-17

- Commands/UX: Added `LOOP <hz>` console command to adjust control loop frequency at runtime (50..500 Hz). Command applies via `configApplyLoopHz`, updates `g_loop_hz`/`g_config_loop_hz`, and, when SD is enabled, persists `loop_hz=<hz>` in `/config.txt` (create or update key). HELP now documents LOOP under [SYSTEM], and NOTES mention the relationship between `loop_hz` and `logging.rate_hz`.
## [0.2.11] - 2025-11-13
### Changed
- PID now accounts for variable loop timing: derivative uses de/dt with dt from the loop timer; integral accumulates e*dt (cd·ms) with proper scaling. Keeps low-pass derivative smoothing and correction clamp. Behavior is more consistent under jitter.

## [0.2.10] - 2025-11-13
### Changed
- PID shadow telemetry (`PID_SHADOW`) now includes, per leg and joint, the PID vs base delta (`diff_cd`), the PID error used (`err_cd = target - est`), the read/estimate angle (`est_cd`), and the base target angle (`tgt_cd`). This enriches shadow-mode analysis without changing control behavior.

## [0.2.7] - 2025-11-13
### Added
- Joint PID integration (sparse-feedback): per-joint P/PI/PD correction computed each tick between estimator and send. Default disabled and safe with zero gains; bounded correction clamp prevents large nudges.
- Config keys: `pid.enabled` and `pid.{kp,ki,kd}_milli.<coxa|femur|tibia>` parsed at boot from `/config.txt`.
- STATUS now includes a `[PID]` section showing enabled flag and kp/ki/kd milli-gains per joint group.

## [0.2.9] - 2025-11-13
### Added
- PID shadow mode: new config keys `pid.mode=active|shadow` and `pid.shadow_report_hz` (default 2 Hz). In shadow mode, PID corrections are computed but not applied; diffs vs base (non-PID) rate-limited outputs streamed periodically as `PID_SHADOW` lines (per-leg c/f/t centidegree deltas). Active mode preserves prior behavior.

### Changed
- Firmware version bumped to 0.2.9; build number incremented.

## [0.2.8] - 2025-11-13
### Added
- Estimator: exponential smoothing toward last command each tick with measurement correction on RR updates; PID uses the estimate for error computation between sparse reads.
- Derivative smoothing: low-pass filtered D term to reduce spikes from measurement noise.
- Config keys: `est.cmd_alpha_milli`, `est.meas_alpha_milli`, and `pid.kd_alpha_milli.<coxa|femur|tibia>`.

## [0.2.5] - 2025-11-13
### Changed
- Permanently enabled fast per-leg batched send and staggered feedback reads; removed compile-time flags. Loop rate stays elastic (adaptive window logic retained). This codifies the proven optimizations and simplifies configuration.

## [0.2.4] - 2025-11-13
### Changed
- Feedback reads staggered behind `MARS_FB_STAGGER` (default ON): position is read every RR visit; vin and temp alternate across visits for the same servo. OOS logic unchanged semantically (any valid vin OR temp resets; only both invalid increments). This reduces `fb_us` per tick and frees timing budget for 166 Hz.

## [0.2.3] - 2025-11-13
### Added
- Fast send path (Option A) behind `MARS_FAST_SEND` feature flag. Batches three MOVE_TIME frames per leg, asserts OE once per leg, writes back-to-back, and flushes/deasserts only on the round-robin feedback leg each tick. Preserves soft-limit and rate-limit. Intended for Teensy 4.1 + 6 UARTs.
### Performance
- Measured STATUS [TIMING] (user data) shows `send_us` reduced from ~19 ms to ~1.7 ms in TEST mode; tick now dominated by feedback (`fb_us` ≈ 4.9 ms). Idle `send_us=0` as expected when disabled. Per-call send profiling counters are 0 under fast path (bypasses `move_time()`); total per-tick send profiling remains valid.

## [0.2.2] - 2025-11-13
### Docs
- Added "Versioning policy" to `docs/PROJECT_SPEC.md`: SemVer MAJOR.MINOR.PATCH with explicit confirmation required for major/minor bumps; FW_BUILD monotonic counter printed in splash/STATUS; patch/build may auto-increment on assistant edits.

## [0.2.1] - 2025-11-13
### Added
- Build metadata: Added FW_BUILD (monotonic build number) and included in splash/STATUS output for traceability across SemVer bumps.

## [0.2.0] - 2025-11-13
### Added
- Phase 2 kickoff: Introduced send-path profiling instrumentation gated by `MARS_PROFILE_SEND`. STATUS now reports `send_profile.per_call_us` and `send_profile.total_us` min/avg/max with counts when enabled. No behavior change.
### Policy
- Phase 2 workflow: defer commits until explicitly instructed; auto-bump FW patch per assistant edit. Changelog entries reserved for completed TODOs or behavior changes to reduce noise.

## [0.1.116] - 2025-11-12
### Validation
- Loop timing validated: STATUS jitter metrics (min/avg/max) observed within <10% of 6024 µs tick budget under idle and tripod test gait; no sustained overruns. Acceptance criteria for Phase 1 timing met. No functional changes beyond existing probes.

## [0.1.115] - 2025-11-12
### Added
- STATUS [TIMING]: Jitter metrics (`jitter_us=min/avg/max`) computed over a rolling window; helps validate the 166 Hz target under idle and test gait.
 - Splash: UART mapping sanity indicator appended when `/config.txt` contains `uart.*` keys — shows `(cfg: match|mismatch)` against the fixed compile-time mapping.
### Changed
- Config loader: Now parses persisted safety keys at boot: `safety.soft_limits`, `safety.collision`, `safety.temp_lockout_c`, and `safety.clearance_mm` so SAFETY settings saved to `/config.txt` are restored on restart.

## [0.1.114] - 2025-11-12
### Added
- Config key expansion: Parse tripod gait parameters at boot from `/config.txt`:
	`test.trigait.{cycle_ms,height_mm,basex_mm,steplen_mm,lift_mm,overlap_pct}` with sane clamps. These populate the existing TEST mode runtime parameters so STATUS reflects configured defaults on startup. HELP notes the new keys.
### Changed
- Startup offsets: At boot, read each servo’s hardware angle offset and populate `g_offset_cd` before the first STATUS. Also accept `offset_cd.<LEG>.<joint>` keys as a seed (hardware read overwrites). Updated `docs/PROJECT_SPEC.md` to reflect current behavior.

## [0.1.100] - 2025-11-12
### Fixed
- Calibration/SAVEHOME semantics: Now truly centidegree-based end-to-end. The routine clears existing hardware angle offsets to establish a raw baseline, re-reads the raw position, computes `offset_cd = raw_cd - 12000` (clamped ±3000 cd), applies and saves it. Removed erroneous legacy `/24` conversion that produced offsets ~24× too small. As a result, `OFFSET LIST` values now match the centidegree delta to 12000, and the logical `pos_cd` readings converge to ~12000 after calibration. Note: STATUS `pos_cd` grid updates round-robin (one servo/tick), so it may take ~18 ticks (~0.1 s at 166 Hz) to reflect new values across all joints.

## [0.1.101] - 2025-11-12
### Fixed
- Angle offset unit conversion: The lx16a-servo library stores hardware angle offsets in tick units (1 tick ≈ 24 centideg ≈ 0.24°), not centidegrees directly. Firmware wrappers now translate between centidegrees and ticks: reads multiply by 24, writes round(cd/24) and clamp to ±125 ticks (≈ ±3000 cd). SAVEHOME and OFFSET LIST now report accurate centidegree offsets; previous behavior could misinterpret raw tick values as cd.

## [0.1.102] - 2025-11-12
### Changed
- STATUS formatting: Reformatted STATUS output to mirror HELP style with explicit section headers (`[SYSTEM]`, `[ENABLES]`, `[TELEMETRY]`, `[HOME]`, `[TEST]`, `[TIMING]`, `[OOS]`) and one key/value per indented line. No semantic or data changes; improves human readability and simplifies host-side log parsing. Firmware version bumped to 0.1.102.

## [0.1.103] - 2025-11-12
### Added
- Command: `TUCK` — moves all enabled, in-service joints to a compact pose: coxa=0 cd, femur=20000 cd, tibia=0 cd. Respects leg/joint enable masks and the OOS bitmask (skips OOS joints). Motion still passes through existing soft-limit and rate-limit safety in the main loop. HELP updated under [MOTION].

## [0.1.104] - 2025-11-12
### Changed
- TUCK enhancements: `TUCK [LEG|ALL]` now accepts an optional leg to tuck a single side; default tucks all enabled legs. Sequencing is non-blocking: tibias move to 0 cd first and, once within tolerance, femur (now 19000 cd) and coxa (0 cd) are commanded together for each qualifying leg. Respects enable and OOS flags throughout. HELP updated accordingly.

## [0.1.105] - 2025-11-12
### Fixed
- TUCK convergence: Updated loopTick sequencing to continuously override femur/coxa targets after tibia reaches tolerance, instead of issuing them only once. Previously femur/coxa could be overwritten by other motion logic (e.g., test gait), leaving only tibia tucked. Completion now requires tibia plus femur and coxa convergence (or timeout). Firmware version bumped to 0.1.105.

## [0.1.106] - 2025-11-12
### Changed
- TUCK pose adjustment: Coxa target updated from 0 cd to 12000 cd (logical center) while femur (19000 cd) and tibia (0 cd) remain unchanged. Provides symmetric tucked stance relative to calibrated neutral. Firmware version bumped to 0.1.106.

## [0.1.107] - 2025-11-12
### Fixed
- Position validity: A measured position of 0 cd was treated as "false" and ignored in several places, causing fallbacks to the last-sent value (e.g., appearing as 24000) and stalling TUCK tibia convergence. Added explicit `g_meas_pos_valid` flags and now choose measured vs last-sent based on validity, not truthiness. TUCK tibia target restored to 0 cd. Firmware version bumped to 0.1.107.

## [0.1.108] - 2025-11-12
### Fixed
- TUCK left/right mirroring: Right-side legs (RF/RM/RR) now mirror femur/tibia TUCK targets around `home_cd` so their motion matches left-side direction. Coxa remains at logical center (12000 cd) for all legs. Also added STATUS [TUCK] section with active flag, masks, and per-leg tibia meas/eff snapshots to aid bring-up.

## [0.1.109] - 2025-11-12
### Added
- TUCK parameters are now runtime-configurable and persisted to `/config.txt` via `TUCK SET <PARAM> <VALUE>`. Supported params: `TIBIA`, `FEMUR`, `COXA` (center-only=12000), `TOL_TIBIA`, `TOL_OTHER`, `TIMEOUT` (ms). TUCK controller uses these values and continues to mirror femur/tibia on right legs.

## [0.1.110] - 2025-11-12
### Changed
- STATUS output trimmed: removed temporary `[TUCK]` debug section (active/masks/params and tibia meas/eff snapshots) now that TUCK bring-up is complete. This reduces STATUS print time and keeps the output focused. No functional changes to TUCK sequencing or parameters.

## [0.1.111] - 2025-11-12
### Changed
- HELP updated: TUCK command help now reflects runtime-configurable parameters and the `TUCK SET <PARAM> <VAL>` subcommand (persisting tuck.* keys: TIBIA, FEMUR, COXA, TOL_TIBIA, TOL_OTHER, TIMEOUT). Clarifies that tuck sequencing uses tibia-first then femur+coxa with mirrored angles on right legs.

## [0.1.112] - 2025-11-12
### Added
- Command: `TUCK PARAMS` prints current tuck parameters (tibia/femur/coxa, tolerances, timeout) sourced from tuck.* config/runtime values. HELP now lists `TUCK PARAMS`.

## [0.1.113] - 2025-11-12
### Added
- TUCK debug instrumentation (feature flag `MARS_TUCK_DEBUG`): prints start of sequence, per-leg tibia convergence (`TUCKDBG TIBIA`), femur+coxa convergence (`TUCKDBG FEMCOXA`), and completion or timeout summary. Adds `g_tuck_tibia_mask` to track stage 1 per leg. STATUS remains trimmed; debug output is lightweight and can be disabled by setting `MARS_TUCK_DEBUG` to 0.

## [0.1.113] - 2025-11-12
### Added
- TUCK debug instrumentation (feature flag `MARS_TUCK_DEBUG`): prints start of sequence, per-leg tibia convergence (`TUCKDBG TIBIA`), femur+coxa convergence (`TUCKDBG FEMCOXA`), and completion or timeout summary. Adds `g_tuck_tibia_mask` to track stage 1 per leg. STATUS remains trimmed; debug output is lightweight and can be disabled by setting `MARS_TUCK_DEBUG` to 0.

## 2025-11-11
## [0.1.99] - 2025-11-11
### Fixed
- Offset wrappers: `angle_offset_read/adjust/write` now restrict servo lookup to the specified leg row instead of scanning all legs. Previous behavior returned the first matching ID globally, causing identical offset values across legs for joints sharing IDs. Requires re-running `SAVEHOME` to regenerate accurate per-leg `offset_cd` values.
- Version bumped to 0.1.99.

## [0.1.98] - 2025-11-11
### Changed
- Calibration flow: `SAVEHOME` now operates purely in centidegrees and performs a clear→read→compute sequence. For each enabled, in-service servo it clears the hardware offset, re-reads the raw position, computes and clamps the absolute `offset_cd = pos_cd - 12000` (±3000), writes and saves it, then persists both `home_cd.<LEG>.<joint>` and `offset_cd.<LEG>.<joint>` to `/config.txt`. Runtime `g_home_cd` and `g_offset_cd` updated accordingly. Serial log line `HOMESSET` reflects the new values.
- Commands: `OFFSET LIST` and `OFFSET CLEAR` updated to use cd semantics end-to-end (no legacy unit conversions). `CLEAR` sets absolute offset to 0 cd, saves, and updates in-memory `home_cd` from the post-clear measured position.
- Version: Bumped FW_VERSION to 0.1.98.

## 2025-11-08
## [0.1.97] - 2025-11-11
### Changed
- Calibration I/O: Wired hardware angle offset helpers to the servo library. Removed in-RAM stubs so `SAVEHOME` and `OFFSET` now operate on real device offsets when running on Teensy. Host builds continue to use no-op stubs from the fake include. Bumped FW_VERSION to 0.1.97.

## [0.1.96] - 2025-11-10
### Fixed
- Logging probe: `g_probe_log_us` now resets to 0 when logging is disabled (previously retained last sampled duration).

## [0.1.95] - 2025-11-10
### Changed
- Loop timing: Raised `LOOP_HZ_DEFAULT` to 166 (spec target) and set `g_config_loop_hz` default ceiling to 166 so adaptive logic can ramp up. Previous default of 100 Hz prevented increases when config key not present.

## [0.1.94] - 2025-11-10
### Changed
- Control loop: adaptive rate logic now uses a 128-tick window. Increases by +1 Hz when a full window has zero overruns; decreases by -1 Hz when a window has >=8 overruns. This replaces strict consecutive-count logic that prevented upward adjustments under mild jitter.

## [0.1.93] - 2025-11-10
### Fixed
- Logging timing probe scope: moved `g_probe_log_us` capture inside sampled block to eliminate undefined identifier compile error (log_start_us out-of-scope). Version bump.

## [0.1.92] - 2025-11-10
### Added
- Timing: Added `g_probe_log_us` to measure logging segment duration; STATUS timing probes now output `serial/send/fb/log/tick`.

## [0.1.91] - 2025-11-10
### Changed
- STATUS: timing probes line now prints in explicit microseconds format: `tprobe_us=serial/send/fb/tick`.

- Version: Bumped FW_VERSION to 0.1.76.

## 2025-11-09
- Protocol: Completed centralization of OK responses. Removed remaining per-command printOK() calls (RAW/RAW3/FOOT/FEET/MODE/I/T/TEST/STAND/SAFETY/HOME/SAVEHOME/OFFSET/SERVO). Dispatcher now emits a single OK on success; handlers only emit ERR on failure.
- Version: Bumped FW_VERSION to 0.1.77.

## 2025-11-09
- Build: Standardized all SD feature guards to `#if defined(MARS_ENABLE_SD) && MARS_ENABLE_SD` for consistency; removed unused `extern printOK` declaration post centralization cleanup.
- Version: Bumped FW_VERSION to 0.1.78.

## 2025-11-09
- Logging: Implemented Phase 1 compact CSV logging. Config keys: `logging.enabled`, `logging.rate_hz` (divisor of loop_hz), `logging.mode` (0=compact), `logging.header` (write header). Lazy opens `/logs/<millis>.csv`, buffered 8KB with opportunistic flushes; rows include `time_ms,leg,joint,cmd_cd,meas_cd,vin_V,temp_C,err`. Sampling uses a tick divisor with minimal overhead; writes only when buffer has data.
- Version: Bumped FW_VERSION to 0.1.80.

## 2025-11-09
- Commands: Added `LOG` command suite for runtime control of logging: `LOG ENABLE <ON|OFF>`, `LOG RATE <hz>`, `LOG MODE <COMPACT|FULL>`, `LOG HEADER <ON|OFF>`, `LOG FLUSH`, `LOG STATUS`. HELP updated with [LOGGING] section.
- Logging: Implemented FULL mode (mode=1) to log all legs each sample (18 rows). Added richer file header (metadata lines and CSV header).
- Version: Bumped FW_VERSION to 0.1.83. (Defined in MARS_Hexapod.ino). Compact mode now logs 3 rows per sample (RR leg only); FULL logs 18 rows.

## 2025-11-09
- Telemetry: Disabled automatic FK streaming at startup (g_fk_stream_mask now initialized to 0 instead of LM leg). Users must explicitly enable via `FK <LEG|ALL> ON`.
- Version: Bumped FW_VERSION to 0.1.84.

## 2025-11-10
- Commands: Added `LOG TAIL <N>` to print the last N data rows of the current log file, prefixed with a header line numbered 0. Skips metadata comments and any in-file CSV header.
- HELP updated to include `LOG TAIL`.
- Version: Bumped FW_VERSION to 0.1.85.
 - UX: `LOG ENABLE` now turns logging on without requiring ON; `LOG DISABLE` turns it off. Backward-compatible with `LOG ENABLE ON|OFF`.

## 2025-11-10
- Logging: Added size-based rotation with `LOG ROTATE <ON|OFF>` and `LOG MAXKB <KB>` (threshold in KB, min 100KB, max 1GB). Runtime status shows `rotate`, `max_kb`, current file `size_kb`, and `seq`. Filenames now use `<millis>_seq<seq>.csv`. Rotation flushes buffer, closes file, increments sequence, and opens a new file preserving header metadata. Hard clamp enforced at 1GB. Default max set to 10MB.
- HELP: Updated logging section with ROTATE and MAXKB commands.
- Version: Bumped FW_VERSION to 0.1.86.

## 2025-11-10
- Logging: Full mode rows now include per-servo OOS flag (`oos` column). Added `LOG CLEAR` to delete the current log file and reopen on the next sample. Updated headers to list new column. Version bumped to 0.1.87.

## 2025-11-10
- Logging: Switched CSV schema to one line per leg (aggregated 3 servos). Compact mode now emits a single row for the current round-robin leg; Full mode emits 6 rows (all legs) instead of 18. Columns: `time_ms,leg,cmd0_cd,meas0_cd,vin0_V,temp0_C,oos0,cmd1_cd,meas1_cd,vin1_V,temp1_C,oos1,cmd2_cd,meas2_cd,vin2_V,temp2_C,oos2,err`. Reduces row count by 3× in full mode and simplifies leg-wise analytics. Version bumped to 0.1.88.

## 2025-11-10
- Logging: Added persistence of runtime LOG settings (`RATE`, `MODE`, `HEADER`, `ROTATE`, `MAXKB`) back to `/config.txt` (excluding `logging.enabled`). Introduced new config keys `logging.rotate` (0|1) and `logging.max_kb` (threshold in KB, min 100, clamp 1GB). Config parser upper bound for `logging.rate_hz` aligned with runtime (≤500). FW_VERSION bumped to 0.1.89.
## 2025-11-10
- Uptime: Introduced continuous wrap-safe uptime accumulator (`g_uptime_ms64`) updated each loop iteration. STATUS now reports uptime without incrementing it, eliminating dependency on STATUS call frequency. FW_VERSION bumped to 0.1.90.

## 2025-11-08
- STATUS: Restored multi-line STATUS with grouped content: system/config, test parameters, safety toggles, leg/joint enables, offsets grid, OOS mask, and FK mask. Kept strings in PROGMEM via F().
- Version: Bumped FW_VERSION to 0.1.75.

## 2025-11-08
- Startup: Restored/enhanced non-blocking startup splash with build date/time and concise runtime summary (FW, loop_hz, UART mapping, config status).
- Version: Bumped FW_VERSION to 0.1.74.

## 2025-11-08
- UX: Echo the processed serial command back to the host in the form `> <line>` immediately after the handler runs. Helps logging and CLI interop.
- Version: Bumped FW_VERSION to 0.1.73.

## 2025-11-08
- Fix: Cleaned up `functions.ino` by removing orphaned legacy command code and unmatched preprocessor fragments; restored a minimal `handleLine` shim that delegates exclusively to the modular dispatcher and a compact `buffersInit`. Resolved compile diagnostics in this unit.
- Fix: Reintroduced lightweight FK helpers `fk_leg_body` and `fk_leg_both` into `functions.ino` after refactor to resolve undefined reference at link time and restore FK telemetry and estimates.
- Fix: Restored/stubbed helper functions required by modular commandprocessor (printOK/ERR, HELP/STATUS, nextToken, rebootNow, angle_offset_*) to close remaining undefined references. Added minimal in-memory angle offset cache. Bumped FW_VERSION.
- Version: Bumped FW_VERSION to 0.1.72.

## 2025-11-08
- Refactor: Removed the legacy if/else command chain from `functions.ino`; `handleLine` now delegates exclusively to the modular dispatcher in `commandprocessor.ino`.
- Version: Bumped FW_VERSION to 0.1.70.

## 2025-11-08
- Commands: Wired the new modular dispatcher into `handleLine` in `functions.ino`. The legacy if/else chain was removed and all serial commands now route through `commandprocessor.ino`'s enum-based switch and per-command handlers.
- Behavior: No functional changes expected; this consolidates command handling so features like `OFFSET` and `SAVEHOME` reliably execute through the new path.
- Version: Bumped FW_VERSION to 0.1.69.

# Changelog

## 2025-11-08
- Fix: Cleaned up `functions.ino` by removing orphaned legacy command code and unmatched preprocessor fragments; restored a minimal `handleLine` shim that delegates exclusively to the modular dispatcher and a compact `buffersInit`. Resolved compile diagnostics in this unit.
- Fix: Reintroduced lightweight forward kinematics helpers `fk_leg_body` and `fk_leg_both` (removed during refactor) to resolve linker undefined reference and restore FK-based telemetry and foot position estimation.
- Version: Bumped FW_VERSION to 0.1.71.

## 2025-11-07
- Refactor: Introduced modular command processor (`commandprocessor.ino`) with an enum-based dispatcher and per-command handler routines (HELP, STATUS, REBOOT, ENABLE, DISABLE, FK, LEGS, SERVOS, LEG, SERVO, RAW, RAW3, FOOT, FEET, MODE, I, T, TEST, STAND, SAFETY, HOME, SAVEHOME, OFFSET). Improves maintainability and isolates parsing logic from core firmware loop.
- Readability: Expanded previously dense one-line handlers into structured blocks; corrected preprocessor directive placement (column 0) to avoid Teensy compile errors; added shared header `command_types.h` to expose `CommandType` before Arduino auto-generated prototypes.
- Build: Resolved 'CommandType does not name a type' error by moving enum to header included early; no functional changes to command behaviors.
- Version: Bumped FW_VERSION to 0.1.68.
- Calibration: `SAVEHOME` now adjusts and saves each servo's hardware angle offset so the current physical neutral maps to 12000 cd (±30° clamp). Residual is persisted to `/config.txt` as `home_cd.<LEG>.<joint>` to preserve exactness.
- Commands: Added `OFFSET LIST` (report per-servo angle offsets in centideg) and `OFFSET CLEAR <LEG|ALL> <JOINT|ALL>` (clear hardware offsets; homes updated to maintain pose).
- STATUS: New `offset_cd` grid shows current hardware angle offsets per leg/joint (centidegrees).
- Docs: Updated spec with calibration flow and new commands.
- Version: Bumped FW_VERSION to 0.1.67.

## 2025-11-04
- Telemetry: Added `FK <LEG|ALL> <ON|OFF>` command to toggle FK body-frame stream per leg. RR_FK output is now gated by a mask (defaults to LM only to preserve prior behavior).
- Version: Bumped FW_VERSION to 0.1.65.

## 2025-11-04
- Docs: Updated `docs/PROJECT_SPEC.md` to document the full SAFETY command suite (LIST/OVERRIDE/SOFTLIMITS/COLLISION/TEMPLOCK/CLEARANCE), expanded STATUS grouped sections, and added implemented config keys (`safety.soft_limits`, `safety.collision`, `safety.temp_lockout_c`, and `test.trigait.overlap_pct`).
- Version: Bumped FW_VERSION to 0.1.64.

## 2025-11-03
- UX: Synced HELP with implemented commands (added SAFETY SOFTLIMITS/COLLISION/TEMPLOCK; removed duplicates). Reworked STATUS into grouped sections for readability without changing semantics.
- Version: Bumped FW_VERSION to 0.1.63.

## 2025-11-03
	- `safety.soft_limits=<true|false>` — clamps commanded joints to configured min/max when true.
	- `safety.collision=<true|false>` — enables/disables foot-to-foot keep-out checks.
	- `safety.temp_lockout_c=<C>` — sets over-temperature lockout threshold in Celsius.

## 2025-11-03
- Telemetry: Print FK body-frame foot position (x/y/z) for the leg served in the round-robin feedback each tick to aid bring-up.
- Version: Bumped FW_VERSION to 0.1.62.

## 2025-11-03
- Safety: FOOT/FEET collision checks now use FK-estimated body-frame positions (X/Z) to predict post-command clearance, matching the loop's FK-based safety and reducing startup false positives.
- Version: Bumped FW_VERSION to 0.1.60.

## 2025-11-03
- Safety: Added simple foot-to-foot keep-out in the X/Z plane. New serial command `SAFETY CLEARANCE <mm>` and config key `safety.clearance_mm` control the minimum clearance between feet; violation triggers a safety lockout (E40 COLLISION). STATUS prints `safety_clearance_mm`.
- Safety UX: Added `SAFETY LIST` and `SAFETY OVERRIDE <ALL|TEMP|COLLISION|NONE>`; STATUS now includes a compact safety summary `safety=<state> cause=0x.. override=0x..`.
- Version: Bumped FW_VERSION to 0.1.59.

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

# rPi Controller Change Log

All modifications specific to the Python controller (controller.py and related Python support code) are logged here. This file is independent from the root firmware CHANGELOG.

FORMAT: `YYYY-MM-DD  <summary>`

## Entries

2025-12-31  v0.8.3 b232: Input: Screen mirror toggle now works in socket mode (joy_controller); Back+Start combo toggles cv2 mirror window.
2025-12-31  v0.8.2 b231: Safety: Low battery protection - graceful shutdown (stop gait → TUCK → DISABLE) when voltage drops below critical threshold (default 10.0V); configurable via [low_battery] section; SAFETY menu items for enable/thresholds/status; recovery voltage hysteresis prevents servo current spikes from re-enabling.
2025-12-31  v0.8.1 b230: Display: Battery voltage now uses average of all servo voltages with low-pass filter (alpha=0.1) for stable display during walking.
2025-12-31  v0.8.0 b229: Architecture: Refactored main loop to Three-Layer Architecture (Firby/Gat); Layer 1=166Hz (timing, Teensy I/O, gait, safety), Layer 2=~33Hz (input, behaviors, display), Layer 3=~1Hz (future SLAM). Configurable via gait.layer2_divisor.
2025-12-29  v0.7.44 b228: PointCloud: WebSocket server for 3D point cloud visualization (SLAM prototype); integrates ToF + IMU; configurable via [pointcloud] section; Three.js viewer with live/accumulate modes.
2025-12-29  v0.7.43 b227: Autonomy: ABXY buttons auto-disable autonomy for manual override; added debug output for non-CONTINUE actions; fixed ToF attribute names (distance_mm, status).
2025-12-27  v0.7.42 b226: Autonomy: Added Back+A gamepad combo to toggle autonomy mode; refactored toggle logic to _toggle_autonomy() helper.
2025-12-27  v0.7.41 b225: Autonomy: Added AUTO menu tab with behavior toggles (obstacle/cliff/caught foot/patrol), threshold adjustments, live status (active behavior, last action), save settings; save_behavior_settings in config_manager.
2025-12-27  v0.7.40 b224: Autonomy: Integrated arbiter into main loop (phase_autonomy); 'a' key toggles autonomy mode; config parsing for [behavior] section; sensor state gathering for behaviors.
2025-12-27  v0.7.39 b223: Autonomy: Created behavior_engine.py (350 lines) with Action enum, Behavior base class, BehaviorArbiter; behaviors.py with ObstacleAvoidance, CliffDetection, CaughtFootRecovery, Patrol; [behavior] config section.
2025-12-27  v0.7.38 b222: Display: LCARS palette now syncs live on menu change; ToF distance moved to top of heatmap.
2025-12-27  v0.7.37 b221: Display: Added 4 LCARS palettes (Classic, Nemesis, LowerDecks, PADD); palette selection via engineering_lcars_palette config; increased font sizes, black text.
2025-12-27  v0.7.36 b220: Display: Added LCARS theme for engineering view; configurable via engineering_lcars in [display].
2025-12-27  v0.7.35 b219: Display: Added phone-style battery icon to EYES mode (top-right corner); configurable via show_battery_icon in [safety_display].
2025-12-26  v0.7.34 b218: Bugfix: Startup display race condition - display thread now waits for set_startup_complete() before rendering eyes.
2025-12-26  v0.7.33 b217: Bugfix: Startup crash after ToF init - fixed undefined try_teensy_connect/_teensyPort/_teensyBaud; use connect_teensy with config values.
2025-12-26  v0.7.32 b216: Startup: Connect to Teensy, controller, and start telemetry during splash for seamless eyes transition.
2025-12-26  v0.7.31 b215: Startup: Full Mars background on splash; configurable delay (0-30s, default 5s) via display.startup_delay_s.
2025-12-26  v0.7.30 b214: Startup: Add StartupSplash with scrolling log during initialization; shows Mars image and progress messages for touch, menu, display thread, IMU, leveling, ToF.
2025-12-26  v0.7.16 b200: FK fix: Rewrote fk_leg_points() to match firmware convention - uses relative angles from 120° home, proper yaw/alpha/beta formulas. Increased 3D view zoom.
2025-12-26  v0.7.15 b199: Bugfix: Fixed draw_hexapod_wireframe_3d() call - parameter names azimuth→azimuth_deg, elevation→elevation_deg.
2025-12-26  v0.7.14 b198: Bugfix: Engineering view now displays immediately when switching modes (added mode_changed detection to display thread).
2025-12-25  v0.7.13 b197: Engineering: 3D wireframe hexapod view with FK-based leg positions; D-pad rotates view (L/R=azimuth, U/D=elevation); temp-colored segments; S6 joint telemetry wired to display.
2025-12-24  v0.7.12 b196: Eyes: Added rotation support to basic eyes using NEAREST resampling (no interpolation) to avoid flicker.
2025-12-24  v0.7.11 b195: Eyes: Rewrote basic eye rendering (ELLIPSE, RECTANGLE, ROUNDRECTANGLE, X) to draw directly to display like special eyes; eliminates rotation/paste flicker.
2025-12-24  v0.7.10 b194: Display: Disabled CRT mode (scanlines caused flicker); reduced display thread to 10Hz; increased min SPI interval to 80ms.
2025-12-24  v0.7.9 b193: Display: Fixed old-style eye flicker by excluding IMU changes from EYES mode render trigger; increased blink_speed to 8.0 for faster blinks.
2025-12-24  v0.7.8 b192: Blink: Made blink animation time-based (blink_speed=3.0/s) instead of frame-based; fixes blink speed varying with loop rate after Xbox controller split.
2025-12-24  v0.7.7 b191: Display: Fixed eye flicker/tearing by adding 40ms min SPI write interval; removed IMU-triggered eye re-renders; increased blink frame divisor to 3.
2025-12-24  v0.7.6 b190: Eye blink: Reduced blink_percent_step from 0.25 to 0.08 for smoother, more natural blink animation without frame tearing.
2025-12-24  v0.7.5 b189: ToF hold-last-valid: Smooths intermittent invalid readings (e.g., glass/mirror reflections) by holding last valid distance for up to 30 frames; stale readings are progressively dimmed; "~" prefix indicates held value.
2025-12-23  v0.7.4 b188: Engineering display: Added hexapod silhouette heat map for servo temperature visualization; layout now shows IMU text header, hex heat map (left) + ToF (right), and voltage bar with hottest servo indicator.
2025-12-23  v0.7.3 b187: Bugfix: LB button now transitions from StandingGait to TripodGait (A→LB works); added _backHeld initialization to fix pounce combo crash.
2025-12-22  v0.7.2 b183: Daemon compat: Handle no-TTY for curses (subprocess without terminal); fixes cbreak() ERR when started by joy_controller.
2025-12-22  v0.7.2 b181: Bugfix: Fixed orphan detection in joy_controller.py — used word-boundary regex to avoid killing itself when searching for controller.py processes.
2025-12-22  v0.7.1 b180: Display: Xbox connection status now reflects actual controller state (connected/sleep), not just socket link to daemon.
2025-12-22  v0.7.0 b179: Architecture: Split Xbox controller into separate joy_controller.py daemon process; IPC via Unix socket; power button starts/stops main controller; haptic feedback on toggle; mars-joy.service for systemd.
2025-12-22  v0.6.8 b178: Motion lean: Body tilts 7-10° into direction of travel when walking; configurable via IMU tab; works with or without leveling.
2025-12-21  v0.6.7 b177: Menu: ToF tab now interactive — edit enabled/resolution/hz/bus, add/remove sensors, apply & restart; settings persist to config.
2025-12-21  v0.6.6 b176: Menu: Added ToF tab with live sensor status, closest distance, temperature, read/error counts, I2C config.
2025-12-21  v0.6.5 b175: Display fix: Removed IMU overlay from EYES mode; engineering view now updates continuously.
2025-12-21  v0.6.4 b174: Display mode: Start button cycles EYES→ENGINEERING→MENU; engineering view shows IMU horizon + ToF heatmap side by side with gradient colors.
2025-12-21  v0.6.3 b172: ToF integration: Integrated tof_sensor.py into controller with [tof] config section, thread startup/shutdown, config parsing.
2025-12-21  v0.6.2 b171: ToF sensor module: Created tof_sensor.py with ToFThread, multi-sensor support, 8×8 distance arrays.
2025-12-21  v0.6.1 b170: Gait direction: Left stick Y now controls forward/backward direction (heading 0°/180°); strafe takes priority.
2025-12-18  v0.5.49 b161: Architecture: Unified telemetry parser (binary/ASCII via telemetry.py with parseBinaryS1..S5, decodeBinaryFrame); added is_robot_enabled property and ensure_enabled() method to Controller class.
2025-12-18  v0.5.48 b160: Modularization complete — added __all__ exports to all modules, updated MODULARIZATION_PLAN.md status.
2025-12-18  v0.5.47 b159: Modularization Phase 7: Final cleanup — removed duplicate get_font, consolidated imports. controller.py: 3838→3825 lines. Total: 4844→3825 (~21% reduction).
2025-12-18  v0.5.46 b158: Modularization Phase 6: Extracted input_handler.py (407 lines) with keyboard/gamepad/Teensy I/O, XboxButton/XboxAxis constants, normalize_joystick/trigger. controller.py: 3946→3838 (-108 lines).
2025-06-18  v0.5.45 b157: Modularization Phase 5: Extracted display_thread.py (520 lines) with DisplayThread, UpdateDisplay, getColor, drawLogo, drawMarsSplash, get_font. controller.py: 4422→3946 (-476 lines).
2025-06-18  v0.5.44 b156: Modularization Phase 4: Created mars_state.py with state container dataclasses (366 lines); foundation for global state consolidation.
2025-06-18  v0.5.43 b155: Modularization Phase 3: Extracted posture.py with ensure_enabled, apply_posture, start_pounce_move (281 lines); wrapper pattern retains original API.
2025-12-18  v0.5.42 b154: Modularization Phase 2: Extracted config_manager.py with save functions and config dataclasses.
2025-12-18  v0.5.41 b153: Modularization Phase 1: Extracted telemetry.py with dataclasses, IDX_* constants, processTelemS1-S5, and helper functions.
2025-12-18  v0.5.40 b152: UX: Use real Mars photo (ESA/Rosetta, CC BY-SA 3.0 IGO) for startup splash instead of procedural drawing.
2025-12-17  v0.5.39 b151: UX: Replaced LCD startup banner with a Mars splash image on black and overlaid firmware/controller versions.
2025-12-17  v0.5.38 b150: Version/docs: Update startup banner firmware version string to match current firmware (0.2.41/b157).
2025-12-17  v0.5.37 b149: Telemetry/protocol: Added binary framed S4 parsing (type=4, 6 per-leg contact flags) to match firmware S4 frames.
2025-12-16  v0.5.36 b148: Telemetry/protocol: Extended binary S1 payload parsing to include battery/current/IMU fields so INFO matches ASCII behavior.
2025-12-16  v0.5.35 b147: Telemetry/protocol: Added binary framed telemetry support (TELEM BIN 1) while keeping Y 1/Y 0 as master enable.
2025-12-16  v0.5.34 b146: Telemetry/perf: Reduced parse-path allocations and gated raw gamepad event printing behind debug.
2025-12-16  v0.5.33 b145: Motion/UI: Added Posture menu tuning params for Pounce, persisted to controller.ini; added Back+Y + U shortcuts to launch.
2025-12-16  v0.5.32 b143: Motion: Added a kinematic “Pounce” move (spider-like jump attack) runnable from Posture menu.
2025-12-16  v0.5.31 b142: Menu: Reordered tab stack (INFO/SYS first); show servo voltage/temp only on INFO.

2025-11-22  Introduced controller CHANGELOG and in-file header log block.
2025-11-22  Normalized Teensy command protocol: removed semicolons; LF-only line termination.
2025-11-22  Added A button enable/disable toggle (LEG ALL ENABLE + ENABLE / DISABLE) using S1 idx9 flag.
2025-11-22  Added 'k' tuck command key: enable sequence then TUCK; auto DISABLE after 5s.
2025-11-22  Added tuck auto-disable scheduler and refactored display of gait state.

2025-11-26  v0.2.0: Session 2 performance optimizations — serial read batching with incomplete line buffering, monotonic loop timing (fixed 200Hz), persistent curses keyboard (init/poll/cleanup pattern).
2025-11-26  v0.2.1 b28: Added monotonic CONTROLLER_BUILD number (never resets across version changes).
2025-11-26  v0.2.2 b29: Bugfix — Fixed spurious Teensy disconnect when no data waiting (readTeensy returning None was incorrectly treated as connection error).
2025-11-26  v0.2.3 b30: Telemetry-synchronized loop timing — derives loop period from Teensy S1 loop_us field; fallback to 166Hz if telemetry stalls >50ms.
2025-11-26  v0.2.4 b31: Startup sends 'LEG ALL ENABLE' before 'Y 1' to ensure telemetry includes valid data from all servos.
2025-11-26  v0.2.5 b32: Keyboard commands now accept both upper and lowercase (t/T, s/S, k/K, q/Q, x/X, m/M, v/V, d/D, g/G, n/N).
2025-11-26  v0.3.0 b33: Gait engine integration - new gait_engine.py module with TripodGait and StationaryPattern classes;
            keyboard controls: w/W=walk (tripod gait), h/H=halt (stop gait), p/P=stationary pattern test.
            FEET commands generated at 166Hz synced to Teensy telemetry.
2025-11-26  v0.3.1 b34: Gamepad gait controls - LB=toggle gait on/off, RB=cycle gait type (tripod/stationary);
            left stick Y=speed, left stick X=heading/strafe, right stick X=turn rate;
            left trigger=reduce step length, right trigger=increase lift height. Eye control preserved when gait off.
2025-11-27  v0.3.2 b35: Fixed gait heading/speed controls - use atan2 for combined stick input giving proper vector heading;
            TripodGait now applies heading_deg rotation and turn_rate_deg_s for yaw; speed_scale modulates stride amplitude.

## Guidelines
- Add one line per logical change affecting runtime behavior or interface.
- Do not duplicate entries already present in the in-file header; keep both synchronized.
- Avoid editing past entries except to correct spelling.

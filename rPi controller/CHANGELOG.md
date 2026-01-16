# rPi Controller Change Log

All modifications specific to the Python controller (controller.py and related Python support code) are logged here. This file is independent from the root firmware CHANGELOG.

FORMAT: `YYYY-MM-DD  <summary>`

## Entries

2026-01-16  v0.12.13 b300: Safety: Collision response now starts a step-to-stand recovery gait (one-leg lift→translate→place) instead of immediate disable, when an unsafe pose is detected and `warn_only` is off.

2026-01-16  v0.12.12 b299: Safety: Collision `stop_on_collision` now enforces an emergency stop (stop gait + idle + LEG ALL DISABLE + DISABLE) when an unsafe pose is detected and `warn_only` is off.

2026-01-16  v0.12.11 b298: Diagnostics: Collision pose compare CSV logger now configured via controller.ini `[collision]` (`pose_log_enabled`, `pose_log_hz`) and exposed in the SAFETY menu + web dashboard.

2026-01-16  v0.12.10 b297: Diagnostics: Added optional collision pose compare CSV log (cmd FEET → IK/FK vs S6 joint telemetry → FK) to debug collision non-triggers. Enable with `MARS_COLLISION_POSE_LOG=1`.

2026-01-15  v0.12.9 b296: Collision tuning: Increased max collision leg radius from 40mm to 100mm across menu/dashboard/config.

2026-01-14  v0.12.8 b295: Collision tuning: Increased max collision safety margin (leg) from 40mm to 100mm across menu/dashboard/config.

2026-01-13  v0.12.6 b293: Engineering view: Accurate body outline using BODY_HULL_XZ polygon (24-vertex convex hull from STL) instead of hexagon approximation. Added collision visualization overlay showing leg segment collision cylinders and collision pair highlights. New DisplayThread methods: set_collision_overlay(), toggle_collision_overlay(), update_collision_state().

2026-01-10  v0.12.6 b293: Feature S2 (Complete): Trained and validated learned collision model. 100k training samples, 3,461 parameter MLP (18→64→32→5), 99.5% agreement with analytical check. Inference: 36.3µs single (14.5× faster than analytical), 1.69M samples/sec batch. Model exported to assets/collision_model.onnx.

2026-01-10  v0.12.5 b292: Feature S2 (Body Geometry): Replaced naive 50mm cylinder body model with accurate convex hull from STL mesh (Frame Assembly.STL). New body model: 24-vertex polygon in XZ plane (206×216mm footprint), height-aware Y bounds (±60mm from hip plane), quick-rejection using bounding/inscribed radii (129.8/103.4mm). Added `_point_in_polygon_xz()`, `check_body_collision_detailed()`. Training data regenerated with accurate geometry.

2026-01-10  v0.12.4 b291: Feature S2 (Started): Created collision_model.py - learned collision prediction module. Includes training data generator (random joint sampling + FK/collision oracle), PyTorch MLP training pipeline (18→64→32→5 architecture), ONNX export for fast inference, CollisionPredictor runtime wrapper. Added validate_pose_safety_learned() and validate_pose_safety_hybrid() hooks in collision.py for integration.

2026-01-09  v0.12.3 b290: Bugfix: Ensure `Controller` always has `self.config` (fixes runtime AttributeError when collision checks/UI sync access config). Fix collision flag update to use `safety_state['collision']`. Remove undefined `audio_pounce()` call.

2026-01-09  v0.12.2 b289: Config parity (Collision): Exposed collision model tuning via MarsMenu SAFETY and web dashboard; added `[collision]` numeric parameters (leg_radius_mm, safety_margin_mm, body_keepout_radius_mm, time_horizon_s, max_velocity_margin_mm) persisted in controller.ini; collision thresholds now read from config.

2026-01-09  v0.12.1 b288: Feature S1 (Complete): Added phase-aware collision risk zones (`LegPhase`, `get_leg_phases_tripod`, `get_risk_pairs`) and velocity-aware safety margins (`compute_velocity_margin`, `TIME_HORIZON_S=50ms`, `MAX_VELOCITY_MARGIN_MM=20mm`). Threshold scales from 35mm (static) to 55mm (at 400mm/s). Updated `validate_pose_safety()` with optional `gait_phase` and `velocity_mm_s` parameters. Comprehensive test suite in `test_collision.py`.

2026-01-08  v0.12.0 b287: Feature S1 (Collision Safety): Implemented analytical pre-IK collision detection. Added `kinematics.py` (Python IK model) and `collision.py` (Capsule-Capsule intersection). Integrated safety check in `send_feet_cmd` to block unsafe commands before transmission. Added `[collision]` section to controller.ini.

2026-01-08  v0.11.14 b286: Refactor M5 (Code Hygiene): Audit and fix bare except blocks with logging; standardize legacy variable names.

2026-01-08  v0.11.13 b285: Refactor M4 (Final Phase): Completely removed legacy state synchronization (`sync_globals_to_ctrl` / `sync_ctrl_to_globals`). Controller class is now the sole source of truth for runtime state. Updated startup logic to direct-assign `ctrl.controller` and `ctrl.menuState`.

2026-01-08  v0.11.12 b284: Refactor M4 Phase 9 (Logic Globals): Migrated Move State, Gait Transition, Display Mode, and IMU Thread logic to Controller instance. Updated runtime phases (`phase_gait_tick`, `poll_gamepad`, `handle_teensy_disconnect`) to use instance state instead of globals. Logic functions are now nearly global-free.
2026-01-08  v0.11.11 b283: Refactor M4 (Phase 8): Migrated PointCloud and Dashboard state to Controller class. Refactored phase_pointcloud/dashboard to use instance state.
2026-01-08  v0.11.10 b282: Refactor M4 (Phase 6): Migrated PID/Impedance/Estimator state to Controller class. Rewrote dashboard handlers to use new state structure. Removed legacy globals.
2026-01-07  v0.11.9 b281: Refactor M4 (Phase 5): Migrated Safety, Low Battery, and Leveling state to Controller class. Removed related globals. Added load_config() method.
2026-01-07  v0.11.8 b280: Refactor M4 (Gait State): Migrated _gaitEngine, _gaitActive, and feet tracking to Controller class. Deleted global helpers send_feet_cmd/_start_standing_gait.

2026-01-06  v0.11.7 b279: Refactor M4 Phase 1: State Encapsulation started. `_setup_mars_menu()` now accepts `ctrl` instance; `phase_keyboard_input()` refactored to use `ctrl.verbose`/`ctrl.mirror` etc directly instead of module globals. Logic updated to make `Controller` instance the state authority for these flags.
2026-01-06  v0.11.6 b278: Refactor M3 Complete: Extracted all menu callbacks from controller.py to menu_controller.py. Created setup functions for PID/IMP/EST, IMU/Leveling, ToF, GAIT, POSTURE/Pounce, AUTONOMY, SAFETY categories plus sync_*_initial_values() helpers. Uses SimpleNamespace context objects with lambda getters/setters to access globals without circular imports. controller.py reduced from 7413 to 6708 lines (~700 lines extracted).
2026-01-06  v0.11.5 b277: Refactor M3.2: Extracted SYSTEM callbacks to menu_controller.py (setup_system_callbacks, sync_system_initial_values).
2026-01-06  v0.11.4 b276: Refactor M3.1: Created menu_controller.py module for extracted menu callbacks. Added setup_eyes_callbacks() and sync_eyes_initial_values() - first extraction of ~80 lines from _setup_mars_menu().
2026-01-06  v0.11.3 b275: Refactor M2 Complete: Added _unpack_config_to_globals() helper (~300 lines) that bridges ControllerConfig dataclass to legacy globals; removed ~350 lines of redundant ConfigParser parsing code; config loading now flows through typed dataclasses. File reduced from 7809 to 7483 lines.
2026-01-06  v0.11.2 b274: Refactor M2: Added load_config() call in controller.py; ControllerConfig loaded on startup alongside legacy ConfigParser for gradual migration.
2026-01-06  v0.11.1 b273: Refactor M2: Integrated load_config() from config_manager; ControllerConfig dataclass now available for dependency injection; updated __all__ exports with all config dataclasses.
2026-01-06  v0.11.0 b272: Refactor: Configured entry point separation (M1) - main.py wrapper created to handle clean startup/shutdown; joy_controller now launches main.py. ConfigManager upgraded (M2) to handle all INI logic; controller.py prepared for dependency injection.
2026-01-06  v0.10.17 b271: Curses: Fixed all print statements across controller, servers, and modules to use end="\r\n" for proper curses terminal output; suppressed pygame banner and I2C warnings during startup; fixed async server shutdown (pointcloud_server, telemetry_server) to exit gracefully without RuntimeError.
2026-01-05  v0.10.16 b270: Help: Press '?' to print keyboard command reference to console. Lists all keyboard shortcuts by category.
2026-01-05  v0.10.15 b269: Menu: Wall Follow behavior in AUTONOMY tab (enable/side/distance); LCD notification overlay when behaviors toggled (auto-dismisses after 2s).
2026-01-05  v0.10.14 b268: Autonomy A4+A5: Wall Following behavior (PD control to maintain target distance from left/right wall); Patrol touch-stop (tap screen to stop patrol gracefully).
2026-01-05  v0.10.13 b267: ToF: Changed filter to mode-based for better motion/SLAM support. Modes: 'off' (raw data), 'light' (reject invalid/noisy only, zero lag), 'full' (EMA smoothing). Default 'light' - filters bad readings without introducing latency. Use 'off' for SLAM.
2026-01-04  v0.10.12 b266: ToF: Added temporal EMA filter for smooth distance readings - eliminates jittery point cloud display. Rejects high-sigma noisy readings, attenuates outliers (sudden jumps). Configurable: filter_enabled, filter_alpha (0.3 default, lower=smoother), filter_sigma_threshold (15mm), filter_outlier_mm (50mm) in [tof] section.
2026-01-04  v0.10.11 b265: Audio: Sound queue for sequential playback - sounds no longer overlap or cut off mid-play. Background worker thread plays queued sounds one at a time. Short UI sounds (click, menu_nav, gait_start etc.) bypass queue for instant response. Priority sounds can interrupt current playback. Queue size configurable (default 10).
2026-01-03  v0.10.10 b264: TTS: Pre-cached TTS phrases as WAV files for instant playback; eliminates Piper ~300ms latency for fixed announcements. 15 cached phrases (startup, shutdown, enabled, disabled, standing, tucking, safety_lockout, autonomy_on/off, obstacle, cliff, battery 10/15/20/25%) via generate_tts_phrases.py; speak_* helpers now use _audio.play() with espeak fallback.
2026-01-03  v0.10.9 b263: TTS: Added Piper neural TTS as alternative engine (engine=piper in [tts] section); natural-sounding voices via piper-tts (en_US-lessac-medium default); supports both espeak (robotic, instant) and piper (neural, natural) backends.
2026-01-03  v0.10.8 b262: Audio: AU4 menu navigation clicks - MarsMenu audio callback for nav/tab/adjust/select events (audio_menu_callback); AU6 created generate_sounds.py to synthesize 22 WAV sound assets (startup, shutdown, enable, disable, click, error, alerts, gait, connect, disconnect, menu, autonomy, postures).
2026-01-03  v0.10.7 b261: Audio: Added gait type change confirmation tones - distinct audio signature per gait (Standing=low double-beep, Tripod=quick triple-pulse, Wave=rising sweep, Ripple=descending cascade, Stationary=gentle pulse); audio_gait_changed() called on transition complete.
2026-01-03  v0.10.6 b260: TTS: Added 2s delay after shutdown announcement to allow speech to complete before pygame/audio cleanup.
2026-01-03  v0.10.5 b259: TTS: Fixed audio device conflict - sox `play` can't access device when pygame mixer is active; now route espeak→sox (amplify)→pygame.mixer.Sound for playback; non-blocking background threads.
2026-01-03  v0.10.4 b258: TTS: Switched from pyttsx3 to espeak-ng + sox pipeline for louder output (+30dB gain via sox); simpler dependencies (no pyttsx3 needed); configurable gain_db in [tts] section.
2026-01-03  v0.10.3 b257: TTS: Hooked speech to low battery warning ("Battery low, X percent remaining"), obstacle detected (autonomy STOP trigger), cliff detected (E-STOP trigger); added speak_cliff() helper.
2026-01-03  v0.10.2 b256: TTS: Integrated pyttsx3 (espeak-ng backend) for voice announcements; speaks enable/disable, postures (stand/tuck), safety lockout, autonomy toggle, startup ("Mars online"), shutdown ("Mars shutting down"); configurable via [tts] section (enabled, rate, volume, voice, cooldown_sec).
2026-01-03  v0.10.1 b255: Audio: Added safety lockout alert (urgent 3-tone), stand/tuck/home posture confirmation sounds, Xbox controller connect/disconnect, autonomy mode toggle sounds.
2026-01-03  v0.10.0 b254: Audio: Created click.wav (25ms synthetic damped impulse), boosted click volume to 1.0 in SOUND_VOLUMES; wired audio_click() to all buttons including D-pad, Start, Back.
2026-01-03  v0.9.0 b253: Audio: Full event audio feedback - button clicks on all controller buttons (A/B/X/Y/LB/RB), enable/disable tones, low battery warning (3-beep alert), Teensy connect/disconnect sounds, gait start/stop chirps.
2026-01-03  v0.8.9 b252: Audio: Integrated audio_manager into controller.py - startup chime (3-tone ascending A4→C#5→E5), shutdown beep (descending E5→A4), config from [audio] section; audio manager initialized during startup, cleaned up on exit.
2026-01-03  v0.8.8 b251: Audio: Created audio_manager.py module - pygame.mixer based, sound pool, volume control, mute, beep generator; configured for Sabrent USB DAC (hw:2,0); [audio] section in controller.ini.
2026-01-03  v0.8.7 b250: Dashboard: Fixed config source - now uses MarsMenu.get_all_config() with full metadata (min/max/step/unit) instead of simple dict; periodic refresh every 2s; inputs show correct decimal precision from step size.
2026-01-02  v0.8.7 b249: Dashboard: Config editing now uses MarsMenu metadata (min/max/step/unit/options) for input constraints; chart x-axis fixed to show full time window; added dropdown support for option-type menu items; key aliasing supports both menu labels and internal keys.
2026-01-02  v0.8.7 b248: Dashboard W4: Overlay mode for charts - "Overlay All" tab shows voltage, loop time, and temperature on single chart with three Y-axes (left voltage, right loop time/temp); legend visible in overlay mode; interaction tooltip shows all values.
2026-01-02  v0.8.7 b247: Dashboard W4: Telemetry history charts - rolling graphs for voltage, loop time, and temperature using Chart.js; selectable 1/5/10 min windows; CSV export button; sample counter badge; tab-based chart switching.
2026-01-02  v0.8.6 b246: Dashboard: Fixed config section names (Gait/Safety/PID/Impedance/Estimator) to match EDITABLE_CONFIG keys; editable input fields now appear correctly for dashboard config editing.
2026-01-02  v0.8.6 b245: Dashboard: Fixed PID/IMP/EST config - defined missing variables (_pidEnabled, _pidKp, etc.), added initialization from config, pushed all 5 categories (Gait, Safety, PID, Impedance, Estimator) to dashboard; fixed save handlers to use correct firmware keys (triplets, milli values).
2026-01-02  v0.8.6 b244: Safety: Low battery filter_alpha now configurable via [low_battery] section and dashboard; smaller alpha = slower/smoother voltage filtering.
2026-01-01  v0.8.6 b243: Dashboard W3: Live config editing from web dashboard - editable Gait (cycle_ms, step_length, step_height, turn_rate), Safety (low_battery_enabled, volt_critical, volt_recovery), PID/IMP/EST parameters; changes persist to controller.ini; bidirectional sync with on-device menu.
2026-01-01  v0.8.5 b242: Dashboard: Fixed LCARS swept elbows with proper inner curve cutouts (::after pseudo-elements); added dynamic palette system syncing to user's System.Palette selection; fixed pointcloud_viewer to handle 'frame' message type and array-based points.
2026-01-01  v0.8.5 b241: Dashboard: Rebuilt LCARS per Manifesto - proper swept elbows (thick→thin frames), cap buttons, 3 font sizes, minimalist layout; simpler, more authentic TNG styling.
2026-01-01  v0.8.5 b240: Dashboard: LCARS theme for dashboard and point cloud viewer - authentic Star Trek TNG styling with distinctive color palette (orange/peach/tan/blue/purple), curved panels, sidebar navigation, stardate timestamps.
2026-01-01  v0.8.5 b239: Dashboard W2: Enhanced config view with all menu categories (12 tabs), collapsible sections, search/filter; MarsMenu.get_all_config() exports menu items to dashboard; config pushed every 2s.
2025-12-31  v0.8.4 b238: joy_controller fix: Send state to client even when Xbox disconnected so dashboard can update status.
2025-12-31  v0.8.4 b237: Dashboard fix: Safety state now uses safety_telem.lockout (S5) instead of system_telem.safety (S1), which was actually rr_index due to firmware format mismatch.
2025-12-31  v0.8.4 b236: Dashboard fix: Use ctrl.joyClient directly instead of stale global _joyClient reference for Xbox status.
2025-12-31  v0.8.4 b235: Dashboard fix: Xbox controller status now uses _joyClient.xbox_connected property instead of socket-only state.
2025-12-31  v0.8.4 b234: Dashboard fix: Rewrote phase_dashboard() to use ctrl.system_telem and get_imu_frame() instead of stale ctrl.state array; proper fallbacks for battery voltage and IMU data.
2025-12-31  v0.8.4 b233: Dashboard: Web-based telemetry dashboard (dashboard.html on port 8766); real-time telemetry streaming via WebSocket; read-only config view; Phase 1 of web configuration system.
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

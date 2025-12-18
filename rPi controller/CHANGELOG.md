# rPi Controller Change Log

All modifications specific to the Python controller (controller.py and related Python support code) are logged here. This file is independent from the root firmware CHANGELOG.

FORMAT: `YYYY-MM-DD  <summary>`

## Entries

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

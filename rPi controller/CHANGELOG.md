# rPi Controller Change Log

All modifications specific to the Python controller (controller.py and related Python support code) are logged here. This file is independent from the root firmware CHANGELOG.

FORMAT: `YYYY-MM-DD  <summary>`

## Entries

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

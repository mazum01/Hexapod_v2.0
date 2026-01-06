#----------------------------------------------------------------------------------------------------------------------
#    controller.py
#----------------------------------------------------------------------------------------------------------------------
# CHANGE LOG (Python Controller)
# Format: YYYY-MM-DD  Summary
# REMINDER: Update CONTROLLER_VERSION below with every behavioral change, feature addition, or bug fix!
# 2026-01-01  v0.8.6 b243: Dashboard W3: Live config editing from web dashboard - editable Gait, Safety, PID/IMP/EST params; persists to controller.ini; bidirectional sync with on-device menu.
# 2026-01-01  v0.8.5 b241: Dashboard: Rebuilt LCARS per Manifesto - proper swept elbows (thick→thin frames), cap buttons, 3 font sizes, minimalist layout; simpler, more authentic TNG styling.
# 2025-12-27  v0.7.38 b222: Display: LCARS palette syncs live when changed in menu; ToF distance moved to top of heatmap.
# 2025-12-27  v0.7.37 b221: Display: LCARS engineering view uses config palette (Classic/Nemesis/LowerDecks/PADD), bigger fonts, black text.
# 2025-12-27  v0.7.36 b220: Display: Added LCARS theme for engineering view; configurable via engineering_lcars in [display].
# 2025-12-27  v0.7.35 b219: Display: Added phone-style battery icon to EYES mode (top-right corner); configurable via show_battery_icon in [safety_display].
# 2025-12-26  v0.7.33 b217: Bugfix: Startup crash after ToF init - fixed undefined try_teensy_connect/_teensyPort/_teensyBaud; use connect_teensy with config values.
# 2025-12-26  v0.7.32 b216: Startup: Connect to Teensy, controller, and start telemetry during splash for seamless transition to eyes.
# 2025-12-26  v0.7.31 b215: Startup: Full Mars background on splash; configurable delay (0-30s, default 5s) via display.startup_delay_s.
# 2025-12-26  v0.7.30 b214: Startup: Add StartupSplash with scrolling log during initialization; shows Mars image and progress messages.
# 2025-12-26  v0.7.29 b213: Display: Fix voltage bar in 3D view; suppress DISABLED overlay in engineering mode.
# 2025-12-26  v0.7.28 b212: Display: Add body pitch/roll from IMU to 3D wireframe visualization.
# 2025-12-26  v0.7.27 b211: Display: Enable all 6 legs in 3D wireframe view.
# 2025-12-26  v0.7.26 b210: Display: Also flip tibia direction for left-side legs.
# 2025-12-26  v0.7.25 b209: Display: Fix LF base angle (+180°); flip femur direction for left-side legs.
# 2025-12-26  v0.7.24 b208: Display: Add LF leg to debug view (now LF+RF+RM+RR).
# 2025-12-26  v0.7.23 b207: Display: Fix RR base angle (315°→135°).
# 2025-12-26  v0.7.22 b206: Display: Add RR leg to debug view (now RF+RM+RR).
# 2025-12-26  v0.7.21 b205: Display: Fix middle leg base angles (+90°); re-enable RF+RM debug view.
# 2025-12-26  v0.7.20 b204: Display: Mirror X-axis; fix body polygon order; remove debug overlays.
# 2025-06-21  v0.7.19 b203: Display: FK coxa now includes leg base angle; DEBUG_LEGS supports multi-leg debug (RF+RM).
# 2025-12-18  v0.5.40 b152: UX: Use real Mars photo (ESA/Rosetta, CC BY-SA 3.0 IGO) for startup splash instead of a procedural drawing.
# 2025-12-17  v0.5.39 b151: UX: Replace LCD startup banner with a Mars splash image (black background) and overlay firmware/controller versions.
# 2025-12-17  v0.5.38 b150: Version/docs: Update startup banner firmware version string to match current firmware (0.2.41/b157).
# 2025-12-17  v0.5.37 b149: Telemetry: Added binary S4 parsing (type=4, 6 leg contact flags) to match new firmware S4 frames.
# 2025-12-16  v0.5.36 b148: Telemetry: Extended binary S1 parsing to include battery/current/IMU fields (when present) so INFO matches ASCII mode.
# 2025-12-16  v0.5.35 b147: Telemetry: Added binary framed telemetry support (TELEM BIN 1) while keeping Y 1/Y 0 as master enable.
# 2025-12-16  v0.5.34 b146: Telemetry perf: reduced parse-path allocations and gated debug I/O/copies.
# 2025-12-16  v0.5.33 b145: Motion/UI: Added Posture menu tuning params for Pounce, persisted to controller.ini; added launch shortcuts.
# 2025-12-16  v0.5.32 b143: Motion: Added a kinematic "Pounce" move (spider-like jump attack) runnable from Posture menu.
# 2025-12-16  v0.5.31 b142: Menu: Reordered tab stack (INFO/SYS first); show servo voltage/temp only on INFO.
# 2025-11-22  Added structured in-file change log block.
# 2025-11-22  Normalized Teensy command protocol: LF only, removed semicolons.
# 2025-11-22  A button toggles enable/disable (LEG ALL ENABLE + ENABLE / DISABLE).
# 2025-11-22  Added 'k' tuck key: enable sequence then TUCK; auto DISABLE after 5s.
# 2025-11-22  Added tuck auto-disable scheduler and gait status display adjustments.
# 2025-11-22  Introduced send_cmd helper (centralized emission, throttling, local enable tracking).
# 2025-11-22  (Revert) Confirmed enable flag source is S1 idx9; removed temporary SQ parsing.
# 2025-11-22  Added posture abstraction helper apply_posture() and refactored 'k'/'s' handlers.
# 2025-11-22  Renamed posture/gait 'PRONE' to 'HOME'.
# 2025-11-22  Added telemetry schema validation (length checks + warnings) for S1/S2/S3/S4.
# 2025-11-23  Telemetry management simplified: auto-enable with 'Y 1' after grace; disable with 'Y 0' on exit.
# 2025-11-23  Fix: Correct telemetry command spacing ('Y 1' / 'Y 0').
# 2025-11-23  Added telemetry debug (raw + parsed S1) toggled by 'd' key.
# 2025-11-23  Added send_cmd debug (suppressed + sent) and S2 raw/parsed capture (toggle 'd' for telemetry, 'g' for send path).
# 2025-11-24  Fixed command emission to use unified bytes; send_cmd now normalizes str/bytes; posture helper enforces bytes set. Version bump 0.1.8.
# 2025-11-24  Updated 'q' key to issue idle + LEG ALL DISABLE + DISABLE for full torque drop.
# 2025-11-24  Fix readTeensy newline normalization (proper CRLF/CR handling).
# 2025-11-24  Telemetry stall retry: if still silent after start, re-send 'Y 1' once after 2s.
# 2025-11-24  Teensy reconnect loop: auto-detect disconnects, backoff rescan, reset telemetry state.
# 2025-11-24  Font caching: cache PIL truetype fonts to avoid per-frame reloads.
# 2025-11-24  Controller refactor (phase 1b): add state mirroring for verbose/mirror/telemetry timers.
# 2025-11-24  Controller refactor (phase 1c): move telemetry auto-start/retry into Controller.housekeeping().
# 2025-11-24  Fix mirror/verbose toggle persistence (sync into Controller before global write-back).
# 2025-11-24  Controller refactor (phase 1d): migrated Teensy polling, gamepad event handling, and display update into Controller methods (no behavioral change).
# 2025-11-25  Controller refactor (phase 1e): completed migration of all global state into Controller class with serial config support from controller.ini.
# 2025-11-25  Critical bugfix: Added missing connect_teensy() and handle_teensy_disconnect() methods that were causing runtime crashes.
# 2025-11-25  Bugfix collection: Fixed menu display toggle, stand/tuck auto-disable scheduling, and state synchronization between globals and Controller.
# 2025-11-25  v0.1.21: Fixed power button application exit - added requestExit flag to Controller class, power button now properly terminates the application.
# 2025-11-25  v0.1.22: Enhanced power button detection - added multiple Xbox guide button event codes (139, 158, 172) and debug logging for unhandled button events.
# 2025-11-25  v0.1.23: Updated Xbox controller button mappings to modern Teensy firmware commands: Y=Test mode, X=Idle mode, A=Stand, B=Home, right joystick press=Disable. Removed obsolete steering mode.
# 2025-11-25  v0.1.24: Updated X button to toggle between TEST/IDLE modes and Y button to TUCK posture command with auto-disable.
# 2025-11-25  v0.1.25: Fixed X/Y button event codes - X button uses code 307, Y button checks multiple codes (306, 308, 309, 310) for compatibility.
# 2025-11-25  v0.1.26: Corrected Xbox controller button codes using evdev directional naming: X=BTN_WEST(308), Y=BTN_NORTH(307). Fixed X/Y button swapping issue.
# 2025-11-26  v0.2.0 b27: Session 2 performance improvements:
#             - Serial read batching: replaced byte-by-byte reads with batch read(in_waiting) + incomplete line buffering
#             - Loop timing drift correction: replaced fixed sleep with monotonic next-tick scheduling
#             - Keyboard input optimization: replaced per-loop curses.wrapper() with persistent stdscr + poll_keyboard()
#             - Removed X/Y button debug print statements
# 2025-11-26  v0.2.1 b28: Added monotonic CONTROLLER_BUILD number (never resets across version changes).
# 2025-11-26  v0.2.2 b29: Bugfix: Fixed spurious Teensy disconnect when no data waiting (readTeensy returning None was incorrectly treated as error).
# 2025-11-26  v0.2.3 b30: Telemetry-synchronized loop timing: derive loop period from Teensy S1 tick, fallback to 166Hz if telemetry stalls.
# 2025-11-26  v0.2.4 b31: Startup sends 'LEG ALL ENABLE' before 'Y 1' to ensure telemetry includes valid data from all servos.
# 2025-11-26  v0.2.5 b32: Keyboard commands now accept both upper and lowercase (t/T, s/S, k/K, etc.).
# 2025-11-26  v0.3.0 b33: Gait engine integration: new gait_engine.py module with TripodGait and StationaryPattern;
#             keyboard controls w/W=walk (tripod), h/H=halt, p/P=stationary pattern; FEET command generation at 166Hz.
# 2025-11-26  v0.3.1 b34: Gamepad gait controls: LB=toggle gait, RB=cycle gait type (tripod/stationary),
#             left stick=speed/heading, right stick X=turn rate, triggers=step length/lift height adjustments.
# 2025-11-27  v0.3.2 b35: Fixed gait heading/speed - use atan2 for combined stick input; speed_scale modulates stride;
#             TripodGait now applies heading_deg rotation and turn_rate_deg_s for yaw control.
# 2025-11-27  v0.3.3 b36: Fixed auto-disable firing during gait (clear _autoDisableAt on gait start);
#             added gait debug output showing heading/speed values.
# 2025-11-27  v0.3.4 b37: Stick neutral = step in place (gait cycles but no translation); forward/back = speed control.
# 2025-11-27  v0.3.5 b38: Fixed joystick normalization - range is 0-65535 with center at 32768, not signed int16.
# 2025-11-27  v0.3.6 b39: Eyelid control now always active on left stick Y (both gait on and off).
# 2025-11-27  v0.3.7 b40: EMA smoothing on gait parameters (speed, heading, turn) - prevents jumps on input changes.
# 2025-11-27  v0.3.8 b41: Rate-limit FEET commands to 83Hz (send every 2nd tick) to reduce serial buffer issues.
# 2025-11-27  v0.3.9 b42: Removed verbose prints and reduced eye updates during gait to prevent joystick event flooding.
# 2025-11-27  v0.3.10 b43: Increased EMA smoothing (alpha 0.08→0.04), FEET rate 83Hz→55Hz for smoother motion.
# 2025-11-27  v0.3.11 b44: Fixed corner leg rotation to include lateral offset (x_move) - enables strafe motion.
# 2025-11-27  v0.3.12 b45: Redesigned stick control: Y=speed (independent), X=heading offset (±90°). Allows slow/fast strafe.
# 2025-11-27  v0.3.13 b46: Simplified gait to forward/back only (Y stick). Matches Teensy TEST mode. Removed debug logging.
# 2025-11-27  v0.3.14 b47: Skip display/eye updates during gait to eliminate blink-induced jitter.
# 2025-11-28  v0.3.15 b48: Fix gait start: initialize speed_scale=0 so legs march in place until joystick moved.
# 2025-11-28  v0.3.16 b49: Strafe (crab walk): right stick X controls heading angle (-90° to +90°), stride rotated by sin/cos.
# 2025-11-28  v0.3.17 b50: Fix Xbox controller axis mapping: right stick X also comes on code 5 (>1023 = stick, <=1023 = trigger).
# 2025-11-29  v0.3.18 b51: Fix left trigger (code 2) also receiving Xbox joystick data - add same >1023 guard.
# 2025-11-29  v0.3.19 b52: Force MODE IDLE when starting Python gait (LB) to prevent Teensy TEST mode from overriding FEET commands.
# 2025-11-30  v0.3.20 b53: Update all print() statements to use end="\r\n" for proper terminal display.
# 2025-11-30  v0.3.21 b54: Fix middle leg strafe: LM/RM now step in body lateral direction with correct sign per side.
# 2025-12-03  v0.3.22 b55: Safe shutdown: send LEG ALL DISABLE + DISABLE on script exit before stopping telemetry.
# 2025-12-03  v0.3.23 b56: Bezier curves for swing phase: 5-point arc with smooth lift/touchdown, stance is linear.
# 2025-12-03  v0.3.24 b57: Tuned Bezier lift: 60mm base, 150% peak compensation for curve attenuation.
# 2025-12-03  v0.3.25 b58: Gait width adjustment: hold left stick button + left trigger to set width (60-140mm).
# 2025-12-03  v0.3.26 b59: Fix left trigger event code (10 not 2); add analog debug when left stick held.
# 2025-12-03  v0.3.27 b60: Gait width range expanded to 50-175mm.
# 2025-12-03  v0.3.28 b61: Lift height adjustment: hold left stick + right trigger to set lift (20-100mm).
# 2025-12-03  v0.3.29 b62: Persist gait width/lift settings to controller.ini [gait] section on left stick release.
# 2025-12-04  v0.3.30 b63: Centralize config: added [timing] and [display] ini sections; gait ranges now configurable.
# 2025-12-04  v0.3.31 b64: Complete config centralization: gait engine params, Bezier curve shape, eye settings, serial error threshold, command throttle, auto-disable all configurable.
# 2025-12-04  v0.4.0 b65: Display thread: background thread for eye animation and LCD updates at configurable Hz (default 15), decoupled from main loop/gait timing.
# 2025-12-04  v0.4.1 b66: Eye tracking: eyes look in direction of joystick input (heading_deg → X, speed → Y). Configurable look_range_x/y in [eyes] section.
# 2025-12-04  v0.4.2 b67: Quick wins: getGait() uses dict lookup; fixed _menuVisable → _menuVisible spelling.
# 2025-12-04  v0.4.3 b68: Modular loop refactor: split main loop into distinct phase functions for clarity and maintainability.
# 2025-12-04  v0.4.4 b69: Teensy disconnect overlay: displays red 'NO TEENSY' watermark when Teensy not connected.
# 2025-12-04  v0.4.5 b70: Bugfix: blink frame skip was checking wrong states (WAITING instead of CLOSED), causing eyes to not return to full height after blink.
# 2025-12-04  v0.4.6 b71: Controller disconnect overlay: displays red 'NO CONTROLLER' watermark (stacks with NO TEENSY if both missing).
# 2025-12-04  v0.4.7 b72: Enhanced status overlays: added 'NO TELEMETRY' (orange) when connected but no data, 'DISABLED' (yellow) when robot disabled. Immediate render on status change.
# 2025-12-04  v0.4.8 b73: Code review completed (TODO #1): documented duplication, cohesion, error handling, timing. See TODO.md "Code Review Recommendations" section.
# 2025-12-04  v0.4.9 b74: Code review fixes: Added ensure_enabled() helper, IDX_* telemetry constants, narrowed exception handlers to specific types with logging.
# 2025-12-04  v0.4.10 b75: Display optimization (TODO #19/#20): frame change detection via hash skips unchanged frames; combined rotation+RGB565 in show_image_rotated().
# 2025-12-04  v0.4.11 b77: Added WaveGait and RippleGait classes; RB cycles through Tripod→Wave→Ripple→Stationary. Removed STRAFE_DEBUG_LOG.
# 2025-12-04  v0.4.12 b78: Phase-locked gait transitions: RB now waits for phase boundary, then blends between gaits over 500ms for smooth switching.
# 2025-12-04  v0.4.13 b79: Walking turn: right stick Y controls yaw rate (±60 deg/s), differential stride per leg enables arc motion.
# 2025-12-06  v0.4.14 b80: Eye shape cycle: 'e' key cycles through all eye shapes; DPAD up/down also cycles shapes.
# 2025-12-06  v0.4.15 b81: Human eye improvements: percentage-based spacing (human_eye_spacing_pct config), randomized iris radial fibers for realism.
# 2025-12-06  v0.4.16 b82: Eye shape persistence: selected eye shape saved to controller.ini and restored on startup.
# 2025-12-06  v0.4.17 b83: Human eye blink fix: eyelid-style blink (polygon overlay) matching other eye types; honors eyelid_angle and eyelid_percent for joystick intensity control.
# 2025-12-06  v0.4.18 b84: Human eye intensity fix: corrected eyelid angle direction, intensity eyelid shows without blink.
# 2025-12-06  v0.4.19 b85: Human eye colors: 5 color options (blue, green, hazel, brown, dark brown); configurable iris size; pupil fades black→red at 50-100% intensity.
# 2025-12-06  v0.4.20 b86: Eye color palettes in config: all 6 eye shapes have configurable colors (color_ellipse, color_rectangle, etc.); human eye palette also configurable.
# 2025-12-06  v0.4.21 b87: UI fixes: DISABLED overlay moved to bottom of screen; fixed eyelid_percent calculation (0.755→75.0); DPAD cycles human eye color when on human eyes.
# 2025-12-06  v0.4.22 b88: Human eye tweaks: intensity eyelid max reduced to 45% (was ~50%); dark eye colors get lighter limbal ring for visibility against black background.
# 2025-12-06  v0.4.23 b89: Touch E-STOP: touching screen while robot is in motion stops gait, sends idle + LEG ALL DISABLE + DISABLE for immediate safety shutdown.
# 2025-12-06  v0.4.24 b90: New eye types: CAT (vertical slit pupils that dilate with intensity), HYPNO (rotating spiral, speed↑ with intensity), ANIME (large kawaii eyes with sparkles).
# 2025-12-06  v0.4.25 b91: Eye tweaks: CAT pupil starts wider, goes full round at max intensity; stable radial fiber seed. HYPNO uses black/color for deeper contrast.
# 2025-12-06  v0.4.26 b92: Eye tweaks: CAT pupil max size uses major axis (full tall circle). HYPNO background now subtle dark color (~20% of spiral color) instead of pure black.
# 2025-12-06  v0.4.27 b93: MARS menu system: new MarsMenu.py replaces legacy GUIMenu. Tabbed interface (Eyes/Gait/Posture/Info/System); DPAD navigates, LB/RB switch tabs, A selects, B closes, Start toggles. Safety gate: menu only opens when robot disabled & not in motion. Touch E-STOP still active.
# 2025-12-07  v0.4.28 b94: Menu UX: touch debouncing (single value change per touch), visual scroll bar for multi-page menus, larger fonts/touch targets, fixed touch coordinate rotation, immediate display refresh after input.
# 2025-12-07  v0.4.29 b95: Menu fixes: use _marsMenu.touched() for proper debouncing (was using old menu), wider scroll bar (20px track, 25px min thumb) for fat fingers.
# 2025-12-07  v0.4.30 b96: Menu touch fix: touched() now returns current state (not debounced), debouncing moved to handle_touch() for value adjustments only. Moved value arrows left to avoid scroll bar overlap.
# 2025-12-07  v0.4.31 b97: Functional scroll bar: Tap above/below thumb scrolls by page. Widened scroll bar to 30px for easier touch.
# 2025-12-07  v0.4.32 b98: Added X close button in top-right corner to close menu via touch.
# 2025-12-07  v0.4.33 b99: LCARS theme: Star Trek inspired menu theme with pill-shaped tabs, orange/peach/lavender palette. Switchable via System > Theme.
# 2025-12-07  v0.4.34 b100: LCARS improvements: Fixed L-bracket curve direction, smaller tabs to fit screen, 4 color palettes (Classic, Nemesis, LwrDecks, PADD).
# 2025-12-07  v0.4.35 b101: Menu/eye settings saved to config: Theme, Palette, Eye V Center. Added V Center to Eyes menu for vertical eye position.
# 2025-12-07  v0.4.36 b102: LCARS design guidelines applied: thick-to-thin frame transitions, proper swept corners, rounded caps, consistent spacing grid, 3 font sizes.
# 2025-12-07  v0.4.37 b103: LCARS fix: Tabs now start after left frame bar (x=14), vertical bar extends full height for continuous frame.
# 2025-12-07  v0.4.38 b104: LCARS frame: Added bottom swept corner, vertical bar now spans between top and bottom sweeps only.
# 2025-12-07  v0.4.39 b105: LCARS touch fix: Tab touch zones now match LCARS tab positions (start_y=32, height=22, gap=3).
# 2025-12-07  v0.4.40 b106: LCARS color variety: frame elements, tabs, items use distinct palette colors; first tap to wake display is now ignored.
# 2025-12-07  v0.4.41 b107: First-tap fix: properly tracks finger lift after wake before allowing subsequent touches to be processed.
# 2025-12-07  v0.4.42 b108: First-tap fix v2: simplified to counter-based approach - ignore touches until finger lifts after wake.
# 2025-12-07  v0.4.43 b109: First-tap fix v3: changed counter from 2 to 1 - only ignore the wake touch, not the next touch too.
# 2025-12-07  v0.5.0 b110: Touchscreen config menu complete (TODO #4): Auto-Disable timeout, Gait controls (Start/Stop, params), Posture buttons, INFO telemetry updates.
# 2025-12-07  v0.5.1 b111: Bugfix: gait callbacks now use _savedGaitLiftMm/_savedGaitWidthMm globals instead of non-existent _gaitLiftMm.
# 2025-12-08  v0.5.2 b112: Gait fix: reverse joystick Y now correctly drives reverse walking (backward motion instead of forward).
# 2025-12-08  v0.5.3 b113: Eye fix: eye intensity now responds symmetrically to both forward and reverse gait speed.
# 2025-12-08  v0.5.4 b114: Gait menu: Cycle Time now wired to gait params and saved config.
# 2025-12-08  v0.5.5 b115: Reason-aware auto-disable: tuck auto-disable no longer fires after subsequent posture/gait commands.
# 2025-12-08  v0.5.6 b116: System tab telemetry: added average servo temperature metric.
# 2025-12-08  v0.5.7 b117: Rebranded controller UI: mirror window title and LCD ASCII logo now use M.A.R.S. — Modular Autonomous Robotic System.
# 2025-12-08  v0.5.8 b118: Mirror window keyboard mapping now mirrors MarsMenu controls (tab/item/value/select/close) when menu visible.
# 2025-12-10  v0.5.9 b119: Wired Eyes tab V Center parameter into SimpleEyes vertical offset and display thread baseline; added persistent vertical_offset config key.
# 2025-12-10  v0.5.10 b120: Reduced Eyes menu Spacing step from 5% to 1% for finer horizontal eye spacing control.
# 2025-12-10  v0.5.11 b121: Reversed strafe mapping so joystick X left/right produces matching left/right crab-walk motion.
# 2025-12-11  v0.5.12 b122: Made walking turn gain configurable via MARS Gait menu and persisted it as [gait] turn_max_deg_s in controller.ini.
# 2025-12-12  v0.5.13 b123: Fixed right-stick turn handler NameError by basing eye intensity on tracked gait speed input instead of an undefined local variable.
# 2025-12-12  v0.5.14 b124: Restored left-stick Y gait speed mapping and eye intensity behavior (pupil dilation/red fade) while keeping right-stick Y for configurable turn rate.
# 2025-12-12  v0.5.15 b125: Added S5 safety telemetry parsing, hard safety lock handling (DISABLE + LEG ALL DISABLE), Safety tab in MARS menu, and full-screen safety overlay when firmware reports a lockout.
# 2025-12-12  v0.5.16 b126: Added tab pagination to MarsMenu so tabs render in fixed, touch-friendly sizes even as more categories are added; only the current page of tabs is shown at once, navigable via existing tab navigation controls. Added on-screen page indicator (e.g., "1/2") near the tab area when multiple pages exist.
# 2025-12-12  v0.5.17 b127: Moved the tab page indicator above the tab stacks so it never overlaps tab labels, and restyled the LCARS indicator as a small LCARS-style pill integrated into the frame.
# 2025-12-12  v0.5.18 b128: Extended the Safety tab with explicit actions to send SAFETY OVERRIDE commands (ALL, TEMP, COLLISION, NONE) to the firmware alongside SAFETY CLEAR, allowing safety causes to be selectively overridden from the on-screen menu.
# 2025-12-13  v0.5.19 b129: Fixed WaveGait foot phase distribution so the Wave gait translates forward instead of marching in place.
# 2025-12-13  v0.5.20 b130: Fixed RippleGait stance phase distribution so the Ripple gait translates forward instead of marching in place.
# 2025-12-13  v0.5.21 b131: Menu telemetry: render valid values (e.g., 0.00A) instead of '---'; fix servo temp stats to use S3 schema.
# 2025-12-13  v0.5.22 b132: Menu telemetry: battery voltage now displays whenever present (0.0V renders as 0.0V, not '---').
# 2025-12-13  v0.5.23 b133: Menu telemetry: when battery voltage is missing/0, show average servo bus voltage from S3.
# 2025-12-14  v0.5.24 b134: Menu: Added PID/IMP/EST controls that send firmware commands.
# 2025-12-15  v0.5.25 b135: Menu: Added dedicated PID/IMP/EST tabs with live LIST polling/parsing and per-field editing.
# 2025-12-15  v0.5.26 b136: PID/IMP/EST menu: Persist edits to controller.ini ([pid]/[imp]/[est]) so values restore on restart.
# 2025-12-15  v0.5.26 b137: Docs/cleanup: Updated USER_MANUAL revision/tabs; removed accidental controller-arachnotron file.
# 2025-12-15  v0.5.27 b138: Version/docs: Startup banner firmware version string updated to match current firmware (0.2.36/b152).
# 2025-12-16  v0.5.28 b139: Telemetry: Added lightweight structured telemetry containers (S1-S5) for clarity; parsers now populate both legacy lists and typed objects.
# 2025-12-16  v0.5.29 b140: Telemetry: Finished wiring structured telemetry into menu updates, enable gating, and display-thread robot_enabled checks.
# 2025-12-16  v0.5.30 b141: Telemetry: Display path now prefers structured telemetry for gait/disabled/contact; safety overlay helper prefers structured safety state.
# 2025-12-18  v0.5.41 b153: Modularization Phase 1: Extracted telemetry.py with dataclasses, IDX_* constants, processTelemS1-S5, and helpers.
# 2025-12-18  v0.5.42 b154: Modularization Phase 2: Extracted config_manager.py with save functions and config dataclasses; shared config state via set_config_state().
# 2025-06-18  v0.5.43 b155: Modularization Phase 3: Extracted posture.py with ensure_enabled, apply_posture, start_pounce_move (281 lines); wrapper pattern retains original API.
# 2025-06-18  v0.5.44 b156: Modularization Phase 4: Created mars_state.py with state container dataclasses (366 lines); foundation for global state consolidation.
# 2025-06-18  v0.5.45 b157: Modularization Phase 5: Extracted display_thread.py (520 lines) with DisplayThread, UpdateDisplay, getColor, drawLogo, drawMarsSplash. controller.py: 4422→3946 (-476 lines).
# 2025-12-18  v0.5.46 b158: Modularization Phase 6: Extracted input_handler.py (407 lines) with keyboard, gamepad, Teensy I/O, XboxButton/XboxAxis constants. controller.py: 3946→3838 (-108 lines).
# 2025-12-18  v0.5.47 b159: Modularization Phase 7: Final cleanup — removed duplicate get_font, consolidated imports. controller.py: 3838→3825 lines. Total reduction: 4844→3825 (~21%).
# 2025-12-18  v0.5.48 b160: Modularization complete — added __all__ exports to all modules, updated MODULARIZATION_PLAN.md.
# 2025-12-18  v0.5.49 b161: Architecture: Unified telemetry parser (binary/ASCII via telemetry.py); added is_robot_enabled property and ensure_enabled() method to Controller.
# 2025-12-19  v0.5.50 b162: Sensors: Added imu_sensor.py module with ImuThread for BNO085; integrated into controller with config loading and startup/shutdown.
# 2025-12-19  v0.5.51 b163: Display: Added real-time IMU gravity vector visualization overlay showing roll/pitch/yaw with graphical horizon indicator.
# 2025-12-19  v0.5.52 b164: Sensors: IMU data integration complete; uses I2C bus 3 via adafruit-extended-bus; horizon display reversed; overlay moved to bottom-left.
# 2025-12-19  v0.5.53 b165: Menu: Added IMU tab to MarsMenu with live orientation, status, read/error counts, I2C config, and overlay toggle.
# 2025-12-19  v0.5.54 b166: Leveling: Body leveling module (leveling.py) with IMU-based Z corrections; tilt safety threshold triggers TUCK+DISABLE; menu items in IMU tab.
# 2025-12-20  v0.6.0  b167: IMU Milestone: Complete sensor integration — driver, display overlay, menu tab, body leveling with tilt safety.
# 2025-12-20  v0.6.1  b168: Leveling: Fixed axis (Y not Z), added StandingGait for leveling tests, Stand uses gait engine, RB cycles through Standing→Tripod→Wave→Ripple→Stationary.
# 2025-12-20  v0.6.1  b170: Gait direction: Left stick Y now controls forward/backward direction (heading 0°/180°); strafe takes priority.
# 2025-12-21  v0.6.2  b171: ToF sensor module: Created tof_sensor.py with ToFThread, multi-sensor support, 8×8 distance arrays.
# 2025-12-21  v0.6.3  b172: ToF integration: Integrated tof_sensor.py into controller with [tof] config, thread startup/shutdown.
# 2025-12-21  v0.6.3  b173: ToF fix: Initialize sensors on main thread (VL53L5CX ctypes segfaults when init from thread); fix nested array data extraction.
# 2025-12-21  v0.6.4  b174: Display mode: Start button cycles EYES→ENGINEERING→MENU; engineering view shows IMU + ToF heatmap side by side.
# 2025-12-21  v0.6.5  b175: Display fix: Removed IMU overlay from EYES mode; engineering view now updates continuously (not just on IMU change).
# 2025-12-21  v0.6.6  b176: Menu: Added ToF tab with live sensor status, closest distance, temperature, read/error counts, I2C config.
# 2025-12-21  v0.6.7  b177: Menu: ToF tab now interactive — edit enabled/resolution/hz/bus, add/remove sensors, apply & restart; settings persist to config.
# 2025-12-22  v0.6.8  b178: Motion lean: Body tilts 7-10° into direction of travel when walking; configurable via IMU tab; works with or without leveling.
# 2025-12-22  v0.7.0  b179: Architecture: Split Xbox controller into separate joy_controller.py process; IPC via Unix socket; power button start/stop.
# 2025-12-22  v0.7.1  b180: Display: Xbox connection status now reflects actual controller state, not just socket link; joy_controller sends connected flag.
# 2025-12-22  v0.7.2  b183: Daemon compat: Handle no-TTY for curses (subprocess without terminal); fixes cbreak() ERR when started by joy_controller.
# 2025-12-23  v0.7.3  b184: Bugfix: poll_joy_client X/Y buttons swapped (now X=TUCK, Y=TEST/IDLE); added rstick DISABLE; fixed eye offset sign; stop gait on posture commands.
# 2025-12-23  v0.7.3  b185: Bugfix: Inverted left stick Y for gait speed (forward now moves forward).
# 2025-12-24  v0.7.6  b190: Eye blink: Reduced blink_percent_step from 0.25 to 0.08 for smoother, more natural blink animation without frame tearing.
# 2025-12-24  v0.7.7  b191: Display: Fixed eye flicker/tearing by adding 40ms min SPI write interval; removed IMU-triggered eye re-renders; increased blink frame divisor to 3.
# 2025-12-24  v0.7.8  b192: Blink: Made blink animation time-based (blink_speed=3.0/s) instead of frame-based; fixes blink speed varying with loop rate after Xbox controller split.
# 2025-12-24  v0.7.9  b193: Display: Fixed old-style eye flicker by excluding IMU changes from EYES mode render trigger; increased blink_speed to 8.0 for faster blinks.
# 2025-12-24  v0.7.10 b194: Display: Disabled CRT mode (scanlines caused flicker); reduced display thread to 10Hz; increased min SPI interval to 80ms.
# 2025-12-24  v0.7.11 b195: Eyes: Rewrote basic eye rendering (ELLIPSE, RECTANGLE, ROUNDRECTANGLE, X) to draw directly to display like special eyes; eliminates rotation/paste flicker.
# 2025-12-24  v0.7.12 b196: Eyes: Added rotation support to basic eyes using NEAREST resampling (no interpolation) to avoid flicker.
# 2025-12-25  v0.7.13 b197: Engineering: 3D wireframe hexapod view with FK-based leg positions; D-pad rotates view (L/R=azimuth, U/D=elevation); temp-colored segments; S6 joint telemetry wired to display.
# 2025-12-26  v0.7.14 b198: Bugfix: Engineering view now displays immediately when switching modes (added mode_changed detection to display thread).
# 2025-12-26  v0.7.15 b199: Bugfix: Fixed draw_hexapod_wireframe_3d() call - parameter names azimuth→azimuth_deg, elevation→elevation_deg.
# 2025-12-26  v0.7.16 b200: FK fix: Rewrote fk_leg_points() to match firmware convention - uses relative angles from 120° home, proper yaw/alpha/beta formulas. Increased 3D view zoom.
# 2025-12-26  v0.7.17 b201: FK debug: Isolated RF leg only, added angle labels and joint markers. More zoom (scale/200).
# 2025-12-26  v0.7.18 b202: FK fix: Added 90° offset to femur (home=horizontal), negated tibia rotation.
# 2025-12-27  v0.7.39 b223: Autonomy: Created behavior_engine.py + behaviors.py with obstacle avoidance, cliff, caught foot, patrol behaviors.
# 2025-12-27  v0.7.40 b224: Autonomy: Integrated arbiter into main loop; 'a' key toggles autonomy mode; config parsing for [behavior] section.
# 2025-12-27  v0.7.41 b225: Autonomy: Added AUTO menu tab with behavior toggles, thresholds, live status display, save settings action.
# 2025-12-27  v0.7.42 b226: Autonomy: Added Back+A gamepad combo to toggle autonomy mode; refactored toggle to _toggle_autonomy() helper.
# 2025-12-29  v0.7.43 b227: Autonomy: ABXY buttons auto-disable autonomy; added debug output for non-CONTINUE actions; fixed ToF attribute names.
# 2025-12-29  v0.7.44 b228: PointCloud: WebSocket server for 3D point cloud visualization (SLAM prototype); integrates ToF + IMU; configurable via [pointcloud] section.
# 2025-12-31  v0.8.0 b229: Architecture: Refactored main loop to Three-Layer Architecture (Firby/Gat); Layer 1=166Hz (timing, Teensy I/O, gait, safety), Layer 2=~33Hz (input, behaviors, display), Layer 3=~1Hz (future SLAM). Configurable via gait.layer2_divisor.
# 2025-12-31  v0.8.1 b230: Display: Battery voltage now uses average of all servo voltages with low-pass filter (alpha=0.1) for stable display during walking.
# 2025-12-31  v0.8.2 b231: Safety: Low battery protection - graceful shutdown (stop gait → TUCK → DISABLE) when voltage drops below critical threshold (default 10.0V); configurable via [low_battery] section; SAFETY menu items for enable/thresholds/status; recovery voltage hysteresis prevents servo current spikes from re-enabling.
# 2025-12-31  v0.8.3 b232: Input: Screen mirror toggle now works in socket mode (joy_controller); Back+Start combo toggles cv2 mirror window.
# 2025-12-31  v0.8.4 b233: Dashboard: Web-based telemetry dashboard (dashboard.html on port 8766); real-time telemetry streaming via WebSocket; read-only config view; Phase 1 of web configuration system.
# 2026-01-02  v0.8.6 b244: Safety: Low battery filter_alpha now configurable via [low_battery] section and dashboard; smaller alpha = slower/smoother voltage filtering.
# 2026-01-02  v0.8.6 b245: Dashboard: Fixed PID/IMP/EST config - defined missing variables, added initialization from _*_ini dicts, pushed all 5 config categories to dashboard; fixed save handlers to use correct firmware keys.
# 2026-01-02  v0.8.6 b246: Dashboard: Fixed config section names (Gait/Safety/PID vs gait/safety/pid) to match EDITABLE_CONFIG; editable inputs now appear correctly.
# 2026-01-02  v0.8.7 b247: Dashboard W4: Telemetry history charts - rolling voltage/loop-time/temperature graphs with Chart.js; 1/5/10 min windows; CSV export; sample counter badge.
# 2026-01-02  v0.8.7 b248: Dashboard W4: Overlay mode - "Overlay All" tab shows all 3 metrics on single chart with 3 Y-axes; legend and tooltip for overlay mode.
# 2026-01-02  v0.8.7 b249: Dashboard: Config editing now uses MarsMenu metadata (min/max/step/options); fixed chart x-axis to show full time window; added dropdown support for option items.
# 2026-01-03  v0.8.7 b250: Dashboard: Fixed config source - now uses MarsMenu.get_all_config() with full metadata (min/max/step/unit) instead of simple dict; periodic refresh every 2s; inputs show correct decimal precision.
# 2026-01-03  v0.8.8 b251: Audio: Created audio_manager.py - pygame.mixer, sound pool, volume control, beep generator; configured for Sabrent USB DAC (hw:2,0); [audio] section in controller.ini.
# 2026-01-03  v0.8.9 b252: Audio: Integrated audio_manager into controller.py - startup chime (3-tone ascending), shutdown beep, config from [audio] section.
# 2026-01-03  v0.9.0 b253: Audio: Added button click sounds, enable/disable tones, low battery warning, Teensy connect/disconnect, gait start/stop audio feedback.
# 2026-01-03  v0.10.0 b254: Audio: Created click.wav (25ms synthetic), boosted click volume to 1.0 for audibility; wired clicks to all buttons (D-pad, Start, Back).
# 2026-01-03  v0.10.1 b255: Audio: Added safety lockout alert, stand/tuck/home confirmation sounds, Xbox connect/disconnect, autonomy mode toggle sounds.
# 2026-01-03  v0.10.2 b256: TTS: Integrated pyttsx3 (espeak-ng backend) for voice announcements; speaks enable/disable, postures, safety lockout, autonomy, startup/shutdown; configurable via [tts] section.
# 2026-01-03  v0.10.3 b257: TTS: Hooked speech to low battery warning (with percent), obstacle detected (autonomy STOP), cliff detected (E-STOP); added speak_cliff() helper.
# 2026-01-03  v0.10.4 b258: TTS: Switched from pyttsx3 to espeak-ng + sox pipeline for louder output; configurable gain_db (default 30dB); simplified dependencies.
# 2026-01-03  v0.10.5 b259: TTS: Fixed audio device conflict - route TTS through pygame mixer (espeak→sox→pygame) instead of sox play; non-blocking background threads.
# 2026-01-03  v0.10.6 b260: TTS: Added 2s delay after shutdown announcement to allow speech to complete before cleanup.
# 2026-01-03  v0.10.7 b261: Audio: Added gait type change confirmation tones - distinct audio signature per gait (Standing, Tripod, Wave, Ripple, Stationary); played on transition complete.
# 2026-01-03  v0.10.8 b262: Audio: AU4 menu navigation clicks (nav/tab/adjust/select events via MarsMenu audio callback); AU6 generated 22 WAV sound assets (generate_sounds.py).
# 2026-01-03  v0.10.9 b263: TTS: Added Piper neural TTS as alternative engine; engine=piper|espeak in config; natural-sounding voices (en_US-lessac-medium default).
# 2026-01-03  v0.10.10 b264: TTS: Pre-cached TTS phrases as WAV files for instant playback; eliminates Piper ~300ms latency for fixed announcements. 15 cached phrases via generate_tts_phrases.py.
# 2026-01-04  v0.10.11 b265: Audio: Sound queue for sequential playback; sounds no longer overlap/cut off. Short UI sounds (clicks) bypass queue; priority sounds can interrupt.
# 2026-01-04  v0.10.12 b266: ToF: Added temporal EMA filter for smooth distance readings. Filters jitter, rejects high-sigma noise, attenuates outliers. Configurable alpha/thresholds in [tof] section.
# 2026-01-05  v0.10.13 b267: ToF: Changed filter to mode-based: 'off' (raw/SLAM), 'light' (reject bad only, no lag), 'full' (EMA smooth). Default 'light' for motion-friendly filtering.
# 2026-01-05  v0.10.14 b268: Autonomy A4+A5: Wall Following behavior (PD control to maintain target distance from left/right wall); Patrol touch-stop (tap screen to stop patrol gracefully).
# 2026-01-05  v0.10.15 b269: Menu: Wall Follow behavior in AUTONOMY tab (enable/side/distance); LCD notification overlay when behaviors toggled (auto-dismisses after 2s).
# 2026-01-05  v0.10.16 b270: Help: Press '?' to print keyboard command reference to console.
# 2026-01-06  v0.10.17 b271: Curses: Fixed all print() to use end="\r\n"; suppressed pygame/I2C warnings; fixed async server shutdown.
#----------------------------------------------------------------------------------------------------------------------
# Controller semantic version (bump on behavior-affecting changes)
CONTROLLER_VERSION = "0.10.17"
# Monotonic build number (never resets across minor/major version changes; increment every code edit)
CONTROLLER_BUILD = 271
#----------------------------------------------------------------------------------------------------------------------

# Firmware version string for UI/banner display.
# Keep this in sync with the Teensy firmware you have flashed.
FW_VERSION_BANNER = "0.2.42/b158"
# Telemetry index constants moved to telemetry.py module
#----------------------------------------------------------------------------------------------------------------------
#    Import all required libraries
#----------------------------------------------------------------------------------------------------------------------
import sys  
import os
import signal
import configparser
import time
import struct
import curses
import warnings
import numpy as np
import threading
from dataclasses import dataclass
from queue import Queue, Empty

# Suppress known harmless warnings from I2C libraries
warnings.filterwarnings("ignore", message="I2C frequency is not settable")
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka")
import serial
import serial.tools.list_ports
import cv2 as cv
#import operator
#import random

# use evdev to connect to the gamepad and read the gamepad events
from evdev import InputDevice, ff, ecodes, list_devices
import XBoxController

# import the simpleeyes library
from SimpleEyes import SimpleEyes as SimpleEyes, EYE_SHAPE, BLINKSTATE

# import the menu library
from Menu import TextMenuImage
from GUIMenu import GUIMenu
from MarsMenu import MarsMenu, MenuCategory, MenuItem

#import the steering mode enum
from SteeringMode import SteeringMode

# import the gait engine
from gait_engine import (GaitEngine, TripodGait, WaveGait, RippleGait, 
                         StationaryPattern, StandingGait, GaitParams, GaitTransition)

# import telemetry module (Phase 1 modularization)
from telemetry import (
    IDX_LOOP_US, IDX_BATTERY_V, IDX_CURRENT_A, IDX_PITCH_DEG,
    IDX_ROLL_DEG, IDX_YAW_DEG, IDX_GAIT, IDX_MODE, IDX_SAFETY, IDX_ROBOT_ENABLED,
    TELEM_SYNC,
    SystemTelemetry, ServoTelemetry, LegTelemetry, SafetyTelemetry, JointTelemetry,
    BinaryS1Result, BinaryS5Result, TelemetryFrame,
    create_servo_telemetry_array, create_leg_telemetry_array,
    processTelemS1, processTelemS2, processTelemS3, processTelemS4, processTelemS5,
    processTelemS6,
    parseBinaryS1, parseBinaryS2, parseBinaryS3, parseBinaryS4, parseBinaryS5,
    parseBinaryS6,
    applyBinaryS1ToState, applyBinaryS5ToState, decodeBinaryFrame,
    get_safety_overlay_text, getGait, GAIT_NAMES,
)

# import config_manager module (Phase 2 modularization)
from config_manager import (
    set_config_state,
    save_gait_settings, save_pounce_settings, save_eye_shape,
    save_human_eye_settings, save_eye_center_offset, save_eye_vertical_offset,
    save_eye_crt_mode, save_menu_settings, save_pid_settings, save_imp_settings, save_est_settings,
    save_tof_settings, save_safety_display_settings, save_low_battery_settings,
)

# import posture module (Phase 3 modularization)
import posture as posture_module

# import audio module (Phase 5 modularization)
from audio_manager import AudioManager, AudioConfig, PYGAME_AVAILABLE

# import mars_state module (Phase 4 modularization)
# State container dataclasses for future global state consolidation
from mars_state import (
    MarsControllerState, LoopTimingState, TeensyState, GaitState, MoveState,
    DisplayState, EyeSettings, PounceSettings, MenuSettings, SafetyState,
    AutoDisableState, DebugState, TelemetryArrays,
    create_default_state, create_gait_state_from_config,
    create_eye_settings_from_config, create_pounce_settings_from_config,
)

# import display_thread module (Phase 5 modularization)
from display_thread import (
    DisplayThread, UpdateDisplay as UpdateDisplayModule,
    DisplayMode, getColor, drawLogo, drawMarsSplash, StartupSplash,
    draw_engineering_view, draw_tof_heatmap, get_tof_color,
    get_font, reset_frame_hash,
    POWER_COLOR_PALETTE, TEMPERATURE_COLOR_PALETTE,
    show_mirror_window,
)

# import input_handler module (Phase 6 modularization)
from input_handler import (
    init_keyboard, cleanup_keyboard, poll_keyboard, getkey,
    find_gamepad, testForGamePad,
    find_teensy as find_teensy_module,
    read_teensy as read_teensy_module,
    read_teensy_bytes as read_teensy_bytes_module,
    reset_teensy_buffer,
    XboxButton, XboxAxis,
    normalize_joystick, normalize_trigger,
    JOYSTICK_CENTER, JOYSTICK_RANGE, TRIGGER_MAX, JOYSTICK_MIN_FOR_STICK,
)

# import joy_client module (socket-based Xbox controller IPC)
from joy_client import JoyClient, JoyState

# import imu_sensor module (sensor integration)
from imu_sensor import ImuThread, ImuConfig, ImuFrame, create_imu_thread

# import leveling module (body leveling based on IMU)
import leveling as leveling_module
from leveling import LevelingConfig, LevelingState, init_leveling, get_leveling_state

# import ToF sensor module (VL53L5CX)
from tof_sensor import ToFThread, ToFConfig, ToFSensorConfig, ToFFrame, create_tof_thread

# import behavior engine (autonomy)
from behavior_engine import BehaviorArbiter, Action, ActionRequest
from behaviors import ObstacleAvoidance, CliffDetection, CaughtFootRecovery, Patrol, WallFollowing

# import point cloud server (SLAM prototype)
try:
    from pointcloud_server import PointCloudServer, PointCloudConfig
    POINTCLOUD_AVAILABLE = True
except ImportError as e:
    POINTCLOUD_AVAILABLE = False
    print(f"PointCloud not available: {e}")

# import telemetry server (web dashboard)
try:
    from telemetry_server import TelemetryServer, TelemetryServerConfig
    TELEMETRY_SERVER_AVAILABLE = True
except ImportError as e:
    TELEMETRY_SERVER_AVAILABLE = False
    print(f"TelemetryServer not available: {e}")

# Gait type cycling order (RB button) - Standing first for leveling tests
GAIT_TYPES = [StandingGait, TripodGait, WaveGait, RippleGait, StationaryPattern]
GAIT_NAMES = ["Standing", "Tripod", "Wave", "Ripple", "Stationary"]

# import the LCD library and related graphics libraries
# sys.path.append("..")
# sys.path.append("./")
#from lib import LCD_1inch9
import st7789
import cst816d
from PIL import Image, ImageDraw, ImageFont
import spidev as SPI

# Telemetry dataclasses (SystemTelemetry, ServoTelemetry, LegTelemetry, SafetyTelemetry)
# are now imported from telemetry.py module

#----------------------------------------------------------------------------------------------------------------------
#
#    Define all helper functions here
#
#----------------------------------------------------------------------------------------------------------------------
# Note: get_font is now imported from display_thread.py (Phase 5 modularization)


class Controller:
    """Lightweight wrapper to encapsulate controller state and helpers.
    Migrated from global state for better organization and testability.
    """
    def __init__(self,
                 disp=None,
                 menu=None,
                 eyes=None,
                 touch=None):
        self.disp = disp
        self.menu = menu
        self.eyes = eyes
        self.touch = touch
        # Main state mirrors
        self.state = None
        self.servo = None
        self.legs = None
        self.verbose = True
        self.mirror = False
        # Teensy connection state
        self.teensy = None
        self.teensyErrorCount = 0
        self.nextTeensyScanAt = 0.0
        self.teensyReconnectBackoffSeconds = 1.5
        # Telemetry state
        self.lastTelemetryTime = None
        self.telemetryGraceDeadline = None
        self.telemetryStartedByScript = False
        self.telemetryRetryDeadline = None
        self.telemetryRetryCount = 0
        self.telemetryGraceSeconds = 0.75
        self.telemetryRetrySeconds = 2.0
        # Controller/gamepad state
        self.controller = None  # Legacy evdev InputDevice (fallback mode)
        self.joyClient: JoyClient = None  # Socket client for joy_controller daemon
        self.useJoySocket = True  # When True, prefer socket-based IPC over direct evdev
        self.retryCount = 0
        self._prevJoyState: JoyState = None  # Previous joy state for edge detection
        # Loop timing
        self.loopStartTime = time.time()
        self.loopdt = 0.0
        self.screenRefreshms = 100.0
        # Display state
        self.forceDisplayUpdate = False
        self.menuState = None
        self.menuVisible = False
        self.steeringMode = SteeringMode.OMNI
        # Auto-disable scheduling
        self.autoDisableAt = None
        self.autoDisableReason = None  # e.g., "TUCK", "STAND", etc.
        self.autoDisableGen = 0       # generation counter to detect stale timers
        # Debug telemetry
        self.lastRawS1 = ''
        self.lastParsedS1 = []
        self.lastRawS2 = ''
        self.lastParsedS2 = []
        # Exit request flag for power button
        self.requestExit = False
        # Mode tracking for X button toggle
        self._lastMode = 'IDLE'
        # Telemetry-synchronized timing
        self.teensyLoopUs = 6024  # Teensy loop period in microseconds (default 166Hz)
        self.lastS1MonoTime = None  # monotonic timestamp of last S1 receipt
        self.telemSyncActive = False  # True when actively syncing to Teensy telemetry

        # Binary telemetry (optional): firmware frames start with 0xA5 0x5A
        self.telemBinDesired = _preferBinaryTelemetry
        self.telemBinActive = False
        self._telem_rx_buf = b""
        self._telem_ascii_buf = ""
        # Gait control inputs (analog sticks)
        self._gaitStrafeInput = 0.0  # Left stick X: -1 (left) to +1 (right)
        self._gaitSpeedInput = 0.0   # Left stick Y: -1 (back) to +1 (forward)
        # Gait parameter adjustment (left stick button + triggers)
        self._leftStickHeld = False  # True when left stick button is held
        self._backHeld = False       # True when back button is held (for pounce combo)
        self._gaitWidthMm = _savedGaitWidthMm   # Current gait width (lateral offset from body center)
        self._gaitLiftMm = _savedGaitLiftMm     # Current lift height during swing phase

        # Structured telemetry mirrors (in addition to legacy list-of-lists state)
        self.system_telem = SystemTelemetry()
        self.servo_telem = [ServoTelemetry() for _ in range(18)]
        self.leg_telem = [LegTelemetry() for _ in range(6)]
        self.safety_telem = SafetyTelemetry()
        self.joint_telem = JointTelemetry()  # S6: 18 joint positions in degrees

    @property
    def is_robot_enabled(self) -> bool:
        """Check if robot is currently enabled (replaces magic index 9 checks).
        
        Prefers structured telemetry when valid; falls back to legacy state array.
        """
        if self.system_telem.valid:
            return self.system_telem.robot_enabled
        if self.state and len(self.state) > IDX_ROBOT_ENABLED:
            return self.state[IDX_ROBOT_ENABLED] == 1.0
        return False

    def ensure_enabled(self) -> bool:
        """Send enable commands if robot is not currently enabled.
        
        Consolidates the repeated pattern of:
            send_cmd(b'LEG ALL ENABLE', force=True)
            send_cmd(b'ENABLE', force=True)
            
        Returns:
            True if enable commands were sent, False if already enabled.
        """
        if self.is_robot_enabled:
            return False
        try:
            send_cmd(b'LEG ALL ENABLE', force=True)
            send_cmd(b'ENABLE', force=True)
            return True
        except Exception:
            return False

    @classmethod
    def from_current_globals(cls):
        try:
            inst = cls(disp=_disp, menu=_menu, eyes=_eyes, touch=_touch)
            # Mirror all current global state
            inst.state = _state
            inst.servo = _servo
            inst.legs = _legs
            inst.verbose = _verbose
            inst.mirror = _mirrorDisplay
            inst.teensy = _teensy
            inst.teensyErrorCount = _teensyErrorCount
            inst.nextTeensyScanAt = _nextTeensyScanAt
            inst.lastTelemetryTime = _lastTelemetryTime
            inst.telemetryGraceDeadline = _telemetryGraceDeadline
            inst.telemetryStartedByScript = _telemetryStartedByScript
            inst.telemetryRetryDeadline = _telemetryRetryDeadline
            inst.telemetryRetryCount = _telemetryRetryCount
            inst.telemetryGraceSeconds = _telemetryGraceSeconds
            inst.telemetryRetrySeconds = _telemetryRetrySeconds
            inst.controller = _controller
            inst.retryCount = _retryCount
            inst.loopStartTime = _loopStartTime
            inst.loopdt = _loopdt
            inst.screenRefreshms = _screenRefreshms
            inst.forceDisplayUpdate = _forceDisplayUpdate
            inst.menuState = _menuState
            inst.menuVisible = _menuVisible
            inst.steeringMode = _steeringMode
            inst.autoDisableAt = _autoDisableAt
            inst.lastRawS1 = _lastRawS1
            inst.lastParsedS1 = _lastParsedS1
            inst.lastRawS2 = _lastRawS2
            inst.lastParsedS2 = _lastParsedS2
            inst.teensyLoopUs = _teensyLoopUs
            inst.lastS1MonoTime = _lastS1MonoTime
            inst.telemSyncActive = _telemSyncActive
            return inst
        except (NameError, AttributeError) as e:
            # Globals not yet defined during early init; fallback to empty
            return cls()

    def connect_teensy(self, port_override=None, baud_override=None):
        """Attempt to connect to Teensy with optional port/baud override from config."""
        now = time.time()
        if now < self.nextTeensyScanAt:
            return False
        
        # Use override from config if specified, otherwise auto-detect
        if port_override and port_override.strip():
            teensyPath = type('MockPath', (), {'device': port_override.strip()})()
        else:
            teensyPath = find_teensy(self.verbose)
        
        if teensyPath is not None:
            try:
                baud = baud_override if baud_override else 1000000
                self.teensy = serial.Serial(teensyPath.device, baudrate=baud, timeout=1)
                self.teensyErrorCount = 0
                # Reset telemetry state on fresh connection
                self.lastTelemetryTime = None
                self.telemetryGraceDeadline = None
                self.telemetryStartedByScript = False
                self.telemetryRetryCount = 0
                if self.verbose:
                    print(f"Connected to Teensy on {teensyPath.device} at {baud} baud", end="\r\n")
                audio_teensy_connect()
                return True
            except Exception as e:
                self.teensy = None
                self.nextTeensyScanAt = now + self.teensyReconnectBackoffSeconds
                if self.verbose:
                    print(f"Error opening Teensy port: {e}; retry after backoff", end="\r\n")
        else:
            self.nextTeensyScanAt = now + self.teensyReconnectBackoffSeconds
        return False

    def handle_teensy_disconnect(self):
        """Handle Teensy disconnection and schedule reconnect."""
        global _teensy_read_buffer
        audio_teensy_disconnect()  # Play disconnect sound
        try:
            if self.teensy:
                self.teensy.close()
        except Exception:
            pass
        self.teensy = None
        self.nextTeensyScanAt = time.time() + self.teensyReconnectBackoffSeconds
        # Reset telemetry state so auto-start logic re-engages
        self.lastTelemetryTime = None
        self.telemetryGraceDeadline = None
        self.telemetryStartedByScript = False
        self.telemetryRetryCount = 0
        # Clear serial read buffer to prevent stale data on reconnect
        _teensy_read_buffer = ""
        if self.verbose:
            print("Teensy appears disconnected; scheduling rescan.", end="\r\n")

    def update_display(self):
        """Perform eyes refresh + display update, honoring force flag.
        Uses instance state instead of globals."""
        try:
            if self.eyes.update() or self.forceDisplayUpdate:
                telemetry_stale = (self.teensy is not None and self.lastTelemetryTime is None)
                if self.system_telem.valid:
                    robot_enabled = self.system_telem.robot_enabled
                else:
                    robot_enabled = (self.state[IDX_ROBOT_ENABLED] == 1.0) if (self.state and len(self.state) > IDX_ROBOT_ENABLED) else True
                safety_active, safety_text = get_safety_overlay_state()
                # Controller connected status (socket mode or legacy)
                if self.useJoySocket:
                    ctrl_connected = (self.joyClient is not None and self.joyClient.xbox_connected)
                else:
                    ctrl_connected = (self.controller is not None)
                UpdateDisplay(self.disp, self.eyes.display_image, self.menu._image, 
                              self.servo, self.legs, self.state, self.mirror, self.menuState,
                              teensy_connected=(self.teensy is not None),
                              controller_connected=ctrl_connected,
                              telemetry_stale=telemetry_stale,
                              robot_enabled=robot_enabled,
                              safety_active=safety_active,
                              safety_text=safety_text)
                self.forceDisplayUpdate = False
                return True
        except (AttributeError, TypeError) as e:
            # Display/eye object not ready or state mismatch - log if verbose
            if self.verbose:
                print(f"Display update error: {e}", end="\r\n")
        return False

    def poll_teensy(self):
        """Read and parse telemetry from Teensy, updating instance state.
        Returns raw data string (may contain multiple lines) or None."""
        global _safety_state, _last_safety_lockout, _gaitActive, _gaitEngine
        if self.teensy is None:
            return None
        teensyData = None
        hadError = False
        try:
            if self.telemBinDesired or self.telemBinActive:
                teensyData = readTeensyBytes(self.teensy, self.verbose)
            else:
                teensyData = readTeensy(self.teensy, self.verbose)
        except Exception as e:
            if self.verbose:
                print(f"Read exception: {e}", end="\r\n")
            teensyData = None
            hadError = True  # Only count actual exceptions as errors
        
        # Only increment error count on actual read failures, not on "no data available"
        if hadError:
            self.teensyErrorCount += 1
            if self.teensyErrorCount >= _teensyErrorThreshold:
                self.handle_teensy_disconnect()
            return None
        elif teensyData is not None:
            self.teensyErrorCount = 0  # Reset on successful data read
        
        # Skip parsing if no data available (normal case, not an error)
        if teensyData is None:
            return None

        # Binary framed telemetry path
        if isinstance(teensyData, (bytes, bytearray)):
            try:
                now = time.time()
                self._telem_rx_buf += bytes(teensyData)
                ascii_out = bytearray()

                # Demux: remove valid binary frames; everything else is treated as ASCII (command replies / LIST lines)
                while True:
                    buf = self._telem_rx_buf
                    if not buf:
                        break
                    i = buf.find(_TELEM_SYNC)
                    if i == -1:
                        # Keep last byte if it could be the start of a split sync sequence
                        if buf.endswith(_TELEM_SYNC[:1]):
                            ascii_out += buf[:-1]
                            self._telem_rx_buf = buf[-1:]
                        else:
                            ascii_out += buf
                            self._telem_rx_buf = b""
                        break
                    if i > 0:
                        ascii_out += buf[:i]
                        self._telem_rx_buf = buf[i:]
                        continue

                    # i == 0: attempt to parse a frame
                    if len(buf) < 7:
                        break
                    ver = buf[2]
                    telem_type = buf[3]
                    seq = buf[4]
                    ln = buf[5]
                    total = 6 + ln + 1
                    if len(buf) < total:
                        break
                    payload = buf[6:6 + ln]
                    cksum = buf[6 + ln]
                    calc = (ver ^ telem_type ^ seq ^ ln) & 0xFF
                    for b in payload:
                        calc ^= b
                    if calc != cksum:
                        # Bad checksum: drop one byte to resync
                        ascii_out += buf[:1]
                        self._telem_rx_buf = buf[1:]
                        continue

                    # Valid frame - use unified binary parsers from telemetry module
                    if ver == 1:
                        if telem_type == 1 and ln in (7, 17):
                            # Parse S1 using unified parser
                            s1_result = parseBinaryS1(payload, ln)
                            if s1_result is not None and s1_result.valid:
                                applyBinaryS1ToState(s1_result, self.state, self.system_telem)
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.lastS1MonoTime = time.monotonic()
                                if 1000 <= s1_result.loop_us <= 50000:
                                    self.teensyLoopUs = s1_result.loop_us
                                    self.telemSyncActive = True
                                self.telemBinActive = True

                        elif telem_type == 2 and ln == 18:
                            # Parse S2 using unified parser
                            if parseBinaryS2(payload, ln, self.servo, self.servo_telem):
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.telemBinActive = True

                        elif telem_type == 3 and ln == 54:
                            # Parse S3 using unified parser
                            if parseBinaryS3(payload, ln, self.servo, self.servo_telem):
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.telemBinActive = True

                        elif telem_type == 4 and ln == 6:
                            # Parse S4 using unified parser
                            if parseBinaryS4(payload, ln, self.legs, self.leg_telem):
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.telemBinActive = True

                        elif telem_type == 5 and ln == 11:
                            # Parse S5 using unified parser
                            prev_lockout = _safety_state.get("lockout", False)
                            s5_result = parseBinaryS5(payload, ln)
                            if s5_result is not None and s5_result.valid:
                                applyBinaryS5ToState(s5_result, _safety_state, self.safety_telem)
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.telemBinActive = True

                                # Handle lockout transition
                                if s5_result.lockout and not prev_lockout:
                                    if _gaitActive and _gaitEngine is not None:
                                        _gaitEngine.stop()
                                    _gaitActive = False
                                    try:
                                        send_cmd(b'LEG ALL DISABLE', force=True)
                                        send_cmd(b'DISABLE', force=True)
                                    except Exception:
                                        pass
                                    if self.verbose:
                                        print("Firmware reported SAFETY LOCKOUT; issued LEG ALL DISABLE + DISABLE.", end="\r\n")
                                _last_safety_lockout = s5_result.lockout

                        elif telem_type == 6 and ln == 36:
                            # Parse S6 joint positions using unified parser
                            angles = parseBinaryS6(payload, ln, self.joint_telem)
                            if angles is not None:
                                self.lastTelemetryTime = now
                                self.telemetryRetryDeadline = None
                                self.telemBinActive = True
                                # Update display thread with joint angles for 3D view
                                if _displayThread is not None:
                                    _displayThread.update_joint_angles(angles)

                    # Consume frame
                    self._telem_rx_buf = buf[total:]

                # Process any ASCII recovered from between frames (command replies / LIST lines)
                if ascii_out:
                    try:
                        chunk = ascii_out.decode('utf-8', errors='ignore')
                    except Exception:
                        chunk = ""
                    if chunk:
                        chunk = chunk.replace('\r\n', '\n').replace('\r', '\n')
                        self._telem_ascii_buf += chunk
                        lines = self._telem_ascii_buf.split('\n')
                        if not self._telem_ascii_buf.endswith('\n'):
                            self._telem_ascii_buf = lines[-1]
                            lines = lines[:-1]
                        else:
                            self._telem_ascii_buf = ""

                        for line in lines:
                            line = line.strip()
                            if not line:
                                continue
                            if line.startswith('PID '):
                                _parse_pid_list_line(line)
                                continue
                            if line.startswith('IMP '):
                                _parse_imp_list_line(line)
                                continue
                            if line.startswith('EST '):
                                _parse_est_list_line(line)
                                continue
                            # Fallback: accept ASCII telemetry even while in binary mode
                            segments = line.split('|')
                            for segment in segments:
                                if not segment:
                                    continue
                                header = segment[:3]
                                payload = segment[3:]
                                elements = payload.split(',') if payload else []
                                if header == 'S1:':
                                    processTelemS1(elements, self.state, self.system_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                elif header == 'S2:':
                                    processTelemS2(elements, self.servo, self.servo_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                elif header == 'S3:':
                                    processTelemS3(elements, self.servo, self.servo_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                elif header == 'S4:':
                                    processTelemS4(elements, self.legs, self.leg_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                elif header == 'S5:':
                                    prev_lockout = _safety_state.get("lockout", False)
                                    processTelemS5(elements, _safety_state, self.safety_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                    lockout = _safety_state.get("lockout", False)
                                    if lockout and not prev_lockout:
                                        if _gaitActive and _gaitEngine is not None:
                                            _gaitEngine.stop()
                                        _gaitActive = False
                                        try:
                                            send_cmd(b'LEG ALL DISABLE', force=True)
                                            send_cmd(b'DISABLE', force=True)
                                        except Exception:
                                            pass
                                        if self.verbose:
                                            print("Firmware reported SAFETY LOCKOUT; issued LEG ALL DISABLE + DISABLE.", end="\r\n")
                                    _last_safety_lockout = lockout
                                elif header == 'S6:':
                                    processTelemS6(elements, self.joint_telem)
                                    self.lastTelemetryTime = now
                                    self.telemetryRetryDeadline = None
                                    # Update display thread with joint angles for 3D view
                                    if _displayThread is not None and self.joint_telem.angles_deg:
                                        _displayThread.update_joint_angles(self.joint_telem.angles_deg)

                return teensyData
            except Exception as e:
                if self.verbose:
                    print(f"Binary telemetry parse error: {e}", end="\r\n")
                return teensyData
        
        try:
            dataLines = teensyData.splitlines()
            for line in dataLines:
                line = line.strip()
                if not line:
                    continue
                # Handle non-telemetry single-line responses (LIST outputs)
                if line.startswith('PID '):
                    _parse_pid_list_line(line)
                    continue
                if line.startswith('IMP '):
                    _parse_imp_list_line(line)
                    continue
                if line.startswith('EST '):
                    _parse_est_list_line(line)
                    continue
                now = time.time()
                segments = line.split('|')
                for segment in segments:
                    if not segment:
                        continue
                    header = segment[:3]
                    payload = segment[3:]
                    elements = payload.split(',') if payload else []
                    if header == 'S1:':
                        if _debugTelemetry:
                            self.lastRawS1 = segment
                        processTelemS1(elements, self.state, self.system_telem)
                        if _debugTelemetry:
                            self.lastParsedS1 = list(self.state)
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                        # Update telemetry-sync timing from S1 loop_us (index 0)
                        self.lastS1MonoTime = time.monotonic()
                        loop_us = self.system_telem.loop_us if self.system_telem.valid else 0
                        if loop_us:
                            if 1000 <= loop_us <= 50000:  # sanity: 20Hz..1000Hz
                                self.teensyLoopUs = loop_us
                                self.telemSyncActive = True
                    elif header == 'S2:':
                        if _debugTelemetry:
                            self.lastRawS2 = segment
                        processTelemS2(elements, self.servo, self.servo_telem)
                        if _debugTelemetry:
                            self.lastParsedS2 = [int(self.servo[i][2]) for i in range(18)]
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                    elif header == 'S3:':
                        processTelemS3(elements, self.servo, self.servo_telem)
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                    elif header == 'S4:':
                        processTelemS4(elements, self.legs, self.leg_telem)
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                    elif header == 'S5:':
                        prev_lockout = _safety_state.get("lockout", False)
                        processTelemS5(elements, _safety_state, self.safety_telem)
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                        lockout = _safety_state.get("lockout", False)
                        if lockout and not prev_lockout:
                            # New safety lockout detected: stop gait and hard-disable
                            if _gaitActive and _gaitEngine is not None:
                                _gaitEngine.stop()
                            _gaitActive = False
                            try:
                                send_cmd(b'LEG ALL DISABLE', force=True)
                                send_cmd(b'DISABLE', force=True)
                            except Exception:
                                pass
                            audio_safety_lockout()  # Play urgent alert
                            speak_safety_lockout()  # Voice announcement
                            if self.verbose:
                                print("Firmware reported SAFETY LOCKOUT; issued LEG ALL DISABLE + DISABLE.", end="\r\n")
                        _last_safety_lockout = lockout
                    elif header == 'S6:':
                        processTelemS6(elements, self.joint_telem)
                        self.lastTelemetryTime = now
                        self.telemetryRetryDeadline = None
                        # Update display thread with joint angles for 3D view
                        if _displayThread is not None and self.joint_telem.angles_deg:
                            _displayThread.update_joint_angles(self.joint_telem.angles_deg)
        except (ValueError, IndexError) as e:
            # Malformed telemetry line - skip but log in verbose mode
            if self.verbose:
                print(f"Telemetry parse error: {e}", end="\r\n")
        return teensyData

    def _updateGaitHeading(self):
        """Compute speed_scale and heading from stick inputs.
        
        Control scheme:
        - Left stick Y = speed and direction (forward/backward)
        - Right stick X = strafe heading angle (takes priority when active)
        
        Forward/backward is implemented via heading_deg:
        - Forward (stick up, negative value): heading = 0°
        - Backward (stick down, positive value): heading = 180°
        
        Right stick X strafe (-90° to +90°) takes priority when active.
        """
        global _gaitEngine
        if _gaitEngine is None:
            return
        
        # Speed from left stick Y: absolute value = magnitude
        speed_magnitude = abs(self._gaitSpeedInput)
        
        # Apply speed
        if speed_magnitude > 0.05:
            _gaitEngine.params.speed_scale = min(1.0, speed_magnitude)
        else:
            # Neutral: step in place
            _gaitEngine.params.speed_scale = 0.0
        
        # Heading: strafe takes priority, otherwise forward/backward from left stick Y
        if abs(self._gaitStrafeInput) > 0.05:
            # Strafe active: heading set by right stick X handler
            # (heading is set to -strafe * 90 in the event handler)
            pass
        elif speed_magnitude > 0.05:
            # No strafe, use forward/backward based on stick direction
            if self._gaitSpeedInput > 0.05:
                # Stick down = backward
                _gaitEngine.params.heading_deg = 180.0
            else:
                # Stick up = forward
                _gaitEngine.params.heading_deg = 0.0

    def poll_gamepad(self):
        """Process pending gamepad events, updating instance state and issuing commands.
        Returns count of processed events or None if controller absent."""
        global _gaitActive, _gaitEngine, _autoDisableAt, _gaitTransition
        if self.controller is None:
            return None
        processed = 0
        try:
            events = self.controller.read()
            for event in events:
                if self.verbose and _debugTelemetry:
                    print(event, end="\r\n")  # Debug print of raw event
                if event.type == 1:  # button
                    if event.code == 158 and event.value == 1:  # mirror toggle
                        if self.verbose:
                            print("\r\nscreen mirror button pressed", end="\r\n")
                        self.mirror = not self.mirror
                        self.forceDisplayUpdate = True
                    elif event.code == 315 and event.value == 1:  # Start button - cycle display mode
                        audio_click()
                        global _displayMode
                        if self.verbose:
                            print("\r\nStart button pressed (cycle display)", end="\r\n")
                        # Get robot state for menu safety check
                        if self.system_telem.valid:
                            robot_enabled = self.system_telem.robot_enabled
                        else:
                            robot_enabled = (self.state[IDX_ROBOT_ENABLED] == 1.0) if (self.state and len(self.state) > IDX_ROBOT_ENABLED) else False
                        
                        # Cycle: EYES → ENGINEERING → MENU → EYES (wrap)
                        # Menu only opens when robot disabled and not in motion
                        current_mode = _displayMode
                        next_mode = DisplayMode((current_mode + 1) % DisplayMode.COUNT)
                        
                        # If trying to enter MENU mode, check safety
                        if next_mode == DisplayMode.MENU:
                            if robot_enabled or _gaitActive:
                                # Skip menu, go to EYES instead
                                next_mode = DisplayMode.EYES
                                if self.verbose:
                                    print("  Menu blocked: disable robot first", end="\r\n")
                        
                        # Apply mode change
                        _displayMode = next_mode
                        
                        # Sync menu visibility with mode
                        if _displayMode == DisplayMode.MENU:
                            _marsMenu.show()
                        else:
                            _marsMenu.hide()
                        
                        self.forceDisplayUpdate = True
                        if self.verbose:
                            mode_names = ["EYES", "ENGINEERING", "MENU"]
                            print(f"  Display mode: {mode_names[_displayMode]}", end="\r\n")
                    elif event.code == 314:  # Back/Select button (used as a modifier)
                        if event.value == 1:
                            audio_click()
                        self._backHeld = (event.value == 1)
                        if self.verbose and event.value == 1:
                            print("\r\nBack/Select pressed", end="\r\n")
                    elif event.code == 172 and event.value == 1:  # power
                        if self.verbose:
                            print("\r\npower button pressed", end="\r\n")
                        self.requestExit = True
                    elif event.code in [139, 158, 172] and event.value == 1:  # menu/guide/power variants
                        if self.verbose:
                            print(f"\r\nXbox guide/power button pressed (code {event.code})", end="\r\n")
                        self.requestExit = True
                    elif event.code == 304 and event.value == 1:  # A button
                        audio_click()
                        if self.verbose:
                            print("\r\nA button pressed", end="\r\n")
                        if getattr(self, '_backHeld', False):
                            # Back + A: Toggle autonomy mode
                            try:
                                _toggle_autonomy(verbose=self.verbose)
                            except Exception as e:
                                print(f"Autonomy toggle error: {e}", end="\r\n")
                        elif _marsMenu.visible:
                            # Select/activate current menu item
                            _marsMenu.select()
                            self.forceDisplayUpdate = True
                        elif self.teensy is not None and not _safety_state.get("lockout", False):
                            # Normal A button behavior: Start standing gait (for leveling support)
                            _disable_autonomy_if_active(verbose=self.verbose)
                            ensure_enabled()
                            _start_standing_gait()
                            if self.verbose:
                                print("  Standing gait started (leveling enabled)", end="\r\n")
                        elif self.teensy is not None and _safety_state.get("lockout", False) and self.verbose:
                            print("  Stand blocked: firmware safety lockout is active.", end="\r\n")
                    elif event.code == 305 and event.value == 1:  # B button
                        audio_click()
                        if self.verbose:
                            print("\r\nB button pressed", end="\r\n")
                        if _marsMenu.visible:
                            # Close menu
                            _marsMenu.hide()
                            self.forceDisplayUpdate = True
                        elif self.teensy is not None:
                            # Normal B button behavior: Home
                            _disable_autonomy_if_active(verbose=self.verbose)
                            apply_posture(b'HOME', auto_disable_s=4.0)
                    elif event.code == 308 and event.value == 1:  # X toggle test/idle (BTN_WEST)
                        audio_click()
                        if self.verbose:
                            print("\r\nX button pressed (Toggle Test/Idle)", end="\r\n")
                        _disable_autonomy_if_active(verbose=self.verbose)
                        if self.teensy is not None:
                            # Check current mode from telemetry state (assuming test mode tracking)
                            # For now, simple toggle - could be enhanced with state tracking
                            if hasattr(self, '_lastMode') and self._lastMode == 'TEST':
                                send_cmd(b'MODE IDLE', force=True)
                                self._lastMode = 'IDLE'
                                if self.verbose:
                                    print("Switched to IDLE mode", end="\r\n")
                            else:
                                send_cmd(b'MODE TEST', force=True)
                                self._lastMode = 'TEST'
                                if self.verbose:
                                    print("Switched to TEST mode", end="\r\n")
                    elif event.code == 307 and event.value == 1:  # Y tuck (BTN_NORTH)
                        audio_click()
                        if self.verbose:
                            print("\r\nY button pressed (Tuck)", end="\r\n")
                        if not _marsMenu.visible and getattr(self, '_backHeld', False):
                            # Shortcut: Back + Y = Pounce
                            start_pounce_move(source="pad")
                        elif self.teensy is not None and not _safety_state.get("lockout", False):
                            _disable_autonomy_if_active(verbose=self.verbose)
                            apply_posture(b'TUCK', auto_disable_s=4.0)
                        elif self.teensy is not None and _safety_state.get("lockout", False) and self.verbose:
                            print("  Tuck blocked: firmware safety lockout is active.", end="\r\n")
                    elif event.code == 318 and event.value == 1:  # right joystick press - disable
                        audio_click()
                        if self.verbose:
                            print("\r\nRight joystick pressed (Disable)", end="\r\n")
                        if self.teensy is not None:
                            send_cmd(b'DISABLE', force=True)
                    elif event.code == 317:  # left joystick press - gait width adjustment mode
                        if event.value == 1:  # Button pressed
                            self._leftStickHeld = True
                            if self.verbose:
                                print(f"\r\nLeft joystick pressed - Gait adjustment mode (width: {self._gaitWidthMm:.0f}mm, lift: {self._gaitLiftMm:.0f}mm)", end="\r\n")
                        else:  # Button released (value == 0)
                            self._leftStickHeld = False
                            # Apply saved parameters to active gait engine
                            if _gaitEngine is not None:
                                _gaitEngine.params.base_x_mm = self._gaitWidthMm
                                _gaitEngine.params.lift_mm = self._gaitLiftMm
                            # Persist to ini file
                            if save_gait_settings(self._gaitWidthMm, self._gaitLiftMm):
                                if self.verbose:
                                    print(f"\r\nGait settings saved: width={self._gaitWidthMm:.0f}mm, lift={self._gaitLiftMm:.0f}mm", end="\r\n")
                            else:
                                if self.verbose:
                                    print(f"\r\nGait settings applied (save failed): width={self._gaitWidthMm:.0f}mm, lift={self._gaitLiftMm:.0f}mm", end="\r\n")
                    elif event.code == 310 and event.value == 1:  # LB - prev tab or toggle gait
                        audio_click()
                        if _marsMenu.visible:
                            _marsMenu.handle_button('LB')
                            self.forceDisplayUpdate = True
                        else:
                            # Check if currently in StandingGait (from A button) - treat as "not walking"
                            is_standing_gait = _gaitActive and isinstance(_gaitEngine, StandingGait)
                            if not _gaitActive or is_standing_gait:
                                if _safety_state.get("lockout", False):
                                    if self.verbose:
                                        print("\r\nGait start blocked: firmware safety lockout is active.", end="\r\n")
                                    continue
                                # Stop StandingGait if transitioning
                                if is_standing_gait and _gaitEngine is not None:
                                    _gaitEngine.stop()
                                    if self.verbose:
                                        print("  Transitioning from StandingGait to TripodGait", end="\r\n")
                                # Start gait - cancel any pending auto-disable first
                                _autoDisableAt = None
                                self.autoDisableAt = None
                                # Switch Teensy to IDLE mode (stop any built-in gait)
                                send_cmd(b'MODE IDLE', force=True)
                                # Enable robot if needed
                                enabled_now = self.system_telem.robot_enabled if self.system_telem.valid else (self.state[IDX_ROBOT_ENABLED] == 1.0 if (self.state and len(self.state) > IDX_ROBOT_ENABLED) else False)
                                if not enabled_now:
                                    ensure_enabled()
                                # Create tripod gait with config params - speed starts at 0 (step in place)
                                params = GaitParams(
                                    cycle_ms=_gaitCycleMs,
                                    base_x_mm=self._gaitWidthMm,  # Use saved gait width
                                    base_y_mm=_gaitBaseYMm,
                                    step_len_mm=_gaitStepLenMm,
                                    lift_mm=self._gaitLiftMm,     # Use saved lift height
                                    overlap_pct=_gaitOverlapPct,
                                    smoothing_alpha=_gaitSmoothingAlpha,
                                    bezier_p1_height=_bezierP1Height,
                                    bezier_p1_overshoot=_bezierP1Overshoot,
                                    bezier_p2_height=_bezierP2Height,
                                    bezier_p3_height=_bezierP3Height,
                                    bezier_p3_overshoot=_bezierP3Overshoot,
                                    speed_scale=0.0  # Start at zero; joystick controls speed
                                )
                                _gaitEngine = TripodGait(params)
                                _gaitEngine.start()
                                _gaitActive = True
                                audio_gait_start()
                                # Reset stick inputs so gait starts at zero speed until user moves stick
                                self._gaitStrafeInput = 0.0
                                self._gaitSpeedInput = 0.0
                                self._updateGaitHeading()  # Sync gait engine with current (zero) inputs
                                if self.verbose:
                                    print("\r\nLB: Gait engine STARTED (tripod)", end="\r\n")
                            else:
                                # Stop gait
                                if _gaitEngine is not None:
                                    _gaitEngine.stop()
                                _gaitActive = False
                                audio_gait_stop()
                                send_cmd(b'STAND', force=True)
                                _autoDisableAt = time.time() + _autoDisableS
                                self.autoDisableAt = _autoDisableAt
                                if self.verbose:
                                    print("\r\nLB: Gait engine STOPPED", end="\r\n")
                    elif event.code == 311 and event.value == 1:  # RB - next tab or cycle gait
                        audio_click()
                        if _marsMenu.visible:
                            _marsMenu.handle_button('RB')
                            self.forceDisplayUpdate = True
                        elif _gaitActive and _gaitEngine is not None and not _gaitTransition.is_active():
                            # Cycle through gait types: Tripod -> Wave -> Ripple -> Stationary -> Tripod
                            # Uses phase-locked transition for smooth switching
                            params = _gaitEngine.params
                            current_type = type(_gaitEngine)
                            try:
                                current_idx = GAIT_TYPES.index(current_type)
                            except ValueError:
                                current_idx = -1
                            next_idx = (current_idx + 1) % len(GAIT_TYPES)
                            next_type = GAIT_TYPES[next_idx]
                            next_name = GAIT_NAMES[next_idx]
                            
                            # Create new gait engine (StationaryPattern needs extra args)
                            if next_type == StationaryPattern:
                                pending_gait = next_type(params, radius_mm=15.0, period_ms=2000)
                            else:
                                pending_gait = next_type(params)
                            
                            # Request phase-locked transition
                            if _gaitTransition.request_transition(_gaitEngine, pending_gait):
                                if self.verbose:
                                    print(f"\r\nRB: Transitioning to {next_name} gait (waiting for phase boundary)", end="\r\n")
                            else:
                                if self.verbose:
                                    print(f"\r\nRB: Transition already in progress", end="\r\n")
                elif event.type == 3:  # analog
                    # Debug: show all analog events when left stick is held
                    if self._leftStickHeld and self.verbose:
                        print(f"Analog event: code={event.code} value={event.value}", end="\r\n")
                    #print(event,end="\r\n")
                    # Gait control: Left stick X for strafe/heading (reserved for future)
                    # Joystick range: 0-65535, center=32768, left=0, right=65535
                    if event.code == 0:  # left stick X
                        if _gaitActive and _gaitEngine is not None:
                            # Normalize to -1.0 (left) to +1.0 (right)
                            strafe = (event.value - 32768.0) / 32768.0
                            if abs(strafe) < 0.1:  # 10% deadzone
                                strafe = 0.0
                            # Store strafe component (not used in current simplified control)
                            self._gaitStrafeInput = strafe
                        # Verbose disabled to reduce overhead during gait
                    # Gait control: Left stick Y for speed
                    # Joystick range: 0-65535, center=32768, up/forward=0, down/back=65535
                    elif event.code == 1:  # left stick Y
                        # Normalize to -1.0 (forward) to +1.0 (back)
                        # up=0 -> -1.0 (forward), center=32768 -> 0, down=65535 -> +1.0 (back)
                        speed = (event.value - 32768.0) / 32768.0

                        # Gait speed control
                        if _gaitActive and _gaitEngine is not None:
                            speed_deadzone = speed
                            if abs(speed_deadzone) < 0.1:  # 10% deadzone
                                speed_deadzone = 0.0
                            # Store speed for gait engine
                            self._gaitSpeedInput = speed_deadzone
                            self._updateGaitHeading()

                        # Eye control (always active, but skip display update during gait to reduce overhead)
                        # Intensity tracks magnitude of speed in either direction (forward/back)
                        value = min(1.0, max(0.0, abs(speed)))
                        self.eyes.eyelid_percent = value * 75.0  # 0 to 75 range for intensity
                        self.eyes.eyelid_angle = value * 35.0
                        self.eyes.eye_size = (25, 45 - int(value * 10))
                        if not _gaitActive:
                            # Only force display update when gait is off (reduce overhead during gait)
                            self.forceDisplayUpdate = True
                            self.eyes.update(force_update=True)
                        # Verbose disabled to reduce overhead during gait
                    # Gait control: Right stick Y for turn rate
                    # Joystick range: 0-65535, center=32768, up=0, down=65535
                    elif event.code == 5:  # right stick Y
                        # Only process if value indicates joystick (not trigger which shares code on some controllers)
                        if event.value > 1023:
                            # Walking turn control: right stick Y for turn rate
                            # Normalize to -1.0 (up/CCW) to +1.0 (down/CW)
                            turn_input = (event.value - 32768.0) / 32768.0
                            if abs(turn_input) < 0.1:  # 10% deadzone
                                turn_input = 0.0

                            if _gaitActive and _gaitEngine is not None:
                                # Map to turn rate using configurable maximum (deg/s)
                                # Positive = clockwise (right turn), negative = CCW (left turn)
                                _gaitEngine.params.turn_rate_deg_s = turn_input * _gaitTurnMaxDegS
                                if self.verbose and abs(turn_input) > 0.1:
                                    print(f"Turn rate: {turn_input * _gaitTurnMaxDegS:.1f} deg/s", end="\r\n")
                    # Event code 2 can be either:
                    # - Left trigger (LT): values 0-1023
                    # - Right stick X (on some controllers): values 0-65535 with center at 32768
                    elif event.code == 2:
                        if event.value <= 1023:
                            # Left trigger value (0-1023)
                            if self._leftStickHeld:
                                # Gait width adjustment mode: map trigger 0-1023 to 60-140mm
                                self._gaitWidthMm = 60.0 + (event.value / 1023.0) * 80.0
                                if _gaitEngine is not None:
                                    _gaitEngine.params.base_x_mm = self._gaitWidthMm
                                if self.verbose:
                                    print(f"Gait width: {self._gaitWidthMm:.0f}mm", end="\r\n")
                        elif _gaitActive and _gaitEngine is not None:
                            # Normalize to -1.0 (left) to +1.0 (right)
                            strafe = (event.value - 32768.0) / 32768.0
                            if abs(strafe) < 0.05:  # 5% deadzone (reduced from 10%)
                                strafe = 0.0
                            # Track strafe input for heading logic in _updateGaitHeading
                            self._gaitStrafeInput = strafe
                            # Map to heading angle: -90° (left) to +90° (right), 0° = forward
                            # Reverse strafe direction so joystick X matches world left/right motion
                            if abs(strafe) > 0.05:
                                _gaitEngine.params.heading_deg = -strafe * 90.0
                            # When strafe returns to center, _updateGaitHeading will set fwd/back heading
                        else:
                            # Eye control when gait not active
                            scaledValue = ((32768.0 - float(event.value)) / 32768.0)
                            # if self.steeringMode == SteeringMode.ACKERMAN:
                            #     scaledValue = max(-0.605, min(0.605, scaledValue))
                            self.eyes.eye_center_offset = int(scaledValue * 56.0)
                            self.forceDisplayUpdate = True
                            self.eyes.update(force_update=True)
                        if self.verbose:
                            print(f"Right Stick X: {event.value}", end="\r\n")
                    elif event.code == 5:  # right stick Y
                        # Only process if value indicates joystick (not trigger which shares code on some controllers)
                        if event.value > 1023:
                            # Walking turn control: right stick Y for turn rate
                            # Joystick range: 0-65535, center=32768, up=0, down=65535
                            # Normalize to -1.0 (up/CCW) to +1.0 (down/CW)
                            turn_input = (event.value - 32768.0) / 32768.0
                            if abs(turn_input) < 0.1:  # 10% deadzone
                                turn_input = 0.0
                            
                            if _gaitActive and _gaitEngine is not None:
                                # Map to turn rate: ±60 deg/s max turn rate
                                # Positive = clockwise (right turn), negative = CCW (left turn)
                                _gaitEngine.params.turn_rate_deg_s = turn_input * 60.0
                                if self.verbose and abs(turn_input) > 0.1:
                                    print(f"Turn rate: {turn_input * 60.0:.1f} deg/s", end="\r\n")
                    elif event.code == 10:  # Left trigger (LT)
                        # Trigger range: 0-1023
                        if self._leftStickHeld:
                            # Gait width adjustment mode: map trigger to config range
                            width_range = _gaitWidthMaxMm - _gaitWidthMinMm
                            self._gaitWidthMm = _gaitWidthMinMm + (event.value / 1023.0) * width_range
                            if _gaitEngine is not None:
                                _gaitEngine.params.base_x_mm = self._gaitWidthMm
                            if self.verbose:
                                print(f"Gait width: {self._gaitWidthMm:.0f}mm", end="\r\n")
                    elif event.code == 9:  # Right trigger (RT)
                        # Trigger range: 0-1023
                        if self._leftStickHeld:
                            # Lift height adjustment mode: map trigger to config range
                            lift_range = _gaitLiftMaxMm - _gaitLiftMinMm
                            self._gaitLiftMm = _gaitLiftMinMm + (event.value / 1023.0) * lift_range
                            if _gaitEngine is not None:
                                _gaitEngine.params.lift_mm = self._gaitLiftMm
                            if self.verbose:
                                print(f"Lift height: {self._gaitLiftMm:.0f}mm", end="\r\n")
                    elif event.code == 16 and event.value == 1:  # dpad right
                        audio_click()
                        if self.verbose:
                            print(f"DPAD RIGHT: {event.value}", end="\r\n")
                        if _marsMenu.visible:
                            _marsMenu.nav_right()
                            self.forceDisplayUpdate = True
                        elif self.menuState == "settings":
                            self.menu.contract(self.menu._selected_path)
                        else:
                            self.eyes.crt_mode = not self.eyes.crt_mode
                            self.eyes.update(force_update=True)
                            self.forceDisplayUpdate = True
                    elif event.code == 16 and event.value == -1:  # dpad left
                        audio_click()
                        if self.verbose:
                            print(f"DPAD LEFT: {event.value}", end="\r\n")
                        if _marsMenu.visible:
                            _marsMenu.nav_left()
                            self.forceDisplayUpdate = True
                        elif self.menuState == "settings":
                            self.menu.contract(self.menu._selected_path)
                        else:
                            self.eyes.crt_mode = not self.eyes.crt_mode
                            self.eyes.update(force_update=True)
                            self.forceDisplayUpdate = True
                    elif event.code == 17 and event.value == -1:  # dpad up
                        audio_click()
                        if self.verbose:
                            print(f"DPAD UP: {event.value}", end="\r\n")
                        if _marsMenu.visible:
                            _marsMenu.nav_up()
                            self.forceDisplayUpdate = True
                        elif self.menuState == "settings":
                            self.menu.contract(self.menu._selected_path)
                            self.menu.move_up()
                            self.menu.select()
                            self.menu.expand()
                        else:
                            self.eyes.left_shape -= 1
                            if self.eyes.left_shape < EYE_SHAPE.ELLIPSE:
                                self.eyes.left_shape = EYE_SHAPE.ANIME
                            self.eyes.right_shape -= 1
                            if self.eyes.right_shape < EYE_SHAPE.ELLIPSE:
                                self.eyes.right_shape = EYE_SHAPE.ANIME
                            self.eyes.eye_color = _eyeColors[self.eyes.left_shape]
                            # Cycle human eye color when on human eyes
                            if self.eyes.left_shape == EYE_SHAPE.HUMAN:
                                self.eyes.human_eye_color_idx = (self.eyes.human_eye_color_idx - 1) % len(self.eyes.human_eye_colors)
                            self.eyes.update(force_update=True)
                            self.forceDisplayUpdate = True
                            save_eye_shape(self.eyes.left_shape)
                    elif event.code == 17 and event.value == 1:  # dpad down
                        audio_click()
                        if self.verbose:
                            print(f"DPAD DOWN: {event.value}", end="\r\n")
                        if _marsMenu.visible:
                            _marsMenu.nav_down()
                            self.forceDisplayUpdate = True
                        elif self.menuState == "settings":
                            self.menu.contract(self.menu._selected_path)
                        else:
                            self.eyes.left_shape += 1
                            if self.eyes.left_shape > EYE_SHAPE.ANIME:
                                self.eyes.left_shape = EYE_SHAPE.ELLIPSE
                            self.eyes.right_shape += 1
                            if self.eyes.right_shape > EYE_SHAPE.ANIME:
                                self.eyes.right_shape = EYE_SHAPE.ELLIPSE
                            self.eyes.eye_color = _eyeColors[self.eyes.left_shape]
                            # Cycle human eye color when on human eyes
                            if self.eyes.left_shape == EYE_SHAPE.HUMAN:
                                self.eyes.human_eye_color_idx = (self.eyes.human_eye_color_idx + 1) % len(self.eyes.human_eye_colors)
                            self.eyes.update(force_update=True)
                            self.forceDisplayUpdate = True
                            save_eye_shape(self.eyes.left_shape)
                    else:
                        # Debug: catch all unhandled button/key events
                        if self.verbose and event.type == 1 and event.value == 1:
                            print(f"Unhandled button event: type={event.type}, code={event.code}, value={event.value}", end="\r\n")
                processed += 1
        except BlockingIOError:
            pass
        except Exception as e:
            if self.verbose:
                print(f"Error reading from gamepad: {e}", end="\r\n")
            self.controller = None
        return processed

    def poll_joy_client(self):
        """Process joystick state from joy_controller socket daemon.
        
        This replaces poll_gamepad when using socket-based IPC.
        Processes the current JoyState and triggers appropriate actions.
        
        Returns:
            True if state was processed, False if not connected.
        """
        global _gaitActive, _gaitEngine, _autoDisableAt, _gaitTransition
        global _displayMode, _marsMenu
        
        if self.joyClient is None:
            return False
        
        # Track previous Xbox connection state for audio feedback
        prev_xbox_connected = getattr(self, '_prevXboxConnected', None)
        
        # Poll for new state (non-blocking)
        new_state = self.joyClient.poll()
        if new_state is None and not self.joyClient.connected:
            return False
        
        # Check for Xbox connect/disconnect state change
        curr_xbox_connected = self.joyClient.xbox_connected
        if prev_xbox_connected is not None and curr_xbox_connected != prev_xbox_connected:
            if curr_xbox_connected:
                audio_xbox_connect()
                if self.verbose:
                    print("Xbox controller connected", end="\r\n")
            else:
                audio_xbox_disconnect()
                if self.verbose:
                    print("Xbox controller disconnected", end="\r\n")
        self._prevXboxConnected = curr_xbox_connected
        
        # Get current state (may be updated or stale)
        joy = self.joyClient.state
        
        # Initialize previous state on first call
        if self._prevJoyState is None:
            self._prevJoyState = JoyState()
        
        #----------------------------------------------------------------------
        # Button processing (edge detection)
        #----------------------------------------------------------------------
        
        # Start button: Back+Start = toggle mirror; Start alone = cycle display mode
        if joy.button_start and not self._prevJoyState.button_start:
            audio_click()
            if self._backHeld:
                # Back + Start: Toggle screen mirror
                self.mirror = not self.mirror
                self.forceDisplayUpdate = True
                if self.verbose:
                    print(f"\r\nScreen mirror {'ON' if self.mirror else 'OFF'}", end="\r\n")
            else:
                # Start alone: Cycle display mode
                if self.verbose:
                    print("\r\nStart button pressed (cycle display)", end="\r\n")
                # Get robot state for menu safety check
                if self.system_telem.valid:
                    robot_enabled = self.system_telem.robot_enabled
                else:
                    robot_enabled = (self.state[IDX_ROBOT_ENABLED] == 1.0) if (self.state and len(self.state) > IDX_ROBOT_ENABLED) else False
                
                # Cycle: EYES → ENGINEERING → MENU → EYES (wrap)
                current_mode = _displayMode
                next_mode = DisplayMode((current_mode + 1) % DisplayMode.COUNT)
                
                # If trying to enter MENU mode, check safety
                if next_mode == DisplayMode.MENU:
                    if robot_enabled or _gaitActive:
                        next_mode = DisplayMode.EYES
                        if self.verbose:
                            print("  Menu blocked: disable robot first", end="\r\n")
                
                # Apply mode change
                _displayMode = next_mode
                if _displayMode == DisplayMode.MENU:
                    _marsMenu.show()
                else:
                    _marsMenu.hide()
                
                self.forceDisplayUpdate = True
                if self.verbose:
                    mode_names = ["EYES", "ENGINEERING", "MENU"]
                    print(f"  Display mode: {mode_names[_displayMode]}", end="\r\n")
        
        # Back button (modifier tracking)
        if joy.button_back != self._prevJoyState.button_back:
            if joy.button_back:
                audio_click()
            self._backHeld = joy.button_back
            if self.verbose and joy.button_back:
                print("\r\nBack/Select pressed", end="\r\n")
        
        # A button
        if joy.button_a and not self._prevJoyState.button_a:
            audio_click()
            if self.verbose:
                print("\r\nA button pressed", end="\r\n")
            if self._backHeld:
                # Back + A: Toggle autonomy mode
                try:
                    _toggle_autonomy(verbose=self.verbose)
                except Exception as e:
                    print(f"Autonomy toggle error: {e}", end="\r\n")
            elif _marsMenu.visible:
                _marsMenu.select()
                self.forceDisplayUpdate = True
            elif self.teensy is not None and not _safety_state.get("lockout", False):
                _disable_autonomy_if_active(verbose=self.verbose)
                ensure_enabled()
                _start_standing_gait()
                if self.verbose:
                    print("  Standing gait started (leveling enabled)", end="\r\n")
            elif self.teensy is not None and _safety_state.get("lockout", False) and self.verbose:
                print("  Stand blocked: firmware safety lockout is active.", end="\r\n")
        
        # B button
        if joy.button_b and not self._prevJoyState.button_b:
            audio_click()
            if self.verbose:
                print("\r\nB button pressed", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.hide()
                self.forceDisplayUpdate = True
            elif self.teensy is not None:
                # Stop any active gait first
                _disable_autonomy_if_active(verbose=self.verbose)
                if _gaitActive and _gaitEngine is not None:
                    _gaitEngine.stop()
                    _gaitActive = False
                apply_posture(b'HOME', auto_disable_s=4.0)
        
        # X button: tuck
        if joy.button_x and not self._prevJoyState.button_x:
            audio_click()
            if self.verbose:
                print("\r\nX button pressed (Tuck)", end="\r\n")
            if not _marsMenu.visible and self._backHeld:
                # Shortcut: Back + X = Pounce
                start_pounce_move(source="pad")
            elif self.teensy is not None and not _safety_state.get("lockout", False):
                # Stop any active gait first
                _disable_autonomy_if_active(verbose=self.verbose)
                if _gaitActive and _gaitEngine is not None:
                    _gaitEngine.stop()
                    _gaitActive = False
                apply_posture(b'TUCK', auto_disable_s=4.0)
            elif _safety_state.get("lockout", False) and self.verbose:
                print("  Tuck blocked: firmware safety lockout is active.", end="\r\n")
        
        # Y button: toggle test/idle mode
        if joy.button_y and not self._prevJoyState.button_y:
            audio_click()
            if self.verbose:
                print("\r\nY button pressed (Toggle Test/Idle)", end="\r\n")
            _disable_autonomy_if_active(verbose=self.verbose)
            if self.teensy is not None:
                # Stop any active gait first
                if _gaitActive and _gaitEngine is not None:
                    _gaitEngine.stop()
                    _gaitActive = False
                if hasattr(self, '_lastMode') and self._lastMode == 'TEST':
                    send_cmd(b'MODE IDLE', force=True)
                    self._lastMode = 'IDLE'
                    if self.verbose:
                        print("Switched to IDLE mode", end="\r\n")
                else:
                    send_cmd(b'MODE TEST', force=True)
                    self._lastMode = 'TEST'
                    if self.verbose:
                        print("Switched to TEST mode", end="\r\n")
        
        # LB button: toggle gait
        if joy.button_lb and not self._prevJoyState.button_lb:
            audio_click()
            if self.verbose:
                print("\r\nLB button pressed", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.handle_button('LB')
                self.forceDisplayUpdate = True
            elif self.teensy is not None:
                # Check if currently in StandingGait (from A button) - treat as "not walking"
                is_standing_gait = _gaitActive and isinstance(_gaitEngine, StandingGait)
                if not _gaitActive or is_standing_gait:
                    # Start walking gait (or transition from StandingGait)
                    if is_standing_gait and _gaitEngine is not None:
                        _gaitEngine.stop()
                        if self.verbose:
                            print("  Transitioning from StandingGait to TripodGait", end="\r\n")
                    ensure_enabled()
                    _gaitActive = True
                    params = GaitParams(
                        cycle_ms=_gaitCycleMs,
                        base_x_mm=self._gaitWidthMm,
                        base_y_mm=_gaitBaseYMm,
                        step_len_mm=_gaitStepLenMm,
                        lift_mm=self._gaitLiftMm,
                        overlap_pct=_gaitOverlapPct,
                        smoothing_alpha=_gaitSmoothingAlpha,
                        bezier_p1_height=_bezierP1Height,
                        bezier_p1_overshoot=_bezierP1Overshoot,
                        bezier_p2_height=_bezierP2Height,
                        bezier_p3_height=_bezierP3Height,
                        bezier_p3_overshoot=_bezierP3Overshoot,
                        speed_scale=0.0
                    )
                    _gaitEngine = TripodGait(params)
                    _gaitEngine.start()
                    _autoDisableAt = None
                    self.autoDisableAt = None
                    audio_gait_start()
                    # Reset stick inputs so gait starts at zero speed
                    self._gaitStrafeInput = 0.0
                    self._gaitSpeedInput = 0.0
                    self._updateGaitHeading()
                    if self.verbose:
                        print("\r\nLB: Gait engine STARTED (tripod)", end="\r\n")
                else:
                    # Stop gait
                    if _gaitEngine is not None:
                        _gaitEngine.stop()
                    _gaitActive = False
                    audio_gait_stop()
                    send_cmd(b'STAND', force=True)
                    _autoDisableAt = time.time() + _autoDisableS
                    self.autoDisableAt = _autoDisableAt
                    if self.verbose:
                        print("\r\nLB: Gait engine STOPPED", end="\r\n")
        
        # RB button: next tab or cycle gait
        if joy.button_rb and not self._prevJoyState.button_rb:
            audio_click()
            if _marsMenu.visible:
                _marsMenu.handle_button('RB')
                self.forceDisplayUpdate = True
            elif _gaitActive and _gaitEngine is not None and not _gaitTransition.is_active():
                # Cycle gait types
                params = _gaitEngine.params
                current_type = type(_gaitEngine)
                try:
                    current_idx = GAIT_TYPES.index(current_type)
                except ValueError:
                    current_idx = -1
                next_idx = (current_idx + 1) % len(GAIT_TYPES)
                next_type = GAIT_TYPES[next_idx]
                next_name = GAIT_NAMES[next_idx]
                
                if next_type == StationaryPattern:
                    pending_gait = next_type(params, radius_mm=15.0, period_ms=2000)
                else:
                    pending_gait = next_type(params)
                
                if _gaitTransition.request_transition(_gaitEngine, pending_gait):
                    if self.verbose:
                        print(f"\r\nRB: Transitioning to {next_name} gait", end="\r\n")
        
        # Left stick button (modifier)
        if joy.button_lstick != self._prevJoyState.button_lstick:
            self._leftStickHeld = joy.button_lstick
            if self.verbose and joy.button_lstick:
                print("\r\nLeft stick button held (param adjust mode)", end="\r\n")
        
        # Right stick button: disable
        if joy.button_rstick and not self._prevJoyState.button_rstick:
            if self.verbose:
                print("\r\nRight stick button pressed (Disable)", end="\r\n")
            if self.teensy is not None:
                # Stop any active gait first
                if _gaitActive and _gaitEngine is not None:
                    _gaitEngine.stop()
                    _gaitActive = False
                send_cmd(b'DISABLE', force=True)
        
        # D-pad
        if joy.dpad_up and not self._prevJoyState.dpad_up:
            audio_click()
            if self.verbose:
                print(f"DPAD UP", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.nav_up()
                self.forceDisplayUpdate = True
            elif _displayMode == DisplayMode.ENGINEERING:
                # Adjust 3D view elevation (more top-down)
                if _displayThread is not None:
                    _displayThread.adjust_view_angle(d_elevation=10.0)
                self.forceDisplayUpdate = True
            else:
                # Cycle eye shape down
                self.eyes.left_shape -= 1
                if self.eyes.left_shape < EYE_SHAPE.ELLIPSE:
                    self.eyes.left_shape = EYE_SHAPE.ANIME
                self.eyes.right_shape -= 1
                if self.eyes.right_shape < EYE_SHAPE.ELLIPSE:
                    self.eyes.right_shape = EYE_SHAPE.ANIME
                self.eyes.eye_color = _eyeColors[self.eyes.left_shape]
                if self.eyes.left_shape == EYE_SHAPE.HUMAN:
                    self.eyes.human_eye_color_idx = (self.eyes.human_eye_color_idx - 1) % len(self.eyes.human_eye_colors)
                self.eyes.update(force_update=True)
                self.forceDisplayUpdate = True
                save_eye_shape(self.eyes.left_shape)
        
        if joy.dpad_down and not self._prevJoyState.dpad_down:
            audio_click()
            if self.verbose:
                print(f"DPAD DOWN", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.nav_down()
                self.forceDisplayUpdate = True
            elif _displayMode == DisplayMode.ENGINEERING:
                # Adjust 3D view elevation (less top-down / more side view)
                if _displayThread is not None:
                    _displayThread.adjust_view_angle(d_elevation=-10.0)
                self.forceDisplayUpdate = True
            else:
                # Cycle eye shape up
                self.eyes.left_shape += 1
                if self.eyes.left_shape > EYE_SHAPE.ANIME:
                    self.eyes.left_shape = EYE_SHAPE.ELLIPSE
                self.eyes.right_shape += 1
                if self.eyes.right_shape > EYE_SHAPE.ANIME:
                    self.eyes.right_shape = EYE_SHAPE.ELLIPSE
                self.eyes.eye_color = _eyeColors[self.eyes.left_shape]
                if self.eyes.left_shape == EYE_SHAPE.HUMAN:
                    self.eyes.human_eye_color_idx = (self.eyes.human_eye_color_idx + 1) % len(self.eyes.human_eye_colors)
                self.eyes.update(force_update=True)
                self.forceDisplayUpdate = True
                save_eye_shape(self.eyes.left_shape)
        
        if joy.dpad_left and not self._prevJoyState.dpad_left:
            audio_click()
            if self.verbose:
                print(f"DPAD LEFT", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.nav_left()
                self.forceDisplayUpdate = True
            elif _displayMode == DisplayMode.ENGINEERING:
                # Rotate 3D view counter-clockwise (from top view)
                if _displayThread is not None:
                    _displayThread.adjust_view_angle(d_azimuth=-15.0)
                self.forceDisplayUpdate = True
            else:
                self.eyes.crt_mode = not self.eyes.crt_mode
                self.eyes.update(force_update=True)
                self.forceDisplayUpdate = True
        
        if joy.dpad_right and not self._prevJoyState.dpad_right:
            audio_click()
            if self.verbose:
                print(f"DPAD RIGHT", end="\r\n")
            if _marsMenu.visible:
                _marsMenu.nav_right()
                self.forceDisplayUpdate = True
            elif _displayMode == DisplayMode.ENGINEERING:
                # Rotate 3D view clockwise (from top view)
                if _displayThread is not None:
                    _displayThread.adjust_view_angle(d_azimuth=15.0)
                self.forceDisplayUpdate = True
            else:
                self.eyes.crt_mode = not self.eyes.crt_mode
                self.eyes.update(force_update=True)
                self.forceDisplayUpdate = True
        
        #----------------------------------------------------------------------
        # Analog axis processing (continuous)
        #----------------------------------------------------------------------
        
        # Left stick Y: gait speed
        if _gaitActive and _gaitEngine is not None:
            # Negate to match original: stick up (positive joy.left_y) = forward (negative speed)
            speed = -joy.left_y  # Forward = negative, back = positive (matches poll_gamepad)
            if abs(speed) < 0.1:
                speed = 0.0
            self._gaitSpeedInput = speed
            self._updateGaitHeading()
        
        # Eye control from left stick Y (always active)
        value = min(1.0, max(0.0, abs(joy.left_y)))
        self.eyes.eyelid_percent = value * 75.0
        self.eyes.eyelid_angle = value * 35.0
        self.eyes.eye_size = (25, 45 - int(value * 10))
        if not _gaitActive:
            self.forceDisplayUpdate = True
            self.eyes.update(force_update=True)
        
        # Left stick X: strafe input
        if _gaitActive and _gaitEngine is not None:
            strafe = joy.left_x  # Already -1 to +1
            if abs(strafe) < 0.1:
                strafe = 0.0
            self._gaitStrafeInput = strafe
        
        # Right stick X: eye center offset / heading when gait active
        if _gaitActive and _gaitEngine is not None:
            # Map to heading angle: -90° (left) to +90° (right)
            strafe = joy.right_x
            if abs(strafe) < 0.05:
                strafe = 0.0
            if abs(strafe) > 0.05:
                _gaitEngine.params.heading_deg = -strafe * 90.0
        else:
            # Eye control when gait not active
            # Negate to match original poll_gamepad behavior (stick right = eyes right)
            self.eyes.eye_center_offset = int(-joy.right_x * 56.0)
            self.forceDisplayUpdate = True
            self.eyes.update(force_update=True)
        
        # Right stick Y: turn rate when gait active
        if _gaitActive and _gaitEngine is not None:
            turn_input = -joy.right_y  # Invert: stick up = CCW
            if abs(turn_input) < 0.1:
                turn_input = 0.0
            _gaitEngine.params.turn_rate_deg_s = turn_input * _gaitTurnMaxDegS
        
        # Triggers: gait parameter adjustment (when left stick held)
        if self._leftStickHeld:
            # Left trigger: gait width
            width_range = _gaitWidthMaxMm - _gaitWidthMinMm
            self._gaitWidthMm = _gaitWidthMinMm + joy.trigger_left * width_range
            if _gaitEngine is not None:
                _gaitEngine.params.base_x_mm = self._gaitWidthMm
            
            # Right trigger: lift height
            lift_range = _gaitLiftMaxMm - _gaitLiftMinMm
            self._gaitLiftMm = _gaitLiftMinMm + joy.trigger_right * lift_range
            if _gaitEngine is not None:
                _gaitEngine.params.lift_mm = self._gaitLiftMm
        
        #----------------------------------------------------------------------
        # Save previous state for next iteration
        #----------------------------------------------------------------------
        # Deep copy relevant fields for edge detection
        self._prevJoyState.button_a = joy.button_a
        self._prevJoyState.button_b = joy.button_b
        self._prevJoyState.button_x = joy.button_x
        self._prevJoyState.button_y = joy.button_y
        self._prevJoyState.button_lb = joy.button_lb
        self._prevJoyState.button_rb = joy.button_rb
        self._prevJoyState.button_back = joy.button_back
        self._prevJoyState.button_start = joy.button_start
        self._prevJoyState.button_lstick = joy.button_lstick
        self._prevJoyState.button_rstick = joy.button_rstick
        self._prevJoyState.dpad_up = joy.dpad_up
        self._prevJoyState.dpad_down = joy.dpad_down
        self._prevJoyState.dpad_left = joy.dpad_left
        self._prevJoyState.dpad_right = joy.dpad_right
        
        return True

    def housekeeping(self):
        """Handle telemetry auto-start and retry logic using instance state."""
        try:
            if self.teensy is not None and self.lastTelemetryTime is None:
                now = time.time()
                if self.telemetryGraceDeadline is None:
                    self.telemetryGraceDeadline = now + self.telemetryGraceSeconds
                elif now >= self.telemetryGraceDeadline and not self.telemetryStartedByScript:
                    if self.verbose:
                        print("No telemetry detected after grace; sending 'LEG ALL ENABLE' + 'Y 1'.", end="\r\n")
                    # Enable all legs so telemetry includes valid data from all servos
                    send_cmd(b'LEG ALL ENABLE', force=True)
                    if self.telemBinDesired:
                        send_cmd(b'TELEM BIN 1', force=True)
                    send_cmd(b'Y 1', force=True)
                    self.telemetryStartedByScript = True
                    self.telemetryRetryDeadline = now + self.telemetryRetrySeconds
            # Telemetry stall retry: if we started it but still no data, resend once with backoff
            if (
                self.teensy is not None and self.telemetryStartedByScript and self.lastTelemetryTime is None
                and self.telemetryRetryDeadline is not None and time.time() >= self.telemetryRetryDeadline
                and self.telemetryRetryCount == 0
            ):
                if self.verbose:
                    print("Telemetry still silent; retrying 'LEG ALL ENABLE' + 'Y 1' once.", end="\r\n")
                send_cmd(b'LEG ALL ENABLE', force=True)
                if self.telemBinDesired:
                    send_cmd(b'TELEM BIN 1', force=True)
                send_cmd(b'Y 1', force=True)
                self.telemetryRetryCount = 1
                self.telemetryRetryDeadline = None

            # Poll PID/IMP/EST LIST while menu is visible so tabs stay in sync.
            global _pid_list_next_at, _imp_list_next_at, _est_list_next_at
            if self.teensy is not None and _marsMenu is not None and _marsMenu.visible:
                now = time.time()
                if _pid_list_next_at <= 0.0:
                    _pid_list_next_at = now
                    _imp_list_next_at = now + 0.35
                    _est_list_next_at = now + 0.70
                if now >= _pid_list_next_at:
                    send_cmd(b'PID LIST', throttle_ms=250)
                    _pid_list_next_at = now + 1.0
                if now >= _imp_list_next_at:
                    send_cmd(b'IMP LIST', throttle_ms=250)
                    _imp_list_next_at = now + 1.0
                if now >= _est_list_next_at:
                    send_cmd(b'EST LIST', throttle_ms=250)
                    _est_list_next_at = now + 1.0
        except serial.SerialException as e:
            if self.verbose:
                print(f"Housekeeping serial error: {e}", end="\r\n")
        return None

#----------------------------------------------------------------------------------------------------------------------
# Input I/O functions delegated to input_handler module (Phase 6 modularization)
# find_teensy, readTeensy, readTeensyBytes, testForGamePad, keyboard functions
# are now imported from input_handler.py. These wrappers maintain compatibility.
#----------------------------------------------------------------------------------------------------------------------

# Binary telemetry framing (firmware: TELEM BIN 1)
# Use TELEM_SYNC imported from telemetry.py; alias for local compatibility
_TELEM_SYNC = TELEM_SYNC
_preferBinaryTelemetry = True


def find_teensy(_verbose=False):
    """Wrapper for input_handler.find_teensy()."""
    return find_teensy_module(verbose=_verbose)


def readTeensy(teensy, _verbose=False):
    """Wrapper for input_handler.read_teensy()."""
    return read_teensy_module(teensy, verbose=_verbose)


def readTeensyBytes(teensy, _verbose=False):
    """Wrapper for input_handler.read_teensy_bytes()."""
    return read_teensy_bytes_module(teensy, verbose=_verbose)


# Gait number to string mapping moved to telemetry.py (GAIT_NAMES, getGait)
# Use GAIT_NAMES dict or getGait() helper directly

# testForGamePad, keyboard functions are now imported from input_handler.py
# The imports provide: testForGamePad, init_keyboard, cleanup_keyboard, poll_keyboard, getkey

#----------------------------------------------------------------------------------------------------------------------
# Display functions delegated to display_thread module (Phase 5 modularization)
# The getColor, drawLogo, drawMarsSplash, UpdateDisplay, and DisplayThread
# are now imported from display_thread.py. These wrappers maintain compatibility.
#----------------------------------------------------------------------------------------------------------------------

# Frame change detection for display optimization
_last_frame_hash = None


def UpdateDisplay(disp, image, menu, servo=None, legs=None, state=None,
                  mirror=False, menuState=None,
                  teensy_connected=True, controller_connected=True,
                  telemetry_stale=False, robot_enabled=True,
                  safety_active=False, safety_text=""):
    """Updates the display with the given image.
    Wrapper around display_thread.UpdateDisplay for backward compatibility.
    """
    global _forceDisplayUpdate
    
    def _force_update_callback():
        global _forceDisplayUpdate
        _forceDisplayUpdate = True
    
    _ctrl = globals().get('ctrl', None)
    
    UpdateDisplayModule(
        disp=disp,
        image=image,
        menu=menu,
        servo=servo,
        legs=legs,
        state=state,
        mirror=mirror,
        menuState=menuState,
        teensy_connected=teensy_connected,
        controller_connected=controller_connected,
        telemetry_stale=telemetry_stale,
        robot_enabled=robot_enabled,
        safety_active=safety_active,
        safety_text=safety_text,
        mars_menu=_marsMenu,
        force_display_update_callback=_force_update_callback,
        power_palette=_powercolorPalette,
        temperature_palette=_temperatureColorPalette,
        ctrl=_ctrl,
    )


# Telemetry parsing functions (processTelemS1-S5) and get_safety_overlay_state
# are now imported from telemetry.py module

# Safety state dict kept for legacy compatibility - used by processTelemS5
_safety_state = {
    "lockout": False,
    "cause_mask": 0,
    "override_mask": 0,
    "clearance_mm": 0,
    "soft_limits": False,
    "collision": False,
    "temp_c": 0,
}
_last_safety_lockout = False


def get_safety_overlay_state():
    """Return (active, text) tuple for safety overlay rendering.
    Wrapper around telemetry.get_safety_overlay_text() for compatibility."""
    _ctrl = globals().get('ctrl', None)
    _safety_telem = getattr(_ctrl, 'safety_telem', None) if _ctrl is not None else None
    if _safety_telem is not None and getattr(_safety_telem, 'valid', False):
        active = bool(getattr(_safety_telem, 'lockout', False))
        mask = int(getattr(_safety_telem, 'cause_mask', 0) or 0)
    else:
        active = _safety_state.get("lockout", False)
        mask = _safety_state.get("cause_mask", 0)
    if not active:
        return False, ""
    return True, get_safety_overlay_text(mask)


# PID/IMP/EST LIST state (parsed from firmware text responses)
_pid_ini = {
    "enabled": None,
    "mode": None,
    "shadow_hz": None,
    "kp": None,
    "ki": None,
    "kd": None,
    "kdalph": None,
}
_imp_ini = {
    "enabled": None,
    "mode": None,
    "scale": None,
    "jspring": None,
    "jdamp": None,
    "cspring": None,
    "cdamp": None,
    "jdb_cd": None,
    "cdb_mm": None,
}
_est_ini = {
    "cmd_alpha_milli": None,
    "meas_alpha_milli": None,
    "meas_vel_alpha_milli": None,
}

# Safety display thresholds (Pi-side visualization, not firmware safety limits)
_safetyDisplayVoltMin = 10.5
_safetyDisplayVoltWarn = 11.0
_safetyDisplayVoltMax = 12.5
_safetyDisplayTempMin = 25.0
_safetyDisplayTempMax = 55.0
_showBatteryIcon = True  # Show battery icon on EYES display

# Low battery protection (Pi-side graceful shutdown)
_lowBatteryVoltCritical = 10.0   # Below this: TUCK + DISABLE sequence
_lowBatteryEnabled = True         # Enable/disable low battery protection
_lowBatteryTriggered = False      # Latch: prevents re-enable until voltage recovers
_lowBatteryRecoveryVolt = 10.5    # Must exceed this to clear the latch
_lowBatteryFilteredVoltage = 0.0  # Low-pass filtered voltage for protection
_lowBatteryFilterAlpha = 0.05     # Slow filter (more stable than display filter)

# Dashboard-editable PID/IMP/EST parameters (synced to _*_ini and firmware)
_pidEnabled = False
_pidKp = 0.0
_pidKi = 0.0
_pidKd = 0.0
_pidMaxCorrection = 10.0

_impEnabled = False
_impKp = 100.0      # scale
_impKd = 0.0        # jdb_cd
_impMaxN = 50.0     # max force

_estAlpha = 0.1     # cmd_alpha (0-1)
_estVelDecay = 0.95 # meas_vel_alpha (0-1)

_pid_state = {
    "enabled": None,        # bool
    "mode": None,           # 'active'|'shadow'
    "kp": None,             # [coxa,femur,tibia] milli
    "ki": None,
    "kd": None,
    "kdalph": None,
    "shadow_hz": None,      # int
    "last_update": 0.0,
}
_imp_state = {
    "enabled": None,        # bool
    "mode": None,           # 'off'|'joint'|'cart'
    "jspring": None,        # [coxa,femur,tibia] milli
    "jdamp": None,
    "cspring": None,        # [x,y,z] milli
    "cdamp": None,
    "scale": None,          # int milli
    "jdb_cd": None,         # int
    "cdb_mm": None,         # float
    "last_update": 0.0,
}
_est_state = {
    "cmd_alpha_milli": None,
    "meas_alpha_milli": None,
    "meas_vel_alpha_milli": None,
    "last_update": 0.0,
}

_pid_list_next_at = 0.0
_imp_list_next_at = 0.0
_est_list_next_at = 0.0


def _parse_triplet_int(value: str):
    parts = value.split('/')
    if len(parts) != 3:
        return None
    out = []
    for p in parts:
        try:
            out.append(int(p))
        except (TypeError, ValueError):
            return None
    return out


def _parse_pid_list_line(line: str):
    global _pid_state
    # Example: PID enabled=1 mode=active kp=0/0/0 ki=0/0/0 kd=0/0/0 kdalph=200/200/200 shadow_hz=2
    tokens = line.strip().split()
    if not tokens or tokens[0] != "PID":
        return
    for tok in tokens[1:]:
        if '=' not in tok:
            continue
        k, v = tok.split('=', 1)
        k = k.strip().lower()
        v = v.strip()
        if k == 'enabled':
            _pid_state['enabled'] = (v == '1' or v.lower() == 'true')
        elif k == 'mode':
            _pid_state['mode'] = v.lower()
        elif k in ('kp', 'ki', 'kd', 'kdalph'):
            tri = _parse_triplet_int(v)
            if tri is not None:
                _pid_state[k] = tri
        elif k == 'shadow_hz':
            try:
                _pid_state['shadow_hz'] = int(v)
            except (TypeError, ValueError):
                pass
    _pid_state['last_update'] = time.time()


def _parse_imp_list_line(line: str):
    global _imp_state
    # Example: IMP enabled=0 mode=off jspring=0/0/0 jdamp=0/0/0 cspring=0/0/0 cdamp=0/0/0 scale=1000 jdb_cd=50 cdb_mm=10.00
    tokens = line.strip().split()
    if not tokens or tokens[0] != "IMP":
        return
    for tok in tokens[1:]:
        if '=' not in tok:
            continue
        k, v = tok.split('=', 1)
        k = k.strip().lower()
        v = v.strip()
        if k == 'enabled':
            _imp_state['enabled'] = (v == '1' or v.lower() == 'true')
        elif k == 'mode':
            _imp_state['mode'] = v.lower()
        elif k in ('jspring', 'jdamp', 'cspring', 'cdamp'):
            tri = _parse_triplet_int(v)
            if tri is not None:
                _imp_state[k] = tri
        elif k in ('scale', 'jdb_cd'):
            try:
                _imp_state[k] = int(v)
            except (TypeError, ValueError):
                pass
        elif k == 'cdb_mm':
            try:
                _imp_state['cdb_mm'] = float(v)
            except (TypeError, ValueError):
                pass
    _imp_state['last_update'] = time.time()


def _parse_est_list_line(line: str):
    global _est_state
    # Example: EST cmd_alpha_milli=100 meas_alpha_milli=150 meas_vel_alpha_milli=100
    tokens = line.strip().split()
    if not tokens or tokens[0] != "EST":
        return
    for tok in tokens[1:]:
        if '=' not in tok:
            continue
        k, v = tok.split('=', 1)
        k = k.strip().lower()
        v = v.strip()
        if k in ('cmd_alpha_milli', 'meas_alpha_milli', 'meas_vel_alpha_milli'):
            try:
                _est_state[k] = int(v)
            except (TypeError, ValueError):
                pass
    _est_state['last_update'] = time.time()



#-----------------------------------------------------------------------------------------------------------------------
#    Define the main controller class
#-----------------------------------------------------------------------------------------------------------------------

# initialze the global variables here
_run = True # controls whether the mail loop is running
_verbose = False # enables verbose logging and debugging
_showLoopTime = False # flag to show the loop timing
_logging = True # flage the use of the logging module
_loopStartTime = time.time() # used to control the loop timing
_loopdt = 0.0 # stores the time delta for the loop
_screenRefreshms = 100. # time in milliseconds to refresh the screen (10Hz)
_retryCount = 0 # number of retries to connect to the controller device
# Monotonic loop timing: target period and next scheduled tick
_loopTargetMs = 5.0  # target loop period in milliseconds (200 Hz max)
_nextTickTime = None  # monotonic time for next scheduled tick (initialized in loop)
# Telemetry-synchronized timing: derive loop period from Teensy S1 tick duration
_teensyLoopUs = 6024  # Teensy loop period in microseconds (default 166Hz = 6024us)
_lastS1MonoTime = None  # monotonic timestamp of last S1 receipt
_telemSyncActive = False  # True when actively syncing to Teensy telemetry
_telemSyncFallbackMs = 50.0  # fallback to fixed timing if no S1 for this long
_displayMode = DisplayMode.EYES  # Current display mode: EYES, ENGINEERING, or MENU
_menuVisible = False # flag to show or hide the menu (legacy, now derived from _displayMode)
_mirrorDisplay = False # flag to mirror the display to the console
_forceDisplayUpdate = False # flag to force the display update
_displayBrightness = 100 # initial brightness of the display
_startupDelayS = 5.0 # delay after startup splash before switching to eyes (0-30s in 0.5s steps)
_menuState = None
_marsMenu = None  # MARS menu instance (created after display init)
_steeringMode = SteeringMode.OMNI  # initial steering mode
_telemetryStartedByScript = False   # whether we issued 'Y 1'
_lastTelemetryTime = None           # timestamp of last received telemetry segment
_telemetryGraceDeadline = None      # time after which we send 'Y 1' if no telemetry
_telemetryGraceSeconds = 0.75       # grace period before attempting auto-start
_telemetryRetrySeconds = 2.0        # if no S1/S2 after start, retry once after this delay
_telemetryRetryDeadline = None      # when to perform the single retry
_telemetryRetryCount = 0            # single retry guard
_teensyReconnectBackoffSeconds = 1.5  # backoff before rescanning after a disconnect
_nextTeensyScanAt = 0.0               # earliest time to attempt a new scan
_teensyErrorCount = 0                 # consecutive read errors

# Directive: Do NOT mark Python controller TODO items completed until user confirmation.
_lastRawS1 = ''                     # raw S1 segment string (header+payload)
_lastParsedS1 = []                  # copy of parsed S1 numeric list
_lastRawS2 = ''                     # raw S2 segment string (enable flags)
_lastParsedS2 = []                  # parsed S2 enable flag list
_debugTelemetry = False             # toggle for printing raw/parsed S1/S2 each loop
_debugSendAll = False               # toggle for printing all send_cmd activity
_gaitDebugCmds = {b'r',b'w',b't',b'm',b'x',b'z',b'ENABLE',b'DISABLE',b'LEG ALL ENABLE',b'TUCK',b'STAND'}

# Gait engine state
_gaitEngine = None  # Active gait engine instance (TripodGait, StationaryPattern, etc.)
_gaitActive = False  # True when gait engine is actively sending FEET commands
_gaitStrafeInput = 0.0  # Left stick X: -1 (left) to +1 (right)
_gaitSpeedInput = 0.0   # Left stick Y: -1 (back) to +1 (forward)
_gaitTickCount = 0      # Tick counter for rate limiting FEET sends
_gaitSendDivisor = 3    # Send FEET every N ticks (2 = 83Hz, 3 = 55Hz)
_gaitTransition = GaitTransition(blend_ms=500)  # Transition manager for smooth gait switching

# Three-Layer Architecture tick counters (inspired by biological motor control)
# Layer 1: Controller (166 Hz) - runs every tick (hardware I/O, safety reflexes)
# Layer 2: Sequencer (~30 Hz) - runs every N ticks (behaviors, input, display)
# Layer 3: Deliberator (~1 Hz) - async/on-demand (planning, SLAM) - future
_layer2TickCount = 0      # Tick counter for Layer 2 phases
_layer2Divisor = 5        # Run Layer 2 every N ticks (166/5 = 33 Hz)
_layer3TickCount = 0      # Tick counter for Layer 3 phases (future)
_layer3Divisor = 166      # Run Layer 3 every N ticks (166/166 = 1 Hz) - future

# Kinematic move state (non-cyclic sequences that send FEET)
_moveEngine = None
_moveActive = False
_moveTickCount = 0
_moveSendDivisor = 1  # Send FEET every tick while a kinematic move is active

# FEET command tolerance filtering to reduce serial spam
_lastFeetPositions = None  # List of 18 floats: [x0, y0, z0, x1, y1, z1, ...] for 6 legs
_feetToleranceMm = 0.3     # Skip FEET command if max position delta < this threshold (mm)
_lastFeetSendTime = 0.0    # Monotonic time of last FEET send
_feetMaxSkipMs = 100.0     # Force send if no FEET sent for this long (prevents stalls during turn/strafe)

# create the Teensy serial object
_teensy = None
# create the Xbox controller object (legacy evdev - used as fallback)
_controller = None
# create the Joy socket client (preferred - connects to joy_controller.py daemon)
_joyClient: JoyClient = None
# Joy client vs evdev mode
_useJoySocket = True  # When True, use socket-based joy_controller; False = direct evdev

# define the state array to store the state telemetry data
_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # # 11 elements for the state telemetry data

# define the servo array to store servo telemetry data
_servo = [[0.0, 0.0, 0.0] for _ in range(18)]  # 18 servos, each with voltage and temperature

# define the legs array to store leg telemetry data
_legs = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]  # 6 legs, each with contact and angles

# define the pallettes for the color mapping
_powercolorPalette = [(255,50,50), (50, 255, 50)]
_temperatureColorPalette = [(0,191,255), (0, 255, 0), (255, 0, 0)]  # blue to green to red

# Load optional controller.ini
_cfg = None
_cfg_path = None
# Config-loaded values with defaults
_teensyErrorThreshold = 3
_gaitCycleMs = 2000
_gaitStepLenMm = 40.0
_gaitBaseYMm = -120.0
_gaitMaxStepLenMm = 40.0
_gaitOverlapPct = 5.0
_gaitSmoothingAlpha = 0.15
_gaitSendDivisor = 3
_gaitTurnMaxDegS = 60.0
_cmdThrottleMs = 50.0
_autoDisableS = 5.0
# Bezier curve shape parameters
_bezierP1Height = 0.15
_bezierP1Overshoot = 1.1
_bezierP2Height = 1.5
_bezierP3Height = 0.35
_bezierP3Overshoot = 1.1
# Eye settings
_eyeSpacingOffset = 10
_eyeCenterOffset = 5      # Horizontal eye center offset (pixels)
_eyeVerticalOffset = 0    # Vertical eye center offset from baseline (pixels)
_eyeEyelidAngle = 0
_eyeBlinkPercentStep = 0.08
_eyeRotation = -10
_eyeSizeX = 25
_eyeSizeY = 45
_eyeLookRangeX = 20.0  # Pixels of eye offset at max joystick X input
_eyeLookRangeY = 10.0  # Pixels of eye offset at max joystick Y input
_humanEyeSpacingPct = 0.25  # Human eye center X as fraction of screen width from each edge
_humanEyeSize = 33  # Human eye iris radius in pixels
_humanEyeColorIdx = 0  # Human eye color index (0=blue, 1=green, 2=hazel, 3=brown, 4=dark brown)
_eyeShape = 2  # Default eye shape (2 = ROUNDRECTANGLE)
# Eye color palettes (R,G,B tuples) - one color per eye shape
# Order: ELLIPSE, RECTANGLE, ROUNDRECTANGLE, X, SPIDER, HUMAN, CAT, HYPNO, ANIME
_eyeColorEllipse = (255, 80, 20)      # Orange
_eyeColorRectangle = (255, 255, 255)  # White
_eyeColorRoundRect = (10, 120, 255)   # Blue
_eyeColorX = (255, 0, 0)              # Red
_eyeColorSpider = (50, 220, 50)       # Green
_eyeColorHuman = (70, 130, 180)       # Steel blue (fallback, human uses palette)
_eyeColorCat = (255, 180, 0)          # Amber (cat eyes)
_eyeColorHypno = (180, 0, 255)        # Purple (hypno spiral)
_eyeColorAnime = (100, 180, 255)      # Light blue (anime style)
# Human eye color palette (5 options)
_humanEyeColorBlue = (70, 130, 180)       # Blue (steel blue)
_humanEyeColorGreen = (60, 140, 80)       # Green
_humanEyeColorHazel = (140, 110, 60)      # Hazel (golden brown-green)
_humanEyeColorBrown = (100, 60, 30)       # Brown
_humanEyeColorDarkBrown = (50, 30, 20)    # Dark brown
_eyeCrtMode = False                       # CRT scanline effect (persisted to config)
# Menu settings (saved to config)
_menuTheme = 0      # 0=MARS, 1=LCARS
_menuPalette = 0    # LCARS palette: 0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD

# Pounce move settings (persisted to controller.ini [pounce])
_pouncePrepMs = 400
_pounceRearMs = 250
_pounceLungeMs = 250
_pounceRecoverMs = 500
_pounceBack1Z = 55.0
_pounceBack2Z = 75.0
_pouncePushZ = -60.0
_pounceStrikeZ = 140.0
_pounceCrouchDy = 55.0
_pounceLiftDy = 110.0
_pounceFrontZ = 20.0


def _parse_ini_triplet_int(s):
    try:
        parts = str(s).strip().split('/')
        if len(parts) != 3:
            return None
        return [int(parts[0]), int(parts[1]), int(parts[2])]
    except Exception:
        return None
# Display thread settings
_displayThreadEnabled = True
_displayThreadHz = 15.0
_blinkFrameDivisor = 3  # Skip N-1 out of N frames during blink (1=no skip, 2=half rate, 3=third rate)
_engineeringLcars = True  # True = LCARS theme for engineering view, False = basic theme

# IMU sensor settings (BNO085)
_imuEnabled = True  # Set to False if IMU not connected
_imuHz = 100.0
_imuAddress = 0x4A  # Default BNO085 address (0x4B if jumper set)
_imuBus = 3  # I2C bus number (run 'i2cdetect -y N' to find your device)
_imuUseGameRotation = False  # True = faster startup, no magnetometer fusion
_imuEnableAccel = False
_imuEnableGyro = False
_imuEnableMag = False

# Body leveling settings (IMU-based Y/height correction)
_levelingEnabled = False  # Enable body leveling
_levelingGain = 1.0  # Scale factor for corrections (0-2)
_levelingMaxCorrMm = 30.0  # Max per-leg Y delta
_levelingFilterAlpha = 0.15  # EMA filter (lower = smoother)
_levelingPitchOffset = 0.0  # Pitch bias correction (deg)
_levelingRollOffset = 0.0  # Roll bias correction (deg)
_levelingTiltLimitDeg = 25.0  # Safety threshold (deg)
_levelingDebugCount = 0  # Debug output counter
# Motion lean settings (tilt into direction of travel)
_leanEnabled = False  # Enable motion lean
_leanMaxDeg = 7.0  # Max lean angle (7-10° typical)
_leanFilterAlpha = 0.1  # Smoothing for lean changes

# ToF sensor settings (VL53L5CX)
_tofEnabled = True  # Set to False if ToF not connected
_tofHz = 15.0  # Target update rate (max 15 Hz at 8x8, 60 Hz at 4x4)
_tofBus = 1  # I2C bus number (run 'i2cdetect -y N' to find device)
_tofResolution = 64  # 16 (4x4) or 64 (8x8) zones - library format
_tofSensors = [("front", 0x29)]  # List of (name, address) tuples
# ToF filter settings (smooths jittery readings)
# filter_mode: 'off' (raw/SLAM), 'light' (reject bad only), 'full' (EMA smooth)
_tofFilterMode = 'light'  # 'light' recommended for motion, 'full' for stationary
_tofFilterAlpha = 0.5  # EMA alpha for 'full' mode (higher = more responsive)
_tofFilterSigmaThresh = 20  # Reject high-sigma readings (mm)
_tofFilterOutlierMm = 100  # Jump threshold for outlier attenuation

# Autonomy / Behavior settings
_autonomyEnabled = False  # Master switch (keyboard 'a' toggles)
_autonomyObstacleAvoidance = True
_autonomyCliffDetection = True
_autonomyCaughtFootRecovery = True
_autonomyPatrol = False
_autonomyWallFollow = False  # Wall following behavior
_autonomyWallSide = 'left'  # Which side to follow: 'left' or 'right'
_autonomyWallDistMm = 200  # Target distance from wall
_autonomyStopDistMm = 150
_autonomySlowDistMm = 300
_autonomyCliffThresholdMm = 100
_autonomySnagErrorDeg = 15.0
_autonomySnagTimeoutMs = 500
_autonomyRecoveryLiftMm = 30.0
_autonomyMaxRecoveryAttempts = 3
_autonomyPatrolDurationS = 60.0
_autonomyTurnIntervalS = 10.0
_behaviorArbiter = None  # BehaviorArbiter instance
_lastAutonomyAction = None  # Last action for display

# Point cloud / SLAM settings
_pointcloudEnabled = False  # Master switch for point cloud server
_pointcloudPort = 8765  # WebSocket port
_pointcloudHttpPort = 8080  # HTTP port for web viewer
_pointcloudHz = 10.0  # Target streaming rate
_pointcloudAccumulate = True  # Accumulate points over time (vs live-only)
_pointcloudMaxPoints = 50000  # Max points to keep in accumulator
_pointcloudVoxelMm = 20.0  # Voxel size for deduplication
_pointcloudServer = None  # PointCloudServer instance

# Web dashboard / telemetry server settings
_dashboardEnabled = True  # Master switch for web dashboard
_dashboardPort = 8766  # WebSocket port for telemetry
_dashboardHz = 10.0  # Target telemetry streaming rate
_dashboardServer = None  # TelemetryServer instance

# Audio settings (Sabrent USB DAC + PAM8403 amp)
_audioEnabled = True  # Master switch for audio feedback
_audioVolume = 0.7  # Master volume 0.0-1.0
_audioDevice = "hw:2,0"  # ALSA device: Sabrent USB DAC
_audioSoundsDir = "assets/sounds"  # Sound files directory
_audio = None  # AudioManager instance

# TTS (Text-to-Speech) settings - supports espeak-ng or piper backends
_ttsEnabled = True  # Master switch for voice announcements
_ttsEngine = "espeak"  # TTS engine: "espeak" or "piper"
_ttsRate = 140  # Speech rate (espeak: wpm 80-200; piper: length_scale 0.5-2.0)
_ttsGainDb = 30  # Sox amplification gain in dB
_ttsVoice = "en-us"  # espeak voice (en-us, en-gb, Storm) or piper model name
_ttsPitch = 50  # Voice pitch (espeak: 0-99, 50=normal; piper: ignored)
_ttsPiperModel = os.path.expanduser("~/.local/share/piper/en_US-lessac-medium.onnx")  # Piper model path
_ttsLastSpeak = 0.0  # Anti-spam: last speech time
_ttsCooldownSec = 2.0  # Minimum seconds between speech
_ttsAvailable = False  # Set True if TTS engine is available
_ttsPiperAvailable = False  # Set True if piper is available
import subprocess
import shutil
if shutil.which('espeak-ng') and shutil.which('sox'):
    _ttsAvailable = True
if shutil.which('piper'):
    _ttsPiperAvailable = True

try:
    _cfg = configparser.ConfigParser()
    _cfg_path = os.path.join(os.path.dirname(__file__), 'controller.ini') if '__file__' in globals() else 'controller.ini'
    _cfg.read(_cfg_path)
    if 'ui' in _cfg and 'verbose' in _cfg['ui']:
        _verbose = _cfg.getboolean('ui', 'verbose', fallback=_verbose)
    if 'serial' in _cfg:
        _teensyErrorThreshold = _cfg.getint('serial', 'error_threshold', fallback=_teensyErrorThreshold)
    if 'telemetry' in _cfg:
        _telemetryGraceSeconds = _cfg.getfloat('telemetry', 'grace_s', fallback=_telemetryGraceSeconds)
        _telemetryRetrySeconds = _cfg.getfloat('telemetry', 'retry_s', fallback=_telemetryRetrySeconds)
    # Load timing config
    if 'timing' in _cfg:
        _loopTargetMs = _cfg.getfloat('timing', 'loop_target_ms', fallback=_loopTargetMs)
        _teensyLoopUs = _cfg.getint('timing', 'teensy_loop_us', fallback=_teensyLoopUs)
        _telemSyncFallbackMs = _cfg.getfloat('timing', 'telem_sync_fallback_ms', fallback=_telemSyncFallbackMs)
        _teensyReconnectBackoffSeconds = _cfg.getfloat('timing', 'teensy_reconnect_backoff_s', fallback=_teensyReconnectBackoffSeconds)
    # Load display config
    if 'display' in _cfg:
        _screenRefreshms = _cfg.getfloat('display', 'screen_refresh_ms', fallback=_screenRefreshms)
        _displayBrightness = _cfg.getint('display', 'brightness', fallback=_displayBrightness)
        _autoDisableS = _cfg.getfloat('display', 'auto_disable_s', fallback=_autoDisableS)
        _displayThreadEnabled = _cfg.getboolean('display', 'thread_enabled', fallback=_displayThreadEnabled)
        _displayThreadHz = _cfg.getfloat('display', 'thread_hz', fallback=_displayThreadHz)
        _blinkFrameDivisor = _cfg.getint('display', 'blink_frame_divisor', fallback=_blinkFrameDivisor)
        _startupDelayS = _cfg.getfloat('display', 'startup_delay_s', fallback=_startupDelayS)
        _engineeringLcars = _cfg.getboolean('display', 'engineering_lcars', fallback=_engineeringLcars)
        # Clamp startup delay to valid range (0-30s in 0.5s steps)
        _startupDelayS = max(0.0, min(30.0, round(_startupDelayS * 2) / 2))
    # Load menu settings
    if 'menu' in _cfg:
        _menuTheme = _cfg.getint('menu', 'theme', fallback=_menuTheme)
        _menuPalette = _cfg.getint('menu', 'palette', fallback=_menuPalette)

    # Load IMU sensor settings
    if 'imu' in _cfg:
        _imuEnabled = _cfg.getboolean('imu', 'enabled', fallback=_imuEnabled)
        _imuHz = _cfg.getfloat('imu', 'hz', fallback=_imuHz)
        _imuBus = _cfg.getint('imu', 'bus', fallback=_imuBus)
        # Accept hex string or int for address
        _imu_addr_str = _cfg.get('imu', 'address', fallback=None)
        if _imu_addr_str is not None:
            try:
                _imuAddress = int(_imu_addr_str, 0)
            except ValueError:
                pass
        _imuUseGameRotation = _cfg.getboolean('imu', 'use_game_rotation', fallback=_imuUseGameRotation)
        _imuEnableAccel = _cfg.getboolean('imu', 'enable_accel', fallback=_imuEnableAccel)
        _imuEnableGyro = _cfg.getboolean('imu', 'enable_gyro', fallback=_imuEnableGyro)
        _imuEnableMag = _cfg.getboolean('imu', 'enable_mag', fallback=_imuEnableMag)

    # Load body leveling settings
    if 'leveling' in _cfg:
        _levelingEnabled = _cfg.getboolean('leveling', 'enabled', fallback=_levelingEnabled)
        _levelingGain = _cfg.getfloat('leveling', 'gain', fallback=_levelingGain)
        _levelingMaxCorrMm = _cfg.getfloat('leveling', 'max_correction_mm', fallback=_levelingMaxCorrMm)
        _levelingFilterAlpha = _cfg.getfloat('leveling', 'filter_alpha', fallback=_levelingFilterAlpha)
        _levelingPitchOffset = _cfg.getfloat('leveling', 'pitch_offset', fallback=_levelingPitchOffset)
        _levelingRollOffset = _cfg.getfloat('leveling', 'roll_offset', fallback=_levelingRollOffset)
        _levelingTiltLimitDeg = _cfg.getfloat('leveling', 'tilt_limit_deg', fallback=_levelingTiltLimitDeg)
        # Motion lean settings
        _leanEnabled = _cfg.getboolean('leveling', 'lean_enabled', fallback=_leanEnabled)
        _leanMaxDeg = _cfg.getfloat('leveling', 'lean_max_deg', fallback=_leanMaxDeg)
        _leanFilterAlpha = _cfg.getfloat('leveling', 'lean_filter_alpha', fallback=_leanFilterAlpha)

    # Load ToF sensor settings
    if 'tof' in _cfg:
        _tofEnabled = _cfg.getboolean('tof', 'enabled', fallback=_tofEnabled)
        _tofHz = _cfg.getfloat('tof', 'hz', fallback=_tofHz)
        _tofBus = _cfg.getint('tof', 'bus', fallback=_tofBus)
        _tof_res_cfg = _cfg.getint('tof', 'resolution', fallback=8)
        # Convert user-friendly 4/8 to library expected 16/64
        _tofResolution = 64 if _tof_res_cfg >= 8 else 16
        # Parse sensors list: "front=0x29,left=0x30" format
        _tof_sensors_str = _cfg.get('tof', 'sensors', fallback=None)
        if _tof_sensors_str:
            _tofSensors = []
            for pair in _tof_sensors_str.split(','):
                pair = pair.strip()
                if '=' in pair:
                    name, addr_str = pair.split('=', 1)
                    try:
                        addr = int(addr_str.strip(), 0)
                        _tofSensors.append((name.strip(), addr))
                    except ValueError:
                        pass
        # Filter settings
        _tofFilterMode = _cfg.get('tof', 'filter_mode', fallback=_tofFilterMode)
        _tofFilterAlpha = _cfg.getfloat('tof', 'filter_alpha', fallback=_tofFilterAlpha)
        _tofFilterSigmaThresh = _cfg.getint('tof', 'filter_sigma_threshold', fallback=_tofFilterSigmaThresh)
        _tofFilterOutlierMm = _cfg.getint('tof', 'filter_outlier_mm', fallback=_tofFilterOutlierMm)

    # Load optional PID/IMP/EST defaults
    if 'pid' in _cfg:
        _pid_ini['enabled'] = _cfg.getboolean('pid', 'enabled', fallback=_pid_ini['enabled'])
        _pid_ini['mode'] = _cfg.get('pid', 'mode', fallback=_pid_ini['mode'])
        _pid_ini['shadow_hz'] = _cfg.getint('pid', 'shadow_hz', fallback=_pid_ini['shadow_hz'])
        _pid_ini['kp'] = _parse_ini_triplet_int(_cfg.get('pid', 'kp', fallback=None))
        _pid_ini['ki'] = _parse_ini_triplet_int(_cfg.get('pid', 'ki', fallback=None))
        _pid_ini['kd'] = _parse_ini_triplet_int(_cfg.get('pid', 'kd', fallback=None))
        _pid_ini['kdalph'] = _parse_ini_triplet_int(_cfg.get('pid', 'kdalph', fallback=None))

    if 'imp' in _cfg:
        _imp_ini['enabled'] = _cfg.getboolean('imp', 'enabled', fallback=_imp_ini['enabled'])
        _imp_ini['mode'] = _cfg.get('imp', 'mode', fallback=_imp_ini['mode'])
        _imp_ini['scale'] = _cfg.getint('imp', 'scale', fallback=_imp_ini['scale'])
        _imp_ini['jspring'] = _parse_ini_triplet_int(_cfg.get('imp', 'jspring', fallback=None))
        _imp_ini['jdamp'] = _parse_ini_triplet_int(_cfg.get('imp', 'jdamp', fallback=None))
        _imp_ini['cspring'] = _parse_ini_triplet_int(_cfg.get('imp', 'cspring', fallback=None))
        _imp_ini['cdamp'] = _parse_ini_triplet_int(_cfg.get('imp', 'cdamp', fallback=None))
        _imp_ini['jdb_cd'] = _cfg.getint('imp', 'jdb_cd', fallback=_imp_ini['jdb_cd'])
        try:
            _imp_ini['cdb_mm'] = _cfg.getfloat('imp', 'cdb_mm', fallback=_imp_ini['cdb_mm'])
        except Exception:
            pass

    if 'est' in _cfg:
        _est_ini['cmd_alpha_milli'] = _cfg.getint('est', 'cmd_alpha_milli', fallback=_est_ini['cmd_alpha_milli'])
        _est_ini['meas_alpha_milli'] = _cfg.getint('est', 'meas_alpha_milli', fallback=_est_ini['meas_alpha_milli'])
        _est_ini['meas_vel_alpha_milli'] = _cfg.getint('est', 'meas_vel_alpha_milli', fallback=_est_ini['meas_vel_alpha_milli'])

    # Initialize dashboard-editable PID/IMP/EST variables from parsed config
    # (Variables already defined at module level, no global needed here)
    if _pid_ini.get('enabled') is not None:
        _pidEnabled = bool(_pid_ini['enabled'])
    if _pid_ini.get('kp'):
        _pidKp = float(_pid_ini['kp'][0]) if isinstance(_pid_ini['kp'], list) else 0.0
    if _pid_ini.get('ki'):
        _pidKi = float(_pid_ini['ki'][0]) if isinstance(_pid_ini['ki'], list) else 0.0
    if _pid_ini.get('kd'):
        _pidKd = float(_pid_ini['kd'][0]) if isinstance(_pid_ini['kd'], list) else 0.0
    
    if _imp_ini.get('enabled') is not None:
        _impEnabled = bool(_imp_ini['enabled'])
    if _imp_ini.get('scale') is not None:
        _impKp = float(_imp_ini['scale'])
    if _imp_ini.get('jdb_cd') is not None:
        _impKd = float(_imp_ini['jdb_cd'])
    
    if _est_ini.get('cmd_alpha_milli') is not None:
        _estAlpha = _est_ini['cmd_alpha_milli'] / 1000.0
    if _est_ini.get('meas_vel_alpha_milli') is not None:
        _estVelDecay = _est_ini['meas_vel_alpha_milli'] / 1000.0

    # Load safety display thresholds (Pi-side visualization)
    if 'safety_display' in _cfg:
        _safetyDisplayVoltMin = _cfg.getfloat('safety_display', 'volt_min', fallback=_safetyDisplayVoltMin)
        _safetyDisplayVoltWarn = _cfg.getfloat('safety_display', 'volt_warn', fallback=_safetyDisplayVoltWarn)
        _safetyDisplayVoltMax = _cfg.getfloat('safety_display', 'volt_max', fallback=_safetyDisplayVoltMax)
        _safetyDisplayTempMin = _cfg.getfloat('safety_display', 'temp_min', fallback=_safetyDisplayTempMin)
        _safetyDisplayTempMax = _cfg.getfloat('safety_display', 'temp_max', fallback=_safetyDisplayTempMax)
        _showBatteryIcon = _cfg.getboolean('safety_display', 'show_battery_icon', fallback=_showBatteryIcon)

    # Load low battery protection settings
    if 'low_battery' in _cfg:
        _lowBatteryEnabled = _cfg.getboolean('low_battery', 'enabled', fallback=_lowBatteryEnabled)
        _lowBatteryVoltCritical = _cfg.getfloat('low_battery', 'volt_critical', fallback=_lowBatteryVoltCritical)
        _lowBatteryRecoveryVolt = _cfg.getfloat('low_battery', 'volt_recovery', fallback=_lowBatteryRecoveryVolt)
        _lowBatteryFilterAlpha = _cfg.getfloat('low_battery', 'filter_alpha', fallback=_lowBatteryFilterAlpha)

    # Load behavior/autonomy settings
    if 'behavior' in _cfg:
        _autonomyEnabled = _cfg.getboolean('behavior', 'enabled', fallback=_autonomyEnabled)
        _autonomyObstacleAvoidance = _cfg.getboolean('behavior', 'obstacle_avoidance', fallback=_autonomyObstacleAvoidance)
        _autonomyCliffDetection = _cfg.getboolean('behavior', 'cliff_detection', fallback=_autonomyCliffDetection)
        _autonomyCaughtFootRecovery = _cfg.getboolean('behavior', 'caught_foot_recovery', fallback=_autonomyCaughtFootRecovery)
        _autonomyPatrol = _cfg.getboolean('behavior', 'patrol', fallback=_autonomyPatrol)
        _autonomyWallFollow = _cfg.getboolean('behavior', 'wall_follow', fallback=_autonomyWallFollow)
        _autonomyWallSide = _cfg.get('behavior', 'wall_side', fallback=_autonomyWallSide).lower().strip()
        _autonomyWallDistMm = _cfg.getint('behavior', 'wall_distance_mm', fallback=_autonomyWallDistMm)
        _autonomyStopDistMm = _cfg.getint('behavior', 'stop_distance_mm', fallback=_autonomyStopDistMm)
        _autonomySlowDistMm = _cfg.getint('behavior', 'slow_distance_mm', fallback=_autonomySlowDistMm)
        _autonomyCliffThresholdMm = _cfg.getint('behavior', 'cliff_threshold_mm', fallback=_autonomyCliffThresholdMm)
        _autonomySnagErrorDeg = _cfg.getfloat('behavior', 'snag_position_error_deg', fallback=_autonomySnagErrorDeg)
        _autonomySnagTimeoutMs = _cfg.getint('behavior', 'snag_timeout_ms', fallback=_autonomySnagTimeoutMs)
        _autonomyRecoveryLiftMm = _cfg.getfloat('behavior', 'recovery_lift_mm', fallback=_autonomyRecoveryLiftMm)
        _autonomyMaxRecoveryAttempts = _cfg.getint('behavior', 'max_recovery_attempts', fallback=_autonomyMaxRecoveryAttempts)
        _autonomyPatrolDurationS = _cfg.getfloat('behavior', 'patrol_duration_s', fallback=_autonomyPatrolDurationS)
        _autonomyTurnIntervalS = _cfg.getfloat('behavior', 'turn_interval_s', fallback=_autonomyTurnIntervalS)

    # Load point cloud / SLAM settings
    if 'pointcloud' in _cfg:
        _pointcloudEnabled = _cfg.getboolean('pointcloud', 'enabled', fallback=_pointcloudEnabled)
        _pointcloudPort = _cfg.getint('pointcloud', 'port', fallback=_pointcloudPort)
        _pointcloudHttpPort = _cfg.getint('pointcloud', 'http_port', fallback=_pointcloudHttpPort)
        _pointcloudHz = _cfg.getfloat('pointcloud', 'stream_hz', fallback=_pointcloudHz)
        _pointcloudAccumulate = _cfg.getboolean('pointcloud', 'accumulate', fallback=_pointcloudAccumulate)
        _pointcloudMaxPoints = _cfg.getint('pointcloud', 'max_points', fallback=_pointcloudMaxPoints)
        _pointcloudVoxelMm = _cfg.getfloat('pointcloud', 'voxel_mm', fallback=_pointcloudVoxelMm)

    # Load web dashboard settings
    if 'dashboard' in _cfg:
        _dashboardEnabled = _cfg.getboolean('dashboard', 'enabled', fallback=_dashboardEnabled)
        _dashboardPort = _cfg.getint('dashboard', 'port', fallback=_dashboardPort)
        _dashboardHz = _cfg.getfloat('dashboard', 'stream_hz', fallback=_dashboardHz)

    # Load audio settings
    if 'audio' in _cfg:
        _audioEnabled = _cfg.getboolean('audio', 'enabled', fallback=_audioEnabled)
        _audioVolume = _cfg.getfloat('audio', 'volume', fallback=_audioVolume)
        _audioDevice = _cfg.get('audio', 'device', fallback=_audioDevice)
        _audioSoundsDir = _cfg.get('audio', 'sounds_dir', fallback=_audioSoundsDir)

    # Load TTS (text-to-speech) settings
    if 'tts' in _cfg:
        _ttsEnabled = _cfg.getboolean('tts', 'enabled', fallback=_ttsEnabled)
        _ttsEngine = _cfg.get('tts', 'engine', fallback=_ttsEngine).lower().strip()
        _ttsRate = _cfg.getint('tts', 'rate', fallback=_ttsRate)
        _ttsGainDb = _cfg.getint('tts', 'gain_db', fallback=_ttsGainDb)
        _ttsVoice = _cfg.get('tts', 'voice', fallback=_ttsVoice)
        _ttsPitch = _cfg.getint('tts', 'pitch', fallback=_ttsPitch)
        _ttsPiperModel = os.path.expanduser(_cfg.get('tts', 'piper_model', fallback=_ttsPiperModel))
        _ttsCooldownSec = _cfg.getfloat('tts', 'cooldown_sec', fallback=_ttsCooldownSec)

    # Merge controller.ini defaults into live PID/IMP/EST menu state (until firmware LIST overwrites)
    try:
        if _pid_ini.get('enabled') is not None:
            _pid_state['enabled'] = bool(_pid_ini['enabled'])
        if _pid_ini.get('mode'):
            _pid_state['mode'] = str(_pid_ini['mode']).strip().lower()
        if _pid_ini.get('shadow_hz') is not None:
            _pid_state['shadow_hz'] = int(_pid_ini['shadow_hz'])
        for k in ('kp', 'ki', 'kd', 'kdalph'):
            if _pid_ini.get(k) is not None:
                _pid_state[k] = list(_pid_ini[k])

        if _imp_ini.get('enabled') is not None:
            _imp_state['enabled'] = bool(_imp_ini['enabled'])
        if _imp_ini.get('mode'):
            _imp_state['mode'] = str(_imp_ini['mode']).strip().lower()
        if _imp_ini.get('scale') is not None:
            _imp_state['scale'] = int(_imp_ini['scale'])
        for k in ('jspring', 'jdamp', 'cspring', 'cdamp'):
            if _imp_ini.get(k) is not None:
                _imp_state[k] = list(_imp_ini[k])
        if _imp_ini.get('jdb_cd') is not None:
            _imp_state['jdb_cd'] = int(_imp_ini['jdb_cd'])
        if _imp_ini.get('cdb_mm') is not None:
            _imp_state['cdb_mm'] = float(_imp_ini['cdb_mm'])

        for k in ('cmd_alpha_milli', 'meas_alpha_milli', 'meas_vel_alpha_milli'):
            if _est_ini.get(k) is not None:
                _est_state[k] = int(_est_ini[k])
    except Exception:
        pass
    # Load saved gait parameters
    _savedGaitWidthMm = 100.0
    _savedGaitLiftMm = 60.0
    _gaitWidthMinMm = 50.0
    _gaitWidthMaxMm = 175.0
    _gaitLiftMinMm = 20.0
    _gaitLiftMaxMm = 100.0
    if 'gait' in _cfg:
        _savedGaitWidthMm = _cfg.getfloat('gait', 'width_mm', fallback=100.0)
        _savedGaitLiftMm = _cfg.getfloat('gait', 'lift_mm', fallback=60.0)
        _gaitWidthMinMm = _cfg.getfloat('gait', 'width_min_mm', fallback=50.0)
        _gaitWidthMaxMm = _cfg.getfloat('gait', 'width_max_mm', fallback=175.0)
        _gaitLiftMinMm = _cfg.getfloat('gait', 'lift_min_mm', fallback=20.0)
        _gaitLiftMaxMm = _cfg.getfloat('gait', 'lift_max_mm', fallback=100.0)
        _gaitCycleMs = _cfg.getint('gait', 'cycle_ms', fallback=_gaitCycleMs)
        _gaitStepLenMm = _cfg.getfloat('gait', 'step_len_mm', fallback=_gaitStepLenMm)
        _gaitBaseYMm = _cfg.getfloat('gait', 'base_y_mm', fallback=_gaitBaseYMm)
        _gaitMaxStepLenMm = _cfg.getfloat('gait', 'max_step_len_mm', fallback=_gaitMaxStepLenMm)
        _gaitOverlapPct = _cfg.getfloat('gait', 'overlap_pct', fallback=_gaitOverlapPct)
        _gaitSmoothingAlpha = _cfg.getfloat('gait', 'smoothing_alpha', fallback=_gaitSmoothingAlpha)
        _gaitSendDivisor = _cfg.getint('gait', 'send_divisor', fallback=_gaitSendDivisor)
        _layer2Divisor = _cfg.getint('gait', 'layer2_divisor', fallback=_layer2Divisor)
        _feetToleranceMm = _cfg.getfloat('gait', 'feet_tolerance_mm', fallback=_feetToleranceMm)
        _feetMaxSkipMs = _cfg.getfloat('gait', 'feet_max_skip_ms', fallback=_feetMaxSkipMs)
        _cmdThrottleMs = _cfg.getfloat('gait', 'cmd_throttle_ms', fallback=_cmdThrottleMs)
        _gaitTurnMaxDegS = _cfg.getfloat('gait', 'turn_max_deg_s', fallback=_gaitTurnMaxDegS)
        # Bezier curve shape
        _bezierP1Height = _cfg.getfloat('gait', 'bezier_p1_height', fallback=_bezierP1Height)
        _bezierP1Overshoot = _cfg.getfloat('gait', 'bezier_p1_overshoot', fallback=_bezierP1Overshoot)
        _bezierP2Height = _cfg.getfloat('gait', 'bezier_p2_height', fallback=_bezierP2Height)
        _bezierP3Height = _cfg.getfloat('gait', 'bezier_p3_height', fallback=_bezierP3Height)
        _bezierP3Overshoot = _cfg.getfloat('gait', 'bezier_p3_overshoot', fallback=_bezierP3Overshoot)

    # Load pounce settings
    if 'pounce' in _cfg:
        _pouncePrepMs = _cfg.getint('pounce', 'prep_ms', fallback=_pouncePrepMs)
        _pounceRearMs = _cfg.getint('pounce', 'rear_ms', fallback=_pounceRearMs)
        _pounceLungeMs = _cfg.getint('pounce', 'lunge_ms', fallback=_pounceLungeMs)
        _pounceRecoverMs = _cfg.getint('pounce', 'recover_ms', fallback=_pounceRecoverMs)
        _pounceBack1Z = _cfg.getfloat('pounce', 'back1_z_mm', fallback=_pounceBack1Z)
        _pounceBack2Z = _cfg.getfloat('pounce', 'back2_z_mm', fallback=_pounceBack2Z)
        _pouncePushZ = _cfg.getfloat('pounce', 'push_z_mm', fallback=_pouncePushZ)
        _pounceStrikeZ = _cfg.getfloat('pounce', 'strike_z_mm', fallback=_pounceStrikeZ)
        _pounceCrouchDy = _cfg.getfloat('pounce', 'crouch_dy_mm', fallback=_pounceCrouchDy)
        _pounceLiftDy = _cfg.getfloat('pounce', 'lift_dy_mm', fallback=_pounceLiftDy)
        _pounceFrontZ = _cfg.getfloat('pounce', 'front_z_mm', fallback=_pounceFrontZ)

    # Load eye settings
    if 'eyes' in _cfg:
        _eyeSpacingOffset = _cfg.getint('eyes', 'spacing_offset', fallback=_eyeSpacingOffset)
        _eyeCenterOffset = _cfg.getint('eyes', 'center_offset', fallback=_eyeCenterOffset)
        _eyeVerticalOffset = _cfg.getint('eyes', 'vertical_offset', fallback=_eyeVerticalOffset)
        _eyeEyelidAngle = _cfg.getint('eyes', 'eyelid_angle', fallback=_eyeEyelidAngle)
        _eyeBlinkPercentStep = _cfg.getfloat('eyes', 'blink_percent_step', fallback=_eyeBlinkPercentStep)
        _eyeRotation = _cfg.getint('eyes', 'rotation', fallback=_eyeRotation)
        _eyeSizeX = _cfg.getint('eyes', 'size_x', fallback=_eyeSizeX)
        _eyeSizeY = _cfg.getint('eyes', 'size_y', fallback=_eyeSizeY)
        _eyeLookRangeX = _cfg.getfloat('eyes', 'look_range_x', fallback=_eyeLookRangeX)
        _eyeLookRangeY = _cfg.getfloat('eyes', 'look_range_y', fallback=_eyeLookRangeY)
        _humanEyeSpacingPct = _cfg.getfloat('eyes', 'human_eye_spacing_pct', fallback=_humanEyeSpacingPct)
        _humanEyeSize = _cfg.getint('eyes', 'human_eye_size', fallback=_humanEyeSize)
        _humanEyeColorIdx = _cfg.getint('eyes', 'human_eye_color', fallback=_humanEyeColorIdx)
        _eyeShape = _cfg.getint('eyes', 'shape', fallback=_eyeShape)
        _eyeCrtMode = _cfg.getboolean('eyes', 'crt_mode', fallback=_eyeCrtMode)
        # Helper to parse R,G,B string to tuple
        def parse_rgb(s, default):
            try:
                parts = [int(x.strip()) for x in s.split(',')]
                if len(parts) == 3:
                    return tuple(max(0, min(255, p)) for p in parts)
            except:
                pass
            return default
        # Load eye color palette
        _eyeColorEllipse = parse_rgb(_cfg.get('eyes', 'color_ellipse', fallback='255,80,20'), _eyeColorEllipse)
        _eyeColorRectangle = parse_rgb(_cfg.get('eyes', 'color_rectangle', fallback='255,255,255'), _eyeColorRectangle)
        _eyeColorRoundRect = parse_rgb(_cfg.get('eyes', 'color_roundrect', fallback='10,120,255'), _eyeColorRoundRect)
        _eyeColorX = parse_rgb(_cfg.get('eyes', 'color_x', fallback='255,0,0'), _eyeColorX)
        _eyeColorSpider = parse_rgb(_cfg.get('eyes', 'color_spider', fallback='50,220,50'), _eyeColorSpider)
        _eyeColorHuman = parse_rgb(_cfg.get('eyes', 'color_human', fallback='70,130,180'), _eyeColorHuman)
        _eyeColorCat = parse_rgb(_cfg.get('eyes', 'color_cat', fallback='255,180,0'), _eyeColorCat)
        _eyeColorHypno = parse_rgb(_cfg.get('eyes', 'color_hypno', fallback='180,0,255'), _eyeColorHypno)
        _eyeColorAnime = parse_rgb(_cfg.get('eyes', 'color_anime', fallback='100,180,255'), _eyeColorAnime)
        # Load human eye color palette
        _humanEyeColorBlue = parse_rgb(_cfg.get('eyes', 'human_color_blue', fallback='70,130,180'), _humanEyeColorBlue)
        _humanEyeColorGreen = parse_rgb(_cfg.get('eyes', 'human_color_green', fallback='60,140,80'), _humanEyeColorGreen)
        _humanEyeColorHazel = parse_rgb(_cfg.get('eyes', 'human_color_hazel', fallback='140,110,60'), _humanEyeColorHazel)
        _humanEyeColorBrown = parse_rgb(_cfg.get('eyes', 'human_color_brown', fallback='100,60,30'), _humanEyeColorBrown)
        _humanEyeColorDarkBrown = parse_rgb(_cfg.get('eyes', 'human_color_darkbrown', fallback='50,30,20'), _humanEyeColorDarkBrown)
    # Share config state with config_manager module for save functions
    set_config_state(_cfg, _cfg_path)
except (configparser.Error, ValueError, KeyError) as e:
    print(f"Config load error (using defaults): {e}", end="\r\n")
    _savedGaitWidthMm = 100.0
    _savedGaitLiftMm = 60.0
    _gaitWidthMinMm = 50.0
    _gaitWidthMaxMm = 175.0
    _gaitLiftMinMm = 20.0
    _gaitLiftMaxMm = 100.0

# Save functions imported from config_manager module (Phase 2 modularization)
# save_gait_settings, save_pounce_settings, save_eye_shape, save_human_eye_settings,
# save_eye_center_offset, save_eye_vertical_offset, save_menu_settings are imported
# directly. save_pid_settings, save_imp_settings, save_est_settings require state dict param.

# initialize the display and start logging to it
#_disp = LCD_1inch9.LCD_1inch9()
_disp = st7789.st7789()
#_menu._disp = _disp  # set the display for the menu

#_disp.Init()
_disp.clear()
_disp.bl_DutyCycle(_displayBrightness) # turn the backlight to full on
_backGroundImage = Image.new("RGB", (_disp.width, _disp.height), "BLACK")  # create a blank image for the background
#_backGroundImage = Image.open("/home/starter/Downloads/LCD_Module_RPI_code/RaspberryPi/python/pic/LCD_1inch9_1.jpg")  # create a blank image for the background
#_backGroundImage = Image.open("/home/starter/OneDrive/Projects/Robots/hexapod-main/Illustrations/Full Body - Small  2.PNG")  # create a blank image for the background
#_backGroundImage = _backGroundImage.convert("RGB")  # convert the image to RGB                                                                                                                          
#r,g,b = _backGroundImage.split()  # split the image into RGB channels
#_backGroundImage = Image.merge("RGB", (g,b,r))  # merge the channels back together
#_backGroundImage = _backGroundImage.rotate(270, expand=True)  # rotate the image to fit the display
#_backGroundImage = _backGroundImage.convert("BGR")  # convert the image to RGB
#_backGroundImage = _backGroundImage.resize((_disp.width, _disp.height), Image.Resampling.LANCZOS)  # resize the image to fit the display

# Create startup splash with scrolling log
_startupSplash = StartupSplash(_disp, FW_VERSION_BANNER, CONTROLLER_VERSION, CONTROLLER_BUILD)
_startupSplash.log("MARS - Modular Autonomous Robotic System", "CYAN")

# Startup banner (printed to stdout for logs/USB serial capture)
print("MARS - Modular Autonomous Robotic System", end="\r\n")
print(f"Firmware {FW_VERSION_BANNER} · Controller {CONTROLLER_VERSION}/b{CONTROLLER_BUILD}", end="\r\n")

# Initialize the touch screen
_startupSplash.log("Initializing touch screen...", "WHITE")
_touch = cst816d.cst816d()
_startupSplash.log("Touch screen OK", "GREEN")

# Create the menu object (legacy menu, kept for compatibility)
_startupSplash.log("Creating menu systems...", "WHITE")
_menu = GUIMenu(_touch)

# Create new MARS menu system
_marsMenu = MarsMenu(_touch)
# Audio callback will be set later after audio_menu_callback is defined
_startupSplash.log("Menu systems OK", "GREEN")

# create and initialize the SimpleEyes object
_startupSplash.log("Initializing eye display...", "WHITE")
_eyes = SimpleEyes((_backGroundImage.height, _backGroundImage.width), eye_color=(10,120,255))
# Eye colors from config: ELLIPSE, RECTANGLE, ROUNDRECTANGLE, X, SPIDER, HUMAN, CAT, HYPNO, ANIME
_eyeColors = [_eyeColorEllipse, _eyeColorRectangle, _eyeColorRoundRect, _eyeColorX, _eyeColorSpider, _eyeColorHuman, _eyeColorCat, _eyeColorHypno, _eyeColorAnime]
# Human eye color palette from config
_humanEyeColorPalette = [_humanEyeColorBlue, _humanEyeColorGreen, _humanEyeColorHazel, _humanEyeColorBrown, _humanEyeColorDarkBrown]
_eyes.left_shape = _eyeShape
_eyes.right_shape = _eyeShape
_eyes.eye_color = _eyeColors[_eyes.left_shape] if _eyes.left_shape < len(_eyeColors) else _eyeColors[0]
_eyes.cat_eye_color = _eyeColorCat
_eyes.hypno_color = _eyeColorHypno
_eyes.anime_eye_color = _eyeColorAnime
_eyes.eye_size = (_eyeSizeX, _eyeSizeY)
_eyes.rotation = _eyeRotation
_eyes.eye_spacing_offset = _eyeSpacingOffset
_eyes.eye_center_offset = _eyeCenterOffset
_eyes.eye_vertical_offset = _eyeVerticalOffset
_eyes.eyelid_angle = _eyeEyelidAngle
_eyes.blink_percent_step = _eyeBlinkPercentStep
_eyes.human_eye_spacing_pct = _humanEyeSpacingPct
_eyes.human_eye_size = _humanEyeSize
_eyes.human_eye_color_idx = _humanEyeColorIdx
_eyes.human_eye_colors = _humanEyeColorPalette  # Pass loaded palette to SimpleEyes
_eyes.crt_mode = _eyeCrtMode  # Set CRT mode from config
_eyes.update(force_update=True)  # force the initial update to draw the eyes
_startupSplash.log("Eye display OK", "GREEN")

# Don't switch to eyes yet - keep splash showing during remaining startup
# UpdateDisplay(_disp, _eyes.display_image, _menu.image, _servo, _legs, _state, _mirrorDisplay, _menuState)

# --- Setup MARS menu callbacks and sync initial values ---
def _setup_mars_menu():
    """Configure MARS menu callbacks and sync with current state."""
    global _marsMenu, _eyes, _displayBrightness, _verbose, _mirrorDisplay
    
    # === EYES callbacks ===
    def on_eye_style_change(val):
        global _eyes, _eyeColors, _forceDisplayUpdate
        _eyes.left_shape = val
        _eyes.right_shape = val
        _eyes.eye_color = _eyeColors[val] if val < len(_eyeColors) else _eyeColors[0]
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
        save_eye_shape(val)
    
    def on_human_color_change(val):
        global _eyes, _forceDisplayUpdate
        _eyes.human_eye_color_idx = val
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
    
    def on_eye_size_change(val):
        global _eyes, _forceDisplayUpdate
        _eyes.human_eye_size = val
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
    
    def on_eye_spacing_change(val):
        global _eyes, _forceDisplayUpdate
        _eyes.human_eye_spacing_pct = val / 100.0
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
    
    def on_crt_change(val):
        global _eyes, _forceDisplayUpdate, _eyeCrtMode
        _eyeCrtMode = (val == 1)
        _eyes.crt_mode = _eyeCrtMode
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
        save_eye_crt_mode(_eyeCrtMode)
    
    def on_eye_vcenter_change(val):
        global _eyes, _eyeVerticalOffset, _forceDisplayUpdate, _displayThread
        _eyeVerticalOffset = val
        _eyes.eye_vertical_offset = val
        if _displayThread is not None:
            _displayThread.set_base_vertical_offset(val)
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
        save_eye_vertical_offset(val)
    
    _marsMenu.set_callback(MenuCategory.EYES, "Style", "on_change", on_eye_style_change)
    _marsMenu.set_callback(MenuCategory.EYES, "Human Color", "on_change", on_human_color_change)
    _marsMenu.set_callback(MenuCategory.EYES, "Size", "on_change", on_eye_size_change)
    _marsMenu.set_callback(MenuCategory.EYES, "V Center", "on_change", on_eye_vcenter_change)
    _marsMenu.set_callback(MenuCategory.EYES, "Spacing", "on_change", on_eye_spacing_change)
    _marsMenu.set_callback(MenuCategory.EYES, "CRT Effect", "on_change", on_crt_change)
    
    # === SYSTEM callbacks ===
    def on_brightness_change(val):
        global _disp, _displayBrightness
        _displayBrightness = val
        _disp.bl_DutyCycle(val)
    
    def on_verbose_change(val):
        global _verbose
        _verbose = (val == 1)
    
    def on_mirror_change(val):
        global _mirrorDisplay, _forceDisplayUpdate
        _mirrorDisplay = (val == 1)
        _forceDisplayUpdate = True
    
    def on_theme_change(val):
        global _marsMenu
        _marsMenu.theme = val
        save_menu_settings(theme=val)
    
    def on_palette_change(val):
        global _marsMenu, _displayThread
        _marsMenu.lcars_palette = val
        if _displayThread is not None:
            _displayThread.set_lcars_palette(val)
        save_menu_settings(palette=val)
    
    def on_save_all():
        # Save current settings to config
        save_eye_shape(_eyes.left_shape)
        if _verbose:
            print("Settings saved", end="\r\n")
    
    def on_shutdown():
        global _run
        _run = False
        if _verbose:
            print("Shutdown requested via menu", end="\r\n")
    
    def on_auto_disable_change(val):
        global _autoDisableS
        _autoDisableS = float(val)
        if _verbose:
            print(f"Auto-disable timeout set to {val}s", end="\r\n")

    def _kick_list_poll(which: str):
        global _pid_list_next_at, _imp_list_next_at, _est_list_next_at
        now = time.time()
        if which == 'PID':
            _pid_list_next_at = min(_pid_list_next_at if _pid_list_next_at > 0.0 else now, now + 0.05)
        elif which == 'IMP':
            _imp_list_next_at = min(_imp_list_next_at if _imp_list_next_at > 0.0 else now, now + 0.05)
        elif which == 'EST':
            _est_list_next_at = min(_est_list_next_at if _est_list_next_at > 0.0 else now, now + 0.05)

    # === PID callbacks ===
    def on_pid_enabled_change(val):
        try:
            _pid_state['enabled'] = (val == 1)
            save_pid_settings(_pid_state)
            send_cmd(b'PID ENABLE' if val == 1 else b'PID DISABLE', force=True)
            _kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_mode_change(val):
        try:
            _pid_state['mode'] = 'active' if val == 0 else 'shadow'
            save_pid_settings(_pid_state)
            send_cmd(b'PID MODE ACTIVE' if val == 0 else b'PID MODE SHADOW', force=True)
            _kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_shadow_hz_change(val):
        try:
            _pid_state['shadow_hz'] = int(val)
            save_pid_settings(_pid_state)
            send_cmd(f"PID SHADOW_RATE {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('PID')
        except Exception:
            pass

    def _make_pid_gain_cb(kind: str, joint: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = _pid_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                _pid_state[key] = arr
                save_pid_settings(_pid_state)
                send_cmd(f"PID {kind} {joint} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('PID')
            except Exception:
                pass
        return _cb

    def _make_pid_kdalpha_cb(joint: str):
        def _cb(val):
            try:
                arr = _pid_state.get('kdalph')
                if arr is None or len(arr) != 3:
                    arr = [200, 200, 200]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                _pid_state['kdalph'] = arr
                save_pid_settings(_pid_state)
                send_cmd(f"PID KDALPHA {joint} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('PID')
            except Exception:
                pass
        return _cb

    # === IMP callbacks ===
    def on_imp_enabled_change(val):
        try:
            _imp_state['enabled'] = (val == 1)
            save_imp_settings(_imp_state)
            send_cmd(b'IMP ENABLE' if val == 1 else b'IMP DISABLE', force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_mode_change(val):
        try:
            _imp_state['mode'] = 'joint' if val == 1 else ('cart' if val == 2 else 'off')
            save_imp_settings(_imp_state)
            if val == 1:
                cmd = b'IMP MODE JOINT'
            elif val == 2:
                cmd = b'IMP MODE CART'
            else:
                cmd = b'IMP MODE OFF'
            send_cmd(cmd, force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_scale_change(val):
        try:
            _imp_state['scale'] = int(val)
            save_imp_settings(_imp_state)
            send_cmd(f"IMP SCALE {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def _make_imp_joint_gain_cb(kind: str, joint: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = _imp_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if joint == 'COXA' else (1 if joint == 'FEMUR' else 2)
                arr[idx] = int(val)
                _imp_state[key] = arr
                save_imp_settings(_imp_state)
                send_cmd(f"IMP {kind} {joint} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('IMP')
            except Exception:
                pass
        return _cb

    def _make_imp_cart_gain_cb(kind: str, axis: str):
        def _cb(val):
            try:
                key = kind.lower()
                arr = _imp_state.get(key)
                if arr is None or len(arr) != 3:
                    arr = [0, 0, 0]
                idx = 0 if axis == 'X' else (1 if axis == 'Y' else 2)
                arr[idx] = int(val)
                _imp_state[key] = arr
                save_imp_settings(_imp_state)
                send_cmd(f"IMP {kind} {axis} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('IMP')
            except Exception:
                pass
        return _cb

    def on_imp_jdb_change(val):
        try:
            _imp_state['jdb_cd'] = int(val)
            save_imp_settings(_imp_state)
            send_cmd(f"IMP JDB {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_cdb_change(val):
        try:
            _imp_state['cdb_mm'] = float(val)
            save_imp_settings(_imp_state)
            send_cmd(f"IMP CDB {float(val):.3f}".encode('ascii'), force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    # === EST callbacks ===
    def on_est_cmd_alpha_change(val):
        try:
            _est_state['cmd_alpha_milli'] = int(val)
            save_est_settings(_est_state)
            send_cmd(f"EST CMD_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass

    def on_est_meas_alpha_change(val):
        try:
            _est_state['meas_alpha_milli'] = int(val)
            save_est_settings(_est_state)
            send_cmd(f"EST MEAS_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass

    def on_est_vel_alpha_change(val):
        try:
            _est_state['meas_vel_alpha_milli'] = int(val)
            save_est_settings(_est_state)
            send_cmd(f"EST MEAS_VEL_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass

    def on_imu_overlay_change(val):
        """Toggle IMU overlay display on/off."""
        global _displayThread
        show_overlay = (val == 1)
        if _displayThread is not None:
            _displayThread.update_imu(show_overlay=show_overlay)
    
    def on_leveling_enabled_change(val):
        """Toggle body leveling on/off."""
        global _levelingState, _levelingEnabled
        _levelingEnabled = (val == 1)
        if _levelingState is not None:
            _levelingState.config.enabled = _levelingEnabled
            if not _levelingEnabled:
                _levelingState.reset()  # Clear corrections when disabled
        if _verbose:
            print(f"Body leveling: {'ON' if _levelingEnabled else 'OFF'}", end="\r\n")
    
    def on_leveling_gain_change(val):
        """Update leveling gain."""
        global _levelingState, _levelingGain
        _levelingGain = val
        if _levelingState is not None:
            _levelingState.config.gain = val
        if _verbose:
            print(f"Leveling gain: {val:.1f}", end="\r\n")
    
    def on_leveling_max_corr_change(val):
        """Update max correction limit."""
        global _levelingState, _levelingMaxCorrMm
        _levelingMaxCorrMm = val
        if _levelingState is not None:
            _levelingState.config.max_correction_mm = val
        if _verbose:
            print(f"Leveling max correction: {val:.0f}mm", end="\r\n")
    
    def on_leveling_tilt_limit_change(val):
        """Update tilt safety threshold."""
        global _levelingState, _levelingTiltLimitDeg
        _levelingTiltLimitDeg = val
        if _levelingState is not None:
            _levelingState.config.tilt_limit_deg = val
        if _verbose:
            print(f"Leveling tilt limit: {val:.0f}°", end="\r\n")
    
    def on_lean_enabled_change(val):
        """Toggle motion lean on/off."""
        global _levelingState, _leanEnabled
        _leanEnabled = (val == 1)
        if _levelingState is not None:
            _levelingState.config.lean_enabled = _leanEnabled
            if not _leanEnabled:
                _levelingState._filtered_lean_pitch = 0.0
                _levelingState._filtered_lean_roll = 0.0
        if _verbose:
            print(f"Motion lean: {'ON' if _leanEnabled else 'OFF'}", end="\r\n")
    
    def on_lean_max_change(val):
        """Update max lean angle."""
        global _levelingState, _leanMaxDeg
        _leanMaxDeg = val
        if _levelingState is not None:
            _levelingState.config.lean_max_deg = val
        if _verbose:
            print(f"Lean max: {val:.0f}°", end="\r\n")
    
    # === ToF callbacks ===
    def on_tof_enabled_change(val):
        """Update ToF enabled setting (requires restart to take effect)."""
        global _tofEnabled
        _tofEnabled = (val == 1)
        if _verbose:
            print(f"ToF enabled: {'On' if _tofEnabled else 'Off'} (restart required)", end="\r\n")
    
    def on_tof_resolution_change(val):
        """Update ToF resolution setting (requires restart to take effect)."""
        global _tofResolution
        # val: 0=4x4, 1=8x8
        _tofResolution = 64 if val == 1 else 16
        if _verbose:
            grid = 8 if _tofResolution >= 64 else 4
            print(f"ToF resolution: {grid}x{grid} (restart required)", end="\r\n")
    
    def on_tof_framerate_change(val):
        """Update ToF frame rate setting (requires restart to take effect)."""
        global _tofHz
        _tofHz = float(val)
        if _verbose:
            print(f"ToF frame rate: {_tofHz:.0f} Hz (restart required)", end="\r\n")
    
    def on_tof_bus_change(val):
        """Update ToF I2C bus setting (requires restart to take effect)."""
        global _tofBus
        _tofBus = int(val)
        if _verbose:
            print(f"ToF I2C bus: {_tofBus} (restart required)", end="\r\n")
    
    def on_tof_add_sensor():
        """Add a new ToF sensor (placeholder for future dynamic sensor management)."""
        global _tofSensors
        # Find next available address (0x29 + N)
        used_addrs = {addr for _, addr in _tofSensors}
        next_addr = 0x29
        for i in range(16):
            if (0x29 + i) not in used_addrs:
                next_addr = 0x29 + i
                break
        sensor_num = len(_tofSensors) + 1
        _tofSensors.append((f"sensor{sensor_num}", next_addr))
        if _verbose:
            print(f"Added ToF sensor{sensor_num} at 0x{next_addr:02X} (restart required)", end="\r\n")
    
    def on_tof_remove_sensor1():
        """Remove first ToF sensor."""
        global _tofSensors
        if _tofSensors:
            removed = _tofSensors.pop(0)
            if _verbose:
                print(f"Removed ToF sensor '{removed[0]}' (restart required)", end="\r\n")
    
    def on_tof_apply_restart():
        """Save ToF settings and restart the ToF thread."""
        global _tofThread, _tofEnabled, _tofHz, _tofBus, _tofResolution, _tofSensors
        # Save settings to config
        tof_state = {
            'enabled': _tofEnabled,
            'hz': _tofHz,
            'bus': _tofBus,
            'resolution': 8 if _tofResolution >= 64 else 4,
            'sensors': _tofSensors,
        }
        if save_tof_settings(tof_state):
            if _verbose:
                print("ToF settings saved", end="\r\n")
        else:
            if _verbose:
                print("ToF settings save failed", end="\r\n")
        
        # Stop existing thread if running
        if _tofThread is not None and _tofThread.is_alive():
            _tofThread.stop()
            if _verbose:
                print("ToF thread stopped", end="\r\n")
            _tofThread = None
        
        # Restart if enabled (note: sensor init must happen on main thread for VL53L5CX)
        if _tofEnabled:
            if _verbose:
                print("ToF restart: settings saved. Full restart recommended for sensor re-init.", end="\r\n")
        else:
            if _verbose:
                print("ToF disabled and stopped", end="\r\n")
    
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Theme", "on_change", on_theme_change)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Palette", "on_change", on_palette_change)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Brightness", "on_change", on_brightness_change)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Auto-Disable", "on_change", on_auto_disable_change)

    _marsMenu.set_callback(MenuCategory.PID, "Enabled", "on_change", on_pid_enabled_change)
    _marsMenu.set_callback(MenuCategory.PID, "Mode", "on_change", on_pid_mode_change)
    _marsMenu.set_callback(MenuCategory.PID, "Shadow Hz", "on_change", on_pid_shadow_hz_change)
    _marsMenu.set_callback(MenuCategory.PID, "Kp Coxa", "on_change", _make_pid_gain_cb('KP', 'COXA'))
    _marsMenu.set_callback(MenuCategory.PID, "Kp Femur", "on_change", _make_pid_gain_cb('KP', 'FEMUR'))
    _marsMenu.set_callback(MenuCategory.PID, "Kp Tibia", "on_change", _make_pid_gain_cb('KP', 'TIBIA'))
    _marsMenu.set_callback(MenuCategory.PID, "Ki Coxa", "on_change", _make_pid_gain_cb('KI', 'COXA'))
    _marsMenu.set_callback(MenuCategory.PID, "Ki Femur", "on_change", _make_pid_gain_cb('KI', 'FEMUR'))
    _marsMenu.set_callback(MenuCategory.PID, "Ki Tibia", "on_change", _make_pid_gain_cb('KI', 'TIBIA'))
    _marsMenu.set_callback(MenuCategory.PID, "Kd Coxa", "on_change", _make_pid_gain_cb('KD', 'COXA'))
    _marsMenu.set_callback(MenuCategory.PID, "Kd Femur", "on_change", _make_pid_gain_cb('KD', 'FEMUR'))
    _marsMenu.set_callback(MenuCategory.PID, "Kd Tibia", "on_change", _make_pid_gain_cb('KD', 'TIBIA'))
    _marsMenu.set_callback(MenuCategory.PID, "Kdα Coxa", "on_change", _make_pid_kdalpha_cb('COXA'))
    _marsMenu.set_callback(MenuCategory.PID, "Kdα Femur", "on_change", _make_pid_kdalpha_cb('FEMUR'))
    _marsMenu.set_callback(MenuCategory.PID, "Kdα Tibia", "on_change", _make_pid_kdalpha_cb('TIBIA'))

    _marsMenu.set_callback(MenuCategory.IMP, "Enabled", "on_change", on_imp_enabled_change)
    _marsMenu.set_callback(MenuCategory.IMP, "Mode", "on_change", on_imp_mode_change)
    _marsMenu.set_callback(MenuCategory.IMP, "Scale", "on_change", on_imp_scale_change)
    _marsMenu.set_callback(MenuCategory.IMP, "J Spring Coxa", "on_change", _make_imp_joint_gain_cb('JSPRING', 'COXA'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Spring Femur", "on_change", _make_imp_joint_gain_cb('JSPRING', 'FEMUR'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Spring Tibia", "on_change", _make_imp_joint_gain_cb('JSPRING', 'TIBIA'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Damp Coxa", "on_change", _make_imp_joint_gain_cb('JDAMP', 'COXA'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Damp Femur", "on_change", _make_imp_joint_gain_cb('JDAMP', 'FEMUR'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Damp Tibia", "on_change", _make_imp_joint_gain_cb('JDAMP', 'TIBIA'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Spring X", "on_change", _make_imp_cart_gain_cb('CSPRING', 'X'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Spring Y", "on_change", _make_imp_cart_gain_cb('CSPRING', 'Y'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Spring Z", "on_change", _make_imp_cart_gain_cb('CSPRING', 'Z'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Damp X", "on_change", _make_imp_cart_gain_cb('CDAMP', 'X'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Damp Y", "on_change", _make_imp_cart_gain_cb('CDAMP', 'Y'))
    _marsMenu.set_callback(MenuCategory.IMP, "C Damp Z", "on_change", _make_imp_cart_gain_cb('CDAMP', 'Z'))
    _marsMenu.set_callback(MenuCategory.IMP, "J Deadband", "on_change", on_imp_jdb_change)
    _marsMenu.set_callback(MenuCategory.IMP, "C Deadband", "on_change", on_imp_cdb_change)

    _marsMenu.set_callback(MenuCategory.EST, "Cmd α", "on_change", on_est_cmd_alpha_change)
    _marsMenu.set_callback(MenuCategory.EST, "Meas α", "on_change", on_est_meas_alpha_change)
    _marsMenu.set_callback(MenuCategory.EST, "Vel α", "on_change", on_est_vel_alpha_change)

    _marsMenu.set_callback(MenuCategory.IMU, "Show Overlay", "on_change", on_imu_overlay_change)
    _marsMenu.set_callback(MenuCategory.IMU, "Leveling", "on_change", on_leveling_enabled_change)
    _marsMenu.set_callback(MenuCategory.IMU, "LVL Gain", "on_change", on_leveling_gain_change)
    _marsMenu.set_callback(MenuCategory.IMU, "Max Corr", "on_change", on_leveling_max_corr_change)
    _marsMenu.set_callback(MenuCategory.IMU, "Tilt Limit", "on_change", on_leveling_tilt_limit_change)
    _marsMenu.set_callback(MenuCategory.IMU, "Motion Lean", "on_change", on_lean_enabled_change)
    _marsMenu.set_callback(MenuCategory.IMU, "Lean Max", "on_change", on_lean_max_change)

    _marsMenu.set_callback(MenuCategory.TOF, "Enabled", "on_change", on_tof_enabled_change)
    _marsMenu.set_callback(MenuCategory.TOF, "Resolution", "on_change", on_tof_resolution_change)
    _marsMenu.set_callback(MenuCategory.TOF, "Frame Rate", "on_change", on_tof_framerate_change)
    _marsMenu.set_callback(MenuCategory.TOF, "I2C Bus", "on_change", on_tof_bus_change)
    _marsMenu.set_callback(MenuCategory.TOF, "Add Sensor", "on_select", on_tof_add_sensor)
    _marsMenu.set_callback(MenuCategory.TOF, "Remove Sensor 1", "on_select", on_tof_remove_sensor1)
    _marsMenu.set_callback(MenuCategory.TOF, "Apply & Restart", "on_select", on_tof_apply_restart)

    # === Autonomy callbacks ===
    def on_autonomy_enabled_change(val):
        """Toggle autonomy mode on/off."""
        global _autonomyEnabled, _behaviorArbiter
        _autonomyEnabled = (val == 1)
        if _autonomyEnabled and _behaviorArbiter is None:
            _behaviorArbiter = _init_behavior_arbiter()
        if _verbose:
            print(f"Autonomy: {'ENABLED' if _autonomyEnabled else 'DISABLED'}", end="\r\n")

    def on_obstacle_avoid_change(val):
        """Toggle obstacle avoidance behavior."""
        global _autonomyObstacleAvoidance, _behaviorArbiter
        _autonomyObstacleAvoidance = (val == 1)
        if _behaviorArbiter is not None:
            _behaviorArbiter.set_behavior_enabled("ObstacleAvoidance", _autonomyObstacleAvoidance)
        if _verbose:
            print(f"Obstacle avoidance: {'On' if _autonomyObstacleAvoidance else 'Off'}", end="\r\n")

    def on_cliff_detect_change(val):
        """Toggle cliff detection behavior."""
        global _autonomyCliffDetection, _behaviorArbiter
        _autonomyCliffDetection = (val == 1)
        if _behaviorArbiter is not None:
            _behaviorArbiter.set_behavior_enabled("CliffDetection", _autonomyCliffDetection)
        if _verbose:
            print(f"Cliff detection: {'On' if _autonomyCliffDetection else 'Off'}", end="\r\n")

    def on_caught_foot_change(val):
        """Toggle caught foot recovery behavior."""
        global _autonomyCaughtFootRecovery, _behaviorArbiter
        _autonomyCaughtFootRecovery = (val == 1)
        if _behaviorArbiter is not None:
            _behaviorArbiter.set_behavior_enabled("CaughtFootRecovery", _autonomyCaughtFootRecovery)
        if _verbose:
            print(f"Caught foot recovery: {'On' if _autonomyCaughtFootRecovery else 'Off'}", end="\r\n")

    def on_patrol_change(val):
        """Toggle patrol behavior."""
        global _autonomyPatrol, _behaviorArbiter
        _autonomyPatrol = (val == 1)
        if _behaviorArbiter is not None:
            _behaviorArbiter.set_behavior_enabled("Patrol", _autonomyPatrol)
        # Show notification on LCD
        if _displayThread is not None:
            _displayThread.show_notification(f"Patrol {'ON' if _autonomyPatrol else 'OFF'}", 2.0)
        if _verbose:
            print(f"Patrol: {'On' if _autonomyPatrol else 'Off'}", end="\r\n")

    def on_wall_follow_change(val):
        """Toggle wall following behavior."""
        global _autonomyWallFollow, _behaviorArbiter
        _autonomyWallFollow = (val == 1)
        if _behaviorArbiter is not None:
            _behaviorArbiter.set_behavior_enabled("WallFollow", _autonomyWallFollow)
        # Show notification on LCD
        if _displayThread is not None:
            side_str = _autonomyWallSide.capitalize()
            _displayThread.show_notification(f"Wall Follow {'ON' if _autonomyWallFollow else 'OFF'} ({side_str})", 2.0)
        if _verbose:
            print(f"Wall follow: {'On' if _autonomyWallFollow else 'Off'} ({_autonomyWallSide})", end="\r\n")

    def on_wall_side_change(val):
        """Update wall following side."""
        global _autonomyWallSide, _behaviorArbiter
        _autonomyWallSide = 'left' if val == 0 else 'right'
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("WallFollow")
            if behavior is not None:
                behavior.wall_side = _autonomyWallSide
        if _verbose:
            print(f"Wall side: {_autonomyWallSide}", end="\r\n")

    def on_wall_dist_change(val):
        """Update wall following distance."""
        global _autonomyWallDistMm, _behaviorArbiter
        _autonomyWallDistMm = int(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("WallFollow")
            if behavior is not None:
                behavior.wall_distance_mm = _autonomyWallDistMm
        if _verbose:
            print(f"Wall distance: {_autonomyWallDistMm}mm", end="\r\n")

    def on_stop_dist_change(val):
        """Update obstacle stop distance."""
        global _autonomyStopDistMm
        _autonomyStopDistMm = int(val)
        # Update behavior if arbiter exists
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("ObstacleAvoidance")
            if behavior is not None:
                behavior.stop_distance_mm = _autonomyStopDistMm
        if _verbose:
            print(f"Stop distance: {_autonomyStopDistMm}mm", end="\r\n")

    def on_slow_dist_change(val):
        """Update obstacle slow distance."""
        global _autonomySlowDistMm
        _autonomySlowDistMm = int(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("ObstacleAvoidance")
            if behavior is not None:
                behavior.slow_distance_mm = _autonomySlowDistMm
        if _verbose:
            print(f"Slow distance: {_autonomySlowDistMm}mm", end="\r\n")

    def on_cliff_thresh_change(val):
        """Update cliff detection threshold."""
        global _autonomyCliffThresholdMm
        _autonomyCliffThresholdMm = int(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("CliffDetection")
            if behavior is not None:
                behavior.cliff_threshold_mm = _autonomyCliffThresholdMm
        if _verbose:
            print(f"Cliff threshold: {_autonomyCliffThresholdMm}mm", end="\r\n")

    def on_snag_error_change(val):
        """Update snag position error threshold."""
        global _autonomySnagErrorDeg
        _autonomySnagErrorDeg = float(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.position_error_threshold = _autonomySnagErrorDeg
        if _verbose:
            print(f"Snag error threshold: {_autonomySnagErrorDeg:.1f}°", end="\r\n")

    def on_snag_timeout_change(val):
        """Update snag timeout."""
        global _autonomySnagTimeoutMs
        _autonomySnagTimeoutMs = int(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.timeout_ms = _autonomySnagTimeoutMs
        if _verbose:
            print(f"Snag timeout: {_autonomySnagTimeoutMs}ms", end="\r\n")

    def on_recovery_lift_change(val):
        """Update recovery lift height."""
        global _autonomyRecoveryLiftMm
        _autonomyRecoveryLiftMm = float(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("CaughtFootRecovery")
            if behavior is not None:
                behavior.recovery_lift_mm = _autonomyRecoveryLiftMm
        if _verbose:
            print(f"Recovery lift: {_autonomyRecoveryLiftMm:.0f}mm", end="\r\n")

    def on_patrol_time_change(val):
        """Update patrol duration."""
        global _autonomyPatrolDurationS
        _autonomyPatrolDurationS = float(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("Patrol")
            if behavior is not None:
                behavior.patrol_duration_s = _autonomyPatrolDurationS
        if _verbose:
            print(f"Patrol duration: {_autonomyPatrolDurationS:.0f}s", end="\r\n")

    def on_turn_interval_change(val):
        """Update patrol turn interval."""
        global _autonomyTurnIntervalS
        _autonomyTurnIntervalS = float(val)
        if _behaviorArbiter is not None:
            behavior = _behaviorArbiter.get_behavior("Patrol")
            if behavior is not None:
                behavior.turn_interval_s = _autonomyTurnIntervalS
        if _verbose:
            print(f"Turn interval: {_autonomyTurnIntervalS:.0f}s", end="\r\n")

    def on_autonomy_save():
        """Save autonomy settings to config."""
        from config_manager import save_behavior_settings
        settings = {
            'enabled': _autonomyEnabled,
            'obstacle_avoidance': _autonomyObstacleAvoidance,
            'cliff_detection': _autonomyCliffDetection,
            'caught_foot_recovery': _autonomyCaughtFootRecovery,
            'wall_follow': _autonomyWallFollow,
            'wall_side': _autonomyWallSide,
            'wall_distance_mm': _autonomyWallDistMm,
            'patrol': _autonomyPatrol,
            'stop_distance_mm': _autonomyStopDistMm,
            'slow_distance_mm': _autonomySlowDistMm,
            'cliff_threshold_mm': _autonomyCliffThresholdMm,
            'snag_position_error_deg': _autonomySnagErrorDeg,
            'snag_timeout_ms': _autonomySnagTimeoutMs,
            'recovery_lift_mm': _autonomyRecoveryLiftMm,
            'patrol_duration_s': _autonomyPatrolDurationS,
            'turn_interval_s': _autonomyTurnIntervalS,
        }
        if save_behavior_settings(settings):
            if _verbose:
                print("Autonomy settings saved", end="\r\n")
        else:
            if _verbose:
                print("Failed to save autonomy settings", end="\r\n")

    _marsMenu.set_callback(MenuCategory.AUTO, "Autonomy", "on_change", on_autonomy_enabled_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Obstacle Avoid", "on_change", on_obstacle_avoid_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Cliff Detect", "on_change", on_cliff_detect_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Caught Foot", "on_change", on_caught_foot_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Wall Follow", "on_change", on_wall_follow_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Wall Side", "on_change", on_wall_side_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Wall Dist", "on_change", on_wall_dist_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Patrol", "on_change", on_patrol_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Stop Dist", "on_change", on_stop_dist_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Slow Dist", "on_change", on_slow_dist_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Cliff Thresh", "on_change", on_cliff_thresh_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Snag Error", "on_change", on_snag_error_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Snag Timeout", "on_change", on_snag_timeout_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Recovery Lift", "on_change", on_recovery_lift_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Patrol Time", "on_change", on_patrol_time_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Turn Interval", "on_change", on_turn_interval_change)
    _marsMenu.set_callback(MenuCategory.AUTO, "Save Settings", "on_select", on_autonomy_save)

    _marsMenu.set_callback(MenuCategory.SYSTEM, "Verbose", "on_change", on_verbose_change)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Mirror Display", "on_change", on_mirror_change)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Save All", "on_select", on_save_all)
    _marsMenu.set_callback(MenuCategory.SYSTEM, "Shutdown", "on_select", on_shutdown)
    
    # === GAIT callbacks ===
    def on_gait_type_change(val):
        # 0=Tripod, 1=Wave, 2=Ripple, 3=Stationary - only Tripod implemented for now
        if _verbose:
            gait_names = ["Tripod", "Wave", "Ripple", "Stationary"]
            print(f"Gait type: {gait_names[val]}", end="\r\n")
    
    def on_step_height_change(val):
        global _savedGaitLiftMm, _gaitEngine
        _savedGaitLiftMm = val
        if _gaitEngine is not None:
            _gaitEngine.params.lift_mm = val
        if _verbose:
            print(f"Step height: {val}mm", end="\r\n")
    
    def on_step_length_change(val):
        global _gaitEngine
        if _gaitEngine is not None:
            _gaitEngine.params.step_length_mm = val
        if _verbose:
            print(f"Step length: {val}mm", end="\r\n")

    def on_turn_rate_change(val):
        global _gaitTurnMaxDegS, _savedGaitWidthMm, _savedGaitLiftMm, _gaitCycleMs
        _gaitTurnMaxDegS = float(val)
        # Persist alongside other gait settings so value survives restart
        save_gait_settings(_savedGaitWidthMm, _savedGaitLiftMm, cycle_ms=_gaitCycleMs, turn_max_deg_s=_gaitTurnMaxDegS)
        if _verbose:
            print(f"Max turn rate: {_gaitTurnMaxDegS:.1f} deg/s", end="\r\n")
    
    def on_cycle_time_change(val):
        global _gaitEngine, _gaitCycleMs, _savedGaitWidthMm, _savedGaitLiftMm
        _gaitCycleMs = int(val)
        if _gaitEngine is not None and hasattr(_gaitEngine, 'params'):
            _gaitEngine.params.cycle_ms = _gaitCycleMs
        # Persist to config alongside width/lift so new gaits pick up the value
        save_gait_settings(_savedGaitWidthMm, _savedGaitLiftMm, cycle_ms=_gaitCycleMs)
        if _verbose:
            print(f"Cycle time: {_gaitCycleMs}ms", end="\r\n")
    
    def on_start_gait():
        global _gaitActive, _gaitEngine, _autoDisableAt
        if not _gaitActive:
            ensure_enabled()
            if _gaitEngine is None:
                from gait_engine import TripodGait, GaitParams
                params = GaitParams()
                params.base_x_mm = _savedGaitWidthMm
                params.lift_mm = _savedGaitLiftMm
                params.cycle_ms = _gaitCycleMs
                _gaitEngine = TripodGait(params)
            _gaitEngine.start()
            _gaitActive = True
            _autoDisableAt = None
            if _verbose:
                print("Gait started via menu", end="\r\n")
    
    def on_stop_gait():
        global _gaitActive, _gaitEngine, _autoDisableAt
        if _gaitActive:
            if _gaitEngine is not None:
                _gaitEngine.stop()
            _gaitActive = False
            _autoDisableAt = time.time() + _autoDisableS
            if _verbose:
                print("Gait stopped via menu", end="\r\n")
    
    _marsMenu.set_callback(MenuCategory.GAIT, "Type", "on_change", on_gait_type_change)
    _marsMenu.set_callback(MenuCategory.GAIT, "Step Height", "on_change", on_step_height_change)
    _marsMenu.set_callback(MenuCategory.GAIT, "Step Length", "on_change", on_step_length_change)
    _marsMenu.set_callback(MenuCategory.GAIT, "Turn Rate", "on_change", on_turn_rate_change)
    _marsMenu.set_callback(MenuCategory.GAIT, "Cycle Time", "on_change", on_cycle_time_change)
    _marsMenu.set_callback(MenuCategory.GAIT, "Start Gait", "on_select", on_start_gait)
    _marsMenu.set_callback(MenuCategory.GAIT, "Stop Gait", "on_select", on_stop_gait)
    
    # === POSTURE callbacks ===
    def on_stand():
        """Start standing gait (uses gait engine for leveling support)."""
        global _enabledLocal
        # Ensure robot is enabled
        if _teensy is not None and not _enabledLocal:
            send_cmd(b'ENABLE', force=True)
            _enabledLocal = True
        _start_standing_gait()
        _marsMenu.hide()
    
    def on_tuck():
        apply_posture(b'TUCK', auto_disable_s=_autoDisableS)
        _marsMenu.hide()
    
    def on_home():
        apply_posture(b'HOME', auto_disable_s=_autoDisableS)
        _marsMenu.hide()

    def on_pounce():
        """Spider-like "pounce" attack: crouch back, front legs up, spring forward."""
        start_pounce_move(source="menu")
    
    _marsMenu.set_callback(MenuCategory.POSTURE, "Stand", "on_select", on_stand)
    _marsMenu.set_callback(MenuCategory.POSTURE, "Tuck", "on_select", on_tuck)
    _marsMenu.set_callback(MenuCategory.POSTURE, "Home", "on_select", on_home)
    _marsMenu.set_callback(MenuCategory.POSTURE, "Pounce", "on_select", on_pounce)

    def _persist_pounce():
        return save_pounce_settings(
            _pouncePrepMs,
            _pounceRearMs,
            _pounceLungeMs,
            _pounceRecoverMs,
            _pounceBack1Z,
            _pounceBack2Z,
            _pouncePushZ,
            _pounceStrikeZ,
            _pounceCrouchDy,
            _pounceLiftDy,
            _pounceFrontZ,
        )

    def on_p_prep(val):
        global _pouncePrepMs
        _pouncePrepMs = int(val)
        _persist_pounce()

    def on_p_rear(val):
        global _pounceRearMs
        _pounceRearMs = int(val)
        _persist_pounce()

    def on_p_lunge(val):
        global _pounceLungeMs
        _pounceLungeMs = int(val)
        _persist_pounce()

    def on_p_recov(val):
        global _pounceRecoverMs
        _pounceRecoverMs = int(val)
        _persist_pounce()

    def on_p_back1(val):
        global _pounceBack1Z
        _pounceBack1Z = float(val)
        _persist_pounce()

    def on_p_back2(val):
        global _pounceBack2Z
        _pounceBack2Z = float(val)
        _persist_pounce()

    def on_p_push(val):
        global _pouncePushZ
        _pouncePushZ = float(val)
        _persist_pounce()

    def on_p_strike(val):
        global _pounceStrikeZ
        _pounceStrikeZ = float(val)
        _persist_pounce()

    def on_p_crouch(val):
        global _pounceCrouchDy
        _pounceCrouchDy = float(val)
        _persist_pounce()

    def on_p_lift(val):
        global _pounceLiftDy
        _pounceLiftDy = float(val)
        _persist_pounce()

    def on_p_frontz(val):
        global _pounceFrontZ
        _pounceFrontZ = float(val)
        _persist_pounce()

    _marsMenu.set_callback(MenuCategory.POSTURE, "P Prep", "on_change", on_p_prep)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Rear", "on_change", on_p_rear)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Lunge", "on_change", on_p_lunge)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Recov", "on_change", on_p_recov)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Back1", "on_change", on_p_back1)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Back2", "on_change", on_p_back2)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Push", "on_change", on_p_push)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Strike", "on_change", on_p_strike)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Crouch", "on_change", on_p_crouch)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P Lift", "on_change", on_p_lift)
    _marsMenu.set_callback(MenuCategory.POSTURE, "P FrontZ", "on_change", on_p_frontz)

    # === SAFETY callbacks ===
    def on_clear_safety():
        """Issue SAFETY CLEAR to Teensy to request clearing lockout.

        Python will continue to honor safety state based on S5; this
        action merely forwards a clear request to firmware.
        """
        if _teensy is not None:
            send_cmd(b'SAFETY CLEAR', force=True)
            if _verbose:
                print("SAFETY CLEAR command sent to Teensy", end="\r\n")

    def _send_safety_override(keyword: bytes, label: str):
        """Helper to send SAFETY OVERRIDE <keyword> to Teensy from menu.

        This only affects firmware safety evaluation; Python continues
        to mirror and honor S5-reported state.
        """
        if _teensy is not None:
            cmd = b'SAFETY OVERRIDE ' + keyword
            send_cmd(cmd, force=True)
            if _verbose:
                print(f"SAFETY OVERRIDE {label} command sent to Teensy", end="\r\n")

    def on_override_all():
        _send_safety_override(b'ALL', 'ALL')

    def on_override_temp():
        _send_safety_override(b'TEMP', 'TEMP')

    def on_override_collision():
        _send_safety_override(b'COLLISION', 'COLLISION')

    def on_override_none():
        _send_safety_override(b'NONE', 'NONE')

    # Safety display threshold callbacks
    def on_volt_min_change(value):
        global _safetyDisplayVoltMin
        _safetyDisplayVoltMin = float(value)
        if _displayThread:
            _displayThread.set_safety_thresholds(volt_min=_safetyDisplayVoltMin)
        save_safety_display_settings({'volt_min': _safetyDisplayVoltMin})
    
    def on_volt_warn_change(value):
        global _safetyDisplayVoltWarn
        _safetyDisplayVoltWarn = float(value)
        if _displayThread:
            _displayThread.set_safety_thresholds(volt_warn=_safetyDisplayVoltWarn)
        save_safety_display_settings({'volt_warn': _safetyDisplayVoltWarn})
    
    def on_volt_max_change(value):
        global _safetyDisplayVoltMax
        _safetyDisplayVoltMax = float(value)
        if _displayThread:
            _displayThread.set_safety_thresholds(volt_max=_safetyDisplayVoltMax)
        save_safety_display_settings({'volt_max': _safetyDisplayVoltMax})
    
    def on_temp_min_change(value):
        global _safetyDisplayTempMin
        _safetyDisplayTempMin = float(value)
        if _displayThread:
            _displayThread.set_safety_thresholds(temp_min=_safetyDisplayTempMin)
        save_safety_display_settings({'temp_min': _safetyDisplayTempMin})
    
    def on_temp_max_change(value):
        global _safetyDisplayTempMax
        _safetyDisplayTempMax = float(value)
        if _displayThread:
            _displayThread.set_safety_thresholds(temp_max=_safetyDisplayTempMax)
        save_safety_display_settings({'temp_max': _safetyDisplayTempMax})

    # Low battery protection callbacks
    def on_lowbatt_prot_change(value):
        global _lowBatteryEnabled
        _lowBatteryEnabled = (value == 1)
        save_low_battery_settings({'enabled': _lowBatteryEnabled})
    
    def on_volt_critical_change(value):
        global _lowBatteryVoltCritical
        _lowBatteryVoltCritical = float(value)
        save_low_battery_settings({'volt_critical': _lowBatteryVoltCritical})
    
    def on_volt_recovery_change(value):
        global _lowBatteryRecoveryVolt
        _lowBatteryRecoveryVolt = float(value)
        save_low_battery_settings({'volt_recovery': _lowBatteryRecoveryVolt})
    
    def on_lowbatt_filter_change(value):
        global _lowBatteryFilterAlpha
        _lowBatteryFilterAlpha = max(0.01, min(1.0, float(value)))
        save_low_battery_settings({'filter_alpha': _lowBatteryFilterAlpha})

    _marsMenu.set_callback(MenuCategory.SAFETY, "Clear Safety", "on_select", on_clear_safety)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override ALL", "on_select", on_override_all)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override TEMP", "on_select", on_override_temp)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override COLLISION", "on_select", on_override_collision)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override NONE", "on_select", on_override_none)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Volt Min", "on_change", on_volt_min_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Volt Warn", "on_change", on_volt_warn_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Volt Max", "on_change", on_volt_max_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Temp Min", "on_change", on_temp_min_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Temp Max", "on_change", on_temp_max_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "LowBatt Prot", "on_change", on_lowbatt_prot_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Volt Critical", "on_change", on_volt_critical_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Volt Recovery", "on_change", on_volt_recovery_change)
    _marsMenu.set_callback(MenuCategory.SAFETY, "LowBatt Filter", "on_change", on_lowbatt_filter_change)
    
    # === Sync initial values ===
    _marsMenu.set_value(MenuCategory.EYES, "Style", _eyes.left_shape)
    _marsMenu.set_value(MenuCategory.EYES, "Human Color", _eyes.human_eye_color_idx)
    _marsMenu.set_value(MenuCategory.EYES, "Size", _eyes.human_eye_size)
    _marsMenu.set_value(MenuCategory.EYES, "V Center", _eyeVerticalOffset)
    _marsMenu.set_value(MenuCategory.EYES, "Spacing", int(_eyes.human_eye_spacing_pct * 100))
    _marsMenu.set_value(MenuCategory.EYES, "CRT Effect", 1 if _eyes.crt_mode else 0)

    # Pounce defaults (from controller.ini [pounce])
    _marsMenu.set_value(MenuCategory.POSTURE, "P Prep", int(_pouncePrepMs))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Rear", int(_pounceRearMs))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Lunge", int(_pounceLungeMs))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Recov", int(_pounceRecoverMs))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Back1", int(round(_pounceBack1Z)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Back2", int(round(_pounceBack2Z)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Push", int(round(_pouncePushZ)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Strike", int(round(_pounceStrikeZ)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Crouch", int(round(_pounceCrouchDy)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P Lift", int(round(_pounceLiftDy)))
    _marsMenu.set_value(MenuCategory.POSTURE, "P FrontZ", int(round(_pounceFrontZ)))
    _marsMenu.set_value(MenuCategory.SYSTEM, "Theme", _menuTheme)
    _marsMenu.set_value(MenuCategory.SYSTEM, "Palette", _menuPalette)
    _marsMenu.set_value(MenuCategory.SYSTEM, "Brightness", _displayBrightness)
    _marsMenu.set_value(MenuCategory.SYSTEM, "Auto-Disable", int(_autoDisableS))
    _marsMenu.set_value(MenuCategory.SYSTEM, "Verbose", 1 if _verbose else 0)
    _marsMenu.set_value(MenuCategory.SYSTEM, "Mirror Display", 1 if _mirrorDisplay else 0)
    _marsMenu.set_value(MenuCategory.GAIT, "Step Height", int(_savedGaitLiftMm))
    _marsMenu.set_value(MenuCategory.GAIT, "Turn Rate", int(_gaitTurnMaxDegS))
    _marsMenu.set_value(MenuCategory.GAIT, "Cycle Time", int(_gaitCycleMs))
    _marsMenu.set_value(MenuCategory.INFO, "Ctrl Version", f"{CONTROLLER_VERSION} b{CONTROLLER_BUILD}")
    
    # Safety display thresholds
    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Min", _safetyDisplayVoltMin)
    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Warn", _safetyDisplayVoltWarn)
    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Max", _safetyDisplayVoltMax)
    _marsMenu.set_value(MenuCategory.SAFETY, "Temp Min", _safetyDisplayTempMin)
    _marsMenu.set_value(MenuCategory.SAFETY, "Temp Max", _safetyDisplayTempMax)
    
    # Low battery protection settings
    _marsMenu.set_value(MenuCategory.SAFETY, "LowBatt Prot", 1 if _lowBatteryEnabled else 0)
    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Critical", _lowBatteryVoltCritical)
    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Recovery", _lowBatteryRecoveryVolt)
    _marsMenu.set_value(MenuCategory.SAFETY, "LowBatt Filter", _lowBatteryFilterAlpha)
    _marsMenu.set_value(MenuCategory.SAFETY, "LowBatt Status", "OK")
    
    # Apply saved theme/palette
    _marsMenu.theme = _menuTheme
    _marsMenu.lcars_palette = _menuPalette

_setup_mars_menu()

def update_menu_info(ctrl: Controller | None = None):
    """Update INFO menu items from current telemetry data.

    INFO tab shows instantaneous values and servo stats used for bring-up.
    """
    global _marsMenu, _state, _servo, _safety_state
    if _marsMenu is None:
        return

    # Prefer structured telemetry when available
    if ctrl is None:
        ctrl = globals().get('ctrl', None)
    system = getattr(ctrl, 'system_telem', None) if ctrl is not None else None
    servo_telem = getattr(ctrl, 'servo_telem', None) if ctrl is not None else None
    safety_telem = getattr(ctrl, 'safety_telem', None) if ctrl is not None else None
    
    # Compute average servo voltage (prefer structured servo telemetry)
    avg_servo_v = None
    if servo_telem is not None:
        v_sum = 0.0
        v_count = 0
        for s in servo_telem:
            if s is None or not getattr(s, 'valid_s3', False):
                continue
            v = getattr(s, 'voltage_v', 0.0)
            if v and v > 0:
                v_sum += float(v)
                v_count += 1
        if v_count > 0:
            avg_servo_v = v_sum / v_count
    elif _servo is not None:
        v_sum = 0.0
        v_count = 0
        for s in _servo:
            if not s or len(s) < 1:
                continue
            v = s[0]
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v <= 0:
                continue
            v_sum += v
            v_count += 1
        if v_count > 0:
            avg_servo_v = v_sum / v_count

    # Servo voltage should be visible on INFO only.
    _marsMenu.set_value(MenuCategory.INFO, "Servo Volt", avg_servo_v)

    # Update INFO items from S1 telemetry - instantaneous values
    if system is not None and getattr(system, 'valid', False):
        batt_v = getattr(system, 'battery_v', None)
        if batt_v is None or batt_v <= 0:
            _marsMenu.set_value(MenuCategory.INFO, "Battery", avg_servo_v)
        else:
            _marsMenu.set_value(MenuCategory.INFO, "Battery", batt_v)
        _marsMenu.set_value(MenuCategory.INFO, "Current", getattr(system, 'current_a', None))
        loop_us = int(getattr(system, 'loop_us', 0) or 0)
        _marsMenu.set_value(MenuCategory.INFO, "Loop Time", loop_us if loop_us > 0 else None)
        _marsMenu.set_value(MenuCategory.INFO, "IMU Pitch", getattr(system, 'pitch_deg', None))
        _marsMenu.set_value(MenuCategory.INFO, "IMU Roll", getattr(system, 'roll_deg', None))
    else:
        if len(_state) > IDX_BATTERY_V:
            batt_v = _state[IDX_BATTERY_V]
            # If battery voltage is missing/zero, fall back to average servo bus voltage.
            if batt_v is None or batt_v <= 0:
                _marsMenu.set_value(MenuCategory.INFO, "Battery", avg_servo_v)
            else:
                _marsMenu.set_value(MenuCategory.INFO, "Battery", batt_v)
        if len(_state) > IDX_CURRENT_A:
            _marsMenu.set_value(MenuCategory.INFO, "Current", _state[IDX_CURRENT_A])
        if len(_state) > IDX_LOOP_US:
            loop_us = int(_state[IDX_LOOP_US])
            _marsMenu.set_value(MenuCategory.INFO, "Loop Time", loop_us if loop_us > 0 else None)
        if len(_state) > IDX_PITCH_DEG:
            _marsMenu.set_value(MenuCategory.INFO, "IMU Pitch", _state[IDX_PITCH_DEG])
        if len(_state) > IDX_ROLL_DEG:
            _marsMenu.set_value(MenuCategory.INFO, "IMU Roll", _state[IDX_ROLL_DEG])
    
    # Find servo temperature statistics (prefer structured servo telemetry)
    max_temp = None
    temp_sum = 0.0
    temp_count = 0
    if servo_telem is not None:
        for s in servo_telem:
            if s is None or not getattr(s, 'valid_s3', False):
                continue
            tval = float(getattr(s, 'temp_c', 0) or 0)
            if tval <= 0:
                continue
            temp_sum += tval
            temp_count += 1
            if max_temp is None or tval > max_temp:
                max_temp = tval
    elif _servo is not None:
        for i in range(len(_servo)):
            # _servo entries are [voltage_V, temp_C, enabled] (S3/S2)
            if len(_servo[i]) > 1:
                temp = _servo[i][1]
                if temp is None:
                    continue
                try:
                    tval = float(temp)
                except (TypeError, ValueError):
                    continue
                # Ignore uninitialized/invalid temps
                if tval <= 0:
                    continue
                temp_sum += tval
                temp_count += 1
                if max_temp is None or tval > max_temp:
                    max_temp = tval
    # INFO tab shows max servo temperature
    _marsMenu.set_value(MenuCategory.INFO, "Servo Temp", int(max_temp) if max_temp is not None else None)

    # SAFETY tab: mirror latest S5 snapshot
    if safety_telem is not None and getattr(safety_telem, 'valid', False):
        lockout = bool(getattr(safety_telem, 'lockout', False))
        cause_mask = int(getattr(safety_telem, 'cause_mask', 0) or 0)
        override_mask = int(getattr(safety_telem, 'override_mask', 0) or 0)
        clearance_mm = int(getattr(safety_telem, 'clearance_mm', 0) or 0)
        soft_limits = bool(getattr(safety_telem, 'soft_limits', False))
        collision = bool(getattr(safety_telem, 'collision', False))
        temp_c = int(getattr(safety_telem, 'temp_c', 0) or 0)
    else:
        lockout = _safety_state.get("lockout", False)
        cause_mask = _safety_state.get("cause_mask", 0)
        override_mask = _safety_state.get("override_mask", 0)
        clearance_mm = _safety_state.get("clearance_mm", 0)
        soft_limits = _safety_state.get("soft_limits", False)
        collision = _safety_state.get("collision", False)
        temp_c = _safety_state.get("temp_c", 0)

    state_str = "LOCKOUT" if lockout else "OK"
    _marsMenu.set_value(MenuCategory.SAFETY, "State", state_str)
    _marsMenu.set_value(MenuCategory.SAFETY, "Cause", f"0x{cause_mask:04X}")
    _marsMenu.set_value(MenuCategory.SAFETY, "Override", f"0x{override_mask:04X}")
    _marsMenu.set_value(MenuCategory.SAFETY, "Clearance", int(clearance_mm))
    _marsMenu.set_value(MenuCategory.SAFETY, "Soft Limits", "On" if soft_limits else "Off")
    _marsMenu.set_value(MenuCategory.SAFETY, "Collision", "On" if collision else "Off")
    _marsMenu.set_value(MenuCategory.SAFETY, "Temp Lock", f"{int(temp_c)}C")
    
    # Low battery protection status
    if _lowBatteryTriggered:
        lowbatt_status = f"SHUT {_lowBatteryFilteredVoltage:.1f}V"
    elif _lowBatteryFilteredVoltage > 0 and _lowBatteryFilteredVoltage < _lowBatteryVoltCritical + 0.5:
        lowbatt_status = f"LOW {_lowBatteryFilteredVoltage:.1f}V"
    elif _lowBatteryFilteredVoltage > 0:
        lowbatt_status = f"OK {_lowBatteryFilteredVoltage:.1f}V"
    else:
        lowbatt_status = "---"
    _marsMenu.set_value(MenuCategory.SAFETY, "LowBatt Status", lowbatt_status)

    # PID/IMP/EST tabs: mirror latest LIST snapshots (if available)
    if _pid_state.get("enabled") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Enabled", 1 if _pid_state["enabled"] else 0)
    if _pid_state.get("mode") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Mode", 0 if _pid_state["mode"] == "active" else 1)
    if _pid_state.get("shadow_hz") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Shadow Hz", int(_pid_state["shadow_hz"]))
    if _pid_state.get("kp") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Kp Coxa", int(_pid_state["kp"][0]))
        _marsMenu.set_value(MenuCategory.PID, "Kp Femur", int(_pid_state["kp"][1]))
        _marsMenu.set_value(MenuCategory.PID, "Kp Tibia", int(_pid_state["kp"][2]))
    if _pid_state.get("ki") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Ki Coxa", int(_pid_state["ki"][0]))
        _marsMenu.set_value(MenuCategory.PID, "Ki Femur", int(_pid_state["ki"][1]))
        _marsMenu.set_value(MenuCategory.PID, "Ki Tibia", int(_pid_state["ki"][2]))
    if _pid_state.get("kd") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Kd Coxa", int(_pid_state["kd"][0]))
        _marsMenu.set_value(MenuCategory.PID, "Kd Femur", int(_pid_state["kd"][1]))
        _marsMenu.set_value(MenuCategory.PID, "Kd Tibia", int(_pid_state["kd"][2]))
    if _pid_state.get("kdalph") is not None:
        _marsMenu.set_value(MenuCategory.PID, "Kdα Coxa", int(_pid_state["kdalph"][0]))
        _marsMenu.set_value(MenuCategory.PID, "Kdα Femur", int(_pid_state["kdalph"][1]))
        _marsMenu.set_value(MenuCategory.PID, "Kdα Tibia", int(_pid_state["kdalph"][2]))

    if _imp_state.get("enabled") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "Enabled", 1 if _imp_state["enabled"] else 0)
    if _imp_state.get("mode") is not None:
        mode = _imp_state["mode"]
        _marsMenu.set_value(MenuCategory.IMP, "Mode", 1 if mode == "joint" else (2 if mode == "cart" else 0))
    if _imp_state.get("scale") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "Scale", int(_imp_state["scale"]))
    if _imp_state.get("jspring") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "J Spring Coxa", int(_imp_state["jspring"][0]))
        _marsMenu.set_value(MenuCategory.IMP, "J Spring Femur", int(_imp_state["jspring"][1]))
        _marsMenu.set_value(MenuCategory.IMP, "J Spring Tibia", int(_imp_state["jspring"][2]))
    if _imp_state.get("jdamp") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "J Damp Coxa", int(_imp_state["jdamp"][0]))
        _marsMenu.set_value(MenuCategory.IMP, "J Damp Femur", int(_imp_state["jdamp"][1]))
        _marsMenu.set_value(MenuCategory.IMP, "J Damp Tibia", int(_imp_state["jdamp"][2]))
    if _imp_state.get("cspring") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "C Spring X", int(_imp_state["cspring"][0]))
        _marsMenu.set_value(MenuCategory.IMP, "C Spring Y", int(_imp_state["cspring"][1]))
        _marsMenu.set_value(MenuCategory.IMP, "C Spring Z", int(_imp_state["cspring"][2]))
    if _imp_state.get("cdamp") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "C Damp X", int(_imp_state["cdamp"][0]))
        _marsMenu.set_value(MenuCategory.IMP, "C Damp Y", int(_imp_state["cdamp"][1]))
        _marsMenu.set_value(MenuCategory.IMP, "C Damp Z", int(_imp_state["cdamp"][2]))
    if _imp_state.get("jdb_cd") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "J Deadband", int(_imp_state["jdb_cd"]))
    if _imp_state.get("cdb_mm") is not None:
        _marsMenu.set_value(MenuCategory.IMP, "C Deadband", float(_imp_state["cdb_mm"]))

    if _est_state.get("cmd_alpha_milli") is not None:
        _marsMenu.set_value(MenuCategory.EST, "Cmd α", int(_est_state["cmd_alpha_milli"]))
    if _est_state.get("meas_alpha_milli") is not None:
        _marsMenu.set_value(MenuCategory.EST, "Meas α", int(_est_state["meas_alpha_milli"]))
    if _est_state.get("meas_vel_alpha_milli") is not None:
        _marsMenu.set_value(MenuCategory.EST, "Vel α", int(_est_state["meas_vel_alpha_milli"]))

    # IMU tab: update live sensor data
    if _imuThread is not None:
        imu_connected = _imuThread.connected
        imu_frame = _imuThread.get_frame() if imu_connected else None
        _marsMenu.set_value(MenuCategory.IMU, "Status", "Connected" if imu_connected else "Disconnected")
        if imu_frame and imu_frame.valid:
            _marsMenu.set_value(MenuCategory.IMU, "Roll", imu_frame.roll_deg)
            _marsMenu.set_value(MenuCategory.IMU, "Pitch", imu_frame.pitch_deg)
            _marsMenu.set_value(MenuCategory.IMU, "Yaw", imu_frame.yaw_deg)
        else:
            _marsMenu.set_value(MenuCategory.IMU, "Roll", None)
            _marsMenu.set_value(MenuCategory.IMU, "Pitch", None)
            _marsMenu.set_value(MenuCategory.IMU, "Yaw", None)
        _marsMenu.set_value(MenuCategory.IMU, "Update Rate", int(_imuHz) if imu_connected else 0)
        _marsMenu.set_value(MenuCategory.IMU, "Read Count", _imuThread.read_count)
        _marsMenu.set_value(MenuCategory.IMU, "Error Count", _imuThread.error_count)
        _marsMenu.set_value(MenuCategory.IMU, "I2C Bus", _imuBus)
        _marsMenu.set_value(MenuCategory.IMU, "I2C Addr", _imuAddress)
        # Sync leveling state to menu
        _marsMenu.set_value(MenuCategory.IMU, "Leveling", 1 if _levelingEnabled else 0)
        _marsMenu.set_value(MenuCategory.IMU, "LVL Gain", _levelingGain)
        _marsMenu.set_value(MenuCategory.IMU, "Max Corr", _levelingMaxCorrMm)
        _marsMenu.set_value(MenuCategory.IMU, "Tilt Limit", _levelingTiltLimitDeg)
        # Show current corrections
        if _levelingState is not None and _levelingState.config.enabled:
            corr = _levelingState.get_last_corrections()
            # Format as compact string: LF/LM/LR RF/RM/RR
            corr_str = f"L{corr[0]:+.0f}/{corr[1]:+.0f}/{corr[2]:+.0f} R{corr[3]:+.0f}/{corr[4]:+.0f}/{corr[5]:+.0f}"
            _marsMenu.set_value(MenuCategory.IMU, "LVL Corrections", corr_str)
        else:
            _marsMenu.set_value(MenuCategory.IMU, "LVL Corrections", "---")
        # Motion lean settings
        _marsMenu.set_value(MenuCategory.IMU, "Motion Lean", 1 if _leanEnabled else 0)
        _marsMenu.set_value(MenuCategory.IMU, "Lean Max", _leanMaxDeg)
    else:
        _marsMenu.set_value(MenuCategory.IMU, "Status", "Disabled")
        _marsMenu.set_value(MenuCategory.IMU, "Roll", None)
        _marsMenu.set_value(MenuCategory.IMU, "Pitch", None)
        _marsMenu.set_value(MenuCategory.IMU, "Yaw", None)
        _marsMenu.set_value(MenuCategory.IMU, "Update Rate", 0)
        _marsMenu.set_value(MenuCategory.IMU, "Read Count", 0)
        _marsMenu.set_value(MenuCategory.IMU, "Error Count", 0)
        # Leveling defaults when IMU disabled
        _marsMenu.set_value(MenuCategory.IMU, "Leveling", 0)
        _marsMenu.set_value(MenuCategory.IMU, "LVL Gain", _levelingGain)
        _marsMenu.set_value(MenuCategory.IMU, "Max Corr", _levelingMaxCorrMm)
        _marsMenu.set_value(MenuCategory.IMU, "Tilt Limit", _levelingTiltLimitDeg)
        _marsMenu.set_value(MenuCategory.IMU, "LVL Corrections", "---")
        # Motion lean defaults when IMU disabled (lean works without IMU)
        _marsMenu.set_value(MenuCategory.IMU, "Motion Lean", 1 if _leanEnabled else 0)
        _marsMenu.set_value(MenuCategory.IMU, "Lean Max", _leanMaxDeg)

    # ToF tab: update live sensor data
    # Settings (editable) - sync from config variables
    _marsMenu.set_value(MenuCategory.TOF, "Enabled", 1 if _tofEnabled else 0)
    _tof_grid_size = 8 if _tofResolution >= 64 else 4
    _marsMenu.set_value(MenuCategory.TOF, "Resolution", 1 if _tof_grid_size == 8 else 0)  # 0=4x4, 1=8x8
    _marsMenu.set_value(MenuCategory.TOF, "Frame Rate", int(_tofHz))
    _marsMenu.set_value(MenuCategory.TOF, "I2C Bus", _tofBus)
    _marsMenu.set_value(MenuCategory.TOF, "Sensors", len(_tofSensors))
    # Sensor info (first sensor only for now; dynamic sensors would need menu rebuild)
    if _tofSensors:
        _marsMenu.set_value(MenuCategory.TOF, "Sensor 1 Name", _tofSensors[0][0])
        _marsMenu.set_value(MenuCategory.TOF, "Sensor 1 Addr", _tofSensors[0][1])
    else:
        _marsMenu.set_value(MenuCategory.TOF, "Sensor 1 Name", "---")
        _marsMenu.set_value(MenuCategory.TOF, "Sensor 1 Addr", 0x29)
    
    # Live data (read-only)
    if _tofThread is not None:
        tof_connected = _tofThread.connected
        _marsMenu.set_value(MenuCategory.TOF, "Status", "Connected" if tof_connected else "Disconnected")
        if tof_connected:
            tof_frame = _tofThread.get_frame()
            # Find closest obstacle
            closest_dist, closest_sensor = tof_frame.get_closest_obstacle()
            if closest_dist > 0:
                _marsMenu.set_value(MenuCategory.TOF, "Closest", closest_dist)
            else:
                _marsMenu.set_value(MenuCategory.TOF, "Closest", "---")
            # Get first sensor frame for temperature
            sensor_names = _tofThread.get_sensor_names()
            sensor_name = sensor_names[0] if sensor_names else None
            if sensor_name and sensor_name in tof_frame.sensors:
                sensor_frame = tof_frame.sensors[sensor_name]
                _marsMenu.set_value(MenuCategory.TOF, "Temperature", sensor_frame.temperature_c)
            else:
                _marsMenu.set_value(MenuCategory.TOF, "Temperature", None)
        else:
            _marsMenu.set_value(MenuCategory.TOF, "Closest", "---")
            _marsMenu.set_value(MenuCategory.TOF, "Temperature", None)
        _marsMenu.set_value(MenuCategory.TOF, "Read Count", _tofThread.read_count)
        _marsMenu.set_value(MenuCategory.TOF, "Error Count", _tofThread.error_count)
    else:
        _marsMenu.set_value(MenuCategory.TOF, "Status", "Disabled")
        _marsMenu.set_value(MenuCategory.TOF, "Read Count", 0)
        _marsMenu.set_value(MenuCategory.TOF, "Error Count", 0)
        _marsMenu.set_value(MenuCategory.TOF, "Closest", "---")
        _marsMenu.set_value(MenuCategory.TOF, "Temperature", None)

    # === Initialize Autonomy menu values ===
    _marsMenu.set_value(MenuCategory.AUTO, "Autonomy", 1 if _autonomyEnabled else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Obstacle Avoid", 1 if _autonomyObstacleAvoidance else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Cliff Detect", 1 if _autonomyCliffDetection else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Caught Foot", 1 if _autonomyCaughtFootRecovery else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Wall Follow", 1 if _autonomyWallFollow else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Wall Side", 0 if _autonomyWallSide == 'left' else 1)
    _marsMenu.set_value(MenuCategory.AUTO, "Wall Dist", _autonomyWallDistMm)
    _marsMenu.set_value(MenuCategory.AUTO, "Patrol", 1 if _autonomyPatrol else 0)
    _marsMenu.set_value(MenuCategory.AUTO, "Stop Dist", _autonomyStopDistMm)
    _marsMenu.set_value(MenuCategory.AUTO, "Slow Dist", _autonomySlowDistMm)
    _marsMenu.set_value(MenuCategory.AUTO, "Cliff Thresh", _autonomyCliffThresholdMm)
    _marsMenu.set_value(MenuCategory.AUTO, "Snag Error", _autonomySnagErrorDeg)
    _marsMenu.set_value(MenuCategory.AUTO, "Snag Timeout", _autonomySnagTimeoutMs)
    _marsMenu.set_value(MenuCategory.AUTO, "Recovery Lift", _autonomyRecoveryLiftMm)
    _marsMenu.set_value(MenuCategory.AUTO, "Patrol Time", int(_autonomyPatrolDurationS))
    _marsMenu.set_value(MenuCategory.AUTO, "Turn Interval", int(_autonomyTurnIntervalS))
    _marsMenu.set_value(MenuCategory.AUTO, "Status", "Off" if not _autonomyEnabled else "Idle")
    _marsMenu.set_value(MenuCategory.AUTO, "Active Behavior", "---")
    _marsMenu.set_value(MenuCategory.AUTO, "Last Action", "---")

# Create and start display thread if enabled
_displayThread = None
if _displayThreadEnabled:
    _startupSplash.log("Starting display thread...")
    _displayThread = DisplayThread(_disp, _eyes, _menu, target_hz=_displayThreadHz,
                                   look_range_x=_eyeLookRangeX, look_range_y=_eyeLookRangeY,
                                   blink_frame_divisor=_blinkFrameDivisor)
    _displayThread.update_state(servo=_servo, legs=_legs, state=_state, 
                                mirror=_mirrorDisplay, menu_state=_menuState, verbose=_verbose,
                                teensy_connected=(_teensy is not None),
                                controller_connected=(_controller is not None),
                                telemetry_stale=(_teensy is not None and _lastTelemetryTime is None),
                                robot_enabled=(_state[IDX_ROBOT_ENABLED] == 1.0 if len(_state) > IDX_ROBOT_ENABLED else True),
                                safety_active=_safety_state.get("lockout", False),
                                safety_text=get_safety_overlay_state()[1])
    _displayThread.set_mars_menu(_marsMenu)
    _displayThread.set_safety_thresholds(
        volt_min=_safetyDisplayVoltMin,
        volt_warn=_safetyDisplayVoltWarn,
        volt_max=_safetyDisplayVoltMax,
        temp_min=_safetyDisplayTempMin,
        temp_max=_safetyDisplayTempMax,
    )
    _displayThread.set_show_battery_icon(_showBatteryIcon)
    _displayThread.set_engineering_lcars(_engineeringLcars)
    _displayThread.set_lcars_palette(_menuPalette)
    _displayThread.start()
    _startupSplash.log(f"Display thread OK ({_displayThreadHz} Hz)", "GREEN")
    if _verbose:
        print(f"Display thread started at {_displayThreadHz} Hz (blink divisor: {_blinkFrameDivisor})", end="\r\n")
else:
    _startupSplash.log("Display thread disabled", "YELLOW")

# Create and start IMU thread if enabled
_imuThread = None
if _imuEnabled:
    _startupSplash.log("Initializing IMU...")
    _imuConfig = ImuConfig(
        enabled=True,
        i2c_bus=_imuBus,
        i2c_address=_imuAddress,
        target_hz=_imuHz,
        use_game_rotation=_imuUseGameRotation,
        enable_accel=_imuEnableAccel,
        enable_gyro=_imuEnableGyro,
        enable_mag=_imuEnableMag,
    )
    _imuThread = ImuThread(_imuConfig)
    _imuThread.start()
    # Give it time to connect - BNO085 initialization can take ~1.5s
    for _wait_i in range(20):  # Up to 2 seconds
        time.sleep(0.1)
        _startupSplash.log(".", append=True)  # Progress dots
        if _imuThread.connected:
            break
        if _imuThread.last_error:
            break
    if _imuThread.connected:
        _startupSplash.log(f"IMU OK ({_imuHz} Hz)", "GREEN")
        if _verbose:
            print(f"IMU thread started at {_imuHz} Hz (bus {_imuBus}, addr 0x{_imuAddress:02X})", end="\r\n")
    else:
        _startupSplash.log(f"IMU not detected: {_imuThread.last_error}", "RED")
        if _verbose:
            print(f"IMU not detected (bus {_imuBus}, addr 0x{_imuAddress:02X}): {_imuThread.last_error}", end="\r\n")
else:
    _startupSplash.log("IMU disabled", "YELLOW")

# Initialize body leveling state
_startupSplash.log("Initializing leveling...")
_levelingConfig = LevelingConfig(
    enabled=_levelingEnabled,
    gain=_levelingGain,
    max_correction_mm=_levelingMaxCorrMm,
    filter_alpha=_levelingFilterAlpha,
    pitch_offset_deg=_levelingPitchOffset,
    roll_offset_deg=_levelingRollOffset,
    tilt_limit_deg=_levelingTiltLimitDeg,
    lean_enabled=_leanEnabled,
    lean_max_deg=_leanMaxDeg,
    lean_filter_alpha=_leanFilterAlpha,
)
_levelingState = init_leveling(_levelingConfig)
_startupSplash.log("Leveling OK", "GREEN")


def _on_tilt_safety_triggered(pitch_deg: float, roll_deg: float) -> None:
    """Callback invoked when tilt exceeds safety threshold.
    Issues TUCK + DISABLE to protect the robot."""
    global _enabledLocal, _verbose
    tilt_max = max(abs(pitch_deg), abs(roll_deg))
    if _verbose:
        print(f"\r\n[LEVELING] TILT SAFETY: pitch={pitch_deg:.1f}° roll={roll_deg:.1f}° (>{_levelingTiltLimitDeg:.0f}°)", end="\r\n")
        print(f"[LEVELING] Issuing emergency TUCK + DISABLE", end="\r\n")
    # Issue TUCK with auto-disable (protective posture)
    apply_posture(b'TUCK', auto_disable_s=2.0, require_enable=False)


# Set up tilt safety callback
_levelingState.set_tilt_safety_callback(_on_tilt_safety_triggered)

# Create and start ToF thread if enabled
_tofThread = None
if _tofEnabled:
    _startupSplash.log("Initializing ToF sensors...")
    # Build sensor config list
    _tof_sensor_configs = [ToFSensorConfig(name=name, i2c_address=addr) for name, addr in _tofSensors]
    _tofConfig = ToFConfig(
        i2c_bus=_tofBus,
        target_hz=_tofHz,
        resolution=_tofResolution,
        sensors=_tof_sensor_configs,
        filter_mode=_tofFilterMode,
        filter_alpha=_tofFilterAlpha,
        filter_sigma_threshold=_tofFilterSigmaThresh,
        filter_outlier_mm=_tofFilterOutlierMm,
    )
    _tofThread = ToFThread(_tofConfig)
    # Initialize sensors on main thread (VL53L5CX ctypes library segfaults if init from thread)
    _tof_grid_size = 8 if _tofResolution >= 64 else 4
    if _verbose:
        print(f"ToF sensors initializing (bus {_tofBus}, resolution {_tof_grid_size}x{_tof_grid_size})...", end="\r\n")
        for name, addr in _tofSensors:
            print(f"  {name}: 0x{addr:02X}", end="\r\n")
    _tof_count = _tofThread.init_sensors()
    if _tof_count > 0:
        _tofThread.start()
        _startupSplash.log(f"ToF OK ({_tof_count} sensor(s) @ {_tofHz} Hz)", "GREEN")
        if _verbose:
            print(f"ToF thread started at {_tofHz} Hz ({_tof_count} sensor(s))", end="\r\n")
    else:
        err_msg = _tofThread.last_error or "unknown error"
        _startupSplash.log(f"ToF not detected: {err_msg}", "RED")
        if _verbose:
            print(f"ToF not detected: {err_msg}", end="\r\n")
else:
    _startupSplash.log("ToF sensors disabled", "YELLOW")

# --------------------------------------------------------------------------------------------------
# Point Cloud Server: WebSocket server for 3D visualization (SLAM prototype)
# --------------------------------------------------------------------------------------------------
if _pointcloudEnabled and PointCloudServer is not None:
    try:
        from pointcloud_server import PointCloudConfig as PCConfig
        _pc_config = PCConfig(
            port=_pointcloudPort,
            http_port=_pointcloudHttpPort,
            stream_rate_hz=_pointcloudHz,
            accumulate_enabled=_pointcloudAccumulate,
            max_points=_pointcloudMaxPoints,
            voxel_size_mm=_pointcloudVoxelMm,
        )
        # Configure ToF sensor mount (front sensor, centered, tilted down 15 degrees)
        # Format: (x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)
        _pc_config.sensor_mounts["front"] = (0.0, 60.0, 40.0, 0.0, -15.0, 0.0)
        _pointcloudServer = PointCloudServer(_pc_config)
        _pointcloudServer.start()
        _startupSplash.log(f"PointCloud http://:{_pointcloudHttpPort}", "GREEN")
        if _verbose:
            print(f"Point cloud viewer: http://0.0.0.0:{_pointcloudHttpPort}/", end="\r\n")
            print(f"Point cloud WebSocket: ws://0.0.0.0:{_pointcloudPort}", end="\r\n")
    except Exception as e:
        _startupSplash.log(f"PointCloud failed: {e}", "RED")
        if _verbose:
            print(f"Point cloud server failed: {e}", end="\r\n")
        _pointcloudServer = None
elif _pointcloudEnabled:
    _startupSplash.log("PointCloud: module not found", "YELLOW")
else:
    pass  # Point cloud disabled in config

# --------------------------------------------------------------------------------------------------
# Dashboard Config Change Handler (Phase 3: Edit configuration from web dashboard)
# --------------------------------------------------------------------------------------------------
def _handle_dashboard_config_change(section: str, key: str, value) -> bool:
    """Handle configuration change requests from web dashboard.
    
    Args:
        section: Config section name (e.g., 'Gait', 'Safety', 'PID')
        key: Config key/item label (e.g., 'Cycle Time', 'volt_critical')
        value: New value to set
        
    Returns:
        True if change was applied successfully, False otherwise
    """
    global _gaitCycleMs, _gaitStepLenMm, _savedGaitWidthMm, _savedGaitLiftMm
    global _gaitTurnMaxDegS, _gaitEngine
    global _lowBatteryEnabled, _lowBatteryVoltCritical, _lowBatteryRecoveryVolt
    global _marsMenu
    
    # Map menu labels to internal keys (supports both formats)
    key_aliases = {
        # Gait
        'cycle time': 'cycle_ms', 'cycle_ms': 'cycle_ms',
        'step length': 'step_length_mm', 'step_length_mm': 'step_length_mm',
        'step height': 'step_height_mm', 'step_height_mm': 'step_height_mm', 'lift_mm': 'step_height_mm',
        'turn rate': 'turn_rate', 'turn_rate': 'turn_rate', 'turn_max_deg_s': 'turn_rate',
        # Safety
        'low batt prot': 'low_battery_enabled', 'low_battery_enabled': 'low_battery_enabled',
        'volt critical': 'volt_critical', 'volt_critical': 'volt_critical',
        'volt recovery': 'volt_recovery', 'volt_recovery': 'volt_recovery',
        'lowbatt filter': 'filter_alpha', 'filter_alpha': 'filter_alpha',
        # PID
        'pid enabled': 'enabled', 'enabled': 'enabled',
        'kp': 'kp', 'ki': 'ki', 'kd': 'kd',
        'max correction': 'max_correction', 'max_correction': 'max_correction',
        # Impedance
        'imp enabled': 'enabled',
        'max force': 'max_force_n', 'max_force_n': 'max_force_n',
        # Estimator
        'est alpha': 'alpha', 'alpha': 'alpha',
        'velocity decay': 'vel_decay', 'vel_decay': 'vel_decay',
    }
    
    # Normalize key
    normalized_key = key_aliases.get(key.lower(), key.lower())
    
    try:
        section_lower = section.lower()
        
        # Gait settings
        if section_lower == 'gait':
            if normalized_key == 'cycle_ms':
                _gaitCycleMs = int(value)
                if _gaitEngine is not None and hasattr(_gaitEngine, 'params'):
                    _gaitEngine.params.cycle_ms = _gaitCycleMs
                save_gait_settings(_savedGaitWidthMm, _savedGaitLiftMm, cycle_ms=_gaitCycleMs)
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.GAIT, "Cycle Time", _gaitCycleMs)
                if _verbose:
                    print(f"[Dashboard] Gait cycle_ms -> {_gaitCycleMs}", end="\r\n")
                return True
                
            elif normalized_key == 'step_length_mm':
                _gaitStepLenMm = float(value)
                if _gaitEngine is not None and hasattr(_gaitEngine, 'params'):
                    _gaitEngine.params.step_length_mm = _gaitStepLenMm
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.GAIT, "Step Length", _gaitStepLenMm)
                if _verbose:
                    print(f"[Dashboard] Gait step_length_mm -> {_gaitStepLenMm}", end="\r\n")
                return True
                
            elif normalized_key == 'step_height_mm':
                _savedGaitLiftMm = float(value)
                if _gaitEngine is not None and hasattr(_gaitEngine, 'params'):
                    _gaitEngine.params.lift_mm = _savedGaitLiftMm
                save_gait_settings(_savedGaitWidthMm, _savedGaitLiftMm, cycle_ms=_gaitCycleMs)
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.GAIT, "Step Height", _savedGaitLiftMm)
                if _verbose:
                    print(f"[Dashboard] Gait lift_mm -> {_savedGaitLiftMm}", end="\r\n")
                return True
                
            elif normalized_key == 'turn_rate':
                _gaitTurnMaxDegS = float(value)
                save_gait_settings(_savedGaitWidthMm, _savedGaitLiftMm, cycle_ms=_gaitCycleMs, turn_max_deg_s=_gaitTurnMaxDegS)
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.GAIT, "Turn Rate", _gaitTurnMaxDegS)
                if _verbose:
                    print(f"[Dashboard] Gait turn_max_deg_s -> {_gaitTurnMaxDegS}", end="\r\n")
                return True
        
        # Safety settings
        elif section_lower == 'safety':
            if normalized_key == 'low_battery_enabled':
                _lowBatteryEnabled = bool(value) if not isinstance(value, int) else value != 0
                save_low_battery_settings({'enabled': _lowBatteryEnabled})
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.SAFETY, "Low Batt Prot", 1 if _lowBatteryEnabled else 0)
                if _verbose:
                    print(f"[Dashboard] Safety low_battery_enabled -> {_lowBatteryEnabled}", end="\r\n")
                return True
                
            elif normalized_key == 'volt_critical':
                _lowBatteryVoltCritical = float(value)
                save_low_battery_settings({'volt_critical': _lowBatteryVoltCritical})
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Critical", _lowBatteryVoltCritical)
                if _verbose:
                    print(f"[Dashboard] Safety volt_critical -> {_lowBatteryVoltCritical}", end="\r\n")
                return True
                
            elif normalized_key == 'volt_recovery':
                _lowBatteryRecoveryVolt = float(value)
                save_low_battery_settings({'volt_recovery': _lowBatteryRecoveryVolt})
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.SAFETY, "Volt Recovery", _lowBatteryRecoveryVolt)
                if _verbose:
                    print(f"[Dashboard] Safety volt_recovery -> {_lowBatteryRecoveryVolt}", end="\r\n")
                return True
                
            elif normalized_key == 'filter_alpha':
                global _lowBatteryFilterAlpha
                _lowBatteryFilterAlpha = max(0.01, min(1.0, float(value)))
                save_low_battery_settings({'filter_alpha': _lowBatteryFilterAlpha})
                if _marsMenu:
                    _marsMenu.set_value(MenuCategory.SAFETY, "LowBatt Filter", _lowBatteryFilterAlpha)
                if _verbose:
                    print(f"[Dashboard] Safety filter_alpha -> {_lowBatteryFilterAlpha}", end="\r\n")
                return True
        
        # PID settings
        elif section_lower == 'pid':
            # Delegate to PID handler
            return _handle_pid_config_change(normalized_key, value)
        
        # IMP (Impedance) settings  
        elif section_lower in ('imp', 'impedance'):
            return _handle_imp_config_change(normalized_key, value)
        
        # EST (Estimator) settings
        elif section_lower in ('est', 'estimator'):
            return _handle_est_config_change(normalized_key, value)
        
        if _verbose:
            print(f"[Dashboard] Unknown config: {section}.{key} = {value}", end="\r\n")
        return False
        
    except Exception as e:
        if _verbose:
            print(f"[Dashboard] Config change error: {e}", end="\r\n")
        return False


def _handle_pid_config_change(key: str, value) -> bool:
    """Handle PID config changes from dashboard."""
    global _pidEnabled, _pidKp, _pidKi, _pidKd, _pidMaxCorrection
    
    # Normalize key for menu labels
    key_lower = key.lower()
    
    try:
        if key_lower == 'enabled' or key_lower == 'pid enabled':
            _pidEnabled = bool(value) if not isinstance(value, int) else value != 0
            save_pid_settings({'enabled': _pidEnabled})
            return True
        elif key_lower == 'kp':
            _pidKp = float(value)
            # Save as triplet (same value for all joints)
            save_pid_settings({'kp': [int(_pidKp), int(_pidKp), int(_pidKp)]})
            return True
        elif key_lower == 'ki':
            _pidKi = float(value)
            save_pid_settings({'ki': [int(_pidKi), int(_pidKi), int(_pidKi)]})
            return True
        elif key_lower == 'kd':
            _pidKd = float(value)
            save_pid_settings({'kd': [int(_pidKd), int(_pidKd), int(_pidKd)]})
            return True
        elif key_lower in ('max_correction', 'max correction'):
            _pidMaxCorrection = float(value)
            # max_correction not in save_pid_settings, store in local state only
            return True
        return False
    except Exception:
        return False


def _handle_imp_config_change(key: str, value) -> bool:
    """Handle Impedance config changes from dashboard."""
    global _impEnabled, _impKp, _impKd, _impMaxN
    
    # Normalize key for menu labels
    key_lower = key.lower()
    
    try:
        if key_lower == 'enabled' or key_lower == 'imp enabled':
            _impEnabled = bool(value) if not isinstance(value, int) else value != 0
            save_imp_settings({'enabled': _impEnabled})
            return True
        elif key_lower == 'kp':
            _impKp = float(value)
            # 'kp' in dashboard maps to 'scale' in firmware
            save_imp_settings({'scale': int(_impKp)})
            return True
        elif key_lower == 'kd':
            _impKd = float(value)
            # 'kd' in dashboard maps to 'jdb_cd' (joint deadband) in firmware
            save_imp_settings({'jdb_cd': int(_impKd)})
            return True
        elif key_lower in ('max_force_n', 'max force'):
            _impMaxN = float(value)
            # max_force_n not in save_imp_settings, store in local state only
            return True
        return False
    except Exception:
        return False


def _handle_est_config_change(key: str, value) -> bool:
    """Handle Estimator config changes from dashboard."""
    global _estAlpha, _estVelDecay
    
    # Normalize key for menu labels
    key_lower = key.lower()
    
    try:
        if key_lower in ('alpha', 'est alpha'):
            _estAlpha = float(value)
            # Convert to milli (0-1000) for save function
            save_est_settings({'cmd_alpha_milli': int(_estAlpha * 1000)})
            return True
        elif key_lower in ('vel_decay', 'velocity decay'):
            _estVelDecay = float(value)
            save_est_settings({'meas_vel_alpha_milli': int(_estVelDecay * 1000)})
            return True
        return False
    except Exception:
        return False


# --------------------------------------------------------------------------------------------------
# Web Dashboard Server: WebSocket server for telemetry streaming to browser dashboard
# --------------------------------------------------------------------------------------------------
if _dashboardEnabled and TELEMETRY_SERVER_AVAILABLE:
    try:
        _dashboard_config = TelemetryServerConfig(
            ws_port=_dashboardPort,
            http_port=_pointcloudHttpPort,  # Share HTTP port with point cloud
            telemetry_rate_hz=_dashboardHz,
        )
        _dashboardServer = TelemetryServer(_dashboard_config)
        _dashboardServer.start()
        _startupSplash.log(f"Dashboard ws://:{_dashboardPort}", "GREEN")
        if _verbose:
            print(f"Dashboard WebSocket: ws://0.0.0.0:{_dashboardPort}", end="\r\n")
            print(f"Dashboard viewer: http://0.0.0.0:{_pointcloudHttpPort}/dashboard", end="\r\n")
        # Set initial config from MarsMenu (includes metadata: min, max, step, unit, options)
        if _marsMenu is not None:
            _dashboardServer.set_full_config(_marsMenu.get_all_config())
        # Register config change callback for Phase 3 editing
        _dashboardServer.set_config_change_callback(_handle_dashboard_config_change)
    except Exception as e:
        _startupSplash.log(f"Dashboard failed: {e}", "RED")
        if _verbose:
            print(f"Dashboard server failed: {e}", end="\r\n")
        _dashboardServer = None
elif _dashboardEnabled:
    _startupSplash.log("Dashboard: module not found", "YELLOW")

# --------------------------------------------------------------------------------------------------
# Audio Manager: Sound feedback via Sabrent USB DAC + PAM8403 amp
# --------------------------------------------------------------------------------------------------
if _audioEnabled and PYGAME_AVAILABLE:
    try:
        _audio_config = AudioConfig(
            enabled=_audioEnabled,
            volume=_audioVolume,
            device=_audioDevice,
            sounds_dir=_audioSoundsDir,
        )
        _audio = AudioManager(_audio_config)
        # Preload sounds from directory (if any)
        _audio.preload()
        _startupSplash.log("Audio OK (Sabrent DAC)", "GREEN")
        if _verbose:
            print(f"Audio initialized: device={_audioDevice}, volume={_audioVolume:.0%}", end="\r\n")
    except Exception as e:
        _startupSplash.log(f"Audio failed: {e}", "RED")
        if _verbose:
            print(f"Audio manager failed: {e}", end="\r\n")
        _audio = None
elif _audioEnabled:
    _startupSplash.log("Audio: pygame not available", "YELLOW")

# --------------------------------------------------------------------------------------------------
# TTS Engine: Text-to-Speech via espeak-ng or piper + sox amplification
# --------------------------------------------------------------------------------------------------
if _ttsEnabled:
    if _ttsEngine == "piper" and _ttsPiperAvailable:
        # Check piper model exists
        if os.path.isfile(_ttsPiperModel):
            _ttsAvailable = True
            _model_name = os.path.basename(_ttsPiperModel).replace('.onnx', '')
            _startupSplash.log(f"TTS OK (piper/{_model_name})", "GREEN")
            if _verbose:
                print(f"TTS initialized: engine=piper, model={_model_name}, gain=+{_ttsGainDb}dB", end="\r\n")
        else:
            _startupSplash.log(f"TTS: piper model not found", "YELLOW")
            if _verbose:
                print(f"TTS: piper model not found: {_ttsPiperModel}", end="\r\n")
    elif _ttsEngine == "espeak" and _ttsAvailable:
        _startupSplash.log(f"TTS OK (espeak/{_ttsVoice})", "GREEN")
        if _verbose:
            print(f"TTS initialized: engine=espeak, voice={_ttsVoice}, rate={_ttsRate}, pitch={_ttsPitch}, gain=+{_ttsGainDb}dB", end="\r\n")
    elif _ttsEngine == "piper":
        _startupSplash.log("TTS: piper not installed", "YELLOW")
    else:
        _startupSplash.log("TTS: espeak-ng or sox not found", "YELLOW")

# --------------------------------------------------------------------------------------------------
# Audio Helper Functions
# --------------------------------------------------------------------------------------------------

def audio_click():
    """Play a short click sound for button presses."""
    if _audio is not None:
        try:
            _audio.play('click')  # Use preloaded click.wav
        except Exception:
            pass

def audio_enable():
    """Play enable sound (ascending two-tone)."""
    if _audio is not None:
        try:
            _audio.beep(440, 80)
            time.sleep(0.1)
            _audio.beep(660, 100)
        except Exception:
            pass

def audio_disable():
    """Play disable sound (descending two-tone)."""
    if _audio is not None:
        try:
            _audio.beep(660, 80)
            time.sleep(0.1)
            _audio.beep(440, 100)
        except Exception:
            pass

def audio_low_battery_warning():
    """Play urgent low battery warning (3 fast beeps)."""
    if _audio is not None:
        try:
            for _ in range(3):
                _audio.beep(880, 100)
                time.sleep(0.15)
        except Exception:
            pass

def audio_safety_lockout():
    """Play safety lockout alert (descending urgent tone)."""
    if _audio is not None:
        try:
            _audio.beep(880, 150)
            time.sleep(0.18)
            _audio.beep(660, 150)
            time.sleep(0.18)
            _audio.beep(440, 200)
        except Exception:
            pass

def audio_teensy_connect():
    """Play Teensy connected sound (happy ascending)."""
    if _audio is not None:
        try:
            _audio.beep(523, 80)  # C5
            time.sleep(0.1)
            _audio.beep(659, 100)  # E5
        except Exception:
            pass

def audio_teensy_disconnect():
    """Play Teensy disconnected sound (sad descending)."""
    if _audio is not None:
        try:
            _audio.beep(659, 80)  # E5
            time.sleep(0.1)
            _audio.beep(392, 150)  # G4
        except Exception:
            pass

def audio_gait_start():
    """Play gait started sound (quick chirp)."""
    if _audio is not None:
        try:
            _audio.beep(880, 50)
            time.sleep(0.06)
            _audio.beep(1100, 60)
        except Exception:
            pass

def audio_gait_stop():
    """Play gait stopped sound (quick descending chirp)."""
    if _audio is not None:
        try:
            _audio.beep(1100, 50)
            time.sleep(0.06)
            _audio.beep(880, 60)
        except Exception:
            pass

def audio_stand():
    """Play stand posture confirmation (rising tone)."""
    if _audio is not None:
        try:
            _audio.beep(523, 60)  # C5
            time.sleep(0.08)
            _audio.beep(659, 80)  # E5
        except Exception:
            pass

def audio_tuck():
    """Play tuck posture confirmation (falling tone)."""
    if _audio is not None:
        try:
            _audio.beep(659, 60)  # E5
            time.sleep(0.08)
            _audio.beep(440, 80)  # A4
        except Exception:
            pass

def audio_home():
    """Play home posture confirmation (neutral tone)."""
    if _audio is not None:
        try:
            _audio.beep(587, 100)  # D5
        except Exception:
            pass

def audio_autonomy_on():
    """Play autonomy enabled sound (sci-fi ascending)."""
    if _audio is not None:
        try:
            _audio.beep(440, 60)   # A4
            time.sleep(0.07)
            _audio.beep(554, 60)   # C#5
            time.sleep(0.07)
            _audio.beep(659, 80)   # E5
        except Exception:
            pass

def audio_autonomy_off():
    """Play autonomy disabled sound (descending)."""
    if _audio is not None:
        try:
            _audio.beep(659, 60)   # E5
            time.sleep(0.07)
            _audio.beep(440, 80)   # A4
        except Exception:
            pass

def audio_xbox_connect():
    """Play Xbox controller connected sound."""
    if _audio is not None:
        try:
            _audio.beep(784, 60)   # G5
            time.sleep(0.08)
            _audio.beep(988, 100)  # B5
        except Exception:
            pass

def audio_xbox_disconnect():
    """Play Xbox controller disconnected sound."""
    if _audio is not None:
        try:
            _audio.beep(659, 80)   # E5
            time.sleep(0.1)
            _audio.beep(392, 120)  # G4
        except Exception:
            pass

def audio_gait_changed(gait_name: str):
    """Play a distinct confirmation tone for each gait type.
    
    Tones are designed to be memorable and distinguishable:
    - Standing:   Low double-beep (stable, grounded)
    - Tripod:     Quick triple-pulse (fast, 3-leg rhythm)
    - Wave:       Rising sweep (flowing motion)
    - Ripple:     Descending cascade (rippling water)
    - Stationary: Gentle pulse (subtle, in-place)
    """
    if _audio is None:
        return
    try:
        name = gait_name.lower() if gait_name else ""
        if "standing" in name:
            # Low double-beep: stable, grounded
            _audio.beep(262, 80)   # C4
            time.sleep(0.1)
            _audio.beep(262, 80)   # C4
        elif "tripod" in name:
            # Quick triple-pulse: fast 3-leg rhythm
            _audio.beep(523, 50)   # C5
            time.sleep(0.06)
            _audio.beep(523, 50)   # C5
            time.sleep(0.06)
            _audio.beep(659, 70)   # E5
        elif "wave" in name:
            # Rising sweep: flowing motion
            _audio.beep(392, 60)   # G4
            time.sleep(0.07)
            _audio.beep(494, 60)   # B4
            time.sleep(0.07)
            _audio.beep(587, 80)   # D5
        elif "ripple" in name:
            # Descending cascade: rippling water
            _audio.beep(659, 50)   # E5
            time.sleep(0.06)
            _audio.beep(523, 50)   # C5
            time.sleep(0.06)
            _audio.beep(440, 50)   # A4
            time.sleep(0.06)
            _audio.beep(349, 70)   # F4
        elif "stationary" in name:
            # Gentle pulse: subtle, in-place movement
            _audio.beep(440, 100)  # A4 - single mellow tone
        else:
            # Unknown gait: neutral confirmation beep
            _audio.beep(523, 80)   # C5
    except Exception:
        pass

def audio_menu_callback(event: str):
    """Audio feedback for menu navigation.
    
    Called by MarsMenu for navigation sounds:
    - 'nav': up/down cursor movement (subtle tick)
    - 'tab': tab change (slightly higher pitch)  
    - 'adjust': value adjustment (quick blip)
    - 'select': item selection (confirmation)
    
    Sounds are kept short and subtle to not be annoying during rapid navigation.
    """
    if _audio is None:
        return
    try:
        if event == 'nav':
            # Subtle tick for cursor movement - very short, quiet
            _audio.beep(1200, 15)  # High, short tick
        elif event == 'tab':
            # Tab switch - slightly more noticeable
            _audio.beep(880, 25)   # A5, short
        elif event == 'adjust':
            # Value adjustment - quick blip
            _audio.beep(1000, 20)  # Short blip
        elif event == 'select':
            # Selection confirmation - two quick tones
            _audio.beep(660, 30)   # E5
            time.sleep(0.04)
            _audio.beep(880, 40)   # A5
    except Exception:
        pass

# Register audio callback with menu now that function is defined
if _marsMenu is not None:
    _marsMenu.set_audio_callback(audio_menu_callback)

# --------------------------------------------------------------------------------------------------
# TTS (Text-to-Speech) Helper Functions - supports espeak-ng or piper, with sox amplification
# --------------------------------------------------------------------------------------------------
import io

def _speak_worker_espeak(text: str):
    """Background worker for espeak-ng TTS.
    Uses espeak-ng -> sox (amplify) -> pygame mixer for playback.
    """
    try:
        # Pipe espeak through sox for amplification, output WAV to stdout
        # -v: voice, -s: speed (wpm), -p: pitch (0-99), -a: amplitude
        espeak = subprocess.Popen(
            ['espeak-ng', '-v', _ttsVoice, '-s', str(_ttsRate), '-p', str(_ttsPitch), 
             '-a', '200', '--stdout', text],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        sox = subprocess.Popen(
            ['sox', '-t', 'wav', '-', '-t', 'wav', '-', 'gain', str(_ttsGainDb)],
            stdin=espeak.stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        wav_data, _ = sox.communicate()
        espeak.wait()
        
        _play_wav_data(wav_data)
    except Exception:
        pass

def _speak_worker_piper(text: str):
    """Background worker for Piper neural TTS.
    Uses piper -> sox (amplify) -> pygame mixer for playback.
    """
    try:
        # Piper reads from stdin and outputs WAV to stdout
        piper = subprocess.Popen(
            ['piper', '-m', _ttsPiperModel, '--output-raw'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        # Sox converts raw PCM (22050Hz, 16-bit, mono) to WAV and amplifies
        sox = subprocess.Popen(
            ['sox', '-t', 'raw', '-r', '22050', '-e', 'signed', '-b', '16', '-c', '1', '-',
             '-t', 'wav', '-', 'gain', str(_ttsGainDb)],
            stdin=piper.stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        # Send text to piper
        piper.stdin.write(text.encode('utf-8'))
        piper.stdin.close()
        
        wav_data, _ = sox.communicate()
        piper.wait()
        
        _play_wav_data(wav_data)
    except Exception:
        pass

def _play_wav_data(wav_data: bytes):
    """Play WAV data through pygame mixer."""
    if wav_data and _audio is not None and PYGAME_AVAILABLE:
        wav_io = io.BytesIO(wav_data)
        try:
            import pygame
            sound = pygame.mixer.Sound(wav_io)
            sound.set_volume(1.0)
            sound.play()
            # Wait for playback to complete
            while pygame.mixer.get_busy():
                time.sleep(0.05)
        except Exception:
            pass

def _speak_worker(text: str):
    """Background worker to run TTS without blocking.
    Dispatches to the appropriate engine based on config.
    """
    if _ttsEngine == "piper":
        _speak_worker_piper(text)
    else:
        _speak_worker_espeak(text)

def speak(text: str, force: bool = False, blocking: bool = False) -> bool:
    """
    Speak text using espeak-ng with sox amplification via pygame.
    
    Args:
        text: Text to speak
        force: If True, bypass cooldown timer
        blocking: If True, wait for speech to complete; if False, run in background thread
    
    Returns:
        True if speech was initiated, False if skipped (cooldown/disabled)
    """
    global _ttsLastSpeak
    if not _ttsEnabled or not _ttsAvailable:
        return False
    if _audio is None:
        return False  # Need pygame mixer for playback
    
    now = time.monotonic()
    if not force and (now - _ttsLastSpeak) < _ttsCooldownSec:
        return False  # Anti-spam: too soon since last speech
    
    _ttsLastSpeak = now
    
    if blocking:
        _speak_worker(text)
    else:
        # Run in background thread to avoid blocking
        t = threading.Thread(target=_speak_worker, args=(text,), daemon=True)
        t.start()
    
    return True

def speak_battery_low(percent: int):
    """Announce low battery warning with percentage.
    
    Uses pre-cached WAV for known percentages (10, 15, 20, 25),
    falls back to real-time TTS for other values.
    """
    cached_percentages = {10, 15, 20, 25}
    if percent in cached_percentages and _audio:
        _audio.play(f'tts_battery_{percent}', priority=True)
    else:
        speak(f"Battery low, {percent} percent remaining")

def speak_enabled():
    """Announce robot enabled (pre-cached)."""
    if _audio:
        _audio.play('tts_enabled', priority=True)
    else:
        speak("Robot enabled", force=True)

def speak_disabled():
    """Announce robot disabled (pre-cached)."""
    if _audio:
        _audio.play('tts_disabled', priority=True)
    else:
        speak("Robot disabled", force=True)

def speak_safety_lockout():
    """Announce safety lockout - urgent (pre-cached)."""
    if _audio:
        _audio.play('tts_safety_lockout', priority=True)
    else:
        speak("Safety lockout activated", force=True)

def speak_stand():
    """Announce stand posture (pre-cached)."""
    if _audio:
        _audio.play('tts_standing', priority=True)
    else:
        speak("Standing")

def speak_tuck():
    """Announce tuck posture (pre-cached)."""
    if _audio:
        _audio.play('tts_tucking', priority=True)
    else:
        speak("Tucking")

def speak_autonomy_on():
    """Announce autonomy mode enabled (pre-cached)."""
    if _audio:
        _audio.play('tts_autonomy_on', priority=True)
    else:
        speak("Autonomy mode on")

def speak_autonomy_off():
    """Announce autonomy mode off (pre-cached)."""
    if _audio:
        _audio.play('tts_autonomy_off', priority=True)
    else:
        speak("Autonomy mode off")

def speak_obstacle():
    """Announce obstacle detected (pre-cached)."""
    if _audio:
        _audio.play('tts_obstacle', priority=True)
    else:
        speak("Obstacle detected")

def speak_cliff():
    """Announce cliff detected - emergency (pre-cached)."""
    if _audio:
        _audio.play('tts_cliff', priority=True)
    else:
        speak("Cliff detected", force=True)

def speak_startup():
    """Announce system startup (pre-cached)."""
    if _audio:
        _audio.play('tts_startup', priority=True)
    else:
        speak("Mars online", force=True)

def speak_shutdown():
    """Announce system shutdown (pre-cached)."""
    if _audio:
        _audio.play('tts_shutdown', priority=True)
    else:
        speak("Mars shutting down", force=True)

# --------------------------------------------------------------------------------------------------
# Command helper: centralizes Teensy command emission (newline termination + throttling/dedup)
# --------------------------------------------------------------------------------------------------
_cmd_last = {}            # bytes(command) -> last send time (seconds)
_enabledLocal = False     # local view (updated when we send ENABLE / DISABLE)


def _parse_feet_positions(feet_cmd: bytes) -> list:
    """Parse FEET command bytes into list of 18 floats [x0,y0,z0,x1,y1,z1,...].
    Returns empty list if parsing fails."""
    try:
        # Expected format: b'FEET x0 y0 z0 x1 y1 z1 ... x5 y5 z5'
        text = feet_cmd.decode('ascii').strip()
        if not text.startswith('FEET '):
            return []
        parts = text[5:].split()
        if len(parts) != 18:
            return []
        return [float(p) for p in parts]
    except Exception:
        return []


def _feet_changed_enough(new_positions: list, tolerance_mm: float) -> bool:
    """Check if any foot position changed by more than tolerance_mm.
    Returns True if command should be sent (first time or delta exceeds tolerance).
    Updates _lastFeetPositions if returning True."""
    global _lastFeetPositions
    if _lastFeetPositions is None or len(_lastFeetPositions) != 18:
        # First FEET command or invalid previous state - always send
        _lastFeetPositions = new_positions[:]
        return True
    
    # Check max delta across all 18 coordinates
    max_delta = 0.0
    for i in range(18):
        delta = abs(new_positions[i] - _lastFeetPositions[i])
        if delta > max_delta:
            max_delta = delta
    
    if max_delta >= tolerance_mm:
        _lastFeetPositions = new_positions[:]
        return True
    return False


# --------------------------------------------------------------------------------------------------
# IMU helper: get current orientation from IMU thread (if running)
# --------------------------------------------------------------------------------------------------

def get_imu_orientation():
    """Get current IMU orientation as (roll_deg, pitch_deg, yaw_deg).
    Returns (0, 0, 0) if IMU not connected or data not available."""
    global _imuThread
    if _imuThread is None or not _imuThread.connected:
        return (0.0, 0.0, 0.0)
    return _imuThread.get_orientation()


def get_imu_frame():
    """Get full IMU frame with all data (ImuFrame object).
    Returns an invalid frame if IMU not connected."""
    global _imuThread
    if _imuThread is None:
        return ImuFrame()
    return _imuThread.get_frame()


def is_imu_connected() -> bool:
    """Return True if IMU is connected and providing data."""
    global _imuThread
    return _imuThread is not None and _imuThread.connected


def _start_standing_gait(params: GaitParams = None) -> None:
    """Start a standing gait for leveling support.
    
    Unlike firmware STAND, this continuously sends FEET commands
    so body leveling corrections can be applied.
    """
    global _gaitActive, _gaitEngine, _savedGaitWidthMm, _gaitBaseYMm, _savedGaitLiftMm, _verbose
    if _gaitEngine is not None:
        _gaitEngine.stop()
    # Force MODE IDLE to ensure FEET commands are accepted by firmware
    send_cmd(b'MODE IDLE', force=True)
    if params is None:
        params = GaitParams(
            base_x_mm=_savedGaitWidthMm,
            base_y_mm=_gaitBaseYMm,
            lift_mm=_savedGaitLiftMm,
        )
    _gaitEngine = StandingGait(params)
    _gaitEngine.start()
    _gaitActive = True
    if _verbose:
        print("Standing gait started (leveling enabled)", end="\r\n")


def send_feet_cmd(feet_cmd: bytes, force: bool = False) -> bool:
    """Send FEET command with optional tolerance filtering and body leveling.
    Applies IMU-based Z corrections if leveling is enabled.
    Skips send if positions haven't changed by more than _feetToleranceMm,
    unless more than _feetMaxSkipMs has passed since last send.
    Set force=True to bypass tolerance check (e.g., for initial/final positions).
    Returns True if command was actually transmitted."""
    global _lastFeetPositions, _feetToleranceMm, _lastFeetSendTime, _feetMaxSkipMs
    global _levelingState, _imuThread, _gaitEngine, _gaitActive
    
    now = time.monotonic()
    time_since_last = (now - _lastFeetSendTime) * 1000.0  # ms
    
    # Apply body leveling and motion lean if enabled
    cmd_to_send = feet_cmd
    global _levelingDebugCount
    _levelingDebugCount += 1
    
    # Check leveling conditions
    leveling_state_ok = _levelingState is not None
    leveling_enabled = leveling_state_ok and _levelingState.config.enabled
    lean_enabled = leveling_state_ok and _levelingState.config.lean_enabled
    imu_ok = _imuThread is not None and _imuThread.connected
    
    # Update motion lean based on gait direction (works without IMU)
    if lean_enabled and _gaitActive and _gaitEngine is not None:
        heading_deg = getattr(_gaitEngine.params, 'heading_deg', 0.0)
        speed_scale = getattr(_gaitEngine.params, 'speed_scale', 0.0)
        _levelingState.update_motion_lean(heading_deg, speed_scale)
    elif leveling_state_ok:
        # Not moving - decay lean to zero
        _levelingState.update_motion_lean(0.0, 0.0)
    
    if leveling_enabled and imu_ok:
        roll_deg, pitch_deg, _ = _imuThread.get_orientation()
        # Update leveling state with filtered orientation (also checks tilt safety)
        _levelingState.update(pitch_deg, roll_deg)
        # Only apply corrections if tilt is safe
        if _levelingState.is_tilt_safe():
            # Use combined corrections (leveling + lean)
            if lean_enabled:
                corrections = _levelingState.compute_combined_corrections()
            else:
                corrections = _levelingState.compute_corrections()
            cmd_to_send = leveling_module.build_corrected_feet_cmd(feet_cmd, corrections)
            # Debug: show roll/pitch and max correction applied
            if _levelingDebugCount % 100 == 0:
                max_corr = max(abs(c) for c in corrections)
                lean_p, lean_r = _levelingState.get_lean_offset()
                if lean_enabled and (abs(lean_p) > 0.1 or abs(lean_r) > 0.1):
                    print(f"[LVL] R:{roll_deg:+.1f}° P:{pitch_deg:+.1f}° lean:P{lean_p:+.1f}°/R{lean_r:+.1f}° max:{max_corr:.1f}mm", end="\r\n")
                else:
                    print(f"[LVL] R:{roll_deg:+.1f}° P:{pitch_deg:+.1f}° max_corr:{max_corr:.1f}mm", end="\r\n")
        elif _levelingDebugCount % 50 == 0:
            print(f"[LVL] TILT UNSAFE - skipping corrections", end="\r\n")
    elif lean_enabled and leveling_state_ok:
        # Lean only (no IMU/leveling) - still apply lean corrections
        corrections = _levelingState.compute_lean_corrections()
        if any(abs(c) > 0.1 for c in corrections):
            cmd_to_send = leveling_module.build_corrected_feet_cmd(feet_cmd, corrections)
            if _levelingDebugCount % 100 == 0:
                lean_p, lean_r = _levelingState.get_lean_offset()
                max_corr = max(abs(c) for c in corrections)
                print(f"[LEAN] P:{lean_p:+.1f}° R:{lean_r:+.1f}° max:{max_corr:.1f}mm", end="\r\n")
    elif _levelingDebugCount % 250 == 0:
        # Periodic diagnostic if leveling not active
        print(f"[LVL] OFF: state={leveling_state_ok} en={leveling_enabled} lean={lean_enabled} imu={imu_ok}", end="\r\n")
    
    if not force and time_since_last < _feetMaxSkipMs:
        positions = _parse_feet_positions(cmd_to_send)
        if positions and not _feet_changed_enough(positions, _feetToleranceMm):
            if _debugSendAll:
                print(f"[send_feet_cmd] FILTERED (delta < {_feetToleranceMm}mm, {time_since_last:.1f}ms ago)", end="\r\n")
            return False
    else:
        # Update last positions even on force/timeout send
        positions = _parse_feet_positions(cmd_to_send)
        if positions:
            _lastFeetPositions = positions[:]
    
    _lastFeetSendTime = now
    return send_cmd(cmd_to_send, force=True)


def send_cmd(cmd, force: bool = False, throttle_ms: float = None):
    """Send command to Teensy as bytes terminated by \n.
    Accepts str or bytes; normalizes to ASCII bytes.
    Suppresses duplicates within throttle window unless force=True.
    Tracks local enable state. Returns True if transmitted."""
    global _cmd_last, _enabledLocal, _teensy
    if throttle_ms is None:
        throttle_ms = _cmdThrottleMs
    if _teensy is None:
        return False
    # normalize to bytes
    if isinstance(cmd, bytes):
        cmd_b = cmd
    elif isinstance(cmd, str):
        try:
            cmd_b = cmd.encode('ascii')
        except Exception:
            return False
    else:
        return False
    now = time.time()
    last = _cmd_last.get(cmd_b)
    if (not force) and last is not None and (now - last) * 1000.0 < throttle_ms:
        if _debugSendAll or (_verbose and cmd_b in _gaitDebugCmds):
            print(f"[send_cmd] SUPPRESSED cmd='{cmd_b}' dt={(now-last)*1000.0:.1f}ms < throttle={throttle_ms}ms", end="\r\n")
        return False
    try:
        _teensy.write(cmd_b + b'\n')
    except Exception as e:
        if _verbose:
            print(f"send_cmd error for '{cmd_b}': {e}", end="\r\n")
        return False
    _cmd_last[cmd_b] = now
    if cmd_b == b'ENABLE':
        _enabledLocal = True
    elif cmd_b == b'DISABLE':
        _enabledLocal = False
        audio_disable()  # Play disable sound
        speak_disabled()  # Voice announcement
    if _debugSendAll or (_verbose and cmd_b in _gaitDebugCmds):
        print(f"[send_cmd] SENT cmd='{cmd_b}' force={force} throttle_ms={throttle_ms} t={now:.3f}", end="\r\n")
    return True


# Tuck auto-disable scheduling (local state, synced with posture_module)
_autoDisableAt = None  # when not None, time (epoch seconds) at which to send DISABLE
_autoDisableReason = None  # string reason for pending auto-disable (e.g., 'TUCK')
_autoDisableGen = 0  # generation counter to detect stale timers
_lastPosture = None  # tracks last posture name sent

# Posture functions delegated to posture_module (Phase 3 modularization)
# These wrappers maintain the original API while using the extracted module.

def ensure_enabled() -> bool:
    """Ensure robot is enabled by sending LEG ALL ENABLE + ENABLE if needed.
    Wrapper around posture_module.ensure_enabled().
    """
    global _enabledLocal
    _ctrl = globals().get('ctrl', None)
    system_telem = getattr(_ctrl, 'system_telem', None) if _ctrl is not None else None
    
    def _send_cmd_wrapper(cmd: bytes, force: bool) -> bool:
        return send_cmd(cmd, force=force)
    
    sent, new_enabled = posture_module.ensure_enabled(
        teensy=_teensy,
        state=_state,
        safety_state=_safety_state,
        system_telem=system_telem,
        enabled_local=_enabledLocal,
        send_cmd_fn=_send_cmd_wrapper,
        verbose=_verbose
    )
    if sent:
        _enabledLocal = new_enabled
        audio_enable()  # Play enable sound
        speak_enabled()  # Voice announcement
    return sent


def apply_posture(name, auto_disable_s: float = None, require_enable: bool = True):
    """Unified posture helper (TUCK/STAND/HOME) using byte commands.
    Wrapper around posture_module.apply_posture().
    """
    global _enabledLocal, _autoDisableAt, _autoDisableReason, _autoDisableGen, _lastPosture
    if auto_disable_s is None:
        auto_disable_s = _autoDisableS
    
    _ctrl = globals().get('ctrl', None)
    system_telem = getattr(_ctrl, 'system_telem', None) if _ctrl is not None else None
    
    def _send_cmd_wrapper(cmd: bytes, force: bool) -> bool:
        return send_cmd(cmd, force=force)
    
    (success, new_enabled, last_posture,
     new_disable_at, new_disable_reason, new_disable_gen) = posture_module.apply_posture(
        name=name,
        teensy=_teensy,
        state=_state,
        safety_state=_safety_state,
        system_telem=system_telem,
        enabled_local=_enabledLocal,
        send_cmd_fn=_send_cmd_wrapper,
        auto_disable_at=_autoDisableAt,
        auto_disable_reason=_autoDisableReason,
        auto_disable_gen=_autoDisableGen,
        auto_disable_s=auto_disable_s,
        require_enable=require_enable,
        verbose=_verbose
    )
    _enabledLocal = new_enabled
    _lastPosture = last_posture
    _autoDisableAt = new_disable_at
    _autoDisableReason = new_disable_reason
    _autoDisableGen = new_disable_gen
    
    # Play posture confirmation sound and speech
    if success:
        posture_name = name.decode('ascii').upper() if isinstance(name, bytes) else str(name).upper()
        if posture_name == 'STAND':
            audio_stand()
            speak_stand()
        elif posture_name == 'TUCK':
            audio_tuck()
            speak_tuck()
        elif posture_name == 'HOME':
            audio_home()
    
    return success


def start_pounce_move(source: str = "") -> bool:
    """Start the kinematic Pounce move using current [pounce] settings.
    Wrapper around posture_module.start_pounce_move().
    """
    global _gaitActive, _gaitEngine, _moveActive, _moveEngine, _moveTickCount, _autoDisableAt
    
    pounce_params = {
        'prep_ms': _pouncePrepMs,
        'rear_ms': _pounceRearMs,
        'lunge_ms': _pounceLungeMs,
        'recover_ms': _pounceRecoverMs,
        'back1_z_mm': _pounceBack1Z,
        'back2_z_mm': _pounceBack2Z,
        'push_z_mm': _pouncePushZ,
        'strike_z_mm': _pounceStrikeZ,
        'crouch_dy_mm': _pounceCrouchDy,
        'lift_dy_mm': _pounceLiftDy,
        'front_z_mm': _pounceFrontZ,
    }
    
    def _send_cmd_wrapper(cmd: bytes, force: bool) -> bool:
        return send_cmd(cmd, force=force)
    
    def _hide_menu():
        if _marsMenu.visible:
            _marsMenu.hide()
    
    result = posture_module.start_pounce_move(
        teensy=_teensy,
        safety_state=_safety_state,
        gait_active=_gaitActive,
        gait_engine=_gaitEngine,
        gait_transition=_gaitTransition,
        saved_gait_width_mm=_savedGaitWidthMm,
        gait_base_y_mm=_gaitBaseYMm,
        pounce_params=pounce_params,
        send_cmd_fn=_send_cmd_wrapper,
        ensure_enabled_fn=ensure_enabled,
        hide_menu_fn=_hide_menu,
        verbose=_verbose,
        source=source
    )
    success, new_gait_active, move_engine, move_active, clear_auto_disable = result
    
    if success:
        _gaitActive = new_gait_active
        _moveEngine = move_engine
        _moveActive = move_active
        _moveTickCount = 0
        if clear_auto_disable:
            _autoDisableAt = None
    
    return success

#----------------------------------------------------------------------------------------------------------------------
# Main Loop Phase Functions
# Split the main loop into distinct phases for clarity and maintainability.
#----------------------------------------------------------------------------------------------------------------------

def phase_timing_update():
    """Phase 1: Update loop timing and screen refresh scheduling.
    
    Returns:
        tuple: (loopdt, forceDisplayUpdate) - loop delta time and display refresh flag
    """
    global _loopStartTime, _screenRefreshms, _forceDisplayUpdate
    
    curTime = time.time()
    loopdt = curTime - _loopStartTime
    _loopStartTime = curTime
    
    # Check for screen refresh timing
    _screenRefreshms -= loopdt * 1000.0
    force_display = False
    if _screenRefreshms <= 0.0:
        _screenRefreshms = 100.0
        force_display = True
        _forceDisplayUpdate = True
    
    if _showLoopTime:
        print(f"Loop time: {loopdt:.4f} seconds, {_screenRefreshms:.2f} ms remaining until next screen refresh", end="\r\n")
    
    return loopdt, force_display


def phase_teensy_connection(ctrl):
    """Phase 2: Manage Teensy connection and polling.
    
    Args:
        ctrl: Controller instance
    """
    if ctrl.teensy is None:
        # Apply serial config settings from controller.ini
        port_override = _cfg.get('serial', 'port', fallback='').strip() if 'serial' in _cfg else ''
        baud_override = _cfg.getint('serial', 'baud', fallback=1000000) if 'serial' in _cfg else 1000000
        ctrl.connect_teensy(port_override if port_override else None, baud_override)
    else:
        ctrl.poll_teensy()


def phase_gamepad_connection(ctrl):
    """Phase 3: Manage Xbox controller connection.
    
    Supports two modes:
    - Socket mode (preferred): Connect to joy_controller.py daemon via Unix socket
    - Legacy mode (fallback): Direct evdev connection to Xbox controller
    
    Args:
        ctrl: Controller instance
    """
    global _joyClient, _useJoySocket
    
    if ctrl.useJoySocket:
        # Socket mode: connect to joy_controller daemon
        if ctrl.joyClient is None:
            ctrl.joyClient = JoyClient(verbose=ctrl.verbose)
            _joyClient = ctrl.joyClient  # Update global reference
        
        # Attempt connection (non-blocking, rate-limited internally)
        if not ctrl.joyClient.connected:
            ctrl.joyClient.connect()
    else:
        # Legacy evdev mode: direct Xbox controller access
        if ctrl.retryCount == 0 and ctrl.controller is None:
            ctrl.controller = testForGamePad(ctrl.verbose)
            ctrl.retryCount = 100
        elif ctrl.controller is None:
            ctrl.retryCount -= 1


def phase_display_update(ctrl, displayThread, gaitEngine, gaitActive):
    """Phase 4: Handle display updates (thread or direct).
    
    Args:
        ctrl: Controller instance
        displayThread: DisplayThread instance or None
        gaitEngine: Current gait engine or None
        gaitActive: Whether gait is currently active
    """
    # Update INFO menu items from telemetry (only if menu visible to save cycles)
    if _marsMenu is not None and _marsMenu.visible:
        update_menu_info(ctrl)
    
    if displayThread is not None and displayThread.is_alive():
        # Compute eye look direction from joystick inputs
        look_x = 0.0
        look_y = 0.0
        if gaitEngine is not None and hasattr(gaitEngine, 'params'):
            look_x = gaitEngine.params.heading_deg / 90.0  # Normalize -90..+90 to -1..+1
        if hasattr(ctrl, '_gaitSpeedInput'):
            look_y = ctrl._gaitSpeedInput  # Already -1 to +1
        
        # Determine telemetry staleness (connected but no data yet)
        telemetry_stale = (ctrl.teensy is not None and ctrl.lastTelemetryTime is None)
        # Robot enabled state from S1 telemetry
        if getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid:
            robot_enabled = ctrl.system_telem.robot_enabled
        else:
            robot_enabled = (ctrl.state[IDX_ROBOT_ENABLED] == 1.0) if (ctrl.state and len(ctrl.state) > IDX_ROBOT_ENABLED) else True
        safety_active, safety_text = get_safety_overlay_state()
        
        # Determine controller connection status (socket mode or legacy)
        if ctrl.useJoySocket:
            controller_connected = (ctrl.joyClient is not None and ctrl.joyClient.xbox_connected)
        else:
            controller_connected = (ctrl.controller is not None)
        
        # Update thread state - it handles rendering at its own rate
        displayThread.update_state(
            servo=ctrl.servo, legs=ctrl.legs, state=ctrl.state,
            mirror=ctrl.mirror, menu_state=ctrl.menuState,
            force_update=ctrl.forceDisplayUpdate, verbose=ctrl.verbose,
            look_x=look_x, look_y=look_y,
            teensy_connected=(ctrl.teensy is not None),
            controller_connected=controller_connected,
                telemetry_stale=telemetry_stale,
                robot_enabled=robot_enabled,
                safety_active=safety_active,
                safety_text=safety_text
        )
        
        # Update IMU state for display overlay
        imu_frame = get_imu_frame()
        displayThread.update_imu(
            connected=imu_frame.valid,
            roll=imu_frame.roll_deg,
            pitch=imu_frame.pitch_deg,
            yaw=imu_frame.yaw_deg,
        )
        
        # Update ToF state for display (engineering view)
        if _tofThread is not None and _tofThread.connected:
            tof_frame = _tofThread.get_frame()
            # Get first sensor's data (typically "front")
            sensor_names = list(tof_frame.sensors.keys())
            if sensor_names:
                sensor_frame = tof_frame.sensors[sensor_names[0]]
                displayThread.update_tof(
                    connected=sensor_frame.valid,
                    distances=sensor_frame.distance_mm,
                    statuses=sensor_frame.status,
                )
            else:
                displayThread.update_tof(connected=False)
        else:
            displayThread.update_tof(connected=False)
        
        # Update display mode
        displayThread.set_display_mode(_displayMode)
        
        ctrl.forceDisplayUpdate = False
        
        # Show mirror window from main thread (cv2 GUI requires main thread on Linux)
        show_mirror_window(force_update_callback=lambda: setattr(ctrl, 'forceDisplayUpdate', True))
    elif not gaitActive:
        # Direct update when thread not running and gait not active
        ctrl.update_display()


def phase_touch_input(menu, ctrl):
    """Phase 5: Handle touchscreen input.
    
    If the robot is in motion (gait active), any touch acts as an emergency stop:
    stops gait, sends idle + LEG ALL DISABLE + DISABLE.
    Otherwise, normal menu touch handling.
    
    Args:
        menu: Menu instance (legacy, kept for compatibility)
        ctrl: Controller instance (for forceDisplayUpdate)
        
    Returns:
        bool: True if menu was touched and processed
    """
    global _menuVisible, _forceDisplayUpdate, _gaitActive, _gaitEngine
    
    # Use _marsMenu.touched() for debounced touch detection
    if _marsMenu.touched():
        # Emergency stop: if robot is in motion, stop everything
        if _gaitActive:
            if _gaitEngine is not None:
                _gaitEngine.stop()
            _gaitActive = False
            if _teensy is not None:
                send_cmd('I', force=True)
                time.sleep(0.05)
                send_cmd(b'LEG ALL DISABLE', force=True)
                time.sleep(0.05)
                send_cmd(b'DISABLE', force=True)
            if _verbose:
                print("\r\nTouchscreen E-STOP: gait stopped, robot disabled", end="\r\n")
            ctrl.forceDisplayUpdate = True
            return True
        
        # MARS menu touch handling when menu is visible
        if _marsMenu.visible:
            _marsMenu.handle_touch()
            ctrl.forceDisplayUpdate = True
            return True
        
        # If robot is disabled and not in motion, allow opening menu
        if getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid:
            robot_enabled = ctrl.system_telem.robot_enabled
        else:
            robot_enabled = (_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False
        if not robot_enabled and not _gaitActive:
            _marsMenu.show()
            ctrl.forceDisplayUpdate = True
            return True
    
    return False


def phase_keyboard_input(ctrl):
    """Phase 6: Process keyboard commands.
    
    Args:
        ctrl: Controller instance
        
    Returns:
        bool: False if exit was requested, True to continue
    """
    global _run, _verbose, _debugTelemetry, _debugSendAll, _mirrorDisplay
    global _menuState, _menuVisible, _forceDisplayUpdate
    global _gaitEngine, _gaitActive, _autoDisableAt
    
    key = poll_keyboard()

    # When MARS menu is visible, arrows/A/D/enter/esc keys drive the menu
    if _marsMenu is not None and _marsMenu.visible and key != -1:
        if key in (curses.KEY_UP, ord('k')):
            _marsMenu.nav_up()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (curses.KEY_DOWN, ord('j')):
            _marsMenu.nav_down()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (curses.KEY_LEFT, ord('h')):
            _marsMenu.nav_tab_left()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (curses.KEY_RIGHT, ord('l')):
            _marsMenu.nav_tab_right()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (ord('a'), ord('A')):
            # Adjust current value/option left
            _marsMenu.nav_left()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (ord('d'), ord('D')):
            # Adjust current value/option right
            _marsMenu.nav_right()
            ctrl.forceDisplayUpdate = True
            return True
        elif key in (10, 13, ord(' ')):  # Enter or space
            _marsMenu.select()
            ctrl.forceDisplayUpdate = True
            return True
        elif key == 27:  # ESC closes menu instead of exiting app
            _marsMenu.hide()
            ctrl.forceDisplayUpdate = True
            if _verbose:
                print("\r\nMARS menu closed (ESC)", end="\r\n")
            return True

    if key == 27:  # ESC key with no menu open: exit application
        if _verbose:
            print("\r\nESC key pressed, exiting loop", end="\r\n")
        return False
    
    elif key in (ord('t'), ord('T')):  # Test gait
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\r\nTest gait blocked: firmware safety lockout is active.", end="\r\n")
        else:
            enabled_now = (ctrl.system_telem.robot_enabled if (getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid)
                           else ((_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False))
            if not enabled_now:
                ensure_enabled()
            send_cmd(b'T', force=True)
            if _verbose:
                print("\r\nTest gait command sent to Teensy", end="\r\n")
    
    elif key in (ord('k'), ord('K')):  # Tuck posture
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\r\nTUCK blocked: firmware safety lockout is active.", end="\r\n")
        else:
            enabled_now = (ctrl.system_telem.robot_enabled if (getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid)
                           else ((_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False))
            if not enabled_now:
                ensure_enabled()
            send_cmd(b'TUCK', force=True)
            _autoDisableAt = time.time() + _autoDisableS
            ctrl.autoDisableAt = _autoDisableAt
    
    elif key in (ord('s'), ord('S')):  # Stand posture (gait-based for leveling)
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\r\nSTAND blocked: firmware safety lockout is active.", end="\r\n")
        else:
            enabled_now = (ctrl.system_telem.robot_enabled if (getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid)
                           else ((_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False))
            if not enabled_now:
                ensure_enabled()
            # Use standing gait instead of firmware STAND command
            _start_standing_gait()
    
    elif key in (ord('n'), ord('N')):  # Cycle display mode (EYES → ENGINEERING → MENU)
        global _displayMode
        # Get robot state for menu safety check
        if getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid:
            robot_enabled = ctrl.system_telem.robot_enabled
        else:
            robot_enabled = (_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False
        
        # Cycle: EYES → ENGINEERING → MENU → EYES (wrap)
        current_mode = _displayMode
        next_mode = DisplayMode((current_mode + 1) % DisplayMode.COUNT)
        
        # If trying to enter MENU mode, check safety
        if next_mode == DisplayMode.MENU:
            if robot_enabled or _gaitActive:
                # Skip menu, go to EYES instead
                next_mode = DisplayMode.EYES
                if _verbose:
                    print("\r\nMenu blocked: disable robot first", end="\r\n")
        
        # Apply mode change
        _displayMode = next_mode
        
        # Sync menu visibility with mode
        if _displayMode == DisplayMode.MENU:
            _marsMenu.show()
        else:
            _marsMenu.hide()
        
        ctrl.forceDisplayUpdate = True
        if _verbose:
            mode_names = ["EYES", "ENGINEERING", "MENU"]
            print(f"\r\nDisplay mode: {mode_names[_displayMode]} (keyboard 'n')", end="\r\n")
    
    elif key in (ord('m'), ord('M')):  # Mirror toggle
        _mirrorDisplay = not _mirrorDisplay
        ctrl.mirror = _mirrorDisplay
        if _verbose:
            print("\r\nMirror display mode ON" if _mirrorDisplay else "\r\nMirror display mode OFF", end="\r\n")
        _forceDisplayUpdate = True
        ctrl.forceDisplayUpdate = True
    
    elif key in (ord('v'), ord('V')):  # Verbose toggle
        _verbose = not _verbose
        ctrl.verbose = _verbose
        print("\r\nverbose mode ON" if _verbose else "\r\nverbose mode OFF", end="\r\n")
    
    elif key in (ord('d'), ord('D')):  # Telemetry debug toggle
        _debugTelemetry = not _debugTelemetry
        print("Telemetry debug ON" if _debugTelemetry else "Telemetry debug OFF", end="\r\n")
    
    elif key in (ord('g'), ord('G')):  # send_cmd debug toggle
        _debugSendAll = not _debugSendAll
        print("send_cmd debug ALL ON" if _debugSendAll else "send_cmd debug ALL OFF", end="\r\n")
    
    elif key in (ord('e'), ord('E')):  # Cycle eye shape
        _eyes.left_shape += 1
        if _eyes.left_shape > EYE_SHAPE.ANIME:
            _eyes.left_shape = EYE_SHAPE.ELLIPSE
        _eyes.right_shape = _eyes.left_shape
        _eyes.eye_color = _eyeColors[_eyes.left_shape]
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
        if _displayThread is not None:
            _displayThread.update_state(force_update=True)
        save_eye_shape(_eyes.left_shape)
        if _verbose:
            shape_names = ["ELLIPSE", "RECTANGLE", "ROUNDRECTANGLE", "X", "SPIDER", "HUMAN", "CAT", "HYPNO", "ANIME"]
            print(f"Eye shape changed to {shape_names[_eyes.left_shape]}", end="\r\n")
    
    elif key in (ord('w'), ord('W')):  # Start gait (walk)
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\r\nGait start blocked: firmware safety lockout is active.", end="\r\n")
        elif not _gaitActive:
            enabled_now = (ctrl.system_telem.robot_enabled if (getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid)
                           else ((_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False))
            if not enabled_now:
                ensure_enabled()
            params = GaitParams(
                cycle_ms=2000,
                base_x_mm=100.0,
                base_y_mm=-120.0,
                step_len_mm=40.0,
                lift_mm=15.0,
                overlap_pct=5.0,
                speed_scale=0.0
            )
            _gaitEngine = TripodGait(params)
            _gaitEngine.start()
            _gaitActive = True
            ctrl._gaitSpeedInput = 0.0
            ctrl._gaitStrafeInput = 0.0
            ctrl._updateGaitHeading()
            if _verbose:
                print("\r\nGait engine STARTED (tripod walk)", end="\r\n")
        elif _verbose:
            print("\r\nGait already active", end="\r\n")
    
    elif key in (ord('h'), ord('H')):  # Stop gait (halt)
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.stop()
            _gaitActive = False
            send_cmd(b'STAND', force=True)
            _autoDisableAt = time.time() + _autoDisableS
            ctrl.autoDisableAt = _autoDisableAt
            if _verbose:
                print("\r\nGait engine STOPPED (halt)", end="\r\n")
        elif _verbose:
            print("\r\nNo gait active to stop", end="\r\n")
    
    elif key in (ord('p'), ord('P')):  # Stationary pattern
        if not _gaitActive:
            enabled_now = (ctrl.system_telem.robot_enabled if (getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid)
                           else ((_state[IDX_ROBOT_ENABLED] == 1.0) if (len(_state) > IDX_ROBOT_ENABLED) else False))
            if not enabled_now:
                ensure_enabled()
            params = GaitParams(base_x_mm=100.0, base_y_mm=-120.0)
            _gaitEngine = StationaryPattern(params, radius_mm=15.0, period_ms=2000)
            _gaitEngine.start()
            _gaitActive = True
            if _verbose:
                print("\r\nStationary pattern STARTED", end="\r\n")
        elif _verbose:
            print("\r\nGait already active (press 'h' to stop first)", end="\r\n")

    elif key in (ord('u'), ord('U')):  # Pounce (jump attack)
        start_pounce_move(source="kbd")
    
    elif key in (ord('q'), ord('Q')):  # Global disable + idle
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.stop()
            _gaitActive = False
        if _teensy is not None:
            send_cmd('I', force=True)
            time.sleep(0.05)
            send_cmd(b'LEG ALL DISABLE', force=True)
            time.sleep(0.05)
            send_cmd(b'DISABLE', force=True)
        if _verbose:
            print("\r\n'q' key pressed: idle + LEG ALL DISABLE + DISABLE sequence sent", end="\r\n")
    
    elif key in (ord('x'), ord('X')):  # Exit
        if _verbose:
            print("\r\n'x' key pressed, exiting loop", end="\r\n")
        return False
    
    elif key in (ord('a'), ord('A')):  # Toggle autonomy mode
        _toggle_autonomy(verbose=_verbose)
    
    elif key in (ord('?'),):  # Help - show keyboard commands
        print_keyboard_help()
    
    return True


def print_keyboard_help():
    """Print keyboard command help to console."""
    lines = [
        "╔═══════════════════════════════════════════════════════════════════════╗",
        "║                    MARS Controller Keyboard Commands                  ║",
        "╠═══════════════════════════════════════════════════════════════════════╣",
        "║  MOTION                                                               ║",
        "║    w/W    Start tripod walk gait                                      ║",
        "║    h/H    Halt gait (stop walking)                                    ║",
        "║    p/P    Start stationary pattern (march in place)                   ║",
        "║    s/S    Stand posture (with body leveling)                          ║",
        "║    k/K    Tuck posture (crouch down)                                  ║",
        "║    t/T    Test gait (firmware tripod test)                            ║",
        "║    u/U    Pounce move (jump attack)                                   ║",
        "║                                                                       ║",
        "║  AUTONOMY                                                             ║",
        "║    a/A    Toggle autonomy mode on/off                                 ║",
        "║                                                                       ║",
        "║  DISPLAY                                                              ║",
        "║    n/N    Cycle display mode (Eyes -> Engineering -> Menu)            ║",
        "║    m/M    Toggle mirror display window                                ║",
        "║    e/E    Cycle eye shape                                             ║",
        "║                                                                       ║",
        "║  DEBUG                                                                ║",
        "║    v/V    Toggle verbose output                                       ║",
        "║    d/D    Toggle telemetry debug                                      ║",
        "║    g/G    Toggle send_cmd debug (all commands)                        ║",
        "║                                                                       ║",
        "║  SYSTEM                                                               ║",
        "║    q/Q    Emergency stop (idle + disable all)                         ║",
        "║    x/X    Exit controller                                             ║",
        "║    ESC    Exit controller (or close menu if open)                     ║",
        "║    ?      Show this help                                              ║",
        "║                                                                       ║",
        "║  MENU NAVIGATION (when menu visible)                                  ║",
        "║    up/k   Navigate up                                                 ║",
        "║    dn/j   Navigate down                                               ║",
        "║    lt/h   Previous tab                                                ║",
        "║    rt/l   Next tab                                                    ║",
        "║    a/A    Adjust value left / previous option                         ║",
        "║    d/D    Adjust value right / next option                            ║",
        "║    Enter  Select / activate item                                      ║",
        "║    ESC    Close menu                                                  ║",
        "╚══════════════════════════════════════════════════════════════════════╝",
    ]
    import sys
    for line in lines:
        sys.stdout.write(line + "\r\n")
    sys.stdout.flush()


def _toggle_autonomy(verbose: bool = True) -> None:
    """Toggle autonomy mode on/off.
    
    Used by keyboard 'a' key and Back+A gamepad combo.
    
    Args:
        verbose: Print status messages if True
    """
    global _autonomyEnabled, _behaviorArbiter
    _autonomyEnabled = not _autonomyEnabled
    if _autonomyEnabled:
        # Initialize arbiter if not already done
        if _behaviorArbiter is None:
            _behaviorArbiter = _init_behavior_arbiter()
        audio_autonomy_on()
        speak_autonomy_on()
        if verbose:
            print(f"\r\nAUTONOMY MODE ENABLED ({_behaviorArbiter.behavior_count} behaviors)", end="\r\n")
    else:
        audio_autonomy_off()
        speak_autonomy_off()
        if verbose:
            print("\r\nAUTONOMY MODE DISABLED", end="\r\n")


def _disable_autonomy_if_active(verbose: bool = True) -> None:
    """Disable autonomy mode if currently active.
    
    Called when ABXY buttons are pressed to give user manual control.
    
    Args:
        verbose: Print status message if True and autonomy was active
    """
    global _autonomyEnabled
    if _autonomyEnabled:
        _autonomyEnabled = False
        if verbose:
            print("  (Autonomy auto-disabled)", end="\r\n")


def _init_behavior_arbiter():
    """Initialize behavior arbiter with configured behaviors.
    
    Returns:
        BehaviorArbiter: Configured arbiter instance
    """
    arbiter = BehaviorArbiter()
    
    # Add behaviors based on config enables
    if _autonomyCliffDetection:
        arbiter.add_behavior(CliffDetection(
            cliff_threshold_mm=_autonomyCliffThresholdMm,
            normal_floor_mm=200,  # Will be calibrated
            priority=90,
            enabled=True
        ))
    
    if _autonomyCaughtFootRecovery:
        arbiter.add_behavior(CaughtFootRecovery(
            position_error_threshold_deg=_autonomySnagErrorDeg,
            timeout_ms=_autonomySnagTimeoutMs,
            recovery_lift_mm=_autonomyRecoveryLiftMm,
            max_attempts=_autonomyMaxRecoveryAttempts,
            priority=85,
            enabled=True
        ))
    
    if _autonomyObstacleAvoidance:
        arbiter.add_behavior(ObstacleAvoidance(
            stop_distance_mm=_autonomyStopDistMm,
            slow_distance_mm=_autonomySlowDistMm,
            priority=80,
            enabled=True
        ))
    
    # Wall following (priority 40: above Patrol, below safety behaviors)
    if _autonomyWallFollow:
        arbiter.add_behavior(WallFollowing(
            wall_distance_mm=_autonomyWallDistMm,
            wall_side=_autonomyWallSide,
            priority=40,
            enabled=True
        ))
    
    if _autonomyPatrol:
        arbiter.add_behavior(Patrol(
            turn_interval_s=_autonomyTurnIntervalS,
            patrol_duration_s=_autonomyPatrolDurationS,
            priority=20,
            enabled=True
        ))
    
    return arbiter


def _gather_sensor_state(ctrl):
    """Gather current sensor state for behavior engine.
    
    Returns:
        dict: Sensor state for behavior compute() methods
    """
    state = {
        'tof_connected': False,
        'tof_distances': [],
        'tof_statuses': [],
        'servo_positions': [],
        'servo_targets': [],
        'gait_active': _gaitActive,
        'touch_active': False,
    }
    
    # Touch detection for touch-stop patrol
    if _marsMenu is not None:
        state['touch_active'] = _marsMenu.touched()
    
    # ToF data
    if _tofThread is not None and _tofThread.connected:
        tof_frame = _tofThread.get_frame()
        if tof_frame is not None:
            state['tof_connected'] = True
            # Get first sensor's data (front)
            sensor_names = _tofThread.get_sensor_names()
            if sensor_names and sensor_names[0] in tof_frame.sensors:
                sensor_data = tof_frame.sensors[sensor_names[0]]
                state['tof_distances'] = list(sensor_data.distance_mm)
                state['tof_statuses'] = list(sensor_data.status)
    
    # Servo positions from telemetry
    if hasattr(ctrl, 'joint_telem') and ctrl.joint_telem.valid:
        angles = ctrl.joint_telem.angles_deg
        if angles and len(angles) >= 18:
            state['servo_positions'] = list(angles)
    
    # Servo targets from gait engine
    if _gaitEngine is not None and hasattr(_gaitEngine, 'get_joint_targets'):
        targets = _gaitEngine.get_joint_targets()
        if targets and len(targets) >= 18:
            state['servo_targets'] = list(targets)
    
    return state


def phase_autonomy(ctrl):
    """Phase 7b: Run autonomy behaviors and apply actions.
    
    Runs behavior arbiter to get navigation action, then modifies
    gait parameters or stops motion as needed.
    
    Args:
        ctrl: Controller instance
    """
    global _lastAutonomyAction, _gaitActive, _gaitEngine, _autonomyEnabled
    
    if not _autonomyEnabled or _behaviorArbiter is None:
        _lastAutonomyAction = None
        return
    
    # Gather sensor state
    sensor_state = _gather_sensor_state(ctrl)
    
    # Run arbiter
    action_req = _behaviorArbiter.update(sensor_state)
    _lastAutonomyAction = action_req
    
    # Debug: show what autonomy is doing (only when action != CONTINUE)
    if _verbose and action_req.action != Action.CONTINUE:
        print(f"\r\n[Autonomy] {action_req.action.name}: {action_req.reason}", end="\r\n")
    
    # Apply action
    if action_req.action == Action.EMERGENCY_STOP:
        # Immediate stop and disable
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.stop()
        _gaitActive = False
        _autonomyEnabled = False
        send_cmd(b'DISABLE', force=True)
        # Announce cliff if reason mentions cliff/edge/drop
        if action_req.reason and ('cliff' in action_req.reason.lower() or 'edge' in action_req.reason.lower() or 'drop' in action_req.reason.lower()):
            speak_cliff()
        if _verbose:
            print(f"\r\n!!! AUTONOMY E-STOP: {action_req.reason}", end="\r\n")
    
    elif action_req.action == Action.STOP:
        # Stop gait but remain enabled
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.stop()
            _gaitActive = False
            send_cmd(b'STAND', force=True)
            # Announce obstacle if reason mentions obstacle/blocked
            if action_req.reason and ('blocked' in action_req.reason.lower() or 'obstacle' in action_req.reason.lower()):
                speak_obstacle()
        if _verbose and action_req.reason:
            print(f"\r\nAutonomy STOP: {action_req.reason}", end="\r\n")
    
    elif action_req.action == Action.BACK_UP:
        # Set gait heading to reverse
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.params.heading_deg = 180.0
            _gaitEngine.params.speed_scale = 0.5 * action_req.intensity
    
    elif action_req.action == Action.TURN_LEFT:
        # Apply left turn via heading (strafe left = -90 deg)
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.params.heading_deg = -90.0 * action_req.intensity
            _gaitEngine.params.speed_scale = 0.5
    
    elif action_req.action == Action.TURN_RIGHT:
        # Apply right turn via heading (strafe right = +90 deg)
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.params.heading_deg = 90.0 * action_req.intensity
            _gaitEngine.params.speed_scale = 0.5
    
    elif action_req.action == Action.SLOW_DOWN:
        # Reduce speed
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.params.speed_scale = 0.3 + 0.5 * (1.0 - action_req.intensity)
    
    elif action_req.action == Action.WALK_FORWARD:
        # Start/maintain forward walking
        if not _gaitActive:
            # Gait not running - need to start it
            # Enable robot if needed
            enabled_now = ctrl.system_telem.robot_enabled if ctrl.system_telem.valid else False
            if not enabled_now:
                ensure_enabled()
            
            # Create gait engine if needed
            if _gaitEngine is None:
                params = GaitParams(
                    cycle_ms=_gaitCycleMs,
                    base_x_mm=_savedGaitWidthMm,
                    base_y_mm=_gaitBaseYMm,
                    step_len_mm=_gaitStepLenMm,
                    lift_mm=_savedGaitLiftMm,
                    overlap_pct=_gaitOverlapPct,
                    smoothing_alpha=_gaitSmoothingAlpha,
                    bezier_p1_height=_bezierP1Height,
                    bezier_p1_overshoot=_bezierP1Overshoot,
                    bezier_p2_height=_bezierP2Height,
                    bezier_p3_height=_bezierP3Height,
                    bezier_p3_overshoot=_bezierP3Overshoot,
                    speed_scale=action_req.intensity,
                    heading_deg=0.0,  # Forward
                )
                _gaitEngine = TripodGait(params)
                send_cmd(b'MODE IDLE', force=True)  # Switch Teensy to IDLE mode
            
            _gaitEngine.start()
            _gaitActive = True
            if _verbose:
                print("\r\n[Autonomy] Started gait engine (WALK_FORWARD)", end="\r\n")
        
        # Set forward direction and speed (even if gait was already running)
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.params.heading_deg = 0.0  # Forward
            _gaitEngine.params.speed_scale = action_req.intensity  # 0.7 from patrol
    
    elif action_req.action == Action.CONTINUE:
        # Normal operation - check if we need to apply recovery data
        if action_req.data and action_req.data.get('recovery_type') == 'lift':
            # Caught foot recovery - increase step height
            extra_lift = action_req.data.get('extra_lift_mm', 30.0)
            if _gaitEngine is not None and hasattr(_gaitEngine, 'params'):
                _gaitEngine.params.lift_mm += extra_lift
                if _verbose:
                    print(f"\r\nAutonomy: Lifting leg {action_req.data.get('leg')} +{extra_lift}mm", end="\r\n")
    
    # Update autonomy menu status
    if _marsMenu is not None:
        active_name = _behaviorArbiter.active_behavior_name or "---"
        action_name = action_req.action.name if action_req.action != Action.CONTINUE else "OK"
        _marsMenu.set_value(MenuCategory.AUTO, "Status", "Running" if _gaitActive else "Idle")
        _marsMenu.set_value(MenuCategory.AUTO, "Active Behavior", active_name)
        _marsMenu.set_value(MenuCategory.AUTO, "Last Action", action_name)


def phase_gait_tick(ctrl, gaitEngine, gaitActive):
    """Phase 7: Execute gait engine tick and send FEET commands.
    
    Handles both normal gait operation and smooth phase-locked transitions
    between gait types.
    
    Args:
        ctrl: Controller instance
        gaitEngine: Current gait engine or None
        gaitActive: Whether gait is currently active
    """
    global _gaitTickCount, _gaitEngine, _gaitTransition
    global _moveActive, _moveEngine, _moveTickCount, _moveSendDivisor
    global _autoDisableAt

    # Kinematic move takes priority over cyclic gait
    if _moveActive and _moveEngine is not None and _teensy is not None:
        # Use telemetry-synced period if available, else fallback
        if _telemSyncActive and ctrl.teensyLoopUs > 0:
            dt_seconds = ctrl.teensyLoopUs / 1_000_000.0
        else:
            dt_seconds = 6.024 / 1000.0  # 166Hz fallback

        _moveEngine.tick(dt_seconds)
        _moveTickCount += 1
        if _moveTickCount >= _moveSendDivisor:
            _moveTickCount = 0
            send_feet_cmd(_moveEngine.get_feet_bytes())

        if hasattr(_moveEngine, 'is_complete') and _moveEngine.is_complete():
            _moveActive = False
            _moveEngine = None
            send_cmd(b'STAND', force=True)
            _autoDisableAt = time.time() + _autoDisableS
        return
    
    if not gaitActive or gaitEngine is None or _teensy is None:
        return
    
    # Use telemetry-synced period if available, else fallback
    if _telemSyncActive and ctrl.teensyLoopUs > 0:
        dt_seconds = ctrl.teensyLoopUs / 1_000_000.0
    else:
        dt_seconds = 6.024 / 1000.0  # 166Hz fallback
    
    # Check if transition is active
    if _gaitTransition.is_active():
        # Let transition manager handle ticking both gaits and blending
        feet, is_complete = _gaitTransition.tick(dt_seconds)
        
        if is_complete:
            # Transition finished - switch to new gait
            new_gait = _gaitTransition.get_target_gait()
            if new_gait is not None:
                _gaitEngine = new_gait
                # Find the new gait name for logging
                new_type = type(new_gait)
                try:
                    new_idx = GAIT_TYPES.index(new_type)
                    new_name = GAIT_NAMES[new_idx]
                except ValueError:
                    new_name = "Unknown"
                # Play gait-specific confirmation tone
                audio_gait_changed(new_name)
                if ctrl.verbose:
                    print(f"\r\nGait transition complete: now running {new_name}", end="\r\n")
            _gaitTransition.reset()
        
        # Send blended FEET command
        if feet is not None:
            _gaitTickCount += 1
            if _gaitTickCount >= _gaitSendDivisor:
                _gaitTickCount = 0
                # Format feet as FEET command
                parts = []
                for foot in feet:
                    parts.extend([f"{foot[0]:.1f}", f"{foot[1]:.1f}", f"{foot[2]:.1f}"])
                feet_cmd = ("FEET " + " ".join(parts)).encode('ascii')
                send_feet_cmd(feet_cmd)
    else:
        # Normal gait operation - tick the single gait engine
        gaitEngine.tick(dt_seconds)
        
        # Send FEET command every N ticks to reduce serial load
        _gaitTickCount += 1
        if _gaitTickCount >= _gaitSendDivisor:
            _gaitTickCount = 0
            feet_cmd = gaitEngine.get_feet_bytes()
            send_feet_cmd(feet_cmd)


def phase_pointcloud(ctrl):
    """Phase 7c: Push sensor data to point cloud server for 3D visualization.
    
    Feeds ToF frames and IMU orientation to the point cloud server which
    transforms points to world frame and streams to connected WebSocket clients.
    
    Args:
        ctrl: Controller instance
    """
    global _pointcloudServer, _tofThread, _imuThread
    
    if _pointcloudServer is None:
        return
    
    # Update IMU orientation (world-frame rotation)
    if _imuThread is not None and _imuThread.connected:
        try:
            roll, pitch, yaw = _imuThread.get_orientation()
            if roll is not None and pitch is not None and yaw is not None:
                _pointcloudServer.update_imu(roll, pitch, yaw)
        except Exception:
            pass  # Ignore IMU read errors
    
    # Push ToF frame data (converted to 3D points by server)
    if _tofThread is not None and _tofThread.connected:
        try:
            tof_frame = _tofThread.get_frame()
            if tof_frame is not None:
                for sensor_name, sensor_data in tof_frame.sensors.items():
                    _pointcloudServer.push_tof_frame(
                        sensor_name=sensor_name,
                        distances_mm=list(sensor_data.distance_mm),
                        statuses=list(sensor_data.status),
                    )
        except Exception:
            pass  # Ignore ToF read errors
    
    # Update velocity estimate from gait engine (for position integration)
    if _gaitEngine is not None and hasattr(_gaitEngine, 'get_body_velocity'):
        try:
            vx, vy = _gaitEngine.get_body_velocity()
            _pointcloudServer.update_velocity(vx, vy)
        except Exception:
            pass


def phase_dashboard(ctrl):
    """Phase 7d: Push telemetry data to web dashboard server.
    
    Streams real-time telemetry to connected web browser clients via WebSocket.
    Called from Layer 2 (~30 Hz) since display updates don't need 166 Hz.
    
    Args:
        ctrl: Controller instance
    """
    global _dashboardServer, _gaitEngine, _gaitActive
    global _lowBatteryTriggered, _joyClient
    
    if _dashboardServer is None:
        return
    
    # Skip if no clients connected (avoid unnecessary work)
    if _dashboardServer.client_count == 0:
        return
    
    # Gather telemetry data from various sources
    loop_time_us = 0.0
    battery_v = 0.0
    current_a = 0.0
    imu_pitch = 0.0
    imu_roll = 0.0
    imu_yaw = 0.0
    robot_enabled = False
    safety_state = "---"
    safety_cause = "---"
    servo_temp_max = 0.0
    servo_volt_min = 12.6
    servo_errors = 0
    
    # Get data from system_telem (S1 telemetry - preferred source)
    system = getattr(ctrl, 'system_telem', None)
    if system is not None and getattr(system, 'valid', False):
        loop_time_us = float(getattr(system, 'loop_us', 0) or 0)
        battery_v = float(getattr(system, 'battery_v', 0) or 0)
        current_a = float(getattr(system, 'current_a', 0) or 0)
        imu_pitch = float(getattr(system, 'pitch_deg', 0) or 0)
        imu_roll = float(getattr(system, 'roll_deg', 0) or 0)
        imu_yaw = float(getattr(system, 'yaw_deg', 0) or 0)
        robot_enabled = getattr(system, 'robot_enabled', False)
    
    # If no S1 telemetry, try IMU thread directly for orientation
    if (imu_pitch == 0 and imu_roll == 0) and _imuThread is not None and _imuThread.connected:
        imu_frame = get_imu_frame()
        if imu_frame.valid:
            imu_pitch = imu_frame.pitch_deg
            imu_roll = imu_frame.roll_deg
            imu_yaw = imu_frame.yaw_deg
    
    # Safety state from safety_telem (S5) - the correct source for lockout status
    # NOTE: S1 'safety' field is actually rr_index due to firmware format mismatch
    safety_telem = getattr(ctrl, 'safety_telem', None)
    if safety_telem is not None and getattr(safety_telem, 'valid', False):
        lockout = getattr(safety_telem, 'lockout', False)
        safety_state = "LOCKOUT" if lockout else "OK"
        cause_mask = getattr(safety_telem, 'cause_mask', 0)
        if cause_mask == 0:
            safety_cause = "---"
        else:
            causes = []
            if cause_mask & 0x01:
                causes.append("SOFT_LIMIT")
            if cause_mask & 0x02:
                causes.append("COLLISION")
            if cause_mask & 0x04:
                causes.append("TEMP")
            safety_cause = ",".join(causes) if causes else f"0x{cause_mask:02X}"
    
    # Servo aggregates - prefer structured servo_telem, fallback to ctrl.servo array
    servo_telem = getattr(ctrl, 'servo_telem', None)
    if servo_telem is not None:
        temps = []
        volts = []
        for s in servo_telem:
            if s is None:
                continue
            if getattr(s, 'valid_s3', False):
                t = float(getattr(s, 'temp_c', 0) or 0)
                if t > 0:
                    temps.append(t)
            if getattr(s, 'valid_s2', False):
                v = float(getattr(s, 'voltage_v', 0) or 0)
                if v > 0:
                    volts.append(v)
        if temps:
            servo_temp_max = max(temps)
        if volts:
            servo_volt_min = min(volts)
            # If battery_v is missing, use average servo voltage
            if battery_v <= 0:
                battery_v = sum(volts) / len(volts)
    elif ctrl.servo:
        # Fallback to legacy array format [voltage, temp, ?]
        temps = [s[1] for s in ctrl.servo if len(s) > 1 and s[1] > 0]
        volts = [s[0] for s in ctrl.servo if len(s) > 0 and s[0] > 0]
        if temps:
            servo_temp_max = max(temps)
        if volts:
            servo_volt_min = min(volts)
            if battery_v <= 0:
                battery_v = sum(volts) / len(volts)
    
    # Gait info
    gait_running = _gaitActive
    gait_type = "TRIPOD"  # Default
    gait_speed = 0.0
    if _gaitEngine is not None:
        if hasattr(_gaitEngine, 'gait_name') and _gaitEngine.gait_name:
            gait_type = _gaitEngine.gait_name
        elif hasattr(_gaitEngine, 'gait_type'):
            # Map class to name
            gait_cls = _gaitEngine.gait_type
            for i, cls in enumerate(GAIT_TYPES):
                if gait_cls == cls:
                    gait_type = GAIT_NAMES[i]
                    break
    
    # Xbox connection - check ctrl.joyClient directly (not global _joyClient)
    xbox_connected = False
    if ctrl.useJoySocket and ctrl.joyClient is not None:
        # Use xbox_connected property which checks socket + controller state
        xbox_connected = ctrl.joyClient.xbox_connected
    elif ctrl.controller is not None:
        xbox_connected = True
    
    # Teensy connection
    teensy_connected = ctrl.teensy is not None and ctrl.teensy.is_open
    
    # Push to dashboard server
    _dashboardServer.push_telemetry(
        loop_time_us=loop_time_us,
        battery_v=battery_v,
        current_a=current_a,
        imu_pitch=imu_pitch,
        imu_roll=imu_roll,
        imu_yaw=imu_yaw,
        robot_enabled=robot_enabled,
        safety_state=safety_state,
        safety_cause=safety_cause,
        low_battery_active=_lowBatteryTriggered,
        gait_running=gait_running,
        gait_type=gait_type,
        gait_speed=gait_speed,
        servo_temp_max=servo_temp_max,
        servo_volt_min=servo_volt_min,
        servo_errors=servo_errors,
        ctrl_version=f"v{CONTROLLER_VERSION} b{CONTROLLER_BUILD}",
        fw_version=FW_VERSION_BANNER,
        xbox_connected=xbox_connected,
        teensy_connected=teensy_connected,
    )
    
    # Periodically refresh full config from MarsMenu (~every 2 seconds)
    global _dashboardConfigRefreshCounter
    if not hasattr(phase_dashboard, '_config_counter'):
        phase_dashboard._config_counter = 0
    phase_dashboard._config_counter += 1
    if phase_dashboard._config_counter >= 60:  # ~2 sec at 30 Hz
        phase_dashboard._config_counter = 0
        if _marsMenu is not None:
            _dashboardServer.set_full_config(_marsMenu.get_all_config())


def phase_auto_disable(ctrl):
    """Phase 8: Check and execute scheduled auto-disable and low battery protection.
    
    Args:
        ctrl: Controller instance
    """
    global _autoDisableAt, _autoDisableReason, _autoDisableGen
    global _lowBatteryTriggered, _lowBatteryFilteredVoltage

    # --- Scheduled auto-disable (posture timeouts) ---
    if ctrl.autoDisableAt is not None and time.time() >= ctrl.autoDisableAt:
        # Only honor auto-disable if reason/generation still match and robot is enabled
        reason = getattr(ctrl, 'autoDisableReason', None)
        gen = getattr(ctrl, 'autoDisableGen', 0)
        if gen == _autoDisableGen and reason == _autoDisableReason:
            if getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid:
                enabled_now = ctrl.system_telem.robot_enabled
            else:
                enabled_now = (ctrl.state[IDX_ROBOT_ENABLED] == 1.0) if (ctrl.state and len(ctrl.state) > IDX_ROBOT_ENABLED) else False
            if ctrl.teensy is not None and enabled_now:
                send_cmd(b'DISABLE', force=True)
                if ctrl.verbose:
                    print(f"\r\nAuto DISABLE executed (reason={reason})", end="\r\n")
        # Clear any pending auto-disable (stale or executed)
        ctrl.autoDisableAt = None
        ctrl.autoDisableReason = None
        ctrl.autoDisableGen = gen
        _autoDisableAt = None
        _autoDisableReason = None

    # --- Low battery protection ---
    if not _lowBatteryEnabled:
        return
    
    # Get current voltage from servo telemetry (average of all servos)
    raw_voltage = 0.0
    servo_telem = getattr(ctrl, 'servo_telem', None)
    if servo_telem is not None:
        v_sum = 0.0
        v_count = 0
        for s in servo_telem:
            if s is None or not getattr(s, 'valid_s3', False):
                continue
            v = getattr(s, 'voltage_v', 0.0)
            if v and v > 0:
                v_sum += float(v)
                v_count += 1
        if v_count > 0:
            raw_voltage = v_sum / v_count
    elif ctrl.servo is not None:
        # Fallback to legacy servo array
        v_sum = 0.0
        v_count = 0
        for s in ctrl.servo:
            if not s or len(s) < 1:
                continue
            v = s[0]
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v > 0:
                v_sum += v
                v_count += 1
        if v_count > 0:
            raw_voltage = v_sum / v_count
    
    # Apply low-pass filter (slower than display filter for stability)
    if raw_voltage > 0:
        if _lowBatteryFilteredVoltage <= 0:
            _lowBatteryFilteredVoltage = raw_voltage  # Initialize
        else:
            _lowBatteryFilteredVoltage = (
                _lowBatteryFilterAlpha * raw_voltage +
                (1.0 - _lowBatteryFilterAlpha) * _lowBatteryFilteredVoltage
            )
    
    # Check for voltage recovery (clears the triggered latch)
    if _lowBatteryTriggered and _lowBatteryFilteredVoltage >= _lowBatteryRecoveryVolt:
        _lowBatteryTriggered = False
        if ctrl.verbose:
            print(f"\r\n[LOW BATTERY] Voltage recovered to {_lowBatteryFilteredVoltage:.2f}V - protection cleared", end="\r\n")
    
    # Check for critical low battery
    if (_lowBatteryFilteredVoltage > 0 and 
        _lowBatteryFilteredVoltage < _lowBatteryVoltCritical and 
        not _lowBatteryTriggered):
        
        # Check if robot is currently enabled
        if getattr(ctrl, 'system_telem', None) is not None and ctrl.system_telem.valid:
            robot_enabled = ctrl.system_telem.robot_enabled
        else:
            robot_enabled = (ctrl.state[IDX_ROBOT_ENABLED] == 1.0) if (ctrl.state and len(ctrl.state) > IDX_ROBOT_ENABLED) else False
        
        if robot_enabled and ctrl.teensy is not None:
            # Set triggered latch (prevents re-enable until voltage recovers)
            _lowBatteryTriggered = True
            
            # Play low battery warning sound
            audio_low_battery_warning()
            # Voice announcement with percentage
            battery_percent = int(max(0, min(100, ((_lowBatteryFilteredVoltage - 9.0) / 3.6) * 100)))  # 9V=0%, 12.6V=100%
            speak_battery_low(battery_percent)
            
            print(f"\r\n[LOW BATTERY] Critical voltage {_lowBatteryFilteredVoltage:.2f}V < {_lowBatteryVoltCritical:.1f}V", end="\r\n")
            print("[LOW BATTERY] Executing graceful shutdown: TUCK → DISABLE", end="\r\n")
            
            # Stop any active gait first
            global _gaitActive
            if _gaitActive:
                _gaitActive = False
                if _gaitEngine is not None:
                    _gaitEngine.stop()
            
            # Execute TUCK posture for graceful kneel
            send_cmd(b'TUCK', force=True)
            
            # Schedule DISABLE after TUCK settles (2 seconds)
            ctrl.autoDisableAt = time.time() + 2.0
            ctrl.autoDisableReason = "LOW_BATTERY"
            ctrl.autoDisableGen += 1
            _autoDisableAt = ctrl.autoDisableAt
            _autoDisableReason = "LOW_BATTERY"
            _autoDisableGen = ctrl.autoDisableGen


def phase_loop_sleep(ctrl):
    """Phase 9: Calculate and execute loop sleep for timing control.
    
    Args:
        ctrl: Controller instance
        
    Returns:
        float: Effective period in ms used for this tick
    """
    global _nextTickTime
    
    now_mono = time.monotonic()
    
    # Telemetry-synchronized loop timing
    if ctrl.telemSyncActive and ctrl.lastS1MonoTime is not None:
        telem_age_ms = (now_mono - ctrl.lastS1MonoTime) * 1000.0
        if telem_age_ms > _telemSyncFallbackMs:
            ctrl.telemSyncActive = False
            effectivePeriodMs = 6.024  # 166Hz fallback
        else:
            effectivePeriodMs = ctrl.teensyLoopUs / 1000.0
    else:
        effectivePeriodMs = 6.024  # 166Hz fallback
    
    _nextTickTime += effectivePeriodMs / 1000.0
    sleepTime = _nextTickTime - now_mono
    
    if sleepTime > 0:
        time.sleep(sleepTime)
    elif sleepTime < -0.050:  # >50ms behind - reset
        _nextTickTime = time.monotonic()
    
    return effectivePeriodMs


# =============================================================================
# THREE-LAYER ARCHITECTURE: Layer wrapper functions
# =============================================================================
# Based on robotics research (Firby/Gat Three-Layer, Brooks Subsumption) and
# biological motor control (spinal reflexes → CPG → cortex hierarchy).
#
# Layer 1 (Controller): 166 Hz - Hardware I/O, safety reflexes, gait execution
# Layer 2 (Sequencer):  ~30 Hz - Behaviors, input processing, display updates
# Layer 3 (Deliberator): ~1 Hz - Planning, SLAM (future - async/on-demand)
# =============================================================================

def run_layer1_controller(ctrl):
    """Layer 1: Controller (166 Hz) - runs every tick.
    
    Fast, reflexive operations that must happen every cycle:
    - Timing synchronization
    - Hardware I/O (Teensy serial)
    - Safety checks (auto-disable, limits)
    - Gait tick (CPG-like FEET generation)
    
    Returns:
        tuple: (loop_dt, continue_running)
    """
    global _run
    
    # L1.1: Timing update
    loop_dt, _ = phase_timing_update()
    
    # L1.2: Teensy connection and serial I/O
    phase_teensy_connection(ctrl)
    
    # L1.3: Housekeeping (telemetry auto-start/retry)
    ctrl.housekeeping()
    
    # L1.4: Telemetry debug (optional - fast, just printing)
    if _debugTelemetry:
        if ctrl.lastRawS1:
            print(f"RAW S1: {ctrl.lastRawS1}", end="\r\n")
            print(f"PARSED S1: {ctrl.lastParsedS1}", end="\r\n")
        if ctrl.lastRawS2:
            print(f"RAW S2: {ctrl.lastRawS2}", end="\r\n")
            print(f"PARSED S2 ENABLES: {ctrl.lastParsedS2}", end="\r\n")
    
    # L1.5: Gait tick (CPG-like - generates FEET commands)
    phase_gait_tick(ctrl, _gaitEngine, _gaitActive)
    
    # L1.6: Safety - auto-disable check
    phase_auto_disable(ctrl)
    
    # L1.7: Point cloud server update (fast - just pushing data to async server)
    phase_pointcloud(ctrl)
    
    return loop_dt, True


def run_layer2_sequencer(ctrl):
    """Layer 2: Sequencer (~30 Hz) - runs every N ticks.
    
    Behavioral operations that don't need 166 Hz updates:
    - User input (gamepad, touch, keyboard)
    - Behavior arbitration (obstacle avoidance, patrol, etc.)
    - Display updates (eye animations, menu rendering)
    - Gamepad connection management
    
    Returns:
        bool: continue_running (False to exit main loop)
    """
    global _run
    
    # L2.1: Gamepad connection check
    phase_gamepad_connection(ctrl)
    
    # L2.2: Gamepad polling
    if ctrl.useJoySocket:
        ctrl.poll_joy_client()
    else:
        ctrl.poll_gamepad()
        if ctrl.requestExit:
            if _verbose:
                print("Exit requested via power button.", end="\r\n")
            return False
    
    # L2.3: Touch input
    phase_touch_input(_menu, ctrl)
    
    # L2.4: Keyboard input
    if not phase_keyboard_input(ctrl):
        return False
    
    # L2.5: Autonomy behaviors (arbiter)
    phase_autonomy(ctrl)
    
    # L2.6: Display update (push state to display thread)
    phase_display_update(ctrl, _displayThread, _gaitEngine, _gaitActive)
    
    # L2.7: Dashboard telemetry push (web clients)
    phase_dashboard(ctrl)
    
    return True


def run_layer3_deliberator(ctrl):
    """Layer 3: Deliberator (~1 Hz) - runs async/on-demand.
    
    Planning and world modeling operations (future):
    - Waypoint navigation
    - SLAM map updates
    - Mission planning
    
    Currently a placeholder for future development.
    """
    # Future: Waypoint planning, SLAM updates
    pass


def sync_globals_to_ctrl(ctrl):
    """Mirror global state into Controller instance at loop start."""
    ctrl.verbose = _verbose
    ctrl.mirror = _mirrorDisplay
    ctrl.teensy = _teensy
    ctrl.teensyErrorCount = _teensyErrorCount
    ctrl.nextTeensyScanAt = _nextTeensyScanAt
    ctrl.lastTelemetryTime = _lastTelemetryTime
    ctrl.telemetryGraceDeadline = _telemetryGraceDeadline
    ctrl.telemetryStartedByScript = _telemetryStartedByScript
    ctrl.telemetryRetryDeadline = _telemetryRetryDeadline
    ctrl.telemetryRetryCount = _telemetryRetryCount
    ctrl.controller = _controller
    ctrl.retryCount = _retryCount
    ctrl.forceDisplayUpdate = _forceDisplayUpdate
    ctrl.menuState = _menuState
    ctrl.menuVisible = _menuVisible
    ctrl.steeringMode = _steeringMode
    ctrl.autoDisableAt = _autoDisableAt
    ctrl.autoDisableReason = _autoDisableReason
    ctrl.autoDisableGen = _autoDisableGen
    ctrl.lastRawS1 = _lastRawS1
    ctrl.lastParsedS1 = _lastParsedS1
    ctrl.lastRawS2 = _lastRawS2
    ctrl.lastParsedS2 = _lastParsedS2
    ctrl.teensyLoopUs = _teensyLoopUs
    ctrl.lastS1MonoTime = _lastS1MonoTime
    ctrl.telemSyncActive = _telemSyncActive


def sync_ctrl_to_globals(ctrl):
    """Mirror Controller state back to globals at loop end."""
    global _verbose, _mirrorDisplay, _teensy, _teensyErrorCount, _nextTeensyScanAt
    global _lastTelemetryTime, _telemetryGraceDeadline, _telemetryStartedByScript
    global _telemetryRetryDeadline, _telemetryRetryCount, _controller, _retryCount
    global _forceDisplayUpdate, _menuState, _menuVisible, _steeringMode, _autoDisableAt
    global _autoDisableReason, _autoDisableGen
    global _lastRawS1, _lastParsedS1, _lastRawS2, _lastParsedS2
    global _teensyLoopUs, _lastS1MonoTime, _telemSyncActive
    
    _verbose = ctrl.verbose
    _mirrorDisplay = ctrl.mirror
    _teensy = ctrl.teensy
    _teensyErrorCount = ctrl.teensyErrorCount
    _nextTeensyScanAt = ctrl.nextTeensyScanAt
    _lastTelemetryTime = ctrl.lastTelemetryTime
    _telemetryGraceDeadline = ctrl.telemetryGraceDeadline
    _telemetryStartedByScript = ctrl.telemetryStartedByScript
    _telemetryRetryDeadline = ctrl.telemetryRetryDeadline
    _telemetryRetryCount = ctrl.telemetryRetryCount
    _controller = ctrl.controller
    _retryCount = ctrl.retryCount
    _forceDisplayUpdate = ctrl.forceDisplayUpdate
    _menuState = ctrl.menuState
    _menuVisible = ctrl.menuVisible
    _steeringMode = ctrl.steeringMode
    _autoDisableAt = ctrl.autoDisableAt
    _autoDisableReason = getattr(ctrl, 'autoDisableReason', _autoDisableReason)
    _autoDisableGen = getattr(ctrl, 'autoDisableGen', _autoDisableGen)
    _lastRawS1 = ctrl.lastRawS1
    _lastParsedS1 = ctrl.lastParsedS1
    _lastRawS2 = ctrl.lastRawS2
    _lastParsedS2 = ctrl.lastParsedS2
    _teensyLoopUs = ctrl.teensyLoopUs
    _lastS1MonoTime = ctrl.lastS1MonoTime
    _telemSyncActive = ctrl.telemSyncActive


#----------------------------------------------------------------------------------------------------------------------
# Main Loop Initialization
#----------------------------------------------------------------------------------------------------------------------

# Signal handler for graceful termination (SIGTERM from joy_controller)
def _sigterm_handler(signum, frame):
    """Handle SIGTERM for graceful shutdown."""
    global _run
    _run = False
    print("\r\n[CTRL] SIGTERM received, shutting down...", end="\r\n")

signal.signal(signal.SIGTERM, _sigterm_handler)

# Construct controller instance early so we can use its connection methods
_startupSplash.log("Initializing controller...")
ctrl = Controller.from_current_globals()
_startupSplash.log("Controller initialized", "GREEN")

# Attempt Teensy connection during startup
_startupSplash.log("Connecting to Teensy...")
_teensy_connected = False
# Read serial port config (same logic as phase_teensy_connection)
_teensy_port_override = _cfg.get('serial', 'port', fallback='').strip() if 'serial' in _cfg else ''
_teensy_baud_override = _cfg.getint('serial', 'baud', fallback=1000000) if 'serial' in _cfg else 1000000
for _attempt in range(10):  # Up to 1 second
    if ctrl.connect_teensy(_teensy_port_override if _teensy_port_override else None, _teensy_baud_override):
        _teensy = ctrl.teensy
        _teensy_connected = True
        _startupSplash.log("Teensy OK", "GREEN")
        break
    time.sleep(0.1)
else:
    _startupSplash.log("Teensy not found (will retry)", "YELLOW")

# Attempt controller connection during startup
_startupSplash.log("Connecting to controller...")
_controller_connected = False
if _useJoySocket:
    ctrl.joyClient = JoyClient(verbose=_verbose)
    for _attempt in range(10):  # Up to 1 second
        ctrl.joyClient.connect()
        if ctrl.joyClient.connected:
            _controller_connected = True
            _startupSplash.log("Controller OK (joy daemon)", "GREEN")
            break
        time.sleep(0.1)
    else:
        _startupSplash.log("Controller not found (will retry)", "YELLOW")
else:
    _controller = testForGamePad(_verbose)
    if _controller is not None:
        _controller_connected = True
        _startupSplash.log("Controller OK (direct)", "GREEN")
    else:
        _startupSplash.log("Controller not found (will retry)", "YELLOW")

# Start telemetry if Teensy connected
_telemetry_started = False
if _teensy_connected and ctrl.teensy is not None:
    _startupSplash.log("Starting telemetry...")
    try:
        ctrl.teensy.write(b'Y 1\n')
        ctrl.telemetryStartedByScript = True
        ctrl.telemetryGraceDeadline = time.time() + _telemetryGraceSeconds
        _telemetry_started = True
        # Wait briefly for first telemetry
        time.sleep(0.3)
        _startupSplash.log("Telemetry started", "GREEN")
    except Exception as e:
        _startupSplash.log(f"Telemetry error: {e}", "RED")

_startupSplash.finish(True)

# Play startup chime (ascending 3-tone sequence)
if _audio is not None:
    try:
        _audio.beep(440, 100)  # A4
        time.sleep(0.12)
        _audio.beep(554, 100)  # C#5
        time.sleep(0.12)
        _audio.beep(659, 150)  # E5
    except Exception:
        pass

# TTS startup announcement
speak_startup()

if _startupDelayS > 0:
    time.sleep(_startupDelayS)  # Configurable pause to see startup messages

# Switch to eyes display (display thread takes over from here)
if _displayThread is not None:
    _displayThread.update_state(servo=_servo, legs=_legs, state=_state,
                                mirror=_mirrorDisplay, menu_state=_menuState, verbose=_verbose,
                                teensy_connected=_teensy_connected,
                                controller_connected=_controller_connected,
                                telemetry_stale=(not _telemetry_started),
                                robot_enabled=False,
                                safety_active=False,
                                safety_text="")
    # Signal display thread that startup is complete - it can now render
    _displayThread.set_startup_complete(True)
else:
    # Fallback: draw eyes directly if no display thread
    UpdateDisplay(_disp, _eyes.display_image, None)

# start the main loop
# Sync controller state after startup connections
sync_globals_to_ctrl(ctrl)
# Initialize monotonic loop timing
_nextTickTime = time.monotonic()
# Initialize persistent keyboard input
init_keyboard()

# =========================================================================
# THREE-LAYER ARCHITECTURE (Firby/Gat, 1990s adaptation)
#   Layer 1 - Controller:  166 Hz  — reflexive, hardware I/O, safety
#   Layer 2 - Sequencer:   ~33 Hz  — behaviors, input, display
#   Layer 3 - Deliberator: ~1 Hz   — planning (future SLAM)
# =========================================================================

while _run:

    sync_globals_to_ctrl(ctrl)
    _layer2TickCount += 1
    _layer3TickCount += 1

    try:
        #----------------------------------------------------------------------
        # LAYER 1 — Controller (every tick, 166 Hz)
        #   Timing, Teensy I/O, housekeeping, gait tick, safety, point cloud
        #----------------------------------------------------------------------
        run_layer1_controller(ctrl)

        #----------------------------------------------------------------------
        # LAYER 2 — Sequencer (every _layer2Divisor ticks, ~33 Hz)
        #   Gamepad connection/polling, touch, keyboard, autonomy, display
        #----------------------------------------------------------------------
        if _layer2TickCount >= _layer2Divisor:
            _layer2TickCount = 0
            if not run_layer2_sequencer(ctrl):
                _run = False

        #----------------------------------------------------------------------
        # LAYER 3 — Deliberator (every _layer3Divisor ticks, ~1 Hz)
        #   Future: path planning, SLAM integration, high-level decisions
        #----------------------------------------------------------------------
        if _layer3TickCount >= _layer3Divisor:
            _layer3TickCount = 0
            run_layer3_deliberator(ctrl)

        #----------------------------------------------------------------------
        # End of tick: sleep and sync state back
        #----------------------------------------------------------------------
        phase_loop_sleep(ctrl)
        sync_ctrl_to_globals(ctrl)

    except KeyboardInterrupt as keyExcept:
        print(end="\r\n")  # Newline to preserve any \r debug output on Ctrl+C
        _run = False
    except Exception as e:
        # handle any other exceptions that may occur
        if _verbose:
            print(f"Error in main loop: {e}", end="\r\n")

# clean up all used objects and resources
# Stop display thread first
if _displayThread is not None and _displayThread.is_alive():
    if _verbose:
        print("Stopping display thread...", end="\r\n")
    _displayThread.stop()
# Stop IMU thread
if _imuThread is not None and _imuThread.is_alive():
    if _verbose:
        print("Stopping IMU thread...", end="\r\n")
    _imuThread.stop()
# Stop ToF thread
if _tofThread is not None and _tofThread.is_alive():
    if _verbose:
        print("Stopping ToF thread...", end="\r\n")
    _tofThread.stop()
# Stop Point Cloud server
if _pointcloudServer is not None:
    if _verbose:
        print("Stopping Point Cloud server...", end="\r\n")
    _pointcloudServer.stop()
# Stop Dashboard server
if _dashboardServer is not None:
    if _verbose:
        print("Stopping Dashboard server...", end="\r\n")
    _dashboardServer.stop()
# Play shutdown beep (descending tone)
if _audio is not None:
    try:
        _audio.beep(659, 100)  # E5
        time.sleep(0.12)
        _audio.beep(440, 150)  # A4
        time.sleep(0.2)
    except Exception:
        pass

# TTS shutdown announcement
speak_shutdown()
time.sleep(2.0)  # Wait for TTS to finish before cleanup

# Restore terminal from curses FIRST (endwin() clears screen)
cleanup_keyboard()
# NOW print debug info so it persists after script exits
print(end="\r\n")  # Newline to preserve any \r debug output
if _verbose:
    print("Exiting main loop and cleaning up...", end="\r\n")
# Disable servos and robot on exit (safe shutdown)
if _teensy is not None:
    if _verbose:
        print("Disabling robot (LEG ALL DISABLE + DISABLE).", end="\r\n")
    send_cmd(b'LEG ALL DISABLE', force=True)
    time.sleep(0.02)
    send_cmd(b'DISABLE', force=True)
    time.sleep(0.02)

    if _verbose:
        print("Stopping telemetry ('Y 0').", end="\r\n")
    send_cmd(b'Y 0', force=True)
    time.sleep(0.05)
print(end="\r\n")

# Shutdown audio manager
if _audio is not None:
    _audio.shutdown()
    _audio = None

_controller = None
_eyes = None
_disp.clear()
_disp.bl_DutyCycle(0)
#UpdateDisplay(_disp, _backGroundImage, None, _legs, _state, _mirrorDisplay)  # clear the display
cv.destroyAllWindows()
#time.sleep(1.1)  # give the display time to clear
#_disp = None
_touch = None
#_teensy.
_teensy = None


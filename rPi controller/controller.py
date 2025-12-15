#----------------------------------------------------------------------------------------------------------------------
#    controller.py
#----------------------------------------------------------------------------------------------------------------------
# CHANGE LOG (Python Controller)
# Format: YYYY-MM-DD  Summary
# REMINDER: Update CONTROLLER_VERSION below with every behavioral change, feature addition, or bug fix!
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
#----------------------------------------------------------------------------------------------------------------------
# Controller semantic version (bump on behavior-affecting changes)
CONTROLLER_VERSION = "0.5.27"
# Monotonic build number (never resets across minor/major version changes; increment every code edit)
CONTROLLER_BUILD = 138
#----------------------------------------------------------------------------------------------------------------------
# Telemetry index constants (S1 schema)
# These named constants prevent magic numbers and make schema changes explicit.
#----------------------------------------------------------------------------------------------------------------------
IDX_LOOP_US = 0          # S1[0]: Teensy loop period in microseconds
IDX_BATTERY_V = 1        # S1[1]: Battery voltage
IDX_CURRENT_A = 2        # S1[2]: Current draw (amps)
IDX_PITCH_DEG = 3        # S1[3]: IMU pitch angle
IDX_ROLL_DEG = 4         # S1[4]: IMU roll angle
IDX_YAW_DEG = 5          # S1[5]: IMU yaw angle
IDX_GAIT = 6             # S1[6]: Current gait mode (0-9)
IDX_MODE = 7             # S1[7]: Operational mode
IDX_SAFETY = 8           # S1[8]: Safety status flags
IDX_ROBOT_ENABLED = 9    # S1[9]: Robot enable flag (1.0 = enabled, 0.0 = disabled)
#----------------------------------------------------------------------------------------------------------------------
#    Import all required libraries
#----------------------------------------------------------------------------------------------------------------------
import sys  
import os
import configparser
import time
import curses
import numpy as np
import threading
from queue import Queue, Empty
#import os
#import math
#import logging
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
                         StationaryPattern, GaitParams, GaitTransition)

# Gait type cycling order (RB button)
GAIT_TYPES = [TripodGait, WaveGait, RippleGait, StationaryPattern]
GAIT_NAMES = ["Tripod", "Wave", "Ripple", "Stationary"]

# import the LCD library and related graphics libraries
# sys.path.append("..")
# sys.path.append("./")
#from lib import LCD_1inch9
import st7789
import cst816d
from PIL import Image, ImageDraw, ImageFont
import spidev as SPI

#----------------------------------------------------------------------------------------------------------------------
#
#    Define all helper functions here
#
#----------------------------------------------------------------------------------------------------------------------
# Font cache to avoid reloading TTF every frame
FONT_PATH = "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf"
_font_cache = {}

def get_font(size: int):
    try:
        key = int(size)
        f = _font_cache.get(key)
        if f is None:
            f = ImageFont.truetype(FONT_PATH, key)
            _font_cache[key] = f
        return f
    except (OSError, IOError):
        # Font file missing or unreadable - use default
        return ImageFont.load_default()


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
        self.controller = None
        self.retryCount = 0
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
        # Gait control inputs (analog sticks)
        self._gaitStrafeInput = 0.0  # Left stick X: -1 (left) to +1 (right)
        self._gaitSpeedInput = 0.0   # Left stick Y: -1 (back) to +1 (forward)
        # Gait parameter adjustment (left stick button + triggers)
        self._leftStickHeld = False  # True when left stick button is held
        self._gaitWidthMm = _savedGaitWidthMm   # Current gait width (lateral offset from body center)
        self._gaitLiftMm = _savedGaitLiftMm     # Current lift height during swing phase

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
                robot_enabled = (self.state[IDX_ROBOT_ENABLED] == 1.0) if len(self.state) > IDX_ROBOT_ENABLED else True
                safety_active, safety_text = get_safety_overlay_state()
                UpdateDisplay(self.disp, self.eyes.display_image, self.menu._image, 
                              self.servo, self.legs, self.state, self.mirror, self.menuState,
                              teensy_connected=(self.teensy is not None),
                              controller_connected=(self.controller is not None),
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
        if self.teensy is None:
            return None
        teensyData = None
        hadError = False
        try:
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
                segments = line.split('|')
                for segment in segments:
                    elements = segment.split(',')
                    header = elements[0][0:3]
                    elements[0] = elements[0][3:]
                    if header == 'S1:':
                        self.lastRawS1 = segment
                        processTelemS1(elements, self.state)
                        self.lastParsedS1 = list(self.state)
                        self.lastTelemetryTime = time.time()
                        self.telemetryRetryDeadline = None
                        # Update telemetry-sync timing from S1 loop_us (index 0)
                        self.lastS1MonoTime = time.monotonic()
                        if len(elements) > 0:
                            try:
                                loop_us = int(float(elements[0]))
                                if 1000 <= loop_us <= 50000:  # sanity: 20Hz..1000Hz
                                    self.teensyLoopUs = loop_us
                                    self.telemSyncActive = True
                            except ValueError:
                                pass
                    elif header == 'S2:':
                        self.lastRawS2 = segment
                        processTelemS2(elements, self.servo)
                        self.lastParsedS2 = [int(self.servo[i][2]) for i in range(18)]
                        self.lastTelemetryTime = time.time()
                        self.telemetryRetryDeadline = None
                    elif header == 'S3:':
                        processTelemS3(elements, self.servo)
                        self.lastTelemetryTime = time.time()
                        self.telemetryRetryDeadline = None
                    elif header == 'S4:':
                        processTelemS4(elements, self.legs)
                        self.lastTelemetryTime = time.time()
                        self.telemetryRetryDeadline = None
                    elif header == 'S5:':
                        global _safety_state, _last_safety_lockout, _gaitActive, _gaitEngine
                        prev_lockout = _safety_state.get("lockout", False)
                        processTelemS5(elements, _safety_state)
                        self.lastTelemetryTime = time.time()
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
                            if self.verbose:
                                print("Firmware reported SAFETY LOCKOUT; issued LEG ALL DISABLE + DISABLE.", end="\r\n")
                        _last_safety_lockout = lockout
        except (ValueError, IndexError) as e:
            # Malformed telemetry line - skip but log in verbose mode
            if self.verbose:
                print(f"Telemetry parse error: {e}", end="\r\n")
        return teensyData

    def _updateGaitHeading(self):
        """Compute speed_scale from left stick Y input.
        
        Control scheme:
        - Left stick Y = speed (0 to 1) - magnitude of motion
        - Right stick X = heading angle (set directly in event handler)
        
        Speed from left Y controls how far legs stride; heading from right X
        controls the direction of the crab walk.
        """
        global _gaitEngine
        if _gaitEngine is None:
            return
        
        # Speed from left stick Y: absolute value = magnitude
        speed_magnitude = abs(self._gaitSpeedInput)
        
        # Apply speed (heading is set directly by right stick X event handler)
        if speed_magnitude > 0.05:
            _gaitEngine.params.speed_scale = min(1.0, speed_magnitude)
        else:
            # Neutral: step in place
            _gaitEngine.params.speed_scale = 0.0

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
                print(event, end="\r\n")  # Debug print of raw event
                if event.type == 1:  # button
                    if event.code == 158 and event.value == 1:  # mirror toggle
                        if self.verbose:
                            print("\nscreen mirror button pressed", end="\r\n")
                        self.mirror = not self.mirror
                        self.forceDisplayUpdate = True
                    elif event.code == 315 and event.value == 1:  # Start button - MARS menu toggle
                        if self.verbose:
                            print("\nStart button pressed (menu)", end="\r\n")
                        # Only allow menu when robot is disabled and not in motion
                        robot_enabled = (self.state[IDX_ROBOT_ENABLED] == 1.0) if len(self.state) > IDX_ROBOT_ENABLED else False
                        if _marsMenu.visible:
                            # Always allow closing menu
                            _marsMenu.hide()
                            self.forceDisplayUpdate = True
                        elif not robot_enabled and not _gaitActive:
                            # Safe to open menu
                            _marsMenu.show()
                            self.forceDisplayUpdate = True
                        else:
                            if self.verbose:
                                print("  Menu blocked: disable robot first", end="\r\n")
                    elif event.code == 172 and event.value == 1:  # power
                        if self.verbose:
                            print("\npower button pressed", end="\r\n")
                        self.requestExit = True
                    elif event.code in [139, 158, 172] and event.value == 1:  # menu/guide/power variants
                        if self.verbose:
                            print(f"\nXbox guide/power button pressed (code {event.code})", end="\r\n")
                        self.requestExit = True
                    elif event.code == 304 and event.value == 1:  # A button
                        if self.verbose:
                            print("\nA button pressed", end="\r\n")
                        if _marsMenu.visible:
                            # Select/activate current menu item
                            _marsMenu.select()
                            self.forceDisplayUpdate = True
                        elif self.teensy is not None and not _safety_state.get("lockout", False):
                            # Normal A button behavior: Stand
                            apply_posture(b'STAND', auto_disable_s=4.0)
                        elif self.teensy is not None and _safety_state.get("lockout", False) and self.verbose:
                            print("  Stand blocked: firmware safety lockout is active.", end="\r\n")
                    elif event.code == 305 and event.value == 1:  # B button
                        if self.verbose:
                            print("\nB button pressed", end="\r\n")
                        if _marsMenu.visible:
                            # Close menu
                            _marsMenu.hide()
                            self.forceDisplayUpdate = True
                        elif self.teensy is not None:
                            # Normal B button behavior: Home
                            apply_posture(b'HOME', auto_disable_s=4.0)
                    elif event.code == 308 and event.value == 1:  # X toggle test/idle (BTN_WEST)
                        if self.verbose:
                            print("\nX button pressed (Toggle Test/Idle)", end="\r\n")
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
                        if self.verbose:
                            print("\nY button pressed (Tuck)", end="\r\n")
                        if self.teensy is not None and not _safety_state.get("lockout", False):
                            apply_posture(b'TUCK', auto_disable_s=4.0)
                        elif self.teensy is not None and _safety_state.get("lockout", False) and self.verbose:
                            print("  Tuck blocked: firmware safety lockout is active.", end="\r\n")
                    elif event.code == 318 and event.value == 1:  # right joystick press - disable
                        if self.verbose:
                            print("\nRight joystick pressed (Disable)", end="\r\n")
                        if self.teensy is not None:
                            send_cmd(b'DISABLE', force=True)
                    elif event.code == 317:  # left joystick press - gait width adjustment mode
                        if event.value == 1:  # Button pressed
                            self._leftStickHeld = True
                            if self.verbose:
                                print(f"\nLeft joystick pressed - Gait adjustment mode (width: {self._gaitWidthMm:.0f}mm, lift: {self._gaitLiftMm:.0f}mm)", end="\r\n")
                        else:  # Button released (value == 0)
                            self._leftStickHeld = False
                            # Apply saved parameters to active gait engine
                            if _gaitEngine is not None:
                                _gaitEngine.params.base_x_mm = self._gaitWidthMm
                                _gaitEngine.params.lift_mm = self._gaitLiftMm
                            # Persist to ini file
                            if save_gait_settings(self._gaitWidthMm, self._gaitLiftMm):
                                if self.verbose:
                                    print(f"\nGait settings saved: width={self._gaitWidthMm:.0f}mm, lift={self._gaitLiftMm:.0f}mm", end="\r\n")
                            else:
                                if self.verbose:
                                    print(f"\nGait settings applied (save failed): width={self._gaitWidthMm:.0f}mm, lift={self._gaitLiftMm:.0f}mm", end="\r\n")
                    elif event.code == 310 and event.value == 1:  # LB - prev tab or toggle gait
                        if _marsMenu.visible:
                            _marsMenu.handle_button('LB')
                            self.forceDisplayUpdate = True
                        elif not _gaitActive:
                            if _safety_state.get("lockout", False):
                                if self.verbose:
                                    print("\nGait start blocked: firmware safety lockout is active.", end="\r\n")
                                continue
                            # Start gait - cancel any pending auto-disable first
                            _autoDisableAt = None
                            self.autoDisableAt = None
                            # Switch Teensy to IDLE mode (stop any built-in gait)
                            send_cmd(b'MODE IDLE', force=True)
                            # Enable robot if needed
                            if self.state[IDX_ROBOT_ENABLED] != 1.0:
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
                            # Reset stick inputs so gait starts at zero speed until user moves stick
                            self._gaitStrafeInput = 0.0
                            self._gaitSpeedInput = 0.0
                            self._updateGaitHeading()  # Sync gait engine with current (zero) inputs
                            if self.verbose:
                                print("\nLB: Gait engine STARTED (tripod)", end="\r\n")
                        else:
                            # Stop gait
                            if _gaitEngine is not None:
                                _gaitEngine.stop()
                            _gaitActive = False
                            send_cmd(b'STAND', force=True)
                            _autoDisableAt = time.time() + _autoDisableS
                            self.autoDisableAt = _autoDisableAt
                            if self.verbose:
                                print("\nLB: Gait engine STOPPED", end="\r\n")
                    elif event.code == 311 and event.value == 1:  # RB - next tab or cycle gait
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
                                    print(f"\nRB: Transitioning to {next_name} gait (waiting for phase boundary)", end="\r\n")
                            else:
                                if self.verbose:
                                    print(f"\nRB: Transition already in progress", end="\r\n")
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
                            # Map to heading angle: -90° (left) to +90° (right), 0° = forward
                            # Reverse strafe direction so joystick X matches world left/right motion
                            _gaitEngine.params.heading_deg = -strafe * 90.0
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
# find_teensy scnas the available serial ports and looks for a Teensy device.
# return: serial port device or none if not found
def find_teensy(_verbose = False):
    # List all available serial ports
    returnPort = None
    if _verbose:
        print("Searching for Teensy...", end="\r\n")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Check if the device description matches a Teensy
        if _verbose:
            print( port.description, port.device, "-", port.manufacturer, "-", port.product, end="\r\n")
        if "teensy" in port.manufacturer.lower() and "0" in port.device:
            if _verbose:
                print(f"Teensy found on port: {port.device}", end="\r\n")
            return port
    if _verbose:
        print("No Teensy found.", end="\r\n")
    return None

# readTeensy reads the Teensy device and returns the data
# Persistent buffer for incomplete lines across reads
_teensy_read_buffer = ""

def readTeensy(teensy, _verbose=False):
    """Reads the Teensy device with batched I/O and incomplete line buffering.
    Returns complete lines only; partial lines are buffered for next call."""
    global _teensy_read_buffer
    if teensy is None:
        if _verbose:
            print("No Teensy device found.", end="\r\n")
        return None
    try:
        # Batch read all available bytes at once (much faster than byte-by-byte)
        waiting = teensy.in_waiting
        if waiting == 0:
            return None
        raw = teensy.read(waiting)
        try:
            chunk = raw.decode('utf-8', errors='ignore')
        except Exception:
            chunk = ""
        
        # Append to persistent buffer
        _teensy_read_buffer += chunk
        
        # Normalize all newlines to \n
        _teensy_read_buffer = _teensy_read_buffer.replace('\r\n', '\n').replace('\r', '\n')
        
        # Split into lines; last element may be incomplete
        lines = _teensy_read_buffer.split('\n')
        
        # If buffer doesn't end with newline, last element is incomplete - keep it
        if not _teensy_read_buffer.endswith('\n'):
            _teensy_read_buffer = lines[-1]  # Save incomplete line for next read
            lines = lines[:-1]               # Return only complete lines
        else:
            _teensy_read_buffer = ""         # All lines complete
        
        # Join complete lines and return
        if lines:
            return '\n'.join(lines)
        return None
    except serial.SerialException as e:
        if _verbose:
            print(f"Error reading from Teensy: {e}", end="\r\n")
        return None
    
# Gait number to string mapping
_GAIT_NAMES = {
    0: "TRI",
    1: "RIPPLE",
    2: "WAV",
    3: "QUAD",
    4: "BI",
    5: "HOP",
    6: "TRANS",
    7: "STILL",
    8: "HOME",
    9: "NONE",
}

def getGait(gait):
    """Convert gait number to string representation."""
    return _GAIT_NAMES.get(gait, "UNKNOWN")

# testForGamePad checks for the presence of an Xbox controller and returns its path if found
def testForGamePad(verbose=False):
    controller = None
    #if verbose:
        #print('Connecting to xbox controller...', end="\r\n")
    found = False
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        #if verbose:
        #    print(device.path, device.name, end="\r\n")

        if str.lower(device.name) == 'xbox wireless controller':
            if verbose:
                print("Xbox controller found at:", device.path, end="\r\n")
            controller = device
            found = True
            break
    #if not found:
       # if verbose:
        #    print("No Xbox controller found.", end="\r\n")  
    return controller

# Persistent curses state for keyboard input (avoids per-loop wrapper overhead)
_stdscr = None

def init_keyboard():
    """Initialize persistent curses for non-blocking keyboard input."""
    global _stdscr
    if _stdscr is None:
        _stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        _stdscr.nodelay(True)  # non-blocking
        _stdscr.keypad(True)
    return _stdscr

def cleanup_keyboard():
    """Restore terminal settings on exit."""
    global _stdscr
    if _stdscr is not None:
        _stdscr.keypad(False)
        curses.nocbreak()
        curses.echo()
        #curses.endwin()
        _stdscr = None

def poll_keyboard():
    """Non-blocking keyboard poll using persistent stdscr."""
    global _stdscr
    if _stdscr is None:
        init_keyboard()
    try:
        return _stdscr.getch()
    except curses.error:
        return -1

# Legacy getkey function (deprecated - use poll_keyboard instead)
def getkey(stdscr):
    """checking for keypress"""
    stdscr.nodelay(True)  # do not wait for input when calling getch
    return stdscr.getch()

def getColor(min_val, max_val, value, color_palette):
    """Returns a color based on the value and the color palette"""
    if value < min_val:
        return color_palette[0]  # return the first color in the palette
    elif value > max_val:
        return color_palette[len(color_palette)-1]  # return the last color in the palette
    else:
        # calculate the index based on the value
        # Calculate the position in the palette
        scaled = (value - min_val) / (max_val - min_val) * (len(color_palette) - 1)
        lower_idx = max(0,int(np.floor(scaled)))
        upper_idx = min(len(color_palette) - 1, lower_idx + 1)  # ensure
        if lower_idx == upper_idx:
            return color_palette[lower_idx]
        # Linear interpolation between the two colors
        ratio = scaled - lower_idx
        color1 = np.array(color_palette[lower_idx])
        color2 = np.array(color_palette[upper_idx])
        interp_color = (1 - ratio) * color1 + ratio * color2
        #print(min_val, max_val, value, scaled, lower_idx, upper_idx, ratio, color1, color2, interp_color,tuple(interp_color.astype(int, end="\r\n")))
        return tuple(interp_color.astype(int))


def drawLogo(disp):
    """Draws a logo on the display"""
    # Create a new image with a white background
    image = Image.new("RGB", (disp.height, disp.width), "BLACK")
    draw = ImageDraw.Draw(image)

    # Draw a simple logo (a blue rectangle with text)
    fontMICR = get_font(12)
    Color = "WHITE"
    Top = 10
    lineHeight = 8
    draw.text((20, Top), u"  __  __    _    ____   ____   ____  \n", fill = Color, font=fontMICR)
    draw.text((20, Top + lineHeight), u" |  \\/  |  / \\  |  _ \\ / ___| / ___|\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 2*lineHeight), u" | |\\/| | / _ \\ | |_) | |  _  \\___ \\ \n", fill = Color, font=fontMICR)
    draw.text((20, Top + 3*lineHeight), u" | |  | |/ ___ \\|  __/| |_| | ___) |\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 4*lineHeight), u" |_|  |_/_/   \\_\\_|    \\____||____/ \n", fill = Color, font=fontMICR)
    draw.text((20, Top + 5*lineHeight), u"   Modular Autonomous Robotic System\n", fill = Color, font=fontMICR)
    #image=image.rotate(180, expand=True)  # rotate the image to fit the display

    # Show the image on the display
    UpdateDisplay(disp, image, None)

# Frame change detection for display optimization
_last_frame_hash = None

def UpdateDisplay(disp, image, menu, servo=None, legs = None, state = None,
                  mirror=False, menuState=None,
                  teensy_connected=True, controller_connected=True,
                  telemetry_stale=False, robot_enabled=True,
                  safety_active=False, safety_text=""):
    """Updates the display with the given image.
    
    Optimizations applied:
    - Frame change detection: skips SPI write if frame unchanged (via hash)
    - Combined rotation + RGB565: single-pass show_image_rotated()
    - Avoids unnecessary copy when no overlay drawing needed
    
    Args:
        teensy_connected: If False, draws 'NO TEENSY' watermark overlay
        controller_connected: If False, draws 'NO CONTROLLER' watermark overlay
        telemetry_stale: If True, draws 'NO TELEMETRY' watermark overlay
        robot_enabled: If False, draws 'DISABLED' watermark overlay
    """
    
    # Safety overlay: replace normal content with full-screen safety view
    if safety_active:
        imageCopy = Image.new("RGB", (disp.height, disp.width), (255, 230, 0))
        draw = ImageDraw.Draw(imageCopy)
        font = get_font(26)
        text = safety_text if safety_text else "SAFETY LOCKOUT"
        # Center text
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        x = (disp.height - text_width) // 2
        y = (disp.width - text_height) // 2
        # Shadow + text
        draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
        draw.text((x, y), text, fill="BLACK", font=font)
    else:
        # Only copy if we need to draw overlays
        has_status_overlay = (not teensy_connected or not controller_connected or
                              telemetry_stale or not robot_enabled)
        needs_overlay = (servo is not None and (_menuState == "data" or _menuState == "settings")) or has_status_overlay
        if needs_overlay:
            imageCopy = image.copy()
        else:
            imageCopy = image  # Use original directly if no modifications needed
        
        # Draw connection/status overlays
        overlay_texts = []  # List of (text, color) tuples
        if not teensy_connected:
            overlay_texts.append(("NO TEENSY", "RED"))
        elif telemetry_stale:
            # Only show stale if teensy connected but no data
            overlay_texts.append(("NO TELEMETRY", "ORANGE"))
        if not controller_connected:
            overlay_texts.append(("NO CONTROLLER", "RED"))
        
        # DISABLED is shown separately at bottom
        show_disabled = not robot_enabled and teensy_connected and not telemetry_stale

        if overlay_texts:
            draw = ImageDraw.Draw(imageCopy)
            font = get_font(28)
            # Calculate total height for vertical centering
            line_height = 36  # font size + spacing
            total_height = len(overlay_texts) * line_height
            start_y = (disp.width - total_height) // 2
            
            for i, (text, color) in enumerate(overlay_texts):
                # Get text bounding box for horizontal centering
                bbox = draw.textbbox((0, 0), text, font=font)
                text_width = bbox[2] - bbox[0]
                x = (disp.height - text_width) // 2
                y = start_y + i * line_height
                # Draw shadow for visibility
                draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
                # Draw text in color
                draw.text((x, y), text, fill=color, font=font)
        
        # Draw DISABLED at bottom of screen
        if show_disabled:
            draw = ImageDraw.Draw(imageCopy)
            font = get_font(28)
            text = "DISABLED"
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            x = (disp.height - text_width) // 2
            y = disp.width - 40  # Near bottom
            draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
            draw.text((x, y), text, fill="YELLOW", font=font)
    
    # add in the leg/servo visualization (suppressed when in safety overlay)
    if servo is not None and not safety_active:
        # create a draw object to draw on the image
        draw = ImageDraw.Draw(imageCopy)

        if _menuState == "data":
            # draw the gait text on the image
            if state is not None:
                gait = getGait(int(state[IDX_GAIT]))
                # Check robot enable flag
                if state[IDX_ROBOT_ENABLED] == 0:
                    gait = "DISABLED"
                font24 = get_font(24)
                textSize = draw.textbbox(((disp.height / 2), disp.width - 24), gait, font=font24, anchor="mt")
                center = disp.height / 2
                blackoutSpace = 70
                draw.rectangle((center - blackoutSpace, disp.width - 24, center + blackoutSpace, disp.width), fill="BLACK")
                draw.text((textSize[0], textSize[1]), gait, fill="WHITE", font=font24)

            for idx, (voltage, temperature, enable) in enumerate(servo):
                #print(f"Servo {idx}: Voltage: {voltage}, Temperature: {temperature}", end="\r\n")
                # draw the servo telemetry data on the image
                vColor = "GRAY"
                tColor = "GRAY"            
                width = 10
                height = 10
                gap = 79.5
                leg = int(idx / 3)
                left = (idx - (leg*3)) * (width + 2)
                top = gap * (2-leg)
                dir = 1
                borderColor = "WHITE"
                if legs[leg][0] == 0:
                    borderColor = "RED"
                if enable == 1:
                    vColor = getColor(10.5,12.5, voltage, _powercolorPalette)
                    tColor = getColor(20, 75, temperature, _temperatureColorPalette)
                    #print(idx, vColor, tColor, end="\r\n")
                leg = int(idx / 3)
                left = (idx - (leg*3)) * (width + 2)
                top = int(gap * (2-leg))
                dir = 1
                if leg >= 3:
                    leg -= 3
                    dir = -1
                    left = disp.height - left
                    top = int(gap * leg)
                draw.rectangle((left, top, left + dir*4, top + height), tColor)
                draw.rectangle((left + dir*5, top, left + dir*9, top + height), vColor)
                draw.rectangle((left,top, left + dir*width, top + height), outline=borderColor, width=1)
        elif _menuState == "settings":
            imageCopy = Image.blend(imageCopy, menu, 0.6)  # blend the menu image onto the display image
    
    # MARS menu overlay (takes priority over old menu system)
    if _marsMenu is not None and _marsMenu.visible:
        _marsMenu.render()
        imageCopy = _marsMenu.image.copy()
    if mirror:
        image2 = np.array(imageCopy)                        
        height, width = image2.shape[:2]
        image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv.INTER_LANCZOS4)
        cv.imshow("M.A.R.S. — Modular Autonomous Robotic System", image2)
        # When mirror window has focus, route keys to MarsMenu
        key = cv.waitKey(1) & 0xFF
        if key != 255 and _marsMenu is not None and _marsMenu.visible:
            # Mirror MarsMenu keyboard semantics for the curses path:
            # - Arrow keys / WASD: item navigation
            # - Tab / Shift+Tab: tab navigation
            # - Left/Right (or J/L): value decrement/increment
            # - Enter/Space: select/activate
            # - Esc/Q: close menu
            if key in (ord('w'), ord('W'), 82):  # up or arrow up
                _marsMenu.nav_up()
            elif key in (ord('s'), ord('S'), 84):  # down or arrow down
                _marsMenu.nav_down()
            elif key in (ord('a'), ord('A'), 81):  # left or arrow left
                _marsMenu.nav_left()
            elif key in (ord('d'), ord('D'), 83):  # right or arrow right
                _marsMenu.nav_right()
            elif key in (9,):  # TAB cycles tabs forward
                _marsMenu.handle_button('RB')
            elif key in (353,):  # Shift+TAB cycles tabs backward (if available)
                _marsMenu.handle_button('LB')
            elif key in (ord('j'), ord('J')):
                _marsMenu.decrement()
            elif key in (ord('l'), ord('L')):
                _marsMenu.increment()
            elif key in (10, 13, ord(' ')):
                _marsMenu.select()
            elif key in (27, ord('q'), ord('Q')):  # ESC or Q closes menu
                _marsMenu.hide()
            # Force display refresh after any menu key
            global _forceDisplayUpdate
            _forceDisplayUpdate = True
    else:
        cv.destroyAllWindows()
    
    # Frame change detection: compute fast hash and skip SPI write if unchanged
    # Uses tobytes() hash which is very fast for numpy arrays
    global _last_frame_hash
    img_array = np.asarray(imageCopy)
    # Sample every 16th pixel for fast approximate hash (320*170/16 = ~3400 samples)
    frame_hash = hash(img_array[::4, ::4, :].tobytes())
    
    if frame_hash != _last_frame_hash:
        _last_frame_hash = frame_hash
        # Use combined rotation + RGB565 conversion for optimal performance
        # show_image_rotated does rot90 + RGB565 in one pass, avoiding intermediate PIL image
        disp.show_image_rotated(imageCopy, rotation_k=3)

#----------------------------------------------------------------------------------------------------------------------
# DisplayThread: Background thread for eye animation and LCD updates
#----------------------------------------------------------------------------------------------------------------------
class DisplayThread(threading.Thread):
    """Background thread for eye animation and LCD display updates.
    
    Runs at configurable Hz (default 15), independently of main loop timing.
    Owns the SimpleEyes instance and performs SPI writes to the display.
    Main thread communicates state changes via thread-safe shared state.
    """
    
    def __init__(self, disp, eyes, menu, target_hz: float = 15.0,
                 look_range_x: float = 20.0, look_range_y: float = 10.0,
                 blink_frame_divisor: int = 2):
        super().__init__(daemon=True, name="DisplayThread")
        self.disp = disp
        self.eyes = eyes
        self.menu = menu
        self.blink_frame_divisor = max(1, blink_frame_divisor)  # At least 1 (no skip)
        self.target_hz = target_hz
        self.period_s = 1.0 / target_hz
        
        # Thread control
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # Shared state (protected by lock)
        self._servo = [[0, 0, 0]] * 18  # Servo telemetry: (voltage, temp, enable)
        self._legs = [[0, 0, 0]] * 6    # Leg status
        self._state = [0.0] * 10        # S1 telemetry state
        self._mirror = False
        self._menu_state = None
        self._force_update = False
        self._verbose = False
        # Eye look direction: -1.0 to +1.0 (left/right, up/down)
        self._look_x = 0.0
        self._look_y = 0.0
        # Connection/status for overlays
        self._teensy_connected = True
        self._controller_connected = True
        self._telemetry_stale = False
        self._robot_enabled = True
        self._safety_active = False
        self._safety_text = ""
        # Configurable look range (pixels of eye offset at max input)
        self.look_range_x = look_range_x
        self.look_range_y = look_range_y
        # Base eye offsets (from config)
        self._base_center_offset = self.eyes.eye_center_offset
        self._base_vertical_offset = self.eyes.eye_vertical_offset

    def set_base_vertical_offset(self, offset: float):
        """Update the baseline vertical eye offset used for joystick look."""
        with self._lock:
            self._base_vertical_offset = offset
        
    def update_state(self, servo=None, legs=None, state=None, mirror=None, 
                     menu_state=None, force_update=False, verbose=None,
                     look_x=None, look_y=None, teensy_connected=None,
                     controller_connected=None, telemetry_stale=None,
                     robot_enabled=None, safety_active=None,
                     safety_text=None):
        """Thread-safe update of display state from main thread."""
        with self._lock:
            if servo is not None:
                self._servo = servo
            if legs is not None:
                self._legs = legs
            if state is not None:
                self._state = state
            if mirror is not None:
                self._mirror = mirror
            if menu_state is not None:
                self._menu_state = menu_state
            if force_update:
                self._force_update = True
            if verbose is not None:
                self._verbose = verbose
            if look_x is not None:
                self._look_x = look_x
            if look_y is not None:
                self._look_y = look_y
            if teensy_connected is not None:
                self._teensy_connected = teensy_connected
            if controller_connected is not None:
                self._controller_connected = controller_connected
            if telemetry_stale is not None:
                self._telemetry_stale = telemetry_stale
            if robot_enabled is not None:
                self._robot_enabled = robot_enabled
            if safety_active is not None:
                self._safety_active = safety_active
            if safety_text is not None:
                self._safety_text = safety_text
    
    def stop(self):
        """Signal thread to stop and wait for it to finish."""
        self._stop_event.set()
        self.join(timeout=1.0)
    
    def run(self):
        """Main thread loop: update eyes and display at target Hz.
        
        Uses os.nice() to lower priority so gait loop gets CPU preference.
        During blink animation, skips frames based on blink_frame_divisor to reduce CPU load.
        """
        import os
        try:
            os.nice(5)  # Lower priority (higher nice = lower priority)
        except (OSError, AttributeError):
            pass  # nice() may fail on some systems
        
        next_tick = time.monotonic()
        last_look_x = 0.0
        last_look_y = 0.0
        blink_frame_counter = 0  # Counter for frame skipping during blink
        # Track for change detection (force render on status change)
        last_teensy_connected = True
        last_controller_connected = True
        last_telemetry_stale = False
        last_robot_enabled = True
        last_safety_active = False
        last_safety_text = ""
        
        while not self._stop_event.is_set():
            try:
                # Get current state snapshot
                with self._lock:
                    servo = self._servo
                    legs = self._legs
                    state = self._state
                    mirror = self._mirror
                    menu_state = self._menu_state
                    force = self._force_update
                    self._force_update = False
                    look_x = self._look_x
                    look_y = self._look_y
                    teensy_connected = self._teensy_connected
                    controller_connected = self._controller_connected
                    telemetry_stale = self._telemetry_stale
                    robot_enabled = self._robot_enabled
                    safety_active = self._safety_active
                    safety_text = self._safety_text
                
                # Detect status changes - force immediate render
                status_changed = (teensy_connected != last_teensy_connected or 
                                 controller_connected != last_controller_connected or
                                 telemetry_stale != last_telemetry_stale or
                                 robot_enabled != last_robot_enabled or
                                 safety_active != last_safety_active or
                                 safety_text != last_safety_text)
                if status_changed:
                    last_teensy_connected = teensy_connected
                    last_controller_connected = controller_connected
                    last_telemetry_stale = telemetry_stale
                    last_robot_enabled = robot_enabled
                    last_safety_active = safety_active
                    last_safety_text = safety_text
                
                # Detect look direction change (threshold to avoid noise)
                look_changed = (abs(look_x - last_look_x) > 0.05 or 
                               abs(look_y - last_look_y) > 0.05)
                if look_changed:
                    last_look_x = look_x
                    last_look_y = look_y
                
                # Apply eye look direction based on joystick input
                # Only override eye position if we have actual look input (gait active)
                # When look values are zero, let other code (gamepad handler) control eyes
                if abs(last_look_x) > 0.01 or abs(last_look_y) > 0.01:
                    self.eyes.eye_center_offset = self._base_center_offset + last_look_x * self.look_range_x
                    self.eyes.eye_vertical_offset = self._base_vertical_offset + last_look_y * self.look_range_y
                
                # Update eye animation (handles blink timing internally)
                needs_render = self.eyes.update(force_update=look_changed or status_changed)
                
                # During blink animation, skip frames based on divisor to reduce CPU/SPI load
                # divisor=1: no skip, divisor=2: render every other, divisor=3: render every third
                # BLINKSTATE: OPEN=0, CLOSED=1, BLINKING=2, WAITING=3
                # Skip during BLINKING (closing) and CLOSED (opening) phases
                is_blinking = (self.eyes.blinkState == 1 or self.eyes.blinkState == 2)  # CLOSED or BLINKING
                if is_blinking and self.blink_frame_divisor > 1:
                    blink_frame_counter = (blink_frame_counter + 1) % self.blink_frame_divisor
                    if blink_frame_counter != 0:
                        needs_render = False  # Skip this frame
                else:
                    blink_frame_counter = 0  # Reset counter when not blinking
                
                if needs_render or force or look_changed or status_changed:
                    # Render and send to display
                    UpdateDisplay(self.disp, self.eyes.display_image, 
                                 self.menu._image if self.menu else None,
                                 servo, legs, state, mirror, menu_state,
                                 teensy_connected=teensy_connected,
                                 controller_connected=controller_connected,
                                 telemetry_stale=telemetry_stale,
                                 robot_enabled=robot_enabled,
                                 safety_active=safety_active,
                                 safety_text=safety_text)
                
            except (AttributeError, TypeError, OSError) as e:
                # Display/SPI errors - log but don't crash thread
                pass  # Thread should continue; errors are transient
            
            # Maintain target frame rate
            next_tick += self.period_s
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Fell behind; reset timing
                next_tick = time.monotonic()


def processTelemS1(elements, state):
    """Processes the S1 telemetry data.
    Expected: 10 numeric fields (indices 0..9). Warn if shorter; ignore extras if longer."""
    expected = 10
    count = len(elements)
    if count < expected:
        if _verbose:
            print(f"WARN S1 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and _verbose:
        print(f"WARN S1 length {count} > expected {expected}, truncating.", end="\r\n")
    try:
        # Only use first expected elements
        for i in range(expected):
            state[i] = float(elements[i])
    except ValueError as e:
        print(f"Error processing S1 telemetry data: {e}", end="\r\n")

def processTelemS2(elements, servo):
    """Processes the S2 telemetry data (servo enable states).
    Expected: 18 enable integers."""
    expected = 18
    count = len(elements)
    if count < expected:
        if _verbose:
            print(f"WARN S2 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and _verbose:
        print(f"WARN S2 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(expected):
            servo[idx][2] = int(elements[idx])
    except ValueError as e:
        print(f"Error processing S1 telemetry data: {e}", end="\r\n")

def processTelemS3(elements, servo):
    """Processes the S3 telemetry data (servo voltage + temperature).
    Expected: 36 numeric entries: 18 voltages then 18 temperatures."""
    expected = 36
    count = len(elements)
    if count < expected:
        if _verbose:
            print(f"WARN S3 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and _verbose:
        print(f"WARN S3 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(18):
            servo[idx][0] = float(elements[idx])
            servo[idx][1] = int(elements[idx + 18])
    except ValueError as e:
        print(f"Error processing S3 telemetry data: {e}", end="\r\n")

def processTelemS4(elements, leg):
    """Processes the S4 telemetry data (leg contact states).
    Expected: 6 integers (one per leg)."""
    expected = 6
    count = len(elements)
    if count < expected:
        if _verbose:
            print(f"WARN S4 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and _verbose:
        print(f"WARN S4 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(expected):
            leg[idx][0] = int(elements[idx])
    except ValueError as e:
        print(f"Error processing S4 telemetry data: {e}", end="\r\n")


# Safety telemetry (S5) and overlay helpers
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


def processTelemS5(elements, safety_state):
    """Process S5 safety telemetry.

    Expected format (7 fields):
        lockout, cause_mask, override_mask, clearance_mm, soft_limits,
        collision, temp_c
    """
    expected = 7
    count = len(elements)
    if count < expected:
        if _verbose:
            print(f"WARN S5 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and _verbose:
        print(f"WARN S5 length {count} > expected {expected}, truncating.", end="\r\n")
    try:
        safety_state["lockout"] = bool(int(elements[0]))
        safety_state["cause_mask"] = int(elements[1])
        safety_state["override_mask"] = int(elements[2])
        safety_state["clearance_mm"] = int(elements[3])
        safety_state["soft_limits"] = bool(int(elements[4]))
        safety_state["collision"] = bool(int(elements[5]))
        safety_state["temp_c"] = int(elements[6])
    except ValueError as e:
        print(f"Error processing S5 telemetry data: {e}", end="\r\n")


def get_safety_overlay_state():
    """Return (active, text) tuple for safety overlay rendering."""
    active = _safety_state.get("lockout", False)
    if not active:
        return False, ""
    mask = _safety_state.get("cause_mask", 0)
    labels = []
    # Bit mapping from firmware LockoutCauseBits
    if mask & 0x01:
        labels.append("TEMP")
    if mask & 0x02:
        labels.append("COLLISION")
    if not labels:
        labels.append("UNKNOWN")
    base = "SAFETY LOCKOUT: " + "/".join(labels)
    return True, base


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
_verbose = True # enables verbose logging and debugging
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
_menuVisible = False # flag to show or hide the menu
_mirrorDisplay = False # flag to mirror the display to the console
_forceDisplayUpdate = False # flag to force the display update
_displayBrightness = 100 # initial brightness of the display
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

# create the Teensy serial object
_teensy = None
# create the Xbox controller object
_controller = None

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
_eyeBlinkPercentStep = 0.25
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
# Menu settings (saved to config)
_menuTheme = 0      # 0=MARS, 1=LCARS
_menuPalette = 0    # LCARS palette: 0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD


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
_blinkFrameDivisor = 2  # Skip N-1 out of N frames during blink (1=no skip, 2=half rate, 3=third rate)
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
    # Load menu settings
    if 'menu' in _cfg:
        _menuTheme = _cfg.getint('menu', 'theme', fallback=_menuTheme)
        _menuPalette = _cfg.getint('menu', 'palette', fallback=_menuPalette)

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
        _cmdThrottleMs = _cfg.getfloat('gait', 'cmd_throttle_ms', fallback=_cmdThrottleMs)
        _gaitTurnMaxDegS = _cfg.getfloat('gait', 'turn_max_deg_s', fallback=_gaitTurnMaxDegS)
        # Bezier curve shape
        _bezierP1Height = _cfg.getfloat('gait', 'bezier_p1_height', fallback=_bezierP1Height)
        _bezierP1Overshoot = _cfg.getfloat('gait', 'bezier_p1_overshoot', fallback=_bezierP1Overshoot)
        _bezierP2Height = _cfg.getfloat('gait', 'bezier_p2_height', fallback=_bezierP2Height)
        _bezierP3Height = _cfg.getfloat('gait', 'bezier_p3_height', fallback=_bezierP3Height)
        _bezierP3Overshoot = _cfg.getfloat('gait', 'bezier_p3_overshoot', fallback=_bezierP3Overshoot)
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
except (configparser.Error, ValueError, KeyError) as e:
    print(f"Config load error (using defaults): {e}", end="\r\n")
    _savedGaitWidthMm = 100.0
    _savedGaitLiftMm = 60.0
    _gaitWidthMinMm = 50.0
    _gaitWidthMaxMm = 175.0
    _gaitLiftMinMm = 20.0
    _gaitLiftMaxMm = 100.0

def save_gait_settings(width_mm: float, lift_mm: float, cycle_ms: int = None, turn_max_deg_s: float = None) -> bool:
    """Save gait parameters to controller.ini [gait] section.

    width_mm and lift_mm are always persisted; cycle_ms is optional and
    only written when provided to avoid clobbering existing config
    unexpectedly."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'gait' not in _cfg:
            _cfg.add_section('gait')
        _cfg.set('gait', 'width_mm', f'{width_mm:.1f}')
        _cfg.set('gait', 'lift_mm', f'{lift_mm:.1f}')
        if cycle_ms is not None:
            _cfg.set('gait', 'cycle_ms', str(int(cycle_ms)))
        if turn_max_deg_s is not None:
            _cfg.set('gait', 'turn_max_deg_s', f'{turn_max_deg_s:.1f}')
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save gait settings: {e}", end="\r\n")
        return False

def save_eye_shape(shape: int) -> bool:
    """Save eye shape to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'shape', str(shape))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye shape: {e}", end="\r\n")
        return False

def save_human_eye_settings(size: int = None, color_idx: int = None) -> bool:
    """Save human eye settings to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        if size is not None:
            _cfg.set('eyes', 'human_eye_size', str(size))
        if color_idx is not None:
            _cfg.set('eyes', 'human_eye_color', str(color_idx))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save human eye settings: {e}", end="\r\n")
        return False

def save_eye_center_offset(offset: int) -> bool:
    """Save eye horizontal center offset to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'center_offset', str(offset))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye center offset: {e}", end="\r\n")
        return False

def save_eye_vertical_offset(offset: int) -> bool:
    """Save eye vertical center offset to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'vertical_offset', str(offset))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye vertical offset: {e}", end="\r\n")
        return False

def save_menu_settings(theme: int = None, palette: int = None) -> bool:
    """Save menu theme and palette to controller.ini [menu] section."""
    global _cfg, _cfg_path, _menuTheme, _menuPalette
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'menu' not in _cfg:
            _cfg.add_section('menu')
        if theme is not None:
            _cfg.set('menu', 'theme', str(theme))
            _menuTheme = theme
        if palette is not None:
            _cfg.set('menu', 'palette', str(palette))
            _menuPalette = palette
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save menu settings: {e}", end="\r\n")
        return False


def _fmt_triplet(tri):
    try:
        if tri is None or len(tri) != 3:
            return None
        return f"{int(tri[0])}/{int(tri[1])}/{int(tri[2])}"
    except Exception:
        return None


def save_pid_settings() -> bool:
    """Persist current PID tab values to controller.ini [pid]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'pid' not in _cfg:
            _cfg.add_section('pid')
        if _pid_state.get('enabled') is not None:
            _cfg.set('pid', 'enabled', 'true' if _pid_state['enabled'] else 'false')
        if _pid_state.get('mode') is not None:
            _cfg.set('pid', 'mode', str(_pid_state['mode']))
        if _pid_state.get('shadow_hz') is not None:
            _cfg.set('pid', 'shadow_hz', str(int(_pid_state['shadow_hz'])))
        for k in ('kp', 'ki', 'kd', 'kdalph'):
            s = _fmt_triplet(_pid_state.get(k))
            if s is not None:
                _cfg.set('pid', k, s)
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_imp_settings() -> bool:
    """Persist current IMP tab values to controller.ini [imp]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'imp' not in _cfg:
            _cfg.add_section('imp')
        if _imp_state.get('enabled') is not None:
            _cfg.set('imp', 'enabled', 'true' if _imp_state['enabled'] else 'false')
        if _imp_state.get('mode') is not None:
            _cfg.set('imp', 'mode', str(_imp_state['mode']))
        if _imp_state.get('scale') is not None:
            _cfg.set('imp', 'scale', str(int(_imp_state['scale'])))
        for k in ('jspring', 'jdamp', 'cspring', 'cdamp'):
            s = _fmt_triplet(_imp_state.get(k))
            if s is not None:
                _cfg.set('imp', k, s)
        if _imp_state.get('jdb_cd') is not None:
            _cfg.set('imp', 'jdb_cd', str(int(_imp_state['jdb_cd'])))
        if _imp_state.get('cdb_mm') is not None:
            _cfg.set('imp', 'cdb_mm', f"{float(_imp_state['cdb_mm']):.3f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_est_settings() -> bool:
    """Persist current EST tab values to controller.ini [est]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'est' not in _cfg:
            _cfg.add_section('est')
        for k in ('cmd_alpha_milli', 'meas_alpha_milli', 'meas_vel_alpha_milli'):
            if _est_state.get(k) is not None:
                _cfg.set('est', k, str(int(_est_state[k])))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False

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
UpdateDisplay(_disp, _backGroundImage, None, _servo, _legs, _state, _mirrorDisplay)  # display the blank image
drawLogo(_disp)  # draw the logo on the display
time.sleep(5)

# Startup banner (printed to stdout for logs/USB serial capture)
print("MARS - Modular Autonomous Robotic System", end="\r\n")
print(f"Firmware 0.2.36/b152 · Controller {CONTROLLER_VERSION}/b{CONTROLLER_BUILD}", end="\r\n")

# initizlizxe the touch screen
_touch = cst816d.cst816d()

#create the menu object (legacy menu, kept for compatibility)
_menu = GUIMenu(_touch)

# Create new MARS menu system
_marsMenu = MarsMenu(_touch)

time.sleep(1) # take a short nap

# create and initialize the SimpleEyes object
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
_eyes.update(force_update=True)  # force the initial update to draw the eyes
UpdateDisplay(_disp, _eyes.display_image, _menu.image, _servo, _legs, _state, _mirrorDisplay, _menuState)  # display the eyes on the display

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
        global _eyes, _forceDisplayUpdate
        _eyes.crt_mode = (val == 1)
        _eyes.update(force_update=True)
        _forceDisplayUpdate = True
    
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
        global _marsMenu
        _marsMenu.lcars_palette = val
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
            save_pid_settings()
            send_cmd(b'PID ENABLE' if val == 1 else b'PID DISABLE', force=True)
            _kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_mode_change(val):
        try:
            _pid_state['mode'] = 'active' if val == 0 else 'shadow'
            save_pid_settings()
            send_cmd(b'PID MODE ACTIVE' if val == 0 else b'PID MODE SHADOW', force=True)
            _kick_list_poll('PID')
        except Exception:
            pass

    def on_pid_shadow_hz_change(val):
        try:
            _pid_state['shadow_hz'] = int(val)
            save_pid_settings()
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
                save_pid_settings()
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
                save_pid_settings()
                send_cmd(f"PID KDALPHA {joint} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('PID')
            except Exception:
                pass
        return _cb

    # === IMP callbacks ===
    def on_imp_enabled_change(val):
        try:
            _imp_state['enabled'] = (val == 1)
            save_imp_settings()
            send_cmd(b'IMP ENABLE' if val == 1 else b'IMP DISABLE', force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_mode_change(val):
        try:
            _imp_state['mode'] = 'joint' if val == 1 else ('cart' if val == 2 else 'off')
            save_imp_settings()
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
            save_imp_settings()
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
                save_imp_settings()
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
                save_imp_settings()
                send_cmd(f"IMP {kind} {axis} {int(val)}".encode('ascii'), force=True)
                _kick_list_poll('IMP')
            except Exception:
                pass
        return _cb

    def on_imp_jdb_change(val):
        try:
            _imp_state['jdb_cd'] = int(val)
            save_imp_settings()
            send_cmd(f"IMP JDB {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    def on_imp_cdb_change(val):
        try:
            _imp_state['cdb_mm'] = float(val)
            save_imp_settings()
            send_cmd(f"IMP CDB {float(val):.3f}".encode('ascii'), force=True)
            _kick_list_poll('IMP')
        except Exception:
            pass

    # === EST callbacks ===
    def on_est_cmd_alpha_change(val):
        try:
            _est_state['cmd_alpha_milli'] = int(val)
            save_est_settings()
            send_cmd(f"EST CMD_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass

    def on_est_meas_alpha_change(val):
        try:
            _est_state['meas_alpha_milli'] = int(val)
            save_est_settings()
            send_cmd(f"EST MEAS_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass

    def on_est_vel_alpha_change(val):
        try:
            _est_state['meas_vel_alpha_milli'] = int(val)
            save_est_settings()
            send_cmd(f"EST MEAS_VEL_ALPHA {int(val)}".encode('ascii'), force=True)
            _kick_list_poll('EST')
        except Exception:
            pass
    
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
        apply_posture(b'STAND', auto_disable_s=_autoDisableS)
        _marsMenu.hide()
    
    def on_tuck():
        apply_posture(b'TUCK', auto_disable_s=_autoDisableS)
        _marsMenu.hide()
    
    def on_home():
        apply_posture(b'HOME', auto_disable_s=_autoDisableS)
        _marsMenu.hide()
    
    _marsMenu.set_callback(MenuCategory.POSTURE, "Stand", "on_select", on_stand)
    _marsMenu.set_callback(MenuCategory.POSTURE, "Tuck", "on_select", on_tuck)
    _marsMenu.set_callback(MenuCategory.POSTURE, "Home", "on_select", on_home)

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

    _marsMenu.set_callback(MenuCategory.SAFETY, "Clear Safety", "on_select", on_clear_safety)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override ALL", "on_select", on_override_all)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override TEMP", "on_select", on_override_temp)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override COLLISION", "on_select", on_override_collision)
    _marsMenu.set_callback(MenuCategory.SAFETY, "Override NONE", "on_select", on_override_none)
    
    # === Sync initial values ===
    _marsMenu.set_value(MenuCategory.EYES, "Style", _eyes.left_shape)
    _marsMenu.set_value(MenuCategory.EYES, "Human Color", _eyes.human_eye_color_idx)
    _marsMenu.set_value(MenuCategory.EYES, "Size", _eyes.human_eye_size)
    _marsMenu.set_value(MenuCategory.EYES, "V Center", _eyeVerticalOffset)
    _marsMenu.set_value(MenuCategory.EYES, "Spacing", int(_eyes.human_eye_spacing_pct * 100))
    _marsMenu.set_value(MenuCategory.EYES, "CRT Effect", 1 if _eyes.crt_mode else 0)
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
    
    # Apply saved theme/palette
    _marsMenu.theme = _menuTheme
    _marsMenu.lcars_palette = _menuPalette

_setup_mars_menu()

def update_menu_info():
    """Update INFO menu items from current telemetry data.

    INFO tab shows instantaneous values; System tab can show aggregated stats
    such as average battery voltage or servo temperature when available.
    """
    global _marsMenu, _state, _servo, _safety_state
    if _marsMenu is None:
        return
    
    # Compute average servo voltage from _servo (S3 telemetry)
    avg_servo_v = None
    if _servo is not None:
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

    # Update INFO items from _state (S1 telemetry) - instantaneous values
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
    
    # Find servo temperature statistics from _servo array
    max_temp = None
    temp_sum = 0.0
    temp_count = 0
    if _servo is not None:
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
    # SYSTEM tab can show average servo temperature when available
    _marsMenu.set_value(MenuCategory.SYSTEM, "Avg Servo Temp", (temp_sum / temp_count) if temp_count > 0 else None)

    # SAFETY tab: mirror latest S5 snapshot
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

# Create and start display thread if enabled
_displayThread = None
if _displayThreadEnabled:
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
    _displayThread.start()
    if _verbose:
        print(f"Display thread started at {_displayThreadHz} Hz (blink divisor: {_blinkFrameDivisor})", end="\r\n")

# --------------------------------------------------------------------------------------------------
# Command helper: centralizes Teensy command emission (newline termination + throttling/dedup)
# --------------------------------------------------------------------------------------------------
_cmd_last = {}            # bytes(command) -> last send time (seconds)
_enabledLocal = False     # local view (updated when we send ENABLE / DISABLE)

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
    if _debugSendAll or (_verbose and cmd_b in _gaitDebugCmds):
        print(f"[send_cmd] SENT cmd='{cmd_b}' force={force} throttle_ms={throttle_ms} t={now:.3f}", end="\r\n")
    return True


def ensure_enabled() -> bool:
    """Ensure robot is enabled by sending LEG ALL ENABLE + ENABLE if needed.
    
    Checks both local tracking (_enabledLocal) and telemetry state (_state[IDX_ROBOT_ENABLED]).
    Sends enable sequence only if robot appears disabled.
    
    Returns:
        True if enable commands were sent, False if already enabled or no Teensy.
    """
    global _enabledLocal
    # Honor firmware safety lockout: never try to re-enable while locked out
    if _safety_state.get("lockout", False):
        if _verbose:
            print("Safety lockout active; refusing ENABLE. Clear SAFETY on Teensy first.", end="\r\n")
        return False
    if _teensy is None:
        return False
    # Check both local tracking and telemetry state
    if _enabledLocal and _state[IDX_ROBOT_ENABLED] == 1.0:
        return False  # Already enabled
    send_cmd(b'LEG ALL ENABLE', force=True)
    send_cmd(b'ENABLE', force=True)
    return True


# Tuck auto-disable scheduling
_autoDisableAt = None  # when not None, time (epoch seconds) at which to send DISABLE after posture command (tuck/stand)
# Enable flag consumed directly from _state[IDX_ROBOT_ENABLED]; no SQ telemetry support required.
_autoDisableReason = None  # string reason for pending auto-disable (e.g., 'TUCK')
_autoDisableGen = 0        # generation counter to detect stale timers
_lastPosture = None        # tracks last posture name sent (for future dynamic messaging)

def apply_posture(name, auto_disable_s: float = None, require_enable: bool = True):
    """Unified posture helper (TUCK/STAND/HOME) using byte commands.
    Auto enable sequence if required; schedules auto disable if >0.
    Auto-disable is reason-aware: posture auto-disable will not fire if superseded
    by a newer posture/gait command.
    """
    global _autoDisableAt, _autoDisableReason, _autoDisableGen, _lastPosture
    if auto_disable_s is None:
        auto_disable_s = _autoDisableS
    if _teensy is None:
        return False
    # normalize posture to bytes upper
    if isinstance(name, bytes):
        posture = name.upper()
    elif isinstance(name, str):
        try:
            posture = name.strip().upper().encode('ascii')
        except Exception:
            return False
    else:
        return False
    if posture not in (b'TUCK', b'STAND', b'HOME'):
        if _verbose:
            print(f"Unknown posture '{posture}'", end="\r\n")
        return False
    if require_enable and _state[IDX_ROBOT_ENABLED] != 1.0:
        ensure_enabled()
    if send_cmd(posture, force=True):
        _lastPosture = posture
        # Bump generation on every successful posture command
        _autoDisableGen += 1
        reason = posture.decode('ascii', errors='ignore') if isinstance(posture, (bytes, bytearray)) else str(posture)
        if auto_disable_s > 0:
            _autoDisableAt = time.time() + auto_disable_s
            _autoDisableReason = reason
            if _verbose:
                print(f"\n{posture} sequence sent. Auto DISABLE in {auto_disable_s:.0f}s (reason={reason}).", end="\r\n")
        else:
            _autoDisableAt = None
            _autoDisableReason = None
        return True
    return False

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
    
    Args:
        ctrl: Controller instance
    """
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
        update_menu_info()
    
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
        robot_enabled = (ctrl.state[IDX_ROBOT_ENABLED] == 1.0) if len(ctrl.state) > IDX_ROBOT_ENABLED else True
        safety_active, safety_text = get_safety_overlay_state()
        
        # Update thread state - it handles rendering at its own rate
        displayThread.update_state(
            servo=ctrl.servo, legs=ctrl.legs, state=ctrl.state,
            mirror=ctrl.mirror, menu_state=ctrl.menuState,
            force_update=ctrl.forceDisplayUpdate, verbose=ctrl.verbose,
            look_x=look_x, look_y=look_y,
            teensy_connected=(ctrl.teensy is not None),
            controller_connected=(ctrl.controller is not None),
                telemetry_stale=telemetry_stale,
                robot_enabled=robot_enabled,
                safety_active=safety_active,
                safety_text=safety_text
        )
        ctrl.forceDisplayUpdate = False
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
                print("\nTouchscreen E-STOP: gait stopped, robot disabled", end="\r\n")
            ctrl.forceDisplayUpdate = True
            return True
        
        # MARS menu touch handling when menu is visible
        if _marsMenu.visible:
            _marsMenu.handle_touch()
            ctrl.forceDisplayUpdate = True
            return True
        
        # If robot is disabled and not in motion, allow opening menu
        robot_enabled = (_state[IDX_ROBOT_ENABLED] == 1.0) if len(_state) > IDX_ROBOT_ENABLED else False
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
                print("\nMARS menu closed (ESC)", end="\r\n")
            return True

    if key == 27:  # ESC key with no menu open: exit application
        if _verbose:
            print("\nESC key pressed, exiting loop", end="\r\n")
        return False
    
    elif key in (ord('t'), ord('T')):  # Test gait
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\nTest gait blocked: firmware safety lockout is active.", end="\r\n")
        else:
            if _state[IDX_ROBOT_ENABLED] != 1.0:
                ensure_enabled()
            send_cmd(b'T', force=True)
            if _verbose:
                print("\nTest gait command sent to Teensy", end="\r\n")
    
    elif key in (ord('k'), ord('K')):  # Tuck posture
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\nTUCK blocked: firmware safety lockout is active.", end="\r\n")
        else:
            if _state[IDX_ROBOT_ENABLED] != 1.0:
                ensure_enabled()
            send_cmd(b'TUCK', force=True)
            _autoDisableAt = time.time() + _autoDisableS
            ctrl.autoDisableAt = _autoDisableAt
    
    elif key in (ord('s'), ord('S')):  # Stand posture
        if _safety_state.get("lockout", False):
            if _verbose:
                print("\nSTAND blocked: firmware safety lockout is active.", end="\r\n")
        else:
            if _state[IDX_ROBOT_ENABLED] != 1.0:
                ensure_enabled()
            send_cmd(b'STAND', force=True)
            _autoDisableAt = time.time() + _autoDisableS
            ctrl.autoDisableAt = _autoDisableAt
    
    elif key in (ord('n'), ord('N')):  # Menu toggle (MARS menu)
        # Mirror Start-button behavior: only allow menu when disabled & not in motion
        robot_enabled = (_state[IDX_ROBOT_ENABLED] == 1.0) if len(_state) > IDX_ROBOT_ENABLED else False
        if _marsMenu.visible:
            _marsMenu.hide()
            ctrl.forceDisplayUpdate = True
            if _verbose:
                print("\nMARS menu closed (keyboard 'n')", end="\r\n")
        elif not robot_enabled and not _gaitActive:
            _marsMenu.show()
            ctrl.forceDisplayUpdate = True
            if _verbose:
                print("\nMARS menu opened (keyboard 'n')", end="\r\n")
        else:
            if _verbose:
                print("\nMARS menu blocked: disable robot first", end="\r\n")
    
    elif key in (ord('m'), ord('M')):  # Mirror toggle
        _mirrorDisplay = not _mirrorDisplay
        ctrl.mirror = _mirrorDisplay
        if _verbose:
            print("\nMirror display mode ON" if _mirrorDisplay else "\nMirror display mode OFF", end="\r\n")
        _forceDisplayUpdate = True
        ctrl.forceDisplayUpdate = True
    
    elif key in (ord('v'), ord('V')):  # Verbose toggle
        _verbose = not _verbose
        ctrl.verbose = _verbose
        print("\nverbose mode ON" if _verbose else "\nverbose mode OFF", end="\r\n")
    
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
                print("\nGait start blocked: firmware safety lockout is active.", end="\r\n")
        elif not _gaitActive:
            if _state[IDX_ROBOT_ENABLED] != 1.0:
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
                print("\nGait engine STARTED (tripod walk)", end="\r\n")
        elif _verbose:
            print("\nGait already active", end="\r\n")
    
    elif key in (ord('h'), ord('H')):  # Stop gait (halt)
        if _gaitActive and _gaitEngine is not None:
            _gaitEngine.stop()
            _gaitActive = False
            send_cmd(b'STAND', force=True)
            _autoDisableAt = time.time() + _autoDisableS
            ctrl.autoDisableAt = _autoDisableAt
            if _verbose:
                print("\nGait engine STOPPED (halt)", end="\r\n")
        elif _verbose:
            print("\nNo gait active to stop", end="\r\n")
    
    elif key in (ord('p'), ord('P')):  # Stationary pattern
        if not _gaitActive:
            if _state[IDX_ROBOT_ENABLED] != 1.0:
                ensure_enabled()
            params = GaitParams(base_x_mm=100.0, base_y_mm=-120.0)
            _gaitEngine = StationaryPattern(params, radius_mm=15.0, period_ms=2000)
            _gaitEngine.start()
            _gaitActive = True
            if _verbose:
                print("\nStationary pattern STARTED", end="\r\n")
        elif _verbose:
            print("\nGait already active (press 'h' to stop first)", end="\r\n")
    
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
            print("\n'q' key pressed: idle + LEG ALL DISABLE + DISABLE sequence sent", end="\r\n")
    
    elif key in (ord('x'), ord('X')):  # Exit
        if _verbose:
            print("\n'x' key pressed, exiting loop", end="\r\n")
        return False
    
    return True


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
                if ctrl.verbose:
                    print(f"\nGait transition complete: now running {new_name}", end="\r\n")
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
                send_cmd(feet_cmd, force=True)
    else:
        # Normal gait operation - tick the single gait engine
        gaitEngine.tick(dt_seconds)
        
        # Send FEET command every N ticks to reduce serial load
        _gaitTickCount += 1
        if _gaitTickCount >= _gaitSendDivisor:
            _gaitTickCount = 0
            feet_cmd = gaitEngine.get_feet_bytes()
            send_cmd(feet_cmd, force=True)


def phase_auto_disable(ctrl):
    """Phase 8: Check and execute scheduled auto-disable.
    
    Args:
        ctrl: Controller instance
    """
    global _autoDisableAt, _autoDisableReason, _autoDisableGen

    if ctrl.autoDisableAt is not None and time.time() >= ctrl.autoDisableAt:
        # Only honor auto-disable if reason/generation still match and robot is enabled
        reason = getattr(ctrl, 'autoDisableReason', None)
        gen = getattr(ctrl, 'autoDisableGen', 0)
        if gen == _autoDisableGen and reason == _autoDisableReason:
            if ctrl.teensy is not None and ctrl.state[IDX_ROBOT_ENABLED] == 1.0:
                send_cmd(b'DISABLE', force=True)
                if ctrl.verbose:
                    print(f"\nAuto DISABLE executed (reason={reason})", end="\r\n")
        # Clear any pending auto-disable (stale or executed)
        ctrl.autoDisableAt = None
        ctrl.autoDisableReason = None
        ctrl.autoDisableGen = gen
        _autoDisableAt = None
        _autoDisableReason = None


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

# start the main loop
# Construct controller instance and mirror initial state
ctrl = Controller.from_current_globals()
# Initialize monotonic loop timing
_nextTickTime = time.monotonic()
# Initialize persistent keyboard input
init_keyboard()

while _run:

    # Phase 0: Sync globals to controller
    sync_globals_to_ctrl(ctrl)
    
    try:
        #----------------------------------------------------------------------
        # Phase 1: Timing update
        #----------------------------------------------------------------------
        _loopdt, _ = phase_timing_update()

        #----------------------------------------------------------------------
        # Phase 2: Teensy connection and polling
        #----------------------------------------------------------------------
        phase_teensy_connection(ctrl)

        #----------------------------------------------------------------------
        # Phase 3: Xbox controller connection
        #----------------------------------------------------------------------
        phase_gamepad_connection(ctrl)

        #----------------------------------------------------------------------
        # Phase 3b: Housekeeping (telemetry auto-start/retry)
        #----------------------------------------------------------------------
        ctrl.housekeeping()

        #----------------------------------------------------------------------
        # Phase 3c: Telemetry debug (optional)
        #----------------------------------------------------------------------
        if _debugTelemetry:
            if ctrl.lastRawS1:
                print(f"RAW S1: {ctrl.lastRawS1}", end="\r\n")
                print(f"PARSED S1: {ctrl.lastParsedS1}", end="\r\n")
            if ctrl.lastRawS2:
                print(f"RAW S2: {ctrl.lastRawS2}", end="\r\n")
                print(f"PARSED S2 ENABLES: {ctrl.lastParsedS2}", end="\r\n")

        #----------------------------------------------------------------------
        # Phase 4: Gamepad polling (before display so button presses render immediately)
        #----------------------------------------------------------------------
        ctrl.poll_gamepad()

        # Check for exit request from Controller (power button)
        if ctrl.requestExit:
            if _verbose:
                print("Exit requested via power button.", end="\r\n")
            _run = False
            break

        #----------------------------------------------------------------------
        # Phase 6: Touch input (before display update so changes show immediately)
        #----------------------------------------------------------------------
        phase_touch_input(_menu, ctrl)

        #----------------------------------------------------------------------
        # Phase 6b: Display update (after touch so menu changes render immediately)
        #----------------------------------------------------------------------
        phase_display_update(ctrl, _displayThread, _gaitEngine, _gaitActive)

        #----------------------------------------------------------------------
        # Phase 7: Keyboard input
        #----------------------------------------------------------------------
        if not phase_keyboard_input(ctrl):
            _run = False

        #----------------------------------------------------------------------
        # Phase 8: Gait engine tick
        #----------------------------------------------------------------------
        phase_gait_tick(ctrl, _gaitEngine, _gaitActive)

        #----------------------------------------------------------------------
        # Phase 9: Auto-disable check
        #----------------------------------------------------------------------
        phase_auto_disable(ctrl)

        #----------------------------------------------------------------------
        # Phase 10: Loop timing / sleep
        #----------------------------------------------------------------------
        phase_loop_sleep(ctrl)

        #----------------------------------------------------------------------
        # Phase 11: Sync controller state back to globals
        #----------------------------------------------------------------------
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


# MARS Hexapod User Manual

Revision: Firmware 0.2.36/b152 · Controller 0.5.27/b138

> Command imperative: Whenever firmware (`FW_VERSION`/`FW_BUILD`) or controller
> (`CONTROLLER_VERSION`/`CONTROLLER_BUILD`) behavior changes, this manual **must**
> be reviewed and updated in the same commit. The revision line above should be
> updated to match the new versions.

---

## 0. Index / Quick Reference

- **1. Overview** – System summary and components
- **2. Hardware & Connections** – Teensy, Pi, buses, and wiring
- **3. Firmware Usage** – Build/flash, config, serial commands
- **4. Python Controller Usage** – Startup, keyboard, gamepad, menus
- **5. Theory of Operation (Firmware)** – Loop timing, IK, safety, telemetry
- **6. Theory of Operation (Controller)** – Main loop phases, gait engine, overlays
- **7. Safety Behavior** – Lockouts, auto-disable, E‑STOP paths
- **8. Quick Reference: Commands & Keys** – One-page command/keys cheat sheet
- **9. Troubleshooting** – Common issues and resolutions
- **10. Maintenance & Versioning Rules** – How to keep code + docs in sync

---

---

## 1. Overview

MARS — Modular Autonomous Robotic System is a hexapod robot built around a
Teensy 4.1 microcontroller running the `MARS_Hexapod` firmware and a Raspberry Pi
running the Python controller (`controller.py`). The Teensy manages all low-level
servo control and safety, while the Pi handles user interaction, gaits, and the
visual LCD/touchscreen UI.

- **Firmware**: `firmware/MARS_Hexapod` (Teensy 4.1)
- **Controller**: `rPi controller/controller.py` (Raspberry Pi)
- **Servos**: 18x Hiwonder HTS‑35S serial bus servos via `lx16a-servo` library

This manual describes:

- Hardware and wiring at a high level
- How to flash and configure the firmware
- How to run and use the Python controller
- Command protocols and safety behavior
- MARS menu system (tabs and controls)

---

## 2. Hardware & Connections (High Level)

- **Host MCU**: Teensy 4.1
  - One UART bus per leg (6 total), 3 servos each
  - SD card used for `/config.txt` and logs
- **Controller host**: Raspberry Pi (tested on Pi 5)
  - USB connection to Teensy
  - SPI connection to 1.9" LCD (ST7789)
  - I2C connection to capacitive touch controller (cst816d)
- **Gamepad**: Xbox-compatible controller via USB, read using `evdev`.

Refer to `docs/PROJECT_SPEC.md` for detailed wiring and acceptance criteria.

---

## 3. Firmware (Teensy) Usage

### 3.1 Building & Flashing

1. Open `firmware/MARS_Hexapod/MARS_Hexapod.ino` in Arduino IDE 1.8.19 with
   Teensyduino installed.
2. Select board **Teensy 4.1**, appropriate USB type (Serial), and correct port.
3. Compile and upload.

On boot, the firmware prints a short banner over USB serial including:

- Project name (MARS)
- Firmware version/build
- Loop rate and UART mapping
- Config/logging status

### 3.2 Configuration (`/config.txt`)

- Located on the Teensy SD card root as `/config.txt`.
- `key=value` per line, `#` for comments, blanks ignored.
- Examples:
  - `loop_hz=166`
  - `uart.LF=Serial1`
  - `tuck.tibia_cd=...`
  - `tuck.tol_tibia_cd=...`

The firmware reads this file on boot. Missing keys are auto-populated with
safe defaults so `/config.txt` always contains a full set of known keys.

The TUCK command family and other config-affecting commands (e.g., safety,
logging, test gait) update their corresponding keys at runtime and persist
them back to `/config.txt`.

### 3.3 Core Commands (USB Serial)

Firmware commands are ASCII lines terminated by `\n`. High-level categories:

- **System & Info**
  - `HELP` – print command help and short descriptions
  - `STATUS` – print grouped system status
- **Enable & Torque**
  - `ENABLE` / `DISABLE` – global motion enable/disable
  - `LEG ALL ENABLE` / `LEG ALL DISABLE` – leg torque control
  - `SERVOS ...` / `SERVO ...` – per-joint enable where needed
- **Motion & Posture**
  - `STAND` / `HOME` / `TUCK` – posture targets
  - `FOOT <LEG> x y z` – single-leg foot target in body frame (mm)
  - `FEET <x1> <y1> <z1> ... <x6> <y6> <z6>` – all legs in fixed order
- **Modes & Test**
  - `MODE TEST` / `MODE IDLE` – firmware gait vs idle mode
  - `TEST ...` – tripod test gait parameters (cycle, height, base, step, lift, overlap)
- **Safety**
  - `SAFETY LIST` / `SAFETY SOFTLIMITS` / `SAFETY COLLISION` / `SAFETY TEMPLOCK` / `SAFETY CLEARANCE`
  - Lockout triggers require explicit re-enable/override.

### 3.4 TUCK Parameters

The TUCK sequence can be tuned at runtime via:

- `TUCK PARAMS` – print current tuck parameters (`tuck.*` keys)
- `TUCK SET <PARAM> <VAL>` – update a parameter and persist to `/config.txt`:
  - `TIBIA`, `FEMUR`, `COXA` – posture targets (commanded positions)
  - `TOL_TIBIA`, `TOL_OTHER` – convergence tolerances
  - `TIMEOUT` – per-sequence timeout in milliseconds

Example:

```text
TUCK SET TOL_TIBIA 2000
TUCK PARAMS
```

---

## 4. Python Controller Usage

### 4.1 Starting the Controller

From the repository root on the Raspberry Pi:

```bash
cd "rPi controller"
python3 controller.py
```

On startup the controller:

- Loads configuration from `controller.ini`
- Connects to the Teensy over serial
- Starts telemetry and the LCD/touch UI
- Shows a MARS banner with controller version/build

### 4.2 Keyboard Controls (Terminal)

- `n` – toggle MARS menu (only when robot disabled and not moving)
- `Esc` – close MARS menu when open; exit app when menu not visible
- `w` / `h` / `p` – gait start/stop/pattern (legacy keyboard gaits)
- `t` – firmware test gait
- `k` – TUCK posture (with auto-disable)
- `s` – STAND posture
- `q` – idle + `LEG ALL DISABLE` + `DISABLE` (soft E‑STOP)

### 4.3 Gamepad Controls (High Level)

- `LB` – toggle gait (Python gait engine)
- `RB` – cycle gait type (Tripod, Wave, Ripple, Stationary)
- `Y` – TUCK posture
- `X` – toggle firmware MODE (TEST/IDLE)
- `A` – STAND
- `B` – HOME
- Left stick – gait speed (Y) and heading/strafe offset (X); pushing left
  strafes left, pushing right strafes right.
- Right stick – turn rate (Y) and, on some controllers, alternate X input
  used for strafe when gait is active; when gait is off, right stick X pans
  the eyes horizontally.

### 4.4 MARS Menu (LCD + Keyboard/Gamepad)

Tabs:

- **Eyes** – eye shape, color, vertical center (V Center), horizontal spacing,
  CRT effect and other visual tweaks. Eye settings persist in `controller.ini`
  under `[eyes]` (shape, colors, center_offset/vertical_offset,
  human_eye_spacing_pct, etc.).
- **Gait** – step height, length, turn rate (max yaw deg/s), cycle time,
  start/stop
- **Posture** – Stand, Tuck, Home
- **Info** – Battery, current, loop time, IMU, max servo temp, versions
- **Safety** – view of firmware safety state (OK/LOCKOUT),
  cause/override masks, clearance, soft limits, collision flag, and
  temperature lockout threshold, plus actions that send `SAFETY CLEAR`
  and `SAFETY OVERRIDE <ALL|TEMP|COLLISION|NONE>` commands to the Teensy.
- **System** – theme, palette, brightness, auto-disable, verbosity, mirror,
  and average servo temperature metric.
- **PID** – firmware PID tuning and mode controls; values display live via `PID LIST` and edits persist in `controller.ini` under `[pid]`.
- **IMP** – firmware impedance tuning (joint/cart, deadbands, scaling); values display live via `IMP LIST` and edits persist in `controller.ini` under `[imp]`.
- **EST** – estimator tuning (alpha parameters); values display live via `EST LIST` and edits persist in `controller.ini` under `[est]`.

Navigation (terminal keyboard):

- Open/close: `n`
- Move between items: Up/Down arrows (or `k`/`j`)
- Change tab: Left/Right arrows (or `h`/`l`)
- Adjust value/option: `A`/`D` keys
- Select/activate: Enter or Space
- Exit menu: `Esc`

Navigation (OpenCV mirror window):

- With window focused and menu visible:
  - `W`/`S` – move up/down
  - `A`/`D` – adjust value/option
  - Space/Enter – select
  - `Esc` – close menu

---

## 5. Theory of Operation (Firmware)

This section summarizes how the Teensy firmware runs the robot at a control
loop of 166 Hz and enforces safety.

### 5.1 Control Loop Structure

Each tick (about 6.024 ms), the firmware performs:

1. **Input targets** – Reads foot targets (body-frame x/y/z per leg) from
   commands such as `FOOT`/`FEET` or from internal test mode.
2. **Inverse kinematics (IK)** – Converts foot positions to joint angles
   (coxa, femur, tibia) in centidegrees.
3. **Command preparation** – Applies rate limits and soft joint limits, then
   converts to servo ticks for the lx16a bus.
4. **Send commands** – Issues MOVE_TIME frames for all 18 servos (batched per leg).
5. **Sparse feedback** – Reads one servo’s position/voltage/temperature using
   a round‑robin index, updating estimated states for the rest.
6. **Safety checks** – Evaluates over‑temperature, collision, OOS feedback,
   and workspace bounds; may trigger lockout and torque‑off.
7. **Optional logging** – If enabled, appends a compact CSV row to the SD log.

Loop timing is enforced via a hardware timer; when work would exceed the
budget, noncritical tasks (e.g., logging) are skipped to preserve 166 Hz.

### 5.2 Feedback & Estimation

- Feedback is **sparse** by design: one servo is read each tick; the estimator
  blends the last command with the latest measurement.
- Voltage and temperature readings are staggered so not every visit reads
  both; OOS logic treats any valid vin OR temp as a healthy sample.
- Estimated joint states are used for safety checks and (optionally) PID.

### 5.3 Safety & Lockout

- **Soft limits**: Joint commands are clamped based on `joint_limits.*` from
  `/config.txt`.
- **Collision model**: Simple foot‑to‑foot keep‑out in X/Z plane plus optional
  foot workspace bounds; violations trigger `SAFETY LOCKOUT`.
- **Temperature**: Servos reporting ≥80°C cause immediate torque‑off and lockout
  until cleared via `SAFETY` commands.
- **OOS (Out‑Of‑Service)**: Consecutive feedback failures mark a servo OOS;
  STATUS prints the OOS bitmask.

In addition to STATUS and `SAFETY LIST`, the firmware streams a compact S5
telemetry segment each tick summarizing safety state and configuration
(`lockout`, cause/override masks, clearance, soft-limit and collision toggles,
and temperature threshold). The Python controller uses this to keep its Safety
tab and safety overlay in sync with the Teensy.

### 5.4 Configuration & Persistence

- `/config.txt` keys control loop rate, safety toggles, tuck parameters,
  test gait parameters, logging mode/rate, and more.
- Many commands (e.g., `LOG`, `TEST OVERLAP`, `TUCK SET`, `SAFETY`) both apply
  changes live and write them back to `/config.txt`.

---

## 6. Theory of Operation (Controller)

The Python controller orchestrates input devices, telemetry, display, and
gait generation, while delegating low‑level actuation/safety to the Teensy.

### 6.1 Main Loop Phases

The main loop is split into small functions (see `phase_*` helpers):

1. `phase_timing_update` – tracks loop timing and target period.
2. `phase_teensy_connection` – handles connect/reconnect and telemetry grace.
3. `phase_gamepad_connection` – connects to the Xbox controller.
4. `phase_display_update` – asks `DisplayThread` and MARS menu to redraw if needed.
5. `phase_touch_input` – processes LCD touch for menu navigation and E‑STOP.
6. `phase_keyboard_input` – polls curses keyboard and routes keys (gaits, menu).
7. `phase_gait_tick` – steps the Python gait engine and emits `FEET` commands.
8. `phase_auto_disable` – applies auto‑disable timers (e.g., after TUCK).
9. `phase_loop_sleep` – sleeps to hit the configured loop rate.

### 6.2 Telemetry & Status Overlays

- `readTeensy` accumulates S1/S2/S3 telemetry lines; parsing is guarded
  against schema mismatches.
- S1 provides loop time, battery, current, IMU angles, robot enable, mode,
  and safety flags; S2/S3 carry per‑servo voltages and temperatures; S5
  carries detailed safety state (lockout flag, cause/override masks,
  clearance, soft-limit and collision toggles, and temperature threshold).
- Status overlays on the LCD provide immediate feedback:
  - **NO TEENSY** – Teensy not connected.
  - **NO CONTROLLER** – Gamepad not detected.
  - **NO TELEMETRY** – Connected but no S1 data yet.
  - **DISABLED** – Robot is not enabled at the firmware level.

When the firmware reports a new safety lockout via S5, the controller enters a
hard safety mode:

- Any active Python gait is stopped, and the controller issues `LEG ALL DISABLE`
  followed by `DISABLE` to drop torque.
- All subsequent enable, gait, and posture commands (keyboard or gamepad) are
  blocked until the lockout is cleared on the Teensy.
- The normal eye display is replaced with a full-screen safety-yellow overlay
  showing a "SAFETY LOCKOUT" message and a brief cause summary (e.g., TEMP,
  COLLISION).

### 6.3 Gait Engine

- The gait engine (`gait_engine.py`) produces Cartesian foot trajectories
  for Tripod, Wave, Ripple, and Stationary patterns.
- Inputs: left/right sticks and triggers, plus menu parameters (step height,
  step length, cycle time, width, lift).
- Outputs: per‑leg foot targets (`FEET` command) at a rate derived from S1 or
  a fallback frequency.
- Smoothing (EMA) is applied to heading/speed/turn commands to avoid sudden
  jumps; the engine also supports phase‑locked transitions between gait types.

### 6.4 UI & MARS Menu

- The LCD display is driven by a background `DisplayThread` that renders eye
  graphics and menu overlays at ~15 Hz.
- `MarsMenu` manages tabs/items for Eyes, Gait, Posture, Info, and System,
  including LCARS‑style themes and persistent settings in `controller.ini`.

---

## 7. Safety Behavior

- **Enable gating**: Motion commands are ignored when disabled; `ensure_enabled()` sends
  `LEG ALL ENABLE` + `ENABLE` when needed.
- **Soft limits**: Firmware enforces configured joint limits and collision checks.
- **Auto-disable**:
  - TUCK uses a reason-aware auto-disable timer; DISABLE is only sent if the
    robot is still in the same tuck context when the timeout expires.
- **E‑STOP**:
  - Touchscreen E‑STOP: any touch during gait stops the gait, sends idle,
    `LEG ALL DISABLE`, and `DISABLE`.
  - Keyboard: `q` key performs the same sequence.

---

## 8. Quick Reference: Commands & Keys

### 8.1 Firmware Serial Commands (Host → Teensy)

- `HELP` – list commands
- `STATUS` – system status
- `ENABLE` / `DISABLE` – global motion enable/disable
- `LEG ALL ENABLE` / `LEG ALL DISABLE` – torque on/off all legs
- `STAND` / `HOME` / `TUCK` – posture control
- `MODE TEST` / `MODE IDLE` – firmware gait on/off
- `FOOT <LEG> x y z` – single‑leg Cartesian target (mm)
- `FEET <x1> <y1> <z1> ... <x6> <y6> <z6>` – all legs
- `TEST CYCLE|HEIGHT|BASEX|STEPLEN|LIFT|OVERLAP <val>` – test gait tuning
- `SAFETY LIST` / `SAFETY SOFTLIMITS` / `SAFETY COLLISION` / `SAFETY TEMPLOCK` / `SAFETY CLEARANCE <mm>`
- `LOG ENABLE|DISABLE|MODE|RATE|STATUS` – SD logging control

### 8.2 Controller Keyboard (Terminal)

- `n` – toggle MARS menu (when robot disabled & stationary)
- `Esc` – close menu; exit app if menu not visible
- `w` / `h` / `p` – legacy gait controls (walk/halt/pattern)
- `t` – firmware TEST gait
- `k` – TUCK posture (with auto‑disable)
- `s` – STAND posture
- `q` – idle + `LEG ALL DISABLE` + `DISABLE` (soft E‑STOP)

### 8.3 Mirror Window Keys (OpenCV)

When the OpenCV mirror window is visible and the MARS menu is open, these keys
control the on‑screen menu without using the terminal or gamepad:

- **Navigation**
  - `W` / `S` or Up / Down arrows – move selection up/down
  - `A` / `D` or Left / Right arrows – move between items/columns
- **Tabs**
  - `Tab` – next tab (same as RB)
  - `Shift+Tab` – previous tab (same as LB)
- **Adjust values**
  - `J` – decrement current value
  - `L` – increment current value
- **Select / close**
  - `Enter` / `Space` – activate/select current item
  - `Esc` or `Q` – close menu

### 8.4 Gamepad (High Level)

- `LB` – toggle Python gait
- `RB` – cycle gait type (Tripod/Wave/Ripple/Stationary)
- `Y` – TUCK posture
- `X` – toggle firmware MODE (TEST/IDLE)
- `A` – STAND
- `B` – HOME
- Left stick – speed (Y) and heading offset/strafe (X)
- Right stick – turn rate / additional strafe depending on config

### 8.5 MARS Menu Navigation

- **Open/Close**: `n` (keyboard), Start (gamepad), tap frame (touch)
- **Navigate items**: Up/Down arrows, `k`/`j`, DPAD up/down
- **Change tab**: Left/Right arrows, `h`/`l`, LB/RB, tap tab
- **Adjust value**: `A`/`D`, DPAD left/right, touch arrows
- **Select/activate**: Enter/Space, gamepad A, tap button
- **Exit**: `Esc`, gamepad B, tap X close button

---

## 9. Troubleshooting

- **No Teensy overlay**:
  - Check USB cable and serial port.
  - Controller will periodically rescan and reconnect.
- **No telemetry overlay**:
  - Ensure telemetry is enabled from firmware (`Y 1`).
- **Menu won’t open**:
  - Safety gate: menu only opens when robot is disabled and no gait is active.

---

## 10. Maintenance & Versioning Rules

- Whenever you change firmware behavior:
  - Update `FW_VERSION`/`FW_BUILD` in `MARS_Hexapod.ino`.
  - Add a new entry to `CHANGELOG.md`.
  - Update this manual’s **Revision** line and any relevant sections.
- Whenever you change controller behavior:
  - Update `CONTROLLER_VERSION`/`CONTROLLER_BUILD` in `controller.py`.
  - Add a new entry to `CHANGELOG.md`.
  - Update this manual’s **Revision** line and any relevant sections.

Keeping this manual synchronized with code and changelog is part of the
acceptance criteria for any feature or bugfix.

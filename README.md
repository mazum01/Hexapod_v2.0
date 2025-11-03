# MARS — Modular Autonomous Robotic System (Hexapod v2.0)

Firmware for a Teensy 4.1 controlling 18x Hiwonder HTS‑35S serial bus servos via the lx16a‑servo library. Designed for deterministic 166 Hz control with a simple tripod test gait and ASCII serial interface.

## Hardware & Toolchain
- MCU: Teensy 4.1
- Servos: Hiwonder HTS‑35S (serial bus), 6 legs × 3 servos
- Buses: One serial bus per leg (6 total), half‑duplex with 74HC126 OE control
- Storage: SD card (BUILTIN_SDCARD) for `/config.txt` and logs
- Build: Arduino IDE/CLI 1.8.19 + Teensyduino (tested on Raspberry Pi 5)

## Key features
- Fixed‑rate control loop (target 166 Hz) with round‑robin feedback (1 servo/tick)
- IK path: foot targets → joint angles → servo commands
- Safety: soft joint limits, temperature lockout, DISABLE/LOCKOUT modes
- Test mode: tripod gait with tunables (cycle, base X/Y, step length, lift, overlap)
- ASCII serial protocol with `OK` / `ERR` replies; startup splash and STATUS
- SD config: `loop_hz`, `servo_id`, `home_cd`, `joint_limits`, `rate_limit`, and `test.trigait.overlap_pct`

## Repo layout
- `firmware/MARS_Hexapod/`
  - `MARS_Hexapod.ino` — setup/loop and core tick logic
  - `functions.ino` — parsing/printing, IK, config helpers
  - `robot_config.h` — geometry and pin mapping
  - `LoopTimer.hpp` — simple polling loop timer
- `docs/` — project spec, style, and TODO
- `tools/` — small utilities (e.g., formatter)

## Build & upload
1. Install Arduino 1.8.19 + Teensyduino and required board support.
2. Open `firmware/MARS_Hexapod/MARS_Hexapod.ino`.
3. Select board: Teensy 4.1, appropriate USB type and clock.
4. Compile and upload.

Optional: Arduino CLI/Teensy CLI can be used in CI later.

## Serial commands (subset)
- `ENABLE`, `DISABLE`, `MODE TEST`, `MODE IDLE`, `I`, `T`
- `STAND` — IK to neutral at (BASE_X, BASE_Y, Z=0)
- `FOOT <LEG> <x> <y> <z>`; `FEET <x1> <y1> <z1> ... <x6> <y6> <z6>`
- `TEST CYCLE|HEIGHT|BASEX|STEPLEN|LIFT|OVERLAP <val>`
- `HOME`, `SAVEHOME` (persist home_cd to `/config.txt` for enabled, in‑service servos)
- `STATUS`, `HELP`

See `docs/PROJECT_SPEC.md` for the full protocol and behavior.

## Configuration: `/config.txt`
- `loop_hz=166`
- `servo_id.<LEG>.<coxa|femur|tibia>=<1..253>`
- `home_cd.<LEG>.<coxa|femur|tibia>=<0..24000>`
- `joint_limits.<LEG>.<joint>.min_deg|max_deg=<deg around home>`
- `rate_limit.deg_per_s=<float>`
- `test.trigait.overlap_pct=<0..25>`

## Versioning
- Firmware version is printed in splash and STATUS. Current: see `FW_VERSION` in `MARS_Hexapod.ino` and tags (e.g., `v0.1.58`).

## License
This project is licensed under the MIT License; see `LICENSE`.

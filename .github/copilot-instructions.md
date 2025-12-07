# AI agent instructions for Hexapod v2.0 (MARS — Modular Autonomous Robotic System)

These rules help AI coding agents work productively in this repo. Keep edits small, deterministic, and aligned with the Teensy 4.1 + Arduino 1.8.19 constraints.

---

## ⚠️ CRITICAL: Version & Changelog Updates (DO NOT SKIP)

**EVERY code change MUST include these updates before task completion:**

1. **Bump version** in source file:
   - Firmware: `FW_VERSION` in `.ino`
   - Python controller: `CONTROLLER_VERSION` in `controller.py`
2. **Increment build number**: `CONTROLLER_BUILD` / firmware build
3. **Add inline changelog entry** (top of `.ino` or `controller.py`): date, version, summary
4. **Add entry to `CHANGELOG.md`** at repo root

### Pre-commit checklist (verify before completing ANY task)
- [ ] Version bumped in source file
- [ ] Build number incremented
- [ ] Inline changelog updated (date, version, one-line summary)
- [ ] `CHANGELOG.md` updated with full description
- [ ] `docs/TODO.md` updated if task was listed there

**Do NOT mark a task complete without these updates.**

---

## Big picture
- Target: Teensy 4.1 controlling 18x Hiwonder HTS‑35S serial bus servos via the lx16a‑servo library.
- Buses: one serial bus per leg (6 total), 3 servos each.
- Control loop: fixed 166 Hz. Each tick: send commands to all servos; read feedback from exactly one servo (round‑robin). Maintain estimated states for others.
- Inputs: foot targets in body frame (mm) → IK → joint angles (deg) → servo commands.
- Storage: SD card for config (`/config.txt`) and engineering logs.
- Interfaces: USB serial (ASCII) for enable/disable, stand, and foot/feet position commands; test mode provides a tripod gait.

Reference: see `docs/PROJECT_SPEC.md` for architecture, safety, timing, and acceptance criteria.

## Repo and code organization
- Prefer a small codebase to fit memory/flash: keep the main `.ino` minimal (setup/loop and core tick logic). Move non-core helpers (printing, serial parsing, utilities) to a sibling `functions.ino` for readability. Avoid deep abstractions.
- Avoid dynamic allocation and heavy STL; use fixed‑size buffers and stack/`static` storage. No exceptions/RTTI.
- Keep ISR/timer code minimal. Enforce the 166 Hz budget; non-blocking IO where possible.
- Place change history in two places: (1) top header block of the main `.ino` (date, author, summary), (2) `CHANGELOG.md` at repo root.

## Build and dev workflow
- Toolchain: Arduino IDE/CLI 1.8.19 + Teensyduino on a Raspberry Pi 5 (8GB).
- Keep compile units minimal; avoid pulling in non‑used libs. Prefer `#define` feature flags over adding dependencies.
- If adding a library, justify size and confirm it compiles for Teensy 4.1.
- Versioning discipline: after completing any todo item, increment the firmware version (FW_VERSION) — use a SemVer patch bump unless otherwise justified — and ensure it’s reflected in the splash/STATUS outputs. Update both the main .ino header changelog and `CHANGELOG.md` in the same commit.

## IO and timing patterns
- Loop schedule (each 6.024 ms tick):
  1) Compute joint angles from foot targets (IK function will be pasted in later; keep interface stable).
  2) Send position commands to ALL servos via lx16a‑servo.
  3) Read ONE servo’s position, voltage, temperature (round‑robin index advances each tick).
  4) Update simple estimator for others (hold‑last or linear prediction only).
  5) Run safety checks (enable gating, soft limits, collision lockout).
  6) Optional SD logging (buffered, rate‑limited).
- Maintain strict timing. If over budget, skip noncritical work (e.g., defer logging).

Startup splash
- On boot, print a short, non-blocking banner over USB Serial with: `MARS — Modular Autonomous Robotic System`, build time, configured loop_hz, UART mapping summary, and config/logging status. Keep it brief to avoid impacting the first 166 Hz tick.

## Configuration: SD `/config.txt`
- Parse `key=value` per line. Ignore blanks; `#` starts comments. Trim whitespace. Provide sane defaults when missing.
- Common keys (examples; add only what is used in code):
  - `loop_hz=166`
  - `uart.LF=Serial1` … per‑leg UART mapping
  - `joint_limits.LF.coxa.min_deg=-45` / `.max_deg=45`
  - `safety.soft_limits=true`
  - `safety.clearance_mm=10`
  - `logging.enabled=true` / `logging.rate_hz=166`
  - `test.trigait.enabled=false`
- Load once at boot in Phase 1. Log invalid keys; don’t crash.

## Serial command protocol (ASCII)
- Commands (one per line); respond with `OK` or `ERR <code> <msg>`:
  - `ENABLE`, `DISABLE`
  - `STAND`
  - `FOOT <LEG> <x_mm> <y_mm> <z_mm>` (body frame)
  - `FEET <x1> <y1> <z1> … <x6> <y6> <z6>` in order LF,LM,LR,RF,RM,RR
  - `MODE TEST` / `MODE IDLE`
- Enforce safety: ignore motion commands when disabled; clamp to soft limits; reject unreachable IK.

## Safety rules
- Soft joint limits from config; clamp before sending to servos.
- Leg/body collision check against a simple geometry/keep‑out model; on violation → lockout and require re‑enable.
- Rate limits on commanded angle deltas to avoid jumps.
- E‑stop: `DISABLE` immediately stops motion; log event.

## Logging (SD)
- CSV preferred. Filename pattern `logs/YYYYMMDD_hhmmss.csv` (or similar). Buffer writes and flush periodically.
- Start with a compact schema (example): `time_ms,leg,joint,cmd_deg,est_deg,meas_deg,voltage_V,temp_C,errors`.
- Make rate configurable; default 166 Hz; allow downsample.

## Tripod test mode
- Provide a simple tripod gait (tri‑gait) for bring‑up. Parameters via config: step height/length, cycle time, duty factor.
- Keep implementation minimal and deterministic; no IMU/body stabilization in Phase 1.

## Phase 2 preview (do not implement unless asked)
- Joint PID around measured/estimated angles; integrate with sparse feedback.
- Virtual spring‑damper (Cartesian/joint impedance) mapped to position targets.

## Editing guidance for AI agents
- Minimize footprint: prefer `constexpr`, `inline`, and small utility functions over new modules. Avoid templates unless zero‑overhead and tiny.
- No heap unless justified; preallocate fixed buffers. Avoid `String`; prefer `char[]` + `snprintf`.
- Guard all new work with acceptance criteria from the spec. Add tiny, fast, device‑level tests where possible.
- When touching the main loop, show the revised tick budget comments and confirm 166 Hz viability in comments.
- Always update: (1) main `.ino` header change log, (2) `CHANGELOG.md`, and (3) `docs/PROJECT_SPEC.md` if the behavior/contract changes.
 - After finishing a todo, bump `FW_VERSION` (SemVer patch by default) and verify splash/STATUS print the new version.

## TODO management directive
- Maintain a persistent project TODO list in `docs/TODO.md`.
  - Treat `docs/TODO.md` as the single source of truth for open tasks; keep it concise and actionable.
  - When adding, changing, or completing tasks, update `docs/TODO.md` in the same commit as code changes.
  - Mirror high‑level plan items in `docs/PROJECT_SPEC.md` when relevant; record completed work in `CHANGELOG.md` with a firmware version bump.
- During sessions, actively manage an in‑memory todo tracker and keep it synchronized with `docs/TODO.md`:
  - Before starting multi‑step work, write or refresh the todo items.
  - Mark exactly one item in‑progress at a time; mark it completed immediately when done.
  - Prefer small, incremental commits mapping to individual todo items.
- Acceptance for a “completed” todo:
  - Code compiles for Teensy 4.1; minimal runtime validation when feasible.
  - FW_VERSION bumped and reflected in splash/STATUS; change log updated.
  - Any new flags/config keys are documented and default‑safe.

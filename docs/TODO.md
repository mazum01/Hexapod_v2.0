# Project TODOs (persistent)

Last updated: 2026-02-02
Owner: Hexapod v2.0 (Firmware + Python controller)

Conventions
- [ ] not-started | [~] in-progress | [x] completed
- Keep items short and actionable; mirror high-level plan in docs/PROJECT_SPEC.md when relevant.
- Copilot will update this file when TODOs change, and reference it in commits.
- Reminder: On any behavior change or TODO completion, also bump FW/Controller versions, update in-file change logs, update CHANGELOG.md, and keep docs/USER_MANUAL.md in sync with the new versions.

---

## Modularization & Refactoring — Phase 2 (Jan 2026)

Goal: Reduce `controller.py` complexity (currently ~7.5k lines), eliminate module-level globals, and improve state management before Phase 2 Autonomy.

### M1. Entry Point Separation ✅ COMPLETE
- [x] Create `main.py` as the new entry point.
- [x] Update `joy_controller.py` to launch `main.py` instead of `controller.py` (update `CONTROLLER_SCRIPT` and process killing logic).
- [x] Move module-level initialization code — deferred. `main.py` wraps `controller.py` via import; full separation would require wrapping 7k lines in `main()` with low practical benefit.
- [x] Protect `main.py` execution with `if __name__ == "__main__":`. Done in main.py; controller runs on import by design (legacy wrapper pattern).
- [x] Ensure `controller.py` contains only `class Controller` — deferred. Current structure works; M4 will migrate globals into Controller naturally.

### M2. Configuration Refactor ✅ COMPLETE
- [x] Extend `config_manager.py` to handle all INI loading logic (was ~400 lines in `controller.py`).
- [x] Create `RobotConfig` class to hold all config values. (Done: `ControllerConfig` master dataclass with 22 sub-configs)
- [x] Add `_unpack_config_to_globals()` helper function to bridge typed config to legacy globals. (Done: v0.11.3 b275)
- [x] Remove redundant ConfigParser parsing (~350 lines) from controller.py. (Done: v0.11.3 b275, file reduced to 7483 lines)
- [x] Update `Controller.__init__` to accept a `RobotConfig` instance — deferred. Globals bridge works; M4 will clean this up as state migrates into Controller.

### M3. Menu Logic Extraction ✓ Complete (v0.11.6 b278)
- [x] Create `menu_controller.py`. (Done: v0.11.4 b276)
- [x] Move `_setup_mars_menu()` callback functions out of `controller.py` — phased extraction:
  - [x] M3.1: EYES callbacks (v0.11.4 b276)
  - [x] M3.2: SYSTEM callbacks (v0.11.5 b277)
  - [x] M3.3: PID/IMP/EST callbacks (v0.11.6 b278)
  - [x] M3.4: IMU/Leveling callbacks (v0.11.6 b278)
  - [x] M3.5: ToF callbacks (v0.11.6 b278)
  - [x] M3.6: GAIT callbacks (v0.11.6 b278)
  - [x] M3.7: POSTURE + Pounce callbacks (v0.11.6 b278)
  - [x] M3.8: AUTONOMY callbacks (v0.11.6 b278)
  - [x] M3.9: SAFETY callbacks (v0.11.6 b278)
  - [x] M3.10: Initial value sync + final wiring (v0.11.6 b278)
- [x] Refactor callbacks to use `Controller` instance methods (Done implicitly via M4 state migration).

### M4. State Encapsulation [~] In Progress
- [x] Phase 1: Fix `_setup_mars_menu` deferred binding (pass `ctrl`). (Done: v0.11.7 b279)
- [x] Phase 2: Refactor `phase_keyboard_input` to use `ctrl` for `verbose`/`mirror`/`forceDisplayUpdate`. (Done: v0.11.7 b279)
- [x] Phase 3: Update `sync_globals_to_ctrl` to stop overwriting authoritative `ctrl` state. (Done: v0.11.7 b279)
- [x] Phase 4: Gait state migration (gait engine, feet tracking, active flags). (Done: v0.11.8 b280)
- [x] Phase 5: Safety, Power, and Leveling state migration. (Done: v0.11.9)
- [x] Phase 6: PID, Impedance, and Estimator state migration. (Done: v0.11.10)
- [x] Phase 7: Autonomy state migration.
- [x] Phase 8: PointCloud and Dashboard state migration.
- [x] Final: Remove all `global` statements from `controller.py`.
- [x] Remove `sync_globals_to_ctrl` entirely (once all incoming globals are migrated).

### M5. Code Hygiene ✅ COMPLETE
- [x] Audit and fix bare `try...except: pass` blocks (add logging). (Done: v0.11.14)
- [x] Standardize variable naming (remove Hungarian/global style underscores where inappropriate). (Done: v0.11.14 - Fixed `_prevJoyState`, `_lastMode`, `_backGroundImage`, etc.)

---

## Control Loop Refactor — Three-Layer Architecture ✅ COMPLETE

Goal: Restructure main loop into biologically/robotically principled layers with appropriate timing.

### Research Basis
- **Three-Layer Architecture** (Firby/Gat): Controller → Sequencer → Deliberator
- **Subsumption Architecture** (Brooks): Parallel behaviors with suppression/inhibition
- **Biological Motor Control**: Spinal reflexes (fast) → CPG (rhythm) → Cortex (planning)

### Implemented Architecture (v0.8.13)
```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 3: DELIBERATOR  (~1 Hz, every 166 ticks)             │
│  • Placeholder for future SLAM/planning                     │
│  • run_layer3_deliberator()                                 │
└─────────────────────────────────────────────────────────────┘
                            ↓ goals/behaviors
┌─────────────────────────────────────────────────────────────┐
│  LAYER 2: SEQUENCER  (~33 Hz, every 5 ticks)                │
│  • Gamepad connection/polling                               │
│  • Touch input, keyboard input                              │
│  • Autonomy behaviors (obstacle/cliff/patrol)               │
│  • Display update                                           │
│  • run_layer2_sequencer()                                   │
└─────────────────────────────────────────────────────────────┘
                            ↓ motor commands
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: CONTROLLER  (166 Hz, every tick)                  │
│  • Timing update, Teensy I/O, housekeeping                  │
│  • Gait tick → FEET commands                                │
│  • Auto-disable safety, point cloud push                    │
│  • run_layer1_controller()                                  │
└─────────────────────────────────────────────────────────────┘
```

### Completed Tasks

#### R1. Layer 1 — Controller (166 Hz)
- [x] run_layer1_controller() wraps: timing, teensy, housekeeping, gait tick, auto-disable, pointcloud

#### R2. Layer 2 — Sequencer (~33 Hz)
- [x] Added _layer2TickCount, _layer2Divisor = 5 (configurable via gait.layer2_divisor)
- [x] run_layer2_sequencer() wraps: gamepad connection/poll, touch, keyboard, autonomy, display

#### R3. Layer 3 — Deliberator (~1 Hz)
- [x] Added _layer3TickCount, _layer3Divisor = 166
- [x] run_layer3_deliberator() placeholder for future SLAM/planning

#### R4. Integration & Testing
- [x] Main loop updated with layered structure
- [x] Flowchart generator updated (controller_flowcharts.py)
- [x] Controller version bumped to v0.8.13
- [x] CHANGELOG.md updated

---

## Autonomy — Phase 1: Reactive Behaviors (No New Hardware)

Goal: Add behavior arbitration layer using existing ToF and IMU sensors for basic autonomous operation.

### Architecture
```
Sensors → Behaviors → Arbitration → Gait Commands
   ToF  →  Avoid     →            → Turn/Stop
   IMU  →  Level     →  Priority  → Adjust stance
  Touch →  E-Stop    →            → Disable
```

### Tasks

#### A1. Behavior Engine Framework
- [x] Create `behavior_engine.py` module — **DONE v0.7.39**
  - Behavior base class with: name, priority, enabled flag, compute() → (active, action)
  - BehaviorArbiter: runs all behaviors, selects highest-priority active one
  - Action types: STOP, TURN_LEFT, TURN_RIGHT, SLOW_DOWN, CONTINUE, BACK_UP, EMERGENCY_STOP
- [x] Integrate with main loop (between sensor reads and gait tick) — **DONE v0.7.40**
  - phase_autonomy() runs arbiter and applies actions to gait
  - Keyboard 'a' toggles autonomy mode
- [x] Add [behavior] config section for enable/disable toggles — **DONE v0.7.39**

#### A2. Obstacle Avoidance Behavior
- [x] Parse ToF 8×8 grid into danger zones (front-left, front-center, front-right) — **DONE v0.7.39**
- [x] Configurable thresholds: stop_distance_mm (default 150), slow_distance_mm (default 300) — **DONE v0.7.39**
- [x] Actions: STOP if obstacle < stop_distance; SLOW_DOWN if < slow_distance; TURN away from obstacle side — **DONE v0.7.39**
- [ ] Test with static obstacles

#### A3. Cliff Detection Behavior
- [x] Use lower rows of ToF sensor (bottom zones) — **DONE v0.7.39**
- [x] Detect sudden distance increase (floor dropout) → cliff — **DONE v0.7.39**
- [x] Action: STOP immediately — **DONE v0.7.39**
- [x] Configurable cliff_threshold_mm (default 100 = 10cm drop) — **DONE v0.7.39**

#### A4. Wall Following Behavior
- [x] Track ToF edge readings (left or right side zones) — **DONE v0.10.14**
- [x] Maintain configurable wall_distance_mm (default 200) — **DONE v0.10.14**
- [x] PD controller to steer parallel to wall — **DONE v0.10.14**
- [x] Enable via menu or command — **DONE v0.10.15**

#### A5. Patrol Mode
- [x] Simple timed forward walk with random turns — **DONE v0.7.39**
- [x] Configurable patrol_duration_s, turn_interval_s — **DONE v0.7.39**
- [x] Respects obstacle avoidance (higher priority) — **DONE v0.7.39**
- [x] Stop on touch screen tap (E-stop already implemented) — **DONE v0.10.14**

#### A6. Behavior Menu Tab
- [x] Add AUTONOMY tab to MarsMenu — **DONE v0.7.41**
- [x] Items: Enable/Disable each behavior, thresholds, patrol settings — **DONE v0.7.41**
- [x] Status: Active behavior name, last action taken — **DONE v0.7.41**
- [x] Save Settings action with save_behavior_settings() — **DONE v0.7.41**
- [x] Controller toggle: Back+A gamepad combo — **DONE v0.7.42**

#### A7. Caught Foot Detection & Recovery
- [x] Detect foot snag during swing phase using: — **DONE v0.7.39**
  - Position error: commanded vs actual divergence > threshold
  - Swing timeout: foot doesn't reach target within expected time
- [ ] Additional detection (future): servo load/current spike, IMU anomaly
- [x] Recovery actions implemented: — **DONE v0.7.39**
  - Lift caught leg higher (increase step height temporarily)
  - Back up one step if lift fails
  - E-stop after N consecutive failures
- [x] Configurable: snag_position_error_deg, snag_timeout_ms, recovery_lift_mm — **DONE v0.7.39**
- [ ] Log snag events for analysis

---

## Autonomy — Phase 2: Deliberative Navigation (Moderate Effort)

Prerequisites: Phase 1 behaviors working reliably

### Tasks

#### A7. Waypoint Navigation
- [ ] Dead reckoning using IMU yaw integration + gait step counting
- [ ] Waypoint list (x, y) in body start frame
- [ ] Simple pursuit: turn toward waypoint, walk, stop when close
- [ ] Drift-limited (~1-2m useful range without correction)

#### A8. Simple SLAM (2D Occupancy Grid)
- [ ] Rotate in place to scan 360° with ToF
- [ ] Build 2D occupancy grid (configurable cell size, default 50mm)
- [ ] Update grid incrementally during patrol
- [ ] Display grid on engineering view or menu

#### A9. Return to Start
- [ ] Record start position on patrol begin
- [ ] Navigate back using dead reckoning + obstacle avoidance
- [ ] Useful for "go explore and come back" behavior

---

## Autonomy — Phase 3: Vision & AI (Requires Camera + Optional Accelerator)

### Hardware Options

| Device | Performance | Price | Notes |
|--------|-------------|-------|-------|
| Pi Camera 3 | N/A | ~\$25 | Required for all vision tasks |
| Coral USB TPU | 4 TOPS | ~\$60 | Easy USB, good ecosystem |
| Hailo-8L M.2 | 13 TOPS | ~\$70 | M.2 HAT needed, excellent perf |
| Hailo-8 M.2 | 26 TOPS | ~\$100 | Top-tier for Pi |

### Tasks (Camera Only, CPU-based ~5-10 FPS)

#### A10. Add Pi Camera Module
- [ ] Mount camera (front-facing, adjustable tilt)
- [ ] Create `camera_sensor.py` module with threaded capture
- [ ] Configurable resolution (default 640×480), frame rate (default 10 FPS)

#### A11. Color Blob Tracking
- [ ] OpenCV HSV color detection
- [ ] Track colored object (ball, marker)
- [ ] Behavior: turn toward blob, approach slowly

#### A12. ArUco Marker Localization
- [ ] Detect ArUco markers for known positions
- [ ] Correct dead reckoning drift when marker seen
- [ ] Place markers around room as waypoints

#### A13. Motion Detection (Sentry Mode)
- [ ] Frame differencing for motion detection
- [ ] Alert behavior: turn toward motion, track

### Tasks (With AI Accelerator, 30+ FPS)

#### A14. Real-time Object Detection
- [ ] MobileNet-SSD or YOLO-Nano on Coral/Hailo
- [ ] Detect: people, pets, furniture, obstacles
- [ ] Behavior: avoid people, follow person on command

#### A15. Person Following
- [ ] Track specific person (largest detection or re-ID)
- [ ] Maintain follow_distance_mm (default 1000)
- [ ] Stop when person stops, follow when they move

#### A16. Semantic Terrain Classification
- [ ] Segment floor vs obstacle vs wall
- [ ] Detect terrain type (carpet, tile, stairs)
- [ ] Behavior: avoid stairs, prefer smooth surfaces

#### A17. Gesture Recognition
- [ ] MediaPipe Hands for hand detection
- [ ] Simple gestures: wave (come here), stop (palm), point (go there)
- [ ] Map gestures to behaviors

#### A18. Voice Commands (Local)
- [ ] Whisper-tiny or similar on accelerator
- [ ] Commands: "come", "stop", "patrol", "follow me"
- [ ] Wake word or always-on with low-power mode

---

## Display & UX

### D1. Battery Status Display
- [x] Add phone-style battery icon to EYES display mode (v0.7.35 b219)
  - Position: top-right corner, semi-transparent
  - Show: battery percentage bar, voltage text
  - Color: green (>50%), yellow (20-50%), red (<20%)
  - Data source: S1 telemetry servo voltage (min of all servos)
  - Configurable: enable/disable via show_battery_icon in [safety_display]

### D2. Battery Voltage Calibration
- [ ] Learn actual battery full/empty voltages for accurate percentage display
  - Current defaults: volt_min=10.5V (empty), volt_max=12.5V (full)
  - For 3S LiPo: 9.0V cutoff, 12.6V full → update defaults or auto-learn
  - Option A: Manual config (document in USER_MANUAL)
  - Option B: Track min/max seen over session, suggest calibration values
  - Option C: Add "Calibrate Battery" menu item that records current voltage as "full"
- [ ] Show percentage instead of/alongside voltage on icon
- [ ] Add low battery audio warning (beep or TTS) → see Audio section

---

## Audio Feedback System

### Hardware
- **USB DAC**: Sabrent USB External Stereo Sound Adapter
- **Amplifier**: PAM8403 mini class-D module (5V from Pi GPIO)
- **Speakers**: 2x 28mm 1W mini speakers

### Wiring
```
Pi 5 USB → Sabrent DAC → 3.5mm → PAM8403 L/R inputs → Speakers
                                       ↑
                                 5V + GND from GPIO
```

### Tasks

#### AU1. Audio Module ✅
- [x] Create `audio_manager.py` module
  - pygame.mixer based for low-latency, non-blocking playback
  - Sound pool for preloaded common sounds
  - Volume control (0-100%)
  - Mute toggle
  - Beep tone generator (numpy-based)
- [x] Configurable via [audio] section in controller.ini
  - enabled=true, volume=0.7
  - device=hw:2,0 (Sabrent USB DAC)
  - sounds_dir=assets/sounds
- [x] Integrate into controller.py (startup, event hooks)
  - Audio manager initialization during startup
  - Startup chime (3-tone ascending A4→C#5→E5)
  - Shutdown beep (descending E5→A4)
  - Clean shutdown of audio manager

#### AU2. System Event Sounds ✅
- [x] Startup chime (on controller boot)
- [x] Shutdown sound (on graceful exit)
- [x] Low battery warning (3-beep alert when < threshold)
- [x] Safety lockout alert (urgent 3-tone warning)
- [x] Teensy connect/disconnect tones

#### AU3. Gait & Mode Sounds ✅
- [x] Gait start/stop chirps
- [x] Gait type change confirmation (different tone per gait) — **DONE v0.10.7**
- [x] Autonomy mode toggle sound (ascending/descending tones)
- [x] Stand/Tuck/Home confirmation sounds

#### AU4. Controller Feedback ✅
- [x] Xbox controller connect/disconnect (rising/falling tones)
- [x] Button press click (all ABXY/LB/RB buttons)
- [x] Menu navigation clicks (nav/tab/adjust/select) — **DONE v0.10.8**

#### AU5. Voice Announcements (TTS) ✅
- [x] Integrate pyttsx3 (espeak-ng backend) for text-to-speech
  - [x] "Mars online" (startup)
  - [x] "Mars shutting down" (shutdown)
  - [x] "Robot enabled" / "Robot disabled"
  - [x] "Standing" / "Tucking" (posture confirmations)
  - [x] "Safety lockout activated"
  - [x] "Autonomy mode on" / "Autonomy mode off"
  - [x] "Battery low, X percent remaining" (hooked to low battery event)
  - [x] "Obstacle detected" (hooked to autonomy STOP)
  - [x] "Cliff detected" (hooked to autonomy E-STOP)
- [x] Configurable: [tts] enabled, rate, volume, voice, cooldown_sec
- [x] Limit announcement frequency (cooldown_sec anti-spam)
- [x] Piper neural TTS integration (engine=piper|espeak) — **DONE v0.10.9**
- [x] Pre-cached TTS WAV files for instant playback (15 phrases, generate_tts_phrases.py) — **DONE v0.10.10**

#### AU6. Sound Assets ✅
- [x] Create/source royalty-free sound effects — **DONE v0.10.8**
  - Robotic beeps/chirps for events (generate_sounds.py)
  - Warning tones (escalating urgency)
  - Confirmation clicks
- [x] Store in `assets/sounds/` directory (22 WAV files generated)
- [x] Keep files small (16-bit, 44.1kHz mono)

---

## Safety — Limb Collision Avoidance

### Current State (Firmware)
The firmware has geometric keep-out zone collision detection that triggers STAND+DISABLE when a foot position violates predefined body/leg clearances. This is reactive (catches violations after IK) rather than preventive.

### S1. Analytical Pre-IK Collision Check ✅ COMPLETE (v0.12.1)
- [x] Implement swept-volume collision test before sending foot targets
  - Model each leg segment as capsule (cylinder + hemispheres)
  - For each leg pair that could collide (LF↔LM, LM↔LR, etc.), test capsule-capsule intersection
  - Run at gait planning time (before IK) to reject or clamp dangerous trajectories
- [x] Precompute collision risk zones per leg phase
  - Added `LegPhase` enum (SWING/STANCE) and tripod group definitions
  - `get_leg_phases_tripod(phase)` returns phase state for all 6 legs
  - `get_risk_pairs(phases)` returns only pairs where at least one leg swings
  - `validate_pose_safety()` now accepts optional `gait_phase` for targeted checking
  - `validate_pose_safety_detailed()` returns distances and diagnostic info
- [x] Add velocity-aware margin (faster motion → larger keep-out buffer)
  - `compute_velocity_margin(velocity_mm_s)` computes look-ahead buffer
  - `TIME_HORIZON_S = 0.05` (50ms look-ahead, ~8 ticks at 166Hz)
  - `MAX_VELOCITY_MARGIN_MM = 20.0` caps margin at high speeds
  - Threshold scales from 35mm (static) to 55mm (at 400 mm/s)
- [x] Config parity: Collision model parameters adjustable from both UIs
  - MarsMenu SAFETY exposes collision knobs (radius/margin/body keepout/velocity horizon/max margin)
  - Web dashboard exposes the same keys and persists them to controller.ini `[collision]`
- [x] **Bugfix (v0.12.5)**: Fixed `kinematics.fk_leg()` geometry — was computing wrong joint positions (ankle above knee for standing pose). Unified FK code in `kinematics.py` as single source of truth; `display_thread.py` now imports `fk_leg_points()` from it.
- [x] Diagnostics: Add pose compare logger (cmd FEET → IK/FK vs S6 telemetry joint → FK) to debug collision mismatch — **DONE v0.12.10**
- [x] Diagnostics: Move pose compare logger controls from env vars to controller.ini + menu/dashboard — **DONE v0.12.11**
- [x] Safety: Collision response now steps to stand (one-leg lift→translate→place) when `warn_only=false` and `stop_on_collision=true` — **DONE v0.12.13**

### S2. Learned Collision Model (AI-Enhanced) ✅ COMPLETE
- [x] Create `collision_model.py` module with training pipeline:
  - `generate_training_data()` — Sample random joint configs, run FK, check collision
  - `CollisionDataset` / `CollisionMLP` — PyTorch training infrastructure
  - `train_model()` / `export_to_onnx()` — Training and model export
  - `CollisionPredictor` — ONNX Runtime inference wrapper
- [x] Integration hooks in `collision.py`:
  - `validate_pose_safety_learned()` — Pure neural network check
  - `validate_pose_safety_hybrid()` — NN pre-filter + analytical confirmation
- [x] Generate training dataset (~100k samples)
  - 100k samples generated with accurate body geometry from STL
  - 1.87% leg collision rate, 0% body collision (legs can't physically reach body)
- [x] Train model and validate accuracy (target: >95% agreement with analytical)
  - Achieved: **99.5% agreement** with analytical check on 1000 test samples
  - Model: 3,461 parameters (18→64→32→5 architecture)
  - Validation accuracy: 99.74%
- [x] Benchmark inference latency (target: <100µs per check on Pi 5)
  - Achieved: **36.3µs single inference** (27.5k inferences/sec)
  - Batch (100): 0.06ms (1.69M samples/sec)
  - **14.5× faster than analytical check** (36µs vs 525µs)
- [x] Integration testing with live controller
  - **17.9× speedup** in hybrid mode (28µs vs 492µs per check)
  - Model correctly skips analytical check when confident (learned_max_prob ≈ 0)
  - Config option: `use_learned_model` in `[collision]` section (default: True)
  - Integration in `send_feet_cmd()` uses `validate_pose_safety_hybrid()` when enabled


### S3. Reinforcement Learning Gait Optimization (Advanced)
- [ ] Train RL policy that maximizes task reward while avoiding collisions
  - Reward: forward progress, stability, energy efficiency
  - Penalty: collision events, near-misses, jerky motion
  - Sim-to-real: train in PyBullet/MuJoCo, fine-tune on real robot
- [ ] Deploy policy on Coral/Hailo for real-time inference
  - Replaces or augments hand-tuned gait engine
  - Automatically learns safe trajectories for complex terrain

### S4. Visual Self-Monitoring (Camera-Based)
- [ ] Downward-facing camera watches leg motion
  - Detect unexpected leg positions or contact events
  - Useful for detecting entanglement, debris, mechanical failure
- [ ] Requires camera + AI accelerator for real-time pose estimation
  - Could use lightweight pose model (MediaPipe, MoveNet)

---

## Free Gait — Adaptive Event-Driven Locomotion

Goal: Replace fixed-phase cyclic gaits with an event-driven free gait that adapts leg timing to terrain and stability conditions. Enables terrain adaptation, obstacle negotiation, and robust recovery.

### Architecture Overview
```
┌─────────────────────────────────────────────────────────────┐
│  FreeGaitCoordinator                                        │
│  • Computes support polygon from stance feet                │
│  • Checks CoG stability margin                              │
│  • Grants swing permission to ready legs                    │
└─────────────────────────────────────────────────────────────┘
                            ↓ permission
┌─────────────────────────────────────────────────────────────┐
│  Per-Leg State Machines (×6)                                │
│  • STANCE → LIFT_PENDING → SWING → PLACING → STANCE         │
│  • Transitions driven by coordinator + contact events       │
└─────────────────────────────────────────────────────────────┘
                            ↓ foot targets
┌─────────────────────────────────────────────────────────────┐
│  Foot Placement Planner                                     │
│  • Computes target position based on heading/speed          │
│  • Pre-checks collision before committing                   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│  Existing: IK + Collision Check + Impedance                 │
└─────────────────────────────────────────────────────────────┘
```

### FG1. Per-Leg State Machine ✅ COMPLETE (v0.12.14)
- [x] Create `LegState` enum: `STANCE`, `LIFT_PENDING`, `SWING`, `PLACING`
- [x] Create `Leg` class with: state, foot_target, swing_progress, contact_sensed, contact_estimated
- [x] State transition logic:
  - `STANCE` → `LIFT_PENDING`: coordinator grants swing permission
  - `LIFT_PENDING` → `SWING`: lift trajectory complete
  - `SWING` → `PLACING`: horizontal motion complete, begin descent
  - `PLACING` → `STANCE`: contact detected or target Y reached
- [x] Track per-leg timing: time_in_state, last_transition_time

### FG2. Support Polygon Computation ✅ COMPLETE (v0.12.14)
- [x] Implement `compute_support_polygon(stance_feet)` → list of (x, z) vertices in body frame
- [x] Use existing FK to get current foot positions — `get_stance_feet_xz()` helper
- [x] Handle degenerate cases: < 3 stance feet returns None (unstable)
- [x] Added to new `free_gait.py` module (convex_hull_2d, point_in_polygon, distance_to_polygon_edge)

### FG3. Center of Gravity Estimation ✅ COMPLETE (v0.12.15)
- [x] Implement `estimate_cog()` → (x, z) projection on ground plane in body frame
- [x] Initial implementation: return body center (0, 0, 0) — sufficient for quasi-static walking
- [x] Optional enhancement: weighted average including leg positions (leg mass ~3% each)
- [x] Integrate with IMU pitch/roll to project CoG onto ground plane

### FG4. Stability Margin Computation ✅ COMPLETE (v0.12.16)
- [x] Implement `compute_stability_margin(cog_xz, support_polygon)` → float (mm)
- [x] Algorithm: distance from CoG projection to nearest polygon edge
- [x] Positive = stable (inside polygon), negative = unstable (outside)
- [x] Configurable minimum margin threshold (default: 30mm)
- [x] Added `StabilityStatus` enum (STABLE/MARGINAL/CRITICAL/UNSTABLE)
- [x] Added `classify_stability()` and `check_swing_safe()` helpers

### FG5. Free Gait Coordinator ✅ COMPLETE (v0.12.17)
- [x] Create `FreeGaitCoordinator` class
- [x] Each tick:
  1. Identify stance legs, compute support polygon
  2. Compute stability margin with current CoG
  3. Evaluate which legs are ready to swing (longest in stance, task-aligned)
  4. Grant permission if: margin > threshold after removing candidate leg
- [x] Configuration: max_simultaneous_swings (1-3), min_stability_margin_mm
- [x] Emergency hold: if margin drops below critical, cancel all pending lifts
- [x] Leg selection heuristics: alternating sides, direction-aligned, longest-waiting
- [x] Added `SwingPriority` enum and `CoordinatorConfig` dataclass

### FG6. Foot Placement Planner ✅ COMPLETE (v0.12.19)
- [x] Implement `plan_foot_placement(leg_idx, heading_deg, speed_scale)` → (x, y, z) target
- [x] Default positions: neutral standing pose (like StandingGait)
- [x] Walking offset: stride based on heading and speed
- [x] Turn adjustment: differential strides for inside/outside legs
- [x] Pre-check candidate against collision model before committing
- [x] Added `plan_swing_trajectory()` for 3-phase lift/translate/place
- [x] Added `middle_leg_offset_z_mm` config (Mostafa et al. 2012 "offset model") for improved stability margin
- [ ] Future: terrain-aware placement using contact height history or ToF sensing

### FG7. Contact Estimation (Pre-Switch Fallback) ✅ COMPLETE (v0.12.20)
- [x] Implement `estimate_contact(leg_idx)` → bool
- [x] Primary: use S4 foot switch telemetry when available
- [x] Fallback: position-based (foot reached target Y within tolerance)
- [x] Fallback: time-based (swing duration exceeded expected time)
- [x] Flag contact source: `SENSED` vs `ESTIMATED` for diagnostics

### FG8. FreeGait Engine Class ✅ COMPLETE (v0.12.21)
- [x] Create `FreeGait(GaitEngine)` in `gait_engine.py`
- [x] Owns: 6 `Leg` instances, `FreeGaitCoordinator`, foot placement planner
- [x] `tick(dt)`: update coordinator, advance leg state machines, compute foot targets
- [x] `get_feet_bytes()`: format current foot positions as FEET command (inherited)
- [x] `start()` / `stop()`: initialize legs to stance, clean shutdown
- [x] Integrate with existing `GaitTransition` system for smooth switching (FG9)

### FG9. Integration & Tuning ✅ COMPLETE (v0.12.22)
- [x] Add `FreeGait` to gait cycle (LB/RB toggle includes it)
- [x] Configuration in `[gait]` section:
  - `free_gait_min_margin_mm` (default: 30)
  - `free_gait_max_swings` (default: 3)
  - `free_gait_swing_speed_mm_s` (default: 200)
- [x] Menu/dashboard exposure for key tuning parameters (FG Margin, FG Max Swing, FG Speed)
- [ ] Stability fallback: if margin critical, auto-transition to StandingGait (deferred to FG11)
- [ ] Logging: stability margin, active swings, state transitions per tick (deferred)

### FG9b. FreeGait Walking Fixes ✅ COMPLETE (v0.12.27)
- [x] **Body-frame coordinates** (v0.12.25): Support polygon now computed in body frame using `_foot_to_body_frame()`. Fixed degenerate polygon (all Z=0) causing infinite negative margin.
- [x] **Swing targets** (v0.12.25): Targets set when leg enters LIFT_PENDING, not just for STANCE.
- [x] **Stance translation** (v0.12.26): Stance legs push backward proportional to speed/heading (matching TripodGait). Enables continuous walking cycle.
- [x] **IMU integration** (v0.12.26): Controller passes pitch/roll to FreeGait for CoG projection stability calculations.
- [x] **Strafe/turn verified**: heading=90° moves X axis; turn_rate adjusts leg stride differentially.
- [x] **Gait transition** (v0.12.27): FreeGait/StandingGait now transition immediately (no phase wait).
- [x] **Lift height alignment** (v0.12.27): Added lift_attenuation=0.69 to match TripodGait effective lift (41.4mm).

### FG10. Foot Switch Integration (Firmware + Telemetry)
- [ ] **Firmware**: Wire 6 foot contact switches to Teensy digital inputs
- [ ] **Firmware**: Implement S4 telemetry frame (already stubbed):
  - Format: 6 contact states packed as single byte (1 bit per leg)
  - Debounce logic: 2-3ms to reject mechanical noise
- [ ] **Controller**: Parse S4 frame in telemetry handler
- [ ] **Controller**: Populate `Leg.contact_sensed` from S4 data
- [ ] **Controller**: Contact events trigger state transitions:
  - Early contact (during PLACING): stop descent, transition to STANCE
  - Late/no contact: extend further or trigger recovery
  - Unexpected loss (during STANCE): slip detection → hold or recovery

---

## Web Dashboard — Remote Configuration & Monitoring

### W1. Phase 1: Telemetry Dashboard ✅ COMPLETE (v0.8.4)
- [x] TelemetryServer module for WebSocket streaming (telemetry_server.py)
- [x] Dashboard HTML/JS with real-time telemetry display (dashboard.html)
  - Power: battery voltage, current, low-battery status
  - IMU: pitch/roll/yaw with visual gauges
  - Status: Xbox/Teensy connection, gait state, safety
  - Servos: max temp, min voltage, error count
  - System: loop time, firmware/controller versions
- [x] Read-only configuration view (gait, safety settings)
- [x] Integration with Layer 2 (phase_dashboard at ~30 Hz)
- [x] Configurable via [dashboard] in controller.ini
  - enabled=true, port=8766, stream_hz=10

### W2. Phase 2: Enhanced Config View ✅ COMPLETE (v0.8.5)
- [x] Expand configuration sections displayed
  - All menu categories: EYES, GAIT, POSTURE, SAFETY, PID, IMP, EST, IMU, TOF, AUTO, INFO, SYSTEM
  - MarsMenu.get_all_config() exports all menu item values
- [x] Add configuration search/filter
- [x] Collapsible sections for better organization

### W2b. LCARS Styling Polish
- [x] Fix dashboard swept elbow curves (header/sidebar/footer)
  - Inner curve sizing and radius proportions per LCARS Manifesto
  - Reference: http://www.lcars-terminal.de/tutorial/guideline.htm
- [x] Apply same fixes to pointcloud_viewer.html swept elbows

### W3. Phase 3: Configuration Editing ✅ COMPLETE (v0.8.6)
- [x] Add set_config WebSocket command support
- [x] Implement config change callback in controller (_handle_dashboard_config_change)
- [x] Edit gait parameters (cycle_ms, step_height, step_length, turn_rate)
- [x] Edit safety thresholds (low_battery_enabled, volt_critical, volt_recovery)
- [x] Edit PID/IMP/EST tuning parameters
- [x] Save to controller.ini from dashboard (via existing save_*_settings functions)

### W4. Phase 4: Graphs & Historical Data ✅ COMPLETE (v0.8.7)
- [x] Rolling graph for battery voltage over time
- [x] Loop time chart (converted from histogram to line chart)
- [x] Servo temperature trends
- [x] Export telemetry data as CSV
- [x] Chart.js integration with LCARS styling
- [x] Tab-based chart switching (Voltage/Loop Time/Temperature)
- [x] Selectable time windows (1/5/10 min)
- [x] Sample counter badge

### W5. Phase 5: Enhanced Integration
- [ ] Link to point cloud viewer (already available at /pointcloud)
- [ ] Embed point cloud in dashboard (iframe or integrated Three.js)
- [ ] Add camera feed view (when camera module added)
- [x] Mobile-responsive layout

---

## Open Tasks (Firmware)

### Phase 2 Backlog

- [ ] Dual-rail power monitoring (V+I)
  - Add current + voltage sensing for both rails: electronics (5V/reg) and mechanicals (servo bus)
  - Recommended: 2x I2C high-side monitor (INA226/INA228-class) + appropriately-sized shunt(s)
  - Telemetry: extend/repurpose S1 fields to report both rails (e.g., elec_v/elec_a + mech_v/mech_a)
  - Safety hooks: brownout warning thresholds + overcurrent event logging; avoid impacting 166 Hz loop

- [ ] Foot 3D workspace bounds
  - Define body-frame foot workspace box (x/y/z mm) as sanity check on IK targets
  - Out-of-box feet trigger STAND+DISABLE collision path when safety.collision enabled

- [ ] FK/IK test harness
  - Add IKTEST / FKTEST serial commands for quick validation
  - Emit diff vs expected; optional stats accumulation

- [ ] Config live reload
  - Implement CONFIG RELOAD (safe mid-run parse) with timing guard
  - Only apply non-critical keys (limits, gait, safety, logging)

- [ ] Pin wiring documentation
  - Document SerialX TX/RX/enable pins & power wiring
  - Mirror in robot_config.h; splash sanity print

- [ ] Logging enhancements
  - Add logging.mode=profile (per-tick timing, rr index, overruns)
  - Jitter histogram snapshot command

- [ ] homeAngles calibration workflow
  - Add CALIBRATE <LEG|ALL> automation
  - Unify offset/home persistence strategy

- [ ] loop_hz strategy review
  - Re-assess adaptive up/down hysteresis
  - Consider fixed 166 Hz with overrun counter

- [ ] Memory/footprint audit
  - Generate RAM/flash map
  - Identify large symbols (float printf, lx16a unused)

---

## Open Tasks (Python Controller)

### General

- [ ] Align commands with firmware
  - Diff Python command usage vs current Teensy HELP output
  - Identify missing abstractions (STATUS polling, SAFETY toggles)
  - Deliverable: Command map + deprecation list + wrapper strategy

- [ ] Logging enhancements (controller)
  - Optional structured event log (enable, posture, errors) with timestamps
  - Simple logger writing CSV or JSON lines; toggle via config

- [ ] Configurable voltage/temp thresholds
  - Move color palette thresholds to config
  - Config keys + usage in display rendering

- [ ] Split display rendering
  - Break UpdateDisplay into pure sub-functions
  - Functions: render_status_text, render_servo_bars, apply_overlays, mirror_output

### Sensor Integration

- [ ] ToF stereo fusion
  - Merge left/right sensor data into unified obstacle representation
  - Polar occupancy grid (12 sectors × 10 distance bins)

- [ ] Reactive obstacle avoidance
  - Gait engine checks obstacle grid before each step
  - Stops or steers away when obstacle detected

- [ ] CoM shift compensation (Phase 2 enhancement)
  - Shift body XY to keep center of mass centered in support polygon on slopes
  - Improved stability margin on steeper slopes (>10°)

### Xbox Controller

- [x] startup.sh update — Removed; joy_controller now manages controller lifecycle.

### View Settings

- [ ] Persist 3D view angle in controller.ini
  - Save/restore engineering view azimuth/elevation
  - Low priority

---

## Completed Tasks

### Startup Fixes (2025-12-27)
- [x] Fix startup crash after ToF init (v0.7.33 b217)
- [x] Fix startup display race condition (v0.7.34 b218)

### 3D Wireframe Engineering View (2025-12-26)
- [x] Firmware: S6 Joint Position Telemetry (FW 0.2.45)
- [x] Python: S6 Parsing (v0.6.3 b172)
- [x] Python: FK forward kinematics (v0.7.13 b197)
- [x] Python: 3D Projection functions
- [x] Python: D-pad view control
- [x] Python: draw_hexapod_wireframe_3d()
- [x] Integration & testing complete

### Modularization (2025-12-18)
- [x] Phase 1: telemetry.py extracted (v0.5.41 b153)
- [x] Phase 2: config_manager.py extracted (v0.5.42 b154)
- [x] Phase 3: posture.py extracted (v0.5.43 b155)
- [x] Phase 4: mars_state.py created (v0.5.44 b156)
- [x] Phase 5: display_thread.py extracted (v0.5.45 b157)
- [x] Phase 6: input_handler.py extracted (v0.5.46 b158)
- [x] Phase 7: Final cleanup (v0.5.47 b159)
- [x] Unify ASCII/binary telemetry parser (v0.5.49 b161)

### Sensor Integration (2025-12-19 - 2025-12-22)
- [x] IMU driver module (v0.5.50 b162)
- [x] IMU data integration (v0.5.52 b164)
- [x] IMU menu tab (v0.5.53 b165)
- [x] Body leveling with tilt safety (v0.5.54 b166)
- [x] Motion lean (v0.6.8 b178)
- [x] ToF driver module (v0.6.2 b171)
- [x] ToF display visualization (v0.6.4 b174)
- [x] ToF menu tab (v0.6.7 b177)

### Xbox Controller Split (2025-12-22)
- [x] joy_controller.py daemon
- [x] joy_client.py socket client
- [x] mars-joy.service systemd unit
- [x] Power button start/stop with haptic feedback
- [x] Xbox connection status display (v0.7.1 b180)

### Display & UX (2025-12-06 - 2025-12-24)
- [x] Eye updates during gait (v0.4.0 b65)
- [x] Eye shape cycle with persistence (v0.4.16 b82)
- [x] Human eye improvements (v0.4.15 b81 - v0.4.22 b88)
- [x] Touch E-STOP (v0.4.23 b89)
- [x] New eye types: CAT, HYPNO, ANIME (v0.4.24 b90)
- [x] MARS menu system (v0.4.27 b93)
- [x] LCARS theme (v0.4.33 b99)
- [x] Eye flicker fixes (v0.7.7 b191 - v0.7.12 b196)
- [x] Mars startup splash (v0.7.30 b214)

### Firmware Hygiene (2025-12-18 - 2025-12-19)
- [x] Remove duplicate jitter metrics block
- [x] Refactor loopTick() into phase helpers
- [x] Consolidate extern declarations into globals.h (FW 0.2.44 b160)

### Safety & Telemetry (2025-12-12 - 2025-12-16)
- [x] Honor firmware safety state (v0.5.15 b125)
- [x] Telemetry data structures upgrade (v0.5.28 b139)
- [x] Binary telemetry support (v0.5.35 b147)

### Gait & Motion (2025-12-10 - 2025-12-13)
- [x] Wave/Ripple gait translation fixes (v0.5.19-v0.5.20)
- [x] Joystick command tolerance filtering (v0.5.50 b162)
- [x] Reverse strafe direction (v0.5.11 b121)
- [x] Turn gain tuning (v0.5.12 b122)
- [x] Eye vertical offset wiring (v0.5.9 b119)
- [x] Finer eye spacing granularity (v0.5.10 b120)

### Menu & Config (2025-12-07 - 2025-12-15)
- [x] Touchscreen config menu complete (v0.5.0 b110)
- [x] Dedicated PID/IMP/EST menu tabs (v0.5.25 b135)
- [x] Mirror window keyboard mapping (v0.5.8 b118)

### Documentation (2025-12-08 - 2025-12-17)
- [x] Firmware + controller user manual (USER_MANUAL.md)
- [x] Document telemetry field mapping
- [x] Rename to MARS branding (v0.5.7 b117)
- [x] Show firmware/controller versions at startup (v0.5.14 b124)

### Earlier Firmware Work (2025-11)
- [x] CSV logging to SD with size rotation
- [x] Tripod test mode with runtime tuning
- [x] Joint PID controller (FW 0.2.11)
- [x] Virtual impedance spring-damper (FW 0.2.13)
- [x] Estimator upgrade (FW 0.2.15)
- [x] Auto-generate missing config keys (FW)
- [x] Temperature safety lockout
- [x] Collision lockout with keep-out zones
- [x] Soft joint limits from config
- [x] Rate limit angle deltas
- [x] Width downsizing of globals
- [x] Bitmask leg/joint enable flags
- [x] Move constant strings to flash
- [x] RR/OOS feedback robustness

---

## Robot Geometry Reference (from robot_config.h)
```cpp
COXA_LENGTH_MM   = 41.70
FEMUR_LENGTH_MM  = 80.00
TIBIA_LENGTH_MM  = 133.78

COXA_OFFSET[6][2] = {  // (X, Z) in body frame
  {-65.60,  88.16},  // LF
  {-86.00,   0.00},  // LM
  {-65.60, -88.16},  // LR
  { 65.60,  88.16},  // RF
  { 86.00,   0.00},  // RM
  { 65.60, -88.16}   // RR
}

LEG_ANGLES = {LF:135°, LM:180°, LR:225°, RF:45°, RM:0°, RR:315°}
```

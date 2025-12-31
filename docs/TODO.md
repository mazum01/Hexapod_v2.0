# Project TODOs (persistent)

Last updated: 2025-12-31 (Control loop refactor COMPLETE)
Owner: Hexapod v2.0 (Firmware + Python controller)

Conventions
- [ ] not-started | [~] in-progress | [x] completed
- Keep items short and actionable; mirror high-level plan in docs/PROJECT_SPEC.md when relevant.
- Copilot will update this file when TODOs change, and reference it in commits.
- Reminder: On any behavior change or TODO completion, also bump FW/Controller versions, update in-file change logs, update CHANGELOG.md, and keep docs/USER_MANUAL.md in sync with the new versions.

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
- [ ] Track ToF edge readings (left or right side zones)
- [ ] Maintain configurable wall_distance_mm (default 200)
- [ ] PD controller to steer parallel to wall
- [ ] Enable via menu or command

#### A5. Patrol Mode
- [x] Simple timed forward walk with random turns — **DONE v0.7.39**
- [x] Configurable patrol_duration_s, turn_interval_s — **DONE v0.7.39**
- [x] Respects obstacle avoidance (higher priority) — **DONE v0.7.39**
- [ ] Stop on touch screen tap (E-stop already implemented)

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
- [ ] Add low battery audio warning (beep or TTS)

---

## Safety — Limb Collision Avoidance

### Current State (Firmware)
The firmware has geometric keep-out zone collision detection that triggers STAND+DISABLE when a foot position violates predefined body/leg clearances. This is reactive (catches violations after IK) rather than preventive.

### S1. Analytical Pre-IK Collision Check (No AI, Recommended First)
- [ ] Implement swept-volume collision test before sending foot targets
  - Model each leg segment as capsule (cylinder + hemispheres)
  - For each leg pair that could collide (LF↔LM, LM↔LR, etc.), test capsule-capsule intersection
  - Run at gait planning time (before IK) to reject or clamp dangerous trajectories
- [ ] Precompute collision risk zones per leg phase
  - During swing: leg is most vulnerable to hitting neighbors
  - During stance: stationary, minimal risk unless body tilts
- [ ] Add velocity-aware margin (faster motion → larger keep-out buffer)

### S2. Learned Collision Model (AI-Enhanced)
- [ ] Train lightweight neural network on collision data
  - Input: 18 joint angles (or 6×3 foot positions)
  - Output: collision probability (0.0–1.0) for each leg pair
  - Training data: exhaustive FK simulation sampling valid vs colliding configs
- [ ] Deploy on Pi 5 CPU (~1ms inference for small MLP)
  - No accelerator needed for 18-input → 6-output regression
  - Run every gait tick to gate trajectory before sending to firmware

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

## Open Tasks (Firmware)

### Phase 2 Backlog

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

- [ ] startup.sh update
  - Remove controller.py auto-start (now managed by joy_controller)
  - Deferred from v0.7.1 release

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

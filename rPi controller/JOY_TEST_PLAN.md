# Joy Controller Test Plan

Run joy_controller in verbose mode to see all button events:
```bash
sudo systemctl stop mars-joy.service
cd "/home/starter/OneDrive/Projects/Robots/Hexapod_v2.0/rPi controller"
python3 joy_controller.py --verbose
```

Then press Xbox button to start controller.py.

---

## Test 1: Button Code Verification

Before testing functions, verify button codes match your controller.

| Test | Action | Expected Output | Actual Code | Pass? |
|------|--------|-----------------|-------------|-------|
| 1.1 | Press A (green, bottom) | Code 304 | 304 | ✅ |
| 1.2 | Press B (red, right) | Code 305 | 305 | ✅ |
| 1.3 | Press X (blue, left) | Code 307 | 307 | ✅ |
| 1.4 | Press Y (yellow, top) | Code 308 | 308 | ✅ |
| 1.5 | Press LB (left bumper) | Code 310 | 310 | ✅ |
| 1.6 | Press RB (right bumper) | Code 311 | 311 | ✅ |
| 1.7 | Press Back/View (left center) | Code 158 or 314 | 158 | ✅ |
| 1.8 | Press Start/Menu (right center) | Code 315 | 315 | ✅ |
| 1.9 | Press Left Stick (click) | Code 317 | 317 | ✅ |
| 1.10 | Press Right Stick (click) | Code 318 | 318 | ✅ |
| 1.11 | Press Xbox Guide (center logo) | Code 172 | 172 | ✅ |
| 1.12 | Press D-pad Up | See dpad event | ✓ | ✅ |
| 1.13 | Press D-pad Down | See dpad event | ✓ | ✅ |
| 1.14 | Press D-pad Left | See dpad event | ✓ | ✅ |
| 1.15 | Press D-pad Right | See dpad event | ✓ | ✅ |

---

## Test 2: Power Button (Start/Stop Controller)

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 2.1 | Press Xbox Guide | Controller.py starts, light rumble | ✅ (no rumble over BT) |
| 2.2 | Press Xbox Guide again | Controller.py stops gracefully, strong rumble | ✅ (no rumble over BT) |
| 2.3 | Press Back button | Should NOT stop controller | ✅ |

**Note:** Rumble/vibration does not work over Bluetooth on Linux — this is a known OS limitation, not a bug.

---

## Test 3: Face Buttons (A, B, X, Y)

**Prerequisite:** Controller running, Teensy connected

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 3.1 | Press A | Standing gait starts (leveling mode) | ✅ |
| 3.2 | Press B | HOME posture, legs go to home position | ✅ |
| 3.3 | Press X | TUCK posture, legs fold in | ✅ |
| 3.4 | Press Y | Toggle TEST/IDLE mode (see verbose output) | ✅ |
| 3.5 | Hold Back + Press X | POUNCE move triggers | ✅ |

---

## Test 4: Bumpers (LB, RB)

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 4.1 | Press LB (gait off) | Start tripod gait | ✅ |
| 4.2 | Press LB (gait on) | Stop gait, go to STAND | ✅ |
| 4.3 | Press RB (gait on) | Cycle gait type (Standing→Tripod→Wave→Ripple→Stationary) | ✅ |

---

## Test 5: Stick Buttons

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 5.1 | Click Left Stick | Enter gait parameter adjust mode | ✅ |
| 5.2 | Click Right Stick | Send DISABLE command | ✅ |

---

## Test 6: Left Stick Axes

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 6.1 | Push Left Stick UP (gait on) | Robot walks FORWARD | ✅ |
| 6.2 | Push Left Stick DOWN (gait on) | Robot walks BACKWARD | ✅ |
| 6.3 | Push Left Stick LEFT (gait on) | Robot strafes left | ✅ |
| 6.4 | Push Left Stick RIGHT (gait on) | Robot strafes right | ✅ |
| 6.5 | Push Left Stick UP (gait off) | Eyes squint/close | ✅ |

---

## Test 7: Right Stick Axes

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 7.1 | Push Right Stick LEFT (gait off) | Eyes look LEFT | ✅ |
| 7.2 | Push Right Stick RIGHT (gait off) | Eyes look RIGHT | ✅ |
| 7.3 | Push Right Stick LEFT (gait on) | Heading changes left | ✅ |
| 7.4 | Push Right Stick UP (gait on) | Turn rate CCW | ✅ |
| 7.5 | Push Right Stick DOWN (gait on) | Turn rate CW | ✅ |

---

## Test 8: Triggers

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 8.1 | Hold Left Stick + Pull LT | Gait width decreases | ✅ |
| 8.2 | Hold Left Stick + Pull RT | Gait lift height increases | ✅ |

---

## Test 9: D-Pad

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 9.1 | Press D-pad UP | Cycle eye shape (prev) | ✅ |
| 9.2 | Press D-pad DOWN | Cycle eye shape (next) | ✅ |
| 9.3 | Press D-pad LEFT | Toggle CRT mode | ✅ |
| 9.4 | Press D-pad RIGHT | Toggle CRT mode | ✅ |

---

## Test 10: Start Button

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 10.1 | Press Start | Cycle display: EYES → ENGINEERING → MENU | ✅ |
| 10.2 | Press Start (in MENU) | Cycle to EYES | ✅ |

---

## Test 11: Menu Navigation (when in MENU mode)

| Test | Action | Expected Result | Pass? |
|------|--------|-----------------|-------|
| 11.1 | D-pad UP | Navigate menu up | ✅ |
| 11.2 | D-pad DOWN | Navigate menu down | ✅ |
| 11.3 | D-pad LEFT | Navigate menu left | ✅ |
| 11.4 | D-pad RIGHT | Navigate menu right | ✅ |
| 11.5 | Press A | Select menu item | ✅ |
| 11.6 | Press B | Close menu | ✅ |
| 11.7 | Press LB | Previous tab | ✅ |
| 11.8 | Press RB | Next tab | ✅ |

---

## Bug Report Section

| Bug # | Test Failed | Observed Behavior | Expected Behavior |
|-------|-------------|-------------------|-------------------|
| | | | |
| | | | |
| | | | |
| | | | |

---

## Quick Button Code Test Script

Run this to identify all button codes on your specific controller:

```bash
cd "/home/starter/OneDrive/Projects/Robots/Hexapod_v2.0/rPi controller"
python3 -c "
from evdev import InputDevice, list_devices

for path in list_devices():
    dev = InputDevice(path)
    if 'xbox' in dev.name.lower() or 'controller' in dev.name.lower():
        print(f'Controller: {dev.name}')
        print('Press each button. Ctrl+C to exit.')
        print()
        try:
            for event in dev.read_loop():
                if event.type == 1:  # Button
                    state = 'PRESSED' if event.value == 1 else 'RELEASED'
                    print(f'Button code {event.code}: {state}')
                elif event.type == 3:  # Axis
                    print(f'Axis code {event.code}: value {event.value}')
        except KeyboardInterrupt:
            print('Done')
        break
"
```

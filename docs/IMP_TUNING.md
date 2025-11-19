# Impedance Tuning Guide

This guide describes a practical procedure for tuning the virtual spring–damper impedance layer implemented by `MarsImpedance` on the Teensy 4.1 hexapod.

The impedance layer sits on top of the existing trajectory + PID controllers and adds a compliant "springy" behavior in joint space or simple Cartesian Z-space.

- **Joint mode**: per-joint virtual springs and dampers.
- **Cartesian mode**: Z-axis (vertical) impedance at the foot, mapped into tibia correction.

All tuning is performed at runtime over the serial command interface using the `IMP` command family.

---

## 1. Concepts and knobs

### 1.1 Modes

- `IMP MODE OFF`  
  Impedance disabled. Only base trajectory and PID act on the joints.

- `IMP MODE JOINT`  
  Per-joint impedance around the commanded joint angles.

- `IMP MODE CART`  
  Cartesian (foot) impedance, currently applied along the body-frame Z axis and mapped into the tibia joint.

Impedance is globally gated by `IMP ENABLE` / `IMP DISABLE` and the `imp.enabled` config flag.

### 1.2 Joint-space gains

Commands:

- `IMP JSPRING <COXA|FEMUR|TIBIA|ALL> <milli>`
- `IMP JDAMP   <COXA|FEMUR|TIBIA|ALL> <milli>`

These adjust per-joint spring and damping gains in **milli-units** (integers). Larger values mean:

- Higher `JSPRING` → stiffer response to joint position error (more "springy").
- Higher `JDAMP`   → stronger resistance to joint velocity (more "viscous").

Gains can be set per joint or for all three at once using `ALL`.

### 1.3 Cartesian Z gains

Commands:

- `IMP CSPRING <Z|ALL> <milli>`
- `IMP CDAMP   <Z|ALL> <milli>`

These adjust spring and damping along the body-frame vertical axis at the foot. The implementation currently uses only Z; `ALL` sets the same value on X/Y/Z for future expansion, but X/Y are effectively unused in Phase 1.

---

## 2. Pre-flight checks

Before tuning impedance:

1. **PID must be stable**
   - Tripod test mode should not oscillate badly.
   - Single-leg PID tests should show monotone or mildly underdamped behavior.

2. **No persistent safety errors**
   - STATUS should not show streaming soft-limit, collision, or temperature lockouts for a static stance.

3. **Start with impedance OFF**
   - `IMP DISABLE`
   - `IMP MODE OFF`
   - Verify `[IMP]` in `STATUS` reports `enabled=0` or `mode=off`.

It is strongly recommended to start with the robot supported (e.g., body on a stand, feet free) while you find initial gains.

---

## 3. Joint-space impedance tuning

### 3.1 Baseline configuration

1. Enable impedance but set all gains to zero:

   - `IMP ENABLE`
   - `IMP MODE JOINT`
   - `IMP JSPRING ALL 0`
   - `IMP JDAMP   ALL 0`

2. Confirm in `STATUS` that `mode=joint` and all joint gains are 0.

3. Place the robot in a comfortable stance with the legs supported or hanging so they can be safely disturbed by hand.

### 3.2 Tibia tuning (vertical compliance)

The tibia has the most direct effect on vertical foot motion. Tune it first.

1. **Single-leg test**
   - Choose one leg (e.g., LF) and keep the robot otherwise supported so tipping is impossible.

2. **Introduce small tibia gains**
   - Start very soft:
     - `IMP JSPRING TIBIA 200`
     - `IMP JDAMP   TIBIA  50`
   - Manually move the tibia (lift/lower the foot) and release.

3. **Adjust spring (JSPRING)**
   - Increase in small steps (e.g., +100–200 at a time):
     - If the leg feels **floppy** and does not return noticeably toward its original angle, increase `JSPRING`.
     - If the leg snaps back aggressively or the servo sounds strained, reduce `JSPRING`.

4. **Adjust damping (JDAMP)**
   - For a given `JSPRING`:
     - If the leg **overshoots and bounces**, increase `JDAMP`.
     - If the leg feels **sluggish or sticky**, decrease `JDAMP`.

A good tibia target for initial testing is typically in the **low-thousands** for `JSPRING` and **20–50% of that value** for `JDAMP`.

**Acceptance for tibia**:

- When you pull the foot down and release, it returns smoothly toward nominal height.
- One small rebound is acceptable; sustained or growing oscillation is not.

### 3.3 Femur tuning (vertical + pitch compliance)

Femur impedance affects both vertical stiffness and body pitch.

1. Reset femur gains:
   - `IMP JSPRING FEMUR 0`
   - `IMP JDAMP   FEMUR 0`

2. With tibia gains fixed, add small femur gains:
   - `IMP JSPRING FEMUR 200`
   - `IMP JDAMP   FEMUR  50`

3. Push down on the body near the leg and release.

4. Increase/decrease `JSPRING FEMUR` and `JDAMP FEMUR` as needed:
   - Too bouncy: increase damping or reduce spring.
   - Too rigid: reduce spring to avoid large torques and pitch shocks.

**Acceptance for femur**:

- Pressing the body down causes a controlled sink and recovery.
- No multi-cycle pitch oscillation when you let go.

### 3.4 Coxa tuning (lateral/yaw compliance)

Coxa motions affect lateral sway and yaw. They should usually be softer than femur/tibia.

1. Start very small:
   - `IMP JSPRING COXA 100`
   - `IMP JDAMP   COXA  30`

2. Gently push the hip sideways or twist the body around the vertical axis.

3. Adjust:
   - If the robot pivots back sharply or oscillates side-to-side, reduce coxa spring and/or increase damping.
   - If it feels completely dead sideways, increase spring slightly.

**Acceptance for coxa**:

- Lateral pushes result in mild sway and a slow recentering.
- No obvious side-to-side ringing.

### 3.5 Whole-robot sanity check

Once per-joint gains feel good:

1. Try a **slow tripod gait** (test mode).
2. During walking:
   - Lightly nudge the body and individual legs.
   - Watch for:
     - Stable foot placement.
     - No violent corrections.
     - No high-frequency chatter.

If gait becomes unstable when impedance is enabled, scale all joint gains down (e.g., halve JSPRING/JDAMP) and revisit.

---

## 4. Cartesian Z impedance tuning

After JOINT mode is working, you can experiment with CART mode to shape vertical compliance at the foot.

### 4.1 Initial CART configuration

1. Set mode and small Z gains:
   - `IMP ENABLE`
   - `IMP MODE CART`
   - `IMP CSPRING Z  500`
   - `IMP CDAMP   Z  100`

2. Confirm in `STATUS` that `mode=cart` and only Z gains are non-zero.

### 4.2 Vertical push test

1. With the robot in a static stance, gently push the body vertically downward.
2. Observe:
   - The body should sink slightly then return.
   - Foot-ground contact should feel more compliant than in pure PID mode.

3. Adjust gains:
   - If too soft (body drops excessively and feels mushy): increase `CSPRING Z`.
   - If it bounces in height: increase `CDAMP Z` and/or reduce `CSPRING Z`.

**Acceptance for CART Z**:

- Body height recovers without repeated bouncing.
- No obvious high-frequency tibia chatter at standstill.

### 4.3 Gait with CART mode

1. Run the tripod test gait at conservative parameters.
2. Watch for:
   - Smooth absorption of small terrain changes.
   - No large heave oscillations (body bobbing up and down).

If gait degrades compared to JOINT mode, treat CART mode as experimental and either:

- Reduce CSPRING/CDAMP; or
- Revert to JOINT mode for production and use CART only for lab experiments.

---

## 5. Heuristics and troubleshooting

### 5.1 Safe tuning order

1. Verify PID stability.
2. Tune JOINT impedance:
   - Tibia → Femur → Coxa.
3. Verify gait with JOINT impedance.
4. Experiment with CART Z impedance last.

### 5.2 Typical issues and remedies

- **Buzzing at standstill**
  - Gains too high or damping too low.
  - Reduce JSPRING/CSPRING or increase JDAMP/CDAMP.

- **Robot feels rigid and harsh**
  - Spring too high relative to base PID.
  - Lower JSPRING/CSPRING until manual pushes result in noticeable but controlled motion.

- **Gait destabilizes when impedance is enabled**
  - Scale all impedance gains down (e.g., multiply by 0.5).
  - Optionally reduce gait aggressiveness (step height/length) while tuning.

- **Feet don’t track commanded pose well**
  - Gains may be too low (overly soft) or PID may be under-tuned.
  - Confirm PID-only tracking first, then add modest impedance.

---

## 6. Example starting profiles

These are conservative starting points intended for lab testing, not final values.

### 6.1 Soft joint-space profile (stand + slow gait)

```text
IMP ENABLE
IMP MODE JOINT
IMP JSPRING COXA  150
IMP JSPRING FEMUR 800
IMP JSPRING TIBIA 1200
IMP JDAMP   COXA  50
IMP JDAMP   FEMUR 200
IMP JDAMP   TIBIA 300
```

### 6.2 Experimental Cartesian Z profile

```text
IMP ENABLE
IMP MODE CART
IMP CSPRING Z 800
IMP CDAMP   Z 200
```

Always adjust from these baselines in small steps and validate after each change.

---

## 7. Persistence

When SD logging/config is enabled, `IMP` commands that change impedance parameters will be persisted back into `/config.txt` via `imp.*` keys, so your tuned gains survive reboot:

- `imp.enabled`
- `imp.mode`
- `imp.joint.k_spring_milli.[coxa|femur|tibia]`
- `imp.joint.k_damp_milli.[coxa|femur|tibia]`
- `imp.cart.k_spring_milli.z`
- `imp.cart.k_damp_milli.z`

Use `IMP LIST` and `STATUS` to confirm both runtime and persisted values match your expectations before field tests.

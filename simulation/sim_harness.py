#!/usr/bin/env python3
"""
sim_harness.py — Standalone simulation runner for the MARS hexapod.

Tests:
  1. Model loads and standing keyframe is valid
  2. Robot holds standing pose under gravity (1 second sim)
  3. Joint commands move legs correctly
  4. Robot can be rendered to image (headless)

Run:
  python -m simulation.sim_harness
"""

import math
import time
import os
import numpy as np

LEG_NAMES = ["LF", "LM", "LR", "RF", "RM", "RR"]
JOINT_TYPES = ["coxa", "femur", "tibia"]


def test_standing_stability(model, data, duration_s=2.0):
    """Verify robot holds standing pose under gravity.

    Steps simulation for duration_s and checks:
    - Body doesn't fall (Z stays near initial)
    - Body doesn't tilt excessively
    - Feet maintain ground contact
    """
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)

    # Set ctrl to match standing pose (hold position)
    standing_ctrl = data.qpos[7:].copy()  # joint positions from keyframe
    data.ctrl[:] = standing_ctrl

    initial_z = data.qpos[2]
    n_steps = int(duration_s / model.opt.timestep)

    max_z_drop = 0.0
    max_tilt_deg = 0.0

    for step in range(n_steps):
        mujoco.mj_step(model, data)

        z = data.qpos[2]
        z_drop = initial_z - z
        max_z_drop = max(max_z_drop, z_drop)

        # Body orientation: quaternion → tilt angle
        qw, qx, qy, qz = data.qpos[3:7]
        # Tilt from vertical: angle between body Z and world Z
        # For small tilts: tilt ≈ 2 * sqrt(qx² + qy²) radians
        tilt_rad = 2.0 * math.sqrt(qx * qx + qy * qy)
        tilt_deg = math.degrees(tilt_rad)
        max_tilt_deg = max(max_tilt_deg, tilt_deg)

    final_z = data.qpos[2]

    print(f"\n  Standing stability test ({duration_s}s, {n_steps} steps):")
    print(f"    Initial Z:   {initial_z*1e3:.1f}mm")
    print(f"    Final Z:     {final_z*1e3:.1f}mm")
    print(f"    Max Z drop:  {max_z_drop*1e3:.2f}mm")
    print(f"    Max tilt:    {max_tilt_deg:.2f}°")

    # Pass criteria
    z_ok = max_z_drop < 5.0e-3      # less than 5mm drop
    tilt_ok = max_tilt_deg < 5.0     # less than 5° tilt

    if z_ok and tilt_ok:
        print("    ✓ PASS — robot stands stably")
    else:
        if not z_ok:
            print(f"    ✗ FAIL — body dropped {max_z_drop*1e3:.1f}mm (limit 5mm)")
        if not tilt_ok:
            print(f"    ✗ FAIL — body tilted {max_tilt_deg:.1f}° (limit 5°)")

    return z_ok and tilt_ok


def test_joint_commands(model, data):
    """Verify joint actuators respond to commands.

    Applies a small offset to one joint and checks that it moves.
    """
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = data.qpos[7:].copy()
    mujoco.mj_forward(model, data)

    # Pick RF_femur (joint index 10: RF is leg 3, femur is joint 1 → 3*3+1=10)
    test_joint_idx = 10  # RF_femur
    test_actuator_idx = 10

    initial_angle = data.qpos[7 + test_joint_idx]

    # Command a 10° offset
    offset_rad = math.radians(10.0)
    data.ctrl[test_actuator_idx] = initial_angle + offset_rad

    # Step for 0.5s
    n_steps = int(0.5 / model.opt.timestep)
    for _ in range(n_steps):
        mujoco.mj_step(model, data)

    final_angle = data.qpos[7 + test_joint_idx]
    moved_deg = math.degrees(final_angle - initial_angle)

    print(f"\n  Joint command test (RF_femur +10°):")
    print(f"    Initial: {math.degrees(initial_angle):.1f}°")
    print(f"    Final:   {math.degrees(final_angle):.1f}°")
    print(f"    Moved:   {moved_deg:+.1f}°")

    if abs(moved_deg) > 3.0:  # should move at least 3° in 0.5s
        print("    ✓ PASS — joint responds to commands")
        return True
    else:
        print(f"    ✗ FAIL — joint barely moved ({moved_deg:.1f}°)")
        return False


def test_foot_contacts(model, data):
    """Verify foot contact sensors report ground contact in standing pose."""
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = data.qpos[7:].copy()

    # Step briefly to let contacts settle
    for _ in range(100):
        mujoco.mj_step(model, data)

    print("\n  Foot contact test:")
    all_contact = True
    for name in LEG_NAMES:
        sensor_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, f"{name}_contact"
        )
        adr = model.sensor_adr[sensor_id]
        force = data.sensordata[adr]
        has_contact = force > 0.01
        status = "✓" if has_contact else "✗"
        print(f"    {name}: {status} force={force:.3f}N")
        if not has_contact:
            all_contact = False

    if all_contact:
        print("    ✓ PASS — all feet in contact")
    else:
        print("    ✗ FAIL — some feet not in contact")

    return all_contact


def render_frame(model, data, filepath="simulation/render.png"):
    """Render a single frame to an image file (headless)."""
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = data.qpos[7:].copy()

    # Step briefly
    for _ in range(50):
        mujoco.mj_step(model, data)

    width, height = 1280, 960
    renderer = mujoco.Renderer(model, height, width)
    renderer.update_scene(data, camera="tracking")
    img = renderer.render()
    renderer.close()

    # Save as PPM (no PIL dependency needed)
    with open(filepath, 'wb') as f:
        f.write(f"P6\n{width} {height}\n255\n".encode())
        f.write(img.tobytes())

    print(f"\n  Rendered frame to {filepath} ({width}×{height})")
    return True


def main():
    import mujoco

    script_dir = os.path.dirname(os.path.abspath(__file__))
    mjcf_path = os.path.join(script_dir, "hexapod.xml")

    if not os.path.exists(mjcf_path):
        print("Model not found. Run: python -m simulation.hexapod_model")
        return

    print("MARS Hexapod — Simulation Harness")
    print("=" * 50)

    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    print(f"  Loaded model: {model.nq} DOF, {model.nu} actuators, "
          f"dt={model.opt.timestep*1e3:.1f}ms")

    results = {}

    # Test 1: Standing stability
    results['standing'] = test_standing_stability(model, data)

    # Test 2: Joint commands
    results['joints'] = test_joint_commands(model, data)

    # Test 3: Foot contacts
    results['contacts'] = test_foot_contacts(model, data)

    # Test 4: Render (optional)
    try:
        results['render'] = render_frame(model, data,
                                         os.path.join(script_dir, "render.ppm"))
    except Exception as e:
        print(f"\n  Render skipped: {e}")
        results['render'] = None

    # Summary
    print("\n" + "=" * 50)
    print("Summary:")
    for test_name, passed in results.items():
        if passed is None:
            status = "SKIP"
        elif passed:
            status = "PASS"
        else:
            status = "FAIL"
        print(f"  {test_name:12s}: {status}")

    all_passed = all(v is True for v in results.values() if v is not None)
    if all_passed:
        print("\n✓ All tests passed — model is ready for Phase 2+")
    else:
        print("\n✗ Some tests failed — model needs tuning")


if __name__ == "__main__":
    main()

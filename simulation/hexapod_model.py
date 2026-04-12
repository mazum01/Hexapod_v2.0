#!/usr/bin/env python3
"""
hexapod_model.py — Generate MuJoCo MJCF model of the MARS hexapod robot.

Generates:
  simulation/hexapod.xml       — MuJoCo model description
  simulation/meshes/body.stl   — Extruded body hull mesh

Run:
  python -m simulation.hexapod_model

Coordinate Frames:
  Robot body frame:  X = right,   Y = up,      Z = forward
  MuJoCo world:      X = right,   Y = forward,  Z = up

  Mapping: robot(X, Y, Z) → mujoco(X, Z, Y)
"""

import math
import struct
import os

# ═══════════════════════════════════════════════════════════════════════════════
# Robot Constants (from firmware/MARS_Hexapod/robot_config.h + HTS-35H specs)
# ═══════════════════════════════════════════════════════════════════════════════

TOTAL_MASS_KG = 2.1  # configurable

# Servo specs (Hiwonder HTS-35H)
NUM_LEGS = 6
SERVOS_PER_LEG = 3
SERVO_MASS_KG = 0.064
SERVO_STALL_TORQUE_NM = 3.43   # 35 kg·cm @ 7.4V
SERVO_MAX_SPEED_RAD_S = 5.82   # 333°/s @ 11.1V (0.18s/60°)
SERVO_RANGE_DEG = 240.0        # 0–240° (0–24000 centidegrees)

# Link lengths (m)
COXA_LENGTH  = 41.70e-3
FEMUR_LENGTH = 80.00e-3
TIBIA_LENGTH = 133.78e-3

# Link capsule radii for visual/collision geometry (m, estimated from CAD)
COXA_RADIUS  = 0.012
FEMUR_RADIUS = 0.010
TIBIA_RADIUS = 0.008
FOOT_RADIUS  = 0.008

LEG_NAMES = ["LF", "LM", "LR", "RF", "RM", "RR"]

# Coxa origin offsets in body frame (m)
# Robot frame: (X=right, Z=forward) → MuJoCo: (X=right, Y=forward)
COXA_OFFSET_X = [  # robot X = MuJoCo X
    -65.5959e-3, -86.0000e-3, -65.5959e-3,
     65.5959e-3,  86.0000e-3,  65.5959e-3,
]
COXA_OFFSET_Z = [  # robot Z = MuJoCo Y
     88.1630e-3,   0.0000e-3, -88.1630e-3,
     88.1630e-3,   0.0000e-3, -88.1630e-3,
]

# Body 2D convex hull (robot XZ plane = MuJoCo XY plane), in mm
# 24 vertices, counterclockwise when viewed from above (+Z in MuJoCo)
BODY_HULL_XZ_MM = [
    ( 70.5,  107.9), (-72.0,  106.4), (-76.2,  102.2), (-79.2,   99.2),
    (-83.4,   95.1), (-84.7,   93.8), (-103.0,   9.3), (-103.0,  -9.5),
    (-86.2,  -93.8), (-80.5,  -99.5), (-79.1, -100.9), (-77.0, -103.0),
    (-73.3, -106.7), (-72.0, -107.9), ( 72.0, -107.9), ( 73.3, -106.7),
    ( 77.0, -103.0), ( 77.6, -102.4), ( 79.8, -100.1), ( 84.2,  -95.8),
    ( 86.2,  -93.8), (103.0,   -9.5), (103.0,    9.3), ( 84.7,   93.8),
]

BODY_Y_BOTTOM_MM = -60.0   # robot Y = MuJoCo Z (bottom of body)
BODY_Y_TOP_MM    =  60.0   # robot Y = MuJoCo Z (top of body)

# ═══════════════════════════════════════════════════════════════════════════════
# Derived Constants
# ═══════════════════════════════════════════════════════════════════════════════

BODY_MASS_KG = TOTAL_MASS_KG - NUM_LEGS * SERVOS_PER_LEG * SERVO_MASS_KG

# Joint range: ±120° from home (full 240° servo range centered at home)
JOINT_RANGE_RAD = math.radians(SERVO_RANGE_DEG / 2.0)  # ±2.094 rad

# Actuator PD gains
ACTUATOR_KP = 20.0     # N·m/rad — position tracking stiffness
JOINT_DAMPING = 0.5    # N·m·s/rad — velocity damping (limits speed)

# Firmware defaults for standing pose
STAND_OUTWARD_MM = 130.0   # base_x — hip-relative outward distance (mm)
STAND_HEIGHT_MM  = 150.0   # |base_y| — below hip (mm)
STANDING_HEIGHT_M = STAND_HEIGHT_MM * 1e-3  # body center above ground (m)

# ═══════════════════════════════════════════════════════════════════════════════
# IK for Standing Pose
# ═══════════════════════════════════════════════════════════════════════════════

def compute_standing_angles(outward_mm=STAND_OUTWARD_MM,
                            height_mm=STAND_HEIGHT_MM):
    """Compute MuJoCo joint angles for a symmetric standing pose.

    Each leg's local frame (after mount rotation):
      +Y = outward from body center
      +Z = up
      +X = perpendicular (femur pitch axis)

    At joint=0 the femur/tibia hang straight down (-Z).
    Positive rotation swings from -Z toward +Y (outward).

    Args:
        outward_mm: foot distance from hip along the outward direction
        height_mm:  foot distance below hip

    Returns:
        (coxa, femur, tibia) angles in radians
    """
    R = (outward_mm * 1e-3) - COXA_LENGTH   # horizontal from femur base
    h = height_mm * 1e-3                      # vertical below femur base

    F, T = FEMUR_LENGTH, TIBIA_LENGTH
    D = math.sqrt(R * R + h * h)

    if D > F + T or D < abs(F - T):
        raise ValueError(
            f"Unreachable: D={D*1e3:.1f}mm, range=[{abs(F-T)*1e3:.1f}, {(F+T)*1e3:.1f}]mm"
        )

    # Tibia: internal knee angle via law of cosines, then supplement.
    # Negate for the "elbow-out" solution (knees pointing outward).
    cos_knee = (F * F + T * T - D * D) / (2.0 * F * T)
    knee_internal = math.acos(max(-1.0, min(1.0, cos_knee)))
    tibia_joint = -(math.pi - knee_internal)

    # Femur: 2-link IK, elbow-out solution (+ instead of -)
    P = F + T * math.cos(tibia_joint)
    Q = T * math.sin(tibia_joint)
    femur_joint = math.atan2(R, h) - math.atan2(Q, P)

    return 0.0, femur_joint, tibia_joint


def compute_mount_yaw(leg_idx):
    """Compute MuJoCo Z-rotation so local +Y points outward from body center."""
    ox = COXA_OFFSET_X[leg_idx]
    oy = COXA_OFFSET_Z[leg_idx]   # robot Z → MuJoCo Y
    return math.atan2(-ox, oy)


# ═══════════════════════════════════════════════════════════════════════════════
# Body Mesh Generation
# ═══════════════════════════════════════════════════════════════════════════════

def generate_body_stl(filepath):
    """Generate body mesh as binary STL by extruding the 2D hull.

    The hull lives in the MuJoCo XY plane, extruded along Z.
    """
    hull = [(x * 1e-3, z * 1e-3) for x, z in BODY_HULL_XZ_MM]
    z_bot = BODY_Y_BOTTOM_MM * 1e-3
    z_top = BODY_Y_TOP_MM * 1e-3
    n = len(hull)

    triangles = []

    # Top face: fan from vertex 0, normal +Z
    for i in range(1, n - 1):
        triangles.append((
            (0.0, 0.0, 1.0),
            (hull[0][0],   hull[0][1],   z_top),
            (hull[i][0],   hull[i][1],   z_top),
            (hull[i+1][0], hull[i+1][1], z_top),
        ))

    # Bottom face: reversed winding, normal -Z
    for i in range(1, n - 1):
        triangles.append((
            (0.0, 0.0, -1.0),
            (hull[0][0],   hull[0][1],   z_bot),
            (hull[i+1][0], hull[i+1][1], z_bot),
            (hull[i][0],   hull[i][1],   z_bot),
        ))

    # Side faces: 2 triangles per edge
    for i in range(n):
        j = (i + 1) % n
        bl = (hull[i][0], hull[i][1], z_bot)
        br = (hull[j][0], hull[j][1], z_bot)
        tl = (hull[i][0], hull[i][1], z_top)
        tr = (hull[j][0], hull[j][1], z_top)

        # Outward normal for CCW winding
        edge_dx = hull[j][0] - hull[i][0]
        edge_dy = hull[j][1] - hull[i][1]
        nx, ny = edge_dy, -edge_dx
        length = math.sqrt(nx * nx + ny * ny)
        if length > 0:
            nx /= length
            ny /= length
        normal = (nx, ny, 0.0)

        triangles.append((normal, bl, tl, br))
        triangles.append((normal, br, tl, tr))

    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(triangles)))
        for normal, v0, v1, v2 in triangles:
            f.write(struct.pack('<3f', *normal))
            f.write(struct.pack('<3f', *v0))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(struct.pack('<H', 0))

    print(f"  Generated {filepath}: {len(triangles)} triangles")


# ═══════════════════════════════════════════════════════════════════════════════
# MJCF Generation
# ═══════════════════════════════════════════════════════════════════════════════

def _build_leg_xml(leg_idx, jr):
    """Build MJCF XML for one leg (coxa → femur → tibia → foot)."""
    name = LEG_NAMES[leg_idx]
    pos_x = COXA_OFFSET_X[leg_idx]
    pos_y = COXA_OFFSET_Z[leg_idx]   # robot Z → MuJoCo Y
    yaw = compute_mount_yaw(leg_idx)

    return f"""\
      <!-- {name} leg -->
      <body name="{name}_coxa" pos="{pos_x:.6f} {pos_y:.6f} 0" euler="0 0 {yaw:.6f}">
        <joint name="{name}_coxa" type="hinge" axis="0 0 1"
               range="{-jr:.4f} {jr:.4f}"/>
        <geom name="{name}_coxa" type="capsule"
              fromto="0 0 0 0 {COXA_LENGTH:.6f} 0" size="{COXA_RADIUS}"
              rgba="0.9 0.6 0.1 1" mass="{SERVO_MASS_KG}"/>

        <body name="{name}_femur" pos="0 {COXA_LENGTH:.6f} 0">
          <joint name="{name}_femur" type="hinge" axis="1 0 0"
                 range="{-jr:.4f} {jr:.4f}"/>
          <geom name="{name}_femur" type="capsule"
                fromto="0 0 0 0 0 {-FEMUR_LENGTH:.6f}" size="{FEMUR_RADIUS}"
                rgba="0.8 0.7 0.2 1" mass="{SERVO_MASS_KG}"/>

          <body name="{name}_tibia" pos="0 0 {-FEMUR_LENGTH:.6f}">
            <joint name="{name}_tibia" type="hinge" axis="1 0 0"
                   range="{-jr:.4f} {jr:.4f}"/>
            <geom name="{name}_tibia" type="capsule"
                  fromto="0 0 0 0 0 {-TIBIA_LENGTH:.6f}" size="{TIBIA_RADIUS}"
                  rgba="0.7 0.3 0.2 1" mass="{SERVO_MASS_KG}"/>
            <geom name="{name}_foot" type="sphere"
                  pos="0 0 {-TIBIA_LENGTH:.6f}" size="{FOOT_RADIUS}"
                  rgba="0.9 0.9 0.9 1" mass="0.001"/>
            <site name="{name}_foot_site" pos="0 0 {-TIBIA_LENGTH:.6f}"
                  type="sphere" size="{FOOT_RADIUS * 3:.4f}" rgba="1 0 0 0.3"/>
          </body>
        </body>
      </body>"""


def _build_actuators_xml(jr):
    """Build actuator XML for all 18 joints."""
    lines = []
    for name in LEG_NAMES:
        for joint in ["coxa", "femur", "tibia"]:
            jn = f"{name}_{joint}"
            lines.append(
                f'    <position name="{jn}_act" joint="{jn}"'
                f' kp="{ACTUATOR_KP}" ctrlrange="{-jr:.4f} {jr:.4f}"'
                f' forcerange="{-SERVO_STALL_TORQUE_NM:.3f}'
                f' {SERVO_STALL_TORQUE_NM:.3f}"/>'
            )
    return "\n".join(lines)


def _build_sensors_xml():
    """Build sensor XML for body IMU, foot contacts, and joint state."""
    lines = [
        '    <!-- Body IMU -->',
        '    <framequat name="body_quat" objtype="body" objname="body"/>',
        '    <framelinvel name="body_linvel" objtype="body" objname="body"/>',
        '    <frameangvel name="body_angvel" objtype="body" objname="body"/>',
        '    <accelerometer name="imu_accel" site="body_imu"/>',
        '    <gyro name="imu_gyro" site="body_imu"/>',
        '',
        '    <!-- Foot contact forces -->',
    ]
    for name in LEG_NAMES:
        lines.append(f'    <touch name="{name}_contact" site="{name}_foot_site"/>')

    lines.append('')
    lines.append('    <!-- Joint positions and velocities -->')
    for name in LEG_NAMES:
        for joint in ["coxa", "femur", "tibia"]:
            jn = f"{name}_{joint}"
            lines.append(f'    <jointpos name="{jn}_pos" joint="{jn}"/>')
            lines.append(f'    <jointvel name="{jn}_vel" joint="{jn}"/>')

    return "\n".join(lines)


def _build_keyframe(coxa_ang, femur_ang, tibia_ang):
    """Build standing keyframe qpos and ctrl strings."""
    # Free joint: [x, y, z, qw, qx, qy, qz]
    body_qpos = [0.0, 0.0, STANDING_HEIGHT_M, 1.0, 0.0, 0.0, 0.0]
    joint_qpos = [coxa_ang, femur_ang, tibia_ang] * NUM_LEGS
    ctrl = [coxa_ang, femur_ang, tibia_ang] * NUM_LEGS

    qpos_str = " ".join(f"{v:.6f}" for v in body_qpos + joint_qpos)
    ctrl_str = " ".join(f"{v:.6f}" for v in ctrl)
    return qpos_str, ctrl_str


def generate_mjcf(filepath, mesh_file="body.stl"):
    """Generate the MuJoCo MJCF XML file."""
    coxa_ang, femur_ang, tibia_ang = compute_standing_angles()
    jr = JOINT_RANGE_RAD

    legs_xml = "\n".join(_build_leg_xml(i, jr) for i in range(NUM_LEGS))
    actuators_xml = _build_actuators_xml(jr)
    sensors_xml = _build_sensors_xml()
    qpos_str, ctrl_str = _build_keyframe(coxa_ang, femur_ang, tibia_ang)

    mjcf = f"""\
<mujoco model="mars_hexapod">
  <!--
    MARS — Modular Autonomous Robotic System — MuJoCo Model
    Generated by simulation/hexapod_model.py

    Coordinate frame: X=right, Y=forward, Z=up
    Robot body frame:  X=right, Y=up, Z=forward
    Mapping: robot(X,Y,Z) → mujoco(X,Z,Y)

    Total mass: {TOTAL_MASS_KG} kg  (body {BODY_MASS_KG:.3f} + 18 servos × {SERVO_MASS_KG} kg)
    Servo: HTS-35H, {SERVO_STALL_TORQUE_NM} N·m stall, {math.degrees(SERVO_MAX_SPEED_RAD_S):.0f}°/s
    Standing: {STAND_OUTWARD_MM:.0f}mm outward, {STAND_HEIGHT_MM:.0f}mm below hip
    Standing angles: femur={math.degrees(femur_ang):.1f}°, tibia={math.degrees(tibia_ang):.1f}°
  -->

  <compiler angle="radian" meshdir="meshes"/>

  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicit">
    <flag contact="enable"/>
  </option>

  <visual>
    <headlight ambient="0.3 0.3 0.3" diffuse="0.6 0.6 0.6"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="1920" offheight="1080"/>
  </visual>

  <default>
    <joint damping="{JOINT_DAMPING}" limited="true" armature="0.01"/>
    <geom condim="3" friction="0.8 0.005 0.0001" margin="0.001"/>
  </default>

  <asset>
    <mesh name="body_mesh" file="{mesh_file}" smoothnormal="true"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.8 0.8 0.8" rgb2="0.6 0.6 0.6"/>
    <material name="floor_mat" texture="grid" texrepeat="8 8" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light directional="true" pos="0 0 3" dir="0 0.3 -1" castshadow="true"/>
    <camera name="tracking" pos="0.6 -0.4 0.4" xyaxes="0.5 0.866 0 -0.2 0.115 0.97"
            mode="trackcom"/>
    <camera name="side" pos="0.5 0 0.3" xyaxes="0 1 0 -0.5 0 0.866"
            mode="trackcom"/>
    <camera name="top" pos="0 0 1.0" xyaxes="1 0 0 0 1 0" mode="trackcom"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="floor_mat"/>

    <body name="body" pos="0 0 {STANDING_HEIGHT_M:.4f}">
      <freejoint name="root"/>
      <site name="body_imu" pos="0 0 0" type="box" size="0.01 0.01 0.01"
            rgba="0 1 0 0.3"/>
      <geom name="body_shell" type="mesh" mesh="body_mesh"
            rgba="0.2 0.2 0.4 1" mass="{BODY_MASS_KG:.4f}"/>

{legs_xml}
    </body>
  </worldbody>

  <actuator>
{actuators_xml}
  </actuator>

  <sensor>
{sensors_xml}
  </sensor>

  <keyframe>
    <key name="stand" qpos="{qpos_str}"
         ctrl="{ctrl_str}"/>
  </keyframe>

</mujoco>
"""

    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    with open(filepath, 'w') as f:
        f.write(mjcf)

    print(f"  Generated {filepath}")
    print(f"  Standing pose: femur={math.degrees(femur_ang):.1f}°,"
          f" tibia={math.degrees(tibia_ang):.1f}°")
    print(f"  Body mass: {BODY_MASS_KG:.3f} kg, Total: {TOTAL_MASS_KG} kg")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(script_dir, "meshes", "body.stl")
    mjcf_path = os.path.join(script_dir, "hexapod.xml")

    print("MARS Hexapod — MuJoCo Model Generator")
    print("=" * 50)

    print("\n1. Generating body mesh...")
    generate_body_stl(mesh_path)

    print("\n2. Computing standing pose IK...")
    coxa, femur, tibia = compute_standing_angles()
    print(f"  coxa  = {math.degrees(coxa):+.2f}°  ({coxa:+.4f} rad)")
    print(f"  femur = {math.degrees(femur):+.2f}°  ({femur:+.4f} rad)")
    print(f"  tibia = {math.degrees(tibia):+.2f}°  ({tibia:+.4f} rad)")

    print("\n3. Generating MJCF model...")
    generate_mjcf(mjcf_path)

    # Quick validation: try loading with MuJoCo
    print("\n4. Validating model...")
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)

        # Load standing keyframe
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)

        body_z = data.qpos[2]
        print(f"  ✓ Model loaded: {model.nq} qpos, {model.nu} actuators,"
              f" {model.ngeom} geoms")
        print(f"  ✓ Body height at keyframe: {body_z*1e3:.1f}mm")
        print(f"  ✓ Total model mass: {sum(model.body_mass):.3f} kg")

        # Check foot heights
        for i, name in enumerate(LEG_NAMES):
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                        f"{name}_foot_site")
            foot_z = data.site_xpos[site_id][2]
            print(f"    {name} foot Z: {foot_z*1e3:+.1f}mm")

    except ImportError:
        print("  ⚠ mujoco not installed — skipping validation")
    except Exception as e:
        print(f"  ✗ Validation failed: {e}")

    print("\n✓ Done! Model at:", mjcf_path)


if __name__ == "__main__":
    main()

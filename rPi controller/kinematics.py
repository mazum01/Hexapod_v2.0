"""
kinematics.py — Geometric model and kinematics for collision detection.

This module provides Forward Kinematics (FK) and Inverse Kinematics (IK)
functions matching the firmware's math, using the canonical dimensions
from robot_config.h. It is used by the analytical collision checker (S1).

Coordinate System (Body Frame):
    X: Right is positive
    Y: Up is positive (ground is typically around Y ≈ -100 mm)
    Z: Forward is positive

Leg Kinematic Chain (LegPoints returned by fk_leg):
    hip    — Body attachment point (coxa servo axis). Fixed offset from body
             center defined by COXA_OFFSET_X/Z. Y is always 0.
    knee   — Coxa-Femur joint. End of coxa link, COXA_LENGTH_MM from hip,
             positioned along the leg's yaw direction in the XZ plane.
    ankle  — Femur-Tibia joint. End of femur link, FEMUR_LENGTH_MM from knee,
             positioned according to femur pitch angle.
    foot   — Tip of tibia (end-effector). TIBIA_LENGTH_MM from ankle,
             following the combined femur+tibia angle chain.

    Visualization (side view):

        (hip)●─────────●(knee)
             coxa        \\
                          \\ femur
                           \\
                            ●(ankle)
                            │
                            │ tibia
                            │
                            ●(foot)

These points define leg segments used for collision detection and visualization.
"""

import math
import numpy as np
from dataclasses import dataclass

# -----------------------------------------------------------------------------
# Robot Geometry Constants (from firmware/MARS_Hexapod/robot_config.h)
# -----------------------------------------------------------------------------

# Link lengths (mm)
COXA_LENGTH_MM   = 41.70
FEMUR_LENGTH_MM  = 80.00
TIBIA_LENGTH_MM  = 133.78

NUM_LEGS = 6

# Leg indices (match firmware)
LEG_LF, LEG_LM, LEG_LR = 0, 1, 2
LEG_RF, LEG_RM, LEG_RR = 3, 4, 5

# Coxa origin offsets in body frame (mm)
# Canonical order: LF, LM, LR, RF, RM, RR
# From robot_config.h
COXA_OFFSET_X = [-65.5959, -86.0000, -65.5959,  65.5959,  86.0000,  65.5959]
COXA_OFFSET_Z = [ 88.1630,   0.0000, -88.1630,  88.1630,   0.0000, -88.1630]
# Y offset is 0 in body frame for coxa mounting plane

# Base mounting angles (radians) derived from COXA_OFFSET
# 0 deg = +Z (Forward), +90 deg = +X (Right)
# Verified with atan2(x, z):
# LF: -36.65, LM: -90, LR: -143.35
# RF: +36.65, RM: +90, RR: +143.35
MOUNT_ANGLE_RAD = [
    math.atan2(-65.5959, 88.1630),   # LF
    math.atan2(-86.0000, 0.0000),    # LM
    math.atan2(-65.5959, -88.1630),  # LR
    math.atan2(65.5959, 88.1630),    # RF
    math.atan2(86.0000, 0.0000),     # RM
    math.atan2(65.5959, -88.1630)    # RR
]

@dataclass
class LegPose:
    """Joint angles in degrees."""
    coxa: float
    femur: float
    tibia: float

@dataclass
class LegPoints:
    """3D points of leg joints in Body Frame (mm)."""
    hip: np.ndarray    # Body attachment (Coxa axis)
    knee: np.ndarray   # Coxa-Femur joint
    ankle: np.ndarray  # Femur-Tibia joint
    foot: np.ndarray   # Tip of Tibia

def compute_ik(leg_idx: int, pt_foot_body: np.ndarray) -> LegPose:
    """
    Compute Inverse Kinematics for a foot target.
    Returns geometric joint angles (degrees).

    Input:
    - pt_foot_body: [x, y, z] target in Body Frame (mm).
      Convention: Y is UP (body center = 0, ground ~ -100).
    """
    # 0. Get Hip Position
    pt_hip = get_body_hip_position(leg_idx)
    
    # 1. Transform to Hip Frame (relative vector)
    # v_leg = Foot - Hip
    v_x = pt_foot_body[0] - pt_hip[0]
    v_y = pt_foot_body[1] - pt_hip[1] # y stays same (since hip y=0)
    v_z = pt_foot_body[2] - pt_hip[2]

    # 2. Coxa Angle (Yaw)
    # Calculate target angle of the leg vector in XZ plane
    yaw_target = math.atan2(v_x, v_z)  # 0=+Z(Front), 90=+X(Right)
    
    # Subtract mounting angle to get local servo angle
    # Normalize to -180...+180 range
    mount_rad = MOUNT_ANGLE_RAD[leg_idx]
    
    # Diff calculation handling wrapping
    coxa_rad = yaw_target - mount_rad
    while coxa_rad > math.pi: coxa_rad -= 2*math.pi
    while coxa_rad < -math.pi: coxa_rad += 2*math.pi
    
    # 3. Leg Plane Arithmetic
    # Horizontal distance from Hip center to Foot (projected on XZ)
    dist_hz_total = math.sqrt(v_x**2 + v_z**2)
    
    # Reach required for Femur+Tibia (planar horizontal component)
    # Subtract Coxa length
    dist_hz_limb = dist_hz_total - COXA_LENGTH_MM
    
    # Vertical distance is just local Y
    dist_v_limb = v_y
    
    # Total distance from Knee to Foot (planar D)
    D = math.sqrt(dist_hz_limb**2 + dist_v_limb**2)
    
    # Safety Check: Reachable?
    # If D > (Femur + Tibia), target is unreachable. Clamp magnitude.
    max_reach = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM
    if D > max_reach:
        D = max_reach
        # Should ideally scale H/V to constrained D, but for simple IK clamping D is enough for angle calculation logic
    
    # Law of Cosines for Femur angle relative to Chord D
    # Triangle: Femur(a), Tibia(b), Chord(D)
    # Angle at Knee (alpha1) inside the triangle
    # cos(alpha1) = (a^2 + D^2 - b^2) / (2*a*D)
    
    a = FEMUR_LENGTH_MM
    b = TIBIA_LENGTH_MM
    
    # Check triangle validity (should be covered by max_reach, but also check collisions/min fold)
    cos_alpha_triangle = (a**2 + D**2 - b**2) / (2 * a * D)
    cos_alpha_triangle = max(-1.0, min(1.0, cos_alpha_triangle)) # Clamp safety
    alpha_triangle = math.acos(cos_alpha_triangle)
    
    # Angle of the Chord D from horizontal
    # tan(theta) = V / H
    angle_chord = math.atan2(dist_v_limb, dist_hz_limb)
    
    # Femur Angle (from Horizontal) = Chord Angle + Triangle Angle
    # Note: If Tibia bends "in" (standard configurations), Femur is pitched "up" relative to chord
    # Standard Hexapod: Knee is Up-Out.
    femur_rad = angle_chord + alpha_triangle
    
    # Tibia Angle (from Femur)
    # Interior angle at Elbow (gamma)
    # cos(gamma) = (a^2 + b^2 - D^2) / (2*a*b)
    cos_gamma = (a**2 + b**2 - D**2) / (2 * a * b)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    
    # Tibia servo angle usually defined relative to Femur extension
    # If leg is straight, gamma = 180 (pi). Servo angle = 0?
    # If leg is bent 90 deg, gamma = 90.
    # Standard convention: Tibia 0 = Perpendicular to Femur? Or Straight?
    # Firmware "tibia_cdeg" calculation:
    # float beta = acosf((b*b + a*a - D*D) / (2*a*b));  <-- This is gamma (angle opposite to D)
    # float gamma_fw = PI - beta; <-- This is exterior angle (deviation from straight)
    
    # So relative angle from straight line:
    tibia_rad = -(math.pi - gamma)  # Negative because it bends "down/in"
    
    return LegPose(
        math.degrees(coxa_rad),
        math.degrees(femur_rad),
        math.degrees(tibia_rad)
    )

def get_body_hip_position(leg_idx: int) -> np.ndarray:
    """Get the [x, y, z] position of the hip (coxa servo axis) in body frame."""
    return np.array([COXA_OFFSET_X[leg_idx], 0.0, COXA_OFFSET_Z[leg_idx]])

# Servo home position in centidegrees (120.00 degrees)
HOME_CD = 12000
HOME_DEG = HOME_CD / 100.0  # 120.0 degrees

# Leg base angles (degrees, used as yaw offset from +Z axis)
# These define the direction each leg points outward from the body
# Note: In FK, cos(yaw) -> Z, sin(yaw) -> X, so 0° = forward, 90° = right
LEG_BASE_ANGLES = [315.0, 270.0, 225.0, 45.0, 90.0, 135.0]  # LF, LM, LR, RF, RM, RR

# Coxa offset (X, Z) pairs for convenient tuple access in FK
COXA_OFFSET_XZ = [
    (-65.5959,  88.1630),   # LF
    (-86.0000,   0.0000),   # LM
    (-65.5959, -88.1630),   # LR
    ( 65.5959,  88.1630),   # RF
    ( 86.0000,   0.0000),   # RM
    ( 65.5959, -88.1630),   # RR
]


def ik_to_servo_angles(leg_idx: int, pose: LegPose) -> tuple:
    """
    Convert IK output (relative angles centered at 0) to absolute servo angles
    (centered at 120°) for use with FK.
    
    The IK returns angles in the "relative" convention:
    - coxa: 0 = pointing along mount direction
    - femur: angle from horizontal
    - tibia: angle relative to femur
    
    Servos use "absolute" convention centered at 120°.
    Left-side legs (0,1,2) have mirrored mounting.
    
    Args:
        leg_idx: Leg index 0-5
        pose: LegPose with coxa, femur, tibia in degrees (IK output)
        
    Returns:
        (coxa_deg, femur_deg, tibia_deg) as absolute servo angles
    """
    is_left = leg_idx < 3
    
    # Coxa: IK gives deviation from mount angle, servo wants absolute
    coxa_servo = HOME_DEG + pose.coxa
    
    # Femur and Tibia: left/right mirroring
    # The IK tibia angle convention: negative = bent inward
    # The FK beta convention: negative = bent inward (from tibia_rel)
    # But FK computes: beta = -tibia_rel, so tibia_rel must be opposite of IK tibia
    if is_left:
        # Left side: servos are mirrored
        femur_servo = HOME_DEG - pose.femur
        tibia_servo = HOME_DEG + pose.tibia  # Opposite sign from femur
    else:
        # Right side
        femur_servo = HOME_DEG + pose.femur
        tibia_servo = HOME_DEG - pose.tibia  # Opposite sign from femur
    
    return (coxa_servo, femur_servo, tibia_servo)


def fk_leg(leg_idx: int, coxa_deg: float, femur_deg: float, tibia_deg: float,
           absolute_angles: bool = False) -> LegPoints:
    """
    Compute Forward Kinematics for a single leg.
    Returns critical points (hip, knee, ankle, foot) in Body Frame.
    
    This matches the firmware's fk_leg_body() function.
    
    Coordinate system (body frame):
    - X: Right (+) / Left (-)
    - Y: Up (+) / Down (-)  
    - Z: Forward (+) / Backward (-)
    
    Args:
        leg_idx: Leg index 0-5 (LF, LM, LR, RF, RM, RR)
        coxa_deg: Coxa joint angle in degrees
        femur_deg: Femur joint angle in degrees
        tibia_deg: Tibia joint angle in degrees
        absolute_angles: If True, angles are absolute servo positions (centered at 120°)
                        and FK uses display convention (LEG_BASE_ANGLES for yaw).
                        If False (default), angles are from IK (centered at 0°)
                        and FK uses IK convention (MOUNT_ANGLE_RAD for yaw).
    
    Returns:
        LegPoints with hip, knee, ankle, foot as numpy arrays.
    """
    # Determine yaw base and relative angles based on input type
    if absolute_angles:
        # Display convention: servo angles centered at 120°, yaw from LEG_BASE_ANGLES
        coxa_rel = coxa_deg - HOME_DEG
        femur_rel = femur_deg - HOME_DEG
        tibia_rel = tibia_deg - HOME_DEG
        yaw_base_rad = math.radians(LEG_BASE_ANGLES[leg_idx])
    else:
        # IK convention: angles centered at 0°, yaw from MOUNT_ANGLE_RAD
        coxa_rel = coxa_deg  # IK coxa is already relative to mount angle
        femur_rel = femur_deg
        tibia_rel = -tibia_deg  # IK tibia sign convention is opposite to FK
        yaw_base_rad = MOUNT_ANGLE_RAD[leg_idx]
    
    # Left-side legs (0,1,2) have mirrored servo mounting for femur/tibia
    is_left = leg_idx < 3
    if is_left and absolute_angles:
        # Only flip for display angles; IK angles are already in correct convention
        femur_rel = -femur_rel
        tibia_rel = -tibia_rel
    
    # Convert to radians
    yaw = yaw_base_rad + math.radians(coxa_rel)
    
    # Femur has 90° offset: at home (rel=0), femur points horizontal
    alpha = math.radians(femur_rel + 90.0)
    # Tibia rotation is reversed relative to femur
    beta = math.radians(-tibia_rel)
    
    # Coxa origin in body frame
    coxa_origin_x, coxa_origin_z = COXA_OFFSET_XZ[leg_idx]
    pt_hip = np.array([coxa_origin_x, 0.0, coxa_origin_z])
    
    # Link lengths
    a = FEMUR_LENGTH_MM
    b = TIBIA_LENGTH_MM
    
    # Knee (end of coxa): coxa length from hip in yaw direction, Y=0
    pt_knee = np.array([
        coxa_origin_x + COXA_LENGTH_MM * math.sin(yaw),
        0.0,
        coxa_origin_z + COXA_LENGTH_MM * math.cos(yaw)
    ])
    
    # Planar projection in leg's sagittal plane
    # alpha=90° at home: sin(90)=1 (horizontal), cos(90)=0 (no vertical)
    # Y is vertical: up is +Y, robot foot down → negative
    
    # Ankle (end of femur)
    femur_y = -(a * math.cos(alpha))  # vertical (negative = down)
    femur_r = a * math.sin(alpha)     # horizontal extension
    
    pt_ankle = np.array([
        pt_knee[0] + femur_r * math.sin(yaw),
        pt_knee[1] + femur_y,
        pt_knee[2] + femur_r * math.cos(yaw)
    ])
    
    # Foot (end of tibia)
    y_foot = -(a * math.cos(alpha) + b * math.cos(alpha + beta))
    R = a * math.sin(alpha) + b * math.sin(alpha + beta)
    rproj = COXA_LENGTH_MM + R
    
    pt_foot = np.array([
        coxa_origin_x + rproj * math.sin(yaw),
        y_foot,
        coxa_origin_z + rproj * math.cos(yaw)
    ])
    
    return LegPoints(pt_hip, pt_knee, pt_ankle, pt_foot)


def fk_leg_points(
    leg_idx: int,
    coxa_deg: float,
    femur_deg: float,
    tibia_deg: float,
) -> list:
    """
    Compute FK and return list of 4 (x, y, z) tuples for visualization.
    
    This is a convenience wrapper around fk_leg() for display use.
    Input angles are absolute servo positions (centered at 120°).
    
    Args:
        leg_idx: Leg index 0-5
        coxa_deg: Coxa joint angle in degrees (absolute servo position)
        femur_deg: Femur joint angle in degrees (absolute servo position)
        tibia_deg: Tibia joint angle in degrees (absolute servo position)
    
    Returns:
        List of 4 (x, y, z) tuples: [coxa_origin, femur_origin, tibia_origin, foot]
    """
    pts = fk_leg(leg_idx, coxa_deg, femur_deg, tibia_deg, absolute_angles=True)
    return [
        (float(pts.hip[0]), float(pts.hip[1]), float(pts.hip[2])),
        (float(pts.knee[0]), float(pts.knee[1]), float(pts.knee[2])),
        (float(pts.ankle[0]), float(pts.ankle[1]), float(pts.ankle[2])),
        (float(pts.foot[0]), float(pts.foot[1]), float(pts.foot[2])),
    ]

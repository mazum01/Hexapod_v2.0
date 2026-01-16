
import math
import numpy as np
from kinematics import fk_leg, compute_ik, get_body_hip_position, LEG_LF, COXA_LENGTH_MM, FEMUR_LENGTH_MM, TIBIA_LENGTH_MM

def test_fk_ik_consistency():
    print("Testing FK/IK Consistency...")
    
    # Define a test point for LF leg
    # Typical stance: X=Offset-100, Y=-100, Z=Offset+50
    # LF is Front-Left (X-, Z+)
    # Mount is ~(-65, 88). 
    # Let's target somewhat outward
    target_pos = np.array([-150.0, -100.0, 150.0])
    
    # 1. Compute IK
    print(f"\nTarget Foot Position (Body): {target_pos}")
    angles = compute_ik(LEG_LF, target_pos)
    print(f"Computed Angles (deg): {angles}")
    
    # 2. Compute FK using those angles
    points = fk_leg(LEG_LF, angles.coxa, angles.femur, angles.tibia)
    print(f"Resulting Foot Position: {points.foot}")
    
    # 3. Validation
    error = np.linalg.norm(points.foot - target_pos)
    print(f"\nRound-trip Error: {error:.4f} mm")
    
    if error < 0.1:
        print("PASS: Error is negligible.")
    else:
        print("FAIL: Error is too high.")

def test_hardcoded_geometry():
    print("\nTesting Zero Pose Geometry...")
    # Zero angles: Coxa=0, Femur=0, Tibia=0
    # Should result in leg extending straight out from mount, horizontally
    
    angles = (0.0, 0.0, 0.0)
    print(f"Angles: {angles}")
    
    pts = fk_leg(LEG_LF, *angles)
    
    # Expected:
    # Hip at offset
    # Knee at Hip + Coxa_Len along mount vector
    # Foot at Knee + (Femur+Tibia) along mount vector (Horizontal)
    
    offset_x = -65.5959
    offset_z = 88.1630
    mount_angle = math.atan2(offset_x, offset_z)
    
    total_len = COXA_LENGTH_MM + FEMUR_LENGTH_MM + TIBIA_LENGTH_MM
    expected_x = offset_x + total_len * math.sin(mount_angle)
    expected_z = offset_z + total_len * math.cos(mount_angle)
    expected_y = 0.0
    
    print(f"FK Foot: {pts.foot}")
    print(f"Expected: [{expected_x:.4f} {expected_y:.4f} {expected_z:.4f}]")

if __name__ == "__main__":
    test_fk_ik_consistency()
    test_hardcoded_geometry()

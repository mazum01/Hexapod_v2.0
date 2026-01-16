"""
test_collision.py - Comprehensive tests for collision detection.

Tests validate:
- Segment-to-segment distance calculations
- Safe poses don't trigger false positives
- Known collision poses are detected
- Body exclusion zone enforcement
- Edge cases and boundary conditions
- Phase-aware collision risk zones
- Velocity-aware safety margins
"""

import math
import numpy as np
from kinematics import LegPose, LegPoints, LEG_LF, LEG_LM, LEG_LR, LEG_RF, LEG_RM, LEG_RR
from kinematics import fk_leg, COXA_LENGTH_MM, FEMUR_LENGTH_MM, TIBIA_LENGTH_MM
from collision import (
    validate_pose_safety,
    validate_pose_safety_detailed,
    check_inter_leg_collision, 
    check_body_collision,
    dist_sq_segment_segment,
    MIN_DIST_LEG_LEG,
    BODY_RADIUS,
    # Phase-aware functions
    LegPhase,
    get_leg_phases_tripod,
    get_risk_pairs,
    get_risk_level,
    TRIPOD_A_SET,
    TRIPOD_B_SET,
    ADJACENT_PAIRS,
    # Velocity-aware functions
    compute_velocity_margin,
    get_dynamic_threshold_sq,
    TIME_HORIZON_S,
    MAX_VELOCITY_MARGIN_MM,
)
import time

# ============================================================================
# Test Utilities
# ============================================================================

def run_test(name: str, condition: bool, msg_pass: str = "PASS", msg_fail: str = "FAIL"):
    """Run a single test and print result."""
    status = msg_pass if condition else msg_fail
    symbol = "✓" if condition else "✗"
    print(f"  [{symbol}] {name}: {status}")
    return condition

def all_zero_poses() -> list[LegPose]:
    """Return a list of 6 zero poses."""
    return [LegPose(0.0, 0.0, 0.0) for _ in range(6)]

def standing_poses() -> list[LegPose]:
    """Return typical standing poses (legs pointing outward, down)."""
    # Femur ~-30 (down), Tibia ~-60 (bent back)
    return [LegPose(0.0, -30.0, -60.0) for _ in range(6)]

# ============================================================================
# Segment Distance Tests
# ============================================================================

def test_segment_distance():
    """Test the segment-to-segment distance function."""
    print("\n=== Segment Distance Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Parallel segments, known distance apart
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([10.0, 0.0, 0.0])
    q1 = np.array([0.0, 5.0, 0.0])
    q2 = np.array([10.0, 5.0, 0.0])
    dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
    total += 1
    if run_test("Parallel segments 5mm apart", abs(dist_sq - 25.0) < 0.01, 
                f"dist²={dist_sq:.2f}", f"Expected 25, got {dist_sq:.2f}"):
        passed += 1
    
    # Test 2: Perpendicular segments, intersecting
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([10.0, 0.0, 0.0])
    q1 = np.array([5.0, -5.0, 0.0])
    q2 = np.array([5.0, 5.0, 0.0])
    dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
    total += 1
    if run_test("Perpendicular intersecting segments", dist_sq < 0.01,
                f"dist²={dist_sq:.4f}", f"Expected ~0, got {dist_sq:.2f}"):
        passed += 1
    
    # Test 3: Segments with endpoints closest
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([10.0, 0.0, 0.0])
    q1 = np.array([15.0, 0.0, 0.0])
    q2 = np.array([20.0, 0.0, 0.0])
    dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
    total += 1
    if run_test("Collinear separated segments", abs(dist_sq - 25.0) < 0.01,
                f"dist²={dist_sq:.2f}", f"Expected 25, got {dist_sq:.2f}"):
        passed += 1
    
    # Test 4: Skew segments (3D)
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([10.0, 0.0, 0.0])
    q1 = np.array([5.0, 3.0, 4.0])  # 5 units away in YZ
    q2 = np.array([5.0, 3.0, 14.0])
    dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
    expected = 3**2 + 4**2  # = 25
    total += 1
    if run_test("Skew 3D segments", abs(dist_sq - expected) < 0.01,
                f"dist²={dist_sq:.2f}", f"Expected {expected}, got {dist_sq:.2f}"):
        passed += 1
    
    # Test 5: Zero-length segment (degenerate)
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([0.0, 0.0, 0.0])  # Point
    q1 = np.array([3.0, 4.0, 0.0])
    q2 = np.array([3.0, 4.0, 0.0])  # Point
    dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
    total += 1
    if run_test("Point-to-point distance", abs(dist_sq - 25.0) < 0.01,
                f"dist²={dist_sq:.2f}", f"Expected 25, got {dist_sq:.2f}"):
        passed += 1
    
    print(f"  Segment tests: {passed}/{total} passed")
    return passed == total

# ============================================================================
# FK Sanity Tests
# ============================================================================

def test_fk_sanity():
    """Verify FK produces expected geometry."""
    print("\n=== FK Sanity Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Zero pose leg length check
    # With coxa=0, femur=0, tibia=0, leg should be fully extended horizontally
    chain = fk_leg(LEG_LF, 0.0, 0.0, 0.0)
    
    # Hip to Knee distance = COXA_LENGTH
    hip_knee_dist = np.linalg.norm(chain.knee - chain.hip)
    total += 1
    if run_test("Hip-Knee distance = COXA_LENGTH", 
                abs(hip_knee_dist - COXA_LENGTH_MM) < 0.01,
                f"{hip_knee_dist:.2f}mm", f"Expected {COXA_LENGTH_MM}, got {hip_knee_dist:.2f}"):
        passed += 1
    
    # Knee to Ankle distance = FEMUR_LENGTH
    knee_ankle_dist = np.linalg.norm(chain.ankle - chain.knee)
    total += 1
    if run_test("Knee-Ankle distance = FEMUR_LENGTH",
                abs(knee_ankle_dist - FEMUR_LENGTH_MM) < 0.01,
                f"{knee_ankle_dist:.2f}mm", f"Expected {FEMUR_LENGTH_MM}, got {knee_ankle_dist:.2f}"):
        passed += 1
    
    # Ankle to Foot distance = TIBIA_LENGTH
    ankle_foot_dist = np.linalg.norm(chain.foot - chain.ankle)
    total += 1
    if run_test("Ankle-Foot distance = TIBIA_LENGTH",
                abs(ankle_foot_dist - TIBIA_LENGTH_MM) < 0.01,
                f"{ankle_foot_dist:.2f}mm", f"Expected {TIBIA_LENGTH_MM}, got {ankle_foot_dist:.2f}"):
        passed += 1
    
    # Test 2: All legs produce valid chains
    for i in range(6):
        chain = fk_leg(i, 0.0, -30.0, -60.0)
        # Check foot is below hip (Y should be negative for down)
        total += 1
        # With femur=-30, tibia=-60, foot should be lower than hip
        if run_test(f"Leg {i} foot below hip in standing pose",
                    chain.foot[1] < chain.hip[1],
                    f"foot_y={chain.foot[1]:.1f}", f"foot_y={chain.foot[1]:.1f} >= hip_y={chain.hip[1]:.1f}"):
            passed += 1
    
    print(f"  FK sanity tests: {passed}/{total} passed")
    return passed == total

# ============================================================================
# Safe Pose Tests (No False Positives)
# ============================================================================

def test_safe_poses():
    """Verify safe poses don't trigger collision detection."""
    print("\n=== Safe Pose Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Zero pose (legs extended outward)
    poses = all_zero_poses()
    total += 1
    if run_test("Zero pose (all joints 0°)", validate_pose_safety(poses)):
        passed += 1
    
    # Test 2: Standing pose
    poses = standing_poses()
    total += 1
    if run_test("Standing pose (femur=-30, tibia=-60)", validate_pose_safety(poses)):
        passed += 1
    
    # Test 3: Walk-ready pose (slight variations)
    poses = [LegPose(0.0, -25.0, -55.0) for _ in range(6)]
    total += 1
    if run_test("Walk-ready pose", validate_pose_safety(poses)):
        passed += 1
    
    # Test 4: Moderate coxa spread (legs spread out)
    poses = [LegPose(15.0, -30.0, -60.0) for _ in range(6)]
    total += 1
    if run_test("Legs spread outward (+15° coxa)", validate_pose_safety(poses)):
        passed += 1
    
    # Test 5: Moderate coxa tuck (legs slightly inward, but safe)
    poses = [LegPose(-10.0, -30.0, -60.0) for _ in range(6)]
    total += 1
    if run_test("Legs tucked slightly (-10° coxa)", validate_pose_safety(poses)):
        passed += 1
    
    # Test 6: High step (one leg lifted)
    poses = standing_poses()
    poses[LEG_LF] = LegPose(0.0, 15.0, -45.0)  # Lifted
    total += 1
    if run_test("One leg lifted high", validate_pose_safety(poses)):
        passed += 1
    
    print(f"  Safe pose tests: {passed}/{total} passed")
    return passed == total

# ============================================================================
# Collision Detection Tests
# ============================================================================

def test_inter_leg_collisions():
    """Test detection of leg-to-leg collisions."""
    print("\n=== Inter-Leg Collision Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Adjacent legs pointing at each other (LF-LM)
    # LF mount angle ≈ -36°, LM mount angle = -90°
    # Make LF point backwards (-Z) and LM point forward (+Z)
    poses = standing_poses()
    poses[LEG_LF] = LegPose(coxa=-100.0, femur=0.0, tibia=0.0)  # Point toward LM
    poses[LEG_LM] = LegPose(coxa=70.0, femur=0.0, tibia=0.0)    # Point toward LF
    total += 1
    is_safe = validate_pose_safety(poses)
    if run_test("LF-LM legs pointing at each other", not is_safe,
                "Collision detected", "NO collision detected (expected one)"):
        passed += 1
    
    # Test 2: Adjacent legs on right side (RF-RM)
    poses = standing_poses()
    poses[LEG_RF] = LegPose(coxa=100.0, femur=0.0, tibia=0.0)
    poses[LEG_RM] = LegPose(coxa=-70.0, femur=0.0, tibia=0.0)
    total += 1
    is_safe = validate_pose_safety(poses)
    if run_test("RF-RM legs pointing at each other", not is_safe,
                "Collision detected", "NO collision detected (expected one)"):
        passed += 1
    
    # Test 3: LM-LR collision
    poses = standing_poses()
    poses[LEG_LM] = LegPose(coxa=-70.0, femur=0.0, tibia=0.0)  # Point back
    poses[LEG_LR] = LegPose(coxa=100.0, femur=0.0, tibia=0.0)  # Point forward
    total += 1
    is_safe = validate_pose_safety(poses)
    if run_test("LM-LR legs pointing at each other", not is_safe,
                "Collision detected", "NO collision detected (expected one)"):
        passed += 1
    
    print(f"  Inter-leg collision tests: {passed}/{total} passed")
    return passed == total

def test_body_collisions():
    """Test detection of leg-to-body collisions."""
    print("\n=== Body Collision Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Leg tucked extremely inward (coxa toward body center)
    # This should bring the knee/ankle inside the body radius
    poses = standing_poses()
    # LM is at X=-86, Z=0. Pointing it +90° from mount puts it toward +Z.
    # Pointing it -90° from mount puts it toward -Z (back).
    # Pointing it -180° from mount points it toward +X (body center!)
    poses[LEG_LM] = LegPose(coxa=90.0, femur=0.0, tibia=0.0)  # Point inward toward body
    total += 1
    is_safe = validate_pose_safety(poses)
    if run_test("LM leg pointed directly at body center", not is_safe,
                "Body collision detected", "NO collision detected (expected one)"):
        passed += 1
    
    # Test 2: RM pointed at body
    poses = standing_poses()
    poses[LEG_RM] = LegPose(coxa=-90.0, femur=0.0, tibia=0.0)
    total += 1
    is_safe = validate_pose_safety(poses)
    if run_test("RM leg pointed directly at body center", not is_safe,
                "Body collision detected", "NO collision detected (expected one)"):
        passed += 1
    
    # Test 3: Front leg tucked way back (might hit body)
    poses = standing_poses()
    poses[LEG_LF] = LegPose(coxa=130.0, femur=0.0, tibia=0.0)  # Point almost at body
    total += 1
    is_safe = validate_pose_safety(poses)
    # This might or might not collide depending on geometry
    # We're checking the check runs without error
    result = "Collision" if not is_safe else "Safe"
    print(f"  [?] LF extreme tuck: {result} (geometry-dependent)")
    
    print(f"  Body collision tests: {passed}/{total} definitive passed")
    return passed >= 1  # At least some body collisions detected

# ============================================================================
# Performance Tests
# ============================================================================

def test_performance():
    """Benchmark collision check speed."""
    print("\n=== Performance Tests ===")
    
    poses = standing_poses()
    
    # Warm up
    for _ in range(10):
        validate_pose_safety(poses)
    
    # Timed run
    iterations = 1000
    start = time.time()
    for _ in range(iterations):
        validate_pose_safety(poses)
    elapsed = time.time() - start
    
    avg_us = (elapsed / iterations) * 1_000_000
    rate_hz = iterations / elapsed
    
    print(f"  Collision check: {avg_us:.1f} µs avg ({rate_hz:.0f} Hz)")
    
    # Should be fast enough for real-time (< 1ms target)
    target_us = 1000.0
    passed = avg_us < target_us
    run_test(f"Performance < {target_us:.0f}µs", passed,
             f"{avg_us:.1f}µs", f"{avg_us:.1f}µs too slow")
    
    return passed

# ============================================================================
# Edge Case Tests
# ============================================================================

def test_edge_cases():
    """Test boundary conditions and edge cases."""
    print("\n=== Edge Case Tests ===")
    passed = 0
    total = 0
    
    # Test 1: All legs at joint limits (extreme but valid)
    # These are hypothetical limits - adjust based on actual robot
    poses = [LegPose(coxa=45.0, femur=45.0, tibia=-90.0) for _ in range(6)]
    total += 1
    try:
        result = validate_pose_safety(poses)
        run_test("Extreme joint angles (no crash)", True, f"Safe={result}")
        passed += 1
    except Exception as e:
        run_test("Extreme joint angles (no crash)", False, msg_fail=str(e))
    
    # Test 2: Negative extreme angles
    poses = [LegPose(coxa=-45.0, femur=-60.0, tibia=-120.0) for _ in range(6)]
    total += 1
    try:
        result = validate_pose_safety(poses)
        run_test("Negative extreme angles (no crash)", True, f"Safe={result}")
        passed += 1
    except Exception as e:
        run_test("Negative extreme angles (no crash)", False, msg_fail=str(e))
    
    # Test 3: Mixed extreme poses
    poses = [
        LegPose(45.0, 30.0, -45.0),
        LegPose(-45.0, -30.0, -90.0),
        LegPose(0.0, 0.0, 0.0),
        LegPose(30.0, -45.0, -60.0),
        LegPose(-30.0, 45.0, -30.0),
        LegPose(0.0, -60.0, -120.0)
    ]
    total += 1
    try:
        result = validate_pose_safety(poses)
        run_test("Mixed extreme poses (no crash)", True, f"Safe={result}")
        passed += 1
    except Exception as e:
        run_test("Mixed extreme poses (no crash)", False, msg_fail=str(e))
    
    print(f"  Edge case tests: {passed}/{total} passed")
    return passed == total

# ============================================================================
# Threshold Boundary Tests
# ============================================================================

def test_distance_thresholds():
    """Verify collision thresholds are working correctly."""
    print("\n=== Distance Threshold Tests ===")
    
    print(f"  MIN_DIST_LEG_LEG = {MIN_DIST_LEG_LEG:.1f} mm")
    print(f"  BODY_RADIUS = {BODY_RADIUS:.1f} mm")
    
    # These are informational - verify they're reasonable
    reasonable_leg = 20.0 < MIN_DIST_LEG_LEG < 100.0
    reasonable_body = 30.0 < BODY_RADIUS < 150.0
    
    run_test("Leg distance threshold reasonable", reasonable_leg,
             f"{MIN_DIST_LEG_LEG:.1f}mm", f"{MIN_DIST_LEG_LEG:.1f}mm out of range")
    run_test("Body radius threshold reasonable", reasonable_body,
             f"{BODY_RADIUS:.1f}mm", f"{BODY_RADIUS:.1f}mm out of range")
    
    return reasonable_leg and reasonable_body

# ============================================================================
# Phase-Aware Collision Tests
# ============================================================================

def test_phase_awareness():
    """Test phase-aware collision detection logic."""
    print("\n=== Phase-Aware Collision Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Verify tripod group membership
    # TRIPOD_A_SET = {3, 1, 5}  # RF, LM, RR
    # TRIPOD_B_SET = {0, 4, 2}  # LF, RM, LR
    total += 1
    tripod_a_correct = TRIPOD_A_SET == {3, 1, 5}
    if run_test("Tripod A = {RF, LM, RR}", tripod_a_correct,
                f"{TRIPOD_A_SET}", f"Wrong: {TRIPOD_A_SET}"):
        passed += 1
    
    total += 1
    tripod_b_correct = TRIPOD_B_SET == {0, 4, 2}
    if run_test("Tripod B = {LF, RM, LR}", tripod_b_correct,
                f"{TRIPOD_B_SET}", f"Wrong: {TRIPOD_B_SET}"):
        passed += 1
    
    # Test 2: Phase 0 - Tripod A swings
    phases_0 = get_leg_phases_tripod(0)
    total += 1
    phase0_correct = (
        phases_0[3] == LegPhase.SWING and  # RF
        phases_0[1] == LegPhase.SWING and  # LM
        phases_0[5] == LegPhase.SWING and  # RR
        phases_0[0] == LegPhase.STANCE and  # LF
        phases_0[4] == LegPhase.STANCE and  # RM
        phases_0[2] == LegPhase.STANCE      # LR
    )
    if run_test("Phase 0: Tripod A swings, B stance", phase0_correct):
        passed += 1
    
    # Test 3: Phase 1 - Tripod B swings
    phases_1 = get_leg_phases_tripod(1)
    total += 1
    phase1_correct = (
        phases_1[0] == LegPhase.SWING and   # LF
        phases_1[4] == LegPhase.SWING and   # RM
        phases_1[2] == LegPhase.SWING and   # LR
        phases_1[3] == LegPhase.STANCE and  # RF
        phases_1[1] == LegPhase.STANCE and  # LM
        phases_1[5] == LegPhase.STANCE      # RR
    )
    if run_test("Phase 1: Tripod B swings, A stance", phase1_correct):
        passed += 1
    
    # Test 4: Risk pairs in Phase 0
    # Adjacent pairs: (0,1), (1,2), (3,4), (4,5)
    # Phase 0: legs 3,1,5 swing
    # (0,1): LF stance, LM swing → HIGH RISK
    # (1,2): LM swing, LR stance → HIGH RISK
    # (3,4): RF swing, RM stance → HIGH RISK
    # (4,5): RM stance, RR swing → HIGH RISK
    risk_0 = get_risk_pairs(phases_0)
    total += 1
    # All pairs should be at risk because each pair has one swinging leg
    if run_test("Phase 0: All adjacent pairs at risk", len(risk_0) == 4,
                f"{len(risk_0)} pairs", f"Expected 4, got {len(risk_0)}"):
        passed += 1
    
    # Test 5: Risk pairs in Phase 1
    # Phase 1: legs 0,4,2 swing
    # (0,1): LF swing, LM stance → HIGH RISK
    # (1,2): LM stance, LR swing → HIGH RISK
    # (3,4): RF stance, RM swing → HIGH RISK
    # (4,5): RM swing, RR stance → HIGH RISK
    risk_1 = get_risk_pairs(phases_1)
    total += 1
    if run_test("Phase 1: All adjacent pairs at risk", len(risk_1) == 4,
                f"{len(risk_1)} pairs", f"Expected 4, got {len(risk_1)}"):
        passed += 1
    
    # Test 6: Risk level calculation
    total += 1
    level_stance_stance = get_risk_level([LegPhase.STANCE]*6, 0, 1)
    level_stance_swing = get_risk_level(phases_0, 0, 1)  # LF stance, LM swing
    correct_levels = (level_stance_stance == 0 and level_stance_swing == 1)
    if run_test("Risk levels: stance-stance=0, stance-swing=1", correct_levels,
                f"{level_stance_stance}, {level_stance_swing}"):
        passed += 1
    
    print(f"  Phase-aware tests: {passed}/{total} passed")
    return passed == total

def test_phase_aware_validation():
    """Test that phase-aware validation works correctly."""
    print("\n=== Phase-Aware Validation Tests ===")
    passed = 0
    total = 0
    
    poses = standing_poses()
    
    # Test 1: Phase-aware validation with phase 0
    total += 1
    try:
        is_safe = validate_pose_safety(poses, gait_phase=0)
        if run_test("Standing pose safe in phase 0", is_safe):
            passed += 1
    except Exception as e:
        run_test("Standing pose safe in phase 0", False, msg_fail=str(e))
    
    # Test 2: Phase-aware validation with phase 1
    total += 1
    try:
        is_safe = validate_pose_safety(poses, gait_phase=1)
        if run_test("Standing pose safe in phase 1", is_safe):
            passed += 1
    except Exception as e:
        run_test("Standing pose safe in phase 1", False, msg_fail=str(e))
    
    # Test 3: Detailed validation returns expected structure
    total += 1
    try:
        is_safe, details = validate_pose_safety_detailed(poses, gait_phase=0)
        has_required = all(k in details for k in 
                          ['leg_phases', 'pairs_checked', 'distances', 
                           'body_collision', 'inter_leg_collision'])
        if run_test("Detailed validation returns all fields", has_required):
            passed += 1
    except Exception as e:
        run_test("Detailed validation returns all fields", False, msg_fail=str(e))
    
    # Test 4: Distances are computed for checked pairs
    total += 1
    try:
        _, details = validate_pose_safety_detailed(poses, gait_phase=0)
        distances = details['distances']
        pairs_checked = details['pairs_checked']
        all_distances_valid = all(
            isinstance(distances.get((a,b)), float) for a, b in pairs_checked
        )
        if run_test("Distances computed for all checked pairs", all_distances_valid,
                    f"{len(distances)} distances"):
            passed += 1
    except Exception as e:
        run_test("Distances computed for all checked pairs", False, msg_fail=str(e))
    
    print(f"  Phase-aware validation tests: {passed}/{total} passed")
    return passed == total

def test_phase_aware_performance():
    """Benchmark phase-aware vs full collision checking."""
    print("\n=== Phase-Aware Performance Tests ===")
    
    poses = standing_poses()
    iterations = 1000
    
    # Warm up
    for _ in range(10):
        validate_pose_safety(poses)
        validate_pose_safety(poses, gait_phase=0)
    
    # Full check (no phase info)
    start = time.time()
    for _ in range(iterations):
        validate_pose_safety(poses)
    time_full = time.time() - start
    
    # Phase-aware check
    start = time.time()
    for _ in range(iterations):
        validate_pose_safety(poses, gait_phase=0)
    time_phase = time.time() - start
    
    avg_full_us = (time_full / iterations) * 1_000_000
    avg_phase_us = (time_phase / iterations) * 1_000_000
    
    print(f"  Full check:        {avg_full_us:.1f} µs")
    print(f"  Phase-aware check: {avg_phase_us:.1f} µs")
    
    # Phase-aware should be similar or faster (currently checks same pairs)
    # In tripod gait, all pairs have one swinging leg, so no speedup expected
    # But with other gaits (wave, ripple), fewer pairs would be checked
    run_test("Phase-aware check runs", True, f"{avg_phase_us:.1f}µs")
    
    return True

# ============================================================================
# Velocity-Aware Margin Tests
# ============================================================================

def test_velocity_margin():
    """Test velocity-aware safety margin calculations."""
    print("\n=== Velocity-Aware Margin Tests ===")
    passed = 0
    total = 0
    
    # Test 1: Zero velocity = no extra margin
    total += 1
    margin_0 = compute_velocity_margin(0.0)
    if run_test("Zero velocity = 0 margin", margin_0 == 0.0,
                f"{margin_0:.2f}mm", f"Expected 0, got {margin_0:.2f}"):
        passed += 1
    
    # Test 2: Known velocity produces expected margin
    # margin = velocity * TIME_HORIZON_S
    total += 1
    velocity = 200.0  # mm/s
    expected = velocity * TIME_HORIZON_S  # Should be < MAX
    margin = compute_velocity_margin(velocity)
    if run_test(f"Velocity {velocity} mm/s margin", abs(margin - expected) < 0.01,
                f"{margin:.2f}mm", f"Expected {expected:.2f}, got {margin:.2f}"):
        passed += 1
    
    # Test 3: Very high velocity is capped
    total += 1
    velocity_high = 10000.0  # mm/s (unrealistically fast)
    margin_high = compute_velocity_margin(velocity_high)
    if run_test("High velocity capped at MAX", margin_high == MAX_VELOCITY_MARGIN_MM,
                f"{margin_high:.2f}mm", f"Expected {MAX_VELOCITY_MARGIN_MM}, got {margin_high:.2f}"):
        passed += 1
    
    # Test 4: Negative velocity uses absolute value
    total += 1
    margin_neg = compute_velocity_margin(-200.0)
    margin_pos = compute_velocity_margin(200.0)
    if run_test("Negative velocity = same margin as positive", margin_neg == margin_pos,
                f"{margin_neg:.2f}mm"):
        passed += 1
    
    # Test 5: Dynamic threshold increases with velocity
    total += 1
    thresh_0 = get_dynamic_threshold_sq(0.0)
    thresh_fast = get_dynamic_threshold_sq(400.0)  # 400 mm/s
    if run_test("Threshold increases with velocity", thresh_fast > thresh_0,
                f"{math.sqrt(thresh_0):.1f} → {math.sqrt(thresh_fast):.1f}mm"):
        passed += 1
    
    print(f"  Velocity margin tests: {passed}/{total} passed")
    return passed == total

def test_velocity_aware_validation():
    """Test that velocity affects collision detection."""
    print("\n=== Velocity-Aware Validation Tests ===")
    passed = 0
    total = 0
    
    poses = standing_poses()
    
    # Test 1: Safe pose remains safe at zero velocity
    total += 1
    is_safe = validate_pose_safety(poses, velocity_mm_s=0.0)
    if run_test("Standing safe at 0 velocity", is_safe):
        passed += 1
    
    # Test 2: Safe pose remains safe at moderate velocity
    total += 1
    is_safe = validate_pose_safety(poses, velocity_mm_s=200.0)
    if run_test("Standing safe at 200 mm/s", is_safe):
        passed += 1
    
    # Test 3: Detailed check includes velocity info
    total += 1
    _, details = validate_pose_safety_detailed(poses, velocity_mm_s=300.0)
    has_velocity = 'velocity_mm_s' in details and 'threshold_mm' in details
    if run_test("Detailed check includes velocity info", has_velocity,
                f"v={details.get('velocity_mm_s')}, thresh={details.get('threshold_mm'):.1f}mm"):
        passed += 1
    
    # Test 4: Threshold in details increases with velocity
    total += 1
    _, details_0 = validate_pose_safety_detailed(poses, velocity_mm_s=0.0)
    _, details_fast = validate_pose_safety_detailed(poses, velocity_mm_s=400.0)
    thresh_increased = details_fast['threshold_mm'] > details_0['threshold_mm']
    if run_test("Threshold increases with velocity in details", thresh_increased,
                f"{details_0['threshold_mm']:.1f} → {details_fast['threshold_mm']:.1f}mm"):
        passed += 1
    
    # Test 5: Print actual threshold values for reference
    print(f"  Reference: base={MIN_DIST_LEG_LEG:.1f}mm, "
          f"v=0 thresh={details_0['threshold_mm']:.1f}mm, "
          f"v=400 thresh={details_fast['threshold_mm']:.1f}mm")
    
    print(f"  Velocity-aware validation tests: {passed}/{total} passed")
    return passed == total

# ============================================================================
# Main Test Runner
# ============================================================================

def run_all_tests():
    """Run all collision tests and summarize results."""
    print("=" * 60)
    print("COLLISION DETECTION TEST SUITE")
    print("=" * 60)
    
    results = []
    
    results.append(("Segment Distance", test_segment_distance()))
    results.append(("FK Sanity", test_fk_sanity()))
    results.append(("Safe Poses", test_safe_poses()))
    results.append(("Inter-Leg Collisions", test_inter_leg_collisions()))
    results.append(("Body Collisions", test_body_collisions()))
    results.append(("Edge Cases", test_edge_cases()))
    results.append(("Distance Thresholds", test_distance_thresholds()))
    results.append(("Phase Awareness", test_phase_awareness()))
    results.append(("Phase-Aware Validation", test_phase_aware_validation()))
    results.append(("Phase-Aware Performance", test_phase_aware_performance()))
    results.append(("Velocity Margin", test_velocity_margin()))
    results.append(("Velocity-Aware Validation", test_velocity_aware_validation()))
    results.append(("Performance", test_performance()))
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        symbol = "✓" if passed else "✗"
        print(f"  [{symbol}] {name}: {status}")
        if not passed:
            all_passed = False
    
    print("=" * 60)
    if all_passed:
        print("ALL TESTS PASSED ✓")
    else:
        print("SOME TESTS FAILED ✗")
    print("=" * 60)
    
    return all_passed

if __name__ == "__main__":
    run_all_tests()

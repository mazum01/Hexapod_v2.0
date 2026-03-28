#!/usr/bin/env python3
"""
Test script to compare TripodGait and FreeGait foot trajectories.

This script generates foot positions over a gait cycle and analyzes the
line of action (direction of motion) for each leg.

No actual robot movement - just data generation and analysis.
"""

import sys
import math

# Add parent to path for imports
sys.path.insert(0, '/home/starter/OneDrive/Projects/Robots/Hexapod_v2.0/rPi controller')

from gait_engine import (
    TripodGait, GaitParams, GaitState,
    LEG_LF, LEG_LM, LEG_LR, LEG_RF, LEG_RM, LEG_RR,
    LEG_BASE_ROTATION_DEG, NUM_LEGS
)
from free_gait import FootPlacementPlanner, PlacementConfig

LEG_NAMES = ['LF', 'LM', 'LR', 'RF', 'RM', 'RR']


def test_tripod_gait_trajectory():
    """Generate TripodGait foot positions over one cycle."""
    print("=" * 70)
    print("TRIPOD GAIT - Forward Walking (heading=0, speed=0.5)")
    print("=" * 70)
    
    params = GaitParams(
        cycle_ms=2000,
        base_x_mm=100.0,
        base_y_mm=-120.0,
        step_len_mm=40.0,
        lift_mm=60.0,
        overlap_pct=5.0,
        speed_scale=0.5,
        heading_deg=0.0,
    )
    
    gait = TripodGait(params)
    gait.start()
    
    # Capture positions over one full cycle
    dt = 0.006  # 6ms per tick (166 Hz)
    num_ticks = int(params.cycle_ms / 1000.0 / dt)  # One full cycle
    
    # Track min/max X and Z for each leg to determine line of action
    leg_positions = {i: {'x': [], 'z': [], 'y': []} for i in range(NUM_LEGS)}
    
    for tick in range(num_ticks):
        feet = gait.tick(dt)
        for leg in range(NUM_LEGS):
            leg_positions[leg]['x'].append(feet[leg][0])
            leg_positions[leg]['y'].append(feet[leg][1])
            leg_positions[leg]['z'].append(feet[leg][2])
    
    # Analyze line of action for each leg
    print("\nFoot Position Ranges (over one cycle):")
    print("-" * 70)
    print(f"{'Leg':<4} {'X min':>10} {'X max':>10} {'ΔX':>10} {'Z min':>10} {'Z max':>10} {'ΔZ':>10}")
    print("-" * 70)
    
    for leg in range(NUM_LEGS):
        x_min = min(leg_positions[leg]['x'])
        x_max = max(leg_positions[leg]['x'])
        z_min = min(leg_positions[leg]['z'])
        z_max = max(leg_positions[leg]['z'])
        delta_x = x_max - x_min
        delta_z = z_max - z_min
        
        print(f"{LEG_NAMES[leg]:<4} {x_min:>10.2f} {x_max:>10.2f} {delta_x:>10.2f} {z_min:>10.2f} {z_max:>10.2f} {delta_z:>10.2f}")
    
    print("\nLine of Action Analysis:")
    print("-" * 70)
    for leg in range(NUM_LEGS):
        x_min = min(leg_positions[leg]['x'])
        x_max = max(leg_positions[leg]['x'])
        z_min = min(leg_positions[leg]['z'])
        z_max = max(leg_positions[leg]['z'])
        delta_x = x_max - x_min
        delta_z = z_max - z_min
        
        # Calculate angle of motion (0 = pure Z/forward, 90 = pure X/lateral)
        if delta_z > 0.01:
            angle_deg = math.degrees(math.atan2(delta_x, delta_z))
        else:
            angle_deg = 90.0 if delta_x > 0.01 else 0.0
        
        # Determine primary direction
        if delta_x > delta_z * 2:
            direction = "LATERAL (X-axis) ← WRONG for forward walk!"
        elif delta_z > delta_x * 2:
            direction = "FORWARD (Z-axis) ← Correct"
        else:
            direction = f"DIAGONAL ({angle_deg:.1f}° from Z)"
        
        print(f"{LEG_NAMES[leg]}: ΔX={delta_x:.2f}mm, ΔZ={delta_z:.2f}mm → {direction}")
    
    return leg_positions


def test_freegait_planner():
    """Test FreeGait foot placement planner directly."""
    print("\n" + "=" * 70)
    print("FREE GAIT PLANNER - Forward Walking (heading=0, speed=0.5)")
    print("=" * 70)
    
    config = PlacementConfig(
        base_x_mm=100.0,
        base_y_mm=-120.0,
        step_len_mm=40.0,
        max_step_len_mm=60.0,
    )
    
    planner = FootPlacementPlanner(config)
    
    print("\nNeutral Positions:")
    print("-" * 50)
    print(f"{'Leg':<4} {'X':>10} {'Y':>10} {'Z':>10}")
    print("-" * 50)
    for leg in range(NUM_LEGS):
        neutral = planner.get_neutral_position(leg)
        print(f"{LEG_NAMES[leg]:<4} {neutral.x:>10.2f} {neutral.y:>10.2f} {neutral.z:>10.2f}")
    
    print("\nSwing Targets (heading=0, speed=0.5):")
    print("-" * 50)
    print(f"{'Leg':<4} {'X':>10} {'Y':>10} {'Z':>10} {'ΔX from neutral':>18} {'ΔZ from neutral':>18}")
    print("-" * 50)
    for leg in range(NUM_LEGS):
        neutral = planner.get_neutral_position(leg)
        target = planner.plan_foot_placement(leg, heading_deg=0.0, speed_scale=0.5)
        delta_x = target.x - neutral.x
        delta_z = target.z - neutral.z
        # Note: target.z is NOT relative to neutral.z in current implementation
        print(f"{LEG_NAMES[leg]:<4} {target.x:>10.2f} {target.y:>10.2f} {target.z:>10.2f} {delta_x:>18.2f} {target.z:>18.2f}")
    
    print("\nLine of Action Analysis (from neutral to target):")
    print("-" * 70)
    for leg in range(NUM_LEGS):
        neutral = planner.get_neutral_position(leg)
        target = planner.plan_foot_placement(leg, heading_deg=0.0, speed_scale=0.5)
        delta_x = target.x - neutral.x
        # Since target.z is just stride_z_prime, the delta from 0 is the stride
        delta_z = target.z  # target.z = stride_z_prime, neutral.z = 0
        
        # Calculate angle of motion
        if abs(delta_z) > 0.01:
            angle_deg = math.degrees(math.atan2(abs(delta_x), abs(delta_z)))
        else:
            angle_deg = 90.0 if abs(delta_x) > 0.01 else 0.0
        
        # Determine primary direction
        if abs(delta_x) > abs(delta_z) * 2:
            direction = "LATERAL (X-axis) ← WRONG!"
        elif abs(delta_z) > abs(delta_x) * 2:
            direction = "FORWARD (Z-axis) ← Correct"
        else:
            direction = f"DIAGONAL ({angle_deg:.1f}° from Z)"
        
        print(f"{LEG_NAMES[leg]}: ΔX={delta_x:+.2f}mm, ΔZ={delta_z:+.2f}mm → {direction}")
    
    # Show the rotation calculation for each leg
    print("\n" + "=" * 70)
    print("ROTATION CALCULATION DEBUG (for forward walk, heading=0)")
    print("=" * 70)
    print(f"{'Leg':<4} {'Base Angle':>12} {'Sign':>6} {'walk_dir':>10} {'Full Angle':>12} {'sin':>10} {'cos':>10}")
    print("-" * 70)
    
    LEG_ROTATIONS_DEG = [-45.0, 0.0, 45.0, 45.0, 0.0, -45.0]
    heading_deg = 0.0
    
    for leg in range(NUM_LEGS):
        sign = 1.0 if leg in (0, 1, 2) else -1.0
        walk_dir = heading_deg / 90.0
        full_angle = LEG_ROTATIONS_DEG[leg] + sign * walk_dir * 90.0
        full_angle_rad = math.radians(full_angle)
        sin_val = math.sin(full_angle_rad)
        cos_val = math.cos(full_angle_rad)
        
        print(f"{LEG_NAMES[leg]:<4} {LEG_ROTATIONS_DEG[leg]:>12.1f}° {sign:>+6.1f} {walk_dir:>10.2f} {full_angle:>12.1f}° {sin_val:>10.4f} {cos_val:>10.4f}")


def test_tripod_apply_leg_rotation():
    """Show exactly what TripodGait's _apply_leg_rotation produces."""
    print("\n" + "=" * 70)
    print("TRIPOD GAIT _apply_leg_rotation DEBUG (heading=0, speed=0.5)")
    print("=" * 70)
    
    params = GaitParams(
        base_x_mm=100.0,
        base_y_mm=-120.0,
        step_len_mm=40.0,
        speed_scale=0.5,
        heading_deg=0.0,
    )
    
    gait = TripodGait(params)
    
    # Simulate what happens in _compute_foot_targets for stride calculation
    speed = 0.5
    heading = 0.0
    effective_step = params.step_len_mm * speed  # 20mm
    walk_dir = heading / 90.0  # 0.0
    
    print(f"\nParameters: step_len={params.step_len_mm}mm, speed={speed}, effective_step={effective_step}mm")
    print(f"Heading={heading}°, walk_dir={walk_dir}")
    print()
    print(f"{'Leg':<4} {'base_x':>10} {'base_z':>10} {'stride_x':>10} {'stride_z':>10} {'final_x':>10} {'final_z':>10}")
    print("-" * 70)
    
    from gait_engine import LEG_BASE_SIN, LEG_BASE_COS
    
    for leg in range(NUM_LEGS):
        # This matches TripodGait._apply_leg_rotation
        base_z = -params.base_x_mm * LEG_BASE_SIN[leg]
        base_x = params.base_x_mm * LEG_BASE_COS[leg]
        sign = 1.0 if leg in (LEG_LF, LEG_LM, LEG_LR) else -1.0
        full_angle = math.radians(LEG_BASE_ROTATION_DEG[leg]) + sign * math.radians(walk_dir * 90.0)
        cos_angle = math.cos(full_angle)
        sin_angle = math.sin(full_angle)
        
        # Stride at front of swing (stride_z = +effective_step)
        stride_z_input = effective_step  # The body-frame stride magnitude
        stride_x_prime = stride_z_input * sin_angle
        stride_z_prime = stride_z_input * cos_angle
        
        l_x_prime = base_x + stride_x_prime
        l_z_prime = stride_z_prime  # Note: NOT base_z + stride_z_prime
        
        print(f"{LEG_NAMES[leg]:<4} {base_x:>10.2f} {base_z:>10.2f} {stride_x_prime:>10.2f} {stride_z_prime:>10.2f} {l_x_prime:>10.2f} {l_z_prime:>10.2f}")
    
    print("\nKey observation: TripodGait's l_z_prime = stride_z_prime (NOT base_z + stride_z_prime)")
    print("This means Z output is purely the rotated stride, independent of base_z")


if __name__ == "__main__":
    test_tripod_gait_trajectory()
    test_freegait_planner()
    test_tripod_apply_leg_rotation()
    
    # Additional test: show swing start to end positions
    print("\n" + "=" * 70)
    print("SWING START vs END POSITIONS (TripodGait forward walk)")
    print("=" * 70)
    print("This shows where each leg STARTS and ENDS its swing phase")
    print("The swing goes from BACK (-stride) to FRONT (+stride)")
    print()
    
    from gait_engine import LEG_BASE_SIN, LEG_BASE_COS
    import math
    
    params_step = 40.0
    speed = 0.5
    effective_step = params_step * speed  # 20mm
    base_x_mm = 100.0
    
    print(f"{'Leg':<4} {'Start X':>10} {'Start Z':>10} {'End X':>10} {'End Z':>10} {'Motion ΔX':>10} {'Motion ΔZ':>10} {'Direction':>12}")
    print("-" * 90)
    
    for leg in range(NUM_LEGS):
        base_x = base_x_mm * LEG_BASE_COS[leg]
        sign = 1.0 if leg in (LEG_LF, LEG_LM, LEG_LR) else -1.0
        full_angle = math.radians(LEG_BASE_ROTATION_DEG[leg])  # No walk_dir for forward
        cos_angle = math.cos(full_angle)
        sin_angle = math.sin(full_angle)
        
        # At START of swing: foot is at BACK (stride = -effective_step)
        start_stride_x = -effective_step * sin_angle
        start_stride_z = -effective_step * cos_angle
        start_x = base_x + start_stride_x
        start_z = start_stride_z
        
        # At END of swing: foot is at FRONT (stride = +effective_step)
        end_stride_x = effective_step * sin_angle
        end_stride_z = effective_step * cos_angle
        end_x = base_x + end_stride_x
        end_z = end_stride_z
        
        # Motion during swing
        motion_dx = end_x - start_x
        motion_dz = end_z - start_z
        
        # Determine direction
        if abs(motion_dx) > abs(motion_dz) * 2:
            direction = "LATERAL"
        elif abs(motion_dz) > abs(motion_dx) * 2:
            direction = "FORWARD"
        else:
            angle = math.degrees(math.atan2(abs(motion_dx), abs(motion_dz)))
            direction = f"DIAG {angle:.0f}°"
        
        print(f"{LEG_NAMES[leg]:<4} {start_x:>10.2f} {start_z:>10.2f} {end_x:>10.2f} {end_z:>10.2f} {motion_dx:>10.2f} {motion_dz:>10.2f} {direction:>12}")
    
    print("\n" + "=" * 70)
    print("EXPECTED vs ACTUAL for LF when walking FORWARD:")
    print("=" * 70)
    print("If heading=0 means 'walk forward' (body moves +Z):")
    print("  - Feet should cycle from FRONT (+Z) to BACK (-Z) during stance")
    print("  - Feet should swing from BACK (-Z) to FRONT (+Z) during swing")
    print()
    print("For LF (left front at -45° base angle):")
    print("  - Current: swing moves from X=84.85 to X=56.57 (ΔX = -28.28)")
    print("             and from Z=-14.14 to Z=+14.14 (ΔZ = +28.28)")
    print("  - This is a 45° diagonal path!")
    print()
    print("But TripodGait works, so this 45° diagonal IS correct behavior.")
    print("The question is: does FreeGait's stance translation match?")
    
    # Now test full FreeGait cycle
    print("\n" + "=" * 70)
    print("FREE GAIT FULL CYCLE TEST (heading=0, speed=0.5)")
    print("=" * 70)
    
    from gait_engine import FreeGait, GaitParams as GP
    
    fg_params = GP(
        cycle_ms=2000,
        base_x_mm=100.0,
        base_y_mm=-120.0,
        step_len_mm=40.0,
        lift_mm=60.0,
        speed_scale=0.5,
        heading_deg=0.0,
    )
    
    freegait = FreeGait(fg_params)
    freegait.start()
    
    dt = 0.006
    num_ticks = 500  # ~3 seconds
    
    fg_positions = {i: {'x': [], 'z': [], 'y': []} for i in range(NUM_LEGS)}
    
    for tick in range(num_ticks):
        feet = freegait.tick(dt)
        for leg in range(NUM_LEGS):
            fg_positions[leg]['x'].append(feet[leg][0])
            fg_positions[leg]['y'].append(feet[leg][1])
            fg_positions[leg]['z'].append(feet[leg][2])
    
    print("\nFreeGait Position Ranges (over 3 seconds):")
    print("-" * 90)
    print(f"{'Leg':<4} {'X min':>10} {'X max':>10} {'ΔX':>10} {'Z min':>10} {'Z max':>10} {'ΔZ':>10} {'Direction':>12}")
    print("-" * 90)
    
    for leg in range(NUM_LEGS):
        x_min = min(fg_positions[leg]['x'])
        x_max = max(fg_positions[leg]['x'])
        z_min = min(fg_positions[leg]['z'])
        z_max = max(fg_positions[leg]['z'])
        delta_x = x_max - x_min
        delta_z = z_max - z_min
        
        if delta_x > delta_z * 2:
            direction = "LATERAL !!!"
        elif delta_z > delta_x * 2:
            direction = "FORWARD"
        else:
            angle = math.degrees(math.atan2(delta_x, delta_z)) if delta_z > 0.01 else 90
            direction = f"DIAG {angle:.0f}°"
        
        print(f"{LEG_NAMES[leg]:<4} {x_min:>10.2f} {x_max:>10.2f} {delta_x:>10.2f} {z_min:>10.2f} {z_max:>10.2f} {delta_z:>10.2f} {direction:>12}")
    
    # Sample first 20 positions for LF to see the actual trajectory
    print("\n" + "=" * 70)
    print("LF POSITION TRACE (first 50 ticks)")
    print("=" * 70)
    print(f"{'Tick':<6} {'X':>10} {'Y':>10} {'Z':>10}")
    print("-" * 40)
    for i in range(min(50, len(fg_positions[0]['x']))):
        x = fg_positions[0]['x'][i]
        y = fg_positions[0]['y'][i]
        z = fg_positions[0]['z'][i]
        print(f"{i:<6} {x:>10.2f} {y:>10.2f} {z:>10.2f}")

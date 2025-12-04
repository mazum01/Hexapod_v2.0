#----------------------------------------------------------------------------------------------------------------------
#    gait_engine.py — Python Gait Engine for MARS Hexapod
#----------------------------------------------------------------------------------------------------------------------
# CHANGE LOG
# Format: YYYY-MM-DD  Summary
# 2025-11-26  Initial architecture: GaitEngine base class, TripodGait, parameter structures.
# 2025-11-29  Rewrite _apply_leg_rotation: base on Teensy test mode corner formulas + linear heading interpolation.
# 2025-12-03  Fixed strafe (crab walk): proper 2D rotation with side-dependent sign on walk direction angle.
#             Left legs (LF,LM,LR) add heading offset; right legs (RF,RM,RR) subtract for coordinated motion.
# 2025-12-03  Bezier curves for swing phase: 5-point Bezier arc with smooth lift-off, peak, and touchdown.
#             Stance phase uses linear motion. Added bezier_point() and binomial_coefficient() utilities.
# 2025-12-03  Tuned lift: 60mm default, Bezier peak at 150% to compensate for curve attenuation.
#----------------------------------------------------------------------------------------------------------------------
"""
Gait Engine Module for MARS Hexapod Python Controller

Architecture:
    - GaitEngine: Base class providing tick()-based foot target generation
    - Gait subclasses: TripodGait, WaveGait, RippleGait (future)
    - GaitParams: Dataclass for gait configuration (step height, stride, cycle time, etc.)
    - Integration: Called from main loop at 166Hz (synced to Teensy telemetry)

Coordinate System (matches Teensy IK):
    - X: lateral (positive = outward from body center)
    - Y: vertical (negative = down toward ground)
    - Z: forward/back (positive = forward)

Leg Order (matches Teensy):
    0: LF (Left Front)   3: RF (Right Front)
    1: LM (Left Middle)  4: RM (Right Middle)
    2: LR (Left Rear)    5: RR (Right Rear)

Usage:
    engine = TripodGait(params)
    engine.start()
    while running:
        feet = engine.tick(dt_seconds)  # Returns 6x3 array of foot positions (mm)
        send_feet_command(feet)
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum, auto

#----------------------------------------------------------------------------------------------------------------------
# Constants
#----------------------------------------------------------------------------------------------------------------------

NUM_LEGS = 6

# Leg indices (match Teensy)
LEG_LF = 0  # Left Front
LEG_LM = 1  # Left Middle
LEG_LR = 2  # Left Rear
LEG_RF = 3  # Right Front
LEG_RM = 4  # Right Middle
LEG_RR = 5  # Right Rear

# Tripod groups
TRIPOD_A = [LEG_RF, LEG_LM, LEG_RR]  # Right-front, Left-middle, Right-rear
TRIPOD_B = [LEG_LF, LEG_RM, LEG_LR]  # Left-front, Right-middle, Left-rear

# Per-leg base rotation angles (degrees) - defines leg orientation relative to body
# Positive = CCW when viewed from above; 0 = pointing straight out (lateral)
# Corner legs at ±45°, middle legs at 0°
LEG_BASE_ROTATION_DEG = [
    45.0,   # LEG_RF (0) - Left Front: -45° (points forward-left)
    0.0,     # LEG_RM (1) - Left Middle: 0° (straight out)
    -45.0,    # LEG_RR (2) - Left Rear: +45° (points backward-left)
    45.0,    # LEG_LR (3) - Right Front: +45° (points forward-right)  
    0.0,     # LEG_LM (4) - Right Middle: 0° (straight out)
    -45.0,   # LEG_LF (5) - Right Rear: -45° (points backward-right)
]

# Precompute sin/cos for each leg's base rotation
LEG_BASE_SIN = [math.sin(math.radians(r)) for r in LEG_BASE_ROTATION_DEG]
LEG_BASE_COS = [math.cos(math.radians(r)) for r in LEG_BASE_ROTATION_DEG]

# Legacy constant for reference
COS_45 = 0.70710678  # cos(45°) = sin(45°)

# Debug logging control
GAIT_DEBUG_LOG_ENABLED = True
GAIT_DEBUG_LOG_PATH = "/tmp/gait_foot_targets.csv"
_gait_debug_log_initialized = False


#----------------------------------------------------------------------------------------------------------------------
# Bezier Curve Utilities
#----------------------------------------------------------------------------------------------------------------------

def binomial_coefficient(n: int, k: int) -> int:
    """Calculate binomial coefficient (n choose k).
    
    Uses the formula: n! / (k! * (n-k)!)
    Matches Arduino implementation for consistency.
    """
    result = 1
    for i in range(1, k + 1):
        result = result * (n - (k - i)) // i
    return result


def bezier_point(points: list, t: float) -> Tuple[float, float, float]:
    """Evaluate a Bezier curve at parameter t.
    
    Args:
        points: List of control points, each as (x, y, z) tuple or list
        t: Parameter 0.0 to 1.0 along the curve
        
    Returns:
        (x, y, z) point on the curve at parameter t
        
    Uses Bernstein polynomial form:
        B(t) = sum(i=0 to n) [ C(n,i) * (1-t)^(n-i) * t^i * P[i] ]
    """
    n = len(points) - 1
    x, y, z = 0.0, 0.0, 0.0
    
    for i in range(len(points)):
        # Bernstein basis polynomial
        b = binomial_coefficient(n, i) * ((1.0 - t) ** (n - i)) * (t ** i)
        x += b * points[i][0]
        y += b * points[i][1]
        z += b * points[i][2]
    
    return x, y, z


def map_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    """Map a value from one range to another (matches Arduino mapFloat)."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class GaitState(Enum):
    """Gait engine state machine."""
    STOPPED = auto()
    STARTING = auto()  # Transitioning to gait from current pose
    RUNNING = auto()
    STOPPING = auto()  # Transitioning from gait to stable pose


#----------------------------------------------------------------------------------------------------------------------
# Data Structures
#----------------------------------------------------------------------------------------------------------------------

@dataclass
class GaitParams:
    """Configuration parameters for gait generation.
    
    Defaults match Teensy test.trigait.* settings for compatibility.
    """
    # Timing
    cycle_ms: int = 3000        # Full gait cycle duration (ms) — both tripod phases
    overlap_pct: float = 5.0    # Overlap percentage at phase transitions
    
    # Geometry (mm, body frame)
    base_x_mm: float = 100.0    # Lateral offset from body center
    base_y_mm: float = -120.0    # Vertical offset (negative = down)
    step_len_mm: float = 20.0   # Forward/back stride amplitude (half-stroke), max 50mm
    lift_mm: float = 60.0       # Lift height during swing phase (increased for Bezier curve attenuation)
    
    # Geometry limits
    MAX_STEP_LEN_MM: float = 40.0  # Safety clamp for step length
    
    # Direction control
    heading_deg: float = 0.0    # Movement direction (0 = forward, 90 = strafe right)
    turn_rate_deg_s: float = 0.0  # Turn rate (degrees per second, positive = CW)
    speed_scale: float = 1.0    # Speed multiplier (0.0 to 1.0)
    
    # Smoothing factor for EMA filter (0 < alpha <= 1)
    # Lower = smoother but slower response; higher = faster but less smooth
    # At 166Hz: alpha=0.04 gives ~150ms time constant
    smoothing_alpha: float = 0.15  # EMA smoothing factor (higher = faster response)
    
    # Bezier curve shape parameters (swing trajectory control points)
    bezier_p1_height: float = 0.15     # P1 height as fraction of lift_mm
    bezier_p1_overshoot: float = 1.1   # P1 overshoot as fraction of stride
    bezier_p2_height: float = 1.5      # P2 peak height (150% to compensate for curve attenuation)
    bezier_p3_height: float = 0.35     # P3 height as fraction of lift_mm
    bezier_p3_overshoot: float = 1.1   # P3 overshoot as fraction of stride
    
    def __post_init__(self):
        """Clamp step_len_mm to safety limit."""
        if self.step_len_mm > self.MAX_STEP_LEN_MM:
            self.step_len_mm = self.MAX_STEP_LEN_MM
    
    def phase_ms(self) -> int:
        """Duration of one tripod phase (half cycle)."""
        return self.cycle_ms // 2
    
    def active_ms(self) -> int:
        """Active swing duration within a phase (excludes overlap)."""
        phase = self.phase_ms()
        overlap = int(self.overlap_pct * 0.01 * phase)
        return phase - overlap


@dataclass
class FootTarget:
    """3D foot position in body frame (mm)."""
    x: float = 0.0  # Lateral
    y: float = 0.0  # Vertical (negative = down)
    z: float = 0.0  # Forward/back
    
    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)
    
    def as_list(self) -> List[float]:
        return [self.x, self.y, self.z]


#----------------------------------------------------------------------------------------------------------------------
# Base Gait Engine
#----------------------------------------------------------------------------------------------------------------------

class GaitEngine:
    """Base class for gait generation.
    
    Subclasses implement _compute_foot_targets() to define specific gait patterns.
    Uses EMA (exponential moving average) smoothing on parameters to prevent jumps.
    """
    
    def __init__(self, params: Optional[GaitParams] = None):
        self.params = params or GaitParams()
        self.state = GaitState.STOPPED
        
        # Timing
        self._start_time: Optional[float] = None
        self._elapsed_ms: int = 0
        self._phase: int = 0  # 0 or 1 for alternating tripod phases
        self._phase_start_ms: int = 0
        
        # Smoothed parameters (EMA filtered) - start at 0 for smooth ramp-up
        self._smoothed_speed: float = 0.0
        self._smoothed_heading: float = 0.0
        self._smoothed_turn_rate: float = 0.0
        
        # Current foot targets (6 legs × 3 coords)
        self._feet: List[FootTarget] = [FootTarget() for _ in range(NUM_LEGS)]
        
        # Home positions (default neutral stance)
        self._home: List[FootTarget] = [
            FootTarget(x=self.params.base_x_mm, y=self.params.base_y_mm, z=0.0)
            for _ in range(NUM_LEGS)
        ]
    
    def start(self) -> None:
        """Begin gait execution."""
        self._start_time = time.monotonic()
        self._elapsed_ms = 0
        self._phase = 0
        self._phase_start_ms = 0
        # Reset smoothed values to 0 for smooth ramp-up from standstill
        self._smoothed_speed = 0.0
        self._smoothed_heading = 0.0
        self._smoothed_turn_rate = 0.0
        self.state = GaitState.RUNNING
    
    def stop(self) -> None:
        """Stop gait and return to neutral stance."""
        self.state = GaitState.STOPPING
        # Transition logic could be added here
        self.state = GaitState.STOPPED
        self._start_time = None
    
    def is_running(self) -> bool:
        """Check if gait is actively running."""
        return self.state == GaitState.RUNNING
    
    def tick(self, dt_seconds: float = None) -> List[List[float]]:
        """Advance gait by one tick and return foot targets.
        
        Args:
            dt_seconds: Time since last tick. If None, computed from monotonic clock.
            
        Returns:
            List of 6 foot positions, each [x, y, z] in mm (body frame).
            Order: LF, LM, LR, RF, RM, RR
        """
        if self.state != GaitState.RUNNING:
            # Return current/home positions when not running
            return [foot.as_list() for foot in self._feet]
        
        # Update elapsed time
        if dt_seconds is not None:
            self._elapsed_ms += int(dt_seconds * 1000)
        elif self._start_time is not None:
            self._elapsed_ms = int((time.monotonic() - self._start_time) * 1000)
        
        # Check for phase transition
        phase_ms = self.params.phase_ms()
        phase_elapsed = self._elapsed_ms - self._phase_start_ms
        if phase_elapsed >= phase_ms:
            self._phase ^= 1  # Toggle phase
            self._phase_start_ms = self._elapsed_ms
            phase_elapsed = 0
        
        # Compute progression within phase
        active_ms = self.params.active_ms()
        t_active = min(1.0, phase_elapsed / active_ms) if active_ms > 0 else 0.0
        in_overlap = phase_elapsed >= active_ms
        
        # Apply EMA smoothing to parameters
        # smoothed = alpha * target + (1 - alpha) * smoothed
        alpha = self.params.smoothing_alpha
        self._smoothed_speed = alpha * self.params.speed_scale + (1.0 - alpha) * self._smoothed_speed
        self._smoothed_heading = alpha * self.params.heading_deg + (1.0 - alpha) * self._smoothed_heading
        self._smoothed_turn_rate = alpha * self.params.turn_rate_deg_s + (1.0 - alpha) * self._smoothed_turn_rate
        
        # Delegate to subclass for actual foot target computation
        self._compute_foot_targets(t_active, in_overlap)
        
        return [foot.as_list() for foot in self._feet]
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Compute foot targets for current state. Override in subclasses."""
        raise NotImplementedError("Subclasses must implement _compute_foot_targets")
    
    def get_feet_command(self) -> str:
        """Format foot targets as FEET command string for Teensy."""
        parts = []
        for foot in self._feet:
            parts.extend([f"{foot.x:.1f}", f"{foot.y:.1f}", f"{foot.z:.1f}"])
        cmd = "FEET " + " ".join(parts)
        # Log every 10th command to debug file (increased from 50 for strafe debugging)
        if hasattr(self, '_feet_cmd_count'):
            self._feet_cmd_count += 1
        else:
            self._feet_cmd_count = 0
        if self._feet_cmd_count % 10 == 0:
            with open('/tmp/feet_commands.log', 'a') as f:
                f.write(f"{cmd}\n")
        return cmd
    
    def get_feet_bytes(self) -> bytes:
        """Format foot targets as FEET command bytes for send_cmd()."""
        return self.get_feet_command().encode('ascii')


#----------------------------------------------------------------------------------------------------------------------
# Tripod Gait Implementation
#----------------------------------------------------------------------------------------------------------------------

class TripodGait(GaitEngine):
    """Alternating tripod gait — 3 legs swing while 3 legs stance.
    
    Matches Teensy TEST mode behavior for validation.
    
    Phase 0: Tripod A (RF, LM, RR) swings, Tripod B (LF, RM, LR) stance
    Phase 1: Tripod B swings, Tripod A stance
    """
    
    def __init__(self, params: Optional[GaitParams] = None):
        super().__init__(params)
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Compute tripod gait foot positions with heading (OMNI/crab walk mode).
        
        When speed_scale=0: legs cycle (lift/lower) but no translation (step in place).
        When speed_scale>0: legs translate based on heading angle.
        
        Matches old Hexapod_Simple OMNI mode: walk direction is added to base leg rotation
        to create combined rotation for the stride vector.
        
        Uses smoothed parameters from EMA filter for smooth transitions.
        """
        global _gait_debug_log_initialized
        
        p = self.params
        
        # Use smoothed values for smooth transitions (set by tick() via EMA)
        speed = self._smoothed_speed
        heading = self._smoothed_heading
        turn_rate = self._smoothed_turn_rate
        
        # Effective step length scaled by smoothed speed (0 = step in place)
        effective_step = p.step_len_mm * speed
        step_offset = 2.0 * effective_step * t_active  # 0 to 2*step_len progression
        
        # Walk direction from heading (normalized -1 to +1 for full strafe)
        # heading_deg: 0° = forward, 90° = strafe right, -90° = strafe left
        walk_dir = heading / 90.0  # Normalize to -1..+1 range
        
        # Debug CSV logging - write header on first call
        debug_log = None
        if GAIT_DEBUG_LOG_ENABLED:
            try:
                if not _gait_debug_log_initialized:
                    debug_log = open(GAIT_DEBUG_LOG_PATH, 'w')
                    debug_log.write("time_ms,phase,t_active,in_overlap,speed,heading,effective_step,step_offset,walk_dir,")
                    debug_log.write("leg,base_rot_deg,rot_cos,rot_sin,is_swing,stride_mag,x_prime,y,z_prime\n")
                    _gait_debug_log_initialized = True
                else:
                    debug_log = open(GAIT_DEBUG_LOG_PATH, 'a')
            except Exception:
                debug_log = None
        
        # Bezier control point parameters for swing trajectory
        # These shape the foot path during swing phase
        lift_height = p.lift_mm
        base_y = p.base_y_mm
        
        for leg in range(NUM_LEGS):
            # Base position
            x = p.base_x_mm
            y = base_y
            
            # Determine tripod membership and swing/stance assignment
            is_tripod_a = leg in TRIPOD_A
            assigned_swing = (is_tripod_a and self._phase == 0) or (not is_tripod_a and self._phase == 1)
            
            # Compute stride magnitude and height using Bezier curves
            if assigned_swing:
                # Swing phase: use 5-point Bezier for smooth arc
                # Map phase progress to 0..1 for swing portion
                # t_active goes 0->1 during active phase, but swing needs its own 0->1
                if in_overlap:
                    # During overlap, foot is at end of swing (on ground, at front)
                    stride_mag = effective_step
                    y = base_y
                else:
                    # Active swing: smooth Bezier trajectory
                    # Swing goes from -step (back) to +step (front)
                    swing_t = t_active  # 0 at start of swing, 1 at end
                    
                    # 5-point Bezier control points for swing arc (x=unused, y=height, z=stride)
                    # P0: Start - back position, on ground
                    # P1: Initial lift - slight overshoot back, beginning lift
                    # P2: Peak - center, maximum height (configurable to compensate for Bezier attenuation)
                    # P3: Descending - near front, coming down
                    # P4: End - front position, on ground
                    swing_points = [
                        (0, base_y,                                    -effective_step),                        # P0: start (back)
                        (0, base_y + p.bezier_p1_height * lift_height, -p.bezier_p1_overshoot * effective_step), # P1: slight lift, overshoot back
                        (0, base_y + p.bezier_p2_height * lift_height, -0.1 * effective_step),                   # P2: peak height, ~center
                        (0, base_y + p.bezier_p3_height * lift_height, p.bezier_p3_overshoot * effective_step),  # P3: descending, overshoot front
                        (0, base_y,                                    effective_step),                          # P4: end (front)
                    ]
                    
                    _, y, stride_mag = bezier_point(swing_points, swing_t)
            else:
                # Stance phase: linear motion from front (+step) to back (-step)
                # This is equivalent to a 2-point Bezier (straight line)
                # t_active goes 0->1, stride goes +step to -step
                stride_mag = effective_step * (1.0 - 2.0 * t_active)
                y = base_y  # Always on ground during stance
            
            # Apply OMNI mode rotation (matches old Hexapod_Simple firmware)
            # Combine base leg rotation with walk direction
            x_prime, z_prime, rot_cos, rot_sin = self._apply_leg_rotation(leg, x, stride_mag, walk_dir)
            
            # Debug CSV logging
            if debug_log is not None and leg == 0:  # Log only for first leg to reduce data
                try:
                    debug_log.write(f"{self._elapsed_ms},{self._phase},{t_active:.4f},{1 if in_overlap else 0},")
                    debug_log.write(f"{speed:.4f},{heading:.2f},{effective_step:.2f},{step_offset:.2f},{walk_dir:.4f},")
                    debug_log.write(f"{leg},{LEG_BASE_ROTATION_DEG[leg]:.1f},{rot_cos:.4f},{rot_sin:.4f},")
                    debug_log.write(f"{1 if assigned_swing else 0},{stride_mag:.2f},{x_prime:.2f},{y:.2f},{z_prime:.2f}\n")
                except Exception:
                    pass

            # Store computed target
            self._feet[leg] = FootTarget(x=x_prime, y=y, z=z_prime)
        
        # Close debug log file
        if debug_log is not None:
            try:
                debug_log.flush()
                debug_log.close()
            except Exception:
                pass
    
    def _apply_leg_rotation(self, leg: int, base_x: float, stride_z: float, walk_dir: float) -> Tuple[float, float, float, float]:
        """Apply per-leg corner rotation (matching Teensy test gait) plus heading offset.
        
        Based on working Teensy test mode:
        - Front corners (LF, RF): xprime = +c*z + offset, zprime = c*z - 0.5*base_x  
        - Rear corners (LR, RR): xprime = -c*z + offset, zprime = c*z + 0.5*base_x
        - Middle legs (LM, RM): xprime = base_x, zprime = z
        
        Heading is applied by rotating the stride vector before corner rotation.
        
        Args:
            leg: Leg index (0-5)
            base_x: Base lateral offset from body center (mm)
            stride_z: Stride magnitude in body-frame Z (forward/back)
            walk_dir: Walk direction from heading, normalized -1 to +1
            
        Returns:
            (x_prime, z_prime, rot_cos, rot_sin): Final foot position and rotation debug values
        """
        # Compute the rotation of a foot about the standing foot position

        # Step 0
        # Calculate the rotation and translation constants
        # Calculate the foot base position base on the base parameters and the leg base
        base_z = -self.params.base_x_mm * LEG_BASE_SIN[leg] # + 0.0 * LEG_BASE_COS[leg]
        base_x = self.params.base_x_mm * LEG_BASE_COS[leg] # + 0.0 * LEG_BASE_SIN[leg]
        sign = 1.0 if leg in (LEG_LF, LEG_LM, LEG_LR) else -1.0
        full_angle = math.radians(LEG_BASE_ROTATION_DEG[leg]) + sign * math.radians(walk_dir * 90.0)
        cos_angle = math.cos(full_angle)
        sin_angle = math.sin(full_angle)

        # Step 1
        # Translate the foot position to origin  -N/A

        # Step 2
        # Apply rotation based on leg base rotation and walk direction
        stride_x_prime = stride_z * sin_angle
        stride_z_prime = stride_z * cos_angle

        # Step 3
        # Translate the foot position back to original base position
        l_x_prime = base_x + stride_x_prime
        l_z_prime = stride_z_prime

        #Debug print
        if leg == 1:
            print(f"ROTATION:  Leg {leg}: walk dir={walk_dir:.2f}, full_angle={math.degrees(full_angle):.2f}, stride_z={stride_z:.2f}, base_x={base_x:.2f}, base_z={base_z:.2f}, stride_x_prime={stride_x_prime:.2f}, stride_z_prime={stride_z_prime:.2f} => x'={l_x_prime:.2f}, z'={l_z_prime:.2f}", end="\r\n")  

        # Return the final position
        return l_x_prime, l_z_prime, cos_angle, sin_angle

#----------------------------------------------------------------------------------------------------------------------
# Stationary Test Pattern
#----------------------------------------------------------------------------------------------------------------------

class StationaryPattern(GaitEngine):
    """Test pattern that moves feet in small circles without locomotion.
    
    Useful for validating FEET command flow and timing without robot movement.
    """
    
    def __init__(self, params: Optional[GaitParams] = None, 
                 radius_mm: float = 10.0, 
                 period_ms: int = 2000):
        super().__init__(params)
        self.radius_mm = radius_mm
        self.period_ms = period_ms
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Compute circular motion pattern."""
        # Use elapsed time for smooth continuous motion
        phase_rad = (self._elapsed_ms % self.period_ms) / self.period_ms * 2.0 * math.pi
        
        for leg in range(NUM_LEGS):
            # Base position
            x = self.params.base_x_mm
            y = self.params.base_y_mm
            z = 0.0
            
            # Add circular offset (in Y-Z plane for vertical circles)
            # Each leg gets a phase offset for visual interest
            leg_phase = phase_rad + (leg * math.pi / 3.0)  # 60° offset per leg
            y_offset = self.radius_mm * math.sin(leg_phase)
            z_offset = self.radius_mm * math.cos(leg_phase)
            
            # Apply corner leg rotation
            x_prime, z_prime = self._apply_leg_rotation(leg, x, z + z_offset)
            
            self._feet[leg] = FootTarget(x=x_prime, y=y + y_offset, z=z_prime)
    
    def _apply_leg_rotation(self, leg: int, x: float, z: float) -> Tuple[float, float]:
        """Apply 45° rotation for corner legs.
        
        For corner legs, rotate the offset from base position by ±45°.
        x = base_x + x_move (lateral offset)
        z = z_move (forward/back offset)
        """
        base_x = self.params.base_x_mm
        
        # Extract the offset from base position
        x_offset = x - base_x  # Lateral movement (from heading)
        z_offset = z          # Forward/back movement
        
        if leg in (LEG_RR, LEG_LR):
            # Rear corners: rotate -45° (clockwise when viewed from above)
            # For rear-right: leg points backward-right, so rotate offset
            x_prime = base_x + COS_45 * x_offset + COS_45 * z_offset
            z_prime = -COS_45 * x_offset + COS_45 * z_offset
        elif leg in (LEG_LF, LEG_RF):
            # Front corners: rotate +45° (counter-clockwise when viewed from above)
            x_prime = base_x + COS_45 * x_offset - COS_45 * z_offset
            z_prime = COS_45 * x_offset + COS_45 * z_offset
        else:
            # Middle legs: no rotation
            x_prime = x
            z_prime = z
        
        return x_prime, z_prime


#----------------------------------------------------------------------------------------------------------------------
# Test / Demo
#----------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Simple test: print foot targets for a few ticks
    print("=== Gait Engine Test ===\n")
    
    params = GaitParams(
        cycle_ms=2000,
        base_x_mm=100.0,
        base_y_mm=-120.0,
        step_len_mm=40.0,
        lift_mm=15.0,
        overlap_pct=5.0
    )
    
    print(f"Parameters: cycle={params.cycle_ms}ms, step={params.step_len_mm}mm, lift={params.lift_mm}mm\n")
    
    # Test tripod gait
    tripod = TripodGait(params)
    tripod.start()
    
    print("Tripod Gait - First 10 ticks at 166Hz:")
    dt = 1.0 / 166.0  # 6.024ms
    for i in range(10):
        feet = tripod.tick(dt)
        if i < 3 or i >= 8:
            print(f"  Tick {i}: phase={tripod._phase}, LF=({feet[0][0]:.1f}, {feet[0][1]:.1f}, {feet[0][2]:.1f})")
    
    print(f"\nFEET command: {tripod.get_feet_command()[:80]}...")
    
    # Test stationary pattern
    print("\n\nStationary Pattern - First 5 ticks:")
    pattern = StationaryPattern(params, radius_mm=15.0, period_ms=1000)
    pattern.start()
    
    for i in range(5):
        feet = pattern.tick(dt)
        print(f"  Tick {i}: LF=({feet[0][0]:.1f}, {feet[0][1]:.1f}, {feet[0][2]:.1f})")
    
    print("\n=== Test Complete ===")

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
# 2025-12-04  Added WaveGait (1 leg at a time, 6 phases) and RippleGait (2 diagonal legs, 3 phases).
# 2025-12-04  Added GaitTransition class for smooth phase-locked transitions between gait types.
# 2025-12-04  Walking turn: differential stride per leg based on hip position and turn rate.
#----------------------------------------------------------------------------------------------------------------------
"""
Gait Engine Module for MARS Hexapod Python Controller

Architecture:
    - GaitEngine: Base class providing tick()-based foot target generation
    - Gait subclasses: TripodGait, WaveGait, RippleGait
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

# Leg hip positions in body frame (mm) - for turn radius calculations
# These define where each leg's hip joint is relative to body center
# X = lateral (positive = right side of body), Z = forward (positive = front)
# Values based on typical hexapod geometry; adjust if needed for specific robot
LEG_HIP_X = [
    -80.0,   # LEG_LF: left side, front corner
    -100.0,  # LEG_LM: left side, middle
    -80.0,   # LEG_LR: left side, rear corner
    80.0,    # LEG_RF: right side, front corner
    100.0,   # LEG_RM: right side, middle
    80.0,    # LEG_RR: right side, rear corner
]
LEG_HIP_Z = [
    100.0,   # LEG_LF: front
    0.0,     # LEG_LM: middle
    -100.0,  # LEG_LR: rear
    100.0,   # LEG_RF: front
    0.0,     # LEG_RM: middle
    -100.0,  # LEG_RR: rear
]

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
        """Compute tripod gait foot positions with heading and walking turn.
        
        When speed_scale=0: legs cycle (lift/lower) but no translation (step in place).
        When speed_scale>0: legs translate based on heading angle.
        When turn_rate!=0: legs move differentially for walking turn (arc motion).
        
        Walking Turn Strategy:
        - turn_rate_deg_s defines how much the body rotates per second
        - Each leg's stride is adjusted based on its arc around the turn center
        - Legs on outside of turn take longer strides, inside legs shorter
        - Pure rotation (speed=0, turn!=0) rotates in place
        
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
        
        # Walking turn: calculate rotation per phase
        # turn_rate is in deg/s, phase duration is cycle_ms/2
        phase_duration_s = (p.cycle_ms / 2.0) / 1000.0
        turn_per_phase_deg = turn_rate * phase_duration_s
        turn_per_phase_rad = math.radians(turn_per_phase_deg)
        
        # Debug CSV logging - write header on first call
        debug_log = None
        if GAIT_DEBUG_LOG_ENABLED:
            try:
                if not _gait_debug_log_initialized:
                    debug_log = open(GAIT_DEBUG_LOG_PATH, 'w')
                    debug_log.write("time_ms,phase,t_active,in_overlap,speed,heading,turn_rate,effective_step,step_offset,walk_dir,")
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
            
            # Calculate turn-adjusted stride for this leg
            # Arc length = angle (rad) * radius
            # For walking turn: each leg travels an arc proportional to its distance from body center
            # LEG_HIP_X gives lateral offset: positive = right side, negative = left side
            # For CW turn (positive turn_rate): right legs (positive X) take shorter strides
            # For CCW turn (negative turn_rate): left legs (negative X) take shorter strides
            turn_arc_mm = turn_per_phase_rad * LEG_HIP_X[leg]  # Arc contribution from turn
            
            # Combine translation stride with turn arc
            # - effective_step: translation component (same for all legs)
            # - turn_arc_mm: rotation component (varies by leg position)
            # For pure rotation (speed=0), legs move in opposite directions
            leg_effective_step = effective_step - turn_arc_mm
            
            # Compute stride magnitude and height using Bezier curves
            if assigned_swing:
                # Swing phase: use 5-point Bezier for smooth arc
                # Map phase progress to 0..1 for swing portion
                # t_active goes 0->1 during active phase, but swing needs its own 0->1
                if in_overlap:
                    # During overlap, foot is at end of swing (on ground, at front)
                    stride_mag = leg_effective_step
                    y = base_y
                else:
                    # Active swing: smooth Bezier trajectory
                    # Swing goes from -step (back) to +step (front)
                    swing_t = t_active  # 0 at start of swing, 1 at end
                    
                    # 5-point Bezier control points for swing arc (x=unused, y=height, z=stride)
                    # Use leg_effective_step for this leg's adjusted stride
                    swing_points = [
                        (0, base_y,                                    -leg_effective_step),                        # P0: start (back)
                        (0, base_y + p.bezier_p1_height * lift_height, -p.bezier_p1_overshoot * leg_effective_step), # P1: slight lift, overshoot back
                        (0, base_y + p.bezier_p2_height * lift_height, -0.1 * leg_effective_step),                   # P2: peak height, ~center
                        (0, base_y + p.bezier_p3_height * lift_height, p.bezier_p3_overshoot * leg_effective_step),  # P3: descending, overshoot front
                        (0, base_y,                                    leg_effective_step),                          # P4: end (front)
                    ]
                    
                    _, y, stride_mag = bezier_point(swing_points, swing_t)
            else:
                # Stance phase: linear motion from front (+step) to back (-step)
                # t_active goes 0->1, stride goes +step to -step
                stride_mag = leg_effective_step * (1.0 - 2.0 * t_active)
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

        # Return the final position
        return l_x_prime, l_z_prime, cos_angle, sin_angle

#----------------------------------------------------------------------------------------------------------------------
# Wave Gait Implementation
#----------------------------------------------------------------------------------------------------------------------

# Wave gait sequence: one leg at a time, maximum stability
# Order: RR -> RM -> RF -> LR -> LM -> LF (rear to front, alternating sides)
WAVE_SEQUENCE = [LEG_RR, LEG_RM, LEG_RF, LEG_LR, LEG_LM, LEG_LF]


class WaveGait(GaitEngine):
    """Wave gait — one leg swings at a time, maximum stability.
    
    6 phases, each phase one leg swings while 5 legs stance.
    Very slow but extremely stable - good for rough terrain.
    
    Sequence: RR -> RM -> RF -> LR -> LM -> LF (rear to front, alternating sides)
    """
    
    def __init__(self, params: Optional[GaitParams] = None):
        super().__init__(params)
        self._wave_phase = 0  # 0-5, which leg is currently swinging
    
    def start(self) -> None:
        """Begin gait execution."""
        super().start()
        self._wave_phase = 0
    
    def tick(self, dt_seconds: float = None) -> List[List[float]]:
        """Advance gait by one tick and return foot targets."""
        if self.state != GaitState.RUNNING:
            return [foot.as_list() for foot in self._feet]
        
        # Update elapsed time
        if dt_seconds is not None:
            self._elapsed_ms += int(dt_seconds * 1000)
        elif self._start_time is not None:
            self._elapsed_ms = int((time.monotonic() - self._start_time) * 1000)
        
        # Wave gait: 6 phases, each phase = cycle_ms / 6
        phase_ms = self.params.cycle_ms // 6
        phase_elapsed = self._elapsed_ms - self._phase_start_ms
        if phase_elapsed >= phase_ms:
            self._wave_phase = (self._wave_phase + 1) % 6
            self._phase_start_ms = self._elapsed_ms
            phase_elapsed = 0
        
        # Compute progression within phase
        active_ms = int(phase_ms * (100.0 - self.params.overlap_pct) / 100.0)
        t_active = min(1.0, phase_elapsed / active_ms) if active_ms > 0 else 0.0
        in_overlap = phase_elapsed >= active_ms
        
        # Apply EMA smoothing to parameters
        alpha = self.params.smoothing_alpha
        self._smoothed_speed = alpha * self.params.speed_scale + (1.0 - alpha) * self._smoothed_speed
        self._smoothed_heading = alpha * self.params.heading_deg + (1.0 - alpha) * self._smoothed_heading
        self._smoothed_turn_rate = alpha * self.params.turn_rate_deg_s + (1.0 - alpha) * self._smoothed_turn_rate
        
        # Delegate to foot target computation
        self._compute_foot_targets(t_active, in_overlap)
        
        return [foot.as_list() for foot in self._feet]
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Compute wave gait foot positions with walking turn support.
        
        Only one leg swings at a time. All others push the body forward (stance).
        Walking turn: each leg's stride adjusted by its arc around turn center.
        """
        p = self.params
        speed = self._smoothed_speed
        heading = self._smoothed_heading
        turn_rate = self._smoothed_turn_rate
        effective_step = p.step_len_mm * speed
        walk_dir = heading / 90.0
        
        # Walking turn: calculate rotation per phase (wave has 6 phases per cycle)
        phase_duration_s = (p.cycle_ms / 6.0) / 1000.0
        turn_per_phase_deg = turn_rate * phase_duration_s
        turn_per_phase_rad = math.radians(turn_per_phase_deg)
        
        # Bezier parameters
        lift_height = p.lift_mm
        base_y = p.base_y_mm
        
        # The currently swinging leg
        swing_leg = WAVE_SEQUENCE[self._wave_phase]

        # Map leg -> its index in the wave sequence so stance legs can be
        # distributed across the stance trajectory instead of marching in sync.
        # Without this offset, all stance legs share the same stride position and
        # the gait tends to "walk in place".
        wave_index_by_leg = {leg: idx for idx, leg in enumerate(WAVE_SEQUENCE)}
        
        for leg in range(NUM_LEGS):
            x = p.base_x_mm
            y = base_y
            
            is_swing = (leg == swing_leg)
            
            # Calculate turn-adjusted stride for this leg
            turn_arc_mm = turn_per_phase_rad * LEG_HIP_X[leg]
            leg_effective_step = effective_step - turn_arc_mm
            
            if is_swing:
                # Swing phase: Bezier trajectory
                if in_overlap:
                    stride_mag = leg_effective_step
                    y = base_y
                else:
                    swing_t = t_active
                    swing_points = [
                        (0, base_y,                                    -leg_effective_step),
                        (0, base_y + p.bezier_p1_height * lift_height, -p.bezier_p1_overshoot * leg_effective_step),
                        (0, base_y + p.bezier_p2_height * lift_height, -0.1 * leg_effective_step),
                        (0, base_y + p.bezier_p3_height * lift_height, p.bezier_p3_overshoot * leg_effective_step),
                        (0, base_y,                                    leg_effective_step),
                    ]
                    _, y, stride_mag = bezier_point(swing_points, swing_t)
            else:
                # Stance phase: 5 phases long (since 1 of 6 phases is swing for this leg).
                # Compute where this leg is along its stance trajectory based on how many
                # phases have elapsed since it last swung.
                idx_leg = wave_index_by_leg.get(leg, 0)
                phases_since_swing = (self._wave_phase - idx_leg) % 6
                # phases_since_swing is 1..5 for stance legs; 0 for the swing leg.
                stance_phase = max(0, phases_since_swing - 1)  # 0..4
                stance_progress = (stance_phase + t_active) / 5.0  # 0..1
                stride_mag = leg_effective_step * (1.0 - 2.0 * stance_progress)
                y = base_y
            
            # Apply leg rotation
            x_prime, z_prime = self._apply_leg_rotation_simple(leg, x, stride_mag, walk_dir)
            self._feet[leg] = FootTarget(x=x_prime, y=y, z=z_prime)
    
    def _apply_leg_rotation_simple(self, leg: int, base_x: float, stride_z: float, walk_dir: float) -> Tuple[float, float]:
        """Apply per-leg rotation (simplified version without debug return values)."""
        base_z = -self.params.base_x_mm * LEG_BASE_SIN[leg]
        base_x_calc = self.params.base_x_mm * LEG_BASE_COS[leg]
        sign = 1.0 if leg in (LEG_LF, LEG_LM, LEG_LR) else -1.0
        full_angle = math.radians(LEG_BASE_ROTATION_DEG[leg]) + sign * math.radians(walk_dir * 90.0)
        cos_angle = math.cos(full_angle)
        sin_angle = math.sin(full_angle)
        
        stride_x_prime = stride_z * sin_angle
        stride_z_prime = stride_z * cos_angle
        
        return base_x_calc + stride_x_prime, stride_z_prime


#----------------------------------------------------------------------------------------------------------------------
# Ripple Gait Implementation
#----------------------------------------------------------------------------------------------------------------------

# Ripple gait sequence: two legs at a time (diagonal pairs)
# 3 phases with 2 legs each for good speed/stability balance
RIPPLE_GROUPS = [
    [LEG_RF, LEG_LR],  # Phase 0: Right-front + Left-rear
    [LEG_RM, LEG_LM],  # Phase 1: Right-middle + Left-middle  
    [LEG_RR, LEG_LF],  # Phase 2: Right-rear + Left-front
]


class RippleGait(GaitEngine):
    """Ripple gait — two legs swing at a time in diagonal pairs.
    
    3 phases, each phase two diagonal legs swing while 4 legs stance.
    Good balance between speed and stability.
    
    Phase 0: RF + LR swing
    Phase 1: RM + LM swing
    Phase 2: RR + LF swing
    """
    
    def __init__(self, params: Optional[GaitParams] = None):
        super().__init__(params)
        self._ripple_phase = 0  # 0-2, which pair is currently swinging
    
    def start(self) -> None:
        """Begin gait execution."""
        super().start()
        self._ripple_phase = 0
    
    def tick(self, dt_seconds: float = None) -> List[List[float]]:
        """Advance gait by one tick and return foot targets."""
        if self.state != GaitState.RUNNING:
            return [foot.as_list() for foot in self._feet]
        
        # Update elapsed time
        if dt_seconds is not None:
            self._elapsed_ms += int(dt_seconds * 1000)
        elif self._start_time is not None:
            self._elapsed_ms = int((time.monotonic() - self._start_time) * 1000)
        
        # Ripple gait: 3 phases, each phase = cycle_ms / 3
        phase_ms = self.params.cycle_ms // 3
        phase_elapsed = self._elapsed_ms - self._phase_start_ms
        if phase_elapsed >= phase_ms:
            self._ripple_phase = (self._ripple_phase + 1) % 3
            self._phase_start_ms = self._elapsed_ms
            phase_elapsed = 0
        
        # Compute progression within phase
        active_ms = int(phase_ms * (100.0 - self.params.overlap_pct) / 100.0)
        t_active = min(1.0, phase_elapsed / active_ms) if active_ms > 0 else 0.0
        in_overlap = phase_elapsed >= active_ms
        
        # Apply EMA smoothing to parameters
        alpha = self.params.smoothing_alpha
        self._smoothed_speed = alpha * self.params.speed_scale + (1.0 - alpha) * self._smoothed_speed
        self._smoothed_heading = alpha * self.params.heading_deg + (1.0 - alpha) * self._smoothed_heading
        self._smoothed_turn_rate = alpha * self.params.turn_rate_deg_s + (1.0 - alpha) * self._smoothed_turn_rate
        
        # Delegate to foot target computation
        self._compute_foot_targets(t_active, in_overlap)
        
        return [foot.as_list() for foot in self._feet]
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Compute ripple gait foot positions with walking turn support.
        
        Two diagonal legs swing at a time. Four legs stance.
        Walking turn: each leg's stride adjusted by its arc around turn center.
        """
        p = self.params
        speed = self._smoothed_speed
        heading = self._smoothed_heading
        turn_rate = self._smoothed_turn_rate
        effective_step = p.step_len_mm * speed
        walk_dir = heading / 90.0
        
        # Walking turn: calculate rotation per phase (ripple has 3 phases per cycle)
        phase_duration_s = (p.cycle_ms / 3.0) / 1000.0
        turn_per_phase_deg = turn_rate * phase_duration_s
        turn_per_phase_rad = math.radians(turn_per_phase_deg)
        
        # Bezier parameters
        lift_height = p.lift_mm
        base_y = p.base_y_mm
        
        # The currently swinging legs
        swing_legs = RIPPLE_GROUPS[self._ripple_phase]

        # Map leg -> its index in the ripple group sequence so stance legs can be
        # distributed across the stance trajectory instead of marching in sync.
        ripple_index_by_leg = {
            leg: idx for idx, group in enumerate(RIPPLE_GROUPS) for leg in group
        }
        
        for leg in range(NUM_LEGS):
            x = p.base_x_mm
            y = base_y
            
            is_swing = (leg in swing_legs)
            
            # Calculate turn-adjusted stride for this leg
            turn_arc_mm = turn_per_phase_rad * LEG_HIP_X[leg]
            leg_effective_step = effective_step - turn_arc_mm
            
            if is_swing:
                # Swing phase: Bezier trajectory
                if in_overlap:
                    stride_mag = leg_effective_step
                    y = base_y
                else:
                    swing_t = t_active
                    swing_points = [
                        (0, base_y,                                    -leg_effective_step),
                        (0, base_y + p.bezier_p1_height * lift_height, -p.bezier_p1_overshoot * leg_effective_step),
                        (0, base_y + p.bezier_p2_height * lift_height, -0.1 * leg_effective_step),
                        (0, base_y + p.bezier_p3_height * lift_height, p.bezier_p3_overshoot * leg_effective_step),
                        (0, base_y,                                    leg_effective_step),
                    ]
                    _, y, stride_mag = bezier_point(swing_points, swing_t)
            else:
                # Stance phase: 2 phases long (since 1 of 3 phases is swing for this leg).
                # Compute where this leg is along its stance trajectory based on how many
                # phases have elapsed since it last swung.
                idx_leg = ripple_index_by_leg.get(leg, 0)
                phases_since_swing = (self._ripple_phase - idx_leg) % 3
                # phases_since_swing is 1..2 for stance legs; 0 for swing legs.
                stance_phase = max(0, phases_since_swing - 1)  # 0..1
                stance_progress = (stance_phase + t_active) / 2.0  # 0..1
                stride_mag = leg_effective_step * (1.0 - 2.0 * stance_progress)
                y = base_y
            
            # Apply leg rotation
            x_prime, z_prime = self._apply_leg_rotation_simple(leg, x, stride_mag, walk_dir)
            self._feet[leg] = FootTarget(x=x_prime, y=y, z=z_prime)
    
    def _apply_leg_rotation_simple(self, leg: int, base_x: float, stride_z: float, walk_dir: float) -> Tuple[float, float]:
        """Apply per-leg rotation (simplified version without debug return values)."""
        base_z = -self.params.base_x_mm * LEG_BASE_SIN[leg]
        base_x_calc = self.params.base_x_mm * LEG_BASE_COS[leg]
        sign = 1.0 if leg in (LEG_LF, LEG_LM, LEG_LR) else -1.0
        full_angle = math.radians(LEG_BASE_ROTATION_DEG[leg]) + sign * math.radians(walk_dir * 90.0)
        cos_angle = math.cos(full_angle)
        sin_angle = math.sin(full_angle)
        
        stride_x_prime = stride_z * sin_angle
        stride_z_prime = stride_z * cos_angle
        
        return base_x_calc + stride_x_prime, stride_z_prime


#----------------------------------------------------------------------------------------------------------------------
# Standing Gait (static pose for leveling tests)
#----------------------------------------------------------------------------------------------------------------------

class StandingGait(GaitEngine):
    """Static standing pose that continuously sends FEET commands.
    
    Unlike the firmware STAND command, this uses the gait engine pipeline
    so body leveling corrections can be applied. Feet stay at neutral position.
    """
    
    def __init__(self, params: Optional[GaitParams] = None):
        super().__init__(params)
    
    def _compute_foot_targets(self, t_active: float, in_overlap: bool) -> None:
        """Set all feet to neutral standing position.
        
        Matches firmware STAND command: all feet at (base_x, base_y, 0).
        No corner leg rotation is applied - same x for all legs.
        """
        base_x = self.params.base_x_mm
        base_y = self.params.base_y_mm
        
        for leg in range(NUM_LEGS):
            # Match firmware STAND: x=base_x, y=base_y, z=0 for all legs
            # (no corner rotation like TripodGait uses)
            self._feet[leg] = FootTarget(x=base_x, y=base_y, z=0.0)


#----------------------------------------------------------------------------------------------------------------------
# Step-to-Stand Recovery Gait (collision recovery)
#----------------------------------------------------------------------------------------------------------------------

class StepToStandGait(GaitEngine):
    """Step from current foot targets to neutral standing pose.

    Moves one leg at a time using a 3-phase step trajectory:
      1) lift (y + lift_mm)
      2) translate (x/z to target while lifted)
      3) place (y back to target)

    This is intended for collision recovery where a straight-line STAND move
    could worsen an inter-leg collision.
    """

    def __init__(
        self,
        params: Optional[GaitParams] = None,
        start_feet: Optional[List[List[float]]] = None,
        lift_mm: float = 60.0,
        lift_ms: int = 120,
        move_ms: int = 360,
        place_ms: int = 120,
        leg_order: Optional[List[int]] = None,
    ):
        super().__init__(params)
        self.lift_mm = float(lift_mm)
        self.lift_ms = int(lift_ms)
        self.move_ms = int(move_ms)
        self.place_ms = int(place_ms)
        self._seg_ms: int = 0
        self._complete: bool = False

        # Choose a stable, simple stepping order: alternate left/right by row.
        # Indices are LF, LM, LR, RF, RM, RR.
        self._order: List[int] = list(leg_order) if leg_order is not None else [0, 3, 1, 4, 2, 5]
        self._order = [int(i) for i in self._order if 0 <= int(i) < NUM_LEGS]
        if not self._order:
            self._order = list(range(NUM_LEGS))

        # Initialize start feet
        if start_feet and len(start_feet) == NUM_LEGS:
            self._feet = [FootTarget(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in start_feet]
        else:
            self._feet = [FootTarget(x=self.params.base_x_mm, y=self.params.base_y_mm, z=0.0) for _ in range(NUM_LEGS)]

        # Hold positions evolve as legs are placed
        self._hold: List[FootTarget] = [FootTarget(x=f.x, y=f.y, z=f.z) for f in self._feet]

        # Neutral stand target for all legs
        self._target: List[FootTarget] = [
            FootTarget(x=self.params.base_x_mm, y=self.params.base_y_mm, z=0.0)
            for _ in range(NUM_LEGS)
        ]

        self._active_idx: int = 0
        self._active_leg: int = self._order[0]
        self._active_start: FootTarget = FootTarget(x=self._hold[self._active_leg].x, y=self._hold[self._active_leg].y, z=self._hold[self._active_leg].z)

    def is_complete(self) -> bool:
        return self._complete

    @staticmethod
    def _smoothstep(t: float) -> float:
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
        return t * t * (3.0 - 2.0 * t)

    def _begin_leg(self, idx: int) -> None:
        self._active_idx = idx
        if self._active_idx >= len(self._order):
            self._complete = True
            self.state = GaitState.STOPPED
            return
        self._active_leg = self._order[self._active_idx]
        h = self._hold[self._active_leg]
        self._active_start = FootTarget(x=h.x, y=h.y, z=h.z)
        self._seg_ms = 0

    def tick(self, dt_seconds: float = None) -> List[List[float]]:
        if self.state != GaitState.RUNNING or self._complete:
            return [foot.as_list() for foot in self._feet]

        # Update segment time
        if dt_seconds is not None:
            dt_ms = int(max(0.0, float(dt_seconds)) * 1000.0)
        else:
            dt_ms = 6  # ~166Hz fallback
        self._seg_ms += dt_ms

        seg_total = max(1, self.lift_ms + self.move_ms + self.place_ms)

        # Advance through completed legs if we overshoot
        while not self._complete and self._seg_ms >= seg_total:
            # Finish current leg at target
            leg = self._active_leg
            tgt = self._target[leg]
            self._hold[leg] = FootTarget(x=tgt.x, y=tgt.y, z=tgt.z)
            self._seg_ms -= seg_total
            self._begin_leg(self._active_idx + 1)

        # Default: hold all legs
        for i in range(NUM_LEGS):
            h = self._hold[i]
            self._feet[i] = FootTarget(x=h.x, y=h.y, z=h.z)

        if self._complete:
            return [foot.as_list() for foot in self._feet]

        # Compute active leg stepped trajectory
        leg = self._active_leg
        start = self._active_start
        tgt = self._target[leg]
        lift_y = start.y + self.lift_mm

        t_ms = self._seg_ms
        if t_ms < self.lift_ms:
            u = self._smoothstep(t_ms / max(1, self.lift_ms))
            x = start.x
            z = start.z
            y = start.y + (lift_y - start.y) * u
        elif t_ms < (self.lift_ms + self.move_ms):
            u = self._smoothstep((t_ms - self.lift_ms) / max(1, self.move_ms))
            x = start.x + (tgt.x - start.x) * u
            z = start.z + (tgt.z - start.z) * u
            y = lift_y
        else:
            u = self._smoothstep((t_ms - self.lift_ms - self.move_ms) / max(1, self.place_ms))
            x = tgt.x
            z = tgt.z
            y = lift_y + (tgt.y - lift_y) * u

        self._feet[leg] = FootTarget(x=float(x), y=float(y), z=float(z))
        return [foot.as_list() for foot in self._feet]


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
# Kinematic Move: Pounce / Spider Jump Attack
#----------------------------------------------------------------------------------------------------------------------

class PounceAttack(GaitEngine):
    """Short kinematic "pounce" sequence (spider-like jump attack).

    This is not a cyclic gait; it is a timed sequence of poses blended
    smoothly:
      1) Move back + crouch on rear 4 legs while lifting front legs
      2) Spring forward: front legs reach forward and drop to "strike"
      3) Recover back to neutral stance

    Output is compatible with the Teensy FEET command.
    """

    # Legs
    _FRONT = (LEG_LF, LEG_RF)
    _REAR4 = (LEG_LM, LEG_LR, LEG_RM, LEG_RR)

    def __init__(
        self,
        params: Optional[GaitParams] = None,
        prep_ms: int = 400,
        rear_ms: int = 250,
        lunge_ms: int = 250,
        recover_ms: int = 500,
        back1_z_mm: float = 55.0,
        back2_z_mm: float = 75.0,
        push_z_mm: float = -60.0,
        strike_z_mm: float = 140.0,
        crouch_dy_mm: float = 55.0,
        lift_dy_mm: float = 110.0,
        front_z_mm: float = 20.0,
    ):
        super().__init__(params)
        self.prep_ms = int(prep_ms)
        self.rear_ms = int(rear_ms)
        self.lunge_ms = int(lunge_ms)
        self.recover_ms = int(recover_ms)

        # Pose parameters (mm)
        self.back1_z_mm = float(back1_z_mm)
        self.back2_z_mm = float(back2_z_mm)
        self.push_z_mm = float(push_z_mm)
        self.strike_z_mm = float(strike_z_mm)
        self.crouch_dy_mm = float(crouch_dy_mm)
        self.lift_dy_mm = float(lift_dy_mm)
        self.front_z_mm = float(front_z_mm)

        self._elapsed_ms = 0
        self._start_time = None

        # Initialize feet to a neutral stance
        self._set_pose(self._pose_home())

    def start(self) -> None:
        self._start_time = time.monotonic()
        self._elapsed_ms = 0
        self.state = GaitState.RUNNING
        self._set_pose(self._pose_home())

    def is_complete(self) -> bool:
        return self.state != GaitState.RUNNING

    def tick(self, dt_seconds: float = None) -> List[List[float]]:
        if self.state != GaitState.RUNNING:
            return [foot.as_list() for foot in self._feet]

        if dt_seconds is not None:
            self._elapsed_ms += int(max(0.0, float(dt_seconds)) * 1000.0)
        elif self._start_time is not None:
            self._elapsed_ms = int((time.monotonic() - self._start_time) * 1000)

        total_ms = self.prep_ms + self.rear_ms + self.lunge_ms + self.recover_ms
        t = max(0, min(self._elapsed_ms, total_ms))

        home = self._pose_home()
        prep = self._pose_prep()
        rear = self._pose_rear()
        lunge = self._pose_lunge()

        if t < self.prep_ms:
            pose = self._blend_pose(home, prep, self._ease(t / max(1, self.prep_ms)))
        elif t < self.prep_ms + self.rear_ms:
            tt = (t - self.prep_ms) / max(1, self.rear_ms)
            pose = self._blend_pose(prep, rear, self._ease(tt))
        elif t < self.prep_ms + self.rear_ms + self.lunge_ms:
            tt = (t - self.prep_ms - self.rear_ms) / max(1, self.lunge_ms)
            pose = self._blend_pose(rear, lunge, self._ease(tt))
        else:
            tt = (t - self.prep_ms - self.rear_ms - self.lunge_ms) / max(1, self.recover_ms)
            pose = self._blend_pose(lunge, home, self._ease(tt))

        self._set_pose(pose)

        if self._elapsed_ms >= total_ms:
            self.state = GaitState.STOPPED

        return [foot.as_list() for foot in self._feet]

    @staticmethod
    def _ease(x: float) -> float:
        """Cubic ease-in-out in [0,1]."""
        x = max(0.0, min(1.0, float(x)))
        return 3.0 * x * x - 2.0 * x * x * x

    def _clamp_y(self, y: float) -> float:
        """Keep feet Y in a conservative range (Y is vertical; negative = down)."""
        # Avoid commanding feet above the hip plane (y >= 0). Keep at least -30mm.
        return min(float(y), -30.0)

    def _pose_home(self) -> List[Tuple[float, float, float]]:
        bx = float(self.params.base_x_mm)
        by = float(self.params.base_y_mm)
        return [(bx, by, 0.0) for _ in range(NUM_LEGS)]

    def _pose_prep(self) -> List[Tuple[float, float, float]]:
        bx = float(self.params.base_x_mm)
        by = float(self.params.base_y_mm)

        # Prep pose is a partial crouch/lift before the full rear set.
        crouch_y = self._clamp_y(by + 0.75 * self.crouch_dy_mm)
        lift_y = self._clamp_y(by + 0.80 * self.lift_dy_mm)
        back_z = self.back1_z_mm   # rear feet forward -> body shifts back
        front_z = self.front_z_mm

        pose = [(bx, by, 0.0) for _ in range(NUM_LEGS)]
        for leg in self._REAR4:
            pose[leg] = (bx, crouch_y, back_z)
        for leg in self._FRONT:
            pose[leg] = (bx, lift_y, front_z)
        return pose

    def _pose_rear(self) -> List[Tuple[float, float, float]]:
        bx = float(self.params.base_x_mm)
        by = float(self.params.base_y_mm)

        crouch_y = self._clamp_y(by + self.crouch_dy_mm)
        lift_y = self._clamp_y(by + self.lift_dy_mm)
        back_z = self.back2_z_mm
        # Keep front feet nearer to the body as they lift.
        front_z = max(0.0, self.front_z_mm - 10.0)

        pose = [(bx, by, 0.0) for _ in range(NUM_LEGS)]
        for leg in self._REAR4:
            pose[leg] = (bx, crouch_y, back_z)
        for leg in self._FRONT:
            pose[leg] = (bx, lift_y, front_z)
        return pose

    def _pose_lunge(self) -> List[Tuple[float, float, float]]:
        bx = float(self.params.base_x_mm)
        by = float(self.params.base_y_mm)

        # Front legs reach forward and drop to ground.
        strike_z = self.strike_z_mm
        # Rear legs push backward to "launch".
        push_z = self.push_z_mm

        pose = [(bx, by, 0.0) for _ in range(NUM_LEGS)]
        for leg in self._FRONT:
            pose[leg] = (bx, by, strike_z)
        for leg in self._REAR4:
            pose[leg] = (bx, by, push_z)
        return pose

    @staticmethod
    def _blend_pose(a: List[Tuple[float, float, float]], b: List[Tuple[float, float, float]], t: float) -> List[Tuple[float, float, float]]:
        t = max(0.0, min(1.0, float(t)))
        out: List[Tuple[float, float, float]] = []
        for i in range(NUM_LEGS):
            ax, ay, az = a[i]
            bx, by, bz = b[i]
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t))
        return out

    def _set_pose(self, pose: List[Tuple[float, float, float]]) -> None:
        for i in range(NUM_LEGS):
            x, y, z = pose[i]
            self._feet[i] = FootTarget(x=float(x), y=float(y), z=float(z))


#----------------------------------------------------------------------------------------------------------------------
# Gait Transition Manager
#----------------------------------------------------------------------------------------------------------------------

class GaitTransitionState(Enum):
    """State machine for gait transitions."""
    IDLE = auto()           # No transition pending
    WAITING = auto()        # Waiting for phase boundary in current gait
    BLENDING = auto()       # Actively blending between gaits
    COMPLETE = auto()       # Transition finished, ready to clean up


class GaitTransition:
    """Manages smooth phase-locked transitions between gait types.
    
    Strategy: Phase-Locked with Blending
    1. When transition requested, mark pending and wait for current gait's phase end
    2. At phase boundary, start blending: run both gaits in parallel
    3. Interpolate foot positions from old to new gait over blend duration
    4. Complete transition when blend finished
    
    This ensures no leg is mid-swing when transition starts, and provides
    smooth motion interpolation during the switch.
    """
    
    # Default blend duration (ms) - one phase of the faster gait
    DEFAULT_BLEND_MS = 500
    
    def __init__(self, blend_ms: int = None):
        self.state = GaitTransitionState.IDLE
        self.blend_ms = blend_ms or self.DEFAULT_BLEND_MS
        
        # Gaits involved in transition
        self._from_gait: Optional[GaitEngine] = None
        self._to_gait: Optional[GaitEngine] = None
        
        # Transition timing
        self._blend_start_ms: int = 0
        self._blend_elapsed_ms: int = 0
        
        # Track phase at transition request (to detect boundary crossing)
        self._waiting_phase: int = -1
    
    def request_transition(self, from_gait: GaitEngine, to_gait: GaitEngine) -> bool:
        """Request a transition from current gait to new gait.
        
        Args:
            from_gait: Currently running gait engine
            to_gait: New gait engine to transition to (should be initialized but not started)
            
        Returns:
            True if transition was queued, False if already transitioning
        """
        if self.state != GaitTransitionState.IDLE:
            return False  # Already transitioning
        
        self._from_gait = from_gait
        self._to_gait = to_gait
        self._waiting_phase = from_gait._phase if hasattr(from_gait, '_phase') else 0
        self.state = GaitTransitionState.WAITING
        return True
    
    def tick(self, dt_seconds: float = None) -> Tuple[Optional[List[List[float]]], bool]:
        """Process one tick of the transition.
        
        Call this instead of the gait engine's tick() when a transition is active.
        
        Args:
            dt_seconds: Time since last tick
            
        Returns:
            Tuple of (foot_targets, is_complete):
            - foot_targets: 6x3 array of foot positions, or None if no transition active
            - is_complete: True when transition is finished (caller should switch to new gait)
        """
        if self.state == GaitTransitionState.IDLE:
            return None, False
        
        if self.state == GaitTransitionState.WAITING:
            # Tick the from_gait normally
            feet_from = self._from_gait.tick(dt_seconds)
            
            # Check if phase boundary crossed (for tripod) or use general check
            current_phase = getattr(self._from_gait, '_phase', 0)
            wave_phase = getattr(self._from_gait, '_wave_phase', -1)
            ripple_phase = getattr(self._from_gait, '_ripple_phase', -1)
            
            # Detect any phase transition
            phase_changed = False
            if wave_phase >= 0:  # WaveGait
                phase_changed = wave_phase != self._waiting_phase
            elif ripple_phase >= 0:  # RippleGait
                phase_changed = ripple_phase != self._waiting_phase
            else:  # TripodGait or others
                phase_changed = current_phase != self._waiting_phase
            
            if phase_changed:
                # Phase boundary reached - start blending
                self.state = GaitTransitionState.BLENDING
                self._blend_start_ms = self._from_gait._elapsed_ms
                self._blend_elapsed_ms = 0
                
                # Start the new gait (copy relevant timing from old gait)
                self._to_gait.params.speed_scale = self._from_gait.params.speed_scale
                self._to_gait.params.heading_deg = self._from_gait.params.heading_deg
                self._to_gait.params.turn_rate_deg_s = self._from_gait.params.turn_rate_deg_s
                self._to_gait._smoothed_speed = self._from_gait._smoothed_speed
                self._to_gait._smoothed_heading = self._from_gait._smoothed_heading
                self._to_gait._smoothed_turn_rate = self._from_gait._smoothed_turn_rate
                self._to_gait.start()
            
            return feet_from, False
        
        if self.state == GaitTransitionState.BLENDING:
            # Tick both gaits
            feet_from = self._from_gait.tick(dt_seconds)
            feet_to = self._to_gait.tick(dt_seconds)
            
            # Update blend elapsed time
            dt_ms = int((dt_seconds or 0.006) * 1000)
            self._blend_elapsed_ms += dt_ms
            
            # Calculate blend factor (0 = all old, 1 = all new)
            blend_t = min(1.0, self._blend_elapsed_ms / self.blend_ms)
            
            # Smooth blend using ease-in-out (cosine interpolation)
            # This gives smooth acceleration/deceleration at blend boundaries
            blend_factor = 0.5 * (1.0 - math.cos(blend_t * math.pi))
            
            # Interpolate foot positions
            blended_feet = []
            for i in range(NUM_LEGS):
                blended = [
                    feet_from[i][0] * (1.0 - blend_factor) + feet_to[i][0] * blend_factor,
                    feet_from[i][1] * (1.0 - blend_factor) + feet_to[i][1] * blend_factor,
                    feet_from[i][2] * (1.0 - blend_factor) + feet_to[i][2] * blend_factor,
                ]
                blended_feet.append(blended)
            
            # Check if blend complete
            if blend_t >= 1.0:
                self.state = GaitTransitionState.COMPLETE
                return blended_feet, True
            
            return blended_feet, False
        
        if self.state == GaitTransitionState.COMPLETE:
            return None, True
        
        return None, False
    
    def get_target_gait(self) -> Optional[GaitEngine]:
        """Return the gait we're transitioning to (for caller to use after completion)."""
        return self._to_gait
    
    def reset(self) -> None:
        """Reset transition state (call after completing transition)."""
        self.state = GaitTransitionState.IDLE
        self._from_gait = None
        self._to_gait = None
        self._waiting_phase = -1
        self._blend_start_ms = 0
        self._blend_elapsed_ms = 0
    
    def is_active(self) -> bool:
        """Check if a transition is in progress."""
        return self.state != GaitTransitionState.IDLE
    
    def get_state_name(self) -> str:
        """Get human-readable state name."""
        return self.state.name


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

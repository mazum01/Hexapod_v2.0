"""Free Gait — Adaptive Event-Driven Locomotion

This module implements a free gait system where leg timing is event-driven
rather than phase-locked. Each leg has its own state machine, and a central
coordinator grants swing permission based on stability constraints.

Architecture:
    FreeGaitCoordinator
        ├── Computes support polygon from stance feet
        ├── Checks CoG stability margin
        └── Grants swing permission to ready legs

    Leg (×6)
        ├── State machine: STANCE → LIFT_PENDING → SWING → PLACING → STANCE
        └── Transitions driven by coordinator + contact events

    FootPlacementPlanner
        ├── Computes target position based on heading/speed
        └── Pre-checks collision before committing

References:
    - Quasi-static free gait assumes slow walking (dynamics negligible)
    - Support polygon: convex hull of stance feet in XZ plane
    - Stability margin: distance from CoG projection to polygon edge
    - Mostafa, K., Her, I., & Wu, Y.-H. (2012). "The Offset Model of a Hexapod
      Robot and the Effect of the Offset Parameter." Int. J. Manufacturing,
      Materials, and Mechanical Engineering, 2(3), 52-59. (Offset model for
      improved stability margin via middle-leg fore-aft shift.)
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict
import time


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

NUM_LEGS = 6

# Leg indices (matches firmware/gait_engine convention)
LEG_LF = 0  # Left Front
LEG_LM = 1  # Left Middle
LEG_LR = 2  # Left Rear
LEG_RF = 3  # Right Front
LEG_RM = 4  # Right Middle
LEG_RR = 5  # Right Rear

LEG_NAMES = ["LF", "LM", "LR", "RF", "RM", "RR"]


# -----------------------------------------------------------------------------
# FG1: Per-Leg State Machine
# -----------------------------------------------------------------------------

class LegState(Enum):
    """State machine states for each leg in free gait.
    
    Transitions:
        STANCE → LIFT_PENDING: Coordinator grants swing permission
        LIFT_PENDING → SWING: Lift trajectory complete (foot cleared ground)
        SWING → PLACING: Horizontal motion complete, begin descent
        PLACING → STANCE: Contact detected or target Y reached
    """
    STANCE = auto()        # Foot on ground, bearing weight
    LIFT_PENDING = auto()  # Approved to lift, executing lift trajectory
    SWING = auto()         # Foot in air, moving toward target XZ
    PLACING = auto()       # Descending toward ground, awaiting contact


class ContactSource(Enum):
    """How contact was determined."""
    NONE = auto()      # No contact information
    SENSED = auto()    # From foot switch (S4 telemetry)
    ESTIMATED = auto() # From position/time heuristic


@dataclass
class FootTarget:
    """Target foot position in body frame (mm)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_list(self) -> List[float]:
        return [self.x, self.y, self.z]

    def copy(self) -> 'FootTarget':
        return FootTarget(x=self.x, y=self.y, z=self.z)


@dataclass
class Leg:
    """Per-leg state for free gait control.
    
    Each leg maintains its own state machine and tracks contact, timing,
    and trajectory progress independently. The coordinator queries leg
    readiness and grants swing permission.
    
    Attributes:
        index: Leg index (0-5)
        name: Human-readable name ("LF", "LM", etc.)
        state: Current state machine state
        foot_current: Current foot position (from FK or command)
        foot_target: Target foot position for current/next step
        swing_progress: 0.0-1.0 progress through swing phase
        contact_sensed: True if foot switch reports contact
        contact_estimated: True if position/time heuristics suggest contact
        contact_source: How contact was determined
        time_in_state_ms: Milliseconds since last state transition
        last_transition_time: Monotonic timestamp of last transition
        stance_start_time: When this stance phase began (for priority)
        lift_height_mm: How high to lift during swing
    """
    index: int
    name: str = field(init=False)
    state: LegState = LegState.STANCE
    
    # Position tracking
    foot_current: FootTarget = field(default_factory=FootTarget)
    foot_target: FootTarget = field(default_factory=FootTarget)
    
    # Swing progress (0.0 = start, 1.0 = complete)
    swing_progress: float = 0.0
    
    # Contact tracking
    contact_sensed: bool = True  # Assume contact at init (standing)
    contact_estimated: bool = True
    contact_source: ContactSource = ContactSource.ESTIMATED
    
    # Timing
    time_in_state_ms: float = 0.0
    last_transition_time: float = field(default_factory=time.monotonic)
    stance_start_time: float = field(default_factory=time.monotonic)
    
    # Trajectory parameters
    lift_height_mm: float = 50.0
    
    def __post_init__(self):
        self.name = LEG_NAMES[self.index] if 0 <= self.index < NUM_LEGS else f"L{self.index}"

    # -------------------------------------------------------------------------
    # State Queries
    # -------------------------------------------------------------------------

    def is_stance(self) -> bool:
        """True if leg is in stance (supporting body weight)."""
        return self.state == LegState.STANCE

    def is_swinging(self) -> bool:
        """True if leg is in any swing-related state (not stance)."""
        return self.state in (LegState.LIFT_PENDING, LegState.SWING, LegState.PLACING)

    def has_contact(self) -> bool:
        """True if contact is detected (sensed or estimated)."""
        return self.contact_sensed or self.contact_estimated

    def time_in_stance_ms(self) -> float:
        """How long this leg has been in current stance phase."""
        if self.state != LegState.STANCE:
            return 0.0
        return (time.monotonic() - self.stance_start_time) * 1000.0

    # -------------------------------------------------------------------------
    # State Transitions
    # -------------------------------------------------------------------------

    def _transition_to(self, new_state: LegState) -> None:
        """Internal: perform state transition with timing update."""
        now = time.monotonic()
        self.time_in_state_ms = 0.0
        self.last_transition_time = now
        self.state = new_state
        
        if new_state == LegState.STANCE:
            self.stance_start_time = now
            self.swing_progress = 0.0

    def request_swing(self) -> bool:
        """Called by coordinator to grant swing permission.
        
        Returns:
            True if transition occurred, False if not in valid state.
        """
        if self.state != LegState.STANCE:
            return False
        
        self._transition_to(LegState.LIFT_PENDING)
        self.swing_progress = 0.0
        self.contact_sensed = False
        self.contact_estimated = False
        self.contact_source = ContactSource.NONE
        return True

    def complete_lift(self) -> bool:
        """Called when lift trajectory is complete (foot cleared ground).
        
        Returns:
            True if transition occurred, False if not in valid state.
        """
        if self.state != LegState.LIFT_PENDING:
            return False
        
        self._transition_to(LegState.SWING)
        return True

    def begin_placing(self) -> bool:
        """Called when horizontal motion complete, begin descent.
        
        Returns:
            True if transition occurred, False if not in valid state.
        """
        if self.state != LegState.SWING:
            return False
        
        self._transition_to(LegState.PLACING)
        return True

    def confirm_contact(self, sensed: bool = False) -> bool:
        """Called when contact detected (foot on ground).
        
        Args:
            sensed: True if from foot switch, False if estimated.
            
        Returns:
            True if transition occurred, False if not in valid state.
        """
        if self.state != LegState.PLACING:
            # Can also force contact during SWING (early contact)
            if self.state == LegState.SWING:
                pass  # Allow early contact
            else:
                return False
        
        self.contact_sensed = sensed
        self.contact_estimated = not sensed
        self.contact_source = ContactSource.SENSED if sensed else ContactSource.ESTIMATED
        self.swing_progress = 1.0
        self._transition_to(LegState.STANCE)
        return True

    def emergency_plant(self) -> None:
        """Force immediate transition to stance (stability emergency).
        
        Called by coordinator if stability margin drops critically.
        """
        self.contact_estimated = True
        self.contact_source = ContactSource.ESTIMATED
        self.swing_progress = 1.0
        self._transition_to(LegState.STANCE)

    # -------------------------------------------------------------------------
    # Timing Update
    # -------------------------------------------------------------------------

    def update_timing(self, dt_ms: float) -> None:
        """Update time-in-state counter.
        
        Args:
            dt_ms: Time since last update in milliseconds.
        """
        self.time_in_state_ms += dt_ms

    # -------------------------------------------------------------------------
    # Contact Update
    # -------------------------------------------------------------------------

    def update_contact(self, sensed: Optional[bool] = None, 
                       current_y: Optional[float] = None,
                       target_y: Optional[float] = None,
                       tolerance_mm: float = 5.0) -> None:
        """Update contact status from sensors and/or estimation.
        
        Args:
            sensed: Foot switch state (True=contact), or None if unavailable.
            current_y: Current foot Y position (for estimation).
            target_y: Target foot Y position (for estimation).
            tolerance_mm: Y tolerance for position-based estimation.
        """
        # Sensed contact from foot switch
        if sensed is not None:
            self.contact_sensed = sensed
            if sensed:
                self.contact_source = ContactSource.SENSED
        
        # Estimated contact from position
        if current_y is not None and target_y is not None:
            self.contact_estimated = abs(current_y - target_y) < tolerance_mm
            if self.contact_estimated and not self.contact_sensed:
                self.contact_source = ContactSource.ESTIMATED

    # -------------------------------------------------------------------------
    # Debug / String
    # -------------------------------------------------------------------------

    def __repr__(self) -> str:
        contact = "C" if self.has_contact() else "-"
        return f"Leg({self.name}, {self.state.name}, {contact}, {self.time_in_state_ms:.0f}ms)"

    def status_string(self) -> str:
        """Short status for logging/display."""
        state_char = {
            LegState.STANCE: "S",
            LegState.LIFT_PENDING: "L",
            LegState.SWING: "W",
            LegState.PLACING: "P",
        }.get(self.state, "?")
        
        contact_char = "●" if self.has_contact() else "○"
        return f"{self.name}:{state_char}{contact_char}"


# -----------------------------------------------------------------------------
# Leg Set Factory
# -----------------------------------------------------------------------------

def create_legs(base_x_mm: float = 120.0, 
                base_y_mm: float = -60.0,
                lift_height_mm: float = 50.0) -> List[Leg]:
    """Create a set of 6 legs initialized in stance at neutral positions.
    
    Uses the SAME coordinate convention as TripodGait for FEET command compatibility:
    - X is always POSITIVE (outward reach) - firmware handles L/R mirroring
    - Z is the stride position (front=positive, rear=negative)
    - Y is vertical (negative = below body)
    
    Args:
        base_x_mm: Lateral offset from body center (stance width).
        base_y_mm: Vertical offset (negative = below body).
        lift_height_mm: Default lift height during swing.
        
    Returns:
        List of 6 Leg instances with proper neutral positions.
    """
    import math
    
    # Leg base rotation angles (matching gait_engine.py LEG_BASE_ROTATION_DEG)
    # 0=LF, 1=LM, 2=LR, 3=RF, 4=RM, 5=RR
    LEG_ROTATIONS_DEG = [45.0, 0.0, -45.0, 45.0, 0.0, -45.0]
    
    legs = []
    for i in range(NUM_LEGS):
        leg = Leg(index=i)
        
        # Use TripodGait coordinate convention:
        # X = base_x_mm * cos(angle) - positive for all legs (firmware mirrors left side)
        # Z = 0 at neutral (stride offset is applied during movement)
        angle_rad = math.radians(LEG_ROTATIONS_DEG[i])
        cos_a = math.cos(angle_rad)
        
        # X is positive for ALL legs - firmware handles left-side mirroring
        x = base_x_mm * cos_a
        
        # Z is stride offset (0 at neutral)
        z = 0.0
        
        leg.foot_current = FootTarget(x=x, y=base_y_mm, z=z)
        leg.foot_target = FootTarget(x=x, y=base_y_mm, z=z)
        leg.lift_height_mm = lift_height_mm
        legs.append(leg)
    return legs


# -----------------------------------------------------------------------------
# FG2: Support Polygon Computation
# -----------------------------------------------------------------------------

def _cross_2d(o: Tuple[float, float], 
              a: Tuple[float, float], 
              b: Tuple[float, float]) -> float:
    """2D cross product of vectors OA and OB.
    
    Positive if counter-clockwise, negative if clockwise, zero if collinear.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull_2d(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Compute convex hull of 2D points using Andrew's monotone chain algorithm.
    
    Args:
        points: List of (x, z) tuples.
        
    Returns:
        List of hull vertices in counter-clockwise order.
        Returns empty list if fewer than 3 unique points.
    """
    # Remove duplicates and sort
    pts = sorted(set(points))
    
    if len(pts) < 3:
        return pts  # Line or point, not a polygon
    
    # Build lower hull
    lower = []
    for p in pts:
        while len(lower) >= 2 and _cross_2d(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    
    # Build upper hull
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross_2d(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    
    # Concatenate (remove last point of each half because it's repeated)
    return lower[:-1] + upper[:-1]


def compute_support_polygon(stance_feet: List[Tuple[float, float]]) -> Optional[List[Tuple[float, float]]]:
    """Compute support polygon from stance foot positions.
    
    The support polygon is the convex hull of all feet currently in stance,
    projected onto the XZ (ground) plane. The body's center of gravity must
    remain inside this polygon for static stability.
    
    Args:
        stance_feet: List of (x, z) positions for feet in stance.
        
    Returns:
        List of (x, z) hull vertices in counter-clockwise order,
        or None if fewer than 3 stance feet (degenerate/unstable).
    """
    if len(stance_feet) < 3:
        return None  # Can't form a polygon
    
    hull = convex_hull_2d(stance_feet)
    
    if len(hull) < 3:
        return None  # Collinear points
    
    return hull


# Hip positions in body frame (mm) for computing body-frame foot locations
# X = lateral (positive = right side), Z = fore/aft (positive = front)
# Must match gait_engine.py LEG_HIP_X and LEG_HIP_Z
_LEG_HIP_X = [-80.0, -100.0, -80.0, 80.0, 100.0, 80.0]  # LF, LM, LR, RF, RM, RR
_LEG_HIP_Z = [100.0, 0.0, -100.0, 100.0, 0.0, -100.0]   # front/middle/rear

# Leg base rotation angles for projecting outward reach to body frame
_LEG_ANGLES_DEG = [-45.0, 0.0, 45.0, 45.0, 0.0, -45.0]


def _foot_to_body_frame(leg_idx: int, foot_x: float, foot_z: float) -> Tuple[float, float]:
    """Convert leg-local foot position to body-frame XZ coordinates.
    
    Args:
        leg_idx: Leg index (0-5).
        foot_x: Outward reach (always positive in FEET command).
        foot_z: Stride offset (fore/aft relative to leg).
        
    Returns:
        (body_x, body_z) in body frame (mm).
    """
    import math
    angle_rad = math.radians(_LEG_ANGLES_DEG[leg_idx])
    
    # Left legs (0,1,2) extend in negative X direction, right legs (3,4,5) in positive X
    is_left = leg_idx < 3
    
    # Angle convention differs by side:
    # LEFT:  negative angle = forward (LF=-45°), positive = backward (LR=+45°)
    # RIGHT: positive angle = forward (RF=+45°), negative = backward (RR=-45°)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # Direction: left legs have negative X direction, right legs positive
    direction = -1.0 if is_left else 1.0
    
    body_x = _LEG_HIP_X[leg_idx] + direction * foot_x * cos_a
    
    # For Z offset from reach:
    # LEFT legs: negate sin (negative angle = forward = positive Z offset)
    # RIGHT legs: keep sin (positive angle = forward = positive Z offset)
    z_sin_factor = -1.0 if is_left else 1.0
    body_z = _LEG_HIP_Z[leg_idx] + z_sin_factor * foot_x * sin_a + foot_z
    
    return (body_x, body_z)


def get_stance_feet_xz(legs: List['Leg']) -> List[Tuple[float, float]]:
    """Extract body-frame XZ positions of all stance feet.
    
    Converts leg-local foot positions to body-frame coordinates for
    support polygon computation. This accounts for hip positions and
    leg rotation angles.
    
    Args:
        legs: List of Leg instances.
        
    Returns:
        List of (body_x, body_z) tuples for feet currently in STANCE state.
    """
    result = []
    for leg in legs:
        if leg.is_stance():
            body_x, body_z = _foot_to_body_frame(
                leg.index,
                leg.foot_current.x,
                leg.foot_current.z
            )
            result.append((body_x, body_z))
    return result


def point_in_polygon(point: Tuple[float, float], 
                     polygon: List[Tuple[float, float]]) -> bool:
    """Test if a point is inside a convex polygon (2D).
    
    Uses cross-product sign consistency test. Point is inside if it's
    on the same side of all edges (all cross products have same sign).
    
    Args:
        point: (x, z) point to test.
        polygon: List of (x, z) vertices in counter-clockwise order.
        
    Returns:
        True if point is inside or on the boundary of the polygon.
    """
    if len(polygon) < 3:
        return False
    
    n = len(polygon)
    sign = None
    
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]
        cross = _cross_2d(p1, p2, point)
        
        if cross != 0:
            if sign is None:
                sign = cross > 0
            elif (cross > 0) != sign:
                return False
    
    return True


def distance_to_polygon_edge(point: Tuple[float, float],
                             polygon: List[Tuple[float, float]]) -> float:
    """Compute minimum distance from point to polygon boundary.
    
    Positive if point is inside, negative if outside.
    This is the "stability margin" — how far the CoG can move before
    exiting the support polygon.
    
    Args:
        point: (x, z) point (typically CoG projection).
        polygon: List of (x, z) vertices in counter-clockwise order.
        
    Returns:
        Signed distance in mm. Positive = inside (stable), negative = outside.
    """
    if len(polygon) < 3:
        return float('-inf')
    
    import math
    
    n = len(polygon)
    min_dist = float('inf')
    inside = point_in_polygon(point, polygon)
    
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]
        
        # Vector from p1 to p2
        edge_x = p2[0] - p1[0]
        edge_z = p2[1] - p1[1]
        edge_len_sq = edge_x * edge_x + edge_z * edge_z
        
        if edge_len_sq < 1e-10:
            # Degenerate edge
            dist = math.sqrt((point[0] - p1[0])**2 + (point[1] - p1[1])**2)
        else:
            # Project point onto edge line
            t = ((point[0] - p1[0]) * edge_x + (point[1] - p1[1]) * edge_z) / edge_len_sq
            t = max(0.0, min(1.0, t))  # Clamp to edge segment
            
            closest_x = p1[0] + t * edge_x
            closest_z = p1[1] + t * edge_z
            
            dist = math.sqrt((point[0] - closest_x)**2 + (point[1] - closest_z)**2)
        
        if dist < min_dist:
            min_dist = dist
    
    return min_dist if inside else -min_dist


# -----------------------------------------------------------------------------
# FG3: Center of Gravity Estimation
# -----------------------------------------------------------------------------

# Approximate leg mass as fraction of total body mass
# Each leg is ~3% of total; used for optional weighted CoG
LEG_MASS_FRACTION = 0.03

# Body height above ground at neutral stance (mm)
# Used for projecting CoG onto ground plane
BODY_HEIGHT_MM = 100.0


def estimate_cog_simple() -> Tuple[float, float, float]:
    """Estimate center of gravity as body center.
    
    For quasi-static walking at slow speeds, the body CoG can be
    approximated as the body frame origin (0, 0, 0). This is sufficient
    when leg masses are small relative to body mass and the body
    remains level.
    
    Returns:
        (x, y, z) CoG in body frame (mm). Always (0, 0, 0).
    """
    return (0.0, 0.0, 0.0)


def estimate_cog_weighted(leg_positions: List[Tuple[float, float, float]],
                          body_cog: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                          leg_mass_fraction: float = LEG_MASS_FRACTION) -> Tuple[float, float, float]:
    """Estimate center of gravity with leg mass contributions.
    
    Computes a weighted average of the body CoG and all leg positions,
    where each leg contributes leg_mass_fraction of the total weight.
    This provides a more accurate CoG when legs are extended far from body.
    
    The leg position is approximated as the foot position (endpoint),
    which is reasonable since most leg mass is near the distal end
    (motors near joints, but lever arm matters more).
    
    Args:
        leg_positions: List of 6 (x, y, z) foot positions in body frame.
        body_cog: Body center of mass in body frame (default origin).
        leg_mass_fraction: Mass of each leg as fraction of total (default 3%).
        
    Returns:
        (x, y, z) weighted CoG in body frame (mm).
    """
    if len(leg_positions) != 6:
        return body_cog  # Fallback to body center
    
    total_leg_mass = 6 * leg_mass_fraction
    body_mass = 1.0 - total_leg_mass  # Remaining mass is body
    
    # Start with body contribution
    cog_x = body_cog[0] * body_mass
    cog_y = body_cog[1] * body_mass
    cog_z = body_cog[2] * body_mass
    
    # Add leg contributions
    for pos in leg_positions:
        cog_x += pos[0] * leg_mass_fraction
        cog_y += pos[1] * leg_mass_fraction
        cog_z += pos[2] * leg_mass_fraction
    
    return (cog_x, cog_y, cog_z)


def project_cog_to_ground(cog: Tuple[float, float, float],
                          pitch_deg: float = 0.0,
                          roll_deg: float = 0.0,
                          body_height_mm: float = BODY_HEIGHT_MM) -> Tuple[float, float]:
    """Project 3D CoG onto ground plane accounting for body tilt.
    
    When the body is tilted, the CoG projection onto the ground shifts
    away from directly below the body center. This function computes
    where the CoG projects onto the XZ (ground) plane, which is needed
    for stability margin calculations.
    
    Uses small-angle approximation for typical walking tilts (<15°).
    
    Coordinate system:
        - Positive pitch: nose down (shifts CoG projection forward +Z)
        - Positive roll: right side down (shifts CoG projection right +X)
    
    Args:
        cog: (x, y, z) center of gravity in body frame (mm).
        pitch_deg: Body pitch angle in degrees.
        roll_deg: Body roll angle in degrees.
        body_height_mm: Height of body frame origin above ground.
        
    Returns:
        (x, z) CoG projection on ground plane in body frame (mm).
    """
    import math
    
    # Total height from CoG to ground (body height minus CoG Y offset)
    # Note: Y is up, so positive cog[1] means CoG is above origin
    total_height = body_height_mm - cog[1]
    
    if total_height <= 0:
        # CoG at or below ground - just return XZ directly
        return (cog[0], cog[2])
    
    # Convert to radians
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)
    
    # Projection shift due to tilt (small angle: tan(θ) ≈ θ for small θ)
    # Pitch tilts about X axis → affects Z projection
    # Roll tilts about Z axis → affects X projection
    dz = total_height * math.tan(pitch_rad)
    dx = total_height * math.tan(roll_rad)
    
    return (cog[0] + dx, cog[2] + dz)


def estimate_cog(leg_positions: Optional[List[Tuple[float, float, float]]] = None,
                 pitch_deg: float = 0.0,
                 roll_deg: float = 0.0,
                 use_weighted: bool = False,
                 body_height_mm: float = BODY_HEIGHT_MM) -> Tuple[float, float]:
    """Estimate center of gravity projected onto ground plane.
    
    Main entry point for CoG estimation. Combines:
    1. Simple or weighted 3D CoG estimation
    2. Projection onto ground plane with IMU tilt compensation
    
    For quasi-static walking, the simple method (body center) is usually
    sufficient. The weighted method is more accurate when legs are
    extended far from the body or when precision is needed.
    
    Args:
        leg_positions: List of 6 (x, y, z) foot positions for weighted mode.
                      Ignored if use_weighted=False.
        pitch_deg: Body pitch angle from IMU (degrees).
        roll_deg: Body roll angle from IMU (degrees).
        use_weighted: If True, use leg-mass-weighted CoG estimation.
        body_height_mm: Height of body frame origin above ground.
        
    Returns:
        (x, z) CoG projection on ground plane in body frame (mm).
        This is suitable for passing to distance_to_polygon_edge().
    """
    # Estimate 3D CoG
    if use_weighted and leg_positions is not None:
        cog_3d = estimate_cog_weighted(leg_positions)
    else:
        cog_3d = estimate_cog_simple()
    
    # Project to ground plane with tilt compensation
    return project_cog_to_ground(cog_3d, pitch_deg, roll_deg, body_height_mm)


# -----------------------------------------------------------------------------
# FG4: Stability Margin Computation
# -----------------------------------------------------------------------------

class StabilityStatus(Enum):
    """Stability classification based on margin thresholds.
    
    Used by coordinator to make leg swing permission decisions.
    """
    STABLE = auto()      # Margin > min_margin: safe to consider swings
    MARGINAL = auto()    # Margin > critical but < min_margin: caution
    CRITICAL = auto()    # Margin > 0 but < critical: emergency hold
    UNSTABLE = auto()    # Margin <= 0: CoG outside polygon


# Default stability thresholds (mm)
MIN_STABILITY_MARGIN_MM = 30.0   # Minimum margin to permit new swings
CRITICAL_MARGIN_MM = 10.0         # Below this, cancel pending lifts


def compute_stability_margin(cog_xz: Tuple[float, float],
                             support_polygon: Optional[List[Tuple[float, float]]]) -> float:
    """Compute stability margin from CoG to support polygon edge.
    
    The stability margin is the signed distance from the center of gravity
    projection to the nearest edge of the support polygon:
    - Positive: CoG is inside polygon (stable)
    - Negative: CoG is outside polygon (unstable, tipping)
    
    This is the core metric for free gait decisions. The coordinator uses
    this to decide whether a leg can safely be lifted for swing.
    
    Args:
        cog_xz: (x, z) center of gravity projection on ground plane.
        support_polygon: Convex hull vertices from compute_support_polygon(),
                        or None if fewer than 3 stance feet.
                        
    Returns:
        Stability margin in mm. Positive = stable, negative = unstable.
        Returns -inf if polygon is None (degenerate case).
    """
    if support_polygon is None or len(support_polygon) < 3:
        return float('-inf')
    
    return distance_to_polygon_edge(cog_xz, support_polygon)


def classify_stability(margin_mm: float,
                       min_margin_mm: float = MIN_STABILITY_MARGIN_MM,
                       critical_margin_mm: float = CRITICAL_MARGIN_MM) -> StabilityStatus:
    """Classify stability status based on margin thresholds.
    
    Used by FreeGaitCoordinator to make swing permission decisions:
    - STABLE: Safe to grant new swing permissions
    - MARGINAL: Proceed with caution, limit simultaneous swings
    - CRITICAL: Emergency hold — cancel any pending lifts
    - UNSTABLE: CoG outside polygon — immediate recovery needed
    
    Args:
        margin_mm: Stability margin from compute_stability_margin().
        min_margin_mm: Threshold for STABLE status (default 30mm).
        critical_margin_mm: Threshold for CRITICAL status (default 10mm).
        
    Returns:
        StabilityStatus enum value.
    """
    if margin_mm <= 0:
        return StabilityStatus.UNSTABLE
    elif margin_mm < critical_margin_mm:
        return StabilityStatus.CRITICAL
    elif margin_mm < min_margin_mm:
        return StabilityStatus.MARGINAL
    else:
        return StabilityStatus.STABLE


def check_swing_safe(legs: List['Leg'],
                     candidate_idx: int,
                     cog_xz: Tuple[float, float],
                     min_margin_mm: float = MIN_STABILITY_MARGIN_MM) -> Tuple[bool, float]:
    """Check if lifting a candidate leg would maintain stability.
    
    Simulates removing the candidate leg from stance and computes the
    resulting stability margin. Used by coordinator to decide if a
    swing request should be granted.
    
    Args:
        legs: List of 6 Leg instances.
        candidate_idx: Index of leg requesting swing permission.
        cog_xz: Current CoG projection on ground plane (body frame).
        min_margin_mm: Minimum margin required to approve swing.
        
    Returns:
        Tuple of (safe, margin_mm):
        - safe: True if swing would maintain margin > min_margin_mm
        - margin_mm: Stability margin after removing candidate leg
    """
    # Get stance feet excluding the candidate, converted to body frame
    stance_xz = []
    for i, leg in enumerate(legs):
        if leg.is_stance() and i != candidate_idx:
            body_x, body_z = _foot_to_body_frame(
                i, leg.foot_current.x, leg.foot_current.z
            )
            stance_xz.append((body_x, body_z))
    
    # Compute hypothetical support polygon
    polygon = compute_support_polygon(stance_xz)
    
    # Compute margin with reduced support
    margin = compute_stability_margin(cog_xz, polygon)
    
    return (margin >= min_margin_mm, margin)


def get_stability_color(status: StabilityStatus) -> Tuple[int, int, int]:
    """Get RGB color for stability status visualization.
    
    Args:
        status: StabilityStatus enum value.
        
    Returns:
        (R, G, B) color tuple for display.
    """
    colors = {
        StabilityStatus.STABLE: (60, 220, 100),    # Green
        StabilityStatus.MARGINAL: (255, 220, 60),  # Yellow
        StabilityStatus.CRITICAL: (255, 150, 50),  # Orange
        StabilityStatus.UNSTABLE: (255, 80, 80),   # Red
    }
    return colors.get(status, (128, 128, 128))


# -----------------------------------------------------------------------------
# FG7: Contact Estimation
# -----------------------------------------------------------------------------

class ContactMethod(Enum):
    """How contact was determined, for diagnostics.
    
    Priority order (highest to lowest):
        SENSED > POSITION > TIMEOUT > NONE
    """
    NONE = auto()      # No contact detected
    TIMEOUT = auto()   # Swing exceeded expected time (assume grounded)
    POSITION = auto()  # Foot Y reached target within tolerance
    SENSED = auto()    # Foot switch S4 telemetry reports contact


@dataclass
class ContactConfig:
    """Configuration for ContactEstimator.
    
    Attributes:
        position_tolerance_mm: Y tolerance for position-based estimation.
        swing_timeout_ms: Max swing duration before assuming contact.
        placing_timeout_ms: Max placing duration before assuming contact.
        prefer_sensed: If True, sensed contact overrides estimations.
        s4_available: Whether S4 foot switch telemetry is enabled.
    """
    position_tolerance_mm: float = 5.0
    swing_timeout_ms: float = 600.0
    placing_timeout_ms: float = 400.0
    prefer_sensed: bool = True
    s4_available: bool = False  # Set True when foot switches are wired


@dataclass
class ContactResult:
    """Result of contact estimation for one leg.
    
    Attributes:
        has_contact: True if any method detected contact.
        method: How contact was determined.
        sensed: Raw value from S4 telemetry (if available).
        position_reached: True if foot Y within tolerance of target.
        timed_out: True if swing/placing exceeded timeout.
        confidence: 0.0-1.0 confidence estimate (higher = more certain).
    """
    has_contact: bool = False
    method: ContactMethod = ContactMethod.NONE
    sensed: Optional[bool] = None
    position_reached: bool = False
    timed_out: bool = False
    confidence: float = 0.0

    def __repr__(self) -> str:
        return f"Contact({self.method.name}, conf={self.confidence:.2f})"


class ContactEstimator:
    """Estimates foot contact using sensors and fallback heuristics.
    
    FG7 Implementation: Contact Estimation (Pre-Switch Fallback)
    
    This estimator provides contact detection for the free gait system
    using a priority-based approach:
    
    1. **SENSED** (primary): Uses S4 foot switch telemetry when available.
       Most reliable but requires hardware foot switches.
    
    2. **POSITION** (fallback): Detects when foot Y reaches target within
       tolerance. Works for flat terrain when IK/FK are accurate.
    
    3. **TIMEOUT** (fallback): Assumes contact if swing/placing exceeds
       expected duration. Prevents legs from hanging indefinitely.
    
    The estimator flags the source of each contact determination for
    diagnostics and can be tuned via ContactConfig.
    
    Usage:
        estimator = ContactEstimator()
        
        # Each tick for a leg in PLACING state:
        result = estimator.estimate_contact(
            leg=leg,
            s4_contact=telemetry.legs[leg.index].contact,  # May be None
            current_y=leg.foot_current.y,
            target_y=leg.foot_target.y
        )
        
        if result.has_contact:
            leg.confirm_contact(sensed=(result.method == ContactMethod.SENSED))
    """
    
    def __init__(self, config: Optional[ContactConfig] = None):
        """Initialize contact estimator.
        
        Args:
            config: Configuration parameters (uses defaults if None).
        """
        self.config = config or ContactConfig()
        
        # Diagnostics: contact method counts per leg
        self._method_counts: List[Dict[ContactMethod, int]] = [
            {m: 0 for m in ContactMethod} for _ in range(NUM_LEGS)
        ]
    
    def estimate_contact(self,
                         leg: 'Leg',
                         s4_contact: Optional[bool] = None,
                         current_y: Optional[float] = None,
                         target_y: Optional[float] = None) -> ContactResult:
        """Estimate contact for a leg using available data.
        
        Checks contact sources in priority order and returns the first
        positive detection, or NONE if no contact is detected.
        
        Args:
            leg: Leg instance (provides state, timing, index).
            s4_contact: Foot switch value from S4 telemetry (True/False/None).
            current_y: Current foot Y position from FK or command.
            target_y: Target foot Y position (ground level for this step).
            
        Returns:
            ContactResult with detection status and method.
        """
        result = ContactResult()
        result.sensed = s4_contact
        
        # Only estimate contact for legs in swing-related states
        if leg.state not in (LegState.SWING, LegState.PLACING):
            # STANCE or LIFT_PENDING - assume current contact state
            if leg.state == LegState.STANCE:
                result.has_contact = True
                result.method = ContactMethod.POSITION
                result.confidence = 1.0
            return result
        
        # -------------------------------------------------------------------------
        # Priority 1: SENSED (foot switch S4 telemetry)
        # -------------------------------------------------------------------------
        if self.config.s4_available and s4_contact is not None:
            result.sensed = s4_contact
            if s4_contact:
                result.has_contact = True
                result.method = ContactMethod.SENSED
                result.confidence = 1.0
                self._record_method(leg.index, ContactMethod.SENSED)
                return result
        
        # -------------------------------------------------------------------------
        # Priority 2: POSITION (foot Y within tolerance of target)
        # -------------------------------------------------------------------------
        if current_y is not None and target_y is not None:
            y_error = abs(current_y - target_y)
            result.position_reached = (y_error <= self.config.position_tolerance_mm)
            
            if result.position_reached:
                result.has_contact = True
                result.method = ContactMethod.POSITION
                # Confidence based on how close we are
                result.confidence = max(0.0, 1.0 - (y_error / self.config.position_tolerance_mm))
                self._record_method(leg.index, ContactMethod.POSITION)
                return result
        
        # -------------------------------------------------------------------------
        # Priority 3: TIMEOUT (swing/placing exceeded expected time)
        # -------------------------------------------------------------------------
        timeout_ms = (self.config.placing_timeout_ms if leg.state == LegState.PLACING 
                      else self.config.swing_timeout_ms)
        
        if leg.time_in_state_ms >= timeout_ms:
            result.has_contact = True
            result.method = ContactMethod.TIMEOUT
            result.timed_out = True
            result.confidence = 0.5  # Lower confidence for timeout
            self._record_method(leg.index, ContactMethod.TIMEOUT)
            return result
        
        # No contact detected
        result.method = ContactMethod.NONE
        result.confidence = 0.0
        return result
    
    def estimate_all(self,
                     legs: List['Leg'],
                     s4_contacts: Optional[List[bool]] = None,
                     current_positions: Optional[List[FootTarget]] = None) -> List[ContactResult]:
        """Estimate contact for all legs at once.
        
        Convenience method for batch processing.
        
        Args:
            legs: List of 6 Leg instances.
            s4_contacts: List of 6 foot switch states (or None if unavailable).
            current_positions: List of 6 current foot positions (or None).
            
        Returns:
            List of 6 ContactResult instances.
        """
        results = []
        for i, leg in enumerate(legs):
            s4 = s4_contacts[i] if s4_contacts else None
            curr_y = current_positions[i].y if current_positions and i < len(current_positions) else None
            tgt_y = leg.foot_target.y if leg.foot_target else None
            
            results.append(self.estimate_contact(leg, s4, curr_y, tgt_y))
        
        return results
    
    def update_legs_from_results(self,
                                 legs: List['Leg'],
                                 results: List[ContactResult]) -> List[int]:
        """Update leg contact states from estimation results.
        
        For legs in PLACING state with detected contact, this triggers
        the transition to STANCE.
        
        Args:
            legs: List of Leg instances to update.
            results: Corresponding ContactResult list from estimate_all().
            
        Returns:
            List of leg indices that transitioned to STANCE.
        """
        transitioned = []
        for i, (leg, result) in enumerate(zip(legs, results)):
            if leg.state == LegState.PLACING and result.has_contact:
                sensed = (result.method == ContactMethod.SENSED)
                if leg.confirm_contact(sensed=sensed):
                    transitioned.append(i)
            elif leg.state == LegState.SWING and result.has_contact:
                # Early contact during swing (stepped on obstacle?)
                sensed = (result.method == ContactMethod.SENSED)
                if leg.confirm_contact(sensed=sensed):
                    transitioned.append(i)
        
        return transitioned
    
    def _record_method(self, leg_idx: int, method: ContactMethod) -> None:
        """Record contact method for diagnostics."""
        if 0 <= leg_idx < NUM_LEGS:
            self._method_counts[leg_idx][method] += 1
    
    def get_stats(self, leg_idx: int) -> Dict[str, int]:
        """Get contact method statistics for a leg.
        
        Args:
            leg_idx: Leg index (0-5).
            
        Returns:
            Dict mapping method names to counts.
        """
        if 0 <= leg_idx < NUM_LEGS:
            return {m.name: c for m, c in self._method_counts[leg_idx].items()}
        return {}
    
    def get_all_stats(self) -> Dict[str, Dict[str, int]]:
        """Get contact statistics for all legs.
        
        Returns:
            Dict mapping leg names to method count dicts.
        """
        return {LEG_NAMES[i]: self.get_stats(i) for i in range(NUM_LEGS)}
    
    def reset_stats(self) -> None:
        """Reset all diagnostic counters."""
        for counts in self._method_counts:
            for method in ContactMethod:
                counts[method] = 0


def create_contact_estimator(s4_available: bool = False,
                             position_tolerance_mm: float = 5.0,
                             swing_timeout_ms: float = 600.0,
                             placing_timeout_ms: float = 400.0) -> ContactEstimator:
    """Factory function to create a configured ContactEstimator.
    
    Args:
        s4_available: Whether S4 foot switch telemetry is available.
        position_tolerance_mm: Y tolerance for position-based detection.
        swing_timeout_ms: Timeout for swing phase.
        placing_timeout_ms: Timeout for placing phase.
        
    Returns:
        Configured ContactEstimator instance.
    """
    config = ContactConfig(
        s4_available=s4_available,
        position_tolerance_mm=position_tolerance_mm,
        swing_timeout_ms=swing_timeout_ms,
        placing_timeout_ms=placing_timeout_ms
    )
    return ContactEstimator(config)


# -----------------------------------------------------------------------------
# FG5: Free Gait Coordinator
# -----------------------------------------------------------------------------

class SwingPriority(Enum):
    """Leg selection heuristic for swing permission."""
    LONGEST_WAITING = auto()   # Prioritize legs longest in stance
    ALTERNATING = auto()       # Alternate left/right sides
    DIRECTION_ALIGNED = auto() # Prioritize legs aligned with movement


@dataclass
class CoordinatorConfig:
    """Configuration for FreeGaitCoordinator.
    
    Attributes:
        max_simultaneous_swings: Maximum legs allowed in swing at once (1-3).
        min_stability_margin_mm: Margin required to grant new swing.
        critical_margin_mm: Margin below which triggers emergency hold.
        swing_priority: Heuristic for selecting which leg swings next.
        use_weighted_cog: If True, include leg mass in CoG calculation.
    """
    max_simultaneous_swings: int = 3
    min_stability_margin_mm: float = MIN_STABILITY_MARGIN_MM
    critical_margin_mm: float = CRITICAL_MARGIN_MM
    swing_priority: SwingPriority = SwingPriority.LONGEST_WAITING
    use_weighted_cog: bool = False


class FreeGaitCoordinator:
    """Orchestrates free gait leg coordination based on stability constraints.
    
    The coordinator is responsible for:
    1. Tracking stability state (support polygon, CoG, margin)
    2. Granting swing permission to legs when stable
    3. Emergency holds when stability is threatened
    4. Selecting which leg(s) should swing next based on heuristics
    
    Each tick, the coordinator:
    1. Identifies stance legs and computes support polygon
    2. Estimates CoG and computes stability margin
    3. Checks for emergency conditions (critical margin)
    4. Evaluates pending swing requests
    5. Grants permission to qualifying legs
    
    Usage:
        coordinator = FreeGaitCoordinator(legs)
        # Each control loop tick:
        coordinator.tick(dt_ms, pitch_deg, roll_deg)
        # The coordinator modifies leg states directly
    """
    
    def __init__(self, legs: List[Leg], config: Optional[CoordinatorConfig] = None):
        """Initialize coordinator with leg references.
        
        Args:
            legs: List of 6 Leg instances (shared with gait engine).
            config: Configuration parameters (uses defaults if None).
        """
        if len(legs) != NUM_LEGS:
            raise ValueError(f"Expected {NUM_LEGS} legs, got {len(legs)}")
        
        self.legs = legs
        self.config = config or CoordinatorConfig()
        
        # State
        self._current_polygon: Optional[List[Tuple[float, float]]] = None
        self._current_cog: Tuple[float, float] = (0.0, 0.0)
        self._current_margin: float = 0.0
        self._current_status: StabilityStatus = StabilityStatus.STABLE
        self._swing_count: int = 0
        self._last_swing_side: int = 0  # 0=left, 1=right (for alternating)
        self._emergency_hold: bool = False
        
        # Diagnostics
        self._ticks: int = 0
        self._last_grant_tick: int = 0
    
    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    
    @property
    def polygon(self) -> Optional[List[Tuple[float, float]]]:
        """Current support polygon (read-only)."""
        return self._current_polygon
    
    @property
    def cog(self) -> Tuple[float, float]:
        """Current CoG projection (read-only)."""
        return self._current_cog
    
    @property
    def margin(self) -> float:
        """Current stability margin in mm (read-only)."""
        return self._current_margin
    
    @property
    def status(self) -> StabilityStatus:
        """Current stability status (read-only)."""
        return self._current_status
    
    @property
    def in_emergency(self) -> bool:
        """True if in emergency hold state."""
        return self._emergency_hold
    
    @property
    def swing_count(self) -> int:
        """Number of legs currently in swing phases."""
        return self._swing_count
    
    # -------------------------------------------------------------------------
    # Main Tick
    # -------------------------------------------------------------------------
    
    def tick(self, dt_ms: float, 
             pitch_deg: float = 0.0, 
             roll_deg: float = 0.0) -> None:
        """Execute one coordinator tick.
        
        Updates stability state, handles emergency conditions, and grants
        swing permissions to qualifying legs.
        
        Args:
            dt_ms: Time since last tick in milliseconds.
            pitch_deg: Current body pitch from IMU.
            roll_deg: Current body roll from IMU.
        """
        self._ticks += 1
        
        # Update leg timing
        for leg in self.legs:
            leg.update_timing(dt_ms)
        
        # Count legs in swing
        self._swing_count = sum(1 for leg in self.legs if leg.is_swinging())
        
        # Compute current stability state
        self._update_stability(pitch_deg, roll_deg)
        
        # Check for emergency conditions
        if self._current_status in (StabilityStatus.CRITICAL, StabilityStatus.UNSTABLE):
            self._handle_emergency()
            return
        
        # Clear emergency if we've recovered
        if self._emergency_hold and self._current_status == StabilityStatus.STABLE:
            self._emergency_hold = False
        
        # Process swing requests if not in emergency
        if not self._emergency_hold:
            self._process_swing_requests()
    
    # -------------------------------------------------------------------------
    # Stability Update
    # -------------------------------------------------------------------------
    
    def _update_stability(self, pitch_deg: float, roll_deg: float) -> None:
        """Recompute support polygon, CoG, and stability margin."""
        # Get stance foot positions
        stance_xz = get_stance_feet_xz(self.legs)
        
        # Compute support polygon
        self._current_polygon = compute_support_polygon(stance_xz)
        
        # Estimate CoG with optional leg mass weighting
        if self.config.use_weighted_cog:
            leg_pos = [
                (leg.foot_current.x, leg.foot_current.y, leg.foot_current.z)
                for leg in self.legs
            ]
            self._current_cog = estimate_cog(
                leg_positions=leg_pos,
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                use_weighted=True
            )
        else:
            self._current_cog = estimate_cog(
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                use_weighted=False
            )
        
        # Compute stability margin
        self._current_margin = compute_stability_margin(
            self._current_cog, self._current_polygon
        )
        
        # Classify status
        self._current_status = classify_stability(
            self._current_margin,
            self.config.min_stability_margin_mm,
            self.config.critical_margin_mm
        )
    
    # -------------------------------------------------------------------------
    # Emergency Handling
    # -------------------------------------------------------------------------
    
    def _handle_emergency(self) -> None:
        """Handle critical stability condition.
        
        Forces all non-stance legs back to stance immediately.
        """
        self._emergency_hold = True
        
        for leg in self.legs:
            if leg.is_swinging():
                leg.emergency_plant()
    
    def clear_emergency(self) -> None:
        """Manually clear emergency hold state.
        
        Call this after recovery actions are complete and stability
        has been verified externally.
        """
        self._emergency_hold = False
    
    # -------------------------------------------------------------------------
    # Swing Permission
    # -------------------------------------------------------------------------
    
    def _process_swing_requests(self) -> None:
        """Evaluate and grant swing permissions to qualifying legs.
        
        Legs must:
        1. Be in STANCE state
        2. Have been in stance long enough (implicit via priority)
        3. Pass the stability check after hypothetical removal
        """
        # Check if we can grant more swings
        available_slots = self.config.max_simultaneous_swings - self._swing_count
        if available_slots <= 0:
            return
        
        # Get candidates in priority order
        candidates = self._get_swing_candidates()
        
        # Grant permissions up to available slots
        granted = 0
        for leg_idx in candidates:
            if granted >= available_slots:
                break
            
            # Check if this swing would maintain stability
            safe, margin = check_swing_safe(
                self.legs, 
                leg_idx, 
                self._current_cog,
                self.config.min_stability_margin_mm
            )
            
            if safe:
                self.legs[leg_idx].request_swing()
                granted += 1
                self._last_grant_tick = self._ticks
                # Update alternating side tracker
                self._last_swing_side = 0 if leg_idx < 3 else 1
    
    def _get_swing_candidates(self) -> List[int]:
        """Get leg indices eligible for swing, sorted by priority.
        
        Returns:
            List of leg indices in priority order.
        """
        # Find all legs in stance
        stance_legs = [
            (i, leg) for i, leg in enumerate(self.legs)
            if leg.is_stance()
        ]
        
        if not stance_legs:
            return []
        
        # Sort by selected priority heuristic
        if self.config.swing_priority == SwingPriority.LONGEST_WAITING:
            # Sort by time in stance (descending)
            stance_legs.sort(key=lambda x: x[1].time_in_stance_ms(), reverse=True)
        
        elif self.config.swing_priority == SwingPriority.ALTERNATING:
            # Prefer opposite side from last swing
            prefer_left = self._last_swing_side == 1
            stance_legs.sort(key=lambda x: (
                0 if (x[0] < 3) == prefer_left else 1,  # Side preference
                -x[1].time_in_stance_ms()  # Then by wait time
            ))
        
        elif self.config.swing_priority == SwingPriority.DIRECTION_ALIGNED:
            # Default to longest waiting (direction alignment needs heading)
            stance_legs.sort(key=lambda x: x[1].time_in_stance_ms(), reverse=True)
        
        return [i for i, _ in stance_legs]
    
    def request_leg_swing(self, leg_idx: int) -> bool:
        """Explicitly request swing permission for a specific leg.
        
        Use this for external control (e.g., foot placement planner).
        
        Args:
            leg_idx: Index of leg to swing.
            
        Returns:
            True if permission granted, False if denied.
        """
        if not (0 <= leg_idx < NUM_LEGS):
            return False
        
        leg = self.legs[leg_idx]
        if not leg.is_stance():
            return False
        
        if self._emergency_hold:
            return False
        
        if self._swing_count >= self.config.max_simultaneous_swings:
            return False
        
        # Check stability
        safe, _ = check_swing_safe(
            self.legs,
            leg_idx,
            self._current_cog,
            self.config.min_stability_margin_mm
        )
        
        if safe:
            leg.request_swing()
            self._last_grant_tick = self._ticks
            return True
        
        return False
    
    # -------------------------------------------------------------------------
    # Status / Debug
    # -------------------------------------------------------------------------
    
    def status_string(self) -> str:
        """Short status for logging/display."""
        legs_str = " ".join(leg.status_string() for leg in self.legs)
        return (f"[{self._current_status.name[:4]}|M:{self._current_margin:.0f}|"
                f"S:{self._swing_count}] {legs_str}")
    
    def __repr__(self) -> str:
        return (f"FreeGaitCoordinator(margin={self._current_margin:.1f}mm, "
                f"status={self._current_status.name}, swings={self._swing_count})")


# -----------------------------------------------------------------------------
# FG6: Foot Placement Planner
# -----------------------------------------------------------------------------
#
# The "offset model" shifts middle leg neutral positions fore/aft to increase
# stability margin during tripod gait. See:
#   Mostafa, Her & Wu (2012) "The Offset Model of a Hexapod Robot and the
#   Effect of the Offset Parameter" - stability margin improved ~80% with
#   offset D applied to middle legs (LM forward, RM backward or vice versa).
#
# This is implemented via the `middle_leg_offset_z_mm` config parameter.
# -----------------------------------------------------------------------------

# Neutral foot positions in body frame (mm) for each leg
# X: lateral offset (positive = right), Y: vertical (negative = down), Z: fore-aft
# These match the StandingGait default configuration
NEUTRAL_FOOT_POSITIONS: List[Tuple[float, float, float]] = [
    (120.0, -60.0, 100.0),    # LF: left-front
    (140.0, -60.0, 0.0),      # LM: left-middle
    (120.0, -60.0, -100.0),   # LR: left-rear
    (-120.0, -60.0, 100.0),   # RF: right-front
    (-140.0, -60.0, 0.0),     # RM: right-middle
    (-120.0, -60.0, -100.0),  # RR: right-rear
]

# Hip mount positions (XZ) for computing turn-adjusted strides
# X positive = right side of body, X negative = left side
# Z positive = front, Z negative = rear
HIP_POSITIONS_XZ: List[Tuple[float, float]] = [
    (-60.0, 80.0),   # LF: left-front
    (-75.0, 0.0),    # LM: left-middle
    (-60.0, -80.0),  # LR: left-rear
    (60.0, 80.0),    # RF: right-front
    (75.0, 0.0),     # RM: right-middle
    (60.0, -80.0),   # RR: right-rear
]


@dataclass
class PlacementConfig:
    """Configuration for foot placement planning.
    
    Attributes:
        base_x_mm: Lateral stance width (from body center).
        base_y_mm: Foot height (negative = below body).
        step_len_mm: Maximum stride length (half-stroke).
        max_step_len_mm: Safety clamp for step length.
        lift_height_mm: Foot lift height during swing.
        lift_attenuation: Scale factor for lift height to match Bezier-based gaits.
            TripodGait uses 5-point Bezier with ~0.69 attenuation (peak at 41mm for 60mm config).
            Set to 0.69 by default to match TripodGait effective lift.
        turn_gain: How much turn rate affects per-leg stride.
        middle_leg_offset_z_mm: Fore-aft shift for middle legs (LM/RM).
            Positive values shift LM forward and RM backward, enlarging the
            support polygon during tripod gait. Based on Mostafa et al. (2012)
            "offset model" which showed ~80% improvement in stability margin.
            Typical range: 0-30mm. Default 0 (no offset).
    """
    base_x_mm: float = 120.0
    base_y_mm: float = -120.0  # Default matches typical robot height
    step_len_mm: float = 20.0
    max_step_len_mm: float = 40.0
    lift_height_mm: float = 50.0
    lift_attenuation: float = 0.69  # Match TripodGait Bezier attenuation
    turn_gain: float = 0.5
    middle_leg_offset_z_mm: float = 0.0  # Mostafa offset model parameter


class FootPlacementPlanner:
    """Plans target foot positions for free gait leg swings.
    
    Computes where a leg should place its foot based on:
    1. Neutral standing position (default)
    2. Heading direction (stride offset)
    3. Speed magnitude (stride length scaling)
    4. Turn rate (differential strides)
    
    Optionally pre-checks candidate positions against collision model
    before committing.
    
    Usage:
        planner = FootPlacementPlanner()
        target = planner.plan_foot_placement(leg_idx, heading_deg, speed)
    """
    
    def __init__(self, config: Optional[PlacementConfig] = None,
                 collision_checker: Optional[callable] = None):
        """Initialize foot placement planner.
        
        Args:
            config: Placement parameters (uses defaults if None).
            collision_checker: Optional callable(leg_idx, x, y, z) -> bool
                              Returns True if position is safe.
        """
        self.config = config or PlacementConfig()
        self._collision_checker = collision_checker
        
        # Initialize neutral positions using TripodGait coordinate convention:
        # - Left-side legs (LF, LM, LR) have negative X; right-side have positive X
        # - Z is stride offset (0 at neutral)
        import math
        
        # Leg base rotation angles (matching gait_engine.py LEG_BASE_ROTATION_DEG)
        LEG_ROTATIONS_DEG = [45.0, 0.0, -45.0, 45.0, 0.0, -45.0]
        
        # Apply "offset model" shift to middle legs (Mostafa et al. 2012)
        offset_z = self.config.middle_leg_offset_z_mm
        
        self._neutral: List[FootTarget] = []
        for i in range(NUM_LEGS):
            angle_rad = math.radians(LEG_ROTATIONS_DEG[i])
            cos_a = math.cos(angle_rad)
            
            # X is positive for ALL legs - firmware handles left-side mirroring
            x = self.config.base_x_mm * cos_a
            
            # Z = 0 at neutral (stride offset applied during movement)
            z = 0.0
            
            # Apply middle leg offset model
            if i == 1:    # LM: shift forward (+Z)
                z += offset_z
            elif i == 4:  # RM: shift backward (-Z)
                z -= offset_z
            
            self._neutral.append(FootTarget(x=x, y=self.config.base_y_mm, z=z))
    
    def get_neutral_position(self, leg_idx: int) -> FootTarget:
        """Get neutral standing position for a leg.
        
        Args:
            leg_idx: Leg index (0-5).
            
        Returns:
            FootTarget at neutral standing position.
        """
        if 0 <= leg_idx < NUM_LEGS:
            return self._neutral[leg_idx].copy()
        return FootTarget()
    
    def plan_foot_placement(self, leg_idx: int,
                            heading_deg: float = 0.0,
                            speed_scale: float = 0.0,
                            turn_rate: float = 0.0) -> FootTarget:
        """Compute target foot position for a swing.
        
        Combines neutral position with directional offset based on
        desired heading and speed. Turn rate adjusts per-leg stride
        for smooth turning.
        
        Uses per-leg rotation to account for hexapod geometry (corner legs
        at ±45°, middle legs at 0°). This matches TripodGait's coordinate
        system where each leg's "forward" is relative to its hip orientation.
        
        Args:
            leg_idx: Leg index (0-5).
            heading_deg: Movement heading in degrees (0=forward, 90=right).
            speed_scale: Speed as fraction of max (0.0-1.0).
            turn_rate: Turn rate (-1.0 to 1.0, positive=CW).
            
        Returns:
            FootTarget with computed position.
        """
        import math
        
        if not (0 <= leg_idx < NUM_LEGS):
            return FootTarget()
        
        # Start with neutral position
        neutral = self._neutral[leg_idx]
        target = neutral.copy()
        
        # If not moving, return neutral
        if abs(speed_scale) < 0.01 and abs(turn_rate) < 0.01:
            return target
        
        # Leg base rotation angles (matching gait_engine.py LEG_BASE_ROTATION_DEG)
        LEG_ROTATIONS_DEG = [45.0, 0.0, -45.0, 45.0, 0.0, -45.0]
        
        # Compute stride magnitude (body-frame forward/back distance)
        stride_magnitude = self.config.step_len_mm * abs(speed_scale)
        stride_magnitude = min(stride_magnitude, self.config.max_step_len_mm)
        
        # Adjust for turn - legs on outside of turn take longer strides
        hip_x, hip_z = HIP_POSITIONS_XZ[leg_idx]
        turn_adjustment = 1.0 + (hip_x / 100.0) * turn_rate * self.config.turn_gain
        turn_adjustment = max(0.3, min(1.7, turn_adjustment))
        
        # Apply turn adjustment to stride
        adjusted_stride = stride_magnitude * turn_adjustment
        
        # Apply per-leg rotation (matching TripodGait's _apply_leg_rotation exactly)
        # This rotates the stride vector by (leg_angle + sign * walk_dir * 90)
        # where walk_dir = heading / 90 (normalized -1 to +1)
        
        # Walk direction sign: left legs add heading offset, right legs subtract
        walk_sign = 1.0 if leg_idx in (0, 1, 2) else -1.0
        walk_dir = heading_deg / 90.0  # Normalize heading to -1..+1
        full_angle_rad = math.radians(LEG_ROTATIONS_DEG[leg_idx]) + walk_sign * math.radians(walk_dir * 90.0)
        
        cos_angle = math.cos(full_angle_rad)
        sin_angle = math.sin(full_angle_rad)
        
        # Rotate stride by full_angle (matching TripodGait)
        # X is positive for ALL legs - firmware handles left-side mirroring
        stride_x_prime = adjusted_stride * sin_angle
        stride_z_prime = adjusted_stride * cos_angle
        
        # Apply stride offset to neutral position
        # X: neutral.x + rotated stride X component
        # Z: just the rotated stride Z component (matching TripodGait's l_z_prime = stride_z_prime)
        target.x = neutral.x + stride_x_prime
        target.z = stride_z_prime  # NOT neutral.z + stride_z_prime!
        
        # Y stays at neutral (ground level)
        target.y = neutral.y
        
        # Collision check if available
        if self._collision_checker is not None:
            if not self._collision_checker(leg_idx, target.x, target.y, target.z):
                # Fallback to neutral if collision detected
                return neutral.copy()
        
        return target
    
    def plan_swing_trajectory(self, leg_idx: int,
                              start: FootTarget,
                              target: FootTarget,
                              progress: float) -> FootTarget:
        """Compute intermediate position during swing trajectory.
        
        Uses a 5-point Bezier curve (matching TripodGait) for smooth continuous
        interpolation of X, Y, Z throughout the swing. This ensures the foot
        follows a natural arc rather than the previous lift-translate-place
        3-phase approach which caused XZ to only move during the middle phase.
        
        Args:
            leg_idx: Leg index (0-5).
            start: Starting foot position.
            target: Target foot position.
            progress: Swing progress (0.0-1.0).
            
        Returns:
            FootTarget at interpolated position.
        """
        import math
        
        progress = max(0.0, min(1.0, progress))
        result = FootTarget()
        
        # Lift height (matching TripodGait's effective lift after Bezier attenuation)
        lift_height = self.config.lift_height_mm * self.config.lift_attenuation
        
        # Base Y is the ground level (start and target should have same Y)
        base_y = start.y
        
        # Bezier control point parameters (matching TripodGait)
        p1_height = 0.15      # P1 height as fraction of lift
        p1_overshoot = 1.1    # P1 overshoot (slightly past start)
        p2_height = 1.5       # P2 peak height (150% to compensate for curve attenuation)
        p3_height = 0.35      # P3 height as fraction of lift
        p3_overshoot = 1.1    # P3 overshoot (slightly past target)
        
        # Compute stride vector (from start to target)
        dx = target.x - start.x
        dz = target.z - start.z
        
        # 5-point Bezier control points for swing arc
        # X and Z interpolate from start to target with slight overshoots
        # Y follows a smooth arc: ground -> lift -> ground
        swing_points = [
            # P0: start position
            (start.x,
             base_y,
             start.z),
            # P1: slight lift, overshoot backward (past start)
            (start.x - p1_overshoot * 0.1 * dx,
             base_y + p1_height * lift_height,
             start.z - p1_overshoot * 0.1 * dz),
            # P2: peak height, approximately center
            (start.x + 0.5 * dx,
             base_y + p2_height * lift_height,
             start.z + 0.5 * dz),
            # P3: descending, overshoot forward (past target)
            (target.x + p3_overshoot * 0.1 * dx,
             base_y + p3_height * lift_height,
             target.z + p3_overshoot * 0.1 * dz),
            # P4: end position (target)
            (target.x,
             base_y,
             target.z),
        ]
        
        # Evaluate Bezier curve at progress
        # Use the same Bezier evaluation as TripodGait
        n = len(swing_points) - 1
        x, y, z = 0.0, 0.0, 0.0
        
        for i in range(len(swing_points)):
            # Bernstein basis polynomial: C(n,i) * (1-t)^(n-i) * t^i
            # Binomial coefficient calculation
            coeff = 1
            for j in range(1, i + 1):
                coeff = coeff * (n - (i - j)) // j
            b = coeff * ((1.0 - progress) ** (n - i)) * (progress ** i)
            
            x += b * swing_points[i][0]
            y += b * swing_points[i][1]
            z += b * swing_points[i][2]
        
        result.x = x
        result.y = y
        result.z = z
        
        return result
    
    def set_collision_checker(self, checker: callable) -> None:
        """Set or update the collision checking function.
        
        Args:
            checker: Callable(leg_idx, x, y, z) -> bool (True = safe).
        """
        self._collision_checker = checker
    
    def __repr__(self) -> str:
        return f"FootPlacementPlanner(step={self.config.step_len_mm}mm, lift={self.config.lift_height_mm}mm)"


# -----------------------------------------------------------------------------
# Test / Demo
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== Free Gait Leg State Machine Test ===\n")
    
    # Create legs
    legs = create_legs()
    
    # Show initial state
    print("Initial state (all stance):")
    for leg in legs:
        print(f"  {leg}")
    
    # Simulate swing cycle for LF
    print("\n--- LF swing cycle ---")
    lf = legs[LEG_LF]
    
    print(f"1. {lf}")
    
    lf.request_swing()
    print(f"2. After request_swing: {lf}")
    
    lf.complete_lift()
    print(f"3. After complete_lift: {lf}")
    
    lf.begin_placing()
    print(f"4. After begin_placing: {lf}")
    
    lf.confirm_contact(sensed=True)
    print(f"5. After confirm_contact: {lf}")
    
    # Status string test
    print("\nStatus strings:")
    print("  " + " | ".join(leg.status_string() for leg in legs))
    
    print("\n=== FG1 Test Complete ===")

    # -------------------------------------------------------------------------
    # FG2: Support Polygon Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG2: Support Polygon Tests ===\n")
    
    # Reset legs to stance
    legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    
    # Set realistic hexapod foot positions (body frame XZ)
    # Front legs: z > 0, rear legs: z < 0, middle legs: z ≈ 0
    foot_positions = [
        (100.0, 120.0),   # LF: left-front
        (120.0, 0.0),     # LM: left-middle  
        (100.0, -120.0),  # LR: left-rear
        (-100.0, 120.0),  # RF: right-front
        (-120.0, 0.0),    # RM: right-middle
        (-100.0, -120.0), # RR: right-rear
    ]
    
    for i, (x, z) in enumerate(foot_positions):
        legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    
    # Get stance feet (all 6)
    stance_xz = get_stance_feet_xz(legs)
    print(f"Stance feet (6): {len(stance_xz)} points")
    
    # Compute support polygon
    polygon = compute_support_polygon(stance_xz)
    print(f"Support polygon: {len(polygon)} vertices")
    for i, (x, z) in enumerate(polygon):
        print(f"  V{i}: ({x:.1f}, {z:.1f})")
    
    # Test CoG at center
    cog = (0.0, 0.0)
    inside = point_in_polygon(cog, polygon)
    margin = distance_to_polygon_edge(cog, polygon)
    print(f"\nCoG at (0,0): inside={inside}, margin={margin:.1f}mm")
    assert inside, "CoG should be inside polygon"
    assert margin > 0, "Margin should be positive when inside"
    
    # Test CoG near edge
    cog_edge = (100.0, 0.0)
    inside_edge = point_in_polygon(cog_edge, polygon)
    margin_edge = distance_to_polygon_edge(cog_edge, polygon)
    print(f"CoG at (100,0): inside={inside_edge}, margin={margin_edge:.1f}mm")
    
    # Test CoG outside
    cog_out = (200.0, 0.0)
    inside_out = point_in_polygon(cog_out, polygon)
    margin_out = distance_to_polygon_edge(cog_out, polygon)
    print(f"CoG at (200,0): inside={inside_out}, margin={margin_out:.1f}mm")
    assert not inside_out, "CoG should be outside polygon"
    assert margin_out < 0, "Margin should be negative when outside"
    
    # Test with only 3 stance feet (tripod)
    print("\n--- Tripod support (3 feet) ---")
    legs[LEG_LF].request_swing()
    legs[LEG_RM].request_swing()
    legs[LEG_LR].request_swing()
    
    stance_xz_tripod = get_stance_feet_xz(legs)
    print(f"Stance feet: {len(stance_xz_tripod)} points")
    
    polygon_tripod = compute_support_polygon(stance_xz_tripod)
    if polygon_tripod:
        print(f"Tripod polygon: {len(polygon_tripod)} vertices")
        cog_tripod = (0.0, 0.0)
        margin_tripod = distance_to_polygon_edge(cog_tripod, polygon_tripod)
        print(f"CoG margin in tripod: {margin_tripod:.1f}mm")
    else:
        print("Tripod polygon: None (degenerate)")
    
    # Test with 2 feet (should return None)
    print("\n--- Degenerate (2 feet) ---")
    legs[LEG_LM].request_swing()
    stance_xz_2 = get_stance_feet_xz(legs)
    polygon_2 = compute_support_polygon(stance_xz_2)
    print(f"2-foot polygon: {polygon_2}")
    assert polygon_2 is None, "Should return None for < 3 feet"
    
    print("\n=== FG2 Test Complete ===")

    # -------------------------------------------------------------------------
    # FG3: Center of Gravity Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG3: Center of Gravity Estimation Tests ===\n")
    
    # Test simple CoG (always returns origin)
    cog_simple = estimate_cog_simple()
    print(f"Simple CoG: {cog_simple}")
    assert cog_simple == (0.0, 0.0, 0.0), "Simple CoG should be origin"
    
    # Test weighted CoG with symmetric leg positions
    leg_pos_symmetric = [
        (100.0, -60.0, 100.0),   # LF
        (120.0, -60.0, 0.0),     # LM
        (100.0, -60.0, -100.0),  # LR
        (-100.0, -60.0, 100.0),  # RF
        (-120.0, -60.0, 0.0),    # RM
        (-100.0, -60.0, -100.0), # RR
    ]
    cog_weighted_sym = estimate_cog_weighted(leg_pos_symmetric)
    print(f"Weighted CoG (symmetric): ({cog_weighted_sym[0]:.2f}, {cog_weighted_sym[1]:.2f}, {cog_weighted_sym[2]:.2f})")
    # With symmetric legs, X and Z should average to 0, Y shifts due to legs
    assert abs(cog_weighted_sym[0]) < 1.0, "Symmetric legs should have ~0 X offset"
    assert abs(cog_weighted_sym[2]) < 1.0, "Symmetric legs should have ~0 Z offset"
    
    # Test weighted CoG with asymmetric leg positions (reaching forward)
    leg_pos_forward = [
        (100.0, -60.0, 150.0),   # LF: extended forward
        (120.0, -60.0, 50.0),    # LM: slightly forward
        (100.0, -60.0, -100.0),  # LR: normal
        (-100.0, -60.0, 150.0),  # RF: extended forward
        (-120.0, -60.0, 50.0),   # RM: slightly forward
        (-100.0, -60.0, -100.0), # RR: normal
    ]
    cog_weighted_fwd = estimate_cog_weighted(leg_pos_forward)
    print(f"Weighted CoG (forward reach): ({cog_weighted_fwd[0]:.2f}, {cog_weighted_fwd[1]:.2f}, {cog_weighted_fwd[2]:.2f})")
    assert cog_weighted_fwd[2] > 0, "Forward-reaching legs should shift CoG forward (+Z)"
    
    # Test ground projection without tilt
    cog_3d = (0.0, 0.0, 0.0)
    proj_flat = project_cog_to_ground(cog_3d, pitch_deg=0.0, roll_deg=0.0)
    print(f"\nProjection (no tilt): {proj_flat}")
    assert proj_flat == (0.0, 0.0), "No tilt should project to XZ origin"
    
    # Test ground projection with pitch (nose down)
    proj_pitch = project_cog_to_ground(cog_3d, pitch_deg=10.0, roll_deg=0.0)
    print(f"Projection (10° pitch down): ({proj_pitch[0]:.2f}, {proj_pitch[1]:.2f})")
    assert proj_pitch[1] > 0, "Nose-down pitch should shift projection forward (+Z)"
    
    # Test ground projection with roll (right side down)
    proj_roll = project_cog_to_ground(cog_3d, pitch_deg=0.0, roll_deg=10.0)
    print(f"Projection (10° roll right): ({proj_roll[0]:.2f}, {proj_roll[1]:.2f})")
    assert proj_roll[0] > 0, "Right-down roll should shift projection right (+X)"
    
    # Test combined projection
    proj_combo = project_cog_to_ground(cog_3d, pitch_deg=5.0, roll_deg=-5.0)
    print(f"Projection (5° pitch, -5° roll): ({proj_combo[0]:.2f}, {proj_combo[1]:.2f})")
    
    # Test main estimate_cog function (simple mode)
    cog_proj_simple = estimate_cog(pitch_deg=0.0, roll_deg=0.0, use_weighted=False)
    print(f"\nestimate_cog (simple, flat): {cog_proj_simple}")
    assert cog_proj_simple == (0.0, 0.0), "Simple mode flat should be origin"
    
    # Test main estimate_cog function (weighted mode)
    cog_proj_weighted = estimate_cog(
        leg_positions=leg_pos_forward,
        pitch_deg=5.0,
        roll_deg=0.0,
        use_weighted=True
    )
    print(f"estimate_cog (weighted, 5° pitch): ({cog_proj_weighted[0]:.2f}, {cog_proj_weighted[1]:.2f})")
    
    # Integration test: use CoG with support polygon
    print("\n--- Integration: CoG + Support Polygon ---")
    # Reset all legs to stance
    legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    for i, (x, z) in enumerate(foot_positions):
        legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    
    stance_xz = get_stance_feet_xz(legs)
    polygon = compute_support_polygon(stance_xz)
    
    # Get CoG with tilt and check stability margin
    cog_xz = estimate_cog(pitch_deg=3.0, roll_deg=-2.0, use_weighted=False)
    margin = distance_to_polygon_edge(cog_xz, polygon)
    print(f"CoG projection: ({cog_xz[0]:.2f}, {cog_xz[1]:.2f})")
    print(f"Stability margin: {margin:.1f}mm")
    assert margin > 0, "CoG with small tilt should still be stable"
    
    print("\n=== FG3 Test Complete ===")

    # -------------------------------------------------------------------------
    # FG4: Stability Margin Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG4: Stability Margin Computation Tests ===\n")
    
    # Reset legs to all stance with hexapod layout
    legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    foot_positions = [
        (100.0, 120.0),   # LF
        (120.0, 0.0),     # LM  
        (100.0, -120.0),  # LR
        (-100.0, 120.0),  # RF
        (-120.0, 0.0),    # RM
        (-100.0, -120.0), # RR
    ]
    for i, (x, z) in enumerate(foot_positions):
        legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    
    # Test compute_stability_margin with all feet in stance
    stance_xz = get_stance_feet_xz(legs)
    polygon = compute_support_polygon(stance_xz)
    cog = (0.0, 0.0)
    
    margin = compute_stability_margin(cog, polygon)
    print(f"6-foot margin at center: {margin:.1f}mm")
    assert margin > 0, "Should be stable with all feet"
    
    # Test classify_stability
    status = classify_stability(margin)
    print(f"Status (margin={margin:.0f}mm): {status.name}")
    assert status == StabilityStatus.STABLE, "High margin should be STABLE"
    
    # Test thresholds
    print("\n--- Stability classification thresholds ---")
    for test_margin in [50.0, 25.0, 8.0, -5.0]:
        test_status = classify_stability(test_margin)
        color = get_stability_color(test_status)
        print(f"  {test_margin:+6.1f}mm → {test_status.name:10s} RGB{color}")
    
    # Test check_swing_safe - can we lift LF?
    print("\n--- Check swing safety ---")
    cog_xz = estimate_cog()
    safe, new_margin = check_swing_safe(legs, LEG_LF, cog_xz)
    print(f"Lift LF: safe={safe}, new_margin={new_margin:.1f}mm")
    
    # With 5 feet, should still have good margin
    assert safe, "Lifting one leg from 6-foot stance should be safe"
    
    # Test with tripod - can we lift another leg?
    print("\n--- Tripod swing check ---")
    # Put LF, RM, LR into swing (tripod A swinging)
    legs[LEG_LF].request_swing()
    legs[LEG_RM].request_swing()
    legs[LEG_LR].request_swing()
    
    # Can we lift LM from tripod B stance?
    safe_tripod, margin_tripod = check_swing_safe(legs, LEG_LM, cog_xz)
    print(f"Lift LM from tripod: safe={safe_tripod}, margin={margin_tripod:.1f}mm")
    # With only RF and RR remaining, polygon degenerates - should not be safe
    
    # Test None polygon case
    print("\n--- Degenerate cases ---")
    margin_none = compute_stability_margin((0.0, 0.0), None)
    print(f"None polygon margin: {margin_none}")
    assert margin_none == float('-inf'), "None polygon should return -inf"
    
    margin_small = compute_stability_margin((0.0, 0.0), [(0.0, 0.0), (1.0, 0.0)])
    print(f"2-point polygon margin: {margin_small}")
    assert margin_small == float('-inf'), "< 3 points should return -inf"
    
    print("\n=== FG4 Test Complete ===")

    # -------------------------------------------------------------------------
    # FG5: Free Gait Coordinator Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG5: Free Gait Coordinator Tests ===\n")
    
    # Create fresh legs with realistic positions
    legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    foot_positions = [
        (100.0, 120.0),   # LF
        (120.0, 0.0),     # LM  
        (100.0, -120.0),  # LR
        (-100.0, 120.0),  # RF
        (-120.0, 0.0),    # RM
        (-100.0, -120.0), # RR
    ]
    for i, (x, z) in enumerate(foot_positions):
        legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    
    # Create coordinator with default config
    config = CoordinatorConfig(max_simultaneous_swings=3)
    coordinator = FreeGaitCoordinator(legs, config)
    print(f"Coordinator created: {coordinator}")
    
    # Test initial state
    print("\n--- Initial state ---")
    print(f"Status: {coordinator.status_string()}")
    assert coordinator.status == StabilityStatus.STABLE, "Should start stable"
    assert coordinator.swing_count == 0, "No swings initially"
    
    # Tick to update state
    coordinator.tick(dt_ms=6.0, pitch_deg=0.0, roll_deg=0.0)
    print(f"After tick: margin={coordinator.margin:.1f}mm")
    assert coordinator.margin > 0, "Margin should be positive"
    
    # Test swing grant - coordinator should grant based on priority
    print("\n--- Swing permission grants ---")
    for tick_num in range(3):
        coordinator.tick(dt_ms=6.0)
        print(f"Tick {tick_num + 1}: {coordinator.status_string()}")
    
    # Should have granted up to max_simultaneous_swings
    print(f"Swing count: {coordinator.swing_count}")
    assert coordinator.swing_count <= config.max_simultaneous_swings
    
    # Test explicit swing request (create fresh legs to avoid auto-grant)
    print("\n--- Explicit swing request ---")
    fresh_legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    for i, (x, z) in enumerate(foot_positions):
        fresh_legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    fresh_coord = FreeGaitCoordinator(fresh_legs, CoordinatorConfig(max_simultaneous_swings=1))
    # Don't call tick() to avoid auto-grant
    fresh_coord._update_stability(0.0, 0.0)  # Just update stability state
    print(f"Fresh coordinator: {fresh_coord.status_string()}")
    
    # Request specific leg - should succeed since no swings yet
    success = fresh_coord.request_leg_swing(LEG_LF)
    print(f"Request LF swing: {success}")
    assert success, "Should be able to swing LF"
    assert fresh_legs[LEG_LF].state == LegState.LIFT_PENDING
    
    # Test emergency hold
    print("\n--- Emergency hold ---")
    # Simulate critical margin by putting most legs in swing
    for i in [LEG_LM, LEG_LR, LEG_RF, LEG_RM]:
        legs[i].request_swing()
    coordinator.tick(dt_ms=6.0)
    print(f"After many swings: {coordinator.status_string()}")
    print(f"Emergency: {coordinator.in_emergency}")
    
    # If margin went critical, legs should be emergency planted
    if coordinator.in_emergency:
        print("Emergency triggered - all legs planted")
        assert coordinator.swing_count == 0 or all(
            leg.is_stance() for leg in legs
        ), "Emergency should plant all legs"
    
    # Test priority heuristics
    print("\n--- Priority heuristics ---")
    # Create fresh legs for clean priority test
    priority_legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    for i, (x, z) in enumerate(foot_positions):
        priority_legs[i].foot_current = FootTarget(x=x, y=-60.0, z=z)
    
    priority_coord = FreeGaitCoordinator(priority_legs, CoordinatorConfig(max_simultaneous_swings=3))
    priority_coord._update_stability(0.0, 0.0)
    
    # Age some legs in stance
    priority_legs[LEG_LR].stance_start_time -= 1.0  # 1 second older
    priority_legs[LEG_RM].stance_start_time -= 0.5  # 0.5 seconds older
    
    candidates = priority_coord._get_swing_candidates()
    print(f"Priority order (LONGEST_WAITING): {[LEG_NAMES[i] for i in candidates[:3]]}")
    # LR should be first (oldest)
    assert candidates[0] == LEG_LR, "LR should have highest priority (longest waiting)"
    
    # Test alternating priority
    priority_coord.config.swing_priority = SwingPriority.ALTERNATING
    priority_coord._last_swing_side = 1  # Last was right side
    candidates_alt = priority_coord._get_swing_candidates()
    print(f"Priority order (ALTERNATING, last=R): {[LEG_NAMES[i] for i in candidates_alt[:3]]}")
    # Should prefer left side (0,1,2)
    assert candidates_alt[0] < 3, "Should prefer left side after right"
    
    print("\n=== FG5 Test Complete ===")

    # -------------------------------------------------------------------------
    # FG6: Foot Placement Planner Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG6: Foot Placement Planner Tests ===\n")
    
    # Create planner with default config
    planner = FootPlacementPlanner()
    print(f"Planner created: {planner}")
    
    # Test neutral positions
    print("\n--- Neutral positions ---")
    for i in range(NUM_LEGS):
        neutral = planner.get_neutral_position(i)
        print(f"  {LEG_NAMES[i]}: ({neutral.x:.1f}, {neutral.y:.1f}, {neutral.z:.1f})")
    
    # Test forward walking placement
    print("\n--- Forward walking (heading=0, speed=0.5) ---")
    for i in range(NUM_LEGS):
        target = planner.plan_foot_placement(i, heading_deg=0.0, speed_scale=0.5)
        neutral = planner.get_neutral_position(i)
        offset_z = target.z - neutral.z
        print(f"  {LEG_NAMES[i]}: z_offset={offset_z:+.1f}mm")
    
    # All legs should have positive Z offset (forward placement)
    lf_target = planner.plan_foot_placement(LEG_LF, heading_deg=0.0, speed_scale=0.5)
    lf_neutral = planner.get_neutral_position(LEG_LF)
    assert lf_target.z > lf_neutral.z, "Forward walking should place feet ahead"
    
    # Test lateral walking (heading=90 = right)
    print("\n--- Lateral walking (heading=90, speed=0.5) ---")
    for i in range(NUM_LEGS):
        target = planner.plan_foot_placement(i, heading_deg=90.0, speed_scale=0.5)
        neutral = planner.get_neutral_position(i)
        offset_x = target.x - neutral.x
        print(f"  {LEG_NAMES[i]}: x_offset={offset_x:+.1f}mm")
    
    # Test turn adjustment
    print("\n--- Turn adjustment (CW turn, turn_rate=0.5) ---")
    for i in range(NUM_LEGS):
        target = planner.plan_foot_placement(i, heading_deg=0.0, speed_scale=0.5, turn_rate=0.5)
        neutral = planner.get_neutral_position(i)
        offset_z = target.z - neutral.z
        side = "L" if i < 3 else "R"
        print(f"  {LEG_NAMES[i]} ({side}): z_offset={offset_z:+.1f}mm")
    
    # Left legs (outside of CW turn) should have longer stride
    lf_turn = planner.plan_foot_placement(LEG_LF, heading_deg=0.0, speed_scale=0.5, turn_rate=0.5)
    rf_turn = planner.plan_foot_placement(LEG_RF, heading_deg=0.0, speed_scale=0.5, turn_rate=0.5)
    lf_stride = lf_turn.z - planner.get_neutral_position(LEG_LF).z
    rf_stride = rf_turn.z - planner.get_neutral_position(LEG_RF).z
    print(f"LF stride: {lf_stride:.1f}mm, RF stride: {rf_stride:.1f}mm")
    assert lf_stride > rf_stride, "Left legs should stride longer in CW turn"
    
    # Test swing trajectory
    print("\n--- Swing trajectory ---")
    start = FootTarget(x=120.0, y=-60.0, z=100.0)
    target = FootTarget(x=120.0, y=-60.0, z=120.0)
    
    for progress in [0.0, 0.15, 0.3, 0.5, 0.7, 0.85, 1.0]:
        pos = planner.plan_swing_trajectory(LEG_LF, start, target, progress)
        print(f"  p={progress:.2f}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
    
    # At progress 0.5 (mid-swing), should be lifted and halfway translated
    mid = planner.plan_swing_trajectory(LEG_LF, start, target, 0.5)
    assert mid.y > start.y + 40, "Mid-swing should be lifted"
    assert 100 < mid.z < 120, "Mid-swing should be mid-translation"
    
    # Test collision checker integration
    print("\n--- Collision checker ---")
    def mock_collision_check(leg_idx, x, y, z):
        # Reject positions with z > 110
        return z <= 110
    
    planner.set_collision_checker(mock_collision_check)
    
    # This should return neutral due to collision rejection
    rejected = planner.plan_foot_placement(LEG_LF, heading_deg=0.0, speed_scale=1.0)
    neutral_lf = planner.get_neutral_position(LEG_LF)
    print(f"With collision rejection: z={rejected.z:.1f} (neutral={neutral_lf.z:.1f})")
    # High speed would place beyond z=110, so should fall back to neutral
    
    # Test offset model (Mostafa et al. 2012)
    print("\n--- Offset model (middle_leg_offset_z_mm=25) ---")
    offset_config = PlacementConfig(middle_leg_offset_z_mm=25.0)
    offset_planner = FootPlacementPlanner(config=offset_config)
    
    # LM (idx 1) should be shifted forward (+Z), RM (idx 4) backward (-Z)
    lm_neutral = offset_planner.get_neutral_position(1)  # LM
    rm_neutral = offset_planner.get_neutral_position(4)  # RM
    baseline_lm_z = 0.0   # from NEUTRAL_FOOT_POSITIONS
    baseline_rm_z = 0.0
    
    print(f"  LM neutral Z: {lm_neutral.z:.1f}mm (expected: +25 from baseline)")
    print(f"  RM neutral Z: {rm_neutral.z:.1f}mm (expected: -25 from baseline)")
    
    assert abs(lm_neutral.z - (baseline_lm_z + 25.0)) < 0.01, "LM should shift forward by offset"
    assert abs(rm_neutral.z - (baseline_rm_z - 25.0)) < 0.01, "RM should shift backward by offset"
    
    # Front/rear legs should be unaffected
    lf_offset = offset_planner.get_neutral_position(0)  # LF
    lr_offset = offset_planner.get_neutral_position(2)  # LR
    print(f"  LF neutral Z: {lf_offset.z:.1f}mm (unchanged at 100)")
    print(f"  LR neutral Z: {lr_offset.z:.1f}mm (unchanged at -100)")
    assert abs(lf_offset.z - 100.0) < 0.01, "LF should be unaffected by offset"
    assert abs(lr_offset.z - (-100.0)) < 0.01, "LR should be unaffected by offset"
    
    print("Offset model: PASS (middle legs shifted, front/rear unchanged)")
    
    print("\n=== FG6 Test Complete ===")
    # -------------------------------------------------------------------------
    # FG7: Contact Estimation Tests
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("=== FG7: Contact Estimation Tests ===\n")
    
    # Create test legs
    test_legs = create_legs(base_x_mm=100.0, base_y_mm=-60.0)
    
    # Test 1: Default estimator (no S4)
    print("--- Estimator without S4 telemetry ---")
    estimator = ContactEstimator()
    print(f"Estimator config: s4_available={estimator.config.s4_available}")
    assert not estimator.config.s4_available, "Default should not have S4"
    
    # Test 2: Leg in STANCE should report contact
    print("\n--- STANCE leg contact ---")
    stance_leg = test_legs[0]
    stance_leg.state = LegState.STANCE
    result = estimator.estimate_contact(stance_leg)
    print(f"STANCE leg: {result}")
    assert result.has_contact, "STANCE leg should have contact"
    assert result.confidence == 1.0, "STANCE should have full confidence"
    
    # Test 3: Leg in PLACING with position reached
    print("\n--- PLACING with position reached ---")
    placing_leg = test_legs[1]
    placing_leg._transition_to(LegState.PLACING)
    placing_leg.foot_target.y = -60.0
    result = estimator.estimate_contact(
        placing_leg, 
        current_y=-58.0,  # Within 5mm tolerance
        target_y=-60.0
    )
    print(f"PLACING (y=-58, target=-60): {result}")
    assert result.has_contact, "Should detect contact via position"
    assert result.method == ContactMethod.POSITION, "Method should be POSITION"
    assert result.position_reached, "position_reached flag should be True"
    
    # Test 4: Leg in PLACING, position NOT reached
    print("\n--- PLACING with position NOT reached ---")
    result_far = estimator.estimate_contact(
        placing_leg,
        current_y=-45.0,  # 15mm away from target
        target_y=-60.0
    )
    print(f"PLACING (y=-45, target=-60): {result_far}")
    assert not result_far.has_contact, "Should not detect contact yet"
    assert not result_far.position_reached, "position_reached should be False"
    
    # Test 5: Timeout detection
    print("\n--- Timeout detection ---")
    timeout_leg = test_legs[2]
    timeout_leg._transition_to(LegState.PLACING)
    timeout_leg.time_in_state_ms = 500.0  # Under default 400ms placing timeout
    result_no_timeout = estimator.estimate_contact(
        timeout_leg, current_y=-30.0, target_y=-60.0
    )
    print(f"PLACING (500ms, tol=400ms): timeout={result_no_timeout.timed_out}")
    # Note: 500 > 400, so should timeout
    assert result_no_timeout.timed_out, "Should timeout at 500ms > 400ms placing_timeout"
    assert result_no_timeout.method == ContactMethod.TIMEOUT
    
    # Test 6: S4 sensed contact
    print("\n--- S4 sensed contact (with s4_available=True) ---")
    s4_estimator = create_contact_estimator(s4_available=True)
    s4_leg = test_legs[3]
    s4_leg._transition_to(LegState.PLACING)
    s4_leg.time_in_state_ms = 50.0  # Fresh
    
    result_sensed = s4_estimator.estimate_contact(
        s4_leg,
        s4_contact=True,  # Foot switch pressed
        current_y=-40.0,  # Not at target
        target_y=-60.0
    )
    print(f"S4 contact=True: {result_sensed}")
    assert result_sensed.has_contact, "Should detect via S4"
    assert result_sensed.method == ContactMethod.SENSED, "Method should be SENSED"
    assert result_sensed.confidence == 1.0, "S4 should have full confidence"
    
    # Test 7: S4 no contact (should fall back to position/timeout)
    print("\n--- S4 no contact, fallback to position ---")
    result_s4_off = s4_estimator.estimate_contact(
        s4_leg,
        s4_contact=False,  # Switch not pressed
        current_y=-59.0,   # But position is close
        target_y=-60.0
    )
    print(f"S4=False, y=-59: {result_s4_off}")
    assert result_s4_off.has_contact, "Should detect via position fallback"
    assert result_s4_off.method == ContactMethod.POSITION, "Should use position fallback"
    
    # Test 8: Batch estimation
    print("\n--- Batch estimation (estimate_all) ---")
    batch_legs = create_legs()
    batch_legs[0]._transition_to(LegState.STANCE)   # Contact
    batch_legs[1]._transition_to(LegState.PLACING)  # Will check position
    batch_legs[2]._transition_to(LegState.SWING)    # Mid-swing
    batch_legs[3]._transition_to(LegState.STANCE)   # Contact
    batch_legs[4]._transition_to(LegState.PLACING)  # Will check position
    batch_legs[5]._transition_to(LegState.STANCE)   # Contact
    
    # Set positions
    for i, leg in enumerate(batch_legs):
        leg.foot_target.y = -60.0
        leg.foot_current.y = -60.0 if leg.state == LegState.STANCE else -40.0
    batch_legs[1].foot_current.y = -58.0  # Close to target
    batch_legs[4].foot_current.y = -59.0  # Close to target
    
    positions = [leg.foot_current for leg in batch_legs]
    results = estimator.estimate_all(batch_legs, current_positions=positions)
    
    for i, (leg, res) in enumerate(zip(batch_legs, results)):
        print(f"  {LEG_NAMES[i]} ({leg.state.name:12s}): {res}")
    
    assert results[0].has_contact and results[3].has_contact and results[5].has_contact, \
        "STANCE legs should have contact"
    assert results[1].has_contact, "PLACING leg near target should have contact"
    
    # Test 9: Statistics tracking
    print("\n--- Statistics tracking ---")
    stats = estimator.get_all_stats()
    print(f"Stats: {stats}")
    assert 'LF' in stats and 'POSITION' in stats['LF'], "Stats should track methods"
    
    # Test 10: update_legs_from_results
    print("\n--- Leg state update from results ---")
    update_legs = create_legs()
    update_legs[0]._transition_to(LegState.PLACING)
    update_legs[0].foot_target.y = -60.0
    update_legs[0].foot_current.y = -59.0
    
    positions = [leg.foot_current for leg in update_legs]
    results = estimator.estimate_all(update_legs, current_positions=positions)
    transitioned = estimator.update_legs_from_results(update_legs, results)
    
    print(f"Transitioned legs: {[LEG_NAMES[i] for i in transitioned]}")
    assert 0 in transitioned, "LF should have transitioned to STANCE"
    assert update_legs[0].state == LegState.STANCE, "LF should be in STANCE now"
    assert update_legs[0].contact_source == ContactSource.ESTIMATED, \
        "Contact source should be ESTIMATED (via position)"
    
    print("\n=== FG7 Test Complete ===")
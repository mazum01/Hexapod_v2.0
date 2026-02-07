"""
collision.py - Analytical Pre-IK Collision Detection.
Consumes kinematics model to prevent self-intersection.

Phase-Aware Collision Detection:
    During gait, collision risk varies by leg phase:
    - SWING phase: Leg is lifted and moving through workspace — HIGH risk
    - STANCE phase: Leg is on ground, stationary — LOW risk
    
    Precomputed risk zones allow faster checking by only testing leg pairs
    where at least one leg is swinging (the dangerous case).

Velocity-Aware Safety Margins:
    Faster motion requires larger safety buffers to account for:
    - Reaction time (controller loop delay)
    - Mechanical inertia (legs can't stop instantly)
    - Trajectory prediction uncertainty
    
    The dynamic margin is: base_margin + velocity_mm_s * TIME_HORIZON_S

Body Collision Model:
    The robot body geometry is derived from the actual STL mesh (Frame Assembly.STL).
    A 2D convex hull in the XZ plane represents the body footprint, with quick
    rejection using bounding/inscribed circle radii. Height bounds (Y axis) are
    also checked to avoid false positives for leg segments above/below the body.
"""
import math
import numpy as np
from enum import IntEnum
from typing import Any, Optional
import kinematics
from kinematics import LegPose, LegPoints

# Default safety parameters (mm). These can be overridden via CollisionConfig.
DEFAULT_LEG_RADIUS_MM = 15.0          # Estimated radius of physical leg parts
DEFAULT_SAFETY_MARGIN_MM = 5.0        # Extra buffer (static)

# Default velocity-aware margin parameters
DEFAULT_TIME_HORIZON_S = 0.05         # Look-ahead time (50ms ≈ 8 ticks at 166Hz)
DEFAULT_MAX_VELOCITY_MARGIN_MM = 20.0 # Cap on velocity-based margin

# Default body keep-out (legacy - superseded by BODY_HULL_XZ)
DEFAULT_BODY_KEEPOUT_RADIUS_MM = 50.0

# Backward-compatible constant aliases (tests and older call sites).
LEG_RADIUS = DEFAULT_LEG_RADIUS_MM
SAFETY_MARGIN = DEFAULT_SAFETY_MARGIN_MM
MIN_DIST_LEG_LEG = 2.0 * DEFAULT_LEG_RADIUS_MM + DEFAULT_SAFETY_MARGIN_MM
TIME_HORIZON_S = DEFAULT_TIME_HORIZON_S
MAX_VELOCITY_MARGIN_MM = DEFAULT_MAX_VELOCITY_MARGIN_MM
BODY_RADIUS = DEFAULT_BODY_KEEPOUT_RADIUS_MM

# -----------------------------------------------------------------------------
# Body Geometry (from STL: Frame Assembly.STL)
# Coordinates: X=left/right, Y=up, Z=front/back. Body centered at origin.
# -----------------------------------------------------------------------------

# 2D convex hull of body in XZ plane (24 vertices, counterclockwise from front-right)
BODY_HULL_XZ = np.array([
    (70.5, 107.9),    # Front-right corner
    (-72.0, 106.4),   # Front-left corner  
    (-76.2, 102.2),   # Front-left chamfer
    (-79.2, 99.2),
    (-83.4, 95.1),
    (-84.7, 93.8),
    (-103.0, 9.3),    # Left side (front)
    (-103.0, -9.5),   # Left side (back)
    (-86.2, -93.8),   # Back-left chamfer
    (-80.5, -99.5),
    (-79.1, -100.9),
    (-77.0, -103.0),
    (-73.3, -106.7),
    (-72.0, -107.9),  # Back-left corner
    (72.0, -107.9),   # Back-right corner
    (73.3, -106.7),   # Back-right chamfer
    (77.0, -103.0),
    (77.6, -102.4),
    (79.8, -100.1),
    (84.2, -95.8),
    (86.2, -93.8),
    (103.0, -9.5),    # Right side (back)
    (103.0, 9.3),     # Right side (front)
    (84.7, 93.8),     # Front-right chamfer
], dtype=np.float32)

# Body dimensions (mm)
BODY_WIDTH_MM = 206.0    # X extent
BODY_DEPTH_MM = 215.9    # Z extent
BODY_HEIGHT_MM = 119.2   # Y extent

# Body vertical bounds (Y axis) - dome on top, flat on bottom
# Note: In robot frame, Y=0 is typically at hip height, so adjust as needed
BODY_Y_BOTTOM_MM = -60.0   # Bottom of body relative to hip plane
BODY_Y_TOP_MM = 60.0       # Top of body (dome) relative to hip plane

# Quick rejection radii for XZ plane checks
BODY_BOUNDING_RADIUS_MM = 129.8    # Points outside this circle are definitely safe
BODY_INSCRIBED_RADIUS_MM = 103.4   # Points inside this circle definitely collide


def _cfg_get(collision_cfg: Any, key: str, default: float) -> float:
    """Fetch a numeric collision tuning value from a config object or dict."""
    if collision_cfg is None:
        return float(default)
    if isinstance(collision_cfg, dict):
        val = collision_cfg.get(key, default)
        return float(default) if val is None else float(val)
    val = getattr(collision_cfg, key, default)
    return float(default) if val is None else float(val)


def get_min_dist_leg_leg_mm(collision_cfg: Any = None) -> float:
    leg_radius_mm = _cfg_get(collision_cfg, 'leg_radius_mm', DEFAULT_LEG_RADIUS_MM)
    safety_margin_mm = _cfg_get(collision_cfg, 'safety_margin_mm', DEFAULT_SAFETY_MARGIN_MM)
    leg_radius_mm = max(0.0, leg_radius_mm)
    safety_margin_mm = max(0.0, safety_margin_mm)
    return 2.0 * leg_radius_mm + safety_margin_mm

# -----------------------------------------------------------------------------
# Velocity-Aware Margin Computation
# -----------------------------------------------------------------------------

def compute_velocity_margin(velocity_mm_s: float, collision_cfg: Any = None) -> float:
    """
    Compute additional safety margin based on motion velocity.
    
    Args:
        velocity_mm_s: Foot velocity magnitude in mm/s
        
    Returns:
        Additional margin in mm to add to MIN_DIST_LEG_LEG
    """
    time_horizon_s = _cfg_get(collision_cfg, 'time_horizon_s', DEFAULT_TIME_HORIZON_S)
    max_velocity_margin_mm = _cfg_get(collision_cfg, 'max_velocity_margin_mm', DEFAULT_MAX_VELOCITY_MARGIN_MM)
    time_horizon_s = max(0.0, time_horizon_s)
    max_velocity_margin_mm = max(0.0, max_velocity_margin_mm)

    # Distance traveled in the look-ahead horizon
    margin = abs(velocity_mm_s) * time_horizon_s
    
    # Cap to prevent excessive margins at high speeds
    return min(margin, max_velocity_margin_mm)

def get_dynamic_threshold_sq(velocity_mm_s: float = 0.0, collision_cfg: Any = None) -> float:
    """
    Get squared distance threshold including velocity margin.
    
    Args:
        velocity_mm_s: Maximum foot velocity in the current motion
        
    Returns:
        Squared threshold distance (mm²)
    """
    base_dist = get_min_dist_leg_leg_mm(collision_cfg)
    velocity_margin = compute_velocity_margin(velocity_mm_s, collision_cfg)
    total_dist = base_dist + velocity_margin
    return total_dist * total_dist

# -----------------------------------------------------------------------------
# Leg Phase Definitions
# -----------------------------------------------------------------------------

class LegPhase(IntEnum):
    """Leg phase in gait cycle."""
    STANCE = 0   # On ground, low risk
    SWING = 1    # In air, high risk

# Adjacent leg pairs that can collide (indices into 6-leg array)
# LF=0, LM=1, LR=2, RF=3, RM=4, RR=5
ADJACENT_PAIRS = [
    (0, 1),  # LF - LM
    (1, 2),  # LM - LR
    (3, 4),  # RF - RM
    (4, 5),  # RM - RR
]

# Tripod gait group definitions
TRIPOD_A = {0, 1, 2}  # LF, LM, LR swing together in phase 0 — wait, check gait_engine
# Actually from gait_engine.py:
# TRIPOD_A = [LEG_RF, LEG_LM, LEG_RR] = [3, 1, 5]
# TRIPOD_B = [LEG_LF, LEG_RM, LEG_LR] = [0, 4, 2]
TRIPOD_A_SET = {3, 1, 5}  # RF, LM, RR
TRIPOD_B_SET = {0, 4, 2}  # LF, RM, LR

# -----------------------------------------------------------------------------
# Phase-Aware Risk Zone Precomputation
# -----------------------------------------------------------------------------

def get_leg_phases_tripod(gait_phase: int) -> list[LegPhase]:
    """
    Get leg phases for tripod gait.
    
    Args:
        gait_phase: 0 or 1 (alternating tripod phases)
        
    Returns:
        List of 6 LegPhase values (one per leg)
    """
    phases = [LegPhase.STANCE] * 6
    
    if gait_phase == 0:
        # Tripod A swings: RF(3), LM(1), RR(5)
        for leg in TRIPOD_A_SET:
            phases[leg] = LegPhase.SWING
    else:
        # Tripod B swings: LF(0), RM(4), LR(2)
        for leg in TRIPOD_B_SET:
            phases[leg] = LegPhase.SWING
            
    return phases

def get_risk_pairs(leg_phases: list[LegPhase]) -> list[tuple[int, int]]:
    """
    Get leg pairs that need collision checking based on current phases.
    
    Only pairs where at least one leg is swinging need checking.
    Stance-stance pairs are low risk (both on ground).
    
    Args:
        leg_phases: List of 6 LegPhase values
        
    Returns:
        List of (leg_a, leg_b) pairs to check
    """
    risk_pairs = []
    
    for a, b in ADJACENT_PAIRS:
        # Check if either leg is swinging
        if leg_phases[a] == LegPhase.SWING or leg_phases[b] == LegPhase.SWING:
            risk_pairs.append((a, b))
            
    return risk_pairs

def get_risk_level(leg_phases: list[LegPhase], leg_a: int, leg_b: int) -> int:
    """
    Get collision risk level for a leg pair.
    
    Returns:
        0 = both stance (low risk)
        1 = one swinging (medium risk) 
        2 = both swinging (high risk - shouldn't happen in tripod)
    """
    a_swing = leg_phases[leg_a] == LegPhase.SWING
    b_swing = leg_phases[leg_b] == LegPhase.SWING
    
    if a_swing and b_swing:
        return 2  # High risk
    elif a_swing or b_swing:
        return 1  # Medium risk
    else:
        return 0  # Low risk

def dist_sq_segment_segment(p1, p2, q1, q2):
    """
    Calculate squared minimum distance between two line segments P1-P2 and Q1-Q2.
    Optimized for collision checks.
    """
    u = p2 - p1
    v = q2 - q1
    w = p1 - q1
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)
    D = a * c - b * b
    sc, sN, sD = 0.0, 0.0, D
    tc, tN, tD = 0.0, 0.0, D

    # Parallel?
    if D < 1e-6:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b * e - c * d)
        tN = (a * e - b * d)
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c

    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    sc = 0.0 if abs(sN) < 1e-6 else sN / sD
    tc = 0.0 if abs(tN) < 1e-6 else tN / tD

    dP = w + (sc * u) - (tc * v)
    return np.dot(dP, dP)

def check_inter_leg_collision(leg_chains: list[LegPoints], 
                               pairs: Optional[list[tuple[int, int]]] = None,
                               velocity_mm_s: float = 0.0,
                               collision_cfg: Any = None) -> bool:
    """
    Check for collisions between leg pairs.
    
    Args:
        leg_chains: FK chains for all 6 legs
        pairs: Optional list of (leg_a, leg_b) pairs to check.
               If None, checks all adjacent pairs.
        velocity_mm_s: Maximum foot velocity for dynamic margin calculation.
               
    Returns:
        True if collision detected.
    """
    if pairs is None:
        pairs = ADJACENT_PAIRS
    
    threshold_sq = get_dynamic_threshold_sq(velocity_mm_s, collision_cfg)
    
    for i, j in pairs:
        chain_a = leg_chains[i]
        chain_b = leg_chains[j]
        
        # Segments for A: Femur (Knee-Ankle), Tibia (Ankle-Foot)
        segs_a = [
            (chain_a.knee, chain_a.ankle),
            (chain_a.ankle, chain_a.foot)
        ]
        
        # Segments for B
        segs_b = [
            (chain_b.knee, chain_b.ankle),
            (chain_b.ankle, chain_b.foot)
        ]
        
        # Check all segment combinations
        for p1, p2 in segs_a:
            for q1, q2 in segs_b:
                dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
                if dist_sq < threshold_sq:
                    return True
                    
    return False

def check_inter_leg_collision_with_distances(
    leg_chains: list[LegPoints],
    pairs: Optional[list[tuple[int, int]]] = None,
    velocity_mm_s: float = 0.0,
    collision_cfg: Any = None,
) -> tuple[bool, dict[tuple[int, int], float], float]:
    """
    Check for collisions and return minimum distances for each pair.
    
    Useful for debugging and visualizing collision margins.
    
    Args:
        velocity_mm_s: Maximum foot velocity for threshold calculation.
    
    Returns:
        (collision_detected, {(leg_a, leg_b): min_distance_mm})
    """
    if pairs is None:
        pairs = ADJACENT_PAIRS
    
    threshold_sq = get_dynamic_threshold_sq(velocity_mm_s, collision_cfg)
    threshold = math.sqrt(threshold_sq)
    distances = {}
    collision = False
    
    for i, j in pairs:
        chain_a = leg_chains[i]
        chain_b = leg_chains[j]
        
        segs_a = [
            (chain_a.knee, chain_a.ankle),
            (chain_a.ankle, chain_a.foot)
        ]
        segs_b = [
            (chain_b.knee, chain_b.ankle),
            (chain_b.ankle, chain_b.foot)
        ]
        
        min_dist_sq = float('inf')
        for p1, p2 in segs_a:
            for q1, q2 in segs_b:
                dist_sq = dist_sq_segment_segment(p1, p2, q1, q2)
                min_dist_sq = min(min_dist_sq, dist_sq)
        
        distances[(i, j)] = math.sqrt(min_dist_sq)
        if min_dist_sq < threshold_sq:
            collision = True
    
    return collision, distances, threshold


def _point_in_polygon_xz(x: float, z: float, hull: np.ndarray) -> bool:
    """
    Check if point (x, z) is inside the 2D convex polygon.
    Uses ray-casting algorithm (odd-even rule).
    """
    n = len(hull)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, zi = hull[i]
        xj, zj = hull[j]
        
        # Check if ray from (x, z) going in +X direction crosses edge (i, j)
        if ((zi > z) != (zj > z)) and (x < (xj - xi) * (z - zi) / (zj - zi) + xi):
            inside = not inside
        j = i
    
    return inside


def _cfg_get_bool(collision_cfg: Any, key: str, default: bool) -> bool:
    """Fetch a boolean collision tuning value from a config object or dict."""
    if collision_cfg is None:
        return default
    if isinstance(collision_cfg, dict):
        val = collision_cfg.get(key, default)
        if val is None:
            return default
        if isinstance(val, bool):
            return val
        if isinstance(val, str):
            return val.lower() in ('true', '1', 'yes')
        return bool(val)
    val = getattr(collision_cfg, key, default)
    if val is None:
        return default
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ('true', '1', 'yes')
    return bool(val)


def check_body_collision(leg_chains: list[LegPoints], collision_cfg: Any = None) -> bool:
    """
    Check if legs intersect the body exclusion zone.
    
    Modes:
    - Simple keepout: circular zone of body_keepout_radius_mm (faster, avoids false positives)
    - Full hull: uses actual body polygon from STL (more accurate, but may flag valid poses)
    
    Set body_use_simple_keepout=true in config for simple circular check.
    """
    # Check if simple circular keepout mode is enabled
    use_simple = _cfg_get_bool(collision_cfg, 'body_use_simple_keepout', False)
    body_margin_mm = _cfg_get(collision_cfg, 'body_safety_margin_mm', DEFAULT_SAFETY_MARGIN_MM)
    
    if use_simple:
        # Simple circular keepout zone
        keepout_r = _cfg_get(collision_cfg, 'body_keepout_radius_mm', DEFAULT_BODY_KEEPOUT_RADIUS_MM)
        keepout_r_sq = (keepout_r + body_margin_mm) ** 2
        
        for chain in leg_chains:
            for pt in [chain.knee, chain.ankle, chain.foot]:
                x, y, z = pt[0], pt[1], pt[2]
                # Height check: skip if point is above or below body
                if y > BODY_Y_TOP_MM or y < BODY_Y_BOTTOM_MM:
                    continue
                dist_sq = x * x + z * z
                if dist_sq < keepout_r_sq:
                    return True
        return False
    
    # Full hull-based check
    bounding_r_sq = (BODY_BOUNDING_RADIUS_MM + body_margin_mm) ** 2
    inscribed_r_sq = (BODY_INSCRIBED_RADIUS_MM - body_margin_mm) ** 2
    inscribed_r_sq = max(0.0, inscribed_r_sq)  # Clamp to avoid negative
    
    for chain in leg_chains:
        # Check knee, ankle, and foot points
        for pt in [chain.knee, chain.ankle, chain.foot]:
            x, y, z = pt[0], pt[1], pt[2]
            
            # Height check: skip if point is above or below body
            if y > BODY_Y_TOP_MM or y < BODY_Y_BOTTOM_MM:
                continue
            
            # XZ plane distance squared from origin
            dist_sq = x * x + z * z
            
            # Quick rejection: outside bounding circle
            if dist_sq > bounding_r_sq:
                continue
            
            # Quick accept: inside inscribed circle
            if dist_sq < inscribed_r_sq:
                return True
            
            # Uncertain zone: do full polygon test
            if _point_in_polygon_xz(x, z, BODY_HULL_XZ):
                return True
    
    return False


def check_body_collision_detailed(leg_chains: list[LegPoints], collision_cfg: Any = None) -> tuple[bool, dict]:
    """
    Check body collision with detailed diagnostics.
    
    Returns:
        (collision_detected, details_dict)
    """
    use_simple = _cfg_get_bool(collision_cfg, 'body_use_simple_keepout', False)
    body_margin_mm = _cfg_get(collision_cfg, 'body_safety_margin_mm', DEFAULT_SAFETY_MARGIN_MM)
    
    details = {
        'collision': False,
        'collision_point': None,
        'collision_leg': None,
        'min_distance_to_hull': float('inf'),
        'mode': 'simple' if use_simple else 'hull',
    }
    
    if use_simple:
        # Simple circular keepout zone
        keepout_r = _cfg_get(collision_cfg, 'body_keepout_radius_mm', DEFAULT_BODY_KEEPOUT_RADIUS_MM)
        keepout_r_sq = (keepout_r + body_margin_mm) ** 2
        
        for leg_idx, chain in enumerate(leg_chains):
            for pt_name, pt in [('knee', chain.knee), ('ankle', chain.ankle), ('foot', chain.foot)]:
                x, y, z = pt[0], pt[1], pt[2]
                if y > BODY_Y_TOP_MM or y < BODY_Y_BOTTOM_MM:
                    continue
                dist_sq = x * x + z * z
                dist = dist_sq ** 0.5
                details['min_distance_to_hull'] = min(details['min_distance_to_hull'], dist - keepout_r)
                if dist_sq < keepout_r_sq:
                    details['collision'] = True
                    details['collision_point'] = pt_name
                    details['collision_leg'] = leg_idx
                    return True, details
        return False, details
    
    # Full hull-based check
    bounding_r_sq = (BODY_BOUNDING_RADIUS_MM + body_margin_mm) ** 2
    inscribed_r_sq = max(0.0, (BODY_INSCRIBED_RADIUS_MM - body_margin_mm) ** 2)
    
    for leg_idx, chain in enumerate(leg_chains):
        for pt_name, pt in [('knee', chain.knee), ('ankle', chain.ankle), ('foot', chain.foot)]:
            x, y, z = pt[0], pt[1], pt[2]
            
            # Height check
            if y > BODY_Y_TOP_MM or y < BODY_Y_BOTTOM_MM:
                continue
            
            dist_sq = x * x + z * z
            
            if dist_sq > bounding_r_sq:
                continue
            
            if dist_sq < inscribed_r_sq:
                details['collision'] = True
                details['collision_point'] = pt_name
                details['collision_leg'] = leg_idx
                return True, details
            
            if _point_in_polygon_xz(x, z, BODY_HULL_XZ):
                details['collision'] = True
                details['collision_point'] = pt_name
                details['collision_leg'] = leg_idx
                return True, details
    
    return False, details


def validate_pose_safety(legs_poses: list[LegPose], 
                         gait_phase: Optional[int] = None,
                         velocity_mm_s: float = 0.0,
                         collision_cfg: Any = None) -> bool:
    """
    Main entry point for collision safety check.
    Calculates FK for all legs, then checks collisions.
    
    Args:
        legs_poses: List of 6 LegPose (joint angles)
        gait_phase: Optional tripod gait phase (0 or 1).
                    If provided, only checks high-risk pairs (faster).
                    If None, checks all adjacent pairs (thorough).
        velocity_mm_s: Maximum foot velocity in mm/s. Higher velocity
                       increases the safety margin (look-ahead buffer).
    
    Returns:
        True if pose is Safe (no collisions).
    """
    # 1. Compute FK chains
    chains = []
    for i in range(6):
        pose = legs_poses[i]
        chains.append(kinematics.fk_leg(i, pose.coxa, pose.femur, pose.tibia))
    
    # 2. Determine which pairs to check
    if gait_phase is not None:
        # Phase-aware: only check pairs with swinging legs
        leg_phases = get_leg_phases_tripod(gait_phase)
        pairs_to_check = get_risk_pairs(leg_phases)
    else:
        # Check all adjacent pairs (conservative)
        pairs_to_check = None  # Will use default ADJACENT_PAIRS
    
    # 3. Check inter-leg collisions (with velocity-aware margin)
    if check_inter_leg_collision(chains, pairs_to_check, velocity_mm_s, collision_cfg):
        return False
        
    # 4. Check body collisions (always check all legs)
    if check_body_collision(chains, collision_cfg):
        return False
        
    return True

def validate_pose_safety_detailed(
    legs_poses: list[LegPose],
    gait_phase: Optional[int] = None,
    velocity_mm_s: float = 0.0,
    collision_cfg: Any = None,
) -> tuple[bool, dict]:
    """
    Detailed collision check with diagnostic info.
    
    Args:
        velocity_mm_s: Maximum foot velocity for dynamic margin.
    
    Returns:
        (is_safe, details_dict)
        
    details_dict contains:
        - 'leg_phases': list of LegPhase for each leg
        - 'pairs_checked': list of (a, b) pairs that were checked
        - 'distances': {(a, b): min_distance_mm} for each pair
        - 'threshold_mm': actual threshold used (includes velocity margin)
        - 'velocity_mm_s': input velocity
        - 'body_collision': bool
        - 'inter_leg_collision': bool
    """
    # 1. Compute FK chains
    chains = []
    for i in range(6):
        pose = legs_poses[i]
        chains.append(kinematics.fk_leg(i, pose.coxa, pose.femur, pose.tibia))
    
    # 2. Determine phases and pairs
    if gait_phase is not None:
        leg_phases = get_leg_phases_tripod(gait_phase)
        pairs_to_check = get_risk_pairs(leg_phases)
    else:
        leg_phases = [LegPhase.SWING] * 6  # Assume all swinging (worst case)
        pairs_to_check = ADJACENT_PAIRS
    
    # 3. Check collisions with distances (velocity-aware)
    inter_leg_collision, distances, threshold = check_inter_leg_collision_with_distances(
        chains, pairs_to_check, velocity_mm_s, collision_cfg
    )
    
    # 4. Check body collision
    body_collision = check_body_collision(chains, collision_cfg)
    
    is_safe = not (inter_leg_collision or body_collision)
    
    details = {
        'leg_phases': leg_phases,
        'pairs_checked': pairs_to_check,
        'distances': distances,
        'threshold_mm': threshold,
        'velocity_mm_s': velocity_mm_s,
        'body_collision': body_collision,
        'inter_leg_collision': inter_leg_collision,
    }
    
    return is_safe, details

# -----------------------------------------------------------------------------
# S2: Learned Collision Model Integration
# -----------------------------------------------------------------------------

# Lazy-loaded predictor (None until first use or model unavailable)
_learned_predictor = None
_learned_model_checked = False


def _load_learned_predictor():
    """Attempt to load learned collision model (once)."""
    global _learned_predictor, _learned_model_checked
    
    if _learned_model_checked:
        return _learned_predictor
    
    _learned_model_checked = True
    
    try:
        from collision_model import CollisionPredictor, DEFAULT_MODEL_PATH
        if DEFAULT_MODEL_PATH.exists():
            _learned_predictor = CollisionPredictor.load(DEFAULT_MODEL_PATH)
            print(f"[collision] Loaded learned model from {DEFAULT_MODEL_PATH}")
        else:
            _learned_predictor = None
    except ImportError:
        _learned_predictor = None
    except Exception as e:
        print(f"[collision] Failed to load learned model: {e}")
        _learned_predictor = None
    
    return _learned_predictor


def validate_pose_safety_learned(
    legs_poses: list[LegPose],
    threshold: float = 0.5,
) -> tuple[bool, dict]:
    """
    Fast collision check using learned neural network model.
    
    This is faster than the analytical check and can be used as a pre-filter
    or for real-time gait planning. Falls back to analytical check if the
    learned model is not available.
    
    Args:
        legs_poses: List of 6 LegPose (coxa, femur, tibia in degrees)
        threshold: Collision probability threshold (default 0.5)
        
    Returns:
        Tuple of (is_safe, details dict)
    """
    predictor = _load_learned_predictor()
    
    if predictor is None:
        # Fallback to analytical check
        return validate_pose_safety_detailed(legs_poses)
    
    # Flatten poses to 18 joint angles
    joint_angles = []
    for pose in legs_poses:
        joint_angles.extend([pose.coxa, pose.femur, pose.tibia])
    
    # Get predictions
    probs = predictor.predict(joint_angles)
    
    # Check if any collision probability exceeds threshold
    is_safe = bool(all(p < threshold for p in probs))
    
    details = {
        'method': 'learned',
        'collision_probs': {
            'pair_LF_LM': float(probs[0]),
            'pair_LM_LR': float(probs[1]),
            'pair_RF_RM': float(probs[2]),
            'pair_RM_RR': float(probs[3]),
            'body': float(probs[4]) if len(probs) > 4 else 0.0,
        },
        'threshold': threshold,
    }
    
    return is_safe, details


def validate_pose_safety_hybrid(
    legs_poses: list[LegPose],
    gait_phase: Optional[float] = None,
    velocity_mm_s: float = 0.0,
    collision_cfg=None,
    learned_threshold: float = 0.3,
) -> tuple[bool, dict]:
    """
    Hybrid collision check: learned model as fast pre-filter, analytical for confirmation.
    
    Pipeline:
        1. Run learned model (fast, ~50µs)
        2. If learned model says SAFE with high confidence (prob < learned_threshold), return safe
        3. If uncertain or collision predicted, run analytical check for ground truth
    
    This gives the speed of the learned model for clearly safe configurations while
    maintaining the accuracy of the analytical check for edge cases.
    
    Args:
        legs_poses: List of 6 LegPose
        gait_phase: Optional phase for tripod gait optimization
        velocity_mm_s: Motion velocity for dynamic margins
        collision_cfg: Configuration object
        learned_threshold: Probability below which we trust the learned model
        
    Returns:
        Tuple of (is_safe, details dict)
    """
    predictor = _load_learned_predictor()
    
    if predictor is None:
        # No learned model, use analytical only
        return validate_pose_safety_detailed(legs_poses, gait_phase, velocity_mm_s, collision_cfg)
    
    # Step 1: Fast learned check
    joint_angles = []
    for pose in legs_poses:
        joint_angles.extend([pose.coxa, pose.femur, pose.tibia])
    
    probs = predictor.predict(joint_angles)
    max_prob = float(max(probs))
    
    details = {
        'method': 'hybrid',
        'learned_max_prob': max_prob,
        'learned_threshold': learned_threshold,
    }
    
    # Step 2: If confidently safe, skip analytical check
    if max_prob < learned_threshold:
        details['analytical_skipped'] = True
        return True, details
    
    # Step 3: Uncertain or collision — run analytical for ground truth
    is_safe, analytical_details = validate_pose_safety_detailed(
        legs_poses, gait_phase, velocity_mm_s, collision_cfg
    )
    
    details['analytical_skipped'] = False
    details['analytical'] = analytical_details
    
    return is_safe, details
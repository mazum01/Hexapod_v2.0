"""
behaviors.py — Concrete behavior implementations for MARS hexapod autonomy.

This module contains the actual behavior implementations that use the
behavior_engine.py framework.

Change log:
  2025-12-27: Initial creation
              - ObstacleAvoidance: ToF-based obstacle detection and avoidance
              - CaughtFootRecovery: Detect and recover from snagged feet
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple, List, Optional
import time

from behavior_engine import Behavior, Action, ActionRequest


# =============================================================================
# ToF Zone Parsing Helpers
# =============================================================================

@dataclass
class ToFZones:
    """Parsed ToF 8×8 grid into navigation zones.
    
    The 8×8 grid is divided into zones for navigation:
    - Front: top 2 rows (rows 0-1)
    - Left: left 2 columns (cols 0-1)  
    - Right: right 2 columns (cols 6-7)
    - Center: middle 4 columns (cols 2-5) of front rows
    - Bottom: bottom 2 rows (for cliff detection)
    
    Each zone has a minimum distance (closest obstacle).
    """
    front_left_min: int = 9999
    front_center_min: int = 9999
    front_right_min: int = 9999
    left_min: int = 9999
    right_min: int = 9999
    bottom_min: int = 9999  # For cliff detection (should be floor distance)
    
    # Statistics
    valid_count: int = 0
    invalid_count: int = 0


def parse_tof_zones(distances: List[int], statuses: List[int]) -> ToFZones:
    """Parse 8×8 ToF grid into navigation zones.
    
    Args:
        distances: 64 distance values in mm (row-major, 0=top-left)
        statuses: 64 status values (5 or 9 = valid)
        
    Returns:
        ToFZones with minimum distances per zone
    """
    VALID_STATUS = (5, 9)
    zones = ToFZones()
    
    if len(distances) < 64 or len(statuses) < 64:
        return zones
    
    # Parse grid into zones
    for row in range(8):
        for col in range(8):
            idx = row * 8 + col
            dist = distances[idx]
            stat = statuses[idx]
            
            # Skip invalid readings
            if stat not in VALID_STATUS or dist <= 0:
                zones.invalid_count += 1
                continue
            zones.valid_count += 1
            
            # Assign to zones based on position
            # Note: sensor may be mounted with different orientation
            # Adjust this mapping based on your physical mounting
            
            # Front zones (top 2 rows = closest to robot's front)
            if row < 2:
                if col < 3:  # Front-left
                    zones.front_left_min = min(zones.front_left_min, dist)
                elif col > 4:  # Front-right
                    zones.front_right_min = min(zones.front_right_min, dist)
                else:  # Front-center
                    zones.front_center_min = min(zones.front_center_min, dist)
            
            # Side zones (all rows)
            if col < 2:  # Left side
                zones.left_min = min(zones.left_min, dist)
            elif col > 5:  # Right side
                zones.right_min = min(zones.right_min, dist)
            
            # Bottom zones (bottom 2 rows = pointing down for cliff detection)
            if row > 5:
                zones.bottom_min = min(zones.bottom_min, dist)
    
    return zones


# =============================================================================
# Obstacle Avoidance Behavior
# =============================================================================

class ObstacleAvoidance(Behavior):
    """Avoid obstacles detected by ToF sensor.
    
    Parses the 8×8 ToF grid into front-left, front-center, front-right zones.
    Takes action based on which zones have close obstacles:
    
    - STOP if center obstacle < stop_distance
    - TURN_LEFT if right obstacle is closer
    - TURN_RIGHT if left obstacle is closer  
    - SLOW_DOWN if any front obstacle < slow_distance
    
    Priority: 80 (high - safety critical)
    """
    
    def __init__(self, 
                 stop_distance_mm: int = 150,
                 slow_distance_mm: int = 300,
                 priority: int = 80,
                 enabled: bool = True):
        """Initialize obstacle avoidance.
        
        Args:
            stop_distance_mm: Distance to trigger full stop
            slow_distance_mm: Distance to trigger slow down
            priority: Behavior priority (default 80)
            enabled: Initial enabled state
        """
        super().__init__("ObstacleAvoidance", priority, enabled)
        self.stop_distance_mm = stop_distance_mm
        self.slow_distance_mm = slow_distance_mm
        self._zones: Optional[ToFZones] = None
        self._last_turn_direction: Optional[Action] = None
    
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        """Evaluate ToF readings and determine avoidance action."""
        
        # Check if ToF is connected
        if not sensor_state.get('tof_connected', False):
            return False, ActionRequest(Action.CONTINUE)
        
        # Get ToF data
        distances = sensor_state.get('tof_distances', [])
        statuses = sensor_state.get('tof_statuses', [])
        
        if not distances or not statuses:
            return False, ActionRequest(Action.CONTINUE)
        
        # Parse zones
        self._zones = parse_tof_zones(distances, statuses)
        zones = self._zones
        
        # Not enough valid readings
        if zones.valid_count < 10:
            return False, ActionRequest(Action.CONTINUE)
        
        # Find closest front obstacle
        front_min = min(zones.front_left_min, zones.front_center_min, zones.front_right_min)
        
        # No obstacle in range
        if front_min > self.slow_distance_mm:
            self._last_turn_direction = None
            return False, ActionRequest(Action.CONTINUE)
        
        # Determine action based on obstacle position
        if zones.front_center_min <= self.stop_distance_mm:
            # Center obstacle very close - stop and decide turn direction
            if zones.front_left_min < zones.front_right_min:
                # Left is more blocked, turn right
                return True, ActionRequest(
                    action=Action.TURN_RIGHT,
                    intensity=1.0,
                    reason=f"Center blocked ({zones.front_center_min}mm), turning right"
                )
            else:
                return True, ActionRequest(
                    action=Action.TURN_LEFT,
                    intensity=1.0,
                    reason=f"Center blocked ({zones.front_center_min}mm), turning left"
                )
        
        if zones.front_left_min <= self.stop_distance_mm:
            # Left obstacle close - turn right
            return True, ActionRequest(
                action=Action.TURN_RIGHT,
                intensity=0.8,
                reason=f"Left obstacle ({zones.front_left_min}mm)"
            )
        
        if zones.front_right_min <= self.stop_distance_mm:
            # Right obstacle close - turn left
            return True, ActionRequest(
                action=Action.TURN_LEFT,
                intensity=0.8,
                reason=f"Right obstacle ({zones.front_right_min}mm)"
            )
        
        # Obstacle in slow zone - reduce speed
        if front_min <= self.slow_distance_mm:
            # Calculate intensity based on distance
            intensity = 1.0 - (front_min - self.stop_distance_mm) / (self.slow_distance_mm - self.stop_distance_mm)
            return True, ActionRequest(
                action=Action.SLOW_DOWN,
                intensity=max(0.3, min(1.0, intensity)),
                reason=f"Obstacle ahead ({front_min}mm)"
            )
        
        return False, ActionRequest(Action.CONTINUE)
    
    @property
    def zones(self) -> Optional[ToFZones]:
        """Last parsed ToF zones (for debugging/display)."""
        return self._zones


# =============================================================================
# Cliff Detection Behavior
# =============================================================================

class CliffDetection(Behavior):
    """Detect cliffs/drops using ToF bottom zones.
    
    If the floor suddenly becomes much farther away (or readings become invalid),
    this indicates a drop-off like stairs or a table edge.
    
    Priority: 90 (very high - safety critical)
    """
    
    def __init__(self,
                 cliff_threshold_mm: int = 100,
                 normal_floor_mm: int = 200,
                 priority: int = 90,
                 enabled: bool = True):
        """Initialize cliff detection.
        
        Args:
            cliff_threshold_mm: How much farther than normal = cliff
            normal_floor_mm: Expected floor distance (calibrate on startup)
            priority: Behavior priority (default 90)
            enabled: Initial enabled state
        """
        super().__init__("CliffDetection", priority, enabled)
        self.cliff_threshold_mm = cliff_threshold_mm
        self.normal_floor_mm = normal_floor_mm
        self._calibrated = False
        self._cliff_detected = False
    
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        """Check for cliff in bottom ToF zones."""
        
        if not sensor_state.get('tof_connected', False):
            return False, ActionRequest(Action.CONTINUE)
        
        distances = sensor_state.get('tof_distances', [])
        statuses = sensor_state.get('tof_statuses', [])
        
        if not distances or not statuses:
            return False, ActionRequest(Action.CONTINUE)
        
        zones = parse_tof_zones(distances, statuses)
        
        # Check if floor is much farther than expected
        if zones.bottom_min < 9999:
            floor_diff = zones.bottom_min - self.normal_floor_mm
            
            if floor_diff > self.cliff_threshold_mm:
                self._cliff_detected = True
                return True, ActionRequest(
                    action=Action.STOP,
                    intensity=1.0,
                    reason=f"Cliff detected! Floor at {zones.bottom_min}mm (expected ~{self.normal_floor_mm}mm)",
                    data={'cliff_depth': floor_diff}
                )
        
        self._cliff_detected = False
        return False, ActionRequest(Action.CONTINUE)
    
    def calibrate_floor(self, floor_distance: int) -> None:
        """Set the expected floor distance.
        
        Call this when robot is on a known flat surface.
        """
        self.normal_floor_mm = floor_distance
        self._calibrated = True


# =============================================================================
# Caught Foot Recovery Behavior
# =============================================================================

class CaughtFootRecovery(Behavior):
    """Detect and recover from feet caught on obstacles.
    
    Detection methods:
    1. Position error: Commanded vs actual joint angle divergence
    2. Timeout: Foot doesn't reach target within expected time
    3. IMU anomaly: Unexpected body tilt during swing
    
    Recovery:
    1. Lift caught leg higher
    2. Back up if lift fails
    3. E-stop after max attempts
    
    Priority: 85 (high - prevents damage)
    """
    
    def __init__(self,
                 position_error_threshold_deg: float = 15.0,
                 timeout_ms: int = 500,
                 recovery_lift_mm: float = 30.0,
                 max_attempts: int = 3,
                 priority: int = 85,
                 enabled: bool = True):
        """Initialize caught foot recovery.
        
        Args:
            position_error_threshold_deg: Joint error to trigger detection
            timeout_ms: Max time for foot to reach target
            recovery_lift_mm: How much higher to lift on recovery
            max_attempts: Max recovery attempts before E-stop
            priority: Behavior priority
            enabled: Initial enabled state
        """
        super().__init__("CaughtFootRecovery", priority, enabled)
        self.position_error_threshold = position_error_threshold_deg
        self.timeout_ms = timeout_ms
        self.recovery_lift_mm = recovery_lift_mm
        self.max_attempts = max_attempts
        
        # State tracking
        self._snag_detected = False
        self._snag_leg: Optional[int] = None
        self._snag_joint: Optional[int] = None
        self._recovery_attempts = 0
        self._snag_start_time: float = 0.0
        self._last_snag_time: float = 0.0
    
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        """Check for caught feet and determine recovery action."""
        
        # Get servo data
        positions = sensor_state.get('servo_positions', [])
        targets = sensor_state.get('servo_targets', [])
        
        if len(positions) < 18 or len(targets) < 18:
            return False, ActionRequest(Action.CONTINUE)
        
        # Only check during gait (movement)
        if not sensor_state.get('gait_active', False):
            self._reset_snag()
            return False, ActionRequest(Action.CONTINUE)
        
        # Check each joint for position error
        max_error = 0.0
        max_error_leg = -1
        max_error_joint = -1
        
        for i in range(18):
            if positions[i] is not None and targets[i] is not None:
                error = abs(positions[i] - targets[i])
                if error > max_error:
                    max_error = error
                    max_error_leg = i // 3
                    max_error_joint = i % 3
        
        # Check if error exceeds threshold
        if max_error > self.position_error_threshold:
            current_time = time.monotonic()
            
            if not self._snag_detected:
                # New snag detected
                self._snag_detected = True
                self._snag_leg = max_error_leg
                self._snag_joint = max_error_joint
                self._snag_start_time = current_time
            
            # Check if snag persists beyond timeout
            snag_duration_ms = (current_time - self._snag_start_time) * 1000
            
            if snag_duration_ms > self.timeout_ms:
                self._recovery_attempts += 1
                self._last_snag_time = current_time
                
                # Determine recovery action
                if self._recovery_attempts >= self.max_attempts:
                    return True, ActionRequest(
                        action=Action.EMERGENCY_STOP,
                        intensity=1.0,
                        reason=f"Leg {self._snag_leg} stuck after {self.max_attempts} recovery attempts",
                        data={'leg': self._snag_leg, 'joint': self._snag_joint, 'error': max_error}
                    )
                
                if self._recovery_attempts > 1:
                    # Already tried lifting, try backing up
                    return True, ActionRequest(
                        action=Action.BACK_UP,
                        intensity=0.5,
                        reason=f"Backing up: Leg {self._snag_leg} caught (error {max_error:.1f}°)",
                        data={'leg': self._snag_leg, 'joint': self._snag_joint, 'recovery_type': 'backup'}
                    )
                else:
                    # First attempt: lift higher
                    return True, ActionRequest(
                        action=Action.CONTINUE,  # Continue but with modified gait
                        intensity=1.0,
                        reason=f"Lifting Leg {self._snag_leg} higher (error {max_error:.1f}°)",
                        data={
                            'leg': self._snag_leg, 
                            'joint': self._snag_joint,
                            'recovery_type': 'lift',
                            'extra_lift_mm': self.recovery_lift_mm
                        }
                    )
        else:
            # No snag - reset if we had one
            if self._snag_detected:
                # Snag resolved
                self._reset_snag()
        
        return False, ActionRequest(Action.CONTINUE)
    
    def _reset_snag(self) -> None:
        """Reset snag detection state."""
        if self._snag_detected:
            self._snag_detected = False
            self._snag_leg = None
            self._snag_joint = None
            # Keep recovery_attempts for a while to detect repeated snags
    
    def reset(self) -> None:
        """Full reset of state."""
        super().reset()
        self._snag_detected = False
        self._snag_leg = None
        self._snag_joint = None
        self._recovery_attempts = 0
        self._snag_start_time = 0.0


# =============================================================================
# Patrol Behavior
# =============================================================================

class Patrol(Behavior):
    """Simple patrol: walk forward with periodic random turns.
    
    This is a low-priority behavior that provides base motion when
    no higher-priority behaviors are active. It actively commands
    forward walking (unlike CONTINUE which is passive).
    
    Priority: 20 (low - easily overridden by safety behaviors)
    """
    
    def __init__(self,
                 turn_interval_s: float = 10.0,
                 patrol_duration_s: float = 0.0,  # 0 = infinite (no timeout)
                 priority: int = 20,
                 enabled: bool = False):  # Disabled by default
        """Initialize patrol behavior.
        
        Args:
            turn_interval_s: Seconds between random turns
            patrol_duration_s: Total patrol duration (0 = infinite)
            priority: Behavior priority
            enabled: Initial enabled state
        """
        super().__init__("Patrol", priority, enabled)
        self.turn_interval_s = turn_interval_s
        self.patrol_duration_s = patrol_duration_s
        
        self._patrol_start: float = 0.0
        self._last_turn: float = 0.0
        self._turn_direction: int = 0  # -1=left, 0=straight, 1=right
        self._turn_duration: float = 0.0
    
    def on_activate(self) -> None:
        """Start patrol timer."""
        self._patrol_start = time.monotonic()
        self._last_turn = self._patrol_start
        self._turn_direction = 0
    
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        """Walk forward, occasionally turn."""
        import random
        
        current_time = time.monotonic()
        
        # Check patrol duration
        if self.patrol_duration_s > 0:
            elapsed = current_time - self._patrol_start
            if elapsed > self.patrol_duration_s:
                return True, ActionRequest(
                    action=Action.STOP,
                    reason="Patrol complete"
                )
        
        # Check for turn
        time_since_turn = current_time - self._last_turn
        
        if self._turn_direction != 0:
            # Currently turning - check if turn complete
            if time_since_turn > self._turn_duration:
                self._turn_direction = 0
                self._last_turn = current_time
        else:
            # Walking straight - check if time to turn
            if time_since_turn > self.turn_interval_s:
                self._turn_direction = random.choice([-1, 1])
                self._turn_duration = random.uniform(0.5, 2.0)
                self._last_turn = current_time
        
        # Return action based on current state
        if self._turn_direction == -1:
            return True, ActionRequest(
                action=Action.TURN_LEFT,
                intensity=0.5,
                reason="Patrol: random left turn"
            )
        elif self._turn_direction == 1:
            return True, ActionRequest(
                action=Action.TURN_RIGHT,
                intensity=0.5,
                reason="Patrol: random right turn"
            )
        else:
            return True, ActionRequest(
                action=Action.WALK_FORWARD,
                intensity=0.7,
                reason="Patrol: walking forward"
            )


# =============================================================================
# Module self-test
# =============================================================================

if __name__ == "__main__":
    print("=== Behaviors Self-Test ===\n")
    
    from behavior_engine import BehaviorArbiter
    
    # Create arbiter with behaviors
    arbiter = BehaviorArbiter()
    arbiter.add_behavior(CliffDetection(priority=90))
    caught_foot = CaughtFootRecovery(position_error_threshold_deg=15.0, 
                                      timeout_ms=100, priority=85)  # Short timeout for test
    arbiter.add_behavior(caught_foot)
    arbiter.add_behavior(ObstacleAvoidance(stop_distance_mm=150, slow_distance_mm=300, priority=80))
    arbiter.add_behavior(Patrol(turn_interval_s=5.0, patrol_duration_s=0, priority=20, enabled=True))  # infinite
    
    print(f"Registered {arbiter.behavior_count} behaviors:")
    for b in arbiter.get_status()['behaviors']:
        print(f"  [{b['priority']:3d}] {b['name']}: enabled={b['enabled']}")
    print()
    
    # Build test ToF grid helper (8x8, row-major)
    # Rows 0-1 = front, rows 6-7 = bottom (floor), cols 0-1 = left, cols 6-7 = right
    def make_tof_grid(front=[500]*16, middle=[500]*32, bottom=[200]*16):
        """Create 8x8 ToF grid with front/middle/bottom zones."""
        return front + middle + bottom
    
    # Simulate some scenarios
    scenarios = [
        {
            "name": "Clear path patrolling",
            "state": {
                "tof_connected": True,
                "tof_distances": make_tof_grid(
                    front=[500]*16,    # All clear ahead
                    middle=[500]*32,
                    bottom=[200]*16    # Normal floor distance
                ),
                "tof_statuses": [5] * 64,
                "servo_positions": [0.0] * 18,
                "servo_targets": [0.0] * 18,
                "gait_active": True,
            }
        },
        {
            "name": "Obstacle on right",
            "state": {
                "tof_connected": True,
                # Front row: left clear, center clear, right blocked
                "tof_distances": make_tof_grid(
                    front=[500]*6 + [100]*2 + [500]*6 + [100]*2,  # Right cols 6-7 close
                    middle=[500]*32,
                    bottom=[200]*16
                ),
                "tof_statuses": [5] * 64,
                "servo_positions": [0.0] * 18,
                "servo_targets": [0.0] * 18,
                "gait_active": True,
            }
        },
        {
            "name": "Center obstacle - close",
            "state": {
                "tof_connected": True,
                # Front rows: center cols 2-5 blocked
                "tof_distances": make_tof_grid(
                    front=[500]*2 + [120]*4 + [500]*2 + [500]*2 + [120]*4 + [500]*2,
                    middle=[500]*32,
                    bottom=[200]*16
                ),
                "tof_statuses": [5] * 64,
                "servo_positions": [0.0] * 18,
                "servo_targets": [0.0] * 18,
                "gait_active": True,
            }
        },
        {
            "name": "Cliff detected - floor drops away",
            "state": {
                "tof_connected": True,
                "tof_distances": make_tof_grid(
                    front=[500]*16,
                    middle=[500]*32,
                    bottom=[500]*16  # Floor suddenly far = cliff!
                ),
                "tof_statuses": [5] * 64,
                "servo_positions": [0.0] * 18,
                "servo_targets": [0.0] * 18,
                "gait_active": True,
            }
        },
    ]
    
    for scenario in scenarios:
        action = arbiter.update(scenario["state"])
        print(f"Scenario: {scenario['name']}")
        print(f"  Active: {arbiter.active_behavior_name}")
        print(f"  Action: {action}")
        print()
    
    # Test caught foot with timeout simulation
    print("--- Testing caught foot recovery with timeout ---")
    caught_state = {
        "tof_connected": True,
        "tof_distances": make_tof_grid(),
        "tof_statuses": [5] * 64,
        "servo_positions": [0.0, 0.0, 0.0, 25.0, 0.0, 0.0] + [0.0]*12,  # Leg 1 joint 0 error
        "servo_targets": [0.0] * 18,
        "gait_active": True,
    }
    
    # First call detects snag but doesn't act yet (waiting for timeout)
    action = arbiter.update(caught_state)
    print(f"  First call (snag detected, waiting): {arbiter.active_behavior_name} -> {action.action.name}")
    
    # Wait for timeout
    time.sleep(0.15)
    
    # Second call after timeout should trigger recovery
    action = arbiter.update(caught_state)
    print(f"  After timeout (recovery triggered): {arbiter.active_behavior_name} -> {action.action.name}")
    if action.data:
        print(f"    Data: {action.data}")
    
    print("\n=== Self-test complete ===")

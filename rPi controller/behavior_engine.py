"""
behavior_engine.py — Behavior arbitration system for MARS hexapod autonomy.

This module provides a priority-based behavior arbitration framework that allows
multiple reactive behaviors to run concurrently, with the highest-priority active
behavior controlling the robot's actions.

Architecture:
    Sensors → Behaviors → Arbiter → GaitEngine
    
    Each behavior:
    - Has a priority (higher = more important)
    - Computes whether it's active based on sensor input
    - Produces an Action when active
    
    The Arbiter:
    - Runs all enabled behaviors each tick
    - Selects the highest-priority active behavior
    - Returns its action (or CONTINUE if none active)

Change log:
  2025-12-27: Initial creation (A1 Behavior Engine Framework)
              - Action enum for robot actions
              - Behavior base class
              - BehaviorArbiter for priority-based selection
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum, auto
from typing import List, Optional, Dict, Any, Tuple
import time


# =============================================================================
# Action Types
# =============================================================================

class Action(IntEnum):
    """Actions that behaviors can request.
    
    These are high-level intentions that the gait engine interprets.
    Listed in rough order of urgency (higher = more urgent).
    """
    CONTINUE = 0       # No action needed, continue current motion
    WALK_FORWARD = 5   # Start/maintain forward walking
    SLOW_DOWN = 10     # Reduce speed but keep moving
    TURN_LEFT = 20     # Turn left (yaw)
    TURN_RIGHT = 21    # Turn right (yaw)
    BACK_UP = 30       # Walk backwards
    STOP = 40          # Stop all motion, hold position
    EMERGENCY_STOP = 50  # Immediate stop, disable servos


@dataclass
class ActionRequest:
    """A behavior's requested action with metadata.
    
    Attributes:
        action: The requested action type
        intensity: How strongly to apply (0.0-1.0), e.g., turn rate
        reason: Human-readable explanation for logging/display
        data: Optional extra data for complex actions
    """
    action: Action = Action.CONTINUE
    intensity: float = 1.0
    reason: str = ""
    data: Dict[str, Any] = field(default_factory=dict)
    
    def __str__(self) -> str:
        if self.action == Action.CONTINUE:
            return "CONTINUE"
        return f"{self.action.name}({self.intensity:.1f}): {self.reason}"


# =============================================================================
# Behavior Base Class
# =============================================================================

class Behavior(ABC):
    """Abstract base class for all behaviors.
    
    Subclasses must implement:
    - compute(): Evaluate sensor state and return (active, action_request)
    
    Subclasses may override:
    - on_activate(): Called when behavior becomes the active one
    - on_deactivate(): Called when behavior is no longer active
    - reset(): Reset internal state
    
    Attributes:
        name: Human-readable behavior name
        priority: Higher = more important (0-100 typical range)
        enabled: Whether this behavior is active
    """
    
    def __init__(self, name: str, priority: int = 50, enabled: bool = True):
        """Initialize behavior.
        
        Args:
            name: Behavior name for logging/display
            priority: Priority level (higher wins arbitration)
            enabled: Initial enabled state
        """
        self.name = name
        self.priority = priority
        self.enabled = enabled
        self._is_active = False
        self._last_action: Optional[ActionRequest] = None
        self._activate_time: float = 0.0
    
    @abstractmethod
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        """Compute whether behavior is active and what action to take.
        
        Args:
            sensor_state: Dictionary with current sensor readings:
                - 'tof_distances': List[int] - 64 ToF distances in mm
                - 'tof_statuses': List[int] - 64 ToF status values
                - 'tof_connected': bool
                - 'imu_roll': float - degrees
                - 'imu_pitch': float - degrees
                - 'imu_yaw': float - degrees
                - 'imu_connected': bool
                - 'servo_positions': List[float] - 18 joint angles (deg)
                - 'servo_targets': List[float] - 18 commanded angles (deg)
                - 'servo_loads': List[float] - 18 servo loads (if available)
                - 'robot_enabled': bool
                - 'gait_active': bool
                - 'velocity': Tuple[float, float, float] - (vx, vy, vyaw)
        
        Returns:
            Tuple of (is_active, action_request):
            - is_active: True if this behavior wants to control the robot
            - action_request: The action to take if active (ignored if not active)
        """
        pass
    
    def on_activate(self) -> None:
        """Called when this behavior becomes the controlling behavior.
        
        Override to perform setup when taking control.
        """
        pass
    
    def on_deactivate(self) -> None:
        """Called when this behavior is no longer controlling.
        
        Override to perform cleanup when releasing control.
        """
        pass
    
    def reset(self) -> None:
        """Reset internal state.
        
        Override to clear any accumulated state.
        """
        self._is_active = False
        self._last_action = None
    
    @property
    def is_active(self) -> bool:
        """Whether this behavior is currently the active controller."""
        return self._is_active
    
    @property
    def active_duration(self) -> float:
        """How long this behavior has been active (seconds)."""
        if not self._is_active:
            return 0.0
        return time.monotonic() - self._activate_time
    
    def _set_active(self, active: bool) -> None:
        """Internal: Update active state and call hooks."""
        was_active = self._is_active
        self._is_active = active
        
        if active and not was_active:
            self._activate_time = time.monotonic()
            self.on_activate()
        elif not active and was_active:
            self.on_deactivate()


# =============================================================================
# Behavior Arbiter
# =============================================================================

class BehaviorArbiter:
    """Priority-based behavior arbitration system.
    
    Runs all enabled behaviors and selects the highest-priority active one.
    Provides the selected action to the gait engine.
    
    Usage:
        arbiter = BehaviorArbiter()
        arbiter.add_behavior(ObstacleAvoidance(priority=80))
        arbiter.add_behavior(Patrol(priority=30))
        
        # In main loop:
        action = arbiter.update(sensor_state)
        gait_engine.apply_action(action)
    """
    
    def __init__(self):
        """Initialize the arbiter with no behaviors."""
        self._behaviors: List[Behavior] = []
        self._active_behavior: Optional[Behavior] = None
        self._last_action: ActionRequest = ActionRequest()
        self._update_count: int = 0
        self._last_update_time: float = 0.0
    
    def add_behavior(self, behavior: Behavior) -> None:
        """Add a behavior to the arbiter.
        
        Behaviors are sorted by priority (highest first) for efficient lookup.
        
        Args:
            behavior: The behavior to add
        """
        self._behaviors.append(behavior)
        # Keep sorted by priority (descending)
        self._behaviors.sort(key=lambda b: b.priority, reverse=True)
    
    def remove_behavior(self, name: str) -> bool:
        """Remove a behavior by name.
        
        Args:
            name: Name of behavior to remove
            
        Returns:
            True if behavior was found and removed
        """
        for i, b in enumerate(self._behaviors):
            if b.name == name:
                if self._active_behavior == b:
                    b._set_active(False)
                    self._active_behavior = None
                self._behaviors.pop(i)
                return True
        return False
    
    def get_behavior(self, name: str) -> Optional[Behavior]:
        """Get a behavior by name.
        
        Args:
            name: Behavior name to find
            
        Returns:
            The behavior, or None if not found
        """
        for b in self._behaviors:
            if b.name == name:
                return b
        return None
    
    def enable_behavior(self, name: str, enabled: bool = True) -> bool:
        """Enable or disable a behavior by name.
        
        Args:
            name: Behavior name
            enabled: Whether to enable
            
        Returns:
            True if behavior was found
        """
        b = self.get_behavior(name)
        if b:
            b.enabled = enabled
            if not enabled and self._active_behavior == b:
                b._set_active(False)
                self._active_behavior = None
            return True
        return False
    
    def set_behavior_enabled(self, name: str, enabled: bool) -> bool:
        """Alias for enable_behavior for clearer API."""
        return self.enable_behavior(name, enabled)
    
    def update(self, sensor_state: Dict[str, Any]) -> ActionRequest:
        """Run all behaviors and return the winning action.
        
        Args:
            sensor_state: Current sensor readings (see Behavior.compute docstring)
            
        Returns:
            The action from the highest-priority active behavior,
            or ActionRequest(CONTINUE) if none are active.
        """
        self._update_count += 1
        self._last_update_time = time.monotonic()
        
        winning_behavior: Optional[Behavior] = None
        winning_action: Optional[ActionRequest] = None
        
        # Evaluate all enabled behaviors (already sorted by priority)
        for behavior in self._behaviors:
            if not behavior.enabled:
                continue
            
            try:
                is_active, action = behavior.compute(sensor_state)
                behavior._last_action = action if is_active else None
                
                if is_active and winning_behavior is None:
                    # First (highest priority) active behavior wins
                    winning_behavior = behavior
                    winning_action = action
                    
            except Exception as e:
                # Log error but don't crash - behavior failed gracefully
                print(f"Behavior '{behavior.name}' error: {e}")
                continue
        
        # Update active states
        for behavior in self._behaviors:
            behavior._set_active(behavior == winning_behavior)
        
        # Update arbiter state
        old_active = self._active_behavior
        self._active_behavior = winning_behavior
        
        if winning_action:
            self._last_action = winning_action
            return winning_action
        else:
            self._last_action = ActionRequest(Action.CONTINUE)
            return self._last_action
    
    def reset(self) -> None:
        """Reset all behaviors and arbiter state."""
        for b in self._behaviors:
            b.reset()
        self._active_behavior = None
        self._last_action = ActionRequest()
        self._update_count = 0
    
    @property
    def active_behavior(self) -> Optional[Behavior]:
        """The currently active (winning) behavior, or None."""
        return self._active_behavior
    
    @property
    def active_behavior_name(self) -> str:
        """Name of active behavior, or 'None'."""
        return self._active_behavior.name if self._active_behavior else "None"
    
    @property
    def last_action(self) -> ActionRequest:
        """The most recent action returned by update()."""
        return self._last_action
    
    @property
    def behavior_count(self) -> int:
        """Number of registered behaviors."""
        return len(self._behaviors)
    
    @property
    def enabled_count(self) -> int:
        """Number of enabled behaviors."""
        return sum(1 for b in self._behaviors if b.enabled)
    
    def get_status(self) -> Dict[str, Any]:
        """Get arbiter status for display/logging.
        
        Returns:
            Dict with arbiter state:
            - active_behavior: str
            - last_action: str
            - behavior_count: int
            - enabled_count: int
            - update_count: int
            - behaviors: List of {name, priority, enabled, active}
        """
        return {
            'active_behavior': self.active_behavior_name,
            'last_action': str(self._last_action),
            'behavior_count': self.behavior_count,
            'enabled_count': self.enabled_count,
            'update_count': self._update_count,
            'behaviors': [
                {
                    'name': b.name,
                    'priority': b.priority,
                    'enabled': b.enabled,
                    'active': b.is_active,
                }
                for b in self._behaviors
            ]
        }


# =============================================================================
# Example/Test Behaviors
# =============================================================================

class TestBehavior(Behavior):
    """Simple test behavior that activates based on a condition function.
    
    Useful for testing the arbiter without real sensor input.
    """
    
    def __init__(self, name: str, priority: int, 
                 condition: callable = None, 
                 action: Action = Action.STOP):
        super().__init__(name, priority)
        self._condition = condition or (lambda s: False)
        self._action = action
    
    def compute(self, sensor_state: Dict[str, Any]) -> Tuple[bool, ActionRequest]:
        active = self._condition(sensor_state)
        return active, ActionRequest(
            action=self._action,
            reason=f"Test: {self.name}"
        )


# =============================================================================
# Module self-test
# =============================================================================

if __name__ == "__main__":
    print("=== Behavior Engine Self-Test ===\n")
    
    # Create arbiter
    arbiter = BehaviorArbiter()
    
    # Add test behaviors
    arbiter.add_behavior(TestBehavior(
        "EmergencyStop", 
        priority=100,
        condition=lambda s: s.get('emergency', False),
        action=Action.EMERGENCY_STOP
    ))
    
    arbiter.add_behavior(TestBehavior(
        "ObstacleStop", 
        priority=80,
        condition=lambda s: s.get('obstacle_close', False),
        action=Action.STOP
    ))
    
    arbiter.add_behavior(TestBehavior(
        "SlowDown", 
        priority=60,
        condition=lambda s: s.get('obstacle_medium', False),
        action=Action.SLOW_DOWN
    ))
    
    arbiter.add_behavior(TestBehavior(
        "Patrol", 
        priority=20,
        condition=lambda s: s.get('patrol_enabled', False),
        action=Action.CONTINUE
    ))
    
    print(f"Registered {arbiter.behavior_count} behaviors\n")
    
    # Test scenarios
    scenarios = [
        {"name": "Idle", "state": {}},
        {"name": "Patrol only", "state": {"patrol_enabled": True}},
        {"name": "Medium obstacle while patrolling", "state": {"patrol_enabled": True, "obstacle_medium": True}},
        {"name": "Close obstacle while patrolling", "state": {"patrol_enabled": True, "obstacle_close": True}},
        {"name": "Emergency", "state": {"patrol_enabled": True, "obstacle_close": True, "emergency": True}},
    ]
    
    for scenario in scenarios:
        action = arbiter.update(scenario["state"])
        print(f"Scenario: {scenario['name']}")
        print(f"  Active: {arbiter.active_behavior_name}")
        print(f"  Action: {action}")
        print()
    
    # Status report
    print("Final status:")
    status = arbiter.get_status()
    for key, val in status.items():
        if key != 'behaviors':
            print(f"  {key}: {val}")
    print("\nBehaviors:")
    for b in status['behaviors']:
        print(f"  [{b['priority']:3d}] {b['name']}: enabled={b['enabled']}, active={b['active']}")
    
    print("\n=== Self-test complete ===")

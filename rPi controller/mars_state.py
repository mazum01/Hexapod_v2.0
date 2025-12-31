"""
mars_state.py — State containers and global state management for MARS controller.

Extracted from controller.py as part of Phase 4 modularization (2025-06-18).
Contains dataclasses for organizing controller state into logical groups.

This module provides structured state containers that can gradually replace
the scattered global variables in controller.py.
"""

from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Optional, Any, List
from enum import IntEnum


# -----------------------------------------------------------------------------
# Enums for state values
# -----------------------------------------------------------------------------

class SteeringMode(IntEnum):
    """Steering mode enum (matches SteeringMode.py)."""
    OMNI = 0
    TANK = 1
    CAR = 2


# -----------------------------------------------------------------------------
# Runtime State Containers
# -----------------------------------------------------------------------------

@dataclass
class LoopTimingState:
    """Loop timing and scheduling state."""
    run: bool = True
    loop_start_time: float = field(default_factory=time.time)
    loop_dt: float = 0.0
    loop_target_ms: float = 5.0
    next_tick_time: Optional[float] = None
    screen_refresh_ms: float = 100.0
    show_loop_time: bool = False
    
    # Telemetry-synchronized timing
    teensy_loop_us: int = 6024
    last_s1_mono_time: Optional[float] = None
    telem_sync_active: bool = False
    telem_sync_fallback_ms: float = 50.0


@dataclass
class TeensyState:
    """Teensy connection and telemetry state."""
    teensy: Any = None  # serial.Serial or None
    error_count: int = 0
    error_threshold: int = 3
    next_scan_at: float = 0.0
    reconnect_backoff_s: float = 1.5
    
    # Telemetry auto-start state
    telemetry_started_by_script: bool = False
    last_telemetry_time: Optional[float] = None
    telemetry_grace_deadline: Optional[float] = None
    telemetry_grace_seconds: float = 0.75
    telemetry_retry_seconds: float = 2.0
    telemetry_retry_deadline: Optional[float] = None
    telemetry_retry_count: int = 0
    
    # Binary telemetry
    prefer_binary_telemetry: bool = True
    binary_active: bool = False
    rx_buffer: bytes = b""
    ascii_buffer: str = ""
    
    # Debug telemetry capture
    last_raw_s1: str = ""
    last_parsed_s1: list = field(default_factory=list)
    last_raw_s2: str = ""
    last_parsed_s2: list = field(default_factory=list)


@dataclass
class GaitState:
    """Gait engine and motion state."""
    engine: Any = None  # GaitEngine instance or None
    active: bool = False
    strafe_input: float = 0.0  # Left stick X: -1 to +1
    speed_input: float = 0.0   # Left stick Y: -1 to +1
    tick_count: int = 0
    send_divisor: int = 3
    transition: Any = None  # GaitTransition instance
    
    # Gait parameters (from config)
    cycle_ms: int = 2000
    step_len_mm: float = 40.0
    max_step_len_mm: float = 40.0
    base_y_mm: float = -120.0
    overlap_pct: float = 5.0
    smoothing_alpha: float = 0.15
    turn_max_deg_s: float = 60.0
    
    # Bezier curve shape parameters
    bezier_p1_height: float = 0.15
    bezier_p1_overshoot: float = 1.1
    bezier_p2_height: float = 1.5
    bezier_p3_height: float = 0.35
    bezier_p3_overshoot: float = 1.1


@dataclass
class MoveState:
    """Kinematic move state (non-cyclic sequences)."""
    engine: Any = None
    active: bool = False
    tick_count: int = 0
    send_divisor: int = 1


@dataclass
class DisplayState:
    """Display and menu state."""
    menu_visible: bool = False
    mirror_display: bool = False
    force_update: bool = False
    brightness: int = 100
    menu_state: Any = None
    mars_menu: Any = None  # MarsMenu instance
    steering_mode: int = 0  # SteeringMode.OMNI
    
    # Display thread settings
    thread_enabled: bool = True
    thread_hz: float = 15.0
    blink_frame_divisor: int = 2


@dataclass 
class EyeSettings:
    """Eye appearance settings."""
    shape: int = 2  # EYE_SHAPE.ROUNDRECTANGLE
    spacing_offset: int = 10
    center_offset: int = 5
    vertical_offset: int = 0
    eyelid_angle: int = 0
    blink_percent_step: float = 0.08
    rotation: int = -10
    size_x: int = 25
    size_y: int = 45
    look_range_x: float = 20.0
    look_range_y: float = 10.0
    
    # Human eye specific
    human_spacing_pct: float = 0.25
    human_size: int = 33
    human_color_idx: int = 0
    
    # Colors by shape (R, G, B)
    color_ellipse: tuple = (255, 80, 20)
    color_rectangle: tuple = (255, 255, 255)
    color_roundrect: tuple = (10, 120, 255)
    color_x: tuple = (255, 0, 0)
    color_spider: tuple = (50, 220, 50)
    color_human: tuple = (70, 130, 180)
    color_cat: tuple = (255, 180, 0)
    color_hypno: tuple = (180, 0, 255)
    color_anime: tuple = (100, 180, 255)
    
    # Human eye color palette
    human_color_blue: tuple = (70, 130, 180)
    human_color_green: tuple = (60, 140, 80)
    human_color_hazel: tuple = (140, 110, 60)
    human_color_brown: tuple = (100, 60, 30)
    human_color_dark_brown: tuple = (50, 30, 20)


@dataclass
class PounceSettings:
    """Pounce move timing and geometry settings."""
    prep_ms: int = 400
    rear_ms: int = 250
    lunge_ms: int = 250
    recover_ms: int = 500
    back1_z_mm: float = 55.0
    back2_z_mm: float = 75.0
    push_z_mm: float = -60.0
    strike_z_mm: float = 140.0
    crouch_dy_mm: float = 55.0
    lift_dy_mm: float = 110.0
    front_z_mm: float = 20.0


@dataclass
class MenuSettings:
    """Menu appearance settings."""
    theme: int = 0       # 0=MARS, 1=LCARS
    palette: int = 0     # LCARS palette index


@dataclass
class SafetyState:
    """Safety and lockout state."""
    lockout: bool = False
    collision: bool = False
    limit: bool = False
    last_lockout: bool = False


@dataclass
class AutoDisableState:
    """Auto-disable scheduling state."""
    at: Optional[float] = None  # Epoch seconds when to send DISABLE
    reason: Optional[str] = None  # e.g., "TUCK", "STAND"
    gen: int = 0  # Generation counter to detect stale timers
    seconds: float = 5.0  # Default auto-disable delay


@dataclass
class DebugState:
    """Debug flags and settings."""
    verbose: bool = True
    logging: bool = True
    debug_telemetry: bool = False
    debug_send_all: bool = False
    gait_debug_cmds: set = field(default_factory=lambda: {
        b'r', b'w', b't', b'm', b'x', b'z',
        b'ENABLE', b'DISABLE', b'LEG ALL ENABLE', b'TUCK', b'STAND'
    })


# -----------------------------------------------------------------------------
# Telemetry Data Containers
# -----------------------------------------------------------------------------

@dataclass
class TelemetryArrays:
    """Raw telemetry data arrays (legacy format)."""
    state: List[float] = field(default_factory=lambda: [0.0] * 11)
    servo: List[List[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0] for _ in range(18)])
    legs: List[List[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0] for _ in range(6)])


# -----------------------------------------------------------------------------
# Main State Container
# -----------------------------------------------------------------------------

@dataclass
class MarsControllerState:
    """Top-level container for all MARS controller state.
    
    This class consolidates all the scattered global variables into
    a single structured container for easier management and testing.
    """
    # Timing
    timing: LoopTimingState = field(default_factory=LoopTimingState)
    
    # Connection
    teensy: TeensyState = field(default_factory=TeensyState)
    controller: Any = None  # XBoxController instance
    retry_count: int = 0
    
    # Motion
    gait: GaitState = field(default_factory=GaitState)
    move: MoveState = field(default_factory=MoveState)
    
    # Display
    display: DisplayState = field(default_factory=DisplayState)
    eyes: EyeSettings = field(default_factory=EyeSettings)
    
    # Settings
    pounce: PounceSettings = field(default_factory=PounceSettings)
    menu: MenuSettings = field(default_factory=MenuSettings)
    
    # Safety
    safety: SafetyState = field(default_factory=SafetyState)
    auto_disable: AutoDisableState = field(default_factory=AutoDisableState)
    
    # Debug
    debug: DebugState = field(default_factory=DebugState)
    
    # Telemetry arrays (legacy)
    telemetry: TelemetryArrays = field(default_factory=TelemetryArrays)
    
    # Command throttling
    cmd_throttle_ms: float = 50.0
    enabled_local: bool = False
    last_posture: Optional[bytes] = None


# -----------------------------------------------------------------------------
# Factory Functions
# -----------------------------------------------------------------------------

def create_default_state() -> MarsControllerState:
    """Create a new MarsControllerState with default values."""
    return MarsControllerState()


def create_gait_state_from_config(cfg: dict) -> GaitState:
    """Create GaitState from a config dictionary.
    
    Args:
        cfg: Dictionary with keys like 'cycle_ms', 'step_len_mm', etc.
        
    Returns:
        Initialized GaitState
    """
    return GaitState(
        cycle_ms=cfg.get('cycle_ms', 2000),
        step_len_mm=cfg.get('step_len_mm', 40.0),
        max_step_len_mm=cfg.get('max_step_len_mm', 40.0),
        base_y_mm=cfg.get('base_y_mm', -120.0),
        overlap_pct=cfg.get('overlap_pct', 5.0),
        smoothing_alpha=cfg.get('smoothing_alpha', 0.15),
        turn_max_deg_s=cfg.get('turn_max_deg_s', 60.0),
        send_divisor=cfg.get('send_divisor', 3),
        bezier_p1_height=cfg.get('bezier_p1_height', 0.15),
        bezier_p1_overshoot=cfg.get('bezier_p1_overshoot', 1.1),
        bezier_p2_height=cfg.get('bezier_p2_height', 1.5),
        bezier_p3_height=cfg.get('bezier_p3_height', 0.35),
        bezier_p3_overshoot=cfg.get('bezier_p3_overshoot', 1.1),
    )


def create_eye_settings_from_config(cfg: dict) -> EyeSettings:
    """Create EyeSettings from a config dictionary.
    
    Args:
        cfg: Dictionary with eye setting keys
        
    Returns:
        Initialized EyeSettings
    """
    return EyeSettings(
        shape=cfg.get('shape', 2),
        spacing_offset=cfg.get('spacing_offset', 10),
        center_offset=cfg.get('center_offset', 5),
        vertical_offset=cfg.get('vertical_offset', 0),
        rotation=cfg.get('rotation', -10),
        size_x=cfg.get('size_x', 25),
        size_y=cfg.get('size_y', 45),
        human_size=cfg.get('human_size', 33),
        human_color_idx=cfg.get('human_color_idx', 0),
    )


def create_pounce_settings_from_config(cfg: dict) -> PounceSettings:
    """Create PounceSettings from a config dictionary.
    
    Args:
        cfg: Dictionary with pounce setting keys
        
    Returns:
        Initialized PounceSettings
    """
    return PounceSettings(
        prep_ms=cfg.get('prep_ms', 400),
        rear_ms=cfg.get('rear_ms', 250),
        lunge_ms=cfg.get('lunge_ms', 250),
        recover_ms=cfg.get('recover_ms', 500),
        back1_z_mm=cfg.get('back1_z_mm', 55.0),
        back2_z_mm=cfg.get('back2_z_mm', 75.0),
        push_z_mm=cfg.get('push_z_mm', -60.0),
        strike_z_mm=cfg.get('strike_z_mm', 140.0),
        crouch_dy_mm=cfg.get('crouch_dy_mm', 55.0),
        lift_dy_mm=cfg.get('lift_dy_mm', 110.0),
        front_z_mm=cfg.get('front_z_mm', 20.0),
    )


# -----------------------------------------------------------------------------
# Module exports
# -----------------------------------------------------------------------------
__all__ = [
    # Enums
    'SteeringMode',
    # State containers
    'LoopTimingState', 'TeensyState', 'GaitState', 'MoveState',
    'DisplayState', 'EyeSettings', 'PounceSettings', 'MenuSettings',
    'SafetyState', 'AutoDisableState', 'DebugState', 'TelemetryArrays',
    'MarsControllerState',
    # Factory functions
    'create_default_state',
    'create_gait_state_from_config',
    'create_eye_settings_from_config',
    'create_pounce_settings_from_config',
]

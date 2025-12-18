#----------------------------------------------------------------------------------------------------------------------
#    telemetry.py — Telemetry Dataclasses and Parsing for MARS Hexapod
#----------------------------------------------------------------------------------------------------------------------
# Extracted from controller.py as part of modularization (Phase 1).
# 
# This module contains:
#   - Telemetry index constants (S1 schema)
#   - Telemetry dataclasses (SystemTelemetry, ServoTelemetry, LegTelemetry, SafetyTelemetry)
#   - ASCII telemetry parsing functions (processTelemS1..S5)
#   - Binary telemetry constants
#   - Safety overlay helper
#
# The binary telemetry parsing is tightly coupled to Controller state updates and remains
# inline in controller.py for now. This module provides the shared data structures and
# ASCII parsing functions.
#----------------------------------------------------------------------------------------------------------------------

from dataclasses import dataclass
from typing import Optional, List, Dict, Any, Tuple

#----------------------------------------------------------------------------------------------------------------------
# Telemetry index constants (S1 schema)
# These named constants prevent magic numbers and make schema changes explicit.
#----------------------------------------------------------------------------------------------------------------------
IDX_LOOP_US = 0          # S1[0]: Teensy loop period in microseconds
IDX_BATTERY_V = 1        # S1[1]: Battery voltage
IDX_CURRENT_A = 2        # S1[2]: Current draw (amps)
IDX_PITCH_DEG = 3        # S1[3]: IMU pitch angle
IDX_ROLL_DEG = 4         # S1[4]: IMU roll angle
IDX_YAW_DEG = 5          # S1[5]: IMU yaw angle
IDX_GAIT = 6             # S1[6]: Current gait mode (0-9)
IDX_MODE = 7             # S1[7]: Operational mode
IDX_SAFETY = 8           # S1[8]: Safety status flags
IDX_ROBOT_ENABLED = 9    # S1[9]: Robot enable flag (1.0 = enabled, 0.0 = disabled)

#----------------------------------------------------------------------------------------------------------------------
# Binary telemetry framing constants
#----------------------------------------------------------------------------------------------------------------------
TELEM_SYNC = b'\xA5\x5A'

#----------------------------------------------------------------------------------------------------------------------
# Telemetry Dataclasses
#----------------------------------------------------------------------------------------------------------------------

@dataclass(slots=True)
class SystemTelemetry:
    """System-level telemetry from S1 segment."""
    loop_us: int = 0
    battery_v: float = 0.0
    current_a: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    gait: int = 0
    mode: int = 0
    safety: int = 0
    robot_enabled: bool = False
    valid: bool = False


@dataclass(slots=True)
class ServoTelemetry:
    """Per-servo telemetry from S2/S3 segments."""
    voltage_v: float = 0.0
    temp_c: int = 0
    enabled: bool = False
    valid_s2: bool = False
    valid_s3: bool = False


@dataclass(slots=True)
class LegTelemetry:
    """Per-leg telemetry from S4 segment."""
    contact: bool = False
    valid: bool = False


@dataclass(slots=True)
class SafetyTelemetry:
    """Safety state telemetry from S5 segment."""
    lockout: bool = False
    cause_mask: int = 0
    override_mask: int = 0
    clearance_mm: int = 0
    soft_limits: bool = False
    collision: bool = False
    temp_c: int = 0
    valid: bool = False


#----------------------------------------------------------------------------------------------------------------------
# Factory functions for telemetry arrays
#----------------------------------------------------------------------------------------------------------------------

def create_servo_telemetry_array() -> List[ServoTelemetry]:
    """Create array of 18 ServoTelemetry objects (one per servo)."""
    return [ServoTelemetry() for _ in range(18)]


def create_leg_telemetry_array() -> List[LegTelemetry]:
    """Create array of 6 LegTelemetry objects (one per leg)."""
    return [LegTelemetry() for _ in range(6)]


#----------------------------------------------------------------------------------------------------------------------
# ASCII Telemetry Parsing Functions
#----------------------------------------------------------------------------------------------------------------------

def processTelemS1(elements: List[str], state: List[float], 
                   out_system: Optional[SystemTelemetry] = None,
                   verbose: bool = False) -> None:
    """Processes the S1 telemetry data.
    
    Expected: 10 numeric fields (indices 0..9). Warn if shorter; ignore extras if longer.
    
    Args:
        elements: List of string values from comma-separated S1 payload
        state: Mutable list of 10 floats to update (legacy state array)
        out_system: Optional SystemTelemetry dataclass to populate
        verbose: Whether to print warnings
    """
    expected = 10
    count = len(elements)
    if count < expected:
        if verbose:
            print(f"WARN S1 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and verbose:
        print(f"WARN S1 length {count} > expected {expected}, truncating.", end="\r\n")
    try:
        # Only use first expected elements
        for i in range(expected):
            state[i] = float(elements[i])

        if out_system is not None:
            out_system.loop_us = int(state[IDX_LOOP_US])
            out_system.battery_v = float(state[IDX_BATTERY_V])
            out_system.current_a = float(state[IDX_CURRENT_A])
            out_system.pitch_deg = float(state[IDX_PITCH_DEG])
            out_system.roll_deg = float(state[IDX_ROLL_DEG])
            out_system.yaw_deg = float(state[IDX_YAW_DEG])
            out_system.gait = int(state[IDX_GAIT])
            out_system.mode = int(state[IDX_MODE])
            out_system.safety = int(state[IDX_SAFETY])
            out_system.robot_enabled = (float(state[IDX_ROBOT_ENABLED]) == 1.0)
            out_system.valid = True
    except ValueError as e:
        print(f"Error processing S1 telemetry data: {e}", end="\r\n")


def processTelemS2(elements: List[str], servo: List[List], 
                   out_servo: Optional[List[ServoTelemetry]] = None,
                   verbose: bool = False) -> None:
    """Processes the S2 telemetry data (servo enable states).
    
    Expected: 18 enable integers.
    
    Args:
        elements: List of string values from comma-separated S2 payload
        servo: Mutable list of servo state arrays [[voltage, temp, enabled], ...]
        out_servo: Optional list of ServoTelemetry dataclasses to populate
        verbose: Whether to print warnings
    """
    expected = 18
    count = len(elements)
    if count < expected:
        if verbose:
            print(f"WARN S2 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and verbose:
        print(f"WARN S2 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(expected):
            enabled_val = int(elements[idx])
            servo[idx][2] = enabled_val
            if out_servo is not None and idx < len(out_servo):
                out_servo[idx].enabled = bool(enabled_val)
                out_servo[idx].valid_s2 = True
    except ValueError as e:
        print(f"Error processing S2 telemetry data: {e}", end="\r\n")


def processTelemS3(elements: List[str], servo: List[List], 
                   out_servo: Optional[List[ServoTelemetry]] = None,
                   verbose: bool = False) -> None:
    """Processes the S3 telemetry data (servo voltage + temperature).
    
    Expected: 36 numeric entries: 18 voltages then 18 temperatures.
    
    Args:
        elements: List of string values from comma-separated S3 payload
        servo: Mutable list of servo state arrays [[voltage, temp, enabled], ...]
        out_servo: Optional list of ServoTelemetry dataclasses to populate
        verbose: Whether to print warnings
    """
    expected = 36
    count = len(elements)
    if count < expected:
        if verbose:
            print(f"WARN S3 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and verbose:
        print(f"WARN S3 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(18):
            voltage_v = float(elements[idx])
            temp_c = int(elements[idx + 18])
            servo[idx][0] = voltage_v
            servo[idx][1] = temp_c
            if out_servo is not None and idx < len(out_servo):
                out_servo[idx].voltage_v = voltage_v
                out_servo[idx].temp_c = temp_c
                out_servo[idx].valid_s3 = True
    except ValueError as e:
        print(f"Error processing S3 telemetry data: {e}", end="\r\n")


def processTelemS4(elements: List[str], leg: List[List], 
                   out_leg: Optional[List[LegTelemetry]] = None,
                   verbose: bool = False) -> None:
    """Processes the S4 telemetry data (leg contact states).
    
    Expected: 6 integers (one per leg).
    
    Args:
        elements: List of string values from comma-separated S4 payload
        leg: Mutable list of leg state arrays [[contact], ...]
        out_leg: Optional list of LegTelemetry dataclasses to populate
        verbose: Whether to print warnings
    """
    expected = 6
    count = len(elements)
    if count < expected:
        if verbose:
            print(f"WARN S4 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and verbose:
        print(f"WARN S4 length {count} > expected {expected}, truncating.", end="\r\n")

    try:
        for idx in range(expected):
            contact_val = int(elements[idx])
            leg[idx][0] = contact_val
            if out_leg is not None and idx < len(out_leg):
                out_leg[idx].contact = bool(contact_val)
                out_leg[idx].valid = True
    except ValueError as e:
        print(f"Error processing S4 telemetry data: {e}", end="\r\n")


def processTelemS5(elements: List[str], safety_state: Dict[str, Any], 
                   out_safety: Optional[SafetyTelemetry] = None,
                   verbose: bool = False) -> None:
    """Process S5 safety telemetry.

    Expected format (7 fields):
        lockout, cause_mask, override_mask, clearance_mm, soft_limits,
        collision, temp_c
    
    Args:
        elements: List of string values from comma-separated S5 payload
        safety_state: Mutable dict to update with safety state
        out_safety: Optional SafetyTelemetry dataclass to populate
        verbose: Whether to print warnings
    """
    expected = 7
    count = len(elements)
    if count < expected:
        if verbose:
            print(f"WARN S5 length {count} < expected {expected}, skipping.", end="\r\n")
        return
    if count > expected and verbose:
        print(f"WARN S5 length {count} > expected {expected}, truncating.", end="\r\n")
    try:
        lockout = bool(int(elements[0]))
        cause_mask = int(elements[1])
        override_mask = int(elements[2])
        clearance_mm = int(elements[3])
        soft_limits = bool(int(elements[4]))
        collision = bool(int(elements[5]))
        temp_c = int(elements[6])

        safety_state["lockout"] = lockout
        safety_state["cause_mask"] = cause_mask
        safety_state["override_mask"] = override_mask
        safety_state["clearance_mm"] = clearance_mm
        safety_state["soft_limits"] = soft_limits
        safety_state["collision"] = collision
        safety_state["temp_c"] = temp_c

        if out_safety is not None:
            out_safety.lockout = lockout
            out_safety.cause_mask = cause_mask
            out_safety.override_mask = override_mask
            out_safety.clearance_mm = clearance_mm
            out_safety.soft_limits = soft_limits
            out_safety.collision = collision
            out_safety.temp_c = temp_c
            out_safety.valid = True
    except ValueError as e:
        print(f"Error processing S5 telemetry data: {e}", end="\r\n")


#----------------------------------------------------------------------------------------------------------------------
# Safety Overlay Helper
#----------------------------------------------------------------------------------------------------------------------

def get_safety_overlay_text(safety_telem: Optional[SafetyTelemetry], 
                            safety_state: Dict[str, Any]) -> Tuple[bool, str]:
    """Return (active, text) tuple for safety overlay rendering.
    
    Args:
        safety_telem: Structured SafetyTelemetry if available
        safety_state: Legacy dict-based safety state
        
    Returns:
        Tuple of (is_active, overlay_text)
    """
    if safety_telem is not None and getattr(safety_telem, 'valid', False):
        active = bool(getattr(safety_telem, 'lockout', False))
        mask = int(getattr(safety_telem, 'cause_mask', 0) or 0)
    else:
        active = safety_state.get("lockout", False)
        mask = safety_state.get("cause_mask", 0)
    
    if not active:
        return False, ""
    
    labels = []
    # Bit mapping from firmware LockoutCauseBits
    if mask & 0x01:
        labels.append("TEMP")
    if mask & 0x02:
        labels.append("COLLISION")
    if not labels:
        labels.append("UNKNOWN")
    base = "SAFETY LOCKOUT: " + "/".join(labels)
    return True, base


#----------------------------------------------------------------------------------------------------------------------
# Gait name mapping
#----------------------------------------------------------------------------------------------------------------------

GAIT_NAMES = {
    0: "TRI",
    1: "RIPPLE",
    2: "WAV",
    3: "QUAD",
    4: "BI",
    5: "HOP",
    6: "TRANS",
    7: "STILL",
    8: "HOME",
    9: "NONE",
}


def getGait(gait: int) -> str:
    """Convert gait number to string representation."""
    return GAIT_NAMES.get(gait, "UNKNOWN")


#----------------------------------------------------------------------------------------------------------------------
# Module exports
#----------------------------------------------------------------------------------------------------------------------

__all__ = [
    # Index constants
    'IDX_LOOP_US', 'IDX_BATTERY_V', 'IDX_CURRENT_A', 'IDX_PITCH_DEG',
    'IDX_ROLL_DEG', 'IDX_YAW_DEG', 'IDX_GAIT', 'IDX_MODE', 'IDX_SAFETY',
    'IDX_ROBOT_ENABLED',
    # Binary framing
    'TELEM_SYNC',
    # Dataclasses
    'SystemTelemetry', 'ServoTelemetry', 'LegTelemetry', 'SafetyTelemetry',
    # Factory functions
    'create_servo_telemetry_array', 'create_leg_telemetry_array',
    # Parsing functions
    'processTelemS1', 'processTelemS2', 'processTelemS3', 'processTelemS4', 'processTelemS5',
    # Helpers
    'get_safety_overlay_text', 'getGait', 'GAIT_NAMES',
]

"""
posture.py — Posture and move routines for MARS controller.

Extracted from controller.py as part of Phase 3 modularization (2025-06-18).
Contains apply_posture(), ensure_enabled(), start_pounce_move(), and auto-disable scheduling.

NOTE: The auto-disable state variables (_autoDisableAt, _autoDisableReason, _autoDisableGen)
are defined in controller.py as the authoritative source. This module receives them via
function parameters and returns updated values.
"""

from __future__ import annotations
import time
from typing import Optional, Callable, Any

from telemetry import IDX_ROBOT_ENABLED

# -----------------------------------------------------------------------------
# Auto-disable scheduling - module-level cache for internal use only
# The authoritative state lives in controller.py; these are updated via sync functions
# -----------------------------------------------------------------------------
_lastPosture: Optional[bytes] = None  # tracks last posture name sent


def get_last_posture() -> Optional[bytes]:
    """Return the last posture sent."""
    return _lastPosture


def set_last_posture(posture: Optional[bytes]) -> None:
    """Set the last posture (for state sync)."""
    global _lastPosture
    _lastPosture = posture


def ensure_enabled(
    teensy: Any,
    state: list,
    safety_state: dict,
    system_telem: Any,
    enabled_local: bool,
    send_cmd_fn: Callable[[bytes, bool], bool],
    verbose: bool = False
) -> tuple:
    """Ensure robot is enabled by sending LEG ALL ENABLE + ENABLE if needed.
    
    Args:
        teensy: Serial connection (None if disconnected)
        state: Telemetry state array
        safety_state: Safety state dict with 'lockout' key
        system_telem: SystemTelemetry object or None
        enabled_local: Local tracking of enabled state
        send_cmd_fn: Function to send commands
        verbose: Print debug messages
        
    Returns:
        (commands_sent: bool, new_enabled_local: bool)
    """
    # Honor firmware safety lockout
    if safety_state.get("lockout", False):
        if verbose:
            print("Safety lockout active; refusing ENABLE. Clear SAFETY on Teensy first.", end="\r\n")
        return (False, enabled_local)
    
    if teensy is None:
        return (False, enabled_local)
    
    # Check both local tracking and telemetry state
    if system_telem is not None and getattr(system_telem, 'valid', False):
        enabled_telem = system_telem.robot_enabled
    else:
        enabled_telem = (state[IDX_ROBOT_ENABLED] == 1.0) if (len(state) > IDX_ROBOT_ENABLED) else False
    
    if enabled_local and enabled_telem:
        return (False, enabled_local)  # Already enabled
    
    send_cmd_fn(b'LEG ALL ENABLE', True)
    send_cmd_fn(b'ENABLE', True)
    return (True, True)


def apply_posture(
    name: bytes | str,
    teensy: Any,
    state: list,
    safety_state: dict,
    system_telem: Any,
    enabled_local: bool,
    send_cmd_fn: Callable[[bytes, bool], bool],
    auto_disable_at: Optional[float],
    auto_disable_reason: Optional[str],
    auto_disable_gen: int,
    auto_disable_s: float = 0.0,
    require_enable: bool = True,
    verbose: bool = False
) -> tuple:
    """Unified posture helper (TUCK/STAND/HOME) using byte commands.
    
    Auto enable sequence if required; schedules auto disable if > 0.
    Auto-disable is reason-aware.
    
    Args:
        name: Posture name (bytes or str): TUCK, STAND, or HOME
        teensy: Serial connection
        state: Telemetry state array
        safety_state: Safety state dict
        system_telem: SystemTelemetry object or None
        enabled_local: Local tracking of enabled state
        send_cmd_fn: Function to send commands
        auto_disable_at: Current auto-disable timestamp (or None)
        auto_disable_reason: Current auto-disable reason (or None)
        auto_disable_gen: Current generation counter
        auto_disable_s: Seconds until auto-disable (0 = no auto-disable)
        require_enable: Whether to auto-enable first
        verbose: Print debug messages
        
    Returns:
        (success: bool, new_enabled_local: bool, last_posture: bytes | None,
         new_auto_disable_at: float | None, new_auto_disable_reason: str | None,
         new_auto_disable_gen: int)
    """
    global _lastPosture
    
    if teensy is None:
        return (False, enabled_local, _lastPosture, auto_disable_at, auto_disable_reason, auto_disable_gen)
    
    # Normalize posture to bytes upper
    if isinstance(name, bytes):
        posture = name.upper()
    elif isinstance(name, str):
        try:
            posture = name.strip().upper().encode('ascii')
        except Exception:
            return (False, enabled_local, _lastPosture, auto_disable_at, auto_disable_reason, auto_disable_gen)
    else:
        return (False, enabled_local, _lastPosture, auto_disable_at, auto_disable_reason, auto_disable_gen)
    
    if posture not in (b'TUCK', b'STAND', b'HOME'):
        if verbose:
            print(f"Unknown posture '{posture}'", end="\r\n")
        return (False, enabled_local, _lastPosture, auto_disable_at, auto_disable_reason, auto_disable_gen)
    
    new_enabled_local = enabled_local
    if require_enable:
        # Check if enabled
        if system_telem is not None and getattr(system_telem, 'valid', False):
            enabled_now = system_telem.robot_enabled
        else:
            enabled_now = (state[IDX_ROBOT_ENABLED] == 1.0) if (len(state) > IDX_ROBOT_ENABLED) else False
        
        if not enabled_now:
            _, new_enabled_local = ensure_enabled(
                teensy, state, safety_state, system_telem,
                enabled_local, send_cmd_fn, verbose
            )
    
    # Prepare new auto-disable state
    new_auto_disable_at = auto_disable_at
    new_auto_disable_reason = auto_disable_reason
    new_auto_disable_gen = auto_disable_gen
    
    if send_cmd_fn(posture, True):
        _lastPosture = posture
        # Bump generation on every successful posture command
        new_auto_disable_gen = auto_disable_gen + 1
        reason = posture.decode('ascii', errors='ignore') if isinstance(posture, (bytes, bytearray)) else str(posture)
        
        if auto_disable_s > 0:
            new_auto_disable_at = time.time() + auto_disable_s
            new_auto_disable_reason = reason
            if verbose:
                print(f"\n{posture} sequence sent. Auto DISABLE in {auto_disable_s:.0f}s (reason={reason}).", end="\r\n")
        else:
            new_auto_disable_at = None
            new_auto_disable_reason = None
        
        return (True, new_enabled_local, _lastPosture, new_auto_disable_at, new_auto_disable_reason, new_auto_disable_gen)
    
    return (False, new_enabled_local, _lastPosture, new_auto_disable_at, new_auto_disable_reason, new_auto_disable_gen)


def start_pounce_move(
    teensy: Any,
    safety_state: dict,
    gait_active: bool,
    gait_engine: Any,
    gait_transition: Any,
    saved_gait_width_mm: float,
    gait_base_y_mm: float,
    pounce_params: dict,
    send_cmd_fn: Callable[[bytes, bool], bool],
    ensure_enabled_fn: Callable[[], bool],
    hide_menu_fn: Callable[[], None] = None,
    verbose: bool = False,
    source: str = ""
) -> tuple:
    """Start the kinematic Pounce move.
    
    Args:
        teensy: Serial connection
        safety_state: Safety state dict
        gait_active: Whether cyclic gait is running
        gait_engine: Current gait engine or None
        gait_transition: GaitTransition instance
        saved_gait_width_mm: Base X dimension
        gait_base_y_mm: Base Y dimension
        pounce_params: Dict with prep_ms, rear_ms, lunge_ms, recover_ms,
                       back1_z_mm, back2_z_mm, push_z_mm, strike_z_mm,
                       crouch_dy_mm, lift_dy_mm, front_z_mm
        send_cmd_fn: Function to send commands
        ensure_enabled_fn: Function to ensure robot is enabled
        hide_menu_fn: Optional function to hide menu
        verbose: Print debug messages
        source: Source string for debug output
        
    Returns:
        (success: bool, new_gait_active: bool, move_engine: Any, move_active: bool,
         clear_auto_disable: bool)
    """
    if safety_state.get("lockout", False):
        if verbose:
            print("Pounce blocked: firmware safety lockout is active.", end="\r\n")
        return (False, gait_active, None, False, False)
    
    if teensy is None:
        return (False, gait_active, None, False, False)
    
    # Cancel any cyclic gait/transition first
    new_gait_active = gait_active
    if gait_active and gait_engine is not None:
        gait_engine.stop()
        new_gait_active = False
    
    try:
        gait_transition.reset()
    except Exception:
        pass
    
    # Ensure Teensy is not running built-in gait
    send_cmd_fn(b'MODE IDLE', True)
    
    # Enable robot if needed
    ensure_enabled_fn()
    
    # Import here to avoid circular dependency
    from gait_engine import GaitParams, PounceAttack
    
    params = GaitParams(base_x_mm=saved_gait_width_mm, base_y_mm=gait_base_y_mm)
    move_engine = PounceAttack(
        params,
        prep_ms=int(pounce_params.get('prep_ms', 400)),
        rear_ms=int(pounce_params.get('rear_ms', 250)),
        lunge_ms=int(pounce_params.get('lunge_ms', 250)),
        recover_ms=int(pounce_params.get('recover_ms', 500)),
        back1_z_mm=float(pounce_params.get('back1_z_mm', 55.0)),
        back2_z_mm=float(pounce_params.get('back2_z_mm', 75.0)),
        push_z_mm=float(pounce_params.get('push_z_mm', -60.0)),
        strike_z_mm=float(pounce_params.get('strike_z_mm', 140.0)),
        crouch_dy_mm=float(pounce_params.get('crouch_dy_mm', 55.0)),
        lift_dy_mm=float(pounce_params.get('lift_dy_mm', 110.0)),
        front_z_mm=float(pounce_params.get('front_z_mm', 20.0)),
    )
    move_engine.start()
    # Caller should clear _autoDisableAt (return flag indicates this)
    
    if hide_menu_fn is not None:
        try:
            hide_menu_fn()
        except Exception:
            pass
    
    if verbose:
        s = f" ({source})" if source else ""
        prep = pounce_params.get('prep_ms', 400)
        rear = pounce_params.get('rear_ms', 250)
        lunge = pounce_params.get('lunge_ms', 250)
        recov = pounce_params.get('recover_ms', 500)
        print(f"Pounce started{s}: prep={prep}ms rear={rear}ms lunge={lunge}ms recov={recov}ms", end="\r\n")
    
    # Last element True signals caller to clear _autoDisableAt
    return (True, new_gait_active, move_engine, True, True)


# -----------------------------------------------------------------------------
# Module exports
# -----------------------------------------------------------------------------
__all__ = [
    'get_last_posture', 'set_last_posture',
    'ensure_enabled',
    'apply_posture',
    'start_pounce_move',
]

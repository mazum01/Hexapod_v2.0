#----------------------------------------------------------------------------------------------------------------------
#    input_handler.py
#----------------------------------------------------------------------------------------------------------------------
# Input handling utilities for the MARS controller.
# Provides keyboard and gamepad detection/polling functions.
#
# Phase 6 of controller.py modularization (2025-06-18).
#----------------------------------------------------------------------------------------------------------------------
"""
Input handling module for MARS hexapod controller.

This module provides:
- Keyboard input via curses (non-blocking)
- Gamepad (Xbox controller) detection
- Teensy serial port discovery and I/O
- Helper functions for input processing

The main poll_gamepad() logic remains in Controller class due to tight coupling
with state, but this module provides the lower-level I/O primitives.
"""

import curses
import time
from typing import Optional, List, Tuple, Callable

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    from evdev import InputDevice, list_devices
except ImportError:
    InputDevice = None
    list_devices = lambda: []

#----------------------------------------------------------------------------------------------------------------------
# Keyboard input (curses-based non-blocking)
#----------------------------------------------------------------------------------------------------------------------

# Persistent curses state for keyboard input (avoids per-loop wrapper overhead)
_stdscr = None


def init_keyboard():
    """Initialize persistent curses for non-blocking keyboard input.
    
    Call this once at startup. Returns the curses stdscr object.
    Safe to call multiple times (idempotent).
    """
    global _stdscr
    if _stdscr is None:
        _stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        _stdscr.nodelay(True)  # non-blocking
        _stdscr.keypad(True)
    return _stdscr


def cleanup_keyboard():
    """Restore terminal settings on exit.
    
    Call this before program termination to restore terminal state.
    Safe to call multiple times (idempotent).
    """
    global _stdscr
    if _stdscr is not None:
        try:
            _stdscr.keypad(False)
            curses.nocbreak()
            curses.echo()
            # Note: endwin() commented out in original to avoid terminal glitches
        except curses.error:
            pass
        _stdscr = None


def poll_keyboard() -> int:
    """Non-blocking keyboard poll using persistent stdscr.
    
    Returns:
        Key code (ord value) if a key is pressed, -1 otherwise.
    """
    global _stdscr
    if _stdscr is None:
        init_keyboard()
    try:
        return _stdscr.getch()
    except curses.error:
        return -1


def getkey(stdscr) -> int:
    """Legacy getkey function (deprecated - use poll_keyboard instead).
    
    Provided for backward compatibility with older code that passes stdscr.
    """
    stdscr.nodelay(True)  # do not wait for input when calling getch
    return stdscr.getch()


#----------------------------------------------------------------------------------------------------------------------
# Gamepad detection (Xbox controller via evdev)
#----------------------------------------------------------------------------------------------------------------------

def find_gamepad(verbose: bool = False) -> Optional['InputDevice']:
    """Search for an Xbox wireless controller.
    
    Args:
        verbose: If True, print debug info about device search.
        
    Returns:
        InputDevice for the Xbox controller if found, None otherwise.
    """
    if InputDevice is None:
        if verbose:
            print("evdev not available - no gamepad support", end="\r\n")
        return None
    
    try:
        devices = [InputDevice(path) for path in list_devices()]
        for device in devices:
            if str.lower(device.name) == 'xbox wireless controller':
                if verbose:
                    print(f"Xbox controller found at: {device.path}", end="\r\n")
                return device
    except Exception as e:
        if verbose:
            print(f"Error searching for gamepad: {e}", end="\r\n")
    return None


# Alias for backward compatibility
testForGamePad = find_gamepad


#----------------------------------------------------------------------------------------------------------------------
# Teensy serial port discovery
#----------------------------------------------------------------------------------------------------------------------

def find_teensy(verbose: bool = False) -> Optional['serial.tools.list_ports.ListPortInfo']:
    """Scan serial ports for a Teensy device.
    
    Args:
        verbose: If True, print debug info about port scanning.
        
    Returns:
        Serial port info object if Teensy found, None otherwise.
    """
    if serial is None:
        if verbose:
            print("pyserial not available - no Teensy support", end="\r\n")
        return None
    
    if verbose:
        print("Searching for Teensy...", end="\r\n")
    
    try:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if verbose:
                print(f"{port.description}, {port.device} - {port.manufacturer} - {port.product}", end="\r\n")
            # Check if the device is a Teensy
            if port.manufacturer and "teensy" in port.manufacturer.lower() and "0" in port.device:
                if verbose:
                    print(f"Teensy found on port: {port.device}", end="\r\n")
                return port
    except Exception as e:
        if verbose:
            print(f"Error scanning for Teensy: {e}", end="\r\n")
    
    if verbose:
        print("No Teensy found.", end="\r\n")
    return None


#----------------------------------------------------------------------------------------------------------------------
# Teensy serial I/O
#----------------------------------------------------------------------------------------------------------------------

# Persistent buffer for incomplete lines across reads
_teensy_read_buffer = ""


def read_teensy(teensy, verbose: bool = False) -> Optional[str]:
    """Read from Teensy with batched I/O and incomplete line buffering.
    
    Uses a global buffer to handle partial lines across calls.
    Returns complete lines only; partial lines are buffered for next call.
    
    Args:
        teensy: Serial port object for the Teensy.
        verbose: If True, print debug info on errors.
        
    Returns:
        String of complete lines (newline-separated) or None if no data.
    """
    global _teensy_read_buffer
    
    if teensy is None:
        if verbose:
            print("No Teensy device found.", end="\r\n")
        return None
    
    try:
        waiting = teensy.in_waiting
        if waiting == 0:
            return None
        
        raw = teensy.read(waiting)
        try:
            chunk = raw.decode('utf-8', errors='ignore')
        except Exception:
            chunk = ""
        
        # Normalize incoming chunk newlines to \n
        if chunk:
            chunk = chunk.replace('\r\n', '\n').replace('\r', '\n')
        
        # Append to persistent buffer
        _teensy_read_buffer += chunk
        
        # Split into lines; last element may be incomplete
        lines = _teensy_read_buffer.split('\n')
        
        # If buffer doesn't end with newline, last element is incomplete - keep it
        if not _teensy_read_buffer.endswith('\n'):
            _teensy_read_buffer = lines[-1]  # Save incomplete line for next read
            lines = lines[:-1]               # Return only complete lines
        else:
            _teensy_read_buffer = ""         # All lines complete
        
        # Join complete lines and return
        if lines:
            return '\n'.join(lines)
        return None
        
    except serial.SerialException as e:
        if verbose:
            print(f"Error reading from Teensy: {e}", end="\r\n")
        return None


def read_teensy_bytes(teensy, verbose: bool = False) -> Optional[bytes]:
    """Read raw bytes from Teensy (no decoding / newline normalization).
    
    Args:
        teensy: Serial port object for the Teensy.
        verbose: If True, print debug info on errors.
        
    Returns:
        Raw bytes or None if no data available.
    """
    if teensy is None:
        if verbose:
            print("No Teensy device found.", end="\r\n")
        return None
    
    try:
        waiting = teensy.in_waiting
        if waiting == 0:
            return None
        return teensy.read(waiting)
    except serial.SerialException as e:
        if verbose:
            print(f"Error reading from Teensy: {e}", end="\r\n")
        return None


def reset_teensy_buffer():
    """Clear the Teensy read buffer.
    
    Call this when reconnecting to clear stale partial data.
    """
    global _teensy_read_buffer
    _teensy_read_buffer = ""


#----------------------------------------------------------------------------------------------------------------------
# Gamepad button/axis code constants (Xbox controller)
#----------------------------------------------------------------------------------------------------------------------

class XboxButton:
    """Xbox controller button event codes."""
    A = 304
    B = 305
    X = 308      # BTN_WEST
    Y = 307      # BTN_NORTH
    LB = 310
    RB = 311
    BACK = 314   # Select/Back
    START = 315
    LEFT_STICK = 317
    RIGHT_STICK = 318
    GUIDE = 172  # Xbox button / power
    GUIDE_ALT1 = 139  # Alternative guide button code
    GUIDE_ALT2 = 158  # Alternative guide button code


class XboxAxis:
    """Xbox controller axis event codes."""
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2  # Note: may also be code 2 on some controllers
    RIGHT_X_ALT = 5  # Alternative right stick X (shares with RT)
    RIGHT_Y = 5
    LEFT_TRIGGER = 10  # LT: 0-1023
    LEFT_TRIGGER_ALT = 2  # Alternative LT code
    RIGHT_TRIGGER = 9  # RT: 0-1023
    DPAD_X = 16  # Left=-1, Right=+1
    DPAD_Y = 17  # Up=-1, Down=+1


# Joystick normalization constants
JOYSTICK_CENTER = 32768
JOYSTICK_RANGE = 32768.0
TRIGGER_MAX = 1023
JOYSTICK_MIN_FOR_STICK = 1024  # Values > this indicate joystick, not trigger


def normalize_joystick(value: int, deadzone: float = 0.1) -> float:
    """Normalize joystick axis value to -1.0 to +1.0 with deadzone.
    
    Args:
        value: Raw joystick value (0-65535, center at 32768).
        deadzone: Deadzone as fraction of full range (default 10%).
        
    Returns:
        Normalized value from -1.0 to +1.0, with deadzone applied.
    """
    normalized = (value - JOYSTICK_CENTER) / JOYSTICK_RANGE
    if abs(normalized) < deadzone:
        return 0.0
    return normalized


def normalize_trigger(value: int) -> float:
    """Normalize trigger axis value to 0.0 to 1.0.
    
    Args:
        value: Raw trigger value (0-1023).
        
    Returns:
        Normalized value from 0.0 to 1.0.
    """
    return min(1.0, max(0.0, value / TRIGGER_MAX))


#----------------------------------------------------------------------------------------------------------------------
# Touch input helpers
#----------------------------------------------------------------------------------------------------------------------

def poll_touch(touch) -> Optional[Tuple[int, int, bool]]:
    """Poll touchscreen for input.
    
    Args:
        touch: Touchscreen device object (e.g., FT6336U).
        
    Returns:
        Tuple of (x, y, touched) or None if touch device unavailable.
    """
    if touch is None:
        return None
    
    try:
        touched = touch.touched
        if touched:
            x = touch.x
            y = touch.y
            return (x, y, True)
        return (0, 0, False)
    except Exception:
        return None


#----------------------------------------------------------------------------------------------------------------------
# Module exports
#----------------------------------------------------------------------------------------------------------------------

__all__ = [
    # Keyboard
    'init_keyboard',
    'cleanup_keyboard', 
    'poll_keyboard',
    'getkey',
    # Gamepad
    'find_gamepad',
    'testForGamePad',
    'XboxButton',
    'XboxAxis',
    'normalize_joystick',
    'normalize_trigger',
    # Teensy
    'find_teensy',
    'read_teensy',
    'read_teensy_bytes',
    'reset_teensy_buffer',
    # Touch
    'poll_touch',
    # Constants
    'JOYSTICK_CENTER',
    'JOYSTICK_RANGE',
    'TRIGGER_MAX',
    'JOYSTICK_MIN_FOR_STICK',
]

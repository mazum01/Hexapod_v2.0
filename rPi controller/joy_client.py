#!/usr/bin/env python3
#----------------------------------------------------------------------------------------------------------------------
#    joy_client.py
#----------------------------------------------------------------------------------------------------------------------
# Socket client for receiving Xbox controller state from joy_controller daemon.
#
# Version: 0.7.1 b180 (2025-12-22)
#----------------------------------------------------------------------------------------------------------------------
"""
Joy controller socket client for MARS hexapod.

This module provides:
- Unix socket client to connect to joy_controller daemon
- Non-blocking reception of joystick/button state
- State parsing and conversion to Controller-compatible format

Usage:
    from joy_client import JoyClient
    
    client = JoyClient()
    if client.connect():
        while True:
            state = client.poll()
            if state:
                # Process joystick/button state
                pass
"""

import os
import json
import socket
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List

#----------------------------------------------------------------------------------------------------------------------
# Configuration
#----------------------------------------------------------------------------------------------------------------------

SOCKET_PATH = "/tmp/mars_joy.sock"
RECONNECT_INTERVAL = 2.0  # seconds between reconnection attempts

#----------------------------------------------------------------------------------------------------------------------
# Joystick State Dataclass
#----------------------------------------------------------------------------------------------------------------------

@dataclass
class JoyState:
    """Current state of the Xbox controller received from joy_controller."""
    
    # Connection status - xbox_connected is whether the Xbox controller is connected to the daemon
    xbox_connected: bool = False
    timestamp: float = 0.0
    
    # Axes (normalized: -1.0 to 1.0, or 0.0 to 1.0 for triggers)
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    trigger_left: float = 0.0
    trigger_right: float = 0.0
    
    # Buttons (True = pressed)
    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    button_lb: bool = False
    button_rb: bool = False
    button_back: bool = False
    button_start: bool = False
    button_lstick: bool = False
    button_rstick: bool = False
    
    # D-pad
    dpad_up: bool = False
    dpad_down: bool = False
    dpad_left: bool = False
    dpad_right: bool = False
    
    # Track previous state for edge detection
    _prev_buttons: Dict[str, bool] = field(default_factory=dict)
    
    def update_from_dict(self, data: Dict[str, Any]):
        """Update state from dictionary received via socket."""
        if data.get("type") != "state":
            return
        
        self.xbox_connected = data.get("connected", False)
        self.timestamp = data.get("ts", time.time())
        
        # Axes
        axes = data.get("axes", {})
        self.left_x = axes.get("lx", 0.0)
        self.left_y = axes.get("ly", 0.0)
        self.right_x = axes.get("rx", 0.0)
        self.right_y = axes.get("ry", 0.0)
        self.trigger_left = axes.get("lt", 0.0)
        self.trigger_right = axes.get("rt", 0.0)
        
        # Save previous button state for edge detection
        self._prev_buttons = {
            "a": self.button_a, "b": self.button_b,
            "x": self.button_x, "y": self.button_y,
            "lb": self.button_lb, "rb": self.button_rb,
            "back": self.button_back, "start": self.button_start,
            "lstick": self.button_lstick, "rstick": self.button_rstick,
        }
        
        # Buttons
        buttons = data.get("buttons", {})
        self.button_a = buttons.get("a", False)
        self.button_b = buttons.get("b", False)
        self.button_x = buttons.get("x", False)
        self.button_y = buttons.get("y", False)
        self.button_lb = buttons.get("lb", False)
        self.button_rb = buttons.get("rb", False)
        self.button_back = buttons.get("back", False)
        self.button_start = buttons.get("start", False)
        self.button_lstick = buttons.get("lstick", False)
        self.button_rstick = buttons.get("rstick", False)
        
        # Save previous dpad state
        self._prev_buttons.update({
            "dpad_up": self.dpad_up, "dpad_down": self.dpad_down,
            "dpad_left": self.dpad_left, "dpad_right": self.dpad_right,
        })
        
        # D-pad
        dpad = data.get("dpad", {})
        self.dpad_up = dpad.get("up", False)
        self.dpad_down = dpad.get("down", False)
        self.dpad_left = dpad.get("left", False)
        self.dpad_right = dpad.get("right", False)
    
    def button_pressed(self, button: str) -> bool:
        """Check if a button was just pressed (rising edge).
        
        Args:
            button: Button name (a, b, x, y, lb, rb, back, start, lstick, rstick,
                    dpad_up, dpad_down, dpad_left, dpad_right)
        
        Returns:
            True if button transitioned from not-pressed to pressed.
        """
        current = getattr(self, f"button_{button}" if not button.startswith("dpad_") else button, False)
        prev = self._prev_buttons.get(button, False)
        return current and not prev
    
    def button_released(self, button: str) -> bool:
        """Check if a button was just released (falling edge)."""
        current = getattr(self, f"button_{button}" if not button.startswith("dpad_") else button, False)
        prev = self._prev_buttons.get(button, False)
        return not current and prev


#----------------------------------------------------------------------------------------------------------------------
# Joy Client
#----------------------------------------------------------------------------------------------------------------------

class JoyClient:
    """Unix socket client for receiving Xbox controller state."""
    
    def __init__(self, socket_path: str = SOCKET_PATH, verbose: bool = False):
        self.socket_path = socket_path
        self.verbose = verbose
        self.sock: Optional[socket.socket] = None
        self.state = JoyState()
        self._buffer = ""
        self._last_connect_attempt = 0.0
        self._reconnect_interval = RECONNECT_INTERVAL
    
    @property
    def connected(self) -> bool:
        """Check if client is connected to joy_controller socket."""
        return self.sock is not None
    
    @property
    def xbox_connected(self) -> bool:
        """Check if Xbox controller is connected to joy_controller daemon."""
        return self.connected and self.state.xbox_connected
    
    def connect(self) -> bool:
        """Connect to joy_controller socket server.
        
        Returns:
            True if connected successfully.
        """
        if self.sock is not None:
            return True
        
        now = time.time()
        if now - self._last_connect_attempt < self._reconnect_interval:
            return False
        
        self._last_connect_attempt = now
        
        if not os.path.exists(self.socket_path):
            if self.verbose:
                print(f"[JOY_CLIENT] Socket not found: {self.socket_path}", end="\r\n")
            return False
        
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.setblocking(False)
            self.sock.connect(self.socket_path)
            self.state.connected = True
            
            if self.verbose:
                print(f"[JOY_CLIENT] Connected to {self.socket_path}", end="\r\n")
            return True
        
        except BlockingIOError:
            # Connection in progress (non-blocking)
            self.state.connected = True
            return True
        except Exception as e:
            if self.verbose:
                print(f"[JOY_CLIENT] Connection failed: {e}", end="\r\n")
            self.sock = None
            self.state.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from joy_controller."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.state.connected = False
        self._buffer = ""
    
    def poll(self) -> Optional[JoyState]:
        """Poll for new state from joy_controller.
        
        Non-blocking. Attempts reconnection if disconnected.
        
        Returns:
            JoyState if new data received, None otherwise.
        """
        # Try to connect if not connected
        if not self.connected:
            if not self.connect():
                return None
        
        # Read available data
        try:
            data = self.sock.recv(4096)
            if not data:
                # Connection closed
                if self.verbose:
                    print("[JOY_CLIENT] Connection closed by server", end="\r\n")
                self.disconnect()
                return None
            
            self._buffer += data.decode("utf-8")
        
        except BlockingIOError:
            # No data available
            pass
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            if self.verbose:
                print(f"[JOY_CLIENT] Connection error: {e}", end="\r\n")
            self.disconnect()
            return None
        
        # Process complete lines in buffer
        updated = False
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            if line.strip():
                try:
                    msg = json.loads(line)
                    if msg.get("type") == "state":
                        self.state.update_from_dict(msg)
                        updated = True
                except json.JSONDecodeError:
                    if self.verbose:
                        print(f"[JOY_CLIENT] Invalid JSON: {line[:50]}", end="\r\n")
        
        return self.state if updated else None
    
    def get_state(self) -> JoyState:
        """Get the current joystick state (whether or not new data was received)."""
        return self.state


#----------------------------------------------------------------------------------------------------------------------
# Module exports
#----------------------------------------------------------------------------------------------------------------------

__all__ = [
    'JoyClient',
    'JoyState',
    'SOCKET_PATH',
]

#!/usr/bin/env python3
#----------------------------------------------------------------------------------------------------------------------
#    joy_controller.py
#----------------------------------------------------------------------------------------------------------------------
# Standalone Xbox controller daemon for MARS hexapod.
# Runs as a systemd service at boot, communicates with main controller.py via Unix socket.
#
# Architecture:
#   - Polls Xbox controller via evdev (non-blocking)
#   - Manages main controller.py subprocess (start/stop via power button)
#   - Provides Unix domain socket server for IPC
#   - Sends joystick/button state as JSON-lines to connected client
#   - Provides haptic feedback on controller start/stop
#
# Version: 0.7.5 b184 (2025-12-23) - Fixed button codes: X=307, Y=308; Back=158 (not power); removed 158 from power codes
#----------------------------------------------------------------------------------------------------------------------
"""
Xbox controller daemon for MARS hexapod.

This module provides:
- Xbox controller polling via evdev
- Unix domain socket server for IPC with main controller
- Subprocess management for controller.py (power button start/stop)
- Haptic feedback on controller start/stop events

Usage:
    python3 joy_controller.py [--verbose]

Or run as a systemd service (mars-joy.service).
"""

import os
import sys
import json
import time
import socket
import signal
import subprocess
import threading
import argparse
from pathlib import Path
from typing import Optional, Dict, Any

try:
    from evdev import InputDevice, ff, ecodes, list_devices
except ImportError:
    print("Error: evdev not installed. Run: pip install evdev", file=sys.stderr)
    sys.exit(1)

#----------------------------------------------------------------------------------------------------------------------
# Configuration
#----------------------------------------------------------------------------------------------------------------------

# Version info
JOY_VERSION = "0.7.5"
JOY_BUILD = 184

# Paths
SCRIPT_DIR = Path(__file__).parent.resolve()
CONTROLLER_SCRIPT = SCRIPT_DIR / "main.py"  # Launch the new entry point wrapper
SOCKET_PATH = "/tmp/mars_joy.sock"

# Xbox button codes (evdev)
POWER_BUTTON_CODES = [172, 139]  # Xbox Guide button variants (removed 158 - it's Back on some controllers)

# Joystick normalization
JOYSTICK_CENTER = 32768
JOYSTICK_MAX = 32768
JOYSTICK_DEADZONE = 0.10  # 10%
TRIGGER_MAX = 1023

#----------------------------------------------------------------------------------------------------------------------
# Joystick State
#----------------------------------------------------------------------------------------------------------------------

class JoystickState:
    """Current state of the Xbox controller."""
    
    def __init__(self):
        self.connected = False
        
        # Axes (normalized -1.0 to 1.0, or 0.0 to 1.0 for triggers)
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.trigger_left = 0.0
        self.trigger_right = 0.0
        
        # Buttons (True = pressed)
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.button_lb = False
        self.button_rb = False
        self.button_back = False
        self.button_start = False
        self.button_lstick = False
        self.button_rstick = False
        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False
        
        # Timestamp of last update
        self.timestamp = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert state to dictionary for JSON serialization."""
        return {
            "type": "state",
            "ts": self.timestamp,
            "connected": self.connected,
            "axes": {
                "lx": round(self.left_x, 3),
                "ly": round(self.left_y, 3),
                "rx": round(self.right_x, 3),
                "ry": round(self.right_y, 3),
                "lt": round(self.trigger_left, 3),
                "rt": round(self.trigger_right, 3),
            },
            "buttons": {
                "a": self.button_a,
                "b": self.button_b,
                "x": self.button_x,
                "y": self.button_y,
                "lb": self.button_lb,
                "rb": self.button_rb,
                "back": self.button_back,
                "start": self.button_start,
                "lstick": self.button_lstick,
                "rstick": self.button_rstick,
            },
            "dpad": {
                "up": self.dpad_up,
                "down": self.dpad_down,
                "left": self.dpad_left,
                "right": self.dpad_right,
            },
        }
    
    def to_json(self) -> str:
        """Serialize state to JSON line."""
        return json.dumps(self.to_dict())


#----------------------------------------------------------------------------------------------------------------------
# Xbox Controller Handler
#----------------------------------------------------------------------------------------------------------------------

class XboxController:
    """Handles Xbox controller input via evdev."""
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.device: Optional[InputDevice] = None
        self.state = JoystickState()
        
        # Rumble effect IDs
        self._effect_light_id: Optional[int] = None
        self._effect_strong_id: Optional[int] = None
    
    def find_controller(self) -> bool:
        """Find and connect to Xbox controller. Returns True if found."""
        try:
            for path in list_devices():
                try:
                    dev = InputDevice(path)
                    name_lower = dev.name.lower()
                    if "xbox" in name_lower or "gamepad" in name_lower or "controller" in name_lower:
                        self.device = dev
                        self.device.grab()  # Exclusive access
                        self.state.connected = True
                        self._load_rumble_effects()
                        if self.verbose:
                            print(f"[JOY] Connected to: {dev.name} ({path})")
                        return True
                except (OSError, PermissionError):
                    continue
        except Exception as e:
            if self.verbose:
                print(f"[JOY] Error finding controller: {e}")
        
        self.state.connected = False
        return False
    
    def disconnect(self):
        """Release the controller."""
        if self.device:
            try:
                self._erase_rumble_effects()
                self.device.ungrab()
                self.device.close()
            except Exception:
                pass
            self.device = None
        self.state.connected = False
    
    def _load_rumble_effects(self):
        """Upload rumble effects to the controller."""
        if not self.device:
            return
        
        try:
            # Light rumble (200ms, weak motor)
            rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0x4000)
            effect = ff.Effect(
                ecodes.FF_RUMBLE, -1, 0,
                ff.Trigger(0, 0),
                ff.Replay(200, 0),
                ff.EffectType(ff_rumble_effect=rumble)
            )
            self._effect_light_id = self.device.upload_effect(effect)
            
            # Strong rumble (300ms, strong motor)
            rumble = ff.Rumble(strong_magnitude=0x8000, weak_magnitude=0x0000)
            effect = ff.Effect(
                ecodes.FF_RUMBLE, -1, 0,
                ff.Trigger(0, 0),
                ff.Replay(300, 0),
                ff.EffectType(ff_rumble_effect=rumble)
            )
            self._effect_strong_id = self.device.upload_effect(effect)
            
            if self.verbose:
                print(f"[JOY] Rumble effects loaded (light={self._effect_light_id}, strong={self._effect_strong_id})")
        
        except Exception as e:
            if self.verbose:
                print(f"[JOY] Failed to load rumble effects: {e}")
            self._effect_light_id = None
            self._effect_strong_id = None
    
    def _erase_rumble_effects(self):
        """Remove uploaded rumble effects."""
        if not self.device:
            return
        try:
            if self._effect_light_id is not None:
                self.device.erase_effect(self._effect_light_id)
            if self._effect_strong_id is not None:
                self.device.erase_effect(self._effect_strong_id)
        except Exception:
            pass
    
    def rumble_light(self):
        """Trigger a light rumble (controller start feedback)."""
        if self.device and self._effect_light_id is not None:
            try:
                self.device.write(ecodes.EV_FF, self._effect_light_id, 1)
                if self.verbose:
                    print(f"[JOY] Rumble light triggered (effect_id={self._effect_light_id})")
            except Exception as e:
                if self.verbose:
                    print(f"[JOY] Rumble light failed: {e}")
    
    def rumble_strong(self):
        """Trigger a strong rumble (controller stop feedback)."""
        if self.device and self._effect_strong_id is not None:
            try:
                self.device.write(ecodes.EV_FF, self._effect_strong_id, 1)
                if self.verbose:
                    print(f"[JOY] Rumble strong triggered (effect_id={self._effect_strong_id})")
            except Exception as e:
                if self.verbose:
                    print(f"[JOY] Rumble strong failed: {e}")
                pass
    
    def poll(self) -> bool:
        """Poll for events, update state. Returns True if power button pressed."""
        if not self.device:
            return False
        
        power_pressed = False
        
        try:
            while True:
                event = self.device.read_one()
                if event is None:
                    break
                
                self.state.timestamp = time.time()
                
                # Analog axes (type 3)
                if event.type == 3:
                    if event.code == 0:  # Left stick X
                        self.state.left_x = self._normalize_axis(event.value)
                    elif event.code == 1:  # Left stick Y (inverted)
                        self.state.left_y = -self._normalize_axis(event.value)
                    elif event.code == 2:  # Left trigger (some controllers) or Right stick X
                        if event.value <= TRIGGER_MAX:
                            self.state.trigger_left = event.value / TRIGGER_MAX
                        else:
                            self.state.right_x = self._normalize_axis(event.value)
                    elif event.code == 3:  # Right stick X
                        self.state.right_x = self._normalize_axis(event.value)
                    elif event.code == 4:  # Right stick Y (inverted)
                        self.state.right_y = -self._normalize_axis(event.value)
                    elif event.code == 5:  # Right trigger (some controllers) or Right stick Y
                        if event.value <= TRIGGER_MAX:
                            self.state.trigger_right = event.value / TRIGGER_MAX
                        else:
                            self.state.right_y = -self._normalize_axis(event.value)
                    elif event.code == 9:  # Right trigger (RT)
                        self.state.trigger_right = event.value / TRIGGER_MAX
                    elif event.code == 10:  # Left trigger (LT)
                        self.state.trigger_left = event.value / TRIGGER_MAX
                    elif event.code == 16:  # D-pad X
                        self.state.dpad_left = (event.value == -1)
                        self.state.dpad_right = (event.value == 1)
                    elif event.code == 17:  # D-pad Y
                        self.state.dpad_up = (event.value == -1)
                        self.state.dpad_down = (event.value == 1)
                
                # Buttons (type 1)
                elif event.type == 1:
                    pressed = (event.value == 1)
                    
                    if event.code == 304:  # A (BTN_SOUTH)
                        self.state.button_a = pressed
                        if self.verbose and pressed:
                            print(f"[JOY] Button A pressed")
                    elif event.code == 305:  # B (BTN_EAST)
                        self.state.button_b = pressed
                        if self.verbose and pressed:
                            print(f"[JOY] Button B pressed")
                    elif event.code == 307:  # X (blue, left button)
                        self.state.button_x = pressed
                        if self.verbose and pressed:
                            print(f"[JOY] Button X pressed")
                    elif event.code == 308:  # Y (yellow, top button)
                        self.state.button_y = pressed
                        if self.verbose and pressed:
                            print(f"[JOY] Button Y pressed")
                    elif event.code == 310:  # LB
                        self.state.button_lb = pressed
                    elif event.code == 311:  # RB
                        self.state.button_rb = pressed
                    elif event.code == 158:  # Back/View
                        self.state.button_back = pressed
                    elif event.code == 315:  # Start/Menu
                        self.state.button_start = pressed
                    elif event.code == 317:  # Left stick click
                        self.state.button_lstick = pressed
                    elif event.code == 318:  # Right stick click
                        self.state.button_rstick = pressed
                    elif event.code in POWER_BUTTON_CODES and pressed:
                        # Power button pressed
                        power_pressed = True
                        if self.verbose:
                            print(f"[JOY] Power button pressed (code {event.code})")
        
        except BlockingIOError:
            pass
        except OSError as e:
            if self.verbose:
                print(f"[JOY] Controller disconnected: {e}")
            self.state.connected = False
            self.device = None
        
        return power_pressed
    
    def _normalize_axis(self, value: int) -> float:
        """Normalize axis value to -1.0 to 1.0 with deadzone."""
        normalized = (value - JOYSTICK_CENTER) / JOYSTICK_MAX
        if abs(normalized) < JOYSTICK_DEADZONE:
            return 0.0
        return max(-1.0, min(1.0, normalized))


#----------------------------------------------------------------------------------------------------------------------
# Controller Process Manager
#----------------------------------------------------------------------------------------------------------------------

def find_controller_pids(script_name: str = "main.py") -> list:
    """Find PIDs of any running main.py (controller wrapper) processes.
    
    Uses a pattern that matches the script name preceded by / or whitespace.
    
    Returns:
        List of integer PIDs.
    """
    pids = []
    my_pid = os.getpid()
    try:
        # Pattern: / or whitespace followed by script_name (e.g. main.py)
        # We also look for the legacy controller.py just in case
        pattern = f"(^|/|\\s)({script_name}|controller.py)($|\\s)"
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    try:
                        pid = int(line.strip())
                        # Don't include our own PID or parent PID
                        if pid != my_pid and pid != os.getppid():
                            pids.append(pid)
                    except ValueError:
                        pass
    except Exception:
        pass
    return pids


def kill_existing_controllers(script_name: str = "main.py", verbose: bool = False) -> int:
    """Kill any existing MARS controller processes.
    
    Returns:
        Number of processes killed.
    """
    pids = find_controller_pids(script_name)
    killed = 0
    for pid in pids:
        try:
            if verbose:
                print(f"[JOY] Killing orphaned controller (PID {pid})")
            os.kill(pid, signal.SIGTERM)
            killed += 1
        except ProcessLookupError:
            pass  # Already dead
        except PermissionError:
            if verbose:
                print(f"[JOY] No permission to kill PID {pid}")
    return killed


class ControllerManager:
    """Manages the main controller.py subprocess."""
    
    def __init__(self, script_path: Path, verbose: bool = False):
        self.script_path = script_path
        self.verbose = verbose
        self.process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        
        # On startup, kill any orphaned controller.py processes
        killed = kill_existing_controllers(verbose=verbose)
        if killed > 0 and verbose:
            print(f"[JOY] Killed {killed} orphaned controller process(es)")
    
    @property
    def is_running(self) -> bool:
        """Check if the controller process is currently running."""
        with self._lock:
            if self.process is None:
                # Also check for externally-started controllers
                pids = find_controller_pids()
                return len(pids) > 0
            poll = self.process.poll()
            if poll is not None:
                # Process has terminated
                self.process = None
                return False
            return True
    
    def start(self) -> bool:
        """Start the main controller. Returns True if started successfully."""
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                if self.verbose:
                    print("[JOY] Controller already running")
                return False
            
            try:
                if self.verbose:
                    print(f"[JOY] Starting controller: {self.script_path}")
                
                # Log file for controller output (for debugging)
                log_path = self.script_path.parent / "controller_output.log"
                log_file = open(log_path, 'a')
                log_file.write(f"\n=== Controller started at {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
                log_file.flush()
                
                # Start controller.py with Python, logging stdout/stderr
                self.process = subprocess.Popen(
                    [sys.executable, str(self.script_path)],
                    cwd=str(self.script_path.parent),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,  # Detach from this process group
                )
                
                if self.verbose:
                    print(f"[JOY] Controller started (PID {self.process.pid})")
                    print(f"[JOY] Output logged to: {log_path}")
                return True
            
            except Exception as e:
                if self.verbose:
                    print(f"[JOY] Failed to start controller: {e}")
                self.process = None
                return False
    
    def stop(self) -> bool:
        """Stop the main controller gracefully. Returns True if stopped."""
        with self._lock:
            stopped = False
            
            # First, stop our tracked process if any
            if self.process is not None and self.process.poll() is None:
                try:
                    if self.verbose:
                        print(f"[JOY] Stopping controller (PID {self.process.pid})")
                    
                    # Send SIGTERM for graceful shutdown (controller has handler)
                    self.process.terminate()
                    if self.verbose:
                        print("[JOY] Sent SIGTERM, waiting for graceful shutdown...")
                    
                    # Wait up to 10 seconds for clean exit (controller cleanup may take time)
                    try:
                        self.process.wait(timeout=10.0)
                    except subprocess.TimeoutExpired:
                        if self.verbose:
                            print("[JOY] Controller not responding after 10s, sending SIGKILL")
                        self.process.kill()
                        self.process.wait(timeout=2.0)
                    
                    if self.verbose:
                        print("[JOY] Controller stopped")
                    stopped = True
                
                except Exception as e:
                    if self.verbose:
                        print(f"[JOY] Error stopping controller: {e}")
                
                self.process = None
            
            # Also kill any externally-started controller.py processes
            killed = kill_existing_controllers(verbose=self.verbose)
            if killed > 0:
                stopped = True
                if self.verbose:
                    print(f"[JOY] Killed {killed} external controller process(es)")
            
            return stopped
    
    def toggle(self) -> bool:
        """Toggle controller state. Returns True if now running."""
        if self.is_running:
            self.stop()
            return False
        else:
            self.start()
            return True


#----------------------------------------------------------------------------------------------------------------------
# Socket Server
#----------------------------------------------------------------------------------------------------------------------

class JoySocketServer:
    """Unix domain socket server for IPC with main controller."""
    
    def __init__(self, socket_path: str, verbose: bool = False):
        self.socket_path = socket_path
        self.verbose = verbose
        self.server: Optional[socket.socket] = None
        self.client: Optional[socket.socket] = None
        self._client_lock = threading.Lock()
        self._running = False
        self._accept_thread: Optional[threading.Thread] = None
    
    def start(self):
        """Start the socket server."""
        # Remove stale socket file
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(self.socket_path)
        self.server.listen(1)
        self.server.setblocking(False)
        
        # Set permissions so controller.py can connect
        os.chmod(self.socket_path, 0o666)
        
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        
        if self.verbose:
            print(f"[JOY] Socket server started: {self.socket_path}")
    
    def stop(self):
        """Stop the socket server."""
        self._running = False
        
        with self._client_lock:
            if self.client:
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None
        
        if self.server:
            try:
                self.server.close()
            except Exception:
                pass
            self.server = None
        
        if os.path.exists(self.socket_path):
            try:
                os.unlink(self.socket_path)
            except Exception:
                pass
        
        if self.verbose:
            print("[JOY] Socket server stopped")
    
    def _accept_loop(self):
        """Background thread to accept incoming connections."""
        while self._running and self.server:
            try:
                conn, _ = self.server.accept()
                conn.setblocking(False)
                
                with self._client_lock:
                    # Close old client if any
                    if self.client:
                        try:
                            self.client.close()
                        except Exception:
                            pass
                    self.client = conn
                
                if self.verbose:
                    print("[JOY] Client connected")
            
            except BlockingIOError:
                time.sleep(0.1)
            except Exception as e:
                if self._running and self.verbose:
                    print(f"[JOY] Accept error: {e}")
                time.sleep(0.5)
    
    def send_state(self, state: JoystickState):
        """Send joystick state to connected client."""
        with self._client_lock:
            if not self.client:
                return
            
            try:
                data = state.to_json() + "\n"
                self.client.sendall(data.encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError, BlockingIOError):
                if self.verbose:
                    print("[JOY] Client disconnected")
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None
            except Exception as e:
                if self.verbose:
                    print(f"[JOY] Send error: {e}")
    
    def send_event(self, event_type: str, **kwargs):
        """Send an event message to connected client."""
        with self._client_lock:
            if not self.client:
                return
            
            try:
                msg = {"type": event_type, "ts": time.time(), **kwargs}
                data = json.dumps(msg) + "\n"
                self.client.sendall(data.encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError, BlockingIOError):
                if self.verbose:
                    print("[JOY] Client disconnected")
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None
            except Exception:
                pass
    
    @property
    def has_client(self) -> bool:
        """Check if a client is connected."""
        with self._client_lock:
            return self.client is not None


#----------------------------------------------------------------------------------------------------------------------
# Main Daemon Loop
#----------------------------------------------------------------------------------------------------------------------

class JoyDaemon:
    """Main daemon coordinating Xbox input, socket server, and controller process."""
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.running = False
        
        self.xbox = XboxController(verbose=verbose)
        self.manager = ControllerManager(CONTROLLER_SCRIPT, verbose=verbose)
        self.server = JoySocketServer(SOCKET_PATH, verbose=verbose)
        
        # Reconnection timing
        self._last_reconnect_attempt = 0.0
        self._reconnect_interval = 2.0  # seconds
        
        # State send rate limiting
        self._last_state_send = 0.0
        self._state_send_interval = 0.016  # ~60 Hz
    
    def start(self):
        """Start the daemon."""
        self.running = True
        
        # Install signal handlers
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Start socket server
        self.server.start()
        
        # Initial controller search
        self.xbox.find_controller()
        
        print(f"[JOY] MARS Joy Controller v{JOY_VERSION} b{JOY_BUILD} started")
        print(f"[JOY] Socket: {SOCKET_PATH}")
        print(f"[JOY] Controller script: {CONTROLLER_SCRIPT}")
        print(f"[JOY] Xbox connected: {self.xbox.state.connected}")
        print("[JOY] Press power button to start/stop main controller")
    
    def stop(self):
        """Stop the daemon gracefully."""
        print("\n[JOY] Shutting down...")
        self.running = False
        
        # Stop main controller if running
        if self.manager.is_running:
            self.manager.stop()
        
        # Disconnect Xbox controller
        self.xbox.disconnect()
        
        # Stop socket server
        self.server.stop()
        
        print("[JOY] Shutdown complete")
    
    def _signal_handler(self, signum, frame):
        """Handle SIGTERM/SIGINT for graceful shutdown."""
        self.running = False
    
    def run(self):
        """Main daemon loop."""
        self.start()
        
        try:
            while self.running:
                now = time.time()
                
                # Check Xbox connection
                if not self.xbox.state.connected:
                    if now - self._last_reconnect_attempt >= self._reconnect_interval:
                        self._last_reconnect_attempt = now
                        if self.xbox.find_controller():
                            # Provide feedback on reconnection
                            self.xbox.rumble_light()
                
                # Poll Xbox events
                if self.xbox.state.connected:
                    power_pressed = self.xbox.poll()
                    
                    if power_pressed:
                        # Toggle main controller
                        was_running = self.manager.is_running
                        now_running = self.manager.toggle()
                        
                        # Haptic feedback
                        if now_running and not was_running:
                            # Started: light double-tap
                            self.xbox.rumble_light()
                            time.sleep(0.15)
                            self.xbox.rumble_light()
                        elif was_running and not now_running:
                            # Stopped: single strong rumble
                            self.xbox.rumble_strong()
                
                # Send state to client at rate-limited interval
                # IMPORTANT: Send even when disconnected so client knows controller is gone
                if self.server.has_client and now - self._last_state_send >= self._state_send_interval:
                    self._last_state_send = now
                    self.server.send_state(self.xbox.state)
                
                # Small sleep to prevent busy-waiting
                time.sleep(0.005)  # 5ms = 200 Hz max poll rate
        
        except Exception as e:
            print(f"[JOY] Fatal error: {e}")
        
        finally:
            self.stop()


#----------------------------------------------------------------------------------------------------------------------
# Entry Point
#----------------------------------------------------------------------------------------------------------------------

def main():
    """Entry point for joy_controller daemon."""
    parser = argparse.ArgumentParser(description="MARS Xbox Controller Daemon")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    args = parser.parse_args()
    
    daemon = JoyDaemon(verbose=args.verbose)
    daemon.run()


if __name__ == "__main__":
    main()

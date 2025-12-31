#!/usr/bin/env python3
"""
Telemetry WebSocket Server for MARS Hexapod Dashboard

Streams real-time telemetry data to a web dashboard. Runs alongside the
PointCloud server (different port) to provide robot state, configuration,
and diagnostic information.

Architecture:
    Controller.py (telemetry data)
         ↓ push_telemetry()
    TelemetryServer
         ↓ WebSocket
    dashboard.html (browser)

Protocol (JSON over WebSocket):
    Server → Client:
        {"type": "telemetry", "data": {...}, "timestamp": ms}
        {"type": "config", "data": {...}}

    Client → Server:
        {"cmd": "get_config"}                   - Request all config
        {"cmd": "get_telemetry"}                - Request current telemetry
        {"cmd": "set_config", "section": "...", "key": "...", "value": ...}

Author: MARS Hexapod Project
Version: 0.1.0
"""

import asyncio
import json
import os
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Any, Callable
import http.server
import socketserver

# Try to import websockets
try:
    import websockets
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("Warning: websockets not installed. Run: pip install websockets")


# =============================================================================
# HTTP Handler for Dashboard
# =============================================================================

class DashboardHTTPHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler that serves the dashboard and static files."""

    def __init__(self, *args, static_dir: str = None, **kwargs):
        self.static_dir = static_dir or os.path.dirname(__file__)
        super().__init__(*args, directory=self.static_dir, **kwargs)

    def translate_path(self, path: str) -> str:
        """Map URL paths to filesystem paths."""
        # Root path serves the dashboard
        if path == '/' or path == '':
            return os.path.join(self.static_dir, 'static', 'dashboard.html')
        if path == '/dashboard' or path == '/dashboard.html':
            return os.path.join(self.static_dir, 'static', 'dashboard.html')
        if path == '/pointcloud' or path == '/pointcloud.html':
            return os.path.join(self.static_dir, 'static', 'pointcloud_viewer.html')
        # Other paths resolve normally under static/
        return super().translate_path(path)

    def log_message(self, format, *args):
        """Suppress HTTP access logs to reduce noise."""
        pass


def create_dashboard_http_handler(static_dir: str):
    """Factory function to create handler with custom static directory."""
    class CustomHandler(DashboardHTTPHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, static_dir=static_dir, **kwargs)
    return CustomHandler


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class TelemetryServerConfig:
    """Configuration for telemetry server."""

    # Server settings
    host: str = "0.0.0.0"
    ws_port: int = 8766                # WebSocket port (8765 is point cloud)
    http_port: int = 8080              # HTTP port (shared with point cloud)

    # Update rates
    telemetry_rate_hz: float = 10.0    # Target telemetry push rate


# =============================================================================
# Telemetry Data Structure
# =============================================================================

@dataclass
class TelemetryData:
    """Container for all telemetry data to stream."""

    # System
    timestamp_ms: float = 0.0
    loop_time_us: float = 0.0

    # Power
    battery_v: float = 0.0
    current_a: float = 0.0

    # IMU
    imu_pitch: float = 0.0
    imu_roll: float = 0.0
    imu_yaw: float = 0.0

    # Safety
    robot_enabled: bool = False
    safety_state: str = "---"
    safety_cause: str = "---"
    low_battery_active: bool = False

    # Gait
    gait_running: bool = False
    gait_type: str = "TRIPOD"
    gait_speed: float = 0.0

    # Servos (aggregated)
    servo_temp_max: float = 0.0
    servo_volt_min: float = 0.0
    servo_errors: int = 0

    # Controller version
    ctrl_version: str = ""
    fw_version: str = ""

    # Connection status
    xbox_connected: bool = False
    teensy_connected: bool = False

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON."""
        return {
            "timestamp_ms": self.timestamp_ms,
            "loop_time_us": self.loop_time_us,
            "battery_v": round(self.battery_v, 2),
            "current_a": round(self.current_a, 2),
            "imu_pitch": round(self.imu_pitch, 1),
            "imu_roll": round(self.imu_roll, 1),
            "imu_yaw": round(self.imu_yaw, 1),
            "robot_enabled": self.robot_enabled,
            "safety_state": self.safety_state,
            "safety_cause": self.safety_cause,
            "low_battery_active": self.low_battery_active,
            "gait_running": self.gait_running,
            "gait_type": self.gait_type,
            "gait_speed": round(self.gait_speed, 2),
            "servo_temp_max": round(self.servo_temp_max, 1),
            "servo_volt_min": round(self.servo_volt_min, 2),
            "servo_errors": self.servo_errors,
            "ctrl_version": self.ctrl_version,
            "fw_version": self.fw_version,
            "xbox_connected": self.xbox_connected,
            "teensy_connected": self.teensy_connected,
        }


# =============================================================================
# Telemetry WebSocket Server
# =============================================================================

class TelemetryServer:
    """WebSocket server for streaming telemetry to web dashboard."""

    def __init__(self, config: TelemetryServerConfig = None):
        self.config = config or TelemetryServerConfig()

        # Current telemetry data
        self._telemetry = TelemetryData()
        self._telemetry_lock = threading.Lock()

        # Configuration data (read-only view)
        self._config_data: Dict[str, Any] = {}
        self._config_lock = threading.Lock()

        # Callback for config changes (for future Phase 3)
        self._config_change_callback: Optional[Callable[[str, str, Any], bool]] = None

        # WebSocket state
        self._clients: Set = set()
        self._running = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._server_task = None

        # Threads
        self._ws_thread: Optional[threading.Thread] = None
        self._http_thread: Optional[threading.Thread] = None

    # -------------------------------------------------------------------------
    # Server lifecycle
    # -------------------------------------------------------------------------

    def start(self) -> bool:
        """Start the telemetry server."""
        if not WEBSOCKETS_AVAILABLE:
            print("[TelemetryServer] websockets library not available")
            return False

        if self._running:
            return True

        self._running = True

        # Start HTTP server thread (serves dashboard.html)
        self._http_thread = threading.Thread(target=self._run_http_server, daemon=True)
        self._http_thread.start()

        # Start WebSocket server thread
        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        print(f"[TelemetryServer] HTTP: http://0.0.0.0:{self.config.http_port}")
        print(f"[TelemetryServer] WebSocket: ws://0.0.0.0:{self.config.ws_port}")
        return True

    def _run_http_server(self) -> None:
        """HTTP server thread - serves static files including dashboard."""
        static_dir = os.path.dirname(__file__)
        handler = create_dashboard_http_handler(static_dir)
        
        try:
            with socketserver.TCPServer(
                (self.config.host, self.config.http_port),
                handler
            ) as httpd:
                httpd.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                while self._running:
                    httpd.handle_request()
        except OSError as e:
            if "Address already in use" in str(e):
                print(f"[TelemetryServer] HTTP port {self.config.http_port} in use (point cloud may be serving)")
            else:
                print(f"[TelemetryServer] HTTP server error: {e}")
        except Exception as e:
            print(f"[TelemetryServer] HTTP server error: {e}")

    def stop(self) -> None:
        """Stop the telemetry server."""
        self._running = False

        # Cancel WebSocket server
        if self._loop and self._server_task:
            self._loop.call_soon_threadsafe(self._server_task.cancel)

        if self._ws_thread:
            self._ws_thread.join(timeout=2.0)

        if self._http_thread:
            self._http_thread.join(timeout=2.0)

        print("[TelemetryServer] Stopped")

    def _run_ws_server(self) -> None:
        """WebSocket server thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        try:
            self._loop.run_until_complete(self._ws_main())
        except Exception as e:
            print(f"[TelemetryServer] WebSocket server error: {e}")
        finally:
            self._loop.close()

    async def _ws_main(self) -> None:
        """Main WebSocket server coroutine."""
        async with serve(self._handle_client, self.config.host, self.config.ws_port):
            # Start telemetry broadcast task
            self._server_task = asyncio.create_task(self._broadcast_loop())
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass

    async def _handle_client(self, websocket) -> None:
        """Handle a WebSocket client connection."""
        self._clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"[TelemetryServer] Client connected: {client_addr}")

        try:
            # Send initial config
            await self._send_config(websocket)

            async for message in websocket:
                await self._handle_message(websocket, message)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)
            print(f"[TelemetryServer] Client disconnected: {client_addr}")

    async def _handle_message(self, websocket, message: str) -> None:
        """Handle incoming WebSocket message."""
        try:
            data = json.loads(message)
            cmd = data.get("cmd", "")

            if cmd == "get_config":
                await self._send_config(websocket)

            elif cmd == "get_telemetry":
                await self._send_telemetry(websocket)

            elif cmd == "set_config":
                # Phase 3: Config editing (future)
                section = data.get("section", "")
                key = data.get("key", "")
                value = data.get("value")

                if self._config_change_callback:
                    success = self._config_change_callback(section, key, value)
                    await websocket.send(json.dumps({
                        "type": "config_result",
                        "success": success,
                        "section": section,
                        "key": key,
                        "value": value
                    }))
                else:
                    await websocket.send(json.dumps({
                        "type": "config_result",
                        "success": False,
                        "error": "Config editing not enabled"
                    }))

        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f"[TelemetryServer] Error handling message: {e}")

    async def _send_config(self, websocket) -> None:
        """Send configuration to a client."""
        with self._config_lock:
            config_copy = dict(self._config_data)

        await websocket.send(json.dumps({
            "type": "config",
            "data": config_copy
        }))

    async def _send_telemetry(self, websocket) -> None:
        """Send current telemetry to a client."""
        with self._telemetry_lock:
            telem = self._telemetry.to_dict()

        await websocket.send(json.dumps({
            "type": "telemetry",
            "data": telem
        }))

    async def _broadcast_loop(self) -> None:
        """Broadcast telemetry to all clients at configured rate."""
        interval = 1.0 / self.config.telemetry_rate_hz

        while self._running:
            if self._clients:
                with self._telemetry_lock:
                    telem = self._telemetry.to_dict()

                message = json.dumps({
                    "type": "telemetry",
                    "data": telem
                })

                # Broadcast to all clients
                await asyncio.gather(
                    *[client.send(message) for client in self._clients.copy()],
                    return_exceptions=True
                )

            await asyncio.sleep(interval)

    # -------------------------------------------------------------------------
    # Public API for feeding data
    # -------------------------------------------------------------------------

    def push_telemetry(self,
                       loop_time_us: float = 0.0,
                       battery_v: float = 0.0,
                       current_a: float = 0.0,
                       imu_pitch: float = 0.0,
                       imu_roll: float = 0.0,
                       imu_yaw: float = 0.0,
                       robot_enabled: bool = False,
                       safety_state: str = "---",
                       safety_cause: str = "---",
                       low_battery_active: bool = False,
                       gait_running: bool = False,
                       gait_type: str = "TRIPOD",
                       gait_speed: float = 0.0,
                       servo_temp_max: float = 0.0,
                       servo_volt_min: float = 0.0,
                       servo_errors: int = 0,
                       ctrl_version: str = "",
                       fw_version: str = "",
                       xbox_connected: bool = False,
                       teensy_connected: bool = False) -> None:
        """Update telemetry data. Call from main loop."""
        with self._telemetry_lock:
            self._telemetry.timestamp_ms = time.monotonic() * 1000
            self._telemetry.loop_time_us = loop_time_us
            self._telemetry.battery_v = battery_v
            self._telemetry.current_a = current_a
            self._telemetry.imu_pitch = imu_pitch
            self._telemetry.imu_roll = imu_roll
            self._telemetry.imu_yaw = imu_yaw
            self._telemetry.robot_enabled = robot_enabled
            self._telemetry.safety_state = safety_state
            self._telemetry.safety_cause = safety_cause
            self._telemetry.low_battery_active = low_battery_active
            self._telemetry.gait_running = gait_running
            self._telemetry.gait_type = gait_type
            self._telemetry.gait_speed = gait_speed
            self._telemetry.servo_temp_max = servo_temp_max
            self._telemetry.servo_volt_min = servo_volt_min
            self._telemetry.servo_errors = servo_errors
            self._telemetry.ctrl_version = ctrl_version
            self._telemetry.fw_version = fw_version
            self._telemetry.xbox_connected = xbox_connected
            self._telemetry.teensy_connected = teensy_connected

    def set_config(self, section: str, data: Dict[str, Any]) -> None:
        """Set configuration data for a section (read-only view)."""
        with self._config_lock:
            if section not in self._config_data:
                self._config_data[section] = {}
            self._config_data[section].update(data)

    def set_full_config(self, config: Dict[str, Dict[str, Any]]) -> None:
        """Set the entire configuration dictionary."""
        with self._config_lock:
            self._config_data = config

    def set_config_change_callback(self, callback: Callable[[str, str, Any], bool]) -> None:
        """Set callback for config change requests (Phase 3).

        Callback signature: callback(section: str, key: str, value: Any) -> bool
        Returns True if change was applied successfully.
        """
        self._config_change_callback = callback

    @property
    def client_count(self) -> int:
        """Number of connected clients."""
        return len(self._clients)


# =============================================================================
# Standalone test mode
# =============================================================================

if __name__ == "__main__":
    import random

    print("Telemetry Server Test Mode")
    print("Connect to ws://localhost:8766 to see telemetry")
    print("Press Ctrl+C to stop\n")

    config = TelemetryServerConfig()
    server = TelemetryServer(config)

    # Set some test config
    server.set_config("gait", {
        "type": "TRIPOD",
        "cycle_ms": 2000,
        "step_height_mm": 30,
        "step_length_mm": 40,
    })
    server.set_config("safety", {
        "soft_limits": True,
        "collision_detect": True,
        "volt_min": 10.0,
        "volt_max": 12.6,
    })

    if not server.start():
        print("Failed to start server")
        exit(1)

    try:
        tick = 0
        while True:
            # Simulate telemetry
            server.push_telemetry(
                loop_time_us=random.uniform(5800, 6200),
                battery_v=random.uniform(11.0, 12.2),
                current_a=random.uniform(0.5, 2.0),
                imu_pitch=random.uniform(-5, 5),
                imu_roll=random.uniform(-5, 5),
                imu_yaw=tick * 0.1 % 360,
                robot_enabled=True,
                safety_state="OK",
                gait_running=True,
                gait_type="TRIPOD",
                gait_speed=random.uniform(0.3, 0.7),
                servo_temp_max=random.uniform(35, 45),
                servo_volt_min=random.uniform(10.8, 11.2),
                ctrl_version="v0.8.3",
                fw_version="v1.0.0",
                xbox_connected=True,
                teensy_connected=True,
            )

            tick += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")

    server.stop()

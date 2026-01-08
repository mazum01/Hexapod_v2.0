#!/usr/bin/env python3
"""
Point Cloud WebSocket Server for MARS Hexapod

Streams ToF + IMU data as 3D point clouds to a Three.js web viewer.
Supports multiple ToF sensors, point accumulation, and world-frame transforms.

Architecture:
    ToF Thread → PointCloudServer → WebSocket → Three.js Viewer
                        ↑
                    IMU data (orientation)
                    Odometry (position estimate)

Protocol (JSON over WebSocket):
    Server → Client:
        {"type": "points", "data": [[x,y,z,r,g,b,a], ...], "timestamp": ms}
        {"type": "pose", "position": [x,y,z], "rotation": [roll,pitch,yaw]}
        {"type": "config", "sensors": [...], "fov": 45, ...}
    
    Client → Server:
        {"cmd": "clear"}           - Clear accumulated points
        {"cmd": "pause"}           - Pause streaming
        {"cmd": "resume"}          - Resume streaming
        {"cmd": "set_mode", "mode": "live"|"accumulate"}

Author: MARS Hexapod Project
Version: 0.1.0
"""

import asyncio
import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set, Any
from collections import deque
import os
import http.server
import socketserver

# Try to import websockets
try:
    import websockets
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("Warning: websockets not installed. Run: pip install websockets", end="\r\n")

import numpy as np


# =============================================================================
# Simple HTTP Server for Static Files
# =============================================================================

class PointCloudHTTPHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler that serves the point cloud viewer and static files."""
    
    def __init__(self, *args, static_dir: str = None, **kwargs):
        self.static_dir = static_dir or os.path.dirname(__file__)
        super().__init__(*args, directory=self.static_dir, **kwargs)
    
    def translate_path(self, path: str) -> str:
        """Map URL paths to filesystem paths."""
        # Root path serves the dashboard (main entry point)
        if path == '/' or path == '':
            return os.path.join(self.static_dir, 'static', 'dashboard.html')
        # /dashboard serves the telemetry dashboard
        if path == '/dashboard' or path == '/dashboard.html':
            return os.path.join(self.static_dir, 'static', 'dashboard.html')
        # /pointcloud or /viewer serves the point cloud viewer
        if path in ('/pointcloud', '/pointcloud.html', '/viewer', '/viewer.html'):
            return os.path.join(self.static_dir, 'static', 'pointcloud_viewer.html')
        # Other paths resolve normally under static/
        return super().translate_path(path)
    
    def log_message(self, format, *args):
        """Suppress HTTP access logs to reduce noise."""
        pass


def create_http_handler(static_dir: str):
    """Factory function to create handler with custom static directory."""
    class CustomHandler(PointCloudHTTPHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, static_dir=static_dir, **kwargs)
    return CustomHandler


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class PointCloudConfig:
    """Configuration for point cloud server."""
    
    # Server settings
    host: str = "0.0.0.0"
    port: int = 8765                    # WebSocket port
    http_port: int = 8080               # HTTP port for viewer
    
    # ToF sensor geometry (VL53L5CX)
    tof_fov_deg: float = 45.0          # Field of view (diagonal)
    tof_resolution: int = 8             # 8x8 grid
    tof_max_range_mm: int = 4000       # Max reliable range
    tof_min_range_mm: int = 20         # Min reliable range
    
    # Sensor mounting (relative to robot center, mm)
    # Format: {sensor_name: (x, y, z, roll, pitch, yaw)}
    sensor_mounts: Dict[str, Tuple[float, ...]] = field(default_factory=lambda: {
        "front": (100.0, 0.0, 50.0, 0.0, 0.0, 0.0),  # Front-facing, 10cm forward, 5cm up
    })
    
    # Accumulation settings
    accumulate_enabled: bool = True
    max_points: int = 50000             # Max accumulated points
    voxel_size_mm: float = 20.0         # Spatial bucketing resolution
    point_lifetime_s: float = 30.0      # Points older than this fade/delete
    
    # Streaming settings
    stream_rate_hz: float = 10.0        # Target WebSocket update rate
    
    # Colors
    color_by: str = "distance"          # "distance", "sensor", "age", "height"
    near_color: Tuple[int, int, int] = (255, 0, 0)    # Red = close
    far_color: Tuple[int, int, int] = (0, 0, 255)     # Blue = far


# =============================================================================
# 3D Point and Cloud Data Structures
# =============================================================================

@dataclass
class Point3D:
    """A single 3D point with metadata."""
    x: float
    y: float
    z: float
    r: int = 255
    g: int = 255
    b: int = 255
    a: int = 255
    sensor_id: str = ""
    timestamp: float = 0.0
    
    def to_list(self) -> List[float]:
        """Convert to [x, y, z, r, g, b, a] for JSON."""
        return [self.x, self.y, self.z, self.r, self.g, self.b, self.a]


class SpatialHashGrid:
    """Spatial hash grid for efficient point deduplication and lookup.
    
    Divides 3D space into voxels. Only one point per voxel is kept,
    with newer points replacing older ones.
    """
    
    def __init__(self, voxel_size_mm: float = 20.0, max_points: int = 50000):
        self.voxel_size = voxel_size_mm
        self.max_points = max_points
        self._grid: Dict[Tuple[int, int, int], Point3D] = {}
        self._insertion_order: deque = deque()
        self._lock = threading.Lock()
    
    def _voxel_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """Compute voxel key for a point."""
        return (
            int(x // self.voxel_size),
            int(y // self.voxel_size),
            int(z // self.voxel_size)
        )
    
    def add_point(self, point: Point3D) -> bool:
        """Add or update a point. Returns True if new voxel."""
        key = self._voxel_key(point.x, point.y, point.z)
        
        with self._lock:
            is_new = key not in self._grid
            self._grid[key] = point
            
            if is_new:
                self._insertion_order.append(key)
                
                # Evict oldest if over limit
                while len(self._grid) > self.max_points:
                    old_key = self._insertion_order.popleft()
                    if old_key in self._grid:
                        del self._grid[old_key]
            
            return is_new
    
    def add_points(self, points: List[Point3D]) -> int:
        """Add multiple points. Returns count of new voxels."""
        new_count = 0
        for p in points:
            if self.add_point(p):
                new_count += 1
        return new_count
    
    def get_all_points(self) -> List[Point3D]:
        """Get all points as a list."""
        with self._lock:
            return list(self._grid.values())
    
    def get_points_as_array(self) -> List[List[float]]:
        """Get all points as [[x,y,z,r,g,b,a], ...] for JSON."""
        with self._lock:
            return [p.to_list() for p in self._grid.values()]
    
    def clear(self) -> None:
        """Clear all points."""
        with self._lock:
            self._grid.clear()
            self._insertion_order.clear()
    
    def prune_old(self, max_age_s: float) -> int:
        """Remove points older than max_age_s. Returns count removed."""
        now = time.monotonic()
        removed = 0
        
        with self._lock:
            keys_to_remove = [
                k for k, p in self._grid.items()
                if now - p.timestamp > max_age_s
            ]
            for k in keys_to_remove:
                del self._grid[k]
                removed += 1
        
        return removed
    
    def __len__(self) -> int:
        return len(self._grid)


# =============================================================================
# ToF to 3D Conversion
# =============================================================================

class ToFProjector:
    """Converts ToF 8x8 distance grid to 3D points.
    
    The VL53L5CX has a 45° diagonal FoV. We compute ray directions for each
    pixel and project the measured distance along that ray.
    
    Note: The sensor reports RADIAL distance (along the ray), not planar depth.
    So we multiply the unit ray direction by the measured distance directly.
    
    Coordinate system (sensor frame):
        - X: right
        - Y: forward (into scene)
        - Z: up
    """
    
    def __init__(self, config: PointCloudConfig):
        self.config = config
        self._ray_cache: Dict[int, np.ndarray] = {}  # resolution -> ray directions
        self._build_ray_cache(8)  # Pre-build for 8x8
        self._build_ray_cache(4)  # Pre-build for 4x4
    
    def _build_ray_cache(self, resolution: int) -> None:
        """Pre-compute ray directions for a given resolution.
        
        The VL53L5CX has 45° diagonal FoV. For a square sensor:
            fov_h = fov_v = fov_diagonal / sqrt(2)
        
        We use an equiangular (fisheye-like) model where each pixel
        subtends an equal angle, which better matches the sensor optics.
        """
        fov_diag_rad = math.radians(self.config.tof_fov_deg)
        
        # Convert diagonal FoV to horizontal/vertical (square sensor)
        # For diagonal d and sides h=v: d² = h² + v² = 2h², so h = d/√2
        fov_hv_rad = fov_diag_rad / math.sqrt(2)
        half_fov = fov_hv_rad / 2  # ≈15.9° for 45° diagonal
        
        rays = np.zeros((resolution, resolution, 3), dtype=np.float32)
        
        for row in range(resolution):
            for col in range(resolution):
                # Normalized pixel coordinates [-1, 1]
                # Row 0 is top of sensor image; we flip to get Z-up
                u = (col - (resolution - 1) / 2) / ((resolution - 1) / 2)
                v = ((resolution - 1) / 2 - row) / ((resolution - 1) / 2)
                
                # Angular offsets (equiangular projection)
                theta_x = u * half_fov  # Horizontal angle from center
                theta_z = v * half_fov  # Vertical angle from center
                
                # Spherical to Cartesian (Y is forward/depth axis)
                # This correctly handles the spherical nature of ToF measurement
                x = math.sin(theta_x) * math.cos(theta_z)
                z = math.sin(theta_z)
                y = math.cos(theta_x) * math.cos(theta_z)
                
                # Normalize (should already be unit, but ensure numerical stability)
                length = math.sqrt(x*x + y*y + z*z)
                rays[row, col] = [x/length, y/length, z/length]
        
        self._ray_cache[resolution] = rays
    
    def project(self, 
                distances_mm: List[int],
                statuses: List[int],
                sensor_name: str = "front",
                resolution: int = 8) -> List[Point3D]:
        """Project ToF distances to 3D points.
        
        Args:
            distances_mm: Flattened distance array (row-major)
            statuses: Flattened status array (5/9 = valid)
            sensor_name: Sensor identifier for mounting transform
            resolution: Grid resolution (8 for 8x8, 4 for 4x4)
        
        Returns:
            List of Point3D in sensor frame
        """
        if resolution not in self._ray_cache:
            self._build_ray_cache(resolution)
        
        rays = self._ray_cache[resolution]
        points = []
        now = time.monotonic()
        
        for i, (dist, status) in enumerate(zip(distances_mm, statuses)):
            # Check validity
            if status not in (5, 9):  # VL53L5CX valid status codes
                continue
            if dist < self.config.tof_min_range_mm or dist > self.config.tof_max_range_mm:
                continue
            
            row = i // resolution
            col = i % resolution
            ray = rays[row, col]
            
            # Project along ray
            x = ray[0] * dist
            y = ray[1] * dist
            z = ray[2] * dist
            
            # Color by distance
            r, g, b = self._color_by_distance(dist)
            
            points.append(Point3D(
                x=x, y=y, z=z,
                r=r, g=g, b=b, a=255,
                sensor_id=sensor_name,
                timestamp=now
            ))
        
        return points
    
    def _color_by_distance(self, dist_mm: float) -> Tuple[int, int, int]:
        """Map distance to RGB color (near=red, far=blue)."""
        # Normalize to [0, 1]
        t = (dist_mm - self.config.tof_min_range_mm) / (
            self.config.tof_max_range_mm - self.config.tof_min_range_mm
        )
        t = max(0.0, min(1.0, t))
        
        # Interpolate
        r = int(self.config.near_color[0] * (1 - t) + self.config.far_color[0] * t)
        g = int(self.config.near_color[1] * (1 - t) + self.config.far_color[1] * t)
        b = int(self.config.near_color[2] * (1 - t) + self.config.far_color[2] * t)
        
        return (r, g, b)


# =============================================================================
# Coordinate Transforms
# =============================================================================

class PoseTracker:
    """Tracks robot pose (position + orientation) in world frame.
    
    Combines IMU orientation with estimated position from odometry.
    For now, position is estimated from gait commands. Future: visual odometry.
    """
    
    def __init__(self):
        # World-frame pose
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z in mm
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw in radians
        
        # Velocity estimate (from gait)
        self.velocity = np.array([0.0, 0.0, 0.0])  # mm/s
        
        self._last_update = time.monotonic()
        self._lock = threading.Lock()
    
    def update_orientation(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        """Update orientation from IMU."""
        with self._lock:
            self.orientation = np.radians([roll_deg, pitch_deg, yaw_deg])
    
    def update_velocity(self, vx: float, vy: float, vz: float = 0.0) -> None:
        """Update velocity estimate (mm/s in robot frame)."""
        with self._lock:
            self.velocity = np.array([vx, vy, vz])
    
    def integrate_position(self) -> None:
        """Integrate velocity to update position (call periodically)."""
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_update
            self._last_update = now
            
            if dt > 0.5:  # Skip large gaps
                return
            
            # Rotate velocity by yaw to get world-frame motion
            yaw = self.orientation[2]
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            # Robot frame to world frame
            world_vx = self.velocity[0] * cos_yaw - self.velocity[1] * sin_yaw
            world_vy = self.velocity[0] * sin_yaw + self.velocity[1] * cos_yaw
            
            self.position[0] += world_vx * dt
            self.position[1] += world_vy * dt
    
    def reset_position(self) -> None:
        """Reset position to origin."""
        with self._lock:
            self.position = np.array([0.0, 0.0, 0.0])
    
    def get_rotation_matrix(self) -> np.ndarray:
        """Get 3x3 rotation matrix from orientation (ZYX Euler)."""
        with self._lock:
            roll, pitch, yaw = self.orientation
        
        # Rotation matrices
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        # ZYX order: R = Rz * Ry * Rx
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        
        return R
    
    def transform_points(self, points: List[Point3D], sensor_mount: Tuple[float, ...]) -> List[Point3D]:
        """Transform points from sensor frame to world frame.
        
        Args:
            points: Points in sensor frame
            sensor_mount: (x, y, z, roll, pitch, yaw) of sensor on robot
        
        Returns:
            Points in world frame
        """
        if not points:
            return []
        
        # Get transforms
        robot_R = self.get_rotation_matrix()
        with self._lock:
            robot_pos = self.position.copy()
        
        # Sensor mount transform
        sx, sy, sz = sensor_mount[0], sensor_mount[1], sensor_mount[2]
        # TODO: Add sensor rotation if needed
        
        transformed = []
        for p in points:
            # Sensor frame -> robot frame (add mount offset)
            robot_x = p.x + sx
            robot_y = p.y + sy
            robot_z = p.z + sz
            
            # Robot frame -> world frame (rotate + translate)
            robot_vec = np.array([robot_x, robot_y, robot_z])
            world_vec = robot_R @ robot_vec + robot_pos
            
            transformed.append(Point3D(
                x=world_vec[0], y=world_vec[1], z=world_vec[2],
                r=p.r, g=p.g, b=p.b, a=p.a,
                sensor_id=p.sensor_id,
                timestamp=p.timestamp
            ))
        
        return transformed


# =============================================================================
# WebSocket Server
# =============================================================================

class PointCloudServer:
    """WebSocket server for streaming point cloud data.
    
    Usage:
        server = PointCloudServer(config)
        server.start()
        
        # In your main loop:
        server.push_tof_frame(sensor_name, distances, statuses)
        server.update_imu(roll, pitch, yaw)
        
        # On exit:
        server.stop()
    """
    
    def __init__(self, config: Optional[PointCloudConfig] = None):
        self.config = config or PointCloudConfig()
        
        # Components
        self.projector = ToFProjector(self.config)
        self.pose = PoseTracker()
        self.cloud = SpatialHashGrid(
            voxel_size_mm=self.config.voxel_size_mm,
            max_points=self.config.max_points
        )
        
        # State
        self._running = False
        self._paused = False
        self._mode = "accumulate" if self.config.accumulate_enabled else "live"
        self._clients: Set[Any] = set()
        self._lock = threading.Lock()
        
        # Threading
        self._server_thread: Optional[threading.Thread] = None
        self._http_thread: Optional[threading.Thread] = None
        self._http_server: Optional[socketserver.TCPServer] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        
        # Latest frame for live mode
        self._latest_points: List[List[float]] = []
        self._latest_points_lock = threading.Lock()
    
    def start(self) -> bool:
        """Start the WebSocket server and HTTP server in background threads."""
        if not WEBSOCKETS_AVAILABLE:
            print("Cannot start PointCloudServer: websockets not installed", end="\r\n")
            return False
        
        if self._running:
            return True
        
        self._running = True
        
        # Start HTTP server for viewer
        self._start_http_server()
        
        # Start WebSocket server
        self._server_thread = threading.Thread(
            target=self._run_server,
            name="PointCloudServer",
            daemon=True
        )
        self._server_thread.start()
        
        print(f"PointCloudServer started (mode={self._mode}):", end="\r\n")
        print(f"  HTTP viewer: http://{self.config.host}:{self.config.http_port}/", end="\r\n")
        print(f"  WebSocket:   ws://{self.config.host}:{self.config.port}", end="\r\n")
        return True
    
    def _start_http_server(self) -> None:
        """Start the HTTP server for serving static files."""
        static_dir = os.path.dirname(os.path.abspath(__file__))
        handler = create_http_handler(static_dir)
        
        try:
            self._http_server = socketserver.TCPServer(
                (self.config.host, self.config.http_port),
                handler
            )
            self._http_server.allow_reuse_address = True
            
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                name="PointCloudHTTP",
                daemon=True
            )
            self._http_thread.start()
        except Exception as e:
            print(f"Failed to start HTTP server on port {self.config.http_port}: {e}", end="\r\n")
    
    def stop(self) -> None:
        """Stop the server."""
        self._running = False
        
        # Stop HTTP server
        if self._http_server:
            try:
                self._http_server.shutdown()
            except Exception:
                pass
        
        # Wait for WebSocket server thread to finish gracefully
        # (the _serve loop exits when _running becomes False)
        if self._server_thread:
            self._server_thread.join(timeout=3.0)
        
        # Only force-stop the loop if it's still running after timeout
        if self._loop and not self._loop.is_closed():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except RuntimeError:
                pass  # Loop already closed
        print("PointCloudServer stopped", end="\r\n")
    
    def _run_server(self) -> None:
        """Run the async server (called in background thread)."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as e:
            print(f"PointCloudServer error: {e}", end="\r\n")
        finally:
            self._loop.close()
    
    async def _serve(self) -> None:
        """Async server main loop."""
        async with serve(self._handle_client, self.config.host, self.config.port):
            # Start broadcast task
            broadcast_task = asyncio.create_task(self._broadcast_loop())
            
            try:
                # Run until stopped
                while self._running:
                    await asyncio.sleep(0.1)
            finally:
                # Cancel broadcast task and wait for it
                broadcast_task.cancel()
                try:
                    await broadcast_task
                except asyncio.CancelledError:
                    pass
    
    async def _handle_client(self, websocket) -> None:
        """Handle a connected WebSocket client."""
        self._clients.add(websocket)
        print(f"PointCloud client connected ({len(self._clients)} total)", end="\r\n")
        
        try:
            # Send config on connect
            await websocket.send(json.dumps({
                "type": "config",
                "sensors": list(self.config.sensor_mounts.keys()),
                "fov": self.config.tof_fov_deg,
                "max_range": self.config.tof_max_range_mm,
                "voxel_size": self.config.voxel_size_mm
            }))
            
            # Handle incoming messages
            async for message in websocket:
                await self._handle_message(message)
        
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)
            print(f"PointCloud client disconnected ({len(self._clients)} remaining)", end="\r\n")

    async def _handle_message(self, message: str) -> None:
        """Handle incoming command from client."""
        try:
            data = json.loads(message)
            cmd = data.get("cmd")
            
            if cmd == "clear":
                self.cloud.clear()
                print("PointCloud cleared", end="\r\n")
            
            elif cmd == "pause":
                self._paused = True
            
            elif cmd == "resume":
                self._paused = False
            
            elif cmd == "set_mode":
                self._mode = data.get("mode", "live")
                print(f"PointCloud mode: {self._mode}", end="\r\n")
            
            elif cmd == "reset_pose":
                self.pose.reset_position()
                print("Pose reset", end="\r\n")
        
        except json.JSONDecodeError:
            pass
    
    async def _broadcast_loop(self) -> None:
        """Periodically broadcast point cloud to all clients."""
        interval = 1.0 / self.config.stream_rate_hz
        
        while self._running:
            await asyncio.sleep(interval)
            
            if self._paused or not self._clients:
                continue
            
            # Get points based on mode
            if self._mode == "accumulate":
                points = self.cloud.get_points_as_array()
            else:
                with self._latest_points_lock:
                    points = self._latest_points.copy()
            
            # Get pose
            with self.pose._lock:
                position = self.pose.position.tolist()
                rotation = self.pose.orientation.tolist()
            
            # Build message
            message = json.dumps({
                "type": "frame",
                "points": points,
                "pose": {
                    "position": position,
                    "rotation": rotation
                },
                "point_count": len(points),
                "timestamp": time.monotonic() * 1000
            })
            
            # Broadcast to all clients
            if self._clients:
                await asyncio.gather(
                    *[client.send(message) for client in self._clients.copy()],
                    return_exceptions=True
                )
    
    # -------------------------------------------------------------------------
    # Public API for feeding data
    # -------------------------------------------------------------------------
    
    def push_tof_frame(self,
                       sensor_name: str,
                       distances_mm: List[int],
                       statuses: List[int],
                       resolution: int = 8) -> int:
        """Push a ToF frame. Returns number of new points added.
        
        Call this from your main loop when new ToF data is available.
        """
        if not self._running or self._paused:
            return 0
        
        # Debug: count valid statuses
        valid_count = sum(1 for s in statuses if s in (5, 9))
        
        # Project to 3D (sensor frame)
        sensor_points = self.projector.project(
            distances_mm, statuses, sensor_name, resolution
        )
        
        if not sensor_points:
            # Debug: why no points?
            if valid_count == 0:
                pass  # No valid readings - normal
            else:
                print(f"[PointCloud] {valid_count} valid statuses but 0 points projected", end="\r\n")
            return 0
        
        # Transform to world frame
        mount = self.config.sensor_mounts.get(sensor_name, (0, 0, 0, 0, 0, 0))
        world_points = self.pose.transform_points(sensor_points, mount)
        
        # Update live view
        with self._latest_points_lock:
            self._latest_points = [p.to_list() for p in world_points]
        
        # Debug: periodically log point counts
        if len(self._clients) > 0 and len(world_points) > 0:
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1
            if self._debug_counter % 100 == 1:  # Every 100 frames
                print(f"[PointCloud] Frame: {len(world_points)} pts, cloud: {len(self.cloud)}, clients: {len(self._clients)}", end="\r\n")
        
        # Add to accumulated cloud
        if self._mode == "accumulate":
            return self.cloud.add_points(world_points)
        
        return len(world_points)
    
    def update_imu(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        """Update IMU orientation. Call from main loop."""
        self.pose.update_orientation(roll_deg, pitch_deg, yaw_deg)
    
    def update_velocity(self, vx_mm_s: float, vy_mm_s: float) -> None:
        """Update velocity estimate (mm/s in robot frame). Call from main loop."""
        self.pose.update_velocity(vx_mm_s, vy_mm_s)
        self.pose.integrate_position()
    
    def add_sensor(self, name: str, x: float, y: float, z: float,
                   roll: float = 0, pitch: float = 0, yaw: float = 0) -> None:
        """Add or update a sensor mount position."""
        self.config.sensor_mounts[name] = (x, y, z, roll, pitch, yaw)
    
    @property
    def client_count(self) -> int:
        """Number of connected clients."""
        return len(self._clients)
    
    @property
    def point_count(self) -> int:
        """Number of accumulated points."""
        return len(self.cloud)


# =============================================================================
# Standalone test mode
# =============================================================================

if __name__ == "__main__":
    import random
    
    print("PointCloud Server Test Mode", end="\r\n")
    print("Open pointcloud_viewer.html in a browser to connect", end="\r\n")
    print("Press Ctrl+C to stop\n", end="\r\n")
    
    config = PointCloudConfig()
    server = PointCloudServer(config)
    
    if not server.start():
        print("Failed to start server", end="\r\n")
        exit(1)
    
    try:
        yaw = 0.0
        while True:
            # Simulate ToF data
            distances = [random.randint(200, 2000) for _ in range(64)]
            statuses = [5] * 64  # All valid
            
            # Simulate slow rotation
            yaw += 0.5
            server.update_imu(0, 0, yaw)
            
            # Simulate forward motion
            server.update_velocity(50, 0)  # 50mm/s forward
            
            # Push frame
            server.push_tof_frame("front", distances, statuses)
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nStopping...", end="\r\n")
    
    server.stop()

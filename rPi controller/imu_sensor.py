"""
imu_sensor.py — IMU (BNO085) driver and threaded polling for MARS controller.

Provides ImuThread class for background I²C polling of the Adafruit BNO085
9-DOF IMU breakout (STEMMA QT / Qwiic compatible). Returns orientation data
(quaternion, euler angles) via thread-safe accessors.

Hardware: Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085 (BNO080)
I²C Address: 0x4A (default) or 0x4B (alternate via address jumper)
Requires: pip install adafruit-circuitpython-bno08x

Created: 2025-12-19
"""

from __future__ import annotations
import os
import time
import threading
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

# Attempt to import the BNO08x library (may not be present on all systems)
try:
    import board
    import busio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
    )
    BNO08X_AVAILABLE = True
except ImportError:
    BNO08X_AVAILABLE = False


# -----------------------------------------------------------------------------
# Data Structures
# -----------------------------------------------------------------------------

@dataclass
class ImuFrame:
    """Container for a single IMU measurement frame.
    
    Attributes:
        timestamp_ms: Monotonic timestamp when frame was captured (ms).
        valid: True if data is from a successful sensor read.
        
        Orientation (from rotation vector):
        quaternion: (w, x, y, z) unit quaternion representing orientation.
        roll_deg: Roll angle in degrees (rotation about X axis).
        pitch_deg: Pitch angle in degrees (rotation about Y axis).
        yaw_deg: Yaw/heading angle in degrees (rotation about Z axis).
        
        Raw sensor data (optional):
        accel_ms2: (x, y, z) acceleration in m/s² (None if not enabled).
        gyro_rads: (x, y, z) angular velocity in rad/s (None if not enabled).
        mag_ut: (x, y, z) magnetic field in µT (None if not enabled).
        
        Calibration:
        calibration_status: 0-3 (0=unreliable, 3=fully calibrated).
    """
    timestamp_ms: float = 0.0
    valid: bool = False
    
    # Orientation from rotation vector
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    
    # Raw sensor data (optional)
    accel_ms2: Optional[Tuple[float, float, float]] = None
    gyro_rads: Optional[Tuple[float, float, float]] = None
    mag_ut: Optional[Tuple[float, float, float]] = None
    
    # Calibration status
    calibration_status: int = 0


@dataclass
class ImuConfig:
    """Configuration for IMU sensor.
    
    Attributes:
        enabled: Whether IMU thread should run (False = no-op stub).
        i2c_bus: I²C bus number (1=default, 3=alternate, etc.).
        i2c_address: I²C address (0x4A default, 0x4B alternate).
        target_hz: Polling rate in Hz (default 100).
        enable_accel: Enable accelerometer reports.
        enable_gyro: Enable gyroscope reports.
        enable_mag: Enable magnetometer reports.
        use_game_rotation: Use game rotation vector (no magnetometer fusion).
    """
    enabled: bool = True
    i2c_bus: int = 3  # I2C bus number (use i2cdetect to find your device)
    i2c_address: int = 0x4A
    target_hz: float = 100.0
    enable_accel: bool = False
    enable_gyro: bool = False
    enable_mag: bool = False
    use_game_rotation: bool = False  # True = no mag, faster startup


# -----------------------------------------------------------------------------
# Quaternion to Euler Conversion
# -----------------------------------------------------------------------------

def quaternion_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in degrees.
    
    Uses ZYX (yaw-pitch-roll) convention common in aerospace.
    Returns (roll_deg, pitch_deg, yaw_deg).
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)  # Clamp to ±90°
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# -----------------------------------------------------------------------------
# ImuThread Class
# -----------------------------------------------------------------------------

class ImuThread(threading.Thread):
    """Background thread for IMU polling via I²C.
    
    Polls the BNO085 at configurable rate (default 100 Hz) and stores the
    latest orientation data. Main thread reads via get_frame() which is
    thread-safe and non-blocking.
    
    Usage:
        imu = ImuThread(config)
        imu.start()
        ...
        frame = imu.get_frame()
        if frame.valid:
            print(f"Roll: {frame.roll_deg:.1f}°")
        ...
        imu.stop()
    """
    
    def __init__(self, config: Optional[ImuConfig] = None):
        super().__init__(daemon=True, name="ImuThread")
        self.config = config or ImuConfig()
        
        # Thread control
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # Latest frame (protected by lock)
        self._frame = ImuFrame()
        
        # Statistics
        self._read_count = 0
        self._error_count = 0
        self._last_error: Optional[str] = None
        
        # Sensor handle (set in run())
        self._sensor: Optional[BNO08X_I2C] = None
        self._connected = False
    
    @property
    def connected(self) -> bool:
        """Return True if sensor is connected and responding."""
        with self._lock:
            return self._connected
    
    @property
    def read_count(self) -> int:
        """Return total successful read count."""
        with self._lock:
            return self._read_count
    
    @property
    def error_count(self) -> int:
        """Return total error count."""
        with self._lock:
            return self._error_count
    
    @property
    def last_error(self) -> Optional[str]:
        """Return last error message, if any."""
        with self._lock:
            return self._last_error
    
    def get_frame(self) -> ImuFrame:
        """Get the latest IMU frame (thread-safe, non-blocking).
        
        Returns a copy of the most recent frame. If sensor is not connected
        or has not yet produced data, returns an invalid frame.
        """
        with self._lock:
            # Return a copy to avoid race conditions
            return ImuFrame(
                timestamp_ms=self._frame.timestamp_ms,
                valid=self._frame.valid,
                quaternion=self._frame.quaternion,
                roll_deg=self._frame.roll_deg,
                pitch_deg=self._frame.pitch_deg,
                yaw_deg=self._frame.yaw_deg,
                accel_ms2=self._frame.accel_ms2,
                gyro_rads=self._frame.gyro_rads,
                mag_ut=self._frame.mag_ut,
                calibration_status=self._frame.calibration_status,
            )
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """Convenience method to get (roll, pitch, yaw) in degrees.
        
        Returns (0, 0, 0) if sensor not connected or data invalid.
        """
        frame = self.get_frame()
        if frame.valid:
            return (frame.roll_deg, frame.pitch_deg, frame.yaw_deg)
        return (0.0, 0.0, 0.0)
    
    def stop(self) -> None:
        """Signal thread to stop and wait for it to finish."""
        self._stop_event.set()
        self.join(timeout=1.0)
    
    def _init_sensor(self) -> bool:
        """Initialize the BNO085 sensor. Returns True on success."""
        if not BNO08X_AVAILABLE:
            with self._lock:
                self._last_error = "adafruit-circuitpython-bno08x not installed"
            return False
        
        try:
            # On Linux (Blinka), use the I2C bus number directly via device path
            # e.g., bus=3 -> /dev/i2c-3
            if self.config.i2c_bus == 1:
                # Use default board.SCL/board.SDA for I2C-1
                i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            else:
                # For other buses, use extended board API or direct device
                # Blinka on Pi: board.SCL and board.SDA are for bus 1
                # For other buses, we need to use the Linux I2C device directly
                try:
                    from adafruit_extended_bus import ExtendedI2C
                    i2c = ExtendedI2C(self.config.i2c_bus, frequency=400000)
                except ImportError as ie:
                    with self._lock:
                        self._last_error = f"Need adafruit-extended-bus for I2C bus {self.config.i2c_bus}. Install: pip install adafruit-extended-bus ({ie})"
                    return False
                except Exception as e:
                    with self._lock:
                        self._last_error = f"Failed to open I2C bus {self.config.i2c_bus}: {e}"
                    return False
            
            # Create sensor instance
            self._sensor = BNO08X_I2C(i2c, address=self.config.i2c_address)
            
            # Enable rotation vector reports
            if self.config.use_game_rotation:
                self._sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            else:
                self._sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            # Enable optional raw sensor reports
            if self.config.enable_accel:
                self._sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
            if self.config.enable_gyro:
                self._sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            if self.config.enable_mag:
                self._sensor.enable_feature(BNO_REPORT_MAGNETOMETER)
            
            with self._lock:
                self._connected = True
                self._last_error = None
            
            return True
            
        except Exception as e:
            with self._lock:
                self._connected = False
                self._last_error = f"Init failed: {type(e).__name__}: {e}"
            return False
    
    def _read_sensor(self) -> Optional[ImuFrame]:
        """Read current sensor data. Returns ImuFrame or None on error."""
        if self._sensor is None:
            return None
        
        try:
            timestamp_ms = time.monotonic() * 1000.0
            
            # Read rotation vector (quaternion)
            if self.config.use_game_rotation:
                quat = self._sensor.game_quaternion
            else:
                quat = self._sensor.quaternion
            
            if quat is None:
                return None
            
            # BNO085 returns (i, j, k, real) but we want (w, x, y, z)
            # Adafruit library returns (x, y, z, w) format
            w, x, y, z = quat[3], quat[0], quat[1], quat[2]
            
            # Convert to Euler angles
            roll_deg, pitch_deg, yaw_deg = quaternion_to_euler(w, x, y, z)
            
            # Read optional raw data
            accel = None
            gyro = None
            mag = None
            
            if self.config.enable_accel:
                accel = self._sensor.acceleration
            if self.config.enable_gyro:
                gyro = self._sensor.gyro
            if self.config.enable_mag:
                mag = self._sensor.magnetic
            
            # Calibration status (if available)
            cal_status = 0
            try:
                # BNO085 provides calibration status per sensor
                cal_status = getattr(self._sensor, 'calibration_status', 0)
            except Exception:
                pass
            
            return ImuFrame(
                timestamp_ms=timestamp_ms,
                valid=True,
                quaternion=(w, x, y, z),
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                accel_ms2=accel,
                gyro_rads=gyro,
                mag_ut=mag,
                calibration_status=cal_status,
            )
            
        except Exception as e:
            with self._lock:
                self._error_count += 1
                self._last_error = f"Read error: {e}"
            return None
    
    def run(self) -> None:
        """Main thread loop: poll IMU at target Hz."""
        # Lower thread priority slightly
        try:
            os.nice(5)
        except (OSError, AttributeError):
            pass
        
        # Skip if disabled
        if not self.config.enabled:
            return
        
        # Initialize sensor with retry
        retry_delay = 1.0
        max_retries = 5
        retries = 0
        
        while not self._stop_event.is_set() and retries < max_retries:
            if self._init_sensor():
                break
            retries += 1
            time.sleep(retry_delay)
        
        if not self._connected:
            return
        
        # Main polling loop
        period_s = 1.0 / self.config.target_hz
        next_tick = time.monotonic()
        
        while not self._stop_event.is_set():
            try:
                frame = self._read_sensor()
                
                if frame is not None:
                    with self._lock:
                        self._frame = frame
                        self._read_count += 1
                
                # Maintain timing
                next_tick += period_s
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -period_s:
                    # Fell behind, reset timing
                    next_tick = time.monotonic()
                    
            except Exception as e:
                with self._lock:
                    self._error_count += 1
                    self._last_error = f"Loop error: {e}"
                time.sleep(0.1)  # Brief pause on error


# -----------------------------------------------------------------------------
# Factory / Convenience
# -----------------------------------------------------------------------------

def create_imu_thread(cfg: Optional[dict] = None) -> ImuThread:
    """Create an ImuThread with optional config dict.
    
    Config keys (all optional):
        enabled: bool (default True)
        i2c_bus: int (default 1)
        i2c_address: int (default 0x4A)
        target_hz: float (default 100.0)
        enable_accel: bool (default False)
        enable_gyro: bool (default False)
        enable_mag: bool (default False)
        use_game_rotation: bool (default False)
    """
    config = ImuConfig()
    
    if cfg:
        if 'enabled' in cfg:
            config.enabled = bool(cfg['enabled'])
        if 'i2c_bus' in cfg:
            config.i2c_bus = int(cfg['i2c_bus'])
        if 'i2c_address' in cfg:
            # Accept hex string or int
            addr = cfg['i2c_address']
            if isinstance(addr, str):
                config.i2c_address = int(addr, 0)
            else:
                config.i2c_address = int(addr)
        if 'target_hz' in cfg:
            config.target_hz = float(cfg['target_hz'])
        if 'enable_accel' in cfg:
            config.enable_accel = bool(cfg['enable_accel'])
        if 'enable_gyro' in cfg:
            config.enable_gyro = bool(cfg['enable_gyro'])
        if 'enable_mag' in cfg:
            config.enable_mag = bool(cfg['enable_mag'])
        if 'use_game_rotation' in cfg:
            config.use_game_rotation = bool(cfg['use_game_rotation'])
    
    return ImuThread(config)


# -----------------------------------------------------------------------------
# Standalone Test
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print("IMU Sensor Test (BNO085)")
    print("=" * 40)
    
    if not BNO08X_AVAILABLE:
        print("ERROR: adafruit-circuitpython-bno08x not installed.")
        print("Install with: pip install adafruit-circuitpython-bno08x")
        exit(1)
    
    # Create and start IMU thread
    config = ImuConfig(target_hz=50.0)  # 50 Hz for test
    imu = ImuThread(config)
    imu.start()
    
    print("Waiting for sensor...")
    time.sleep(1.0)
    
    if not imu.connected:
        print(f"ERROR: {imu.last_error}")
        imu.stop()
        exit(1)
    
    print("Sensor connected. Reading orientation...")
    print("-" * 40)
    
    try:
        while True:
            frame = imu.get_frame()
            if frame.valid:
                print(f"\rRoll: {frame.roll_deg:7.2f}°  "
                      f"Pitch: {frame.pitch_deg:7.2f}°  "
                      f"Yaw: {frame.yaw_deg:7.2f}°  "
                      f"[{imu.read_count} reads, {imu.error_count} errors]",
                      end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        imu.stop()
        print("Done.")

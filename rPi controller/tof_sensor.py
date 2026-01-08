"""
tof_sensor.py — VL53L5CX ToF sensor driver and threaded polling for MARS controller.

Provides ToFThread class for background I²C polling of one or more VL53L5CX
8×8 Time-of-Flight ranging sensors. Returns distance arrays via thread-safe
accessors.

Hardware: Pimoroni VL53L5CX or ST VL53L5CX breakouts (I²C, Qwiic compatible)
Default I²C Address: 0x29 (can be changed programmatically per sensor)
Requires: pip install vl53l5cx-ctypes

Created: 2025-12-21
"""

from __future__ import annotations
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict

# Attempt to import the VL53L5CX library (may not be present on all systems)
try:
    import vl53l5cx_ctypes as vl53l5cx
    from vl53l5cx_ctypes import (
        DEFAULT_I2C_ADDRESS,
        STATUS_RANGE_VALID,
        STATUS_RANGE_VALID_LARGE_PULSE,
    )
    VL53L5CX_AVAILABLE = True
except ImportError:
    VL53L5CX_AVAILABLE = False
    DEFAULT_I2C_ADDRESS = 0x29
    STATUS_RANGE_VALID = 5
    STATUS_RANGE_VALID_LARGE_PULSE = 9


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

# Valid target status values (ranges 5 and 9 are considered valid)
VALID_TARGET_STATUS = (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE)

# Resolution options
RESOLUTION_4X4 = 16
RESOLUTION_8X8 = 64


# -----------------------------------------------------------------------------
# Data Structures
# -----------------------------------------------------------------------------

@dataclass
class ToFSensorConfig:
    """Configuration for a single ToF sensor.
    
    Attributes:
        name: Friendly name for this sensor (e.g., "left", "right", "front").
        i2c_address: I²C address (0x29 default, can be reassigned).
        enabled: Whether this sensor should be used.
    """
    name: str = "default"
    i2c_address: int = 0x29
    enabled: bool = True


@dataclass
class ToFConfig:
    """Configuration for ToF sensor system.
    
    Attributes:
        enabled: Whether ToF thread should run (False = no-op stub).
        i2c_bus: I²C bus number (1=default on Pi).
        target_hz: Polling rate in Hz (default 15, max at 8×8 resolution).
        resolution: Either 16 (4×4) or 64 (8×8).
        integration_time_ms: Sensor integration time (2-1000ms, lower = faster).
        sensors: List of individual sensor configurations.
        
        Filtering options:
        filter_mode: Filter mode - 'off', 'light', or 'full'
            - 'off': No filtering, raw sensor data (best for SLAM/motion)
            - 'light': Only reject invalid/high-sigma, no EMA (fast, minimal lag)
            - 'full': EMA smoothing + outlier rejection (smooth, for stationary)
        filter_alpha: EMA alpha for 'full' mode (0.0-1.0). Higher = more responsive.
        filter_sigma_threshold: Reject readings with sigma > this (mm).
        filter_outlier_mm: Jump threshold for outlier attenuation in 'full' mode.
    """
    enabled: bool = True
    i2c_bus: int = 1
    target_hz: float = 15.0
    resolution: int = RESOLUTION_8X8
    integration_time_ms: int = 20
    sensors: List[ToFSensorConfig] = field(default_factory=lambda: [ToFSensorConfig()])
    
    # Filtering - mode: 'off', 'light', 'full'
    filter_mode: str = 'light'      # 'off'=raw, 'light'=reject bad, 'full'=EMA
    filter_alpha: float = 0.5       # EMA smoothing (higher = more responsive)
    filter_sigma_threshold: int = 20  # Reject high-sigma readings (mm)
    filter_outlier_mm: int = 100    # Jump threshold for outlier rejection


@dataclass
class ToFZoneData:
    """Data for a single zone (pixel) in the ToF array.
    
    Attributes:
        distance_mm: Measured distance in millimeters (-1 if invalid).
        valid: True if this zone has a valid reading.
        reflectance: Estimated target reflectance (0-100%).
        sigma_mm: Noise estimate in mm (lower = more confident).
    """
    distance_mm: int = -1
    valid: bool = False
    reflectance: int = 0
    sigma_mm: int = 0


@dataclass
class ToFSensorFrame:
    """Container for a single sensor's measurement frame.
    
    Attributes:
        name: Sensor name (from config).
        i2c_address: Sensor I²C address.
        timestamp_ms: Monotonic timestamp when frame was captured (ms).
        valid: True if data is from a successful sensor read.
        resolution: Current resolution (16 for 4×4, 64 for 8×8).
        temperature_c: Sensor silicon temperature in Celsius.
        
        Distance data (flattened array, row-major order):
        distance_mm: List of distance values in mm (length = resolution).
        status: List of target status values (5 or 9 = valid).
        reflectance: List of reflectance percentages (0-100).
        sigma_mm: List of range sigma (noise estimate) values.
    """
    name: str = ""
    i2c_address: int = 0x29
    timestamp_ms: float = 0.0
    valid: bool = False
    resolution: int = 64
    temperature_c: int = 0
    
    # Distance arrays (length = resolution, 64 for 8×8 or 16 for 4×4)
    distance_mm: List[int] = field(default_factory=lambda: [-1] * 64)
    status: List[int] = field(default_factory=lambda: [0] * 64)
    reflectance: List[int] = field(default_factory=lambda: [0] * 64)
    sigma_mm: List[int] = field(default_factory=lambda: [0] * 64)
    
    def get_distance_grid(self) -> List[List[int]]:
        """Return distance data as 2D grid (8×8 or 4×4).
        
        Grid is oriented with [0][0] at bottom-left (sensor view).
        Returns list of rows, each row is a list of distances.
        """
        size = 8 if self.resolution == 64 else 4
        grid = []
        for row in range(size):
            start = row * size
            grid.append(self.distance_mm[start:start + size])
        # Flip vertically to match physical orientation (bottom-left origin)
        grid.reverse()
        return grid
    
    def get_closest_distance(self) -> Tuple[int, int, int]:
        """Find the closest valid distance reading.
        
        Returns (distance_mm, row, col) or (-1, -1, -1) if no valid readings.
        Row/col are in the flipped grid coordinate system.
        """
        closest_dist = 9999
        closest_idx = -1
        
        for i, (dist, stat) in enumerate(zip(self.distance_mm, self.status)):
            if stat in VALID_TARGET_STATUS and 0 < dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        
        if closest_idx < 0:
            return (-1, -1, -1)
        
        size = 8 if self.resolution == 64 else 4
        row = closest_idx // size
        col = closest_idx % size
        # Flip row to match grid orientation
        row = size - 1 - row
        return (closest_dist, row, col)


@dataclass
class ToFFrame:
    """Container for all ToF sensor data (multi-sensor support).
    
    Attributes:
        timestamp_ms: Monotonic timestamp when frame was captured (ms).
        sensors: Dict of sensor name -> ToFSensorFrame.
    """
    timestamp_ms: float = 0.0
    sensors: Dict[str, ToFSensorFrame] = field(default_factory=dict)
    
    def get_sensor(self, name: str) -> Optional[ToFSensorFrame]:
        """Get frame for a specific sensor by name."""
        return self.sensors.get(name)
    
    def get_closest_obstacle(self) -> Tuple[int, str]:
        """Find the closest obstacle across all sensors.
        
        Returns (distance_mm, sensor_name) or (-1, "") if no valid readings.
        """
        closest_dist = 9999
        closest_sensor = ""
        
        for name, frame in self.sensors.items():
            if not frame.valid:
                continue
            dist, _, _ = frame.get_closest_distance()
            if 0 < dist < closest_dist:
                closest_dist = dist
                closest_sensor = name
        
        if closest_dist == 9999:
            return (-1, "")
        return (closest_dist, closest_sensor)


# -----------------------------------------------------------------------------
# ToF Sensor Handle (wraps vl53l5cx library)
# -----------------------------------------------------------------------------

class ToFSensorHandle:
    """Handle for a single VL53L5CX sensor.
    
    Wraps the vl53l5cx_ctypes library to provide a simpler interface.
    Handles initialization, configuration, and data reading.
    """
    
    def __init__(self, config: ToFSensorConfig, i2c_bus: int = 1):
        self.config = config
        self.i2c_bus = i2c_bus
        self._sensor = None
        self._connected = False
        self._last_error: Optional[str] = None
        self._initialized = False
    
    @property
    def connected(self) -> bool:
        return self._connected
    
    @property
    def last_error(self) -> Optional[str]:
        return self._last_error
    
    def init(self, resolution: int = 64, frequency_hz: int = 15, 
             integration_time_ms: int = 20) -> bool:
        """Initialize the sensor.
        
        Note: VL53L5CX requires ~85KB firmware upload which takes several seconds.
        
        Returns True on success.
        """
        if not VL53L5CX_AVAILABLE:
            self._last_error = "vl53l5cx-ctypes library not installed"
            return False
        
        try:
            # First, probe the I2C bus to verify the sensor is present
            # This avoids segfaults from the C library if sensor is missing
            from smbus2 import SMBus
            i2c_dev = SMBus(self.i2c_bus)
            
            try:
                # Try to read a byte from the sensor's address
                # VL53L5CX has a device ID register, but any read will do
                i2c_dev.read_byte(self.config.i2c_address)
            except OSError as e:
                self._last_error = f"I2C probe failed at 0x{self.config.i2c_address:02X}: {e}"
                i2c_dev.close()
                return False
            
            print(f"[ToF] I2C probe OK at 0x{self.config.i2c_address:02X}, starting firmware upload...", end="\r\n")
            
            self._sensor = vl53l5cx.VL53L5CX(
                i2c_addr=self.config.i2c_address,
                i2c_dev=i2c_dev
            )
            
            # Configure sensor
            self._sensor.set_resolution(resolution)
            self._sensor.set_ranging_frequency_hz(frequency_hz)
            self._sensor.set_integration_time_ms(integration_time_ms)
            
            self._connected = True
            self._initialized = True
            self._last_error = None
            return True
            
        except Exception as e:
            self._connected = False
            self._last_error = f"Init failed: {type(e).__name__}: {e}"
            return False
    
    def start_ranging(self) -> bool:
        """Start continuous ranging."""
        if not self._connected or self._sensor is None:
            return False
        try:
            self._sensor.start_ranging()
            return True
        except Exception as e:
            self._last_error = f"Start ranging failed: {e}"
            return False
    
    def stop_ranging(self) -> bool:
        """Stop ranging."""
        if not self._connected or self._sensor is None:
            return False
        try:
            self._sensor.stop_ranging()
            return True
        except Exception as e:
            self._last_error = f"Stop ranging failed: {e}"
            return False
    
    def data_ready(self) -> bool:
        """Check if new data is available."""
        if not self._connected or self._sensor is None:
            return False
        try:
            return self._sensor.data_ready()
        except Exception:
            return False
    
    def read_data(self) -> Optional[ToFSensorFrame]:
        """Read current ranging data.
        
        Returns ToFSensorFrame or None on error.
        """
        if not self._connected or self._sensor is None:
            return None
        
        try:
            data = self._sensor.get_data()
            
            # VL53L5CX returns data as nested ctypes arrays: [target_index][zone_index]
            # We only use target 0 (first target per zone)
            distance_mm = [data.distance_mm[0][i] for i in range(64)]
            status = [data.target_status[0][i] for i in range(64)]
            reflectance = [data.reflectance[0][i] for i in range(64)]
            sigma_mm = [data.range_sigma_mm[0][i] for i in range(64)]
            
            # Build frame
            frame = ToFSensorFrame(
                name=self.config.name,
                i2c_address=self.config.i2c_address,
                timestamp_ms=time.monotonic() * 1000.0,
                valid=True,
                resolution=64,  # Will be set properly based on config
                temperature_c=data.silicon_temp_degc,
                distance_mm=distance_mm,
                status=status,
                reflectance=reflectance,
                sigma_mm=sigma_mm,
            )
            return frame
            
        except Exception as e:
            self._last_error = f"Read failed: {e}"
            return None
    
    def set_i2c_address(self, new_address: int) -> bool:
        """Change the sensor's I²C address.
        
        Note: This is volatile - address resets on power cycle.
        """
        if not self._connected or self._sensor is None:
            return False
        try:
            if self._sensor.set_i2c_address(new_address):
                self.config.i2c_address = new_address
                return True
            return False
        except Exception as e:
            self._last_error = f"Set address failed: {e}"
            return False


# -----------------------------------------------------------------------------
# ToFThread Class
# -----------------------------------------------------------------------------

class ToFThread(threading.Thread):
    """Background thread for ToF sensor polling.
    
    Polls one or more VL53L5CX sensors at configurable rate and stores the
    latest data. Main thread reads via get_frame() which is thread-safe
    and non-blocking.
    
    Usage:
        config = ToFConfig(sensors=[
            ToFSensorConfig(name="front", i2c_address=0x29),
        ])
        tof = ToFThread(config)
        tof.start()
        ...
        frame = tof.get_frame()
        if "front" in frame.sensors:
            sensor_frame = frame.sensors["front"]
            if sensor_frame.valid:
                dist, row, col = sensor_frame.get_closest_distance()
                print(f"Closest: {dist}mm at ({row},{col})")
        ...
        tof.stop()
    """
    
    def __init__(self, config: Optional[ToFConfig] = None):
        super().__init__(daemon=True, name="ToFThread")
        self.config = config or ToFConfig()
        
        # Thread control
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # Sensor handles
        self._handles: Dict[str, ToFSensorHandle] = {}
        
        # Latest frame (protected by lock) - holds filtered data
        self._frame = ToFFrame()
        
        # EMA filter state per sensor: Dict[sensor_name, List[float]]
        # Each list holds the smoothed distance for each zone
        self._ema_state: Dict[str, List[float]] = {}
        
        # Statistics
        self._read_count = 0
        self._error_count = 0
        self._last_error: Optional[str] = None
        self._connected_count = 0
        self._initialized = False
    
    def init_sensors(self) -> int:
        """Initialize all configured sensors. MUST be called from main thread.
        
        The VL53L5CX ctypes library segfaults when initialized from a 
        non-main thread, so this must be called before start().
        
        Returns number of successfully initialized sensors.
        """
        if not VL53L5CX_AVAILABLE:
            with self._lock:
                self._last_error = "vl53l5cx-ctypes not installed"
            return 0
        
        success_count = 0
        
        for sensor_config in self.config.sensors:
            if not sensor_config.enabled:
                continue
            
            handle = ToFSensorHandle(sensor_config, self.config.i2c_bus)
            
            # Calculate frequency based on resolution
            # 8×8: max 15Hz, 4×4: max 60Hz
            freq = min(
                self.config.target_hz,
                15 if self.config.resolution == 64 else 60
            )
            
            print(f"[ToF] Initializing sensor '{sensor_config.name}' at 0x{sensor_config.i2c_address:02X}...", end="\r\n")
            
            if handle.init(
                resolution=self.config.resolution,
                frequency_hz=int(freq),
                integration_time_ms=self.config.integration_time_ms
            ):
                if handle.start_ranging():
                    self._handles[sensor_config.name] = handle
                    success_count += 1
                    print(f"[ToF] Sensor '{sensor_config.name}' initialized OK", end="\r\n")
                else:
                    print(f"[ToF] Sensor '{sensor_config.name}' failed to start: {handle.last_error}", end="\r\n")
            else:
                print(f"[ToF] Sensor '{sensor_config.name}' init failed: {handle.last_error}", end="\r\n")
        
        with self._lock:
            self._connected_count = success_count
            self._initialized = True
        
        return success_count
    
    def _apply_filter(self, sensor_name: str, raw_frame: ToFSensorFrame) -> ToFSensorFrame:
        """Apply temporal EMA filter and outlier rejection to raw sensor data.
        
        Args:
            sensor_name: Name of the sensor (for EMA state lookup)
            raw_frame: Unfiltered sensor frame from hardware
            
        Returns:
            Filtered frame with smoothed distance values
        """
        mode = self.config.filter_mode.lower()
        
        # Mode: 'off' - return raw data unchanged
        if mode == 'off':
            return raw_frame
        
        sigma_thresh = self.config.filter_sigma_threshold
        resolution = raw_frame.resolution
        
        # Mode: 'light' - only reject invalid/high-sigma readings, no EMA
        if mode == 'light':
            filtered_distances = []
            for i in range(resolution):
                raw_dist = raw_frame.distance_mm[i]
                raw_sigma = raw_frame.sigma_mm[i]
                status = raw_frame.status[i]
                
                # Check if reading is valid
                is_valid = (status in VALID_TARGET_STATUS and raw_dist > 0)
                
                # Reject high-sigma (noisy) readings
                if is_valid and raw_sigma > sigma_thresh:
                    is_valid = False
                
                if is_valid:
                    filtered_distances.append(raw_dist)
                else:
                    filtered_distances.append(-1)  # Mark as invalid
            
            return ToFSensorFrame(
                name=raw_frame.name,
                i2c_address=raw_frame.i2c_address,
                timestamp_ms=raw_frame.timestamp_ms,
                valid=raw_frame.valid,
                resolution=raw_frame.resolution,
                temperature_c=raw_frame.temperature_c,
                distance_mm=filtered_distances,
                status=raw_frame.status[:],
                reflectance=raw_frame.reflectance[:],
                sigma_mm=raw_frame.sigma_mm[:],
            )
        
        # Mode: 'full' - EMA smoothing with outlier rejection
        alpha = self.config.filter_alpha
        outlier_thresh = self.config.filter_outlier_mm
        
        # Initialize EMA state if needed
        if sensor_name not in self._ema_state:
            # Start with first valid readings
            self._ema_state[sensor_name] = [float(d) if d > 0 else 0.0 
                                            for d in raw_frame.distance_mm]
        
        ema = self._ema_state[sensor_name]
        filtered_distances = []
        
        for i in range(resolution):
            raw_dist = raw_frame.distance_mm[i]
            raw_sigma = raw_frame.sigma_mm[i]
            status = raw_frame.status[i]
            current_ema = ema[i]
            
            # Check if reading is valid
            is_valid = (status in VALID_TARGET_STATUS and raw_dist > 0)
            
            # Reject high-sigma (noisy) readings
            if is_valid and raw_sigma > sigma_thresh:
                is_valid = False
            
            if is_valid:
                # Check for outlier (large jump from current EMA)
                if current_ema > 0:
                    jump = abs(raw_dist - current_ema)
                    if jump > outlier_thresh:
                        # Outlier detected - use reduced alpha for slower adaptation
                        alpha_adj = alpha * 0.3
                    else:
                        alpha_adj = alpha
                else:
                    # No prior EMA - accept reading at full weight
                    alpha_adj = 1.0
                
                # Update EMA: new = alpha * raw + (1 - alpha) * old
                new_ema = alpha_adj * raw_dist + (1.0 - alpha_adj) * current_ema
                ema[i] = new_ema
                filtered_distances.append(int(round(new_ema)))
            else:
                # Invalid reading - hold previous value
                filtered_distances.append(int(round(current_ema)) if current_ema > 0 else -1)
        
        # Create filtered frame
        filtered_frame = ToFSensorFrame(
            name=raw_frame.name,
            i2c_address=raw_frame.i2c_address,
            timestamp_ms=raw_frame.timestamp_ms,
            valid=raw_frame.valid,
            resolution=raw_frame.resolution,
            temperature_c=raw_frame.temperature_c,
            distance_mm=filtered_distances,
            status=raw_frame.status[:],  # Keep original status
            reflectance=raw_frame.reflectance[:],
            sigma_mm=raw_frame.sigma_mm[:],
        )
        
        return filtered_frame
    
    @property
    def connected(self) -> bool:
        """Return True if at least one sensor is connected."""
        with self._lock:
            return self._connected_count > 0
    
    @property
    def connected_count(self) -> int:
        """Return number of connected sensors."""
        with self._lock:
            return self._connected_count
    
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
    
    def get_sensor_names(self) -> List[str]:
        """Return list of configured sensor names."""
        return [s.name for s in self.config.sensors if s.enabled]
    
    def get_frame(self) -> ToFFrame:
        """Get the latest ToF frame (thread-safe, non-blocking).
        
        Returns a copy of the most recent frame. If sensors are not connected
        or have not yet produced data, returns an empty frame.
        """
        with self._lock:
            # Return a deep copy to avoid race conditions
            frame_copy = ToFFrame(timestamp_ms=self._frame.timestamp_ms)
            for name, sensor_frame in self._frame.sensors.items():
                frame_copy.sensors[name] = ToFSensorFrame(
                    name=sensor_frame.name,
                    i2c_address=sensor_frame.i2c_address,
                    timestamp_ms=sensor_frame.timestamp_ms,
                    valid=sensor_frame.valid,
                    resolution=sensor_frame.resolution,
                    temperature_c=sensor_frame.temperature_c,
                    distance_mm=sensor_frame.distance_mm[:],
                    status=sensor_frame.status[:],
                    reflectance=sensor_frame.reflectance[:],
                    sigma_mm=sensor_frame.sigma_mm[:],
                )
            return frame_copy
    
    def get_sensor_frame(self, name: str) -> Optional[ToFSensorFrame]:
        """Get frame for a specific sensor by name.
        
        Returns None if sensor not found or not connected.
        """
        frame = self.get_frame()
        return frame.sensors.get(name)
    
    def get_closest_obstacle(self) -> Tuple[int, str]:
        """Convenience method to get closest obstacle across all sensors.
        
        Returns (distance_mm, sensor_name) or (-1, "") if no valid readings.
        """
        return self.get_frame().get_closest_obstacle()
    
    def stop(self) -> None:
        """Signal thread to stop and wait for it to finish."""
        self._stop_event.set()
        self.join(timeout=2.0)
    
    def run(self) -> None:
        """Thread main loop - poll sensors and update frame."""
        if not self.config.enabled:
            return
        
        # Check that init_sensors() was called before start()
        with self._lock:
            if not self._initialized:
                self._last_error = "init_sensors() must be called before start()"
                return
            if self._connected_count == 0:
                self._last_error = "No sensors connected"
                return
        
        # Calculate loop timing
        target_period_s = 1.0 / self.config.target_hz
        
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            
            # Poll each sensor
            for name, handle in self._handles.items():
                try:
                    if handle.data_ready():
                        raw_frame = handle.read_data()
                        if raw_frame is not None:
                            # Apply temporal filter to smooth noise
                            filtered_frame = self._apply_filter(name, raw_frame)
                            with self._lock:
                                self._frame.sensors[name] = filtered_frame
                                self._frame.timestamp_ms = filtered_frame.timestamp_ms
                                self._read_count += 1
                        else:
                            with self._lock:
                                self._error_count += 1
                except Exception as e:
                    with self._lock:
                        self._error_count += 1
                        self._last_error = f"{name}: {e}"
            
            # Sleep for remainder of period
            elapsed = time.monotonic() - loop_start
            sleep_time = target_period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Cleanup - stop ranging on all sensors
        for name, handle in self._handles.items():
            try:
                handle.stop_ranging()
            except Exception:
                pass


# -----------------------------------------------------------------------------
# Module-level convenience functions
# -----------------------------------------------------------------------------

def create_tof_thread(
    sensors: Optional[List[Tuple[str, int]]] = None,
    target_hz: float = 15.0,
    resolution: int = RESOLUTION_8X8,
    i2c_bus: int = 1
) -> ToFThread:
    """Create and configure a ToF thread.
    
    Args:
        sensors: List of (name, i2c_address) tuples. Default: [("front", 0x29)]
        target_hz: Polling rate (default 15Hz)
        resolution: 16 (4×4) or 64 (8×8)
        i2c_bus: I²C bus number
    
    Returns:
        Configured ToFThread (not started)
    """
    if sensors is None:
        sensors = [("front", 0x29)]
    
    sensor_configs = [
        ToFSensorConfig(name=name, i2c_address=addr)
        for name, addr in sensors
    ]
    
    config = ToFConfig(
        enabled=True,
        i2c_bus=i2c_bus,
        target_hz=target_hz,
        resolution=resolution,
        sensors=sensor_configs,
    )
    
    return ToFThread(config)


# -----------------------------------------------------------------------------
# Test code
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    print("VL53L5CX ToF Sensor Test")
    print(f"Library available: {VL53L5CX_AVAILABLE}")
    
    if not VL53L5CX_AVAILABLE:
        print("Install with: pip install vl53l5cx-ctypes")
        exit(1)
    
    # Create thread with single sensor at default address
    tof = create_tof_thread(
        sensors=[("front", 0x29)],
        target_hz=10.0,
        resolution=RESOLUTION_8X8,
        i2c_bus=3  # Adjust for your setup
    )
    
    print("\nInitializing sensors (firmware upload takes ~10 seconds)...")
    count = tof.init_sensors()
    
    if count == 0:
        print(f"Failed to connect: {tof.last_error}")
        exit(1)
    
    print(f"Starting polling thread...")
    tof.start()
    
    print(f"\nConnected sensors: {tof.connected_count}")
    print("Reading data (Ctrl+C to stop)...\n")
    
    try:
        while True:
            frame = tof.get_frame()
            
            for name, sensor_frame in frame.sensors.items():
                if not sensor_frame.valid:
                    continue
                
                dist, row, col = sensor_frame.get_closest_distance()
                print(f"[{name}] Closest: {dist:4d}mm at ({row},{col}) | "
                      f"Temp: {sensor_frame.temperature_c}°C | "
                      f"Reads: {tof.read_count}", end="\r")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    tof.stop()
    print("Done.")

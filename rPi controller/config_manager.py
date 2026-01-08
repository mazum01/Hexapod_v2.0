"""
config_manager.py — Configuration loading and persistence for MARS controller.

Extracted from controller.py as part of Phase 2 modularization (2025-12-18).
Handles controller.ini parsing, defaults, and save functions.
"""

from __future__ import annotations
import os
import configparser
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# -----------------------------------------------------------------------------
# Configuration file path (module-level state shared with controller.py)
# -----------------------------------------------------------------------------
_cfg: Optional[configparser.ConfigParser] = None
_cfg_path: Optional[str] = None


def _get_config_path() -> str:
    """Return path to controller.ini relative to this module."""
    if '__file__' in globals():
        return os.path.join(os.path.dirname(__file__), 'controller.ini')
    return 'controller.ini'


def set_config_state(cfg: configparser.ConfigParser, cfg_path: str) -> None:
    """Set shared config state from controller.py after it loads the config.
    
    This allows save functions in this module to use the already-loaded
    ConfigParser instance from controller.py.
    """
    global _cfg, _cfg_path
    _cfg = cfg
    _cfg_path = cfg_path


# -----------------------------------------------------------------------------
# Default configuration values
# -----------------------------------------------------------------------------
@dataclass
class SerialConfig:
    error_threshold: int = 3


@dataclass
class TelemetryConfig:
    grace_s: float = 2.0
    retry_s: float = 5.0


@dataclass
class TimingConfig:
    loop_target_ms: float = 50.0
    teensy_loop_us: int = 6024
    telem_sync_fallback_ms: float = 100.0
    teensy_reconnect_backoff_s: float = 2.0


@dataclass
class DisplayConfig:
    screen_refresh_ms: float = 100.0
    brightness: int = 100
    auto_disable_s: float = 5.0
    thread_enabled: bool = True
    thread_hz: float = 15.0
    blink_frame_divisor: int = 2
    engineering_lcars: bool = True
    startup_delay_s: float = 5.0


@dataclass
class MenuConfig:
    theme: int = 0  # 0=MARS, 1=LCARS
    palette: int = 0  # LCARS palette index


@dataclass
class GaitConfig:
    cycle_ms: int = 2000
    step_len_mm: float = 40.0
    base_y_mm: float = -120.0
    max_step_len_mm: float = 40.0
    overlap_pct: float = 5.0
    smoothing_alpha: float = 0.15
    send_divisor: int = 3
    turn_max_deg_s: float = 60.0
    cmd_throttle_ms: float = 50.0
    layer2_divisor: int = 5
    feet_tolerance_mm: float = 0.3
    feet_max_skip_ms: float = 100.0
    # Saved width/lift
    width_mm: float = 100.0
    lift_mm: float = 60.0
    width_min_mm: float = 50.0
    width_max_mm: float = 175.0
    lift_min_mm: float = 20.0
    lift_max_mm: float = 100.0
    # Bezier curve shape
    bezier_p1_height: float = 0.15
    bezier_p1_overshoot: float = 1.1
    bezier_p2_height: float = 1.5
    bezier_p3_height: float = 0.35
    bezier_p3_overshoot: float = 1.1


@dataclass
class ImuConfig:
    enabled: bool = True
    hz: float = 100.0
    bus: int = 3
    address: int = 0x4A
    use_game_rotation: bool = False
    enable_accel: bool = False
    enable_gyro: bool = False
    enable_mag: bool = False


@dataclass
class LevelingConfig:
    enabled: bool = False
    gain: float = 1.0
    max_correction_mm: float = 30.0
    filter_alpha: float = 0.15
    pitch_offset: float = 0.0
    roll_offset: float = 0.0
    tilt_limit_deg: float = 25.0
    lean_enabled: bool = False
    lean_max_deg: float = 7.0
    lean_filter_alpha: float = 0.1


@dataclass
class ToFConfig:
    enabled: bool = True
    hz: float = 15.0
    bus: int = 1
    resolution: int = 64
    sensors: List[Tuple[str, int]] = field(default_factory=lambda: [("front", 0x29)])
    filter_mode: str = 'light'
    filter_alpha: float = 0.5
    filter_sigma_threshold: int = 20
    filter_outlier_mm: int = 100


@dataclass
class PounceConfig:
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
class EyeConfig:
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
    human_eye_spacing_pct: float = 0.25
    human_eye_size: int = 33
    human_eye_color: int = 0
    shape: int = 2  # ROUNDRECTANGLE default
    crt_mode: bool = False
    # Eye colors (R,G,B tuples)
    color_ellipse: Tuple[int, int, int] = (255, 80, 20)
    color_rectangle: Tuple[int, int, int] = (255, 255, 255)
    color_roundrect: Tuple[int, int, int] = (10, 120, 255)
    color_x: Tuple[int, int, int] = (255, 0, 0)
    color_spider: Tuple[int, int, int] = (50, 220, 50)
    color_human: Tuple[int, int, int] = (70, 130, 180)
    color_cat: Tuple[int, int, int] = (255, 180, 0)
    color_hypno: Tuple[int, int, int] = (180, 0, 255)
    color_anime: Tuple[int, int, int] = (100, 180, 255)
    # Human eye palette
    human_color_blue: Tuple[int, int, int] = (70, 130, 180)
    human_color_green: Tuple[int, int, int] = (60, 140, 80)
    human_color_hazel: Tuple[int, int, int] = (140, 110, 60)
    human_color_brown: Tuple[int, int, int] = (100, 60, 30)
    human_color_darkbrown: Tuple[int, int, int] = (50, 30, 20)


@dataclass
class PIDConfig:
    enabled: Optional[bool] = None
    mode: Optional[str] = None
    shadow_hz: Optional[int] = None
    kp: Optional[List[int]] = None
    ki: Optional[List[int]] = None
    kd: Optional[List[int]] = None
    kdalph: Optional[List[int]] = None


@dataclass
class IMPConfig:
    enabled: Optional[bool] = None
    mode: Optional[str] = None
    scale: Optional[int] = None
    jspring: Optional[List[int]] = None
    jdamp: Optional[List[int]] = None
    cspring: Optional[List[int]] = None
    cdamp: Optional[List[int]] = None
    jdb_cd: Optional[int] = None
    cdb_mm: Optional[float] = None


@dataclass
class ESTConfig:
    cmd_alpha_milli: Optional[int] = None
    meas_alpha_milli: Optional[int] = None
    meas_vel_alpha_milli: Optional[int] = None


@dataclass
class SafetyDisplayConfig:
    """Configuration for safety display thresholds (Pi-side visualization only).
    
    These thresholds control the color gradients for voltage/temperature
    displays on the engineering view and are separate from firmware safety limits.
    """
    # Voltage thresholds for display color gradient
    volt_min: float = 10.5     # Red (low) threshold
    volt_warn: float = 11.0    # Warning threshold (triggers low battery overlay)
    volt_max: float = 12.5     # Green (high) threshold
    # Temperature thresholds for display color gradient
    temp_min: float = 25.0     # Green (cool) threshold
    temp_max: float = 55.0     # Red (hot) threshold
    # Display options
    show_battery_icon: bool = True  # Show battery icon on display


@dataclass
class LowBatteryConfig:
    enabled: bool = True
    volt_critical: float = 10.0
    volt_recovery: float = 11.5
    filter_alpha: float = 0.1


@dataclass
class BehaviorConfig:
    enabled: bool = False
    obstacle_avoidance: bool = True
    cliff_detection: bool = True
    caught_foot_recovery: bool = True
    patrol: bool = False
    wall_follow: bool = False
    wall_side: str = 'left'
    wall_distance_mm: int = 200
    stop_distance_mm: int = 150
    slow_distance_mm: int = 300
    cliff_threshold_mm: int = 100
    snag_position_error_deg: float = 15.0
    snag_timeout_ms: int = 500
    recovery_lift_mm: float = 30.0
    max_recovery_attempts: int = 3
    patrol_duration_s: float = 60.0
    turn_interval_s: float = 10.0


@dataclass
class PointCloudConfig:
    enabled: bool = False
    port: int = 8765
    http_port: int = 8080
    stream_hz: float = 10.0
    accumulate: bool = True
    max_points: int = 50000
    voxel_mm: float = 20.0


@dataclass
class DashboardConfig:
    enabled: bool = True
    port: int = 8766
    stream_hz: float = 4.0


@dataclass
class AudioConfig:
    enabled: bool = True
    volume: float = 1.0
    device: str = "hw:2,0"
    sounds_dir: str = "assets/sounds"


@dataclass
class TTSConfig:
    enabled: bool = True
    engine: str = "piper"
    rate: int = 175
    gain_db: int = 0
    voice: str = "en_US-lessac-medium"
    pitch: int = 50
    piper_model: str = "~/piper/en_US-lessac-medium.onnx"
    cooldown_sec: float = 2.0


@dataclass
class ControllerConfig:
    """Master configuration container holding all subsystem configs."""
    verbose: bool = False
    serial: SerialConfig = field(default_factory=SerialConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    menu: MenuConfig = field(default_factory=MenuConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    leveling: LevelingConfig = field(default_factory=LevelingConfig)
    tof: ToFConfig = field(default_factory=ToFConfig)
    gait: GaitConfig = field(default_factory=GaitConfig)
    pounce: PounceConfig = field(default_factory=PounceConfig)
    eyes: EyeConfig = field(default_factory=EyeConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    imp: IMPConfig = field(default_factory=IMPConfig)
    est: ESTConfig = field(default_factory=ESTConfig)
    safety_display: SafetyDisplayConfig = field(default_factory=SafetyDisplayConfig)
    low_battery: LowBatteryConfig = field(default_factory=LowBatteryConfig)
    behavior: BehaviorConfig = field(default_factory=BehaviorConfig)
    pointcloud: PointCloudConfig = field(default_factory=PointCloudConfig)
    dashboard: DashboardConfig = field(default_factory=DashboardConfig)
    audio: AudioConfig = field(default_factory=AudioConfig)
    tts: TTSConfig = field(default_factory=TTSConfig)


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
def _parse_ini_triplet_int(s: Optional[str]) -> Optional[List[int]]:
    """Parse 'a/b/c' string into [int, int, int] list."""
    if s is None:
        return None
    try:
        parts = str(s).strip().split('/')
        if len(parts) != 3:
            return None
        return [int(parts[0]), int(parts[1]), int(parts[2])]
    except Exception:
        return None


def _fmt_triplet(tri: Optional[List[int]]) -> Optional[str]:
    """Format [int, int, int] list to 'a/b/c' string."""
    try:
        if tri is None or len(tri) != 3:
            return None
        return f"{int(tri[0])}/{int(tri[1])}/{int(tri[2])}"
    except Exception:
        return None


def _parse_rgb(s: str, default: Tuple[int, int, int]) -> Tuple[int, int, int]:
    """Parse 'R,G,B' string to tuple, clamping to 0-255."""
    try:
        parts = [int(x.strip()) for x in s.split(',')]
        if len(parts) == 3:
            return tuple(max(0, min(255, p)) for p in parts)
    except Exception:
        pass
    return default


# -----------------------------------------------------------------------------
# Load configuration
# -----------------------------------------------------------------------------
def load_config(config_path: Optional[str] = None) -> ControllerConfig:
    """Load configuration from controller.ini file.
    
    Returns a ControllerConfig dataclass with all values populated.
    Missing keys use defaults.
    """
    global _cfg, _cfg_path
    
    cfg = ControllerConfig()
    
    if config_path is None:
        config_path = _get_config_path()
    
    _cfg_path = config_path
    _cfg = configparser.ConfigParser()
    
    try:
        _cfg.read(config_path)
    except Exception as e:
        print(f"Config read error: {e}", end="\r\n")
        return cfg
    
    try:
        # UI section
        if 'ui' in _cfg:
            cfg.verbose = _cfg.getboolean('ui', 'verbose', fallback=cfg.verbose)
        
        # Serial section
        if 'serial' in _cfg:
            cfg.serial.error_threshold = _cfg.getint('serial', 'error_threshold', 
                                                      fallback=cfg.serial.error_threshold)
        
        # Telemetry section
        if 'telemetry' in _cfg:
            cfg.telemetry.grace_s = _cfg.getfloat('telemetry', 'grace_s', 
                                                   fallback=cfg.telemetry.grace_s)
            cfg.telemetry.retry_s = _cfg.getfloat('telemetry', 'retry_s', 
                                                   fallback=cfg.telemetry.retry_s)
        
        # Timing section
        if 'timing' in _cfg:
            cfg.timing.loop_target_ms = _cfg.getfloat('timing', 'loop_target_ms', 
                                                       fallback=cfg.timing.loop_target_ms)
            cfg.timing.teensy_loop_us = _cfg.getint('timing', 'teensy_loop_us', 
                                                     fallback=cfg.timing.teensy_loop_us)
            cfg.timing.telem_sync_fallback_ms = _cfg.getfloat('timing', 'telem_sync_fallback_ms', 
                                                               fallback=cfg.timing.telem_sync_fallback_ms)
            cfg.timing.teensy_reconnect_backoff_s = _cfg.getfloat('timing', 'teensy_reconnect_backoff_s', 
                                                                   fallback=cfg.timing.teensy_reconnect_backoff_s)
        
        # Display section
        if 'display' in _cfg:
            cfg.display.screen_refresh_ms = _cfg.getfloat('display', 'screen_refresh_ms', 
                                                           fallback=cfg.display.screen_refresh_ms)
            cfg.display.brightness = _cfg.getint('display', 'brightness', 
                                                  fallback=cfg.display.brightness)
            cfg.display.auto_disable_s = _cfg.getfloat('display', 'auto_disable_s', 
                                                        fallback=cfg.display.auto_disable_s)
            cfg.display.thread_enabled = _cfg.getboolean('display', 'thread_enabled', 
                                                          fallback=cfg.display.thread_enabled)
            cfg.display.thread_hz = _cfg.getfloat('display', 'thread_hz', 
                                                   fallback=cfg.display.thread_hz)
            cfg.display.blink_frame_divisor = _cfg.getint('display', 'blink_frame_divisor', 
                                                           fallback=cfg.display.blink_frame_divisor)
            cfg.display.engineering_lcars = _cfg.getboolean('display', 'engineering_lcars',
                                                          fallback=True)
            startup_delay = _cfg.getfloat('display', 'startup_delay_s', fallback=5.0)
            cfg.display.startup_delay_s = max(0.0, min(30.0, round(startup_delay * 2) / 2))

        # Menu section
        if 'menu' in _cfg:
            cfg.menu.theme = _cfg.getint('menu', 'theme', fallback=cfg.menu.theme)
            cfg.menu.palette = _cfg.getint('menu', 'palette', fallback=cfg.menu.palette)
        
        # IMU section
        if 'imu' in _cfg:
            cfg.imu.enabled = _cfg.getboolean('imu', 'enabled', fallback=cfg.imu.enabled)
            cfg.imu.hz = _cfg.getfloat('imu', 'hz', fallback=cfg.imu.hz)
            cfg.imu.bus = _cfg.getint('imu', 'bus', fallback=cfg.imu.bus)
            addr_str = _cfg.get('imu', 'address', fallback=None)
            if addr_str:
                try:
                    cfg.imu.address = int(addr_str, 0)
                except ValueError:
                    pass
            cfg.imu.use_game_rotation = _cfg.getboolean('imu', 'use_game_rotation', fallback=cfg.imu.use_game_rotation)
            cfg.imu.enable_accel = _cfg.getboolean('imu', 'enable_accel', fallback=cfg.imu.enable_accel)
            cfg.imu.enable_gyro = _cfg.getboolean('imu', 'enable_gyro', fallback=cfg.imu.enable_gyro)
            cfg.imu.enable_mag = _cfg.getboolean('imu', 'enable_mag', fallback=cfg.imu.enable_mag)
            
        # Leveling section
        if 'leveling' in _cfg:
            cfg.leveling.enabled = _cfg.getboolean('leveling', 'enabled', fallback=cfg.leveling.enabled)
            cfg.leveling.gain = _cfg.getfloat('leveling', 'gain', fallback=cfg.leveling.gain)
            cfg.leveling.max_correction_mm = _cfg.getfloat('leveling', 'max_correction_mm', fallback=cfg.leveling.max_correction_mm)
            cfg.leveling.filter_alpha = _cfg.getfloat('leveling', 'filter_alpha', fallback=cfg.leveling.filter_alpha)
            cfg.leveling.pitch_offset = _cfg.getfloat('leveling', 'pitch_offset', fallback=cfg.leveling.pitch_offset)
            cfg.leveling.roll_offset = _cfg.getfloat('leveling', 'roll_offset', fallback=cfg.leveling.roll_offset)
            cfg.leveling.tilt_limit_deg = _cfg.getfloat('leveling', 'tilt_limit_deg', fallback=cfg.leveling.tilt_limit_deg)
            cfg.leveling.lean_enabled = _cfg.getboolean('leveling', 'lean_enabled', fallback=cfg.leveling.lean_enabled)
            cfg.leveling.lean_max_deg = _cfg.getfloat('leveling', 'lean_max_deg', fallback=cfg.leveling.lean_max_deg)
            cfg.leveling.lean_filter_alpha = _cfg.getfloat('leveling', 'lean_filter_alpha', fallback=cfg.leveling.lean_filter_alpha)
            
        # ToF section
        if 'tof' in _cfg:
            cfg.tof.enabled = _cfg.getboolean('tof', 'enabled', fallback=cfg.tof.enabled)
            cfg.tof.hz = _cfg.getfloat('tof', 'hz', fallback=cfg.tof.hz)
            cfg.tof.bus = _cfg.getint('tof', 'bus', fallback=cfg.tof.bus)
            res_cfg = _cfg.getint('tof', 'resolution', fallback=8)
            cfg.tof.resolution = 64 if res_cfg >= 8 else 16
            
            sensors_str = _cfg.get('tof', 'sensors', fallback=None)
            if sensors_str:
                cfg.tof.sensors = []
                for pair in sensors_str.split(','):
                    pair = pair.strip()
                    if '=' in pair:
                        name, addr_str = pair.split('=', 1)
                        try:
                            addr = int(addr_str.strip(), 0)
                            cfg.tof.sensors.append((name.strip(), addr))
                        except ValueError:
                            pass
            
            cfg.tof.filter_mode = _cfg.get('tof', 'filter_mode', fallback=cfg.tof.filter_mode)
            cfg.tof.filter_alpha = _cfg.getfloat('tof', 'filter_alpha', fallback=cfg.tof.filter_alpha)
            cfg.tof.filter_sigma_threshold = _cfg.getint('tof', 'filter_sigma_threshold', fallback=cfg.tof.filter_sigma_threshold)
            cfg.tof.filter_outlier_mm = _cfg.getint('tof', 'filter_outlier_mm', fallback=cfg.tof.filter_outlier_mm)
        
        # Gait section
        if 'gait' in _cfg:
            cfg.gait.width_mm = _cfg.getfloat('gait', 'width_mm', fallback=cfg.gait.width_mm)
            cfg.gait.lift_mm = _cfg.getfloat('gait', 'lift_mm', fallback=cfg.gait.lift_mm)
            cfg.gait.width_min_mm = _cfg.getfloat('gait', 'width_min_mm', fallback=cfg.gait.width_min_mm)
            cfg.gait.width_max_mm = _cfg.getfloat('gait', 'width_max_mm', fallback=cfg.gait.width_max_mm)
            cfg.gait.lift_min_mm = _cfg.getfloat('gait', 'lift_min_mm', fallback=cfg.gait.lift_min_mm)
            cfg.gait.lift_max_mm = _cfg.getfloat('gait', 'lift_max_mm', fallback=cfg.gait.lift_max_mm)
            cfg.gait.cycle_ms = _cfg.getint('gait', 'cycle_ms', fallback=cfg.gait.cycle_ms)
            cfg.gait.step_len_mm = _cfg.getfloat('gait', 'step_len_mm', fallback=cfg.gait.step_len_mm)
            cfg.gait.base_y_mm = _cfg.getfloat('gait', 'base_y_mm', fallback=cfg.gait.base_y_mm)
            cfg.gait.max_step_len_mm = _cfg.getfloat('gait', 'max_step_len_mm', fallback=cfg.gait.max_step_len_mm)
            cfg.gait.overlap_pct = _cfg.getfloat('gait', 'overlap_pct', fallback=cfg.gait.overlap_pct)
            cfg.gait.smoothing_alpha = _cfg.getfloat('gait', 'smoothing_alpha', fallback=cfg.gait.smoothing_alpha)
            cfg.gait.send_divisor = _cfg.getint('gait', 'send_divisor', fallback=cfg.gait.send_divisor)
            cfg.gait.cmd_throttle_ms = _cfg.getfloat('gait', 'cmd_throttle_ms', fallback=cfg.gait.cmd_throttle_ms)
            cfg.gait.turn_max_deg_s = _cfg.getfloat('gait', 'turn_max_deg_s', fallback=cfg.gait.turn_max_deg_s)
            # Bezier
            cfg.gait.bezier_p1_height = _cfg.getfloat('gait', 'bezier_p1_height', fallback=cfg.gait.bezier_p1_height)
            cfg.gait.bezier_p1_overshoot = _cfg.getfloat('gait', 'bezier_p1_overshoot', fallback=cfg.gait.bezier_p1_overshoot)
            cfg.gait.bezier_p2_height = _cfg.getfloat('gait', 'bezier_p2_height', fallback=cfg.gait.bezier_p2_height)
            cfg.gait.bezier_p3_height = _cfg.getfloat('gait', 'bezier_p3_height', fallback=cfg.gait.bezier_p3_height)
            cfg.gait.bezier_p3_overshoot = _cfg.getfloat('gait', 'bezier_p3_overshoot', fallback=cfg.gait.bezier_p3_overshoot)
        
        # Pounce section
        if 'pounce' in _cfg:
            cfg.pounce.prep_ms = _cfg.getint('pounce', 'prep_ms', fallback=cfg.pounce.prep_ms)
            cfg.pounce.rear_ms = _cfg.getint('pounce', 'rear_ms', fallback=cfg.pounce.rear_ms)
            cfg.pounce.lunge_ms = _cfg.getint('pounce', 'lunge_ms', fallback=cfg.pounce.lunge_ms)
            cfg.pounce.recover_ms = _cfg.getint('pounce', 'recover_ms', fallback=cfg.pounce.recover_ms)
            cfg.pounce.back1_z_mm = _cfg.getfloat('pounce', 'back1_z_mm', fallback=cfg.pounce.back1_z_mm)
            cfg.pounce.back2_z_mm = _cfg.getfloat('pounce', 'back2_z_mm', fallback=cfg.pounce.back2_z_mm)
            cfg.pounce.push_z_mm = _cfg.getfloat('pounce', 'push_z_mm', fallback=cfg.pounce.push_z_mm)
            cfg.pounce.strike_z_mm = _cfg.getfloat('pounce', 'strike_z_mm', fallback=cfg.pounce.strike_z_mm)
            cfg.pounce.crouch_dy_mm = _cfg.getfloat('pounce', 'crouch_dy_mm', fallback=cfg.pounce.crouch_dy_mm)
            cfg.pounce.lift_dy_mm = _cfg.getfloat('pounce', 'lift_dy_mm', fallback=cfg.pounce.lift_dy_mm)
            cfg.pounce.front_z_mm = _cfg.getfloat('pounce', 'front_z_mm', fallback=cfg.pounce.front_z_mm)
        
        # Eyes section
        if 'eyes' in _cfg:
            cfg.eyes.spacing_offset = _cfg.getint('eyes', 'spacing_offset', fallback=cfg.eyes.spacing_offset)
            cfg.eyes.center_offset = _cfg.getint('eyes', 'center_offset', fallback=cfg.eyes.center_offset)
            cfg.eyes.vertical_offset = _cfg.getint('eyes', 'vertical_offset', fallback=cfg.eyes.vertical_offset)
            cfg.eyes.eyelid_angle = _cfg.getint('eyes', 'eyelid_angle', fallback=cfg.eyes.eyelid_angle)
            cfg.eyes.blink_percent_step = _cfg.getfloat('eyes', 'blink_percent_step', fallback=cfg.eyes.blink_percent_step)
            cfg.eyes.rotation = _cfg.getint('eyes', 'rotation', fallback=cfg.eyes.rotation)
            cfg.eyes.size_x = _cfg.getint('eyes', 'size_x', fallback=cfg.eyes.size_x)
            cfg.eyes.size_y = _cfg.getint('eyes', 'size_y', fallback=cfg.eyes.size_y)
            cfg.eyes.look_range_x = _cfg.getfloat('eyes', 'look_range_x', fallback=cfg.eyes.look_range_x)
            cfg.eyes.look_range_y = _cfg.getfloat('eyes', 'look_range_y', fallback=cfg.eyes.look_range_y)
            cfg.eyes.human_eye_spacing_pct = _cfg.getfloat('eyes', 'human_eye_spacing_pct', fallback=cfg.eyes.human_eye_spacing_pct)
            cfg.eyes.human_eye_size = _cfg.getint('eyes', 'human_eye_size', fallback=cfg.eyes.human_eye_size)
            cfg.eyes.human_eye_color = _cfg.getint('eyes', 'human_eye_color', fallback=cfg.eyes.human_eye_color)
            cfg.eyes.shape = _cfg.getint('eyes', 'shape', fallback=cfg.eyes.shape)
            cfg.eyes.crt_mode = _cfg.getboolean('eyes', 'crt_mode', fallback=cfg.eyes.crt_mode)
            # Eye colors
            cfg.eyes.color_ellipse = _parse_rgb(_cfg.get('eyes', 'color_ellipse', fallback='255,80,20'), cfg.eyes.color_ellipse)
            cfg.eyes.color_rectangle = _parse_rgb(_cfg.get('eyes', 'color_rectangle', fallback='255,255,255'), cfg.eyes.color_rectangle)
            cfg.eyes.color_roundrect = _parse_rgb(_cfg.get('eyes', 'color_roundrect', fallback='10,120,255'), cfg.eyes.color_roundrect)
            cfg.eyes.color_x = _parse_rgb(_cfg.get('eyes', 'color_x', fallback='255,0,0'), cfg.eyes.color_x)
            cfg.eyes.color_spider = _parse_rgb(_cfg.get('eyes', 'color_spider', fallback='50,220,50'), cfg.eyes.color_spider)
            cfg.eyes.color_human = _parse_rgb(_cfg.get('eyes', 'color_human', fallback='70,130,180'), cfg.eyes.color_human)
            cfg.eyes.color_cat = _parse_rgb(_cfg.get('eyes', 'color_cat', fallback='255,180,0'), cfg.eyes.color_cat)
            cfg.eyes.color_hypno = _parse_rgb(_cfg.get('eyes', 'color_hypno', fallback='180,0,255'), cfg.eyes.color_hypno)
            cfg.eyes.color_anime = _parse_rgb(_cfg.get('eyes', 'color_anime', fallback='100,180,255'), cfg.eyes.color_anime)
            # Human colors
            cfg.eyes.human_color_blue = _parse_rgb(_cfg.get('eyes', 'human_color_blue', fallback='70,130,180'), cfg.eyes.human_color_blue)
            cfg.eyes.human_color_green = _parse_rgb(_cfg.get('eyes', 'human_color_green', fallback='60,140,80'), cfg.eyes.human_color_green)
            cfg.eyes.human_color_hazel = _parse_rgb(_cfg.get('eyes', 'human_color_hazel', fallback='140,110,60'), cfg.eyes.human_color_hazel)
            cfg.eyes.human_color_brown = _parse_rgb(_cfg.get('eyes', 'human_color_brown', fallback='100,60,30'), cfg.eyes.human_color_brown)
            cfg.eyes.human_color_darkbrown = _parse_rgb(_cfg.get('eyes', 'human_color_darkbrown', fallback='50,30,20'), cfg.eyes.human_color_darkbrown)
        
        # Low Battery section
        if 'low_battery' in _cfg:
            cfg.low_battery.enabled = _cfg.getboolean('low_battery', 'enabled', fallback=cfg.low_battery.enabled)
            cfg.low_battery.volt_critical = _cfg.getfloat('low_battery', 'volt_critical', fallback=cfg.low_battery.volt_critical)
            cfg.low_battery.volt_recovery = _cfg.getfloat('low_battery', 'volt_recovery', fallback=cfg.low_battery.volt_recovery)
            cfg.low_battery.filter_alpha = _cfg.getfloat('low_battery', 'filter_alpha', fallback=cfg.low_battery.filter_alpha)

        # Autonomy / Behavior
        if 'behavior' in _cfg:
            cfg.behavior.enabled = _cfg.getboolean('behavior', 'enabled', fallback=cfg.behavior.enabled)
            cfg.behavior.obstacle_avoidance = _cfg.getboolean('behavior', 'obstacle_avoidance', fallback=cfg.behavior.obstacle_avoidance)
            cfg.behavior.cliff_detection = _cfg.getboolean('behavior', 'cliff_detection', fallback=cfg.behavior.cliff_detection)
            cfg.behavior.caught_foot_recovery = _cfg.getboolean('behavior', 'caught_foot_recovery', fallback=cfg.behavior.caught_foot_recovery)
            cfg.behavior.patrol = _cfg.getboolean('behavior', 'patrol', fallback=cfg.behavior.patrol)
            cfg.behavior.wall_follow = _cfg.getboolean('behavior', 'wall_follow', fallback=cfg.behavior.wall_follow)
            cfg.behavior.wall_side = _cfg.get('behavior', 'wall_side', fallback=cfg.behavior.wall_side).lower().strip()
            cfg.behavior.wall_distance_mm = _cfg.getint('behavior', 'wall_distance_mm', fallback=cfg.behavior.wall_distance_mm)
            cfg.behavior.stop_distance_mm = _cfg.getint('behavior', 'stop_distance_mm', fallback=cfg.behavior.stop_distance_mm)
            cfg.behavior.slow_distance_mm = _cfg.getint('behavior', 'slow_distance_mm', fallback=cfg.behavior.slow_distance_mm)
            cfg.behavior.cliff_threshold_mm = _cfg.getint('behavior', 'cliff_threshold_mm', fallback=cfg.behavior.cliff_threshold_mm)
            cfg.behavior.snag_position_error_deg = _cfg.getfloat('behavior', 'snag_position_error_deg', fallback=cfg.behavior.snag_position_error_deg)
            cfg.behavior.snag_timeout_ms = _cfg.getint('behavior', 'snag_timeout_ms', fallback=cfg.behavior.snag_timeout_ms)
            cfg.behavior.recovery_lift_mm = _cfg.getfloat('behavior', 'recovery_lift_mm', fallback=cfg.behavior.recovery_lift_mm)
            cfg.behavior.max_recovery_attempts = _cfg.getint('behavior', 'max_recovery_attempts', fallback=cfg.behavior.max_recovery_attempts)
            cfg.behavior.patrol_duration_s = _cfg.getfloat('behavior', 'patrol_duration_s', fallback=cfg.behavior.patrol_duration_s)
            cfg.behavior.turn_interval_s = _cfg.getfloat('behavior', 'turn_interval_s', fallback=cfg.behavior.turn_interval_s)

        # Point Cloud / SLAM
        if 'pointcloud' in _cfg:
            cfg.pointcloud.enabled = _cfg.getboolean('pointcloud', 'enabled', fallback=cfg.pointcloud.enabled)
            cfg.pointcloud.port = _cfg.getint('pointcloud', 'port', fallback=cfg.pointcloud.port)
            cfg.pointcloud.http_port = _cfg.getint('pointcloud', 'http_port', fallback=cfg.pointcloud.http_port)
            cfg.pointcloud.stream_hz = _cfg.getfloat('pointcloud', 'stream_hz', fallback=cfg.pointcloud.stream_hz)
            cfg.pointcloud.accumulate = _cfg.getboolean('pointcloud', 'accumulate', fallback=cfg.pointcloud.accumulate)
            cfg.pointcloud.max_points = _cfg.getint('pointcloud', 'max_points', fallback=cfg.pointcloud.max_points)
            cfg.pointcloud.voxel_mm = _cfg.getfloat('pointcloud', 'voxel_mm', fallback=cfg.pointcloud.voxel_mm)

        # Dashboard
        if 'dashboard' in _cfg:
            cfg.dashboard.enabled = _cfg.getboolean('dashboard', 'enabled', fallback=cfg.dashboard.enabled)
            cfg.dashboard.port = _cfg.getint('dashboard', 'port', fallback=cfg.dashboard.port)
            cfg.dashboard.stream_hz = _cfg.getfloat('dashboard', 'stream_hz', fallback=cfg.dashboard.stream_hz)

        # Audio
        if 'audio' in _cfg:
            cfg.audio.enabled = _cfg.getboolean('audio', 'enabled', fallback=cfg.audio.enabled)
            cfg.audio.volume = _cfg.getfloat('audio', 'volume', fallback=cfg.audio.volume)
            cfg.audio.device = _cfg.get('audio', 'device', fallback=cfg.audio.device)
            cfg.audio.sounds_dir = _cfg.get('audio', 'sounds_dir', fallback=cfg.audio.sounds_dir)

        # TTS
        if 'tts' in _cfg:
            cfg.tts.enabled = _cfg.getboolean('tts', 'enabled', fallback=cfg.tts.enabled)
            cfg.tts.engine = _cfg.get('tts', 'engine', fallback=cfg.tts.engine).lower().strip()
            cfg.tts.rate = _cfg.getint('tts', 'rate', fallback=cfg.tts.rate)
            cfg.tts.gain_db = _cfg.getint('tts', 'gain_db', fallback=cfg.tts.gain_db)
            cfg.tts.voice = _cfg.get('tts', 'voice', fallback=cfg.tts.voice)
            cfg.tts.pitch = _cfg.getint('tts', 'pitch', fallback=cfg.tts.pitch)
            cfg.tts.piper_model = os.path.expanduser(_cfg.get('tts', 'piper_model', fallback=cfg.tts.piper_model))
            cfg.tts.cooldown_sec = _cfg.getfloat('tts', 'cooldown_sec', fallback=cfg.tts.cooldown_sec)

        # PID section
        if 'pid' in _cfg:
            cfg.pid.enabled = _cfg.getboolean('pid', 'enabled', fallback=None)
            cfg.pid.mode = _cfg.get('pid', 'mode', fallback=None)
            cfg.pid.shadow_hz = _cfg.getint('pid', 'shadow_hz', fallback=None) if 'shadow_hz' in _cfg['pid'] else None
            cfg.pid.kp = _parse_ini_triplet_int(_cfg.get('pid', 'kp', fallback=None))
            cfg.pid.ki = _parse_ini_triplet_int(_cfg.get('pid', 'ki', fallback=None))
            cfg.pid.kd = _parse_ini_triplet_int(_cfg.get('pid', 'kd', fallback=None))
            cfg.pid.kdalph = _parse_ini_triplet_int(_cfg.get('pid', 'kdalph', fallback=None))
        
        # IMP section
        if 'imp' in _cfg:
            cfg.imp.enabled = _cfg.getboolean('imp', 'enabled', fallback=None)
            cfg.imp.mode = _cfg.get('imp', 'mode', fallback=None)
            cfg.imp.scale = _cfg.getint('imp', 'scale', fallback=None) if 'scale' in _cfg['imp'] else None
            cfg.imp.jspring = _parse_ini_triplet_int(_cfg.get('imp', 'jspring', fallback=None))
            cfg.imp.jdamp = _parse_ini_triplet_int(_cfg.get('imp', 'jdamp', fallback=None))
            cfg.imp.cspring = _parse_ini_triplet_int(_cfg.get('imp', 'cspring', fallback=None))
            cfg.imp.cdamp = _parse_ini_triplet_int(_cfg.get('imp', 'cdamp', fallback=None))
            cfg.imp.jdb_cd = _cfg.getint('imp', 'jdb_cd', fallback=None) if 'jdb_cd' in _cfg['imp'] else None
            try:
                cfg.imp.cdb_mm = _cfg.getfloat('imp', 'cdb_mm', fallback=None) if 'cdb_mm' in _cfg['imp'] else None
            except Exception:
                pass
        
        # EST section
        if 'est' in _cfg:
            cfg.est.cmd_alpha_milli = _cfg.getint('est', 'cmd_alpha_milli', fallback=None) if 'cmd_alpha_milli' in _cfg['est'] else None
            cfg.est.meas_alpha_milli = _cfg.getint('est', 'meas_alpha_milli', fallback=None) if 'meas_alpha_milli' in _cfg['est'] else None
            cfg.est.meas_vel_alpha_milli = _cfg.getint('est', 'meas_vel_alpha_milli', fallback=None) if 'meas_vel_alpha_milli' in _cfg['est'] else None
        
        # Safety display section (Pi-side visualization thresholds)
        if 'safety_display' in _cfg:
            cfg.safety_display.volt_min = _cfg.getfloat('safety_display', 'volt_min', 
                                                         fallback=cfg.safety_display.volt_min)
            cfg.safety_display.volt_warn = _cfg.getfloat('safety_display', 'volt_warn', 
                                                         fallback=cfg.safety_display.volt_warn)
            cfg.safety_display.volt_max = _cfg.getfloat('safety_display', 'volt_max', 
                                                         fallback=cfg.safety_display.volt_max)
            cfg.safety_display.temp_min = _cfg.getfloat('safety_display', 'temp_min', 
                                                         fallback=cfg.safety_display.temp_min)
            cfg.safety_display.temp_max = _cfg.getfloat('safety_display', 'temp_max', 
                                                         fallback=cfg.safety_display.temp_max)
            cfg.safety_display.show_battery_icon = _cfg.getboolean('safety_display', 'show_battery_icon',
                                                                    fallback=cfg.safety_display.show_battery_icon)
    
    except (configparser.Error, ValueError, KeyError) as e:
        print(f"Config parse error (using defaults): {e}", end="\r\n")
    
    return cfg


# -----------------------------------------------------------------------------
# Save functions
# -----------------------------------------------------------------------------
def save_gait_settings(width_mm: float, lift_mm: float, 
                       cycle_ms: Optional[int] = None, 
                       turn_max_deg_s: Optional[float] = None) -> bool:
    """Save gait parameters to controller.ini [gait] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'gait' not in _cfg:
            _cfg.add_section('gait')
        _cfg.set('gait', 'width_mm', f'{width_mm:.1f}')
        _cfg.set('gait', 'lift_mm', f'{lift_mm:.1f}')
        if cycle_ms is not None:
            _cfg.set('gait', 'cycle_ms', str(int(cycle_ms)))
        if turn_max_deg_s is not None:
            _cfg.set('gait', 'turn_max_deg_s', f'{turn_max_deg_s:.1f}')
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save gait settings: {e}", end="\r\n")
        return False


def save_pounce_settings(
    prep_ms: int,
    rear_ms: int,
    lunge_ms: int,
    recover_ms: int,
    back1_z_mm: float,
    back2_z_mm: float,
    push_z_mm: float,
    strike_z_mm: float,
    crouch_dy_mm: float,
    lift_dy_mm: float,
    front_z_mm: float,
) -> bool:
    """Save pounce parameters to controller.ini [pounce] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'pounce' not in _cfg:
            _cfg.add_section('pounce')
        _cfg.set('pounce', 'prep_ms', str(int(prep_ms)))
        _cfg.set('pounce', 'rear_ms', str(int(rear_ms)))
        _cfg.set('pounce', 'lunge_ms', str(int(lunge_ms)))
        _cfg.set('pounce', 'recover_ms', str(int(recover_ms)))
        _cfg.set('pounce', 'back1_z_mm', f"{float(back1_z_mm):.1f}")
        _cfg.set('pounce', 'back2_z_mm', f"{float(back2_z_mm):.1f}")
        _cfg.set('pounce', 'push_z_mm', f"{float(push_z_mm):.1f}")
        _cfg.set('pounce', 'strike_z_mm', f"{float(strike_z_mm):.1f}")
        _cfg.set('pounce', 'crouch_dy_mm', f"{float(crouch_dy_mm):.1f}")
        _cfg.set('pounce', 'lift_dy_mm', f"{float(lift_dy_mm):.1f}")
        _cfg.set('pounce', 'front_z_mm', f"{float(front_z_mm):.1f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save pounce settings: {e}", end="\r\n")
        return False


def save_eye_shape(shape: int) -> bool:
    """Save eye shape to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'shape', str(shape))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye shape: {e}", end="\r\n")
        return False


def save_human_eye_settings(size: Optional[int] = None, 
                            color_idx: Optional[int] = None) -> bool:
    """Save human eye settings to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        if size is not None:
            _cfg.set('eyes', 'human_eye_size', str(size))
        if color_idx is not None:
            _cfg.set('eyes', 'human_eye_color', str(color_idx))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save human eye settings: {e}", end="\r\n")
        return False


def save_eye_center_offset(offset: int) -> bool:
    """Save eye horizontal center offset to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'center_offset', str(offset))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye center offset: {e}", end="\r\n")
        return False


def save_eye_vertical_offset(offset: int) -> bool:
    """Save eye vertical center offset to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'vertical_offset', str(offset))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye vertical offset: {e}", end="\r\n")
        return False


def save_eye_crt_mode(enabled: bool) -> bool:
    """Save eye CRT effect mode to controller.ini [eyes] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'eyes' not in _cfg:
            _cfg.add_section('eyes')
        _cfg.set('eyes', 'crt_mode', str(enabled).lower())
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save eye CRT mode: {e}", end="\r\n")
        return False


def save_menu_settings(theme: Optional[int] = None, 
                       palette: Optional[int] = None) -> bool:
    """Save menu theme and palette to controller.ini [menu] section."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'menu' not in _cfg:
            _cfg.add_section('menu')
        if theme is not None:
            _cfg.set('menu', 'theme', str(theme))
        if palette is not None:
            _cfg.set('menu', 'palette', str(palette))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error) as e:
        print(f"Failed to save menu settings: {e}", end="\r\n")
        return False


def save_pid_settings(pid_state: dict) -> bool:
    """Persist PID settings to controller.ini [pid]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'pid' not in _cfg:
            _cfg.add_section('pid')
        if pid_state.get('enabled') is not None:
            _cfg.set('pid', 'enabled', 'true' if pid_state['enabled'] else 'false')
        if pid_state.get('mode') is not None:
            _cfg.set('pid', 'mode', str(pid_state['mode']))
        if pid_state.get('shadow_hz') is not None:
            _cfg.set('pid', 'shadow_hz', str(int(pid_state['shadow_hz'])))
        for k in ('kp', 'ki', 'kd', 'kdalph'):
            s = _fmt_triplet(pid_state.get(k))
            if s is not None:
                _cfg.set('pid', k, s)
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_imp_settings(imp_state: dict) -> bool:
    """Persist IMP settings to controller.ini [imp]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'imp' not in _cfg:
            _cfg.add_section('imp')
        if imp_state.get('enabled') is not None:
            _cfg.set('imp', 'enabled', 'true' if imp_state['enabled'] else 'false')
        if imp_state.get('mode') is not None:
            _cfg.set('imp', 'mode', str(imp_state['mode']))
        if imp_state.get('scale') is not None:
            _cfg.set('imp', 'scale', str(int(imp_state['scale'])))
        for k in ('jspring', 'jdamp', 'cspring', 'cdamp'):
            s = _fmt_triplet(imp_state.get(k))
            if s is not None:
                _cfg.set('imp', k, s)
        if imp_state.get('jdb_cd') is not None:
            _cfg.set('imp', 'jdb_cd', str(int(imp_state['jdb_cd'])))
        if imp_state.get('cdb_mm') is not None:
            _cfg.set('imp', 'cdb_mm', f"{float(imp_state['cdb_mm']):.3f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_est_settings(est_state: dict) -> bool:
    """Persist EST settings to controller.ini [est]."""
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'est' not in _cfg:
            _cfg.add_section('est')
        for k in ('cmd_alpha_milli', 'meas_alpha_milli', 'meas_vel_alpha_milli'):
            if est_state.get(k) is not None:
                _cfg.set('est', k, str(int(est_state[k])))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_tof_settings(tof_state: dict) -> bool:
    """Persist ToF settings to controller.ini [tof].
    
    Args:
        tof_state: Dictionary with keys:
            - enabled (bool): Enable ToF system
            - hz (float): Frame rate in Hz
            - bus (int): I2C bus number
            - resolution (int): 4 or 8 for grid size
            - sensors (list): List of (name, address) tuples
    
    Returns:
        True on success, False on error.
    """
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'tof' not in _cfg:
            _cfg.add_section('tof')
        if 'enabled' in tof_state:
            _cfg.set('tof', 'enabled', 'true' if tof_state['enabled'] else 'false')
        if 'hz' in tof_state:
            _cfg.set('tof', 'hz', str(float(tof_state['hz'])))
        if 'bus' in tof_state:
            _cfg.set('tof', 'bus', str(int(tof_state['bus'])))
        if 'resolution' in tof_state:
            _cfg.set('tof', 'resolution', str(int(tof_state['resolution'])))
        if 'sensors' in tof_state and tof_state['sensors']:
            # Format: "front=0x29,left=0x30"
            sensor_strs = [f"{name}=0x{addr:02X}" for name, addr in tof_state['sensors']]
            _cfg.set('tof', 'sensors', ','.join(sensor_strs))
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_safety_display_settings(safety_state: dict) -> bool:
    """Persist safety display thresholds to controller.ini [safety_display].
    
    Args:
        safety_state: Dictionary with keys:
            - volt_min (float): Low voltage threshold for display (red)
            - volt_warn (float): Low battery warning threshold for overlay
            - volt_max (float): High voltage threshold for display (green)
            - temp_min (float): Low temperature threshold for display (green)
            - temp_max (float): High temperature threshold for display (red)
    
    Returns:
        True on success, False on error.
    """
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'safety_display' not in _cfg:
            _cfg.add_section('safety_display')
        if 'volt_min' in safety_state:
            _cfg.set('safety_display', 'volt_min', f"{float(safety_state['volt_min']):.1f}")
        if 'volt_warn' in safety_state:
            _cfg.set('safety_display', 'volt_warn', f"{float(safety_state['volt_warn']):.1f}")
        if 'volt_max' in safety_state:
            _cfg.set('safety_display', 'volt_max', f"{float(safety_state['volt_max']):.1f}")
        if 'temp_min' in safety_state:
            _cfg.set('safety_display', 'temp_min', f"{float(safety_state['temp_min']):.1f}")
        if 'temp_max' in safety_state:
            _cfg.set('safety_display', 'temp_max', f"{float(safety_state['temp_max']):.1f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_behavior_settings(behavior_state: dict) -> bool:
    """Persist autonomy/behavior settings to controller.ini [behavior].
    
    Args:
        behavior_state: Dictionary with keys:
            - enabled (bool): Master autonomy enable
            - obstacle_avoidance (bool): Enable obstacle avoidance
            - cliff_detection (bool): Enable cliff detection
            - caught_foot_recovery (bool): Enable caught foot recovery
            - patrol (bool): Enable patrol behavior
            - stop_distance_mm (int): Obstacle stop distance
            - slow_distance_mm (int): Obstacle slow distance
            - cliff_threshold_mm (int): Cliff detection threshold
            - snag_position_error_deg (float): Snag detection error
            - snag_timeout_ms (int): Snag detection timeout
            - recovery_lift_mm (float): Recovery lift height
            - patrol_duration_s (float): Patrol duration
            - turn_interval_s (float): Patrol turn interval
    
    Returns:
        True on success, False on error.
    """
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'behavior' not in _cfg:
            _cfg.add_section('behavior')
        # Boolean settings
        for key in ('enabled', 'obstacle_avoidance', 'cliff_detection', 
                    'caught_foot_recovery', 'patrol'):
            if key in behavior_state:
                _cfg.set('behavior', key, 'true' if behavior_state[key] else 'false')
        # Integer settings
        for key in ('stop_distance_mm', 'slow_distance_mm', 'cliff_threshold_mm', 
                    'snag_timeout_ms', 'max_recovery_attempts'):
            if key in behavior_state:
                _cfg.set('behavior', key, str(int(behavior_state[key])))
        # Float settings
        for key in ('snag_position_error_deg', 'recovery_lift_mm', 
                    'patrol_duration_s', 'turn_interval_s'):
            if key in behavior_state:
                _cfg.set('behavior', key, f"{float(behavior_state[key]):.1f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


def save_low_battery_settings(low_batt_state: dict) -> bool:
    """Persist low battery protection settings to controller.ini [low_battery].
    
    Args:
        low_batt_state: Dictionary with keys:
            - enabled (bool): Enable low battery protection
            - volt_critical (float): Voltage threshold for TUCK+DISABLE
            - volt_recovery (float): Voltage to clear protection latch
            - filter_alpha (float): Low-pass filter smoothing (0.01-1.0)
    
    Returns:
        True on success, False on error.
    """
    global _cfg, _cfg_path
    if _cfg is None or _cfg_path is None:
        return False
    try:
        if 'low_battery' not in _cfg:
            _cfg.add_section('low_battery')
        if 'enabled' in low_batt_state:
            _cfg.set('low_battery', 'enabled', 'true' if low_batt_state['enabled'] else 'false')
        if 'volt_critical' in low_batt_state:
            _cfg.set('low_battery', 'volt_critical', f"{float(low_batt_state['volt_critical']):.1f}")
        if 'volt_recovery' in low_batt_state:
            _cfg.set('low_battery', 'volt_recovery', f"{float(low_batt_state['volt_recovery']):.1f}")
        if 'filter_alpha' in low_batt_state:
            _cfg.set('low_battery', 'filter_alpha', f"{float(low_batt_state['filter_alpha']):.3f}")
        with open(_cfg_path, 'w') as f:
            _cfg.write(f)
        return True
    except (IOError, OSError, configparser.Error, ValueError, TypeError):
        return False


# -----------------------------------------------------------------------------
# Module exports
# -----------------------------------------------------------------------------
__all__ = [
    # Config loading
    'load_config', 'ControllerConfig',
    # Config state management
    'set_config_state',
    # Dataclasses
    'SerialConfig', 'TelemetryConfig', 'GaitConfig', 'EyeConfig',
    'DisplayConfig', 'PounceConfig', 'TimingConfig', 'SafetyDisplayConfig',
    'ImuConfig', 'LevelingConfig', 'ToFConfig', 'LowBatteryConfig',
    'BehaviorConfig', 'PointCloudConfig', 'DashboardConfig', 'AudioConfig', 'TTSConfig',
    'MenuConfig', 'PIDConfig', 'IMPConfig', 'ESTConfig',
    # Save functions
    'save_gait_settings', 'save_pounce_settings',
    'save_eye_shape', 'save_human_eye_settings', 'save_eye_vertical_offset',
    'save_eye_crt_mode', 'save_eye_center_offset',
    'save_menu_settings',
    'save_pid_settings', 'save_imp_settings', 'save_est_settings', 'save_tof_settings',
    'save_safety_display_settings', 'save_behavior_settings', 'save_low_battery_settings',
]

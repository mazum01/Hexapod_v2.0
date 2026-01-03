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
    human_spacing_pct: float = 0.25
    human_size: int = 33
    human_color_idx: int = 0
    shape: int = 2  # ROUNDRECTANGLE default
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


@dataclass
class ControllerConfig:
    """Master configuration container holding all subsystem configs."""
    verbose: bool = False
    serial: SerialConfig = field(default_factory=SerialConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    menu: MenuConfig = field(default_factory=MenuConfig)
    gait: GaitConfig = field(default_factory=GaitConfig)
    pounce: PounceConfig = field(default_factory=PounceConfig)
    eyes: EyeConfig = field(default_factory=EyeConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    imp: IMPConfig = field(default_factory=IMPConfig)
    est: ESTConfig = field(default_factory=ESTConfig)
    safety_display: SafetyDisplayConfig = field(default_factory=SafetyDisplayConfig)


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
        
        # Menu section
        if 'menu' in _cfg:
            cfg.menu.theme = _cfg.getint('menu', 'theme', fallback=cfg.menu.theme)
            cfg.menu.palette = _cfg.getint('menu', 'palette', fallback=cfg.menu.palette)
        
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
            cfg.eyes.human_spacing_pct = _cfg.getfloat('eyes', 'human_eye_spacing_pct', fallback=cfg.eyes.human_spacing_pct)
            cfg.eyes.human_size = _cfg.getint('eyes', 'human_eye_size', fallback=cfg.eyes.human_size)
            cfg.eyes.human_color_idx = _cfg.getint('eyes', 'human_eye_color', fallback=cfg.eyes.human_color_idx)
            cfg.eyes.shape = _cfg.getint('eyes', 'shape', fallback=cfg.eyes.shape)
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
    # Config state management
    'set_config_state',
    # Dataclasses
    'SerialConfig', 'TelemetryConfig', 'GaitConfig', 'EyeConfig',
    'DisplayConfig', 'PounceConfig', 'TimingConfig', 'SafetyDisplayConfig',
    # Save functions
    'save_gait_settings', 'save_pounce_settings',
    'save_eye_shape', 'save_eye_color', 'save_eye_spacing', 'save_eye_vertical_offset',
    'save_eye_crt_mode', 'save_eye_center_offset',
    'save_menu_settings',
    'save_pid_settings', 'save_imp_settings', 'save_est_settings', 'save_tof_settings',
    'save_safety_display_settings', 'save_behavior_settings', 'save_low_battery_settings',
]

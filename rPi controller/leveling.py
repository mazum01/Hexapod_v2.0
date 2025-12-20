"""
leveling.py – Body leveling module for MARS Hexapod

Applies Z-axis corrections to foot targets based on IMU pitch/roll data
to keep the body level relative to gravity. Also provides tilt safety
thresholds to trigger protective actions when orientation exceeds limits.

Theory of Operation:
  For each leg at body-frame position (x_hip, 0, z_hip), the Z correction
  needed to compensate for body tilt is:
    Δz = x_hip × sin(pitch) + z_hip × sin(roll)
  
  Where:
    - pitch = rotation about the lateral (Y) axis (nose up/down)
    - roll  = rotation about the longitudinal (X) axis (left/right tilt)
    - x_hip = leg's fore-aft offset from body center (+ = front)
    - z_hip = leg's lateral offset from body center (+ = left)
  
  Note: LEG_HIP_X in gait_engine is lateral offset, LEG_HIP_Z is fore-aft.

Phase 2 Enhancement (planned):
  Center of mass shift compensation could adjust targets when legs extend
  differentially. Currently not implemented.

Author: MARS Project
Created: 2024-12
"""

import math
from dataclasses import dataclass
from typing import Tuple, List, Callable, Optional

# Leg indices (matches gait_engine.py)
LEG_LF, LEG_LM, LEG_LR = 0, 1, 2
LEG_RF, LEG_RM, LEG_RR = 3, 4, 5

# Leg hip positions in body frame (mm)
# X = lateral offset (+ = right), Z = fore-aft (+ = front)
# From gait_engine.py: LEG_HIP_X = [-80, -100, -80, 80, 100, 80]
#                      LEG_HIP_Z = [100, 0, -100, 100, 0, -100]
LEG_HIP_X = [-80, -100, -80, 80, 100, 80]  # lateral: left legs negative
LEG_HIP_Z = [100, 0, -100, 100, 0, -100]   # fore-aft: front positive


@dataclass
class LevelingConfig:
    """Configuration parameters for body leveling."""
    enabled: bool = False
    gain: float = 1.0               # scale factor for corrections (0-2)
    max_correction_mm: float = 30.0  # clamp per-leg Z delta
    filter_alpha: float = 0.15       # EMA filter (lower = smoother)
    pitch_offset_deg: float = 0.0    # pitch bias correction
    roll_offset_deg: float = 0.0     # roll bias correction
    tilt_limit_deg: float = 25.0     # safety threshold
    
    @classmethod
    def from_config(cls, cfg) -> 'LevelingConfig':
        """Load from ConfigParser [leveling] section."""
        return cls(
            enabled=cfg.getboolean('leveling', 'enabled', fallback=False),
            gain=cfg.getfloat('leveling', 'gain', fallback=1.0),
            max_correction_mm=cfg.getfloat('leveling', 'max_correction_mm', fallback=30.0),
            filter_alpha=cfg.getfloat('leveling', 'filter_alpha', fallback=0.15),
            pitch_offset_deg=cfg.getfloat('leveling', 'pitch_offset', fallback=0.0),
            roll_offset_deg=cfg.getfloat('leveling', 'roll_offset', fallback=0.0),
            tilt_limit_deg=cfg.getfloat('leveling', 'tilt_limit_deg', fallback=25.0),
        )


class LevelingState:
    """Runtime state for leveling with EMA-filtered orientation."""
    
    def __init__(self, config: LevelingConfig):
        self.config = config
        self._filtered_pitch_deg: float = 0.0
        self._filtered_roll_deg: float = 0.0
        self._last_corrections: List[float] = [0.0] * 6  # per-leg Z delta
        self._tilt_safety_triggered: bool = False
        self._tilt_safety_callback: Optional[Callable[[float, float], None]] = None
    
    def set_tilt_safety_callback(self, callback: Callable[[float, float], None]) -> None:
        """Set callback(pitch_deg, roll_deg) invoked on tilt threshold breach."""
        self._tilt_safety_callback = callback
    
    def update(self, raw_pitch_deg: float, raw_roll_deg: float) -> bool:
        """Update filtered orientation. Returns True if tilt is within safe limits."""
        cfg = self.config
        
        # Apply offset bias corrections
        pitch = raw_pitch_deg - cfg.pitch_offset_deg
        roll = raw_roll_deg - cfg.roll_offset_deg
        
        # EMA filter
        alpha = cfg.filter_alpha
        self._filtered_pitch_deg = alpha * pitch + (1 - alpha) * self._filtered_pitch_deg
        self._filtered_roll_deg = alpha * roll + (1 - alpha) * self._filtered_roll_deg
        
        # Check tilt safety threshold
        if abs(self._filtered_pitch_deg) > cfg.tilt_limit_deg or \
           abs(self._filtered_roll_deg) > cfg.tilt_limit_deg:
            if not self._tilt_safety_triggered:
                self._tilt_safety_triggered = True
                if self._tilt_safety_callback is not None:
                    self._tilt_safety_callback(self._filtered_pitch_deg, self._filtered_roll_deg)
            return False
        else:
            self._tilt_safety_triggered = False
            return True
    
    def get_filtered_orientation(self) -> Tuple[float, float]:
        """Return (filtered_pitch_deg, filtered_roll_deg)."""
        return (self._filtered_pitch_deg, self._filtered_roll_deg)
    
    def is_tilt_safe(self) -> bool:
        """Return True if current tilt is within safe limits."""
        cfg = self.config
        return abs(self._filtered_pitch_deg) <= cfg.tilt_limit_deg and \
               abs(self._filtered_roll_deg) <= cfg.tilt_limit_deg
    
    def compute_corrections(self) -> List[float]:
        """Compute per-leg Z corrections (mm) for body leveling.
        
        Returns list of 6 Z deltas [LF, LM, LR, RF, RM, RR].
        Positive delta = extend leg (lower foot relative to body).
        """
        if not self.config.enabled:
            self._last_corrections = [0.0] * 6
            return self._last_corrections
        
        cfg = self.config
        pitch_rad = math.radians(self._filtered_pitch_deg)
        roll_rad = math.radians(self._filtered_roll_deg)
        
        corrections = []
        for i in range(6):
            # Hip positions: z_hip = fore-aft, x_hip = lateral
            z_hip = LEG_HIP_Z[i]  # fore-aft offset (front positive)
            x_hip = LEG_HIP_X[i]  # lateral offset (right positive)
            
            # Z correction to counteract tilt:
            # - Pitch (nose up): front legs need to extend (positive delta)
            # - Roll (right down): right legs need to extend (positive delta)
            dz = z_hip * math.sin(pitch_rad) + x_hip * math.sin(roll_rad)
            
            # Apply gain and clamp
            dz *= cfg.gain
            dz = max(-cfg.max_correction_mm, min(cfg.max_correction_mm, dz))
            corrections.append(dz)
        
        self._last_corrections = corrections
        return corrections
    
    def get_last_corrections(self) -> List[float]:
        """Return the last computed corrections without recomputing."""
        return self._last_corrections
    
    def reset(self) -> None:
        """Reset filtered state and corrections."""
        self._filtered_pitch_deg = 0.0
        self._filtered_roll_deg = 0.0
        self._last_corrections = [0.0] * 6
        self._tilt_safety_triggered = False


def apply_leveling_to_feet(positions: List[float], corrections: List[float]) -> List[float]:
    """Apply Z corrections to a FEET position list.
    
    Args:
        positions: 18 floats [x0,y0,z0, x1,y1,z1, ..., x5,y5,z5]
        corrections: 6 floats [dz0, dz1, ..., dz5] per-leg Z deltas
    
    Returns:
        New list with Z values adjusted: z_new = z_old + dz
    """
    if len(positions) != 18 or len(corrections) != 6:
        return positions  # invalid input, return unchanged
    
    result = positions[:]
    for leg in range(6):
        z_idx = leg * 3 + 2  # Z is at index 2, 5, 8, 11, 14, 17
        result[z_idx] = positions[z_idx] + corrections[leg]
    return result


def build_corrected_feet_cmd(feet_cmd: bytes, corrections: List[float]) -> bytes:
    """Parse FEET command, apply corrections, rebuild command bytes.
    
    Args:
        feet_cmd: Original FEET command bytes (e.g., b'FEET x0 y0 z0 ...')
        corrections: 6 floats [dz0, dz1, ..., dz5] per-leg Z deltas
    
    Returns:
        Corrected FEET command bytes.
    """
    try:
        text = feet_cmd.decode('ascii').strip()
        if not text.startswith('FEET '):
            return feet_cmd
        parts = text[5:].split()
        if len(parts) != 18:
            return feet_cmd
        
        positions = [float(p) for p in parts]
        corrected = apply_leveling_to_feet(positions, corrections)
        
        # Rebuild command
        parts_out = []
        for p in corrected:
            parts_out.append(f"{p:.1f}")
        return ('FEET ' + ' '.join(parts_out)).encode('ascii')
    except Exception:
        return feet_cmd


# Global singleton for easy access from controller
_leveling_state: Optional[LevelingState] = None


def init_leveling(config: LevelingConfig) -> LevelingState:
    """Initialize the global leveling state singleton."""
    global _leveling_state
    _leveling_state = LevelingState(config)
    return _leveling_state


def get_leveling_state() -> Optional[LevelingState]:
    """Return the global leveling state, or None if not initialized."""
    return _leveling_state

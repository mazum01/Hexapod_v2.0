"""
display_thread.py — Display rendering and LCD update routines for MARS controller.

Extracted from controller.py as part of Phase 5 modularization (2025-06-18).
Contains DisplayThread class, UpdateDisplay function, and display helpers.

Change log:
  2025-12-27: LCARS engineering view enhancements.
              - Uses config palette (Classic/Nemesis/LowerDecks/PADD) from menu.palette setting
              - Larger fonts (10pt/12pt), black text for better readability
              - Added all 4 LCARS palettes: LCARS_CLASSIC, LCARS_NEMESIS, LCARS_LOWER_DECKS, LCARS_PADD
  2025-12-27: Added LCARS theme for engineering display mode.
              - draw_lcars_engineering_view(): Full LCARS-styled layout with swept corners, pill buttons
              - LCARS_COLORS palette: orange, gold, peach, violet, ice
              - Configurable via engineering_lcars in [display] config
  2025-12-27: Added phone-style battery icon for EYES display mode.
              - draw_battery_icon(): Top-right corner, color-coded fill (green/yellow/red)
              - Configurable via show_battery_icon in [safety_display] config
              - Shows voltage text below icon; semi-transparent for minimal distraction
  2025-12-24: Fixed eye flicker/tearing:
              - Added 40ms minimum SPI write interval (25fps max) to prevent frame tearing
              - Removed IMU changes from eye force_update (IMU only affects engineering view)
              - Increased IMU change threshold from 0.5° to 2.0°
  2025-12-24: ToF hold-last-valid: Smooths intermittent invalid readings (e.g., glass reflections).
              - Holds last valid distance for up to 30 frames before going gray
              - Stale readings are progressively dimmed; "~" prefix on distance text
              - reset_tof_hold_buffers() clears state on disconnect or config change
  2025-12-23: Added hexapod silhouette heat map for servo temperature visualization.
              - draw_hexapod_heatmap(): Top-down view with 6 legs × 3 color-coded segments
              - draw_voltage_bar(): Horizontal voltage indicator with color scale
              - Updated draw_engineering_view() layout: IMU text header + hex heat map + ToF + voltage bar
"""

from __future__ import annotations
import os
import time
import math
import threading
from enum import IntEnum
from typing import Optional, Any, List, Tuple

import numpy as np
import cv2 as cv
from PIL import Image, ImageDraw, ImageFont

# Import telemetry for structured data access
from telemetry import IDX_GAIT, getGait, get_safety_overlay_text


# -----------------------------------------------------------------------------
# Mirror Display Buffer (for main-thread cv2.imshow)
# -----------------------------------------------------------------------------
# cv2.imshow must be called from the main thread on Linux/X11.
# The display thread stores the prepared image here; main loop retrieves it.
_mirror_lock = threading.Lock()
_mirror_image = None      # numpy array (BGR) ready for cv2.imshow, or None
_mirror_active = False    # True when mirror should be displayed
_mirror_menu = None       # Reference to MarsMenu for keyboard handling


# -----------------------------------------------------------------------------
# Display Mode Enum
# -----------------------------------------------------------------------------

class DisplayMode(IntEnum):
    """Display mode for the main screen (cycled by Start button)."""
    EYES = 0         # Main eyes animation display
    ENGINEERING = 1  # Engineering view: IMU + ToF visualization
    MENU = 2         # MarsMenu system
    COUNT = 3        # Number of modes (for cycling)


# -----------------------------------------------------------------------------
# Font Cache
# -----------------------------------------------------------------------------

FONT_PATH = "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf"
_font_cache = {}


def get_font(size: int) -> ImageFont.FreeTypeFont:
    """Get a cached font of the specified size."""
    try:
        key = int(size)
        f = _font_cache.get(key)
        if f is None:
            f = ImageFont.truetype(FONT_PATH, key)
            _font_cache[key] = f
        return f
    except (OSError, IOError):
        return ImageFont.load_default()


# -----------------------------------------------------------------------------
# Color Helpers
# -----------------------------------------------------------------------------

# Default color palettes
POWER_COLOR_PALETTE = [(255, 50, 50), (50, 255, 50)]
TEMPERATURE_COLOR_PALETTE = [(0, 191, 255), (0, 255, 0), (255, 0, 0)]


def getColor(min_val: float, max_val: float, value: float, 
             color_palette: List[Tuple[int, int, int]]) -> Tuple[int, int, int]:
    """Returns an interpolated color based on the value and the color palette."""
    if value < min_val:
        return color_palette[0]
    elif value > max_val:
        return color_palette[len(color_palette) - 1]
    else:
        scaled = (value - min_val) / (max_val - min_val) * (len(color_palette) - 1)
        lower_idx = max(0, int(np.floor(scaled)))
        upper_idx = min(len(color_palette) - 1, lower_idx + 1)
        if lower_idx == upper_idx:
            return color_palette[lower_idx]
        ratio = scaled - lower_idx
        color1 = np.array(color_palette[lower_idx])
        color2 = np.array(color_palette[upper_idx])
        interp_color = (1 - ratio) * color1 + ratio * color2
        return tuple(interp_color.astype(int))


# -----------------------------------------------------------------------------
# Robot Geometry Constants (from firmware robot_config.h)
# -----------------------------------------------------------------------------

# Link lengths in mm
COXA_LENGTH_MM = 41.70
FEMUR_LENGTH_MM = 80.00
TIBIA_LENGTH_MM = 133.78

# Home position in centidegrees (servo center = 120°)
HOME_CD = 12000  # 120.00 degrees

# Coxa origin offsets in body frame (X, Z) for each leg
# Order: LF, LM, LR, RF, RM, RR
COXA_OFFSET = [
    (-65.60,  88.16),   # LF
    (-86.00,   0.00),   # LM
    (-65.60, -88.16),   # LR
    ( 65.60,  88.16),   # RF
    ( 86.00,   0.00),   # RM
    ( 65.60, -88.16),   # RR
]

# Leg base angles (degrees, used as yaw offset from +Z axis)
# These define the direction each leg points outward from the body
# Note: In FK, cos(yaw) -> Z, sin(yaw) -> X, so 0° = forward, 90° = right
LEG_BASE_ANGLES = [315.0, 270.0, 225.0, 45.0, 90.0, 135.0]  # LF, LM, LR, RF, RM, RR


# -----------------------------------------------------------------------------
# Forward Kinematics (matches firmware fk_leg_body())
# -----------------------------------------------------------------------------

def fk_leg_points(
    leg_index: int,
    coxa_deg: float,
    femur_deg: float,
    tibia_deg: float,
) -> List[Tuple[float, float, float]]:
    """Compute 3D positions of leg joints using forward kinematics.
    
    This matches the firmware's fk_leg_body() function. The input angles are
    absolute servo positions in degrees. We convert to relative angles from
    the home position (120°) before computing FK.
    
    Coordinate system (body frame):
    - X: Right (+) / Left (-)
    - Y: Up (+) / Down (-)  
    - Z: Forward (+) / Backward (-)
    
    Args:
        leg_index: Leg index 0-5 (LF, LM, LR, RF, RM, RR)
        coxa_deg: Coxa joint angle in degrees (absolute servo position)
        femur_deg: Femur joint angle in degrees (absolute servo position)
        tibia_deg: Tibia joint angle in degrees (absolute servo position)
    
    Returns:
        List of 4 (x, y, z) tuples: [coxa_origin, femur_origin, tibia_origin, foot]
    """
    # Convert absolute degrees to centidegrees, then to relative from home
    home_deg = HOME_CD / 100.0  # 120.0 degrees
    coxa_rel = coxa_deg - home_deg
    femur_rel = femur_deg - home_deg
    tibia_rel = tibia_deg - home_deg
    
    # Left-side legs (0,1,2) have mirrored servo mounting - flip femur and tibia direction
    is_left = leg_index < 3
    if is_left:
        femur_rel = -femur_rel
        tibia_rel = -tibia_rel
    
    # Convert to radians
    # Coxa yaw: coxa_rel=0 means pointing in leg's base direction
    # LEG_BASE_ANGLES defines where each leg points (e.g., RF=45° from +Z toward +X)
    # We need to convert from body frame where +Z=forward to screen coords
    leg_base_rad = math.radians(LEG_BASE_ANGLES[leg_index])
    yaw = leg_base_rad + math.radians(coxa_rel)  # total yaw from +Z axis
    
    # Femur has 90° offset: at home (rel=0), femur points horizontal (outward)
    # So we add 90° to convert from "home=horizontal" to trig convention
    alpha = math.radians(femur_rel + 90.0)   # femur pitch (with 90° offset)
    # Tibia rotation is reversed relative to femur
    beta = math.radians(-tibia_rel)    # tibia relative pitch (negated)
    
    # Coxa origin in body frame
    coxa_origin_x, coxa_origin_z = COXA_OFFSET[leg_index]
    coxa_origin = (coxa_origin_x, 0.0, coxa_origin_z)
    
    # Link lengths
    a = FEMUR_LENGTH_MM
    b = TIBIA_LENGTH_MM
    
    # Planar projection in leg's sagittal plane
    # alpha=90° at home means: sin(90)=1 (horizontal), cos(90)=0 (no vertical)
    # Y is vertical: up is +Y, robot foot down → negative
    y_foot = -(a * math.cos(alpha) + b * math.cos(alpha + beta))
    # R is horizontal (sagittal plane) radius from femur base
    R = a * math.sin(alpha) + b * math.sin(alpha + beta)
    
    # Add coxa standoff
    rproj = COXA_LENGTH_MM + R
    
    # Rotate by yaw to get local X,Z from coxa origin
    x_local = rproj * math.sin(yaw)
    z_local = rproj * math.cos(yaw)
    
    # Foot position in body frame
    foot_x = coxa_origin_x + x_local
    foot_y = y_foot
    foot_z = coxa_origin_z + z_local
    foot = (foot_x, foot_y, foot_z)
    
    # Now compute intermediate joint positions for visualization
    # Femur origin: end of coxa (coxa length from coxa origin in yaw direction)
    femur_x = coxa_origin_x + COXA_LENGTH_MM * math.sin(yaw)
    femur_z = coxa_origin_z + COXA_LENGTH_MM * math.cos(yaw)
    femur_y = 0.0  # Coxa is horizontal
    femur_origin = (femur_x, femur_y, femur_z)
    
    # Tibia origin: end of femur
    # Femur extends from femur_origin: horizontal component in yaw direction, vertical by alpha
    femur_y_contrib = -(a * math.cos(alpha))  # vertical (negative = down)
    femur_r_contrib = a * math.sin(alpha)     # horizontal extension
    
    tibia_x = femur_x + femur_r_contrib * math.sin(yaw)
    tibia_z = femur_z + femur_r_contrib * math.cos(yaw)
    tibia_y = femur_y + femur_y_contrib
    tibia_origin = (tibia_x, tibia_y, tibia_z)
    
    return [coxa_origin, femur_origin, tibia_origin, foot]


# -----------------------------------------------------------------------------
# 3D Projection
# -----------------------------------------------------------------------------

def project_3d_to_2d(
    points: List[Tuple[float, float, float]],
    azimuth_deg: float,
    elevation_deg: float,
    center_x: float,
    center_y: float,
    scale: float,
) -> List[Tuple[float, float, float]]:
    """Project 3D points to 2D screen coordinates with depth.
    
    Uses orthographic projection with rotation. The camera looks at the origin
    from a position defined by azimuth and elevation angles.
    
    Args:
        points: List of (x, y, z) 3D coordinates in body frame
        azimuth_deg: Horizontal rotation angle (0 = looking from +X, 90 = from +Z)
        elevation_deg: Vertical angle (0 = horizontal, 90 = looking straight down)
        center_x, center_y: Screen center for projection
        scale: Scale factor (mm to pixels)
    
    Returns:
        List of (screen_x, screen_y, depth) tuples. Depth is for sorting (higher = farther).
    """
    az_rad = math.radians(azimuth_deg)
    el_rad = math.radians(elevation_deg)
    
    # Rotation matrices for camera orientation
    cos_az, sin_az = math.cos(az_rad), math.sin(az_rad)
    cos_el, sin_el = math.cos(el_rad), math.sin(el_rad)
    
    result = []
    for x, y, z in points:
        # Rotate around Y axis (azimuth)
        x1 = x * cos_az - z * sin_az
        z1 = x * sin_az + z * cos_az
        y1 = y
        
        # Rotate around X axis (elevation)
        y2 = y1 * cos_el - z1 * sin_el
        z2 = y1 * sin_el + z1 * cos_el
        x2 = x1
        
        # Project to screen (orthographic: just drop Z, use X and Y)
        # X maps to screen X, Y maps to screen Y (inverted because screen Y is down)
        # Negate X to mirror display (so right side of robot shows on right side of screen)
        screen_x = center_x - x2 * scale
        screen_y = center_y - y2 * scale  # Invert Y for screen coordinates
        depth = z2  # Depth for back-to-front sorting
        
        result.append((screen_x, screen_y, depth))
    
    return result


# -----------------------------------------------------------------------------
# Frame Change Detection
# -----------------------------------------------------------------------------

_last_frame_hash = None
_last_spi_write_time = 0.0  # monotonic timestamp of last SPI write
MIN_SPI_WRITE_INTERVAL_S = 0.040  # Minimum 40ms (25fps max) between SPI writes to prevent tearing


# -----------------------------------------------------------------------------
# ToF "Hold Last Valid" Buffers
# -----------------------------------------------------------------------------
# Keep last valid distance for each cell to smooth over intermittent invalid readings
# (e.g., glass reflections). Staleness counter tracks how many frames since last valid.

_tof_held_distances: List[int] = [0] * 64  # Last valid distance per cell
_tof_staleness: List[int] = [999] * 64     # Frames since last valid (0 = fresh, 999 = never valid)
TOF_MAX_HOLD_FRAMES = 30                    # Max frames to hold a stale reading before graying out
TOF_STALE_DIM_FACTOR = 0.6                  # Dim factor for stale readings (0.0 = black, 1.0 = full)


def reset_tof_hold_buffers():
    """Reset ToF hold buffers (call when sensor disconnects or config changes)."""
    global _tof_held_distances, _tof_staleness
    _tof_held_distances = [0] * 64
    _tof_staleness = [999] * 64


def reset_frame_hash():
    """Reset the frame hash to force a display update."""
    global _last_frame_hash
    _last_frame_hash = None


# -----------------------------------------------------------------------------
# IMU Visualization
# -----------------------------------------------------------------------------

def draw_imu_overlay(
    draw: ImageDraw.Draw,
    center_x: int,
    center_y: int,
    radius: int,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    connected: bool = True,
    show_values: bool = True,
) -> None:
    """Draw an IMU gravity vector visualization.
    
    Renders a circular horizon indicator with:
    - Outer ring (fixed reference)
    - Tilted horizon line (shows roll)
    - Gravity dot (shows pitch as distance from center)
    - Numeric values for roll/pitch/yaw
    
    Args:
        draw: PIL ImageDraw object
        center_x, center_y: Center of the indicator
        radius: Radius of the outer ring
        roll_deg: Roll angle in degrees (tilts horizon line)
        pitch_deg: Pitch angle in degrees (moves dot up/down)
        yaw_deg: Yaw angle in degrees (displayed as text)
        connected: If False, show disconnected state
        show_values: If True, show numeric values
    """
    # Colors
    ring_color = (100, 100, 100) if connected else (60, 60, 60)
    horizon_color = (0, 200, 255) if connected else (80, 80, 80)
    dot_color = (255, 100, 50) if connected else (100, 60, 40)
    text_color = (200, 200, 200) if connected else (80, 80, 80)
    crosshair_color = (60, 60, 60)
    
    # Draw outer ring
    draw.ellipse(
        (center_x - radius, center_y - radius, center_x + radius, center_y + radius),
        outline=ring_color,
        width=2
    )
    
    # Draw crosshairs (fixed reference)
    draw.line((center_x - radius + 5, center_y, center_x + radius - 5, center_y), fill=crosshair_color, width=1)
    draw.line((center_x, center_y - radius + 5, center_x, center_y + radius - 5), fill=crosshair_color, width=1)
    
    if connected:
        # Calculate horizon line endpoints (rotated by roll)
        # Negate roll to show "artificial horizon" style (horizon tilts opposite to aircraft)
        roll_rad = math.radians(-roll_deg)
        line_len = radius - 4
        dx = line_len * math.cos(roll_rad)
        dy = line_len * math.sin(roll_rad)
        
        # Draw tilted horizon line
        draw.line(
            (center_x - dx, center_y + dy, center_x + dx, center_y - dy),
            fill=horizon_color,
            width=2
        )
        
        # Calculate gravity dot position
        # Pitch moves dot along the roll-perpendicular axis
        # Positive pitch = nose up = dot moves toward top of display
        pitch_offset = (pitch_deg / 90.0) * (radius - 8)
        # Clamp to stay within circle
        pitch_offset = max(-radius + 8, min(radius - 8, pitch_offset))
        
        # Dot position: perpendicular to roll axis
        perp_angle = roll_rad + math.pi / 2
        dot_x = center_x + pitch_offset * math.cos(perp_angle)
        dot_y = center_y - pitch_offset * math.sin(perp_angle)
        
        # Draw gravity dot
        dot_radius = 5
        draw.ellipse(
            (dot_x - dot_radius, dot_y - dot_radius, dot_x + dot_radius, dot_y + dot_radius),
            fill=dot_color,
            outline=(255, 255, 255),
            width=1
        )
        
        # Draw small markers at ±45° on the ring
        for angle in [-45, 45]:
            mark_rad = math.radians(angle)
            mx = center_x + (radius - 3) * math.cos(mark_rad - math.pi/2)
            my = center_y + (radius - 3) * math.sin(mark_rad - math.pi/2)
            draw.ellipse((mx - 2, my - 2, mx + 2, my + 2), fill=ring_color)
    
    # Draw numeric values below the indicator
    if show_values:
        try:
            font = get_font(10)
            if connected:
                text = f"R:{roll_deg:+5.1f} P:{pitch_deg:+5.1f} Y:{yaw_deg:+5.1f}"
            else:
                text = "IMU: --"
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            text_x = center_x - text_width // 2
            text_y = center_y + radius + 4
            draw.text((text_x, text_y), text, fill=text_color, font=font)
        except Exception:
            pass


# -----------------------------------------------------------------------------
# ToF Heatmap Visualization
# -----------------------------------------------------------------------------

# ToF distance color palette: blue (far) → green → yellow → red (close)
# Gradient defined as (distance_mm, R, G, B) breakpoints
TOF_COLOR_BREAKPOINTS = [
    (100, 255, 0, 0),      # Red: < 100mm (very close)
    (300, 255, 165, 0),    # Orange: 300mm
    (600, 255, 255, 0),    # Yellow: 600mm
    (1200, 0, 255, 0),     # Green: 1200mm
    (2500, 0, 100, 255),   # Blue: 2500mm
    (4000, 30, 30, 100),   # Dark blue: > 4000mm (far)
]


def get_tof_color(distance_mm: int, valid: bool = True) -> Tuple[int, int, int]:
    """Get interpolated color for a ToF distance reading.
    
    Args:
        distance_mm: Distance in millimeters (-1 or 0 for invalid)
        valid: Whether the reading is valid
    
    Returns:
        RGB tuple (0-255)
    """
    if not valid or distance_mm <= 0:
        return (40, 40, 40)  # Gray for invalid
    
    # Clamp to range
    if distance_mm <= TOF_COLOR_BREAKPOINTS[0][0]:
        return TOF_COLOR_BREAKPOINTS[0][1:4]
    if distance_mm >= TOF_COLOR_BREAKPOINTS[-1][0]:
        return TOF_COLOR_BREAKPOINTS[-1][1:4]
    
    # Find bracketing breakpoints and interpolate
    for i in range(len(TOF_COLOR_BREAKPOINTS) - 1):
        d1, r1, g1, b1 = TOF_COLOR_BREAKPOINTS[i]
        d2, r2, g2, b2 = TOF_COLOR_BREAKPOINTS[i + 1]
        if d1 <= distance_mm <= d2:
            # Linear interpolation
            t = (distance_mm - d1) / (d2 - d1)
            r = int(r1 + t * (r2 - r1))
            g = int(g1 + t * (g2 - g1))
            b = int(b1 + t * (b2 - b1))
            return (r, g, b)
    
    return (40, 40, 40)


def draw_tof_heatmap(
    draw: ImageDraw.Draw,
    x: int,
    y: int,
    width: int,
    height: int,
    distances: List[int],
    statuses: List[int],
    connected: bool = True,
    show_closest: bool = True,
    title: str = "ToF",
    distance_position: str = "bottom",
) -> None:
    """Draw a ToF 8×8 heatmap visualization with hold-last-valid smoothing.
    
    When a cell returns an invalid reading (e.g., glass reflection), the last
    valid distance is held and displayed with slight dimming. After TOF_MAX_HOLD_FRAMES
    frames without a valid reading, the cell goes gray.
    
    Args:
        draw: PIL ImageDraw object
        x, y: Top-left corner of the heatmap area
        width, height: Size of the heatmap area
        distances: Flat list of 64 distance values (mm), row-major order
        statuses: Flat list of 64 target status values (5 or 9 = valid)
        connected: If False, show grayed out state
        show_closest: If True, highlight the closest reading
        title: Title text to display above heatmap
        distance_position: 'top' or 'bottom' for closest distance text
    """
    global _tof_held_distances, _tof_staleness
    
    # Valid status codes from VL53L5CX
    VALID_STATUS = (5, 9)
    
    # Colors
    border_color = (100, 100, 100) if connected else (60, 60, 60)
    text_color = (200, 200, 200) if connected else (80, 80, 80)
    
    # Calculate cell size (8×8 grid)
    grid_size = 8
    title_height = 16 if title else 0
    dist_height = 14 if show_closest else 0
    top_padding = title_height + (dist_height if distance_position == 'top' else 0)
    bottom_padding = dist_height if distance_position == 'bottom' else 0
    
    cell_w = (width - 4) // grid_size  # -4 for border
    cell_h = (height - top_padding - bottom_padding - 4) // grid_size
    
    # Actual grid dimensions
    grid_w = cell_w * grid_size
    grid_h = cell_h * grid_size
    grid_x = x + (width - grid_w) // 2
    grid_y = y + top_padding + 2  # Space for title and optional distance
    
    # Draw title
    try:
        font = get_font(10)
        bbox = draw.textbbox((0, 0), title, font=font)
        title_w = bbox[2] - bbox[0]
        draw.text((x + (width - title_w) // 2, y + 2), title, fill=text_color, font=font)
    except Exception:
        pass
    
    # Draw border
    draw.rectangle(
        (grid_x - 1, grid_y - 1, grid_x + grid_w, grid_y + grid_h),
        outline=border_color,
        width=1
    )
    
    if not connected:
        # Reset hold buffers when disconnected
        reset_tof_hold_buffers()
        # Draw "NO ToF" text in center
        try:
            font = get_font(12)
            text = "NO ToF"
            bbox = draw.textbbox((0, 0), text, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            draw.text(
                (grid_x + (grid_w - tw) // 2, grid_y + (grid_h - th) // 2),
                text, fill=(80, 80, 80), font=font
            )
        except Exception:
            pass
        return
    
    # Update hold buffers and find closest valid reading
    closest_idx = -1
    closest_dist = 9999
    
    for idx in range(64):
        if idx < len(distances) and idx < len(statuses):
            dist = distances[idx]
            stat = statuses[idx]
            valid = stat in VALID_STATUS and dist > 0
            
            if valid:
                # Fresh valid reading: update buffer and reset staleness
                _tof_held_distances[idx] = dist
                _tof_staleness[idx] = 0
            else:
                # Invalid reading: increment staleness counter
                if _tof_staleness[idx] < 999:
                    _tof_staleness[idx] += 1
        else:
            # No data for this index
            if _tof_staleness[idx] < 999:
                _tof_staleness[idx] += 1
        
        # Use held distance for closest calculation if within hold limit
        if _tof_staleness[idx] < TOF_MAX_HOLD_FRAMES and _tof_held_distances[idx] > 0:
            if _tof_held_distances[idx] < closest_dist:
                closest_dist = _tof_held_distances[idx]
                closest_idx = idx
    
    # Draw cells
    for row in range(grid_size):
        for col in range(grid_size):
            # Index in row-major order, mirrored horizontally for display
            # (sensor is mounted backwards relative to display)
            disp_col = grid_size - 1 - col
            idx = row * grid_size + disp_col
            
            staleness = _tof_staleness[idx]
            held_dist = _tof_held_distances[idx]
            
            if staleness < TOF_MAX_HOLD_FRAMES and held_dist > 0:
                # Use held value (either fresh or within hold limit)
                color = get_tof_color(held_dist, valid=True)
                
                # Dim the color if stale (not fresh)
                if staleness > 0:
                    # Calculate dim factor: starts at TOF_STALE_DIM_FACTOR, fades to darker
                    fade = 1.0 - (staleness / TOF_MAX_HOLD_FRAMES) * (1.0 - TOF_STALE_DIM_FACTOR)
                    color = (
                        int(color[0] * fade),
                        int(color[1] * fade),
                        int(color[2] * fade)
                    )
            else:
                # No valid data or too stale: gray
                color = (40, 40, 40)
            
            cx = grid_x + col * cell_w
            cy = grid_y + row * cell_h
            
            draw.rectangle(
                (cx, cy, cx + cell_w - 1, cy + cell_h - 1),
                fill=color
            )
            
            # Highlight closest cell
            if show_closest and idx == closest_idx:
                draw.rectangle(
                    (cx, cy, cx + cell_w - 1, cy + cell_h - 1),
                    outline=(255, 255, 255),
                    width=1
                )
    
    # Draw closest distance text (above or below grid based on distance_position)
    if show_closest and closest_idx >= 0:
        try:
            font = get_font(10)
            # Show stale indicator if the closest reading is held
            stale_marker = "~" if _tof_staleness[closest_idx] > 0 else ""
            text = f"{stale_marker}{closest_dist}mm"
            bbox = draw.textbbox((0, 0), text, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            # Use slightly dimmer color if stale
            dist_color = (150, 150, 150) if _tof_staleness[closest_idx] > 0 else text_color
            
            if distance_position == 'top':
                # Draw above grid, below title
                dist_y = y + (16 if title else 0) + 1
            else:
                # Draw below grid
                dist_y = grid_y + grid_h + 2
            
            draw.text(
                (grid_x + (grid_w - tw) // 2, dist_y),
                text, fill=dist_color, font=font
            )
        except Exception:
            pass


# -----------------------------------------------------------------------------
# 3D Hexapod Wireframe Visualization
# -----------------------------------------------------------------------------

def draw_hexapod_wireframe_3d(
    draw: ImageDraw.Draw,
    center_x: int,
    center_y: int,
    size: int,
    joint_angles: List[float],
    servo_temps: List[float],
    servo_voltages: List[float] = None,
    azimuth_deg: float = 30.0,
    elevation_deg: float = 30.0,
    body_pitch_deg: float = 0.0,
    body_roll_deg: float = 0.0,
    temp_min: float = 25.0,
    temp_max: float = 55.0,
    color_palette: List[Tuple[int, int, int]] = None,
) -> Tuple[float, float, float, str]:
    """Draw a 3D wireframe visualization of the hexapod using FK.
    
    Args:
        draw: PIL ImageDraw object
        center_x, center_y: Center position for drawing
        size: Overall size (used to calculate scale)
        joint_angles: List of 18 joint angles in degrees (leg-major: LF_C,LF_F,LF_T,...)
        servo_temps: List of 18 servo temperatures (°C)
        servo_voltages: List of 18 servo voltages (V)
        azimuth_deg: Camera azimuth angle (rotation around Y axis)
        elevation_deg: Camera elevation angle (tilt from horizontal)
        body_pitch_deg: Body pitch from IMU (positive = nose up)
        body_roll_deg: Body roll from IMU (positive = right side down)
        temp_min, temp_max: Temperature range for color mapping
        color_palette: Temperature color palette
    
    Returns:
        Tuple of (avg_temp, avg_voltage, max_temp, hottest_label) for compatibility
    """
    if color_palette is None:
        color_palette = TEMPERATURE_COLOR_PALETTE
    
    # Ensure we have valid joint angles (default to home position if not)
    if joint_angles is None or len(joint_angles) < 18:
        joint_angles = [120.0] * 18  # Home position = 120 degrees
    
    # Calculate scale: fit robot into given size
    # Robot roughly spans ~500mm diameter at full reach (coxa offset + leg reach)
    # Increase scale for better zoom (was 300, now 200 for more zoom)
    scale = size / 200.0
    
    segment_width = max(2, size // 18)  # Thicker lines for visibility
    joint_radius = segment_width // 2 + 2
    
    # DEBUG: Only draw specific legs for FK diagnosis (set to [] for all legs)
    DEBUG_LEGS = []  # All legs
    
    # Body rotation from IMU (pitch around X, roll around Z)
    pitch_rad = math.radians(body_pitch_deg)
    roll_rad = math.radians(body_roll_deg)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
    
    def apply_body_rotation(pts: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """Apply body pitch (around X) and roll (around Z) to points."""
        result = []
        for x, y, z in pts:
            # Roll around Z axis (body X axis in our frame)
            x1 = x * cos_r - y * sin_r
            y1 = x * sin_r + y * cos_r
            z1 = z
            # Pitch around X axis
            y2 = y1 * cos_p - z1 * sin_p
            z2 = y1 * sin_p + z1 * cos_p
            x2 = x1
            result.append((x2, y2, z2))
        return result
    
    # Stats tracking
    temps = []
    max_temp = -999.0
    hottest_label = ""
    
    # Compute all leg points using FK
    all_leg_data = []  # List of (leg_idx, points_3d, points_2d, temps)
    
    legs_to_draw = DEBUG_LEGS if DEBUG_LEGS else range(6)
    for leg_idx in legs_to_draw:
        # Get joint angles for this leg
        base_idx = leg_idx * 3
        coxa_deg = joint_angles[base_idx] if base_idx < len(joint_angles) else 0.0
        femur_deg = joint_angles[base_idx + 1] if base_idx + 1 < len(joint_angles) else 0.0
        tibia_deg = joint_angles[base_idx + 2] if base_idx + 2 < len(joint_angles) else 0.0
        
        # Forward kinematics: get 4 3D points
        points_3d = fk_leg_points(leg_idx, coxa_deg, femur_deg, tibia_deg)
        
        # Apply body rotation (pitch/roll from IMU)
        points_3d = apply_body_rotation(points_3d)
        
        # Project to 2D
        points_2d = project_3d_to_2d(
            points_3d, azimuth_deg, elevation_deg, center_x, center_y, scale
        )
        
        # Get temperatures for this leg's servos
        leg_temps = []
        for joint_idx in range(3):
            servo_idx = leg_idx * 3 + joint_idx
            temp = servo_temps[servo_idx] if servo_idx < len(servo_temps) else 0.0
            leg_temps.append(temp)
            if temp > 0:
                temps.append(temp)
                if temp > max_temp:
                    max_temp = temp
                    hottest_label = f"{LEG_NAMES[leg_idx]}-{JOINT_NAMES[joint_idx][0]}"
        
        # Average depth for sorting (use midpoint of leg)
        avg_depth = sum(p[2] for p in points_2d) / len(points_2d)
        
        all_leg_data.append((leg_idx, points_3d, points_2d, leg_temps, avg_depth))
    
    # Sort legs by depth (back to front for proper occlusion)
    all_leg_data.sort(key=lambda x: x[4], reverse=True)
    
    # Draw body hexagon first (always behind legs in top-down-ish view)
    # Trace perimeter: LF(0)->RF(3)->RM(4)->RR(5)->LR(2)->LM(1)->back to LF
    body_order = [0, 3, 4, 5, 2, 1]
    body_points_3d = [(COXA_OFFSET[i][0], 0.0, COXA_OFFSET[i][1]) for i in body_order]
    body_points_3d = apply_body_rotation(body_points_3d)
    body_points_2d = project_3d_to_2d(
        body_points_3d, azimuth_deg, elevation_deg, center_x, center_y, scale
    )
    
    # Draw body as filled polygon
    body_poly = [(int(p[0]), int(p[1])) for p in body_points_2d]
    draw.polygon(body_poly, fill=(50, 50, 50), outline=(100, 100, 100))
    
    # Draw each leg (back to front)
    for leg_idx, points_3d, points_2d, leg_temps, _ in all_leg_data:
        # Draw 3 segments: coxa→femur, femur→tibia, tibia→foot
        for seg_idx in range(3):
            p1 = points_2d[seg_idx]
            p2 = points_2d[seg_idx + 1]
            temp = leg_temps[seg_idx]
            
            # Get color based on temperature
            if temp > 0:
                color = getColor(temp_min, temp_max, temp, color_palette)
            else:
                color = (60, 60, 60)  # No data: dark gray
            
            # Draw segment
            draw.line(
                [(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1]))],
                fill=color,
                width=segment_width
            )
            
            # Draw joint circle at start of segment
            jx, jy = int(p1[0]), int(p1[1])
            draw.ellipse(
                (jx - joint_radius, jy - joint_radius,
                 jx + joint_radius, jy + joint_radius),
                fill=color
            )
        
        # Draw foot (end of tibia) as small gray circle
        foot = points_2d[3]
        fx, fy = int(foot[0]), int(foot[1])
        foot_radius = segment_width // 2
        draw.ellipse(
            (fx - foot_radius, fy - foot_radius,
             fx + foot_radius, fy + foot_radius),
            fill=(150, 150, 150)
        )
    
    # Calculate summary stats
    avg_temp = sum(temps) / len(temps) if temps else 0.0
    # Calculate average voltage from servo_voltages (more stable than min)
    if servo_voltages:
        valid_volts = [v for v in servo_voltages if v and v > 0]
        avg_voltage = sum(valid_volts) / len(valid_volts) if valid_volts else 0.0
    else:
        avg_voltage = 0.0
    
    return (avg_temp, avg_voltage, max_temp, hottest_label)


# -----------------------------------------------------------------------------
# Hexapod Silhouette Heat Map (flat 2D fallback)
# -----------------------------------------------------------------------------

# Leg names and servo indices: leg order LF,LM,LR,RF,RM,RR; joints Coxa,Femur,Tibia
# Servo indices: LF=0-2, LM=3-5, LR=6-8, RF=9-11, RM=12-14, RR=15-17
LEG_NAMES = ['LF', 'LM', 'LR', 'RF', 'RM', 'RR']
JOINT_NAMES = ['Coxa', 'Femur', 'Tibia']

# Leg geometry for top-down view: (angle_deg, x_sign) from body center
# Angles: 0=front, 90=right, 180=back, 270=left
LEG_ANGLES = {
    'LF': 315,  # Front-left
    'LM': 270,  # Mid-left
    'LR': 225,  # Rear-left
    'RF': 45,   # Front-right
    'RM': 90,   # Mid-right
    'RR': 135,  # Rear-right
}


def draw_hexapod_heatmap(
    draw: ImageDraw.Draw,
    center_x: int,
    center_y: int,
    size: int,
    servo_temps: List[float],
    servo_voltages: List[float],
    temp_min: float = 25.0,
    temp_max: float = 55.0,
    color_palette: List[Tuple[int, int, int]] = None,
) -> Tuple[float, float, float, str]:
    """Draw a top-down hexapod silhouette with servo temperatures as segment colors.
    
    Args:
        draw: PIL ImageDraw object
        center_x, center_y: Center position of the hexapod body
        size: Overall size (body radius + leg length)
        servo_temps: List of 18 temperatures (°C), indexed by servo
        servo_voltages: List of 18 voltages (V), indexed by servo
        temp_min: Temperature for 'cool' color (green)
        temp_max: Temperature for 'hot' color (red)
        color_palette: Temperature color palette [(cool), (mid), (hot)]
    
    Returns:
        (avg_temp, avg_voltage, max_temp, hottest_label): Summary stats
    """
    if color_palette is None:
        color_palette = TEMPERATURE_COLOR_PALETTE
    
    # Geometry
    body_radius = size * 0.25
    coxa_len = size * 0.15
    femur_len = size * 0.25
    tibia_len = size * 0.30
    segment_width = max(3, size // 20)
    
    # Draw body (hexagon)
    body_points = []
    for i in range(6):
        angle = math.radians(60 * i - 30)
        bx = center_x + body_radius * math.cos(angle)
        by = center_y + body_radius * math.sin(angle)
        body_points.append((bx, by))
    draw.polygon(body_points, fill=(60, 60, 60), outline=(100, 100, 100))
    
    # Stats tracking
    temps = []
    volts = []
    max_temp = -999
    hottest_label = ""
    
    # Draw each leg
    for leg_idx, leg_name in enumerate(LEG_NAMES):
        angle_deg = LEG_ANGLES[leg_name]
        angle_rad = math.radians(angle_deg)
        
        # Starting point: edge of body
        start_x = center_x + body_radius * math.cos(angle_rad)
        start_y = center_y + body_radius * math.sin(angle_rad)
        
        # Servo indices for this leg
        servo_base = leg_idx * 3
        
        # Draw 3 segments: Coxa -> Femur -> Tibia
        seg_lengths = [coxa_len, femur_len, tibia_len]
        curr_x, curr_y = start_x, start_y
        
        for joint_idx, seg_len in enumerate(seg_lengths):
            servo_idx = servo_base + joint_idx
            
            # Get temperature and voltage
            temp = servo_temps[servo_idx] if servo_idx < len(servo_temps) else 0
            volt = servo_voltages[servo_idx] if servo_idx < len(servo_voltages) else 0
            
            if temp > 0:
                temps.append(temp)
                if temp > max_temp:
                    max_temp = temp
                    hottest_label = f"{leg_name}-{JOINT_NAMES[joint_idx][:1]}"
            if volt > 0:
                volts.append(volt)
            
            # Calculate segment end point
            end_x = curr_x + seg_len * math.cos(angle_rad)
            end_y = curr_y + seg_len * math.sin(angle_rad)
            
            # Get color based on temperature
            if temp > 0:
                color = getColor(temp_min, temp_max, temp, color_palette)
            else:
                color = (50, 50, 50)  # No data: dark gray
            
            # Draw segment as thick line
            draw.line(
                [(curr_x, curr_y), (end_x, end_y)],
                fill=color,
                width=segment_width
            )
            
            # Draw joint circle at connection point
            joint_radius = segment_width // 2 + 1
            draw.ellipse(
                (curr_x - joint_radius, curr_y - joint_radius,
                 curr_x + joint_radius, curr_y + joint_radius),
                fill=color
            )
            
            curr_x, curr_y = end_x, end_y
        
        # Draw foot (end of tibia) as small circle
        foot_radius = segment_width // 2
        draw.ellipse(
            (curr_x - foot_radius, curr_y - foot_radius,
             curr_x + foot_radius, curr_y + foot_radius),
            fill=(150, 150, 150)
        )
    
    # Draw leg labels
    try:
        font = get_font(8)
        label_offset = size * 0.72
        for leg_name in LEG_NAMES:
            angle_rad = math.radians(LEG_ANGLES[leg_name])
            lx = center_x + label_offset * math.cos(angle_rad)
            ly = center_y + label_offset * math.sin(angle_rad)
            bbox = draw.textbbox((0, 0), leg_name, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            draw.text((lx - tw // 2, ly - th // 2), leg_name, fill=(180, 180, 180), font=font)
    except Exception:
        pass
    
    # Calculate summary stats
    avg_temp = sum(temps) / len(temps) if temps else 0
    # Use average voltage (more stable than min during walking)
    avg_voltage = sum(volts) / len(volts) if volts else 0
    
    return (avg_temp, avg_voltage, max_temp, hottest_label)


def draw_battery_icon(
    image: Image.Image,
    voltage: float,
    volt_min: float = 10.5,
    volt_max: float = 12.5,
    x: int = None,
    y: int = 4,
    width: int = 28,
    height: int = 14,
    show_text: bool = True,
    alpha: float = 0.7,
) -> Image.Image:
    """Draw a phone-style battery icon with percentage fill.
    
    Position defaults to top-right corner. Shows colored fill bar representing
    charge level (green=good, yellow=medium, red=low) with optional voltage text.
    
    Args:
        image: Base image to draw on (will be copied)
        voltage: Current battery voltage (V)
        volt_min: Voltage at 0% (empty)
        volt_max: Voltage at 100% (full)
        x: X position (None = right edge - width - margin)
        y: Y position from top
        width: Battery body width
        height: Battery body height
        show_text: If True, show voltage text below icon
        alpha: Transparency (0.0 = invisible, 1.0 = fully opaque)
    
    Returns:
        Modified image with battery overlay
    """
    img = image.copy()
    draw = ImageDraw.Draw(img)
    
    # Calculate percentage
    pct = max(0.0, min(1.0, (voltage - volt_min) / (volt_max - volt_min))) if volt_max > volt_min else 0.0
    
    # Position: default to top-right corner
    if x is None:
        x = img.width - width - 6
    
    # Apply alpha by blending colors with existing pixels
    # For simplicity, draw at reduced intensity
    def alpha_color(r, g, b):
        return (int(r * alpha), int(g * alpha), int(b * alpha))
    
    # Battery body outline (rounded rectangle effect)
    body_outline = alpha_color(180, 180, 180)
    body_fill = alpha_color(30, 30, 30)
    
    # Draw body background
    draw.rectangle((x, y, x + width, y + height), fill=body_fill, outline=body_outline)
    
    # Battery positive terminal (nub on right side)
    nub_w = 3
    nub_h = height // 3
    nub_y = y + (height - nub_h) // 2
    draw.rectangle((x + width, nub_y, x + width + nub_w, nub_y + nub_h), fill=body_outline)
    
    # Fill bar (inside body with 2px padding)
    pad = 2
    fill_max_w = width - 2 * pad
    fill_w = int(fill_max_w * pct)
    fill_x = x + pad
    fill_y = y + pad
    fill_h = height - 2 * pad
    
    # Fill color based on percentage
    if pct > 0.5:
        fill_color = alpha_color(0, 200, 0)    # Green
    elif pct > 0.2:
        fill_color = alpha_color(255, 180, 0)  # Yellow/orange
    else:
        fill_color = alpha_color(220, 50, 50)  # Red
    
    if fill_w > 0:
        draw.rectangle((fill_x, fill_y, fill_x + fill_w, fill_y + fill_h - 1), fill=fill_color)
    
    # Optional voltage text below icon
    if show_text:
        try:
            font = get_font(9)
            text = f"{voltage:.1f}V"
            bbox = draw.textbbox((0, 0), text, font=font)
            tw = bbox[2] - bbox[0]
            tx = x + (width - tw) // 2
            ty = y + height + 2
            # Shadow for contrast
            shadow = alpha_color(0, 0, 0)
            text_color = alpha_color(200, 200, 200)
            draw.text((tx + 1, ty), text, fill=shadow, font=font)
            draw.text((tx, ty), text, fill=text_color, font=font)
        except Exception:
            pass
    
    return img


def draw_low_battery_warning(
    image: Image.Image,
    voltage: float,
    x: int = 0,
    y: int = 0,
    width: int = 0,
    height: int = 0,
) -> Image.Image:
    """Draw a low battery warning overlay on the image.
    
    Shows a flashing-style red bar at the bottom with "LOW BATTERY" text and voltage.
    
    Args:
        image: Base image to draw on (will be copied)
        voltage: Current voltage value
        x, y: Top-left of display area (usually 0, 0)
        width, height: Display dimensions
    
    Returns:
        Modified image with warning overlay
    """
    img = image.copy()
    draw = ImageDraw.Draw(img)
    
    if width <= 0:
        width = img.width
    if height <= 0:
        height = img.height
    
    # Warning bar at bottom of screen
    bar_height = 22
    bar_y = height - bar_height
    
    # Red background with slight transparency effect
    draw.rectangle((x, bar_y, x + width, height), fill=(180, 0, 0))
    
    # Warning text
    try:
        font = get_font(14)
        text = f"LOW BATTERY {voltage:.1f}V"
        bbox = draw.textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        tx = x + (width - tw) // 2
        ty = bar_y + (bar_height - th) // 2
        # White text with black shadow for contrast
        draw.text((tx + 1, ty + 1), text, fill=(0, 0, 0), font=font)
        draw.text((tx, ty), text, fill=(255, 255, 255), font=font)
    except Exception:
        pass
    
    return img


def draw_voltage_bar(
    draw: ImageDraw.Draw,
    x: int,
    y: int,
    width: int,
    height: int,
    voltage: float,
    min_v: float = 6.0,
    max_v: float = 8.4,
    color_palette: List[Tuple[int, int, int]] = None,
) -> None:
    """Draw a horizontal voltage bar indicator.
    
    Args:
        draw: PIL ImageDraw object
        x, y: Top-left position
        width, height: Bar dimensions
        voltage: Current voltage value
        min_v, max_v: Voltage range for color scale
        color_palette: Voltage color palette [(low/red), (high/green)]
    """
    if color_palette is None:
        color_palette = POWER_COLOR_PALETTE
    
    # Background
    draw.rectangle((x, y, x + width, y + height), fill=(40, 40, 40), outline=(80, 80, 80))
    
    # Calculate fill level
    fill_pct = max(0, min(1, (voltage - min_v) / (max_v - min_v))) if max_v > min_v else 0
    fill_w = int((width - 2) * fill_pct)
    
    # Get color based on voltage level
    color = getColor(min_v, max_v, voltage, color_palette)
    
    # Draw filled portion
    if fill_w > 0:
        draw.rectangle((x + 1, y + 1, x + 1 + fill_w, y + height - 1), fill=color)
    
    # Draw voltage text
    try:
        font = get_font(10)
        text = f"{voltage:.1f}V"
        bbox = draw.textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        tx = x + (width - tw) // 2
        ty = y + (height - th) // 2
        # Shadow for readability
        draw.text((tx + 1, ty + 1), text, fill=(0, 0, 0), font=font)
        draw.text((tx, ty), text, fill=(255, 255, 255), font=font)
    except Exception:
        pass


# -----------------------------------------------------------------------------
# LCARS Color Palettes for Engineering View
# -----------------------------------------------------------------------------

# Classic TNG palette (default)
LCARS_CLASSIC = {
    'orange': (255, 136, 0),
    'gold': (255, 170, 0),
    'peach': (255, 136, 102),
    'sunflower': (255, 204, 153),
    'violet': (204, 153, 255),
    'lilac': (204, 85, 255),
    'almond': (255, 170, 144),
    'ice': (153, 204, 255),
    'sky': (170, 170, 255),
    'tomato': (255, 85, 85),
    'gray': (102, 102, 136),
    'bg': (0, 0, 0),
}

# Nemesis Blue palette
LCARS_NEMESIS = {
    'orange': (102, 153, 255),    # "cool" - main frame color
    'gold': (136, 187, 255),      # "ghost"
    'peach': (34, 102, 255),      # "evening"
    'sunflower': (235, 240, 255), # "moonbeam"
    'violet': (153, 102, 204),    # "grape"
    'lilac': (153, 102, 204),     # "grape"
    'almond': (204, 170, 136),    # "wheat"
    'ice': (136, 187, 255),       # "ghost"
    'sky': (102, 153, 255),       # "cool"
    'tomato': (204, 34, 51),      # "cardinal"
    'gray': (82, 82, 106),        # "galaxy_gray"
    'bg': (0, 0, 0),
}

# Lower Decks palette
LCARS_LOWER_DECKS = {
    'orange': (255, 119, 0),
    'gold': (255, 153, 17),       # "daybreak"
    'peach': (255, 170, 68),      # "harvestgold"
    'sunflower': (255, 238, 204), # "butter"
    'violet': (255, 68, 0),       # "october_sunset"
    'lilac': (204, 85, 0),        # "rich_pumpkin"
    'almond': (255, 204, 153),    # "honey"
    'ice': (255, 238, 204),       # "butter"
    'sky': (255, 204, 153),       # "honey"
    'tomato': (255, 68, 0),       # "october_sunset"
    'gray': (204, 85, 0),         # "rich_pumpkin"
    'bg': (0, 0, 0),
}

# Lower Decks PADD palette
LCARS_PADD = {
    'orange': (85, 136, 238),     # "alpha_blue"
    'gold': (119, 153, 221),      # "beta_blue"
    'peach': (69, 85, 128),       # "night_rain"
    'sunflower': (153, 204, 255), # "arctic_snow"
    'violet': (136, 255, 255),    # "radioactive"
    'lilac': (102, 204, 255),     # "arctic_ice"
    'almond': (153, 204, 255),    # "arctic_snow"
    'ice': (136, 255, 255),       # "radioactive"
    'sky': (102, 204, 255),       # "arctic_ice"
    'tomato': (255, 53, 0),       # "sunset_red"
    'gray': (52, 68, 112),        # "night_cloud"
    'bg': (0, 0, 0),
}

# List of palettes for index-based selection (0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD)
LCARS_PALETTES = [LCARS_CLASSIC, LCARS_NEMESIS, LCARS_LOWER_DECKS, LCARS_PADD]

def get_lcars_palette(palette_idx: int = 0) -> dict:
    """Get LCARS color palette by index.
    
    Args:
        palette_idx: 0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD
    
    Returns:
        Dict with color names mapped to RGB tuples
    """
    if 0 <= palette_idx < len(LCARS_PALETTES):
        return LCARS_PALETTES[palette_idx]
    return LCARS_CLASSIC  # Default fallback


def draw_lcars_pill(
    draw: ImageDraw.Draw,
    x: int, y: int,
    width: int, height: int,
    color: Tuple[int, int, int],
    text: str = None,
    font: ImageFont.FreeTypeFont = None,
    text_color: Tuple[int, int, int] = None,
    align: str = 'center',
) -> None:
    """Draw an LCARS-style pill (rounded rectangle button).
    
    Args:
        draw: ImageDraw object
        x, y: Top-left position
        width, height: Pill dimensions
        color: Fill color
        text: Optional text inside pill
        font: Font for text
        text_color: Text color (default white)
        align: Text alignment ('left', 'center', 'right')
    """
    r = height // 2  # Radius for rounded ends
    
    # Draw the pill shape (rectangle + two semicircles)
    if width > height:
        # Horizontal pill
        draw.rectangle((x + r, y, x + width - r, y + height - 1), fill=color)
        draw.ellipse((x, y, x + height - 1, y + height - 1), fill=color)
        draw.ellipse((x + width - height, y, x + width - 1, y + height - 1), fill=color)
    else:
        # Just draw an ellipse for very small pills
        draw.ellipse((x, y, x + width - 1, y + height - 1), fill=color)
    
    # Draw text if provided
    if text and font:
        tc = text_color or (255, 255, 255)
        bbox = draw.textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        
        if align == 'left':
            tx = x + r + 2
        elif align == 'right':
            tx = x + width - r - tw - 2
        else:  # center
            tx = x + (width - tw) // 2
        
        ty = y + (height - th) // 2
        draw.text((tx, ty), text, fill=tc, font=font)


def draw_lcars_sweep(
    draw: ImageDraw.Draw,
    x: int, y: int,
    outer_r: int,
    thick: int,
    thin: int,
    color: Tuple[int, int, int],
    bg_color: Tuple[int, int, int],
    corner: str = 'tl',
) -> None:
    """Draw an LCARS swept corner (quarter arc with thickness transition).
    
    Args:
        draw: ImageDraw object
        x, y: Position (meaning depends on corner)
        outer_r: Outer radius of sweep
        thick: Thick bar dimension
        thin: Thin bar dimension
        color: Sweep color
        bg_color: Background color for cutout
        corner: Which corner ('tl', 'tr', 'bl', 'br')
    """
    if corner == 'tl':
        # Top-left: arc from thick vertical to thin horizontal
        draw.pieslice((x, y, x + outer_r * 2, y + outer_r * 2), 180, 270, fill=color)
        inner_r = outer_r - thin
        draw.pieslice((x + thick, y + thin, x + thick + inner_r * 2, y + thin + inner_r * 2),
                     180, 270, fill=bg_color)
    elif corner == 'bl':
        # Bottom-left: arc from thick vertical to thin horizontal
        draw.pieslice((x, y, x + outer_r * 2, y + outer_r * 2), 90, 180, fill=color)
        inner_r = outer_r - thin
        draw.pieslice((x + thick, y - outer_r + thin, x + thick + inner_r * 2, y + inner_r),
                     90, 180, fill=bg_color)
    elif corner == 'tr':
        # Top-right: arc from thin horizontal to thick vertical
        draw.pieslice((x - outer_r * 2, y, x, y + outer_r * 2), 270, 360, fill=color)
        inner_r = outer_r - thin
        draw.pieslice((x - outer_r * 2 - thick + thin, y + thin, 
                      x - thick + thin, y + thin + inner_r * 2),
                     270, 360, fill=bg_color)
    elif corner == 'br':
        # Bottom-right: arc from thick vertical to thin horizontal
        draw.pieslice((x - outer_r * 2, y, x, y + outer_r * 2), 0, 90, fill=color)
        inner_r = outer_r - thin
        draw.pieslice((x - outer_r * 2 - thick + thin, y - outer_r + thin,
                      x - thick + thin, y + inner_r),
                     0, 90, fill=bg_color)


def draw_lcars_engineering_view(
    image: Image.Image,
    disp_width: int,
    disp_height: int,
    imu_connected: bool,
    imu_roll: float,
    imu_pitch: float,
    imu_yaw: float,
    tof_connected: bool,
    tof_distances: List[int],
    tof_statuses: List[int],
    servo_temps: List[float] = None,
    servo_voltages: List[float] = None,
    temp_min: float = 25.0,
    temp_max: float = 55.0,
    volt_min: float = 6.0,
    volt_max: float = 8.4,
    joint_angles: List[float] = None,
    joint_angles_valid: bool = False,
    view_azimuth: float = 0.0,
    view_elevation: float = 30.0,
    lcars_palette: int = 0,
) -> Image.Image:
    """Draw LCARS-themed engineering view.
    
    Star Trek LCARS-inspired interface with:
    - Swept corner frame in top-left
    - Pill-shaped status labels
    - Configurable color palette (Classic, Nemesis, Lower Decks, PADD)
    - Clean vector aesthetic
    
    Layout (320x170):
    - Left frame: Swept corner + vertical bar (12px wide)
    - Top header: IMU status pills
    - Main area: Hexapod 3D view (left), ToF heatmap (right)
    - Bottom: Voltage bar + status pills
    
    Args:
        lcars_palette: Palette index (0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD)
    """
    img = image.copy()
    draw = ImageDraw.Draw(img)
    
    # Get color palette
    colors = get_lcars_palette(lcars_palette)
    
    # Fill with LCARS black background
    draw.rectangle((0, 0, disp_width, disp_height), fill=colors['bg'])
    
    # Ensure we have servo data arrays
    if servo_temps is None:
        servo_temps = [0.0] * 18
    if servo_voltages is None:
        servo_voltages = [0.0] * 18
    
    # === Layout constants ===
    thick = 10          # Left frame bar width
    thin = 4            # Top/bottom bar height
    sweep_r = 14        # Sweep corner radius
    margin = 3
    
    # Colors from palette
    c_frame = colors['orange']
    c_frame_bottom = colors['peach']
    c_accent = colors['gold']
    c_violet = colors['violet']
    c_ice = colors['ice']
    c_tomato = colors['tomato']
    c_bg = colors['bg']
    c_text = (0, 0, 0)  # Black text for readability
    
    # === Draw LCARS frame ===
    
    # Top-left swept corner
    draw.pieslice((0, 0, sweep_r * 2, sweep_r * 2), 180, 270, fill=c_frame)
    inner_r = sweep_r - thin
    draw.pieslice((thick, thin, thick + inner_r * 2, thin + inner_r * 2), 
                 180, 270, fill=c_bg)
    
    # Top horizontal bar (thin) - from sweep to near right edge
    draw.rectangle((sweep_r, 0, disp_width - 60, thin - 1), fill=c_frame)
    
    # Right top accent (pill shape)
    draw_lcars_pill(draw, disp_width - 55, 0, 55, thin + 4, c_accent)
    
    # Bottom-left swept corner
    bottom_sweep_y = disp_height - sweep_r * 2
    draw.pieslice((0, bottom_sweep_y, sweep_r * 2, disp_height), 90, 180, fill=c_frame_bottom)
    draw.pieslice((thick, bottom_sweep_y + thin, thick + inner_r * 2, disp_height - thin), 
                 90, 180, fill=c_bg)
    
    # Bottom horizontal bar (thin)
    draw.rectangle((sweep_r, disp_height - thin, disp_width - 100, disp_height - 1), fill=c_frame_bottom)
    # Rounded cap at end
    cap_r = thin // 2
    draw.ellipse((disp_width - 100 - cap_r, disp_height - thin, 
                 disp_width - 100 + cap_r, disp_height - 1), fill=c_frame_bottom)
    
    # Left vertical bar (between the two sweeps)
    bar_top = sweep_r + thin
    bar_bottom = bottom_sweep_y + thin - 1
    draw.rectangle((0, bar_top, thick - 1, bar_bottom), fill=c_frame)
    
    # === Header: IMU status pills ===
    header_y = thin + 2
    pill_h = 14
    font_sm = get_font(10)
    font_md = get_font(12)
    
    # Title pill
    draw_lcars_pill(draw, thick + margin, header_y, 78, pill_h, c_violet,
                   text="ENGINEERING", font=font_sm, text_color=c_text)
    
    # IMU status pills
    if imu_connected:
        # Roll
        roll_color = c_ice if abs(imu_roll) < 15 else c_tomato
        draw_lcars_pill(draw, thick + 84, header_y, 52, pill_h, roll_color,
                       text=f"R{imu_roll:+.0f}°", font=font_sm, text_color=c_text)
        # Pitch
        pitch_color = c_ice if abs(imu_pitch) < 15 else c_tomato
        draw_lcars_pill(draw, thick + 139, header_y, 52, pill_h, pitch_color,
                       text=f"P{imu_pitch:+.0f}°", font=font_sm, text_color=c_text)
        # Yaw
        draw_lcars_pill(draw, thick + 194, header_y, 56, pill_h, c_accent,
                       text=f"Y{imu_yaw:+.0f}°", font=font_sm, text_color=c_text)
    else:
        draw_lcars_pill(draw, thick + 84, header_y, 85, pill_h, c_tomato,
                       text="IMU OFFLINE", font=font_sm, text_color=c_text)
    
    # === Main content area ===
    main_y = header_y + pill_h + margin + 2
    main_h = disp_height - main_y - thin - 16  # Leave room for footer
    content_x = thick + margin
    content_w = disp_width - content_x - margin
    
    # Divider between hexapod and ToF (vertical line with rounded ends)
    div_x = content_x + content_w // 2
    div_y1 = main_y + 2
    div_y2 = main_y + main_h - 2
    draw.rectangle((div_x - 1, div_y1, div_x + 1, div_y2), fill=c_frame)
    draw.ellipse((div_x - 2, div_y1 - 2, div_x + 2, div_y1 + 2), fill=c_frame)
    draw.ellipse((div_x - 2, div_y2 - 2, div_x + 2, div_y2 + 2), fill=c_frame)
    
    # Hexapod visualization (left half)
    hex_size = min((div_x - content_x) // 2 - margin, main_h // 2) + 5
    hex_center_x = content_x + (div_x - content_x) // 2
    hex_center_y = main_y + main_h // 2
    
    if joint_angles_valid and joint_angles and len(joint_angles) == 18:
        avg_temp, avg_voltage, max_temp, hottest_label = draw_hexapod_wireframe_3d(
            draw,
            center_x=hex_center_x,
            center_y=hex_center_y,
            size=hex_size,
            joint_angles=joint_angles,
            servo_temps=servo_temps,
            servo_voltages=servo_voltages,
            temp_min=temp_min,
            temp_max=temp_max,
            azimuth_deg=view_azimuth,
            elevation_deg=view_elevation,
            body_pitch_deg=imu_pitch if imu_connected else 0.0,
            body_roll_deg=imu_roll if imu_connected else 0.0,
        )
    else:
        avg_temp, avg_voltage, max_temp, hottest_label = draw_hexapod_heatmap(
            draw,
            center_x=hex_center_x,
            center_y=hex_center_y,
            size=hex_size,
            servo_temps=servo_temps,
            servo_voltages=servo_voltages,
            temp_min=temp_min,
            temp_max=temp_max,
        )
    
    # ToF heatmap (right half)
    tof_x = div_x + margin + 2
    tof_y = main_y + 2
    tof_w = disp_width - tof_x - margin - 2
    tof_h = main_h - 4
    
    draw_tof_heatmap(
        draw,
        x=tof_x,
        y=tof_y,
        width=tof_w,
        height=tof_h,
        distances=tof_distances if tof_distances else [],
        statuses=tof_statuses if tof_statuses else [],
        connected=tof_connected,
        show_closest=True,
        title=None,  # We'll add LCARS-style title
        distance_position='top',  # Show distance above the heatmap
    )
    
    # ToF label pill
    draw_lcars_pill(draw, div_x + 4, main_y - 1, 36, 12, c_violet,
                   text="ToF", font=font_sm, text_color=c_text)
    
    # === Footer: Voltage and status ===
    footer_y = disp_height - thin - 16
    
    # Voltage bar with LCARS styling
    volt_pct = max(0, min(1, (avg_voltage - volt_min) / (volt_max - volt_min))) if volt_max > volt_min else 0
    bar_x = content_x
    bar_w = 110
    bar_h = 12
    
    # Bar background (dark)
    draw.rectangle((bar_x, footer_y, bar_x + bar_w, footer_y + bar_h), fill=(30, 30, 40))
    
    # Filled portion with gradient color
    if volt_pct > 0.5:
        bar_color = c_ice
    elif volt_pct > 0.2:
        bar_color = c_accent
    else:
        bar_color = c_tomato
    
    fill_w = int(bar_w * volt_pct)
    if fill_w > 0:
        draw.rectangle((bar_x, footer_y, bar_x + fill_w, footer_y + bar_h), fill=bar_color)
    
    # Voltage text pill
    draw_lcars_pill(draw, bar_x + bar_w + 4, footer_y - 1, 52, bar_h + 2, c_frame,
                   text=f"{avg_voltage:.1f}V", font=font_md, text_color=c_text)
    
    # Temperature pill
    if max_temp > 0:
        temp_color = c_ice if max_temp < (temp_min + temp_max) / 2 else (
            c_accent if max_temp < temp_max - 5 else c_tomato
        )
        draw_lcars_pill(draw, bar_x + bar_w + 60, footer_y - 1, 56, bar_h + 2, temp_color,
                       text=f"{max_temp:.0f}°C", font=font_md, text_color=c_text)
    
    # Status pill (right side)
    if joint_angles_valid:
        draw_lcars_pill(draw, disp_width - 58, footer_y - 1, 55, bar_h + 2, c_violet,
                       text="ACTIVE", font=font_md, text_color=c_text)
    else:
        draw_lcars_pill(draw, disp_width - 58, footer_y - 1, 55, bar_h + 2, colors['gray'],
                       text="STANDBY", font=font_md, text_color=(120, 120, 120))
    
    return img


def draw_engineering_view(
    image: Image.Image,
    disp_width: int,
    disp_height: int,
    imu_connected: bool,
    imu_roll: float,
    imu_pitch: float,
    imu_yaw: float,
    tof_connected: bool,
    tof_distances: List[int],
    tof_statuses: List[int],
    servo_temps: List[float] = None,
    servo_voltages: List[float] = None,
    temp_min: float = 25.0,
    temp_max: float = 55.0,
    volt_min: float = 6.0,
    volt_max: float = 8.4,
    joint_angles: List[float] = None,
    joint_angles_valid: bool = False,
    view_azimuth: float = 0.0,
    view_elevation: float = 30.0,
) -> Image.Image:
    """Draw the engineering view with IMU, ToF, and servo 3D visualization.
    
    Layout (320x170):
    - Row 1 (y=0-16): IMU text header + title
    - Row 2 (y=18-142): Left=3D hexapod wireframe (or heatmap fallback), Right=ToF heatmap
    - Row 3 (y=144-168): Voltage bar + hottest servo indicator
    
    Args:
        image: Base image (typically black background)
        disp_width: Display width (320)
        disp_height: Display height (170)
        imu_connected: Whether IMU sensor is connected
        imu_roll, imu_pitch, imu_yaw: IMU orientation in degrees
        tof_connected: Whether ToF sensor is connected
        tof_distances: Flat list of 64 distance values (mm)
        tof_statuses: Flat list of 64 target status values
        servo_temps: List of 18 servo temperatures (°C)
        servo_voltages: List of 18 servo voltages (V)
        temp_min: Temperature (°C) for cool color (default 25)
        temp_max: Temperature (°C) for hot color (default 55)
        volt_min: Voltage (V) for low/red color (default 6.0)
        volt_max: Voltage (V) for high/green color (default 8.4)
        joint_angles: List of 18 joint angles in degrees (from S6 telemetry)
        joint_angles_valid: Whether joint_angles contains valid data
        view_azimuth: Horizontal rotation for 3D view (degrees)
        view_elevation: Vertical tilt for 3D view (degrees)
    
    Returns:
        Modified image with engineering visualization
    """
    img = image.copy()
    draw = ImageDraw.Draw(img)
    
    # Ensure we have servo data arrays
    if servo_temps is None:
        servo_temps = [0.0] * 18
    if servo_voltages is None:
        servo_voltages = [0.0] * 18
    
    # Layout constants
    header_h = 16
    footer_h = 24
    main_h = disp_height - header_h - footer_h
    margin = 4
    
    # --- Header: IMU text + Title ---
    try:
        font = get_font(10)
        if imu_connected:
            imu_text = f"R:{imu_roll:+5.1f}° P:{imu_pitch:+5.1f}° Y:{imu_yaw:+5.1f}°"
            imu_color = (100, 255, 100)
        else:
            imu_text = "IMU: --"
            imu_color = (150, 150, 150)
        draw.text((margin, 2), imu_text, fill=imu_color, font=font)
        
        # Title on right
        title = "ENGINEERING"
        bbox = draw.textbbox((0, 0), title, font=font)
        tw = bbox[2] - bbox[0]
        draw.text((disp_width - tw - margin, 2), title, fill=(150, 150, 150), font=font)
    except Exception:
        pass
    
    # --- Main area: Hexapod (left) + ToF (right) ---
    main_y = header_h + 2
    half_w = disp_width // 2
    
    # Hexapod visualization (left half)
    # Use 3D wireframe when joint angles are valid, else fall back to heatmap
    hex_size = min(half_w - margin * 2, main_h - margin) // 2 + 10
    hex_center_x = half_w // 2
    hex_center_y = main_y + main_h // 2
    
    if joint_angles_valid and joint_angles and len(joint_angles) == 18:
        # 3D wireframe view
        avg_temp, avg_voltage, max_temp, hottest_label = draw_hexapod_wireframe_3d(
            draw,
            center_x=hex_center_x,
            center_y=hex_center_y,
            size=hex_size,
            joint_angles=joint_angles,
            servo_temps=servo_temps,
            servo_voltages=servo_voltages,
            temp_min=temp_min,
            temp_max=temp_max,
            azimuth_deg=view_azimuth,
            elevation_deg=view_elevation,
            body_pitch_deg=imu_pitch if imu_connected else 0.0,
            body_roll_deg=imu_roll if imu_connected else 0.0,
        )
    else:
        # Fallback to flat heatmap when no joint data
        avg_temp, avg_voltage, max_temp, hottest_label = draw_hexapod_heatmap(
            draw,
            center_x=hex_center_x,
            center_y=hex_center_y,
            size=hex_size,
            servo_temps=servo_temps,
            servo_voltages=servo_voltages,
            temp_min=temp_min,
            temp_max=temp_max,
        )
    
    # ToF heatmap (right half)
    tof_x = half_w + margin
    tof_y = main_y + margin
    tof_w = half_w - margin * 2
    tof_h = main_h - margin * 2
    
    draw_tof_heatmap(
        draw,
        x=tof_x,
        y=tof_y,
        width=tof_w,
        height=tof_h,
        distances=tof_distances if tof_distances else [],
        statuses=tof_statuses if tof_statuses else [],
        connected=tof_connected,
        show_closest=True,
        title="ToF",
    )
    
    # --- Footer: Voltage bar + hottest servo ---
    footer_y = disp_height - footer_h + 2
    
    # Voltage bar (left 2/3)
    bar_width = disp_width * 2 // 3 - margin * 2
    bar_height = footer_h - 6
    draw_voltage_bar(
        draw,
        x=margin,
        y=footer_y,
        width=bar_width,
        height=bar_height,
        voltage=avg_voltage,
        min_v=volt_min,
        max_v=volt_max,
    )
    
    # Hottest servo indicator (right 1/3)
    try:
        font = get_font(10)
        if max_temp > 0:
            hot_text = f"MAX:{max_temp:.0f}°C"
            hot_color = getColor(temp_min, temp_max, max_temp, TEMPERATURE_COLOR_PALETTE)
        else:
            hot_text = "TEMP:--"
            hot_color = (150, 150, 150)
        info_x = disp_width * 2 // 3 + margin
        draw.text((info_x, footer_y + 2), hot_text, fill=hot_color, font=font)
        if hottest_label:
            draw.text((info_x, footer_y + 12), hottest_label, fill=(180, 180, 180), font=get_font(8))
    except Exception:
        pass
    
    return img


# -----------------------------------------------------------------------------
# Display Update Function
# -----------------------------------------------------------------------------

def UpdateDisplay(
    disp: Any,
    image: Image.Image,
    menu: Optional[Image.Image],
    servo: Optional[List] = None,
    legs: Optional[List] = None,
    state: Optional[List] = None,
    mirror: bool = False,
    menuState: Optional[str] = None,
    teensy_connected: bool = True,
    controller_connected: bool = True,
    telemetry_stale: bool = False,
    robot_enabled: bool = True,
    safety_active: bool = False,
    safety_text: str = "",
    mars_menu: Any = None,
    force_display_update_callback: Optional[callable] = None,
    power_palette: List[Tuple[int, int, int]] = None,
    temperature_palette: List[Tuple[int, int, int]] = None,
    ctrl: Any = None,
) -> None:
    """Updates the display with the given image.
    
    Optimizations applied:
    - Frame change detection: skips SPI write if frame unchanged (via hash)
    - Combined rotation + RGB565: single-pass show_image_rotated()
    - Avoids unnecessary copy when no overlay drawing needed
    
    Args:
        disp: Display object with show_image_rotated() method
        image: Base PIL Image to render
        menu: Menu overlay image (optional)
        servo: Servo telemetry list [[voltage, temp, enable], ...]
        legs: Leg status list
        state: S1 telemetry state list
        mirror: If True, also show on CV2 window
        menuState: Current menu state string
        teensy_connected: If False, draws 'NO TEENSY' watermark overlay
        controller_connected: If False, draws 'NO CONTROLLER' watermark overlay
        telemetry_stale: If True, draws 'NO TELEMETRY' watermark overlay
        robot_enabled: If False, draws 'DISABLED' watermark overlay
        safety_active: If True, show safety lockout screen
        safety_text: Text for safety overlay
        mars_menu: MarsMenu instance (optional)
        force_display_update_callback: Callback to set force update flag
        power_palette: Color palette for voltage display
        temperature_palette: Color palette for temperature display
        ctrl: Controller instance for accessing structured telemetry
    """
    global _last_frame_hash
    
    if power_palette is None:
        power_palette = POWER_COLOR_PALETTE
    if temperature_palette is None:
        temperature_palette = TEMPERATURE_COLOR_PALETTE
    
    # Safety overlay: replace normal content with full-screen safety view
    if safety_active:
        imageCopy = Image.new("RGB", (disp.height, disp.width), (255, 230, 0))
        draw = ImageDraw.Draw(imageCopy)
        font = get_font(26)
        text = safety_text if safety_text else "SAFETY LOCKOUT"
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        x = (disp.height - text_width) // 2
        y = (disp.width - text_height) // 2
        draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
        draw.text((x, y), text, fill="BLACK", font=font)
    else:
        # Only copy if we need to draw overlays
        has_status_overlay = (not teensy_connected or not controller_connected or
                              telemetry_stale or not robot_enabled)
        needs_overlay = (servo is not None and (menuState == "data" or menuState == "settings")) or has_status_overlay
        if needs_overlay:
            imageCopy = image.copy()
        else:
            imageCopy = image
        
        # Draw connection/status overlays
        overlay_texts = []
        if not teensy_connected:
            overlay_texts.append(("NO TEENSY", "RED"))
        elif telemetry_stale:
            overlay_texts.append(("NO TELEMETRY", "ORANGE"))
        if not controller_connected:
            overlay_texts.append(("NO CONTROLLER", "RED"))
        
        # Skip DISABLED overlay in engineering mode (menuState == None means engineering view)
        is_engineering = (menuState is None)
        show_disabled = not robot_enabled and teensy_connected and not telemetry_stale and not is_engineering

        if overlay_texts:
            draw = ImageDraw.Draw(imageCopy)
            font = get_font(28)
            line_height = 36
            total_height = len(overlay_texts) * line_height
            start_y = (disp.width - total_height) // 2
            
            for i, (text, color) in enumerate(overlay_texts):
                bbox = draw.textbbox((0, 0), text, font=font)
                text_width = bbox[2] - bbox[0]
                x = (disp.height - text_width) // 2
                y = start_y + i * line_height
                draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
                draw.text((x, y), text, fill=color, font=font)
        
        if show_disabled:
            draw = ImageDraw.Draw(imageCopy)
            font = get_font(28)
            text = "DISABLED"
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            x = (disp.height - text_width) // 2
            y = disp.width - 40
            draw.text((x + 2, y + 2), text, fill="BLACK", font=font)
            draw.text((x, y), text, fill="YELLOW", font=font)
    
    # Add leg/servo visualization (suppressed when in safety overlay)
    if servo is not None and not safety_active:
        draw = ImageDraw.Draw(imageCopy)

        if menuState == "data":
            # Draw gait text
            _sys = getattr(ctrl, 'system_telem', None) if ctrl is not None else None
            gait_id = None
            if _sys is not None and getattr(_sys, 'valid', False):
                gait_id = getattr(_sys, 'gait', None)
            elif state is not None and len(state) > IDX_GAIT:
                try:
                    gait_id = int(state[IDX_GAIT])
                except Exception:
                    gait_id = None
            gait = getGait(int(gait_id)) if gait_id is not None else "---"
            if not robot_enabled:
                gait = "DISABLED"
            font24 = get_font(24)
            textSize = draw.textbbox(((disp.height / 2), disp.width - 24), gait, font=font24, anchor="mt")
            center = disp.height / 2
            blackoutSpace = 70
            draw.rectangle((center - blackoutSpace, disp.width - 24, center + blackoutSpace, disp.width), fill="BLACK")
            draw.text((textSize[0], textSize[1]), gait, fill="WHITE", font=font24)

            for idx, (voltage, temperature, enable) in enumerate(servo):
                vColor = "GRAY"
                tColor = "GRAY"            
                width = 10
                height = 10
                gap = 79.5
                leg = int(idx / 3)
                left = (idx - (leg * 3)) * (width + 2)
                top = gap * (2 - leg)
                dir = 1
                borderColor = "WHITE"
                contact_ok = True
                _leg_telem = getattr(ctrl, 'leg_telem', None) if ctrl is not None else None
                if _leg_telem is not None and 0 <= leg < len(_leg_telem) and getattr(_leg_telem[leg], 'valid', False):
                    contact_ok = bool(getattr(_leg_telem[leg], 'contact', True))
                elif legs is not None and 0 <= leg < len(legs) and len(legs[leg]) > 0:
                    contact_ok = (legs[leg][0] != 0)
                if not contact_ok:
                    borderColor = "RED"
                if enable == 1:
                    vColor = getColor(10.5, 12.5, voltage, power_palette)
                    tColor = getColor(20, 75, temperature, temperature_palette)
                leg = int(idx / 3)
                left = (idx - (leg * 3)) * (width + 2)
                top = int(gap * (2 - leg))
                dir = 1
                if leg >= 3:
                    leg -= 3
                    dir = -1
                    left = disp.height - left
                    top = int(gap * leg)
                draw.rectangle((left, top, left + dir * 4, top + height), tColor)
                draw.rectangle((left + dir * 5, top, left + dir * 9, top + height), vColor)
                draw.rectangle((left, top, left + dir * width, top + height), outline=borderColor, width=1)
        elif menuState == "settings":
            imageCopy = Image.blend(imageCopy, menu, 0.6)
    
    # MARS menu overlay (takes priority over old menu system)
    if mars_menu is not None and mars_menu.visible:
        mars_menu.render()
        imageCopy = mars_menu.image.copy()
    
    # Store mirror image in buffer for main-thread cv2.imshow
    # (cv2 GUI functions must run in main thread on Linux/X11)
    global _mirror_lock, _mirror_image, _mirror_active, _mirror_menu
    with _mirror_lock:
        _mirror_active = mirror
        _mirror_menu = mars_menu
        if mirror:
            image2 = np.array(imageCopy)
            height, width = image2.shape[:2]
            _mirror_image = cv.resize(
                cv.cvtColor(image2, cv.COLOR_RGB2BGR),
                (width * 2, height * 2),
                interpolation=cv.INTER_LANCZOS4
            )
        else:
            _mirror_image = None
    
    # Frame change detection with minimum SPI write interval
    global _last_frame_hash, _last_spi_write_time
    
    now = time.monotonic()
    time_since_last_write = now - _last_spi_write_time
    
    # Skip SPI write if too soon (prevents tearing during fast animations)
    if time_since_last_write < MIN_SPI_WRITE_INTERVAL_S:
        return
    
    img_array = np.asarray(imageCopy)
    frame_hash = hash(img_array[::4, ::4, :].tobytes())
    
    if frame_hash != _last_frame_hash:
        _last_frame_hash = frame_hash
        _last_spi_write_time = now
        disp.show_image_rotated(imageCopy, rotation_k=3)


# -----------------------------------------------------------------------------
# Splash/Logo Drawing
# -----------------------------------------------------------------------------

def drawLogo(disp: Any) -> None:
    """Draws an ASCII art logo on the display."""
    image = Image.new("RGB", (disp.height, disp.width), "BLACK")
    draw = ImageDraw.Draw(image)

    fontMICR = get_font(12)
    Color = "WHITE"
    Top = 10
    lineHeight = 8
    draw.text((20, Top), u"  __  __    _    ____   ____   ____  \n", fill=Color, font=fontMICR)
    draw.text((20, Top + lineHeight), u" |  \\/  |  / \\  |  _ \\ / ___| / ___|\n", fill=Color, font=fontMICR)
    draw.text((20, Top + 2 * lineHeight), u" | |\\/| | / _ \\ | |_) | |  _  \\___ \\ \n", fill=Color, font=fontMICR)
    draw.text((20, Top + 3 * lineHeight), u" | |  | |/ ___ \\|  __/| |_| | ___) |\n", fill=Color, font=fontMICR)
    draw.text((20, Top + 4 * lineHeight), u" |_|  |_/_/   \\_\\_|    \\____||____/ \n", fill=Color, font=fontMICR)
    draw.text((20, Top + 5 * lineHeight), u"   Modular Autonomous Robotic System\n", fill=Color, font=fontMICR)

    UpdateDisplay(disp, image, None)


def drawMarsSplash(disp: Any, fw_version: str, ctrl_version: str, ctrl_build: int) -> None:
    """Draw a Mars splash image (real photo) on black with version text overlaid.

    Image credit: ESA & MPS for OSIRIS Team (Rosetta, 2007). CC BY-SA 3.0 IGO.
    """
    canvas_w = disp.height
    canvas_h = disp.width
    image = Image.new("RGB", (canvas_w, canvas_h), "BLACK")

    asset_path = os.path.join(os.path.dirname(__file__), "assets", "mars.jpg")
    try:
        mars_img = Image.open(asset_path).convert("RGB")
        max_dim = int(min(canvas_w, canvas_h) * 0.85)
        mars_img.thumbnail((max_dim, max_dim), Image.Resampling.LANCZOS)
        mx = (canvas_w - mars_img.width) // 2
        my = (canvas_h - mars_img.height) // 2
        image.paste(mars_img, (mx, my))
    except Exception as e:
        draw = ImageDraw.Draw(image)
        r = int(min(canvas_w, canvas_h) * 0.32)
        cx, cy = canvas_w // 2, canvas_h // 2
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(190, 70, 40))
        print(f"WARN: Mars splash image not found ({e}); using fallback.", end="\r\n")

    draw = ImageDraw.Draw(image)

    font_title = get_font(16)
    font_small = get_font(12)
    title = "MARS"
    ver = f"FW {fw_version}  |  Ctrl {ctrl_version}/b{int(ctrl_build)}"

    def _shadow_text(x, y, text, font, fill=(255, 255, 255)):
        draw.text((x + 1, y + 1), text, fill=(0, 0, 0), font=font)
        draw.text((x, y), text, fill=fill, font=font)

    _shadow_text(10, 8, title, font_title, fill=(255, 255, 255))
    _shadow_text(10, canvas_h - 20, ver, font_small, fill=(255, 255, 255))

    UpdateDisplay(disp, image, None)


# -----------------------------------------------------------------------------
# Startup Splash with Scrolling Log
# -----------------------------------------------------------------------------

class StartupSplash:
    """Startup splash display with Mars image background and scrolling log.
    
    Shows Mars image as the full background with a semi-transparent overlay,
    and a scrolling text log for startup progress messages.
    """
    
    def __init__(self, disp: Any, fw_version: str, ctrl_version: str, ctrl_build: int):
        """Initialize the startup splash.
        
        Args:
            disp: Display object with show_image_rotated() method
            fw_version: Firmware version string
            ctrl_version: Controller version string
            ctrl_build: Controller build number
        """
        self.disp = disp
        self.canvas_w = disp.height  # Rotated display
        self.canvas_h = disp.width
        self.fw_version = fw_version
        self.ctrl_version = ctrl_version
        self.ctrl_build = ctrl_build
        
        # Log area configuration - starts after title area
        self.log_y_start = 24  # Leave room for title at top
        self.log_font_size = 10
        self.log_line_height = 12
        self.log_max_lines = (self.canvas_h - self.log_y_start - 16) // self.log_line_height
        self.log_lines: List[Tuple[str, str]] = []  # (text, color)
        
        # Load Mars image once - scale to fit height while preserving aspect ratio
        self.bg_img = None
        asset_path = os.path.join(os.path.dirname(__file__), "assets", "mars.jpg")
        try:
            mars_raw = Image.open(asset_path).convert("RGB")
            # Scale to fit height, center horizontally
            img_w, img_h = mars_raw.size
            scale = self.canvas_h / img_h
            new_w = int(img_w * scale)
            new_h = self.canvas_h
            mars_raw = mars_raw.resize((new_w, new_h), Image.Resampling.LANCZOS)
            # Create black background and paste centered image
            bg = Image.new("RGB", (self.canvas_w, self.canvas_h), (0, 0, 0))
            left = (self.canvas_w - new_w) // 2
            bg.paste(mars_raw, (left, 0))
            # Darken the image to make text readable
            from PIL import ImageEnhance
            enhancer = ImageEnhance.Brightness(bg)
            self.bg_img = enhancer.enhance(0.4)  # 40% brightness
        except Exception:
            pass
        
        # Draw initial splash
        self._render()
    
    def log(self, text: str, color: str = "WHITE", append: bool = False) -> None:
        """Add a log line and refresh the display.
        
        Args:
            text: Log message text
            color: Color name (WHITE, GREEN, YELLOW, RED, CYAN, etc.)
            append: If True, append text to the last log line instead of adding a new line
        """
        if append and self.log_lines:
            # Append to the last line
            last_text, last_color = self.log_lines[-1]
            self.log_lines[-1] = (last_text + text, last_color)
        else:
            self.log_lines.append((text, color))
            # Keep only the most recent lines that fit
            if len(self.log_lines) > self.log_max_lines:
                self.log_lines = self.log_lines[-self.log_max_lines:]
        self._render()
    
    def _render(self) -> None:
        """Render the splash screen with current log content."""
        # Start with Mars background or black
        if self.bg_img is not None:
            image = self.bg_img.copy()
        else:
            image = Image.new("RGB", (self.canvas_w, self.canvas_h), (30, 15, 10))
        draw = ImageDraw.Draw(image)
        
        # Draw title and version at top
        font_title = get_font(14)
        font_small = get_font(10)
        
        def _shadow_text(x, y, text, font, fill=(255, 255, 255)):
            # Double shadow for better readability on image background
            draw.text((x + 1, y + 1), text, fill=(0, 0, 0), font=font)
            draw.text((x - 1, y + 1), text, fill=(0, 0, 0), font=font)
            draw.text((x, y), text, fill=fill, font=font)
        
        _shadow_text(8, 4, "MARS", font_title)
        ver_text = f"FW {self.fw_version} | Ctrl {self.ctrl_version}/b{self.ctrl_build}"
        _shadow_text(self.canvas_w - len(ver_text) * 5 - 8, 6, ver_text, font_small, fill=(200, 200, 200))
        
        # Draw log lines with shadow for readability
        font_log = get_font(self.log_font_size)
        y = self.log_y_start
        for text, color in self.log_lines:
            try:
                # Shadow for readability on image background
                draw.text((5, y + 1), text, fill=(0, 0, 0), font=font_log)
                draw.text((3, y + 1), text, fill=(0, 0, 0), font=font_log)
                draw.text((4, y), text, fill=color, font=font_log)
            except Exception:
                draw.text((4, y), text, fill=(255, 255, 255), font=font_log)
            y += self.log_line_height
        
        # Display the image
        UpdateDisplay(self.disp, image, None)
    
    def finish(self, success: bool = True) -> None:
        """Mark startup complete with success/failure indicator.
        
        Args:
            success: If True, show green "READY"; if False, show red "FAILED"
        """
        if success:
            self.log("Startup complete", "GREEN")
        else:
            self.log("Startup FAILED", "RED")


# -----------------------------------------------------------------------------
# DisplayThread Class
# -----------------------------------------------------------------------------

class DisplayThread(threading.Thread):
    """Background thread for eye animation and LCD display updates.
    
    Runs at configurable Hz (default 15), independently of main loop timing.
    Owns the SimpleEyes instance and performs SPI writes to the display.
    Main thread communicates state changes via thread-safe shared state.
    """
    
    def __init__(self, disp: Any, eyes: Any, menu: Any, target_hz: float = 15.0,
                 look_range_x: float = 20.0, look_range_y: float = 10.0,
                 blink_frame_divisor: int = 2):
        super().__init__(daemon=True, name="DisplayThread")
        self.disp = disp
        self.eyes = eyes
        self.menu = menu
        self.blink_frame_divisor = max(1, blink_frame_divisor)
        self.target_hz = target_hz
        self.period_s = 1.0 / target_hz
        
        # Thread control
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # Shared state (protected by lock)
        self._servo = [[0, 0, 0]] * 18
        self._legs = [[0, 0, 0]] * 6
        self._state = [0.0] * 10
        self._mirror = False
        self._menu_state = None
        self._force_update = False
        self._verbose = False
        self._look_x = 0.0
        self._look_y = 0.0
        self._teensy_connected = True
        self._controller_connected = True
        self._telemetry_stale = False
        self._robot_enabled = True
        self._safety_active = False
        self._safety_text = ""
        self.look_range_x = look_range_x
        self.look_range_y = look_range_y
        self._base_center_offset = self.eyes.eye_center_offset
        self._base_vertical_offset = self.eyes.eye_vertical_offset
        
        # External references (set after init)
        self._mars_menu = None
        self._ctrl = None
        self._power_palette = POWER_COLOR_PALETTE
        self._temperature_palette = TEMPERATURE_COLOR_PALETTE
        
        # Safety display thresholds (Pi-side visualization)
        self._volt_min = 10.5
        self._volt_warn = 11.0
        self._volt_max = 12.5
        self._temp_min = 25.0
        self._temp_max = 55.0
        
        # Low-pass filtered battery voltage (average of all servos)
        self._filtered_voltage = 0.0     # Current filtered voltage value
        self._voltage_alpha = 0.1        # Low-pass filter coefficient (0.1 = slow, 0.5 = fast)
        
        # Battery icon display (EYES mode)
        self._show_battery_icon = True   # Enable battery icon on eyes display
        
        # IMU state
        self._imu_connected = False
        self._imu_roll = 0.0
        self._imu_pitch = 0.0
        self._imu_yaw = 0.0
        self._imu_show_overlay = True  # Toggle for IMU display
        
        # ToF state
        self._tof_connected = False
        self._tof_distances = []  # Flat list of 64 distances (mm)
        self._tof_statuses = []   # Flat list of 64 status values
        
        # 3D view state for engineering display
        self._eng_view_azimuth = 0.0    # Horizontal rotation (degrees)
        self._eng_view_elevation = 30.0  # Vertical tilt (degrees), 0=side, 90=top
        self._joint_angles = [0.0] * 18  # Joint angles in degrees (from S6)
        self._joint_angles_valid = False
        self._eng_lcars_theme = True    # Use LCARS theme for engineering view
        self._lcars_palette = 0         # LCARS palette index (0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD)
        
        # Display mode (EYES, ENGINEERING, MENU)
        self._display_mode = DisplayMode.EYES
        
        # Startup suppression: prevent rendering until startup splash is complete
        self._startup_complete = False

    def set_startup_complete(self, complete: bool = True) -> None:
        """Signal that startup splash is complete and display thread can render.
        
        Args:
            complete: True to allow rendering, False to suppress display updates
        """
        with self._lock:
            self._startup_complete = complete

    def set_mars_menu(self, mars_menu: Any) -> None:
        """Set the MarsMenu reference for overlay rendering."""
        with self._lock:
            self._mars_menu = mars_menu

    def set_controller(self, ctrl: Any) -> None:
        """Set the Controller instance for structured telemetry access."""
        with self._lock:
            self._ctrl = ctrl

    def set_palettes(self, power: List, temperature: List) -> None:
        """Set color palettes for servo visualization."""
        with self._lock:
            self._power_palette = power
            self._temperature_palette = temperature

    def set_safety_thresholds(self, volt_min: float = None, volt_warn: float = None,
                               volt_max: float = None,
                               temp_min: float = None, temp_max: float = None) -> None:
        """Set voltage/temperature display thresholds for engineering view."""
        with self._lock:
            if volt_min is not None:
                self._volt_min = volt_min
            if volt_warn is not None:
                self._volt_warn = volt_warn
            if volt_max is not None:
                self._volt_max = volt_max
            if temp_min is not None:
                self._temp_min = temp_min
            if temp_max is not None:
                self._temp_max = temp_max

    def set_show_battery_icon(self, show: bool) -> None:
        """Enable or disable battery icon on EYES display mode."""
        with self._lock:
            self._show_battery_icon = show

    def set_engineering_lcars(self, use_lcars: bool) -> None:
        """Enable or disable LCARS theme for engineering display."""
        with self._lock:
            self._eng_lcars_theme = use_lcars

    def set_lcars_palette(self, palette: int) -> None:
        """Set the LCARS color palette for engineering display.
        
        Args:
            palette: 0=Classic, 1=Nemesis, 2=LowerDecks, 3=PADD
        """
        with self._lock:
            self._lcars_palette = max(0, min(3, palette))

    def set_base_vertical_offset(self, offset: float) -> None:
        """Update the baseline vertical eye offset used for joystick look."""
        with self._lock:
            self._base_vertical_offset = offset
        
    def update_state(self, servo=None, legs=None, state=None, mirror=None, 
                     menu_state=None, force_update=False, verbose=None,
                     look_x=None, look_y=None, teensy_connected=None,
                     controller_connected=None, telemetry_stale=None,
                     robot_enabled=None, safety_active=None,
                     safety_text=None) -> None:
        """Thread-safe update of display state from main thread."""
        with self._lock:
            if servo is not None:
                self._servo = servo
            if legs is not None:
                self._legs = legs
            if state is not None:
                self._state = state
            if mirror is not None:
                self._mirror = mirror
            if menu_state is not None:
                self._menu_state = menu_state
            if force_update:
                self._force_update = True
            if verbose is not None:
                self._verbose = verbose
            if look_x is not None:
                self._look_x = look_x
            if look_y is not None:
                self._look_y = look_y
            if teensy_connected is not None:
                self._teensy_connected = teensy_connected
            if controller_connected is not None:
                self._controller_connected = controller_connected
            if telemetry_stale is not None:
                self._telemetry_stale = telemetry_stale
            if robot_enabled is not None:
                self._robot_enabled = robot_enabled
            if safety_active is not None:
                self._safety_active = safety_active
            if safety_text is not None:
                self._safety_text = safety_text
    
    def update_imu(self, connected: bool = None, roll: float = None, 
                   pitch: float = None, yaw: float = None,
                   show_overlay: bool = None) -> None:
        """Thread-safe update of IMU state from main thread."""
        with self._lock:
            if connected is not None:
                self._imu_connected = connected
            if roll is not None:
                self._imu_roll = roll
            if pitch is not None:
                self._imu_pitch = pitch
            if yaw is not None:
                self._imu_yaw = yaw
            if show_overlay is not None:
                self._imu_show_overlay = show_overlay
    
    def update_tof(self, connected: bool = None, distances: List[int] = None,
                   statuses: List[int] = None) -> None:
        """Thread-safe update of ToF sensor state from main thread."""
        with self._lock:
            if connected is not None:
                self._tof_connected = connected
            if distances is not None:
                self._tof_distances = distances[:]  # Copy to avoid race
            if statuses is not None:
                self._tof_statuses = statuses[:]
    
    def update_joint_angles(self, angles: List[float]) -> None:
        """Thread-safe update of joint angles from S6 telemetry.
        
        Args:
            angles: List of 18 joint angles in degrees 
                    [LF_coxa, LF_femur, LF_tibia, LM_..., LR_..., RF_..., RM_..., RR_...]
        """
        with self._lock:
            if angles and len(angles) == 18:
                self._joint_angles = angles[:]
                self._joint_angles_valid = True
    
    def adjust_view_angle(self, d_azimuth: float = 0.0, d_elevation: float = 0.0) -> None:
        """Adjust 3D view angles for engineering display (called from D-pad).
        
        Args:
            d_azimuth: Change in horizontal rotation (degrees), + = CW from top
            d_elevation: Change in vertical tilt (degrees), + = more top-down
        """
        with self._lock:
            self._eng_view_azimuth = (self._eng_view_azimuth + d_azimuth) % 360.0
            self._eng_view_elevation = max(10.0, min(80.0, self._eng_view_elevation + d_elevation))
    
    def set_display_mode(self, mode: DisplayMode) -> None:
        """Thread-safe update of display mode."""
        with self._lock:
            self._display_mode = mode
    
    def stop(self) -> None:
        """Signal thread to stop and wait for it to finish."""
        self._stop_event.set()
        self.join(timeout=1.0)
    
    def run(self) -> None:
        """Main thread loop: update eyes and display at target Hz."""
        try:
            os.nice(5)
        except (OSError, AttributeError):
            pass
        
        next_tick = time.monotonic()
        last_look_x = 0.0
        last_look_y = 0.0
        blink_frame_counter = 0
        last_teensy_connected = True
        last_controller_connected = True
        last_telemetry_stale = False
        last_robot_enabled = True
        last_safety_active = False
        last_safety_text = ""
        last_imu_roll = 0.0
        last_imu_pitch = 0.0
        last_display_mode = DisplayMode.EYES
        
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    startup_complete = self._startup_complete
                    servo = self._servo
                    legs = self._legs
                    state = self._state
                    mirror = self._mirror
                    menu_state = self._menu_state
                    force = self._force_update
                    self._force_update = False
                    look_x = self._look_x
                    look_y = self._look_y
                    teensy_connected = self._teensy_connected
                    controller_connected = self._controller_connected
                    telemetry_stale = self._telemetry_stale
                    robot_enabled = self._robot_enabled
                    safety_active = self._safety_active
                    safety_text = self._safety_text
                    # IMU state
                    imu_connected = self._imu_connected
                    imu_roll = self._imu_roll
                    imu_pitch = self._imu_pitch
                    imu_yaw = self._imu_yaw
                    imu_show_overlay = self._imu_show_overlay
                    # ToF state
                    tof_connected = self._tof_connected
                    tof_distances = self._tof_distances[:]
                    tof_statuses = self._tof_statuses[:]
                    # Display mode
                    display_mode = self._display_mode
                    mars_menu = self._mars_menu
                    ctrl = self._ctrl
                    power_palette = self._power_palette
                    temperature_palette = self._temperature_palette
                    # 3D view state
                    joint_angles = self._joint_angles[:]
                    joint_angles_valid = self._joint_angles_valid
                    view_azimuth = self._eng_view_azimuth
                    view_elevation = self._eng_view_elevation
                    eng_lcars_theme = self._eng_lcars_theme
                    lcars_palette = self._lcars_palette
                
                status_changed = (teensy_connected != last_teensy_connected or 
                                 controller_connected != last_controller_connected or
                                 telemetry_stale != last_telemetry_stale or
                                 robot_enabled != last_robot_enabled or
                                 safety_active != last_safety_active or
                                 safety_text != last_safety_text)
                if status_changed:
                    last_teensy_connected = teensy_connected
                    last_controller_connected = controller_connected
                    last_telemetry_stale = telemetry_stale
                    last_robot_enabled = robot_enabled
                    last_safety_active = safety_active
                    last_safety_text = safety_text
                
                look_changed = (abs(look_x - last_look_x) > 0.05 or 
                               abs(look_y - last_look_y) > 0.05)
                if look_changed:
                    last_look_x = look_x
                    last_look_y = look_y
                
                # Check if IMU changed significantly (only matters for engineering view, NOT eyes)
                imu_changed = (abs(imu_roll - last_imu_roll) > 0.5 or 
                              abs(imu_pitch - last_imu_pitch) > 0.5)
                if imu_changed:
                    last_imu_roll = imu_roll
                    last_imu_pitch = imu_pitch
                
                # Detect display mode changes
                mode_changed = (display_mode != last_display_mode)
                if mode_changed:
                    last_display_mode = display_mode
                
                # Engineering mode always needs render (ToF data updates continuously)
                engineering_active = (display_mode == DisplayMode.ENGINEERING)
                
                if abs(last_look_x) > 0.01 or abs(last_look_y) > 0.01:
                    self.eyes.eye_center_offset = self._base_center_offset + last_look_x * self.look_range_x
                    self.eyes.eye_vertical_offset = self._base_vertical_offset + last_look_y * self.look_range_y
                
                # Only force eye re-render for look/status changes, NOT for IMU (IMU is for engineering view only)
                needs_render = self.eyes.update(force_update=look_changed or status_changed or mode_changed)
                
                is_blinking = (self.eyes.blinkState == 1 or self.eyes.blinkState == 2)
                if is_blinking and self.blink_frame_divisor > 1:
                    blink_frame_counter = (blink_frame_counter + 1) % self.blink_frame_divisor
                    if blink_frame_counter != 0:
                        needs_render = False
                else:
                    blink_frame_counter = 0
                
                # Determine if we need to update the display
                # EYES mode: only update on eye changes (not IMU drift)
                # ENGINEERING mode: update on IMU changes too
                # Always update on mode change
                # Skip ALL rendering if startup is not complete (splash screen owns the display)
                if not startup_complete:
                    # Just sleep and wait for startup to finish
                    time.sleep(self.period_s)
                    continue
                
                should_update = (needs_render or force or look_changed or status_changed or
                                mode_changed or (engineering_active and imu_changed) or engineering_active)
                
                if should_update:
                    # Choose display content based on display mode
                    if display_mode == DisplayMode.ENGINEERING:
                        # Engineering view: IMU + ToF + Servo heat map
                        base_image = Image.new("RGB", (self.disp.height, self.disp.width), "BLACK")
                        # Extract servo temps and voltages from servo array [[voltage, temp, enabled], ...]
                        servo_temps = []
                        servo_voltages = []
                        if servo and len(servo) >= 18:
                            for i in range(18):
                                if servo[i] and len(servo[i]) >= 2:
                                    servo_voltages.append(float(servo[i][0]) if servo[i][0] else 0.0)
                                    servo_temps.append(float(servo[i][1]) if servo[i][1] else 0.0)
                                else:
                                    servo_voltages.append(0.0)
                                    servo_temps.append(0.0)
                        else:
                            servo_temps = [0.0] * 18
                            servo_voltages = [0.0] * 18
                        
                        # Choose LCARS or basic engineering view
                        if eng_lcars_theme:
                            display_image = draw_lcars_engineering_view(
                                base_image,
                                disp_width=self.disp.height,  # Note: swapped due to rotation
                                disp_height=self.disp.width,
                                imu_connected=imu_connected,
                                imu_roll=imu_roll,
                                imu_pitch=imu_pitch,
                                imu_yaw=imu_yaw,
                                tof_connected=tof_connected,
                                tof_distances=tof_distances,
                                tof_statuses=tof_statuses,
                                servo_temps=servo_temps,
                                servo_voltages=servo_voltages,
                                temp_min=self._temp_min,
                                temp_max=self._temp_max,
                                volt_min=self._volt_min,
                                volt_max=self._volt_max,
                                joint_angles=joint_angles,
                                joint_angles_valid=joint_angles_valid,
                                view_azimuth=view_azimuth,
                                view_elevation=view_elevation,
                                lcars_palette=lcars_palette,
                            )
                        else:
                            display_image = draw_engineering_view(
                                base_image,
                                disp_width=self.disp.height,  # Note: swapped due to rotation
                                disp_height=self.disp.width,
                                imu_connected=imu_connected,
                                imu_roll=imu_roll,
                                imu_pitch=imu_pitch,
                                imu_yaw=imu_yaw,
                                tof_connected=tof_connected,
                                tof_distances=tof_distances,
                                tof_statuses=tof_statuses,
                                servo_temps=servo_temps,
                                servo_voltages=servo_voltages,
                                temp_min=self._temp_min,
                                temp_max=self._temp_max,
                                volt_min=self._volt_min,
                                volt_max=self._volt_max,
                                joint_angles=joint_angles,
                                joint_angles_valid=joint_angles_valid,
                                view_azimuth=view_azimuth,
                                view_elevation=view_elevation,
                            )
                    elif display_mode == DisplayMode.MENU:
                        # Menu mode: just use eyes as base (menu overlays on top)
                        display_image = self.eyes.display_image
                    else:
                        # EYES mode (default): eyes with battery icon and low battery warning
                        display_image = self.eyes.display_image
                        
                        # Calculate average voltage from all servos (more stable than min)
                        raw_voltage = 0.0
                        if servo and len(servo) >= 18:
                            volt_sum = 0.0
                            volt_count = 0
                            for i in range(18):
                                if servo[i] and len(servo[i]) >= 1 and servo[i][0]:
                                    v = float(servo[i][0])
                                    if v > 0:
                                        volt_sum += v
                                        volt_count += 1
                            if volt_count > 0:
                                raw_voltage = volt_sum / volt_count
                        
                        # Apply low-pass filter for smooth display
                        if raw_voltage > 0:
                            if self._filtered_voltage <= 0:
                                # First valid reading: initialize filter
                                self._filtered_voltage = raw_voltage
                            else:
                                # Exponential moving average: new = alpha * raw + (1-alpha) * old
                                self._filtered_voltage = (
                                    self._voltage_alpha * raw_voltage +
                                    (1.0 - self._voltage_alpha) * self._filtered_voltage
                                )
                        
                        # Battery icon overlay (phone-style, top-right)
                        if self._show_battery_icon and self._filtered_voltage > 0:
                            display_image = draw_battery_icon(
                                display_image,
                                voltage=self._filtered_voltage,
                                volt_min=self._volt_min,
                                volt_max=self._volt_max,
                            )
                        
                        # Low battery warning bar (overrides icon when critical)
                        if self._filtered_voltage > 0 and self._filtered_voltage < self._volt_warn:
                            display_image = draw_low_battery_warning(
                                display_image,
                                voltage=self._filtered_voltage,
                                width=self.disp.height,  # Swapped due to rotation
                                height=self.disp.width,
                            )
                    
                    UpdateDisplay(
                        self.disp, display_image,
                        self.menu._image if self.menu else None,
                        servo, legs, state, mirror, menu_state,
                        teensy_connected=teensy_connected,
                        controller_connected=controller_connected,
                        telemetry_stale=telemetry_stale,
                        robot_enabled=robot_enabled,
                        safety_active=safety_active,
                        safety_text=safety_text,
                        mars_menu=mars_menu,
                        power_palette=power_palette,
                        temperature_palette=temperature_palette,
                        ctrl=ctrl,
                    )
                
            except (AttributeError, TypeError, OSError):
                pass
            
            next_tick += self.period_s
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()


# -----------------------------------------------------------------------------
# Main-Thread Mirror Display Function
# -----------------------------------------------------------------------------

def show_mirror_window(force_update_callback=None) -> None:
    """Display the mirror window from the main thread.
    
    This function must be called from the main thread because cv2.imshow
    requires the main thread on Linux/X11 systems.
    
    Args:
        force_update_callback: Optional callback to trigger display update
                               after keyboard input is processed.
    """
    global _mirror_lock, _mirror_image, _mirror_active, _mirror_menu
    
    with _mirror_lock:
        active = _mirror_active
        image = _mirror_image
        menu = _mirror_menu
    
    if active and image is not None:
        cv.imshow("M.A.R.S. — Modular Autonomous Robotic System", image)
        key = cv.waitKey(1) & 0xFF
        # Handle keyboard input for menu navigation
        if key != 255 and menu is not None and menu.visible:
            if key in (ord('w'), ord('W'), 82):       # W or Up arrow
                menu.nav_up()
            elif key in (ord('s'), ord('S'), 84):     # S or Down arrow
                menu.nav_down()
            elif key in (ord('a'), ord('A'), 81):     # A or Left arrow
                menu.nav_left()
            elif key in (ord('d'), ord('D'), 83):     # D or Right arrow
                menu.nav_right()
            elif key in (9,):                          # Tab
                menu.handle_button('RB')
            elif key in (353,):                        # Shift+Tab
                menu.handle_button('LB')
            elif key in (ord('j'), ord('J')):         # J = decrement
                menu.decrement()
            elif key in (ord('l'), ord('L')):         # L = increment
                menu.increment()
            elif key in (10, 13, ord(' ')):           # Enter or Space
                menu.select()
            elif key in (27, ord('q'), ord('Q')):     # Escape or Q
                menu.hide()
            if force_update_callback is not None:
                force_update_callback()
    else:
        cv.destroyAllWindows()


# -----------------------------------------------------------------------------
# Module exports
# -----------------------------------------------------------------------------
__all__ = [
    # Enums
    'DisplayMode',
    # Font/color utilities
    'get_font', 'reset_frame_hash', 'getColor',
    # Color palettes
    'POWER_COLOR_PALETTE', 'TEMPERATURE_COLOR_PALETTE', 'TOF_COLOR_BREAKPOINTS',
    # ToF hold-last-valid
    'reset_tof_hold_buffers', 'TOF_MAX_HOLD_FRAMES', 'TOF_STALE_DIM_FACTOR',
    # Drawing functions
    'drawLogo', 'drawMarsSplash', 'draw_imu_overlay', 'draw_tof_heatmap',
    'draw_engineering_view', 'get_tof_color', 'draw_hexapod_heatmap', 'draw_voltage_bar',
    # Leg/servo constants
    'LEG_NAMES', 'JOINT_NAMES', 'LEG_ANGLES',
    # Main display functions
    'UpdateDisplay',
    # Mirror display (main thread)
    'show_mirror_window',
    # Threading
    'DisplayThread',
]

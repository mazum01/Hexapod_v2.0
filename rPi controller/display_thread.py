"""
display_thread.py — Display rendering and LCD update routines for MARS controller.

Extracted from controller.py as part of Phase 5 modularization (2025-06-18).
Contains DisplayThread class, UpdateDisplay function, and display helpers.
"""

from __future__ import annotations
import os
import time
import threading
from typing import Optional, Any, List, Tuple

import numpy as np
import cv2 as cv
from PIL import Image, ImageDraw, ImageFont

# Import telemetry for structured data access
from telemetry import IDX_GAIT, getGait, get_safety_overlay_text


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
# Frame Change Detection
# -----------------------------------------------------------------------------

_last_frame_hash = None


def reset_frame_hash():
    """Reset the frame hash to force a display update."""
    global _last_frame_hash
    _last_frame_hash = None


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
        
        show_disabled = not robot_enabled and teensy_connected and not telemetry_stale

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
    
    if mirror:
        image2 = np.array(imageCopy)                        
        height, width = image2.shape[:2]
        image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width * 2, height * 2), interpolation=cv.INTER_LANCZOS4)
        cv.imshow("M.A.R.S. — Modular Autonomous Robotic System", image2)
        key = cv.waitKey(1) & 0xFF
        if key != 255 and mars_menu is not None and mars_menu.visible:
            if key in (ord('w'), ord('W'), 82):
                mars_menu.nav_up()
            elif key in (ord('s'), ord('S'), 84):
                mars_menu.nav_down()
            elif key in (ord('a'), ord('A'), 81):
                mars_menu.nav_left()
            elif key in (ord('d'), ord('D'), 83):
                mars_menu.nav_right()
            elif key in (9,):
                mars_menu.handle_button('RB')
            elif key in (353,):
                mars_menu.handle_button('LB')
            elif key in (ord('j'), ord('J')):
                mars_menu.decrement()
            elif key in (ord('l'), ord('L')):
                mars_menu.increment()
            elif key in (10, 13, ord(' ')):
                mars_menu.select()
            elif key in (27, ord('q'), ord('Q')):
                mars_menu.hide()
            if force_display_update_callback is not None:
                force_display_update_callback()
    else:
        cv.destroyAllWindows()
    
    # Frame change detection
    img_array = np.asarray(imageCopy)
    frame_hash = hash(img_array[::4, ::4, :].tobytes())
    
    if frame_hash != _last_frame_hash:
        _last_frame_hash = frame_hash
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
        
        while not self._stop_event.is_set():
            try:
                with self._lock:
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
                    mars_menu = self._mars_menu
                    ctrl = self._ctrl
                    power_palette = self._power_palette
                    temperature_palette = self._temperature_palette
                
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
                
                if abs(last_look_x) > 0.01 or abs(last_look_y) > 0.01:
                    self.eyes.eye_center_offset = self._base_center_offset + last_look_x * self.look_range_x
                    self.eyes.eye_vertical_offset = self._base_vertical_offset + last_look_y * self.look_range_y
                
                needs_render = self.eyes.update(force_update=look_changed or status_changed)
                
                is_blinking = (self.eyes.blinkState == 1 or self.eyes.blinkState == 2)
                if is_blinking and self.blink_frame_divisor > 1:
                    blink_frame_counter = (blink_frame_counter + 1) % self.blink_frame_divisor
                    if blink_frame_counter != 0:
                        needs_render = False
                else:
                    blink_frame_counter = 0
                
                if needs_render or force or look_changed or status_changed:
                    UpdateDisplay(
                        self.disp, self.eyes.display_image,
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
# Module exports
# -----------------------------------------------------------------------------
__all__ = [
    # Font/color utilities
    'get_font', 'reset_frame_hash', 'getColor',
    # Color palettes
    'POWER_COLOR_PALETTE', 'TEMPERATURE_COLOR_PALETTE',
    # Drawing functions
    'drawLogo', 'drawMarsSplash',
    # Main display functions
    'UpdateDisplay',
    # Threading
    'DisplayThread',
]

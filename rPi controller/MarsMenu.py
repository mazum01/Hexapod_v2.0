"""
MARS Menu System - Modular Autonomous Robotic System
A compact, touch and joystick-friendly menu for the hexapod robot.
Screen size: 320x170 pixels (Waveshare 1.9" LCD)

Design:
- Tabbed interface to fit content in limited space
- Left sidebar with category tabs
- Right content area with scrollable items
- Works with DPAD/joystick and touchscreen
"""

from PIL import Image, ImageDraw, ImageFont
import math


class MenuCategory:
    """Enumeration of menu categories (tabs)."""
    EYES = 0
    GAIT = 1
    POSTURE = 2
    INFO = 3
    SAFETY = 4
    SYSTEM = 5
    PID = 6
    IMP = 7
    EST = 8
    IMU = 9
    TOF = 10
    AUTO = 11  # Autonomy behaviors
    COUNT = 12  # Total number of categories


class MenuTheme:
    """Available menu visual themes."""
    MARS = 0      # Default dark blue theme
    LCARS = 1     # Star Trek LCARS inspired
    COUNT = 2
    NAMES = ["MARS", "LCARS"]


class LcarsPalette:
    """LCARS color palettes from TheLCARS.com."""
    CLASSIC = 0       # TNG/DS9 - oranges, peach, violet
    NEMESIS = 1       # Nemesis Blue - cool blues
    LOWER_DECKS = 2   # Lower Decks - warm oranges
    PADD = 3          # Lower Decks PADD - arctic blues
    COUNT = 4
    NAMES = ["Classic", "Nemesis", "LwrDecks", "PADD"]


class MenuItem:
    """A single menu item that can be a value selector, action, or info display."""
    
    def __init__(self, label, item_type="action", value=None, min_val=None, max_val=None,
                 step=1, options=None, unit="", on_change=None, on_select=None, format_func=None):
        """
        Args:
            label: Display name
            item_type: "action" (button), "value" (adjustable number), "option" (cycle through list), "info" (read-only)
            value: Current value (for value/option types)
            min_val/max_val: Range limits (for value type)
            step: Increment amount (for value type)
            options: List of options (for option type)
            unit: Unit suffix (e.g., "mm", "ms", "%")
            on_change: Callback(new_value) when value changes
            on_select: Callback() when action is selected
            format_func: Custom formatting function for display
        """
        self.label = label
        self.item_type = item_type
        self.value = value
        self.min_val = min_val
        self.max_val = max_val
        self.step = step
        self.options = options or []
        self.unit = unit
        self.on_change = on_change
        self.on_select = on_select
        self.format_func = format_func
    
    def get_display_value(self):
        """Get the formatted display string for the current value."""
        if self.item_type == "info":
            if self.format_func:
                return self.format_func(self.value)
            return str(self.value) if self.value is not None else "---"
        elif self.item_type == "option":
            if self.options and 0 <= self.value < len(self.options):
                return self.options[self.value]
            return "---"
        elif self.item_type == "value":
            if self.format_func:
                return self.format_func(self.value)
            return f"{self.value}{self.unit}"
        return ""
    
    def adjust(self, direction):
        """Adjust the value by direction (-1 or +1). Returns True if changed."""
        if self.item_type == "option":
            if self.options:
                old_val = self.value
                self.value = (self.value + direction) % len(self.options)
                if self.value != old_val and self.on_change:
                    self.on_change(self.value)
                return self.value != old_val
        elif self.item_type == "value":
            old_val = self.value
            new_val = self.value + direction * self.step
            if self.min_val is not None:
                new_val = max(self.min_val, new_val)
            if self.max_val is not None:
                new_val = min(self.max_val, new_val)
            self.value = new_val
            if self.value != old_val and self.on_change:
                self.on_change(self.value)
            return self.value != old_val
        return False
    
    def select(self):
        """Execute the action for this item. Returns True if handled."""
        if self.item_type == "action" and self.on_select:
            self.on_select()
            return True
        return False


class MarsMenu:
    """
    Compact tabbed menu system for the MARS hexapod.
    
    Layout (320x170):
    ┌────────┬─────────────────────────────┐
    │  TABS  │      CONTENT AREA           │
    │  (50px)│         (270px)             │
    │        │                             │
    │ [EYES] │  Eye Style    [ANIME  ◄►]   │
    │ [GAIT] │  Eye Color    [Blue   ◄►]   │
    │ [POST] │                             │
    │ [INFO] │                             │
    │ [SYS ] │                             │
    └────────┴─────────────────────────────┘
    """
    
    # Layout constants
    WIDTH = 320
    HEIGHT = 170
    TAB_WIDTH = 55  # Wider tabs for touch
    CONTENT_X = TAB_WIDTH
    CONTENT_WIDTH = WIDTH - TAB_WIDTH
    # Maximum number of tabs to show per page; excess tabs spill to additional pages
    MAX_TABS_PER_PAGE = 5
    
    # Appearance - MARS theme (default)
    BG_COLOR = (20, 20, 25)
    TAB_COLOR = (40, 40, 50)
    TAB_SELECTED_COLOR = (60, 80, 120)
    ITEM_COLOR = (35, 35, 45)
    ITEM_SELECTED_COLOR = (50, 100, 150)
    TEXT_COLOR = (220, 220, 220)
    TEXT_DIM_COLOR = (120, 120, 130)
    ACCENT_COLOR = (100, 180, 255)
    VALUE_COLOR = (180, 220, 255)
    ARROW_COLOR = (150, 150, 160)
    
    # LCARS color palettes (from TheLCARS.com)
    LCARS_BLACK = (0, 0, 0)
    
    # Classic palette (TNG/DS9)
    LCARS_CLASSIC = {
        'orange': (255, 136, 0),        # #ff8800 - orange
        'gold': (255, 170, 0),          # #ffaa00 - gold
        'peach': (255, 136, 102),       # #ff8866 - peach
        'sunflower': (255, 204, 153),   # #ffcc99 - sunflower
        'african_violet': (204, 153, 255),  # #cc99ff - african-violet
        'lilac': (204, 85, 255),        # #cc55ff - lilac
        'almond': (255, 170, 144),      # #ffaa90 - almond
        'ice': (153, 204, 255),         # #99ccff - ice
        'sky': (170, 170, 255),         # #aaaaff - sky
        'tomato': (255, 85, 85),        # #ff5555 - tomato
        'gray': (102, 102, 136),        # #666688 - gray
    }
    
    # Nemesis Blue palette
    LCARS_NEMESIS = {
        'midnight': (34, 51, 255),      # #2233ff - midnight
        'evening': (34, 102, 255),      # #2266ff - evening
        'cool': (102, 153, 255),        # #6699ff - cool
        'ghost': (136, 187, 255),       # #88bbff - ghost
        'moonbeam': (235, 240, 255),    # #ebf0ff - moonbeam
        'grape': (153, 102, 204),       # #9966cc - grape
        'tangerine': (255, 136, 51),    # #ff8833 - tangerine
        'honey': (255, 204, 153),       # #ffcc99 - honey
        'cardinal': (204, 34, 51),      # #cc2233 - cardinal
        'wheat': (204, 170, 136),       # #ccaa88 - wheat
        'galaxy_gray': (82, 82, 106),   # #52526a - galaxy-gray
    }
    
    # Lower Decks palette
    LCARS_LOWER_DECKS = {
        'orange': (255, 119, 0),        # #ff7700 - orange
        'daybreak': (255, 153, 17),     # #ff9911 - daybreak
        'harvestgold': (255, 170, 68),  # #ffaa44 - harvestgold
        'honey': (255, 204, 153),       # #ffcc99 - honey
        'butter': (255, 238, 204),      # #ffeecc - butter
        'october_sunset': (255, 68, 0), # #ff4400 - october-sunset
        'rich_pumpkin': (204, 85, 0),   # #cc5500 - rich-pumpkin
    }
    
    # Lower Decks PADD palette
    LCARS_PADD = {
        'alpha_blue': (85, 136, 238),   # #5588ee - alpha-blue
        'beta_blue': (119, 153, 221),   # #7799dd - beta-blue
        'arctic_ice': (102, 204, 255),  # #66ccff - arctic-ice
        'arctic_snow': (153, 204, 255), # #99ccff - arctic-snow
        'radioactive': (136, 255, 255), # #88ffff - radioactive
        'night_cloud': (52, 68, 112),   # #344470 - night-cloud
        'night_rain': (69, 85, 128),    # #455580 - night-rain
        'sunset_red': (255, 53, 0),     # #ff3500 - sunset-red
    }
    
    # Tab labels (ASCII text for font compatibility)
    TAB_LABELS = ["EYE", "GAIT", "POSE", "INFO", "SAFE", "SYS", "PID", "IMP", "EST", "IMU", "TOF", "AUTO"]
    TAB_NAMES = ["Eyes", "Gait", "Pose", "Info", "Safety", "Sys", "PID", "IMP", "EST", "IMU", "ToF", "Auto"]

    # Visual ordering for the sidebar tab stack.
    # Keep MenuCategory numeric IDs stable (they are used as keys) and
    # control the UI tab order via this mapping.
    TAB_ORDER = [
        MenuCategory.INFO,
        MenuCategory.IMU,
        MenuCategory.TOF,
        MenuCategory.AUTO,
        MenuCategory.SYSTEM,
        MenuCategory.EYES,
        MenuCategory.GAIT,
        MenuCategory.POSTURE,
        MenuCategory.SAFETY,
        MenuCategory.PID,
        MenuCategory.IMP,
        MenuCategory.EST,
    ]
    
    def __init__(self, touch=None):
        """Initialize the menu system."""
        self._touch = touch
        self._visible = False
        self._current_tab = MenuCategory.EYES
        self._selected_item = 0
        self._scroll_offset = 0
        self._theme = MenuTheme.MARS  # Default theme
        self._lcars_palette = LcarsPalette.CLASSIC  # Default LCARS palette
        
        # Font setup - larger fonts for touch-friendly UI
        self._font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        try:
            self._font = ImageFont.truetype(self._font_path, 14)
            self._font_small = ImageFont.truetype(self._font_path, 11)
        except IOError:
            self._font = ImageFont.load_default()
            self._font_small = self._font
        
        # Build menu items for each category
        self._items = {
            MenuCategory.EYES: [],
            MenuCategory.GAIT: [],
            MenuCategory.POSTURE: [],
            MenuCategory.INFO: [],
            MenuCategory.SAFETY: [],
            MenuCategory.SYSTEM: [],
            MenuCategory.PID: [],
            MenuCategory.IMP: [],
            MenuCategory.EST: [],
            MenuCategory.IMU: [],
            MenuCategory.TOF: [],
            MenuCategory.AUTO: [],
        }
        
        # Callbacks will be set by controller
        self._callbacks = {}
        
        # Build default items (values will be updated by controller)
        self._build_menu_items()
        
        # Pre-render image
        self._image = None
        self._needs_render = True
       
        # Touch debouncing: track if we're currently touching
        self._touch_active = False
        # First-tap ignore: skip N touch events after wake
        self._ignore_touches = 0

    def _get_tab_paging(self):
        """Compute paging window for tabs.

        Returns (start_index, end_index, visible_count, max_per_page, page_index, page_count).
        """
        total = len(self.TAB_ORDER)
        if total <= 0:
            return 0, 0, 0, 1, 0, 1

        max_per = min(self.MAX_TABS_PER_PAGE, total)
        page_count = (total + max_per - 1) // max_per

        # Derive page from current tab position within TAB_ORDER
        try:
            current_pos = self.TAB_ORDER.index(self._current_tab)
        except ValueError:
            current_pos = 0
        page = current_pos // max_per
        if page >= page_count:
            page = page_count - 1
        start = page * max_per
        end = min(start + max_per, total)
        visible = max(0, end - start)
        return start, end, visible, max_per, page, page_count
    
    def _build_menu_items(self):
        """Build the menu item structure."""
        # === EYES ===
        self._items[MenuCategory.EYES] = [
            MenuItem("Style", "option", value=0,
                    options=["Ellipse", "Rectangle", "RoundRect", "X", "Spider", "Human", "Cat", "Hypno", "Anime"]),
            MenuItem("Color", "option", value=0,
                    options=["Orange", "White", "Blue", "Red", "Green", "Custom"]),
            MenuItem("Human Color", "option", value=0,
                    options=["Blue", "Green", "Hazel", "Brown", "DkBrown"]),
            MenuItem("Size", "value", value=33, min_val=15, max_val=50, step=2, unit="px"),
            MenuItem("V Center", "value", value=0, min_val=-30, max_val=30, step=2, unit="px"),
            MenuItem("Spacing", "value", value=25, min_val=10, max_val=45, step=1, unit="%"),
            MenuItem("CRT Effect", "option", value=1, options=["Off", "On"]),
        ]
        
        # === GAIT ===
        self._items[MenuCategory.GAIT] = [
            MenuItem("Type", "option", value=0,
                    options=["Tripod", "Wave", "Ripple", "Stationary"]),
            MenuItem("Step Height", "value", value=60, min_val=20, max_val=100, step=5, unit="mm"),
            MenuItem("Step Length", "value", value=100, min_val=50, max_val=175, step=5, unit="mm"),
            MenuItem("Turn Rate", "value", value=60, min_val=10, max_val=90, step=5, unit="deg/s"),
            MenuItem("Cycle Time", "value", value=2000, min_val=500, max_val=4000, step=100, unit="ms"),
            MenuItem("Start Gait", "action"),
            MenuItem("Stop Gait", "action"),
        ]
        
        # === POSTURE ===
        self._items[MenuCategory.POSTURE] = [
            MenuItem("Stand", "action"),
            MenuItem("Tuck", "action"),
            MenuItem("Home", "action"),
            MenuItem("Pounce", "action"),
            MenuItem("P Prep", "value", value=400, min_val=100, max_val=1500, step=25, unit="ms"),
            MenuItem("P Rear", "value", value=250, min_val=50, max_val=1000, step=25, unit="ms"),
            MenuItem("P Lunge", "value", value=250, min_val=50, max_val=1000, step=25, unit="ms"),
            MenuItem("P Recov", "value", value=500, min_val=100, max_val=2000, step=50, unit="ms"),
            MenuItem("P Back1", "value", value=55, min_val=0, max_val=140, step=5, unit="mm"),
            MenuItem("P Back2", "value", value=75, min_val=0, max_val=180, step=5, unit="mm"),
            MenuItem("P Push", "value", value=-60, min_val=-200, max_val=0, step=10, unit="mm"),
            MenuItem("P Strike", "value", value=140, min_val=20, max_val=220, step=10, unit="mm"),
            MenuItem("P Crouch", "value", value=55, min_val=0, max_val=120, step=5, unit="mm"),
            MenuItem("P Lift", "value", value=110, min_val=0, max_val=180, step=5, unit="mm"),
            MenuItem("P FrontZ", "value", value=20, min_val=0, max_val=80, step=5, unit="mm"),
            MenuItem("Stand Height", "value", value=120, min_val=80, max_val=160, step=5, unit="mm"),
            MenuItem("Body Pitch", "value", value=0, min_val=-15, max_val=15, step=1, unit="°"),
            MenuItem("Body Roll", "value", value=0, min_val=-15, max_val=15, step=1, unit="°"),
        ]
        
        # === INFO ===
        self._items[MenuCategory.INFO] = [
            MenuItem("Battery", "info", value=12.6, unit="V",
                format_func=lambda v: f"{v:.1f}V" if v is not None else "---"),
            MenuItem("Current", "info", value=0.0, unit="A",
                format_func=lambda v: f"{v:.2f}A" if v is not None else "---"),
            MenuItem("Loop Time", "info", value=0, unit="μs",
                format_func=lambda v: f"{v}μs" if (v is not None and v > 0) else "---"),
            MenuItem("IMU Pitch", "info", value=0.0,
                    format_func=lambda v: f"{v:.1f}°" if v is not None else "---"),
            MenuItem("IMU Roll", "info", value=0.0,
                    format_func=lambda v: f"{v:.1f}°" if v is not None else "---"),
            MenuItem("Servo Temp", "info", value=0,
                format_func=lambda v: f"{v}°C" if (v is not None and v > 0) else "---"),
            MenuItem("Servo Volt", "info", value=0.0, unit="V",
                format_func=lambda v: f"{v:.1f}V" if v is not None else "---"),
            MenuItem("FW Version", "info", value="0.0.0"),
            MenuItem("Ctrl Version", "info", value="0.0.0"),
        ]

        # === SAFETY ===
        self._items[MenuCategory.SAFETY] = [
            MenuItem("State", "info", value="OK"),
            MenuItem("Cause", "info", value="0x0000"),
            MenuItem("Override", "info", value="0x0000"),
            MenuItem("Clearance", "info", value=0, unit="mm",
                     format_func=lambda v: f"{int(v)}mm" if v is not None else "---"),
            MenuItem("Soft Limits", "info", value="On"),
            MenuItem("Collision", "info", value="On"),
            MenuItem("Temp Lock", "info", value="80C"),
            MenuItem("---Display---", "info", value=""),
            MenuItem("Volt Min", "value", value=10.5, min_val=9.0, max_val=11.5, step=0.1, unit="V",
                     format_func=lambda v: f"{v:.1f}V"),
            MenuItem("Volt Warn", "value", value=11.0, min_val=10.0, max_val=12.0, step=0.1, unit="V",
                     format_func=lambda v: f"{v:.1f}V"),
            MenuItem("Volt Max", "value", value=12.5, min_val=11.0, max_val=14.0, step=0.1, unit="V",
                     format_func=lambda v: f"{v:.1f}V"),
            MenuItem("Temp Min", "value", value=25.0, min_val=15.0, max_val=40.0, step=1.0, unit="°C",
                     format_func=lambda v: f"{v:.0f}°C"),
            MenuItem("Temp Max", "value", value=55.0, min_val=40.0, max_val=80.0, step=1.0, unit="°C",
                     format_func=lambda v: f"{v:.0f}°C"),
            MenuItem("---Low Battery---", "info", value=""),
            MenuItem("LowBatt Prot", "option", value=1, options=["Off", "On"]),
            MenuItem("Volt Critical", "value", value=10.0, min_val=9.0, max_val=11.0, step=0.1, unit="V",
                     format_func=lambda v: f"{v:.1f}V"),
            MenuItem("Volt Recovery", "value", value=10.5, min_val=9.5, max_val=11.5, step=0.1, unit="V",
                     format_func=lambda v: f"{v:.1f}V"),
            MenuItem("LowBatt Filter", "value", value=0.005, min_val=0.001, max_val=0.100, step=0.001, unit="α",
                     format_func=lambda v: f"{v:.3f}"),
            MenuItem("LowBatt Status", "info", value="OK"),
            MenuItem("---Actions---", "info", value=""),
            MenuItem("Clear Safety", "action"),
            MenuItem("Override ALL", "action"),
            MenuItem("Override TEMP", "action"),
            MenuItem("Override COLLISION", "action"),
            MenuItem("Override NONE", "action"),
        ]
        
        # === SYSTEM ===
        self._items[MenuCategory.SYSTEM] = [
            MenuItem("Theme", "option", value=0, options=["MARS", "LCARS"],
                    on_change=self._on_theme_change),
            MenuItem("Palette", "option", value=0, 
                    options=["Classic", "Nemesis", "LwrDecks", "PADD"],
                    on_change=self._on_palette_change),
            MenuItem("Brightness", "value", value=100, min_val=10, max_val=100, step=10, unit="%"),
            MenuItem("Auto-Disable", "value", value=5, min_val=0, max_val=30, step=1, unit="s"),
            MenuItem("Verbose", "option", value=0, options=["Off", "On"]),
            MenuItem("Mirror Display", "option", value=0, options=["Off", "On"]),
            MenuItem("Save All", "action"),
            MenuItem("Shutdown", "action"),
        ]

        # === PID ===
        self._items[MenuCategory.PID] = [
            MenuItem("Enabled", "option", value=0, options=["Off", "On"]),
            MenuItem("Mode", "option", value=0, options=["Active", "Shadow"]),
            MenuItem("Shadow Hz", "value", value=2, min_val=1, max_val=50, step=1, unit="Hz"),
            MenuItem("Kp Coxa", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kp Femur", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kp Tibia", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Ki Coxa", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Ki Femur", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Ki Tibia", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kd Coxa", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kd Femur", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kd Tibia", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("Kdα Coxa", "value", value=200, min_val=0, max_val=1000, step=25),
            MenuItem("Kdα Femur", "value", value=200, min_val=0, max_val=1000, step=25),
            MenuItem("Kdα Tibia", "value", value=200, min_val=0, max_val=1000, step=25),
        ]

        # === IMP ===
        self._items[MenuCategory.IMP] = [
            MenuItem("Enabled", "option", value=0, options=["Off", "On"]),
            MenuItem("Mode", "option", value=0, options=["Off", "Joint", "Cart"]),
            MenuItem("Scale", "value", value=1000, min_val=0, max_val=1000, step=25),
            MenuItem("J Spring Coxa", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Spring Femur", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Spring Tibia", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Damp Coxa", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Damp Femur", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Damp Tibia", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Spring X", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Spring Y", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Spring Z", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Damp X", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Damp Y", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("C Damp Z", "value", value=0, min_val=0, max_val=5000, step=25),
            MenuItem("J Deadband", "value", value=50, min_val=0, max_val=1000, step=10, unit="cd"),
            MenuItem("C Deadband", "value", value=10.0, min_val=0.0, max_val=100.0, step=0.5, unit="mm",
                     format_func=lambda v: f"{v:.1f}mm" if v is not None else "---"),
        ]

        # === EST ===
        self._items[MenuCategory.EST] = [
            MenuItem("Cmd α", "value", value=500, min_val=0, max_val=1000, step=25),
            MenuItem("Meas α", "value", value=500, min_val=0, max_val=1000, step=25),
            MenuItem("Vel α", "value", value=500, min_val=0, max_val=2000, step=25),
        ]

        # === IMU ===
        self._items[MenuCategory.IMU] = [
            MenuItem("Status", "info", value="---"),
            MenuItem("Roll", "info", value=0.0,
                     format_func=lambda v: f"{v:+.1f}°" if v is not None else "---"),
            MenuItem("Pitch", "info", value=0.0,
                     format_func=lambda v: f"{v:+.1f}°" if v is not None else "---"),
            MenuItem("Yaw", "info", value=0.0,
                     format_func=lambda v: f"{v:+.1f}°" if v is not None else "---"),
            MenuItem("Update Rate", "info", value=0,
                     format_func=lambda v: f"{v} Hz" if v is not None and v > 0 else "---"),
            MenuItem("Read Count", "info", value=0,
                     format_func=lambda v: f"{v}" if v is not None else "---"),
            MenuItem("Error Count", "info", value=0,
                     format_func=lambda v: f"{v}" if v is not None else "---"),
            MenuItem("I2C Bus", "info", value=1,
                     format_func=lambda v: f"Bus {v}" if v is not None else "---"),
            MenuItem("I2C Addr", "info", value=0x4A,
                     format_func=lambda v: f"0x{v:02X}" if v is not None else "---"),
            MenuItem("Show Overlay", "option", value=1, options=["Off", "On"]),
            # Body leveling items
            MenuItem("Leveling", "option", value=0, options=["Off", "On"]),
            MenuItem("LVL Gain", "value", value=1.0, min_val=0.0, max_val=2.0, step=0.1,
                     format_func=lambda v: f"{v:.1f}"),
            MenuItem("Max Corr", "value", value=30.0, min_val=5.0, max_val=50.0, step=5.0,
                     format_func=lambda v: f"{v:.0f}mm"),
            MenuItem("Tilt Limit", "value", value=25.0, min_val=10.0, max_val=45.0, step=5.0,
                     format_func=lambda v: f"{v:.0f}°"),
            MenuItem("LVL Corrections", "info", value="---",
                     format_func=lambda v: v if isinstance(v, str) else "---"),
            # Motion lean items (tilt into direction of travel)
            MenuItem("Motion Lean", "option", value=0, options=["Off", "On"]),
            MenuItem("Lean Max", "value", value=7.0, min_val=3.0, max_val=15.0, step=1.0,
                     format_func=lambda v: f"{v:.0f}°"),
        ]

        # === ToF (Time of Flight) ===
        self._items[MenuCategory.TOF] = [
            MenuItem("Enabled", "option", value=1, options=["Off", "On"]),
            MenuItem("Status", "info", value="---"),
            MenuItem("Resolution", "option", value=1, options=["4x4", "8x8"]),
            MenuItem("Frame Rate", "value", value=15, min_val=1, max_val=60, step=1,
                     format_func=lambda v: f"{v} Hz"),
            MenuItem("I2C Bus", "value", value=1, min_val=1, max_val=6, step=1,
                     format_func=lambda v: f"Bus {v}"),
            MenuItem("Sensors", "info", value=0,
                     format_func=lambda v: f"{v} sensor(s)" if v is not None else "---"),
            MenuItem("Add Sensor", "action"),
            # Sensor 1 (default)
            MenuItem("Sensor 1 Name", "info", value="front",
                     format_func=lambda v: v if isinstance(v, str) else "---"),
            MenuItem("Sensor 1 Addr", "info", value=0x29,
                     format_func=lambda v: f"0x{v:02X}" if v is not None else "---"),
            MenuItem("Remove Sensor 1", "action"),
            # Statistics (read-only)
            MenuItem("Read Count", "info", value=0,
                     format_func=lambda v: f"{v}" if v is not None else "---"),
            MenuItem("Error Count", "info", value=0,
                     format_func=lambda v: f"{v}" if v is not None else "---"),
            MenuItem("Closest", "info", value="---",
                     format_func=lambda v: f"{v} mm" if isinstance(v, int) and v > 0 else v if isinstance(v, str) else "---"),
            MenuItem("Temperature", "info", value=0,
                     format_func=lambda v: f"{v}°C" if v is not None else "---"),
            MenuItem("Apply & Restart", "action"),
        ]

        # === Autonomy (Behavior Engine) ===
        self._items[MenuCategory.AUTO] = [
            # Master enable
            MenuItem("Autonomy", "option", value=0, options=["Off", "On"]),
            MenuItem("Status", "info", value="Idle"),
            MenuItem("Active Behavior", "info", value="---"),
            MenuItem("Last Action", "info", value="---"),
            # Individual behavior toggles
            MenuItem("Obstacle Avoid", "option", value=1, options=["Off", "On"]),
            MenuItem("Cliff Detect", "option", value=1, options=["Off", "On"]),
            MenuItem("Caught Foot", "option", value=1, options=["Off", "On"]),
            MenuItem("Patrol", "option", value=0, options=["Off", "On"]),
            # Obstacle avoidance thresholds
            MenuItem("Stop Dist", "value", value=150, min_val=50, max_val=500, step=25,
                     format_func=lambda v: f"{v}mm"),
            MenuItem("Slow Dist", "value", value=300, min_val=100, max_val=800, step=25,
                     format_func=lambda v: f"{v}mm"),
            # Cliff detection
            MenuItem("Cliff Thresh", "value", value=100, min_val=50, max_val=300, step=25,
                     format_func=lambda v: f"{v}mm"),
            # Caught foot recovery
            MenuItem("Snag Error", "value", value=15.0, min_val=5.0, max_val=45.0, step=2.5,
                     format_func=lambda v: f"{v:.1f}°"),
            MenuItem("Snag Timeout", "value", value=500, min_val=100, max_val=2000, step=100,
                     format_func=lambda v: f"{v}ms"),
            MenuItem("Recovery Lift", "value", value=30.0, min_val=10.0, max_val=80.0, step=5.0,
                     format_func=lambda v: f"{v:.0f}mm"),
            # Patrol settings
            MenuItem("Patrol Time", "value", value=60, min_val=0, max_val=600, step=30,
                     format_func=lambda v: f"{v}s" if v > 0 else "∞"),
            MenuItem("Turn Interval", "value", value=10, min_val=2, max_val=60, step=2,
                     format_func=lambda v: f"{v}s"),
            # Actions
            MenuItem("Save Settings", "action"),
        ]
    
    def set_callback(self, category, item_label, callback_type, callback):
        """Set a callback for a menu item.
        
        Args:
            category: MenuCategory enum value
            item_label: Label of the menu item
            callback_type: "on_change" or "on_select"
            callback: The callback function
        """
        for item in self._items.get(category, []):
            if item.label == item_label:
                if callback_type == "on_change":
                    item.on_change = callback
                elif callback_type == "on_select":
                    item.on_select = callback
                break
    
    def set_value(self, category, item_label, value):
        """Set the value of a menu item (for syncing with controller state)."""
        for item in self._items.get(category, []):
            if item.label == item_label:
                item.value = value
                self._needs_render = True
                break
    
    def get_value(self, category, item_label):
        """Get the current value of a menu item."""
        for item in self._items.get(category, []):
            if item.label == item_label:
                return item.value
        return None
    
    @property
    def theme(self):
        """Get current theme."""
        return self._theme
    
    @theme.setter
    def theme(self, value):
        """Set theme (MenuTheme.MARS or MenuTheme.LCARS)."""
        if self._theme != value:
            self._theme = value
            self._needs_render = True
    
    def _on_theme_change(self, value):
        """Internal callback when theme is changed via menu."""
        self._theme = value
        self._needs_render = True
    
    @property
    def lcars_palette(self):
        """Get current LCARS palette."""
        return self._lcars_palette
    
    @lcars_palette.setter
    def lcars_palette(self, value):
        """Set LCARS palette."""
        if self._lcars_palette != value:
            self._lcars_palette = value
            self._needs_render = True
    
    def _on_palette_change(self, value):
        """Internal callback when LCARS palette is changed via menu."""
        self._lcars_palette = value
        self._needs_render = True
    
    @property
    def visible(self):
        return self._visible
    
    @visible.setter
    def visible(self, value):
        if self._visible != value:
            self._visible = value
            self._needs_render = True
    
    def show(self):
        """Show the menu."""
        self._visible = True
        self._needs_render = True
        # Ignore touches until finger lifts once (the wake-up touch)
        self._ignore_touches = 1
    
    def hide(self):
        """Hide the menu."""
        self._visible = False
        self._needs_render = True
    
    def toggle(self):
        """Toggle menu visibility."""
        self._visible = not self._visible
        self._needs_render = True
        return self._visible
    
    # === Navigation ===
    
    def nav_up(self):
        """Move selection up."""
        if not self._visible:
            return
        items = self._items.get(self._current_tab, [])
        if items:
            self._selected_item = (self._selected_item - 1) % len(items)
            self._ensure_visible()
            self._needs_render = True
    
    def nav_down(self):
        """Move selection down."""
        if not self._visible:
            return
        items = self._items.get(self._current_tab, [])
        if items:
            self._selected_item = (self._selected_item + 1) % len(items)
            self._ensure_visible()
            self._needs_render = True
    
    def nav_left(self):
        """Adjust value left or switch to previous tab."""
        if not self._visible:
            return
        items = self._items.get(self._current_tab, [])
        if items and 0 <= self._selected_item < len(items):
            item = items[self._selected_item]
            if item.item_type in ("option", "value"):
                item.adjust(-1)
                self._needs_render = True
            else:
                self.nav_tab_left()
    
    def nav_right(self):
        """Adjust value right or switch to next tab."""
        if not self._visible:
            return
        items = self._items.get(self._current_tab, [])
        if items and 0 <= self._selected_item < len(items):
            item = items[self._selected_item]
            if item.item_type in ("option", "value"):
                item.adjust(1)
                self._needs_render = True
            else:
                self.nav_tab_right()
    
    def nav_tab_left(self):
        """Switch to previous tab."""
        if not self._visible:
            return

        total = len(self.TAB_ORDER)
        if total <= 0:
            return
        try:
            pos = self.TAB_ORDER.index(self._current_tab)
        except ValueError:
            pos = 0
        pos = (pos - 1) % total
        self._current_tab = self.TAB_ORDER[pos]
        self._selected_item = 0
        self._scroll_offset = 0
        self._needs_render = True
    
    def nav_tab_right(self):
        """Switch to next tab."""
        if not self._visible:
            return

        total = len(self.TAB_ORDER)
        if total <= 0:
            return
        try:
            pos = self.TAB_ORDER.index(self._current_tab)
        except ValueError:
            pos = 0
        pos = (pos + 1) % total
        self._current_tab = self.TAB_ORDER[pos]
        self._selected_item = 0
        self._scroll_offset = 0
        self._needs_render = True
    
    def select(self):
        """Select/activate the current item."""
        if not self._visible:
            return False
        items = self._items.get(self._current_tab, [])
        if items and 0 <= self._selected_item < len(items):
            item = items[self._selected_item]
            if item.item_type == "action":
                return item.select()
            elif item.item_type == "option":
                # Cycle to next option on select
                item.adjust(1)
                self._needs_render = True
                return True
        return False
    
    def handle_button(self, button):
        """Handle controller button press.
        
        Args:
            button: Button name ('A', 'B', 'LB', 'RB', 'UP', 'DOWN', 'LEFT', 'RIGHT')
            
        Returns:
            bool: True if button was handled
        """
        if not self._visible:
            return False
        
        if button == 'A':
            return self.select()
        elif button == 'B':
            self.hide()
            return True
        elif button == 'LB':
            self.nav_tab_left()
            return True
        elif button == 'RB':
            self.nav_tab_right()
            return True
        elif button == 'UP':
            self.nav_up()
            return True
        elif button == 'DOWN':
            self.nav_down()
            return True
        elif button == 'LEFT':
            self.nav_left()
            return True
        elif button == 'RIGHT':
            self.nav_right()
            return True
        return False
    
    def _ensure_visible(self):
        """Ensure selected item is visible (scroll if needed)."""
        max_visible = 4  # Max items visible at once (larger items)
        if self._selected_item < self._scroll_offset:
            self._scroll_offset = self._selected_item
        elif self._selected_item >= self._scroll_offset + max_visible:
            self._scroll_offset = self._selected_item - max_visible + 1
    
    # === Touch Handling ===
    
    def touched(self):
        """Check if touch screen is currently being touched.
        
        Returns True if touch detected. Debouncing for value changes
        is handled separately in handle_touch().
        """
        if self._touch:
            self._touch.read_touch_data()
            is_touching = self._touch.point_count > 0
            
            if not is_touching:
                # No touch - release the debounce lock
                self._touch_active = False
                # Decrement ignore counter when finger lifts
                if self._ignore_touches > 0:
                    self._ignore_touches -= 1
            
            return is_touching
        return False
    
    def handle_touch(self):
        """Process touch input. Returns True if touch was handled.
        
        Implements debouncing: value adjustments only happen on NEW touch.
        Tab switches and item selection always work.
        First tap after wake-up is ignored to prevent accidental actions.
        """
        if not self._touch or not self._visible:
            return False
        
        # Ignore touches while counter is active (wake-up protection)
        if self._ignore_touches > 0:
            self._ignore_touches -= 1
            return True  # Consume but don't process
        
        point, coordinates = self._touch.get_touch_xy()
        if not coordinates or not coordinates[0]:
            return False
        
        # Raw touch coordinates (physical screen is 170x320 portrait)
        raw_x = coordinates[0].get('x', 0)
        raw_y = coordinates[0].get('y', 0)
        
        # Transform for display rotation_k=3 (270 deg CW)
        # Physical 170x320 -> Logical 320x170
        # new_x = raw_y, new_y = 170 - raw_x
        x = raw_y
        y = 170 - raw_x
        
        # Check tab touches (left sidebar)
        if x < self.TAB_WIDTH:
            start, end, visible, max_per, page, page_count = self._get_tab_paging()
            if visible == 0:
                return False
            # Tab touch zones depend on theme
            if self._theme == MenuTheme.LCARS:
                # LCARS tabs: start_y=32, tab_height=22, gap=3
                tab_start_y = 32
                tab_height = 22
                tab_gap = 3
                tab_total = tab_height + tab_gap
                
                # Check if touch is in the tab area
                if y >= tab_start_y:
                    relative_y = y - tab_start_y
                    local_idx = relative_y // tab_total
                    # Check if within a tab (not in the gap) and within visible window
                    if local_idx < visible and (relative_y % tab_total) < tab_height:
                        touched_pos = start + local_idx
                        if 0 <= touched_pos < len(self.TAB_ORDER):
                            touched_tab = self.TAB_ORDER[touched_pos]
                        else:
                            return True
                        if touched_tab != self._current_tab:
                            self._current_tab = touched_tab
                            self._selected_item = 0
                            self._scroll_offset = 0
                            self._needs_render = True
                        return True
            else:
                # MARS theme: visible tabs divide height evenly
                tab_height = self.HEIGHT // visible
                local_idx = min(y // tab_height, visible - 1)
                touched_pos = start + local_idx
                if 0 <= touched_pos < len(self.TAB_ORDER):
                    touched_tab = self.TAB_ORDER[touched_pos]
                else:
                    return True
                if touched_tab != self._current_tab:
                    self._current_tab = touched_tab
                    self._selected_item = 0
                    self._scroll_offset = 0
                    self._needs_render = True
                return True
        
        # Check close button (X) in top-right corner
        if x > self.WIDTH - 35 and y < 28:
            self.hide()
            return True
        
        # Check scroll bar touches (right edge, 30px wide)
        items = self._items.get(self._current_tab, [])
        max_visible = 4
        if len(items) > max_visible and x > self.WIDTH - 32:
            # Scroll bar was touched
            content_y_start = 28 + 24  # After title and separator
            track_height = max_visible * 34 - 4
            
            # Calculate thumb position
            scroll_range = len(items) - max_visible
            visible_ratio = max_visible / len(items)
            thumb_height = max(25, int(track_height * visible_ratio))
            thumb_travel = track_height - thumb_height
            thumb_y = content_y_start + int(thumb_travel * (self._scroll_offset / scroll_range)) if scroll_range > 0 else content_y_start
            
            # Tap above thumb = scroll up, below = scroll down
            if not self._touch_active:
                self._touch_active = True
                if y < thumb_y:
                    # Scroll up (show earlier items)
                    self._scroll_offset = max(0, self._scroll_offset - max_visible)
                    self._selected_item = max(0, self._selected_item - max_visible)
                elif y > thumb_y + thumb_height:
                    # Scroll down (show later items)
                    self._scroll_offset = min(scroll_range, self._scroll_offset + max_visible)
                    self._selected_item = min(len(items) - 1, self._selected_item + max_visible)
                self._needs_render = True
            return True
        
        # Check content area touches
        content_y_start = 28  # After title
        item_height = 34  # Larger touch targets
        
        # Calculate which item was touched
        relative_y = y - content_y_start
        if relative_y >= 0:
            touched_idx = self._scroll_offset + (relative_y // item_height)
            if 0 <= touched_idx < len(items):
                self._selected_item = touched_idx
                
                # Check if touch was on left/right arrows for adjustable items
                # Only adjust value on NEW touch (debounced)
                item = items[touched_idx]
                if item.item_type in ("option", "value"):
                    if not self._touch_active:
                        # New touch - adjust value and set debounce
                        self._touch_active = True
                        # Left half = decrease, right half = increase (simple zones)
                        mid_x = (self.TAB_WIDTH + self.WIDTH - 32) // 2  # Exclude scroll bar
                        if x > self.WIDTH - 90:  # Right zone (avoid scroll bar)
                            item.adjust(1)
                        elif x > mid_x:  # Middle-right = also increase
                            item.adjust(1)
                        else:  # Left of center = decrease
                            item.adjust(-1)
                elif item.item_type == "action":
                    if not self._touch_active:
                        self._touch_active = True
                        item.select()
                
                self._needs_render = True
                return True
        
        return False
    
    # === Theme Color Helpers ===
    
    def _get_colors(self):
        """Return color dict based on current theme and palette."""
        if self._theme == MenuTheme.LCARS:
            # Select palette
            if self._lcars_palette == LcarsPalette.NEMESIS:
                p = self.LCARS_NEMESIS
                return {
                    'bg': self.LCARS_BLACK,
                    'tab': p['wheat'],
                    'tab_selected': p['cool'],
                    'tab_colors': [p['wheat'], p['grape'], p['honey'], p['tangerine'], p['ghost']],
                    'item': self.LCARS_BLACK,
                    'item_selected': p['galaxy_gray'],
                    'text': p['moonbeam'],
                    'text_dim': p['ghost'],
                    'accent': p['cool'],
                    'accent_alt': p['grape'],
                    'frame_top': p['cool'],
                    'frame_bottom': p['grape'],
                    'frame_bar': p['midnight'],
                    'value': p['ghost'],
                    'arrow': p['grape'],
                    'border': p['evening'],
                    'close': p['cardinal'],
                    'scroll_track': p['galaxy_gray'],
                    'scroll_thumb': p['tangerine'],
                    'item_alt': p['evening'],
                }
            elif self._lcars_palette == LcarsPalette.LOWER_DECKS:
                p = self.LCARS_LOWER_DECKS
                return {
                    'bg': self.LCARS_BLACK,
                    'tab': p['harvestgold'],
                    'tab_selected': p['orange'],
                    'tab_colors': [p['harvestgold'], p['honey'], p['daybreak'], 
                                   p['butter'], p['rich_pumpkin']],
                    'item': self.LCARS_BLACK,
                    'item_selected': p['rich_pumpkin'],
                    'text': p['butter'],
                    'text_dim': p['honey'],
                    'accent': p['orange'],
                    'accent_alt': p['october_sunset'],
                    'frame_top': p['orange'],
                    'frame_bottom': p['harvestgold'],
                    'frame_bar': p['daybreak'],
                    'value': p['butter'],
                    'arrow': p['daybreak'],
                    'border': p['orange'],
                    'close': p['october_sunset'],
                    'scroll_track': p['rich_pumpkin'],
                    'scroll_thumb': p['honey'],
                    'item_alt': p['harvestgold'],
                }
            elif self._lcars_palette == LcarsPalette.PADD:
                p = self.LCARS_PADD
                return {
                    'bg': self.LCARS_BLACK,
                    'tab': p['night_rain'],
                    'tab_selected': p['arctic_ice'],
                    'tab_colors': [p['alpha_blue'], p['beta_blue'], p['arctic_snow'],
                                   p['night_rain'], p['radioactive']],
                    'item': self.LCARS_BLACK,
                    'item_selected': p['night_cloud'],
                    'text': p['arctic_snow'],
                    'text_dim': p['beta_blue'],
                    'accent': p['arctic_ice'],
                    'accent_alt': p['radioactive'],
                    'frame_top': p['arctic_ice'],
                    'frame_bottom': p['alpha_blue'],
                    'frame_bar': p['beta_blue'],
                    'value': p['radioactive'],
                    'arrow': p['alpha_blue'],
                    'border': p['arctic_ice'],
                    'close': p['sunset_red'],
                    'scroll_track': p['night_cloud'],
                    'scroll_thumb': p['radioactive'],
                    'item_alt': p['night_rain'],
                }
            else:  # Classic palette (default)
                p = self.LCARS_CLASSIC
                return {
                    'bg': self.LCARS_BLACK,
                    'tab': p['almond'],
                    'tab_selected': p['orange'],
                    'tab_colors': [p['peach'], p['african_violet'], p['sunflower'],
                                   p['lilac'], p['almond']],
                    'item': self.LCARS_BLACK,
                    'item_selected': p['gray'],
                    'text': p['sunflower'],
                    'text_dim': p['almond'],
                    'accent': p['orange'],
                    'accent_alt': p['lilac'],         # Secondary accent for variety
                    'frame_top': p['orange'],         # Top frame sweep
                    'frame_bottom': p['african_violet'],  # Bottom frame sweep 
                    'frame_bar': p['peach'],          # Vertical bar between sweeps
                    'value': p['ice'],
                    'arrow': p['african_violet'],
                    'border': p['orange'],
                    'close': p['tomato'],
                    'scroll_track': p['gray'],
                    'scroll_thumb': p['lilac'],
                    'item_alt': p['sky'],             # Alternating item accent
                }
        else:  # MARS theme (default)
            return {
                'bg': self.BG_COLOR,
                'tab': self.TAB_COLOR,
                'tab_selected': self.TAB_SELECTED_COLOR,
                'tab_colors': None,  # Not used for MARS
                'item': self.ITEM_COLOR,
                'item_selected': self.ITEM_SELECTED_COLOR,
                'text': self.TEXT_COLOR,
                'text_dim': self.TEXT_DIM_COLOR,
                'accent': self.ACCENT_COLOR,
                'value': self.VALUE_COLOR,
                'arrow': self.ARROW_COLOR,
                'border': self.TAB_COLOR,
                'close': self.ACCENT_COLOR,
                'scroll_track': self.TAB_COLOR,
                'scroll_thumb': self.ACCENT_COLOR,
            }
    
    # === Rendering ===
    
    def render(self):
        """Render the menu to an image. Returns PIL Image."""
        if not self._needs_render and self._image is not None:
            return self._image
        
        colors = self._get_colors()
        img = Image.new("RGB", (self.WIDTH, self.HEIGHT), colors['bg'])
        draw = ImageDraw.Draw(img)
        
        if not self._visible:
            self._image = img
            self._needs_render = False
            return img
        
        # Draw based on theme
        if self._theme == MenuTheme.LCARS:
            self._draw_lcars(draw, colors)
        else:
            self._draw_mars(draw, colors)
        
        self._image = img
        self._needs_render = False
        return img
    
    def _draw_mars(self, draw, colors):
        """Draw MARS theme (original style)."""
        self._draw_tabs(draw, colors)
        self._draw_content(draw, colors)
    
    def _draw_lcars(self, draw, colors):
        """Draw LCARS theme (Star Trek inspired)."""
        # LCARS uses rounded pill-shaped elements and L-shaped brackets
        self._draw_lcars_frame(draw, colors)
        self._draw_lcars_tabs(draw, colors)
        self._draw_lcars_content(draw, colors)
    
    def _draw_lcars_frame(self, draw, colors):
        """Draw LCARS decorative frame elements.
        
        Following LCARS design guidelines:
        - Frame goes thick to thin at turns
        - Swept corners curve outward (signature LCARS element)
        - Consistent spacing grid
        - Clean vector look (no gradients)
        """
        # === Layout constants (spacing grid) ===
        thick = 12       # Thick bar width
        thin = 6         # Thin bar width  
        sweep_r = 18     # Sweep radius (outer)
        gap = 3          # Standard gap between elements
        
        # Get frame colors (use dedicated keys if available, fallback to accent/tab)
        frame_top = colors.get('frame_top', colors['accent'])
        frame_bottom = colors.get('frame_bottom', colors['tab'])
        frame_bar = colors.get('frame_bar', colors['accent'])
        accent_alt = colors.get('accent_alt', colors['accent'])
        
        # === Top-left sweep (thick vertical to thin horizontal) ===
        # Horizontal thin bar (top, from after sweep to end of tab area)
        draw.rectangle((sweep_r, 0, self.TAB_WIDTH + 25, thin - 1), fill=frame_top)
        
        # Sweep corner (quarter circle, outer) - top left
        draw.pieslice((0, 0, sweep_r * 2, sweep_r * 2), 180, 270, fill=frame_top)
        # Inner cutout (creates the swept effect - thick to thin transition)
        inner_r = sweep_r - thin
        draw.pieslice((thick, thin, thick + inner_r * 2, thin + inner_r * 2), 
                     180, 270, fill=colors['bg'])
        
        # === Bottom-left sweep (thick vertical to thin horizontal) ===
        # Calculate bottom sweep position
        bottom_sweep_y = self.HEIGHT - sweep_r * 2
        
        # Sweep corner (quarter circle, outer) - bottom left
        draw.pieslice((0, bottom_sweep_y, sweep_r * 2, self.HEIGHT), 90, 180, fill=frame_bottom)
        # Inner cutout
        draw.pieslice((thick, bottom_sweep_y + thin, thick + inner_r * 2, self.HEIGHT - thin), 
                     90, 180, fill=colors['bg'])
        
        # Horizontal thin bar (bottom, from after sweep to right)
        draw.rectangle((sweep_r, self.HEIGHT - thin, self.TAB_WIDTH - 10, self.HEIGHT - 1), fill=frame_bottom)
        # Rounded cap at end
        cap_r = thin // 2
        draw.ellipse((self.TAB_WIDTH - 10 - cap_r, self.HEIGHT - thin, 
                     self.TAB_WIDTH - 10 + cap_r, self.HEIGHT - 1), fill=frame_bottom)
        
        # === Vertical thick bar (between the two sweeps) ===
        # Starts after top sweep ends, stops before bottom sweep starts
        bar_top = sweep_r + thin  # Bottom of top sweep inner cutout
        bar_bottom = bottom_sweep_y + thin - 1  # Top of bottom sweep inner cutout
        draw.rectangle((0, bar_top, thick - 1, bar_bottom), fill=frame_bar)
        
        # === Right edge accent bar (alternate color for variety) ===
        # Thin vertical bar on right side of content area
        draw.rectangle((self.WIDTH - thin, thin + gap, self.WIDTH - 1, 24), fill=accent_alt)
    
    def _draw_lcars_tabs(self, draw, colors):
        """Draw LCARS-style tabs with rounded caps.
        
        LCARS guidelines:
        - Rounded caps mark termination points and are used as buttons
        - Text right-aligned inside tab
        - Consistent spacing between tabs
        - Limited color palette (3-5 colors)
        """
        tab_height = 22  # Height for proper proportions
        tab_gap = 3      # Standard LCARS gap
        start_y = 32     # After top frame sweep
        tab_x = 14       # After left frame bar (12px thick + 2px gap)
        
        # Get tab colors from palette (limited to 3 colors for coherent look)
        tab_colors = colors.get('tab_colors') or [colors['tab']] * 5
        
        start, end, visible, max_per, page, page_count = self._get_tab_paging()

        # Page indicator above the tab stack when multiple pages exist.
        # Draw as a small LCARS-style pill so it feels native to the frame.
        if page_count > 1 and visible > 0:
            indicator = f"{page + 1}/{page_count}"
            bbox = draw.textbbox((0, 0), indicator, font=self._font_small)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
            pill_h = text_h + 4
            pill_w = max(text_w + 10, 30)
            # Center the pill horizontally over the tabs but inside the sidebar
            pill_x = tab_x + (self.TAB_WIDTH - tab_x - pill_w) // 2
            pill_y = max(8, start_y - pill_h - 4)

            cap_r = pill_h // 2
            # Left cap
            draw.ellipse((pill_x, pill_y, pill_x + pill_h, pill_y + pill_h - 1),
                         fill=colors['accent'])
            # Body
            draw.rectangle((pill_x + cap_r, pill_y, pill_x + pill_w - cap_r,
                            pill_y + pill_h - 1),
                           fill=colors['accent'])
            # Right cap
            draw.ellipse((pill_x + pill_w - pill_h, pill_y,
                          pill_x + pill_w, pill_y + pill_h - 1),
                         fill=colors['accent'])

            # Black LCARS text centered in the pill
            text_x = pill_x + (pill_w - text_w) // 2
            text_y = pill_y + (pill_h - text_h) // 2
            draw.text((text_x, text_y), indicator, fill=self.LCARS_BLACK,
                      font=self._font_small)

        for visible_idx, pos in enumerate(range(start, end)):
            y1 = start_y + visible_idx * (tab_height + tab_gap)
            y2 = y1 + tab_height - 1

            cat = self.TAB_ORDER[pos]
            
            # Determine tab color
            if cat == self._current_tab:
                tab_color = colors['tab_selected']
            else:
                # Use alternating colors from limited palette
                tab_color = tab_colors[pos % len(tab_colors)]
            
            # Draw tab with rounded cap on left (LCARS cap = termination/button)
            cap_r = tab_height // 2
            # Left rounded cap (full semicircle)
            draw.ellipse((tab_x, y1, tab_x + tab_height - 1, y2), fill=tab_color)
            # Rectangular body extending to right edge
            draw.rectangle((tab_x + cap_r, y1, self.TAB_WIDTH - 1, y2), fill=tab_color)
            
            # Tab label (right-aligned, black text on colored background)
            label = self.TAB_LABELS[cat]
            bbox = draw.textbbox((0, 0), label, font=self._font_small)
            text_w = bbox[2] - bbox[0]
            text_x = self.TAB_WIDTH - text_w - 6
            text_y = y1 + (tab_height - 12) // 2
            
            # LCARS uses black text on colored elements
            draw.text((text_x, text_y), label, fill=self.LCARS_BLACK,
                      font=self._font_small)
    
    def _draw_lcars_content(self, draw, colors):
        """Draw LCARS-style content area.
        
        LCARS guidelines:
        - 3 font sizes only: Main Title, Sub Header, Normal Data
        - Text colors: normal display + highlight (2 colors max)
        - Consistent spacing grid
        - Rounded caps for buttons/terminators
        """
        x_start = self.TAB_WIDTH + 10
        y_start = 8
        gap = 3  # Standard LCARS gap
        
        # === Main Title (largest font size) ===
        title = self.TAB_NAMES[self._current_tab].upper()
        draw.text((x_start, y_start), title, fill=colors['accent'], font=self._font)
        
        # === Close button (rounded cap = termination point) ===
        close_h = 16
        close_w = 28
        close_x = self.WIDTH - close_w - 4
        close_y = y_start
        cap_r = close_h // 2
        # Left cap
        draw.ellipse((close_x, close_y, close_x + close_h, close_y + close_h - 1), fill=colors['close'])
        # Body
        draw.rectangle((close_x + cap_r, close_y, close_x + close_w - cap_r, close_y + close_h - 1), fill=colors['close'])
        # Right cap
        draw.ellipse((close_x + close_w - close_h, close_y, close_x + close_w, close_y + close_h - 1), fill=colors['close'])
        # X label
        draw.text((close_x + close_w // 2 - 4, close_y + 1), "X", fill=self.LCARS_BLACK, font=self._font_small)
        
        # === Separator bar (thin horizontal) ===
        sep_y = y_start + 20
        draw.rectangle((x_start, sep_y, self.WIDTH - 4, sep_y + 2), fill=colors['accent'])
        
        # === Content items ===
        items = self._items.get(self._current_tab, [])
        item_height = 32
        content_y = sep_y + gap + 3
        max_visible = 4
        
        # === Scroll bar (vertical bar with rounded caps) ===
        if len(items) > max_visible:
            track_w = 24
            track_x = self.WIDTH - track_w - 4
            track_y = content_y
            track_h = max_visible * item_height - gap
            cap_r = track_w // 2
            
            # Track with rounded ends
            draw.ellipse((track_x, track_y, track_x + track_w - 1, track_y + track_w - 1), 
                        fill=colors['scroll_track'])
            draw.rectangle((track_x, track_y + cap_r, track_x + track_w - 1, track_y + track_h - cap_r),
                          fill=colors['scroll_track'])
            draw.ellipse((track_x, track_y + track_h - track_w, track_x + track_w - 1, track_y + track_h - 1),
                        fill=colors['scroll_track'])
            
            # Thumb (indicator)
            total_items = len(items)
            visible_ratio = max_visible / total_items
            thumb_h = max(24, int((track_h - track_w) * visible_ratio))
            scroll_range = total_items - max_visible
            scroll_ratio = self._scroll_offset / scroll_range if scroll_range > 0 else 0
            thumb_y = track_y + cap_r + int((track_h - track_w - thumb_h) * scroll_ratio)
            
            # Thumb with rounded ends
            thumb_cap = 10
            draw.ellipse((track_x + 3, thumb_y, track_x + track_w - 4, thumb_y + thumb_cap * 2 - 1), 
                        fill=colors['scroll_thumb'])
            if thumb_h > thumb_cap * 2:
                draw.rectangle((track_x + 3, thumb_y + thumb_cap, track_x + track_w - 4, thumb_y + thumb_h - thumb_cap),
                              fill=colors['scroll_thumb'])
                draw.ellipse((track_x + 3, thumb_y + thumb_h - thumb_cap * 2, track_x + track_w - 4, thumb_y + thumb_h - 1),
                            fill=colors['scroll_thumb'])
        
        # === Draw visible items (Normal Data font size) ===
        # Get alternate colors for variety
        accent_alt = colors.get('accent_alt', colors['accent'])
        item_alt = colors.get('item_alt', colors['value'])
        
        for i in range(self._scroll_offset, min(self._scroll_offset + max_visible, len(items))):
            item = items[i]
            is_selected = (i == self._selected_item)
            
            item_y = content_y + (i - self._scroll_offset) * item_height
            text_y = item_y + 8
            
            # Selection indicator (LCARS style - accent bar on left)
            # Alternate between accent colors for visual variety
            if is_selected:
                indicator_color = colors['accent'] if i % 2 == 0 else accent_alt
                draw.rectangle((x_start - 4, item_y + 4, x_start - 1, item_y + item_height - 6),
                              fill=indicator_color)
            
            # Item label (normal data size)
            label_color = colors['text'] if is_selected else colors['text_dim']
            draw.text((x_start, text_y), item.label, fill=label_color, font=self._font)
            
            # Value/action area (right side)
            value_x_end = self.WIDTH - 36 if len(items) > max_visible else self.WIDTH - 8
            
            # Alternate value colors for variety
            value_color = colors['value'] if i % 2 == 0 else item_alt
            
            if item.item_type == "info":
                # Right-aligned info value
                value_text = item.get_display_value()
                bbox = draw.textbbox((0, 0), value_text, font=self._font)
                text_w = bbox[2] - bbox[0]
                draw.text((value_x_end - text_w, text_y), value_text,
                         fill=value_color, font=self._font)
            
            elif item.item_type in ("option", "value"):
                # Value with arrow indicators
                value_text = item.get_display_value()
                bbox = draw.textbbox((0, 0), value_text, font=self._font)
                text_w = bbox[2] - bbox[0]
                
                # Arrow positions
                arrow_left_x = self.WIDTH - 125
                arrow_right_x = value_x_end - 8
                value_center = (arrow_left_x + arrow_right_x) // 2
                
                arrow_color = colors['accent'] if is_selected else colors['arrow']
                draw.text((arrow_left_x, text_y), "<", fill=arrow_color, font=self._font)
                draw.text((arrow_right_x, text_y), ">", fill=arrow_color, font=self._font)
                draw.text((value_center - text_w // 2, text_y), value_text,
                         fill=value_color, font=self._font)
            
            elif item.item_type == "action":
                # Action indicator arrow
                draw.text((value_x_end - 8, text_y), ">", 
                         fill=colors['accent'] if is_selected else colors['arrow'], font=self._font)

    def _draw_tabs(self, draw, colors):
        """Draw the tab sidebar (MARS theme)."""
        start, end, visible, max_per, page, page_count = self._get_tab_paging()
        if visible <= 0:
            return

        # If we have multiple pages, reserve a small header area above the
        # tabs for the page indicator so it doesn't overlap the first tab.
        header_h = 0
        if page_count > 1:
            indicator = f"{page + 1}/{page_count}"
            bbox = draw.textbbox((0, 0), indicator, font=self._font_small)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
            header_h = text_h + 4
            text_x = (self.TAB_WIDTH - text_w) // 2
            text_y = 2
            draw.text((text_x, text_y), indicator,
                      fill=colors.get('text_dim', colors['accent']),
                      font=self._font_small)

        available_h = max(1, self.HEIGHT - header_h)
        tab_height = available_h // visible
        
        for visible_idx, pos in enumerate(range(start, end)):
            y1 = header_h + visible_idx * tab_height
            y2 = y1 + tab_height - 1

            cat = self.TAB_ORDER[pos]
            
            # Tab background
            if cat == self._current_tab:
                color = colors['tab_selected']
            else:
                color = colors['tab']
            
            draw.rectangle((0, y1, self.TAB_WIDTH - 1, y2), fill=color)
            
            # Tab label (use text name, emoji may not render well)
            label = self.TAB_NAMES[cat]
            bbox = draw.textbbox((0, 0), label, font=self._font_small)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
            text_x = (self.TAB_WIDTH - text_w) // 2
            text_y = y1 + (tab_height - text_h) // 2
            
            text_color = colors['accent'] if cat == self._current_tab else colors['text_dim']
            draw.text((text_x, text_y), label, fill=text_color, font=self._font_small)
            
            # Selection indicator
            if cat == self._current_tab:
                draw.rectangle((self.TAB_WIDTH - 3, y1 + 2,
                                self.TAB_WIDTH - 1, y2 - 2),
                               fill=colors['accent'])
    
    def _draw_content(self, draw, colors):
        """Draw the content area for current tab (MARS theme)."""
        x_start = self.TAB_WIDTH + 5
        y_start = 5
        
        # Title
        title = self.TAB_NAMES[self._current_tab].upper()
        draw.text((x_start, y_start), title, fill=colors['accent'], font=self._font)
        
        # Close button (X) in top-right corner - 30x24 touch zone
        close_x = self.WIDTH - 32
        close_y = y_start
        draw.rectangle((close_x, close_y, close_x + 28, close_y + 20),
                       fill=colors['tab'])
        draw.text((close_x + 8, close_y + 2), "X", fill=colors['close'], font=self._font)
        
        # Separator line
        draw.line((x_start, y_start + 22, self.WIDTH - 5, y_start + 22),
                 fill=colors['border'], width=1)
        
        # Items
        items = self._items.get(self._current_tab, [])
        item_height = 34  # Larger items for touch
        y = y_start + 24
        max_visible = 4  # Fewer items, larger touch targets
        
        # Draw scroll bar if needed (30px wide for fat fingers, functional)
        if len(items) > max_visible:
            # Scroll bar track (right edge, 30px wide for fat fingers)
            track_x = self.WIDTH - 32
            track_y_start = y
            track_height = max_visible * item_height - 4
            
            # Track background
            draw.rectangle((track_x, track_y_start, track_x + 28, track_y_start + track_height),
                          fill=colors['scroll_track'])
            
            # Calculate thumb size and position
            total_items = len(items)
            visible_ratio = max_visible / total_items
            thumb_height = max(30, int(track_height * visible_ratio))  # Min 30px thumb for touch
            
            scroll_range = total_items - max_visible
            if scroll_range > 0:
                scroll_ratio = self._scroll_offset / scroll_range
            else:
                scroll_ratio = 0
            
            thumb_travel = track_height - thumb_height
            thumb_y = track_y_start + int(thumb_travel * scroll_ratio)
            
            # Thumb (scrollbar handle) - wide for touch
            draw.rectangle((track_x + 3, thumb_y, track_x + 25, thumb_y + thumb_height),
                          fill=colors['scroll_thumb'])
        
        # Draw visible items
        for i in range(self._scroll_offset, min(self._scroll_offset + max_visible, len(items))):
            item = items[i]
            is_selected = (i == self._selected_item)
            
            item_y = y + (i - self._scroll_offset) * item_height
            text_y = item_y + 8  # Center text vertically in 34px item
            
            # Item background
            if is_selected:
                draw.rectangle((x_start - 2, item_y, self.WIDTH - 5, item_y + item_height - 2),
                              fill=colors['item_selected'])
            
            # Item label
            label_color = colors['text'] if is_selected else colors['text_dim']
            draw.text((x_start, text_y), item.label, fill=label_color, font=self._font)
            
            # Value/action area
            if item.item_type == "info":
                # Right-aligned info value
                value_text = item.get_display_value()
                bbox = draw.textbbox((0, 0), value_text, font=self._font)
                text_w = bbox[2] - bbox[0]
                draw.text((self.WIDTH - text_w - 10, text_y), value_text,
                         fill=colors['value'], font=self._font)
            
            elif item.item_type in ("option", "value"):
                # Value with arrows (leave room for 30px scroll bar on right)
                value_text = item.get_display_value()
                bbox = draw.textbbox((0, 0), value_text, font=self._font)
                text_w = bbox[2] - bbox[0]
                
                # Center value in value area (moved left to avoid scroll bar)
                value_area_x = self.WIDTH - 130
                value_center = value_area_x + 40
                
                # Draw arrows (use ASCII < > instead of Unicode arrows for font compatibility)
                arrow_color = colors['accent'] if is_selected else colors['arrow']
                draw.text((value_area_x, text_y), "<", fill=arrow_color, font=self._font)
                draw.text((self.WIDTH - 45, text_y), ">", fill=arrow_color, font=self._font)
                
                # Draw value centered between arrows
                draw.text((value_center - text_w // 2, text_y), value_text,
                         fill=colors['value'], font=self._font)
            
            elif item.item_type == "action":
                # Action button indicator (ASCII for font compatibility, left of scroll bar)
                draw.text((self.WIDTH - 45, text_y), ">", fill=colors['accent'] if is_selected else colors['arrow'], font=self._font)
    
    @property
    def image(self):
        """Return the current menu image (renders if needed)."""
        return self.render()
    
    def invalidate(self):
        """Mark the menu as needing re-render."""
        self._needs_render = True

    def get_all_config(self) -> dict:
        """Export all menu items as a dictionary for web dashboard.
        
        Returns a dict of {category_name: {item_label: item_data, ...}, ...}
        Each item_data is a dict with: value, type, and optional min/max/step/options/unit.
        
        Returns:
            dict: Configuration organized by category with full metadata
        """
        # Category ID to display name mapping
        category_names = {
            MenuCategory.EYES: "Eyes",
            MenuCategory.GAIT: "Gait",
            MenuCategory.POSTURE: "Posture",
            MenuCategory.INFO: "Info",
            MenuCategory.SAFETY: "Safety",
            MenuCategory.SYSTEM: "System",
            MenuCategory.PID: "PID",
            MenuCategory.IMP: "Impedance",
            MenuCategory.EST: "Estimator",
            MenuCategory.IMU: "IMU",
            MenuCategory.TOF: "ToF",
            MenuCategory.AUTO: "Autonomy",
        }
        
        config = {}
        
        for category_id, items in self._items.items():
            category_name = category_names.get(category_id, f"Category_{category_id}")
            category_config = {}
            
            for item in items:
                # Skip action items (buttons) - they have no value to export
                if item.item_type == "action":
                    continue
                
                # Build item data with metadata
                item_data = {
                    "type": item.item_type,
                    "raw_value": item.value,
                }
                
                # Get the display value for dashboard
                if item.item_type == "option":
                    # For options, show the selected option text
                    if item.options and 0 <= item.value < len(item.options):
                        item_data["display"] = item.options[item.value]
                    else:
                        item_data["display"] = str(item.value)
                    item_data["options"] = item.options
                elif item.item_type == "value":
                    # For numeric values, include unit and constraints
                    if item.format_func:
                        item_data["display"] = item.format_func(item.value)
                    else:
                        item_data["display"] = f"{item.value}{item.unit}" if item.unit else str(item.value)
                    if item.min_val is not None:
                        item_data["min"] = item.min_val
                    if item.max_val is not None:
                        item_data["max"] = item.max_val
                    if item.step is not None:
                        item_data["step"] = item.step
                    if item.unit:
                        item_data["unit"] = item.unit
                elif item.item_type == "info":
                    # Info items show their display value
                    item_data["display"] = item.get_display_value()
                else:
                    item_data["display"] = str(item.value) if item.value is not None else "---"
                
                category_config[item.label] = item_data
            
            # Only include category if it has items
            if category_config:
                config[category_name] = category_config
        
        return config


# Test standalone
if __name__ == "__main__":
    menu = MarsMenu()
    menu.show()
    
    # Render and save test image
    img = menu.render()
    img.save("/tmp/mars_menu_test.png")
    print("Menu rendered to /tmp/mars_menu_test.png")
    
    # Test navigation
    menu.nav_down()
    menu.nav_down()
    menu.nav_right()
    img = menu.render()
    img.save("/tmp/mars_menu_test2.png")
    print("After navigation: /tmp/mars_menu_test2.png")

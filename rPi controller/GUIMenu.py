from PIL import Image, ImageDraw, ImageFont
import st7789
import cst816d
from time import sleep

class GUIMenu:
    def __init__(self, touch):
        self._width = 320  # Waveshare 1.9" LCD resolution (landscape)
        self._height = 170
        self._font_size = 10
        self._font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        self._menu = self._build_menu()
        self._state = self._build_state()
        self._selected_path = "File"
        self._visible_items = []  # (path, (x1, y1, x2, y2))
        self._image = self._render_image()
        # Initialize LCD
        self._disp = None
        # self._disp = st7789.ST7789(
        #     port=0,
        #     cs=0,
        #     dc=24,
        #     rst=25,
        #     width=320,
        #     height=170,
        #     rotation=90,  # Landscape
        #     backlight=18
        # )
        # self._disp.set_backlight(True)
        # Initialize touch
        self._touch = touch

    def _build_menu(self):
        """Define the menu structure."""
        return {
            "File": {
                "New": [],
                "Open": [],
                "Save": []
            },
            "Edit": {
                "Cut": [],
                "Copy": [],
                "Format": ["Bold", "Italic", "Underline", "Strikethrough"]
            },
            "View": {
                "Zoom": [],
                "Full Screen": [],
                "Themes": []
            }
        }

    def _build_state(self):
        """Initialize the expanded/collapsed state."""
        state = {}
        for root, submenus in self._menu.items():
            state[root] = True
            for submenu in submenus:
                state[f"{root}.{submenu}"] = True
        return state

    def expand(self):
        """Expand the currently selected menu or sub-menu."""
        if self._selected_path in self._state:
            self._state[self._selected_path] = True
            self._image = self._render_image()
        else:
            raise ValueError(f"Cannot expand: {self._selected_path} is not a menu or sub-menu")

    def contract(self, menu_path):
        """Collapse a specific menu or sub-menu."""
        if menu_path in self._state:
            self._state[menu_path] = False
            self._image = self._render_image()
        else:
            raise ValueError(f"Invalid menu path: {menu_path}")

    def expand_all(self):
        """Expand all menus and sub-menus."""
        for key in self._state:
            self._state[key] = True
        self._image = self._render_image()

    def contract_all(self):
        """Collapse all menus and sub-menus."""
        for key in self._state:
            self._state[key] = False
        self._image = self._render_image()

    def move_up(self):
        """Move selection to the previous visible menu item."""
        if not self._visible_items:
            return
        current_idx = next((i for i, item in enumerate(self._visible_items) if item[0] == self._selected_path), 0)
        new_idx = max(0, current_idx - 1)
        self._selected_path = self._visible_items[new_idx][0]
        self._image = self._render_image()

    def move_down(self):
        """Move selection to the next visible menu item."""
        if not self._visible_items:
            return
        current_idx = next((i for i, item in enumerate(self._visible_items) if item[0] == self._selected_path), 0)
        new_idx = min(len(self._visible_items) - 1, current_idx + 1)
        self._selected_path = self._visible_items[new_idx][0]
        self._image = self._render_image()

    def move_in(self):
        """Move selection to the first child of the current menu item."""
        if self._selected_path in self._menu and self._state[self._selected_path]:
            first_submenu = next(iter(self._menu[self._selected_path]), None)
            if first_submenu:
                self._selected_path = f"{self._selected_path}.{first_submenu}"
                self._image = self._render_image()
                return
        parts = self._selected_path.split(".")
        if len(parts) == 2 and self._state[self._selected_path]:
            root, submenu = parts
            options = self._menu[root][submenu]
            if options:
                self._selected_path = f"{self._selected_path}.{options[0]}"
                self._image = self._render_image()

    def move_out(self):
        """Move selection to the parent menu."""
        parts = self._selected_path.split(".")
        if len(parts) > 1:
            self._selected_path = ".".join(parts[:-1])
            self._image = self._render_image()

    def select(self):
        """Toggle expand/collapse or handle option selection."""
        if self._selected_path in self._state:
            self._state[self._selected_path] = not self._state[self._selected_path]
        else:
            print(f"Selected option: {self._selected_path}")
        self._image = self._render_image()

    def _render_image(self):
        """Generate the PIL Image with buttons."""
        img = Image.new("RGB", (self._width, self._height), "white")
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype(self._font_path, self._font_size)
        except IOError:
            font = ImageFont.load_default()

        # Draw title
        title = "Menu"
        title_bbox = draw.textbbox((0, 0), title, font=font)
        title_width = title_bbox[2] - title_bbox[0]
        draw.text(((self._width - title_width) // 2, 5), title, fill="black", font=font)

        # Reset visible items
        self._visible_items = []
        y = 20
        indent = 10
        button_height = self._font_size + 4  # 2px padding
        button_width = 120  # Fixed width for buttons

        # Render menu hierarchy as buttons
        for root, submenus in self._menu.items():
            indicator = "+" if not self._state[root] else "-"
            text = f"{indicator} {root}"
            x = indent
            if self._selected_path == root:
                draw.rectangle((x, y, x + button_width, y + button_height), fill="lightblue", outline="black")
            else:
                draw.rectangle((x, y, x + button_width, y + button_height), fill="lightgray", outline="black")
            text_bbox = draw.textbbox((0, 0), text, font=font)
            text_width = text_bbox[2] - text_bbox[0]
            draw.text((x + (button_width - text_width) // 2, y + 2), text, fill="black", font=font)
            self._visible_items.append((root, (x, y, x + button_width, y + button_height)))
            y += button_height + 2

            if self._state[root]:
                for submenu, options in submenus.items():
                    indicator = "+" if not self._state[f"{root}.{submenu}"] else "-"
                    text = f"└─ {indicator} {submenu}"
                    x = indent * 2
                    if self._selected_path == f"{root}.{submenu}":
                        draw.rectangle((x, y, x + button_width, y + button_height), fill="lightblue", outline="black")
                    else:
                        draw.rectangle((x, y, x + button_width, y + button_height), fill="lightgray", outline="black")
                    text_bbox = draw.textbbox((0, 0), text, font=font)
                    text_width = text_bbox[2] - text_bbox[0]
                    draw.text((x + (button_width - text_width) // 2, y + 2), text, fill="black", font=font)
                    self._visible_items.append((f"{root}.{submenu}", (x, y, x + button_width, y + button_height)))
                    y += button_height + 2

                    if self._state[f"{root}.{submenu}"]:
                        for option in options:
                            text = f"   └─ {option}"
                            x = indent * 3
                            if self._selected_path == f"{root}.{submenu}.{option}":
                                draw.rectangle((x, y, x + button_width, y + button_height), fill="lightblue", outline="black")
                            else:
                                draw.rectangle((x, y, x + button_width, y + button_height), fill="lightgray", outline="black")
                            text_bbox = draw.textbbox((0, 0), text, font=font)
                            text_width = text_bbox[2] - text_bbox[0]
                            draw.text((x + (button_width - text_width) // 2, y + 2), text, fill="black", font=font)
                            self._visible_items.append((f"{root}.{submenu}.{option}", (x, y, x + button_width, y + button_height)))
                            y += button_height + 2

                y += 2

        return img

    @property
    def image(self):
        """Return the PIL Image object."""
        return self._image

    def touched(self):
        """Check if the touch screen has been touched."""
        self._touch.read_touch_data()
        return self._touch.point_count > 0
    
    def handle_touch(self):
        """Process touch events and update selection."""
        #self._touch.read_touch_data()
        point, coordinates = self._touch.get_touch_xy()
        for path, (x1, y1, x2, y2) in self._visible_items:
            if x1 <= coordinates[0]['x'] <= x2 and y1 <= coordinates[0]['y'] <= y2:
                self._selected_path = path
                self.select()
                return True
        return False

    def display(self):
        """Display the current image on the LCD."""
        self._disp.display(self._image)

if __name__ == "__main__":
    menu = GUIMenu()
    try:
        while True:
            menu.display()
            if menu.handle_touch():
                menu.display()
            sleep(0.033)  # ~30 FPS
    except KeyboardInterrupt:
        menu._disp.set_backlight(False)
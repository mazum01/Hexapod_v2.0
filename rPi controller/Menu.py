from PIL import Image, ImageDraw, ImageFont

class TextMenuImage:
    def __init__(self):
        self._width = 320  # Waveshare 1.9" LCD resolution (landscape)
        self._height = 170
        self._font_size = 20
        self._font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"  # Monospaced font on RPi
        self.background_color = "BLACK"  # Background color for the menu
        self.foreground_color = "WHITE"  # Foreground color for text
        self._menu = self._build_menu()
        self._state = self._build_state()
        self._selected_path = "Eyes"  # Initial selection
        self._visible_items = []  # Tracks visible items and their positions
        self._image = self._render_image()

    def _build_menu(self):
        """Define the menu structure."""
        return {
            "Eyes": {
                "Shape": ["Rectangle", "Rounded Rectangle", "Round", "Dead"],
                "Color": ["White", "Red", "Green", "Blue", "Yellow", "Orange", "Purple", "Gray"],
                "CRT Mode": ["On", "Off"]
            },
            "System": {
                "Menu": [],
                "Font": []
            }
        }

    def _build_state(self):
        """Initialize the expanded/collapsed state for each menu000000000 and sub-menu."""
        state = {}
        for root, submenus in self._menu.items():
            state[root] = True  # Root menus expanded by default
            for submenu in submenus:
                state[f"{root}.{submenu}"] = True  # Sub-menus expanded by default
        return state

    def expand(self):
        """Expand the currently selected menu or sub-menu."""
        if self._selected_path in self._state:
            self._state[self._selected_path] = True
            self._image = self._render_image()
        else:
            raise ValueError(f"Cannot expand: {self._selected_path} is not a menu or sub-menu")

    def contract(self, menu_path):
        """Collapse a specific menu or sub-menu by its path."""
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
        """Move selection to the first child of the current menu item, if expanded and available."""
        # Check if current path is a root menu
        if self._selected_path in self._menu and self._state[self._selected_path]:
            # Move to first sub-menu
            first_submenu = next(iter(self._menu[self._selected_path]), None)
            if first_submenu:
                self._selected_path = f"{self._selected_path}.{first_submenu}"
                self._image = self._render_image()
                return
        # Check if current path is a sub-menu with options
        parts = self._selected_path.split(".")
        if len(parts) == 2 and self._state[self._selected_path]:
            root, submenu = parts
            options = self._menu[root][submenu]
            if options:
                self._selected_path = f"{self._selected_path}.{options[0]}"
                self._image = self._render_image()
                return
        # No children or not expanded, do nothing

    def move_out(self):
        """Move selection to the parent menu of the current item."""
        parts = self._selected_path.split(".")
        if len(parts) > 1:
            # Move to parent (remove last part of path)
            self._selected_path = ".".join(parts[:-1])
            self._image = self._render_image()
            
    def select(self):
        """Toggle expand/collapse for the selected menu or handle option selection."""
        if self._selected_path in self._state:
            # Toggle expand/collapse for root or sub-menu
            self._state[self._selected_path] = not self._state[self._selected_path]
        else:
            # Handle option selection (placeholder action)
            print(f"Selected option: {self._selected_path}")
        self._image = self._render_image()

    def _render_image(self):
        """Generate the PIL Image of the menu based on current state and selection."""
        img = Image.new("RGB", (self._width, self._height), self.background_color)
        draw = ImageDraw.Draw(img)

        try:
            font = ImageFont.truetype(self._font_path, self._font_size)
        except IOError:
            font = ImageFont.load_default()

        # Draw border
        draw.rectangle((2, 2, self._width - 3, self._height - 3), outline=self.foreground_color)

        # Draw title
        title = "Menu"
        title_bbox = draw.textbbox((0, 0), title, font=font)
        title_width = title_bbox[2] - title_bbox[0]
        title_x = (self._width - title_width) // 2
        draw.text((title_x, 5), title, fill=self.foreground_color, font=font)

        # Reset visible items
        self._visible_items = []
        y = 20
        indent = 10

        # Render menu hierarchy
        for root, submenus in self._menu.items():
            # Draw root menu
            indicator = "+" if not self._state[root] else "-"
            text = f"{indicator} {root}"
            text_bbox = draw.textbbox((0, 0), text, font=font)
            text_width = text_bbox[2] - text_bbox[0]
            if self._selected_path == root:
                draw.rectangle((indent, y, indent + text_width, y + self._font_size), fill="lightgray")
            draw.text((indent, y), text, fill=self.foreground_color, font=font)
            self._visible_items.append((root, (indent, y, indent + text_width, y + self._font_size)))
            y += self._font_size + 2

            if self._state[root]:
                for submenu, options in submenus.items():
                    # Draw sub-menu
                    indicator = "+" if not self._state[f"{root}.{submenu}"] else "-"
                    text = f"└─ {indicator} {submenu}"
                    text_bbox = draw.textbbox((0, 0), text, font=font)
                    text_width = text_bbox[2] - text_bbox[0]
                    if self._selected_path == f"{root}.{submenu}":
                        draw.rectangle((indent * 2, y, indent * 2 + text_width, y + self._font_size), fill="lightgray")
                    draw.text((indent * 2, y), text, fill=self.foreground_color, font=font)
                    self._visible_items.append((f"{root}.{submenu}", (indent * 2, y, indent * 2 + text_width, y + self._font_size)))
                    y += self._font_size + 2

                    if self._state[f"{root}.{submenu}"]:
                        for option in options:
                            text = f"   └─ {option}"
                            text_bbox = draw.textbbox((0, 0), text, font=font)
                            text_width = text_bbox[2] - text_bbox[0]
                            if self._selected_path == f"{root}.{submenu}.{option}":
                                draw.rectangle((indent * 3, y, indent * 3 + text_width, y + self._font_size), fill="lightgray")
                            draw.text((indent * 3, y), text, fill=self.foreground_color, font=font)
                            self._visible_items.append((f"{root}.{submenu}.{option}", (indent * 3, y, indent * 3 + text_width, y + self._font_size)))
                            y += self._font_size + 2

                y += 2  # Extra spacing between root menus

        return img

    @property
    def image(self):
        """Return the PIL Image object."""
        return self._image

if __name__ == "__main__":
    # Example usage
    menu = TextMenuImage()
    menu.image.save("menu_initial.png")  # Initial state (File selected)
    menu.move_down()
    menu.image.save("menu_moved_down.png")  # Next item selected
    menu.select()
    menu.image.save("menu_selected.png")  # After toggling selection
    menu.contract_all()
    menu.image.save("menu_contracted.png")  # All collapsed
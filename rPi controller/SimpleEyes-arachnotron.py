import math
from PIL import Image, ImageDraw, ImageFont
import random
import time


class BLINKSTATE:
    OPEN = 0
    CLOSED = 1
    BLINKING = 2
    WAITING = 3
    def __init__(self):
        pass

class EYE_SHAPE:
    ELLIPSE = 0
    RECTANGLE = 1
    ROUNDRECTANGLE = 2
    X = 3
    SPIDER = 4  # 8-eye spider pattern
    HUMAN = 5   # Realistic human eyes with iris, pupil, sclera
    CAT = 6     # Cat eyes with vertical slit pupils
    HYPNO = 7   # Hypnotic spiral eyes
    ANIME = 8   # Large kawaii anime-style eyes

    def __init__(self):
        pass

class SimpleEyes:
    def __init__(self, display_size, eye_color="BLUE", eye_size=(50,37) ):

        # define the eye attributes
        self.eye_color = eye_color
        self.eye_size = eye_size
        self.left_center = (int(display_size[0] * .25), int(display_size[1] * .45))
        self.right_center = (int(display_size[0] * .75), int(display_size[1] * .45))
        self.left_shape = EYE_SHAPE.ROUNDRECTANGLE
        self.right_shape = EYE_SHAPE.ROUNDRECTANGLE
        self.rotation = 0
        self.eye_center_offset = 15;  # offset from the screen center to the middle of the eyes
        self.eye_vertical_offset = 0  # vertical offset in pixels (+ for down, - for up)
        self.eye_spacing_offset = 0 # number of pixels between the eyes to move them closer or farther apart (+ for closer, - for farther)
        self.human_eye_spacing_pct = 0.25  # Human eye center X as fraction of screen width from each edge
        self.human_eye_size = 33  # Human eye iris radius in pixels
        self.human_eye_color_idx = 0  # Human eye color index (0=blue, 1=green, 2=hazel, 3=brown, 4=dark brown)
        # Human eye color palette: (iris base color)
        self.human_eye_colors = [
            (70, 130, 180),   # 0: Blue (steel blue)
            (60, 140, 80),    # 1: Green
            (140, 110, 60),   # 2: Hazel (golden brown-green)
            (100, 60, 30),    # 3: Brown
            (50, 30, 20),     # 4: Dark brown
        ]
        # Cat eye color (amber/gold by default)
        self.cat_eye_color = (255, 180, 0)  # Amber
        # Hypno spiral color
        self.hypno_color = (180, 0, 255)    # Purple
        self.hypno_phase = 0.0              # Animation phase for spiral rotation
        # Anime eye color
        self.anime_eye_color = (100, 180, 255)  # Light blue
        self.eyelid_angle = 0 # angle of the eyelid vs the vertical axis of the unrotaated eye, in degrees
        self.eyelid_percent = 0.0 # percentage of the eye that is closed, from 0.0 to 1.0

        # Blink attributes
        self.blink_min_interval = 3.0 # minimum number for blink interval
        self.blink_rnd_interval = 4.0 # random number for blink interval added to the minimum
        self.blinkState = BLINKSTATE.WAITING # current state of the blink
        self.blink_timer = time.time() # start time for the delay between blinks
        self.blink_duration = 0.1 # duration of the blink in seconds
        self.blink_interval = self.blink_min_interval + self.blink_rnd_interval * random.random() # initial blink interval
        self.blink_percent = 0.0 # current percentage of the eye that is closed during the blink
        self.blink_percent_step = 0.035
        self.blink_max_percent = 1.75 # maximum percentage of the eye that is closed during the blink
        self.last_blink_state = BLINKSTATE.WAITING # last state of the blink

        # Display attributes
        self.display_size = display_size
        self.eye_image_left = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
        self.eye_image_right = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
        self.eye_draw_left = ImageDraw.Draw(self.eye_image_left)
        self.eye_draw_right = ImageDraw.Draw(self.eye_image_right)
        self.render_eye()
        self.display_image = Image.new("RGB", display_size, "BLACK")
        self.render_flag = True
        self.crt_mode = True # whether to simulate CRT mode by blanking out some of the lines

    def blink(self):

        self.last_blink_state = self.blinkState
        if self.blinkState == BLINKSTATE.WAITING:
            # check the blink timer to start a new blink
            if time.time() - self.blink_timer > self.blink_interval:
                # set the blink state to blinking
                self.blinkState = BLINKSTATE.BLINKING
                # set the blink timer to the current time
                self.blink_timer = time.time()
                # set the blink interval to a random value
                self.blink_interval = self.blink_min_interval + self.blink_rnd_interval * random.random()
                self.blink_percent = 0.0

        elif self.blinkState == BLINKSTATE.BLINKING:    
            self.blink_percent += self.blink_percent_step
            if self.blink_percent >= self.blink_max_percent:
                # set the blink state to closed
                self.blinkState = BLINKSTATE.CLOSED
                # set the blink timer to the current time
                self.blink_timer = time.time()
                # set the blink interval to a random value
                self.blink_interval = self.blink_min_interval + self.blink_rnd_interval * random.random()
        elif self.blinkState == BLINKSTATE.CLOSED:
            self.blink_percent -= self.blink_percent_step
            if self.blink_percent <= 0.0:
                # set the blink state to waiting
                self.blinkState = BLINKSTATE.WAITING
                # set the blink timer to the current time
                self.blink_timer = time.time()
                # set the blink interval to a random value
                self.blink_interval = self.blink_min_interval + self.blink_rnd_interval * random.random()
                self.blink_percent = 0.0
    def look(self, direction):
        print(f"The {self.eye_color} eyes look {direction}.")

    def set_eye_size(self, size):
        self.eye_size = size
        self.eye_image_left = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
        self.eye_draw_left = ImageDraw.Draw(self.eye_image_left)
        self.eye_image_right = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
        self.eye_draw_right = ImageDraw.Draw(self.eye_image_right)
        self.render_eye()
    
    def update(self, force_update=False):
        # Update the blink state
        self.blink()
        
        # Hypno eyes need continuous updates for animation
        needs_update = (self.blinkState != self.last_blink_state or 
                       self.blinkState == BLINKSTATE.BLINKING or 
                       self.blinkState == BLINKSTATE.CLOSED or
                       self.left_shape == EYE_SHAPE.HYPNO or  # Hypno always animates
                       force_update)
        
        if needs_update:
            self.display_image = Image.new("RGB", self.display_size, "BLACK")

            # Check for special eye modes that render directly to display
            if self.left_shape == EYE_SHAPE.SPIDER and self.right_shape == EYE_SHAPE.SPIDER:
                # Spider mode: draw 8 eyes directly to display
                self.render_spider_eyes()
            elif self.left_shape == EYE_SHAPE.HUMAN and self.right_shape == EYE_SHAPE.HUMAN:
                # Human mode: draw realistic human eyes directly to display
                self.render_human_eyes()
            elif self.left_shape == EYE_SHAPE.CAT and self.right_shape == EYE_SHAPE.CAT:
                # Cat mode: draw cat eyes with vertical slit pupils
                self.render_cat_eyes()
            elif self.left_shape == EYE_SHAPE.HYPNO and self.right_shape == EYE_SHAPE.HYPNO:
                # Hypno mode: draw hypnotic spiral eyes
                self.render_hypno_eyes()
            elif self.left_shape == EYE_SHAPE.ANIME and self.right_shape == EYE_SHAPE.ANIME:
                # Anime mode: draw large kawaii-style eyes
                self.render_anime_eyes()
            else:
                # Normal mode: draw left and right eyes separately
                self.eye_image_left = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
                self.eye_draw_left = ImageDraw.Draw(self.eye_image_left)
                self.eye_image_right = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
                self.eye_draw_right = ImageDraw.Draw(self.eye_image_right)
                self.render_eye()
                
                # draw the right eye
                middle = (self.eye_center_offset + self.eye_spacing_offset + self.display_size[0] * .25, 
                          self.eye_vertical_offset + (self.display_size[1] - 2) * .45)
                eye_rot = self.eye_image_right.rotate(-self.rotation, resample = 3, expand=True)
                self.display_image.paste(eye_rot, (int(middle[0] - eye_rot.width/2), int(middle[1] - eye_rot.height/2)))

                # draw the left eye
                middle = (self.eye_center_offset - self.eye_spacing_offset + self.display_size[0] * .75, 
                          self.eye_vertical_offset + (self.display_size[1] - 2) * .45)
                eye_rot = self.eye_image_left.rotate(self.rotation, resample = 3, expand=True)
                self.display_image.paste(eye_rot, (int(middle[0] - eye_rot.width/2), int(middle[1] - eye_rot.height/2)))

                # Simulate CRT by blanking out some of the lines
                if self.crt_mode:
                    # draw horizontal lines to simulate CRT
                    display_image_draw = ImageDraw.Draw(self.display_image)
                    for i in range(int(middle[1] - int(eye_rot.height/2)), int(middle[1] + eye_rot.height/2),3):
                        display_image_draw.line((0, i, self.display_size[0], i), fill="BLACK", width=2)
            
            # flag to update the display
            self.render_flag = True
        else:
            self.render_flag = False

        # return the render flag to indicate if the image was updated    
        return self.render_flag
    
    def render_eye(self):

        # how much is the sys closed
        height_offset = self.eye_size[1] * self.blink_percent

        # draw the left eye
        if self.left_shape == EYE_SHAPE.ELLIPSE:
            self.eye_draw_left.ellipse((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, outline=self.eye_color)
        elif self.left_shape == EYE_SHAPE.RECTANGLE:
            self.eye_draw_left.rectangle((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, outline=self.eye_color)
        elif self.left_shape == EYE_SHAPE.ROUNDRECTANGLE:
            self.eye_draw_left.rounded_rectangle((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), radius=10, fill=self.eye_color, outline=self.eye_color)
        elif self.left_shape == EYE_SHAPE.X:
            self.eye_draw_left.line((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, width=10)
            self.eye_draw_left.line((0, 2 * self.eye_size[1] - height_offset, self.eye_size[0]* 2, height_offset), fill=self.eye_color, width=10)

        #draw the eyelid (at the full eye height  
        dy = int(self.eye_size[0] * math.atan(self.eyelid_angle * math.pi / 180))
        # if (height_offset - dy) < 0:
        #     poly = [(0, 0), (0, height_offset + dy), (self.eye_size[0]*2, height_offset- dy)]
        # else:
        if ( self.left_shape != EYE_SHAPE.X):
            if ( self.eyelid_angle > 0):
                poly = [(0, 0), (0, height_offset + 2 * dy), (self.eye_size[0]*2, height_offset), (self.eye_size[0]*2, 0)]
            else:
                poly = [(0, 0), (0, height_offset), (self.eye_size[0]*2, height_offset - 2 * dy), (self.eye_size[0]*2, 0)]
            self.eye_draw_left.polygon(poly, fill="BLACK", outline="BLACK")

        # draw the right eye
        if self.right_shape == EYE_SHAPE.ELLIPSE:
            self.eye_draw_right.ellipse((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, outline=self.eye_color)
        elif self.right_shape == EYE_SHAPE.RECTANGLE:
            self.eye_draw_right.rectangle((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, outline=self.eye_color)
        elif self.right_shape == EYE_SHAPE.ROUNDRECTANGLE:
            self.eye_draw_right.rounded_rectangle((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), radius=10, fill=self.eye_color, outline=self.eye_color)
        elif self.right_shape == EYE_SHAPE.X:
            self.eye_draw_right.line((0, height_offset, self.eye_size[0]* 2, 2 * self.eye_size[1] - height_offset), fill=self.eye_color, width=10)
            self.eye_draw_right.line((0, 2 * self.eye_size[1] - height_offset, self.eye_size[0]* 2, height_offset), fill=self.eye_color, width=10)

        # draw the eyelid (at the full eye height
        if (self.right_shape != EYE_SHAPE.X):
            if (self.eyelid_angle > 0):
                poly = [(0, 0), (0, height_offset), (self.eye_size[0]*2, height_offset + 2 * dy), (self.eye_size[0]*2, 0)]
            else:
                poly = [(0, 0), (0, height_offset - 2 * dy), (self.eye_size[0]*2, height_offset), (self.eye_size[0]*2, 0)]  
            self.eye_draw_right.polygon(poly, fill="BLACK", outline="BLACK")

    def render_spider_eyes(self):
        """Render 8 spider eyes directly to display_image.
        
        Spider eye layout (typical jumping spider pattern):
        - 2 large anterior median eyes (AME) - the big front eyes
        - 2 smaller anterior lateral eyes (ALE) - flanking the big ones
        - 4 smaller posterior eyes in a row behind
        
        Layout on display:
              [ALE]  [AME] [AME]  [ALE]
           [PLE] [PME]       [PME] [PLE]
        """
        draw = ImageDraw.Draw(self.display_image)
        
        # Display center
        cx = self.display_size[0] // 2 + self.eye_center_offset
        cy = self.display_size[1] // 2 + self.eye_vertical_offset - 10
        
        # Blink factor (shrink height during blink)
        blink_scale = max(0.1, 1.0 - self.blink_percent * 0.9)
        
        # Eye sizes (relative to display)
        ame_r = int(21 * blink_scale)  # Anterior Median Eyes (big front eyes)
        ale_r = int(12 * blink_scale)  # Anterior Lateral Eyes
        pme_r = int(8 * blink_scale)   # Posterior Median Eyes
        ple_r = int(6 * blink_scale)   # Posterior Lateral Eyes
        
        # Spacing
        ame_spacing = 85   # Distance between the two big eyes
        ale_offset_x = ame_spacing + 5  # How far out the ALE are
        ale_offset_y = -5  # Slightly higher than AME
        
        posterior_y = 35   # How far down the back row is
        pme_x = 35         # PME distance from center
        ple_x = 55         # PLE distance from center
        
        # Highlight color (brighter center for that spider eye shine)
        highlight = tuple(min(255, c + 80) for c in self.eye_color) if isinstance(self.eye_color, tuple) else "WHITE"
        
        # Draw each eye as filled ellipse with highlight
        def draw_spider_eye(x, y, r, is_ame=False):
            """Draw a single spider eye with optional highlight."""
            # Main eye
            draw.ellipse((x - r, y - r, x + r, y + r), fill=self.eye_color, outline=self.eye_color)
            # Highlight (smaller, offset up-left)
            if r > 5:
                hr = max(2, r // 3)
                hx, hy = x - r // 3, y - r // 3
                draw.ellipse((hx - hr, hy - hr, hx + hr, hy + hr), fill=highlight)
        
        # Anterior Median Eyes (AME) - the big characteristic spider eyes
        draw_spider_eye(cx - ame_spacing // 2, cy, ame_r, is_ame=True)
        draw_spider_eye(cx + ame_spacing // 2, cy, ame_r, is_ame=True)
        
        # Anterior Lateral Eyes (ALE) - flanking the big ones
        draw_spider_eye(cx - ale_offset_x, cy + ale_offset_y, ale_r)
        draw_spider_eye(cx + ale_offset_x, cy + ale_offset_y, ale_r)
        
        # Posterior Median Eyes (PME) - back row inner
        draw_spider_eye(cx - pme_x, cy + posterior_y, pme_r)
        draw_spider_eye(cx + pme_x, cy + posterior_y, pme_r)
        
        # Posterior Lateral Eyes (PLE) - back row outer
        draw_spider_eye(cx - ple_x, cy + posterior_y, ple_r)
        draw_spider_eye(cx + ple_x, cy + posterior_y, ple_r)
        
        # Apply CRT effect if enabled
        if self.crt_mode:
            for i in range(0, self.display_size[1], 3):
                draw.line((0, i, self.display_size[0], i), fill="BLACK", width=1)

    def render_human_eyes(self):
        """Render stylized human eyes (iris + pupil only) directly to display_image.
        
        Features:
        - Iris with radial fiber pattern and limbal ring
        - Pupil (black center, fades to red at high intensity)
        - Catchlight highlights (window reflections)
        - Eyelid blink with angle (like other eye types)
        - Configurable eye size and color
        """
        draw = ImageDraw.Draw(self.display_image)
        
        # Eye positioning using percentage of screen width
        spacing_pct = max(0.1, min(0.45, self.human_eye_spacing_pct))  # Clamp to valid range
        left_cx = int(self.display_size[0] * spacing_pct) + self.eye_center_offset - self.eye_spacing_offset
        right_cx = int(self.display_size[0] * (1.0 - spacing_pct)) + self.eye_center_offset + self.eye_spacing_offset
        cy = self.display_size[1] // 2 + self.eye_vertical_offset
        
        # Eye dimensions from config
        iris_r = max(15, min(60, self.human_eye_size))  # Clamp to reasonable range
        pupil_r = int(iris_r * 0.36)  # Pupil is ~36% of iris size
        limbal_rx = iris_r + 3
        limbal_ry = iris_r + 3
        
        # Total eye height (full diameter of limbal ring)
        eye_height = limbal_ry * 2
        
        # Eyelid offset combines:
        # 1. blink_percent - automatic blink animation (0 to ~1.75 during blink)
        # 2. eyelid_percent - joystick-controlled intensity (0 to ~75)
        # Scale blink_percent to cover full eye height during blink
        blink_offset = eye_height * self.blink_percent * 0.6  # 0.6 to reach full close at blink_max ~1.75
        # Scale eyelid_percent (0-75 from joystick) to ~45% max lid coverage
        # 45% of eye_height (~72) = ~32 pixels. With eyelid_percent max 75: 75 * 0.25 ≈ 19 pixels
        intensity_offset = self.eyelid_percent * 0.25    # Joystick intensity (max ~45% eye height)
        height_offset = int(blink_offset + intensity_offset)
        
        # Eyelid angle offset (matches render_eye behavior)
        dy = int(iris_r * math.atan(self.eyelid_angle * math.pi / 180))
        
        # Get iris color from palette based on color index
        color_idx = max(0, min(len(self.human_eye_colors) - 1, self.human_eye_color_idx))
        iris_color = self.human_eye_colors[color_idx]
        iris_dark = tuple(max(0, c - 50) for c in iris_color)  # Darker iris edge
        iris_mid = tuple(max(0, c - 20) for c in iris_color)   # Mid iris
        iris_light = tuple(min(255, c + 40) for c in iris_color)  # Lighter iris center
        
        # Limbal ring: for dark colors, use a lighter outline so eye is visible against black background
        avg_brightness = sum(iris_color) / 3
        if avg_brightness < 60:  # Dark eye colors (brown, dark brown)
            # Use a subtle lighter outline instead of darker
            limbal_ring_color = tuple(min(255, c + 10) for c in iris_color)
        else:
            limbal_ring_color = tuple(max(0, c - 90) for c in iris_color)  # Dark ring around iris
        
        # Pupil color: fade from black to red based on intensity (50% to 100%)
        # eyelid_percent ranges from 0 to ~75 from joystick
        intensity_pct = min(1.0, self.eyelid_percent / 75.0)  # 0.0 to 1.0
        if intensity_pct > 0.5:
            # Fade from black to red over the 50% to 100% range
            red_factor = (intensity_pct - 0.5) * 2.0  # 0.0 to 1.0 over 50%-100%
            pupil_color = (int(10 + 180 * red_factor), int(5 * (1 - red_factor)), int(5 * (1 - red_factor)))
        else:
            pupil_color = (10, 5, 5)  # Near black
        highlight_color = (255, 255, 255)  # Pure white catchlight
        
        def draw_human_eye(cx, is_left_eye):
            """Draw a single stylized human eye (iris + pupil) with eyelid overlay."""
            
            # 1. Draw limbal ring (dark ring around iris)
            draw.ellipse((cx - limbal_rx, cy - limbal_ry, cx + limbal_rx, cy + limbal_ry), 
                        fill=limbal_ring_color, outline=limbal_ring_color)
            
            # 2. Draw iris layers (outer dark to inner light gradient)
            # Outer iris
            draw.ellipse((cx - iris_r, cy - iris_r, cx + iris_r, cy + iris_r), 
                        fill=iris_dark, outline=iris_dark)
            
            # Mid-outer iris
            mid_outer_r = int(iris_r * 0.85)
            draw.ellipse((cx - mid_outer_r, cy - mid_outer_r, cx + mid_outer_r, cy + mid_outer_r), 
                        fill=iris_mid, outline=iris_mid)
            
            # Middle iris  
            mid_r = int(iris_r * 0.70)
            draw.ellipse((cx - mid_r, cy - mid_r, cx + mid_r, cy + mid_r), 
                        fill=iris_color, outline=iris_color)
            
            # Inner iris (lighter)
            inner_r = int(iris_r * 0.55)
            draw.ellipse((cx - inner_r, cy - inner_r, cx + inner_r, cy + inner_r), 
                        fill=iris_light, outline=iris_light)
            
            # 3. Draw radial iris fibers (subtle texture with randomization)
            # Use deterministic seed per eye so fibers don't flicker
            fiber_rng = random.Random(int(cx * 1000))
            num_fibers = fiber_rng.randint(18, 28)  # Random number of fibers
            for i in range(num_fibers):
                # Random angle with slight jitter
                base_angle = (360.0 / num_fibers) * i
                angle = base_angle + fiber_rng.uniform(-8, 8)  # Jitter ±8 degrees
                rad = math.radians(angle)
                # Random fiber start/end positions for organic look
                inner_offset = fiber_rng.uniform(2, 5)
                outer_offset = fiber_rng.uniform(1, 4)
                x1 = cx + (pupil_r + inner_offset) * math.cos(rad)
                y1 = cy + (pupil_r + inner_offset * 0.7) * math.sin(rad)
                x2 = cx + (iris_r - outer_offset) * math.cos(rad)
                y2 = cy + (iris_r - outer_offset * 0.5) * math.sin(rad)
                # Randomly vary fiber color for natural look
                color_choice = fiber_rng.random()
                if color_choice < 0.3:
                    fiber_color = iris_light
                elif color_choice < 0.6:
                    fiber_color = iris_dark
                else:
                    fiber_color = iris_mid
                draw.line((x1, y1, x2, y2), fill=fiber_color, width=1)
            
            # 4. Draw pupil
            draw.ellipse((cx - pupil_r, cy - pupil_r, cx + pupil_r, cy + pupil_r), 
                        fill=pupil_color, outline=pupil_color)
            
            # 5. Draw catchlight highlights (window reflections)
            # Main highlight - upper left
            hl_x = cx - iris_r // 3
            hl_y = cy - int(iris_r * 0.4)
            hl_rx = 5
            hl_ry = 5
            draw.ellipse((hl_x - hl_rx, hl_y - hl_ry, hl_x + hl_rx, hl_y + hl_ry), 
                        fill=highlight_color)
            
            # Secondary smaller highlight - lower right
            hl2_x = cx + iris_r // 4
            hl2_y = cy + int(iris_r * 0.3)
            hl2_rx = 3
            hl2_ry = 3
            draw.ellipse((hl2_x - hl2_rx, hl2_y - hl2_ry, hl2_x + hl2_rx, hl2_y + hl2_ry), 
                        fill=highlight_color)
            
            # 6. Draw eyelid overlay (black polygon from top, with angle)
            # This matches the eyelid behavior from render_eye()
            # Always draw eyelid if there's any height_offset (from blink OR joystick intensity)
            if height_offset > 0 or dy != 0:
                # Define the eye bounding box for eyelid
                eye_left = cx - limbal_rx - 5
                eye_right = cx + limbal_rx + 5
                eye_top = cy - limbal_ry - 5
                
                # Calculate eyelid bottom edge with angle
                # Note: is_left_eye means left side of DISPLAY (robot's right eye)
                # Original render_eye uses swapped naming (eye_image_right is at left of display)
                # When angle > 0: inner edges (toward center of face) should be lower
                # - Left side of display (robot's right eye): right edge is inner, so right edge lower
                # - Right side of display (robot's left eye): left edge is inner, so left edge lower
                if is_left_eye:
                    # Left side of display = robot's right eye
                    # Inner edge is on the right, so right edge goes lower when angle > 0
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset + 2 * dy
                    else:
                        lid_left_y = eye_top + height_offset - 2 * dy
                        lid_right_y = eye_top + height_offset
                else:
                    # Right side of display = robot's left eye
                    # Inner edge is on the left, so left edge goes lower when angle > 0
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset + 2 * dy
                        lid_right_y = eye_top + height_offset
                    else:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset - 2 * dy
                
                # Draw eyelid polygon (from above screen to current lid position)
                poly = [
                    (eye_left, eye_top - 10),      # Top left (above screen)
                    (eye_left, lid_left_y),         # Bottom left of lid
                    (eye_right, lid_right_y),       # Bottom right of lid  
                    (eye_right, eye_top - 10)       # Top right (above screen)
                ]
                draw.polygon(poly, fill="BLACK", outline="BLACK")
        
        # Draw both eyes
        draw_human_eye(left_cx, is_left_eye=True)
        draw_human_eye(right_cx, is_left_eye=False)
        
        # Apply CRT effect if enabled
        if self.crt_mode:
            for i in range(0, self.display_size[1], 3):
                draw.line((0, i, self.display_size[0], i), fill="BLACK", width=1)

    def render_cat_eyes(self):
        """Render cat eyes with vertical slit pupils directly to display_image.
        
        Features:
        - Vertical slit pupil that dilates with intensity
        - Amber/green iris with radial texture
        - Reflective highlight for that feline gleam
        - Eyelid blink with angle
        """
        draw = ImageDraw.Draw(self.display_image)
        
        # Eye positioning (similar to human eyes)
        spacing_pct = max(0.1, min(0.45, self.human_eye_spacing_pct))
        left_cx = int(self.display_size[0] * spacing_pct) + self.eye_center_offset - self.eye_spacing_offset
        right_cx = int(self.display_size[0] * (1.0 - spacing_pct)) + self.eye_center_offset + self.eye_spacing_offset
        cy = self.display_size[1] // 2 + self.eye_vertical_offset
        
        # Eye dimensions
        iris_rx = 38  # Slightly wider than tall (cat eyes are more almond-shaped)
        iris_ry = 32
        
        # Pupil slit dimensions - dilates with intensity
        # At rest: narrow vertical slit. At max intensity: full round pupil using major axis
        intensity_pct = min(1.0, self.eyelid_percent / 75.0)
        pupil_height = int(iris_ry * 1.4)  # Major axis (tall slit height)
        # Width goes from 8px (narrow slit) to pupil_height (full round using major axis)
        pupil_min_width = 8
        pupil_max_width = pupil_height  # Full round = width equals height (major axis)
        pupil_width = int(pupil_min_width + (pupil_max_width - pupil_min_width) * intensity_pct)
        
        # Total eye height for eyelid calculation
        eye_height = iris_ry * 2 + 6
        
        # Eyelid offset (same formula as human eyes)
        blink_offset = eye_height * self.blink_percent * 0.6
        intensity_offset = self.eyelid_percent * 0.25
        height_offset = int(blink_offset + intensity_offset)
        dy = int(iris_rx * math.atan(self.eyelid_angle * math.pi / 180))
        
        # Colors
        iris_color = self.cat_eye_color
        iris_dark = tuple(max(0, c - 60) for c in iris_color)
        iris_light = tuple(min(255, c + 30) for c in iris_color)
        limbal_color = tuple(max(0, c - 100) for c in iris_color)
        
        # Pupil color: dark with red tint at high intensity
        if intensity_pct > 0.5:
            red_factor = (intensity_pct - 0.5) * 2.0
            pupil_color = (int(30 + 80 * red_factor), int(15 * (1 - red_factor)), int(15 * (1 - red_factor)))
        else:
            pupil_color = (15, 5, 5)
        
        def draw_cat_eye(cx, is_left_eye):
            """Draw a single cat eye."""
            # 1. Limbal ring (dark outline)
            draw.ellipse((cx - iris_rx - 3, cy - iris_ry - 3, cx + iris_rx + 3, cy + iris_ry + 3),
                        fill=limbal_color, outline=limbal_color)
            
            # 2. Outer iris
            draw.ellipse((cx - iris_rx, cy - iris_ry, cx + iris_rx, cy + iris_ry),
                        fill=iris_dark, outline=iris_dark)
            
            # 3. Inner iris (gradient effect)
            inner_rx = int(iris_rx * 0.75)
            inner_ry = int(iris_ry * 0.75)
            draw.ellipse((cx - inner_rx, cy - inner_ry, cx + inner_rx, cy + inner_ry),
                        fill=iris_color, outline=iris_color)
            
            # 4. Radial iris texture (use stable seed so fibers don't move)
            rng = random.Random(42 if is_left_eye else 43)  # Fixed seed per eye
            num_fibers = 24
            fiber_data = []  # Pre-generate all fiber data with stable seed
            for i in range(num_fibers):
                angle_jitter = rng.uniform(-5, 5)
                color_choice = rng.random()
                fiber_data.append((angle_jitter, color_choice))
            
            for i, (angle_jitter, color_choice) in enumerate(fiber_data):
                angle = (360.0 / num_fibers) * i + angle_jitter
                rad = math.radians(angle)
                # Use fixed pupil size for fiber start to avoid movement
                x1 = cx + 12 * math.cos(rad)
                y1 = cy + 18 * math.sin(rad)
                x2 = cx + iris_rx * 0.95 * math.cos(rad)
                y2 = cy + iris_ry * 0.95 * math.sin(rad)
                fiber_color = iris_light if color_choice > 0.5 else iris_dark
                draw.line((x1, y1, x2, y2), fill=fiber_color, width=1)
            
            # 5. Vertical slit pupil
            draw.ellipse((cx - pupil_width // 2, cy - pupil_height // 2,
                         cx + pupil_width // 2, cy + pupil_height // 2),
                        fill=pupil_color, outline=pupil_color)
            
            # 6. Catchlight (distinctive cat eye gleam - positioned upper)
            hl_x = cx - iris_rx // 4
            hl_y = cy - iris_ry // 2
            draw.ellipse((hl_x - 4, hl_y - 4, hl_x + 4, hl_y + 4), fill=(255, 255, 255))
            # Secondary smaller highlight
            hl2_x = cx + iris_rx // 5
            hl2_y = cy + iris_ry // 3
            draw.ellipse((hl2_x - 2, hl2_y - 2, hl2_x + 2, hl2_y + 2), fill=(255, 255, 255))
            
            # 7. Eyelid overlay
            if height_offset > 0 or dy != 0:
                eye_left = cx - iris_rx - 8
                eye_right = cx + iris_rx + 8
                eye_top = cy - iris_ry - 8
                
                if is_left_eye:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset + 2 * dy
                    else:
                        lid_left_y = eye_top + height_offset - 2 * dy
                        lid_right_y = eye_top + height_offset
                else:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset + 2 * dy
                        lid_right_y = eye_top + height_offset
                    else:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset - 2 * dy
                
                poly = [(eye_left, eye_top - 10), (eye_left, lid_left_y),
                        (eye_right, lid_right_y), (eye_right, eye_top - 10)]
                draw.polygon(poly, fill="BLACK", outline="BLACK")
        
        draw_cat_eye(left_cx, is_left_eye=True)
        draw_cat_eye(right_cx, is_left_eye=False)
        
        if self.crt_mode:
            for i in range(0, self.display_size[1], 3):
                draw.line((0, i, self.display_size[0], i), fill="BLACK", width=1)

    def render_hypno_eyes(self):
        """Render hypnotic spiral eyes directly to display_image.
        
        Features:
        - Rotating spiral pattern (mesmerizing!)
        - Speed increases with intensity
        - Blink closes with eyelid overlay
        - Color shifts slightly with rotation
        """
        draw = ImageDraw.Draw(self.display_image)
        
        # Advance the animation phase (speed increases with intensity)
        intensity_pct = min(1.0, self.eyelid_percent / 75.0)
        base_speed = 0.08
        speed_boost = 0.15 * intensity_pct
        self.hypno_phase += base_speed + speed_boost
        if self.hypno_phase > 2 * math.pi:
            self.hypno_phase -= 2 * math.pi
        
        # Eye positioning
        spacing_pct = max(0.1, min(0.45, self.human_eye_spacing_pct))
        left_cx = int(self.display_size[0] * spacing_pct) + self.eye_center_offset - self.eye_spacing_offset
        right_cx = int(self.display_size[0] * (1.0 - spacing_pct)) + self.eye_center_offset + self.eye_spacing_offset
        cy = self.display_size[1] // 2 + self.eye_vertical_offset
        
        # Eye size
        eye_r = 35
        eye_height = eye_r * 2 + 6
        
        # Eyelid calculation
        blink_offset = eye_height * self.blink_percent * 0.6
        intensity_offset = self.eyelid_percent * 0.25
        height_offset = int(blink_offset + intensity_offset)
        dy = int(eye_r * math.atan(self.eyelid_angle * math.pi / 180))
        
        # Colors - high contrast with subtle colored background
        spiral_color_1 = self.hypno_color
        # Background: very dark version of the spiral color (not pure black)
        spiral_color_2 = tuple(max(0, c // 5) for c in spiral_color_1)  # ~20% brightness
        
        def draw_hypno_eye(cx, is_left_eye, direction=1):
            """Draw a single hypnotic spiral eye."""
            # Draw outer circle with dark colored background
            draw.ellipse((cx - eye_r - 3, cy - eye_r - 3, cx + eye_r + 3, cy + eye_r + 3),
                        fill=spiral_color_2, outline=spiral_color_1)
            
            # Draw spiral arms with high contrast
            num_arms = 6
            arm_width = 10  # Thicker arms for more visibility
            for arm in range(num_arms):
                base_angle = (2 * math.pi / num_arms) * arm + self.hypno_phase * direction
                
                # Draw spiral arm as series of small segments
                points = []
                for t in range(0, 30):
                    # Spiral equation: r increases as angle increases
                    r = (t / 30.0) * eye_r
                    angle = base_angle + (t / 30.0) * 3 * math.pi  # 1.5 full rotations
                    x = cx + r * math.cos(angle)
                    y = cy + r * math.sin(angle)
                    points.append((x, y))
                
                # Draw the spiral arm
                if len(points) >= 2:
                    # Use alternating colors for each arm
                    arm_color = spiral_color_1 if arm % 2 == 0 else spiral_color_2
                    for i in range(len(points) - 1):
                        draw.line([points[i], points[i + 1]], fill=arm_color, width=arm_width)
            
            # Draw center dot (black hole effect)
            center_r = 6
            draw.ellipse((cx - center_r, cy - center_r, cx + center_r, cy + center_r),
                        fill=(0, 0, 0), outline=(0, 0, 0))
            
            # Tiny white highlight for depth
            draw.ellipse((cx - 2, cy - 3, cx + 2, cy - 1), fill=(255, 255, 255))
            
            # Eyelid overlay
            if height_offset > 0 or dy != 0:
                eye_left = cx - eye_r - 8
                eye_right = cx + eye_r + 8
                eye_top = cy - eye_r - 8
                
                if is_left_eye:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset + 2 * dy
                    else:
                        lid_left_y = eye_top + height_offset - 2 * dy
                        lid_right_y = eye_top + height_offset
                else:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset + 2 * dy
                        lid_right_y = eye_top + height_offset
                    else:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset - 2 * dy
                
                poly = [(eye_left, eye_top - 10), (eye_left, lid_left_y),
                        (eye_right, lid_right_y), (eye_right, eye_top - 10)]
                draw.polygon(poly, fill="BLACK", outline="BLACK")
        
        # Draw eyes with opposite rotation directions for extra effect
        draw_hypno_eye(left_cx, is_left_eye=True, direction=1)
        draw_hypno_eye(right_cx, is_left_eye=False, direction=-1)
        
        if self.crt_mode:
            for i in range(0, self.display_size[1], 3):
                draw.line((0, i, self.display_size[0], i), fill="BLACK", width=1)

    def render_anime_eyes(self):
        """Render large kawaii anime-style eyes directly to display_image.
        
        Features:
        - Large, expressive eyes with big iris
        - Multiple large catchlight highlights (anime signature look)
        - Thick upper eyelid line
        - Sparkle effects
        - Subtle color gradient in iris
        """
        draw = ImageDraw.Draw(self.display_image)
        
        # Eye positioning - anime eyes are bigger and closer together
        spacing_pct = max(0.15, min(0.40, self.human_eye_spacing_pct * 0.9))
        left_cx = int(self.display_size[0] * spacing_pct) + self.eye_center_offset - self.eye_spacing_offset
        right_cx = int(self.display_size[0] * (1.0 - spacing_pct)) + self.eye_center_offset + self.eye_spacing_offset
        cy = self.display_size[1] // 2 + self.eye_vertical_offset
        
        # Anime eyes are large and round
        iris_r = 38
        pupil_r = int(iris_r * 0.35)
        eye_height = iris_r * 2 + 10
        
        # Eyelid calculation
        blink_offset = eye_height * self.blink_percent * 0.6
        intensity_offset = self.eyelid_percent * 0.25
        height_offset = int(blink_offset + intensity_offset)
        dy = int(iris_r * math.atan(self.eyelid_angle * math.pi / 180))
        
        # Colors
        iris_color = self.anime_eye_color
        iris_dark = tuple(max(0, c - 40) for c in iris_color)
        iris_light = tuple(min(255, c + 60) for c in iris_color)
        iris_highlight = tuple(min(255, c + 100) for c in iris_color)
        
        # Intensity affects sparkle size
        intensity_pct = min(1.0, self.eyelid_percent / 75.0)
        
        def draw_anime_eye(cx, is_left_eye):
            """Draw a single anime-style eye."""
            # 1. White of eye (sclera) - subtle outer glow
            draw.ellipse((cx - iris_r - 5, cy - iris_r - 5, cx + iris_r + 5, cy + iris_r + 5),
                        fill=(240, 240, 245), outline=(200, 200, 210))
            
            # 2. Outer iris (dark edge)
            draw.ellipse((cx - iris_r, cy - iris_r, cx + iris_r, cy + iris_r),
                        fill=iris_dark, outline=iris_dark)
            
            # 3. Main iris color
            mid_r = int(iris_r * 0.85)
            draw.ellipse((cx - mid_r, cy - mid_r, cx + mid_r, cy + mid_r),
                        fill=iris_color, outline=iris_color)
            
            # 4. Lighter inner iris (gradient effect)
            inner_r = int(iris_r * 0.6)
            draw.ellipse((cx - inner_r, cy - inner_r, cx + inner_r, cy + inner_r),
                        fill=iris_light, outline=iris_light)
            
            # 5. Pupil
            draw.ellipse((cx - pupil_r, cy - pupil_r, cx + pupil_r, cy + pupil_r),
                        fill=(5, 5, 10), outline=(5, 5, 10))
            
            # 6. Large primary catchlight (signature anime look - upper left)
            hl_x = cx - iris_r // 2.5
            hl_y = cy - iris_r // 2.5
            hl_r = 10
            draw.ellipse((hl_x - hl_r, hl_y - hl_r, hl_x + hl_r, hl_y + hl_r),
                        fill=(255, 255, 255))
            
            # 7. Secondary catchlight (lower right, smaller)
            hl2_x = cx + iris_r // 3
            hl2_y = cy + iris_r // 4
            hl2_r = 5
            draw.ellipse((hl2_x - hl2_r, hl2_y - hl2_r, hl2_x + hl2_r, hl2_y + hl2_r),
                        fill=(255, 255, 255))
            
            # 8. Sparkle effects (more prominent with intensity)
            sparkle_size = int(2 + 3 * intensity_pct)
            # Top sparkle
            sp_x = cx + iris_r // 4
            sp_y = cy - iris_r // 1.5
            draw.ellipse((sp_x - sparkle_size, sp_y - sparkle_size,
                         sp_x + sparkle_size, sp_y + sparkle_size), fill=(255, 255, 255))
            # Side sparkle
            sp2_x = cx - iris_r // 1.8
            sp2_y = cy + iris_r // 6
            sparkle_size2 = max(1, sparkle_size - 1)
            draw.ellipse((sp2_x - sparkle_size2, sp2_y - sparkle_size2,
                         sp2_x + sparkle_size2, sp2_y + sparkle_size2), fill=(255, 255, 255))
            
            # 9. Thick upper eyelid line (anime signature)
            lid_thickness = 4
            draw.arc((cx - iris_r - 3, cy - iris_r - 3, cx + iris_r + 3, cy + iris_r + 3),
                    start=200, end=340, fill=(30, 30, 35), width=lid_thickness)
            
            # 10. Eyelid overlay for blink
            if height_offset > 0 or dy != 0:
                eye_left = cx - iris_r - 10
                eye_right = cx + iris_r + 10
                eye_top = cy - iris_r - 10
                
                if is_left_eye:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset + 2 * dy
                    else:
                        lid_left_y = eye_top + height_offset - 2 * dy
                        lid_right_y = eye_top + height_offset
                else:
                    if self.eyelid_angle > 0:
                        lid_left_y = eye_top + height_offset + 2 * dy
                        lid_right_y = eye_top + height_offset
                    else:
                        lid_left_y = eye_top + height_offset
                        lid_right_y = eye_top + height_offset - 2 * dy
                
                poly = [(eye_left, eye_top - 10), (eye_left, lid_left_y),
                        (eye_right, lid_right_y), (eye_right, eye_top - 10)]
                draw.polygon(poly, fill="BLACK", outline="BLACK")
        
        draw_anime_eye(left_cx, is_left_eye=True)
        draw_anime_eye(right_cx, is_left_eye=False)
        
        if self.crt_mode:
            for i in range(0, self.display_size[1], 3):
                draw.line((0, i, self.display_size[0], i), fill="BLACK", width=1)
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
        self.eye_spacing_offset = 0 # number of pixels between the eyes to move them closer or farther apart (+ for closer, - for farther)
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
        if (self.blinkState != self.last_blink_state or 
            self.blinkState == BLINKSTATE.BLINKING or 
            self.blinkState == BLINKSTATE.CLOSED or
            force_update):

            self.display_image = Image.new("RGB", self.display_size, "BLACK")

            self.eye_image_left = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
            self.eye_draw_left = ImageDraw.Draw(self.eye_image_left)
            self.eye_image_right = Image.new("RGB", (self.eye_size[0]*2,self.eye_size[1]*2), "BLACK")
            self.eye_draw_right = ImageDraw.Draw(self.eye_image_right)
            self.render_eye()
            
            # draw the right eye
            middle = (self.eye_center_offset + self.eye_spacing_offset + self.display_size[0] * .25, (self.display_size[1] - 2) * .45)
            eye_rot = self.eye_image_right.rotate(-self.rotation, resample = 3, expand=True)
            self.display_image.paste(eye_rot, (int(middle[0] - eye_rot.width/2), int(middle[1] - eye_rot.height/2)))

            # draw the left eye
            middle = (self.eye_center_offset - self.eye_spacing_offset + self.display_size[0] * .75, (self.display_size[1] - 2) * .45)
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
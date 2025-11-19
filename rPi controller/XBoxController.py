# use evdev to connect to the gamepad and read the gamepad events
from evdev import InputDevice, ff, ecodes, list_devices

class XBoxController:

    def __init__(self):
        self.power_on = True
        self.device_file = None # InputDevice(file)
        self.joystick_left_y = 0 # values are mapped to [-1 ... 1]
        self.joystick_left_x = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_x = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_y = 0 # values are mapped to [-1 ... 1]
        self.trigger_right = 0 # values are mapped to [0 ... 1]
        self.trigger_left = 0 # values are mapped to [0 ... 1]
        self.joystick_right_button = False
        self.joystick_left_button = False
        self.button_x = False
        self.button_y = False
        self.button_b = False
        self.button_a = False
        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False
        self.bump_left = False
        self.bump_right = False
        self.button_menu = False
        self.rumble_effect = 0
        self.effect1_id = 0 # light rumble, played continuously
        self.effect2_id = 0 # strong rumble, played once
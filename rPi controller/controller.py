#----------------------------------------------------------------------------------------------------------------------
#    controller.py
#----------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------
#    Import all required libraries
#----------------------------------------------------------------------------------------------------------------------
import sys  
import time
import curses
import numpy as np
#import os
#import math
#import logging
import serial
import serial.tools.list_ports
import cv2 as cv
import numpy as np
#import operator
#import random

# use evdev to connect to the gamepad and read the gamepad events
from evdev import InputDevice, ff, ecodes, list_devices
import XBoxController

# import the simpleeyes library
from SimpleEyes import SimpleEyes as SimpleEyes, EYE_SHAPE, BLINKSTATE

# import the menu library
from Menu import TextMenuImage
from GUIMenu import GUIMenu

#import the steering mode enum
from SteeringMode import SteeringMode

# import the LCD library and related graphics libraries
# sys.path.append("..")
# sys.path.append("./")
#from lib import LCD_1inch9
import st7789
import cst816d
from PIL import Image, ImageDraw, ImageFont
import spidev as SPI

#----------------------------------------------------------------------------------------------------------------------
#
#    Define all helper functions here
#
#----------------------------------------------------------------------------------------------------------------------
# find_teensy scnas the available serial ports and looks for a Teensy device.
# return: serial port device or none if not found
def find_teensy(_verbose = False):
    # List all available serial ports
    returnPort = None
    if _verbose:
        print("Searching for Teensy...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Check if the device description matches a Teensy
        if _verbose:
            print( port.description, port.device, "-", port.manufacturer, "-", port.product)
        if "teensy" in port.manufacturer.lower() and "0" in port.device:
            if _verbose:
                print(f"Teensy found on port: {port.device}")
            return port
    if _verbose:
        print("No Teensy found.")
    return None

# readTeensy reads the Teensy device and returns the data
def readTeensy(teensy, _verbose = False):
    """Reads the Teensy device and returns the data"""
    if teensy is None:
        if _verbose:
            print("No Teensy device found.")
        return None
    try:
        # Open the serial port
        # Read data from the Teensy
        data = ""
        while teensy.in_waiting != 0:
            c = teensy.read(1)  # read one byte at a time
            data += c.decode('utf-8') # convert the byte to a string
        #if _verbose:
        #    print(f"Teensy data: {data.strip()}")
        return data.strip().replace('/r/n', '')  # return the data without leading/trailing whitespace
    except serial.SerialException as e:
        if _verbose:
            print(f"Error reading from Teensy: {e}")
        return None
    
# getGait converts the gait number to a string representation
def getGait(gait):
    # convert the gait number to a string
    if gait == 0:
        return "TRI"
    elif gait == 1:
        return "RIPPLE"
    elif gait == 2:
        return "WAV"
    elif gait == 3:
        return "QUAD"
    elif gait == 4:
        return "BI"
    elif gait == 5:
        return "HOP"
    elif gait == 6:
        return "TRANS"
    elif gait == 7:
        return "STILL"
    elif gait == 8:
        return "PRONE"
    elif gait == 9:
        return "NONE"
    else:
        return "UNKNOWN"

# testForGamePad checks for the presence of an Xbox controller and returns its path if found
def testForGamePad(verbose=False):
    controller = None
    #if verbose:
        #print('Connecting to xbox controller...')
    found = False
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        #if verbose:
        #    print(device.path, device.name)

        if str.lower(device.name) == 'xbox wireless controller':
            if verbose:
                print("Xbox controller found at:", device.path)
            controller = device
            found = True
            break
    #if not found:
       # if verbose:
        #    print("No Xbox controller found.")  
    return controller

# getkey is a helper function to check for keypresses in a curses application
def getkey(stdscr):
    """checking for keypress"""
    stdscr.nodelay(True)  # do not wait for input when calling getch
    return stdscr.getch()

def getColor(min_val, max_val, value, color_palette):
    """Returns a color based on the value and the color palette"""
    if value < min_val:
        return color_palette[0]  # return the first color in the palette
    elif value > max_val:
        return color_palette[len(color_palette)-1]  # return the last color in the palette
    else:
        # calculate the index based on the value
        # Calculate the position in the palette
        scaled = (value - min_val) / (max_val - min_val) * (len(color_palette) - 1)
        lower_idx = max(0,int(np.floor(scaled)))
        upper_idx = min(len(color_palette) - 1, lower_idx + 1)  # ensure
        if lower_idx == upper_idx:
            return color_palette[lower_idx]
        # Linear interpolation between the two colors
        ratio = scaled - lower_idx
        color1 = np.array(color_palette[lower_idx])
        color2 = np.array(color_palette[upper_idx])
        interp_color = (1 - ratio) * color1 + ratio * color2
        #print(min_val, max_val, value, scaled, lower_idx, upper_idx, ratio, color1, color2, interp_color,tuple(interp_color.astype(int)))
        return tuple(interp_color.astype(int))


def drawLogo(disp):
    """Draws a logo on the display"""
    # Create a new image with a white background
    image = Image.new("RGB", (disp.height, disp.width), "BLACK")
    draw = ImageDraw.Draw(image)

    # Draw a simple logo (a blue rectangle with text)
    fontMICR = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf", 6)
    Color = "WHITE"
    Top = 10
    lineHeight = 8
    draw.text((20, Top), u"   ____  ____    ____     __  __ __  ____    ___   ______  ____    ___   ____  \n", fill = Color, font=fontMICR)
    draw.text((20, Top + lineHeight), u"  /    ||    \\  /    |   /  ]|  |  ||    \\  /   \\ |      ||    \\  /   \\ |    \\ \n", fill = Color, font=fontMICR)
    draw.text((20, Top + 2*lineHeight), u" |  o  ||  D  )|  o  |  /  / |  |  ||  _  ||     ||      ||  D  )|     ||  _  |\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 3*lineHeight), u" |     ||    / |     | /  /  |  _  ||  |  ||  O  ||_|  |_||    / |  O  ||  |  |\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 4*lineHeight), u" |  _  ||    \\ |  _  |/   \\_ |  |  ||  |  ||     |  |  |  |    \\ |     ||  |  |\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 5*lineHeight), u" |  |  ||  .  \\|  |  |\\     ||  |  ||  |  ||     |  |  |  |  .  \\|     ||  |  |\n", fill = Color, font=fontMICR)
    draw.text((20, Top + 6*lineHeight), u" |__|__||__|\\_||__|__| \\____||__|__||__|__| \\___/   |__|  |__|\\_| \\___/ |__|__|\n", fill = Color, font=fontMICR)
    #image=image.rotate(180, expand=True)  # rotate the image to fit the display

    # Show the image on the display
    UpdateDisplay(disp, image, None)

def UpdateDisplay(disp, image, menu, servo=None, legs = None, state = None, mirror=False, menuState=None):
    """Updates the display with the given image"""

    imageCopy = image.copy()  # create a copy of the image to draw on
    # add in the leg/servo visualization
    if servo is not None:
        # create a draw object to draw on the image
        draw = ImageDraw.Draw(imageCopy)

        if _menuState == "data":
            # draw the gait text on the image
            if state is not None:
                gait = getGait(int(state[6]))
                if state[9] == 1:
                    gait = "(FLOAT)"
                textSize = draw.textbbox(((disp.height / 2), disp.width - 24), gait, font=ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf", 24), anchor="mt")
                center = disp.height / 2
                blackoutSpace = 70
                draw.rectangle((center - blackoutSpace, disp.width - 24, center + blackoutSpace, disp.width), fill="BLACK")
                draw.text((textSize[0], textSize[1]), gait, fill="WHITE", font=ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf", 24))

            for idx, (voltage, temperature, enable) in enumerate(servo):
                #print(f"Servo {idx}: Voltage: {voltage}, Temperature: {temperature}")
                # draw the servo telemetry data on the image
                vColor = "GRAY"
                tColor = "GRAY"            
                width = 10
                height = 10
                gap = 79.5
                leg = int(idx / 3)
                left = (idx - (leg*3)) * (width + 2)
                top = gap * (2-leg)
                dir = 1
                borderColor = "WHITE"
                if legs[leg][0] == 0:
                    borderColor = "RED"
                if enable == 1:
                    vColor = getColor(10.5,12.5, voltage, _powercolorPalette)
                    tColor = getColor(20, 75, temperature, _temperatureColorPalette)
                    #print(idx, vColor, tColor)
                leg = int(idx / 3)
                left = (idx - (leg*3)) * (width + 2)
                top = int(gap * (2-leg))
                dir = 1
                if leg >= 3:
                    leg -= 3
                    dir = -1
                    left = disp.height - left
                    top = int(gap * leg)
                draw.rectangle((left, top, left + dir*4, top + height), tColor)
                draw.rectangle((left + dir*5, top, left + dir*9, top + height), vColor)
                draw.rectangle((left,top, left + dir*width, top + height), outline=borderColor, width=1)
        elif _menuState == "settings":
            imageCopy = Image.blend(imageCopy, menu, 0.6)  # blend the menu image onto the display image
    if mirror:
        image2 = np.array(imageCopy)                        
        height, width = image2.shape[:2]
        image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv.INTER_LANCZOS4)
        cv.imshow("arachnotron", image2)
        cv.waitKey(1)
    else:
        cv.destroyAllWindows()
    imageCopy=imageCopy.rotate(270, expand=True)  # rotate the image to fit the display
    disp.show_image(imageCopy)  # display the image on the display

def processTelemS1(elements, state):
    """Processes the S1 telemetry data from the Teensy device"""
    if len(elements) < 2:
        print("Invalid S1 telemetry data")
        return

    try:
   
        # Extract the telemetry data from the elements...
        # Example: elements = ['S1:', 'gait', '0', 'speed', '100'....]
        state[0] = float(elements[0])  # loop time
        state[1] = float(elements[1])  # center x
        state[2] = float(elements[2])  # center y
        state[3] = float(elements[3])  # motor enabled
        state[4] = float(elements[4])  # last gait
        state[5] = float(elements[5])  # new gait
        state[6] = float(elements[6])  # current gait
        state[7] = float(elements[7])  # phase
        state[8] = float(elements[8])  # current servo
        state[9] = float(elements[9])  # motor float
    except ValueError as e:
        print(f"Error processing S1 telemetry data: {e}")


def processTelemS2(elements, servo):
    """Processes the S1 telemetry data from the Teensy device"""
    if len(elements) < 2:
        print("Invalid S1 telemetry data")
        return

    try:
        # Extract the telemetry data from the elements
        # Example: elements = ['S1:', 'gait', '0', 'speed', '100']
        for idx in range(18):
            servo[idx][2] = int(elements[idx])  # servo voltage
            #legs[idx][1] = int(elements[idx + 18])  # servo temperature
            #legs[idx][2] = int(elements[idx + 6])  # contact state
        #print(servo)
    except ValueError as e:
        print(f"Error processing S1 telemetry data: {e}")

def processTelemS3(elements, servo):
    """Processes the S3 telemetry data from the Teensy device"""
    if len(elements) < 2:
        print("Invalid S3 telemetry data")
        return

    try:
        # Extract the telemetry data from the elements
        # Example: elements = ['S3:', 'gait', '0', 'speed', '100']
        for idx in range(18):
            servo[idx][0] = int(elements[idx])  # servo voltage
            servo[idx][1] = int(elements[idx + 18])  # servo temperature
    except ValueError as e:
        print(f"Error processing S3 telemetry data: {e}")

def processTelemS4(elements, leg):
    """Processes the S4 telemetry data from the Teensy device"""
    if len(elements) < 2:
        print("Invalid S4 telemetry data")
        return

    try:
        # Extract the telemetry data from the elements
        # Example: elements = ['S4:', contact, '0', 'angle1', '100', 'angle2', '200', ...]
        for idx in range(6):
            leg[idx][0] = int(elements[idx])  # leg contact state
    except ValueError as e:
        print(f"Error processing S4 telemetry data: {e}")

#-----------------------------------------------------------------------------------------------------------------------
#    Define the main controller class
#-----------------------------------------------------------------------------------------------------------------------

# initialze the global variables here
_run = True # controls whether the mail loop is running
_verbose = True # enables verbose logging and debugging
_showLoopTime = False # flag to show the loop timing
_logging = True # flage the use of the logging module
_loopStartTime = time.time() # used to control the loop timing
_loopdt = 0.0 # stores the time delta for the loop
_screenRefreshms = 100. # time in milliseconds to refresh the screen (10Hz)
_retryCount = 0 # number of retries to connect to the controller device
_menuVisable = False # flag to show or hide the menu
_mirrorDisplay = False # flag to mirror the display to the console
_forceDisplayUpdate = False # flag to force the display update
_displayBrightness = 100 # initial brightness of the display
_menuState = None
_steeringMode = SteeringMode.OMNI  # initial steering mode

# create the Teensy serial object
_teensy = None
# create the Xbox controller object
_controller = None

# define the state array to store the state telemetry data
_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # # 11 elements for the state telemetry data

# define the servo array to store servo telemetry data
_servo = [[0.0, 0.0, 0.0] for _ in range(18)]  # 18 servos, each with voltage and temperature

# define the legs array to store leg telemetry data
_legs = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]  # 6 legs, each with contact and angles

# define the pallettes for the color mapping
_powercolorPalette = [(255,50,50), (50, 255, 50)]
_temperatureColorPalette = [(0,191,255), (0, 255, 0), (255, 0, 0)]  # blue to green to red

# initialize the display and start logging to it
#_disp = LCD_1inch9.LCD_1inch9()
_disp = st7789.st7789()
#_menu._disp = _disp  # set the display for the menu

#_disp.Init()
_disp.clear()
_disp.bl_DutyCycle(_displayBrightness) # turn the backlight to full on
_backGroundImage = Image.new("RGB", (_disp.width, _disp.height), "BLACK")  # create a blank image for the background
#_backGroundImage = Image.open("/home/starter/Downloads/LCD_Module_RPI_code/RaspberryPi/python/pic/LCD_1inch9_1.jpg")  # create a blank image for the background
#_backGroundImage = Image.open("/home/starter/OneDrive/Projects/Robots/hexapod-main/Illustrations/Full Body - Small  2.PNG")  # create a blank image for the background
#_backGroundImage = _backGroundImage.convert("RGB")  # convert the image to RGB                                                                                                                          
#r,g,b = _backGroundImage.split()  # split the image into RGB channels
#_backGroundImage = Image.merge("RGB", (g,b,r))  # merge the channels back together
#_backGroundImage = _backGroundImage.rotate(270, expand=True)  # rotate the image to fit the display
#_backGroundImage = _backGroundImage.convert("BGR")  # convert the image to RGB
#_backGroundImage = _backGroundImage.resize((_disp.width, _disp.height), Image.Resampling.LANCZOS)  # resize the image to fit the display
UpdateDisplay(_disp, _backGroundImage, None, _servo, _legs, _state, _mirrorDisplay)  # display the blank image
drawLogo(_disp)  # draw the logo on the display

# initizlizxe the touch screen
_touch = cst816d.cst816d()

#create the menu object
_menu = GUIMenu(_touch)

time.sleep(1) # take a short nap

# create and initialize the SimpleEyes object
_eyes = SimpleEyes((_backGroundImage.height, _backGroundImage.width), eye_color=(10,120,255))
_eyeColors = [(255,80,20), (255,255,255), (10,120,255), (255,0,0)]
_eyes.left_shape = EYE_SHAPE.ROUNDRECTANGLE
_eyes.right_shape = EYE_SHAPE.ROUNDRECTANGLE
_eyes.eye_color = _eyeColors[_eyes.left_shape]
_eyes.eye_size = (25,45)
_eyes.rotation = -10
_eyes.eye_spacing_offset = 10
_eyes.eye_center_offset = 5
_eyes.eyelid_angle = 0
_eyes.blink_percent_step = 0.25
_eyes.update(force_update=True)  # force the initial update to draw the eyes
UpdateDisplay(_disp, _eyes.display_image, _menu.image, _servo, _legs, _state, _mirrorDisplay, _menuState)  # display the eyes on the display

# start the main lqoop
while _run:

    #TODO: add startup logging to the display
    #TODO: add logging to a file or display
    #TODO: add emotion detection and display
    #TODO: control the Teensy device based on the Xbox controller input
    #TODO: create a controller class to process and store events from the Xbox controller
    #TODO: add an blinking icon if the controller is not connected
    #TODO: add an error display if the Teensy is not connected
    #TODO: add scrolling text display to the display - connected to the logging ability
    #TODO: add a toggle between the eyes, text output, and a camera feed on the display
    #TODO: Add a pupil to the eyes that follows the motion of the controller joystick
    #TODO: Turn on the menu with a touch on the screen
    #DONE
    #TODO: add loop timing to control the loop speed - Done 6/18/25
    #TODO: add a way to exit the loop gracefully - Controller X button and ESC key - Done 6/19/25
    #TODO: add try catch to the entire main loop to handle exceptions gracefully - Done 6/19/25
    #TODO: add eyes and blinking - Done 6/21/25

    
    try:

        # calculate the loop time
        curTime = time.time()  # get the current time
        _loopdt = curTime - _loopStartTime  # calculate the time delta
        _loopStartTime = curTime  # update the loop start time

        # check for screen refresh timing
        _screenRefreshms -= _loopdt * 1000.0  # convert the loop time to milliseconds
        if _screenRefreshms <= 0.0:  # if the screen refresh time has elapsed
            _screenRefreshms = 100.0  # reset the screen refresh time
            _forceDisplayUpdate = True  # force the display

        if _showLoopTime:
            print(f"Loop time: {_loopdt:.4f} seconds, {_screenRefreshms:.2f} ms remaining until next screen refresh")

        # connect to the Teensy device
        if _teensy is None:
            teensyPath = find_teensy(_verbose)
            if teensyPath is not None:
                _teensy = serial.Serial(teensyPath.device, baudrate=1000000, timeout=1)
                #reset the teensy and wait for the cli prompt
                #_teensy.write(b'M;q;')
        else:
            teensyData = readTeensy(_teensy, _verbose)  # read the Teensy device for any commands or data
            if teensyData is not None:
                dataLines = teensyData.splitlines()  # split the data into lines
                for line in dataLines:
                    segments = line.split('|')  # split the line into segments
                    for segment in segments:
                        elements = segment.split(',')  # split the segment into elements
                        header = elements[0][0:3]  # get the header of the segment
                        elements[0] = elements[0][3:]  # remove the SX: prefix
                        #print(f"Received segment: {header}")  # print the received segment
                        #print(f"Received segment: {elements}")  # print the received segment
                        if header == 'S1:':
                            # process the segment for the S1 command
                            processTelemS1(elements, _state)
                            if _verbose:
                                pass
                                #print(f"Received S1 command: {elements}")
                        elif header == 'S2:':
                            # process the segment for the S2 command
                            processTelemS2(elements, _servo)
                            if _verbose:
                                pass
                                #print(f"Received S2 command: {elements}")
                        elif header == 'S3:':
                            # process the segment for the S3 command
                            processTelemS3(elements, _servo)
                            # if _verbose:
                            #     print(f"Received S3 command: {elements, _servo}")
                        elif header == 'S4:':
                            # process the segment for the S3 command
                            processTelemS4(elements, _legs)
                            # if _verbose:
                            #     print(f"Received S3 command: {elements, _servo}")
        # connect to the Xbox controller
        if _retryCount == 0 and _controller is None:
            _controller = testForGamePad(_verbose)
            _retryCount = 100  # reset the retry count
        elif _controller is None:
            _retryCount -= 1 #decrement the retry count

        # update the eyes and display them
        if _eyes.update() or _forceDisplayUpdate:

            UpdateDisplay(_disp, _eyes.display_image, _menu._image, _servo, _legs, _state, _mirrorDisplay, _menuState)
            _forceDisplayUpdate = False  # reset the force display update flag
                    
        # if the controller is connected, read the events
        if _controller is not None:
            try:
                # read the events from the controller
                events = _controller.read()
                for event in events:
                    # print the event
                    # if _verbose:
                    #     print(event)
                    if event.type == 1:
                        # handle button press events
                        if event.code == 158 and event.value == 1: # menu button pressed
                            if _verbose:
                                print("\nscreen mirror button pressed")
                            _mirrorDisplay = not _mirrorDisplay # toggle the menu visibility
                            _forceDisplayUpdate = True  # display the eyes on the display
                        elif event.code == 315 and event.value == 1: # menu button pressed
                            if _verbose:
                                print("\nmenu button pressed")
                            if _menuState is None:
                                _menuState = "data"
                            elif _menuState == "data":
                                _menuState = "settings"
                            elif _menuState == "settings":
                                _menuState = None
                            _menuVisable = _menuState != None # toggle the menu visibility
                            _forceDisplayUpdate = True  # display the menu
                        elif event.code == 172 and event.value == 1: # power button pressed
                            if _verbose:
                                print("\npower button pressed")
                            # send a command to the Teensy device
                            _run = False  # exit the loop
                        elif event.code == 304 and event.value == 1: #A button pressed - transition to stationary gait
                            if _verbose:
                                print("\nA button pressed\n")
                            _teensy.write(b's;')  # send move command to the Teensy device
                        elif event.code == 305 and event.value == 1: #B button pressed - Start motion
                            if _verbose:
                                print("\nB button pressed\n")
                            _teensy.write(b'm;')  # send move command to the Teensy device
                        elif event.code == 306 and event.value == 1: #X button pressed
                            if _verbose:
                                print("\nX button pressed\n")
                            #_teensy.write(b'x;')  # send stop command to the Teensy device
                        elif event.code == 307 and event.value == 1: #X button pressed
                            if _verbose:
                                print("\nX button pressed\n")
                            
                            # cycle through the different gaits
                            if _state[6] == 0:
                                _teensy.write(b'r;')  # send wave gait the Teensy device
                            elif  _state[6] == 1:
                                _teensy.write(b'w;')  # send ripple gait to the Teensy device
                            elif  _state[6] == 2:
                                _teensy.write(b't;')  # send tri gait to the Teensy device
                            else:  
                                _teensy.write(b't;')  # default send tri gait to the Teensy device
                        elif event.code == 308 and event.value == 1: #Y button pressed - Shut down all motors
                            if _verbose:
                                print("\nY button pressed\n")
                            _teensy.write(b'f;')  # send stop command to the Teensy device
                        elif event.code == 317 and event.value == 1: # left joystick button pressed
                            if _verbose:
                                print("\nLeft joystick button pressed\n")
                            _steeringMode = SteeringMode.ACKERMAN  # set the steering mode to ACKERMAN
                            _teensy.write(b'b1;')  # send ackerman command to the Teensy device
                        elif event.code == 317 and event.value == 0: # left joystick button released
                            if _verbose:
                                print("\nLeft joystick button released\n")
                            # send a command to the Teensy device
                            _steeringMode = SteeringMode.OMNI  # set the steering mode to NONE
                            _teensy.write(b'b0;')  # send ackerman command to the Teensy device
                    elif event.type == 3: # joystick events
                        # handle joystick events
                        if event.code == 0: # left stick X axis
                            if _verbose:
                                print(f"Left Stick X: {event.value}")
                            #value = ((32768.0 - float(event.value)) / 32768.0) * 180 / np.pi  #center and scale the value from -180 to 180
                            #_teensy.write(f'x{value};'.encode('utf-8'))  # send the direction command to the Teensy device
                        elif event.code == 1: # left stick y axis
                            if _verbose:
                                print(f"Left Stick Y: {event.value}")
                            # send the speed command to the Teensy device
                            value = max(0,(32768.0 - float(event.value)) / 32768.0)  # center and scale the value from -1 to 1, clamp 0 to 1
                            # have the eyelids come down to show determination
                            _eyes.eyelid_percent = int(value * 0.755)  # scale the value to the eyelid percent
                            _eyes.eyelid_angle = value * 35.0  # scale the value to the eyelid angle
                            _eyes.eye_size = (25, 45 - int(value * 10))  # scale the value to the eye size
                            _forceDisplayUpdate = True  # force the display update
                            _eyes.update(force_update=True)  # force the update to draw the eyes
                            _teensy.write(f'z{value};'.encode('utf-8'))  # send the direction command to the Teensy device
                        elif event.code == 2: # left stick X axis

                            # handle the left stick X axis for steering
                            if _verbose:
                                print(f"Right Stick X: {event.value}")
                            scaledValue = ((32768.0 - float(event.value)) / 32768.0) #center and scale the value from 1 to -1

                            # limit the scaled value to the range of -0.605 to 0.605 if in ACKERMAN mode
                            if _steeringMode == SteeringMode.ACKERMAN:
                                scaledValue = max(-0.605, min(0.605, scaledValue))
                            
                            # have the eyes look in the direction of the joystick
                            _eyes.eye_center_offset = int(scaledValue * 56.0)  # scale the value to the eye center offset
                            _forceDisplayUpdate = True  # force the display update
                            _eyes.update(force_update=True)  # force the update to draw the eyes
                            print( scaledValue)
                            _teensy.write(f'x{scaledValue * np.pi};'.encode('utf-8'))  # send the direction command to the Teensy device                        
                        elif event.code == 16 and event.value == 1: # right DPAD
                            # _menu.move_in()  # move into the current menu item
                            # _menu.select()  # select the current menu item
                            # _menu.expand()  # expand the current menu item
                            if _menuState == "settings":
                                _menu.contract(_menu._selected_path)
                            else:
                                _eyes.crt_mode = not _eyes.crt_mode  # toggle the CRT mode
                                _eyes.update(force_update=True)  # force the update to draw the eyes
                                _forceDisplayUpdate = True  # display the eyes on the display
                            if _verbose:
                                print(f"DPAD RIGHT: {event.value}")
                        elif event.code == 16 and event.value == -1: # left DPAD
                            # _menu.move_out()  # move into the current menu item
                            # _menu.select()  # select the current menu item
                            # _menu.expand()  # expand the current menu item
                            if _menuState == "settings":
                                _menu.contract(_menu._selected_path)
                            else:
                                _eyes.crt_mode = not _eyes.crt_mode  # toggle the CRT mode
                                _eyes.update(force_update=True)  # force the update to draw the eyes
                                _forceDisplayUpdate = True  # display the eyes on the display
                            if _verbose:
                                print(f"DPAD LEFT: {event.value}")
                        elif event.code == 17 and event.value == -1: # DOWN DPAD
                            # left stick X axis
                            if _verbose:
                                print(f"DPAD UP: {event.value}")
                            if _menuState == "settings":
                                _menu.contract(_menu._selected_path)  # collapse the current menu item
                                _menu.move_up()  # move the menu selection down
                                _menu.select()  # select the current menu item
                                _menu.expand()  # expand the current menu item
                            else:
                                _eyes.left_shape -= 1
                                if _eyes.left_shape < EYE_SHAPE.ELLIPSE:
                                    _eyes.left_shape = EYE_SHAPE.X
                                _eyes.right_shape -= 1
                                if _eyes.right_shape < EYE_SHAPE.ELLIPSE:
                                    _eyes.right_shape = EYE_SHAPE.X
                                _eyes.eye_color = _eyeColors[_eyes.left_shape]
                                _eyes.update(force_update=True)  # force the update to draw the eyes
                                _forceDisplayUpdate = True  # display the eyes on the display
                        elif event.code == 17 and event.value == 1: # UP DPAD
                            if _verbose:
                                print(f"DPAD DOWN: {event.value}")
                            # _menu.contract(_menu._selected_path)  # collapse the current menu item
                            # _menu.move_down()  # move the menu selection down
                            # _menu.select()  # select the current menu item
                            # _menu.expand()  # expand the current menu item
                            if _menuState == "settings":
                                _menu.contract(_menu._selected_path)
                            else:
                                _eyes.left_shape += 1
                                if _eyes.left_shape > EYE_SHAPE.X:
                                    _eyes.left_shape = EYE_SHAPE.ELLIPSE
                                _eyes.right_shape += 1
                                if _eyes.right_shape > EYE_SHAPE.X:
                                    _eyes.right_shape = EYE_SHAPE.ELLIPSE
                                _eyes.eye_color = _eyeColors[_eyes.left_shape]
                                _eyes.update(force_update=True)  # force the update to draw the eyes
                                _forceDisplayUpdate = True  # display the eyes on the display
                    elif event.type == 4:
                        pass

            except KeyboardInterrupt as keyExcept:
                # handle keyboard interrupt gracefully
                _run = False
            except BlockingIOError as bioe:
                # Handle the case where no events are available
                pass
            except Exception as e:
                if _verbose:
                    print(f"Error reading from gamepad: {e}")
                _controller = None

        if _menu.touched():  # check if the touch screen has been touched
            _menuVisable = True  # toggle the menu visibility
            _forceDisplayUpdate = True  # display the menu on the display
            _menu.handle_touch()  # process the touch event

        # process key commands
        key = curses.wrapper(getkey)
        if key == 27:  # ESC key pressed
            if _verbose:
                print("\nESC key pressed, exiting loop")
            _run = False
        elif key == ord('n'):  # 'm' key pressed
            if _menuState is None:
                _menuState = "data"
            elif _menuState == "data":
                _menuState = "settings"
            elif _menuState == "settings":
                _menuState = None
            _menuVisable = _menuState != None # toggle the menu visibility
            _forceDisplayUpdate = True

            if _verbose:
                print("\nMenu visibility toggled ON" if _menuVisable else "\nMenu visibility toggled OFF")
        elif key == ord('m'):  # 'm' key pressed
            _mirrorDisplay = not _mirrorDisplay
            if _verbose:
                print("\nMirror display mode ON" if _mirrorDisplay else "\nMirror display mode OFF")
            _forceDisplayUpdate = True  # display the eyes on the display
        elif key == ord('v'):  # 'v' key pressed
            _verbose = not _verbose
            if _verbose:
                print("\nverbose mode ON")
            else:
                print("\nverbose mode OFF")
        elif key == ord('q'):  # 'q' key pressed
            _teensy.write(b'q;')  # send stop command to the Teensy device
            time.sleep(0.1)  # give the Teensy time to process the command
            _teensy.write(b'f;')  # send stop command to the Teensy device
            if _verbose:
                print("\n'q' key pressed, sending stop command to Teensy")
        elif key == ord('x'):  # 'x' key pressed
            if _verbose:
                print("\n'x' key pressed, exiting loop")
            _run = False

        # take a nap
        time.sleep(0.005)
    except KeyboardInterrupt as keyExcept:
        _run = False
    except Exception as e:
        # handle any other exceptions that may occur
        if _verbose:
            print(f"Error in main loop: {e}")
    # finally:
    #     print("Cleaning up...")

# clean up all used objects and resources
if _verbose:
    print("\nExiting main loop and cleaning up...")
_controller = None
_eyes = None
_disp.clear()
_disp.bl_DutyCycle(0)
#UpdateDisplay(_disp, _backGroundImage, None, _legs, _state, _mirrorDisplay)  # clear the display
cv.destroyAllWindows()
#time.sleep(1.1)  # give the display time to clear
#_disp = None
_touch = None
#_teensy.
_teensy = None


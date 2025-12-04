#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
example class to use the controller with asyncio
python>=3.6 is necessary
script is tested on a raspberry pi 3
"""
import sys 
import time
import math
import logging
import spidev as SPI
import serial
import serial.tools.list_ports
import asyncio
import cv2 as cv
import numpy as np
import operator
import random
from evdev import InputDevice, ff, ecodes, list_devices
sys.path.append("..")
sys.path.append("./")
#import SimpleEyes
from lib import LCD_1inch9
from PIL import Image, ImageDraw, ImageFont
from SimpleEyes import SimpleEyes as SimpleEyes, EYE_SHAPE, BLINKSTATE

class gamepad():
    def __init__(self, file = '/dev/input/event3'):
        #self.event_value = 0
        self.power_on = True
        self.device_file = InputDevice(file)
        self.joystick_left_y = 0 # values are mapped to [-1 ... 1]
        self.joystick_left_x = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_x = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_y = 0 # values are mapped to [-1 ... 1]
        self.joystick_right_button = False
        self.joystick_left_button = False
        self.trigger_right = 0 # values are mapped to [0 ... 1]
        self.trigger_left = 0 # values are mapped to [0 ... 1]
        self.button_left = False
        self.button_right = False
        self.button_x = False
        self.button_y = False
        self.button_a = False
        self.button_b = False
        self.button_menu = False
        self.rumble_effect = 0
        self.effect1_id = 0 # light rumble, played continuously
        self.effect2_id = 0 # strong rumble, played once
        self.load_effects()


    def load_effects(self):
        try:
            #effect 1, light rumble
            rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0x500)
            duration_ms = 300
            effect = ff.Effect(ecodes.FF_RUMBLE, -1, 0, ff.Trigger(0, 0), ff.Replay(duration_ms, 0), ff.EffectType(ff_rumble_effect=rumble))
            self.effect1_id = self.device_file.upload_effect(effect)
            # effect 2, strong rumble
            rumble = ff.Rumble(strong_magnitude=0xc000, weak_magnitude=0x0000)
            duration_ms = 200
            effect = ff.Effect(ecodes.FF_RUMBLE, -1, 0, ff.Trigger(0, 0), ff.Replay(duration_ms, 0), ff.EffectType(ff_rumble_effect=rumble))
            self.effect2_id = self.device_file.upload_effect(effect)
        except Exception as e:
            logging.error("Error loading rumble effects: %s", e)
            self.effect1_id = 0
            self.effect2_id = 0 

    async def read_gamepad_input(self): # asyncronus read-out of events
        max_abs_joystick_left_x = 0xFFFF/2
        uncertainty_joystick_left_x = 2500
        max_abs_joystick_left_y = 0xFFFF/2
        uncertainty_joystick_left_y = 2500
        max_abs_joystick_right_x = 0xFFFF/2
        uncertainty_joystick_right_x = 2000
        max_abs_joystick_right_y = 0xFFFF/2
        uncertainty_joystick_right_y = 2500
        max_trigger = 1023

        async for event in self.device_file.async_read_loop():
                if not(self.power_on): #stop reading device when power_on = false
                    break
                if event.type == 3: # type is analog trigger or joystick
                    if event.code == 1: # left joystick y-axis
                        if -event.value > uncertainty_joystick_left_y:
                            self.joystick_left_y = (-event.value - uncertainty_joystick_left_y) / (max_abs_joystick_left_y - uncertainty_joystick_left_y + 1)
                        elif -event.value < -uncertainty_joystick_left_y:
                            self.joystick_left_y = (-event.value + uncertainty_joystick_left_y) / (max_abs_joystick_left_y - uncertainty_joystick_left_y + 1)
                        else:
                            self.joystick_left_y = 0
                    elif event.code == 0: # left joystick x-axis
                        if event.value > uncertainty_joystick_left_x:
                            self.joystick_left_x = (event.value - uncertainty_joystick_left_x) / (max_abs_joystick_left_x - uncertainty_joystick_left_x + 1)
                        elif event.value < -uncertainty_joystick_left_x:
                            self.joystick_left_x = (event.value + uncertainty_joystick_left_x) / (max_abs_joystick_left_x - uncertainty_joystick_left_x + 1)
                        else:
                            self.joystick_left_x = 0
                    elif event.code == 3: # right joystick x-axis
                        if event.value > uncertainty_joystick_right_x:
                            self.joystick_right_x = (event.value - uncertainty_joystick_right_x) / (max_abs_joystick_right_x - uncertainty_joystick_right_x + 1)
                        elif event.value < -uncertainty_joystick_right_x:
                            self.joystick_right_x = (event.value + uncertainty_joystick_right_x) / (max_abs_joystick_right_x - uncertainty_joystick_right_x + 1)
                        else:
                            self.joystick_right_x = 0
                    elif event.code == 4: # left joystick y-axis
                        if -event.value > uncertainty_joystick_right_y:
                            self.joystick_right_y = (-event.value - uncertainty_joystick_right_y) / (max_abs_joystick_right_y - uncertainty_joystick_right_y + 1)
                        elif -event.value < -uncertainty_joystick_left_y:
                            self.joystick_right_y = (-event.value + uncertainty_joystick_right_y) / (max_abs_joystick_right_y - uncertainty_joystick_right_y + 1)
                        else:
                            self.joystick_right_y = 0
                    elif event.code == 5: # right trigger
                        self.trigger_right = event.value / max_trigger
                        #print( event.code, event.value, max_trigger )
                    elif event.code == 2: # left trigger
                        self.trigger_left = event.value / max_trigger
                if (event.type == 1): # type is button
                    print(event.code, event.value)
                    if event.code == 307: # button "X" pressed ?
                        self.button_x = True
                    if event.code == 308: # button "Y" pressed ?
                        self.button_y = True
                    if event.code == 305: # button "B" pressed ?
                        self.button_b = True
                    if event.code == 304: # button "A" pressed ?
                        self.button_a = True
                    if event.code == 310: # button "LB" pressed ?
                        self.button_left = True
                    if event.code == 311: # button "RB" pressed ?
                        self.button_right = True
                    if event.code == 315:
                        self.button_menu= True if event.value == 1 else False
                    if event.code == 318:
                        self.joystick_right_button = True if event.value == 1 else False
                    if event.code == 317:
                        self.joystick_left_button = True if event.value == 1 else False

    async def rumble(self): # asyncronus control of force feed back effects
        repeat_count = 1
        while self.power_on:
            if self.rumble_effect == 1:
                self.device_file.write(ecodes.EV_FF, self.effect1_id, repeat_count)
            elif self.rumble_effect == 2:
                self.device_file.write(ecodes.EV_FF, self.effect2_id, repeat_count)
                self.rumble_effect = 0 # turn of effect in order to play effect2 only once
            await asyncio.sleep(0.2)

    def erase_rumble(self):
        self.device_file.erase_effect(self.effect1_id)

def find_teensy():
    # List all available serial ports
    print("Searching for Teensy...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Check if the device description matches a Teensy
        print( port.description, port.device, "-", port.manufacturer, "-", port.product)
        if "teensy" in port.manufacturer.lower():
            print(f"Teensy found on port: {port.device}")
            return port.device
    print("No Teensy found.")
    return None

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

def testForGamePad():
    xbox_path = None
    remote_control = None
    _useGamePad = True
    devices = [InputDevice(path) for path in list_devices()]
    print('Connecting to xbox controller...')
    found = False
    for device in devices:
        #print(device.path, device.name) 
        if str.lower(device.name) == 'xbox wireless controller':
            xbox_path = str(device.path)
            found = True
            break
    if not found:
        print("No Xbox controller found.  Exiting...")  
        _useGamePad = False
    return _useGamePad, xbox_path
        
# old draw screen function    
def draw_screen(data = None):

    startTime = time.time()
    gait = 0

    # set the colors
    uiBorderColor = (0, 255, 253)
    uiAccentColor = (89, 30, 38 )
    uiPhaseStartColor = uiAccentColor #(32, 32, 32)

    # load the fonts
    fontMICR_6pt = ImageFont.truetype("/usr/share/fonts/truetype/freefont/25240_MICR.ttf", 6)
    fontMICR_10pt = ImageFont.truetype("/usr/share/fonts/truetype/freefont/25240_MICR.ttf", 10)
    fontMICR_16pt = ImageFont.truetype("/usr/share/fonts/truetype/freefont/25240_MICR.ttf", 16)
    fontMICR_14pt = ImageFont.truetype("/usr/share/fonts/truetype/freefont/25240_MICR.ttf", 14)

    # create a new blank image
    image1 = Image.new("RGB", (disp.height,disp.width), (0,49,45))
    draw = ImageDraw.Draw(image1)
    draw.line((2,2,2,disp.width-12), fill = uiBorderColor, width = 1) # left side
    draw.line((2,disp.width-12, 12, disp.width - 2, ), fill = uiBorderColor, width = 1) # bottom left diagnal  
    draw.line((12,disp.width-2, disp.height - 120, disp.width -2), fill = uiBorderColor, width = 1) # bottom 
    draw.line((disp.height - 120, disp.width - 2, disp.height - 100, disp.width - 22), fill = uiBorderColor, width = 1)
    draw.line((disp.height - 100, disp.width - 22, disp.height - 2, disp.width - 22), fill = uiBorderColor, width = 1) # bottom right
    draw.line((disp.height - 2, disp.width - 22, disp.height - 2, 12), fill = uiBorderColor, width = 1) # right side
    draw.line((disp.height - 2, 12, disp.height - 12, 2), fill = uiBorderColor, width = 1) # top right diagnal
    draw.line((disp.height - 12, 2, disp.height - 95, 2), fill = uiBorderColor, width = 1) # top
    draw.line((disp.height - 95, 2, disp.height - 110, 17), fill = uiBorderColor, width = 1) # top
    draw.line((disp.height - 110, 17, 110, 17), fill = uiBorderColor, width = 1) # top
    draw.line((110, 17, 95, 2), fill = uiBorderColor, width = 1) # top
    draw.line((95, 2, 2, 2), fill = uiBorderColor, width = 1) # top left diagnal
    draw.polygon([(99,2), (disp.height - 99, 2), (disp.height - 111, 14), (111, 14), (99, 2)], fill = uiAccentColor, outline = "RED")
    draw.polygon([(disp.height -117, disp.width - 2), (disp.height - 2, disp.width - 2), (disp.height - 2, disp.width - 20), (disp.height - 99, disp.width - 20)], fill = uiAccentColor, outline = "RED")
    draw.text((128, 4), u"ARACHNOTRON", fill = "YELLOW", font=fontMICR_10pt)
    draw.text((disp.height - 81, disp.width- 18), u"ON-LINE", fill = "BLACK", font=fontMICR_16pt)    
    draw.text((disp.height - 82, disp.width- 19), u"ON-LINE", fill = "GREEN", font=fontMICR_16pt)
    #image1=ge1.rotate(180)

    # add the telemetry data to the screen
    if data is not None:

        for dLine in data:

            if (len(dLine)):
                #print("Data segment: " + dLine)
                # split out the segment header
                header = dLine.split(':')[0]
                #print("Header: " + header)
                # split out the segment data elements
                dataElements = dLine.split(':')[1].split(',')

                #process the segment s1
                if (header == "S1"):

                    # set the current gait
                    gait = int(dataElements[6])
                    currentGait = getGait(gait)
                    lastGait = getGait(int(dataElements[4]))
                    newGait = getGait(int(dataElements[5]))
                    draw.text((7, disp.width - 19), u"  LOOP          Hz", fill = uiBorderColor, font=fontMICR_14pt)
                    draw.text((45, disp.width - 19), u"%.1f" % (1000000/ float(dataElements[0])), fill = "YELLOW", font=fontMICR_14pt)
                    draw.text((4, 4), "CURRENT GAIT", fill = uiBorderColor, font=fontMICR_14pt)
                    draw.text((14, 18), currentGait, fill = "YELLOW", font=fontMICR_14pt)
                    draw.text((disp.height - 11, 4), "NEXT GAIT", fill = uiBorderColor, font=fontMICR_14pt, anchor="rt")
                    draw.text((disp.height - 21, 18), newGait, fill = "YELLOW", font=fontMICR_14pt, anchor="rt")
                    phase = float(dataElements[7])
                    draw.rectangle((110, 19, 212, 31), fill = "BLACK", outline = "BLACK")
                    for i in range(0, int(phase * 100)):
                        p = i / 100
                        antiPhase = 1 - p
                        color = (int( antiPhase * uiPhaseStartColor[0] + p * uiBorderColor[0]),
                                int( antiPhase * uiPhaseStartColor[1] + p * uiBorderColor[1]),
                                int( antiPhase * uiPhaseStartColor[2] + p * uiBorderColor[2]))
                        draw.line((111+i, 20, 111+i, 30), fill = color, width = 1)
                    
                    # show if the motors are enabled
                    if (int(dataElements[3]) == 1):
                        color = "GREEN"
                    else:
                        color = "RED"
                    draw.rectangle((124, 50, 134, 60), fill = color, outline = uiBorderColor)
                    draw.text((138, 51), "MOTOR ENBL", fill = uiBorderColor, font=fontMICR_10pt)

                    #show if the motors are in float mode
                    if (int(dataElements[9]) == 1):
                        color = "GREEN"
                    else:
                        color = "RED"
                    draw.rectangle((124, 65, 134, 75), fill = color, outline = uiBorderColor)
                    draw.text((138, 66), "MOTOR FLOAT", fill = uiBorderColor, font=fontMICR_10pt)
                elif (header == "S2"):

                    # draw a box for each servo
                    for i in range(0, 18):

                        # draw a box for each servo
                        num = 0
                        offset = 0
                        color = "BLACK"
                        if i < 9:
                            x = 30 + ((i % 3 ) * 30)
                            y = 70 + (int(i / 3) * 30)
                            offset = (3 * int(i/3)) + (3 - i%3) - 1
                            if (int(i/3) == 0):
                                offset *= -5                                                                                                                      
                            elif (int(i/3) == 1):
                                offset = 0
                            elif (int(i/3) == 2):
                                offset = offset * 5 - 30
                            num = str((3 * int(i/3)) + (3 - i%3) - 1)
                        else:
                            x = 205 + ((i % 3 ) * 30)
                            y = 70 + (int((i-9) / 3) * 30)
                            offset = i % 3
                            if (int(i/3) == 3):
                                offset *= -5                                                                                                                      
                            elif (int(i/3) == 4):
                                offset = 0
                            elif (int(i/3) == 5):
                                offset = offset * 5
                            num = str(i)
                        if (int(dataElements[i]) == 0):
                            color = uiAccentColor
                        else:
                            color = uiBorderColor
                        draw.rectangle((x, y + offset, x + 26, y - 20 + offset), fill = "BLACK", outline = color)

                        # draw the servo number
                        if ( num.__len__() == 2):
                            x -= 0
                        else:
                            x += 1
                        draw.text((x, y - 28 + offset), num, fill = color, font=fontMICR_6pt)
                        # draw the servo value
                        #draw.text((x + 1, y - 5), str(dataElements[i]), fill = uiBorderColor, font=fontMICR_14pt)
                elif (header == "S3"):                                                                                                      
                    # show the voltage in each servo
                    for i in range(0, 18):
                        # draw a box for each servo
                        num = 0
                        offset = 0
                        color = "BLACK"
                        if i < 9:
                            x = 30 + ((i % 3 ) * 30)
                            y = 70 + (int(i / 3) * 30)
                            offset = (3 * int(i/3)) + (3 - i%3) - 1
                            if (int(i/3) == 0):
                                offset *= -5                                                                                                                      
                            elif (int(i/3) == 1):
                                offset = 0
                            elif (int(i/3) == 2):
                                offset = offset * 5 - 30
                            num = float(dataElements[i])/1000
                        else:
                            x = 205 + ((i % 3 ) * 30)
                            y = 70 + (int((i-9) / 3) * 30)
                            offset = i % 3
                            if (int(i/3) == 3):
                                offset *= -5                                                                                                                      
                            elif (int(i/3) == 4):
                                offset = 0
                            elif (int(i/3) == 5):
                                offset = offset * 5
                            num = float(dataElements[i])/1000
                        if (num < 10.0):
                            color = "RED"
                        else:
                            color = uiBorderColor

                        tmpStr = str(num)[0:4]
                        draw.text((x + 2, y - 17 + offset), tmpStr + "v", fill = color, font=fontMICR_6pt)
                        tempStr = dataElements[i+18] + " C"
                        if (float(dataElements[i+18]) > 40.0):
                            color = "RED"
                        else:
                            color = uiBorderColor
                        draw.text((x + 2, y - 9 + offset), tempStr, fill = color, font=fontMICR_6pt)
                        # draw the servo value
                        #draw.text((x + 1, y - 5), str(dataElements[i]), fill = uiBorderColor, font=fontMICR_14pt)

                    #draw.text((20, Top+2*lineHeight), u"  button_x = qq%s   button_y = %s   button_a = %s   button_b = %s" % (data[6], data[7], data[8], data[9]), fill = Color, font=fontMICR)
        #draw.text((20, Top+3*lineHeight), u"  joystick_right_button = %s   joystick_left_button = %s   button_menu = %s" % (data[10], data[11], data[12]), fill = Color, font=fontMICR)
        #draw.text((20, Top+4*lineHeight), u"  joystick_left_x = %.3f   joystick_left_y = %.3f   joystick_right_x = %.3f   joystick_right_y = %.3f" % (data[13], data[14], data[15], data[16]), fill = Color, font=fontMICR)
    endTime = time.time()
    return image1, gait

if __name__ == "__main__":

    async def main():
        #try:
        lastTIme = time.time()
        _useGamePad = False
        # display the image
        #image1, gait = draw_eyes()
        eyes.update()
        image1 = eyes.display_image
        image2 = np.array(image1)

        height, width = image2.shape[:2]
        image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv.INTER_CUBIC)
        cv.imshow("arachnotron", image2)
        cv.waitKey(1)                                                                                                                       
        disp.ShowImage(image1)
        time.sleep(1)
        eyes.eye_color = (10,120,255) #(255,80,20) #
        eyes.left_shape = EYE_SHAPE.ROUNDRECTANGLE
        eyes.right_shape = EYE_SHAPE.ROUNDRECTANGLE
        eyes.eye_size = (25,45)
        eyes.rotation = -10
        eyes.eye_spacing_offset = 10
        eyes.eyelid_angle = 0
        eyes.blink_percent_step = 0.3
        while True:
            # #read the telemetry from the teensy and process accordingly
            # if (port.in_waiting > 0):

            try:
                #@ update the eyes
                eyes.update()

                if (eyes.render_flag):
                    image1 = eyes.display_image
                    d = ImageDraw.Draw(image1)
                    d.rectangle((0, image1.height - 18 , image1.width, image1.height), fill=(0, 49, 45))
                    ctrlImage = Image.open("./controller 2.png")
                    image1.paste(ctrlImage, (1, image1.height - 17), ctrlImage)
                    if not _useGamePad:
                        d.ellipse((1, image1.height - 16, 16, image1.height -1), outline=(255, 0, 0))
                        d.line((4, image1.height - 4, 14, image1.height - 14), fill=(255, 0, 0), width=2)

                    image2 = np.array(image1)                        
                    height, width = image2.shape[:2]
                    image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv.INTER_LANCZOS4)
                    cv.imshow("arachnotron", image2)
                    cv.waitKey(1) 
                    disp.ShowImage(image1) 
            except:
                print("Error in telemetry read")
                print( sys.exc_info())

            #send the speedevery 2 seconds
            oldUseGamePad = _useGamePad
            #_useGamePad, xbox_path = testForGamePad()
            if _useGamePad != oldUseGamePad:
                _exit = False
                break
            if _useGamePad:
                #print("using Gameapad")
                currTime = time.time()
                if (currTime - lastTIme) > .1:
                    lastTIme = currTime
                    output = "z" + str(remote_control.trigger_right) + ";"
                    output  += "x" + str(math.atan2(remote_control.joystick_right_x, remote_control.joystick_right_y)) + ";"
                    #print(math.atan2(remote_control.joystick_right_x, remote_control.joystick_right_y) * 180/math.pi )
                    port.write(output.encode())
                if remote_control.button_y: # turn on light rumble effect
                    remote_control.button_y = False
                    port.write(b'r')
                    port.write(b';')
                if remote_control.button_b: # play once strong rumble effect
                    remote_control.button_b = False
                    port.write(b'f')
                    port.write(b';')
                if remote_control.button_a: # play once strong rumble effect
                    remote_control.button_a = False
                    port.write(b's')
                    port.write(b';')
                if remote_control.joystick_right_button: # play once strong rumble effect
                    remote_control.joystick_right_button = False
                    port.write(b'q')
                    port.write(b';')
                if remote_control.button_menu: # toggle telemetry
                    remote_control.button_menu = False
                    port.write(b'T')
                    port.write(b';')
                if remote_control.joystick_left_button: #rotate gait
                    remote_control.joystick_left_button = False
                    if ( gait == 0):  # tri to ripple
                        port.write(b'r')
                        port.write(b';')
                    if (gait == 1):  # ripple to wave gait
                        port.write(b'w')
                        port.write(b';')
                    if (gait == 2):  # wave to tri gait 
                        port.write(b't')
                        port.write(b';')
                if remote_control.button_left: # stop the script
                    remote_control.button_left = False
                    remote_control.power_on = False
                    remote_control.erase_rumble()
                    _exit = True
                    break

            # read the key input
            key = cv.waitKeyEx(2)
            if ( key != -1):
                print(key)
                if (key == 27): # esc key
                
                    # show dead eyes
                    eyes.eye_color = "RED"
                    eyes.left_shape = EYE_SHAPE.X
                    eyes.right_shape = EYE_SHAPE.X
                    eyes.render_eye()
                    eyes.update(force_update=True)

                    image1 = eyes.display_image
                    image2 = np.array(image1)                        
                    height, width = image2.shape[:2]
                    image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv.INTER_LANCZOS4)
                    cv.imshow("arachnotron", image2)
                    cv.waitKey(1) 
                    disp.ShowImage(image1) 
                    for i in range(0, 10):
                        time.sleep(.1)
                    _exit = True
                    break
                elif (key == 65620): # T key
                    print( "Toggle telemetry")
                    port.write(b'T')
                    port.write(b';')
                elif (key == 113): # q key
                    print( "Toggle motors")
                    port.write(b'q')
                    port.write(b';')
                elif (key == 102): # f key
                    print( "Float motors")
                    port.write(b'f')
                    port.write(b';')
                elif (key == 119): # w key
                    if ( gait == 0):  # tri to ripple
                        port.write(b'r')
                        port.write(b';')
                    if (gait == 1):  # ripple to wave gait
                        port.write(b'w')
                        port.write(b';')
                    if (gait == 2):  # wave to tri gait 
                        port.write(b't')
                        port.write(b';')
                elif (key == 109): # m key
                    port.write(b't')
                    port.write(b';')   

                
            time.sleep(0.01)
            await asyncio.sleep(0)
        #except:
        #    print("Error in main loop")
        #    print(sys.exc_info())
        #finally:
        print("Cleaning up...")
        print("Exiting main loop")
        return
    
    # Initialize the display
        # Raspberry Pi pin configuration:
    global _useGamePad
    global _exit
    _useGamePad = False
    _exit = False
    RST = 27
    DC = 25
    BL = 18
    bus = 0 
    device = 0 
    global disp
    disp = LCD_1inch9.LCD_1inch9()
    # Initialize library.
    disp.Init()
    # Clear display.
    disp.clear()
    #Set the backlight to 100
    disp.bl_DutyCycle(100)
    global image1
    global image2
    image1 = Image.new("RGB", (disp.width,disp.height ), "BLACK")
    d = ImageDraw.Draw(image1)
    for i in range (0,169):
        for j in range (0,319):
            d.point((i,j), fill=(int(i*1.5),int(j*.8),255))
    disp.ShowImage(image1)
    image2 = np.array(image1)                        
    height, width = image2.shape[:2]
    image2 = cv.resize(cv.cvtColor(image2, cv.COLOR_RGB2BGR), (height*2, width*2), interpolation=cv.INTER_LANCZOS4)
    cv.imshow("arachnotron", image2)

    for i in range(0, 100):
        cv.waitKey(1)
    #time.sleep(15)

    eyes = SimpleEyes((image1.height, image1.width), eye_color=(10,120,255))
    eyes.set_eye_size((25,55))
    eyes.rotation = 20
    eyes.update()
    image1 = eyes.display_image
    #image1, gait = draw_eyes()
    disp.ShowImage(image1)
    image2 = np.full((disp.width * 2, disp.height * 2, 3), 255, dtype=np.uint8)
    cv.imshow("arachnotron", image2)
    time.sleep(1)

    #image1 = Image.new("RGB", (disp.height,disp.width ), "WHITE")
    #draw = ImageDraw.Draw(image1)

    # initialize the display
    fontMICR = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf", 6)
    Color = "BLACK"
    Top = 10
    lineHeight = 8
    # draw.text((20, Top), u"  ____  ____    ____     __  __ __  ____    ___   ______  ____    ___   ____  \n", fill = Color, font=fontMICR)
    # draw.text((20, Top + lineHeight), u" /    ||    \\  /    |   /  ]|  |  ||    \\  /   \\ |      ||    \\  /   \\ |    \\ \n", fill = Color, font=fontMICR)
    # draw.text((20, Top + 2*lineHeight), u"|  o  ||  D  )|  o  |  /  / |  |  ||  _  ||     ||      ||  D  )|     ||  _  |\n", fill = Color, font=fontMICR)
    # draw.text((20, Top + 3*lineHeight), u"|     ||    / |     | /  /  |  _  ||  |  ||  O  ||_|  |_||    / |  O  ||  |  |\n", fill = Color, font=fontMICR)
    # draw.text((20, Top + 4*lineHeight), u"|  _  ||    \\ |  _  |/   \\_ |  |  ||  |  ||     |  |  |  |    \\ |     ||  |  |\n", fill = Color, font=fontMICR)
    # draw.text((20, Top + 5*lineHeight), u"|  |  ||  .  \\|  |  |\\     ||  |  ||  |  ||     |  |  |  |  .  \\|     ||  |  |\n", fill = Color, font=fontMICR)
    # draw.text((20, Top + 6*lineHeight), u"|__|__||__|\\_||__|__| \\____||__|__||__|__| \\___/   |__|  |__|\\_| \\___/ |__|__|\n", fill = Color, font=fontMICR)
    # #image1=image1.rotate(180)

# declare the remote controller for the robot (xbox gamepad)
##remote_control = gamepad()
#_useGamePad = True

# connect to the teensy for low level control of the robot
teensy_port = find_teensy()
if teensy_port:
    # Open the serial connection to the Teensy
    try:
        port = serial.Serial(teensy_port, baudrate=6000000, timeout=0.25)
        print("Connected to Teensy!")
    except serial.SerialException as e:
        print(f"Failed to connect to Teensy: {e}.  Exiting ...")
        exit()

    #reset the teensy and wait for the cli prompt
    port.write(b'M;')

    stream = port.read_until(b'arachnotron>',99999)
    print(stream)
    time.sleep(0.1)
    #port.write(b'q;')
    time.sleep(0.25)
    port.write(b'0;')
    time.sleep(0.25)
    #port.write(b'T;')
    loop = asyncio.get_event_loop()

    while not _exit:
        _useGamePad, xbox_path = testForGamePad()
        if _useGamePad:
            remote_control = gamepad(xbox_path)
            futures = [
                loop.create_task(remote_control.read_gamepad_input()),
                loop.create_task(remote_control.rumble()),
                loop.create_task(main())
            ]
        else:
            remote_control = None
            futures = [
                loop.create_task(main())
            ]
        loop.run_until_complete(asyncio.wait(futures))
        print(_exit)
    loop.close()
    port.close()

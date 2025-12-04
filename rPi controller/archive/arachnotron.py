#!/usr/bin/python
# -*- coding: UTF-8 -*-
#import chardet
import asyncio
import math
import os
import signal
import subprocess
import sys
import threading
import time
import sys 
import time
import logging
import spidev as SPI
import serial
sys.path.append("..")
from lib import LCD_1inch9
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
import RPi.GPIO as GPIO
from evdev import InputDevice, ecodes, ff, list_devices
import gamepad
from soundplayer import SoundPlayer
import led

def connect(): # asyncronus read-out of events
        xbox_path = None
        remote_control = None
        devices = [InputDevice(path) for path in list_devices()]
        logging.info('Connecting to xbox controller...')
        for device in devices:
            print(device.path, device.name)
            if str.lower(device.name) == 'xbox wireless controller':
                xbox_path = str(device.path)
                remote_control = gamepad.gamepad(file = xbox_path)
                remote_control.rumble_effect = 2
                return remote_control
        return None

def is_connected(): # asyncronus read-out of events
    path = None
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        if str.lower(device.name) == 'xbox wireless controller':
            path = str(device.path)
    if(path == None):
        print('Xbox controller disconnected!!')
        return False
    return True


async def read_gamepad_inputs():
    global head_light_flag
    #logging.info("Ready to drive!!")
    #turn_sound = SoundPlayer("/home/pi/xbox-raspberrypi-rover/soundfiles/turn-signal.mp3", card)
    #horn_sound = SoundPlayer("/home/pi/xbox-raspberrypi-rover/soundfiles/Horn.mp3", card)        

    if is_connected():
        remote_control.read_gamepad_input()
        remote_control.rumble() 
        print(" trigger_right = ", round(remote_control.trigger_right,2))
        x = round(remote_control.joystick_left_x,2)
        y = round(remote_control.joystick_left_y,2)
        angle = get_angle_from_coords(x,y)
        if angle > 180:
            angle = 360 - angle
        print("x:", x, " y:", y, " angle: ",angle)
        #turn_head(angle)
        #direction = get_motor_direction(x,y)
        #print("x:", x, " y:", y, " direction: ",direction,end="\r")
        #drive_motor(direction,y)

        if round(remote_control.trigger_right,2) > 0.0:
            #horn_sound.play(1.0)
            led.blue(disp)
        elif round(remote_control.trigger_left,2) > 0.0:
            led.cyan(disp)
        elif remote_control.bump_left:
            #turn_sound.play(1.0)
            led.turn_left(5,disp)
        elif remote_control.bump_right:
            #turn_sound.play(1.0)
            led.turn_right(5,disp)
        elif remote_control.dpad_up:
            remote_control.dpad_up = False
        elif remote_control.dpad_left:
            remote_control.dpad_left = False
        elif remote_control.dpad_right:
            remote_control.dpad_right = False
        elif remote_control.button_a:
            remote_control.button_a = False
        # elif head_light_flag == False:
        #     led.both_off()
        #     #led_strip.colorWipe(strip, Color(0,0,0))
        #     if turn_sound.isPlaying():
        #         turn_sound.stop()

        #await asyncio.sleep(100e-3) #100ms
    return

def get_angle_from_coords(x,y):
    angle = 0.0
    if x==0.0 and y==0.0:
        angle = 90.0
    elif x>=0.0 and y>=0.0:
        # first quadrant
        angle = math.degrees(math.atan(y/x)) if x!=0.0 else 90.0
    elif x<0.0 and y>=0.0:
        # second quadrant
        angle = math.degrees(math.atan(y/x))
        angle += 180.0
    elif x<0.0 and y<0.0:
        # third quadrant
        angle = math.degrees(math.atan(y/x))
        angle += 180.0
    elif x>=0.0 and y<0.0:
        # third quadrant
        angle = math.degrees(math.atan(y/x)) if x!=0.0 else -90.0
        angle += 360.0
    return angle

async def removetasks(loop):
    tasks = [t for t in asyncio.all_tasks() if t is not
             asyncio.current_task()]

    for task in tasks:
        # skipping over shielded coro still does not help
        if task._coro.__name__ == "cant_stop_me":
            continue
        task.cancel()

    print("Cancelling outstanding tasks")
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

async def shutdown_signal(signal, loop):
    print(f"Received exit signal {signal.name}...")
    await removetasks(loop)

# Main code body

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    card = 1 #(default)
    strip = None
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
  
    logging.basicConfig(level = logging.DEBUG)
    logging.info("connecting to xbox controller")
    remote_control = connect()
    if(remote_control == None):
        logging.info('Please connect an Xbox controller then restart the program!')
        sys.exit()  
    logging.info('Xbox controller connected!')

    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(True)
    GPIO.setup(27, GPIO.OUT)
    GPIO.output(27, GPIO.HIGH)
    GPIO.cleanup(27)
    # check if the display is connected
    print( cv2.__version__)

    # Raspberry Pi pin configuration:
    RST = 27
    DC = 25
    BL = 18
    bus = 0 
    device = 0 


    # Create blank image for drawing.
    #disp = LCD_1inch9.LCD_1inch9(spi=SPI.SpiDev(bus, device),spi_freq=10000000,rst=RST,dc=DC,bl=BL)
    disp = LCD_1inch9.LCD_1inch9()
    # Initialize library.
    disp.Init()
    # Clear display.
    disp.clear()
    #Set the backlight to 100
    disp.bl_DutyCycle(100)
    image1 = Image.new("RGB", (disp.width,disp.height ), "WHITE")
    image2 = np.full((disp.width * 2, disp.height * 2, 3), 255, dtype=np.uint8)
    draw = ImageDraw.Draw(image1)
    disp.ShowImage(image1)
    cv2.imshow("arachnotron", image2)
    led.setup(disp)
    time.sleep(1)

    for s in signals:
        loop.add_signal_handler(
            s, lambda s=s: asyncio.create_task(shutdown_signal(s, loop)))
    try:

        # display with hardware SPI:
        ''' Warning!!!Don't  creation of multiple displayer objects!!! '''
        #disp = LCD_1inch9.LCD_1inch9(spi=SPI.SpiDev(bus, device),spi_freq=10000000,rst=RST,dc=DC,bl=BL)
        #disp = LCD_1inch9.LCD_1inch9()
        # Initialize library.
        #disp.Init()
        # Clear display.
        #disp.clear()
        #Set the backlight to 100
        #disp.bl_DutyCycle(50)

        Font1 = ImageFont.truetype("../Font/Font01.ttf", 8)
        Font2 = ImageFont.truetype("../Font/Font01.ttf", 35)
        Font3 = ImageFont.truetype("../Font/Font02.ttf", 32)
        Font4 = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf", 6)

        # Create blank image for drawing.
        image1 = Image.new("RGB", (disp.width,disp.height ), "WHITE")
        draw = ImageDraw.Draw(image1)

        # logging.info("draw point")
        # draw.rectangle((5, 10, 6, 11), fill = "BLACK")
        # draw.rectangle((5, 25, 7, 27), fill = "BLACK")
        # draw.rectangle((5, 40, 8, 43), fill = "BLACK")
        # draw.rectangle((5, 55, 9, 59), fill = "BLACK")
        # #time.sleep(5)

        # logging.info("draw rectangle")
        # draw.rectangle([(20, 10), (70, 60)], fill = "WHITE", outline="BLUE")
        # draw.rectangle([(85, 10), (130, 60)], fill = "BLUE")
        # #time.sleep(5)
        # logging.info("draw line")
        # draw.line([(20, 10), (70, 60)], fill = "RED", width = 1)
        # draw.line([(70, 10), (20, 60)], fill = "RED", width = 1)
        # draw.line([(110, 65), (110, 115)], fill = "RED", width = 1)
        # draw.line([(85, 90), (135, 90)], fill = "RED", width = 1)
        # #time.sleep(5)
        # logging.info("draw circle")
        # draw.arc((85, 65, 135, 115), 0, 360, fill =(0, 255, 0))
        # draw.ellipse((20, 65, 70, 115), fill = (0, 255, 0))
        # #time.sleep(5)
        # logging.info("draw text")
        # draw.rectangle([(0, 120), (140, 153)], fill = "BLUE")
        # draw.text((5, 120), 'Hello world', fill = "RED", font=Font1)
        # draw.rectangle([(0,155), (172, 195)], fill = "RED")
        # draw.text((1, 155), 'WaveShare', fill = "WHITE", font=Font2)
        # draw.text((5, 190), '1234567890', fill = "GREEN", font=Font3)
        # text= u"微雪电子"
        # draw.text((5, 230),text, fill = "BLUE", font=Font3)
        # image1=image1.rotate(180)
        # disp.ShowImage(image1)
        # #time.sleep(5)
        
        image1 = Image.new("RGB", (disp.height,disp.width ), "WHITE")
        draw = ImageDraw.Draw(image1)

        Color = "BLACK"
        Top = 10
        lineHeight = 8
        draw.text((20, Top), u"  ____  ____    ____     __  __ __  ____    ___   ______  ____    ___   ____  \n", fill = Color, font=Font4)
        draw.text((20, Top + lineHeight), u" /    ||    \\  /    |   /  ]|  |  ||    \\  /   \\ |      ||    \\  /   \\ |    \\ \n", fill = Color, font=Font4)
        draw.text((20, Top + 2*lineHeight), u"|  o  ||  D  )|  o  |  /  / |  |  ||  _  ||     ||      ||  D  )|     ||  _  |\n", fill = Color, font=Font4)
        draw.text((20, Top + 3*lineHeight), u"|     ||    / |     | /  /  |  _  ||  |  ||  O  ||_|  |_||    / |  O  ||  |  |\n", fill = Color, font=Font4)
        draw.text((20, Top + 4*lineHeight), u"|  _  ||    \\ |  _  |/   \\_ |  |  ||  |  ||     |  |  |  |    \\ |     ||  |  |\n", fill = Color, font=Font4)
        draw.text((20, Top + 5*lineHeight), u"|  |  ||  .  \\|  |  |\\     ||  |  ||  |  ||     |  |  |  |  .  \\|     ||  |  |\n", fill = Color, font=Font4)
        draw.text((20, Top + 6*lineHeight), u"|__|__||__|\\_||__|__| \\____||__|__||__|__| \\___/   |__|  |__|\\_| \\___/ |__|__|\n", fill = Color, font=Font4)
        image2 = np.array(image1)
        image1=image1.rotate(180)


        print( image2.shape[:2])
        height, width = image2.shape[:2]
        image2 = cv2.resize(cv2.cvtColor(image2, cv2.COLOR_RGB2BGR), (width*2, height*2), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("arachnotron", image2)
        disp.ShowImage(image1)
        time.sleep(1
                )

        # connect to the teensy board via tha usb serial
        try:
            portStr = "/dev/ttyACM0"
            port = serial.Serial(portStr, 1000000, timeout=1)
            if not port.isOpen():
                portStr = "/dev/ttyACM1"
                port = serial.Serial(portStr, 1000000, timeout=1)
                if not port.isOpen():
                    logging.info("No Teensy board found")
                    exit()
        except serial.SerialException as e:
            portStr = "/dev/ttyACM1"
            port = serial.Serial(portStr, 1000000, timeout=1)
            if not port.isOpen():
                logging.info("No Teensy board found")
                exit()


        
        #reset the teensy and wait for the cli prompt
        port.write(b'M')
        #time.sleep(5)
        #port.open()
        time.sleep(1)
        port.write(b'0')

        #read all the data from in the buffer and look for "arachnotron>"
        #timeout if we don't get a response in 5 seconds
        stream = port.read_until(b'arachnotron>',99999)
        #print the data we read
        print(stream)
        time.sleep(1)
        #port.write(b'T')
        # loop and grab the next telemetry line
        # and print it to the screen

        
        while True: 
            # read the next line from the teensy
            stream = port.readline(-1).replace(b'\r\n', b'')

            # read the xbox controller
            #read_gamepad_inputs()

            #print the data we read
            print(stream)
            #time.sleep(0.1)
            #if we get a line that starts with "ara" then we have a telemetry line
            key = cv2.waitKey(1)
            if key == ord('='):
                break
            else:
                port.write(bytes([key & 0xFF]))
                #

        # logging.info("show image")
        # ImagePath = ["../pic/LCD_1inch9_1.jpg", "../pic/LCD_1inch9_2.jpg", "../pic/LCD_1inch9_3.jpg"]
        # for i in range(0, 3):
        #     image = Image.open(ImagePath[i])	
        #     image = image.rotate(180)
        #     disp.ShowImage(image)
        #     time.sleep(5)

        # clear the display and kill the connecttion
        disp.bl_DutyCycle(1)
        image1 = Image.new("RGB", (disp.width,disp.height ), "BLACK")
        disp.ShowImage(image1)
        disp.module_exit()
        cv2.destroyAllWindows() 
        # close the serial port
        port.close()


        logging.info("quit:")
        
    except IOError as e:
        logging.info(e)    
        
    except KeyboardInterrupt:
        disp.module_exit()
        logging.info("quit:")
        exit()
import os
import sys 
import time
import logging
import spidev as SPI
import serial
sys.path.append("..")
from lib import LCD_1inch9
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import cv2

disp = LCD_1inch9.LCD_1inch9()
                # Initialize library.
disp.Init()
                # Clear display.
disp.clear()
                #Set the backlight to 100
disp.bl_DutyCycle(50)
cap = cv2.VideoCapture('/home/starter/Downloads/XNXX_lesbian_stepdaughter_home_lessons_jelena_jensen_casey_calvert_360p.mp4')
while(cap.isOpened()):
  ret, frame = cap.read()
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  cv2.imshow('frame',frame)
  img = Image.fromarray(cv2.cvtColor(cv2.resize(frame, (disp.height, disp.width)), cv2.COLOR_BGR2RGB)).rotate(-180)
  disp.ShowImage(img)
  #time.sleep(1.0/45)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

disp.bl_DutyCycle(0)
disp.clear()
time.sleep(1)
disp.module_exit()
cap.release()
cv2.destroyAllWindows()
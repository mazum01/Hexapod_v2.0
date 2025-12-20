"""
cst816d.py — Touch controller driver for CST816D (Pi 5 compatible).

Uses gpiozero instead of RPi.GPIO for Pi 5 RP1 chip compatibility.
"""

import time
import smbus
from gpiozero import Button, DigitalOutputDevice


CST816D_ADDRESS = 0x15
CST816D_LCD_TOUCH_MAX_POINTS = 2

TP_INT = 4
TP_RST = 17


class cst816d():
    def __init__(self):
        # Initialize I2C
        self.I2C = smbus.SMBus(1)
        
        # Use gpiozero for Pi 5 compatibility
        self.GPIO_TP_INT = Button(TP_INT, pull_up=True)
        self.GPIO_TP_RST = DigitalOutputDevice(TP_RST, active_high=True, initial_value=True)
        
        self.coordinates = [{"x": 0, "y": 0} for _ in range(CST816D_LCD_TOUCH_MAX_POINTS)]
        self.point_count = 0
        self.touch_rst()
    
    def Int_Callback(self):
        self.read_touch_data()
    
    # Reset
    def touch_rst(self):
        self.GPIO_TP_RST.off()  # Low
        time.sleep(0.001)       # 1 ms
        self.GPIO_TP_RST.on()   # High
        time.sleep(0.050)       # 50 ms
        
    def write_cmd(self, data):
        self.I2C.write_byte(CST816D_ADDRESS, data)

    def read_bytes(self, reg_addr, length):
        # Send register address and read multiple bytes
        data = self.I2C.read_i2c_block_data(CST816D_ADDRESS, reg_addr, length)
        return data
    
    def read_touch_data(self):
        TOUCH_NUM_REG = 0x02
        TOUCH_XY_REG = 0x03
        
        buf = self.read_bytes(TOUCH_NUM_REG, 1)
        
        if buf and buf[0] != 0:
            self.point_count = buf[0]
            buf = self.read_bytes(TOUCH_XY_REG, 6 * self.point_count)
            for i in range(2):
                self.coordinates[i]["x"] = 0
                self.coordinates[i]["y"] = 0
            
            if buf:
                for i in range(self.point_count):
                    self.coordinates[i]["x"] = ((buf[(i * 6) + 0] & 0x0f) << 8) + buf[(i * 6) + 1]
                    self.coordinates[i]["y"] = ((buf[(i * 6) + 2] & 0x0f) << 8) + buf[(i * 6) + 3]
    
    def get_touch_xy(self):
        point = self.point_count
        # Reset touch point count to 0
        self.point_count = 0

        if point != 0:
            # Return touch status and coordinates
            return point, self.coordinates
        else:
            # Return 0 and empty coordinate list
            return 0, []
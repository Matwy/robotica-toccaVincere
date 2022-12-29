"""
from smbus2 import SMBus
from time import sleep
addr = 0x10
bus = SMBus(1)
"""
import busio
from board import SCL, SDA
from adafruit_bus_device import i2c_device

class Motors():

    M_LEFT = 0
    M_RIGHT = 1
    
    def __init__(self):
        i2c_bus = busio.I2C(SCL, SDA)
        self.arduinoi2c = i2c_device.I2CDevice(i2c_bus, 0x10)
    
    def send_motor_power(self, motor, power):
        if (power < 0):
            power = 256 - abs(power)
        data = [motor, power]
        try:
            self.arduinoi2c.write(bytes(data))
        except:
            print("i2c nel drifting")
        
    def motors(self, m1_power, m2_power):
        self.send_motor_power(self.M_LEFT, m1_power)
        self.send_motor_power(self.M_RIGHT, m2_power)
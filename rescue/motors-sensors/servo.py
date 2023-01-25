from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

class Servo:
    FREQUENCY_HZ = 50
    
    def __init__(self):
        i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c_bus)
        try:
            self.pca.frequency = self.FREQUENCY_HZ
        except:
            print("diomerdolo")
        self.cam = servo.Servo(self.pca.channels[4])
        self.braccio_sx = servo.Servo(self.pca.channels[1])
        self.braccio_dx = servo.Servo(self.pca.channels[2])
    
    def deinit_pca(self):
        self.pca.reset()
        self.pca.deinit()
        
    """
    CAM
    """
    def set_cam_angle(self, angle):
        self.cam.angle = angle

    def cam_linea(self):
        self.set_cam_angle(160)
        
    def cam_EZ(self):
        self.set_cam_angle(120)
    
    """
    PINZA
    """
    def set_pinza_angle(self, angle):
        # angle 0  pinza bassa
        # angle 180  pinza alta

        self.braccio_dx.angle = angle
        self.braccio_sx.angle = 180-angle

    def pinza_su(self):
        self.set_pinza_angle(180)

    def pinza_giu(self):
        self.set_pinza_angle(0)
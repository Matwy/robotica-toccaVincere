from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

class Servo:
    FREQUENCY_HZ = 50
    current_cam_angle = 0

    def __init__(self):
        i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = self.FREQUENCY_HZ
        self.braccio_sx = servo.Servo(self.pca.channels[1])
        self.braccio_dx = servo.Servo(self.pca.channels[2])
        self.becco_sx = servo.Servo(self.pca.channels[10])
        self.becco_dx = servo.Servo(self.pca.channels[8])
        self.cam = servo.Servo(self.pca.channels[4])
        self.morti = servo.Servo(self.pca.channels[7], max_pulse=2320)
        self.vivi = servo.Servo(self.pca.channels[6], min_pulse=770)
    
    def deinit_pca(self):
        self.pca.reset()
        self.pca.deinit()
        
    """
    CAM
    """
    def set_cam_angle(self, angle):
        self.cam.angle = angle
        self.current_cam_angle = angle

    def cam_su(self):
        try:
            self.cam.angle = 150
        except:
            print("dio sporco negro")
    
    def cam_linea(self):
        self.set_cam_angle(160)
    
    def cam_cubo(self):
        self.set_cam_angle(130)
    
     
    def cam_EZ(self):
        self.set_cam_angle(110)
    
    """
    PINZA
    """
    def set_pinza_angle(self, angle):
        # angle 0  pinza bassa
        # angle 180  pinza alta

        self.braccio_dx.angle = angle
        self.braccio_sx.angle = 180-angle

    def pinza_su(self):
        self.set_pinza_angle(153)
    
    def pinza_su_max(self):
        self.set_pinza_angle(158)
        
    def pinza_giu(self):
        self.set_pinza_angle(0)

    def pinza_salita(self):
        self.set_pinza_angle(30)

    def pinza_svuota_cassoni(self):
        self.set_pinza_angle(150)
        
    """
    BOCCHI
    """
    def becco_aperto(self):
        self.becco_sx.angle = 115
        self.becco_dx.angle = 0
        
    def becco_chiuso(self):
        self.becco_sx.angle = 60
        self.becco_dx.angle = 60
    
    def becco_molla_morti(self):
        self.becco_dx.angle = 0
        for i in range(5):
            self.becco_sx.angle = 20
            time.sleep(0.2)
            self.becco_sx.angle = 70
            time.sleep(0.2)
    
    def becco_molla_vivi(self):
        """
        self.morti.angle = 155
        self.becco_sx.angle = 115
        
        self.vivi.angle = 5
        time.sleep(0.3)
        self.morti.angle = 180
        """
        self.becco_sx.angle = 115
        time.sleep(0.3)
        for i in range(3):
            self.becco_dx.angle = 60
            time.sleep(0.2)
            self.becco_dx.angle = 30
            time.sleep(0.2)        
        
        self.pinza_svuota_cassoni()
        self.vivi.angle = 0
        
    """
    CASSONI
    """
    def vivi_default(self):
        self.vivi.angle = 0
    def vivi_svuota(self):
        self.vivi.angle = 110
        
    def morti_default(self):
        self.morti.angle = 180
    def morti_svuota(self):
        self.morti.angle = 85

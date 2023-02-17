import time
from PiVideoStreamUndistorted import PiVideoStream
from Motors import Motors
from multiTof import sensors_stream
from servo import Servo
from gyro import get_inclinazione
class Robot():
    def __init__(self):
        self.servo = Servo()
        self.cam_stream = PiVideoStream().start()
        self.sensors_stream = sensors_stream().start()
        time.sleep(0.5)
        self.motors = Motors()
    
    def get_tof_mesures(self):
        return self.sensors_stream.get_range()
    def get_frame(self):
        return self.cam_stream.read()
    
    def get_resolution(self):
        return (self.cam_stream.LARGHEZZA, self.cam_stream.ALTEZZA)
    
    def is_salita(self):
        print(get_inclinazione())
        return get_inclinazione() > 10
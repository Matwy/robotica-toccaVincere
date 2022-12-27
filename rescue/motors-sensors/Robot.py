import time
from PiVideoStream import PiVideoStream
from Motors import Motors
from multiTof import sensors_stream
class Robot():
    def __init__(self):
        self.cam_stream = PiVideoStream().start()
        self.sensors_stream = sensors_stream().start()
        time.sleep(0.5)
        self.motors = Motors()
    
    def get_tof_mesures(self):
        self.sensors_stream.get_range()
    def get_frame(self):
        return self.cam_stream.read()
    
    def get_resolution(self):
        return (self.cam_stream.LARGHEZZA, self.cam_stream.ALTEZZA)
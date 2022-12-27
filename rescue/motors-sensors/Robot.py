import time
from PiVideoStream import PiVideoStream
from Motors import Motors
class Robot():
    def __init__(self):
        self.cam_stream = PiVideoStream().start()
        time.sleep(0.5)
        self.motors = Motors()
    
    def get_frame(self):
        return self.cam_stream.read()
    
    def get_resolution(self):
        return (self.cam_stream.LARGHEZZA, self.cam_stream.ALTEZZA)
import time
from PiVideoStream import PiVideoStream
import Motors
class Robot():
    def __init__(self):
        self.cam_stream = PiVideoStream().start()
        time.sleep(0.5)
        self.motors = Motors()
    
    def get_frame(self):
        return self.cam_stream.read()
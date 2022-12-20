from PiVideoStream import PiVideoStream
import Motors
class Robot():

    cam_stream = PiVideoStream().start()
    motors = Motors()
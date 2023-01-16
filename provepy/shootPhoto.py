from picamera import PiCamera
from time import sleep

camera = PiCamera()
for i in range(10):
    camera.start_preview()
    sleep(2)
    camera.capture(str(i)+'a.jpg')
    camera.stop_preview()
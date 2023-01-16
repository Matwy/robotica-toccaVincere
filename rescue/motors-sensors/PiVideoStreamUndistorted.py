from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np
import cv2
import yaml

class PiVideoStream:
    def __init__(self, resolution=(320, 240), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        #self.camera.rotation = 180
        self.camera.brightness = 50 #50
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

        with open("motors-sensors/calibration_matrix.yaml", "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        self.mtx = np.array(data['camera_matrix'])
        self.dist = np.array(data['dist_coeff'])
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, resolution, 1, resolution)
        print("------ROI------")
        print(self.roi)
    
    def start(self):
    # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # undistort
            dst = cv2.undistort(f.array, self.mtx, self.dist, None, self.newcameramtx)
            # crop the image
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]
            self.frame = dst
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
    
    def read(self):
        # return the frame most recently read
        return self.frame
    
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

import cv2
import numpy as np
from PiVideoStream import PiVideoStream
from time import sleep
cv2.namedWindow('image')
def nothing(x):
    pass 
cv2.createTrackbar('LOW_H', 'image', 0, 179, nothing)
cv2.createTrackbar('LOW_S', 'image', 0, 255, nothing)
cv2.createTrackbar('LOW_V', 'image', 0, 255, nothing)

cv2.setTrackbarPos('LOW_H', 'image', 179)
cv2.setTrackbarPos('LOW_S', 'image', 255)
cv2.setTrackbarPos('LOW_V', 'image', 255)

cv2.createTrackbar('HIGH_H', 'image', 0, 179, nothing)
cv2.createTrackbar('HIGH_S', 'image', 0, 255, nothing)
cv2.createTrackbar('HIGH_V', 'image', 0, 255, nothing)

cv2.setTrackbarPos('HIGH_H', 'image', 179)
cv2.setTrackbarPos('HIGH_S', 'image', 255)
cv2.setTrackbarPos('HIGH_V', 'image', 255)

cv2.createTrackbar('LOW_WHITE', 'image', 0, 255, nothing)
cam_stream = PiVideoStream().start()
sleep(0.5)


while True:
    frame = cam_stream.read().copy()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    SENSITIVITY = cv2.getTrackbarPos('LOW_WHITE', 'image')
    lower_white = 255-SENSITIVITY

    gray_scale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_scale, (7,7), 5)
    ret, bianco = cv2.threshold(blur,lower_white,255,cv2.THRESH_BINARY) 

    low_h, low_s, low_v = cv2.getTrackbarPos('LOW_H', 'image'), cv2.getTrackbarPos('LOW_S', 'image'), cv2.getTrackbarPos('LOW_V', 'image')
    high_h, high_s, high_v = cv2.getTrackbarPos('HIGH_H', 'image'), cv2.getTrackbarPos('HIGH_S', 'image'), cv2.getTrackbarPos('HIGH_V', 'image')

    lower = np.array([low_h, low_s, low_v])
    upper = np.array([high_h, high_s, high_v])

    mask = cv2.inRange(hsv, lower, upper)

    print('lower')
    print(lower)
    print('upper')
    print(upper)

    img = cv2.hconcat([mask, bianco])
    cv2.imshow('image', img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
cam_stream.stop()
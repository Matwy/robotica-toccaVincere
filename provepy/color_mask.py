import cv2
import numpy as np
from PiVideoStream import PiVideoStream
from time import sleep
cv2.namedWindow('image')
def nothing(x):
    pass

KERNEL = np.ones((5,5), np.uint8)

cv2.namedWindow('image')
cv2.namedWindow('puzzo')

cv2.createTrackbar('ERODE', 'image', 0, 10, nothing)
cv2.createTrackbar('DILATE', 'image', 0, 10, nothing)

cv2.setTrackbarPos('ERODE', 'image', 5)
cv2.setTrackbarPos('DILATE', 'image', 1)

cv2.createTrackbar('LOW_H', 'image', 0, 179, nothing)
cv2.createTrackbar('LOW_S', 'image', 0, 255, nothing)
cv2.createTrackbar('LOW_V', 'image', 0, 255, nothing)

cv2.setTrackbarPos('LOW_H', 'image', 40)
cv2.setTrackbarPos('LOW_S', 'image', 66)
cv2.setTrackbarPos('LOW_V', 'image', 55)

cv2.createTrackbar('HIGH_H', 'image', 0, 179, nothing)
cv2.createTrackbar('HIGH_S', 'image', 0, 255, nothing)
cv2.createTrackbar('HIGH_V', 'image', 0, 255, nothing)

cv2.setTrackbarPos('HIGH_H', 'image', 90)
cv2.setTrackbarPos('HIGH_S', 'image', 255)
cv2.setTrackbarPos('HIGH_V', 'image', 121)

cv2.createTrackbar('LOW_WHITE', 'image', 0, 255, nothing)
cv2.setTrackbarPos('LOW_WHITE', 'image', 200)
cam_stream = PiVideoStream().start()
sleep(0.5)

def show_pixel_value(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        pixel_value = param[y, x]
        cv2.imshow('hsv', np.zeros((50, 200), dtype=np.uint8))
        print(pixel_value)

cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
while True:
    frame = cam_stream.read().copy()
    cv2.imshow('frame', frame.copy())

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # hsv_resized =cv2.resize(hsv, (cam_stream.camera.resolution[0]*2, cam_stream.camera.resolution[1]*2))
    cv2.setMouseCallback('hsv', show_pixel_value, hsv)
    cv2.imshow('hsv', hsv.copy())
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
    
    erode_iteration = cv2.getTrackbarPos('ERODE', 'image')
    dilate_iteration = cv2.getTrackbarPos('DILATE', 'image')
    bianco_dilate=cv2.dilate(bianco,KERNEL,iterations=dilate_iteration)
    bianco_erode=cv2.erode(bianco_dilate,KERNEL,iterations=erode_iteration)

    pizzo = cv2.hconcat([bianco_erode, bianco_dilate, bianco])
    # print('lower')
    # print(lower)
    # print('upper')
    # print(upper)

    img = cv2.hconcat([mask])
    cv2.imshow('puzzo', pizzo)
    cv2.imshow('image', img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
cam_stream.stop()
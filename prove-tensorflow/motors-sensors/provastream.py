import sys
import cv2
from Robot import Robot
import time

start_time = time.time()
counter_frame = 0
count = 0
robot = Robot()
    
while True:
    counter_frame += 1
    frame = robot.get_frame().copy()
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("p"):
        count += 1
        cv2.imwrite((str(count)) +'.jpg', frame)

    if key == ord("q"):
        robot.motors.motors(0, 0)
        robot.cam_stream.stop()
        robot.sensors_stream.stop()
        robot.servo.deinit_pca()
        cv2.destroyAllWindows()
        break

finish_time = time.time()
fps = counter_frame / (finish_time - start_time)
print("fps: ", fps)
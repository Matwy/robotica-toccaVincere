import cv2
from PiVideoStream import PiVideoStream
import time

vs = PiVideoStream().start()
time.sleep(2.0)

start_time = time.time()
counter_frame = 0
    
while True:
    counter_frame += 1
    frame = vs.read()
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        break

finish_time = time.time()
fps = counter_frame / (finish_time - start_time)
print("fps: ", fps)

cv2.destroyAllWindows()
vs.stop()
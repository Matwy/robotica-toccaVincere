import cv2
import numpy as np
import time
from cvtools import scan
from global_var import ALTEZZA, LARGHEZZA

def ostacolo(robot):
    robot.motors.motors(0, 0)
    cv2.destroyAllWindows()

    robot.motors.motors(50, 50)
    time.sleep(0.3)
    robot.motors.motors(-60, 60)
    time.sleep(1.2)

    t_inizio_ostacolo = time.time()

    while True:
        
        side_tof, _, _ = robot.get_tof_mesures()
        print(side_tof)
        if side_tof < 200:
            robot.motors.motors(50, -80)
        else:
            robot.motors.motors(-30, -80)
        time.sleep(0.05)

        frame = robot.get_frame()

        roiOffset = 25
        roi = frame[roiOffset : ALTEZZA-roiOffset, roiOffset :  LARGHEZZA-roiOffset]
        
        mask_nero, _, _ =  scan(roi)
        non_zeri = np.count_nonzero(mask_nero)
        print("ostacolo", non_zeri)
        if time.time() - t_inizio_ostacolo > 3 and non_zeri > 2000:
            robot.motors.motors(-50, -50)
            time.sleep(1.3)
            robot.motors.motors(-50, 50)
            time.sleep(0.9)
            robot.motors.motors(30, 30)
            time.sleep(2.5)
            robot.motors.motors(0, 0)
            break

        cv2.imshow("ostacolo", mask_nero)  

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            exit()
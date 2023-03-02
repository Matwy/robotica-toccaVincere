import cv2
import numpy as np
import time
from cvtools import scan
from global_var import ALTEZZA, LARGHEZZA

def ostacolo(robot, _dir = 1):
    robot.motors.motors(0, 0)
    cv2.destroyAllWindows()

    robot.motors.motors(30, 30)
    time.sleep(0.3)
    robot.motors.motors(60*_dir, -60*_dir)
    time.sleep(1.3)

    t_inizio_ostacolo = time.time()
    side_tof_index = 0 if _dir == 1 else 1

    while True:
        side_tof = robot.get_tof_mesures()[side_tof_index]
        print("side_tof", side_tof)
        if side_tof < 200:
            if _dir == 1:
                robot.motors.motors(-50, 80)
            else:
                robot.motors.motors(80, -50)
        else:
            if _dir == 1:
                robot.motors.motors(30, 80)
            else:
                robot.motors.motors(80, 30)
                
        time.sleep(0.05)

        frame = robot.get_frame()

        roiOffset = 25
        roi = frame[roiOffset : ALTEZZA-roiOffset, roiOffset :  LARGHEZZA-roiOffset]
        
        mask_nero, _, _ =  scan(roi)
        non_zeri = np.count_nonzero(mask_nero)
        print("ostacolo", non_zeri)
        if time.time() - t_inizio_ostacolo > 1 and non_zeri > 2000:
            robot.motors.motors(50, 50)
            time.sleep(1.7)
            robot.motors.motors(50*_dir, -50*_dir)
            time.sleep(0.9)
            robot.motors.motors(-30, -30)
            time.sleep(2.2)
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
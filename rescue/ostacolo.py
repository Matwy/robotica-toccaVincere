import cv2
import numpy as np
import time
from cvtools import scan
from global_var import ALTEZZA, LARGHEZZA

def raggiungi_ostacolo(robot):
    front_tof_measure = 130
    ostacolo_perso_counter = 0
    ostacolo_vicino_count = 0
    print("diocan")
    while ostacolo_vicino_count < 5:
        front_tof_measure = robot.get_tof_mesures()[2]
        # NO ostacolo/
        if front_tof_measure > 130:
            ostacolo_perso_counter += 1
        else:
            ostacolo_perso_counter = 0
        if ostacolo_perso_counter > 5:
            return False
        
        # vicinanza ostacolo
        if front_tof_measure <= 40:
            ostacolo_vicino_count += 1
        else:
            ostacolo_vicino_count = 0
        print("[OSTACOLO] front_tof:", front_tof_measure)
        robot.motors.motors(40, 40)
    
    robot.motors.motors(-40, -40)
    time.sleep(0.2)
    robot.motors.motors(0, 0)
    return True

def ostacolo(robot, _dir = 1):
    cv2.destroyAllWindows()
    if not raggiungi_ostacolo(robot):
        return
    
    robot.motors.motors(60*_dir, -60*_dir)
    time.sleep(1.5)

    t_inizio_ostacolo = time.time()
    side_tof_index = 0 if _dir == 1 else 1

    while True:
        side_tof = robot.get_tof_mesures()[side_tof_index]
        print("side_tof", side_tof)
        if side_tof < 200:
            if _dir == 1:
                robot.motors.motors(25, 80)
            else:
                robot.motors.motors(80, 20)
        else:
            if _dir == 1:
                robot.motors.motors(40, 80)
            else:
                robot.motors.motors(80, 40)
                
        time.sleep(0.05)

        frame = robot.get_frame()

        roi = frame[ALTEZZA-25 : ALTEZZA, 0 : LARGHEZZA//2]
        
        mask_nero, _, _ =  scan(roi)
        non_zeri = np.count_nonzero(mask_nero)
        print("ostacolo", non_zeri)
        if time.time() - t_inizio_ostacolo > 1 and non_zeri > 800:
            if _dir == 1:
                robot.motors.motors(60, 40)
            else:
                robot.motors.motors(40, 60)
                
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
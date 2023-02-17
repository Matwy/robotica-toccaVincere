import sys
sys.path.insert(1, './motors-sensors')
import cv2
from Robot import Robot
from linea import linea
from ostacolo import ostacolo
from cuboblu import detect_blu, centra_raccogli_cubo
from salita import salita

import time

def rescue(robot):
    
    robot.servo.cam_linea()
    robot.servo.pinza_su()
    robot.servo.becco_aperto()
    
    ostacolo_count = 0
    cubo_count = 0
    salita_count = 0


    while True:
        """
        LINEA
        """
        frame = robot.get_frame()
        errore_linea, errore_angolo = linea(frame, robot)
        
        speed = 55
        kp, ki, kd = 2, 1, 2.2
        P, I, D= int(errore_linea*kp), 0, int(errore_angolo*kd)
        print("[LINEA]", "P = ", P, "   D =", D, "   time =", time.time())
        robot.motors.motors(speed + (P+D), speed - (P+D))
        """
        OSTACOLO
        """
        _, _, front_tof = robot.get_tof_mesures()
        if front_tof < 60:
            ostacolo_count += 1
        else:
            ostacolo_count = 0
        
        if ostacolo_count > 5:
            ostacolo(robot)
        """
        CUBOBBLU
        """
        if detect_blu(frame):
            cubo_count += 1
        else:
            cubo_count = 0
            
        if cubo_count > 5:
            centra_raccogli_cubo(robot)
        
        """
        SALITA
        """
        if robot.is_salita():
            salita_count += 1
        else:
            salita_count = 0
            
        if salita_count > 10:
            salita(robot)
        
        
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            robot.servo.deinit_pca()
            cv2.destroyAllWindows()
            break

if __name__ == '__main__':
    robot = Robot()
    # robot.servo.becco_chiuso()
    rescue(robot)
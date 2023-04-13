import sys
sys.path.insert(1, './motors-sensors')
import cv2
from Robot import Robot
from linea import linea
from ostacolo import ostacolo
from cuboblu import detect_blu, centra_raccogli_cubo
from salita import salita
from cvtools import isRosso
from ez import EZ

import time

def rescue(robot):
    
    robot.servo.cam_linea()
    robot.servo.pinza_su()
    robot.servo.becco_aperto()
    robot.servo.morti_default()
    robot.servo.vivi_default()
    
    ostacolo_count = 0
    cubo_count = 0
    salita_count = 0
    rosso_count = 0
    while True:
        """
        LINEA
        """
        frame = robot.get_frame().copy()
        errore_linea, errore_alto_x, errore_alto_y = linea(frame, robot)
        
        speed = 40
        kp, ki, kd = 2, 1, 2
        
        P, altoX, altoY= int(errore_linea*kp), int(errore_alto_x*kd), int(errore_alto_y*kd)
        # print("[LINEA]", "P = ", P, "   D =", D, "   time =", time.time())
        # if altoY > 0:
        # errore_alto = abs(altoY) + altoX, abs(altoY) - altoX                
        # else:
            # errore_alto = -abs(altoY) + altoX, -abs(altoY) - altoX
        
        robot.motors.motors(speed + (P+altoX), speed - (P+altoX))
        """
        OSTACOLO
        """
        _, _, front_tof = robot.get_tof_mesures()
        if front_tof < 130:
            ostacolo_count += 1
        else:
            ostacolo_count = 0
        
        if ostacolo_count > 4:
            ostacolo(robot, 1)
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
            
        if salita_count > 5:
            salita(robot)
        
        """
        EZ
        """
        if robot.check_ez():
            ez = EZ(robot)
            ez.loop_palle()
            ez.loop_triangoli()
            ez.loop_uscita()
        
        """
        ROSSO
        """
        if isRosso(frame):
            rosso_count += 1
        else:
            rosso_count = 0
        
        if rosso_count > 5:
            robot.motors.motors(0,0)
            print("[ROSSO] rosso")
            # time.sleep(6)
        
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
    robot.servo.cam_cubo()
    # while True:
        # print(robot.get_gyro_value())
    # robot.servo.cam_linea()
    # robot.servo.pinza_giu()
    # robot.servo.pinza_su()
    # robot.motors.motors(50, 100)
    # ez = EZ(robot)
    # ez.loop_palle()
    # ez.loop_triangoli()
    # ez.loop_uscita()
    # while True:
    #     print(robot.get_tof_mesures())
    print("[MAIN] rescue()")
    rescue(robot)
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
        errore_basso_x, errore_basso_y, errore_alto_x, errore_alto_y, errore_tot = linea(frame, robot)
        speed = 40
        k_basso_x, k_basso_y, k_alto_x, k_alto_y = 0.1, 0, 0.25, 0.1 #1 0.3
        
        basso_x, basso_y, altoX, altoY= int(errore_basso_x*k_basso_x), int(errore_basso_y*k_basso_y), int(errore_alto_x*k_alto_x), int(errore_alto_y*k_alto_y)
        print("[LINEA]", "basso = ",basso_x, basso_y, (basso_x-basso_y), "   alto =", (altoX+altoY), "   time =", time.time())
        # if altoY > 0:
        # errore_alto = abs(altoY) + altoX, abs(altoY) - altoX                
        # else:
            # errore_alto = -abs(altoY) + altoX, -abs(altoY) - altoX
        # if basso_y != 0:
            # robot.motors.motors(speed + (altoX*altoY), speed - (altoX*altoY))
        # else:
        robot.motors.motors(speed + ((basso_x)+(altoX*altoY)), speed - ((basso_x)+(altoX*altoY)))
        
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
    # robot.servo.cam_cubo()
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
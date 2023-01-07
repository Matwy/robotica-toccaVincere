import sys
sys.path.insert(1, './motors-sensors')
import cv2
from Robot import Robot
from linea import linea
import time

def rescue(robot):
    
    while True:
        """
        LINEA
        """
        frame = robot.get_frame()
        errore_linea, errore_angolo = linea(frame)
        mid = 320//2
        pwr = (mid - errore_linea)//5
        if pwr > 0: 
            robot.motors.motors(20-pwr, 20)
        else:
            robot.motors.motors(20, 20+pwr)
            
        """
        OSTACOLO
        """
        tof_mesures = robot.get_tof_mesures()
        print(tof_mesures)
        
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            cv2.destroyAllWindows()
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            break

if __name__ == '__main__':
    robot = Robot()
    rescue(robot)
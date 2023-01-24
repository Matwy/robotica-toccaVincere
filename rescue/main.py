import sys
sys.path.insert(1, './motors-sensors')
import cv2
from Robot import Robot
from linea import linea
from ostacolo import ostacolo
import time

def rescue(robot):
    ostacolo_count = 0    
    while True:
        """
        LINEA
        """
        frame = robot.get_frame()
        errore_linea, errore_angolo = linea(frame, robot)
        
        speed = -55
        kp, ki, kd = 2, 1, 2.2
        P, I, D= int(errore_linea*kp), 0, int(errore_angolo*kd)
        print("P = ", P, "   D =", D, "   time =", time.time())
        robot.motors.motors(speed - (P+D), speed + (P+D))
        """
        OSTACOLO
        """
        front_tof, _, _ = robot.get_tof_mesures()
        if front_tof < 50:
            ostacolo_count += 1
            print(front_tof, "      count", ostacolo_count)
        else:
            ostacolo_count = 0
        
        #if ostacolo_count > 10:
        #    ostacolo(robot)

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            break

if __name__ == '__main__':
    robot = Robot()
    rescue(robot)
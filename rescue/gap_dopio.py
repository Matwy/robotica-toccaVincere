import cv2
from cvtools import scan, get_bigger_area
import time
from global_var import ALTEZZA, LARGHEZZA

def doppio_verde(robot):
    robot.motors.motors(0, 0)
    cv2.destroyAllWindows()
    i = 0

    while True:
        frame = robot.get_frame()
        mask_nero, _, mask_verde = scan(frame)
        
        # prendo come roi la parte bassa della linea nera e calcolo l'errore
        cut = mask_nero[-50:, :]
        M = cv2.moments(cut)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
            cv2.circle(frame, (x,ALTEZZA), 10, (190, 170, 200), -1)
        else:
            x = 0

        #trovo il centro dei verdi per capire quanto sono distante 
        M = cv2.moments(mask_verde) 
        if M["m00"] != 0:
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (LARGHEZZA//2,cY), 10, (90, 70, 200), -1)
        else:
            cY = 0
        
        # se sono molto vicino ai due verdi giro
        if cY > ALTEZZA-20:
            robot.motors.motors(-40, -40)
            time.sleep(1)
            #gira di 180°
            robot.motors.motors(73, -70)
            time.sleep(2.7)

            robot.motors.motors(40, 40)
            time.sleep(0.5)
            cv2.destroyAllWindows()
            break

        # uso una correzione proporzionale per centrarmi
        sp = -20
        kp = 4
        errore = x - (LARGHEZZA//2)
        robot.motors.motors(sp - (errore*kp), sp + (errore*kp))
        
        cv2.rectangle(frame, (0, ALTEZZA-50), (LARGHEZZA, ALTEZZA), (0,0,255), 2)
        cv2.imshow("dopio", frame)  
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            exit()
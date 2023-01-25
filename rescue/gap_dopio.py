import cv2
from cvtools import scan, get_bigger_area, scan_nero, sort_aree
import time
from global_var import ALTEZZA, LARGHEZZA

def get_x_bottom_line(frame):
    mask_nero, _, mask_verde = scan(frame)
    cut = mask_nero[-50:, :]
    
    cut = mask_nero[-50:, :]
    M = cv2.moments(cut)
    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
    else:
        x = 0
    
    return x

def doppio_verde(robot):
    robot.motors.motors(0, 0)
    cv2.destroyAllWindows()

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

def is_gap(frame):
    _, mask_bianco, _ = scan(frame)
    amount_bianco, _ = cv2.connectedComponents(mask_bianco)
    if amount_bianco >= 3:
        return False
    
    return True
    

def gap(robot):
    cv2.destroyAllWindows()
    #robot.servo.set_cam_angle(140)
    while True:
        frame = robot.get_frame()
        
        mask_nero, _, mask_verde = scan(frame)
        
        # prendo come roi la parte bassa della linea nera e calcolo l'errore
        area_alta = mask_nero[:, 30:-30]
        M = cv2.moments(area_alta)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
        else:
            x = 0

        """ QUIT GAP """
        if not is_gap(frame): 
            robot.servo.cam_linea()
            break 

        mask_nero = scan_nero(frame)
        #amount, labels = cv2.connectedComponents(mask_nero)
        #area_alta = sort_aree(amount, labels, 1)[-1] # prende le aree di nero e le ordina per la y
        #area_alta = area_alta[:, 30:-30]
        cv2.imshow("area_alta", mask_nero)  
        """ TROVA CENTRO LINEA ALTA """
        M = cv2.moments(mask_nero)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cv2.circle(frame, (cx,ALTEZZA), 10, (190, 170, 200), -1)
        else:
            cx = 0
        print(cx)
        
        # uso una correzione proporzionale per centrarmi
        sp = 20
        kp = 4
        errore = cx - (LARGHEZZA//2)
        robot.motors.motors(sp + (errore*kp), sp - (errore*kp))

        cv2.circle(frame, (x,ALTEZZA), 10, (190, 170, 200), -1)
        cv2.imshow("gapping", frame)  
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            robot.servo.deinit_pca()
            exit()
        

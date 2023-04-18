import cv2
import numpy as np
from cvtools import scan, get_bigger_area
from global_var import ALTEZZA, LARGHEZZA
import time
from Incrocio import Incrocio
from gap_dopio import get_centro_linea

MASK_BORDI_SALITA_NOISE = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
cv2.rectangle(MASK_BORDI_SALITA_NOISE, (20, 35), (LARGHEZZA-20, ALTEZZA), (255), -1)
MASK_BORDI_SALITA_NOISE = cv2.bitwise_not(MASK_BORDI_SALITA_NOISE)

def salita(robot):
    robot.motors.motors(-60, -60)
    robot.servo.pinza_salita()
    time.sleep(0.3)
    robot.motors.motors(30, 30)
    time.sleep(0.5)
    cv2.destroyAllWindows()
    piano_count = 0
    incrocio_count = 0

    while True:
        frame = robot.get_frame().copy()
        mask_nero, _, mask_verde = scan(frame)
        mask = get_bigger_area(mask_nero)
        mask_bianco = cv2.bitwise_not(mask_nero)
        amount_bianco, _ = cv2.connectedComponents(mask_bianco)
        if amount_bianco > 3:
            print("incrocio_count", incrocio_count)
            incrocio_count +=1
        else:
            incrocio_count = 0
            
        if incrocio_count > 5:
            Incrocio(robot).loop_centra_incrocio()
            robot.servo.pinza_su()
            break
        
        #tolgo 20px left top e right dall'immagine per il noise
        frame[MASK_BORDI_SALITA_NOISE == 255] = 255
        #calcola il centro della linea sotto e l'angolo
        x_linea, angle = get_centro_linea(robot, frame)
        if x_linea is False or angle is False:
            robot.motors.motors(30,30)
            continue

        # centro il robot 
        sp, kp, kd = 30, 1.5, 0.5
        errore_linea = x_linea - (LARGHEZZA//2)
        P, D= int(errore_linea*kp), int(angle*kd)
        robot.motors.motors(sp + (P+D), sp - (P+D))
        print("[artito democratico]", P, D)
        
        if not robot.is_salita():
            piano_count += 1
        else:
            piano_count = 0
        
        if piano_count > 10:
            robot.motors.motors(30,30)
            robot.servo.pinza_su()
            time.sleep(0.3)
            robot.motors.motors(0, 0)
            break

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            cv2.destroyAllWindows()
            robot.cam_stream.stop()
            exit()
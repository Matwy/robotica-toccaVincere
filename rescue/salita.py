import cv2
from cvtools import scan, get_bigger_area
from global_var import ALTEZZA, LARGHEZZA
import time
from Incrocio import Incrocio
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
            robot.is_salita()
            Incrocio(robot).loop_centra_incrocio()
            robot.servo.pinza_su()
            break
        #trovo la linea nera e calcolo l'errore
        cut = mask[-40:-10, :]
        M = cv2.moments(cut)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
        else:
            x = 0

        robot.last_punto_alto, robot.last_punto_basso = (x, ALTEZZA-30), (x, ALTEZZA-30)
        cv2.circle(frame, (x, ALTEZZA-10), 20, (230,230,50), 2)
        
        sp = 30
        kp = 1
        errore = int((x*kp) - (LARGHEZZA//2))
        robot.motors.motors(sp + errore, sp - errore)
        
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
        cv2.imshow("salita mask", cut)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            cv2.destroyAllWindows()
            robot.cam_stream.stop()
            exit()
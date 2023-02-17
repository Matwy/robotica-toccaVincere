import cv2
from cvtools import scan, get_bigger_area
from global_var import ALTEZZA, LARGHEZZA
def salita(robot):
    robot.servo.pinza_salita()
    cv2.destroyAllWindows()
    piano_count = 0

    while True:
        frame = robot.get_frame()
        mask_nero, _, mask_verde = scan(frame)
        mask = get_bigger_area(mask_nero)
        #trovo la linea nera e calcolo l'errore
        cut = mask[-40:-20, :]
        M = cv2.moments(cut)
        if M["m00"] != 0:
            x = int(M["m10"] / M["m00"])
        else:
            x = 0

        sp = 30
        kp = 2
        errore = x - (LARGHEZZA//2)
        robot.motors.motors(sp + errore, sp - errore)
        
        if not robot.is_salita():
            piano_count += 1
        else:
            piano_count = 0
        
        if piano_count > 5:
            robot.servo.pinza_su()
            break

        cv2.imshow("salita", frame)
        cv2.imshow("salita mask", cut)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            cv2.destroyAllWindows()
            robot.cam_stream.stop()
            exit()
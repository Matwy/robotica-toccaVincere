import cv2
from cvtools import scan, get_bigger_area, scan_nero, sort_aree
import time
from global_var import ALTEZZA, LARGHEZZA
from ez import EZ
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
        frame = robot.get_frame().copy()
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
            robot.motors.motors(40, 40)
            time.sleep(1)
            #gira di 180°
            robot.motors.motors(-70, 70)
            time.sleep(2.5)

            robot.motors.motors(-40, -40)
            time.sleep(0.5)
            cv2.destroyAllWindows()
            break

        # uso una correzione proporzionale per centrarmi
        sp = 20
        kp = 4
        errore = x - (LARGHEZZA//2)
        robot.motors.motors(sp + int(errore*kp), sp - int(errore*kp))
        
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
    
def get_centro_linea(frame):
    mask_nero = scan_nero(frame)
    mask_bigger_nero = get_bigger_area(mask_nero)
    _, cnts, _ = cv2.findContours(mask_bigger_nero.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if(len(cnts)) == 0 : return False, False
    cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)
    contour_linea = cnts[0]

    y_top = tuple(contour_linea[contour_linea[:, :, 1].argmin()][0])[1]
    y_bot = tuple(contour_linea[contour_linea[:, :, 1].argmax()][0])[1]
    
    cut_top = mask_bigger_nero[y_top+5:y_top+10, :]
    cut_bot = mask_bigger_nero[y_bot-10:y_bot-5, :]
    
    M1, M2 = cv2.moments(cut_top), cv2.moments(cut_bot),
    x_top = int(M1["m10"] / M1["m00"]) if M1["m00"] != 0 else ALTEZZA//2
    x_bot = int(M2["m10"] / M2["m00"]) if M2["m00"] != 0 else ALTEZZA//2

    return x_top, x_top-x_bot

def trova_linea(robot):

    print("[GAP] BIANCHETTO")
    timer_alza_cam = time.time()
    avanti = True

    while True:
        if robot.check_ez():
            ez = EZ(robot)
            # ez.loop_palle()
            ez.loop_triangoli()

        frame = robot.get_frame()
        centro_linea, angle = get_centro_linea(frame)
        
        if centro_linea != False and avanti:
            robot.servo.cam_linea() # linea trovata
            break

        if angle != False and not avanti:
            #parcheggio ad S
            sp, kp, kd = -30, 1, 1.5
            errore_linea = centro_linea - (LARGHEZZA//2)
            P, D= int(errore_linea*kp), -int(angle*kd)
            print("[GAP] linea", errore_linea, "  angolo", angle)
            robot.motors.motors(sp - (P+D), sp + (P+D))

            if abs(errore_linea) > 40:
                robot.motors.motors(50, 50)
                time.sleep(0.5)
                robot.motors.motors(sp - (P+D), sp + (P+D))
                time.sleep(0.3)

                break


        if avanti:            
            robot.motors.motors(40,40)
            robot.servo.cam_su()

            if time.time() - timer_alza_cam  > 2:
                avanti = not avanti
                timer_alza_cam = time.time()

        if not avanti:
            if centro_linea == False : robot.motors.motors(-40, -40)
            robot.servo.cam_linea()

            if time.time() - timer_alza_cam  > 4:
                avanti = not avanti
                timer_alza_cam = time.time()

        cv2.imshow("gapping", frame)  
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            robot.servo.deinit_pca()
            exit()    

def gap(robot):
    cv2.destroyAllWindows()
    quit_gap_counter = 0
    no_linea_counter = 0
    while True:
        if robot.check_ez():
            ez = EZ(robot)
            # ez.loop_palle()
            ez.loop_triangoli()

        frame = robot.get_frame().copy()

        """ QUIT GAP """
        if not is_gap(frame):
            quit_gap_counter += 1
            if quit_gap_counter > 10:
                robot.servo.cam_linea()
                cv2.destroyAllWindows()
                break
        else:
            quit_gap_counter = 0
        
        """ CONTROLLA SE LA LINEA C'E' """
        centro_linea, angle = get_centro_linea(frame)

        if centro_linea == False : 
            no_linea_counter += 1
            centro_linea = ALTEZZA//2
        else:
            no_linea_counter = 0
        if no_linea_counter > 30:
            trova_linea(robot)
            continue

        cv2.circle(frame, (centro_linea,ALTEZZA), 10, (190, 170, 200), -1)
        # uso una correzione proporzionale per centrarmi
        sp, kp, kd = 40, 1.5, 2
        print("[GAP] angolo", angle)
        errore_linea = centro_linea - (LARGHEZZA//2)
        P, D= int(errore_linea*kp), int(angle*kd)
        robot.motors.motors(sp + (P+D), sp - (P+D))

        cv2.imshow("gapping", frame)  
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            robot.servo.deinit_pca()
            exit()
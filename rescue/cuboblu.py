import cv2
import numpy as np
import time
from global_var import ALTEZZA, LARGHEZZA

lower_blue = np.array([90,70,70])
upper_blue = np.array([130,255,230])

kernel = np.ones((9,9), np.uint8)

def detect_blu(frame):
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    blu = cv2.inRange(hsv, lower_blue, upper_blue)
    blu = cv2.erode(blu,kernel,iterations=2)
    blu = cv2.dilate(blu,kernel,iterations=2)

    M = cv2.moments(blu)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        #se il cubo e troppo lontano None
        return (cX, cY)
    else:
        return None

def raggiungi_cubo(robot):
    # allontanati dal cubetto e tira giù la pinza
    # cam_check_pinza()
    # becco_aperto()
    robot.motors.motors(-30, -30)
    time.sleep(1.3)
    # braccio_giu()
    robot.motors.motors(0, 0)
    #  avvicinati al cubetto fino
    #  a quando lo vedi dentro alla pinza
    t_inizio = time.time()
    while True:
        frame = robot.get_frame()
        cubo = detect_blu(frame)
        robot.motors.motors(15, 15)

        if cubo == None: continue
        print(cubo)        
        if cubo[1] > ALTEZZA-53:
            t_fine = time.time()
            robot.motors.motors(0, 0)
            # becco_chiuso()
            time.sleep(0.3)
            # braccio_su_drop_cubo()

            time.sleep(0.3)
            # becco_rilascia_morti()
            time.sleep(0.3)
            # cubo_avanti()
            # braccio_default()
            robot.motors.motors(-15, -15)
            time.sleep(t_fine - t_inizio)
            # cam_giu()
            break

def centra_raccogli_cubo(robot):
    cv2.destroyAllWindows()
    n_errori_positivi = 0
    n_errori_negativi = 0
    cubo_perso_count = 0

    #per capire quanto tempo ci mette a raddrizzarsi
    t_inizio = time.time()
    
    while True:
        frame = robot.get_frame()
        cubo = detect_blu(frame)
        
        #se non vedo più il cubo per x frame esco dal loop
        if cubo_perso_count > 60:
            t_fine = time.time()
            #raddrizza ed esci dal ciclo
            if n_errori_negativi < n_errori_positivi:
                robot.motors.motors(-20, 20)
            else:
                robot.motors.motors(20, -20)
            
            time.sleep(t_fine - t_inizio)
            robot.motors.motors(0, 0)
            break

        if cubo == None:
            robot.motors.motors(0, 0)
            cubo_perso_count += 1
            print("[CUBOBLU] cubo_perso_count ", cubo_perso_count)
            continue
        #il cubo c'è nell'immagine
        cubo_perso_count = 0

        #negativo cubo sinistra positivo destra
        errore_cubo = cubo[0] - (LARGHEZZA//2)
        
        #il cubo centrato
        if abs(errore_cubo) < 10:
            t_fine = time.time()
            robot.motors.motors(0, 0)
            """
            RACCOGLI CUBO
            """
            raggiungi_cubo(robot)
            
            #raddrizza ed esci dal ciclo
            if n_errori_negativi < n_errori_positivi:
                robot.motors.motors(-20, 20)
            else:
                robot.motors.motors(20, -20)
            
            time.sleep(t_fine - t_inizio)
            robot.motors.motors(0, 0)
            break
        
        #centra il cubetto
        if errore_cubo < 0:
            robot.motors.motors(-20, 20)
            n_errori_negativi += 1
        if errore_cubo > 0:
            robot.motors.motors(20, -20)
            n_errori_positivi += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            robot.motors.motors(0, 0)
            robot.cam_stream.stop()
            robot.sensors_stream.stop()
            cv2.destroyAllWindows()
            robot.servo.deinit_pca()
            exit()
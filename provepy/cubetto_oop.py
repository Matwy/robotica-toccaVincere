import cv2
import numpy as np
import time
from global_var import ALTEZZA, LARGHEZZA

class RescueKit:
    LOWER_BLUE = np.array([90,70,70])
    UPPER_BLUE = np.array([130,255,230])
    KERNEL = np.ones((9,9), np.uint8)

    def __init__(self, robot):
        self.robot = robot

    def detect_blu(self, frame):
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        blu = cv2.inRange(hsv, self.LOWER_BLUE, self.UPPER_BLUE)
        blu = cv2.erode(blu,self.KERNEL,iterations=1)
        blu = cv2.dilate(blu,self.KERNEL,iterations=2)
        key = cv2.waitKey(1) & 0xFF
        M = cv2.moments(blu)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #se il cubo e troppo lontano None
            return (cX, cY)
        else:
            return None

    def raggiungi_cubo(self):
        # allontanati dal cubetto e tira giù la pinza
        # cam_check_pinza()
        self.robot.servo.becco_aperto()
        self.robot.motors.motors(-40, -40)
        time.sleep(1)
        self.robot.servo.pinza_giu()
        time.sleep(0.5)
        self.robot.motors.motors(0, 0)
        self.robot.servo.cam_cubo()
        #  avvicinati al cubetto fino
        #  a quando lo vedi dentro alla pinza
        t_inizio = time.time()
        while True:
            frame = self.robot.get_frame()
            cubo = self.detect_blu(frame)
            self.robot.motors.motors(30, 30)
            
            if time.time() - t_inizio > 5:
                self.robot.servo.pinza_su()
                self.robot.motors.motors(-30, -30)
                time.sleep(time.time() - t_inizio)
                self.robot.servo.cam_linea()
                break 

            if cubo == None: continue
            print("[CUBOBLU] cubo_y", cubo[1])
            if cubo[1] > 65:
                t_fine = time.time()
                self.robot.motors.motors(0, 0)
                self.robot.servo.becco_chiuso()
                time.sleep(0.3)
                self.robot.servo.pinza_su_max()
                time.sleep(1)
                self.robot.servo.becco_molla_vivi()

                self.robot.motors.motors(-30, -30)
                time.sleep(t_fine - t_inizio)
                self.robot.servo.cam_linea()
                break        
            
            cv2.circle(frame, cubo, 20, (230,230,50), 2)
            cv2.imshow("raggiungicubo", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                cv2.destroyAllWindows()
                self.robot.servo.deinit_pca()
                exit()

    def centra_raccogli_cubo(self):
        cv2.destroyAllWindows()
        n_errori_positivi = 0
        n_errori_negativi = 0
        cubo_perso_count = 0

        #per capire quanto tempo ci mette a raddrizzarsi
        t_inizio = time.time()
        
        while True:
            frame = self.robot.get_frame()
            cubo = self.detect_blu(frame)
            
            #se non vedo più il cubo per x frame esco dal loop
            if cubo_perso_count > 60:
                t_fine = time.time()
                #raddrizza ed esci dal ciclo
                if n_errori_negativi < n_errori_positivi:
                    self.robot.motors.motors(-20, 20)
                else:
                    self.robot.motors.motors(20, -20)
                
                time.sleep(t_fine - t_inizio)
                self.robot.motors.motors(0, 0)
                break

            if cubo == None:
                self.robot.motors.motors(0, 0)
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
                self.robot.motors.motors(0, 0)
                """
                RACCOGLI CUBO
                """
                self.raggiungi_cubo(self.robot)
                
                #raddrizza ed esci dal ciclo
                if n_errori_negativi < n_errori_positivi:
                    self.robot.motors.motors(-20, 20)
                else:
                    self.robot.motors.motors(20, -20)
                
                time.sleep(t_fine - t_inizio)
                self.robot.motors.motors(0, 0)
                break
            
            #centra il cubetto
            if errore_cubo < 0:
                self.robot.motors.motors(-20, 20)
                n_errori_negativi += 1
            if errore_cubo > 0:
                self.robot.motors.motors(20, -20)
                n_errori_positivi += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                cv2.destroyAllWindows()
                self.robot.servo.deinit_pca()
                exit()
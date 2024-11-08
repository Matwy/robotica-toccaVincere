import sys
import time

import cv2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

from cvtools import scan_bordi, sort_aree, get_nearest_countourn_point, get_bigger_component, scan, get_nearest_area_from_2points
class EZ:
    Y_ABBASSA_BRACCIO = 50
    Y_BECCO_SOPRA = 83
    Y_BECCO_SOTTO = 110
    MODELLO_PALLE = 'model_4.tflite' # le mie non le vittime
    MODELLO_TRIANGOLI = 'triangoli.tflite'
    MODELLO_USCITA = 'android.tflite'
    
    def __init__(self, robot):
        robot.motors.motors(90, 90) # entra un po' durante il setup
        cv2.destroyAllWindows()

        # Initialize the object detection model
        base_options = core.BaseOptions(file_name=self.MODELLO_PALLE, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.70)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_palle = vision.ObjectDetector.create_from_options(options)
        
        base_options = core.BaseOptions(file_name=self.MODELLO_TRIANGOLI, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.6)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_triangoli = vision.ObjectDetector.create_from_options(options)
        
        base_options = core.BaseOptions(file_name=self.MODELLO_USCITA, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.80)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_uscita = vision.ObjectDetector.create_from_options(options)
        
        self.robot = robot
        self.robot.servo.cam_EZ()
        self.robot.servo.pinza_su()
        self.robot.servo.vivi_default()
        self.robot.servo.morti_default()
        
        # change camera settings
        self.robot.camstream_EZ()
        time.sleep(2)
        self.LARGHEZZA, self.ALTEZZA = robot.cam_stream.camera.resolution
        
        self.pinza_su = True
        self.output = None
        self.palline_vive = 0
        self.palline_morte = 0
        self.triangolo_rosso = 0
        self.triangolo_verde = 0
        
        
        robot.motors.motors(60, -60)
        time.sleep(2)
        robot.motors.motors(0, 0)
        
        self.timer_stuck = time.time()
    
    def check_if_stuck(self):
        self.robot.motors.motors(0, 0)
        time.sleep(1.5)

        self.robot.motors.motors(-70, -70)
        time.sleep(0.5)
        self.robot.servo.cam_linea()
        time.sleep(0.5)
        self.robot.motors.motors(0, 0)

        front_tof_mesure = self.robot.get_tof_mesures()[2]
        self.robot.servo.cam_EZ()
        if front_tof_mesure < 300:
            self.robot.motors.motors(-70, 70)
            time.sleep(1.5)
            self.robot.motors.motors(0, 0)
            return "iera bloca"
            
    
    def giro_bordi(self, frame, mode):
        bordi = scan_bordi(frame)
        cv2.imshow("bordi",bordi)
        aree_muri_pavimento = cv2.bitwise_not(bordi)
        amount, labels = cv2.connectedComponents(aree_muri_pavimento)
        
        if amount == 1: self.robot.motors.motors(80, 50) # destra

        aree_muri_pavimento = sort_aree(amount, labels, 1, _blank='ez')
        if len(aree_muri_pavimento) > 0:
           area_bassa = aree_muri_pavimento[-1]
        else:
           return None
        
        if area_bassa is None: return
        if mode == 'alto':
            roi_pt1, roi_pt2 = (0, 65), (self.LARGHEZZA, 70)
        elif mode == 'basso':
            roi_pt1, roi_pt2 = (0, 105), (self.LARGHEZZA, 110)
        else:
            roi_pt1, roi_pt2 = (0, 70), (self.LARGHEZZA, 75)
            
        roi = area_bassa[roi_pt1[1] : roi_pt2[1], roi_pt1[0] : roi_pt2[0]]
        cv2.rectangle(self.output, roi_pt1, roi_pt2, (125, 125, 7), 2)
        
        if np.count_nonzero(roi == 0) < 100:
            #destra
            self.robot.motors.motors(80, 40)
            return

        roi_mask_bordi = cv2.bitwise_not(roi)
        M = cv2.moments(roi_mask_bordi)
        if M["m00"] == 0: return
        cX = int(M["m10"] / M["m00"])
        
        errore_bordo = self.LARGHEZZA-cX
        if errore_bordo < self.LARGHEZZA // 3.5:
            # sinistra piano
            self.robot.motors.motors(50, 50 + 2*errore_bordo)
        else:
            # sinistra pott
            if mode == 'basso':
                print("[EZ girobordi basso] pottt")
                self.robot.motors.motors(-20, 30)
            else:
                self.robot.motors.motors(-70, 90)
            
    
    def get_selected_ball(self, frame):
        # detect ball with tensor
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector_palle.detect(input_tensor)
        # add to array and draw ball on the frame
        ball = []
        for d in detection_result.detections:
            x, y, w, h, index = d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height, d.categories[0].index
            ball.append((x,y,w,h, index))
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,0), 1)
        # return the bigger ball
        if len(ball) > 0:
            ball.sort(key=lambda b : b[2]*b[3], reverse=True) # ordina le palle in base alla più grande (quindi la più vicina)
            x, y, w, h, _ = ball[0]
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,255), 4)
            return ball[0]
    
    def raccogli_palla(self, ball, tipo_palla):
        ball_x = ball[0] + (ball[2]//2)
        ball_y = ball[1] + (ball[3]//2)
        # palla troppo vicina vai indietro con la pinza su
        if ball_y > self.Y_BECCO_SOTTO:
            self.robot.motors.motors(-40, -40)
            if not self.pinza_su:
                self.robot.servo.pinza_su()
                self.pinza_su = True
                return
                
        # PID tenendo in considerazione la distanza della palla
        speed = 70 if ball_y < self.Y_ABBASSA_BRACCIO+10 else 40
        errore_x = ball_x - self.LARGHEZZA//2
        self.robot.motors.motors(speed + int(errore_x//2), speed - int(errore_x//2))
        if ball_y > self.Y_ABBASSA_BRACCIO and self.pinza_su:
            # abbassa pinza se la palla è vicina e la pinza è su
            
            if self.robot.get_tof_mesures()[1] < 200:
                # robot troppo vicino al muro di destra quindi fai manovra
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                self.robot.motors.motors(-60, 60)
                time.sleep(0.7)
                self.robot.motors.motors(100, 100)
                time.sleep(1)
                self.robot.motors.motors(60, -60)
                time.sleep(1.3)
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                return
            self.robot.motors.motors(0, 0)
            self.robot.servo.pinza_giu()
            self.robot.servo.becco_raccolta()
            self.pinza_su = False
        if ball_y > self.Y_BECCO_SOPRA and abs(errore_x) > 10 and not self.pinza_su:
            # palla troppo sotto e fuori dal becco
            self.robot.motors.motors(-40, -40)
            
        if ball_y > self.Y_BECCO_SOPRA and ball_y < self.Y_BECCO_SOTTO and abs(errore_x) < 10 and not self.pinza_su:
            # raccogli palla se è dentro i becchi e al centro dello schermo e la pinza è giu
            self.robot.motors.motors(40, 40)
            time.sleep(1)
            self.robot.servo.becco_chiuso()
            time.sleep(0.3)
            self.robot.motors.motors(-35, -35)
            self.robot.servo.pinza_su()
            time.sleep(1)
            if tipo_palla >= 0: # distinzione morte vive
                self.robot.servo.becco_molla_morti()
                self.palline_morte += 1
            else:
                self.robot.servo.becco_molla_vivi()
                self.palline_vive += 1
            self.robot.servo.becco_raccolta()
            self.pinza_su = True
            return 'raccolta'
    
    def loop_palle(self):
        palla_persa_count = 0
        tipo_palla = 0
        t_inizio_ricerca = time.time()
        while True:
            frame = self.robot.get_frame()
            self.output = frame.copy()
            
            ball = self.get_selected_ball(frame)
            cv2.circle(self.output, (self.LARGHEZZA//2, self.Y_BECCO_SOPRA), 5, (0, 0, 255), -1)
            cv2.circle(self.output, (self.LARGHEZZA//2, self.Y_BECCO_SOTTO), 5, (0, 0, 255), -1)
            if ball:
                self.timer_stuck = time.time() # reset timer (not stuck)
                # PALLE 
                palla_persa_count = 0
                if ball[1] > self.Y_ABBASSA_BRACCIO:
                    tipo_palla += 1 if ball[-1] == 1 else -1
                print("[EZ] loop_palle() palla: ", ball, "tipo_palla ", tipo_palla, " tof ", self.robot.get_tof_mesures()[1])
                if self.raccogli_palla(ball, tipo_palla) == 'raccolta':
                    tipo_palla = 0
            elif self.pinza_su:
                # BORDI
                tipo_palla = 0
                mode_giro_bordi = 'alto' #if time.time() - t_inizio_ricerca < 20 else 'basso' 
                self.giro_bordi(frame, mode_giro_bordi)
                if time.time() - self.timer_stuck > 5:
                    self.check_if_stuck()
                    self.timer_stuck = time.time() # reset timer
                    
            else:
                self.robot.motors.motors(-30, -30)
                palla_persa_count += 1
                print(palla_persa_count)

            if palla_persa_count > 10:
                self.robot.servo.pinza_su()
                self.pinza_su = True
                
            if self.palline_vive >= 2 and self.palline_morte >= 1:
                break
                                
            cv2.imshow("output", self.output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                self.robot.servo.deinit_pca()
                cv2.destroyAllWindows()
                exit()
    
    def get_selected_triangolo(self, frame):
        # detect triangolo with tensor
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector_triangoli.detect(input_tensor)
        # add to array and draw triangolo on the frame
        triangoli = []
        for d in detection_result.detections:
            print(d)
            x, y, w, h, index = d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height, d.categories[0].index
            triangoli.append((x,y,w,h, index))
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,0), 1)
        # return the bigger triangolo
        if len(triangoli) > 0:
            triangoli.sort(key=lambda b : b[2]*b[3], reverse=True) # ordina le palle in base alla più grande (quindi la più vicina)
            x, y, w, h, _ = triangoli[0]
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,255), 4)
            return triangoli[0]
    
    def raggiungi_triangolo(self, triangolo):
        speed = 90
        if self.triangolo_verde == 0 and triangolo[-1] == 1:
            self.robot.motors.motors(-50,50)    
            return
        
        errore_x = triangolo[0] + (triangolo[2]//2) - (self.LARGHEZZA//2)
        self.robot.motors.motors(speed + (errore_x), speed - (errore_x))
        # controllo che la y+h del triangolo sia bassa quindi vicina al robot
        if triangolo[1]+triangolo[3] > 60:
            self.triangolo_vicino_counter += 1
        else:
            self.triangolo_vicino_counter = 0
            
        if self.triangolo_vicino_counter > 5:
            
            if self.robot.get_tof_mesures()[1] < 200:
                # robot troppo vicino al muro di destra quindi fai manovra
                self.robot.motors.motors(-50, -50)
                time.sleep(1.5)
                self.robot.motors.motors(-100, 100)
                time.sleep(0.7)
                self.robot.motors.motors(100, 100)
                time.sleep(0.8)
                self.robot.motors.motors(100, -100)
                time.sleep(1)
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                self.triangolo_vicino_counter = 0
                return

            if self.robot.get_tof_mesures()[0] < 200:
                # robot troppo vicino al muro di sinistra quindi fai manovra
                self.robot.motors.motors(-50, -50)
                time.sleep(1.5)
                self.robot.motors.motors(100, -100)
                time.sleep(0.7)
                self.robot.motors.motors(100, 100)
                time.sleep(0.8)
                self.robot.motors.motors(-100, 100)
                time.sleep(1)
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                self.triangolo_vicino_counter = 0
                return
            
            self.robot.motors.motors(50, 50)
            time.sleep(2)
            self.robot.servo.pinza_svuota_cassoni()
                
            if self.tipo_triangolo <= 0:
                self.robot.motors.motors(60, -60)
                time.sleep(2)
                self.robot.motors.motors(-25, -50)
                time.sleep(2.5)
                self.robot.motors.motors(-30, 50)
                time.sleep(1.5)  
                self.robot.motors.motors(40, 40)
                time.sleep(2)  
                
                self.robot.servo.vivi_svuota()
                time.sleep(1)
                self.robot.motors.motors(60, -50)
                
                self.triangolo_verde += 1
                self.tipo_triangolo = 0
            elif self.triangolo_verde >= 1 and self.tipo_triangolo > 0:
                self.robot.motors.motors(-60, 60)
                time.sleep(2)
                self.robot.motors.motors(-50, -35)
                time.sleep(2.5)
                self.robot.motors.motors(50, -30)
                time.sleep(1.5)  
                self.robot.motors.motors(40, 40)
                time.sleep(2)  
                self.robot.motors.motors(0, 0)
                
                self.robot.servo.morti_svuota()
                time.sleep(1)
                self.robot.motors.motors(-50, 60)
                
                self.tipo_triangolo = 0
                self.triangolo_rosso += 1
            time.sleep(1)
            self.robot.motors.motors(50, 50)
            time.sleep(0.5)

            self.triangolo_vicino_counter = 0
            self.robot.servo.vivi_default()
            self.robot.servo.morti_default()
            self.robot.motors.motors(40, 0)
            time.sleep(2)

    def loop_triangoli(self):
        self.triangolo_vicino_counter = 0
        self.tipo_triangolo = 0
        while True:
            frame = self.robot.get_frame()
            self.output = frame.copy()
            
            triangolo = self.get_selected_triangolo(frame)
            if triangolo is not None:
                self.timer_stuck = time.time() # reset timer (not stuck)
                
                self.tipo_triangolo += -1 if triangolo[-1] == 0 else 1
                
                centro_triangolo = (triangolo[0]+(triangolo[2]//2), triangolo[1]+(triangolo[3]//2))
                bordi = scan_bordi(frame)
                
                punto_bordo_vicino_triangolo = get_nearest_countourn_point(bordi, centro_triangolo)
                distanza_bordi_triangolo = np.linalg.norm(np.array(centro_triangolo) - punto_bordo_vicino_triangolo)
                
                self.output[bordi == 255] = (20, 200, 200)
                print("[loop_triangoli()] ", triangolo, "bordi", distanza_bordi_triangolo," triangolo vicino ", self.triangolo_vicino_counter, "  tof ", self.robot.get_tof_mesures()[1], "  tipo_triangolo  ", self.tipo_triangolo)
                self.raggiungi_triangolo(triangolo)
                
            else:
                # BORDI
                self.tipo_triangolo = 0
                mode_giro_bordi = 'altissimo' 
                self.giro_bordi(frame, mode_giro_bordi)
                if time.time() - self.timer_stuck > 5:
                    self.check_if_stuck()
                    self.timer_stuck = time.time() # reset timer
                
            if self.triangolo_rosso >= 1 and self.triangolo_verde >= 1:
                break
            
            cv2.imshow("output", self.output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                self.robot.servo.deinit_pca()
                cv2.destroyAllWindows()
                exit()
    
    def trova_buco_uscita(self):
        self.robot.motors.motors(0, 0)
        time.sleep(10)
        """
        self.robot.servo.cam_linea()
        self.robot.motors.motors(60, 60)
        time.sleep(1)
        self.robot.motors.motors(50, -50)
        time.sleep(1.5)
        
        buco_first_value = None
        buco_last_value = None
        self.robot.motors.motors(20, -20)
        t_inizio = time.time()
        while time.time() - t_inizio < 8:
            tof_front = self.robot.get_tof_mesures()[2]
            if tof_front > 600 and buco_first_value is None:
                buco_first_value = time.time() - t_inizio
                print("first", buco_first_value)
            
            if tof_front < 600 and buco_first_value is not None and buco_last_value is None:
                buco_last_value = time.time() - t_inizio
                print("last",buco_last_value)

        buco_mid_time = 8 - buco_last_value + (buco_last_value - buco_first_value)/2
        print("midddd",buco_mid_time)
        self.robot.motors.motors(-20, 20)
        time.sleep(buco_mid_time)
        """
        return True
    

    def get_selected_uscita(self, frame):
        # detect triangolo with tensor
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector_uscita.detect(input_tensor)
        # add to array and draw triangolo on the frame
        uscite = []
        for d in detection_result.detections:
            print(d)
            x, y, w, h, index = d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height, d.categories[0].index
            uscite.append((x,y,w,h, index))
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,0), 1)
        # return the bigger triangolo
        if len(uscite) > 0:
            uscite.sort(key=lambda b : b[2]*b[3], reverse=True)
            x, y, w, h, _ = uscite[0]
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (255,0,255), 4)
            return uscite[0]
    
    
    def centro_striscia(self, striscia):
        M = cv2.moments(striscia)
        if M["m00"] == 0: return None
        striscia_x = int(M["m10"] / M["m00"])
        striscia_y = int(M["m01"] / M["m00"])        
        return striscia_x, striscia_y
    
    def controllo_tipo_uscita(self):
        self.robot.motors.motors(0,0)
        
        striscia_persa_counter = 0
        t_inizio = time.time()
        while True:
            frame = self.robot.get_frame().copy()
            
            mask_nero, mask_bianco, mask_verde = scan(frame)
            mask_nearest_area = get_nearest_area_from_2points(mask_nero, (136//2, 0), (136//2, 137))
            cut = mask_nearest_area[0:20, :]                        
            end_point = self.centro_striscia(cut)
            
            if striscia_persa_counter >= 2000:
                break
            if end_point is None:
                striscia_persa_counter += 1
                continue
            striscia_persa_counter = 0
            
            errore_linea = end_point[0] - (136//2)
            P = int(errore_linea*1)
            self.robot.motors.motors(25 + P, 25 - P)
            cv2.circle(frame, end_point, 5, (100, 0, 255), 3)
            cv2.imshow("trova_striscia", frame)
            key = cv2.waitKey(1) & 0xFF
            if self.robot.cavo_sinistra.is_pressed or self.robot.cavo_destra.is_pressed:
                break
            if time.time() - t_inizio > 3.5:
                return True

        self.robot.servo.cam_EZ()
        self.robot.camstream_EZ()
        self.robot.motors.motors(-50,-50)
        time.sleep(2)
        self.robot.motors.motors(-50, 50)
        time.sleep(2)
        return False
        
    def raggiungi_uscita(self, uscita):
        
        centro_uscita = (uscita[0]+(uscita[2]//2), uscita[1]+(uscita[3]//2))
        errore_x = uscita[0] + (uscita[2]//2) - (self.LARGHEZZA//2)
        
        speed = 70
        self.robot.motors.motors(speed + (errore_x), speed - (errore_x))
        
        # controllo che la y+h del uscita sia bassa quindi vicina al robot
        print('[USCITA] y+h ', uscita[1]+uscita[3])
        if uscita[1]+uscita[3] > 100:
            self.uscita_vicina_counter += 1
        else:
            self.uscita_vicina_counter = 0
            
        if self.uscita_vicina_counter > 5:
            self.robot.motors.motors(0,0)
            return True
        
    def loop_uscita(self):
        self.uscita_vicina_counter = 0
        uscita_persa_count = 0
        while True:
            frame = self.robot.get_frame()
            self.output = frame.copy()
            
            uscita = self.get_selected_uscita(frame)

            if uscita:
                # USCITA TROVATA

                if self.raggiungi_uscita(uscita):
                    print("uscita trovata")
                    self.robot.motors.motors(0, 0)
                    self.robot.camstream_linea()
                    self.robot.servo.cam_linea()
                    time.sleep(1)
                    if self.controllo_tipo_uscita():
                        break
                    
            else:
                # BORDI
                self.uscita_vicina_counter = 0
                tipo_uscita = 0
                self.giro_bordi(frame, 'alto')
                if time.time() - self.timer_stuck > 5:
                    self.check_if_stuck()
                    self.timer_stuck = time.time() # reset timer
                
            cv2.imshow("output", self.output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                self.robot.servo.deinit_pca()
                cv2.destroyAllWindows()
                exit()
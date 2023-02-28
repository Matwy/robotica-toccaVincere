import sys
import time

import cv2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

from cvtools import scan_bordi, sort_aree, get_nearest_countourn_point

class EZ:
    Y_ABBASSA_BRACCIO = 35
    Y_BECCO_SOPRA = 64
    Y_BECCO_SOTTO = 90
    MODELLO_PALLE = 'giugia_pestami_le_palle.tflite' # le mie non le vittime
    MODELLO_TRIANGOLI = 'giugia_leccami_il_triangolo.tflite'
    
    def __init__(self, robot):
        robot.motors.motors(30, 30) # entra un po' durante il setup
        cv2.destroyAllWindows()

        # Initialize the object detection model
        base_options = core.BaseOptions(file_name=self.MODELLO_PALLE, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.25)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_palle = vision.ObjectDetector.create_from_options(options)
        
        base_options = core.BaseOptions(file_name=self.MODELLO_TRIANGOLI, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.6)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_triangoli = vision.ObjectDetector.create_from_options(options)
        
        self.robot = robot
        self.robot.servo.cam_EZ()
        self.robot.servo.pinza_su()
        self.robot.servo.vivi_default()
        self.robot.servo.morti_default()
        
        # change camera settings
        self.robot.camstream_EZ()
        time.sleep(0.5)
        self.LARGHEZZA, self.ALTEZZA = robot.cam_stream.camera.resolution
        
        self.pinza_su = True
        self.output = None
        self.palline_vive = 0
        self.palline_morte = 0
        self.triangolo_rosso = 0
        self.triangolo_verde = 0
        
        robot.motors.motors(0, 0)
    
    def giro_bordi(self, frame, mode):
        bordi = scan_bordi(frame)
        
        aree_muri_pavimento = cv2.bitwise_not(bordi)
        amount, labels = cv2.connectedComponents(aree_muri_pavimento)
        
        if amount == 1: self.robot.motors.motors(80, 30) # destra

        aree_muri_pavimento = sort_aree(amount, labels, 1, _blank='ez')
        if len(aree_muri_pavimento) > 0:
           area_bassa = aree_muri_pavimento[-1]
        else:
           return None
        
        if area_bassa is None: return
        if mode == 'alto':
            roi_pt1, roi_pt2 = (0, 43), (self.LARGHEZZA, 45)
        else:
            roi_pt1, roi_pt2 = (0, 60), (self.LARGHEZZA, 65)
            
        roi = area_bassa[roi_pt1[1] : roi_pt2[1], roi_pt1[0] : roi_pt2[0]]
        cv2.rectangle(self.output, roi_pt1, roi_pt2, (125, 125, 7), 2)
        cv2.imshow("bordi", roi)
        
        if np.count_nonzero(roi == 0) < 100:
            #destra
            self.robot.motors.motors(80, 30)
            return

        roi_mask_bordi = cv2.bitwise_not(roi)
        M = cv2.moments(roi_mask_bordi)
        if M["m00"] == 0: return
        cX = int(M["m10"] / M["m00"])
        
        errore_bordo = self.LARGHEZZA-cX
        if errore_bordo < self.LARGHEZZA // 3.5:
            # sinistra piano
            self.robot.motors.motors(20, 4*errore_bordo)
        else:
            # sinistra pott
            self.robot.motors.motors(-30, 50)
            
    
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
        speed = 50 if ball_y < self.Y_ABBASSA_BRACCIO else 20
        errore_x = ball_x - self.ALTEZZA//1.5
        self.robot.motors.motors(speed + int(errore_x//2), speed - int(errore_x//2))
        if ball_y > self.Y_ABBASSA_BRACCIO and self.pinza_su:
            # abbassa pinza se la palla è vicina e la pinza è su
            
            if self.robot.get_tof_mesures()[1] < 200:
                # robot troppo vicino al muro di destra quindi fai manovra
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                self.robot.motors.motors(-60, 60)
                time.sleep(0.7)
                self.robot.motors.motors(80, 80)
                time.sleep(1)
                self.robot.motors.motors(60, -60)
                time.sleep(1.3)
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                return
            self.robot.motors.motors(0, 0)
            self.robot.servo.pinza_giu()
            self.robot.servo.becco_aperto()
            self.pinza_su = False
        
        if ball_y > self.Y_BECCO_SOPRA and ball_y < self.Y_BECCO_SOTTO and abs(errore_x) < 10 and not self.pinza_su:
            # raccogli palla se è dentro i becchi e al centro dello schermo e la pinza è giu
            self.robot.motors.motors(20, 20)
            time.sleep(0.5)
            self.robot.servo.becco_chiuso()
            time.sleep(0.3)
            self.robot.motors.motors(-20, -20)
            self.robot.servo.pinza_su()
            time.sleep(1)
            if tipo_palla <= 0: # distinzione morte vive
                self.robot.servo.becco_molla_morti()
                self.palline_morte += 1
            else:
                self.robot.servo.becco_molla_vivi()
                self.palline_vive += 1
            self.robot.servo.becco_aperto()
            self.pinza_su = True
            tipo_palla = 0
    
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
                # PALLE 
                palla_persa_count = 0
                if ball[1] > 30:
                    tipo_palla += 1 if ball[-1] == 1 else -1
                print("[EZ] loop_palle() palla: ", ball, "tipo_palla ", tipo_palla, " tof ", self.robot.get_tof_mesures()[1])
                self.raccogli_palla(ball, tipo_palla)
            elif self.pinza_su:
                # BORDI
                tipo_palla = 0
                mode_giro_bordi = 'alto' #if time.time() - t_inizio_ricerca < 20 else 'basso' 
                self.giro_bordi(frame, mode_giro_bordi)
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
        speed = 50
        errore_x = triangolo[0] + (triangolo[2]//2) - (self.ALTEZZA//2)
        self.robot.motors.motors(speed + (errore_x), speed - (errore_x))
        # controllo che la y+h del triangolo sia bassa quindi vicina al robot
        if triangolo[1]+triangolo[3] > 90:
            print("sassi")
            self.triangolo_vicino_counter += 1
        else:
            self.triangolo_vicino_counter = 0
            
        if self.triangolo_vicino_counter > 5:
            
            if self.robot.get_tof_mesures()[1] < 250:
                # robot troppo vicino al muro di destra quindi fai manovra
                self.robot.motors.motors(-50, -50)
                time.sleep(1)
                self.robot.motors.motors(-60, 60)
                time.sleep(0.7)
                self.robot.motors.motors(80, 80)
                time.sleep(1)
                self.robot.motors.motors(60, -60)
                time.sleep(0.7)
                self.robot.motors.motors(-50, -50)
                time.sleep(0.5)
                self.triangolo_vicino_counter = 0
                return
            
            self.robot.motors.motors(25, 25)
            time.sleep(4)
            self.robot.motors.motors(-40, -40)
            time.sleep(2)
            self.robot.motors.motors(-80, 80)
            time.sleep(2.2)
            self.robot.motors.motors(-40, -40)
            time.sleep(2.7)
            self.robot.motors.motors(-25, -25)
            self.robot.servo.pinza_svuota_cassoni()
            time.sleep(1)
            
            if triangolo[-1] == 0:
                self.robot.servo.vivi_svuota()
                self.triangolo_verde += 1
            else:
                self.robot.servo.morti_svuota()
                self.triangolo_rosso += 1
            time.sleep(1)
            self.robot.servo.vivi_default()
            self.robot.servo.morti_default()

    def loop_triangoli(self):
        self.triangolo_vicino_counter = 0
        while True:
            frame = self.robot.get_frame()
            self.output = frame.copy()
            
            triangolo = self.get_selected_triangolo(frame)
            if triangolo is not None:
                centro_triangolo = (triangolo[0]+(triangolo[2]//2), triangolo[1]+(triangolo[3]//2))
                bordi = scan_bordi(frame)
                
                punto_bordo_vicino_triangolo = get_nearest_countourn_point(bordi, centro_triangolo)
                distanza_bordi_triangolo = np.linalg.norm(np.array(centro_triangolo) - punto_bordo_vicino_triangolo)
                
                self.output[bordi == 255] = (20, 200, 200)
                print("[loop_triangoli()] ", triangolo, "bordi", distanza_bordi_triangolo," triangolo vicino ", self.triangolo_vicino_counter, "  tof ", self.robot.get_tof_mesures()[1])
                self.raggiungi_triangolo(triangolo)
                
            else:
                # BORDI
                triangolo_checked = False
                mode_giro_bordi = 'alto' 
                self.giro_bordi(frame, mode_giro_bordi)
                
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
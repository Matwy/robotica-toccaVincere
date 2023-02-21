import sys
import time

import cv2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

from cvtools import scan_bordi, sort_aree

class EZ:
    Y_ABBASSA_BRACCIO = 35
    Y_BECCO_SOPRA = 53
    Y_BECCO_SOTTO = 90
    MODELLO_PALLE = 'giugia_pestami_le_palle.tflite' # le vittime non le mie palle
    
    def __init__(self, robot):
        cv2.destroyAllWindows()

        # Initialize the object detection model
        base_options = core.BaseOptions(file_name=self.MODELLO_PALLE, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector = vision.ObjectDetector.create_from_options(options)
        
        self.robot = robot
        self.robot.motors.motors(0, 0)
        # change camera settings
        self.robot.servo.cam_EZ()
        self.robot.camstream_EZ()
        time.sleep(0.5)
        self.LARGHEZZA, self.ALTEZZA = robot.cam_stream.camera.resolution
        self.pinza_su = True
        self.output = None
    
    def giro_bordi(self, frame):
        
        bordi = scan_bordi(frame)
        aree_muri_pavimento = cv2.bitwise_not(bordi)
        amount, labels = cv2.connectedComponents(aree_muri_pavimento)
        
        if amount == 1: self.robot.motors.motors(50, 30) # destra

        area_bassa = sort_aree(amount, labels, 1, _blank='ez')[-1]
        
        roi_pt1, roi_pt2 = (0, 60), (self.LARGHEZZA, 65)
        roi = area_bassa[roi_pt1[1] : roi_pt2[1], roi_pt1[0] : roi_pt2[0]]
        cv2.rectangle(self.output, roi_pt1, roi_pt2, (125, 125, 7), 2)
        cv2.imshow("bordi", roi)
        
        if np.count_nonzero(roi == 0) < 100:
            #destra
            self.robot.motors.motors(50, 20)
            return

        roi_mask_bordi = cv2.bitwise_not(roi)
        M = cv2.moments(roi_mask_bordi)
        if M["m00"] == 0: return
        cX = int(M["m10"] / M["m00"])
        
        errore_bordo = self.LARGHEZZA-cX
        if errore_bordo < self.LARGHEZZA // 3.5:
            # sinistra piano
            self.robot.motors.motors(30, 50)
        else:
            # sinistra pott
            self.robot.motors.motors(-30, 50)
            
    
    def get_selected_ball(self, frame):
        # detect ball with tensor
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector.detect(input_tensor)
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
    
    def raccogli_palla(self, ball):
        # PID tenendo in considerazione la distanza della palla
        speed = 40 if ball[1] < self.Y_ABBASSA_BRACCIO else 20
        errore_x = ball[0] - self.ALTEZZA//2
        self.robot.motors.motors(speed + (errore_x), speed - (errore_x))
        
        if ball[1] > self.Y_ABBASSA_BRACCIO and self.pinza_su:
            # abbassa pinza se la palla è vicina
            self.robot.motors.motors(0, 0)
            self.robot.servo.pinza_giu()
            self.robot.servo.becco_aperto()
            self.pinza_su = False
        
        if ball[1] > self.Y_BECCO_SOPRA and ball[1] < self.Y_BECCO_SOTTO and abs(errore_x) < 10 and not self.pinza_su:
            # raccogli palla se è dentro i becchi
            self.robot.servo.becco_chiuso()
            time.sleep(0.3)
            self.robot.motors.motors(-20, -20)
            self.robot.servo.pinza_su()
            time.sleep(1)
            if ball[4] == 0: # distinzione morte vive
                self.robot.servo.becco_molla_morti()
            else:
                self.robot.servo.becco_molla_vivi()
            self.robot.servo.becco_aperto()
            self.pinza_su = True
    
    def loop_palle(self):
        palla_persa_count = 0
        while True:
            frame = self.robot.get_frame()
            self.output = frame.copy()
            
            ball = self.get_selected_ball(frame)
            cv2.circle(self.output, (self.LARGHEZZA//2, self.Y_BECCO_SOPRA), 5, (0, 0, 255), -1)
            cv2.circle(self.output, (self.LARGHEZZA//2, self.Y_BECCO_SOTTO), 5, (0, 0, 255), -1)
            if ball:
                # PALLE 
                palla_persa_count = 0
                print("[EZ] loop_palle() palla: ", ball)
                self.raccogli_palla(ball)
            elif self.pinza_su:
                # BORDI
                self.giro_bordi(frame)
            else:
                self.robot.motors.motors(-30, -30)
                palla_persa_count += 1

            if palla_persa_count > 300:
                self.robot.servo.pinza_su()
                self.pinza_su = True
                                
            cv2.imshow("output", self.output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.robot.motors.motors(0, 0)
                self.robot.cam_stream.stop()
                self.robot.sensors_stream.stop()
                self.robot.servo.deinit_pca()
                cv2.destroyAllWindows()
                break
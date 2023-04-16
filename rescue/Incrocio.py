import cv2
import numpy as np
from global_var import ALTEZZA, LARGHEZZA
from cvtools import scan, get_bigger_area, get_centri_aree, get_collisioni_with_angles, calcola_inizio_linea, get_valid_verdi, get_n_aree_biance, get_nearest_countourn_point
from gap_dopio import doppio_verde
import time
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

OFFSET_TARGET_INCROCIO = 20
TARGET_DOPPIOVERDE = ALTEZZA - 40

class Incrocio:
    @staticmethod
    def get_incrocio(amount_bianco, labels_bianco):
        kernel=np.ones((60, 60),np.uint8)
        mask_somma_aree=BLANK.copy()
        

        #dilato tutte le aree e trovo dove si intersecano        
        for i in range(1, amount_bianco):
            #scorro le singole aree (maskArea è l'area singola)
            maskArea = BLANK.copy()
            maskArea[labels_bianco == i] = 255
            #la dilato in modo da farla andare anche sopra le altre aree 
            maskArea = cv2.dilate(maskArea, kernel, iterations = 1)
            maskArea //= 5 #dove c'è il bianco è 255 ora sarà 255//5 = 51
            mask_somma_aree += maskArea #sommo tutte le aree in una maschera

        #il valore più alto sarà dove si sono intersecate più aree quindi l'incrocio
        #max_val = np.max(mask_somma_aree) 
        mask_incrocio = BLANK.copy()
        mask_incrocio[mask_somma_aree > 103] = 255
        
        amount_incroci, labels_incroci = cv2.connectedComponents(mask_incrocio)
        
        centri_incroci = get_centri_aree(amount_incroci, labels_incroci)
        centri_incroci.sort(key= lambda i:i[1], reverse=True) #il più basso
        if len(centri_incroci) > 0:
            return centri_incroci[0]
        # no incrocio 
        return None


    MODELLO_VERDE = 'verde_ef0.tflite' # le mie non le vittime

    def __init__(self, robot):
        self.robot = robot
        self.centro = None
        self.is_centered = False
        self.end_point = None
        self.uscita_counter = 0
        self.doppioverde = False
        # Initialize the object detection model
        base_options = core.BaseOptions(file_name=self.MODELLO_VERDE, use_coral=False, num_threads=4)
        detection_options = processor.DetectionOptions(max_results=4, score_threshold=0.80)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector_verdi = vision.ObjectDetector.create_from_options(options)
    
    def get_verdi_with_tensor(self, frame):
        # detect ball with tensor
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector_verdi.detect(input_tensor)
        # add to array and draw ball on the frame
        verdi = []
        for d in detection_result.detections:
            x, y, w, h, index = d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height, d.categories[0].index
            if w*h  > 2000:
                continue
            print(d)
            verdi.append((x+(w//2),y+(h//2)))
            cv2.rectangle(self.output, (x,y), (x+w, y+h), (0,255,0), -1)
            cv2.circle(self.output, (x+(w//2),y+(h//2)), 4, (70,10, 200), 2)

        return verdi
    
    def calcolo_fine_incrocio(self, mask_nero, amount_bianco, labels_bianco, raw_verdi):
        # PUNTO BASSO
        puntoL, puntoR = calcola_inizio_linea(mask_nero, amount_bianco, labels_bianco)
        punto_basso = ((puntoL[0] + puntoR[0]) // 2), ((puntoL[1] + puntoR[1]) // 2)
        self.robot.last_punto_basso = punto_basso
        # COLLISIONE MINORE
        collisioni_bordo = get_collisioni_with_angles(mask_nero, punto_basso, self.centro)
        if len(collisioni_bordo) == 0: return (LARGHEZZA//2, 0)
        collisione_meno_ampia = collisioni_bordo[0][1]
        cv2.circle(self.output, collisione_meno_ampia, 10, (30,200, 50), 2)
        # VERDE
        verdi = get_valid_verdi(raw_verdi, self.centro, collisione_meno_ampia)
        for v in verdi:
            cv2.circle(self.output, v, 5, (200,50, 100), 2)
        
        end_point = None
        if len(verdi) == 0:
            end_point = collisioni_bordo[-1][1]
        
        elif len(verdi) == 1:
            verde_dx = False if verdi[0][0] < self.centro[0] else True # verde_dx = True verde a destra
            collisioni_bordo.sort(key= lambda c : c[1][0], reverse=verde_dx) # ordino le collisioni in base alle loro x        ad esempio se il verde a destra prendo la collisione con la x più grande 
            end_point = collisioni_bordo[0][1]
            
        elif len(verdi) == 2:
            print("[INCROCIO] DOPPIOVERDE verdi:", verdi, "centro: ", self.centro)
            self.robot.motors.motors(0,0)
            self.doppioverde = True
            end_point = (LARGHEZZA//2, 0)
            
        cv2.circle(self.output, end_point, 10, (50,50, 255), 2)
        return end_point
    def centra_incrocio(self):
        # centra l'incrocio in modo proporzionale
        errore_x = self.centro[0] - (LARGHEZZA//2)
        errore_y = (ALTEZZA//2) - self.centro[1]
        Px = int(errore_x*1.3)
        Py = int(errore_y*0.7)
        if errore_y > 0:
            self.robot.motors.motors(abs(Py) + Px, abs(Py) - Px)                
        else:
            self.robot.motors.motors(-abs(Py) + Px, -abs(Py) - Px) 
        
        if abs(errore_x) < OFFSET_TARGET_INCROCIO and abs(errore_y) < OFFSET_TARGET_INCROCIO:
            self.is_centered = True
            pass
        
    def loop_centra_incrocio(self):
        while True:
            self.output = BLANK_COLORI.copy()
            frame = self.robot.get_frame().copy()
            mask_nero, mask_bianco, mask_verde = scan(frame)
            mask_nero = get_bigger_area(mask_nero)

            self.output[mask_nero == 255] = 0
            self.output[mask_verde == 255] = (0,255,0)
            
            amount_bianco, labels_bianco = cv2.connectedComponents(mask_bianco)
            n_aree_bianche_senza_loli = get_n_aree_biance(amount_bianco, labels_bianco)
            # CENTRO INCROCIO 
            _centro = Incrocio.get_incrocio(amount_bianco, labels_bianco)
            if self.centro is None: self.centro = _centro
            # controlli per uscire
            
            if self.uscita_counter > 5:
                return
            
            if _centro is None:
                self.uscita_counter += 1
                print('[INCROCIO] USCITA _centro None')
                continue
            
            if self.centro is None or np.linalg.norm(np.array(self.centro) - np.array(_centro)) > 50:
                self.uscita_counter += 1
                print('[INCROCIO] USCITA', self.centro is None, n_aree_bianche_senza_loli <= 2, np.linalg.norm(np.array(self.centro) - np.array(_centro)))
                continue
            
            self.uscita_counter = 0
            self.centro = _centro
            
            if self.doppioverde:
                sp, kp = 20, 4
                errore = self.centro[0] - (LARGHEZZA//2)
                self.robot.motors.motors(sp + int(errore*kp), sp - int(errore*kp))
                if self.centro[1] > TARGET_DOPPIOVERDE:
                    #  MANOVRA 180° #
                    self.robot.motors.motors(40, 40)
                    time.sleep(1)
                    #gira di 180°
                    self.robot.motors.motors(-70, 70)
                    time.sleep(2.5)

                    self.robot.motors.motors(-40, -40)
                    time.sleep(0.5)
                    cv2.destroyAllWindows()
                
                cv2.circle(self.output, self.centro, 15, (0,0,255), 2)
                cv2.imshow('output', self.output)
                key = cv2.waitKey(1) & 0xFF
                continue
            
            if not self.is_centered and self.centro[1] < ALTEZZA - 30:
                #    FASE 1    #
                # Centra incrocio se non è centrato e l'incrocio è alto
                print('[INCROCIO] CENTRA INCROCIO')
                self.centra_incrocio()
                
            elif self.end_point is not None:
                #    FASE 3    #
                # se ho già trovato l'end point seguilo
                self.end_point = get_nearest_countourn_point(mask_nero, self.end_point)
                # print('[INCROCIO] RAGGIUNGI ENDPOINT ', self.end_point)
                if self.end_point is None: return
                cv2.circle(self.output, self.end_point, 10, (50,50, 255), 2)
                self.robot.last_punto_basso = self.centro
                self.robot.last_punto_alto = self.end_point
                
                errore_end_point = self.end_point[0] - (LARGHEZZA//2)
                errore_centro = self.centro[0] - (LARGHEZZA//2)
                        
                P, D= int(errore_centro*0.4), int(errore_end_point*1)
                self.robot.motors.motors(20 + (P+D), 20 - (P+D))                    
                    
            else:
                #    FASE 2    #
                # faccio tre foto all'incrocio e per ognuna uso il modello di tensor
                # rimuovo i verdi ripetuti e quelli troppo grandi
                self.robot.motors.motors(0,0)
                self.robot.servo.pinza_su()
                raw_verdi = self.get_verdi_with_tensor(self.robot.get_frame().copy())
                
                # cv2.imshow('output', self.output)
                # cv2.waitKey(1)
                # time.sleep(5)
                # verdi_not_repeated = remove_repeated_and_big_verdi(raw_verdi)
                self.end_point = self.calcolo_fine_incrocio(mask_nero, amount_bianco, labels_bianco, raw_verdi)
                print('[INCROCIO] CALCOLO ENDPOINT ', self.end_point)
                

            cv2.circle(self.output, self.centro, 15, (0,0,255), 2)
            cv2.imshow('output', self.output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                robot.motors.motors(0, 0)
                robot.cam_stream.stop()
                robot.sensors_stream.stop()
                cv2.destroyAllWindows()
                robot.servo.deinit_pca()
                exit()   
            
import cv2
import numpy as np
from global_var import ALTEZZA, LARGHEZZA
from cvtools import scan, get_bigger_area, get_centri_aree, get_collisioni_with_angles, calcola_inizio_linea

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

OFFSET_TARGET_INCROCIO = 10

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
        cv2.imshow("incroci", mask_somma_aree)
        mask_incrocio = BLANK.copy()
        mask_incrocio[mask_somma_aree > 103] = 255
        
        amount_incroci, labels_incroci = cv2.connectedComponents(mask_incrocio)
        
        centri_incroci = get_centri_aree(amount_incroci, labels_incroci)
        centri_incroci.sort(key= lambda i:i[1], reverse=True) #il più basso
        if len(centri_incroci) > 0:
            return centri_incroci[0]
        # no incrocio 
        return None

    def __init__(self, robot):
        cv2.destroyAllWindows()
        self.robot = robot
    
    def calcolo_fine_incrocio(self, mask_nero, amount_bianco, labels_bianco):
        # PUNTO BASSO
        puntoL, puntoR = calcola_inizio_linea(mask_nero, amount_bianco, labels_bianco)
        punto_basso = ((puntoL[0] + puntoR[0]) // 2), ((puntoL[1] + puntoR[1]) // 2)
        # COLLISIONE MINORE
        collisione_linea_incrocio = get_collisioni_with_angles(mask_nero, punto_basso, self.centro)[0][1]
        cv2.circle(self.output, collisione_linea_incrocio, 10, (30,200, 50), 2)
        
    def loop_centra_incrocio(self):
        incrocio_perso = 0
        while True:
            self.output = BLANK_COLORI.copy()
            frame = self.robot.get_frame().copy()
            mask_nero, mask_bianco, mask_verde = scan(frame)
            mask_nero = get_bigger_area(mask_nero)

            self.output[mask_nero == 255] = 0
            self.output[mask_verde == 255] = (0,255,0)
            
            amount_bianco, labels_bianco = cv2.connectedComponents(mask_bianco)
            print("[INCROCIO] aree_bianche", amount_bianco-1)
            # CENTRO INCROCIO 
            self.centro = Incrocio.get_incrocio(amount_bianco, labels_bianco)
            if incrocio_perso > 5:
                return
            if self.centro is None:
                incrocio_perso += 1
                self.robot.motors.motors(0,0)
                continue
            
            # centra l'incrocio in modo proporzionale
            errore_x = self.centro[0] - (LARGHEZZA//2)
            Px = int(errore_x*1.5)
            errore_y = (ALTEZZA//2) - self.centro[1]
            if errore_y > 0:
                self.robot.motors.motors(abs(errore_y) + Px, abs(errore_y) - Px)                
            else:
                self.robot.motors.motors(-abs(errore_y) + Px, -abs(errore_y) - Px)  
            
            if abs(errore_x) < OFFSET_TARGET_INCROCIO and abs(errore_y) < OFFSET_TARGET_INCROCIO:
                self.calcolo_fine_incrocio(mask_nero, amount_bianco, labels_bianco)

            print("[INCROCIO] correzione", errore_x, errore_y)
            cv2.circle(self.output, self.centro, 15, (0,0,255), 2)
            cv2.imshow('incriocio', self.output)
            cv2.imshow('frame', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                robot.motors.motors(0, 0)
                robot.cam_stream.stop()
                robot.sensors_stream.stop()
                cv2.destroyAllWindows()
                robot.servo.deinit_pca()
                exit()   
            
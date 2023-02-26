import cv2
import numpy as np
from global_var import ALTEZZA, LARGHEZZA
from cvtools import scan, get_bigger_area, get_centri_aree, get_collisioni_with_angles, calcola_inizio_linea, get_points_verdi, get_n_aree_biance
from gap_dopio import doppio_verde
import time
BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

OFFSET_TARGET_INCROCIO = 20

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
        self.is_centered = False
    
    def calcolo_fine_incrocio(self, mask_nero, amount_bianco, labels_bianco, mask_verde):
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
        verdi = get_points_verdi(mask_verde, self.centro, collisione_meno_ampia)
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
            doppio_verde(self.robot)
            end_point = (LARGHEZZA//2, 0)
            
        cv2.circle(self.output, end_point, 10, (50,50, 255), 2)
        self.robot.last_punto_alto = end_point
        return end_point
    def centra_incrocio(self):
        # centra l'incrocio in modo proporzionale
        errore_x = self.centro[0] - (LARGHEZZA//2)
        errore_y = (ALTEZZA//2) - self.centro[1]
        Px = int(errore_x*2)
        Py = int(errore_y*1)
        if errore_y > 0:
            self.robot.motors.motors(abs(Py) + Px, abs(Py) - Px)                
        else:
            self.robot.motors.motors(-abs(Py) + Px, -abs(Py) - Px) 
        
        if abs(errore_x) < OFFSET_TARGET_INCROCIO and abs(errore_y) < OFFSET_TARGET_INCROCIO:
            self.is_centered = True
        
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
            n_aree_bianche_senza_loli = get_n_aree_biance(amount_bianco, labels_bianco)
            # CENTRO INCROCIO 
            self.centro = Incrocio.get_incrocio(amount_bianco, labels_bianco)
            if incrocio_perso > 5:
                return
            if self.centro is None or n_aree_bianche_senza_loli <= 2:
                incrocio_perso += 1
                continue
            
            if not self.is_centered and self.centro[1] < ALTEZZA - 50:
                self.centra_incrocio()
            else:
                end_point = self.calcolo_fine_incrocio(mask_nero, amount_bianco, labels_bianco, mask_verde)
                errore_end_point = end_point[0] - (LARGHEZZA//2)
                errore_centro = self.centro[0] - (LARGHEZZA//2)
                        
                P, D= int(errore_centro*0.5), int(errore_end_point*1)
                self.robot.motors.motors(15 + (P+D), 15 - (P+D))

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
            
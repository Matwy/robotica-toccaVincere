import cv2
from global_var import ALTEZZA, LARGHEZZA
from cvtools import scan

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

class Incrocio:
    @staticmethod
    def get_incrocio(mask_bianco):
        kernel=np.ones((60, 60),np.uint8)
        mask_somma_aree=BLANK.copy()
        
        amount_bianco, labels_bianco = cv2.connectedComponents(mask_bianco)

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
        
        amount_bianco, labels_bianco = cv2.connectedComponents(mask_incrocio)
        
        centri_incroci = get_centri_aree(amount_bianco, labels_bianco)
        print("[INCROCIO] n_incroci", len(centri_incroci))
        centri_incroci.sort(key= lambda i:i[1], reverse=True) #il più basso
        if len(centri_incroci) > 0:
            return centri_incroci[0]
        # no incrocio 
        return None

    def __init__(self, robot):
        self.robot = robot
    
    def loop_centra_incrocio(self):
        incrocio_perso = 0
        while True:
            self.output = BLANK_COLORI.copy()
            frame = self.robot.get_frame().copy()
            mask_nero, mask_bianco, mask_verde = scan(frame)
            mask = get_bigger_area(mask_nero)
            # aggiorno l'output
            self.output[mask == 255] = 0
            self.output[mask_verde == 255] = (0,255,0)
            
            self.centro = Incrocio.get_incrocio(mask_bianco)
            if incrocio_perso > 5:
                return
            if self.centro is None:
                incrocio_perso += 1
                continue
            
            cv2.circle(self.output, self.centro, 30, (0,0,255), 2)
            cv2.imshow('incriocio', self.output)
            if key == ord("q"):
                robot.motors.motors(0, 0)
                robot.cam_stream.stop()
                robot.sensors_stream.stop()
                cv2.destroyAllWindows()
                robot.servo.deinit_pca()
                exit()   
            
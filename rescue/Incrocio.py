import cv2
from global_var import ALTEZZA, LARGHEZZA

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')

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
        
        amount_bianco, labels_bianco = cv2.connectedComponents(mask_incrocio)
        
        centri_incroci = get_centri_aree(amount_bianco, labels_bianco)
        print("n_incroci", len(centri_incroci))
        centri_incroci.sort(key= lambda i:i[1], reverse=True)
        if len(centri_incroci) > 0:
            return centri_incroci[0]
        
        print("problema")
        #c'è stato un problema con il calcolo del centro
        return None

    def __init__(self, robot):
        self.robot = robot
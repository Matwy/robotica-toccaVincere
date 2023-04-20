import cv2
import numpy as np
import time
from cvtools import get_punto_alto, getAngle, scan, get_bigger_area, calcola_inizio_linea, get_n_aree_biance, get_nearest_area_from_2points
#incroci
from Incrocio import Incrocio
from cvtools import get_punto_basso, get_centro_incrocio, get_points_verdi, get_collisioni_with_angles,get_collisione_90, rimuovi_collisioni, taglio_verde_singolo
#gap e doppioverde
from gap_dopio import doppio_verde, gap
from global_var import ALTEZZA, LARGHEZZA

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

doppio_verde_counter = 0

def linea(frame, robot):
    global doppio_verde_counter
    output = BLANK_COLORI.copy()
    
    #pulisco l'immagine
    #trovo quindi gli elementi che vogli e li metto in mask separate
    mask_nero, mask_bianco, mask_verde = scan(frame)
    
    #decido che area nera prendere
    mask_nearest_area = get_nearest_area_from_2points(mask_nero, robot.last_punto_alto, robot.last_punto_basso)
    mask_bianco = cv2.bitwise_not(mask_nearest_area)
    #DA RICONTROLLARE SE CI SONO PROVLEMI LA MASK POTREBBE ESSERE VUOTA
    
    #aggiorno l'output
    output[mask_nearest_area == 255] = 0
    output[mask_verde == 255] = (0,255,0)
    cv2.circle(output, robot.last_punto_alto, 3, (0, 90, 19), 5)
    cv2.circle(output, robot.last_punto_basso, 3, (0, 90, 19), 5)
    
    #trovo le aree bianche
    amount_bianco, labels_bianco = cv2.connectedComponents(mask_bianco)
    #scarta le aree bianche che sono troppo piccole se non c'è verde (decidi se farlo prima o dopo il controllo del gap)
    if np.count_nonzero(mask_verde) < 1000:
        n_aree_bianche = get_n_aree_biance(amount_bianco, labels_bianco)
    else:
        n_aree_bianche = amount_bianco-1
    
    """
    GAP
    """
    #se c'è solo un'area bianca c'è un gap 
    if amount_bianco < 3:
        gap(robot)
        return 0, 0, 0, 0
    #trovo l'inizio della linea 
    # punto_basso = get_punto_basso(mask_nearest_area, robot.last_punto_basso)
    cv2.circle(output, robot.last_punto_basso, 20, (0, 50, 200), 2)
    # if punto_basso is None:
    puntoL, puntoR = calcola_inizio_linea(mask_nearest_area, amount_bianco, labels_bianco)
    punto_basso = ((puntoL[0] + puntoR[0]) // 2), ((puntoL[1] + puntoR[1]) // 2)
    robot.last_punto_basso = punto_basso
    #mostro i punti
    # cv2.circle(output, puntoL, 10, (255, 0, 0), -1)
    # cv2.circle(output, puntoR, 10, (0, 0, 255), -1)
    
    #se ci sono più di due aree bianche allora 
    # c'è un incrocio scegli la strada cancellando il resto
    """
    INCROCIO
    """
    if n_aree_bianche > 2:
        #trovo centor incrocio e lo mostro
        print("[LINEA] Nuovo Incrocio")
        incrocio = Incrocio(robot).loop_centra_incrocio()
        errore_basso_x = robot.last_punto_basso[0] - (LARGHEZZA//2)
        errore_basso_y = ALTEZZA - robot.last_punto_basso[1]
        errore_alto_x = robot.last_punto_alto[0] - (LARGHEZZA//2)
        errore_alto_y = robot.last_punto_alto[1]
        return errore_basso_x, errore_basso_y, errore_alto_x, errore_alto_y

    #trova i punti alti della linea e decidi quale seguire tenendo in considerazione quello precedente

    #trovo le collisioni con la parte più alta dello schermo
    punto_alto = get_punto_alto(mask_nearest_area, robot.last_punto_alto)
    robot.last_punto_alto = punto_alto
    cv2.circle(output, punto_alto, 20, (230,230,50), 2)
    if punto_basso[1] > ALTEZZA - 30 and punto_alto[1] > ALTEZZA - 30:
        if punto_alto[0] < LARGHEZZA//2:
            robot.motors.motors(-30, -30)
        else:
            robot.motors.motors(-30, -30)
        time.sleep(0.1)
    #trova errore angolo
    errore_angolo = getAngle(punto_basso, (0, ALTEZZA), punto_alto) #angolo tra l'angolo in basso sx il punto basso della linea e la parte alta della linea
    errore_angolo = errore_angolo - 90 # 90 è il target value
    
    #trova errore linea
    # errore linea tenendo in considerazione ancheil punto alto errore_linea = ((punto_basso[0]+punto_alto[0])//2) - LARGHEZZA//2
    errore_basso_x = punto_basso[0] - (LARGHEZZA//2)
    basso_y = punto_basso[1]
    
    #ritorno (errore linea, errore angolo)
    errore_alto_x = punto_alto[0] - (LARGHEZZA//2)
    alto_y = punto_alto[1]
    
    cv2.imshow('output', output)
    return errore_basso_x, basso_y, errore_alto_x, alto_y
import cv2
import numpy as np
import time
from cvtools import get_punto_alto, getAngle, scan, get_bigger_area, calcola_inizio_linea, get_n_aree_biance
#incroci
from cvtools import get_centro_incrocio, get_points_verdi, get_collisioni_with_angles,get_collisione_90, rimuovi_collisioni, taglio_verde_singolo
#gap e doppioverde

LARGHEZZA = 320
ALTEZZA = 240

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')
BLANK_COLORI = np.full((ALTEZZA, LARGHEZZA, 3), 255, dtype='uint8')

x_last = LARGHEZZA//2
y_last = 0


def linea(frame):
    global x_last, y_last
    output = BLANK_COLORI.copy()
    
    #pulisco l'immagine
    #trovo quindi gli elementi che vogli e li metto in mask separate
    mask_nero, mask_bianco, mask_verde = scan(frame)
    
    #decido che area nera prendere
    mask = get_bigger_area(mask_nero)
    #DA RICONTROLLARE SE CI SONO PROVLEMI LA MASK POTREBBE ESSERE VUOTA
    
    #aggiorno l'output
    output[mask == 255] = 0
    output[mask_verde == 255] = (0,255,0)
    
    
    #trovo le aree bianche
    amount_bianco, labels_bianco = cv2.connectedComponents(mask_bianco)
    #scarta le aree bianche che sono troppo piccole se non c'è verde (decidi se farlo prima o dopo il controllo del gap)
    if np.count_nonzero(mask_verde) < 1000:
        n_aree_bianche = get_n_aree_biance(amount_bianco, labels_bianco)
    else:
        n_aree_bianche = amount_bianco-1
    
    """
    GAP
    
    #se c'è solo un'area bianca c'è un gap 
    if amount_bianco < 3:
        gap(motors, silver_count)
        return 0, 0
    """
    #trovo l'inizio della linea 
    puntoL, puntoR = calcola_inizio_linea(mask, amount_bianco, labels_bianco)
    punto_basso = ((puntoL[0] + puntoR[0]) // 2), ((puntoL[1] + puntoR[1]) // 2)
    #mostro i punti
    cv2.circle(output, punto_basso, 5, (150, 90, 190), 5)
    cv2.circle(output, puntoL, 10, (255, 0, 0), -1)
    cv2.circle(output, puntoR, 10, (0, 0, 255), -1)
    
    #se ci sono più di due aree bianche allora 
    # c'è un incrocio scegli la strada cancellando il resto
    """
    INCROCIO
    """
    if n_aree_bianche > 2:
        #trovo centor incrocio e lo mostro
        centro_incrocio = get_centro_incrocio(amount_bianco, labels_bianco)
        cv2.circle(output, centro_incrocio, 50, (0,0,255), 2)
        if centro_incrocio == None:
            return punto_basso[0] - (LARGHEZZA//2), 0

        #prendo le collisioni e i corrispondenti angoli 
        collisioni_angoli = get_collisioni_with_angles(mask, punto_basso, centro_incrocio)
        
        #prendo solo i verdi sotto la linea nera
        collisione_angolo_piccolo = get_collisione_90(collisioni_angoli, punto_basso) #la collisione con l'angolo simile a 90
        if collisione_angolo_piccolo == None:
            return punto_basso[0] - (LARGHEZZA//2), 0
        
        verdi = get_points_verdi(mask_verde, centro_incrocio, collisione_angolo_piccolo)
        for verde in verdi:
            cv2.circle(output, verde, 15, (0,150,255), 2)
        if len(verdi) == 0:
            #rimuove tutte le collisioni tranne la più ampia che tolgo con lo slicing
            rimuovi_collisioni(mask, output, centro_incrocio, collisioni_angoli[:-1])
        if len(verdi) == 1:
            taglio_verde_singolo(mask, output, centro_incrocio, verdi[0])
        #if len(verdi) == 2:
            #DOPPIOVERDE


    #trova i punti alti della linea e decidi quale seguire tenendo in considerazione quello precedente

    #trovo le collisioni con la parte più alta dello schermo
    punto_alto = get_punto_alto(mask, x_last, y_last)
    x_last, y_last = punto_alto
    cv2.circle(output, punto_alto, 30, (230,230,50), 2)

    #trova errore angolo
    errore_angolo = getAngle(punto_basso, (0, ALTEZZA), punto_alto) #angolo tra l'angolo in basso sx il punto basso della linea e la parte alta della linea
    errore_angolo = errore_angolo - 90 # 90 è il target value
    
    #trova errore linea
    # errore linea tenendo in considerazione ancheil punto alto errore_linea = ((punto_basso[0]+punto_alto[0])//2) - LARGHEZZA//2
    errore_linea = punto_basso[0] - (LARGHEZZA//2)
    
    #ritorno (errore linea, errore angolo)
    errori = (errore_linea, errore_angolo)
    cv2.imshow('output', output)

    return errori
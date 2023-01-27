import cv2
import numpy as np

from global_var import ALTEZZA, LARGHEZZA

SENSITIVITY = 170

lower_white = 255-SENSITIVITY
lower_green = np.array([20,50,25]) 
upper_green = np.array([90,255,230])

lower_green_EZ = np.array([30,120,40]) 
upper_green_EZ = np.array([95,255,205])

lower_red = np.array([0,70,90])
upper_red = np.array([20,255,240])
lower_red2 = np.array([130,70,90])
upper_red2 = np.array([255,255,240])

KERNEL = np.ones((5,5), np.uint8)

BLANK = np.zeros((ALTEZZA, LARGHEZZA), dtype='uint8')

#maschera del bordo sx top dx
#la uso per vedere dove "esce" la linea
MASK_BORDI = BLANK.copy()
cv2.rectangle(MASK_BORDI, (5, 5), (LARGHEZZA-5, ALTEZZA), (255), -1)
MASK_BORDI = cv2.bitwise_not(MASK_BORDI)

def scan(img):
    #rimuovo i pezzi di robot che si vedono nell'immagine
    #cv2.rectangle(img, (0, ALTEZZA-50), (10, ALTEZZA), (255,255,255), -1)
    #cv2.rectangle(img, (LARGHEZZA- 15, ALTEZZA-60), (LARGHEZZA, ALTEZZA), (255,255,255), -1)
    
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    verde = cv2.inRange(hsv, lower_green, upper_green)
    verde=cv2.erode(verde,KERNEL,iterations=3)
    verde=cv2.dilate(verde,KERNEL,iterations=2)

    gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_scale, (7,7), 5)
    ret, bianco = cv2.threshold(blur,lower_white,255,cv2.THRESH_BINARY) #bianco serializzato
    
    bianco[verde == 255] = 255 #tolgo l'eventuale verde dal nero aggiungendolo al bianco

    bianco=cv2.dilate(bianco,KERNEL,iterations=1)
    bianco=cv2.erode(bianco,KERNEL,iterations=4)
    #cv2.imshow("debug bianco", bianco)
    mask_nero = cv2.bitwise_not(bianco)#nero

    return mask_nero, bianco, verde
def scan_nero(img):
    gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_scale, (7,7), 5)
    ret, bianco = cv2.threshold(blur,lower_white-50,255,cv2.THRESH_BINARY)

    bianco=cv2.erode(bianco,KERNEL,iterations=6)
    bianco=cv2.dilate(bianco,KERNEL,iterations=4)
    mask_nero = cv2.bitwise_not(bianco)#nero
    return mask_nero

def get_bigger_area(mask):
    amount, labels = cv2.connectedComponents(mask)
    bigger_area = (None, 0)
    
    for i in range(1, amount):
        area = BLANK.copy()
        area[labels == i] = 255
        area_nonzero = np.count_nonzero(area)
        
        #se l'area è più grande delle precedenti selezionala
        if area_nonzero > bigger_area[1]:
            bigger_area = (area, area_nonzero)
    
    if bigger_area[1] < 1000: #5500
        return BLANK.copy()
    
    return bigger_area[0]

def get_area_with_last_point(mask, x_last, y_last):
    amount, labels = cv2.connectedComponents(mask)
    selected_area = None
    
    for i in range(1, amount):
        area = BLANK.copy()
        area[labels == i] = 255
        
        #se l'area è nera dove c'è la xy_last selezionala
        if area[x_last][y_last] == 255:
            selected_area = area

    return selected_area


def sort_aree(amount, labels, coordinata):
    aree = []
    for i in range(1, amount):
        area = BLANK.copy()
        area[labels == i] = 255 # immagine con solo area bianca colorata
        
        M = cv2.moments(area)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])        
            aree.append((cX, cY, area))
    
    aree.sort(key=lambda i:i[coordinata])#ordina le aree per x o y
    #faccio un array con solo le aree ordinate
    aree_sorted = []
    for item in aree:
        aree_sorted.append(item[-1])

    return aree_sorted

def calcola_inizio_linea(mask, amount, labels):
    if(amount < 3):
        return (ALTEZZA, LARGHEZZA//2), (ALTEZZA, LARGHEZZA//2)
    #trovo le due aree più basse
    aree = sort_aree(amount, labels, 1)
    area1, area2 = aree[-1], aree[-2]
    #determino quale è di sx e dx
    M1, M2 = cv2.moments(area1), cv2.moments(area2)
    if M1["m00"] != 0 and M2["m00"] != 0:
        cX1, cX2 = int(M1["m10"] / M1["m00"]), int(M2["m10"] / M2["m00"])
        if cX1 < cX2:
            area_sx = np.where(area1 == 255)
            area_dx = np.where(area2 == 255)
        else:
            area_dx = np.where(area1 == 255)
            area_sx = np.where(area2 == 255)   
    
    y_maggiore = np.max(area_sx[0])#y con valore maggiore
    maggiori = np.where(area_sx[0] == y_maggiore) #tutti i punti con la y == alla y più grande
    x_maggiore = area_sx[1][np.max(maggiori)] #x del punto con la x più alta tra le y maggiori
    puntoL = x_maggiore, y_maggiore
    
    y_maggiore = np.max(area_dx[0])#y con valore maggiore
    maggiori = np.where(area_dx[0] == y_maggiore) #tutti i punti con la y == alla y più grande
    x_minore = area_dx[1][np.min(maggiori)] #x del punto con la x più bassa tra le y maggiori
    puntoR = x_minore, y_maggiore

    return puntoL, puntoR

def get_n_aree_biance(amount, labels):
    n_aree = 0
    for i in range(1, amount):
        area_bianca = BLANK.copy()
        area_bianca[labels == i] = 255
        n_punti_bianchi = np.count_nonzero(area_bianca)
        if n_punti_bianchi > 1000:
            n_aree += 1
    return n_aree

def confronta_last(amount, labels, collisione, x_last, y_last):
    #confronta le collisioni con la collisione scelta nel frame prima
    #quella simile verrà scelta
    
    # (punto, distanza) 1000 non verrà scelto perchè grande
    punto_alto = ((0,0), 1000)
    for i in range(1, amount):
        collisione = BLANK.copy()
        collisione[labels == i] = 255
        M = cv2.moments(collisione)  #trova il centro delle intersezioni
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"]) 
            distanza = ((x_last - cX)**2 + (y_last - cY)**2)**0.5
            if distanza < punto_alto[1]:
                punto_alto = ((cX, cY), distanza)
    
    return punto_alto[0]

def get_punto_alto(mask, x_last, y_last):
    collisione = np.logical_and(mask, MASK_BORDI)#and tra i bordi e la linea
    collisione = np.uint8(collisione)*255 #per riconvertire in immagine
    
    amount, labels = cv2.connectedComponents(collisione)
    if amount > 2:
        #caso in cui le intersezioni sono due o più quindi viene scelta la più alta
        return confronta_last(amount, labels, collisione, x_last, y_last)

    if amount == 2:
        #c'è solo un'intersezione quindi trovane il centro
        M = cv2.moments(collisione)
        if M["m00"] != 0:
            return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    
    #ERRORE
    return (LARGHEZZA//2, 0)

def getAngle(vertice, p1, p2):
    a = np.array([p1[0], p1[1]])
    b = np.array([vertice[0], vertice[1]])
    c = np.array([p2[0], p2[1]])

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    angle = np.degrees(angle)
    
    return int(angle)

def get_centro_incrocio(amount, labels):
    
    kernel=np.ones((60, 60),np.uint8)
    mask_somma_aree=BLANK.copy()

    #dilato tutte le aree e trovo dove si intersecano        
    for i in range(1, amount):
        #scorro le singole aree (maskArea è l'area singola)
        maskArea = BLANK.copy()
        maskArea[labels == i] = 255
        #la dilato in modo da farla andare anche sopra le altre aree 
        maskArea = cv2.dilate(maskArea,kernel,iterations = 1)
        maskArea//=5 #dove c'è il bianco è 255 ora sarà 255//5 = 51
        mask_somma_aree+=maskArea #sommo tutte le aree in una maschera

    #il valore più alto sarà dove si sono intersecate più aree quindi l'incrocio
    #max_val = np.max(mask_somma_aree) 
    cv2.imshow("incroci", mask_somma_aree)
    mask_incrocio = BLANK.copy()
    mask_incrocio[mask_somma_aree > 103] = 255
    
    amount, labels = cv2.connectedComponents(mask_incrocio)
    
    centri_incroci = get_centri_aree(amount, labels)
    print("n_incroci", len(centri_incroci))
    centri_incroci.sort(key= lambda i:i[1], reverse=True)
    if len(centri_incroci) > 0:
        return centri_incroci[0]
    
    print("problema")
    #c'è stato un problema con il calcolo del centro
    return None

def get_centri_aree(amount, labels):
    centri = []
    for i in range(1, amount):
        mask = BLANK.copy()
        mask[labels == i] = 255
        
        M = cv2.moments(mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])        
            centri.append((cX, cY))
    return centri

def get_collisioni_with_angles(mask_nero, punto_basso, centro_incrocio):
    collisione = cv2.bitwise_and(mask_nero, MASK_BORDI)#and tra i bordi e la linea
    amount, labels = cv2.connectedComponents(collisione)
    collisioni = get_centri_aree(amount, labels) #punto centrale di tutte le linee che escono dal frame

    angoli = []
    #calcola gli angoli del frame
    for centro in collisioni:
        angolo = getAngle(centro_incrocio, punto_basso, centro)
        angoli.append((angolo, centro))
    
    angoli.sort(key=lambda i:abs(i[0]), reverse = False) # li ordino per l'angolo meno ampio

    return angoli
    
def distanza_punti(p1, p2):
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis

def get_collisione_90(collisioni_angoli, punto_basso):
    best = 9999
    collisione = None 
    for angle, c in collisioni_angoli:
        #per ogni possibile direzione
        #trovo la distanza dall'inizion della linea
        distanza_punto_basso = distanza_punti(punto_basso, c)
        #se la distanza è poca vuol dire che è la linea vera e propria quindi la scarto
        #scelgo quella più vicina a 90
        if abs(90 - angle) < best and distanza_punto_basso > 100:
            best = abs(90 - angle)
            collisione = c
    
    return collisione

def get_points_verdi(mask_verde, centro_incrocio, collisione_angolo_piccolo):
    #punti dei centri di ogni quadrato verde
    amount, labels = cv2.connectedComponents(mask_verde)
    punti_verdi = get_centri_aree(amount, labels)
    #trovo la retta 'perpendicolare' dell'incrocio 
    # m = (centro_incrocio[1]-collisione_angolo_piccolo[1])/(centro_incrocio[0]-collisione_angolo_piccolo[0])
    # q = centro_incrocio[1] - (m*centro_incrocio[0])

    # distanza_
    if centro_incrocio[0] < collisione_angolo_piccolo[0]:
        x1, y1 = centro_incrocio
        x2, y2 = collisione_angolo_piccolo
    else:
        x1, y1 = collisione_angolo_piccolo
        x2, y2 = centro_incrocio

    v1 = (x2-x1, y2-y1)   # Vector 1
    
    #check verdi sotto l'incrocio
    verdi_sotto_incrocio = []
    #check verdi sotto il nero
    for xA, yA in punti_verdi:
        #d = (((x2-x1)*(y1-y0)) - ((x1-x0)*(y2-y1))) / np.sqrt(np.square(x2-x1) + np.square(y2-y1))
        v2 = (x2-xA, y2-yA)   # Vector 2
        xp = v1[0]*v2[1] - v1[1]*v2[0]
        if xp < 0:
            verdi_sotto_incrocio.append((xA, yA))
    #check distaza dall'incrocio    
    verdi_vicini_incrocio = []
    for verde in verdi_sotto_incrocio:
        d = distanza_punti(verde, centro_incrocio)
        if d < 50:
            print("d", d)
            verdi_vicini_incrocio.append(verde)
    print("verdi vicini a incrocio", verdi_vicini_incrocio)
    print("incrocio", centro_incrocio)
    #se ci sono due verdi validi allora controllo che siano
    #  uno a destra e l'altro a sinistra dell'incrocio
    if len(verdi_vicini_incrocio) == 2:
        is_left1 = verdi_vicini_incrocio[0][0] < centro_incrocio[0]
        is_left2 = verdi_vicini_incrocio[1][0] < centro_incrocio[0]
        if not(is_left1 ^ is_left2):
            verdi_vicini_incrocio = []
    
    return verdi_vicini_incrocio


def get_punti_cut_verde_singolo(centrolinea, verde):
    try:
        if centrolinea[0] - verde[0] != 0:
            m = (centrolinea[1] - verde[1]) / (centrolinea[0] - verde[0])
            m = -(1 / m) #coefficente angolare perpedicolare
        else:
            m = 1
        q = centrolinea[1] - (m * centrolinea[0])
        
        res1 = (int(verde[0]), int(((verde[0]*m)+q)))
        res2 = (int((verde[1]-q)/m), int(verde[1]))

        return res1, res2
    except ZeroDivisionError:
        return (0,0), (0,0)

def taglio_verde_singolo(mask, output, centro_incrocio, punto_verde):
    p1, p2 = get_punti_cut_verde_singolo(centro_incrocio, punto_verde)
    
    if punto_verde[0] < centro_incrocio[0]:
        #verde sx  
        points = np.array([(LARGHEZZA, 0), (LARGHEZZA, p2[1]+30), p2, p1, (p1[0], 0)])
    else:
        #verde dx
        points = np.array([(0,0), (0, p2[1]+30), p2, p1, (p1[0], 0)])
    
    cv2.fillPoly(mask, pts=[points], color=(0))
    cv2.fillPoly(output, pts=[points], color=(240, 240, 240))

def rimuovi_collisioni(mask, output, centro_incrocio, collisioni):
    for angolo, collisione_removable in collisioni:
        if collisione_removable[0] > centro_incrocio[0]:
            segno = 1 #collisione dx
        else:
            segno = -1 #collisione sx
        if angolo < 160:
            m = (collisione_removable[1] - centro_incrocio[1]) / (collisione_removable[0] - centro_incrocio[0])
            q =  centro_incrocio[1] - (m*centro_incrocio[0])
            p1_remove = [centro_incrocio[0]+(30*segno), int((m*(centro_incrocio[0]+30))+q)-50]
            p2_remove = [collisione_removable[0]+(20*segno), collisione_removable[1]-50]
            p3_remove = [collisione_removable[0]+(20*segno), collisione_removable[1]+50]
            p4_remove = [p1_remove[0], p1_remove[1]+100]
        else:
            p1_remove = [centro_incrocio[0]-40, centro_incrocio[1]+30]
            p2_remove = [centro_incrocio[0]+40, centro_incrocio[1]+30]
            p3_remove = [collisione_removable[0]-40, centro_incrocio[1]+30]
            p4_remove = [collisione_removable[0]+40, centro_incrocio[1]+30]

        pts = np.array([p1_remove, p2_remove, p3_remove, p4_remove])
        cv2.fillPoly(output, pts=[pts], color=(255,20,30))
        cv2.fillPoly(mask, pts=[pts], color=0)

def get_bigger_component(mask):
    amount, labels = cv2.connectedComponents(mask)
    bigger_area = (None, 0)

    for i in range(1, amount):
        area = BLANK.copy()
        area[labels == i] = 255
        area_nonzero = np.count_nonzero(area)
        
        #se l'area è più grande delle precedenti selezionala
        if area_nonzero > bigger_area[1]:
            bigger_area = (area, area_nonzero)

    return bigger_area[0]

def scan_bordi(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    laplacian = cv2.Laplacian(gray,cv2.CV_32F,ksize=1)
    laplacian = cv2.blur(laplacian,(9,9))
    laplacian = np.clip(laplacian,0,1)
    
    laplacian = np.uint8(laplacian*255)
    
    #laplacian = cv2.morphologyEx(laplacian, cv2.MORPH_CLOSE, kernel)
    laplacian = cv2.blur(laplacian,(5,5))
    
    #bordi = cv2.Canny(laplacian,50,200)
    bordi = cv2.adaptiveThreshold(255-laplacian,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,101,50)
    kernel = np.ones((7,7),np.uint8)
    bordi = cv2.erode(bordi,kernel,iterations = 4)
    
    bordi = cv2.bitwise_not(bordi)
    bordo = get_bigger_component(bordi)
    
    return bordo

def sort_aree(amount, labels, coordinata):
    aree = []
    for i in range(1, amount):
        area = BLANK.copy()
        area[labels == i] = 255 # immagine con solo area bianca colorata
        
        M = cv2.moments(area)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])        
            aree.append((cX, cY, area))
    
    aree.sort(key=lambda i:i[coordinata])#ordina le aree per x o y
    #faccio un array con solo le aree ordinate
    aree_sorted = []
    for item in aree:
        aree_sorted.append(item[-1])

    return aree_sorted

def get_green_uscita(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, lower_green_EZ, upper_green_EZ)
    return green

def isRosso(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rosso = cv2.inRange(hsv, lower_red, upper_red)
    rosso2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    rosso=rosso+rosso2
    
    rosso = cv2.erode(rosso,KERNEL,iterations=2)
    rosso = cv2.dilate(rosso,KERNEL,iterations=2)
    
    n_rosso = np.count_nonzero(rosso)

    return n_rosso > 4000
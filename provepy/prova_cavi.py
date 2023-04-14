from gpiozero import Button

destra = Button(19)
sinistra = Button(13)
switch = Button(14)
while True:
    print("switch",switch.is_pressed)
    print("sinistra",sinistra.is_pressed)
    print("destra",destra.is_pressed)
    print("")
    
        
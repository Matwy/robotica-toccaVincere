from gpiozero import Button

destra = Button(12)
sinistra = Button(4)
switch = Button(14)
while True:
    print("switch",switch.is_pressed)
    print("sinistra",sinistra.is_pressed)
    print("destra",destra.is_pressed)
    print("\n\n\n")
    
        
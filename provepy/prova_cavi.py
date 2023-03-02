from gpiozero import Button

destra = Button(14)
sinistra = Button(4)
switch = Button(12)
while True:
    print("switch",switch.is_pressed)
    print("switch",sinistra.is_pressed)
    print("switch",destra.is_pressed)
    print("\n\n\n")
    
        
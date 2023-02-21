from gpiozero import Button

destra = Button(26)
sinistra = Button(13)

if destra.is_pressed and sinistra.is_pressed:
    print("conduce tutti e due")
elif destra.is_pressed:
    print("destra conduce")
elif sinistra.is_pressed:
    print("sinistra conduce")
else:
    print("no conduce")
        
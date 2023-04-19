from threading import Thread
import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

class Tofs:
    def __init__(self):
        self.distanze = []
        self.stopped = False

        self.i2c = board.I2C()
        self.xshut = [
            DigitalInOut(board.D16),
            DigitalInOut(board.D20),
            DigitalInOut(board.D21),
            # add more VL53L0X sensors by defining their SHDN pins here
        ]
        #pin_stronzo = DigitalInOut(board.D21)
        #pin_stronzo.switch_to_output(value=False)
        
        for power_pin in self.xshut:
            # make sure these pins are a digital output, not a digital input
            power_pin.switch_to_output(value=False)

        self.vl53 = []

        for i, power_pin in enumerate(self.xshut):
            # turn on the VL53L0X to allow hardware check
            power_pin.value = True
            # instantiate the VL53L0X sensor on the I2C bus & insert it into the "vl53" list
            #time.sleep(0.5)
            time.sleep(0.02)
            self.vl53.insert(i, VL53L0X(self.i2c))  # also performs VL53L0X hardware check
            time.sleep(0.02)
            # no need to change the address of the last VL53L0X sensor
            if i < len(self.xshut) - 1:
                # default address is 0x29. Change that to something else
                self.vl53[i].set_address(i + 0x30)
    
    def start(self):
        pass
    
    def stop(self):
        pass

    def detect_range(self):
        distanza_tmp = []
        for index, sensor in enumerate(self.vl53):
            try:
                dis = sensor.range
            except:
                print("l'i2c almeno diventa più simpatico con un trycatch le donne no")
            if(dis == "None"):
                continue
            else:
                distanza_tmp.append(dis)
        return distanza_tmp
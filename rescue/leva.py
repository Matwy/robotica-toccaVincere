from gpiozero import Button
import os
import time
import signal
# Motors
import busio
from board import SCL, SDA
from adafruit_bus_device import i2c_device

leva = Button(14)
pid = None
run = 0

# Motors
i2c_bus = busio.I2C(SCL, SDA)
arduinoi2c = i2c_device.I2CDevice(i2c_bus, 0x10)

def run_script():
    # Change the working directory to the root of your project where your script is located
    os.chdir('/home/pi/Desktop/robotica-toccaVincere/rescue')
    
    # Activate the virtual environment
    activate_this = '/home/pi/.virtualenvs/cv/bin/activate_this.py'
    with open(activate_this) as file_:
        exec(file_.read(), dict(__file__=activate_this))
    
    # Run your script within the virtual environment and return the process ID
    global pid
    pid = os.spawnlp(os.P_NOWAIT, 'python', 'python', 'main.py')
    return pid

def stop_script():
    # Send a SIGTERM signal to the process with the given PID
    global pid
    if pid:
        try:
            arduinoi2c.write(bytes([0,0]))
            arduinoi2c.write(bytes([1,0]))
        except:
            print("un casino for real")
        os.kill(pid, signal.SIGTERM)

print ("[LEVA] Waiting")
while True:
    if leva.is_pressed and run == 0:
        run = 1
        run_script()
        print ("[LEVA] ---- Started ---- ")
        while leva.is_pressed:
            time.sleep(0.1)
            
    if not leva.is_pressed and run == 1:
        time.sleep(0.2)
        stop_script()
        print ("[LEVA] ---- Stopped ---- " )
        run = 0
        try:
            arduinoi2c.write(bytes([1,0]))
            arduinoi2c.write(bytes([0,0]))
        except:
            print("un casino for real")
        while not leva.is_pressed:
            time.sleep(0.1)
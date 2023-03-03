from gpiozero import Button
import time
import PiVideoStreamUndistorted
import PiVideoStream
from Motors import Motors
from multiTof import sensors_stream
from servo import Servo
from gyro import get_inclinazione
class Robot():
    def __init__(self):
        self.servo = Servo()
        self.cam_stream = PiVideoStreamUndistorted.PiVideoStream().start()
        self.sensors_stream = sensors_stream().start()
        time.sleep(0.5)
        self.motors = Motors()
        self.cavo_sinistra = Button(4)
        self.cavo_destra = Button(14)
        self.last_punto_alto = (136//2, 0)
        self.last_punto_basso = (136//2, 137)
            
    def get_tof_mesures(self):
        return self.sensors_stream.get_range()
    def get_frame(self):
        return self.cam_stream.read()
    
    def get_resolution(self):
        return (self.cam_stream.LARGHEZZA, self.cam_stream.ALTEZZA)
    
    def is_salita(self):
        print(get_inclinazione())
        return get_inclinazione() > 6
    
    def camstream_EZ(self):
        self.cam_stream.stop()
        time.sleep(0.3)
        self.cam_stream = PiVideoStream.PiVideoStream(framerate=7, resolution=(160, 128)).start()
    
    def camstream_linea(self):
        self.cam_stream.stop()
        time.sleep(0.3)
        self.cam_stream = PiVideoStreamUndistorted.PiVideoStream().start()
    
    def restart_tof(self):
        self.sensors_stream.stop()
        self.sensors_stream = sensors_stream().start()
      
    def check_ez(self):
        if self.cavo_sinistra.is_pressed or self.cavo_destra.is_pressed:
            self.motors.motors(0,0)
            print("conduzio")
            if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                self.motors.motors(0,0)
                return True
            
            sx_speed, dx_speed = (0,30) if self.cavo_sinistra.is_pressed else (30,0)    
            
            self.motors.motors(-sx_speed,-dx_speed)
            t_fine = time.time()+4
            while time.time() < t_fine:
                if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                    return True
            
            self.motors.motors(sx_speed,dx_speed)
            t_fine = time.time()+5
            while time.time() < t_fine:
                if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                    return True

from gpiozero import Button
import time
import PiVideoStreamUndistorted
import PiVideoStream
from Motors import Motors
from tofs import Tofs
from servo import Servo
from gyro import Gyro
class Robot():
    def __init__(self):
        self.servo = Servo()
        self.cam_stream = PiVideoStreamUndistorted.PiVideoStream().start()
        self.sensors_stream = Tofs()
        self.gyro = Gyro()
        time.sleep(0.5)
        self.motors = Motors()
        self.cavo_sinistra = Button(13)
        self.cavo_destra = Button(19)
        self.last_punto_alto = (136//2, 0)
        self.last_punto_basso = (136//2, 137)
            
    def get_tof_mesures(self):
        try:
            return self.sensors_stream.detect_range()
        except:
            print("diocancaoncaoncaonc")
            return [8190, 8190, 8190]
    def get_frame(self):
        return self.cam_stream.read()
    
    def get_resolution(self):
        return (self.cam_stream.LARGHEZZA, self.cam_stream.ALTEZZA)
    
    def get_gyro_value(self):
        try:
            return self.gyro.read()
        except:
            return 0

    def is_salita(self):
        # print("inclinamelo ", get_inclinazione())
        return self.get_gyro_value() > 6
    
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

            #  provo un po avanti 
            self.motors.motors(0,0)
            t_fine = time.time()+0.2
            while time.time() < t_fine:
                if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                    return True
            
            sx_speed, dx_speed = (-10,40) if self.cavo_sinistra.is_pressed else (40,-10)    
            # porto avanti il lato che non tocca
            self.motors.motors(sx_speed,dx_speed)
            t_fine = time.time()+2
            while time.time() < t_fine:
                if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                    return True
            """
            self.motors.motors(-sx_speed,-dx_speed)
            t_fine = time.time()+2
            while time.time() < t_fine:
                if self.cavo_sinistra.is_pressed and self.cavo_destra.is_pressed:
                    return True
            """
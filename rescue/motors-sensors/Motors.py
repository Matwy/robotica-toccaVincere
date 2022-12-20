from smbus2 import SMBus

addr = 0x10
bus = SMBus(1)

class Motors():

    M_LEFT = 1
    M_RIGHT = 2

    def send_motor_power(self, motor, power):
        bus.write_byte_data(addr, motor, power)

    def motors(self, m1_power, m2_power):
        self.send_motor_power(Motors.M_LEFT, m1_power)
        self.send_motor_power(Motors.M_RIGHT, m2_power)
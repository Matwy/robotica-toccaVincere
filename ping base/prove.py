from smbus2 import SMBus

addr = 0x10
bus = SMBus(1)

bus.write_byte_data(addr, 0, 45)
from smbus2 import SMBus, i2c_msg

addr = 0x10
bus = SMBus(1)

data = [100, 0]
bus.write_i2c_block_data(addr, 0, data)
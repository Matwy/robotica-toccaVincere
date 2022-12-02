from smbus2 import SMBus

addr = 0x10
with SMBus(1) as bus:
    bus.write_i2c_block_data(addr, 0, [0,1,50,0,100])
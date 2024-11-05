import smbus
import time
bus=smbus.SMBus(1)
brightness=100
Device_address=0x48
control_byte=0x40
read_byte = 0x00
while True:
    bus.write_byte(Device_address,read_byte)
    num = bus.read_byte(Device_address)
    print(f"{num/255*3.3:.2f}")
    bus.write_byte_data(Device_address,control_byte,num)
    
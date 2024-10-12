import smbus
import time
bus=smbus.SMBus(1)
brightness=100
Device_address=0x48
control_byte=0x40
gap=1/256
while True:
    brightness = int(input("输入一个led数值（范围0-255）："))
    bus.write_byte_data(Device_address,control_byte,brightness)
    
import smbus
import time
import math

bus = smbus.SMBus(1)
brightness = 100
Device_address = 0x48
control_byte = 0x02  # 读取channel2
B=3950
T0=298.15
R0=1000
while True:
    try:
        bus.write_byte(Device_address, control_byte)
        num = bus.read_byte(Device_address)
        time.sleep(0.1)
        v = num / 255 * 5
        r = v *1000 /(5-v)
        T = 1/(math.log(r/R0)/B+1/T0)-273.15
        print(f"当前电压大小：{v:.2f}")
        print(f"当前电阻大小：{r:.2f}")
        print(f"当前温度：{T:.2f}")
        # print(f"{num / 255 * 5:.2f}")
        time.sleep(1)
    except Exception as e:
        print("等待连接")

import smbus
import time
import queue
import threading
import numpy as np
# PCF8591的I2C地址
PCF8591_ADDRESS = 0x48
# 创建一个队列来存储电压值
voltage_queue = queue.Queue()

bus = smbus.SMBus(1)  # 使用I2C1
class Joystick:
    def __init__(self):
        self.values = np.zeros(4)
        self.PCF8591_ADDRESS = 0x48
        self.bus = smbus.SMBus(1)  # 使用I2C1
        self.data = np.zeros(4)
        self.values = np.zeros(4)
    def read_all_channels_auto_increment(self):
    # 第一次读取时，选择通道0，并设置自动递增标志（通常是0x40）
        for i in range(4):
            self.data[i] = self.read_analog(i)
        return True
        # try:
        #     data = self.bus.read_i2c_block_data(self.PCF8591_ADDRESS, 0x40, 4)
        #     self.data = np.array(data).astype(np.float32)
        #     print(f"读取数据：{data}")
        #     return True
        # except OSError:
        #     print("读取所有通道失败")
        #     return False  # 如果读取失败，返回Falsedef read_
    def change_format(self):
        self.values = self.data-128
        self.values[np.abs(self.values)<28] = 0
        self.values[self.values>=28] = self.values[self.values>=28]-28
        self.values[self.values<=-28] = self.values[self.values<=-28]+28
        self.values = self.values/100
        return self.values
    def read_values(self):
        self.read_all_channels_auto_increment()
        self.change_format()
        return self.values[:3],self.values[3]
    def read_analog(self, channel):
        # 写入控制字节，设置要读取的通道
        try:
            data = self.bus.read_i2c_block_data(self.PCF8591_ADDRESS, channel, 1)
            return data[0]
        except OSError:
            print("读取失败")
            return 128  # 如果读取失败，返回0
    
def read_analog(channel):
    # 写入控制字节，设置要读取的通道
    # bus.write_byte(PCF8591_ADDRESS, channel | 0x40)
    # # 读取4个字节数据，其中第一个字节为控制字节，后面3个字节为模拟输入值
    # time.sleep(0.01)
    
    data = bus.read_i2c_block_data(PCF8591_ADDRESS, channel, 1)
    return data[0]
def read_sensor_values():
    while True:
        value_an0 = read_analog(0)  # 读取AN0
        value_an1 = read_analog(1)  # 读取AN1
        
        # 计算电压 (假设使用5V供电，进行适当的比例计算)
        voltage_an0 = value_an0 * (5.0 / 255.0)
        voltage_an1 = value_an1 * (5.0 / 255.0)

        # 将电压值放入队列中
        voltage_queue.put((voltage_an0, voltage_an1))
        
        time.sleep(0.1)  # 每隔一秒读取一次

# 创建并启动线程
# sensor_thread = threading.Thread(target=read_sensor_values)
# sensor_thread.daemon = True  # 设置为守护线程
# sensor_thread.start()

# try:
#     while True:
#         # 从队列中获取电压值
#         try:
#             voltage_an0, voltage_an1 = voltage_queue.get(timeout=1)  # 获取电压值
#             print("Main Loop: AN0 Voltage: {:.2f}V, AN1 Voltage: {:.2f}V".format(voltage_an0, voltage_an1))
#         except queue.Empty:
#             print("Main Loop: 等待电压值...")
        
#         # 主程序可以继续执行其他任务
#         time.sleep(0.1)  # 每两秒打印一次
# except KeyboardInterrupt:
#     print("程序结束")
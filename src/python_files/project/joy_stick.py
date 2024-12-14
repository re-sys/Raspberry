import smbus
import time
import queue
import threading
# PCF8591的I2C地址
PCF8591_ADDRESS = 0x48
# 创建一个队列来存储电压值
voltage_queue = queue.Queue()

bus = smbus.SMBus(1)  # 使用I2C1

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
sensor_thread = threading.Thread(target=read_sensor_values)
sensor_thread.daemon = True  # 设置为守护线程
sensor_thread.start()

try:
    while True:
        # 从队列中获取电压值
        try:
            voltage_an0, voltage_an1 = voltage_queue.get(timeout=1)  # 获取电压值
            print("Main Loop: AN0 Voltage: {:.2f}V, AN1 Voltage: {:.2f}V".format(voltage_an0, voltage_an1))
        except queue.Empty:
            print("Main Loop: 等待电压值...")
        
        # 主程序可以继续执行其他任务
        time.sleep(0.1)  # 每两秒打印一次
except KeyboardInterrupt:
    print("程序结束")
import RPi.GPIO as GPIO
import time
from dataclasses import dataclass

GPIO.setmode(GPIO.BCM)
# 定义GPIO引脚
SDA_PIN = 20  # 可以根据您的实际连接更改
SCL_PIN = 21  # 可以根据您的实际连接更改
WHO_AM_I = 0x75
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
WHO_AM_I     = 0x75
PWR_MGMT_2   = 0x6C
DEVICE_ADDRESS = 0x68
@dataclass
class MPU6050_Data:
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
def sleep_us(us):
    time.sleep(us / 1000000.0)

# def i2c_init():
#       # 使用BCM编号方式
#     GPIO.setup(SDA_PIN, GPIO.OUT, initial=GPIO.HIGH)
#     GPIO.setup(SCL_PIN, GPIO.OUT, initial=GPIO.HIGH)

def scl_write(value):
    if value == 0:
        GPIO.setup(SCL_PIN, GPIO.OUT)
        GPIO.output(SCL_PIN, 0)
    else:
        GPIO.setup(SCL_PIN, GPIO.IN)  # 释放SCL引脚为输入模式
    sleep_us(10)

def sda_write(value):
    if value == 0:
        GPIO.setup(SDA_PIN, GPIO.OUT)
        GPIO.output(SDA_PIN, 0)
    else:
        GPIO.setup(SDA_PIN, GPIO.IN)  # 释放SDA引脚为输入模式
    sleep_us(10)
# 初始化GPIO
# 设置SDA引脚为输出模式并写入值
# 设置SDA引脚为输入模式并读取值
def sda_read():
    GPIO.setup(SDA_PIN, GPIO.IN)
    return GPIO.input(SDA_PIN)

# 设置SCL引脚
# 生成I2C开始条件
def i2c_start():
    #先释放，并且兼容重复开始，所以先释放sda
    sda_write(1)
    scl_write(1)
    #开始条件就是先拉低SDA，然后拉低scl
    sda_write(0)
    scl_write(0)

# 生成I2C停止条件
def i2c_stop():
    #先进行拉低最后的sda，保证结束时是先拉高scl，然后拉高sda
    sda_write(0)
    #除了这个结束符号外，其它数据处理都是保证以scl低电平结束
    scl_write(1)
    
    sda_write(1)
# 发送一个字节的数据
def i2c_send_byte(byte):
    
    for i in range(8):
        # sda_write(byte & (0x80 >> i))
        sda_write((byte >> (7 - i)) & 1)
        scl_write(1)
        scl_write(0)
    
    # # 释放SDA引脚为输入模式以接收ACK
    # GPIO.setup(SDA_PIN, GPIO.IN)
    # ack = GPIO.input(SDA_PIN)
    # scl_write(1)
    
    # scl_write(0)
    # # 恢复SDA引脚为输出模式
    # GPIO.setup(SDA_PIN, GPIO.OUT)
    # return ack
# 接收一个字节的数据
def i2c_receive_byte():
    byte = 0x00
    sda_write(1)  # 释放
    for i in range(8):
        #这时从机已经放好数据，拉高scl读取数据
        scl_write(1)
        if GPIO.input(SDA_PIN):
            byte |= (0x80 >> i)
        # byte |= (GPIO.input(SDA_PIN) << (7 - i))
        scl_write(0)
        
    # if ack:
    #     sda_write(0)  # 发送ACK
    # else:
    #     sda_write(1)  # 发送NACK
    
    # scl_write(1)
    
    # scl_write(0)
    return byte
def i2c_send_ack(ack):
    sda_write(ack)
    scl_write(1)
    scl_write(0)
def i2c_receive_ack():
    #主机释放sda防止干扰从机
    sda_write(1)
    scl_write(1)
    ack = GPIO.input(SDA_PIN)
    scl_write(0)
    return ack
# 清理GPIO设置
def i2c_cleanup():
    GPIO.cleanup()

def MPU6050_WriteReg(register_address,data):
    i2c_start()
    i2c_send_byte((DEVICE_ADDRESS<<1) | 0x00)  # 发送设备地址(这里是) + 写位
    i2c_receive_ack()  # 接收ACK
    i2c_send_byte(register_address)  # 发送寄存器地址
    i2c_receive_ack()  # 接收ACK
    i2c_send_byte(data)  # 发送数据
    i2c_receive_ack()  # 发送NACK
    i2c_stop()
def MPU6050_ReadReg(register_address,is_2byte=False):
    i2c_start()
    i2c_send_byte((DEVICE_ADDRESS<<1) | 0x00)  # 发送设备地址(这里是) + 写位
    i2c_receive_ack()  # 接收ACK
    i2c_send_byte(register_address)  # 发送寄存器地址
    i2c_receive_ack()  # 接收ACK

    i2c_start()
    i2c_send_byte((DEVICE_ADDRESS<<1) | 0x01)  # 发送设备地址(这里是) + 读位
    i2c_receive_ack()  # 接收ACK
    # for i in range(length):
    if is_2byte:
        high = i2c_receive_byte()
        i2c_send_ack(0)  # 发送ACK
        low = i2c_receive_byte()
        i2c_send_ack(1)  # 发送NACK
        i2c_stop()        
        # 组合成16位数据
        value = (high << 8) | low       
        # 处理符号位
        if value > 32768:
            value = value - 65536

        return value
    data = i2c_receive_byte()  # 接收字节，并发送NACK
    i2c_send_ack(1)  # 不给从机应答，防止从机认为你还想要
    i2c_stop()
    return data
def MPU6050_INIT(selfdetect):
    MPU6050_WriteReg(PWR_MGMT_1, 0x01)  # 复位MPU6050,解除原本上电的睡眠模式，时钟使用x轴的pll时钟
    MPU6050_WriteReg(PWR_MGMT_2, 0x00)  # 不进行循环时钟配置，和不关掉轴的输出
    MPU6050_WriteReg(SMPLRT_DIV, 0x09)  # 十分频
    MPU6050_WriteReg(CONFIG, 0x06)  # 不设置外部引脚配置，低通滤波器采用最大平滑也就是110
    if selfdetect:
        MPU6050_WriteReg(GYRO_CONFIG, 0xF8)  # 设置陀螺仪传感器满量程范围，使能自测
        MPU6050_WriteReg(ACCEL_CONFIG, 0xF8)  # 设置加速度传感器满量程范围，使能自测
    else:
        MPU6050_WriteReg(GYRO_CONFIG, 0x18)  # 设置陀螺仪传感器满量程范围，不使能自测
        MPU6050_WriteReg(ACCEL_CONFIG, 0x18)  # 设置加速度传感器满量程范围，不使能自测
def cancel_self_test():
    MPU6050_WriteReg(GYRO_CONFIG, 0x18)  # 设置陀螺仪传感器满量程范围，不使能自测
    MPU6050_WriteReg(ACCEL_CONFIG, 0x18)  # 设置加速度传感器满量程范围，不使能自测
def MPU6050_ReadData():
    accel_x = MPU6050_ReadReg(ACCEL_XOUT_H,True)/32768*16  # 读取加速度数据X轴
    accel_y = MPU6050_ReadReg(ACCEL_YOUT_H,True)/32768*16  # 读取加速度数据Y轴
    accel_z = MPU6050_ReadReg(ACCEL_ZOUT_H,True)/32768*16  # 读取加速度数据Z轴
    gyro_x = MPU6050_ReadReg(GYRO_XOUT_H,True)/32768*2000  # 读取陀螺仪数据X轴
    gyro_y = MPU6050_ReadReg(GYRO_YOUT_H,True)/32768*2000  # 读取陀螺仪数据Y轴
    gyro_z = MPU6050_ReadReg(GYRO_ZOUT_H,True)/32768*2000  # 读取陀螺仪数据Z轴
    
    return MPU6050_Data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

# 示例使用
def read_who_am_i():
    i2c_start()
    i2c_send_byte((0x68<<1) | 0x00)  # 发送设备地址(这里是) + 写位
    ack = i2c_receive_ack()  # 接收ACK
    print("ack:", ack)
    # if not ack:
    #     print("Device did not acknowledge the address.")
    #     i2c_stop()
    #     return None
    byte = MPU6050_ReadReg(WHO_AM_I)
    print("WHO_AM_I value: 0x{:02X}".format(byte))
    temp = MPU6050_ReadReg(PWR_MGMT_1)
    print("PWR_MGMT_1 value: 0x{:02X}".format(temp))

    MPU6050_WriteReg(PWR_MGMT_1, 0x00)  # 复位MPU6050,解除原本上电的睡眠模式
    temp = MPU6050_ReadReg(PWR_MGMT_1)
    print("PWR_MGMT_1 value after reset: 0x{:02X}".format(temp))
    MPU6050_INIT(True)
    data  = MPU6050_ReadData()
    print("accel_x: ", data.accel_x)
    print("accel_y: ", data.accel_y)
    print("accel_z: ", data.accel_z)
    print("gyro_x: ", data.gyro_x)
    print("gyro_y: ", data.gyro_y)
    print("gyro_z: ", data.gyro_z)
    MPU6050_INIT(False)
    data  = MPU6050_ReadData()
    print("accel_x: ", data.accel_x)
    print("accel_y: ", data.accel_y)
    print("accel_z: ", data.accel_z)
    print("gyro_x: ", data.gyro_x)
    print("gyro_y: ", data.gyro_y)
    print("gyro_z: ", data.gyro_z)
    if not ack:
        print("Device did not acknowledge the register address.")
        i2c_stop()
        return None
    i2c_start()
    ack = i2c_send_byte(0xD1)  # 发送设备地址 + 读位
    if not ack:
        print("Device did not acknowledge the address for read.")
        i2c_stop()
        return None
    who_am_i = i2c_receive_byte(0)  # 接收字节，并发送NACK
    i2c_stop()
    return who_am_i

# 常量定义


# 主程序
try:
    
    who_am_i = read_who_am_i()
    if who_am_i is not None:
        print("WHO_AM_I value: 0x{:02X}".format(who_am_i))
        if who_am_i == 0x68:
            print("MPU6050 is correctly connected.")
        else:
            print("MPU6050 connection failed.")
finally:
    i2c_cleanup()

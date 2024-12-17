'''
    Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus			#import SMBus module of I2C
from time import sleep          #import
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from queue import Queue	
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
PWR_MGMT_2   = 0x6C
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
WHO_AM_I     = 0x75
ACCEL_CONFIG = 0x1C
# 创建一个固定长度的队列
# fixed_queue = Queue(maxsize=100)

# # 初始化图表
# fig, ax = plt.subplots()
# line, = ax.plot([], [], lw=2)
# ax.set_xlabel('Time')
# ax.set_ylabel('Norm')
# ax.set_title('Norm over Time')
# ax.set_xlim(0, 100)
# ax.set_ylim(0, 2)  # 根据实际情况调整 y 轴的范围


# def get_acceleration_values():
#     acc_x_offset, acc_y_offset, acc_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset = get_median_offset()
#     Ax = acc_x - acc_x_offset
#     Ay = acc_y - acc_y_offset
#     Az = acc_z - acc_z_offset


# def init():
#     line.set_data([], [])
#     return line,

# def update(frame):
#     Ax, Ay, Az = get_acceleration_values()
#     norm = np.linalg.norm([Ax, Ay, Az])
    
#     if fixed_queue.full():
#         fixed_queue.get()
#     fixed_queue.put(norm)

#     # 获取队列中的所有值
#     queue_values = list(fixed_queue.queue)
    
#     # 更新图表数据
#     line.set_data(range(len(queue_values)), queue_values)
#     ax.set_xlim(0, len(queue_values))
#     ax.set_ylim(0, max(queue_values) + 0.1)  # 动态调整 y 轴的范围
#     return line,

# 创建动画
# ani = FuncAnimation(fig, update, frames=range(100), init_func=init, blit=True, interval=10000/1000)

plt.show()
#initialize the bus
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
def sleep_us(us):
    sleep(us / 1000000.0)
def MPU_Init():
    
	temp = bus.read_byte_data(Device_Address, WHO_AM_I)#查找这个设备是否是自己
	print("WHO_AM_I: ", hex(temp))
	if temp != 0x68:
		print("MPU6050 not found")
		return
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 0x00)#重启设备,如果重启设备就高位放置1，他有休眠模式等
    
	bus.write_byte_data(Device_Address, PWR_MGMT_2, 0x00)

	# Read data back to verify
	temp = bus.read_byte_data(Device_Address, PWR_MGMT_1)
	print("PWR_MGMT_1: ", hex(temp))
	
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 0x09)#设置分频值，Gyro的采样率为8kHz，真实数据刷新率就是8k/(1+div)
       
    #如果采用了低通滤波器，那么Gyro的初始采样率就是1khz
    # read data back to verify
	temp = bus.read_byte_data(Device_Address, SMPLRT_DIV)
	print("SMPLRT_DIV: ", hex(temp))
	#Write to Configuration register
	
	bus.write_byte_data(Device_Address, CONFIG, 0x06)#设置最高低通滤波

	data = bus.read_byte_data(Device_Address, CONFIG)
	print("CONFIG: ", data)

	#Write to power management register
	
	
	#Write to Configuration register
    #配置寄存器，这里可以配置是否使用低通滤波器，还有外部同步
	
	
	#Write to Gyro configuration register
    #配置Gyro的量程范围，还可以配置使能自测试位等
    
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 0x00)
	
	temp = bus.read_byte_data(Device_Address, GYRO_CONFIG)	#读取配置寄存器
	
	
	print("GYRO_CONFIG: ", hex(temp))
	
	#Write to Accel configuration register
	bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0x00)

	temp = bus.read_byte_data(Device_Address, ACCEL_CONFIG)
	print("ACCEL_CONFIG: ", hex(temp))
	
	#Write to interrupt enable register
	# bus.write_byte_data(Device_Address, INT_ENABLE, 1)
def read_median_data(addr):
    data_points = []
    for _ in range(100):
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value = value - 65536
        data_points.append(value)
    data_points.sort()
    return data_points[50]  # 返回中值
def read_block_data():
    data_block = bus.read_i2c_block_data(Device_Address, ACCEL_XOUT_H, 6)

    # 解析数据并赋值给结构体
    acc_x = (data_block[0] << 8) | data_block[1]
    acc_y = (data_block[2] << 8) | data_block[3]
    acc_z = (data_block[4] << 8) | data_block[5]
    # gyro_x = (data_block[8] << 8) | data_block[9]
    # gyro_y = (data_block[10] << 8) | data_block[11]
    # gyro_z = (data_block[12] << 8) | data_block[13]

    # 转换为有符号的16位整数
    acc_x = acc_x if acc_x < 32768 else acc_x - 65536
    acc_y = acc_y if acc_y < 32768 else acc_y - 65536
    acc_z = acc_z if acc_z < 32768 else acc_z - 65536
    # gyro_x = gyro_x if gyro_x < 32768 else gyro_x - 65536
    # gyro_y = gyro_y if gyro_y < 32768 else gyro_y - 65536
    # gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536
    
	# 加上量程
    acc_x = acc_x / 32768.0 * 2.0
    acc_y = acc_y / 32768.0 * 2.0
    acc_z = acc_z / 32768.0 * 2.0
    # gyro_x = gyro_x / 32768.0 * 250.0
    # gyro_y = gyro_y / 32768.0 * 250.0
    # gyro_z = gyro_z / 32768.0 * 250.0


    return acc_x, acc_y, acc_z
def get_median_offset():
    acc_x_list = []
    acc_y_list = []
    acc_z_list = []
    # gyro_x_list = []
    # gyro_y_list = []
    # gyro_z_list = []
    time_start = time.time()
    for _ in range(100):
        acc_x, acc_y, acc_z = read_block_data()
        sleep_us(1000)
        acc_x_list.append(acc_x)
        acc_y_list.append(acc_y)
        acc_z_list.append(acc_z-1)
        
        # gyro_x_list.append(gyro_x)
        # gyro_y_list.append(gyro_y)
        # gyro_z_list.append(gyro_z)

    # 对每个列表进行排序并取中值
    acc_x_list.sort()
    acc_y_list.sort()
    acc_z_list.sort()
    # gyro_x_list.sort()
    # gyro_y_list.sort()
    # gyro_z_list.sort()

    acc_x_median = acc_x_list[50]
    acc_y_median = acc_y_list[50]
    acc_z_median = acc_z_list[50]
    # gyro_x_median = gyro_x_list[50]
    # gyro_y_median = gyro_y_list[50]
    # gyro_z_median = gyro_z_list[50]
    time_end = time.time()
    print("Time taken to get median offset: ", time_end - time_start)
    print(acc_z_median)
    return acc_x_median, acc_y_median, acc_z_median
      
def get_offset():
	time_start = time.time()
	acc_x = read_median_data(ACCEL_XOUT_H)
	acc_y = read_median_data(ACCEL_YOUT_H)
	acc_z = read_median_data(ACCEL_ZOUT_H)

	gyro_x = read_median_data(GYRO_XOUT_H)
	gyro_y = read_median_data(GYRO_YOUT_H)
	gyro_z = read_median_data(GYRO_ZOUT_H)
	time_end = time.time()
	print("Time taken to get offset: ", time_end - time_start)
	return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z


def get_velPos(dt,flag):
    global current_vel, current_acc, last_acc,last_vel,pos
    if flag:
        current_vel = last_vel + (current_acc+last_acc)/2*dt
        pos = pos + (current_vel+last_vel)/2*dt
        last_acc = current_acc
        last_vel = current_vel
    else:
        current_vel=np.zeros((3,1))
        last_vel = current_vel
    return current_vel,pos

    
def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
def rotation_matrix_from_euler(roll, pitch, yaw=0):
    """
    根据绕x轴的roll, 绕y轴的pitch, 和绕z轴的yaw角度计算旋转矩阵。
    
    参数:
    roll (float): 绕x轴的旋转角度（弧度）
    pitch (float): 绕y轴的旋转角度（弧度）
    yaw (float): 绕z轴的旋转角度（弧度）
    
    返回:
    numpy.ndarray: 3x3的旋转矩阵
    """
    # 绕z轴的旋转矩阵
    # R_z = np.array([
    #     [np.cos(yaw), -np.sin(yaw), 0],
    #     [np.sin(yaw), np.cos(yaw), 0],
    #     [0, 0, 1]
    # ])
    
    # 绕y轴的旋转矩阵
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # 绕x轴的旋转矩阵
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # 总的旋转矩阵，按照Z-Y-X顺序
    R = np.dot(R_y, R_x)
    
    return R
def visualize_rotation_matrix(R):
    """
    可视化旋转矩阵的位姿。
    
    参数:
    R (numpy.ndarray): 3x3的旋转矩阵
    """
    # 创建一个新的图形
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 原始坐标系
    original_axes = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    
    # 应用旋转矩阵到原始坐标系
    rotated_axes = np.dot(R, original_axes)
    
    # 绘制原始坐标系
    ax.quiver(0, 0, 0, original_axes[0, 0], original_axes[0, 1], original_axes[0, 2], color='r', label='X (原始)')
    ax.quiver(0, 0, 0, original_axes[1, 0], original_axes[1, 1], original_axes[1, 2], color='g', label='Y (原始)')
    ax.quiver(0, 0, 0, original_axes[2, 0], original_axes[2, 1], original_axes[2, 2], color='b', label='Z (原始)')
    
    # 绘制旋转后的坐标系
    ax.quiver(0, 0, 0, rotated_axes[0, 0], rotated_axes[0, 1], rotated_axes[0, 2], color='m', label='X (旋转后)')
    ax.quiver(0, 0, 0, rotated_axes[1, 0], rotated_axes[1, 1], rotated_axes[1, 2], color='c', label='Y (旋转后)')
    ax.quiver(0, 0, 0, rotated_axes[2, 0], rotated_axes[2, 1], rotated_axes[2, 2], color='y', label='Z (旋转后)')
    
    # 设置坐标轴范围
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    # 设置坐标轴标签
    ax.set_xlabel('X 轴')
    ax.set_ylabel('Y 轴')
    ax.set_zlabel('Z 轴')
    
    # 添加图例
    ax.legend()
    
    # 设置标题
    ax.set_title('旋转矩阵可视化')
    
    # 显示图形
    plt.draw()
    plt.pause(0.01)

def update(R):
    global lines
    # 应用旋转矩阵到原始坐标系
    rotated_axes = np.dot(R, original_axes)
    
    # 更新旋转后的坐标系
    for i, line in enumerate(lines):
        line.remove()
        lines[i] = ax.quiver(0, 0, 0, rotated_axes[i, 0], rotated_axes[i, 1], rotated_axes[i, 2], color=['m', 'c', 'y'][i], label=['X (旋转后)', 'Y (旋转后)', 'Z (旋转后)'][i])
    
    # 刷新图形
    plt.draw()
    plt.pause(0.01)

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")
# acc_x_offset, acc_y_offset, acc_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset = get_median_offset()

# print("Accelerometer X Offset:", acc_x_offset)
# print("Accelerometer Y Offset:", acc_y_offset)
# print("Accelerometer Z Offset:", acc_z_offset)
# print("Gyroscope X Offset:", gyro_x_offset)
# print("Gyroscope Y Offset:", gyro_y_offset)
# print("Gyroscope Z Offset:", gyro_z_offset)
class kalman_filter:
    def __init__(self,Q=0.001,R=0.1):
        self.Q = Q
        self.R = R
        
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0
        
    def kalman(self,ADC_Value):
       
        self.Z_k = ADC_Value
        
        if (abs(self.kalman_adc_old-ADC_Value)>=0.005):
            self.x_k1_k1= ADC_Value*0.382 + self.kalman_adc_old*0.618
        else:
            self.x_k1_k1 = self.kalman_adc_old
    
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
    
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)
    
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
    
        self.kalman_adc_old = kalman_adc
        
        return kalman_adc

R = np.eye(3)
previous_time = time.time()
alpha = 0.98  # 互补滤波器的权重
dt = 0.0  # 时间间隔
roll = 0.0
pitch = 0.0
yaw = 0.0
# fig = plt.figure(figsize=(8, 8))
# ax = fig.add_subplot(111, projection='3d')

# # 设置坐标轴范围
# ax.set_xlim([-1, 1])
# ax.set_ylim([-1, 1])
# ax.set_zlim([-1, 1])

# # 设置坐标轴标签
# ax.set_xlabel('X 轴')
# ax.set_ylabel('Y 轴')
# ax.set_zlabel('Z 轴')

# # 原始坐标系
# original_axes = np.array([
#     [1, 0, 0],
#     [0, 1, 0],
#     [0, 0, 1]
# ])

# # 初始化绘图对象
# lines = [
#     ax.quiver(0, 0, 0, original_axes[0, 0], original_axes[0, 1], original_axes[0, 2], color='r', label='X (原始)'),
#     ax.quiver(0, 0, 0, original_axes[1, 0], original_axes[1, 1], original_axes[1, 2], color='g', label='Y (原始)'),
#     ax.quiver(0, 0, 0, original_axes[2, 0], original_axes[2, 1], original_axes[2, 2], color='b', label='Z (原始)')
# ]

# # 添加图例
# ax.legend()

if "__main__" == __name__:
    MPU_Init()
    acc_x_offset, acc_y_offset, acc_z_offset= get_median_offset()
    norm_values = []
    ax_values = []
    ay_values = []
    az_values = []
    pos_values = []
    vel_values = []
    last_vel = np.zeros((3,1))
    current_vel = np.zeros((3,1))
    current_acc = np.zeros((3,1))
    pos = np.zeros((3,1))
    last_acc = np.zeros((3,1))
    last_time = time.time()
    cnt = 0
    flag = True
    times = []
    last_Ax = None
    kf = kalman_filter(0.001,0.1)
    # Read Accelerometer raw value
    while True:
        # start_time = time.time()  # 记录循环开始时间
        acc_x, acc_y, acc_z= read_block_data()
        
        # acc_x = read_raw_data(ACCEL_XOUT_H)
        # acc_y = read_raw_data(ACCEL_YOUT_H)
        # acc_z = read_raw_data(ACCEL_ZOUT_H)

        Ax = acc_x - acc_x_offset
        Ay = acc_y - acc_y_offset
        Az = acc_z - acc_z_offset
        Ax = kf.kalman(Ax)
        dt = time.time() - last_time
        print(dt)
        last_time = time.time()
        current_acc = np.array([[Ax],[Ay],[Az]])
        # 计算角速度
        current_vel,pos = get_velPos(dt,flag)
        # if last_Ax is not None and abs(Ax-last_Ax) > 0.001 :
        pos_values.append(pos[0])
        ax_values.append(Ax)
        # 比较当前的 Ax 值和上一次的 Ax 值
        # if last_Ax is None or abs(Ax-last_Ax) > 0.005:
        #     print("Ax: %.2f" % Ax)
        #     flag = True
        
        # else:
        #      flag = False
        # if abs(Ax-last_Ax) > 0.01:
        # else:
        #     print('-')
        
        # 更新上一次的 Ax 值
        last_Ax = Ax
        if len(ax_values)%10000 > 1500:
            print("已收集1000个数据点，请输入 'q' 退出或按回车继续...")
            user_input = input()
            if user_input.lower() == 'q':
                plt.plot(ax_values)
                # plt.plot(pos_values)
                plt.xlabel('Time')
                plt.ylabel('Acceleration')
                plt.title('Acceleration over Time')
                plt.show()
                ax_values = []
                
            elif user_input.lower() == '':
                ax_values = []
                pos_values = []

    

        # print("Ax: %.2f" %Ax, "Ay: %.2f" %Ay, "Az: %.2f" %Az)
        # end_time = time.time()  # 记录循环结束时间
        # times.append(end_time - start_time)  # 计算并存储循环时间间隔

        # # 如果你想要在一定数量的读取后计算频率，可以添加一个计数器
        # if len(times) > 1000:
        #     break

        # 计算平均读取时间间隔
    # average_time_interval = sum(times) / len(times)

    # # 计算读取频率（每秒读取次数）
    # read_frequency = 1 / average_time_interval

    # print("平均读取时间间隔: %.4f 秒" % average_time_interval)
    # print("读取频率: %.2f Hz" % read_frequency)


        # current_acc = np.array([[Ax],[Ay],[Az-1]])
        # cur_time = time.time()
        # dt = cur_time - last_time
        # last_time = cur_time
        # current_vel,pos =  get_velPos(dt,flag)
        # if np.linalg.norm(current_acc-last_acc) < 0.02:
        #     cnt += 1
        # else:
        #     cnt = 0
        # if cnt > 10:
        #     flag = False
        # else:
        #     flag = True
        # print("Pos: ", pos)
        # print("Vel: ", current_vel)
        # print("Ax: %.2f" %Ax, "Ay: %.2f" %Ay, "Az: %.2f" %Az)

        # current_acc = np.array([[Ax],[Ay],[Az]])
            


        # Gx = gyro_x - gyro_x_offset
        # Gy = gyro_y - gyro_y_offset
        # Gz = gyro_z - gyro_z_offset
        
        
        # for _ in range(100):  # 假设循环100次
        # ax_values.append(Ax)
        # ay_values.append(Ay)
        # az_values.append(Az)
        # # # # pos_values.append(pos)
        # # # # vel_values.append(current_vel)
        # if len(ax_values) > 50:
        #     ax_values.pop(0)
        #     ay_values.pop(0)
        #     az_values.pop(0)
        
            # pos_values.pop(0)
            # vel_values.pop(0)
        # norm = np.linalg.norm([Ax, Ay, Az])
        # norm_values.append(norm)
        # if len(norm_values) > 1000:
        #     norm_values.pop(0)
        # print("norm:", norm)
        # x_values = [p[0] for p in pos_values]
        # y_values = [p[1] for p in pos_values]
        # z_values = [p[2] for p in pos_values]
            # sleep_us(1000)
        # if len(ax_values) == 1000:
        #     plt.plot(ax_values, label='Ax')
        #     plt.plot(ay_values, label='Ay')
        #     plt.plot(az_values, label='Az')

        #     plt.xlabel('Time')
        #     plt.ylabel('Acceleration')
        #     plt.title('Acceleration over Time')
        #     plt.legend()  # 添加图例以便区分不同的加速度分量
        #     plt.pause(0.01)
        # # 绘制 norm 随时间变化的图
        # plt.clf()
        # plt.plot(norm_values)
        # plt.xlabel('Time')
        # plt.ylabel('Norm')
        # plt.title('Norm over Time')
        # plt.pause(0.01)
        # if len(ax_values) == 50:
        #     plt.clf()
        #     # plt.plot(x_values, label='X')
        #     # plt.plot(y_values, label='Y')
        #     # plt.plot(z_values, label='Z')
        #     plt.plot(ax_values, label='Ax')
        #     # plt.plot(ay_values, label='Ay')
        #     # plt.plot(az_values, label='Az')

        #     # plt.xlabel('时间')
        #     # plt.ylabel('加速度')
        #     # plt.title('加速度随时间变化')
        #     # plt.legend()  # 添加图例
        #     plt.pause(0.01)
        #     end_time = time.time()
        #     print("Time: %.2f" % (end_time - last_time))
        #     last_time = end_time
        # # 计算角速度

        # roll = math.atan2(Ay, math.sqrt(Ax * Ax + Az * Az))/np.pi*180 
        # pitch = math.atan2(-Ax, math.sqrt(Ay * Ay + Az * Az)) /np.pi*180
        # sleep(0.01)

        # 获取当前时间
        # current_time = time.time()
        # dt = current_time - previous_time  # 计算时间间隔
        # previous_time = current_time

        
        

        # 互补滤波计算角度
        # roll =  roll_acc
        # pitch =  pitch_acc
        # print("Roll: %.2f" %roll, "Pitch: %.2f" %pitch)
        # rotation_matrix = rotation_matrix_from_euler(roll, pitch)
        # update(rotation_matrix)
        # print("Accelerometer X:", acc_x, "Accelerometer Y:", acc_y, "Accelerometer Z:", acc_z)
        # print("Gyroscope X:", gyro_x, "Gyroscope Y:", gyro_y, "Gyroscope Z:", gyro_z)
        # sleep(1)
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        # Ax = acc_x/16384.0
        # Ay = acc_y/16384.0
        # Az = acc_z/16384.0
        
        # Gx = gyro_x/131.0
        # Gy = gyro_y/131.0
        # Gz = gyro_z/131.0
        
        
        # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
        # sleep(1)

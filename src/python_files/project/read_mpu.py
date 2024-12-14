import sys
import smbus
from time import sleep
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import numpy as np

# MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
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

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)#使用x轴时钟
    
    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
def read_block_data():
    data_block = bus.read_i2c_block_data(Device_Address, ACCEL_XOUT_H, 14)

    # 解析数据并赋值给结构体
    acc_x = (data_block[0] << 8) | data_block[1]
    acc_y = (data_block[2] << 8) | data_block[3]
    acc_z = (data_block[4] << 8) | data_block[5]
    gyro_x = (data_block[8] << 8) | data_block[9]
    gyro_y = (data_block[10] << 8) | data_block[11]
    gyro_z = (data_block[12] << 8) | data_block[13]

    # 转换为有符号的16位整数
    acc_x = acc_x if acc_x < 32768 else acc_x - 65536
    acc_y = acc_y if acc_y < 32768 else acc_y - 65536
    acc_z = acc_z if acc_z < 32768 else acc_z - 65536
    gyro_x = gyro_x if gyro_x < 32768 else gyro_x - 65536
    gyro_y = gyro_y if gyro_y < 32768 else gyro_y - 65536
    gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536
    
	# 加上量程
    acc_x = acc_x / 16384.0
    acc_y = acc_y / 16384.0
    acc_z = acc_z / 16384.0
    gyro_x = gyro_x / 16.4
    gyro_y = gyro_y / 16.4
    gyro_z = gyro_z / 16.4


    return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    
    # Concatenate higher and lower value
    value = ((high << 8) | low)
    
    # To get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value
def calculate_acc_angles(acc_x, acc_y, acc_z):
    roll_acc = np.arctan2(acc_y, acc_z)
    pitch_acc = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2))
    return roll_acc, pitch_acc

# 从陀螺仪数据中计算姿态角的变化
def calculate_gyro_angles(gyro_x, gyro_y, gyro_z, dt,roll, pitch, yaw):
    roll_gyro = roll + gyro_x * dt
    pitch_gyro = pitch + gyro_y * dt
    yaw_gyro = yaw + gyro_z * dt
    return roll_gyro, pitch_gyro, yaw_gyro
def complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt, alpha,roll, pitch, yaw):
    roll_acc, pitch_acc = calculate_acc_angles(acc_x, acc_y, acc_z)
    roll_gyro, pitch_gyro, yaw_gyro = calculate_gyro_angles(gyro_x, gyro_y, gyro_z, dt,roll, pitch, yaw)
    
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    yaw = yaw_gyro  # 陀螺仪在yaw方向上没有漂移
    
    return roll, pitch, yaw
# 从姿态角计算旋转矩阵
def rotation_matrix_from_angles(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rz = np.array([
    #     [np.cos(yaw), -np.sin(yaw), 0],
    #     [np.sin(yaw), np.cos(yaw), 0],
    #     [0, 0, 1]
    # ])
    
    R =  Ry @ Rx
    return R
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.dt = 0.05  # 示例时间步长，单位为秒
        self.alpha = 0.1  # 示例权重参数
        self.last_vel = np.zeros(3)  # 示例速度
        # self.last_pos = np.zeros(3)  # 示例位置
        self.cur_vel = np.zeros(3)  # 示例速度
        self.cur_pos = np.zeros(3)  # 示例位置
        self.last_acc = np.zeros(3)  # 示例加速度
        self.cur_acc = np.zeros(3)  # 示例加速度
        self.countx = 0
        self.county = 0
        self.countz = 0
        self.flagx = False
        self.flagy = False
        self.flagz = False
        # 初始化姿态角
        # self.roll = 0.0
        # self.pitch = 0.0
        # self.yaw = 0.0
        self.offset_ax = -0.06
        self.offset_ay = -0.04
        self.offset_az = 0.98
        self.main_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.main_widget)
        layout = QtWidgets.QVBoxLayout(self.main_widget)

        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)

        self.setup_plots()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # Update every 50ms
    def process_data(self,Ax,Ay,Az,mean,var):
        if var[0] < 0.005:
            self.offset_ax = mean[0]
            # print(1111)
        if var[1] < 0.005:
            self.offset_ay = mean[1]
        if var[2] < 0.005:
            self.offset_az = mean[2]
        # print(f"val[0]={var[0]:.4f},val[1]={var[1]:.4f},val[2]={var[2]:.4f}")
        ax_real = Ax - self.offset_ax
        ay_real = Ay - self.offset_ay
        az_real = Az - self.offset_az
        # print(f"ax_real:{ax_real:.2f},ay_real:{ay_real:.2f},az_real:{az_real:.2f}")
        if abs(ax_real)<0.02:
            ax_real = 0
            self.countx = self.countx + 1
        else:
            self.countx = 0
        if abs(ay_real)<0.02:
            ay_real = 0
            self.county = self.county + 1
        else:
            self.county = 0
        if abs(az_real)<0.02:
            az_real = 0
            self.countz = self.countz + 1
        else:
            self.countz = 0
        self.cur_acc = np.array([ax_real,ay_real,az_real])
        # print(f"ax_real:{ax_real:.2f},ay_real:{ay_real:.2f}")
        # return ax_real,ay_real,az_real
    def integral(self,var):

        self.cur_vel = self.last_vel + (self.last_acc+self.cur_acc)*self.dt/2
        
        if ((self.last_vel[0] > 0 and self.cur_vel[0] < 0) or (self.last_vel[0] < 0 and self.cur_vel[0] > 0)) and var[0]>0.01:
            self.flagx = True
        if ((self.last_vel[1] > 0 and self.cur_vel[1] < 0) or (self.last_vel[1] < 0 and self.cur_vel[1] > 0)) and var[1]>0.01:
            self.flagy = True
        if ((self.last_vel[2] > 0 and self.cur_vel[2] < 0) or (self.last_vel[2] < 0 and self.cur_vel[2] > 0)) and var[2]>0.01:
            self.flagz = True

        if self.flagx:
            self.cur_vel[0] = 0
            self.last_vel[0] = 0
        if self.flagy:
            self.cur_vel[1] = 0
            self.last_vel[1] = 0
        if self.flagz:
            self.cur_vel[2] = 0
            self.last_vel[2] = 0
        
        self.cur_pos = self.cur_pos + (self.last_vel+self.cur_vel)*self.dt/2
        self.last_acc = self.cur_acc
        self.last_vel = self.cur_vel
        

        if self.countx > 1:
            self.flagx = False
            self.cur_vel[0] = 0
            # self.countx = 0
            # self.last_vel[0] = 0
        if self.county > 1:
            self.flagy = False
            self.cur_vel[1] = 0
            # # self.county = 0
            # self.last_vel[1] = 0
        if self.countz > 1:
            self.flagz = False
            self.cur_vel[2] = 0
            # # self.countz = 0
            # self.last_vel[2] = 0
    def setup_plots(self):
        self.p1 = self.graphWidget.addPlot(title="velocity")
        self.p2 = self.graphWidget.addPlot(title="position")
        self.graphWidget.nextRow()
        self.p3 = self.graphWidget.addPlot(title="Ax")
        self.p4 = self.graphWidget.addPlot(title="Ay")
        self.graphWidget.nextRow()
        self.p5 = self.graphWidget.addPlot(title="Az")
        self.p6 = self.graphWidget.addPlot(title="Gyro Magnitude")
        # self.p3 = self.graphWidget.addPlot(title="Ax")
        # self.graphWidget.nextRow()
        # self.p4 = self.graphWidget.addPlot(title="Ay")
        # self.graphWidget.nextRow()
        # self.p5 = self.graphWidget.addPlot(title="Az")
        # Create curves
        # self.curve_ax = self.p1.plot(pen='r', name='x')
        # self.curve_ay = self.p1.plot(pen='g', name='y')
        # self.curve_az = self.p1.plot(pen='b', name='z')
        # self.curve_gx = self.p2.plot(pen='r', name='x')
        # self.curve_gy = self.p2.plot(pen='g', name='y')
        # self.curve_gz = self.p2.plot(pen='b', name='z')
        self.curve_velx = self.p1.plot(pen='r', name='x')
        self.curve_posx = self.p2.plot(pen='r', name='x')
        self.curve_accx = self.p1.plot(pen='g', name='ax')
        self.curve_ax_single = self.p3.plot(pen='r')
        self.curve_ay_single = self.p4.plot(pen='g')
        self.curve_az_single = self.p5.plot(pen='b')
        self.curve_ax_another = self.p3.plot(pen='b', name='Another Ax')
        self.curve_ay_another = self.p4.plot(pen='r', name='Another Ay')
        self.curve_az_another = self.p5.plot(pen='g', name='Another Az')

        # self.curve_g_mag = self.p6.plot(pen='w')

        # Initialize data arrays
        self.data_ax = np.zeros(100)
        self.data_ay = np.zeros(100)
        self.data_az = np.zeros(100)
        self.data_ax_another = np.zeros(100)
        self.data_ay_another = np.zeros(100)
        self.data_az_another = np.zeros(100)
        self.data_velx = np.zeros(100)
        self.data_posx = np.zeros(100)
        self.data_accx = np.zeros(100)
        # self.data_gx = np.zeros(100)
        # self.data_gy = np.zeros(100)
        # self.data_gz = np.zeros(100)

    def update(self):
        # Read sensor data
        acc_x = np.zeros(5)
        acc_y = np.zeros(5)
        acc_z = np.zeros(5)
        gyro_x = np.zeros(5)
        gyro_y = np.zeros(5)
        gyro_z = np.zeros(5)
        mean = np.zeros(3)
        var = np.zeros(3)
        for i in range(5):
            acc_x[i], acc_y[i], acc_z[i], gyro_x[i], gyro_y[i], gyro_z[i] = read_block_data()
        
        # acc_x = read_raw_data(ACCEL_XOUT_H)
        # acc_y = read_raw_data(ACCEL_YOUT_H)
        # acc_z = read_raw_data(ACCEL_ZOUT_H)
        # gyro_x = read_raw_data(GYRO_XOUT_H)
        # gyro_y = read_raw_data(GYRO_YOUT_H)
        # gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        # Convert data
        Ax = np.mean(acc_x)
        Ay = np.mean(acc_y)
        Az = np.mean(acc_z) 

        
        # if np.abs(Ax) < 0.01:
        #     Ax = 0.0
        # if np.abs(Ay) < 0.01:
        #     Ay = 0.0
        # if np.abs(Az) < 0.01:
        #     Az = 0.0
        # G_x = np.mean(gyro_x)
        # G_y = np.mean(gyro_y)
        # G_z = np.mean(gyro_z)
        # self.roll, self.pitch, self.yaw = complementary_filter(Ax, Ay, Az, G_x, G_y, G_z, self.dt, self.alpha,self.roll, self.pitch, self.yaw)
        # rotation_matrix = rotation_matrix_from_angles(self.roll, self.pitch, self.yaw)
        # ax_real, ay_real, az_real = rotation_matrix @ np.array([Ax, Ay, Az])
        # print(f"ax_real: {ax_real:.2f}, ay_real: {ay_real:.2f}, az_real: {az_real:.2f}")
        # Ax = acc_x/16384.0
        # Ay = acc_y/16384.0
        # Az = acc_z/16384.0
        # Gx = gyro_x/131.0
        # Gy = gyro_y/131.0
        # Gz = gyro_z/131.0
        
        # Update data arrays
        self.data_ax = np.roll(self.data_ax, -1)
        self.data_ax[-1] = Ax
        # self.data_ax[-1] = ax_real
        self.data_ay = np.roll(self.data_ay, -1)
        self.data_ay[-1] = Ay
        # self.data_ay[-1] = ay_real
        self.data_az = np.roll(self.data_az, -1)
        self.data_az[-1] = Az
        # self.data_az[-1] = az_real
        self.data_velx = np.roll(self.data_velx, -1)
        self.data_velx[-1] = self.cur_vel[0]*10
        self.data_posx = np.roll(self.data_posx, -1)
        self.data_posx[-1] = self.cur_pos[0]*10
        self.data_accx = np.roll(self.data_accx, -1)
        self.data_accx[-1] = self.cur_acc[0]
        

        # mean_ax = np.mean(self.data_ax)
        # var_ax = np.var(self.data_ax)
        # mean_ay = np.mean(self.data_ay)
        # var_ay = np.var(self.data_ay)
        # mean_az = np.mean(self.data_az)
        # var_az = np.var(self.data_az)
        size=5
        mean_ax = np.mean(self.data_ax[-size:])
        var_ax = np.var(self.data_ax[-size:])
        mean_ay = np.mean(self.data_ay[-size:])
        var_ay = np.var(self.data_ay[-size:])
        mean_az = np.mean(self.data_az[-size:])
        var_az = np.var(self.data_az[-size:])
        mean = np.array([mean_ax,mean_ay,mean_az])
        var = np.array([var_ax,var_ay,var_az])
        self.process_data(Ax,Ay,Az,mean,var)

        self.integral(var)
        print(f"flagx:{self.flagx},flagy:{self.flagy},flagz:{self.flagz}")
        print(f"var[0]:{var[0]:.4f},var[1]:{var[1]:.4f},var[2]:{var[2]:.4f}")

        self.data_ax_another = np.roll(self.data_ax_another, -1)
        # self.data_ax_another[-1] = self.cur_acc[0]
        self.data_ax_another[-1] = self.cur_vel[0]*10
        # self.data_ax_another[-1] = ax_real
        # self.data_ax_another[-1] = 0
        self.data_ay_another = np.roll(self.data_ay_another, -1)
        # self.data_ay_another[-1] = self.cur_acc[1]
        self.data_ay_another[-1] = self.cur_vel[1]*10
        # self.data_ay_another[-1] = ay_real
        # self.data_ay_another[-1] = 0
        self.data_az_another = np.roll(self.data_az_another, -1)
        # self.data_az_another[-1] = self.cur_acc[2]
        self.data_az_another[-1] = self.cur_vel[2]*10
        # self.data_az_another[-1] = az_real
        # self.data_az_another[-1] = 0
        # Print or use mean and variance
        print(f"cur_vel: {self.cur_vel[0]:.2f}, cur_pos: {self.cur_pos[0]:.2f}")
        # print(f"Mean Ax: {mean_ax:.2f}, Var Ax: {var_ax:.2f}")
        # print(f"Mean Ay: {mean_ay:.2f}, Var Ay: {var_ay:.2f}")
        # print(f"Mean Az: {mean_az:.2f}, Var Az: {var_az:.2f}")
        # if var_ax > 0.01:
        #     self.data_ax_another[-1]=0.2
        # if var_ay > 0.01:
        #     self.data_ay_another[-1]=0.2
        # if var_az > 0.01:
        #     self.data_az_another[-1]=0.2
        # self.data_ax_another[-1] = ax_real
        # self.data_ay_another[-1] = ay_real
        # self.data_az_another[-1] = az_real

        self.curve_ax_another.setData(self.data_ax_another)
        self.curve_ay_another.setData(self.data_ay_another)
        self.curve_az_another.setData(self.data_az_another)

        self.curve_velx.setData(self.data_velx)
        self.curve_posx.setData(self.data_posx)
        self.curve_accx.setData(self.data_accx)
        # self.data_gx = np.roll(self.data_gx, -1)
        # self.data_gx[-1] = Gx
        # self.data_gy = np.roll(self.data_gy, -1)
        # self.data_gy[-1] = Gy
        # self.data_gz = np.roll(self.data_gz, -1)
        # self.data_gz[-1] = Gz
        
        # Calculate gyro magnitude
        # g_mag = np.sqrt(Gx**2 + Gy**2 + Gz**2)
        
        # Update plots
        # self.curve_ax.setData(self.data_ax)
        # self.curve_ay.setData(self.data_ay)
        # self.curve_az.setData(self.data_az)
        # self.curve_gx.setData(self.data_gx)
        # self.curve_gy.setData(self.data_gy)
        # self.curve_gz.setData(self.data_gz)
        self.curve_ax_single.setData(self.data_ax)
        self.curve_ay_single.setData(self.data_ay)
        self.curve_az_single.setData(self.data_az)
        # self.curve_g_mag.setData(np.full_like(self.data_gx, g_mag))

if __name__ == '__main__':
    MPU_Init()
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec())
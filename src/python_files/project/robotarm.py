import math
from lu9685 import DeviceController
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import sleep
from joy_stick import Joystick
# from read_mpu import MPU6050
import RPi.GPIO as GPIO
import time
import tkinter as tk
from tkinter import messagebox
import threading
class RobotArm:
    def __init__(self):
        self.speed = 0.3
        self.base_height = 5
        self.link1_length = 4.5
        self.link2_length = 4
        self.offset = 5
        self.grip = 0
        self.flag = True
        # self.thetas = np.array([90,219,112,242])
        self.thetas = np.array([0,203,141,254])
        self.outbound=False
        bus_number = 1
        device_address = 0x00  # 示例地址
        resetReg = 0xFB  # 示例寄存器
        command = 0xFB  # 示例命令
        self.controller = DeviceController(bus_number, device_address, resetReg, command)
        # self.controller.resetDevice()
        self.pos = np.array([0,0,0])
        self.recorded_data = []
        self.theta = 0
        self.recording_enabled = False
    def control_joints(self, target_angles):
    # 控制关节角度
    # 这里的angles是四个关节角度的列表
    # 判断当前关节角和目标关节角的差值，如果大于15度，就分批控制
        
        current_angles = self.thetas
        
        # 计算每个关节的角度差
        angle_diffs = np.abs(target_angles - current_angles)
        
        # 检查是否有角度差大于15度
        if np.any(angle_diffs > 15):
            # 如果有，我们需要分批控制
            # 计算需要的步数
            steps = np.ceil(angle_diffs / 15).astype(int)
            max_steps = np.max(steps)
            
            # 分批调整角度
            for step in range(1, max_steps + 1):
                new_angles = current_angles + (target_angles - current_angles) * (step / max_steps)
                
                self.thetas = new_angles
                # 这里可以添加代码来实际控制关节，例如发送信号给硬件
                self.controller.set_group_angle(new_angles)
                sleep(0.1)
                print(f"Step {step}: {self.thetas}")  # 示例输出
        else:
            # 如果角度差都不大于15度，直接设置为目标角度
            self.thetas = target_angles
            # 这里可以添加代码来实际控制关节，例如发送信号给硬件
            self.controller.set_group_angle(target_angles)
            print(f"Direct Set: {self.thetas}")  # 示例输出
    def vel2pos(self, vel,theta=0,need_project=False):
        # 速度转位置
        new_pos = self.pos + vel*self.speed
        print(f"ori={new_pos}")
        print(f"self.pos1={self.pos}")
        new_theta = self.theta + theta*self.speed
        if need_project:
            new_pos = self.project_position_v2(new_pos[0], new_pos[1], new_pos[2],new_theta)
        return new_pos
    def record_data(self, new_pos, grip):
        if self.recording_enabled:
            self.recorded_data.append((new_pos[0],new_pos[1],new_pos[2], grip))
    # def control_ToPoint(self, x=None, y=None, z=None, new_pos=None,need_project=True):
    #     # 控制机械臂到达指定点
    #     # 这里的x,y,z是目标点的坐标
    #     # 计算每个关节的角度
    #     if new_pos is not None:
    #         x, y, z = new_pos[0],new_pos[1],new_pos[2]
    #     elif x is None or y is None or z is None:
    #         raise ValueError("Invalid arguments, either provide x, y, z or new_pos")
    #     if need_project:
    #         self.pos = self.project_position_v2(x, y, z)
    #     else:
    #         self.pos = np.array([x,y,z])
    #     print(f"Current Position: {self.pos}")
    #     theta1, theta2, theta3,theta4 = self.inverse_kinematics(self.pos[0], self.pos[1], self.pos[2])
    #     # 控制关节角度
    #     if self.flag == True:
    #         angles = np.array([theta1, theta2, theta3,theta4]).astype(np.int32)
    #         self.control_joints(angles)
    #         if self.recording_enabled:
    #             self.record_data(self.pos, self.grip)
    #     else:
    #         print("Inverse Kinematics Failed!")
    def set_group_pos_grip(self, pos, grip):
        # 设置机械臂的位置和夹爪的状态
        # 这里的pos是机械臂的位置，grip是夹爪的状态
        # 计算每个关节的角度
        # new_pos = self.project_position(pos[0], pos[1], pos[2])
        self.control_ToPoint_v2(new_pos = pos)
        self.set_grip(grip)
    def set_grip(self, grip):
        # 设置夹爪的状态
        # 这里的grip是夹爪的状态
        self.grip = np.clip(grip,100,180)
        print(f"Grip: {self.grip}")
        self.controller.set_channel5_angle(self.grip)
    def generate_points(self):
        points = []
        count = 0
        for x in range(-9, 9, 2):
           
                for z in range(5, 13, 2):
                    if abs(x) >=3:
                        self.flag = True  # 重置flag为True
                        self.inverse_kinematics(x, y, z)
                        points.append((x, y, z, self.flag))
                        if self.flag ==True:
                            count = count + 1
        print(f"共生成{count}个点")
        return points    
    def set_theta(self, theta):
        self.theta = theta
    def vel_theta(self, vel):
        
        new_theta = self.theta + vel*self.speed
        if new_theta > np.pi/2:
            new_theta = np.pi/2
        elif new_theta < -np.pi/2:
            new_theta = -np.pi/2
        print(f"new theta: {new_theta}")
        return new_theta
    def control_ToPoint_v2(self, x=None, y=None, z=None, new_pos=None,need_project=False):
        # 控制机械臂到达指定点
        # 这里的x,y,z是目标点的坐标
        # 计算每个关节的角度
        print(f"ori={new_pos}")
        print(f"self.pos1={self.pos}")
        if new_pos is not None:
            x, y, z = new_pos[0],new_pos[1],new_pos[2]
            
        elif x is None or y is None or z is None:
            raise ValueError("Invalid arguments, either provide x, y, z or new_pos")
        # if need_project:
        #     self.pos = self.project_position_v2(x, y, z)
        # else:
        #     self.pos = np.array([x,y,z])
        # print(f"Current Position: {self.pos}")
        theta1, theta2, theta3,theta4 = self.inverse_kinematics(x, y, z,use_v2=True)
        # print(f"current positon: {self.pos}")
            
        # 控制关节角度
        if self.flag == True:
            self.pos = np.array([x,y,z])
            angles = np.array([theta1, theta2, theta3,theta4]).astype(np.int32)
            self.control_joints(angles)
            if self.recording_enabled:
                self.record_data(self.pos, self.grip)
        else:
            print("Inverse Kinematics Failed!")
    def inverse_kinematics(self, x, y, z,use_v2=True,need_offset=True):
        # calculate the angle of the first joint，采用投影的方案，计算底座的旋转角度theta1
        if y<0:
            self.flag = False
            return 0,0,0,0
        theta1 = math.atan2(x,y)
        new_z = z-np.sin(self.theta)*self.offset#由于我得到的是三轴机械臂的运动学解算，因此先将机械臂转换到求解第三个关节，最后再求解末端
        if need_offset:
            projection = math.sqrt(x**2 + y**2)-self.offset
        else:
            projection = math.sqrt(x**2 + y**2)
        short = abs(z - self.base_height)#这个对应
        third_len = math.sqrt(short**2+projection**2)
        #上面三个长度计算是没有考虑末端姿态的，下面三个的计算是考虑末端姿态theta的
        if use_v2:
            projection = math.sqrt(x**2 + y**2)-np.cos(self.theta)*self.offset
            short = abs(z-np.sin(self.theta)*self.offset - self.base_height)
            third_len = math.sqrt(short**2+projection**2)
           
        try:#这里根据三角函数公式计算theta2和theta3对应第二个和第三个舵机旋转角度
            if self.base_height > new_z:
                temp_theta2 = math.atan2(projection, short)
                temp_theta3 = math.acos((self.link1_length**2 + third_len**2 - self.link2_length**2) / (2*self.link1_length*third_len))
                theta2 = temp_theta2 + temp_theta3
            if self.base_height <= new_z:
                temp_theta2 = math.atan2(short, projection)
                temp_theta3 = math.acos((self.link1_length**2 + third_len**2 - self.link2_length**2) / (2*self.link1_length*third_len))
                theta2 = temp_theta2 + temp_theta3 + math.pi/2
            # calculate the angle of the third joint
            theta3 = math.acos((self.link1_length**2 + self.link2_length**2 - third_len**2) / (2*self.link1_length*self.link2_length))
        except ValueError:
            self.flag = False    #如果无法计算就直接返回0000
            return 0,0,0,0
        theta4 = theta2 + theta3 -math.pi/2 - self.theta#姿态控制，计算第四个舵机的旋转角度
     #将四个弧度转换为角度
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2)
        theta3 = math.degrees(theta3)
        theta4 = math.degrees(theta4)
        self.flag = True
        return (theta1, theta2, theta3,theta4)
    def plot_points(self, points):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 分离出True和False的点
        true_points = [point for point in points if point[3]]
        false_points = [point for point in points if not point[3]]

        # 提取坐标
        true_x, true_y, true_z = zip(*[(p[0], p[1], p[2]) for p in true_points]) if true_points else ([], [], [])
        # false_x, false_y, false_z = zip(*[(p[0], p[1], p[2]) for p in false_points]) if false_points else ([], [], [])

        # 绘制点
        ax.scatter(true_x, true_y, true_z, c='g', marker='o', label='True')
        # ax.scatter(false_x, false_y, false_z, c='r', marker='x', label='False')

        # 设置标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 设置图例
        ax.legend()

        # 显示图像
        plt.show()
    def project_position(self, x, y, z):

        prj = (x**2 + y**2)**0.5
        new_z = np.clip(z, 5, 13)
        
        
        if z<=7:
            new_prj = np.clip(prj, 9,13)
        elif z<=8:
            new_prj = np.clip(prj, 9,12)
        elif z<=9:
            new_prj = np.clip(prj, 7,12)
        elif z<=10:
            new_prj = np.clip(prj, 7,11)
        elif z<=11:
            new_prj = np.clip(prj, 6,11)
        elif z<=12:
            new_prj = np.clip(prj, 5,9)
        else:
            new_prj = np.clip(prj, 4,7)
        # new_x = new_prj*math.cos(angle)
        # new_y = new_prj*math.sin(angle)
        # if np.abs
        if new_prj == prj and new_z == z:
            new_pos = np.array([x,y,z])
        else:
            new_pos = self.pos#返回当前的位置
        print(f"Projected Position: {new_pos}")
        return new_pos
    def project_position_v2(self, x, y, z,theta=0):
        offset_x = np.cos(theta)*self.offset
        offset_z = np.sin(theta)*self.offset
        prj = (x**2 + y**2)**0.5-offset_x+5
        new_z = np.clip(z-offset_z, 5, 13)
        # if grip is not None:
        #     new_theta = self.vel_theta(grip)
        
        if new_z<=7:
            new_prj = np.clip(prj, 6,12)
        elif new_z<=8:
            new_prj = np.clip(prj, 5.5,11.5)
        elif new_z<=9:
            new_prj = np.clip(prj, 5,11)
        elif new_z<=10:
            new_prj = np.clip(prj, 4,10)#3.5,10
        elif new_z<=11:
            new_prj = np.clip(prj, 3,9)#3,9
        elif new_z<=12:
            new_prj = np.clip(prj, 2.5,8)#2.5-8
        else:
            new_prj = np.clip(prj, 2.5,7) #z=13,2.5,7
            
        # print(f"Projected Position: {new_prj}, {new_z}")
        # print(f"new_prj={new_prj}, new_z={new_z}, y={y}")
        # print(f"{prj}")
        print(f"theta={theta}")
        # print(f"{new_prj==prj}, {new_z==z-offset_z}, {y>=0}")
        if new_prj == prj and new_z == z-offset_z and y>=0:
            # if grip is not None:
            #     theta) = new_theta
            self.theta = theta
            new_pos = np.array([x,y,z])
        else:
            new_pos = self.pos#返回当前的位置
            
        print(f"Projected Position: {new_pos}")
        return new_pos
    def go_home(self):
        # self.control_ToPoint(4,0,12)
        self.control_ToPoint_v2(0,4.2,12.8)
    def velocity_control(self, vel):
        # 这里的delta_xyz是位移增量
        # 计算每个关节的角度
        new_pos = self.pos + vel*self.speed
        self.control_ToPoint(new_pos[0], new_pos[1], new_pos[2])
        # 控制关节角度
    def set_speed(self, speed):
        self.speed = speed
    def vel_grip(self, grip):
        # 这里的grip是夹爪的力度
        # 这里可以添加代码来控制夹爪的力度
        self.grip = np.clip(self.grip +grip*self.speed*10,100,180)
        print(f"Grip: {self.grip}")
        self.controller.set_channel5_angle(self.grip)
    def save_points_to_file(self, points=None, filename='points.txt'):
        with open(filename, 'w') as file:
            if points is None:
                points = self.recorded_data
            for point in points:
                x, y, z, grip = point
                file.write(f"({x}, {y}, {z}, {grip})\n")
            self.recorded_data = []
    def load_points_from_file(self, filename='points.txt'):
        with open(filename, 'r') as file:
            lines = file.readlines()
            for line in lines:
                try:
                    x, y, z, grip = map(float, line.strip().strip('()').split(','))
                    self.set_group_pos_grip((x,y,z), grip)
                    sleep(0.1)
                except ValueError:
                    print(f"Invalid line in file: {line}")
    def enable_recording(self):
        self.recording_enabled = True
        print("Recording Enabled")

    def disable_recording(self):
        self.recording_enabled = False
        print("Recording Disabled")
menu_visible = False
def show_menu():
    global menu_visible
    if menu_visible:
        return  # 如果菜单已经显示，则不显示新的菜单

    menu_visible = True

    def enable_record():
        if not robot.recording_enabled:
            robot.enable_recording()
        else:
            messagebox.showinfo("信息", "记录已经在进行中")
        # menu.destroy()

    def disable_record():
        if robot.recording_enabled:
            robot.disable_recording()
            robot.save_points_to_file()
        else:
            messagebox.showinfo("信息", "没有正在进行的记录")
        # menu.destroy()

    def playback_record():
        if not robot.recording_enabled:
            robot.load_points_from_file()
        else:
            messagebox.showinfo("信息", "请先结束记录再进行回放")
        # menu.destroy()

    def cancel_menu():
        global menu_visible
        menu.destroy()
        menu_visible = False
        # main_loop()

    def exit_program():
        global stop
        menu.destroy()
        stop = True
        # root.destroy()
        exit()

    menu = tk.Toplevel(root)
    menu.title("菜单")

    tk.Button(menu, text="开启记录", command=enable_record).pack(pady=5)
    tk.Button(menu, text="结束记录", command=disable_record).pack(pady=5)
    tk.Button(menu, text="回放记录", command=playback_record).pack(pady=5)
    tk.Button(menu, text="取消菜单", command=cancel_menu).pack(pady=5)
    tk.Button(menu, text="结束程序", command=exit_program).pack(pady=5)
stop = False
loop_running = False
# def stop_loop():
#     global loop_running
#     loop_running = False
# def run_loop():
#     while loop_running:
#         root.update()  # 更新GUI事件
#         vel, grip = js.read_values()
#         if vel[0] != 0 or vel[1] != 0 or vel[2] != 0 or grip != 0:
#             print(f"Velocity: {vel}, Grip: {grip}")
#             new_pos = robot.vel2pos(vel)
#             robot.control_ToPoint_v2(new_pos=new_pos)
#             robot.vel_grip(grip)
#             time.sleep(0.1)
#         else:
#             time.sleep(0.1)  # 如果没有输入，等待一段时间再检查
# def start_loop():
#     global loop_running
#     loop_running = True
#     run_loop()
button_pin = 23
def main_loop():
    while True:
        root.update()  # 更新GUI事件
        if not loop_running:
            vel, grip = js.read_values()
            if vel[0] != 0 or vel[1] != 0 or vel[2] != 0 or grip != 0:
                print(f"Velocity: {vel}, Grip: {grip}")
                new_pos = robot.vel2pos(vel)
                robot.control_ToPoint_v2(new_pos=new_pos)
                robot.vel_grip(grip)
                time.sleep(0.1)
            else:
                time.sleep(0.1)  # 如果没有输入，等待一段时间再检查
        if stop:
            break
        # 检查按钮状态
        if GPIO.input(button_pin) == GPIO.LOW:
            
            time.sleep(0.2)  # 防止抖动
            if GPIO.input(button_pin) == GPIO.LOW:
                # stop_loop()
                
                show_menu()
                # while loop_running:  # 阻塞主循环直到菜单关闭
                #     root.update()  # 更新GUI事件
                #     time.sleep(0.1)       
if __name__ == '__main__':
    robot = RobotArm()
    js = Joystick()
    # mpu = MPU6050()
    root = tk.Tk()
    root.title("主窗口")
    GPIO.setmode(GPIO.BCM)

    # 定义按钮GPIO引脚
    button_pin = 23

    # 设置按钮引脚为输入，并启用上拉电阻（按钮按下时为低电平）
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # # 定义一个防抖函数
    # def debounce_button(channel):
    #     # 等待一段时间以防止抖动
    #     time.sleep(0.2)
    #     # 再次检查按钮状态
    #     if GPIO.input(channel) == GPIO.LOW:
    #         # if not robot.recording_enabled:
    #         #     robot.enable_recording()
    #         # else:
    #         #     robot.disable_recording()
    #         #     robot.save_points_to_file()
    #         print("按钮已按下")
    #         show_menu()
    # GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=debounce_button, bouncetime=200)
    # 设置一个标签，仅用于示例
    tk.Label(root, text="按下按钮进入菜单").pack(pady=20)
    robot.go_home()
    main_loop()
    # main_loop_thread = threading.Thread(target=main_loop)
    # main_loop_thread.daemon = True
    # main_loop_thread.start()

    # # 运行主GUI循环
    # root.mainloop()
    # robot.load_points_from_file()
    
    # vel = np.zeros(3)
    # grip = 0
    # 设置GPIO模式为BCM
   

    # 添加中断检测事件
   
    # 
    # main_loop()
    # root.mainloop()
    # while True:
    #     vel,grip = js.read_values()
    #     if vel[0]!=0 or vel[1]!=0 or vel[2]!=0 or grip!=0:
    #         print(f"Velocity: {vel}, Grip: {grip}")
    #         new_pos = robot.vel2pos(vel)
    #         robot.control_ToPoint_v2(new_pos=new_pos)
    #         robot.vel_grip(grip)
    #         sleep(0.1)
    #     vel,grip = js.read_values()
        # ###
        # if vel[0]!=0 or vel[1]!=0 or vel[2]!=0 or grip!=0:
        #     print(f"Velocity: {vel}, Grip: {grip}")
        #     # robot.vel_theta(grip)
        #     new_pos = robot.vel2pos(vel,grip = grip)
        #     robot.control_ToPoint_v2(new_pos=new_pos)
            
        #     sleep(0.1)
        
    #待测试
    # while True:
    #     x,y,z = input("请输入坐标(x,y,z):").split(",")
    #     x,y,z = float(x),float(y),float(z)
    #     theta1, theta2, theta3,theta4 = robot.inverse_kinematics(x,y,z,need_offset=False)
    #     robot.control_ToPoint(x,y,z,need_project=False)
    # 待测试
    
    # robot.velocity_control(vel)
    # bus_number = 1
    # device_address = 0x00  # 示例地址
    # resetReg = 0xFB  # 示例寄存器
    # command = 0xFB  # 示例命令
    # controller = DeviceController(bus_number, device_address, resetReg, command)
# def save_points_to_file(self, points, filename='points.txt'):
#         with open(filename, 'w') as file:
#             for point in points:
#                 x, y, z, flag = point
#                 file.write(f"({x}, {y}, {z}): {str(flag)}\n")
    # 重置设备
    # controller.resetDevice()
    # controller.resetDevice()
    # controller.resetDevice()
    # 设置通道角度
    #z轴最大11，最小5
    #x轴最大9，最小5
    #y轴最大9，最小5
    # x,y,z = 9.5,0,5
    # x,y,z = 11.5,0,5
    # x,y,z = 9.5,0,9
    # x,y,z = 9.5,0,12
    # x,y,z = 7.5,0,13
    # x,y,z = 5.5,0,13
    # x,y,z = 3.5,0,13
    # x,y,z = 5,5,13
    # x,y,z = 4,4,13.5
    # # theta1, theta2, theta3,theta4 = robot.inverse_kinematics(x,y,z)
    # # robot.control_ToPoint(x,y,z)
    # points = robot.generate_points()
    # robot.save_points_to_file(points)

    # # robot.plot_points(points)
    # # angles = np.array([theta1, theta2, theta3,theta4])
    # while True:
    #     x,y,z = input("请输入坐标(x,y,z):").split(",")
    #     x,y,z = float(x),float(y),float(z)
    #     theta1, theta2, theta3,theta4 = robot.inverse_kinematics(x,y,z)
    #     robot.control_ToPoint(x,y,z)
    # robot.control_joints(angles)
    # angles = np.array([0,180,180,180]).astype(np.int32)
    # robot.control_joints(angles)
    # print(theta1, theta2, theta3,theta4)
    # controller.set_group_angle(angles)
    # controller.set_group_angle(angles)
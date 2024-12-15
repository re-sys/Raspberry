import math
from lu9685 import DeviceController
import numpy as np
class RobotArm:
    def __init__(self):
        
        self.base_height = 5
        self.link1_length = 4.5
        self.link2_length = 4
        self.offset = 5
    def inverse_kinematics(self, x, y, z):
        # calculate the angle of the first joint
        theta1 = math.atan2(x,y)
        projection = math.sqrt(x**2 + y**2)-self.offset
        short = abs(z - self.base_height)
        third_len = math.sqrt(short**2+projection**2)
        
        # calculate the angle of the second joint
        if self.base_height > z:
            
            
            temp_theta2 = math.atan2(projection, short)
            temp_theta3 = math.acos((self.link1_length**2 + third_len**2 - self.link2_length**2) / (2*self.link1_length*third_len))
            theta2 = temp_theta2 + temp_theta3#这里对应的theta角度就是
        if self.base_height <= z:
            
            
            temp_theta2 = math.atan2(short, projection)
            temp_theta3 = math.acos((self.link1_length**2 + third_len**2 - self.link2_length**2) / (2*self.link1_length*third_len))
            theta2 = temp_theta2 + temp_theta3 + math.pi/2
        # calculate the angle of the third joint
        theta3 = math.acos((self.link1_length**2 + self.link2_length**2 - third_len**2) / (2*self.link1_length*self.link2_length))
        theta4 = theta2 + theta3 -math.pi/2
        # return the angles in degrees
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2)
        theta3 = math.degrees(theta3)
        theta4 = math.degrees(theta4)
        return (theta1, theta2, theta3,theta4)
if __name__ == '__main__':
    robot = RobotArm()
    bus_number = 1
    device_address = 0x00  # 示例地址
    resetReg = 0xFB  # 示例寄存器
    command = 0xFB  # 示例命令
    controller = DeviceController(bus_number, device_address, resetReg, command)

    # 重置设备
    controller.resetDevice()

    # 设置通道角度
    #z轴最大13，最小5
    x,y,z = 9.5,0,5
    x,y,z = 11.5,0,5
    x,y,z = 9.5,0,9
    x,y,z = 9.5,0,12
    x,y,z = 7.5,0,13
    x,y,z = 5.5,0,13
    x,y,z = 3.5,0,13
    x,y,z = 5,5,13
    theta1, theta2, theta3,theta4 = robot.inverse_kinematics(x,y,z)
    angles = np.array([theta1, theta2, theta3,theta4]).astype(np.int32)
    # angles = np.array([90,180,90,180]).astype(np.int32)
    print(theta1, theta2, theta3,theta4)
    controller.set_group_angle(angles)
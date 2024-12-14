import smbus
import time
import math
import tkinter as tk
from tkinter import ttk
# 创建SMBus实例，指定I2C总线编号（通常是1）
bus = smbus.SMBus(1)

# PCA9685模块的I2C地址
address = 0x00

# 模块频率设置
frequency = 60

# PCA9685寄存器地址
Address = 0x00

# 设备地址（7位）
device_address = 0x00  # 这里需要替换为你的具体设备地址

# 要写的两个命令
resetReg = 0xFB  # 第一个命令
command = 0xFB  # 第二个命令
teaching_data = []
def resetDevice():
    
    try:
        # 连续写两个命令字节
        bus.write_byte_data(device_address, resetReg, command)
        # temp = bus.read_byte_data(device_address, resetReg)
        print(f"成功读取寄存器{resetReg}的值: {1}")
        # print(f"成功连续写入命令1: {command1} 和 命令2: {command2}")

    except IOError as e:
        print(f"写入命令时发生错误: {e.strerror}")

def controlChannel(channel, angle):
    bus.write_byte_data(device_address, channel, angle)

# def on_slider_change(event, channel):
#     value = int(slider_var.get())
#     controlChannel(channel, value)
def on_slider_change(event, channel, slider_var, angle_label):
    value = int(slider_var.get())
    angle_label.config(text=f"通道 {channel}: {value}°")
    controlChannel(channel, value)

def save_teaching_data():
    with open("teaching_data.txt", "w") as file:
        for data in teaching_data:
            file.write(f"Channel {data[0]}: {data[1]}\n")
    print("示教数据已保存到 teaching_data.txt")

if __name__ == '__main__':
    root = tk.Tk()
    root.title("示教控制")
    initial_angles = {0: 0, 1: 10, 2: 55, 3: 90, 4: 0, 5: 90}
    # 创建一个变量来存储滑杆的值
    # slider_var = tk.IntVar()
    for i in range(0, 6):
        initial_angle = initial_angles.get(i, 0)  # 获取初始角度，如果不存在则默认为0
        slider_var = tk.IntVar(value=initial_angle)
        angle_label = ttk.Label(root, text=f"通道 {i}: {initial_angle}°")
        if i == 1:
            slider = ttk.Scale(root, from_=0, to=70, orient='horizontal', variable=slider_var, command=lambda event, ch=i, sv=slider_var, al=angle_label: on_slider_change(event, ch, sv, al))
        else:
            slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', variable=slider_var, command=lambda event, ch=i, sv=slider_var, al=angle_label: on_slider_change(event, ch, sv, al))
# slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', variable=slider_var, command=lambda event, ch=i: on_slider_change(event, ch))
        slider.set(initial_angle)  # 设置滑动条的初始角度
        slider.grid(column=0, row=i, padx=10, pady=10, sticky='ew')
          # 创建标签显示当前角度
        
        angle_label.grid(column=1, row=i, padx=10, pady=10, sticky='w')

    # 创建保存按钮
        # save_button = ttk.Button(root, text="保存示教数据", command=save_teaching_data)
        # save_button.grid(column=0, row=3, columnspan=2, padx=10, pady=10, sticky='ew')

    # 运行主循环
    root.mainloop()
    try:
        resetDevice()
        # controlChannel(0, 0)#底座可以随意配置
        # controlChannel(1, 10)#0-70,10是竖直，70就是下降到水平，顺时针转动
        # controlChannel(2, 55)#55度是相等，逆时针转动
        # controlChannel(3, 90)#90度是水平，顺时针转动
        # controlChannel(4, 180)0度是超前有偏移
        # controlChannel(5, 90)夹爪。
    finally:
        bus.close()



import lirc

def pasreset(data): #解析按键
    if data == 'echo "KEY_1"':
        print("1 按下") #遥控器按下1:
    elif data == 'echo "KEY_2"':
        print("2 按下") #遥控器按下2:
    elif data == 'echo "KEY_3"':
        print("3 按下") #遥控器按下3:

with lirc.LircdConnection("test.py",) as conn:
    while True:
        string = conn.readline()
        pasreset(string)
        #print("收到:",end = '')
        #print(type(string))

# import lirc
# import RPi.GPIO as GPIO

# # 配置 GPIO
# GPIO.cleanup()          # 清理之前的 GPIO 设置
# GPIO.setmode(GPIO.BCM)  # 使用 BCM 模式
# IR_PIN = 17              # 设置红外遥控信号检测引脚为 17

# # 设置引脚为输入并启用内部上拉电阻
# GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# # 初始化 LIRC
# conn = lirc.LircdConnection("test.py")

# def pasreset(data):  # 解析按键
#     if data == 'echo "KEY_1"':
#         print("1 按下")  # 遥控器按下1
#     elif data == 'echo "KEY_2"':
#         print("2 按下")  # 遥控器按下2
#     elif data == 'echo "KEY_3"':
#         print("3 按下")  # 遥控器按下3

# def callback(channel):
#     if GPIO.input(channel) == GPIO.LOW:  # 检测到低电平，即有信号
#         print("检测到红外遥控信号")
#         # 读取遥控器按键信息
#         string = conn.readline()  # 从 LIRC 读取数据
#         pasreset(string)          # 调用解析按键函数处理信息

# # 添加边沿检测，检测到信号变化时调用回调函数
# GPIO.add_event_detect(IR_PIN, GPIO.BOTH, callback)

# try:
#     print("等待检测红外遥控信号...")
#     while True:
#         pass

# except KeyboardInterrupt:
#     GPIO.cleanup()  # 清理 GPIO 设置
# finally:
#     GPIO.cleanup()  # 确保在退出时清理 GPIO 设置

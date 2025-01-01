import RPi.GPIO as GPIO
import time
import threading
import sys

# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)

# 定义LED连接的GPIO引脚
LED_PIN = 35

# 设置GPIO引脚为输出模式
GPIO.setup(LED_PIN, GPIO.OUT)

# 用于线程控制的标志变量
running = True

def monitor_input():
    global running
    while running:
        key = input()  # 等待用户输入
        if key.lower() == 'q':
            running = False  # 设置标志以结束LED控制循环

# 提示用户
print("Press 'q' to exit the program.")

# 启动监测线程
input_thread = threading.Thread(target=monitor_input)
input_thread.start()

try:
    while running:
        # 点亮LED
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(1)  # 亮1秒

        # 熄灭LED
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(1)  # 灭1秒

except KeyboardInterrupt:
    # 捕捉按键中断，退出时清理GPIO设置
    running = False

finally:
    GPIO.cleanup()

# 等待监测线程结束
input_thread.join()
print("LED control program has been exited.")

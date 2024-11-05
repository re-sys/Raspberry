import RPi.GPIO as GPIO
import time
import threading

# 设置 GPIO 模式为 BOARD 模式
GPIO.setmode(GPIO.BOARD)

# 定义端口号
trig = 33
echo = 35
led = 31

# 设置输入和输出端口
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(led, GPIO.OUT)  # 设置 LED 为输出端口

def blink_led():
    while True:
        # 读取共享的 blink_delay 值，调整 LED 的闪烁
        if 'blink_delay' in globals():
            delay = blink_delay
            if delay < 2:
                GPIO.output(led, True)  # 打开 LED
                time.sleep(delay)       # 等待指定时间
                GPIO.output(led, False) # 关闭 LED
                time.sleep(delay)       # 等待指定时间
            else:
                GPIO.output(led, True)  # 打开 LED
                time.sleep(delay)       # 如果延迟过长，保持打开状态

def measure_distance():
    global blink_delay  # 声明为全局变量
    while True:
        # 发送 10 微秒的高电平信号
        GPIO.output(trig, True)
        time.sleep(0.000015)
        GPIO.output(trig, False)

        # 等待回声信号
        while GPIO.input(echo) == 0:
            pulse_start = time.time()

        while GPIO.input(echo) == 1:
            pulse_end = time.time()

        # 计算脉冲持续时间
        pulse_duration = pulse_end - pulse_start

        # 声音在空气中的速度约为34300 cm/s，计算距离
        distance = pulse_duration * 17150  # 17150 = 34300 cm/s / 2
        distance = round(distance, 2)  # 四舍五入到两位小数
        print(f"测得距离: {distance} cm")

        # 根据距离调整 LED 的闪烁频率
        if distance <= 5:
            blink_delay = 0.1  # 小于等于 5 cm 时闪烁延迟为 0.1 秒
        elif distance >= 20:
            blink_delay = 2.0   # 大于等于 20 cm 时闪烁延迟为 2 秒
        else:
            blink_delay = (distance-5) / 15 * 1.9 + 0.1  # 其他情况保持原有计算

        print(f"闪烁延迟: {blink_delay:.2f} s")
        time.sleep(0.1)  # 每0.1秒测量一次

# 创建线程
blink_thread = threading.Thread(target=blink_led)  # 不再需要传递初始延迟
measure_thread = threading.Thread(target=measure_distance)

# 启动线程
blink_thread.start()
measure_thread.start()

try:
    while True:
        time.sleep(0.1)  # 主线程保持活跃
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()  # 清理 GPIO 设置

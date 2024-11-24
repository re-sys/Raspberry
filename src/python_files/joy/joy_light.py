import smbus
import time
import RPi.GPIO as GPIO

# 初始化GPIO
LED_PIN1 = 35  # 第一个LED连接到GPIO 35
LED_PIN2 = 37  # 第二个LED连接到GPIO 37
BUTTON_PIN = 33  # 按钮连接到GPIO 40

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN1, GPIO.OUT)
GPIO.setup(LED_PIN2, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 设置按钮输入，使用上拉电阻

# 初始化I2C
PCF8591_ADDRESS = 0x48  # PCF8591的I2C地址
bus = smbus.SMBus(1)

def read_pcf8591(channel):
    bus.write_byte(PCF8591_ADDRESS, channel)  # 选择通道
    bus.read_byte(PCF8591_ADDRESS)  # 读取数据寄存器
    value = bus.read_byte(PCF8591_ADDRESS)  # 读取模拟值
    return value
def button_pressed():
    # 让灯光闪烁
    print("Button pressed!")
    for _ in range(5):  # 闪烁5次
        pwm1.ChangeDutyCycle(100)  # 打开LED1
        pwm2.ChangeDutyCycle(100)  # 打开LED2
        time.sleep(0.2)  # 持续200毫秒
        pwm1.ChangeDutyCycle(0)  # 关闭LED1
        pwm2.ChangeDutyCycle(0)  # 关闭LED2
        time.sleep(0.2)  # 持续200毫秒
# 控制LED的亮度 (PWM)
pwm1 = GPIO.PWM(LED_PIN1, 1000)  # 1kHz
pwm2 = GPIO.PWM(LED_PIN2, 1000)  # 1kHz
pwm1.start(0)  # 初始化第一个LED占空比为0%
pwm2.start(0)  # 初始化第二个LED占空比为0%

try:
    while True:
        # 读取AIN0和AIN1
        ain0_value = read_pcf8591(0)  # 读取通道0的值
        ain1_value = read_pcf8591(1)  # 读取通道1的值

        # 将模拟值转换为0-1之间的比例
        relative_value1 = ain0_value / 255.0  # 假设值范围为0-255
        relative_value2 = ain1_value / 255.0  # 假设值范围为0-255
        
        # 根据相对值设置占空比
        pwm1.ChangeDutyCycle(relative_value1 * 100)  # 调整第一个LED占空比到0-100%
        pwm2.ChangeDutyCycle(relative_value2 * 100)  # 调整第二个LED占空比到0-100%

        # 检查按钮状态
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # 按键被按下
            time.sleep(0.05)  # 等待50毫秒以消抖
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # 再次检查按钮状态
                button_pressed()  # 调用闪烁函数
                time.sleep(0.5)  # 防止按钮重复触发的延迟按钮重复触发的延迟

        print(f"AIN0: {ain0_value}, AIN1: {ain1_value}, LED1 Brightness: {relative_value1 * 100:.2f}%, LED2 Brightness: {relative_value2 * 100:.2f}%")

        time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()

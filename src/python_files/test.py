import RPi.GPIO as GPIO
import time

# 设置引脚编号模式
GPIO.setmode(GPIO.BOARD)

# 设置引脚为输入，并使用上拉电阻
pin_number = 40  # 替换为你的引脚编号
GPIO.setup(pin_number, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        input_state = GPIO.input(pin_number)
        print("引脚电平状态:", input_state)
        time.sleep(1)  # 每隔1秒读取一次
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()  # 清理GPIO设置

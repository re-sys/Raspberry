import RPi.GPIO as GPIO
import time

# 设置GPIO模式为BCM
GPIO.setmode(GPIO.BCM)

# 定义按键和LED引脚
button_pin = 26
led_pins = [21]

# 设置按键引脚为输入，默认上拉
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 设置LED引脚为输出
for pin in led_pins:
    GPIO.setup(pin, GPIO.OUT)

# 流水灯状态
led_state = 0
running=False
# 按键中断的回调函数
def button_callback(channel):
    global running
    if not GPIO.input(button_pin):
        running=not running
        if not running:
            GPIO.output(led_pins[0], GPIO.LOW)
            
        print("Button pressed")
        # 切换LED状态

# 添加按键中断事件
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=200)

try:
    while True:
        # 闪烁
        
        GPIO.output(led_pins[0], GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(led_pins[0], GPIO.LOW)
        time.sleep(0.1)
        
        if not running:
            GPIO.output(led_pins[0], GPIO.LOW)
            time.sleep(2)
            running=True
        # 延迟一段时间，让LED保持点亮状态
        # time.sleep(0.5)

except KeyboardInterrupt:
    # 清除GPIO设置
    GPIO.cleanup()

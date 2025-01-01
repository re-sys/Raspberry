import RPi.GPIO as GPIO
import time

# 配置GPIO
GPIO.setmode(GPIO.BCM)
IR_PIN = 26        # 红外遥控信号检测引脚
LED_PIN = 19         # LED 灯引脚（您可以选择其他引脚）
running = False
# 设置引脚
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 红外引脚为输入
GPIO.setup(LED_PIN, GPIO.OUT)                          # LED 引脚为输出

def callback(channel):
    global running
    running = True # 检测到低电平，即有信号
    print("检测到按钮信号")
        
        

# 添加边沿检测，检测到信号变化时调用回调函数
GPIO.add_event_detect(IR_PIN, GPIO.BOTH, callback,bouncetime=300)

try:
    # print("等待检测红外遥控信号...")
    while True:
        # ir_status = GPIO.input(IR_PIN)  # 读取 GPIO 状态
        # if ir_status == GPIO.HIGH:
        #     print("GPIO 引脚状态: 高电平")
        # else:
        #     print("GPIO 引脚状态: 低电平")
        if running:
            # print("检测到红外遥控信号")
            print(GPIO.input(IR_PIN))
            GPIO.output(LED_PIN, GPIO.HIGH)  # 点亮 LED
            time.sleep(1)                    # LED 保持亮起 1 秒
            GPIO.output(LED_PIN, GPIO.LOW)   # 熄灭 LED
            time.sleep(0.5)  # 打印状态的间隔时间
            running = False
except KeyboardInterrupt:
    GPIO.cleanup()  # 清理 GPIO 设置
finally:
    GPIO.cleanup()  # 确保在退出时清理 GPIO 设置

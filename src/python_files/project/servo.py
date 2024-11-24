import RPi.GPIO as GPIO
import time

# 设置 GPIO 引脚号
SERVO_PIN = 37

# 设置 GPIO 模式
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# 设置 PWM
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz
pwm.start(0)

def set_servo_angle(angle):
    # 将角度转换为 PWM 脉冲宽度
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)

try:
    while True:
        # 设置舵机到 0 度
        set_servo_angle(0)
        print("舵机转到 0 度")
        time.sleep(1)


        # # 设置舵机到 90 度
        # set_servo_angle(90)
        # time.sleep(1)

        # 设置舵机到 180 度
        set_servo_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    # 如果按下 Ctrl+C，则停止舵机
    print("停止舵机")
    pwm.stop()
    GPIO.cleanup()

finally:
    # 清理
    pwm.stop()
    GPIO.cleanup()

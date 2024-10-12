'''
实现按下按键红灯亮，松开按键绿灯亮
'''
import RPi.GPIO as GPIO
import time
led_green = 37 #控制绿灯
led_red = 35   #控制红灯gpio端口
button = 40    #按键检测gpio端口
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_green, GPIO.OUT)
GPIO.setup(led_red, GPIO.OUT)
GPIO.setup(button, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)#设置gpio端口为下拉输入
button_pressed = True

def red():
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)

def green():
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)

while True:
    if(GPIO.input(button)==button_pressed):
        red()
    else:
        green()

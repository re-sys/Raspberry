# Write your code here :-)
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

"""
sleep_and_wait_for_button_press():在延迟过程中每10ms进行一次按键检测，在按键无按下时，程序休眠输入的延迟时间
输入：延迟时间（必须是0.01的整数倍）
输出：如果检测按键按下返回false 否则返回true

green_toggle():设置gpio端口使灯光传感器闪烁绿光,间隔为interval second
输入：闪烁间隔interval

red_toggle():设置gpio端口使灯光传感器闪烁红光,间隔为interval second
输入：闪烁间隔interval

green():配置gpio口使灯光传感器恒为绿色

red():配置gpio口使灯光传感器恒为红色
"""
def sleep_and_wait_for_button_press(time_to_sleep):
    for i in range(int(time_to_sleep*100)):
        if GPIO.input(button) == button_pressed:
            return False
        time.sleep(0.01)
    return True

def green_toggle(interval):
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)
    sleep_and_wait_for_button_press(interval)
    GPIO.output(led_green, 0)
    sleep_and_wait_for_button_press(interval)
def red():
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)

def red_toggle(interval):
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)
    sleep_and_wait_for_button_press(interval)
    GPIO.output(led_red, 0)
    sleep_and_wait_for_button_press(interval)
def green():
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)
    
state = 0
interval=0.1
while True:
    
    if(GPIO.input(button)==button_pressed):#检测按键按下
        time.sleep(0.05)#消除抖动
        if(GPIO.input(button)==button_pressed):#判断是否真的按下
            print("certain")
            state = (state+1)%4   #按下后改变状态
            print(state)          #打印当前状态
            while(GPIO.input(button)==button_pressed):#知道按键松开
                pass
            time.sleep(.05)#消除按键松开抖动
    #根据当前状态决定灯光状态
    if(state==0):   
        red()
    elif(state==1):
        red_toggle(interval)
    elif(state==2):
        green()
    elif(state==3):
        green_toggle(interval)
        


        
# -*- coding: utf-8 -*-
 
#键盘控制舵机转动
from __future__ import division
import sys, select, termios, tty
 
 
import time
 
# Import the PCA9685 module.
import Adafruit_PCA9685
 
#获取键盘事件
def getKey():
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
 
def set_servo_angle(channel, angle):#输入角度转换成12^精度的数值
    date=int(4096*((angle*11)+500)/20000)#进行四舍五入运算 date=int(4096*((angle*11)+500)/(20000)+0.5)    
    pwm.set_pwm(channel, 0, date)
 
 
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
 
beangle = 0
channel1 = 15
channel2 = 14
 
while 1:
    key = getKey()
 
    if key == 'a':
        while True:            
            beangle += 2.5
            if beangle >= 180:
                beangle = 180
            set_servo_angle(channel1,beangle)
            set_servo_angle(channel2,beangle)
            key = getKey()
            if (key != 'a'):
                break
    if key == 'd':
        while True:
            beangle -= 2.5
            if beangle <= 0:
                beangle = 0
            set_servo_angle(channel1,beangle)
            set_servo_angle(channel2,beangle)
            key = getKey()
            if (key != 'd'):
                break
    if key == 'q':
        break
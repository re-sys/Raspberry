# Write your code here :-)
import RPi.GPIO as GPIO
import time
led_green = 37
led_red = 35
button = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_green, GPIO.OUT)
GPIO.setup(led_red, GPIO.OUT)
GPIO.setup(button, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
def green_toggle():
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)
    time.sleep(0.1)
    GPIO.output(led_green, 0)
    time.sleep(0.1)
def red():
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)

def red_toggle():
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)
    time.sleep(0.1)
    GPIO.output(led_red, 0)
    time.sleep(0.1)
def green():
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)
state = 0

while True:
    # print(GPIO.input(button))
    time.sleep(.1)
    if(GPIO.input(button)==1):
        # print("1")
        time.sleep(0.1)
        if(GPIO.input(button)==1):
            print("certain")
            state = (state+1)%4
            print(state)
            while(GPIO.input(button)==1):
                pass
            time.sleep(.1)

    if(state==0):   
        red()
    elif(state==1):
        red_toggle()
    elif(state==2):
        green()
    elif(state==3):
        green_toggle()
        

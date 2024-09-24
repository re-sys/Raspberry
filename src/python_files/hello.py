# Write your code here :-)
import RPi.GPIO as GPIO
import time
led_green = 35
led_red = 37
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_green, GPIO.OUT)
GPIO.setup(led_red, GPIO.OUT)
def green():
    GPIO.output(led_green, 1)
    GPIO.output(led_red, 0)
def red():
    GPIO.output(led_green, 0)
    GPIO.output(led_red, 1)

def alter_light():
    green()
    time.sleep(0.5)
    red()
    time.sleep(0.5)
while True:
    alter_light()
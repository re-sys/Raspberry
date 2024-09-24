# Write your code here :-)
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(35, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
def green(isopen):
    GPIO.output(35, isopen)
def red(isopen):
    GPIO.output(37, isopen)


time.sleep(0.5)
def loop():
    green(1)
    red(0)
    time.sleep(0.5)
    green(0)
    red(1)
    time.sleep(0.5)
while True:
    loop()

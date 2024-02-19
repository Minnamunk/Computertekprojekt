# coding: utf-8
import time
import RPi.GPIO as GPIO
from gpiozero import LED

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 16
LED = 25

print ("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output
GPIO.setup(LED, GPIO.OUT)       #LED set as output

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()

    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance

try:
    while True:
        distance = measure()
        print ("  Distance : %.1f cm" % distance)
        if(distance<18.0):
                GPIO.output(LED,GPIO.HIGH)
                time.sleep(0.25)
                GPIO.output(LED,GPIO.LOW)
                time.sleep(0.25)
        elif(18.0<distance<25.0):
                GPIO.output(LED,GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(LED,GPIO.LOW)
                time.sleep(1)
        elif(25.0<distance<30.0):
                GPIO.output(LED,GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(LED,GPIO.LOW)
                time.sleep(2)
        else:
                time.sleep(1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
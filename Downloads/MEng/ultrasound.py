import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

trigPin = 19
echoPin = 26

def checklist():
    GPIO.output(trigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(trigPin, GPIO.LOW)

    while not GPIO.input(echoPin):
        pass
    t1 = time.time()
    
    while GPIO.input(echoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

time.sleep(1)

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

try:
    while (sysRunning_flag):
        time.sleep(0.1)
        print('calculating distance....')
        dist = checklist()
        print('Distance: %0.2f cm'%dist)
        

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    
print("exit")
GPIO.cleanup()

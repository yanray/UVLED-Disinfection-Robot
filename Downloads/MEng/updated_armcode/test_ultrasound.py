import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

TRIG = 18
ECHO = 16

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.output(TRIG, False)
print("I AM HERE")

# ultrasound: to get the distance measured by ultrasound
def ultrasound(trigPin, echoPin):
    print("I AM HERE 1")

    GPIO.output(trigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(trigPin, GPIO.LOW)
    print("I AM HERE 2")

    while not GPIO.input(echoPin):
        pass
    t1 = time.time()
    print("T1",t1)
    
    while GPIO.input(echoPin):
        pass
    t2 = time.time()
    print("T2",t2)
    
    return (t2 - t1) * 340 * 100 / 2


    
    #print('calculating distance....')
time.sleep(0.2)
Dist = ultrasound(TRIG, ECHO)
time.sleep(0.2)
print('Distance: %0.2f cm' %leftDist)

GPIO.cleanup()

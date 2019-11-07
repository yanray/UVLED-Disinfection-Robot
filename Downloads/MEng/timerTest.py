from threading import Timer
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

# pin for ultrasound // trigPin, echoPin
leftTrigPin = 20
leftEchoPin = 21
rightTrigPin = 19
rightEchoPin = 26

GPIO.setmode(GPIO.BCM)   #set up GPIO pins

## GPIO setup for two ultrasound's pins
GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)
GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)

# ultrasound: to get the distance measured by ultrasound
def ultrasound(trigPin, echoPin):
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

def hello():
    print('calculating distance....')
    leftDist = ultrasound(leftTrigPin, leftEchoPin)   
    rightDist = ultrasound(rightTrigPin, rightEchoPin)
    print('Left Distance: %0.2f cm' %leftDist)
    print('Right Distance: %0.2f cm' %rightDist)
    
    t = Timer(1, hello)
    t.start()
    

t = Timer(1, hello)
t.start()

try:
    while (sysRunning_flag):
        print("do")
        time.sleep(0.01)
        

except KeyboardInterrupt:
    GPIO.cleanup() # clea
    
    
print("exit")
GPIO.cleanup()



##
##import time
##import RPi.GPIO as GPIO
##sysRunning_flag = True
##
### pin for ultrasound // trigPin, echoPin
##leftTrigPin = 20
##leftEchoPin = 21
##rightTrigPin = 19
##rightEchoPin = 26
##
##GPIO.setmode(GPIO.BCM)
###set up GPIO pins
#### GPIO setup for two ultrasound's pins
##GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
##GPIO.setup(leftEchoPin, GPIO.IN)
##GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
##GPIO.setup(rightEchoPin, GPIO.IN)
##
### ultrasound: to get the distance measured by ultrasound
##def ultrasound(trigPin, echoPin):
##    GPIO.output(trigPin, GPIO.HIGH)
##    time.sleep(0.00015)
##    GPIO.output(trigPin, GPIO.LOW)
##
##    while not GPIO.input(echoPin):
##        pass
##    t1 = time.time()
##    
##    while GPIO.input(echoPin):
##        pass
##    t2 = time.time()
##    
##    return (t2 - t1) * 340 * 100 / 2
##
##def do_every(period,f,*args):
##    global sysRunning_flag
##    global p
##    
##    def g_tick():
##        t = time.time()
##        count = 0
##        while sysRunning_flag:
##            count += 1
##            yield max(t + count*period - time.time(),0)
##    g = g_tick()
##    while sysRunning_flag:
##        p = p + 1;
##        print(p, 'p')
##        if(p >= 20):
##            sysRunning_flag = False
##        time.sleep(next(g))
##        f(*args)
##
##def poll_ultrasound(s):
##    global leftTrigPin
##    global leftEchoPin
##    global rightTrigPin
##    global rightEchoPin    
##    print('calculating distance....')
##    leftDist = ultrasound(leftTrigPin, leftEchoPin)   
##    rightDist = ultrasound(rightTrigPin, rightEchoPin)
##    print('Left Distance: %0.2f cm' %leftDist)
##    print('Right Distance: %0.2f cm' %rightDist)    
##    time.sleep(.3)
##
##p = 1
##print('before')
##do_every(0.02,poll_ultrasound,'foo')
##print('after')
##
##
##
##while(sysRunning_flag):
##    p = p + 1;
##    print(p, 'p')
##    if(p >= 20):
##        sysRunning_flag = False
##
##    
##print("exit")
##GPIO.cleanup()
##
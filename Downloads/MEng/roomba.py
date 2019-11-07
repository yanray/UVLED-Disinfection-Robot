import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

# pin for ultrasound // trigPin, echoPin
leftTrigPin = 20
leftEchoPin = 21

rightTrigPin = 19
rightEchoPin = 26

# hex number
CLEANMODE = '\x87'
MOTOR_PWM = '\x92'
LEFT = '\x00\x8F'
RIGHT = '\x00\x8F'

ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)

ser.write('\x80')
ser.write('\x83')
time.sleep(0.1)

def GPIO5_callback(channel):
    print ("")
    print "Sensor 5 ..."
    global RIGHT
    RIGHT = '\x00\x00'
    
def GPIO6_callback(channel):
    print ("")
    print "Sensor 6 ..."
    global LEFT
    LEFT = '\x00\x00'

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    
    
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for two ultrasound's pins
GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)
GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)

## add callback event
GPIO.add_event_detect(5, GPIO.FALLING, callback=GPIO5_callback, bouncetime = 300)
GPIO.add_event_detect(6, GPIO.FALLING, callback=GPIO6_callback, bouncetime = 300)
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

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

#ser.write(CLEANMODE)
try:
    while (sysRunning_flag):
        time.sleep(0.3)
        ser.write('\x89\xFF\x38\x01\xF4')    ## Drive mode 137/ velocity / radius
        ser.write('\x92\x00\x9F\x00\x9F')     ## PWM drive mode 146 / Right / Left
        ser.write(MOTOR_PWM + RIGHT + LEFT)
        
        print('calculating distance....')
        leftDist = ultrasound(leftTrigPin, leftEchoPin)   
        rightDist = ultrasound(rightTrigPin, rightEchoPin)
        print('Distance: %0.2f cm' %leftDist)
        #print('Distance: %0.2f cm' %rightDist)

##        if(dist <= 90 and dist >= 70):
##            RIGHT = '\x00\x00'
##            LEFT = '\x00\x00'

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    ser.close()
    
print("exit")
GPIO.cleanup()

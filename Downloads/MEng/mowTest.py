import serial
import time
import RPi.GPIO as GPIO
from threading import Timer

sysRunning_flag = True

# pin for ultrasound // trigPin, echoPin
leftTrigPin = 20
leftEchoPin = 21
rightTrigPin = 19
rightEchoPin = 26
turn_CW = True

# hex number
leftSpeed = '\x00\x8f'
rightSpeed = '\x00\x8f'

CLEANMODE = '\x87'
MOTOR_PWM = '\x92'
LEFT_UNDER = False  #'\x00\x8F'
RIGHT_UNDER = False #'\x00\x8F'
under = LEFT_UNDER and RIGHT_UNDER

ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)

ser.write('\x80')
ser.write('\x83')
time.sleep(0.1)

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")

   
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for two ultrasound's pins
GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)
GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)

## add callback event
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

def do_every(period,f,*args):
    def g_tick():
        t = time.time()
        count = 0
        while True:
            count += 1
            yield max(t + count*period - time.time(),0)
    g = g_tick()
    while True:
        time.sleep(next(g))
        f(*args)

def poll_ultrasound():
    global leftTrigPin
    global leftEchoPin
    global rightTrigPin
    global rightEchoPin
    global LEFT_UNDER
    global RIGHT_UNDER
    global under
    
    #print('calculating distance....')
    time.sleep(0.2)
    leftDist = ultrasound(leftTrigPin, leftEchoPin)
    time.sleep(0.2)
    rightDist = ultrasound(rightTrigPin, rightEchoPin)
    print('Left Distance: %0.2f cm' %leftDist)
    print('Right Distance: %0.2f cm' %rightDist)
    
    if(leftDist > 50 and leftDist < 87):
        LEFT_UNDER = True
    if(rightDist > 50 and rightDist < 87):
        RIGHT_UNDER = True
    under = LEFT_UNDER and RIGHT_UNDER


    t = Timer(0.3, poll_ultrasound)
    t.start()


# NEW CHANGES START HERE!!!!!!  besides initalizations
# align detects edge during wandering and aligns roomba 
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
    print(under)
    #while (not under):
        #print("align while loop")
    #if(under):
    #    ser.write('\x92\x00\x00\x00\x00')
    #elif (RIGHT_UNDER):
    #    ser.write('\x92\x00\x00\x00\x8F')
    #elif (LEFT_UNDER):
    #else:  
    #    ser.write('\x92\x00\x8F\x00\x00')             
    #else:
    #    ser.write('\x92\x00\x00\x00\x00') 

    
    if(LEFT_UNDER):
        leftSpeed = '\x00\x00'
        rightSpeed = '\x00\x30'
    if(RIGHT_UNDER):
        rightSpeed = '\x00\x00'
        leftSpeed = '\x00\x30'
    if(under):
        rightSpeed = '\x00\x00'
        leftSpeed = '\x00\x00'

    print('leftSpeed', leftSpeed)
    print('rightSpeed', rightSpeed)
    ser.write(MOTOR_PWM + rightSpeed + leftSpeed)


# mow the lawn algorithm
def mow():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global turn_CW
    global turn_time
    global turn_limit
    # if we are still mowing a table 
    # use turn_limit to tell if we finished a table, it should take more time if when we hit the corner
    while (turn_time < turn_limit):
        # continue straight if under table
        if (under):
            ser.write('\x92\x00\x8F\x00\x8F') 
        # else turn to meet edge and realign
        else:
            time = time.time()
            # turn CW, move L motor until true, then R
            if (turn_CW):
                while (not LEFT_UNDER):
                    ser.write('\x92\x00\x00\x00\x8F') 
                    turn_time = time.time() - time
                    turn = not turn
                while (not under):
                    ser.write('\x92\x00\x8F\x00\x00') 
            # turn CCW, move R motor until true, then L
            else:
                while (not RIGHT_UNDER):
                    ser.write('\x92\x00\x8F\x00\x00') 
                    turn_time = time.time() - time
                    turn = not turn
                while (not under):
                    ser.write('\x92\x00\x00\x00\x8F') 
    time_gap = time.time()
    # turn for 2 seconds, move away for 3
    while (time.time() - time_gap < 2):
        ser.write('\x92\x00\x00\x00\x8F')
    time_gap = time.time()
    while (time.time() - time_gap < 3):
        ser.write('\x92\x00\x8F\x00\x8F')

t = Timer(0.3, poll_ultrasound)
t.start()

try:
    while (sysRunning_flag):
        time.sleep(0.3)
        
        if(LEFT_UNDER or RIGHT_UNDER):
            print("in align")
            align()
        else:
            print("mow")
            ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
                



except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    ser.close()
    
print("exit")
GPIO.cleanup()

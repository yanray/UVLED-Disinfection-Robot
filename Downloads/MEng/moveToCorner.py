import serial
import time
import RPi.GPIO as GPIO

from threading import Timer
# import numpy as np
# import math
# from math import pi

sysRunning_flag = True


#start off random walk
#ultrasound detects
#align
#turn 90 CCW
#move forward until nothing detected
#turn 90 CW
#mow


# pin for ultrasound // trigPin, echoPin
frontTrigPin = 20
frontEchoPin = 21
# value for ultrasound to detect distance
dist = 0

ser.write('\x80') #start
print("started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("clean")

# global RIGHT_UNDER
# RIGHT_UNDER = False
# global LEFT_UNDER
# LEFT_UNDER = False
# global ClockWise
# global C_ClockWise
## variables for robot turning under table
turn_CW = True      # turn clockwise
clean = False
turn_time = 0
turn_limit = 7
mowing = False

# hex number
leftSpeed = '\x00\x8f'      #PWM mode speed control of Roomba wheels 
rightSpeed = '\x00\x8f'

STOP = '\xAD'               #stop command
SAFEMODE = '\x83'           #safe mode, allows for user control
CLEANMODE = '\x87'          #default cleaning mode
MOTOR_PWM = '\x92'          #controls raw forward and backward motion of Roomba's wheels independently +/-255 is range
LEFT_UNDER = False          #IR sensor variable to make robot aligned with table edge. 
RIGHT_UNDER = False         #False if IR sensor doesn't detect anything, True if detects
under = LEFT_UNDER and RIGHT_UNDER      # whether robot is under table, while vertical with table edge


ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)    #port for Raspberry Pi. May need to change port to ttyUSB0
ser.flushOutput()

# GPIO5_callback AND GPIO6_callback are call back functions when IR sensors are fired, used to align robot vertical with table edge. 
def GPIO5_callback(channel):
    global RIGHT_UNDER
    global mowing
    ClockWise = False # if the left IR is under the table, stop truing ClockWise
    if (not mowing):
        RIGHT_UNDER = not GPIO.input(5)

def GPIO6_callback(channel):
    global LEFT_UNDER
    global mowing
    C_ClockWise = False # if the left IR is under the table, stop truing CounterClockWise
    if (not mowing):
        LEFT_UNDER = not GPIO.input(6)

# safe button to quit the system 
def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")

# GPIO initial setup 
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# IR sensors
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for two ultrasound's pins
GPIO.setup(frontTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(frontEchoPin, GPIO.IN)

## add callback event
GPIO.add_event_detect(5, GPIO.FALLING, callback=GPIO5_callback, bouncetime = 150)
GPIO.add_event_detect(6, GPIO.FALLING, callback=GPIO6_callback, bouncetime = 150)
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)


# align detects edge during wandering and aligns roomba 
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
    global sysRunning_flag

    ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
    while (not under):
        print('in align')

        # if left IR sensor fired, set left wheel speed to zero, 
        # if right IR sensor fired, set right wheel speed to zero, 
        # basically it's a function to make robot aligned with robot, 
        # This could be modified to apply in your own system 
        if(LEFT_UNDER):
            print("Left IR under, stop left wheels")
            leftSpeed = '\x00\x00'
        if(RIGHT_UNDER):
            print("Right IR under, stop right wheels")
            rightSpeed = '\x00\x00'
        #else:
            #rightSpeed = '\x00\x8F'
            #leftSpeed = '\x00\x8F'

        #update wheel speed
        ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
        under = LEFT_UNDER and RIGHT_UNDER



# mow the lawn algorithmdisplayLayer. All this does is go forward and turn on LEDs
def mow():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global sysRunning_flag

    # mow lawn with lights on
    while(RIGHT_UNDER or LEFT_UNDER):
        print("In mow. Just moving forward")
        ser.write('\x92\x00\x6F\x00\x6F') #move forward with speed 111 out of 255
        RIGHT_UNDER = not GPIO.input(5) #right IR, check for IR while traversing table to know when to break loop
        LEFT_UNDER = not GPIO.input(6)  #left IR, check for IR while traversing table to know when to break loop
        

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
    
    return (t2 - t1) * 340 * 100 / 2			# calculate distance by time 


# a timer to trig ultrasound
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
    global dist
    
    #print('calculating distance....')displayLayer
    dist = ultrasound(frontTrigPin, frontEchoPin)
    print('Distance: %0.2f cm' %dist)


    # if the Roomba is under the table, 
    # we try to make it turn left until it arrives the left corner
    if(dist > 25 and dist < 50):
        if(LEFT_UNDER or RIGHT_UNDER): #if one or both of the IR sensors are under the table
            print("left " + str(LEFT_UNDER))
            print("right "+ str(RIGHT_UNDER))
            ser.write('\x80') #start
            time.sleep(0.2)
            ser.write('\x83') #safe mode
            time.sleep(0.2)
            ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
            time.sleep(0.1)

            print("in align")
            align()
            ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
            print("wait 0.3 sec")
            time.sleep(0.3)

            #finished align, now rotate left
            while(LEFT_UNDER):
                ser.write('\x92\x00\x00\x00\x01')#Turn in place counter-clockwise = 1 = 0x0001 
                LEFT_UNDER = not GPIO.input(6) 
                print("updated value of LEFT_UNDER: "+str(LEFT_UNDER))


            #move forward until we don't detect a table
            

            #finished moving forward, now rotate right

            #now mow
            print("mowing...")
            mow() #mow just moves forward



            # if LEFT_UNDER:
            #     ClockWise = False

            # if(C_ClockWise):
            #     print('in turn')
            #     ser.write('\x92\x00\x00\x00\x6F') #move left wheels not right wheels

            # else: 
            #     ser.write('\x92\x00\x6F\x00\x00') #right wheel moves and left doesn't

        # After we can move under the table vertically, we need to turn CCk 90 degress 
        # untill the untrasound sensor moves out of the table
    else:
        t = Timer(0.2, poll_ultrasound) #after 0.2 seconds, poll ultrasound again
        t.start()



      

    
print("exit")
#safe mode then stop
time.sleep(0.2)
ser.write('\x83')#safe mode
time.sleep(0.2)
ser.write('\x92\x00\x00\00\00') #wheel speed of 0
time.sleep(0.2)
#stop command when we are done working
ser.write('\xAD') #stop
GPIO.cleanup()
ser.close()


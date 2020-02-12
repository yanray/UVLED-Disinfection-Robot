import serial
import time
import RPi.GPIO as GPIO

from threading import Timer
import numpy as np
import math
from math import pi

sysRunning_flag = True

ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)


ser.write('\x80') #start
print("started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("clean")

global RIGHT_UNDER
global LEFT_UNDER
global ClockWise
global C_ClockWise

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




def poll_ultrasound():
    global dist
    global modeFlag
    
    #print('calculating distance....')displayLayer
    dist = ultrasound(frontTrigPin, frontEchoPin)
    print('Distance: %0.2f cm' %dist)


    # if the Roomba is under the table, 
    # we try to make it turn left untill it arrives the left corner
    
    if(dist > 25 and dist < 50):
         if(LEFT_UNDER or RIGHT_UNDER): #if one or both of the IR sensors are under the table
                    ser.write('\x80') #start
                    ser.write('\x83') #safe mode
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    time.sleep(0.1)
                    print("left", LEFT_UNDER)
                    print("right", RIGHT_UNDER)

                    print("in align")
                    align()
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    print("wait 0.3 sec")
                    time.sleep(0.3)
                    print("mowing...")
                    mow() #mow just moves forward

                    if LEFT_UNDER:
                        ClockWise = False

                    if(C_ClockWise):
                        print('in turn')
                        ser.write('\x92\x00\x00\x00\x6F') #move left wheels not right wheels

                    else: 
                        ser.write('\x92\x00\x6F\x00\x00') #right wheel moves and left doesn't

                    # After we can move under the table vertically, we need to turn CCk 90 degress 
                    # untill the untrasound sensor moves out of the table




#move left 90 degrees
#move forward
#move right 90 degrees        

    
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


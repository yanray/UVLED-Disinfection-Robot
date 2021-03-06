import time 
import serial
import time
import RPi.GPIO as GPIO
from threading import Timer

sysRunning_flag = True
turn_CW = True
clean = False
turn_time = 0
turn_limit = 7
mowing = False

# hex number
leftSpeed = '\x00\x8f'
rightSpeed = '\x00\x8f'

CLEANMODE = '\x87'
MOTOR_PWM = '\x92'
LEFT_UNDER = False  
RIGHT_UNDER = False 
under = LEFT_UNDER and RIGHT_UNDER

ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)
ser.flushOutput()
ser.write('\x80')
ser.write('\x83')
ser.write('\x92\x00\x00\x00\x00')
time.sleep(0.7)
print("stop!!! before start")
  
def GPIO5_callback(channel):
    global RIGHT_UNDER
    global mowing
    if (not mowing):
        RIGHT_UNDER = not GPIO.input(5)

def GPIO6_callback(channel):
    global LEFT_UNDER
    global mowing
    if (not mowing):
        LEFT_UNDER = not GPIO.input(6)

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
   
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# IR sensors
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# LEDS
#GPIO.setup(26, GPIO.OUT)
#GPIO.setup(19,GPIO.OUT)
#GPIO.output(26,1)
#GPIO.output(19,1)

## add callback event
GPIO.add_event_detect(5, GPIO.FALLING, callback=GPIO5_callback)
GPIO.add_event_detect(6, GPIO.FALLING, callback=GPIO6_callback)
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

# NEW CHANGES START HERE!!!!!!  besides initalizations
# align detects edge during wandering and aligns roomba 
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
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

    while (not under):
        if(LEFT_UNDER):
            leftSpeed = '\x00\x00'
        if(RIGHT_UNDER):
            rightSpeed = '\x00\x00'
        #else:
            #rightSpeed = '\x00\x8F'
            #leftSpeed = '\x00\x8F'

        ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
        under = LEFT_UNDER and RIGHT_UNDER


# mow the lawn algorithm
def mow():
    global mow
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global turn_CW
    global turn_time
    global turn_limit
    # if we are still mowing a table 
    # use turn_limit to tell if we finished a table, it should take more time if when we hit the corner
    start_time = 0
    mowing = True
    mode_align=True
    while (True):#(turn_time < turn_limit):
        time.sleep(0.3)
        RIGHT_UNDER = not GPIO.input(5)
        LEFT_UNDER = not GPIO.input(6)
        under = RIGHT_UNDER and LEFT_UNDER
        #print("left")
        #print(LEFT_UNDER)
        #print("right")
        #print(RIGHT_UNDER)   
        #print(under)
        #continue straight if under table        
        if (mode_align):
            print("straight")
            while(under):
                RIGHT_UNDER = not GPIO.input(5)
                LEFT_UNDER = not GPIO.input(6)
                under = RIGHT_UNDER and LEFT_UNDER
                ser.write('\x92\x00\x8F\x00\x8F')
            mode_align = False
            #turn ON LEDs
#           GPIO.output(26,0)
#           GPIO.output(19,0)
        # else turn to meet edge and realign
        else:
            start_time = time.time()
            # turn OFF LEDs
#            GPIO.output(26,1)
#            GPIO.output(19,1)
            print("turning..")
            # turn CW, move L motor until true, then R
            if (turn_CW):
                while (not LEFT_UNDER):
                    LEFT_UNDER = not GPIO.input(6)
                    print(LEFT_UNDER)
                    ser.write('\x92\x00\x00\x00\x8F')  
                turn_time = time.time() - start_time
                ser.write('\x92\x00\x00\x00\x00')
                time.sleep(1)
                turn_CW = not turn_CW
                print(turn_CW)
                while (not RIGHT_UNDER):
                    RIGHT_UNDER = not GPIO.input(5)
                    LEFT_UNDER = not GPIO.input(6)
                    under = RIGHT_UNDER and LEFT_UNDER
                    ser.write('\x92\x00\x5F\x00\x00')
                print("re-aligned")
                ser.write('\x92\x00\x00\x00\x00')
                time.sleep(0.3)
                ser.write('\x92\x00\x8F\x00\x8F')
                time.sleep(1)
            # turn CCW, move R motor until true, then L
            else:
                while (not RIGHT_UNDER):
                    RIGHT_UNDER = not GPIO.input(5)
                    ser.write('\x92\x00\x8F\x00\x00') 
                turn_time = time.time() - start_time
                ser.write('\x92\x00\x00\x00\x00')
                time.sleep(1)
                turn_CW = not turn_CW
                print(turn_CW)
                while (not LEFT_UNDER):
                    RIGHT_UNDER = not GPIO.input(5)
                    LEFT_UNDER = not GPIO.input(6)
                    under = RIGHT_UNDER and LEFT_UNDER                   
                    ser.write('\x92\x00\x00\x00\x8F')
                print("re-aligned")
                ser.write('\x92\x00\x00\x00\x00')
                time.sleep(0.3)
                ser.write('\x92\x00\x8F\x00\x8F')
                time.sleep(1)
            mode_align = True

    time_gap = time.time()
    # turn for 2 seconds, move away for 3
    while (time.time() - time_gap < 2):
        ser.write('\x92\x00\x00\x00\x8F')
    time_gap = time.time()
    while (time.time() - time_gap < 3):
        ser.write('\x92\x00\x8F\x00\x8F')
   
try:
    while (sysRunning_flag):
        time.sleep(0.7)
        #LEFT_UNDER = not GPIO.input(6)
        #RIGHT_UNDER = not GPIO.input(5)
        #print("left")
        #print(LEFT_UNDER)
    
        if(not clean):
            ser.write('\x92\x00\x8F\x00\x8F')   
            clean = True
            pass

        if(LEFT_UNDER or RIGHT_UNDER):
            clean = False
            ser.write('\x80')
            ser.write('\x83')
            ser.write('\x92\x00\x00\x00\x00')
            time.sleep(0.3)
            print(LEFT_UNDER)
            print(RIGHT_UNDER)

            print("in align")
            align()
            ser.write('\x92\x00\x00\x00\x00')
            print("wait 1 sec")
            time.sleep(1)
            print("mowing...")
            mow()
            pass

        print("wandering")
        

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    ser.close()
    
print("exit")
GPIO.cleanup()


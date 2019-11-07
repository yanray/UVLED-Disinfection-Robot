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
LEFT_UNDER = False  #'\x00\x8F'
RIGHT_UNDER = False #'\x00\x8F'

# table sensing 
under = False   # Roomba is under table
mowLawn = False # we should mow the lawn
align = False
turn_CW = True       # toggle between CW and CCW turns
turn_time = 0
turn_limit= 10

ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)

ser.write('\x80')
ser.write('\x87')
time.sleep(0.1)

def GPIO5_callback(channel):
    print ("")
    print "Sensor 5 ..."
    global RIGHT_UNDER
    RIGHT_UNDER = True
    
def GPIO6_callback(channel):
    print ("")
    print "Sensor 6 ..."
    global LEFT_UNDER
    LEFT_UNDER = True

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

# NEW CHANGES START HERE!!!!!!  besides initalizations
# align detects edge during wandering and aligns roomba 
def align():
    global align
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    while (not under):
        if (RIGHT_UNDER):
            ser.write('\x92\x00\x00\x00\x8F') 
        else:
            ser.write('\x92\x00\x8F\x00\x00') 
        under = RIGHT_UNDER and LEFT_UNDER
    align = True

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
    while (time.time() - time_gap < 2)
        ser.write('\x92\x00\x00\x00\x8F')
    time_gap = time.time()
    while (time.time() - time_gap < 3)
        ser.write('\x92\x00\x83\x00\x8F')



try:
    while (sysRunning_flag):
        time.sleep(0.3)
        under = RIGHT_UNDER and LEFT_UNDER
        if(not mowLawn):
            ser.write('\x87')   # change to clean mode
        else:
            ser.write('\x83')   # change to safe mode
            if(not align):
                align()
            else:
                mow()
                
                
##        print('calculating distance....')
##        leftDist = ultrasound(leftTrigPin, leftEchoPin)   
##        rightDist = ultrasound(rightTrigPin, rightEchoPin)
##        print('Distance: %0.2f cm' %leftDist)
##        print('Distance: %0.2f cm' %rightDist)


except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    ser.close()
    
print("exit")
GPIO.cleanup()

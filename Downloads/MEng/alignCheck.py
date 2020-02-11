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

ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
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


    while (not under):
		if(LEFT_UNDER):
			leftSpeed = '\x00\x00'
		if(RIGHT_UNDER):
			rightSpeed = '\x00\x00'

		ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
		under = LEFT_UNDER and RIGHT_UNDER

try:
    while (sysRunning_flag):
        time.sleep(0.7)
    
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
            print("done align")
            ser.write('\x92\x00\x00\x00\x00')
            print("wait 1 sec")
            time.sleep(1)
            print("mowing...")
            break

        

except KeyboardInterrupt:
	GPIO.cleanup() # clean up GPIO on CTRL+C exit
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
    
print("exit")
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



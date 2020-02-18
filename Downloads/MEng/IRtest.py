import time 
import serial
import time
import RPi.GPIO as GPIO
from threading import Timer

RIGHT_UNDER = False
LEFT_UNDER = False

#current setup works as expected
#IR returns True when object detected
#IR returns False when nothing detected

#IR works only if very close to surface i.e. within 5cm
def GPIO5_callback(channel):
    global RIGHT_UNDER
    RIGHT_UNDER =  not GPIO.input(5)

def GPIO6_callback(channel):
    global LEFT_UNDER
    LEFT_UNDER = not GPIO.input(6)

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    print("System shut down")
   
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# IR sensors
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## add callback event
GPIO.add_event_detect(5, GPIO.FALLING, callback=GPIO5_callback)
GPIO.add_event_detect(6, GPIO.FALLING, callback=GPIO6_callback)
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

#test boolean values for IR sensors
try:
    counter = 0
    while(True):
        RIGHT_UNDER = not GPIO.input(5)
        if RIGHT_UNDER and counter ==0:
            print("right under: " + str(RIGHT_UNDER))
            RIGHT_UNDER = not GPIO.input(5)
            print("counter 0")
            counter +=1
        elif RIGHT_UNDER and counter ==1:
            print("right under: " + str(RIGHT_UNDER))
            RIGHT_UNDER = not GPIO.input(5)
            print("counter 0")
            counter +=1
     

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    #ser.close()
    
print("exit")
GPIO.cleanup()
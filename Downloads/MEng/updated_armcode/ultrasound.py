import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True
global under_the_table


under_the_table = True
trigPin = 20
echoPin = 21

def checklist():
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


def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    ser.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset
    time.sleep(2)#Action for 1000ms*1 , therefore delay for 1s
    
ser=serial.Serial(port='/dev/ttyS0',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)

GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

time.sleep(1)


## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)
ser.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset

try:
    while (sysRunning_flag):
        time.sleep(0.1)
        time.sleep(1)
        print('calculating distance....')
        dist = checklist()
        print('Distance: %0.2f cm'%dist)
       
        if(under_the_table):
           print("in1")
           ser.write('\x55\x55\x05\x06\x09\x01\x00')
           time.sleep(2)
           if(dist > 5 and dist < 10):
                print("in2")
                ser.write('\x55\x55\x05\x06\x05\x01\x00')
                #ser.write('\x55\x55\x05\x06\x05\x01\x00')#Action group 1 is for disinfection of the table 
                time.sleep(2) #Action is set for 18*1000ms therefore delay 19s
           elif(dist > 10 and dist < 20):
                ser.write('\x55\x55\x05\x06\x06\x01\x00')
                #ser.write('\x55\x55\x05\x06\x09\x01\x00')
                #ser.write('\x55\x55\x05\x06\x04\x01\x00')#Action group 4 is for disinfection of the table 
                time.sleep(2) #Action is set for 18*1000ms therefore delay 19s
        else:
         ser.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset
         time.sleep(2)#Action for 1000ms*1 , therefore delay for 1s
            
            

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    
print("exit")
GPIO.cleanup()

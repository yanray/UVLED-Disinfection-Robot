import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)


ser.write('\x80') #start
print("started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("clean")


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


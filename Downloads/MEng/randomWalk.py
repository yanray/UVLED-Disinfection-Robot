import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True

#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)

time.sleep(0.1)
ser.write('\x80') #start
print("started")

time.sleep(0.1)
ser.write('\x87') #clean mode
print("clean")
time.sleep(8)

print('safe mode') #safe mode
ser.write('\x83')

ser.write('\xAD') #stop
print("stop")

time.sleep(0.1)
ser.write('\x80') #start
print("started")


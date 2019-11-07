import serial
import time
import RPi.GPIO as GPIO

sysRunning_flag = True


ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)

time.sleep(0.1)
ser.write('\x80')
ser.write('\x83')

#ser.write(CLEANMODE)
try:
    while (sysRunning_flag):
        time.sleep(0.5)
        #ser.write('\x92\x00\x00\x00\x00')
        x = ser.write('\x8E\x01\x07')    ## Drive mode 137/ velocity / radius
        print(x)

except KeyboardInterrupt:
    ser.close()
    
print("exit")


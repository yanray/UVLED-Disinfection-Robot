import serial
import time 

ser=serial.Serial(port='/dev/ttyS0',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)
#time.sleep(1)
i = 1
counter = 0
while(1):
    #print("Connection established")
    ser.write(counter)
    time.sleep(10)
    counter += 100
ser.close()

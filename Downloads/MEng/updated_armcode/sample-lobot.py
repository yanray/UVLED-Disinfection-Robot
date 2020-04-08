import serial
import time 

ser=serial.Serial(port='/dev/ttyS0',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)
i=1
#time.sleep(1)
while(i<2):
    i = i+1
    print("Connection established") 
    print("command sent")
    ser.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset
    time.sleep(2)#Action for 1000ms*1 , therefore delay for 1s
    ser.write('\x55\x55\x05\x06\x01\x01\x00')#Action group 1 is for disinfection of the table 
    time.sleep(19) #Action is set for 18*1000ms therefore delay 19s
    ser.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset 
    print("packet rec")
    time.sleep(2)#Action for 1000ms*1 , therefore delay for 1s
ser.write('\x55\x55\x02\x07') #to stop the running action 
ser.close()

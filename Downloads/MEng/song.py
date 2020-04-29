import serial
#setting up serial connection to Raspberry Pi for the Roomba
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()
START = '\x80'
ser.write(START)
SAFEMODE = '\x83' 
ser.write(SAFEMODE)

#SING A SONG
ser.write("\x8C\x00\x01\x45\xFF")
<<<<<<< HEAD
ser.write("\x8D\x00")`
ser.close()
=======

ser.close()
>>>>>>> a92a1ae7cec24c8cf013b1e49d86f55dcd5df333

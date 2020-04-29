#setting up serial connection to Raspberry Pi for the Roomba
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()
START = '\x80'
ser.write(START)
SAFEMODE = '\x83' 
ser.write(SAFEMODE)

#SING A SONG
ser.write("\x8C\x00\x01\x45\xFF")
ser.write("\x8D\x00")`
ser.close()
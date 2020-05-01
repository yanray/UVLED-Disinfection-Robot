"""
This is the UV Robot MEng project. The goal of this project is to create a robot that can autonomously
disinfect hard to reach surfaces such as the underside of hospital beds by mounting a UV light and ultrasound
sensors to a Roomba. Applications of this include reducing the amount of lingering bacteria found in 
hospital rooms.

At a high level, we have 3 ultrasound sensors and a UV light attached to a robot arm mounted on a Roomba.
The 3 ultrasound sensors are used for navigation, the detection of surfaces, and for gauging how far
the UV light on the robot arm needs to move to adjust to the surface height. The robot arm is used
to move the UV light close to the surface to increase the intensity of the UV rays. 

The Roomba first wanders around the room; this is done by taking advantage of its built-in "Clean Mode".
While the Roomba wanders, it is constantly using the front ultrasound sensor to see if there is any object
within a certain threshold distance. Once an object such as a table is detected, the Roomba moves itself
to the bottom left corner of the table using its ultrasound sensors.

The robot proceeds to navigate the underside of the table using a "lawn mowing algorithm". This just means
that the robot moves forward until the end of the table width, turns 180, moves forward until the end of the table,
and repeatedly moves up and down the table until the entire table has been traversed. While traversing
the table, the UV light is turned on while the robot arm holds it close to the table surface.

When the surface is successfully disinfected, the Roomba leaves the table and resumes wandering until another surface
is detected.
"""


#import packages
import serial
import time
import RPi.GPIO as GPIO
from threading import Timer

#setting up serial connection to Raspberry Pi for the Roomba
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

"""
----------------------
#setting up serial connection to Raspberry Pi for the robot arm
serArm = serial.Serial(port='',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)
---------------------
"""

#flags for Roomba running
sysRunning_flag = True #general flag for if the overall roomba software is still running
modeFlag = 0 #mode 0: wander and poll ultrasound. Mode 2: traverse table and clean
wander = False #True = Roomba is not wandering but should be

# pin for ultrasound // trigPin, echoPin
frontTrigPin = 20
frontEchoPin = 21

leftTrigPin = 5
leftEchoPin = 6

rightTrigPin = 19
rightEchoPin = 26


threshold = 50 #how close surface has to be to the ultrasound sensor for Roomba to consider itself "under" the surface

#used for Roomba find table corner 
global ClockWise #do I need to turn CW?
global C_ClockWise #do I need to turn CCW?
C_ClockWise = True #T means need to turn left after align for corner find
ClockWise = False #T means I have turned left for corner find and need to turn right for lawn mow

## variables for robot turning under table
global moveToCorner #flag for moving to table corner. Need to move to corner whenever a new surface is encountered
moveToCorner = True
turn_CW = True      # turn clockwise
#totalTurns = 0

"""
----------------------
#detects if robot arm is under the table
#may not need this. If statement with ultrasound could suffice
global arm_under_table
arm_under_table = False
---------------------
"""


#we don't use these two variables. From older previous code. Can potentially get rid of this
clean = False
mowing = False

# handy constants for various modes of Roomba operation and some wheel speeds. All in hex 
leftSpeed = '\x00\x8f'      #PWM mode speed control of Roomba wheels 
rightSpeed = '\x00\x8f'

START = '\x80'
STOP = '\xAD'               #stop command
SAFEMODE = '\x83'           #safe mode, allows for user control
CLEANMODE = '\x87'          #default cleaning mode
#command to make the robot wheels stop aka speed of 0. 
#Command is of form \x move wheel mode \x MSB right wheel \x LSB right wheel \x MSB left wheel \x LSB left wheel
STOPMOVING = '\x92\x00\x00\00\00' 
MOTOR_PWM = '\x92'          #controls raw forward and backward motion of Roomba's wheels independently +/-255 is range
LEFT_UNDER = False          #IR sensor variable to make robot aligned with table edge. 
RIGHT_UNDER = False         #False if IR sensor doesn't detect anything, True if detects
under = LEFT_UNDER and RIGHT_UNDER      # whether robot is under table, while vertical with table edge

#initial start up code that starts Roomba and sets it to random wander/clean mode
ser.write(START) #start
print("Started")

time.sleep(0.2)
ser.write(CLEANMODE) #clean mode
print("Wandering")
#time.sleep(0.2)
#ser.write(SAFEMODE)
#time.sleep(0.2)
#print("forward")
#ser.write('\x92\x00\x60\x00\x5F')
#time.sleep(10)
"""
---------------------
#initiate arm in the reset position. Can turn this into a method tbh
serArm.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset
---------------------
"""


# safe button to quit the system 
def GPIO27_callback(channel):
    print ("")
    print ("Button 27 pressed...")
    global sysRunning_flag
    sysRunning_flag = False
    #safe mode then stop
    ser.write(SAFEMODE)
    time.sleep(0.2)
    ser.write(STOPMOVING) #wheel speed of 0
    time.sleep(0.2)
    #stop command when we are done working
    ser.write(STOP)
    ser.close()
    """
    ------------------
    #returns robot arm to initial position i.e. reset
    serArm.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset\
    serArm.close()
    ------------------
    """
    #do we need this?
    #GPIO.cleanup()
    print("System shut down")

# GPIO initial setup 
GPIO.setmode(GPIO.BCM)   #set up GPIO pins

# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for ultrasound's pins
GPIO.setup(frontTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(frontEchoPin, GPIO.IN)

GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)

GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)
"""
------------------
#arm ultrasound
GPIO.setup(armTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(armEchoPin, GPIO.IN)
------------------
"""

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

"""
-------------
#we may not need this method. This is just an idea
#returns arm to intial position. Resets the arm.
def resetArm(serArm):
    serArm.write('\x55\x55\x05\x06\x00\x01\x00')
-------------
"""


"""
-------------
#Once arm is under table, adjust arm to correct height
def adjustArm(serArm,armTrigPin,armEchoPin,threshold):
    dist = ultrasound(armTrigPin, armEchoPin)
    if(dist < threshold): #could just use dist < threshold instead of arm_under_table boolean
        serArm.write('\x55\x55\x05\x06\x09\x01\x00')
        time.sleep(2)
        if(dist > 5 and dist < 10):
            print("in2")
            serArm.write('\x55\x55\x05\x06\x05\x01\x00')
            #serArm.write('\x55\x55\x05\x06\x05\x01\x00')#Action group 1 is for disinfection of the table 
            time.sleep(2) #Action is set for 18*1000ms therefore delay 19s
        elif(dist > 10 and dist < 20):
            serArm.write('\x55\x55\x05\x06\x06\x01\x00')
            #serArm.write('\x55\x55\x05\x06\x09\x01\x00')
            #serArm.write('\x55\x55\x05\x06\x04\x01\x00')#Action group 4 is for disinfection of the table 
            time.sleep(2) #Action is set for 18*1000ms therefore delay 19s
    else:
        serArm.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset
        time.sleep(2)#Action for 1000ms*1 , therefore delay for 1s
-------------
"""

#changes Roomba into clean mode aka wandering mode
def toCleanMode():
    global wander
    print("Going to clean mode")
    time.sleep(0.1)
    ser.write(START) #start

    time.sleep(0.2)
    ser.write(CLEANMODE) #clean mode
    time.sleep(0.1)
    wander = False

#checks if an ultrasound sensor is under a surface, specified by threshold height. 
def checkIfUnder(trigPin,echoPin, threshold):
    #use simple majority voting to determine final boolean value
    count = 0
    for i in range(3):
        dist = ultrasound(trigPin,echoPin)
        time.sleep(0.01)
        if dist < threshold:
            count = count + 1
    
    if(count >=2):
        return True
    else:
        return False

# align detects edge during wandering and aligns roomba to surface edge
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
    global sysRunning_flag
    print("In align")
    #sets wheels moving forward
    leftSpeed = '\x00\x8f'
    rightSpeed = '\x00\x8f'
    ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
    #check left and right sensors to see if they are under a surface
    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
    time.sleep(0.01)
    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
    time.sleep(0.01)
    under = LEFT_UNDER and RIGHT_UNDER
    while (not under):

        # if left sensor fired, set left wheel speed to zero, 
        # if right sensor fired, set right wheel speed to zero, 
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
        time.sleep(0.01)
        if(LEFT_UNDER):
            #print("Left sensor under, stop left wheels")
            leftSpeed = '\x00\x00'
        if(RIGHT_UNDER):
            #print("Right sensor under, stop right wheels")
            rightSpeed = '\x00\x00'

        #update wheel speed
        ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
        under = LEFT_UNDER and RIGHT_UNDER



# moves forward until left and right sensors are no longer under the surface
# takes in param firstTimeMow to help recognize when to stop the mowing algorithm
def mow(firstTimeMow):
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global sysRunning_flag
    print("In mow")

    #check left and right sensor values i.e. if sensors are under table or not
    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
    time.sleep(0.01)
    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
    time.sleep(0.01)
    ser.write('\x92\x00\x60\x00\x5F') #move forward with speed 63 out of 255

    #boolean to determine when to stop
    stop = False
    #if it is not the first run of mow() and only 1 sensor is under the surface,
    #then we need to stop mowing after this run
    if(firstTimeMow==False):
        if((LEFT_UNDER and RIGHT_UNDER==False) or (LEFT_UNDER==False and RIGHT_UNDER)):
                print("Stop mowing after this!")
                stop = True

    """
    ---------------------
    #Adjust the arm while it is under the table
    adjustArm(serArm,armTrigPin,armEchoPin,threshold)
    ---------------------
    """

    # if one of the left or right sensor is under the surface, then keep moving forward.
    while(LEFT_UNDER or RIGHT_UNDER):
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
        time.sleep(0.01)


    """
    ---------------------
    #No longer under table. Reset table arm
    resetArm(serArm)
    ---------------------
    """

    #returns whether to keep mowing or to stop
    return stop

# ultrasound: to get the distance measured by an ultrasound sensor
def ultrasound(trigPin, echoPin):
    GPIO.output(trigPin, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(trigPin, GPIO.LOW)

    while not GPIO.input(echoPin):
        pass
    t1 = time.time()
    
    while GPIO.input(echoPin):
        pass
    t2 = time.time()
    
    return (t2 - t1) * 340 * 100 / 2			# calculate distance by time 



"""
----------
TEMPORARY CODE FOR BREAKING CODE INTO SECTIONS
-----------
"""
def pause(ser):
    #safe mode then stop
    print("exit")
    time.sleep(0.2)
    ser.write('\x83')#safe mode
    time.sleep(0.2)
    ser.write('\x92\x00\x00\00\00') #wheel speed of 0
    time.sleep(0.2)
    #stop command when we are done working
    ser.write('\xAD') #stop
    GPIO.cleanup()
    ser.close()



#driver code. This is where all the logic and navigation happens
try:    
    while(sysRunning_flag):
        # ultrasound polling and random walk mode
        if (modeFlag == 0):
            #if we need to return to wandering/cleaning mode
            if(wander == True):
                toCleanMode()
                continue
            #use front sensor to detect surface
            frontUnder = checkIfUnder(frontTrigPin,frontEchoPin,threshold)
            time.sleep(0.01)
            if(frontUnder):
                print("Object detected by front ultrasound")
                modeFlag = 2
                ser.write(SAFEMODE)
                time.sleep(0.2)
                #code to move forward. 
                ser.write('\x92\x00\x6F\x00\x6F')
        else:
            LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
            time.sleep(0.01)
            RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
            time.sleep(0.01)
            if(LEFT_UNDER or RIGHT_UNDER): #if one or both of the left/right sensors are under the table
                #first time detecting surface. 
                #Move to the corner of the table
                if(moveToCorner):
                    print("Moving to table corner")
                    ser.write(START) #start
                    time.sleep(0.2)
                    ser.write(SAFEMODE) #safe mode
                    time.sleep(0.2)
                    ser.write(STOPMOVING) #stop wheels moving
                    time.sleep(0.1)

                    #Robot has stopped moving and will now align with surface
                    align()
                    ser.write(STOPMOVING) #stop wheels moving
                    time.sleep(0.1)

                    #this is for before the roomba starts moving to the corner, but after align
                    #print("this is for before the roomba starts moving to the corner, but after align")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    did not work at the first time
                    hold the table a little bit away from the Rommba
                    """
                    #pause(ser)
                    #break
                    
                    #We found that sometimes the Roomba overshoots so this moves backward a little bit
                    #to account for that. Don't worry this is based off of sensor readings and not timing
                    temp = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    print("Moving backwards")
                    ser.write('\x92\xFF\xA1\xFF\xA1')
                    while(temp==True):
                        temp = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.01)
                    ser.write(STOPMOVING)
                    time.sleep(0.1)
                    

                    #Finished align, now rotate left. 
                    #Front of robot is facing bottom left table corner once this finishes
                    ser.write('\x92\x00\x5F\xFF\xA1')
                    #Countdown on when to stop turning towards bottom left table corner direction. 
                    #Stop when ultrasound doesn't detect table
                    print("In counter clockwise turn.")
                    while(C_ClockWise):
                        distcheck = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                        #stop turning once front sensor is out from under surface
                        if(distcheck == False):
                            print("Done with counter clockwise")
                            C_ClockWise = False
                    C_ClockWise = True #reset this flag for if we find another surface to traverse later

                    #Roomba can overshoot and turn too much. Adjust the overshoot
                    ##########################################
                    #print("adjusting for counter clockwise overshoot")
                    #ser.write('\x92\xFF\xA1\x00\x5F')
                    #temp2 = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                    #while(temp2==False):
                    #    temp2 = checkIfUnder(frontTrigPin, frontEchoPin,threshold)

                    #Finished rotating left. Facing bottom left table corner
                    #print("Finished rotating left. Facing bottom left table corner")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break
                    
                    
                    #This moves the robot to the table corner
                    #Facing bottom left table corner. Move forward to the bottom left table corner.  
                    ser.write(STOPMOVING)
                    time.sleep(0.1)
                    print("Moving forward")
                    #code to move forward
                    ser.write('\x92\x00\x5F\x00\x5F')
                    #countdown to move forward. Keep moving forward until no table is detected
                    while(RIGHT_UNDER):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.01)
                    ser.write(STOPMOVING)
                    time.sleep(0.1)

                    #We found that sometimes the Roomba overshoots so this moves backward a little bit
                    #to account for that. Don't worry this is based off of sensor readings and not timing
                    #print("Moving backwards")
                    ser.write('\x92\xFF\xA1\xFF\xA1')
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.01)
                    ser.write(STOPMOVING)
                    time.sleep(0.1)

                    #Moved to table corner and did the backward adjust.
                    #print("Moved to table corner and did the backward adjust.")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break


                    #Finished moving to the table edge, but we are facing away from the table. 
                    #Rotate right to face forward and get in the correct position for mowing
                    ser.write('\x92\xFF\xA1\x00\x5F')
                    ClockWise = True
                    print("Moving clockwise")
                    #countdown on when to stop turning clockwise
                    while(ClockWise):
                        distcheck = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                        time.sleep(0.01)
                        #print("dist is: "+str(dist))
                        #if dist < 25
                        if(distcheck):
                            ClockWise = False
                    #may need to change ClockWise back to True
                    ClockWise = True
                    print("Finished moving to the corner")
                    #set move to corner to false after initial run is done
                    moveToCorner = False

                    #move to corner is now complete
                    #print("move to corner is now complete")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break
                    #ser.write(STOPMOVING)
                    #time.sleep(0.2)
                    #ser.write('\x92\x00\x5F\x00\x5F')
                    #time.sleep(0.05)




                    #now mow
                    print("Entering lawn mowing algorithm for the first time")
                    #True means it is our first time mowing
                    a= ultrasound(19,26)
                    print(a)
                    mow(True) #mow just moves forward until no sensors are under the table
                    a= ultrasound(19,26)
                    print(a)
                    #Done with the first lawn mow
                    print("Done with the first lawn mow")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break



                #ELSE we do NOT need to move to the corner 
                else:
                    print("Not the first time in lawn mowing algorithm")
                    #stop condition comes from calling the mow method. Details specified in mow() method
                    stopCondition = mow(False) #False means not the first time mowing
                    print("")

                    #Done with not first lawn mowing
                    print("Done with not first lawn mowing")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break


                    if(stopCondition==True):
                        print("In end condition because of ultrasound sensor stop condition. Return to wandering and polling")
                        ser.write(STOPMOVING)
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        """
                        -------------------------
                        #reset arm after stop condition
                        resetArm(serArm)
                        -------------------------
                        """
                        continue



                #now to handle turning
                print("Am I turning clockwise? " + str(turn_CW))
                if(turn_CW):
                    """
                    if totalTurns > 5:
                        print("Resuming wandering")
                        ser.write(STOPMOVING)
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        totalTurns = 0
                        continue
                    """
                    print('In clockwise turn')
                    ser.write('\x92\x00\x00\x00\x7F') #move left wheels not right wheels

                    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
                    time.sleep(0.01)
                    #keep rotating left wheels while left sensor is not yet under table
                    startTime = time.time() #timer to check if a turn is taking too long
                    while(LEFT_UNDER==False):
                        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
                        time.sleep(0.01)
                    endTime = time.time()
                    #totalTurns = totalTurns +1
                    #Done with CW turn
                    print("Done with clockwise turn")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break

                    #check if the elapsed time is too long.
                    #this implies we need to do a 180 turn and then wander
                    #the time that is considered "too long" can be modified
                    print("Time spent in turn: "+str(endTime-startTime))

                    if(endTime - startTime > 18.9):
                        #stop, turn 180, wander
                        print("In end condition because of turn duration. Return to wandering and polling")
                        ser.write(STOPMOVING)
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        #turn 180 degrees ish, then stop
                        ser.write('\x92\x00\x00\xFF\x71') #move left wheels backwards
                        s = time.time()
                        e = time.time()
                        while(e - s < 2.5):
                            e = time.time()
                        ser.write(STOPMOVING)  
                        """
                        -------------------------
                        #reset arm after stop condition
                        resetArm(serArm)
                        -------------------------
                        """                      
                        continue

                        #ignore comments below. Used when Patrick and Beau were testing
                        # #safe mode then stop
                        # print("exit")
                        # time.sleep(0.2)
                        # ser.write('\x83')#safe mode
                        # time.sleep(0.2)
                        # ser.write('\x92\x00\x00\00\00') #wheel speed of 0
                        # time.sleep(0.2)
                        # #stop command when we are done working
                        # ser.write('\xAD') #stop
                        # GPIO.cleanup()
                        # ser.close()
                        # break 

                    turn_CW = False #we know to turn CCW now
                    #align after turn so that turns are ~crisp~
                    align()
                    ser.write(STOPMOVING)
                    time.sleep(5)
                 

                # turn CCW     
                else:
                    """
                    if totalTurns > 5:
                        print("Resuming wandering")
                        ser.write(STOPMOVING)
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        totalTurns = 0
                        continue
                    """
                    print('In counter clockwise turn')
                    ser.write('\x92\x00\x7F\x00\x00') #right wheel moves and left doesn't

                    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    time.sleep(0.01)
                    #keep rotating left wheels while left sensor is not yet under table
                    startTime = time.time()
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.01)
                    endTime = time.time()
                    #totalTurns = totalTurns + 1

                    #Done with CW turn
                    print("Done with counter clockwise turn")
                    """
                    ------------------
                    ADDING A PAUSE AND BREAK
                    ------------------
                    """
                    #pause(ser)
                    #break

                    print("Time spent in turn: "+str(endTime-startTime))
                    
                    if(endTime - startTime > 8.9):
                        #stop, turn 180, wander
                        print("In end condition b/c of timing. Return to wandering and polling")
                        ser.write(STOPMOVING)
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        #turn 180 degrees ish, then stop
                        ser.write('\x92\xFF\x71\x00\x00') #move right wheels backwards
                        s = time.time()
                        e = time.time()
                        while(e - s < 2.5):
                            e = time.time()
                        ser.write(STOPMOVING)
                        """
                        -------------------------
                        #reset arm after stop condition
                        resetArm(serArm)
                        -------------------------
                        """
                        continue

                        # #safe mode then stop
                        # print("exit")
                        # time.sleep(0.2)
                        # ser.write('\x83')#safe mode
                        # time.sleep(0.2)
                        # ser.write('\x92\x00\x00\00\00') #wheel speed of 0
                        # time.sleep(0.2)
                        # #stop command when we are done working
                        # ser.write('\xAD') #stop
                        # GPIO.cleanup()
                        # ser.close()
                        # break 

                    turn_CW = True
                    #align after turn so that turns are ~crisp~
                    align()     
            else:
                ser.write('\x92\x00\x4F\x00\x4F')

      
except KeyboardInterrupt:
    print("Keyboard Interrupt: exiting")
    #safe mode then stop
    ser.write(SAFEMODE)
    time.sleep(0.2)
    ser.write(STOPMOVING) #wheel speed of 0
    time.sleep(0.2)
    #stop command when we are done working
    ser.write(STOP)
    ser.close()
    """
    ------------------
    #returns robot arm to initial position i.e. reset
    serArm.write('\x55\x55\x05\x06\x00\x01\x00') #Action Group 0 running. It brings into initial position i.e. reset\
    serArm.close()
    ------------------
    """
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    



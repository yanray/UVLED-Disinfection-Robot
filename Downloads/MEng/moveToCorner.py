#import packages
import serial
import time
import RPi.GPIO as GPIO
from threading import Timer

#setting up serial connection to Raspberry Pi
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()

#flags for Roomba running
sysRunning_flag = True #general flag for if the overall roomba software is still running
modeFlag = 0 #mode 0: wander and poll ultrasound. Mode 1: set up/ adjust arm. Mode 2: traverse table
wander = False #True = Roomba is not wandering but should be

# pin for ultrasound // trigPin, echoPin
frontTrigPin = 20
frontEchoPin = 21

leftTrigPin = 5
leftEchoPin = 6

rightTrigPin = 19
rightEchoPin = 26

threshold = 25 #how close surface has to be to the ultrasound sensor for Roomba to consider itself "under" the surface

#used for Roomba find table corner 
global ClockWise #do I need to turn CW?
global C_ClockWise #do I need to turn CCW?
C_ClockWise = True #T means need to turn left after align for corner find
ClockWise = False #T means I have turned left for corner find and need to turn right for lawn mow

## variables for robot turning under table
global moveToCorner #flag for moving to table corner. Need to move to corner whenever a new surface is encountered
moveToCorner = True
turn_CW = True      # turn clockwise

#we don't use these two variables. From older previous code. Can potentially get rid of this
clean = False
mowing = False

# handy constants for various modes of Roomba operation and some wheel speeds. All in hex 
leftSpeed = '\x00\x8f'      #PWM mode speed control of Roomba wheels 
rightSpeed = '\x00\x8f'

STOP = '\xAD'               #stop command
SAFEMODE = '\x83'           #safe mode, allows for user control
CLEANMODE = '\x87'          #default cleaning mode
MOTOR_PWM = '\x92'          #controls raw forward and backward motion of Roomba's wheels independently +/-255 is range
LEFT_UNDER = False          #IR sensor variable to make robot aligned with table edge. 
RIGHT_UNDER = False         #False if IR sensor doesn't detect anything, True if detects
under = LEFT_UNDER and RIGHT_UNDER      # whether robot is under table, while vertical with table edge

#initial start up code that starts Roomba and sets it to random wander/clean mode
ser.write('\x80') #start
print("Started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("Wandering")
time.sleep(0.1)


# safe button to quit the system 
def GPIO27_callback(channel):
    print ("")
    print ("Button 27 pressed...")
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")

# GPIO initial setup 
GPIO.setmode(GPIO.BCM)   #set up GPIO pins

# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## GPIO setup for two ultrasound's pins
GPIO.setup(frontTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(frontEchoPin, GPIO.IN)

GPIO.setup(leftTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(leftEchoPin, GPIO.IN)

GPIO.setup(rightTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(rightEchoPin, GPIO.IN)

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

#changes Roomba into clean mode aka wandering mode
def toCleanMode():
    global wander
    print("Going to clean mode")
    time.sleep(0.2)
    ser.write('\x80') #start

    time.sleep(0.2)
    ser.write(CLEANMODE) #clean mode
    time.sleep(0.2)
    wander = False

#checks if an ultrasound sensor is under a surface, specified by threshold height. 
def checkIfUnder(trigPin,echoPin, threshold):
    #use simple majority voting to determine final boolean value
    count = 0
    for i in range(3):
        dist = ultrasound(trigPin,echoPin)
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
    under = LEFT_UNDER and RIGHT_UNDER
    while (not under):

        # if left sensor fired, set left wheel speed to zero, 
        # if right sensor fired, set right wheel speed to zero, 
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
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
    ser.write('\x92\x00\x6F\x00\x6F') #move forward with speed 111 out of 255

    #boolean to determine when to stop
    stop = False
    #if it is not the first run of mow() and only 1 sensor is under the surface,
    #then we need to stop mowing after this run
    if(firstTimeMow==False):
        if((LEFT_UNDER and RIGHT_UNDER==False) or (LEFT_UNDER==False and RIGHT_UNDER)):
                print("Stop mowing after this!")
                stop = True
    # if one of the left or right sensor is under the surface, then keep moving forward.
    while(LEFT_UNDER or RIGHT_UNDER):
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
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





#driver code
try:    
    while(sysRunning_flag):
        # ultrasound polling and random walk mode
        if (modeFlag == 0):
            #if we need to return to wandering/cleaning mode
            if(wander == True):
                toCleanMode()
            #use front sensor to detect surface
            frontUnder = checkIfUnder(frontTrigPin,frontEchoPin,threshold)
            if(frontUnder):
                print("Object detected by front ultrasound")
                modeFlag = 1
                ser.write('\x83')
                time.sleep(0.2)
                #code to move forward. May need to change this to stop and then move forward in mode 1
                ser.write('\x92\x00\x6F\x00\x6F')
        #moving robot arm. 
        #TODO: POPULATE WITH CODE
        elif modeFlag == 1:
            modeFlag = 2
        #traversing underneath the surface
        else:
            LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
            time.sleep(0.01)
            RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
            if(LEFT_UNDER or RIGHT_UNDER): #if one or both of the left/right sensors are under the table
                #first time through, need to move to corner
                if(moveToCorner):
                    print("Moving to table corner")
                    ser.write('\x80') #start
                    time.sleep(0.2)
                    ser.write('\x83') #safe mode
                    time.sleep(0.2)
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    time.sleep(0.1)

                    #Robot has stopped moving and will now align with surface
                    align()
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    #print("wait 0.2 sec")
                    time.sleep(0.2)


                    #finished align, now rotate left. Front of robot is facing table corner 
                    ser.write('\x92\x00\x6F\xFF\x91')
                    #countdown on when to stop. Stop when ultrasound doesn't detect table
                    print("In counter clockwise turn.")
                    while(C_ClockWise):
                        distcheck = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                        #stop turning once front sensor is out from under surface
                        if(distcheck == False):
                            print("Done with counter clockwise")
                            C_ClockWise = False
                    C_ClockWise = True #reset this flag for if we find another surface to traverse later
                    
                    

                    #move forward until we don't detect a table. This moves the robot to the table corner
                    ser.write('\x92\x00\x00\x00\x00')
                    time.sleep(0.2)
                    print("Moving forward")
                    #code to move forward
                    ser.write('\x92\x00\x6F\x00\x6F')
                    #countdown to move forward. Keep moving forward until no table is detected
                    while(RIGHT_UNDER):
                        #print(RIGHT_UNDER)
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.1)
                    ser.write('\x92\x00\x00\x00\x00')
                    time.sleep(0.2)

                    #we found that sometimes the roomba overshoots so this moves backward a little bit
                    #to account for that. Don't worry this is based off of sensor readings and not timing
                    print("Moving backwards")
                    ser.write('\x92\xFF\x91\xFF\x91')
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    ser.write('\x92\x00\x00\x00\x00')

                    #finished moving to the table edge, now rotate right to get in the correct position for mowing
                    ser.write('\x92\xFF\x91\x00\x6F')
                    time.sleep(0.2)
                    ClockWise = True
                    print("Moving clockwise")
                    #countdown on when to stop turning clockwise
                    while(ClockWise):
                        distcheck = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                        #print("dist is: "+str(dist))
                        #if dist < 25
                        if(distcheck):
                            ClockWise = False
                    #may need to change ClockWise back to True
                    ClockWise = True
                    print("Finished moving ot the corner")
                    #set move to corner to false after initial run is done
                    moveToCorner = False
                    #now mow
                    print("Entering lawn mowing algorithm for the first time")
                    #True means it is our first time mowing
                    mow(True) #mow just moves forward until no sensors are under the table



                #ELSE we do NOT need to move to the corner 
                print("Not the first time in lawn mowing algorithm")
                #stop condition comes from calling the mow method. Details specified in mow() method
                stopCondition = mow(False) #False means not the first time mowing
                if(stopCondition==True):
                    print("In end condition because of ultrasound sensor stop condition. Return to wandering and polling")
                    ser.write('\x92\x00\x00\x00\x00')
                    moveToCorner = True #done mowing, next time we mow need to find corner
                    #transition back to wandering and polling
                    modeFlag = 0
                    turn_CW = True #restored to original value
                    wander = True #go back to wandering
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


                #now to handle turning
                print("Am I turning clockwise? " + str(turn_CW))
                if(turn_CW):
                    print('In clockwise turn')
                    ser.write('\x92\x00\x00\x00\x8F') #move left wheels not right wheels

                    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
                    #keep rotating left wheels while left sensor is not yet under table
                    startTime = time.time() #timer to check if a turn is taking too long
                    while(LEFT_UNDER==False):
                        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
                    endTime = time.time()
                    #check if the elapsed time is too long.
                    #this implies we need to do a 180 turn and then wander
                    #the time that is considered "too long" can be modified
                    print("Time spent in turn: "+str(endTime-startTime))

                    if(endTime - startTime > 3.9):
                        #stop, turn 180, wander
                        print("In end condition because of turn duration. Return to wandering and polling")
                        ser.write('\x92\x00\x00\x00\x00')
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        #turn 180 degrees ish, then stop
                        ser.write('\x92\x00\x00\xFF\x71') #move left wheels backwards
                        s = time.time()
                        e = time.time()
                        while(e - s < 2):
                            e = time.time()
                        ser.write('\x92\x00\x00\x00\x00')                        
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

                    turn_CW = False #we know to turn CCW now
                    #align after turn so that turns are ~crisp~
                    align()
                 

                # turn CCW     
                else:
                    print('In counter clockwise turn')
                    ser.write('\x92\x00\x8F\x00\x00') #right wheel moves and left doesn't

                    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    #keep rotating left wheels while left sensor is not yet under table
                    startTime = time.time()
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    endTime = time.time()

                    print("Time spent in turn: "+str(endTime-startTime))
                    
                    if(endTime - startTime > 3.9):
                        #stop, turn 180, wander
                        print("In end condition b/c of timing. Return to wandering and polling")
                        ser.write('\x92\x00\x00\x00\x00')
                        moveToCorner = True #done mowing, next time we mow need to find corner
                        #transition back to wandering and polling
                        modeFlag = 0
                        turn_CW = True #restored to original value
                        wander = True #go back to wandering
                        #turn 180 degrees ish, then stop
                        ser.write('\x92\xFF\x71\x00\x00') #move right wheels backwards
                        s = time.time()
                        e = time.time()
                        while(e - s < 2):
                            e = time.time()
                        ser.write('\x92\x00\x00\x00\x00')
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


      
except KeyboardInterrupt:
    print("Keyboard Interrupt: exiting")
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    #safe mode then stop
    ser.write(SAFEMODE)
    time.sleep(0.2)
    ser.write('\x92\x00\x00\00\00') #wheel speed of 0
    time.sleep(0.2)
    #stop command when we are done working
    ser.write(STOP)
    ser.close()
    



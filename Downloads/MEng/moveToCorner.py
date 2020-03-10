import serial
import time
import RPi.GPIO as GPIO

from threading import Timer

sysRunning_flag = True
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
ser.flushOutput()
modeFlag = 0
#True = Roomba is not wandering but should be
wander = False

# pin for ultrasound // trigPin, echoPin
frontTrigPin = 20
frontEchoPin = 21

leftTrigPin = 5
leftEchoPin = 6

rightTrigPin = 19
rightEchoPin = 26

threshold = 25
# value for ultrasound to detect distance
dist = 0

ser.write('\x80') #start
print("Started")

time.sleep(0.2)
ser.write('\x87') #clean mode
print("Wandering")
time.sleep(0.1)

global ClockWise
global C_ClockWise
C_ClockWise = True #T means need to turn left after align for corner find
ClockWise = False #T means I have turned left for corner find and need to turn right for lawn mow


## variables for robot turning under table
global moveToCorner
moveToCorner = True
turn_CW = True      # turn clockwise
clean = False
mowing = False

# hex number
leftSpeed = '\x00\x8f'      #PWM mode speed control of Roomba wheels 
rightSpeed = '\x00\x8f'

STOP = '\xAD'               #stop command
SAFEMODE = '\x83'           #safe mode, allows for user control
CLEANMODE = '\x87'          #default cleaning mode
MOTOR_PWM = '\x92'          #controls raw forward and backward motion of Roomba's wheels independently +/-255 is range
LEFT_UNDER = False          #IR sensor variable to make robot aligned with table edge. 
RIGHT_UNDER = False         #False if IR sensor doesn't detect anything, True if detects
under = LEFT_UNDER and RIGHT_UNDER      # whether robot is under table, while vertical with table edge



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

#changes Roomba into clean mode aka wandering
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
    #use simple majority voting
    count = 0
    for i in range(3):
        dist = ultrasound(trigPin,echoPin)
        if dist < threshold:
            count = count + 1
    
    if(count >=2):
        return True
    else:
        return False

# align detects edge during wandering and aligns roomba 
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
    global sysRunning_flag
    print("In align")
    leftSpeed = '\x00\x8f'
    rightSpeed = '\x00\x8f'
    ser.write(MOTOR_PWM + rightSpeed + leftSpeed)

    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
    time.sleep(0.01)
    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
    under = LEFT_UNDER and RIGHT_UNDER
    while (not under):

        # if left IR sensor fired, set left wheel speed to zero, 
        # if right IR sensor fired, set right wheel speed to zero, 
        # basically it's a function to make robot aligned with robot, 
        # This could be modified to apply in your own system 
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
        if(LEFT_UNDER):
            #print("Left IR under, stop left wheels")
            leftSpeed = '\x00\x00'
        if(RIGHT_UNDER):
            #print("Right IR under, stop right wheels")
            rightSpeed = '\x00\x00'

        #update wheel speed
        ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
        under = LEFT_UNDER and RIGHT_UNDER



# mow the lawn algorithmdisplayLayer. All this does is go forward and turn on LEDs
def mow(firstTimeMow):
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global sysRunning_flag
    print("In mow")
    # mow lawn with lights on
    LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
    time.sleep(0.01)
    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
    ser.write('\x92\x00\x6F\x00\x6F') #move forward with speed 111 out of 255

    #boolean to determine when to stop
    stop = False

    if(firstTimeMow==False):
        if((LEFT_UNDER and RIGHT_UNDER==False) or (LEFT_UNDER==False and RIGHT_UNDER)):
                print("Stop mowing after this!")
                stop = True

    while(LEFT_UNDER or RIGHT_UNDER):
        LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
        time.sleep(0.01)
        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
    return stop

# ultrasound: to get the distance measured by ultrasound
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


# # a timer to trig ultrasound
# def do_every(period,f,*args):
#     def g_tick():
#         t = time.time()
#         count = 0
#         while True:
#             count += 1
#             yield max(t + count*period - time.time(),0)
#     g = g_tick()
#     while True:
#         time.sleep(next(g))
#         f(*args)


# def poll_ultrasound():
#     global dist
#     global modeFlag
    
#     #print('calculating distance....')displayLayer
#     dist = ultrasound(frontTrigPin, frontEchoPin)
#     #print('Distance: %0.2f cm' %dist)


#     # if the Roomba is under the table, 
#     # we try to make it turn left until it arrives the left corner
#     if(dist < 25):
#         print('object detected')
#         ser.write('\x83')
#         time.sleep(0.2)
#         ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
#         time.sleep(0.1)
#         modeFlag = 2

#         # After we can move under the table vertically, we need to turn CCk 90 degress 
#         # untill the untrasound sensor moves out of the table
#     else:
#         t = Timer(0.2, poll_ultrasound) #after 0.2 seconds, poll ultrasound again
#         t.start()


#driver code
try:    
    while(sysRunning_flag):
        if modeFlag == 0:
            #print("In modeflag 0")
            frontUnder = checkIfUnder(frontTrigPin,frontEchoPin,threshold)
            if(frontUnder):
                print("Object detected by front ultrasound")
                modeFlag = 2
                ser.write('\x83')
                time.sleep(0.2)
                #code to move forward
                ser.write('\x92\x00\x6F\x00\x6F')
        else:
            #may need an if statement for handling mowing vs finding table corner since while loop repeats
            #which means that after each loop the table finding would repeat

            #can have a flag for find corner
            #True initially
            #after corner found, set to false
            #mow. Once you are done mowing, return to random walk and set flag to true


            LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
            time.sleep(0.01)
            RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
            if(LEFT_UNDER or RIGHT_UNDER): #if one or both of the IR sensors are under the table
                #print("Move to corner value: " + str(moveToCorner))
                #first time through, need to move to corner
                if(moveToCorner):
                    print("Moving to table corner")
                    #print("left " + str(LEFT_UNDER))
                    #print("right "+ str(RIGHT_UNDER))
                    ser.write('\x80') #start
                    time.sleep(0.2)
                    ser.write('\x83') #safe mode
                    time.sleep(0.2)
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    time.sleep(0.1)

                    align()
                    ser.write('\x92\x00\x00\x00\x00') #stop wheels moving
                    #print("wait 0.2 sec")
                    time.sleep(0.2)


                    #finished align, now rotate left
                    ser.write('\x92\x00\x6F\xFF\x91')#Turn in place counter-clockwise = 1 = 0x0001 
                    #countdown on when to stop. Stop when ultrasound doesn't detect table
                    print("In counter clockwise turn.")
                    while(C_ClockWise):

                        distcheck = checkIfUnder(frontTrigPin, frontEchoPin,threshold)
                        #print("dist is: "+str(dist))

                        #if dist > 25
                        if(distcheck == False):
                            print("Done with counter clockwise")
                            C_ClockWise = False
                    #may need to turn C_ClockWise back to True later
                    C_ClockWise = True
                    
                    

                    #move forward until we don't detect a table
                    ser.write('\x92\x00\x00\x00\x00')
                    time.sleep(0.2)
                    print("Moving forward")
                    #code to move forward
                    ser.write('\x92\x00\x6F\x00\x6F')
                    

                    #countdown to move forward
                    while(RIGHT_UNDER):
                        #print(RIGHT_UNDER)
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                        time.sleep(0.1)
                    ser.write('\x92\x00\x00\x00\x00')
                    time.sleep(0.2)

                    #moveBackwards
                    print("Moving backwards")
                    ser.write('\x92\xFF\x91\xFF\x91')
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    ser.write('\x92\x00\x00\x00\x00')


                    #finished moving forward, now rotate right
                    ser.write('\x92\xFF\x91\x00\x6F')
                    time.sleep(0.2)
                    ClockWise = True
                    #ser.write('\x92\x0F\x0F\x0F\x0F') #turn in place clockwise
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

                    

                    
                    
                print("Not the first time in lawn mowing algorithm")
                stopCondition = mow(False)#False means not the first time mowing
                if(stopCondition==True):
                    print("In end condition because of ultrasound sensor stop condition. Return to wandering and polling")
                    ser.write('\x92\x00\x00\x00\x00')
                    moveToCorner = True #done mowing, next time we mow need to find corner

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
                    break 


                #now to handle turning
                print("Am I turning clockwise? " + str(turn_CW))
                if(turn_CW):
                    print('In clockwise turn')
                    #print("moving left wheels but not right")
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
                    if(endTime - startTime > 4):
                        #stop, turn 180, wander
                        print("In end condition because of turn duration. Return to wandering and polling")
                        ser.write('\x92\x00\x00\x00\x00')
                        moveToCorner = True #done mowing, next time we mow need to find corner

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
                        break 


                    turn_CW = False
                    
                    align()
                 

                # turn CCW     
                else:
                    print('In counter clockwise turn')
                    #print("moving right wheels but not left wheels")
                    ser.write('\x92\x00\x8F\x00\x00') #right wheel moves and left doesn't

                    RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    #keep rotating left wheels while left sensor is not yet under table
                    startTime = time.time()
                    while(RIGHT_UNDER==False):
                        RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)
                    endTime = time.time()

                    print("Time spent in turn: "+str(endTime-startTime))
                    
                    if(endTime - startTime > 4):
                        #stop, turn 180, wander
                        print("In end condition b/c of timing. Return to wandering and polling")
                        ser.write('\x92\x00\x00\x00\x00')
                        moveToCorner = True #done mowing, next time we mow need to find corner

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
                        break 

                    turn_CW = True

                    align()

                    # #move forward a little bit
                    # ser.write('\x92\x00\x6F\x00\x6F')
                    # pass
                    # time.sleep(0.5)
                    # #then check ultrasound
                    # print("Checking if we still need to mow")
                    # LEFT_UNDER = checkIfUnder(leftTrigPin,leftEchoPin,threshold)
                    # RIGHT_UNDER = checkIfUnder(rightTrigPin,rightEchoPin,threshold)


                    
                    # if (LEFT_UNDER == False and RIGHT_UNDER == False):
                    #     #sysRunning_flag = False
                    #     #break
                    #     print("In end condition. Return to wandering and polling")
                    #     ser.write('\x92\x00\x00\x00\x00')
                    #     moveToCorner = True #done mowing, next time we mow need to find corner

                    #     # #rather than breaking out of the loop...
                    #     # modeFlag = 0 #mode flag back to polling
                    #     # turn_CW = True #restored to original value
                    #     # wander = True #go back to wandering
                    #     # continue
                    #     print("exit")
                    #     time.sleep(0.2)
                    #     ser.write('\x83')#safe mode
                    #     time.sleep(0.2)
                    #     ser.write('\x92\x00\x00\00\00') #wheel speed of 0
                    #     time.sleep(0.2)
                    #     #stop command when we are done working
                    #     ser.write('\xAD') #stop
                    #     GPIO.cleanup()
                    #     ser.close()
                    #     break 

                    





                



                # if LEFT_UNDER:
                #     ClockWise = False

                # if(C_ClockWise):
                #     print('in turn')
                #     ser.write('\x92\x00\x00\x00\x6F') #move left wheels not right wheels

                # else: 
                #     ser.write('\x92\x00\x6F\x00\x00') #right wheel moves and left doesn't
        


      
except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    #safe mode then stop
    ser.write(SAFEMODE)
    time.sleep(0.2)
    #stop command when we are done working
    ser.write(STOP)
    ser.close()
    



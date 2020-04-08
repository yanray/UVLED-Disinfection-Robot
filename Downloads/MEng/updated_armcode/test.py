import os, sys, pygame, time
from pygame.locals import *
import numpy as np
import RPi.GPIO as GPIO
import math
from math import pi
import serial

import FK, VK, IK

port = "/dev/ttyAMA0" #port initialisation

pos_offset = [0, 0, 5]
servo_ID1 = 0.0
servo_degree = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #intialise servo motors

## system flag
sysRunning_flag = True    # system Running flag
type_flag = True



#callback for quiting the program
def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    exit()
    GPIO.cleanup

    
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

dist = 0.37
i = 0
try:
    while (sysRunning_flag):
        time.sleep(0.02)
        
        s1 = serial.Serial('/dev/ttyACM0', 9600)#setup serial communication-uart
        s1.flushInput() #clean/flush input buffer

        y = -0.9 * dist + 0.4143

        #y = -0.9 * dist + 0.4143
        goal_position = [0, y, dist]     ## x, y, z should be converted to meters, like 0.025
        
        print(y)
        i = i + 1
        
        print("Goal position read: ", goal_position)
        print("reached1")
        ## FK service
        current_angles = [0, 0, 0, 0, 0]
        test_angle = [pi / 2, pi / 2, -pi / 2, 0,0]
        FK_result =  (test_angle)
        print(FK_result)
        print("above is FK")
        
        print("reached2")
        ## VK service
        jac = VK.vk_srv(current_angles)
        print(jac)
        
        print("above is VK")
        
        
        ## IK service
        #goal_position = [FK_result[0, 3], FK_result[1, 3], FK_result[2, 3]]
        #goal_position = [0.175, 0, 0.08]
        rotating_angle = list(IK.ik_srv(goal_position))
        
        rotating_angle.append(servo_ID1)
        #rotating_angle[4] = -(rotating_angle[1] + rotating_angle[2] + rotating_angle[3]) + (pi / 2)
        rotating_angle[4] = 0
        print("rotating angle:", rotating_angle)
    
        print("reached3")
        for i in range(0, 3):
            servo_degree[i] = int(-(((rotating_angle[i] * 360) / (2 * pi)) / 0.24) - 0.5)
            print("reached6")
        servo_degree[3] = int((((rotating_angle[3] * 360) / (2 * pi)) / 0.24) + 0.5)
        servo_degree[4] = int((((rotating_angle[4] * 360) / (2 * pi)) / 0.24) + 0.5)
        servo_degree[5] = int((((rotating_angle[5] * 360) / (2 * pi)) / 0.24) + 0.5)
        
        print("reached4")
        print("servo degree:", servo_degree)
        ## serial communication 
        

        print("reached5")
        count = 0
        
        comp_list = ["Completed\r\n", "Hello Pi, This is Arduino UNO...:\r\n", "All completed\r\n"]
        done_signal = ["done\r\n"]
        while count < 6:
            print("reached6")
            if s1.inWaiting()>0:
                inputValue = s1.readline()
                print(inputValue)
                if inputValue in comp_list:
                    try:
                        n = servo_degree[count]
                        print("reached200")
                        print("Pi's pos and number:",count,n)
                        s1.write('%d'%n)
                        count = count+1
                    except:
                        print("reached100")
                        print("Input error, please input a number")
                        s1.write('0')
        
        inputValue = s1.readline()
        while inputValue not in done_signal:
            inputValue = s1.readline()
        #sysRunning_flag = False


except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit

print("exit")
GPIO.cleanup()



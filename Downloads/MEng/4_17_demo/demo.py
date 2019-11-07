import serial
import RPi.GPIO as GPIO
from threading import Timer
import os, sys, pygame, time
from pygame.locals import *
import numpy as np
import math
from math import pi

import FK, VK, IK

endConditionCounter = 0

modeFlag = -1    # 0 when polling ultrasounds, 1 when moving arm (no polling),
                # 2 for initiating mowing the lawn code

mode2Flag = 1

port = "/dev/ttyAMA0"

pos_offset = [0, 0, 5]
servo_ID1 = 0.0
servo_degree = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

sysRunning_flag = True

# pin for ultrasound // trigPin, echoPin
frontTrigPin = 20
frontEchoPin = 21
time.sleep(0.1)

dist = 0

turn_CW = True
clean = False
turn_time = 0
turn_limit = 7
LED_ON = False
mowing = False

# hex number
leftSpeed = '\x00\x8f'
rightSpeed = '\x00\x8f'

CLEANMODE = '\x87'
MOTOR_PWM = '\x92'
LEFT_UNDER = False  
RIGHT_UNDER = False 
under = LEFT_UNDER and RIGHT_UNDER

ser=serial.Serial(port='/dev/ttyS0',baudrate=115200)
ser.flushOutput()
ser.write('\x80')
ser.write('\x83')
ser.write('\x92\x00\x00\x00\x00')
time.sleep(0.7)
print("stop!!! before start")



## display on piTFT
##os.putenv('SDL_VIDEODRIVER', 'fbcon')
##os.putenv('SDL_FBDEV', '/dev/fb1')
##os.putenv('SDL_MOUSEDRV', 'TSLIB')
##os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

displayLayer = 1

screen_back_color = (0, 0, 0)        # screen color Black
screen_size = width, height = (320, 240)   # screen size
black = (0, 0, 0)  
white = (255, 255, 255)     
red = (200, 0, 0)        
green = (0, 200, 0)

## coordinates
x = 0
y = 0

## button name
start_button = 'Start'
quit_button = 'Quit'
back_button = 'Back'

## button
button_size = 40
ctr_button_pos = (160, 120)
side_button_pos = (220, 200)
top_text_pos = (2,20)
bottom_text_pos = (10,55)

bg_path = "/home/pi/MEng/clean.jpeg"


def GPIO5_callback(channel):
    global RIGHT_UNDER
    global mowing
    if (not mowing):
        RIGHT_UNDER = not GPIO.input(5)

def GPIO6_callback(channel):
    global LEFT_UNDER
    global mowing
    if (not mowing):
        LEFT_UNDER = not GPIO.input(6)

def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    
    
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# IR sensors
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# LEDs
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.output(19,0)
GPIO.output(26,0)

## GPIO setup for two ultrasound's pins
GPIO.setup(frontTrigPin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(frontEchoPin, GPIO.IN)

## add callback event
GPIO.add_event_detect(5, GPIO.FALLING, callback=GPIO5_callback, bouncetime = 150)
GPIO.add_event_detect(6, GPIO.FALLING, callback=GPIO6_callback, bouncetime = 150)
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)

class Screen:
    def __init__(self, (R, G, B), (x_size, y_size)):
        ## screen initialization 
        self.back_color = (R, G, B)
        self.screen_size = (x_size, y_size)
        pygame.init()
        pygame.mouse.set_visible(True)   # conceal mouse
        self.my_font_button = pygame.font.Font(None, button_size)      # set the buttons text size
        self.my_font_text = pygame.font.Font(None, 20) # set the size of coordinates display
        
    def set_size(self):
        ## set screen size
        self.screen = pygame.display.set_mode(self.screen_size)
        return self.screen
    
    def clear_screen(self):
        ## fill background with black color
        self.screen.fill(screen_back_color)
        
    def display_button(self, my_text, color, text_pos):
        ## display button on the screen
        text_surface = self.my_font_button.render(my_text, True, color)
        rect = text_surface.get_rect(center = text_pos)
        self.screen.blit(text_surface, rect)  
        
    def display_button_withCircle(self, my_text, text_color, circle_color, text_pos):
        ## display button on the screen
        text_surface = self.my_font_button.render(my_text, True, text_color)
        rect = text_surface.get_rect(center = text_pos)
        pygame.draw.circle(self.screen, circle_color, text_pos , 60)
        self.screen.blit(text_surface, rect)  

    def display_text(self, my_text, color, text_pos):
        ## display text on the screen
        text_surface = self.my_font_text.render(my_text, True, color)
        rect = text_surface.get_rect(left = text_pos[0], bottom = text_pos[1])
        self.screen.blit(text_surface, rect)  



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
    
    return (t2 - t1) * 340 * 100 / 2


def do_every(period,f,*args):
    def g_tick():
        t = time.time()
        count = 0
        while True:
            count += 1
            yield max(t + count*period - time.time(),0)
    g = g_tick()
    while True:
        time.sleep(next(g))
        f(*args)

def poll_ultrasound():
    global dist
    global modeFlag
    
    #print('calculating distance....')displayLayer
    dist = ultrasound(frontTrigPin, frontEchoPin)
    print('Distance: %0.2f cm' %dist)

    if(dist > 25 and dist < 50):
        modeFlag = 1
        ser.write('\x92\x00\x00\x00\x00')
    elif(modeFlag == 0):
        t = Timer(0.2, poll_ultrasound)
        t.start()
    else:
        pass

# NEW CHANGES START HERE!!!!!!  besides initalizations
# align detects edge during wandering and aligns roomba 
def align():
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global MOTOR_PWM
    global leftSpeed
    global rightSpeed
    global ctr_button_pos
    global button_size
    global sysRunning_flag
    global displayLayer

    ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
    while (not under):
        print('in align')
        # check for quit button
        for event in pygame.event.get():
            if(event.type is MOUSEBUTTONDOWN):
                pos = pygame.mouse.get_pos()
            elif(event.type is MOUSEBUTTONUP):
                pos = pygame.mouse.get_pos()
                x,y = pos
                    
                # if touch the quit button, the program will quit and Roomba will stop
                if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                       and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                   print("quit button is pressed")
                   ser.write('\x92\x00\x00\x00\x00')   
                   sysRunning_flag = False
                   displayLayer = 1
                   break

        if(LEFT_UNDER):
            leftSpeed = '\x00\x00'
        if(RIGHT_UNDER):
            rightSpeed = '\x00\x00'
        #else:
            #rightSpeed = '\x00\x8F'
            #leftSpeed = '\x00\x8F'

        ser.write(MOTOR_PWM + rightSpeed + leftSpeed)
        under = LEFT_UNDER and RIGHT_UNDER



# mow the lawn algorithmdisplayLayer
def mow():
    global LED_ON
    global RIGHT_UNDER
    global LEFT_UNDER
    global under
    global bg_path
    global screen
    global red
    global green
    global white
    global bottom_text_pos
    global top_text_pos
    global quit_button
    global ctr_button_pos
    global button_size
    global sysRunning_flag
    global displayLayer

    # update screen 
    bg = pygame.image.load(bg_path)
    screen.blit(bg, bg.get_rect())      
    display_screen.display_button_withCircle(quit_button, white, red, ctr_button_pos)
    display_screen.display_text("ROOMBA IN PROGRESS", white, bottom_text_pos)
    display_screen.display_text("CAUTION: LED ON!!!!", red, top_text_pos)
    pygame.display.flip()  # display everything on the screen


    # mow lawn with lights on
    while(RIGHT_UNDER or LEFT_UNDER):
        # check for quit button
        for event in pygame.event.get():
            if(event.type is MOUSEBUTTONDOWN):
                pos = pygame.mouse.get_pos()
            elif(event.type is MOUSEBUTTONUP):
                pos = pygame.mouse.get_pos()
                x,y = pos
                    
                # if touch the quit button, the program will quit and Roomba will stop
                if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                       and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                   print("quit button is pressed")
                   ser.write('\x92\x00\x00\x00\x00')   
                   sysRunning_flag = False
                   displayLayer = 1
                   GPIO.output(26,0)
                   GPIO.output(19,0)
                   break

        ser.write('\x92\x00\x6F\x00\x6F')
        RIGHT_UNDER = not GPIO.input(5)
        LEFT_UNDER = not GPIO.input(6)
        #print('mowing right', RIGHT_UNDER)
        #print('mowing left', LEFT_UNDER)
        GPIO.output(26,1)
        GPIO.output(19,1)

    # lights off
    GPIO.output(26,0)
    GPIO.output(19,0)
    # update screen 
    bg = pygame.image.load(bg_path)
    screen.blit(bg, bg.get_rect())      
    display_screen.display_button_withCircle(quit_button, white, red, ctr_button_pos)
    display_screen.display_text("ROOMBA IN PROGRESS", white, bottom_text_pos)
    display_screen.display_text("LED OFF", green, top_text_pos)
    pygame.display.flip()  # display everything on the screen

## screen
display_screen = Screen(screen_back_color, screen_size)    # get a screen object
screen = display_screen.set_size()
displayLayer
try:
    while (sysRunning_flag):
        time.sleep(0.3)
        
        
        display_screen.clear_screen() # clear the screen with blackcolor 
        if(displayLayer == 1):
            for event in pygame.event.get():
                if(event.type is MOUSEBUTTONDOWN):
                    pos = pygame.mouse.get_pos()
                elif(event.type is MOUSEBUTTONUP):
                    pos = pygame.mouse.get_pos()
                    x,y = pos
                    
                    # if touch the start button, robot will move
                    if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                       and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                            print("start button is pressed")
                            modeFlag = 0
                            displayLayer = 2
                            
                    # if touch the quit button, the program will quit
                    if(y > side_button_pos[1] - button_size / 2 and y < side_button_pos[1] + button_size / 2
                       and x > side_button_pos[0] - button_size / 2 and x < side_button_pos[0] + button_size / 2):
                            print("quit button is pressed")
                            sysRunning_flag = False


            bg = pygame.image.load(bg_path)
            screen.blit(bg, bg.get_rect())      

            display_screen.display_button_withCircle(start_button, white, green, ctr_button_pos)
            display_screen.display_button(quit_button, white, side_button_pos)
            display_screen.display_text("MAKE SURE ROOMBA IS ON BEFORE STARTING", green, top_text_pos)
            display_screen.display_text("ROOMBA STOPPED", white, bottom_text_pos)

        elif(displayLayer == 2):
            for event in pygame.event.get():
                if(event.type is MOUSEBUTTONDOWN):
                    pos = pygame.mouse.get_pos()
                elif(event.type is MOUSEBUTTONUP):
                    pos = pygame.mouse.get_pos()
                    x,y = pos
                    
                    # if touch the quit button, the program will quit and Roomba will stop
                    if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                       and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                            print("quit button is pressed")
                            ser.write('\x92\x00\x00\x00\x00')   
                            sysRunning_flag = False
                            displayLayer = 1
                            
            bg = pygame.image.load(bg_path)
            screen.blit(bg, bg.get_rect())      

            display_screen.display_button_withCircle(quit_button, white, red, ctr_button_pos)
            display_screen.display_text("ROOMBA IN PROGRESS", white, bottom_text_pos)
            display_screen.display_text("LED_OFF", green, top_text_pos)
            
        pygame.display.flip()  # display everything on the screen
            

        # poll ultrasounds
        if modeFlag == 0:
            t = Timer(0.2, poll_ultrasound)
            t.start()
            ser.write('\x92\x00\x8F\x00\x8F')   
        # control robotic arm
        elif modeFlag == 1:
            z = (dist - 1.0) / 100.0
            print(dist)
            print(z)
            y = -0.9 * z + 0.4143
            goal_position = [0, y, z]     ## x, y, z should be converted to meters, like 0.025
            
            print("Goal position read: ", goal_position)
            
            s1 = serial.Serial('/dev/ttyACM0', 9600)
            s1.flushInput()

            rotating_angle = list(IK.ik_srv(goal_position))
            print("rotating angle:", rotating_angle)  
            
            rotating_angle.append(servo_ID1)
            rotating_angle[4] = 0
        
            print("rotating angle:", rotating_angle)  
            
            for i in range(0, 3):
                servo_degree[i] = int(-(((rotating_angle[i] * 360) / (2 * pi)) / 0.24) - 0.5)
            servo_degree[3] = int((((rotating_angle[3] * 360) / (2 * pi)) / 0.24) + 0.5)
            servo_degree[4] = int((((rotating_angle[4] * 360) / (2 * pi)) / 0.24) + 0.5)
            servo_degree[5] = int((((rotating_angle[5] * 360) / (2 * pi)) / 0.24) + 0.5)
            
            print("servo degree:", servo_degree)
            ## serial communication 
            
            count = 0
            
            comp_list = ["Completed\r\n", "Hello Pi, This is Arduino UNO...:\r\n", "All completed\r\n"]
            done_signal = ["done\r\n"]
            while count < 6:
                if s1.inWaiting()>0:
                    inputValue = s1.readline()
                    print(inputValue)
                    if inputValue in comp_list:
                        try:
                            n = servo_degree[count]
                            print("Pi's pos and number:",count,n)
                            s1.write('%d'%n)
                            count = count+1
                        except:
                            print("Input error, please input a number")
                            s1.write('0')
            
            #time.sleep(10)
            # timer version with quit button check
            start = time.time()
            while(time.time() - start < 3.5):
                for event in pygame.event.get():
                    if(event.type is MOUSEBUTTONDOWN):
                        pos = pygame.mouse.get_pos()
                    elif(event.type is MOUSEBUTTONUP):
                        pos = pygame.mouse.get_pos()
                        x,y = pos
                    
                        # if touch the quit button, the program will quit and Roomba will stop
                        if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                            and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                            print("quit button is pressed")
                            ser.write('\x92\x00\x00\x00\x00')   
                            sysRunning_flag = False
                            displayLayer = 1  
                            modeFlag == 0
                            break
            modeFlag = 2

            
        # mowing the lawn
        elif modeFlag == 2:
            print(LEFT_UNDER)
            print(RIGHT_UNDER)
            if(mode2Flag == 1):
                #time.sleep(2)
                # timer version with quit button check
                start = time.time()
                while(time.time() - start < 3.5):
                    for event in pygame.event.get():
                        if(event.type is MOUSEBUTTONDOWN):
                            pos = pygame.mouse.get_pos()
                        elif(event.type is MOUSEBUTTONUP):
                            pos = pygame.mouse.get_pos()
                            x,y = pos
                    
                            # if touch the quit button, the program will quit and Roomba will stop
                            if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                                and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                                print("quit button is pressed")
                                ser.write('\x92\x00\x00\x00\x00')   
                                sysRunning_flag = False
                                displayLayer = 1  
                                modeFlag == 0
                                break
                mode2Flag = 0
            else:
                if(LEFT_UNDER or RIGHT_UNDER):
                    ser.write('\x80')
                    ser.write('\x83')
                    ser.write('\x92\x00\x00\x00\x00')
                    time.sleep(0.1)
                    print("left", LEFT_UNDER)
                    print("right", RIGHT_UNDER)

                    print("in align")
                    align()
                    ser.write('\x92\x00\x00\x00\x00')
                    print("wait 0.3 sec")
                    time.sleep(0.3)
                    print("mowing...")
                    mow()

                    if(turn_CW):
                        print('in turn')
                        ser.write('\x92\x00\x00\x00\x6F')
                        #time.sleep(3.5)
                        # timer version with quit button check
                        start = time.time()
                        while(time.time() - start < 3.5):
                            for event in pygame.event.get():
                                if(event.type is MOUSEBUTTONDOWN):
                                    pos = pygame.mouse.get_pos()
                                elif(event.type is MOUSEBUTTONUP):
                                    pos = pygame.mouse.get_pos()
                                    x,y = pos
                    
                                    # if touch the quit button, the program will quit and Roomba will stop
                                    if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                                        and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                                        print("quit button is pressed")
                                        ser.write('\x92\x00\x00\x00\x00')   
                                        sysRunning_flag = False
                                        displayLayer = 1                           
                                        turn_CW = True
                                        break
                        turn_CW = False
                        endConditionCounter += 1
                        if (endConditionCounter >= 4):
                            sysRunning_flag = False
                            break
                            
                    else:
                        print('in turn')
                        ser.write('\x92\x00\x6F\x00\x00')
                        #time.sleep(3.5)
                        # timer version with quit button check
                        start = time.time()
                        while(time.time() - start < 3):
                            for event in pygame.event.get():
                                if(event.type is MOUSEBUTTONDOWN):
                                    pos = pygame.mouse.get_pos()
                                elif(event.type is MOUSEBUTTONUP):
                                    pos = pygame.mouse.get_pos()
                                    x,y = pos
                    
                                    # if touch the quit button, the program will quit and Roomba will stop
                                    if(y > ctr_button_pos[1] - button_size / 2 and y < ctr_button_pos[1] + button_size / 2
                                        and x > ctr_button_pos[0] - button_size / 2 and x < ctr_button_pos[0] + button_size / 2):
                                        print("quit button is pressed")
                                        ser.write('\x92\x00\x00\x00\x00')   
                                        sysRunning_flag = False
                                        displayLayer = 1                           
                                        turn_CW = True
                                        break
                        turn_CW = True
                        endConditionCounter += 1
                        if (endConditionCounter >= 4):
                            sysRunning_flag = False
                            break
                else:
                    ser.write('\x92\x00\x4F\x00\x4F')

        else:
            pass
        
        

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    ser.close()
    
print("exit")
ser.write('\x92\x00\x00\00\00')
GPIO.cleanup()

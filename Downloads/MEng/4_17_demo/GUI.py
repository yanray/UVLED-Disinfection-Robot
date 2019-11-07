import os, sys, pygame, time
from pygame.locals import *
import numpy as np
import RPi.GPIO as GPIO

## display on piTFT
##os.putenv('SDL_VIDEODRIVER', 'fbcon')
##os.putenv('SDL_FBDEV', '/dev/fb1')
##os.putenv('SDL_MOUSEDRV', 'TSLIB')
##os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')


screen_back_color = (0, 0, 0)        # screen color Black
screen_size = width, height = (320, 240)   # screen size
black = (0, 0, 0)  
white = (255, 255, 255)     
red = (255, 0, 0)        
green = (0, 255, 0)

## coordinates
x = 0
y = 0

## button name
start_button = 'Start'
quit_button = 'Quit'

## button
button_size = 40
start_button_pos = (160, 120)
quit_button_pos = (220, 200)

bg_path = "/home/pi/MEng/clean.jpeg"

## system flag
sysRunning_flag = True    # system Running flag


def GPIO27_callback(channel):
    print ("")
    print "Button 27 pressed..."
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    
    
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
# quit button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(19, GPIO.OUT)
GPIO.output(19, 0)

GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=200)

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

## screen
display_screen = Screen(screen_back_color, screen_size)    # get a screen object
screen = display_screen.set_size()


try:
    while (sysRunning_flag):       
        time.sleep(0.02)
        
        display_screen.clear_screen() # clear the screen with blackcolor 
        
        for event in pygame.event.get():
            if(event.type is MOUSEBUTTONDOWN):
                pos = pygame.mouse.get_pos()
            elif(event.type is MOUSEBUTTONUP):
                pos = pygame.mouse.get_pos()
                x,y = pos
                
                # if touch the start button, robot would go 
                if(y > start_button_pos[1] - button_size / 2 and y < start_button_pos[1] + button_size / 2
                   and x > start_button_pos[0] - button_size / 2 and x < start_button_pos[0] + button_size / 2):
                        print("Button start is pressed")
                        GPIO.output(19, 1)
                        
                # if touch the quit button, the program will be quitted
                if(y > quit_button_pos[1] - button_size / 2 and y < quit_button_pos[1] + button_size / 2
                   and x > quit_button_pos[0] - button_size / 2 and x < quit_button_pos[0] + button_size / 2):
                        print("Button quit is pressed")
                        sysRunning_flag = False


        bg = pygame.image.load(bg_path)
        screen.blit(bg, bg.get_rect())      

        display_screen.display_button_withCircle(start_button, white, red, start_button_pos)
        display_screen.display_button(quit_button, white, quit_button_pos)
        
        pygame.display.flip()  # display everything on the screen

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
    
GPIO.cleanup()


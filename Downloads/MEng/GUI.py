import pygame
import os
import RPi.GPIO as GPIO
import time

os.putenv('SDL_VIDEODRIVER', 'fbcon')
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV', 'TSLIB')
os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

pygame.init()
pygame.mouse.set_visible(False)
screen = pygame.display.set_mode((320,240))

BLACK = 0,0,0
WHITE = 255,255,255
RED = 255,0,0
GRAY = 180,0,0

TABLE = False
LED = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)


sysRunning_flag = True

def GPIO27_callback(channel):
    global sysRunning_flag
    sysRunning_flag = False

def GPIO17_callback(channel):
    global TABLE
    TABLE = not TABLE


def GPIO22_callback(channel):
    global LED
    LED = not LED

GPIO.add_event_detect(27,GPIO.FALLING,callback=GPIO27_callback, bouncetime=300)
GPIO.add_event_detect(17,GPIO.FALLING,callback=GPIO17_callback, bouncetime=300)
GPIO.add_event_detect(22,GPIO.FALLING,callback=GPIO22_callback, bouncetime=300)


# define quit and start buttons
font = pygame.font.Font(pygame.font.get_default_font(), 20)
my_button = {'RELEASE THE ROOMBA!':(160,120)}
my_button2 = {'STOP':(160,120), 'LED':(160,70), 'MODE:':(160,20)}

try: 
    while(sysRunning_flag):
        screen.fill(BLACK)  

        if(TABLE):
            bg_path = "/home/pi/MEng/clean.jpeg"
            MODE = "CLEANING..."
        else:
            bg_path = "/home/pi/MEng/search.png"
            MODE = "WANDER..."

        if(LED):
            ledSTAT = "ON"
            COLOR = RED
        else:
            ledSTAT = "OFF"
            COLOR = GRAY

        bg = pygame.image.load(bg_path)
        screen.blit(bg, bg.get_rect())           
        for my_text, text_pos in my_button2.items():
            if(my_text == 'MODE:'):
                text_surface = font.render(str(my_text + MODE), True, BLACK) 
            elif(my_text == 'LED'):
                text_surface = font.render(str(my_text + " " + ledSTAT), True, COLOR)
            
            else:
                text_surface = font.render(my_text, True, BLACK)
            rect = text_surface.get_rect(center=text_pos)
            screen.blit(text_surface, rect)

        for event in pygame.event.get():
            if(event.type == pygame.MOUSEBUTTONDOWN):
                position = pygame.mouse.get_pos()
            elif(event.type == pygame.MOUSEBUTTONUP):
                position = pygame.mouse.get_pos()
                x,y = position
                print(position)
                text_surface = font.render('STOP', True, WHITE)
                rect = text_surface.get_rect(center=my_button2['STOP'])
#                print(my_button2['STOP'])
                if( rect.collidepoint(position)):
                    pygame.draw.circle(screen, COLOR, [50, 100], 20)
                    print("pushed")

        pygame.display.flip()
        time.sleep(0.2)

except KeyboardInterrupt:
    GPIO.cleanup()

GPIO.cleanup()

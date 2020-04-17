####text-without-filter)


import time
import numpy as np
import cv2
from picamera import PiCamera
import subprocess
import os
from PIL import Image
import pytesseract

#camera = PiCamera()                #initialise an object of class picamera

#camera.rotation = 270

#if os.path.isfile("/home/pi/Project/image.PNG"):
 #   os.remove("/home/pi/Project/image.PNG")

#print("FILE REMOVED")

#camera.start_preview()             #use this object to start camera
#time.sleep(3)                      #delay
#camera.capture('image.PNG')        #capture image and save as file image.jpg
#camera.stop_preview()              #use this object to stop camera preview

img = cv2.imread('keyboard.jpg')

img_gray = img[10:200, 10:200] #starting wala upar kamm karre h to upar hora h
#,2nd = niche k liye(increase to zada niche), left k liye (jitna kamm utna left me jayega, increase to zada right)

cv2.imwrite('gray_before.jpg',img_gray)

# RGB to Greyscale

img = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)

cv2.imwrite('gray_converted.jpg',img)

text = pytesseract.image_to_string(img,lang='eng')
print(text)


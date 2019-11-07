"""
Loads and displays a video.
"""

# Importing OpenCV
from picamera.array import PiRGBArray
from picamera import PiCamera

import time
import cv2
import numpy as np


# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()
#camera.resolution = (640, 480)
#camera.framerate = 32
#rawCapture = PiRGBArray(camra, size=(640, 480))
img = cv2.imread('test8.jpg')

#ret, frame = cap.read()

#img = np.flip(imgaxis=1)
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
# range for black
lower_black = np.array([0,0,0])
upper_black = np.array([180,30,100])
mask = cv2.inRange(hsv, lower_black, upper_black)

    #mask = cv2.morphologyEx(mask,MORPH_OPEN, np.ones((3,3),np.uint8)
    #mask = cv2.morphologyEx(mask,MORPH_DILATE, np.ones((3,3),np.uint8)   
   
    #mask_not = cv2.bitwise_not(mask)

res = cv2.bitwise_and(img,img,mask=mask)

contours, hierarhcy = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
for j, contour in enumerate(contours):
    rect = cv2.minAreaRect(contour)
    bbox = cv2.cv.BoxPoints(rect)
    bbox = np.int0(bbox)
    #contour_mask = np.zeros_like(mask)
    #cv2.drawContours(contour_mask, contours, j, 255, -1)
        
    #result = cv2.bitwise_and(res,res,mask=contour_mask)
    result = img 
    area = rect[1][0] * rect[1][1]
#    print(area)
    if (area > 1000):
#    cv2.rectangle(result, top_left, bottom_right, (255,255,0),2)
        cv2.drawContours(result, [bbox], 0, (255,255,0),2)

    #imshow("bounded", result)
    cv2.imwrite("result8.jpeg", result)

   # key = cv2.waitKey(33)
    # Press Q on keyboard to  exit
   # if key == 27:
    #  break
   # elif key == -1:
    #    continue
   # else:
    #    print key


# When everything done, release the video capture object
#cap.release()

# Closes all the frames
cv2.destroyAllWindows()

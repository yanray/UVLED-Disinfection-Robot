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
#rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

# Read the video
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:

    # Converting the image to grayscale.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Using the Canny filter to get contours
    edges = cv2.Canny(gray, 20, 30)
    # Using the Canny filter with different parameters
    edges_high_thresh = cv2.Canny(gray, 30, 150)
    # Stacking the images to print them together
    # For comparison
    canny_images = np.hstack((gray, edges_high_thresh))

    
    imThres = edges_high_thresh
    cnts= cv2.findContours(imThres, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    print(cnts)

    # Display the resulting frame
    cv2.imshow('Frame', canny_images)
    

    key = cv2.waitKey(33)
    # Press Q on keyboard to  exit
    if key == 27:
      break
    elif key == -1:
        continue
    else:
        print key


# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()

import cv2
import numpy as np

#cap = cv2.VideoCapture(0)

# set blue thresh
lower_blue=np.array([78,43,46])
upper_blue=np.array([110,255,255])

frame = cv2.imread('foobar.jpg')

hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_blue, upper_blue)



# get a frame and show
#ret, frame = cap.read()
cv2.imshow('Capture', frame)

# change to hsv model
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# get mask
mask = cv2.inRange(hsv, lower_blue, upper_blue)
cv2.imshow('Mask', mask)

# detect blue
res = cv2.bitwise_and(frame, frame, mask=mask)
cv2.imshow('Result', res)

cnts= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
while(1):
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

print(cnts)
cap.release()
cv2.destroyAllWindows()

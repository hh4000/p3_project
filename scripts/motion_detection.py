# import libraries
import cv2 as cv
import numpy as np
from time import sleep
from imutils.object_detection import non_max_suppression

def motionDetectionNMS(camera):
# define a video capture object
	vid = cv.VideoCapture(camera)

	previousFrame = None

	while(True):
		sleep(0.05)
		# Capture the video frame
		# by frame
		ref, frame = vid.read()

		grayFrame = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		blurredFrame= cv.GaussianBlur(grayFrame,(11,11),0)
		preparedFrame = blurredFrame
		if (previousFrame is None):
			previousFrame = preparedFrame

		diffFrame = cv.absdiff(src1=previousFrame,src2=preparedFrame)
		previousFrame = preparedFrame

		#Dilate the image to make differences more seeable
		kernel = np.ones((21,21))
		diffFrame = cv.dilate(diffFrame,kernel,20)
		diffFrame = cv.morphologyEx(diffFrame,cv.MORPH_CLOSE,kernel)
		threshFrame= cv.threshold(src=diffFrame, thresh=20, maxval=255,type=cv.THRESH_BINARY)[1]
		#erodedFrame = cv.erode(threshFrame,kernel,None,(-1,-1),2)
		contours, _ = cv.findContours(image=threshFrame, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_SIMPLE)
		rects = []

		for contour in contours:
			if cv.contourArea(contour) < 3000:
				# too small: skip!
				continue
			
			(x, y, w, h) = cv.boundingRect(contour)
			rects.append((x, y, w, h))
			#cv.rectangle(img=frame, pt1=(x, y), pt2=(x + w, y + h), color=(0, 255, 0), thickness=2)

		pick = non_max_suppression(np.array(rects))

		for (x, y, w, h) in pick:
		# draw the bounding box on the image
		#print("X",startX,"      ", "Y",startY)
			cv.rectangle(frame, (x, y), (x + w, y + h),
				(0, 0, 255), 2)

		# Display the resulting frame
		cv.imshow('ThreshedFrame', threshFrame)
		cv.imshow('Img', frame)
		
		if cv.waitKey(1) & 0xFF == ord('0'):
			break

	# Destroy all the windows
	cv.destroyAllWindows()

motionDetectionNMS(1)
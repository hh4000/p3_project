# import libraries
import cv2 as cv
import numpy as np
from time import sleep
from imutils.object_detection import non_max_suppression
import rospy
from std_msgs.msg import Bool

def motionDetectionNMS():
# define a video capture object

	rospy.init_node('motion_detecter',anonymous=True)
	bool_publiser = rospy.Publisher('/motion_detection/motion_detected', Bool, queue_size=10)
	rate = rospy.Rate(10)
 
 
	camera = int(input('Input camera ID: '))
	vid = cv.VideoCapture(camera)

	previousFrame = None

	while not rospy.is_shutdown():
		is_motion = Bool()
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
			if cv.contourArea(contour) < 3000: #Movement threshold
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

		if len(pick) > 0:
			is_motion.data = True
			rospy.loginfo('Motion is detected ')
		else:
			is_motion.data = False
		bool_publiser.publish(is_motion)
		rate.sleep()

	# Destroy all the windows
	cv.destroyAllWindows()

#motionDetectionNMS(1)
if __name__ == '__main__':
    try:
        motionDetectionNMS()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import math
import ConfigParser
from camera_Class import VideoCamera
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from serial_handler.msg import Status
import cv2

# config_file = "/camera.cfg"

'''add draw box on face'''

rospy.init_node('detection_camera_node', anonymous=True)

current = 0
last = 0

# interacting = False

# def interact_callback(data):
# 	global interacting
# 	if data.interaction_mode:
# 		if not interacting:
# 			interacting = True
# 		else:
# 			interacting = False

# def read_Path():
# 	global config_file
# 	cf = ConfigParser.ConfigParser()
# 	val = ""
# 	ret = True
# 	path = os.path.dirname(os.path.abspath(__file__)) + config_file
# 	field = "path"
# 	key = "camera_back"
# 	try:
# 		cf.read(path)
# 		result = cf.get(field, key)
# 		if os.path.exists(result):
# 			val = "/dev/" + os.listdir(result)[0]
# 		else:
# 			ret = False
# 			rospy.loginfo("Back camera not connected")
# 	except:
# 		ret = False
# 		rospy.loginfo("Failed to read %s-%s", field, key)
# 	return ret, val


# def read_Size():
# 	global config_file
# 	cf = ConfigParser.ConfigParser()
# 	val = ""
# 	ret = True
# 	path = os.path.dirname(os.path.abspath(__file__)) + config_file
# 	field = "size"
# 	key1 = "width"
# 	key2 = "height"
# 	try:
# 		cf.read(path)
# 		result1 = cf.get(field, key1)
# 		result2 = cf.get(field, key2)
# 		val = (int(result1), int(result2))
# 	except:
# 		ret = False
# 		rospy.loginfo("Failed to read %s-%s", field, key)
# 	return ret, val


if __name__ == '__main__':
	# path = read_Path()
	# size = read_Size()
	size = (1,(640, 480))
	back_cam = VideoCamera()
	back_cam.video.set(3, size[1][0])
	back_cam.video.set(4, size[1][1])
	back_cam.video.set(5, 15)

	image_pub = rospy.Publisher("Detection_Image/image_raw", Image, queue_size = 0) 

	# rospy.Subscriber("hardware_status", Status, interact_callback)
	last = rospy.get_time()
	current = rospy.get_time()

	try:

		while not rospy.is_shutdown():
			if not back_cam.video.isOpened():
				rospy.loginfo('Unable to load back camera.')
				sleep(5)
				pass
			else:
				# if interacting:
				current = rospy.get_time()	
				frame = back_cam.get_frame()
				frame = cv2.resize(frame, size[1], interpolation=cv2.INTER_CUBIC)
				try:
					if current - last > 0.0:
						image_pub.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))
						last = rospy.get_time()
				except CvBridgeError as e:
					print e

				cv2.putText(frame,"Back Cam",(10,size[1][1]-10),cv2.FONT_HERSHEY_PLAIN, 1,(150,50,50),2)

				cv2.imshow("Back Camera", frame)

				input_folder = os.path.dirname(os.path.abspath(__file__)) + '/input/img1.jpg'

				cv2.imwrite(input_folder, frame)
				if cv2.waitKey(40) & 0xFF == 27:
					quit()

				# cv2.waitKey(40)
		rospy.spin()

	except rospy.ROSInterruptException or KeyboardInterrupt:
		pass

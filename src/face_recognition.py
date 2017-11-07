#!/usr/bin/env python
import rospy
import cv2
import os
import numpy as np
import math
import ConfigParser
from create_csv import create_csv_func
from camera_Class import VideoCamera, FaceDetector

config_file = "/camera.cfg"
rospy.init_node('recognition_camera_node', anonymous=True)


#class FaceDetector:
#	def __init__(self, xml_path):
#		self.classifier = cv2.CascadeClassifier(xml_path)

#	def detect(self, image, biggest_only=True):
#		scale_factor = 1.2
#		min_neighbors = 5
#		min_size = (30,30)
#		biggest_only = True
#		flags = cv2.CASCADE_FIND_BIGGEST_OBJECT | cv2.CASCADE_DO_ROUGH_SEARCH if biggest_only else cv2.CASCADE_SCALE_IMAGE

#		faces_coord = self.classifier.detectMultiScale(image,scaleFactor = scale_factor,minNeighbors = min_neighbors,minSize = min_size,flags = flags)
#		return faces_coord

#class VideoCamera:
#	def __init__(self, index=0):
#		self.video = cv2.VideoCapture(index)
#		self.index = index
#		rospy.loginfo("Camera on: %d", self.video.isOpened())

#	def __del__(self):
#		self.video.release()
#		rospy.loginfo("webcam closed")

#	def get_frame(self, in_grayscale=False):
#		_, frame = self.video.read()
#		if in_grayscale:
#			frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#		return frame

def read_Path():
	global config_file
	cf = ConfigParser.ConfigParser()
	val = ""
	ret = True
	path = os.path.dirname(os.path.abspath(__file__)) + config_file
	field = "path"
	key = "camera_front"
	try:
		cf.read(path)
		result = cf.get(field, key)
		if os.path.exists(result):
			val = "/dev/" + os.listdir(result)[0]
		else:
			ret = False
			rospy.loginfo("Front camera not connected")
	except:
		ret = False
		rospy.loginfo("Failed to read %s-%s", field, key)
	return ret, val


def read_Size():
	global config_file
	cf = ConfigParser.ConfigParser()
	val = ""
	ret = True
	path = os.path.dirname(os.path.abspath(__file__)) + config_file
	field = "size"
	key1 = "width"
	key2 = "height"
	try:
		cf.read(path)
		result1 = cf.get(field, key1)
		result2 = cf.get(field, key2)
		val = (int(result1), int(result2))
	except:
		ret = False
		rospy.loginfo("Failed to read %s-%s", field, key)
	return ret, val

def draw_circle(event, x, y, flags, param):
	global x_in, y_in
	print (x,y)
	if event == cv2.EVENT_LBUTTONDOWN:
		x_in = x
		y_in = y
	elif event == cv2.EVENT_LBUTTONUP:
		cv2.circle(mask,(x_in, y_in),int(math.sqrt((y-y_in)**2+(x-x_in)**2))
				, (0,255,0), -1)

def cut_faces(image, faces_coord):
	faces = []
	for (x,y,w,h) in faces_coord:
		w_rm = int(0.2 * w/2)
		faces.append(image[y:y+h,x+w_rm:x+w-w_rm])
	return faces

def normalize_intensity(images):
	images_norm = []
	for image in images:
		is_color = len(image.shape) == 3
		if is_color:
			image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		images_norm.append(cv2.equalizeHist(image))
		# cv2.imshow("image",image)
		# cv2.imshow("image_norm",images_norm[0])
		# cv2.waitKey(0)
	return images_norm

def resize(images, size=(200,200)):
	images_norm = []
	for image in images:
		if image.shape < size:
			image_norm = cv2.resize(image,size,interpolation=cv2.INTER_AREA)
		else:
			image_norm = cv2.resize(image,size,interpolation = cv2.INTER_CUBIC)
		images_norm.append(image_norm)
	return images_norm

def normalize_faces(frame, faces_coord):
	faces = cut_faces(frame, faces_coord)
	faces = normalize_intensity(faces)
	faces = resize(faces)
	return faces

def draw_rectangle(image,coords):
	for (x,y,w,h) in coords:
		w_rm = int(0.2*w/2)
		cv2.rectangle(image,(x+w_rm,y),(x+w-w_rm,y+h),(123,32,100),5)


def collect_dataset():
	images = []
	labels = []
	labels_dic = {}
	parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/images/"
	path = os.listdir(parent_path)
	people = []
	for k in path:
		if os.path.isdir(parent_path + k):
			people.append(k)
	for i, person in enumerate(people):
		labels_dic[i] = person
		for image in os.listdir(parent_path + person):
			images.append(cv2.imread(parent_path + person + "/" + image, 0))
			labels.append(i)
	return (images, np.array(labels), labels_dic)

def Capture(camera_path, imgNumbering = 20):
	webcam = VideoCamera(camera_path)
	detector = FaceDetector('haarcascade_frontalface_default.xml')

	folder = "images"
	path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/" + folder
	subfolder = raw_input('Person: ').lower()
	cv2.namedWindow("Capturing Images", cv2.WINDOW_AUTOSIZE)
	if not os.path.exists(path + '/' + subfolder):
		if os.path.exists(path):
			pass
		else:
			os.mkdir(path)
		os.mkdir(path +'/' + subfolder)
	
	new_path = path + '/' + subfolder

	count = len(os.listdir(new_path))

	counter = count + 1
	imgNumbering = count + imgNumbering
	timer = 0
	while counter < imgNumbering:
		frame = webcam.get_frame()
		faces_coord = detector.detect(frame)
		if len(faces_coord) and timer %700 == 50:
			faces = normalize_faces(frame, faces_coord)
			cv2.imwrite(new_path + '/img' + str(counter) + '.jpg',faces[0])
			cv2.imshow("Image saved: %d"%counter,faces[0])
			cv2.waitKey(20)
			cv2.destroyWindow("Image saved: %d"%(counter-1))
			counter += 1
		draw_rectangle(frame, faces_coord)
		cv2.imshow("Capturing Images", frame)
		cv2.waitKey(20)
		timer += 50
		# if cv2.waitKey(20) & 0xFF == 27:
		# 	break
	cv2.destroyAllWindows()

	del webcam
	create_csv_func()

def Recognition(camera_path):

	images, labels, labels_dic = collect_dataset()

	# rec_eig = cv2.face.createEigenFaceRecognizer()
	# rec_eig.train(images, labels)

	# rec_fisher = cv2.face.createFisherFaceRecognizer()
	# rec_fisher.train(images,labels)

	rec_lbph = cv2.face.createLBPHFaceRecognizer()
	rec_lbph.train(images,labels)

	rospy.loginfo("Models Trained Succesfully")

	# a = raw_input("Press Enter: ")
	# if a + "1":

	webcam = VideoCamera(camera_path)

	detector=FaceDetector('haarcascade_frontalface_default.xml')
	cv2.namedWindow("Recognition", cv2.WINDOW_AUTOSIZE)

	
	while not rospy.is_shutdown():
		try:
			frame = webcam.get_frame()
			faces_coord = detector.detect(frame)
			if len(faces_coord):
				faces = normalize_faces(frame, faces_coord)
				for i, face in enumerate(faces):
					pred, conf = rec_lbph.predict(face)
					threshold = 50
					if conf <= threshold:
						
						rospy.loginfo("Prediction: %s\nConfidence: %s", labels_dic[pred].capitalize(), str(round(conf)))
						cv2.putText(frame, labels_dic[pred].capitalize(),\
							(faces_coord[i][0],faces_coord[i][1]-10), \
							cv2.FONT_HERSHEY_PLAIN, 1, (66,53,243), 2)
					else:
						
						rospy.loginfo("Prediction: Unknown")
						cv2.putText(frame, "Unknown",\
							(faces_coord[i][0],faces_coord[i][1]-10), \
							cv2.FONT_HERSHEY_PLAIN, 1, (66,53,243), 2)
					
				draw_rectangle(frame, faces_coord)
			cv2.imshow("Recognition", frame)
			if cv2.waitKey(20) & 0xFF == 27:
				cv2.destroyAllWindows()
				break
		except rospy.ROSInterruptException or KeyboardInterrupt:
			break
	del webcam

if __name__=='__main__':
	path = read_Path()
	path = (True, 0)
	if path[0]:
		mode = input("Mode 1 - capture images for new faces/update existing faces\n\
			Mode 2 - execute face recognition\n\
			Mode:  ")
		if mode == 1:
			Capture(path[1], 10)
		elif mode == 2:
			Recognition(path[1])
		else:
			rospy.logerr("Wrong mode number")
	else:
		rospy.logerr("Camera not connected")
	rospy.loginfo("quit program")
	

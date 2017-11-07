import cv2




class FaceDetector(object):
	def __init__(self, xml_path):
		self.classifier = cv2.CascadeClassifier(xml_path)

	def detect(self, image, biggest_only=True):
		scale_factor = 1.1
		min_neighbors = 5
		min_size = (30,30)
		biggest_only = True
		flags = cv2.CASCADE_FIND_BIGGEST_OBJECT | cv2.CASCADE_DO_ROUGH_SEARCH if biggest_only else cv2.CASCADE_SCALE_IMAGE

		faces_coord = self.classifier.detectMultiScale(image,scaleFactor = scale_factor,minNeighbors = min_neighbors,minSize = min_size,flags = flags)
		return faces_coord

class VideoCamera(object):
	def __init__(self, index=0):
		self.video = cv2.VideoCapture(index)
		self.index = index
		print self.video.isOpened()

	def __del__(self):
		self.video.release()
		print "Camera closed"

	def get_frame(self, in_grayscale=False):
		_, frame = self.video.read()
		if in_grayscale:
			frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		return frame



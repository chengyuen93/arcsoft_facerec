#!/use/bin/env python

import cv2
from camera_Class import VideoCamera, FaceDetector

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
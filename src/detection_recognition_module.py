#!/usr/bin/env python

import rospy
import sys
import os
import ConfigParser as cp
from camera_Class import VideoCamera
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from arcsoft import CLibrary, ASVL_COLOR_FORMAT, ASVLOFFSCREEN,c_ubyte_p,FaceInfo
from arcsoft.utils import BufferInfo, ImageLoader
from arcsoft.AFD_FSDKLibrary import *
from arcsoft.AFT_FSDKLibrary import *
from arcsoft.AFR_FSDKLibrary import *
from ctypes import *
import traceback

config_file = "/camera.cfg"
APPID = c_char_p(b'GftPTk199Uat2fp2X9svyMbChTUhyabJsKxn9FYUy621')
FD_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi7ua6x553hFEAvG1BuRZ5Ta')
FT_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi7nQhgscCC9GUWDGzuYmDi3')
FR_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi8QDhzkuAxfHhFrPVNwQ57C')
FD_WORKBUF_SIZE = 20 * 1024 * 1024
FT_WORKBUF_SIZE = 40 * 1024 * 1024
FR_WORKBUF_SIZE = 40 * 1024 * 1024
MAX_FACE_NUM = 10 #max20 for tracking

got_face = False
threshold = 0.6
recognise = True
# capture = False

rospy.init_node("detection_node")
detect_pub = rospy.Publisher("detected_person", String, queue_size = 1)

def read_path(key_path):
	global config_file
	cf = cp.ConfigParser()
	val = ""
	ret = True
	path = os.path.dirname(os.path.abspath(__file__)) + config_file
	field = "path"
	key = key_path
	try:
		cf.read(path)
		result = cf.get(field, key)
		if os.path.exists(result):
			val = "/dev" + os.listdir(result)[0]
		else:
			ret = False
			rospy.loginfo("Camera not connected")
	except:
		ret = False
		rospy.loginfo("Failed to read %s-%s", field, key)
	return ret, val

def read_size():
	global config_file
	cf = cp.ConfigParser()
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
		val = [int(result1), int(result2)]
	except:
		ret = False
		rospy.loginfo("Failed to read %s-%s", field, key)
	return ret, val

def doFaceDetection(hFDEngine, inputImg):
    faceInfo = []

    pFaceRes = POINTER(AFD_FSDK_FACERES)()
    ret = AFD_FSDK_StillImageFaceDetection(hFDEngine, byref(inputImg), byref(pFaceRes))
    if ret != 0:
        print(u'AFD_FSDK_StillImageFaceDetection 0x{0:x}'.format(ret))
        return faceInfo

    faceRes = pFaceRes.contents
    if faceRes.nFace > 0:
        for i in range(0, faceRes.nFace):
            rect = faceRes.rcFace[i]
            orient = faceRes.lfaceOrient[i]
            faceInfo.append(FaceInfo(rect.left,rect.top,rect.right,rect.bottom,orient))
    
    return faceInfo

def doFaceTracking(hFTEngine, inputImg):
    faceInfo = []

    pFaceRes = POINTER(AFT_FSDK_FACERES)()
    ret = AFT_FSDK_FaceFeatureDetect(hFTEngine, byref(inputImg), byref(pFaceRes))
    if ret != 0:
        print(u'AFT_FSDK_FaceFeatureDetect 0x{0:x}'.format(ret))
        return faceInfo

    faceRes = pFaceRes.contents

    if faceRes.nFace > 0:
        for i in range(0, faceRes.nFace):
			rect = faceRes.rcFace[i]
			orient = faceRes.lfaceOrient 
			faceInfo.append(FaceInfo(rect.left,rect.top,rect.right,rect.bottom,orient))
    
    return faceInfo

def extractFRFeature(hFREngine, inputImg,faceInfo):

    faceinput = AFR_FSDK_FACEINPUT()
    faceinput.lOrient = faceInfo.orient
    faceinput.rcFace.left = faceInfo.left
    faceinput.rcFace.top = faceInfo.top
    faceinput.rcFace.right = faceInfo.right
    faceinput.rcFace.bottom = faceInfo.bottom

    faceFeature = AFR_FSDK_FACEMODEL()
    ret = AFR_FSDK_ExtractFRFeature(hFREngine, inputImg, faceinput, faceFeature)
    if ret != 0:
        print(u'AFR_FSDK_ExtractFRFeature ret 0x{0:x}'.format(ret))
        return None

    try:
        return faceFeature.deepCopy()
    except Exception as e:
        traceback.print_exc()
        print(e.message)
        return None

def loadImage(filePath):

    inputImg = ASVLOFFSCREEN()
    
    bufferInfo = ImageLoader.getBGRFromFile(filePath)
    inputImg.u32PixelArrayFormat = ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8
    inputImg.i32Width = bufferInfo.width
    inputImg.i32Height = bufferInfo.height
    inputImg.pi32Pitch[0] = bufferInfo.width*3
    inputImg.ppu8Plane[0] = cast(bufferInfo.buffer, c_ubyte_p)
    inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
    inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
    inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    
    inputImg.gc_ppu8Plane0 = bufferInfo.buffer

    return inputImg


def get_feature_from_database():
	ret = 0
	featureOut = {}
	data_base_path = os.path.dirname(os.path.abspath(__file__)) + '/data_base/'
	for img_folder in os.listdir(data_base_path):
		img_list = []
		for img in os.listdir(data_base_path + img_folder):
			if 'txt' in img:
				continue
			filePathA1 = data_base_path + img_folder + '/' + img
			filePathA = u'{}'.format(filePathA1)
			inputImgA = loadImage(filePathA)

			name = img_folder[:]

			faceInfoA = doFaceDetection(hFDEngine, inputImgA)
			if len(faceInfoA) >= 1:
				faceFeatureA = extractFRFeature(hFREngine, inputImgA, faceInfoA[0])
				if faceFeatureA == None:
					# faceFeatureA.freeUnmanaged()
					rospy.logwarn("%s>>%s feature not extracted", name, img)
				else:
					img_list.append(faceFeatureA)
					# featureAs.update({name:faceFeatureA})
					# faceFeatureA.freeUnmanaged()
			
			else:
				rospy.logwarn("%s>>%s does not have a face", name, img)
			featureOut.update({name: img_list})
	# print featureAs
	if len(featureOut) == 0:
		rospy.logwarn("No faces found in the data base")
		ret = 1
	return ret, featureOut
		
def closest_index(faceInfo):
	area = 0.0
	index = 0
	for i in range(len(faceInfo)):
		face_area = abs(faceInfo[i].right - faceInfo[i].left) * abs(faceInfo[i].top - faceInfo[i].bottom)
		# print i, face_area
		if face_area >= area:
			area = face_area
			index = i
	return i

def recognition(frame, featureAs, hFREngine, inputImg, faceInfo):
	score = {}
	# for i in range(len(faceInfo)):
	i = closest_index(faceInfo)


	faceFeature = extractFRFeature(hFREngine, inputImg, faceInfo[i])
	if faceFeature == None:
		rospy.logwarn("%s", "extract face feature failed")
	else:
		# print featureAs
		for f in featureAs:
			similscore_list = []
			for ff in range(len(featureAs[f])):
				# print f
				fSimilScore = c_float(0.0)
				ret = AFR_FSDK_FacePairMatching(hFREngine, faceFeature, featureAs[f][ff], byref(fSimilScore))
		        # faceFeature.freeUnmanaged()
				if ret != 0:
					print(u'AFR_FSDK_FacePairMatching failed:ret 0x{0:x}'.format(ret))
				similscore_list.append(fSimilScore.value)
			score.update({f: similscore_list})
			# print score
		faceFeature.freeUnmanaged()
	default_pt = 0.0
	winner = ''
	# print score
	for person in score:
		for pt in range(len(score[person])):
			if score[person][pt] >= default_pt:
				default_pt = score[person][pt]
				winner = person
	if default_pt > threshold:
		put_name = winner
		detect_pub.publish(put_name)
	else:
		put_name = "Unknown"
		# put_name = winner
	cv2.putText(frame, put_name, (faceInfo[i].left, faceInfo[i].top - 10), \
		cv2.FONT_HERSHEY_PLAIN, 1, (66,53,243), 2)
	


def takeImage(data):
    inputImg = ASVLOFFSCREEN()
    
    # bufferInfo = ImageLoader.getBGRFromFile(filePath)
    inputImg.u32PixelArrayFormat = ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8
    inputImg.i32Width = data.width
    inputImg.i32Height = data.height
    inputImg.pi32Pitch[0] = data.width*3
    inputImg.ppu8Plane[0] = cast(data.data, c_ubyte_p)
    inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
    inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
    inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    
    inputImg.gc_ppu8Plane0 = data.data

    return inputImg


if __name__ == "__main__":

	''' Camera Settings '''
	camera_path = rospy.get_param('~camera_path', 'default')

	if camera_path == 'default':
		cam = VideoCamera(0)
	else:
		ret, cam_path = read_path(camera_path)
		if ret:
			cam = VideoCamera(cam_path)
		else:
			exit(0)
	
	ret, size = read_size()
	if not ret:
		exit(0)
	else:
		if not size[0] % 2 == 0:
			size[0] = size[0] + 1
		if not size[1] % 2 == 0:
			size[1] = size[1] + 1

	cam.video.set(3, size[0]) #width
	cam.video.set(4, size[1]) #height
	cam.video.set(5, 15) #fps

	''' Face Tracking Initialisation '''

	pFTWorkMem = CLibrary.malloc(c_size_t(FT_WORKBUF_SIZE))
	hFTEngine = c_void_p()
	ret = AFT_FSDK_InitialFaceEngine(APPID, FT_SDKKEY, pFTWorkMem, c_int32(FT_WORKBUF_SIZE), byref(hFTEngine), AFT_FSDK_OPF_0_HIGHER_EXT, 16, MAX_FACE_NUM)
	if ret != 0:
		CLibrary.free(pFTWorkMem)
		print(u'AFT_FSDK_InitialFaceEngine ret 0x{:x}'.format(ret))
		exit(0)

	''' Face Recognition Initialisation and get ready the data base '''

	if recognise:
		pFDWorkMem = CLibrary.malloc(c_size_t(FD_WORKBUF_SIZE))
		hFDEngine = c_void_p()
		ret = AFD_FSDK_InitialFaceEngine(APPID, FD_SDKKEY, pFDWorkMem, c_int32(FD_WORKBUF_SIZE), byref(hFDEngine), AFD_FSDK_OPF_0_HIGHER_EXT, 32, MAX_FACE_NUM)
		if ret != 0:
			AFT_FSDK_UninitialFaceEngine(hFTEngine)
			CLibrary.free(pFDWorkMem)
			CLibrary.free(pFTWorkMem)
			print(u'AFD_FSDK_InitialFaceEngine ret 0x{:x}'.format(ret))
			exit(0)


		pFRWorkMem = CLibrary.malloc(c_size_t(FR_WORKBUF_SIZE))
		hFREngine = c_void_p()
		ret = AFR_FSDK_InitialEngine(APPID, FR_SDKKEY, pFRWorkMem, c_int32(FR_WORKBUF_SIZE), byref(hFREngine))
		if ret != 0:
			AFT_FSDK_UninitialFaceEngine(hFTEngine)
			AFD_FSDK_UninitialFaceEngine(hFDEngine)
			CLibrary.free(pFTWorkMem)
			CLibrary.free(pFRWorkMem)
			CLibrary.free(pFDWorkMem)
			print(u'AFR_FSDK_InitialEngine ret 0x{:x}'.format(ret))
			exit(0)

		ret, featureAs = get_feature_from_database()
		if ret:
			sys.exit(0)		


	try:
		rospy.loginfo("Camera rolling")
		while not rospy.is_shutdown():
			if not cam.video.isOpened():
				rospy.loginfo('Unable to load camera for detection')
				sleep(3)
				pass
			else:
				frame = cam.get_frame()
				frame = cv2.resize(frame, (size[0], size[1]), interpolation=cv2.INTER_CUBIC)
				frame_detect = frame[:]
				try:
					cam_img = CvBridge().cv2_to_imgmsg(frame, "bgr8")
				except CvBridgeError as e:
					rospy.logerr("%s", e)
					continue

				''' Detection '''

				inputImg = takeImage(cam_img)	# convert into the right format for FSDK functions

				# faceInfo = doFaceDetection(hFDEngine, inputImg)
				faceInfo = doFaceTracking(hFTEngine, inputImg)
				
				if len(faceInfo) < 1:
					rospy.loginfo("no face detected")
					got_face = False
				else:
					got_face = True
					for i in range(len(faceInfo)):
						cv2.rectangle(frame_detect, (faceInfo[i].left, faceInfo[i].top), (faceInfo[i].right, faceInfo[i].bottom), (255,100,0), 2)

				''' Extract Features '''
				if got_face and recognise:

					recognition(frame_detect, featureAs, hFREngine, inputImg, faceInfo)

				cv2.imshow("Camera", frame_detect)

				# if capture and got_face and not recognise:
				# 	if cv2.waitKey(40) & 0xFF == 13:

				# 		count = len(os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/data_base'))
				# 		save_path = os.path.dirname(os.path.abspath(__file__)) + '/data_base/img%d.jpg'%(count+1)
				# 		cv2.imwrite(save_path, frame) 

				# 	elif cv2.waitKey(40) & 0xFF == 27:
				# 		AFT_FSDK_UninitialFaceEngine(hFTEngine)
				# 		CLibrary.free(pFTWorkMem)
				# 		quit()

				# else:
				if cv2.waitKey(40) & 0xFF == 27:
					# AFD_FSDK_UninitialFaceEngine(hFDEngine)
					AFT_FSDK_UninitialFaceEngine(hFTEngine)
					# CLibrary.free(pFDWorkMem)
					CLibrary.free(pFTWorkMem)
					if recognise:
						AFR_FSDK_UninitialEngine(hFREngine)
						CLibrary.free(pFRWorkMem)
					exit(0)

		# AFD_FSDK_UninitialFaceEngine(hFDEngine)
		AFT_FSDK_UninitialFaceEngine(hFTEngine)
		# CLibrary.free(pFDWorkMem)
		CLibrary.free(pFTWorkMem)
		if recognise:
			AFR_FSDK_UninitialEngine(hFREngine)
			CLibrary.free(pFRWorkMem)
		print(u'#####################################################')

	except rospy.ROSInterruptException or KeyboardInterrupt:
		# AFD_FSDK_UninitialFaceEngine(hFDEngine)
		AFT_FSDK_UninitialFaceEngine(hFTEngine)
		# CLibrary.free(pFDWorkMem)
		CLibrary.free(pFTWorkMem)
		if recognise:
			AFR_FSDK_UninitialEngine(hFREngine)
			CLibrary.free(pFRWorkMem)
		print(u'#####################################################')
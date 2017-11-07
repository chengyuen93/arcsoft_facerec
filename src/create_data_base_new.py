#!/usr/bin/env python

import rospy
import sys
import os
import ConfigParser as cp
from camera_Class import VideoCamera
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from arcsoft import CLibrary, ASVL_COLOR_FORMAT, ASVLOFFSCREEN,c_ubyte_p,FaceInfo
from arcsoft.utils import BufferInfo, ImageLoader
# from arcsoft.AFD_FSDKLibrary import *
from arcsoft.AFT_FSDKLibrary import *
from arcsoft.AFR_FSDKLibrary import *
from ctypes import *
import traceback

config_file = "/camera.cfg"
APPID = c_char_p(b'GftPTk199Uat2fp2X9svyMbChTUhyabJsKxn9FYUy621')
# FD_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi7ua6x553hFEAvG1BuRZ5Ta')
FT_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi7nQhgscCC9GUWDGzuYmDi3')
FR_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi8QDhzkuAxfHhFrPVNwQ57C')
# FD_WORKBUF_SIZE = 20 * 1024 * 1024
FT_WORKBUF_SIZE = 40 * 1024 * 1024
FR_WORKBUF_SIZE = 40 * 1024 * 1024
MAX_FACE_NUM = 10 #max20 for tracking

got_face = False
got_feature = False

# recognise = True
# capture = False
phase = 0

rospy.init_node("capture_node")

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

def doFaceDetection(hFTEngine, inputImg):
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

# def loadImage(filePath):

#     inputImg = ASVLOFFSCREEN()
    
#     bufferInfo = ImageLoader.getBGRFromFile(filePath)
#     inputImg.u32PixelArrayFormat = ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8
#     inputImg.i32Width = bufferInfo.width
#     inputImg.i32Height = bufferInfo.height
#     inputImg.pi32Pitch[0] = bufferInfo.width*3
#     inputImg.ppu8Plane[0] = cast(bufferInfo.buffer, c_ubyte_p)
#     inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
#     inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
#     inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    
#     inputImg.gc_ppu8Plane0 = bufferInfo.buffer

#     return inputImg


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

	# if recognise:
	pFRWorkMem = CLibrary.malloc(c_size_t(FR_WORKBUF_SIZE))
	hFREngine = c_void_p()
	ret = AFR_FSDK_InitialEngine(APPID, FR_SDKKEY, pFRWorkMem, c_int32(FR_WORKBUF_SIZE), byref(hFREngine))
	if ret != 0:
		AFT_FSDK_UninitialFaceEngine(hFTEngine)
		CLibrary.free(pFTWorkMem)
		CLibrary.free(pFRWorkMem)
		print(u'AFR_FSDK_InitialEngine ret 0x{:x}'.format(ret))
		exit(0)

	# 	featureAs = {}
	# 	for img in os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/data_base'):
	# 		# print img
	# 		filePathA1 = os.path.dirname(os.path.abspath(__file__)) + '/data_base/' + img
	# 		filePathA = u'{}'.format(filePathA1)
	# 		inputImgA = loadImage(filePathA)

	# 		name = img[:-4]

	# 		faceInfoA = doFaceDetection(hFREngine, inputImgA)
	# 		if len(faceInfoA) >= 1:
	# 			faceFeatureA = extractFRFeature(hFREngine, inputImgA, faceInfoA[0])
	# 			if faceFeatureA == None:
	# 				faceFeatureA.freeUnmanaged()
	# 				rospy.logwarn("%s feature not extracted", name)
	# 			else:
	# 				featureAs[name] = faceFeatureA
	# 		else:
	# 			rospy.warn("%s does not have a face", name)
	# 	if len(featureAs) == 0:
	# 		rospy.logwarn("No faces found in the data base")
	# 		exit(0)


	try:
		rospy.loginfo("Camera rolling")
		while not rospy.is_shutdown():
			if not cam.video.isOpened():
				rospy.loginfo('Unable to load camera for detection')
				sleep(3)
				pass
			else:
				if phase == 0:
					try:
						person_name = raw_input("Name: ")
						phase = 1
					except KeyboardInterrupt:
						sys.exit(0)
				elif phase == 1:
					frame = cam.get_frame()
					frame_detect = cv2.resize(frame, (size[0], size[1]), interpolation=cv2.INTER_CUBIC)
					
					try:
						cam_img = CvBridge().cv2_to_imgmsg(frame_detect, "bgr8")
					except CvBridgeError as e:
						rospy.logerr("%s", e)
						continue

					''' Detection '''

					inputImg = takeImage(cam_img)	# convert into the right format for FSDK functions

					# faceInfo = doFaceDetection(hFDEngine, inputImg)
					faceInfo = doFaceDetection(hFTEngine, inputImg)
					
					if len(faceInfo) < 1:
						rospy.loginfo("no face detected")
						got_face = False
					else:
						got_face = True
						for i in range(len(faceInfo)):
							cv2.rectangle(frame_detect, (faceInfo[i].left, faceInfo[i].top), (faceInfo[i].right, faceInfo[i].bottom), (255,100,0), 2)

					''' Extract Features '''
					if got_face:
						faceFeature = extractFRFeature(hFREngine, inputImg, faceInfo[0])
						if faceFeature == None:
							got_feature = False
						else:
							got_feature = True
					# if got_face and recognise:
					# 	score = {}
					# 	for i in range(len(faceInfo)):
					# 		faceFeature = extractFRFeature(hFREngine, inputImg, faceInfo[i])
					# 		if faceFeature == None:
					# 			rospy.logwarn("%s", "extract face feature failed")
					# 		else:
					# 			for f in featureAs:
					# 				fSimilScore = c_float(0.0)
					# 		        ret = AFR_FSDK_FacePairMatching(hFREngine, faceFeature, featureAs[f], byref(fSimilScore))
					# 		        faceFeature.freeUnmanaged()
					# 		        if ret != 0:
					# 		            print(u'AFR_FSDK_FacePairMatching failed:ret 0x{0:x}'.format(ret))
					# 		        score[f] = fSimilScore.value
						
					# 		default_pt = 0.0
					# 		winner = ''
					# 		for person in score:
					# 			if score[person] >= default_pt:
					# 				default_pt = score[person]
					# 				winner = person
					# 		if default_pt > 0.6:
					# 			put_name = winner
					# 		else:
					# 			put_name = "Unknown"
					# 		cv2.putText(frame_detect, put_name, (faceInfo[i].left, faceInfo[i].top - 10), \
					# 			cv2.FONT_HERSHEY_PLAIN, 1, (66,53,243), 2)
					
					text = ''
					if got_face:
						if got_feature:
							text = 'ready'
						else:
							text = 'waiting to get feature'
					else:
						text = 'no face detected'
					cv2.putText(frame_detect, text, (10, size[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1, (66, 53,243), 2)

					cv2.imshow("Camera", frame_detect)

					if got_face:
						if cv2.waitKey(20) & 0xFF == 13:
							frame = cv2.resize(frame, (size[0], size[1]), interpolation=cv2.INTER_CUBIC)
							# count = len(os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/data_base'))
							# save_path = os.path.dirname(os.path.abspath(__file__)) + '/data_base/img%d.jpg'%(count+1)
							person_folder = os.path.dirname(os.path.abspath(__file__)) + '/data_base/' + person_name
							if not os.path.exists(person_folder):
								os.mkdir(person_folder)
							count = len(os.listdir(person_folder))
							save_path = person_folder + '/img%d.jpg'%(count+1)

							cv2.imwrite(save_path, frame) 
							text = 'image taken'
							cv2.putText(frame, text, (10, size[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1, (66, 53,243), 2)

							cv2.imshow("Camera", frame)
							cv2.waitKey(1000)
							sys.exit(0)

						elif cv2.waitKey(40) & 0xFF == 27:
							print "releasing engine"
							AFT_FSDK_UninitialFaceEngine(hFTEngine)
							CLibrary.free(pFTWorkMem)
							AFR_FSDK_UninitialEngine(hFREngine)
							CLibrary.free(pFRWorkMem)
							print "quitting"
							# quit()
							sys.exit(0)

					else:
						if cv2.waitKey(40) & 0xFF == 27:
							# AFD_FSDK_UninitialFaceEngine(hFDEngine)
							AFT_FSDK_UninitialFaceEngine(hFTEngine)
							# CLibrary.free(pFDWorkMem)
							CLibrary.free(pFTWorkMem)
							# if recognise:
							AFR_FSDK_UninitialEngine(hFREngine)
							CLibrary.free(pFRWorkMem)
							# quit()
							sys.exit(0)

		AFR_FSDK_UninitialEngine(hFREngine)
		CLibrary.free(pFRWorkMem)
		# AFD_FSDK_UninitialFaceEngine(hFDEngine)
		AFT_FSDK_UninitialFaceEngine(hFTEngine)
		# CLibrary.free(pFDWorkMem)
		CLibrary.free(pFTWorkMem)
		# if recognise:
		# 	AFR_FSDK_UninitialEngine(hFREngine)
		# 	CLibrary.free(pFRWorkMem)
		print(u'#####################################################')

	except rospy.ROSInterruptException or KeyboardInterrupt:
		# AFD_FSDK_UninitialFaceEngine(hFDEngine)
		AFT_FSDK_UninitialFaceEngine(hFTEngine)
		# CLibrary.free(pFDWorkMem)
		CLibrary.free(pFTWorkMem)
		AFR_FSDK_UninitialEngine(hFREngine)
		CLibrary.free(pFRWorkMem)
		# if recognise:
		# 	AFR_FSDK_UninitialEngine(hFREngine)
		# 	CLibrary.free(pFRWorkMem)
		print(u'#####################################################')
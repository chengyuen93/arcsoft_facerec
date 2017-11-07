#!/usr/bin/env python
#-*- encoding=utf-8 -*-
import rospy
from arcsoft import CLibrary, ASVL_COLOR_FORMAT, ASVLOFFSCREEN,c_ubyte_p,FaceInfo
from arcsoft.utils import BufferInfo, ImageLoader
from arcsoft.AFD_FSDKLibrary import *
from arcsoft.AFR_FSDKLibrary import *
from ctypes import *
import traceback
import os
from sensor_msgs.msg import Image

APPID = c_char_p(b'GftPTk199Uat2fp2X9svyMbChTUhyabJsKxn9FYUy621')
FD_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi7ua6x553hFEAvG1BuRZ5Ta')
FR_SDKKEY = c_char_p(b'A6BRrouepWFkXYhdEeqnHi8QDhzkuAxfHhFrPVNwQ57C')
FD_WORKBUF_SIZE = 20 * 1024 * 1024
FR_WORKBUF_SIZE = 40 * 1024 * 1024
MAX_FACE_NUM = 50
bUseYUVFile = False
bUseBGRToEngine = True

got_face = False

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


def compareFaceSimilarity(hFDEngine, hFREngine, inputImgA, inputImgB):
        global got_face
        # Do Face Detect
        faceInfosA = doFaceDetection(hFDEngine, inputImgA)
        if len(faceInfosA) < 1:
            print(u'no face in Image A ')
            got_face = False
            return 0.0

        # print "left: ", faceInfosA[0].left
        # print "top: ", faceInfosA[0].top
        # print "right: ", faceInfosA[0].right
        # print "bottom: ", faceInfosA[0].bottom
        # print "orient: ", faceInfosA[0].orient
        # print 30*"-"
        got_face = True

        faceInfosB = doFaceDetection(hFDEngine, inputImgB)
        if len(faceInfosB) < 1:
            print(u'no face in Image B ')
            return 0.0

        #Extract Face Feature
        faceFeatureA = extractFRFeature(hFREngine, inputImgA, faceInfosA[0])
        if faceFeatureA == None:
            print(u'extract face feature in Image A faile')
            return 0.0
        faceFeatureB = extractFRFeature(hFREngine, inputImgB, faceInfosB[0])
        if faceFeatureB == None:
            print(u'extract face feature in Image B failed')
            faceFeatureA.freeUnmanaged()
            return 0.0
        #calc similarity between faceA and faceB
        fSimilScore = c_float(0.0)
        ret = AFR_FSDK_FacePairMatching(hFREngine, faceFeatureA, faceFeatureB, byref(fSimilScore))
        faceFeatureA.freeUnmanaged()
        faceFeatureB.freeUnmanaged()
        if ret != 0:
            print(u'AFR_FSDK_FacePairMatching failed:ret 0x{0:x}'.format(ret))
            return 0.0
        return fSimilScore.value

def loadYUVImage(yuv_filePath, yuv_width, yuv_height, yuv_format):
    yuv_rawdata_size = 0

    inputImg = ASVLOFFSCREEN()
    inputImg.u32PixelArrayFormat = yuv_format
    inputImg.i32Width = yuv_width
    inputImg.i32Height = yuv_height
    if ASVL_COLOR_FORMAT.ASVL_PAF_I420 == inputImg.u32PixelArrayFormat:
        inputImg.pi32Pitch[0] = inputImg.i32Width
        inputImg.pi32Pitch[1] = inputImg.i32Width // 2
        inputImg.pi32Pitch[2] = inputImg.i32Width // 2
        yuv_rawdata_size = inputImg.i32Width * inputImg.i32Height * 3 // 2
    elif ASVL_COLOR_FORMAT.ASVL_PAF_NV12 == inputImg.u32PixelArrayFormat:
        inputImg.pi32Pitch[0] = inputImg.i32Width
        inputImg.pi32Pitch[1] = inputImg.i32Width
        yuv_rawdata_size = inputImg.i32Width * inputImg.i32Height * 3 // 2
    elif ASVL_COLOR_FORMAT.ASVL_PAF_NV21 == inputImg.u32PixelArrayFormat:
        inputImg.pi32Pitch[0] = inputImg.i32Width
        inputImg.pi32Pitch[1] = inputImg.i32Width
        yuv_rawdata_size = inputImg.i32Width * inputImg.i32Height * 3 // 2
    elif ASVL_COLOR_FORMAT.ASVL_PAF_YUYV == inputImg.u32PixelArrayFormat:
        inputImg.pi32Pitch[0] = inputImg.i32Width * 2
        yuv_rawdata_size = inputImg.i32Width * inputImg.i32Height * 2
    elif ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8 == inputImg.u32PixelArrayFormat:
        inputImg.pi32Pitch[0] = inputImg.i32Width * 3
        yuv_rawdata_size = inputImg.i32Width * inputImg.i32Height * 3
    else:
        print(u'unsupported  yuv format')
        exit(0)

    # load YUV Image Data from File
    f = None
    try:
        f = open(yuv_filePath, u'rb')
        imagedata = f.read(yuv_rawdata_size)
    except Exception as e:
        traceback.print_exc()
        print(e.message)
        exit(0)
    finally:
        if f is not None:
            f.close()

    if ASVL_COLOR_FORMAT.ASVL_PAF_I420 == inputImg.u32PixelArrayFormat:
        inputImg.ppu8Plane[0] = cast(imagedata, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(addressof(inputImg.ppu8Plane[0].contents) + (inputImg.pi32Pitch[0] * inputImg.i32Height), c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(addressof(inputImg.ppu8Plane[1].contents) + (inputImg.pi32Pitch[1] * inputImg.i32Height // 2), c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    elif ASVL_COLOR_FORMAT.ASVL_PAF_NV12 == inputImg.u32PixelArrayFormat:
        inputImg.ppu8Plane[0] = cast(imagedata, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(addressof(inputImg.ppu8Plane[0].contents) + (inputImg.pi32Pitch[0] * inputImg.i32Height), c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    elif ASVL_COLOR_FORMAT.ASVL_PAF_NV21 == inputImg.u32PixelArrayFormat:
        inputImg.ppu8Plane[0] = cast(imagedata, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(addressof(inputImg.ppu8Plane[0].contents) + (inputImg.pi32Pitch[0] * inputImg.i32Height), c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    elif ASVL_COLOR_FORMAT.ASVL_PAF_YUYV == inputImg.u32PixelArrayFormat:
        inputImg.ppu8Plane[0] = cast(imagedata, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    elif ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8 == inputImg.u32PixelArrayFormat:
        inputImg.ppu8Plane[0] = cast(imagedata, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    else:
        print(u'unsupported yuv format')
        exit(0)

    inputImg.gc_ppu8Plane0 = imagedata
    return inputImg

def loadImage(filePath):

    inputImg = ASVLOFFSCREEN()
    if bUseBGRToEngine:
        bufferInfo = ImageLoader.getBGRFromFile(filePath)
        inputImg.u32PixelArrayFormat = ASVL_COLOR_FORMAT.ASVL_PAF_RGB24_B8G8R8
        inputImg.i32Width = bufferInfo.width
        inputImg.i32Height = bufferInfo.height
        inputImg.pi32Pitch[0] = bufferInfo.width*3
        inputImg.ppu8Plane[0] = cast(bufferInfo.buffer, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(0, c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    else:
        bufferInfo = ImageLoader.getI420FromFile(filePath)
        inputImg.u32PixelArrayFormat = ASVL_COLOR_FORMAT.ASVL_PAF_I420
        inputImg.i32Width = bufferInfo.width
        inputImg.i32Height = bufferInfo.height
        inputImg.pi32Pitch[0] = inputImg.i32Width
        inputImg.pi32Pitch[1] = inputImg.i32Width // 2
        inputImg.pi32Pitch[2] = inputImg.i32Width // 2
        inputImg.ppu8Plane[0] = cast(bufferInfo.buffer, c_ubyte_p)
        inputImg.ppu8Plane[1] = cast(addressof(inputImg.ppu8Plane[0].contents) + (inputImg.pi32Pitch[0] * inputImg.i32Height), c_ubyte_p)
        inputImg.ppu8Plane[2] = cast(addressof(inputImg.ppu8Plane[1].contents) + (inputImg.pi32Pitch[1] * inputImg.i32Height // 2), c_ubyte_p)
        inputImg.ppu8Plane[3] = cast(0, c_ubyte_p)
    inputImg.gc_ppu8Plane0 = bufferInfo.buffer

    return inputImg

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

def img_callback(data):
    global got_face
    score = {}
    # filePathA = u'img1.jpg'
    for img in os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/data_base'):
        filePathB1 = os.path.dirname(os.path.abspath(__file__)) + '/data_base/' + img
        # filePathB = u'img2.jpg'
        filePathB = u'{}'.format(filePathB1)

        inputImgA = takeImage(data)
        inputImgB = loadImage(filePathB)

        name = img[:-4]

        score[name] = compareFaceSimilarity(hFDEngine, hFREngine, inputImgA, inputImgB)

        # print(u'similarity between the person captured and {} is {}'.format(name, compareFaceSimilarity(hFDEngine, hFREngine, inputImgA, inputImgB)))
    if got_face:
        default_pt = 0.0
        winner = ''
        for person in score:
            if score[person] >= default_pt:
                default_pt = score[person]
                winner = person
        if default_pt > 0.6:
            print "the person is %s with score %f"%(winner,default_pt)
        else:
            print "person unknown, score: %f"%default_pt
    else:
        print "no face"

if __name__ == u'__main__':
    print(u'#####################################################')

    rospy.init_node("face_recognition")
    rospy.Subscriber("Detection_Image/image_raw", Image, img_callback)
    # r = rospy.Rate(1)
    # init Engine
    pFDWorkMem = CLibrary.malloc(c_size_t(FD_WORKBUF_SIZE))
    pFRWorkMem = CLibrary.malloc(c_size_t(FR_WORKBUF_SIZE))

    hFDEngine = c_void_p()
    ret = AFD_FSDK_InitialFaceEngine(APPID, FD_SDKKEY, pFDWorkMem, c_int32(FD_WORKBUF_SIZE), byref(hFDEngine), AFD_FSDK_OPF_0_HIGHER_EXT, 32, MAX_FACE_NUM)
    if ret != 0:
        CLibrary.free(pFDWorkMem)
        print(u'AFD_FSDK_InitialFaceEngine ret 0x{:x}'.format(ret))
        exit(0)

    # print FDEngine version
    # versionFD = AFD_FSDK_GetVersion(hFDEngine)
    # print(u'{} {} {} {}'.format(versionFD.contents.lCodebase, versionFD.contents.lMajor, versionFD.contents.lMinor, versionFD.contents.lBuild))
    # print(c_char_p(versionFD.contents.Version).value.decode(u'utf-8'))
    # print(c_char_p(versionFD.contents.BuildDate).value.decode(u'utf-8'))
    # print(c_char_p(versionFD.contents.CopyRight).value.decode(u'utf-8'))

    hFREngine = c_void_p()
    ret = AFR_FSDK_InitialEngine(APPID, FR_SDKKEY, pFRWorkMem, c_int32(FR_WORKBUF_SIZE), byref(hFREngine))
    if ret != 0:
        AFD_FSDK_UninitialFaceEngine(hFDEngine)
        CLibrary.free(pFDWorkMem)
        CLibrary.free(pFRWorkMem)
        print(u'AFR_FSDK_InitialEngine ret 0x{:x}'.format(ret))
        exit(0)

    # print FREngine version
    # versionFR = AFR_FSDK_GetVersion(hFREngine)
    # print(u'{} {} {} {}'.format(versionFR.contents.lCodebase, versionFR.contents.lMajor, versionFR.contents.lMinor, versionFR.contents.lBuild))
    # print(c_char_p(versionFR.contents.Version).value.decode(u'utf-8'))
    # print(c_char_p(versionFR.contents.BuildDate).value.decode(u'utf-8'))
    # print(c_char_p(versionFR.contents.CopyRight).value.decode(u'utf-8'))

    # load Image Data
    # if bUseYUVFile:
    #     filePathA = u'001_640x480_I420.YUV'
    #     yuv_widthA = 640
    #     yuv_heightA = 480
    #     yuv_formatA = ASVL_COLOR_FORMAT.ASVL_PAF_I420

    #     filePathB = u'003_640x480_I420.YUV'
    #     yuv_widthB = 640
    #     yuv_heightB = 480
    #     yuv_formatB = ASVL_COLOR_FORMAT.ASVL_PAF_I420

    #     inputImgA = loadYUVImage(filePathA, yuv_widthA, yuv_heightA, yuv_formatA)
    #     inputImgB = loadYUVImage(filePathB, yuv_widthB, yuv_heightB, yuv_formatB)
    # else:
    try:
        while not rospy.is_shutdown():
            
            # filePathA1 = os.path.dirname(os.path.abspath(__file__)) + '/input/img1.jpg'
            # filePathA = u'{}'.format(filePathA1)
            # score = {}
            # # filePathA = u'img1.jpg'
            # for img in os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/data_base'):
            #     filePathB1 = os.path.dirname(os.path.abspath(__file__)) + '/data_base/' + img
            #     # filePathB = u'img2.jpg'
            #     filePathB = u'{}'.format(filePathB1)

            #     inputImgA = loadImage(filePathA)
            #     inputImgB = loadImage(filePathB)

            #     name = img[:-4]

            #     score[name] = compareFaceSimilarity(hFDEngine, hFREngine, inputImgA, inputImgB)

            #     # print(u'similarity between the person captured and {} is {}'.format(name, compareFaceSimilarity(hFDEngine, hFREngine, inputImgA, inputImgB)))
            # default_pt = 0.0
            # winner = ''
            # for person in score:
            #     if score[person] >= default_pt:
            #         default_pt = score[person]
            #         winner = person
            # print "the person is %s with score %f"%(winner,default_pt)
            pass
            # r.sleep()
        # rospy.spin()
        AFD_FSDK_UninitialFaceEngine(hFDEngine)
        AFR_FSDK_UninitialEngine(hFREngine)

        CLibrary.free(pFDWorkMem)
        CLibrary.free(pFRWorkMem)

        print(u'#####################################################')

    except rospy.ROSInterruptException:

        # release Engine
        AFD_FSDK_UninitialFaceEngine(hFDEngine)
        AFR_FSDK_UninitialEngine(hFREngine)

        CLibrary.free(pFDWorkMem)
        CLibrary.free(pFRWorkMem)

        print(u'#####################################################')

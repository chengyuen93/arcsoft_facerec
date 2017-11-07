#!/usr/bin/env python
#-*- encoding=utf-8 -*-
import platform
from ctypes import *
from . import MRECT,ASVLOFFSCREEN
import os

class AFT_FSDK_FACERES(Structure):
    _fields_ = [(u'nFace',c_int32),(u'rcFace',POINTER(MRECT)),(u'lfaceOrient',c_int32)]

class AFT_FSDK_Version(Structure):
    _fields_ = [(u'lCodebase',c_int32),(u'lMajor',c_int32),(u'lMinor',c_int32),(u'lBuild',c_int32),
                (u'Version',c_char_p),(u'BuildDate',c_char_p),(u'CopyRight',c_char_p)]

AFT_FSDK_OPF_0_ONLY = 0x1;       # 0; 0; ...
AFT_FSDK_OPF_90_ONLY = 0x2;      # 90; 90; ...
AFT_FSDK_OPF_270_ONLY = 0x3;     # 270; 270; ...        
AFT_FSDK_OPF_180_ONLY = 0x4;     # 180; 180; ...
AFT_FSDK_OPF_0_HIGHER_EXT = 0x5; # 0; 90; 270; 180; 0; 90; 270; 180; ...

AFT_FSDK_FOC_0 = 0x1;# 0 degree
AFT_FSDK_FOC_90 = 0x2;  # 90 degree
AFT_FSDK_FOC_270 = 0x3; # 270 degree
AFT_FSDK_FOC_180 = 0x4; # 180 degree
# AFT_FSDK_FOC_30 = 0x5;  # 30 degree
# AFT_FSDK_FOC_60 = 0x6;  # 60 degree
# AFT_FSDK_FOC_120 = 0x7; # 120 degree
# AFT_FSDK_FOC_150 = 0x8; # 150 degree
# AFT_FSDK_FOC_210 = 0x9; # 210 degree
# AFT_FSDK_FOC_240 = 0xa; # 240 degree
# AFT_FSDK_FOC_300 = 0xb; # 300 degree
# AFT_FSDK_FOC_330 = 0xc;  # 330 degree

if platform.system() == u'Windows':
    internalLibrary = CDLL(u'libarcsoft_fsdk_face_tracking.dll')
else:
	arcsoft_folder = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
	package_folder = os.path.dirname(arcsoft_folder)
	path = package_folder + '/lib/libarcsoft_fsdk_face_tracking.so'
	internalLibrary = CDLL(u'{}'.format(path))
    # internalLibrary = CDLL(u'/home/chengyuen93/Projects/ArcSoft_FaceRecognition/ArcSoft_FreeSDK_Demo/FR/python/libarcsoft_fsdk_face_detection.so')

AFT_FSDK_InitialFaceEngine = internalLibrary.AFT_FSDK_InitialFaceEngine
AFT_FSDK_UninitialFaceEngine = internalLibrary.AFT_FSDK_UninitialFaceEngine
AFT_FSDK_FaceFeatureDetect = internalLibrary.AFT_FSDK_FaceFeatureDetect
AFT_FSDK_GetVersion = internalLibrary.AFT_FSDK_GetVersion

AFT_FSDK_InitialFaceEngine.restype = c_long
AFT_FSDK_InitialFaceEngine.argtypes = (c_char_p,c_char_p,c_void_p,c_int32,POINTER(c_void_p),c_int32,c_int32,c_int32)
AFT_FSDK_UninitialFaceEngine.restype = c_long
AFT_FSDK_UninitialFaceEngine.argtypes = (c_void_p,)
AFT_FSDK_FaceFeatureDetect.restype = c_long
AFT_FSDK_FaceFeatureDetect.argtypes = (c_void_p,POINTER(ASVLOFFSCREEN),POINTER(POINTER(AFT_FSDK_FACERES)))
AFT_FSDK_GetVersion.restype = POINTER(AFT_FSDK_Version)
AFT_FSDK_GetVersion.argtypes =(c_void_p,)
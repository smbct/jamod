#pragma once

// dirty fix
#define linux 1
#define __x86_64__ 1

#include <XnCppWrapper.h>

#include <map>

#define SAMPLE_XML_PATH "Config/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


        
typedef std::map<XnSkeletonJoint, XnSkeletonJointTransformation> skeleton;


//------------------------------------------------
int kinect_init();

//------------------------------------------------
void kinect_update();

//------------------------------------------------
void kinect_getSkeleton(XnUserID player, skeleton& skeleton);

//------------------------------------------------
int kinect_ready();

//------------------------------------------------
void kinect_clean();




//------------------------------------------------
// Callback: New user was detected
void XN_CALLBACK_TYPE kinect_User_NewUser(xn::UserGenerator&, XnUserID, void*);

//------------------------------------------------
// Callback: An existing user was lost
void XN_CALLBACK_TYPE kinect_User_LostUser(xn::UserGenerator&, XnUserID, void*);

//------------------------------------------------
// Callback: Detected a pose
void XN_CALLBACK_TYPE kinect_UserPose_PoseDetected(xn::PoseDetectionCapability&, const XnChar* strPose, XnUserID nId, void*);

//------------------------------------------------
// Callback: Started calibration
void XN_CALLBACK_TYPE kinect_UserCalibration_CalibrationStart(xn::SkeletonCapability&, XnUserID, void*);

//------------------------------------------------
// Callback: Finished calibration
void XN_CALLBACK_TYPE kinect_UserCalibration_CalibrationComplete(xn::SkeletonCapability&, XnUserID, XnCalibrationStatus, void*);

//------------------------------------------------
void XN_CALLBACK_TYPE kinect_MyCalibrationInProgress(xn::SkeletonCapability&, XnUserID, XnCalibrationStatus, void*);

//------------------------------------------------
void XN_CALLBACK_TYPE kinect_MyPoseInProgress(xn::PoseDetectionCapability&, const XnChar*, XnUserID, XnPoseDetectionStatus, void*);

        


        

#include "kinect.h"

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include "qcommon/q_shared.h"

#include <iostream>

using namespace std;

XnBool g_bPause;

xn::Context g_context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Player g_Player;

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;

// processing variables
XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";



static constexpr const XnSkeletonJoint joints[] = {XN_SKEL_HEAD, XN_SKEL_NECK, XN_SKEL_TORSO, XN_SKEL_WAIST, 
XN_SKEL_LEFT_COLLAR, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP,
XN_SKEL_RIGHT_COLLAR, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW, 
XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP, 
XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_ANKLE, XN_SKEL_LEFT_FOOT, 
XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_ANKLE, XN_SKEL_RIGHT_FOOT};

static constexpr const std::pair<XnSkeletonJoint, XnSkeletonJoint> limbs[] = {
std::make_pair(XN_SKEL_HEAD, XN_SKEL_NECK),
std::make_pair(XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER),
std::make_pair(XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW),
std::make_pair(XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND),
std::make_pair(XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER),
std::make_pair(XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW),
std::make_pair(XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND),
std::make_pair(XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO),
std::make_pair(XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO),
std::make_pair(XN_SKEL_TORSO, XN_SKEL_LEFT_HIP  ),
std::make_pair(XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE),
std::make_pair(XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT),
std::make_pair(XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP),
std::make_pair(XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE),
std::make_pair(XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT),
std::make_pair(XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP)};

int kinect_status = 1;

//-------------------------------------------------------------------
int kinect_ready() {

	if(kinect_status == 0) {
		return 16;
	}

    XnUserID aUsers[15];
	XnUInt16 nUsers = 15;

    g_UserGenerator.GetUsers(aUsers, nUsers);

    if(nUsers > 0) {
        if(g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[0])) {
            return aUsers[0];
        } else {
            return 16;
        }
    } else {
        return 16;
    }

}

//-------------------------------------------------------------------
void kinect_getSkeleton(XnUserID player, skeleton& skeleton) {

	if(kinect_status == 0) {
		return;
	}

    skeleton.clear();
    for(auto& joint: joints) {

        XnSkeletonJointTransformation joint_transformation;

        g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(player, joint, joint_transformation);

        skeleton.insert(std::make_pair(joint, joint_transformation));
    }

    // cout << joint.position.X << " ; " << joint.position.Y << " ; " << joint.position.Z << endl; 
}

//-------------------------------------------------------------------
int kinect_init() {

	kinect_status = 0;

    g_bPause = false;

    g_bNeedPose = FALSE;

	xn::EnumerationErrors	errors;

	cout << "hello openni" << endl;

    XnStatus nRetVal = XN_STATUS_OK;

    

	// Create a context with default settings
	nRetVal = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);

	if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {

		XnChar strError[1024];
		errors.ToString(strError, 1024);
		Com_Printf("Kinect: %s\n", strError);
		return (nRetVal);

	} else if (nRetVal != XN_STATUS_OK) {

		Com_Printf("Kinect: Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);

	}       

    nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	
    if (nRetVal != XN_STATUS_OK) {
		Com_Printf("No depth generator found. Using a default one...");
		xn::MockDepthGenerator mockDepth;
		nRetVal = mockDepth.Create(g_context);
		CHECK_RC(nRetVal, "Create mock depth");

		// set some defaults
		XnMapOutputMode defaultMode;
		defaultMode.nXRes = 320;
		defaultMode.nYRes = 240;
		defaultMode.nFPS = 30;
		nRetVal = mockDepth.SetMapOutputMode(defaultMode);
		CHECK_RC(nRetVal, "set default mode");

		// set FOV
		XnFieldOfView fov;
		fov.fHFOV = 1.0225999419141749;
		fov.fVFOV = 0.79661567681716894;
		nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
		CHECK_RC(nRetVal, "set FOV");

		XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
		XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

		nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
		CHECK_RC(nRetVal, "set empty depth map");

		g_DepthGenerator = mockDepth;
	}

    nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		Com_Printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}


    // callbacks
	nRetVal = g_UserGenerator.RegisterUserCallbacks(kinect_User_NewUser, kinect_User_LostUser, NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(kinect_UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(kinect_UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			Com_Printf("Pose required, but not supported\n");
			return 1;
		}

		nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(kinect_UserPose_PoseDetected, NULL, hPoseDetected);
		CHECK_RC(nRetVal, "Register to Pose Detected");
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);

		nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(kinect_MyPoseInProgress, NULL, hPoseInProgress);
		CHECK_RC(nRetVal, "Register to pose in progress");
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(kinect_MyCalibrationInProgress, NULL, hCalibrationInProgress);
	CHECK_RC(nRetVal, "Register to calibration in progress");

	nRetVal = g_context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

    // cout << "end init" << endl;

	kinect_status = 1;

    return 0;

}

//-------------------------------------------------------------------
void kinect_clean() {

    cout << "Kinect: cleaning the kinect object" << endl;

    g_scriptNode.Release();
	g_DepthGenerator.Release();
	g_UserGenerator.Release();
	g_Player.Release();
	g_context.Release();

}


// Callback: New user was detected
//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_User_NewUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/) {


	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	Com_Printf("Kinect:%d New User %d\n", epochTime, nId);
	
    // New user found
	if (g_bNeedPose) {
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	} else {
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}

}


//-------------------------------------------------------------------
void kinect_update() {

	if(kinect_status == 0) {
		return;
	}

    xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);

    if (!g_bPause)
	{
		// Read next available data
		g_context.WaitOneUpdateAll(g_UserGenerator);
	}

	// Process the data
	// g_DepthGenerator.GetMetaData(depthMD);
	// g_UserGenerator.GetUserPixels(0, sceneMD);
	// DrawDepthMap(depthMD, sceneMD);

}


// Callback: An existing user was lost
//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/) {

	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	Com_Printf("Kinect:%d Lost user %d\n", epochTime, nId);	

}

// Callback: Detected a pose
//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_UserPose_PoseDetected(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* /*pCookie*/)
{

	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	Com_Printf("Kinect: %d Pose %s detected for user %d\n", epochTime, strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}


// Callback: Started calibration
//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	Com_Printf("Kinect:%d Calibration started for user %d\n", epochTime, nId);
}


// Callback: Finished calibration
//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_UserCalibration_CalibrationComplete(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* /*pCookie*/)
{


	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK) {
		// Calibration succeeded
		Com_Printf("Kinect:%d Calibration complete, start tracking user %d\n", epochTime, nId);		
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		// Calibration failed
		Com_Printf("Kinect:%d Calibration failed for user %d\n", epochTime, nId);
        if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT) {
            Com_Printf("Manual abort occured, stop attempting to calibrate!");
            return;
        }

		if (g_bNeedPose) {
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		} else {
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/) {

	m_Errors[id].first = calibrationError;
}

//-------------------------------------------------------------------
void XN_CALLBACK_TYPE kinect_MyPoseInProgress(xn::PoseDetectionCapability& /*capability*/, const XnChar* /*strPose*/, XnUserID id, XnPoseDetectionStatus poseError, void* /*pCookie*/) {
	m_Errors[id].second = poseError;
}


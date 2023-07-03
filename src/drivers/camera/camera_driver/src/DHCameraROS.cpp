#include "DHCameraROS.h"
//----------------------------------------------------------------------------------------------------
DHCameraROS::DHCameraROS()
	:mCamera(nullptr), mInitilized(false)
{
	_OpenCamera();
}
//-------------------------------------------------------------
DHCameraROS::~DHCameraROS()
{
	_CloseCamera();
}
//-------------------------------------------------------------
void DHCameraROS::_OpenCamera()
{

	// �����ʼ��
	int iOffsetX = 0;
	int iOffsetY = 0;
	// int iWidth = 1440;
	// int iHeight = 1080;
	// int iCamWidth = 1440;
	// int iCamHeight = 1080;

	int iWidth = 640;
	int iHeight = 480;
	int iCamWidth = 640;
	int iCamHeight = 480;

	std::string strSN = "EBJ22080036";

	int iFrameRate = 10;
	int iFrameBufferSize = 32;

	DHCamera2::OpenDriver();
	mCamera = new DHCamera2(iOffsetX, iOffsetY, iCamWidth, iCamHeight);
	mInitilized = mCamera->InitDevice(strSN);
}
//-------------------------------------------------------------
void DHCameraROS::_CloseCamera()
{
	if (mCamera)
	{
		if (mInitilized)
		{
			mCamera->CameraStop();
			mCamera->CameraClose();
		}

		delete mCamera;
		mCamera = nullptr;
	}
}
//----------------------------------------------------------------------------------------------------
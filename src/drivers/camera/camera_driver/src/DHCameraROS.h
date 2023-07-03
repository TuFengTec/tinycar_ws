
#ifndef __DH_CAMERA_ROS_H__
#define __DH_CAMERA_ROS_H__

#include "DHCamera2.h"
//----------------------------------------------------------------------------------------------------
class DHCameraROS
{
public:
	DHCameraROS();
	~DHCameraROS();

	inline bool isOpened() const
	{
		return mInitilized;
	}

	cv::Mat getImage() const
	{
		assert(mCamera);

		CameraFrameData cfd;
		if (mCamera->GetRawData(cfd))
		{
			return cfd.toCvMat();
		}

		return cv::Mat();
	}

	bool getImageData(CameraFrameData& cfd)
	{
		assert(mCamera);		
		return mCamera->GetRawData(cfd);
	}

protected:
	void _OpenCamera();
	void _CloseCamera();

private:
	DHCamera2* mCamera;
	bool       mInitilized;
};
//----------------------------------------------------------------------------------------------------
#endif
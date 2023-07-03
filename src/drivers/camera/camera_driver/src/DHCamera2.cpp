#include "DHCamera2.h"

#ifdef _VI_FLAG_USE_DAHENG_CAMERA

#include <GxIAPI.h>

//-------------------------------------------------------------
bool DHCamera2::ms_bInitLib = false; // 库初始化标志
int  DHCamera2::ms_iCamCount = 0;    // 启动的全局相机计数
//----------------------------------------------------------------------------------------------------
static void OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
//====================================================================================================
//----------------------------------------------------------------------------------------------------
bool DHCamera2::OpenDriver()
{
	//初始化全局库，只是初始化一次
	if (!ms_bInitLib)
	{
		GX_STATUS eStatus = GXInitLib();
		if (eStatus != GX_STATUS_SUCCESS)
			return false;

		ms_bInitLib = true;
	}

	if (!FillCamMap())
		return false;

	return true;
}
//-------------------------------------------------------------
void DHCamera2::CloseDriver()
{
	GXCloseLib();
}
//-------------------------------------------------------------
bool DHCamera2::FillCamMap()
{
	ICamController::ms_kSNIndexMap.clear();

	// 更新设备信息
	GX_STATUS eStatus = GX_STATUS_SUCCESS;
	uint32_t nDeviceNum = 0;
	eStatus = GXUpdateDeviceList(&nDeviceNum, 100);
	if (eStatus != GX_STATUS_SUCCESS)
		return false;

	if (nDeviceNum < 1)
		return false;

	// //获取所有设备的基础信息 -- 获取相机序列号
	GX_DEVICE_BASE_INFO *baseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
	size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
	eStatus = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		delete[]baseinfo;
		return false;
	}

	printf("--------------------------------------------------\n");
	printf("## Gige/USB Camera Count : %d\n", nDeviceNum);
	char acSN[64];
	for (int i = 0; i < (int)nDeviceNum; i++)
	{
		const char* pSN = baseinfo[i].szSN;
		memcpy(acSN, pSN, 32);

		std::string strSN(acSN);
		ICamController::ms_kSNIndexMap[strSN] = i;
		printf("## Enum Camera [%d] SN    : %s\n", i + 1, strSN.c_str());
	}
	delete[]baseinfo;

	return true;
}
//-------------------------------------------------------------
DHCamera2::DHCamera2(int iCamW, int iCamH)
	:ICamController(iCamW, iCamH)
{
	m_hDevice = nullptr;
	m_bIsSnaping = false;

#ifdef TI_FLAG_USE_COLOR_CAMERA
	m_bColorCamera = true;
#else
	m_bColorCamera = false;
#endif

	// 加载控制信息
	ICamController::DefaultExpo = 1500;
	ICamController::DefaultGain = 2;
	ICamController::DefaultFrameRate = 30;
	LoadDefualtParam();
	_LoadSettings();
}
//-------------------------------------------------------------
DHCamera2::DHCamera2(int iOffsetX, int iOffsetY, int iWidth, int iHeight)
	:ICamController(iOffsetX, iOffsetY, iWidth, iHeight)
{
	m_hDevice = nullptr;
	m_bIsSnaping = false;

#ifdef TI_FLAG_USE_COLOR_CAMERA
	m_bColorCamera = true;
#else
	m_bColorCamera = false;
#endif

	// 加载控制信息
	ICamController::DefaultExpo = 1500;
	ICamController::DefaultGain = 2;
	ICamController::DefaultFrameRate = 10;  //原来30
	LoadDefualtParam();
	_LoadSettings();
}
//-------------------------------------------------------------
DHCamera2::~DHCamera2(void)
{

}
//-------------------------------------------------------------
bool DHCamera2::InitDevice(std::string strSN)
{
	//  查找相机索引
	auto it = ms_kSNIndexMap.find(strSN);
	if (it == ms_kSNIndexMap.end())
	{
		if (ms_kSNIndexMap.size()>0)
		{
			it = ms_kSNIndexMap.begin();
			/* code */
		}
		else
		{
			printf("** FindCamera FAILED\n");
			return false;
		}

		//for (it = ms_kSNIndexMap.begin(); it != ms_kSNIndexMap.end(); ++it)
		//{
		//	std::cout << it->first << std::endl;
		//}
		//printf("** FindCamera FAILED\n");
		//return false;
	}
	
	m_strSN = strSN;
	m_iIndex = ms_kSNIndexMap[strSN];
	_UpdateNameByIndex();

	// 加载控制信息
	LoadDefualtParam();
	_LoadSettings();

	if (!CameraOpen())
	{
		printf("** CameraOpen FAILED\n");
		return false;
	}

	if (!_InitParameters())
	{
		printf("** _InitParameters FAILED\n");
		return false;
	}

	//! 设置图像大小--ROI
	//if (!SetImageROI(0, 0, m_iImageW, m_iImageH))
	if (!SetImageROI(m_iOffsetX, m_iOffsetY, m_iImageW, m_iImageH))
	{
		printf("** SetImageROI FAILED\n");
		return false;
	}

	// 设置触发模式--CONTINUE/SOFTWARE/LINE0
	if (!SetTriggerMode(m_iTriggerMode))
	{
		printf("** SetTriggerState FAILED\n");
		return false;
	}
	
	// 设置帧率
	if (!_SetFrameRate(m_iFPS))
	{
		printf("** SetFrameRate FAILED\n");
		return false;
	}

	// 增益范围范围
	int iGMin, iGMax;
	if (!GetGainRange(iGMin, iGMax))
	{
		printf("** GetGainRange FAILED\n");
		return false;
	}

	// 曝光范围
	int iEMin, iEMax;
	if (!GetExpoRange(iEMin, iEMax))
	{
		printf("** GetExpoRange FAILED\n");
		return false;
	}

	// 更新相机曝光时间和增益
	_SetExpo(m_iExpo);
	_SetGain(m_iGain);

	// 开始采集
	if (!CameraStart())
	{
		printf("** _Start FAILED\n");
		return false;
	}

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::DeinitDevice()
{
	_SaveSettings();

	// 停止采集
	if (m_bIsSnaping)
	{
		if (CameraStop())
			return false;
	}

	if (CameraClose())
		return false;

	//关闭设备库
	if (DHCamera2::ms_iCamCount == 0 && ms_bInitLib)
	{
		GX_STATUS eStatus = GXCloseLib();
		if (eStatus != GX_STATUS_SUCCESS)
		{
			assert( false );
			return false;
			//QMessageBox::warning( nullptr, "相机控制", "[E0021]:关闭设备库失败！" );
		}
		ms_bInitLib = false;
	}

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::CameraOpen()
{
	if (m_iIndex < 0)
		return false;

	// 初始化打开设备用参数,默认打开序号为1的设备
	char acCameraIndex[10];
	sprintf(acCameraIndex, "%d", m_iIndex + 1); // 设备编号从 1，2，3
	GX_OPEN_PARAM stOpenParam;
	stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
	stOpenParam.openMode = GX_OPEN_INDEX;
	//stOpenParam.pszContent = "1";
	stOpenParam.pszContent = acCameraIndex;

	// 打开指定相机失败
	GX_STATUS eStatus = GXOpenDevice(&stOpenParam, &m_hDevice);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//ASSERT( false );
		//DBWindow::Instance()->Puts("** 相机控制: 打开指定设备失败！\n");		
		return false;
	}
	DHCamera2::ms_iCamCount++;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::CameraClose()
{
	if (m_hDevice == nullptr)
		return false;

	//关闭设备
	GX_STATUS eStatus = GX_STATUS_SUCCESS;
	eStatus = GXCloseDevice(m_hDevice);

	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}
	DHCamera2::ms_iCamCount--;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::CameraStart()
{
	assert(!m_bIsSnaping);
	if (m_bIsSnaping)
		return true;

	//注册回调
	GX_STATUS eStatus = GXRegisterCaptureCallback(m_hDevice, this, OnFrameCallbackFun);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}

	//发开始采集命令
	eStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}

	m_bIsSnaping = true;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::CameraStop()
{
	if (!m_bIsSnaping)
		return true;

	//发送停止采集命令
	GX_STATUS eStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}

	//注销回调
	eStatus = GXUnregisterCaptureCallback(m_hDevice);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}

	m_bIsSnaping = false;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::SetImageROI(int iX, int iY, int iW, int iH)
{
	assert(iX >= 0 && iY >= 0);

	GX_STATUS eStatus = GX_STATUS_SUCCESS;

	//获取宽度(需在停止采集状态下设置)
	int64_t nValue = 0;
	//eStatus = GXGetInt(m_hDevice, GX_INT_WIDTH, &nValue);
	eStatus = GXGetInt(m_hDevice, GX_INT_SENSOR_WIDTH, &nValue);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera Width Failed！";
		return false;
	}
	int iCamW = (int)nValue;

	//获取高度(需在停止采集状态下设置)
	//eStatus = GXGetInt(m_hDevice, GX_INT_HEIGHT, &nValue);
	eStatus = GXGetInt(m_hDevice, GX_INT_SENSOR_HEIGHT, &nValue);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera Height Failed！";
		return false;
	}
	int iCamH = (int)nValue;

	if (iW > iCamW || iH > iCamH)
	{
		//LOG(INFO) << " Initialize Camera: Image Resolustion Error！";
		return false;
	}
	
	GX_STATUS status = GX_STATUS_SUCCESS;
	int64_t nOffsetX = (iCamW - iW) / 2;
	int64_t nOffsetY = (iCamH - iH) / 2;
	status = GXSetInt(m_hDevice, GX_INT_WIDTH, iW);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Set Camera Width Failed ！" << iW;
		return false;
	}

	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, iH);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera Height Failed ！" << iH;
		return false;
	}

	status = GXSetInt(m_hDevice, GX_INT_OFFSET_X, nOffsetX);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera OffsetX Failed ！" << nOffsetX;
		return false;
	}

	status = GXSetInt(m_hDevice, GX_INT_OFFSET_Y, nOffsetY);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera OffsetY Failed ！" << nOffsetY;
		return false;
	}

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::GetImageROI(int& iX, int& iY, int& iW, int& iH)
{
	GX_STATUS eStatus = GX_STATUS_SUCCESS;
	iX = -1;
	iX = -1;
	iW = -1;
	iH = -1;

	int64_t nWidth = 0;
	//eStatus = GXGetInt(m_hDevice, GX_INT_WIDTH, &nValue);
	eStatus = GXGetInt(m_hDevice, GX_INT_SENSOR_WIDTH, &nWidth);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera Width Failed！" << nWidth;
		return false;
	}

	int64_t nHeight = 0;
	eStatus = GXGetInt(m_hDevice, GX_INT_HEIGHT, &nHeight);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera Height Failed ！" << nHeight;
		return false;
	}

	int64_t nOffsetX = 0;
	eStatus = GXGetInt(m_hDevice, GX_INT_OFFSET_X, &nOffsetX);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera OffsetX Failed ！" << nOffsetX;
		return false;
	}

	int64_t nOffsetY = 0;
	eStatus = GXGetInt(m_hDevice, GX_INT_OFFSET_Y, &nOffsetY);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Get Camera OffsetY Failed ！" << nOffsetY;
		return false;
	}

	iX = (int)nOffsetX;
	iY = (int)nOffsetY;
	iW = (int)nWidth;
	iH = (int)nHeight;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::SetTriggerMode(int iTriggerMode)
{
	if (iTriggerMode == TM_TRIGGER_CONTINUE)
	{
		// 设置触发模式--关闭
		//GX_STATUS eStatus = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
		GX_STATUS eStatus = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
		if (eStatus != GX_STATUS_SUCCESS)
		{
			//LOG(INFO) << " Initialize Camera: Set Trigger Mode Failed！";
			return false;
		}

		eStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
		if (eStatus != GX_STATUS_SUCCESS)
		{
			//LOG(INFO) << " Initialize Camera: Set CONTINUOUS Mode Failed！";
			return false;
		}

		return true;
	}
	else
	{
		GX_STATUS eStatus = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
		if (eStatus != GX_STATUS_SUCCESS)
		{
			//LOG(INFO) << " Initialize Camera: Set Trigger Mode Failed！";
			return false;
		}
	}

	int64_t nMode = 0;
	if (iTriggerMode == TM_TRIGGER_SOFTWARE) nMode = GX_TRIGGER_SOURCE_SOFTWARE;
	if (iTriggerMode == TM_TRIGGER_LINE0) nMode = GX_TRIGGER_SOURCE_LINE0;
	if (iTriggerMode == TM_TRIGGER_LINE1) nMode = GX_TRIGGER_SOURCE_LINE1;

	GX_STATUS eStatus = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_SOURCE, nMode);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Set Trigger Source Failed！";
		return false;
	}

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::IsRunning()
{
	return m_bIsSnaping;
}
//-------------------------------------------------------------
bool DHCamera2::_SetFrameRate(int iFrameRate)
{
	if (m_hDevice == nullptr)
		return false;

	GX_STATUS eStatus = GX_STATUS_SUCCESS;
	//使能采集帧率调节模式--启用或关闭帧率控制
	//eStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
	
	//设置采集帧率,假设设置为10.0，用户按照实际需求设置此值
	eStatus = GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, double(iFrameRate));

	return (eStatus == GX_STATUS_SUCCESS);
}
//-------------------------------------------------------------
bool DHCamera2::_SetExpo(int iExpo)
{
	double dVal = iExpo * 10; // 讲曝光时间从[4-10000] 调整到 [40-100000]

	//GX_STATUS eStatus = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, dVal);
	GX_STATUS eStatus = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, dVal);
	return (eStatus == GX_STATUS_SUCCESS);
}
//-------------------------------------------------------------
bool DHCamera2::_SetGain(int iGain)
{
	//GX_STATUS eStatus = GXSetInt(m_hDevice, GX_INT_GAIN, m_iGain);
	GX_STATUS eStatus = GXSetFloat(m_hDevice, GX_FLOAT_GAIN, (double)iGain);
	return (eStatus == GX_STATUS_SUCCESS);
}
//-------------------------------------------------------------
bool DHCamera2::GetExpoRange(int& iMin, int& iMax)
{
	GX_FLOAT_RANGE stFloatRange;

	//曝光值滑动条	
	//GX_STATUS eStatus = GXGetFloatRange( m_hDevice, GX_FLOAT_EXPOSURE_TIME, &stFloatRange);
	GX_STATUS eStatus = GXGetFloatRange(m_hDevice, GX_FLOAT_EXPOSURE_TIME, &stFloatRange);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		return false;
	}
	//pSliderCtrl->SetRange((int32_t)stFloatRange.dMin, (int32_t)stFloatRange.dMax, true);
	int iMinVal = (int)stFloatRange.dMin;
	int iMaxVal = (int)stFloatRange.dMax;

	// 将曝光时间从[33-100000]调整到 [4-10000]
	iMin = iMinVal / 10;
	iMax = iMaxVal / 10;
	if (iMax > 10000)
	{
		iMax = 10000;
	}

	if (iMinVal % 10 > 0) iMin++;
	//if ( iMaxVal%10 > 0 ) iMax--;

	return true;
}
//-------------------------------------------------------------
bool DHCamera2::GetGainRange(int& iMin, int& iMax)
{
	GX_FLOAT_RANGE stFloatRange;

	//曝光值滑动条	
	//GX_STATUS eStatus = GXGetIntRange( m_hDevice, GX_INT_GAIN, &stIntRange);
	GX_STATUS eStatus = GXGetFloatRange(m_hDevice, GX_FLOAT_GAIN, &stFloatRange);
	assert(eStatus == GX_STATUS_SUCCESS);
	if (eStatus != GX_STATUS_SUCCESS)
		return false;

	iMin = (int32_t)stFloatRange.dMin;
	iMax = (int32_t)stFloatRange.dMax;
	if (iMax>12)
	{
		iMax = 12;
	}

	return true;
}
//-------------------------------------------------------------
//! 相机CallBack 函数
static void OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
	ICamController* pkHCam = (ICamController*)(pFrame->pUserParam);
	if (pkHCam && pFrame->status == 0)
	{
		pkHCam->InternalUpdateFrame((uchar*)pFrame->pImgBuf);

		//if (pkHCam->IsColorCamera())
		//{
		//	// 直接将彩色相机数据转换为单色数据
		//	uchar* pRawBuffer = pkHCam->GetRawBuffer();
		//	uchar* pRgbBuffer = pkHCam->GetRgbBuffer();
		//	int iImageW = pkHCam->GetWidth();
		//	int iImageH = pkHCam->GetHeight();
		//	{
		//		//////BAYERRG = 1,   ///< 第一行以RG开始
		//		//////BAYERGB = 2,   ///< 第一行以GB开始
		//		//////BAYERGR = 3,   ///< 第一行以GR开始
		//		//////BAYERBG = 4    ///< 第一行以BG开始
		//		//DX_PIXEL_COLOR_FILTER iFilter = BAYERGR;
		//		//DxRaw8toRGB24((uchar*)pFrame->pImgBuf, pRgbBuffer, iImageW, iImageH, RAW2RGB_NEIGHBOUR, iFilter, FALSE);
		//		//rgb2gray(pRawBuffer, pRgbBuffer, iImageW*iImageH);
		//	}
		//	pkHCam->InternalUpdateFrame(nullptr);
		//}
		//else
		//{
		//	// 单色相机
		//	pkHCam->InternalUpdateFrame((uchar*)pFrame->pImgBuf);
		//}
		// 单色相机
	}
}
//-------------------------------------------------------------
bool DHCamera2::SoftwareTrigger()
{
	if (m_hDevice)
	{
		GX_STATUS status = GXSendCommand(m_hDevice, GX_COMMAND_TRIGGER_SOFTWARE); //软触发命令

		if (status != GX_STATUS_SUCCESS)
		{
			//LOG(INFO) << "##Camera " << m_strSN << " -- Soft-Trigger Error Code: " << status;
		}

		//HV_VERIFY(status);
		return status == GX_STATUS_SUCCESS;
	}

	return false;
}
//-------------------------------------------------------------
bool DHCamera2::_InitParameters()
{
	GX_STATUS eStatus = GX_STATUS_SUCCESS;

	// 采集模式--连续
	//eStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
	eStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		//LOG(INFO) << " Initialize Camera: Set CONTINUOUS Mode Failed！";
		return false;
	}

	/*
	GX_STATUS status = GX_STATUS_SUCCESS;
	//使能采集帧率调节模式
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
	//设置采集帧率,假设设置为10.0，用户按照实际需求设置此值
	status = GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 10.0);
	*/

	/*
	GX_STATUS status = GX_STATUS_SUCCESS;
	//使能降噪
	GX_NOISE_REDUCTION_MODE_ENTRY nValue;
	nValue = GX_NOISE_REDUCTION_MODE_ON;
	status = GXSetEnum(hDevice, GX_ENUM_NOISE_REDUCTION_MODE, nValue);
	//获取降噪的值[1-4]
	double dNoiseReductionParam = 0.0;
	status = GXGetFloat(hDevice, GX_FLOAT_NOISE_REDUCTION,
		&dNoiseReductionParam);
	*/



	//// 设置采集速度 --12
	//GX_INT_RANGE stIntRange;
	////eStatus = GXGetIntRange(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, &stIntRange);
	//eStatus = GXGetIntRange(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, &stIntRange);
	//if (eStatus != GX_STATUS_SUCCESS)
	//{
	//	assert( false );
	//	//QMessageBox::warning( nullptr, "相机控制", "[E0008]:查询设备采集速度失败！" );
	//	return false;
	//}

	// 使用默认的采集速度--开启之后可以设置自己的速度
	eStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		assert(false);
		return false;
	}
	// 或者 GX_ACQUISITION_FRAME_RATE_MODE_ON,设置自己的采集帧率
	//GX_STATUS eStatus = GX_STATUS_SUCCESS;
	//eStatus = GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 36.0000);
	//if (eStatus != GX_STATUS_SUCCESS)
	//{
	//	assert(false);
	//	return false;
	//}

	// 禁止自动曝光
	eStatus = GXSetEnum(m_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		return false;
	}
	// 自动自动增益
	eStatus = GXSetEnum(m_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		return false;
	}

	//eStatus = GXSetEnum(m_hDevice, GX_ENUM_GAIN_MODE, GX_GAIN_AUTO_OFF);
	//eStatus = GXSetEnum(m_hDevice, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);

	bool m_bColorFilter = false;
	eStatus = GXIsImplemented(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_bColorFilter);
	if (m_bColorFilter)
	{
		eStatus = GXGetEnum(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_nPixelColorFilter);
	}

	return true;
}
//----------------------------------------------------------------------------------------------------
#endif
/********************************************************************
* Copyright(c) 2014-2020,  Virtual Islands Co.Ltd
* All rights reversed
*
* 文件名称: DHCamera2.h
* 摘    要: 大恒水星相机控制
*
* 当前版本: 1.0
* 创建日期:	2014/04/25
* 作    者:	LG
* 项目名称:
* 操作系统: MS Windows 7/8 64bit
* 编译平台: MS VS .Net 2013 Professional + SP3
*
* 修改编号-|-编者-|--日期--|----------------- 注释 -----------------
*
*********************************************************************/
#ifndef __DH_MERCURY_CAMERA_H__
#define __DH_MERCURY_CAMERA_H__

#define _VI_FLAG_USE_DAHENG_CAMERA
#ifdef _VI_FLAG_USE_DAHENG_CAMERA

#include "ICamController.h"
//----------------------------------------------------------------------------------------------------
class DHCamera2 : public ICamController
{
public:
	DHCamera2(int iCamW, int iCamH);
	DHCamera2(int iOffsetX, int iOffsetY, int iWidth, int iHeight);
	~DHCamera2(void);

public:
	//! 加载相机驱动函数
	static bool OpenDriver();
	static void CloseDriver();
	static bool FillCamMap();

public:
	virtual int  GetCamType() { return CT_DHMER; };
	virtual bool InitDevice(std::string strSN);
	virtual bool DeinitDevice();
	virtual bool IsRunning();

	virtual bool CameraOpen();
	virtual bool CameraClose();
	virtual bool CameraStart();
	virtual bool CameraStop();

	virtual bool _SetFrameRate(int iFrameRate);
	virtual bool _SetExpo(int iExpo);
	virtual bool _SetGain(int iGain);
	virtual bool GetExpoRange(int& iMin, int& iMax);
	virtual bool GetGainRange(int& iMin, int& iMax);

	virtual bool SetImageROI(int iX, int iY, int iW, int iH);
	virtual bool GetImageROI(int& iX, int& iY, int& iW, int& iH);

	virtual bool SetTriggerMode(int iTrigerMode);
	virtual bool SoftwareTrigger();

protected:
	//! 初始化相机其他参数
	bool _InitParameters();

protected:
	void*   m_hDevice;             // 设备句柄
	bool    m_bIsSnaping;          // 相机启动状态

	bool    m_bColorCamera;
	int64_t m_nPixelColorFilter;

protected:
	static bool ms_bInitLib;      // 库初始化标志
	static int  ms_iCamCount;     // 打开的设备计数
};
//----------------------------------------------------------------------------------------------------
#endif

#endif
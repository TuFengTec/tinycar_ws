/********************************************************************
* Copyright(c) 2014-2020,  Virtual Islands Co.Ltd
* All rights reversed
*
* 文件名称: ICamController.h
* 摘    要: 相机控制抽象接口类
*
* 当前版本: 1.0
* 创建日期: 2016/10/18
* 作    者: LG
* 项目名称:
* 操作系统: MS Windows 7/8 64bit
* 编译平台: MS VS .Net 2013 Professional + SP3
*
* 修改编号-|-编者-|--日期--|----------------- 注释 -----------------
*
*********************************************************************/
#ifndef __I_CAM_CONTROLLER_H__
#define __I_CAM_CONTROLLER_H__

//#include <QMutex>
//#include <QTSemaphore>
#include "TFrameQueue.h" 

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable
#include <map>
#include <vector>
#include <string>
#include <assert.h>

#ifndef uchar
typedef unsigned char uchar;
#endif
//----------------------------------------------------------------------------------------------------
class ICamController
{
protected:
	ICamController(int iCamW, int iCamH);
	ICamController(int iOffsetX, int iOffsetY, int iWidth, int iHeight);


public:
	virtual ~ICamController(void);

	//! 禁止复制构造
	ICamController(const ICamController&) = delete;
	const ICamController& operator=(const ICamController&) = delete;

	enum CameraType
	{
		CT_VVCAM = 1000,
		CT_DH1351 = 2001,
		CT_DHMER = 2002,
		CT_MVCAM = 3001,
		CT_PGCAM = 4001,
		CT_JAICAM = 5001,
		CT_HIKCAM = 5101,
		CT_HUACAM = 6001,
		CT_MATROXCAM = 6002,
	};

	enum TriggerModeState
	{
		TM_TRIGGER_CONTINUE = 0,
		TM_TRIGGER_SOFTWARE = 1,
		TM_TRIGGER_LINE0 = 2,
		TM_TRIGGER_LINE1 = 3,
	};

public:
	//! 虚接口定义
	virtual int  GetCamType() = 0;
	virtual bool InitDevice(std::string strSN) = 0;
	virtual bool DeinitDevice() = 0;
	virtual bool IsRunning() = 0;
	
	virtual bool CameraOpen() = 0;
	virtual bool CameraClose() = 0;
	virtual bool CameraStart() = 0;
	virtual bool CameraStop() = 0;

	virtual bool _SetFrameRate(int iFrameRate) = 0;
	virtual bool _SetExpo(int iExpo) = 0;
	virtual bool _SetGain(int iGain) = 0;
	virtual bool GetExpoRange(int& iMin, int& iMax) = 0;
	virtual bool GetGainRange(int& iMin, int& iMax) = 0;

	virtual bool SetImageROI(int iX, int iY, int iW, int iH) = 0;
	virtual bool GetImageROI(int& iX, int& iY, int& iW, int& iH) = 0;

	virtual bool SetTriggerMode(int iTrigerState) = 0;
	virtual bool SoftwareTrigger() = 0;

public:
	//! QT格式软触发操作
	bool SoftTrigger(TSemaphore* pSem);
	bool SoftTrggerFinished();

	//! 内部更新一帧数据，禁止调用此函数
	void InternalUpdateFrame(uchar* pRawBuffer);

	//! 复制最后一帧图像数据
	bool CopyRawDataTo(uchar* pImage);
	bool GetRawData(CameraFrameData& cfd);

public:
	std::string GetName()    const  { return m_strName; }
	inline int GetIndex()    const  { return m_iIndex; }

	inline int GetImgWidth() const  { return m_iImageW; }
	inline int GetImgHeight()const	{ return m_iImageH; }
	inline int GetImgSize()  const	{ return m_iImageW* m_iImageH; }

	bool   SetFarmeRate(int iFrameRate);
	inline int GetFarmeRate() const { return m_iFPS; }
	bool   SetGain(int iGain);
	bool   SetExpo(int iExpo);
	inline int GetExpo() const  { return m_iExpo; }
	inline int GetGain() const  { return m_iGain; }

	//! 获取线程队列--线程安全
	//inline CameraFrameQ& GetFrameList() { return m_kFrameQueue;  }
	//inline const CameraFrameQ& GetFrameList() const { return m_kFrameQueue; }
	inline int  GetFrameBufferSize() const { return m_iFrameQueueSize; };
	inline void SetFrameBufferSize( int iMaxNum ) { m_iFrameQueueSize = iMaxNum; };

protected:
	//! 根据索引设置相机名称: Camera_%d
	void _UpdateNameByIndex();

	//! 注册表键值保存和读入
	void _LoadSettings();
	void _SaveSettings();

	//!  设置参数--恢复默认值
	void LoadDefualtParam();

public:
	//! 设备驱动初始化时候填充此列表
	static std::map<std::string, int> ms_kSNIndexMap;

	//! 默认帧率增益和曝光时间
	static int DefaultFrameRate;
	static int DefaultGain;
	static int DefaultExpo;

protected:
	int          m_iIndex;			 // 相机编号
	std::string  m_strName;			 // 相机名字	
	std::string  m_strSN;			 // 相机序列号
	int          m_iOffsetX;		 // 相机ROI位置X
	int          m_iOffsetY;		 // 相机ROI位置Y
	int          m_iImageW;			 // 相机图像宽度
	int          m_iImageH;			 // 相机图像高度
	int          m_iFPS;	    	 // 相机帧率
	int          m_iGain;		     // 相机增益系数
	int          m_iExpo;		     // 相机曝光系数
	int          m_iTriggerMode;     // 相机触发模式
	int          m_iFrameID;		 // 相机图像帧号
	CameraFrameQ m_kFrameQueue;      // 图像队列 
	int          m_iFrameQueueSize;  // 图像队列中缓存最多的图像数量

	std::mutex       m_kTriggerMutex;    // 软触发互斥锁
	TSemaphore*  m_pkSoftTriggerSem; // 软触发信号灯
};
//----------------------------------------------------------------------------------------------------
#endif

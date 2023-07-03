#include "ICamController.h"
//----------------------------------------------------------------------------------------------------
std::map<std::string, int> ICamController::ms_kSNIndexMap;
int ICamController::DefaultFrameRate = 30;
int ICamController::DefaultGain = 5;
int ICamController::DefaultExpo = 4000;
//----------------------------------------------------------------------------------------------------
ICamController::ICamController(int iCamW, int iCamH)
{
	m_iIndex = -1;
	m_strName = "NULL_NAME";
	m_strSN = "NULL_SN";
	m_iFrameID = 0;

	m_iImageW = iCamW;
	m_iImageH = iCamH;
	m_iFPS = 30;
	m_iGain = ICamController::DefaultGain;
	m_iExpo = ICamController::DefaultExpo;
	m_iTriggerMode = TM_TRIGGER_CONTINUE;
	//m_iFrameQueueSize = 16;
	m_iFrameQueueSize = 16;

	//m_pkSoftTriggerSem = nullptr;
}
//-------------------------------------------------------------
ICamController::ICamController(int iOffsetX, int iOffsetY, int iWidth, int iHeight)
{
	m_iIndex = -1;
	m_strName = "NULL_NAME";
	m_strSN = "NULL_SN";
	m_iFrameID = 0;

	m_iOffsetX = iOffsetX;
	m_iOffsetY = iOffsetY;
	m_iImageW = iWidth;
	m_iImageH = iHeight;
	m_iFPS = 30;
	m_iGain = ICamController::DefaultGain;
	m_iExpo = ICamController::DefaultExpo;
	m_iTriggerMode = TM_TRIGGER_CONTINUE;
	m_iFrameQueueSize = 16;

	//m_pkSoftTriggerSem = nullptr;
}
//-------------------------------------------------------------
ICamController::~ICamController(void)
{
	int iStop = 0;
}
//-------------------------------------------------------------
bool ICamController::SoftTrigger(TSemaphore* pSem)
{
	if (pSem == nullptr)
	{
		m_kTriggerMutex.lock();
		m_pkSoftTriggerSem = nullptr;
		m_kTriggerMutex.unlock();

		return true;
	}

	m_kTriggerMutex.lock();
	{
		m_pkSoftTriggerSem = pSem;
	}
	m_kTriggerMutex.unlock();

	//! 不同相机实现自己的内部触发
	bool bScucess = SoftwareTrigger();

	return bScucess;
}
//-------------------------------------------------------------
bool ICamController::SoftTrggerFinished()
{
	bool bFinished = false;;
	m_kTriggerMutex.lock();
	{
		bFinished = m_pkSoftTriggerSem == nullptr;
	}
	m_kTriggerMutex.unlock();

	return bFinished;
}
//-------------------------------------------------------------
bool ICamController::CopyRawDataTo(uchar* pImage)
{
	if (m_kFrameQueue.size() > 0)
	{
		CameraFrameData cfd;
		m_kFrameQueue.get(cfd);
		memcpy(pImage, &cfd.ImageData[0], cfd.ImageH*cfd.ImageW * sizeof(uchar));

		return true;
	}

	return false;
}
//-------------------------------------------------------------
bool ICamController::GetRawData(CameraFrameData& cfd)
{
	if (m_kFrameQueue.size() > 0)
	{
		m_kFrameQueue.get(cfd);
		return true;
	}

	return false;
}
//----------------------------------------------------------------------------------------------------
void ICamController::InternalUpdateFrame(uchar* pRawBuffer)
{
	m_iFrameID++;

	while (m_kFrameQueue.size() > m_iFrameQueueSize)
	{
		m_kFrameQueue.pop_front();
	}
	m_kFrameQueue.push_back(CameraFrameData(m_iImageW, m_iImageH, m_iFrameID, pRawBuffer));
}
//-------------------------------------------------------------
void ICamController::_UpdateNameByIndex()
{
	char acName[28];
	sprintf(acName, "Camera_%d", m_iIndex);
	m_strName = acName;
}
//-------------------------------------------------------------
void ICamController::_SaveSettings()
{
	//QString strGroup;
	//strGroup.sprintf("Camera_%s", m_strSN.c_str());

	//TParam()->WriteProfile(strGroup, "Gain", m_iGain);
	//TParam()->WriteProfile(strGroup, "Expo", m_iExpo);
}
//------------------------------------------------------------
void ICamController::_LoadSettings()
{
	//QString strGroup;
	//strGroup.sprintf("Camera_%s", m_strSN.c_str());

	//int iGain = TParam()->GetProfileInt(strGroup, "Gain", -1);
	//double iExpo = TParam()->GetProfileDouble(strGroup, "Expo", -1);
	//if (iGain > 0)  m_iGain = iGain;
	//if (iExpo > 0) m_iExpo = iExpo;
}
//------------------------------------------------------------
void ICamController::LoadDefualtParam()
{
	m_iFPS = DefaultFrameRate;
	m_iGain = DefaultGain;
	m_iExpo = DefaultExpo;
}
//-------------------------------------------------------------
bool ICamController::SetFarmeRate(int iFrameRate)
{
	m_iFPS = iFrameRate;
	return _SetFrameRate(iFrameRate);
}
//-------------------------------------------------------------
bool ICamController::SetGain(int iGain)
{
	m_iGain = iGain;
	return _SetGain(m_iGain);
}
//-------------------------------------------------------------
bool ICamController::SetExpo(int iExpo)
{
	m_iExpo = iExpo;
	return _SetExpo(iExpo);
}
//----------------------------------------------------------------------------------------------------
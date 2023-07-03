/********************************************************************
* Copyright(c) 2014-2020,  Virtual Islands Co.Ltd
* All rights reversed
*
* 文件名称: TFrameQueue.h
* 摘    要: 线程安全队列
*
* 当前版本: 1.0
* 创建日期: 2021/11/12
* 作    者:
* 项目名称:
* 操作系统: MS Windows 10 64bit
* 编译平台: MS VS 2017 Professional
*
* 修改编号-|-编者-|--日期--|----------------- 注释 -----------------
*
*********************************************************************/
#ifndef __CAMERA_FRAME_QUEUE_H__
#define __CAMERA_FRAME_QUEUE_H__
//----------------------------------------------------------------------------------------------------
#include <list>
#include <vector>

#include <iostream>  
#include <vector>  
#include <thread>  
#include <mutex>  
#include <chrono>  
#include <condition_variable> 

// Opencv 3.4 headers.
#pragma warning( push )  
#pragma warning( disable: 4819)
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2\imgproc\types_c.h>
//#include <opencv2\calib3d\calib3d.hpp>
#pragma warning( pop )

#ifndef uchar
typedef unsigned char uchar;
#endif // !uchar

//----------------------------------------------------------------------------------------------------
class TSemaphore 
{
public:
	TSemaphore(int n = 1) : _count{ n}, _wakeups{ 0 } {}
	
	void acquire(int n = 1)
	{
		std::unique_lock<std::mutex> lock(_mutex);
		_count -= n;
		if (_count < 0)
		{ // count is not enough ?
			_condition.wait(lock, [&]()->bool { return _wakeups > 0; }); // suspend and wait ...
			_wakeups -= n; // ok, me wakeup !
		}
	}
	bool tryAcquire(int n = 1)
	{
		std::unique_lock<std::mutex> lock(_mutex);
		_count -= n;
		if (_count - n < 0)
			return false;

		_count -= n;
		_wakeups -= n;

		return true;
	}
	bool tryAcquire(int n, int timeout)
	{
		std::unique_lock<std::mutex> lock(_mutex);
		_count -= n;
		if (_count - n < 0)
		{
			if (_condition.wait_for(lock, std::chrono::seconds(timeout)) == std::cv_status::timeout)
				return false;
		}

		_count -= n;
		_wakeups -= n;

		return true;
	}

	void release(int n = 1)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_count += n;
		if (_count <= 0)
		{ // have some thread suspended ?
			_wakeups += n;
			_condition.notify_all(); // notify one !
		}
	}

	int available()// const
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _count;
	}

private:
	int _count;
	int _wakeups;

	std::mutex               _mutex;
	std::condition_variable _condition;
};

//#define TFRAME_QUEUE_DEBUG 1
//----------------------------------------------------------------------------------------------------
//! 队列模板<Thread-Safe>
template<typename T>
class TFrameQueue 
{
public:
	//！ 添加数据到队尾
	void push_back(T const& msg)
	{
		std::lock_guard<std::mutex> g(_mutex);
		
		_list.push_back(msg);
		_sem.release();

#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [push_back  ]: %d - %d\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
	}

	//！ 移除队头数据
	void pop_front()
	{
		std::lock_guard<std::mutex> g(_mutex);
		if(!_list.empty())
		{
			_sem.acquire();
			_list.pop_front();
		}
		
#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [pop_front  ]: %d - %d\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
	}

	//！ 获取队头数据，并从队列移除队头数据;若无数据，一直等待
	void get(T& msg)
	{
		_mutex.lock();
		while (_list.empty())
		{
			_mutex.unlock();
			_sem.acquire();
			_mutex.lock();
		}

		msg = _list.front();
		_list.pop_front();
		_mutex.unlock();

#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [get        ]: %d - %d\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
	}

	//！ 获取队头数据，并从队列移除队头数据，有等待时间
	bool get(T& msg, int timeout)
	{
		_mutex.lock();
		if ( _list.size()>0)
		{
			msg = _list.front();
			_list.pop_front();
			_sem.acquire();
		}
		else
		{
			_mutex.unlock();
			if (_sem.tryAcquire(1, timeout))
			{
				_mutex.lock();
				msg = _list.front();
				//_list.pop_front();
				_sem.release(1); // 链表不移除所以+1
			}
			else
			{
#ifdef TFRAME_QUEUE_DEBUG
				QMutexLocker g(_mutex);
				int ss = _sem.available();
				int sz = _list.size();
				assert(ss == sz);
				printf("Quene Size [get-timeout]: %d - %d -- Failed!\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
				return false;
			}
		}
		_mutex.unlock();

#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [get-timeout]: %d - %d -- OK!\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG

		return true;
	}

	//！ 获取队头数据，并从队列移除队头数据，有等待时间
	bool query(T& msg, int timeout)
	{
		_mutex.lock();
		if (_list.size() > 0)
		{
			msg = _list.front();
			//_list.pop_front();
			//_sem.acquire();
		}
		else
		{
			_mutex.unlock();
			if (_sem.tryAcquire(1, timeout))
			{
				_mutex.lock();
				msg = _list.front();
				//_list.pop_front();
				_sem.release();
			}
			else
			{
#ifdef TFRAME_QUEUE_DEBUG
				QMutexLocker g(&_mutex);
				int ss = _sem.available();
				int sz = _list.size();
				assert(ss == sz);
				printf("Quene Size [get-timeout]: %d - %d -- Failed!\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
				return false;
			}
		}
		_mutex.unlock();

#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [get-timeout]: %d - %d -- OK!\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG

		return true;
	}

	//！ 获取队头数据，并从队列移除队头数据，有等待时间
	bool query_back(T& msg, int timeout)
	{
		_mutex.lock();
		if (_list.size() > 0)
		{
			msg = _list.back();
		}
		else
		{
			_mutex.unlock();
			if (_sem.tryAcquire(1, timeout))
			{
				_mutex.lock();
				msg = _list.back();
				//_list.pop_front();
				_sem.release();
			}
			else
			{
				return false;
			}
		}
		_mutex.unlock();
		return true;
	}


	//！当前队列大小
	int size()
	{
		std::lock_guard<std::mutex> g(_mutex);

#ifdef TFRAME_QUEUE_DEBUG
		int ss = _sem.available();
		int sz = _list.size();
		assert(ss == sz);
		printf("Quene Size [size       ]: %d - %d\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG

		return (int)_list.size();
	}

	//！清空队列
	void clear()
	{
		std::lock_guard<std::mutex> g(_mutex);
		_list.clear();

		int n = _sem.available();
		if ( n>0)
		{
			_sem.release(n);
		}

#ifdef TFRAME_QUEUE_DEBUG
		int sz = _list.size();
		int ss = _sem.available();
		assert(ss == sz);
		printf("Quene Size [clear      ]: %d - %d\n", ss, sz);
#endif // TFRAME_QUEUE_DEBUG
	}

private:
	TSemaphore    _sem;
	std::mutex   _mutex;
	std::list<T> _list;
};
//----------------------------------------------------------------------------------------------------
//! 相机帧数据
struct CameraFrameData
{
	int ImageW;					  // 图像宽度
	int ImageH;					  // 图像高度
	int FrameID;                  // 帧号
	//double Timestamp;
	double Timestamp;
	std::vector<uchar> ImageData; // 图像数据

	CameraFrameData()
		:ImageW(0), ImageH(0), FrameID(0)
	{
		Timestamp = (double)clock();
	}

	CameraFrameData(int iW, int iH, int iFrameID, uchar* pData)
	{
		ImageW = iW;
		ImageH = iH;
		FrameID = iFrameID;
		Timestamp = (double)clock();
		ImageData.resize(iW*iH);
		memcpy(&ImageData[0], pData, sizeof(uchar)*iW*iH);
	}

	CameraFrameData(const CameraFrameData& other)
	{
		ImageW = other.ImageW;
		ImageH = other.ImageH;
		FrameID = other.FrameID;
		Timestamp = other.Timestamp;
		ImageData = other.ImageData;
	}

	CameraFrameData& operator= (const CameraFrameData& other)
	{
		if ( this == &other)
			return *this;

		ImageW = other.ImageW;
		ImageH = other.ImageH;
		FrameID = other.FrameID;
		Timestamp = other.Timestamp;
		ImageData = other.ImageData;

		return *this;
	}

	cv::Mat toCvMat()
	{
		return cv::Mat(ImageH, ImageW, CV_8UC1, &ImageData[0]);
	}
};
//----------------------------------------------------------------------------------------------------
typedef TFrameQueue<CameraFrameData> CameraFrameQ;
//----------------------------------------------------------------------------------------------------
#endif

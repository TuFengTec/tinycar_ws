#pragma once
#ifndef __PAVO2_NETCONN_H__
#define __PAVO2_NETCONN_H__
#include <string>
#include <boost/asio.hpp>
#include <boost/function.hpp>
using std::string;

#include "pavo2_common.h"

namespace pavo2
{
	class NetConnect {
	public:
		virtual ~NetConnect();
		virtual bool open(string lidar_ip, uint16_t lidar_port) = 0;
        virtual bool open(string lidar_ip, uint16_t lidar_port, uint32_t timeout) = 0;
        virtual bool isOpen() = 0;
		virtual bool close() = 0;
		virtual bool readPoint(unsigned char * content, uint16_t &size, uint16_t time_out = 0) = 0;
		virtual bool readCMD(PAVO2_CMD_INDEX cmd,unsigned char * content, uint16_t &size, uint16_t time_out = 0) = 0;
		virtual bool write(unsigned char * content, uint16_t size) = 0;
        virtual int getErrorCode() = 0;
	protected:
		NetConnect();
	private:
	};
}
#endif

#pragma once
#ifndef __PAVO2_HANDLETCP_H__
#define __PAVO2_HANDLETCP_H__
#include <boost/make_shared.hpp>
#include <pavo2_handle.h>
#include "pavo2_common.h"
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
namespace pavo2
{
	class pavo_driver;
	class NetConnect;

	class HandlePacketTCP :public HandlePacket
	{
	public:
		HandlePacketTCP(NetConnect * conn_ptr);
		~HandlePacketTCP();
		virtual bool recvData(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t &time_stamp, uint16_t time_out = 0);
		virtual bool recvData(std::vector<pavo_response_scan_t>& vec_buff, uint32_t &time_stamp, uint16_t time_out = 0);
		virtual bool recvCMD(PAVO2_CMD_INDEX cmd,unsigned char * content, uint16_t& count, uint16_t time_out=0);
		virtual bool sendCMD(unsigned char * content, uint16_t size);
	protected:
	private:
		bool enable_Unlock(bool val);
		NetConnect * conn_;
	};
}
#endif

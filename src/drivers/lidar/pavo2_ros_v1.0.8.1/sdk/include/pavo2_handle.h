#pragma once

#ifndef __PAVO2_HANDLE_H__
#define __PAVO2_HANDLE_H__
#include <boost/make_shared.hpp>
#include <vector>
#include "pavo2_types.h"
#include "pavo2_filter.h"

#include "pavo2_common.h"

namespace pavo2
{
	class HandlePacket
	{
	public:
        virtual ~HandlePacket();
		virtual bool recvData(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t &time_stamp, uint16_t time_out = 0) = 0;
		virtual bool recvData(std::vector<pavo_response_scan_t>& vec_buff, uint32_t &time_stamp, uint16_t time_out = 0) = 0;
		virtual bool recvCMD(PAVO2_CMD_INDEX cmd,unsigned char * content, uint16_t& count, uint16_t time_out=0) = 0;
		virtual bool sendCMD(unsigned char * content, uint16_t size) = 0;
		virtual bool enable_tail_filter(uint16_t method = 0);
		virtual bool enable_front_filter(uint16_t iLevel = 0);
	protected:
		HandlePacket();
		MADataFilter* m_pMADataFilter;
	private:		
	};
}
#endif

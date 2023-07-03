#include "pavo2_handletcp.h"
#include "pavo2_netconn.h"
#include "pavo2_filter.h"
#include <boost/bind.hpp>
#include <iostream>

#include <iomanip>
#include <fstream>
#include <sstream>
using namespace std;
//static std::ofstream logfile("pavo_handle_log.dat", ios::binary | ios::app);

pavo2::HandlePacketTCP::HandlePacketTCP(NetConnect * conn_ptr)
{
	conn_ = conn_ptr;
}

pavo2::HandlePacketTCP::~HandlePacketTCP()
{
	conn_ = nullptr;
}

bool pavo2::HandlePacketTCP::recvData(pavo_response_scan_t * data_buffer, uint16_t& count, uint32_t & time_stamp, uint16_t time_out)
{
	bool ret;
	unsigned char temp[BUFFER_SIZE];
	uint16_t temp_size = BUFFER_SIZE;
	data_info_t data_info;
	uint32_t loc, cnt;
	ret = conn_->readPoint(temp, temp_size, time_out);
	if (ret)
	{
		data_info.data_tag = ntohl(*(unsigned int*)(temp));
		data_info.packet_id = ntohl(*(unsigned int*)(temp + 4));
		data_info.data_type = ntohs(*(short*)(temp + 12));
		data_info.angle_resolution = ntohs(*(short*)(temp + 14));
		data_info.start_angle = ntohs(*(short*)(temp + 16));
		data_info.end_angle = ntohs(*(short*)(temp + 18));
		//with the intensity and distance
		if (data_info.data_type == INTENSITY_DISTANCE)
		{
			for (cnt = 0, loc = 20; (loc < (temp_size - 4)) && cnt < count; loc = loc + 4, cnt++)
			{
				data_buffer[cnt].angle = data_info.start_angle + cnt*data_info.angle_resolution;
				data_buffer[cnt].intensity = ntohs(*(short*)(temp + loc));
				data_buffer[cnt].distance = ntohs(*(short*)(temp + loc + 2));
				if (data_buffer[cnt].angle == data_info.end_angle) break;
			}
		}
		if (data_info.data_type == CHECK_MODE)
		{
			for (cnt = 0, loc = 20; (loc < (temp_size - 4)) && cnt < count; loc = loc + 4, cnt++)
			{
				data_buffer[cnt].angle = data_info.start_angle;
				data_buffer[cnt].intensity = ntohs(*(short*)(temp + loc));
				data_buffer[cnt].distance = ntohs(*(short*)(temp + loc + 2));
				if (data_buffer[cnt].angle == data_info.end_angle) break;
			}
		}
		if (data_info.data_type == DISTANCE)
		{
			for (cnt = 0, loc = 20; (loc < (temp_size - 4)) && cnt < count; loc = loc + 2, cnt++)
			{
				data_buffer[cnt].angle = data_info.start_angle + cnt*data_info.angle_resolution;
				data_buffer[cnt].intensity = 0;
				data_buffer[cnt].distance = ntohs(*(short*)(temp + loc));
				if (data_buffer[cnt].angle == data_info.end_angle) break;
			}
		}
		//with the distance
		count = cnt;

		if(m_pMADataFilter)
		{
			m_pMADataFilter->Filter(data_buffer,count);
		}
		return true;
	}
	return false;
}
static uint32_t packet_id_org = 0;
static bool is_first = true;
static uint32_t error_count = 0;
bool pavo2::HandlePacketTCP::recvData(std::vector<pavo_response_scan_t>& vec_buff, uint32_t & time_stamp, uint16_t time_out)
{
	bool ret;
	unsigned char temp[BUFFER_SIZE];
	uint16_t temp_size = BUFFER_SIZE;
	data_info_t data_info;
	pavo_response_scan_t point;
	uint32_t loc, cnt;
	ret = conn_->readPoint(temp, temp_size, time_out);
	if (ret)
	{
		data_info.data_tag = ntohl(*(unsigned int*)(temp));
		data_info.packet_id = ntohl(*(unsigned int*)(temp + 4));
		data_info.data_type = ntohs(*(short*)(temp + 12));
		data_info.angle_resolution = ntohs(*(short*)(temp + 14));
		data_info.start_angle = ntohs(*(short*)(temp + 16));
		data_info.end_angle = ntohs(*(short*)(temp + 18));
		if (is_first)
		{
			is_first = false;
			error_count = 0;
			packet_id_org = data_info.packet_id;
		}
		else
		{
			int diff = (data_info.packet_id - packet_id_org + 0x100000000) % 0x100000000;
			if (diff != 1) error_count++;
			packet_id_org = data_info.packet_id;
		}
		//std::cout << "Error count:" << error_count << std::endl;
		if (data_info.data_type == INTENSITY_DISTANCE)
		{
			for (cnt = 0, loc = 20; loc < temp_size - 4; loc = loc + 4, cnt++)
			{
				point.angle = data_info.start_angle + cnt*data_info.angle_resolution;
				point.intensity = ntohs(*(short*)(temp + loc));
				point.distance = ntohs(*(short*)(temp + loc + 2));
				vec_buff.push_back(point);
				if (point.angle == data_info.end_angle) break;
			}
		}
		if (data_info.data_type == CHECK_MODE)
		{
			for (cnt = 0, loc = 20; loc < temp_size - 4; loc = loc + 4, cnt++)
			{
				point.angle = data_info.start_angle;
				point.intensity = ntohs(*(short*)(temp + loc));
				point.distance = ntohs(*(short*)(temp + loc + 2));
				vec_buff.push_back(point);
				if (point.angle == data_info.end_angle) break;
			}
		}
		if (data_info.data_type == DISTANCE)
		{
			for (cnt = 0, loc = 20; loc < temp_size - 4; loc = loc + 2, cnt++)
			{
				point.angle = data_info.start_angle + cnt*data_info.angle_resolution;
				point.intensity = 0;
				point.distance = ntohs(*(short*)(temp + loc));
				vec_buff.push_back(point);
				if (point.angle == data_info.end_angle) break;
			}
		}

		if(m_pMADataFilter)
		{
			m_pMADataFilter->Filter(&(vec_buff[0]),vec_buff.size());
		}
		return true;
	}
	return false;
}

bool pavo2::HandlePacketTCP::recvCMD(PAVO2_CMD_INDEX cmd,unsigned char * content, uint16_t & count, uint16_t time_out)
{
	bool ret;
	unsigned char temp[BUFFER_SIZE];
	uint16_t temp_size = BUFFER_SIZE;
	data_info_t data_info;
	uint32_t loc, cnt;
	if (time_out == 0) time_out = 800;
	ret = conn_->readCMD(cmd,temp,temp_size,time_out);
	if (ret)
	{
		memcpy(content, temp, temp_size);
		count = temp_size;
		return true;
	}
	return false;
}

bool pavo2::HandlePacketTCP::sendCMD(unsigned char * content, uint16_t size)
{
	bool ret;
	if (content[CMD_SET_GET_INDEX] == CMD_SET)
	{
		ret = enable_Unlock(true);
		if (!ret) return false;
		ret = conn_->write(content, size);
		if (!ret)
		{
			enable_Unlock(false);
			return false;
		}
		enable_Unlock(false);
	}
	if (content[CMD_SET_GET_INDEX] == CMD_GET)
	{
		ret = conn_->write(content, size);
		if (!ret)
		{
			return false;
		}
	}

	return true;
}

bool pavo2::HandlePacketTCP::enable_Unlock(bool val)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_UNLOCK;;
	memset(send_cmd,0,8);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	if (val) send_cmd[7] = 0x01;
	ret = conn_->write(send_cmd, 8);
	return ret;
}


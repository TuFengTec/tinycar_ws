#include "pavo2_driver.h"
#include "pavo2_nettcp.h"
#include "pavo2_handle.h"
#include "pavo2_common.h"
#include "pavo2_handletcp.h"
#include <boost/bind.hpp>
#include <iostream>
#include <string>
#include <iomanip>

#include <fstream>
#include <sstream>
using namespace std;
//static std::ofstream logfile("pavo2_driver_log.dat", ios::binary | ios::app);
pavo2::pavo2_driver::pavo2_driver()
{
	conn_ptr_ = new NetConnectTCP();
	handle_packet_ptr_ = new HandlePacketTCP(conn_ptr_);
}

pavo2::pavo2_driver::~pavo2_driver()
{
	if (conn_ptr_)delete conn_ptr_;
	if (handle_packet_ptr_) delete handle_packet_ptr_;
}

bool pavo2::pavo2_driver::pavo2_open(string lidar_ip, uint16_t lidar_port, uint32_t timeout)
{
	bool ret;
    if(timeout == 0)
        ret = conn_ptr_->open(lidar_ip, lidar_port);
    else
        ret = conn_ptr_->open(lidar_ip, lidar_port, timeout);
	return ret;
}

bool pavo2::pavo2_driver::pavo2_is_open()
{
    return conn_ptr_->isOpen();
}

bool pavo2::pavo2_driver::pavo2_close()
{
    return conn_ptr_->close();
}

bool pavo2::pavo2_driver::get_scanned_data(pavo_response_scan_t * data_buffer, uint16_t & count, uint16_t timeout)
{
	bool ret;
	uint32_t time_stamp;
	ret = handle_packet_ptr_->recvData(data_buffer, count, time_stamp, timeout);
	return ret;
}

bool pavo2::pavo2_driver::get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint16_t timeout)
{
	bool ret;
	uint32_t time_stamp;
	vec_buff.clear();
	ret = handle_packet_ptr_->recvData(vec_buff, time_stamp, timeout);
	return ret;
}

bool pavo2::pavo2_driver::get_scanned_data(pavo_response_pcd_t * data_buffer, uint16_t & count, uint16_t timeout)
{
	bool ret;
	uint32_t time_stamp;
	uint32_t cnt;
	std::vector<pavo_response_scan_t> cn;
	ret = handle_packet_ptr_->recvData(cn, time_stamp, timeout);
	if (ret)
	{
		for (cnt = 0; cnt < cn.size() && (cnt < count); cnt++)
		{
			data_buffer[cnt].intensity = cn[cnt].intensity;
			data_buffer[cnt].x = cn[cnt].distance*cos(Degree2Radians(cn[cnt].angle/100.0f));
			data_buffer[cnt].y = cn[cnt].distance*sin(Degree2Radians(cn[cnt].angle/100.0f));
			data_buffer[cnt].z = 0;
		}
		count = cnt;
	}
	return ret;
}

bool pavo2::pavo2_driver::get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint16_t timeout)
{
	bool ret;
	uint32_t time_stamp;
	uint32_t cnt;
	vec_buff.clear();
	std::vector<pavo_response_scan_t> cn;
	pavo_response_pcd_t pcd;
	ret = handle_packet_ptr_->recvData(cn, time_stamp, timeout);
	if (ret)
	{
		for (cnt = 0; cnt < cn.size(); cnt++)
		{
			pcd.intensity = cn[cnt].intensity;
			pcd.x = cn[cnt].distance*cos(Degree2Radians(cn[cnt].angle/100.0f));
			pcd.y = cn[cnt].distance*sin(Degree2Radians(cn[cnt].angle/100.0f));
			pcd.z = 0;
			vec_buff.push_back(pcd);
		}
	}
	return ret;
}

bool pavo2::pavo2_driver::get_device_sn(string & sn)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_SN;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	stringstream ss;
	for (uint8_t i = CMD_LEN; i < recv_len; i++)
	{
		ss << std::hex << setw(2) << setfill('0') << setiosflags(ios::uppercase) << (short)recv_cmd[i];
		if (i == (recv_len - 1)) continue;
	}
	sn = ss.str();
	return ret;
}

bool pavo2::pavo2_driver::get_device_pn(string & pn)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_PN;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	stringstream ss;
	for (uint8_t i = CMD_LEN; i < recv_len; i++)
	{
		ss << std::hex << setw(2) << setfill('0') << setiosflags(ios::uppercase) << (short)recv_cmd[i];
		if (i == (recv_len - 1)) continue;
	}
	pn = ss.str();
	return ret;
}

bool pavo2::pavo2_driver::get_sdk_ver(string & ver)
{
	ver = string(SDK_VER);
	return true;
}

bool pavo2::pavo2_driver::get_os_ver(string & ver)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_OS_VER;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	stringstream ss;
	for (uint8_t i = CMD_LEN; i < recv_len; i++)
	{
		ss << (short)recv_cmd[i];
		if (i == (recv_len - 1)) continue;
		ss << ".";
	}
	ver = ss.str();
	return ret;
}

bool pavo2::pavo2_driver::get_firmware_ver(string & ver)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_FW_VER;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	stringstream ss;
	for (uint8_t i = CMD_LEN; i < recv_len; i++)
	{
		ss << (short)recv_cmd[i];
		if (i == (recv_len - 1)) continue;
		ss << ".";
	}
	ver = ss.str();
	return ret;
}

bool pavo2::pavo2_driver::get_os_error(uint32_t& errCode,uint8_t iIndex/* = 0*/)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_OS_EX;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd,PAVO2_CMD[cmd_type],CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd,CMD_LEN);
	if(!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the value
	errCode = 0;
	if(iIndex >= 0 && iIndex < 4)
	{
		errCode = recv_cmd[CMD_LEN + iIndex];
	}
	else
	{
		uint32_t errTemp = 0;
		for(uint8_t i = CMD_LEN; i < recv_len; i++)
		{
			errTemp = recv_cmd[i];
			for(uint8_t k = 0; k < (i - CMD_LEN); k++)
			{
				errTemp = errTemp << 8;
			}
			errCode |= errTemp;
		}
	}	
	return ret;
}

int  pavo2::pavo2_driver::get_error_code()
{
    return conn_ptr_->getErrorCode();
}

bool pavo2::pavo2_driver::enable_data(bool en)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ENABLE_DATA;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	if (en) send_cmd[send_len - 1] = 0x01;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::set_intensity_mode(bool en)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_INTENSITY_MODE;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	if (en) send_cmd[send_len - 1] = 0x01;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_intensity_mode(bool & en)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_INTENSITY_MODE;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	if (recv_cmd[recv_len - 1])
	{
		en = true;
	}
	else
	{
		en = false;
	}
	return ret;
}

bool pavo2::pavo2_driver::set_angle_range(uint16_t start, uint16_t end)
{
	if(start < ANGLE_RANGLE_MIN || end > ANGLE_RANGLE_MAX || start >= end)
		return false;

	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ANGLE_RANGLE;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	//set the content
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_cmd[CMD_LEN] = 0xFF & (start >> 8);
	send_cmd[CMD_LEN + 1] = 0xFF & (start);
	send_cmd[CMD_LEN + 2] = 0xFF & (end >> 8);
	send_cmd[CMD_LEN + 3] = 0xFF & (end);
	//set the send len
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_angle_range(uint16_t & start, uint16_t & end)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ANGLE_RANGLE;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	start = (uint16_t)recv_cmd[CMD_LEN] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 1];
	end = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
	return ret;
}

bool pavo2::pavo2_driver::get_motor_speed(uint8_t & val)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_MOTOR_SPEED;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	val = (uint16_t)recv_cmd[CMD_LEN + 3];
	return ret;
}

bool pavo2::pavo2_driver::set_angle_resolution(uint16_t val)
{
	if(val != 8 && val != 16 && val != 32) return false;

	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ANGLE_RESOLUTION;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);
	memset(send_cmd, 0, send_len);
	//set the content
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;

	send_cmd[CMD_LEN + 2] = 0xFF & (val >> 8);
	send_cmd[CMD_LEN + 3] = 0xFF & (val);
	//set the send len
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_angle_resolution(uint16_t & val)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ANGLE_RESOLUTION;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	val = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
	return ret;
}

bool pavo2::pavo2_driver::set_device_ip(string ip)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_IP;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	//set the content
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	ret = string2Byte(ip, send_cmd + CMD_LEN);
	if (!ret) return false;
	//set the send len
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_device_ip(string & ip)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_IP;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	ret = bytes2String(ip, recv_cmd + CMD_LEN);
	return ret;
}

bool pavo2::pavo2_driver::set_tcp_port(uint16_t val)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_TCPPORT;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	//set the content
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_cmd[CMD_LEN + 2] = 0xFF & (val >> 8);
	send_cmd[CMD_LEN + 3] = 0xFF & (val);
	//set the send len
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_tcp_port(uint16_t & val)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_TCPPORT;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, CMD_LEN);
	if (!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd, recv_len);
	if (!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type, recv_cmd, recv_len);
	if (!ret) return false;
	//check the value
	val = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
	return ret;
}

bool pavo2::pavo2_driver::reset_device_default()
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_RESET;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);
	const unsigned char cmd_arg[4] = { 0xF1,0x2F,0xE3,00 };
	memset(send_cmd, 0, send_len);
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	memcpy(send_cmd + CMD_LEN, cmd_arg, PAVO2_CMD_ARG_LEN[cmd_type]);
	send_cmd[CMD_LEN + 3] = 0x01;
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::set_os_restart()
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_SLEEP_RESTART;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd, 0, send_len);
	//set the content
	memcpy(send_cmd, PAVO2_CMD[cmd_type], CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_cmd[CMD_LEN + 3] = 0x02;
	//set the send len
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd, send_len);
	if (!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::enable_tail_filter(uint16_t method)
{
	return handle_packet_ptr_->enable_tail_filter(method);
}

bool pavo2::pavo2_driver::enable_front_filter(uint16_t iLevel)
{
	return handle_packet_ptr_->enable_front_filter(iLevel);
}

bool pavo2::pavo2_driver::get_device_mac(string& mac)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[12];
	PAVO2_CMD_INDEX cmd_type = CMD_MAC;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd,PAVO2_CMD[cmd_type],CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd,CMD_LEN);
	if(!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the value
	stringstream ss;
	for(uint8_t i = CMD_LEN + 2; i < recv_len; i++)
	{
		ss <<hex<<(short)recv_cmd[i];
		if(i == (recv_len - 1)) continue;
		ss << ":";
	}
	mac = ss.str();
	return ret;
}

bool pavo2::pavo2_driver::set_echo_mode(bool en)
{
	bool ret;
	unsigned char send_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ECHO_MODE;
	uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
	memset(send_cmd,0,send_len);
	memcpy(send_cmd,PAVO2_CMD[cmd_type],CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
	send_len = CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_type];
	if(en) send_cmd[send_len - 1] = 0x01;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd,send_len);
	if(!ret) return false;
	return ret;
}

bool pavo2::pavo2_driver::get_echo_mode(bool& en)
{
	bool ret;
	unsigned char send_cmd[8];
	unsigned char recv_cmd[8];
	PAVO2_CMD_INDEX cmd_type = CMD_ECHO_MODE;
	uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
	memcpy(send_cmd,PAVO2_CMD[cmd_type],CMD_LEN);
	send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
	//send the cmd
	ret = handle_packet_ptr_->sendCMD(send_cmd,CMD_LEN);
	if(!ret) return false;
	//recvice the cmd
	ret = handle_packet_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the type and arg len
	ret = check_cmd(cmd_type,recv_cmd,recv_len);
	if(!ret) return false;
	//check the value
	if(recv_cmd[recv_len - 1])
	{
		en = true;
	}
	else
	{
		en = false;
	}
	return ret;
}

std::string & pavo2::pavo2_driver::TrimSpace(std::string & str)
{
	// TODO: 在此处插入 return 语句
	if (str.empty())
	{
		return str;
	}
	str.erase(0, str.find_first_not_of(" "));  //trim white space
	str.erase(str.find_last_not_of(" ") + 1);

	str.erase(0, str.find_first_not_of("\t"));  //trim tab
	str.erase(str.find_last_not_of("\t") + 1);

	return str;
}

bool pavo2::pavo2_driver::string2Byte(const std::string & ip_str, unsigned char * ip_bytes)
{
	if (ip_str.empty())  //empty string
		return false;

	if (count(ip_str.begin(), ip_str.end(), '.') != 3)
		return false;

	std::string ip_str_copy(ip_str);
	TrimSpace(ip_str_copy); //remove leading and tail space

	std::string str_tmp;
	std::size_t prev_pos = 0;

	int dot_num = 0;
	int byte_single = 0;
	char tmp_bytes[4];

	for (std::size_t i = 0; i < ip_str_copy.length(); i++)
	{
		if (ip_str_copy[i] == '.')
		{
			if (dot_num++ > 3)
				return false;

			str_tmp = ip_str_copy.substr(prev_pos, i - prev_pos);
			prev_pos = i + 1;

			byte_single = atoi(str_tmp.c_str());  //invalid value  
			if (byte_single > 255)
				return false;
			tmp_bytes[dot_num - 1] = static_cast<char>(byte_single);

			continue;
		}
		if ((ip_str_copy[i] < '0') || (ip_str_copy[i] > '9')) //invalid character
			return false;

	}

	str_tmp = ip_str_copy.substr(prev_pos);
	byte_single = atoi(str_tmp.c_str());
	tmp_bytes[dot_num] = static_cast<char>(byte_single);

	memcpy(ip_bytes, tmp_bytes, 4);

	return true;
}

bool pavo2::pavo2_driver::bytes2String(std::string & ip_str, const unsigned char * ip_bytes)
{
	if (ip_bytes == 0)
		return false;

	ip_str = std::to_string(static_cast<unsigned char>(ip_bytes[0])) + "." +
		std::to_string(static_cast<unsigned char>(ip_bytes[1])) + "." +
		std::to_string(static_cast<unsigned char>(ip_bytes[2])) + "." +
		std::to_string(static_cast<unsigned char>(ip_bytes[3]));
	return true;
}

inline bool pavo2::pavo2_driver::check_cmd(PAVO2_CMD_INDEX cmd, unsigned char * content, uint16_t size)
{
	content[CMD_SET_GET_INDEX] = 0x00;
	if (size < CMD_LEN) return false;
	if (memcmp(content, PAVO2_CMD[cmd], CMD_LEN))return false;
	uint16_t arg_len;
	arg_len = size - CMD_LEN;
	if (arg_len != PAVO2_CMD_ARG_LEN[cmd]) return false;
	return true;
}

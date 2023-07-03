#pragma once
#ifndef __PAVO2_DRIVER_H__
#define __PAVO2_DRIVER_H__
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <string>
#include "pavo2_types.h"
#include "pavo2_common.h"
using std::string;
#define SDK_VER "1.0.6.1"

namespace pavo2
{
	class NetConnect;
	class HandlePacket;
	class pavo2_driver
	{
	public:
		pavo2_driver();
		~pavo2_driver();
        bool pavo2_open(string lidar_ip, uint16_t lidar_port, uint32_t timeout = 0);
        bool pavo2_is_open();
		bool pavo2_close();
		bool get_scanned_data(pavo_response_scan_t* data_buffer, uint16_t& count, uint16_t timeout = 0);
		bool get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint16_t timeout = 0);
		bool get_scanned_data(pavo_response_pcd_t* data_buffer, uint16_t& count, uint16_t timeout = 0);
		bool get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint16_t timeout = 0);
		bool get_device_sn(string & sn);
		bool get_device_pn(string & pn);
		bool get_sdk_ver(string & ver);
		bool get_os_ver(string & ver);
		bool get_firmware_ver(string & ver);
		bool get_os_error(uint32_t& errCode,uint8_t iIndex = 0);
        int  get_error_code();
		bool enable_data(bool en);
		bool set_intensity_mode(bool en);
		bool get_intensity_mode(bool & en);
		bool set_angle_range(uint16_t start, uint16_t end);
		bool get_angle_range(uint16_t & start, uint16_t & end);
		bool get_motor_speed(uint8_t & val);
		bool set_angle_resolution(uint16_t val);
		bool get_angle_resolution(uint16_t& val);
		bool set_device_ip(string ip);
		bool get_device_ip(string & ip);
		bool set_tcp_port(uint16_t val);
		bool get_tcp_port(uint16_t & val);
		bool reset_device_default();
		bool set_os_restart();
		bool enable_tail_filter(uint16_t method);
		bool get_device_mac(string& mac);
		bool set_echo_mode(bool en);
		bool get_echo_mode(bool& en);
		bool enable_front_filter(uint16_t iLevel);
	protected:
		NetConnect * conn_ptr_;
		HandlePacket * handle_packet_ptr_;
	private:
	    std::string& TrimSpace(std::string &str);
		bool string2Byte(const std::string& ip_str, unsigned char* ip_bytes);
		bool bytes2String(std::string& ip_str, const unsigned char* ip_bytes);
		inline bool check_cmd(PAVO2_CMD_INDEX cmd,unsigned char * content,uint16_t size);
	};
}
#endif

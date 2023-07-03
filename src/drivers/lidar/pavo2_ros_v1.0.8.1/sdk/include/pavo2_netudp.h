#pragma once

#ifndef __PAVO2_NETUDP_H__
#define __PAVO2_NETUDP_H__

#include "pavo2_netconn.h"
#include "pavo2_common.h"
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace pavo2
{
	class NetConnectUDP :public NetConnect
	{
	public:
		NetConnectUDP();
		~NetConnectUDP();
		virtual bool open(string lidar_ip,uint16_t lidar_port);
		virtual bool open(string lidar_ip,uint16_t lidar_port,uint32_t timeout);
		virtual bool isOpen();
		virtual bool close();
		virtual bool readPoint(unsigned char * content,uint16_t &size,uint16_t time_out = 0);
		virtual bool readCMD(unsigned char * content,uint16_t &size,uint16_t time_out = 0);
		virtual bool write(unsigned char * content,uint16_t size);
		virtual int  getErrorCode();
	private:
		void run();
		void startThread();
		void parseReceiveUDP(const boost::system::error_code& error,std::size_t rxBytes);
		void parsePacket(uint16_t start,uint16_t len);

		void handle_connect(const boost::system::error_code& err);
		void handle_wait(const boost::system::error_code& err);
	protected:
	private:
		boost::scoped_ptr<boost::thread> thread_;
		boost::asio::io_service IOService_;
		boost::scoped_ptr<boost::asio::io_service::work> dummy_work_;
		boost::scoped_ptr<boost::asio::ip::udp::socket> sock_;
		boost::asio::ip::udp::endpoint lidar_endpoint_;
		boost::asio::ip::udp::endpoint RemoteEndpoint;

		int error_code_;

		unsigned char recv_vec_[BUFFER_SIZE];
		unsigned char send_vec_[BUFFER_SIZE];
		unsigned char temp_buff_[BUFFER_SIZE * 2];
		int temp_buffer_size_;

		bool should_stop_;

		boost::mutex is_receiving_mtx_;
		boost::condition_variable is_receiving_cond_;
		bool is_receiving_;

		boost::mutex point_op_mutx_;  //protect: IsResponseReceived, FrameCount, IsDataMode
		boost::condition_variable point_op_cond_;
		int frame_count_;
		unsigned char point_vec_[FRAME_CAPCITY][BUFFER_SIZE];
		uint16_t point_vec_size_[FRAME_CAPCITY];

		boost::mutex cmd_op_mutx_;  //protect: IsResponseReceived, FrameCount, IsDataMode
		boost::condition_variable cmd_op_cond_;
		int cmd_count_;
		unsigned char cmd_vec_[FRAME_CAPCITY][BUFFER_SIZE];
		uint16_t cmd_vec_size_[FRAME_CAPCITY];

		//For timeout open
		boost::scoped_ptr<boost::asio::deadline_timer> sock_timer_;
		boost::mutex is_connectiong_mtx_;
		boost::condition_variable is_connecting_cond_;
		bool is_connecting_;
		bool is_connecting_error_;
	};
}

#endif
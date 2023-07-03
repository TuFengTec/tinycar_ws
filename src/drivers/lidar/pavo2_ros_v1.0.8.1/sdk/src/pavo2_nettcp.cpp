#include "pavo2_nettcp.h"
#include "pavo2_common.h"
#include <boost/function.hpp>
#include <boost/thread.hpp>


#include <fstream>
#include <sstream>
//#include <QDebug>
using namespace std;
static std::ofstream logfile("pavo_nettcp_log.dat", ios::binary | ios::app);

pavo2::NetConnectTCP::NetConnectTCP() :
	NetConnect(),
	IOService_(),
	dummy_work_(nullptr),
	should_stop_(false),
    is_receiving_(false),
    error_code_(0)
{

}

pavo2::NetConnectTCP::~NetConnectTCP()
{
	close();
}


bool pavo2::NetConnectTCP::open(string lidar_ip, uint16_t lidar_port)
{
	//init the global variable
	IOService_.reset();
	dummy_work_.reset(new boost::asio::io_service::work(IOService_));

	//////////////////////////
	lidar_endpoint_.address(boost::asio::ip::address::from_string(lidar_ip));
	lidar_endpoint_.port(lidar_port);
	sock_.reset(new boost::asio::ip::tcp::socket(IOService_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0)));	
	boost::system::error_code ec;
	sock_->connect(lidar_endpoint_, ec);
	if (ec.value())
	{
		IOService_.stop();

		if (this->thread_)
		{
			this->thread_->join();
			this->thread_.reset();
		}
		dummy_work_.reset();
		sock_.reset();

		return false;
	}
	sock_->set_option(boost::asio::socket_base::reuse_address(true), ec);
	sock_->set_option(boost::asio::ip::tcp::no_delay(true),ec);
	should_stop_ = false;
	is_receiving_ = false;
	temp_buffer_size_ = 0;
	frame_count_ = 0;
	m_listCmd.clear();
	startThread();
	run();
	return true;
}

bool pavo2::NetConnectTCP::open(string lidar_ip, uint16_t lidar_port, uint32_t timeout) //milliseconds
{
    IOService_.reset();
    dummy_work_.reset(new boost::asio::io_service::work(IOService_));

    lidar_endpoint_.address(boost::asio::ip::address::from_string(lidar_ip));
    lidar_endpoint_.port(lidar_port);
    sock_.reset(new boost::asio::ip::tcp::socket(IOService_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0)));
    boost::system::error_code ec;
    sock_->async_connect(
            lidar_endpoint_,
            boost::bind(&NetConnectTCP::handle_connect, this,
              boost::asio::placeholders::error));

    sock_timer_.reset(new boost::asio::deadline_timer(IOService_));
    sock_timer_->expires_from_now(boost::posix_time::milliseconds(timeout));
    sock_timer_->async_wait(boost::bind(&NetConnectTCP::handle_wait, this, boost::asio::placeholders::error));

    is_connecting_ = true;
    is_connecting_error_ = false;

    startThread();

    {
        boost::unique_lock<boost::mutex> guard(this->is_connectiong_mtx_);
        while (this->is_connecting_)
        {
            this->is_connecting_cond_.wait(guard);
        }
    }

    if(is_connecting_error_)
    {
        IOService_.stop();

        if (this->thread_)
        {
            this->thread_->join();
            this->thread_.reset();
        }
        dummy_work_.reset();
        sock_.reset();
    }

    return !is_connecting_error_;
}


//Will always be called
//1. connection succes; 2. connection error; 3. closed by timer
void pavo2::NetConnectTCP::handle_connect(const boost::system::error_code& err)
{
    boost::unique_lock<boost::mutex> guard(this->is_connectiong_mtx_);
    sock_timer_->cancel();
    is_connecting_ = false;

    // The async_connect() function automatically opens the socket at the start
    // of the asynchronous operation. If the socket is closed at this time then
    // the timeout handler must have run first.
    if (!sock_->is_open() || err)
    {
        is_connecting_error_ = true;
    }
    else
    {
        sock_->set_option(boost::asio::socket_base::reuse_address(true));
        sock_->set_option(boost::asio::ip::tcp::no_delay(true));
        should_stop_ = false;
        is_receiving_ = false;
        temp_buffer_size_ = 0;
		frame_count_ = 0;
		m_listCmd.clear();
        run(); //start receiving
    }
    is_connecting_cond_.notify_one();
    return;
}

//will always be called
//1. timeout; 2. cancelled by connect handler
void pavo2::NetConnectTCP::handle_wait(const boost::system::error_code& err)
{
    boost::unique_lock<boost::mutex> guard(this->is_connectiong_mtx_);
    if(is_connecting_ == false)
    {
        is_connecting_cond_.notify_one();
        return;
    }

    is_connecting_ = false;

    if(err) //on error, may be cancelled
    {
        is_connecting_cond_.notify_one();
        return;
    }

    sock_->close();
    is_connecting_error_ = true;
    is_connecting_cond_.notify_one();
    return;
}


bool pavo2::NetConnectTCP::isOpen()
{
    if(!sock_)
        return false;

    bool ret = sock_->is_open();

	return (ret && this->is_receiving_);
}

bool pavo2::NetConnectTCP::close()
{
    boost::system::error_code ec;
	if (this->sock_ != nullptr && this->sock_->is_open())
	{
		{
			boost::unique_lock<boost::mutex> guard(this->is_receiving_mtx_);
			this->should_stop_ = true;
            //this->sock_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
            //by_zhb//this->sock_->cancel();
            this->sock_->close(ec);
			while (this->is_receiving_)
			{
				this->is_receiving_cond_.wait(guard);
			}
		}
	}

    //inform all user threads to stop waiting
    //this->point_op_cond_.notify_all();
    //this->cmd_op_cond_.notify_all();

	IOService_.stop();

	if (this->thread_)
	{
		this->thread_->join();
		this->thread_.reset();
	}
	dummy_work_.reset();
    sock_.reset();
	return true;
}

bool pavo2::NetConnectTCP::readPoint(unsigned char * content, uint16_t &size, uint16_t time_out)
{
	if (time_out < 0)
		return false;

	{
        boost::unique_lock<boost::mutex> guard(this->point_op_mutx_);
		if (time_out == 0)
		{
			while(this->frame_count_ == 0 && this->is_receiving_)
			{
				//wake up for: 1.data ready; 2.force ending
				this->point_op_cond_.wait(guard);//FIXME: block here
			}
		}
		else //timeout > 0
		{
			boost::system_time const sys_timeout = boost::get_system_time() + boost::posix_time::milliseconds(time_out);
			while(this->frame_count_ == 0 && this->is_receiving_)
			{
				if(!this->point_op_cond_.timed_wait(guard,sys_timeout))
					return false;
			}
		}

        if(!this->is_receiving_)
            return false;

		this->frame_count_--;
		size = this->point_vec_size_[this->frame_count_];
		memcpy(content,this->point_vec_ + this->frame_count_,size);
	}
	return true;
}

bool pavo2::NetConnectTCP::readCMD(PAVO2_CMD_INDEX cmd,unsigned char * content, uint16_t &size, uint16_t time_out)
{
	if (time_out < 0)
		return false;

	{
		boost::unique_lock<boost::mutex> guard(this->cmd_op_mutx_);
		if (time_out == 0)
		{
            while(this->noMatchingCmd(cmd) && this->is_receiving_)
			{
				this->cmd_op_cond_.wait(guard);
			}

		}
		else //timeout > 0
		{
			boost::system_time const sys_timeout = boost::get_system_time() + boost::posix_time::milliseconds(time_out);
            while (this->noMatchingCmd(cmd) && this->is_receiving_)
			{
				if (!this->cmd_op_cond_.timed_wait(guard, sys_timeout))
					return false;
			}
		}

        if(!this->is_receiving_)
            return false;
		
		bool bFind = false;
		unsigned char contentTemp[128] = { 0 };
		for(auto it = m_listCmd.begin(); it != m_listCmd.end(); it++)
		{
			size = (*it)->cmdSize;
			memcpy(contentTemp,(*it)->szCmd,size);
			contentTemp[CMD_SET_GET_INDEX] = 0x00;
			if(size < CMD_LEN)
				continue;
			if(memcmp(contentTemp,PAVO2_CMD[cmd],CMD_LEN))
				continue;
			memcpy(content,(*it)->szCmd,size);
			CmdDataDef* cmdTemp = *it;
			m_listCmd.erase(it);
			delete cmdTemp;
			bFind = true;
			break;
		}
		if(bFind == false)
			return false;
	}
	return true;
}

bool pavo2::NetConnectTCP::write(unsigned char * content, uint16_t size)
{
	uint16_t send_len;
	memset(send_vec_, 0, BUFFER_SIZE);
	//add the protocol fixed head
	send_len = 0;
	memcpy(send_vec_ + send_len, PROTOCOL_HEAD, PROTOCOL_HEAD_LEN);
	send_len = send_len + PROTOCOL_HEAD_LEN;
	//add the version and reserver
	memcpy(send_vec_ + send_len, PROTOCOL_VERDION_RESERVER, PROTOCOL_VERSION_RESERVER_LEN);
	send_len = send_len + PROTOCOL_VERSION_RESERVER_LEN;
	//add the len
	send_vec_[send_len++] = 0x00;
	send_vec_[send_len++] = 0x00;
	send_vec_[send_len++] = (0xFF & (size >> 8));
	send_vec_[send_len++] = (0xFF & (size));
	//add the content
	memcpy(send_vec_ + send_len, content, size);
	send_len = send_len + size;
	//add the protocol fixed tail
	memcpy(send_vec_ + send_len, PROTOCOL_END, PROTOCOL_END_LEN);
	send_len = send_len + PROTOCOL_END_LEN;
	//send the data
	size_t send_count;
	send_count = 0;
	try
	{
		send_count = sock_->send(boost::asio::buffer(send_vec_, send_len));
	}
	catch (...)
	{
        //logfile<<__FUNCTION__<<":"<<__LINE__<<":"<<this->frame_count_<<std::endl;
	}
	return send_count ? true : false;
}

int pavo2::NetConnectTCP::getErrorCode()
{
    return error_code_;

}

void pavo2::NetConnectTCP::run()
{
	{
		boost::lock_guard<boost::mutex> guard(this->is_receiving_mtx_);
		this->is_receiving_ = true;
	}
	sock_->async_receive(boost::asio::buffer(recv_vec_), boost::bind(&NetConnectTCP::parseReceiveTCP, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));
}

void pavo2::NetConnectTCP::parseReceiveTCP(const boost::system::error_code & error, std::size_t rxBytes)
{
	if (error || this->should_stop_)
	{
		{
			boost::lock_guard<boost::mutex> guard(this->is_receiving_mtx_);
			this->is_receiving_ = false;
            if(!this->should_stop_)
                this->error_code_ = -1; //FIXME: need more information
		}
		this->is_receiving_cond_.notify_one();
        this->point_op_cond_.notify_all();
        this->cmd_op_cond_.notify_all();
		return;
	}
	memcpy(this->temp_buff_ + this->temp_buffer_size_, this->recv_vec_, rxBytes);
#if 0
	for (int i = 0; i < rxBytes; i++)
	{
        //logfile << std::dec << "id:" << i << ":" << std::hex << setiosflags(ios::right | ios::uppercase)
        //	<< setfill('0') << setw(2) << (unsigned short)this->recv_vec_[i] << std::endl;
	}
#endif 
	this->temp_buffer_size_ += rxBytes;
	int cur = 0;
	for (int i = 0; i < temp_buffer_size_;)
	{
		if (((i + PROTOCOL_LEAST_LEN - 1) < temp_buffer_size_) && !memcmp(temp_buff_ + i, PROTOCOL_HEAD, PROTOCOL_HEAD_LEN))
		{
			uint16_t loc = i + PACKET_DATA_LEN_LOC;
			uint32_t packet_len = ((unsigned int)(temp_buff_[loc])) * 0x1000000 + ((unsigned int)(temp_buff_[loc + 1])) * 0x10000 + ((unsigned int)(temp_buff_[loc + 2])) * 0x100 + ((unsigned int)(temp_buff_[loc + 3]));
			uint16_t next_cur = i + PROTOCOL_LEAST_LEN + packet_len;
			if (next_cur > temp_buffer_size_)
			{
				break;
			}
			//handle the packet
			parsePacket(i + PACKET_START_LOC, packet_len);
			/////////////////////////////////////////////////////////////
			cur = next_cur;
			i = cur;
			continue;
         }
		i++;
		continue;
	}
	if ((temp_buffer_size_ - cur) > BUFFER_LEFT_SIZE)
	{
		cur = temp_buffer_size_ - BUFFER_LEFT_SIZE;
		temp_buffer_size_ = BUFFER_LEFT_SIZE;
	}
	else
	{
		temp_buffer_size_ = temp_buffer_size_ - cur;
	}
	memcpy(this->temp_buff_, this->temp_buff_ + cur, temp_buffer_size_);
	run();
}

void pavo2::NetConnectTCP::parsePacket(uint16_t start, uint16_t len)
{
	const unsigned char PACKET_DATA_TYPE[4] = { 0xEB,0x90,0xC0,0x00 };
	const unsigned char PACKET_CMD_GET_TYPE[4] = { 0xEB,0x90,0xB0,0x00 };
	const unsigned char PACKET_CMD_SET_TYPE[4] = { 0xEB,0x90,0xA0,0x00 };
	if (!memcmp(this->temp_buff_ + start, PACKET_DATA_TYPE, 4))
	{
		boost::unique_lock<boost::mutex> guard(this->point_op_mutx_);

		this->frame_count_ = this->frame_count_%FRAME_CAPCITY;
		memcpy(point_vec_ + this->frame_count_,this->temp_buff_ + start,len);
		this->point_vec_size_[this->frame_count_] = len;
		this->frame_count_++;
		this->point_op_cond_.notify_one();
	}
	if (!memcmp(this->temp_buff_ + start, PACKET_CMD_SET_TYPE, 3) || !memcmp(this->temp_buff_ + start, PACKET_CMD_GET_TYPE, 3))
	{
		boost::unique_lock<boost::mutex> guard(this->cmd_op_mutx_);
		CmdDataDef* cmdData = new CmdDataDef();
		cmdData->cmdSize = len;
		memset(cmdData->szCmd,0,sizeof(cmdData->szCmd));
		memcpy(cmdData->szCmd,this->temp_buff_ + start,len);
		m_listCmd.push_back(cmdData);
		this->cmd_op_cond_.notify_one();
	}
}

void pavo2::NetConnectTCP::startThread()
{
	if (thread_) return;
	thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &IOService_)));
}

inline bool pavo2::NetConnectTCP::noMatchingCmd(PAVO2_CMD_INDEX cmd)
{
	if(m_listCmd.empty())
		return true;

	uint16_t size;
	uint16_t arg_len;
	unsigned char content[128] = { 0 };
	for(auto it = m_listCmd.begin();it != m_listCmd.end();it++)
	{
		size = (*it)->cmdSize;
		memcpy(content,(*it)->szCmd,size);
		content[CMD_SET_GET_INDEX] = 0x00;
		if(size < CMD_LEN)
			continue;
		if(memcmp(content,PAVO2_CMD[cmd],CMD_LEN))
			continue;		
		arg_len = size - CMD_LEN;
		if(arg_len != PAVO2_CMD_ARG_LEN[cmd])
			continue;
		return false;
	}
	return true;
}


#ifndef DGNSS_H
#define DGNSS_H

#include <stdlib.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "ins_structures.h"
#include "ring_buffer.h"
#include "../serial.h"

namespace DGNSS_NAMESPACE {
//ringbuffer
#define DEFAULT_PACKET_TIMEOUT_MS  10 //s

// Messaging callbacks
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarningMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
//! NAV callbacks
typedef boost::function<void(ImuMsg_t&, double&)> ImuMsgCallback;
typedef boost::function<void(GnssMsg_t&, double&)> GnssMsgCallback;
typedef boost::function<void(GnssAjMsg_t&, double&)> GnssAjMsgCallback;
typedef boost::function<void(InsMsg_t&, double&)> InsMsgCallback;
typedef boost::function<void(WheelSpeedMsg_t&, double&)> WheelSpeedMsgCallback;
typedef boost::function<void(UwbMsg_t&, double&)> UwbMsgCallback;
typedef boost::function<void(UwbTdoaPosMsg_t&, double&)> UwbTdoaPosMsgCallback;


class Dgnss
{
public:
	Dgnss();
	~Dgnss();

    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out);
    int ValueInitOpenSerial(const char * device_port, int baudrate);
    void ReadSerialPort();

    //////////////////////////////////////////////////////
    // Set Callback Methods
    //////////////////////////////////////////////////////
    void SetImuMsgCallback(ImuMsgCallback callback){imu_msg_callback_=callback;};
    void SetGnssMsgCallback(GnssMsgCallback callback){gnss_msg_callback_=callback;};
    void SetGnssAjMsgCallback(GnssAjMsgCallback callback){gnss_aj_msg_callback_=callback;};
    void SetInsMsgCallback(InsMsgCallback callback){ins_msg_callback_=callback;};
    void SetWheelSpeedMsgCallback(WheelSpeedMsgCallback callback){wheel_speed_msg_callback_=callback;};
    void SetUwbMsgCallback(UwbMsgCallback callback){uwb_msg_callback_=callback;};
    void SetUwbTdoaPosMsgCallback(UwbTdoaPosMsgCallback callback){uwb_tdoa_pos_msg_callback_=callback;};

protected:
	Serial *m_serial;
    int m_serial_fd;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.
	double read_timestamp_; 		//!< time stamp when last serial port read completed
    
private:
    void ParseLog(uint8_t* log, size_t logID, uint32_t log_len); //! Function to parse logs into a usable structure
    void ProcIncomingData();

    //////////////////////////////////////////////////////
    // Callbacks
    //////////////////////////////////////////////////////
    //! Logging Callbacks
    DebugMsgCallback log_debug_;
    InfoMsgCallback log_info_;
    WarningMsgCallback log_warning_;
    ErrorMsgCallback log_error_;

    //! UBX Data Callbacks
    ImuMsgCallback imu_msg_callback_;
    GnssMsgCallback gnss_msg_callback_;
    GnssAjMsgCallback gnss_aj_msg_callback_;
    InsMsgCallback ins_msg_callback_;
    WheelSpeedMsgCallback wheel_speed_msg_callback_;
    UwbMsgCallback uwb_msg_callback_;
    UwbTdoaPosMsgCallback uwb_tdoa_pos_msg_callback_;

    ring_buffer ring_buffer_;  // ringbuffer
    unsigned char data_buffer_[MAX_PACKET_SIZE];
    unsigned char packet[MAX_PACKET_SIZE];
    size_t packet_byte_count;
    unsigned int parser_num_bad_checksums;
    unsigned int parser_headers_skipped;
    bool  parser_in_sync;
    bool reading_data_;
    double parser_start_time;
    unsigned int parser_timeouts;
    unsigned int packet_timeout;
    

    uint16_t payload_length_;  // g/
    size_t bytes_remaining_;    //!< bytes remaining to be read in the current message
    size_t buffer_index_;       //!< index into data_buffer_
};
}

#endif // DGNSS_H

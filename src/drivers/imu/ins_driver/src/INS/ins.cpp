#include <ros/ros.h>
#include <sys/time.h>
#include "ins.h"

using namespace std;
using namespace DGNSS_NAMESPACE;

#define PI 3.14159265

inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cout << "Dgnss Debug: " << msg << std::endl;
}
inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cout << "Dgnss Info: " << msg << std::endl;
}
inline void DefaultWarningMsgCallback(const std::string &msg) {
    std::cout << "Dgnss Warning: " << msg << std::endl;
}
inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Dgnss Error: " << msg << std::endl;
}

Dgnss::Dgnss() {
    m_serial = new Serial();
    reading_status_ = true;
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    bytes_remaining_ = false;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    reading_data_ = false;

    memset(packet, 0, MAX_PACKET_SIZE);
    packet_byte_count = 0;
    parser_start_time = 0;
    parser_num_bad_checksums = 0;
    parser_timeouts = 0;
    parser_in_sync = 0;
    parser_headers_skipped = 0;
    packet_timeout = DEFAULT_PACKET_TIMEOUT_MS;

    // init ring_buffer_
    if(ring_buffer_init_static(&ring_buffer_, data_buffer_, MAX_PACKET_SIZE, 1) != RING_BUFFER_OK) {
        std::cout << "ring buffer init fail!" <<std::endl;
    }
}

Dgnss::~Dgnss() {
    m_serial->ClosePort(m_serial_fd);

    if (m_serial) {
        delete m_serial;
        m_serial = NULL;
    }
}

int Dgnss::ValueInitOpenSerial(const char * device_port, int baudrate)
{
    m_serial_fd = m_serial->OpenPort(device_port);
    if (m_serial_fd < 0)
    {
        printf("Open serial port %s failed!\n", device_port);
        return -1;
    }
    else
    {
        printf("Open serial port %s Success\n", device_port);
    }

    m_serial->SetPara(m_serial_fd, baudrate);

    return 0;
}

void Dgnss::ReadSerialPort() {
    unsigned char buffer[MAX_NOUT_SIZE];
    unsigned int len = 0;
    unsigned int bytes_written = 0;

    // continuously read data from serial port
    while (reading_status_) {
        try {
            len = m_serial->ReadData(m_serial_fd, buffer, 40);
            // std::cout << "received: " << len << std::endl;
        } catch (std::exception &e) {
            std::stringstream output;
            output << "Error reading from serial port: " << e.what();
            log_error_(output.str());
        }

        if(len > 0) {
            // add data to the buffer to be parsed
            // for(int i = 0; i < len;i++)
            //     printf("%02x ",buffer[i]);
            ring_buffer_write_multi(&ring_buffer_, buffer, len, &bytes_written);
            len = 0;
        }
        ProcIncomingData();
    }
}

void Dgnss::ProcIncomingData() {
    int i = 0, ret;
    // std::cout << packet_byte_count << ", " << ring_buffer_count(&ring_buffer_) << std::endl;
    if(packet_byte_count < HEADER_SIZE){
        while(0==packet_byte_count && ring_buffer_count(&ring_buffer_)){
            ret = ring_buffer_read(&ring_buffer_, &packet[0], 1);
            // std::cout << "parse head1" << std::endl;
            if(RING_BUFFER_OK==ret){
                if(DGNSS_SYNC_BYTE_1==packet[0]){
                    packet_byte_count = 1;
                    reading_data_ = true;
                    parser_start_time = ros::Time::now().toSec();
                } else{
                    printf("ParseIncomingData::Received unknown data.\n");
                    printf("%2x ",packet[0]);
                }
            }
        }

        if(0<packet_byte_count){
            if(reading_data_ && ring_buffer_count(&ring_buffer_)>=HEADER_SIZE-1){
                for(i=0; i<HEADER_SIZE-1; i++){
                    ring_buffer_lookahead_read(&ring_buffer_, i, &packet[i+1],1);
                }
                payload_length_ = (((uint16_t) *(packet+3)) << 8) + ((uint16_t) *(packet+2));  // g/
                if(DGNSS_SYNC_BYTE_2==packet[1] && (payload_length_+HEADER_SIZE)<MAX_PACKET_SIZE){
                    packet_byte_count = HEADER_SIZE;
                } else {
                    parser_in_sync = false;
                    packet_byte_count = 0;
                    reading_data_ = false;
                    parser_headers_skipped++;
                    log_info_("Head sync fail, parser_headers skipped.\n");
                }
            } else{
                // check for timeout on incoming packet and report
                double utime = ros::Time::now().toSec();
                if(utime-parser_start_time > packet_timeout){
                    parser_timeouts++;
                    packet_byte_count = 0;
                    reading_data_ = false;
                    parser_in_sync = false;
                    parser_headers_skipped++;
                    log_info_("TIMEOUT POST, parser_headers skipped.\n");
                }
            }
        }
    }

    // header located, get the rest of a packet data
    if(packet_byte_count >= HEADER_SIZE){
        // wait for the rest of the packet to be available in the buffer
        if(ring_buffer_count(&ring_buffer_) >= (HEADER_SIZE - 1 + payload_length_ )) {
            for (i = 0; i < payload_length_; i++) {
                ring_buffer_lookahead_read(&ring_buffer_, HEADER_SIZE - 1 + i, &packet[HEADER_SIZE + i], 1);
            }
            uint8_t crc_cal;
            calculateCheckSum(packet + 2, 1 + payload_length_, &crc_cal);
            if (crc_cal == packet[HEADER_SIZE + payload_length_-1]) {
                packet_byte_count += payload_length_;
                read_timestamp_ = ros::Time::now().toSec();
                uint16_t msgID = packet[HEADER_SIZE + payload_length_-4];
                ParseLog(packet, msgID, packet_byte_count);
                ring_buffer_consume_entries(&ring_buffer_, packet_byte_count-1);
                parser_in_sync = true;
            } else {
                printf("error %2x %2x %d %d\n",packet[2],packet[3],payload_length_,ring_buffer_count(&ring_buffer_));
                if(parser_in_sync){
                    parser_num_bad_checksums++;
                    printf("checksum error.\n");
                }
                parser_in_sync = false;
            }
            // reset the parser
            packet_byte_count = 0;
            reading_data_ = false;
        } else {
            // check for timeout on incoming packet and report
            double utime = ros::Time::now().toSec();
            if(utime-parser_start_time > packet_timeout){
                parser_timeouts++;
                packet_byte_count = 0;
                reading_data_ = false;
                parser_in_sync = false;
                parser_headers_skipped++;
                printf("TIMEOUT POST, parser_headers skipped.\n");
            }
        }
    }
}

void Dgnss::ParseLog(uint8_t *log, size_t logID, uint32_t log_len) {
	try {
		uint16_t payload_length;
		uint8_t num_of_svs;
		uint8_t num_of_channels;
		uint16_t header_length;

		switch (logID) {
            case MSG_ID_IMU:
                ImuMsg_t imu_msg;
                memcpy(&imu_msg, log, log_len);
                if (imu_msg_callback_) {
                    imu_msg_callback_(imu_msg, read_timestamp_);
                }
                break;
            case MSG_ID_GNSS:
                GnssMsg_t gnss_msg;
                memcpy(&gnss_msg, log, log_len);
                if (gnss_msg_callback_) {
                    gnss_msg_callback_(gnss_msg, read_timestamp_);
                }
                break;
            case MSG_ID_GNSS_AJ:
                GnssAjMsg_t gnss_msg_aj;
                memcpy(&gnss_msg_aj, log, log_len);
                if (gnss_aj_msg_callback_) {
                    gnss_aj_msg_callback_(gnss_msg_aj, read_timestamp_);
                }
                break;
            case MSG_ID_INS:
                InsMsg_t ins_msg;
                memcpy(&ins_msg, log, log_len);
                if (ins_msg_callback_) {
                    ins_msg_callback_(ins_msg, read_timestamp_);
                }
                break;
            case MSG_ID_WHEEL_SPEED:
                WheelSpeedMsg_t wheel_speed_msg;
                memcpy(&wheel_speed_msg, log, log_len);
                if (wheel_speed_msg_callback_) {
                    wheel_speed_msg_callback_(wheel_speed_msg, read_timestamp_);
                }
                break;
            case MSG_ID_UWB:
                UwbMsg_t uwb_msg;
                memcpy(&uwb_msg, log, log_len);
                //std::cout<<"Log_Len:"<<log_len<<std::endl;
                if (uwb_msg_callback_) {
                    uwb_msg_callback_(uwb_msg, read_timestamp_);
                }
                break;
            case MSG_ID_UWB_TDOA_POS:
                UwbTdoaPosMsg_t uwb_tdoa_pos_msg;
                memcpy(&uwb_tdoa_pos_msg, log, log_len);
                //std::cout<<"Log_Len:"<<log_len<<std::endl;
                if (uwb_tdoa_pos_msg_callback_) {
                    uwb_tdoa_pos_msg_callback_(uwb_tdoa_pos_msg, read_timestamp_);
                }
                break;
            default:
                break;
		} // end switch (logID)

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error parsing dgnss log: " << e.what();
		log_error_(output.str());
	}
} 

void Dgnss::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {

	try {
        uint32_t crc_sum = 0;
        uint32_t i = 0;
        for(i=0;i<length;i++) {
            crc_sum += in[i];
        }
        *out = crc_sum&0xFF;
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error calculating dgnss checksum: " << e.what();
		log_error_(output.str());
	}
}

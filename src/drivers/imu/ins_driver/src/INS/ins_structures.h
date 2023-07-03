#ifndef DGNSS_STRUCTURES_H
#define DGNSS_STRUCTURES_H
#include "stdint.h"

namespace DGNSS_NAMESPACE {

#define MAX_NOUT_SIZE      (5000)   // Maximum size of a log buffer
#define HEADER_SIZE 4
#define CHECKSUM_SIZE 1
#define DGNSS_SYNC_BYTE_1 0xEB
#define DGNSS_SYNC_BYTE_2 0x90

//ringbuffer
#define MAX_PAYLOAD_SIZE      10000 // Maximum payload of a Ublox log buffer (ALMANACA logs are big!)
#define MAX_PACKET_SIZE       (HEADER_SIZE + MAX_PAYLOAD_SIZE + CHECKSUM_SIZE)

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#define MSG_ID_IMU  0x01  // IMU 00
#define MSG_ID_GNSS 0x11  // GNSS 01
#define MSG_ID_GNSS_AJ 0x12
#define MSG_ID_INS  0x21  // INS 02
#define MSG_ID_WHEEL_SPEED 0x31  // ODOMETRY 03
#define MSG_ID_UWB  0x02  // UWB 04   
#define MSG_ID_UWB_TDOA_POS  0x03


PACK(
        struct ImuMsg_t{
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // 36, length of "the left of this msg"
        
        double utime;
        float gyro[3];
        float acc[3];

        float attitude[3];
        float temperature;
        
        unsigned char type;  // 0x01  IMU
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum;  // crc_sum from "length" to "msg_cnt"
    });

PACK(
        struct GnssMsg_t {
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // 0x70, 112, length of "the left of this msg"
        
        double utime ;
        uint8_t pos_type ; // 0:NONE, 1:SP_2D, 2:SP_3D, 3:DGPS, 4:RTK_float, 5:RTK_fix
        uint8_t meas_enable; // bit0:pos, bit1:vel, bit2:roll, bit3:pitch, bit4:heading, bit5~7:unused
        uint8_t GNSS_mask;   // 0~8 bit for GPS, GLONASS, BDS, Galileo, QZSS, RNSS, SBAS, PSAT
        uint8_t ant_num;
        uint8_t sv_num_tracked;
        uint8_t sv_num_used;
        uint8_t diff_age;
        uint8_t sol_age;

        float ms;          // Milliseconds from the beginning of the GPS reference week
        double longitude;
        double latitude;
        float height;
        float ve;
        float vn;
        float vu;
        float roll;
        float pitch;
        float heading;

        float std_longitude;
        float std_latitude;
        float std_height;
        float std_ve;
        float std_vn;
        float std_vu;
        float std_roll;
        float std_pitch;
        float std_heading;

        float undulation;
        float baseline_length;

        unsigned char type;  // 0x11  GNSS
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum; // crc_sum from "length" to "msg_cnt"
    });

PACK(
        struct GnssAjMsg_t {
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // 0x70, 112, length of "the left of this msg"
        
        double utime ;
        uint8_t pos_type ; // 0:NONE, 1:SP_2D, 2:SP_3D, 3:DGPS, 4:RTK_float, 5:RTK_fix
        uint8_t meas_enable; // bit0:pos, bit1:vel, bit2:roll, bit3:pitch, bit4:heading, bit5~7:unused
        uint8_t GNSS_mask;   // 0~8 bit for GPS, GLONASS, BDS, Galileo, QZSS, RNSS, SBAS, PSAT
        uint8_t ant_num;
        uint8_t sv_num_tracked;
        uint8_t sv_num_used;
        uint8_t diff_age;
        uint8_t sol_age;

        float ms;          // Milliseconds from the beginning of the GPS reference week
        double longitude;
        double latitude;
        float height;
        float ve;
        float vn;
        float vu;
        float roll;
        float pitch;
        float heading;

        float std_longitude;
        float std_latitude;
        float std_height;
        float std_ve;
        float std_vn;
        float std_vu;
        float std_roll;
        float std_pitch;
        float std_heading;

        float undulation;
        float baseline_length;

        unsigned char chan_valid[48];              // chan valid
        unsigned char chan_svid[48];               // chan svid
        float chan_plllock[48];         // chan plllock
        float chan_cn0[48];             // chan cn0
        double chan_pseudoRange[48];    // chan pseudoRange
        double chan_carrPhase[48];      // chan carrPhase

        unsigned char type;  // 0x11  GNSS
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum; // crc_sum from "length" to "msg_cnt"
    });

PACK(
        struct InsMsg_t {
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // 0x38, 56, length of "the left of this msg"
        
        double utime;
        double longitude;
        double latitude;
        float height;
        float ve;
        float vn;
        float vu;
        float roll;
        float pitch;
        float yaw;

        unsigned char type;  // 0x21  INS
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum; // crc_sum from "length" to "msg_cnt"
    });

PACK(
        struct WheelSpeedMsg_t{
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // 16, length of "the left of this msg"
        
        double utime;
        float left_wheel_speed;
        float right_wheel_speed;
        float steering_angle;
        
        unsigned char type;  // 0x31  WheelSpeed
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum;  // crc_sum from "length" to "msg_cnt"
    });


PACK(
        struct UwbTdoaPosMsg_t {
        unsigned char head1; // 0xEB;
        unsigned char head2; // 0x90;
        unsigned short length; // length of "the left of this msg"
        
        double utime;  // us

        unsigned char tagID;

        unsigned int position_x; // unit: cm
        unsigned int position_y; // unit: cm
        unsigned int position_z; // unit: cm

        unsigned char type;  // 0x03  UwbTdoaPos
        unsigned char status;
        unsigned char msg_cnt;
        unsigned char crc_sum; // crc_sum from "length" to "msg_cnt"
    });

    #define  TOF_MAX_ANCHOR_NUMBERS 8
    #pragma pack(1)
        typedef struct
        {
            uint64_t blinkTxTime[TOF_MAX_ANCHOR_NUMBERS];
            uint64_t blinkRxTime[TOF_MAX_ANCHOR_NUMBERS];
            uint64_t respTxTime[TOF_MAX_ANCHOR_NUMBERS];
            uint64_t respRxTimeStamp[TOF_MAX_ANCHOR_NUMBERS];
            uint32_t carrierintegrator[TOF_MAX_ANCHOR_NUMBERS];
        } calculateTofParamter_t;

        typedef struct{
            unsigned char  anchor_serial; //基站的serialID
            unsigned short  range; //标签与基站之间的测距值（cm）
        } UwbAnchorRangeMsg;

        struct UwbMsg_t{

            unsigned char head1; // 0xEB;
            unsigned char head2; // 0x90;
            unsigned short length; //
            
            double time_stamp;

            UwbAnchorRangeMsg anchor_range[8];  //最多有8个基站
            calculateTofParamter_t TofParamter;

            unsigned char type;  // 0x02
            unsigned char status;
            unsigned char msg_cnt;
            unsigned char crc_sum;
        };
    #pragma pack()
} // end namespace

#endif // DGNSS_STRUCTURES_H

#pragma once
#ifndef __PAVO2_COMMON_H__
#define __PAVO2_COMMON_H__
#define BUFFER_SIZE (0x8000)   	//32K
#define FRAME_CAPCITY (0x1) //1
#define PI   (std::acos(-1))  //PI  3.14156
#define Degree2Radians(x) ((x) * PI / 180.0)

#define CMD_SET_GET_INDEX (2)
#define CMD_SET (0xA0)
#define CMD_GET (0xB0)
#define CMD_LEN (0x4)

#define ANGLE_RANGLE_MIN (0)
#define ANGLE_RANGLE_MAX (36000)

enum PAVO2_CMD_INDEX {
	CMD_UNLOCK = 0,
	CMD_OS_VER=1,
	CMD_FW_VER,
	CMD_OS_EX,
    CMD_PN=4,

    CMD_SN, //5
    CMD_ENABLE_DATA=6,  //6
    CMD_ANGLE_RANGLE,
    CMD_ANGLE_RESOLUTION,
	CMD_MOTOR_SPEED=9,

    CMD_INTENSITY_MODE, //10 INTENSITY
	CMD_SLEEP_RESTART=11,
	CMD_RESET,
	CMD_ECHO_MODE,
	CMD_MAC,

	CMD_IP,       //15
	CMD_TCPPORT,
    CMD_UDPPORT, 
	CMD_INDEX_LEN
};
enum DATA_MODE
{
	INTENSITY_DISTANCE=0x0000,
	CHECK_MODE = 0x0001,
	DISTANCE=0x0002,
};
static const unsigned char PAVO2_CMD[][CMD_INDEX_LEN] = {
    { 0xEB, 0x90, 0x00, 0xA0 },  //解锁指令 //0
	{ 0xEB, 0x90, 0x00, 0xA1 },  //系统版本    
	{ 0xEB, 0x90, 0x00, 0xA2 },  //FPGA版本
	{ 0xEB, 0x90, 0x00, 0xA3 },  //系统异常
    { 0xEB, 0x90, 0x00, 0xA4 },  //PN

    { 0xEB, 0x90, 0x00, 0xA5 },  //SN //5
    { 0xEB, 0x90, 0x00, 0xA6 },  //ENABLE_DATA
    { 0xEB, 0x90, 0x00, 0xA7 },  //ANGLE_RANGE
    { 0xEB, 0x90, 0x00, 0xA8 },  //ANGLE_RESOLUTION
    { 0xEB, 0x90, 0x00, 0xA9 },  //MOTOR_SPEED

    { 0xEB, 0x90, 0x00, 0xAA },  //INTENSITY 10
    { 0xEB, 0x90, 0x00, 0xAB },  //SLEEP_START
    { 0xEB, 0x90, 0x00, 0xAC },  //RESET
	{ 0xEB, 0x90, 0x00, 0xAE },  //echo
	{ 0xEB, 0x90, 0x00, 0xE0 },  //mac

    { 0xEB, 0x90, 0x00, 0xE1 },  //SET_IP
    { 0xEB, 0x90, 0x00, 0xE2 },  //SET_TCP_PORT //15
    { 0xEB, 0x90, 0x00, 0xE3 },  //UDP_PORT 
};

static const uint8_t PAVO2_CMD_ARG_LEN[CMD_INDEX_LEN] = {
	0x4,
	0x4,
	0x4,
	0x4,
	0x4,

	0x4,
	0x4,
	0x4,
	0x4,
	0x4,

	0x4,
	0x4,
	0x4,
	0x4,
	0x8,

	0x4,
	0x4,
	0x4,
};

namespace pavo2 {
	const int BUFFER_LEFT_SIZE = 0x8000;
	const unsigned char PROTOCOL_HEAD[8] = { 0xFF,0x53,0x4D,0x49,0x4E,0x49,0x43,0x53 };
	const int PROTOCOL_HEAD_LEN = 8;
	const unsigned char PROTOCOL_VERDION_RESERVER[4] = { 0x01,0x0,0x0,0x0 };
	const int PROTOCOL_VERSION_RESERVER_LEN = 4;
	const int PROTOCOL_DATA_LEN = 4;
	const unsigned char PROTOCOL_END[4] = { 0xFE,0xFE,0xFE,0xFE };
	const int PROTOCOL_END_LEN = 4;

	const int PROTOCOL_LEAST_LEN = PROTOCOL_HEAD_LEN + PROTOCOL_VERSION_RESERVER_LEN + PROTOCOL_DATA_LEN + PROTOCOL_END_LEN;

	const int PACKET_DATA_LEN_LOC = PROTOCOL_HEAD_LEN + PROTOCOL_VERSION_RESERVER_LEN;
	const int PACKET_START_LOC = PROTOCOL_HEAD_LEN + PROTOCOL_VERSION_RESERVER_LEN + PROTOCOL_DATA_LEN;
}

typedef struct PointData
{
	unsigned char point_vec[BUFFER_SIZE];
	uint16_t point_vec_size;
}PointDataDef;

typedef struct CmdData
{
	unsigned char szCmd[128];
	uint16_t cmdSize;
}CmdDataDef;

#endif

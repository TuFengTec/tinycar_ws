#pragma once

#include "sys_types.h"


#if defined(_WIN32)
#pragma pack(1)
#define __DATA_ALIGN__  
#else
#define __DATA_ALIGN__ __attribute__((packed))
#endif

typedef struct pavo_response_scan
{
	uint16_t angle;
	uint16_t distance;
	uint16_t  intensity;
} __DATA_ALIGN__ pavo_response_scan_t;

typedef struct pavo_response_pcd
{
	double x;
	double y;
	double z;  //fixed as 0 for single-laser device
	uint16_t  intensity;
} __DATA_ALIGN__ pavo_response_pcd_t;

typedef struct {
	uint32_t data_tag;
	uint32_t packet_id;
	uint32_t reserver;
	uint16_t data_type;
	uint16_t angle_resolution;
	uint16_t start_angle;
	uint16_t end_angle;
	uint32_t time_stamp;
} __DATA_ALIGN__ data_info_t;


//when the distance is 0 ,set the output value is 0 or 50000(100m) that is out of the valid range 
#define DISTANCE_ZERO 0
#if DISTANCE_ZERO
const int distance_max = 50000; //100m
#else
const int distance_max = 0; //0m
#endif

#if defined(_WIN32)
#pragma pack()
#endif




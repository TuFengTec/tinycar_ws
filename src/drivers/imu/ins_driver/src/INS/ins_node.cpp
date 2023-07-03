#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ins_msg/ImuMsg.h"
#include "ins_msg/GnssMsg.h"
#include "ins_msg/GnssAjMsg.h"
#include "ins_msg/InsMsg.h"
#include "ins_msg/WheelSpeedMsg.h"
#include "ins_msg/UwbMsg.h"
#include "ins_msg/UwbTdoaPosMsg.h"
#include <sensor_msgs/Imu.h>

#include "ins.h"

using namespace DGNSS_NAMESPACE;

#define SET_HIGH_PRIORITY

ins_msg::ImuMsg imu_msg_pub;
ins_msg::GnssMsg gnss_msg_pub;
ins_msg::GnssAjMsg gnss_aj_msg_pub;
ins_msg::InsMsg ins_msg_pub;
ins_msg::WheelSpeedMsg wheel_speed_msg_pub;
ins_msg::UwbMsg uwb_msg_pub;
ins_msg::UwbTdoaPosMsg uwb_tdoa_pos_msg_pub;

ros::Publisher pub_imu;
ros::Publisher pub_imu_ros;
ros::Publisher pub_gnss;
ros::Publisher pub_gnss_aj;
ros::Publisher pub_ins;
ros::Publisher pub_wheel_speed;
ros::Publisher pub_uwb;
ros::Publisher pub_uwb_tdoa_pos;


void ImuMsgHandler(ImuMsg_t &imu_msg, double &timestamp) {
    
    int i = 0;
    static int32_t msg_cnt_delay = 0;
    int32_t msg_cnt_dfif = 0;

    imu_msg_pub.header.stamp = ros::Time::now();
    imu_msg_pub.header.frame_id = "base_link"; //base_link

    imu_msg_pub.utime = (int64_t)(imu_msg.utime * 1e6);
    imu_msg_pub.acc[0] = -imu_msg.acc[1];
    imu_msg_pub.acc[1] = imu_msg.acc[0];
    imu_msg_pub.acc[2] = imu_msg.acc[2];
    imu_msg_pub.gyro[0] = -imu_msg.gyro[1];
    imu_msg_pub.gyro[1] = imu_msg.gyro[0];
    imu_msg_pub.gyro[2] = imu_msg.gyro[2];
    imu_msg_pub.msg_cnt = imu_msg.msg_cnt;
    pub_imu.publish(imu_msg_pub);

    sensor_msgs::Imu imu_out;
    imu_out.header.stamp = ros::Time::now();
    imu_out.header.frame_id = "imu_link"; //
    imu_out.linear_acceleration.x = -imu_msg.acc[1];
    imu_out.linear_acceleration.y = imu_msg.acc[0];
    imu_out.linear_acceleration.z = imu_msg.acc[2];
    // rotate gyroscope
    imu_out.angular_velocity.x = -imu_msg.gyro[1];
    imu_out.angular_velocity.y = imu_msg.gyro[0];
    imu_out.angular_velocity.z = imu_msg.gyro[2];

    imu_out.orientation.x = 0;
    imu_out.orientation.y = 0;
    imu_out.orientation.z = 0;
    imu_out.orientation.w = 1;

    imu_out.orientation_covariance[0] = 1e6;
    imu_out.orientation_covariance[4] = 1e6;
    imu_out.orientation_covariance[8] = 1e6;

    imu_out.angular_velocity_covariance[0] = 1e-6;
    imu_out.angular_velocity_covariance[4] = 1e-6;
    imu_out.angular_velocity_covariance[8] = 1e-6;

    imu_out.linear_acceleration_covariance[0] = 1e-6;
    imu_out.linear_acceleration_covariance[4] = 1e-6;
    imu_out.linear_acceleration_covariance[8] = 1e-6;
    pub_imu_ros.publish(imu_out);

    // msg_cnt_check
    msg_cnt_dfif = imu_msg.msg_cnt - msg_cnt_delay;
    if(msg_cnt_dfif!=1 && msg_cnt_dfif!=-255) {
        std::cout << "WARN: imu msg_cnt discontinuous, msg_cnt_dfif: " << msg_cnt_dfif << std::endl;
    }
    msg_cnt_delay = imu_msg.msg_cnt;

    // printf("\nReceived ImuMsg.\n");
    
}


void GnssMsgHandler(GnssMsg_t &gnss_msg, double &timestamp) {

    gnss_msg_pub.header.stamp = ros::Time::now();
    gnss_msg_pub.header.frame_id = "base_link";

    gnss_msg_pub.utime = (int64_t)(gnss_msg.utime * 1e6);
    gnss_msg_pub.pos_type = gnss_msg.pos_type;
    gnss_msg_pub.meas_enable = gnss_msg.meas_enable;
    gnss_msg_pub.GNSS_mask = gnss_msg.GNSS_mask;
    gnss_msg_pub.ant_num = gnss_msg.ant_num;
    gnss_msg_pub.sv_num_tracked = gnss_msg.sv_num_tracked;
    gnss_msg_pub.sv_num_used = gnss_msg.sv_num_used;
    gnss_msg_pub.diff_age = gnss_msg.diff_age;
    gnss_msg_pub.sol_age = gnss_msg.sol_age;
    gnss_msg_pub.ms = gnss_msg.ms;
    gnss_msg_pub.longitude = gnss_msg.longitude;
    gnss_msg_pub.latitude = gnss_msg.latitude;
    gnss_msg_pub.height = gnss_msg.height;
    gnss_msg_pub.ve = gnss_msg.ve;
    gnss_msg_pub.vn = gnss_msg.vn;
    gnss_msg_pub.vu = gnss_msg.vu;
    gnss_msg_pub.roll = gnss_msg.roll;
    gnss_msg_pub.pitch = gnss_msg.pitch;
    gnss_msg_pub.heading = gnss_msg.heading;
    gnss_msg_pub.std_longitude = gnss_msg.std_longitude;
    gnss_msg_pub.std_latitude = gnss_msg.std_latitude;
    gnss_msg_pub.std_height = gnss_msg.std_height;
    gnss_msg_pub.std_ve = gnss_msg.std_ve;
    gnss_msg_pub.std_vn = gnss_msg.std_vn;
    gnss_msg_pub.std_vu = gnss_msg.std_vu;
    gnss_msg_pub.std_roll = gnss_msg.std_roll;
    gnss_msg_pub.std_pitch = gnss_msg.std_pitch;
    gnss_msg_pub.std_heading = gnss_msg.std_heading;
    gnss_msg_pub.undulation = gnss_msg.undulation;
    gnss_msg_pub.baseline_length = gnss_msg.baseline_length;
    gnss_msg_pub.msg_cnt = gnss_msg.msg_cnt;    

    pub_gnss.publish(gnss_msg_pub);

    printf("\nReceived GnssMsg.\n");
}

void GnssAjMsgHandler(GnssAjMsg_t &gnss_aj_msg, double &timestamp) {

    gnss_aj_msg_pub.header.stamp = ros::Time::now();
    gnss_aj_msg_pub.header.frame_id = "base_link";

    gnss_aj_msg_pub.utime = (int64_t)(gnss_aj_msg.utime * 1e6);
    gnss_aj_msg_pub.pos_type = gnss_aj_msg.pos_type;
    gnss_aj_msg_pub.meas_enable = gnss_aj_msg.meas_enable;
    gnss_aj_msg_pub.GNSS_mask = gnss_aj_msg.GNSS_mask;
    gnss_aj_msg_pub.ant_num = gnss_aj_msg.ant_num;
    gnss_aj_msg_pub.sv_num_tracked = gnss_aj_msg.sv_num_tracked;
    gnss_aj_msg_pub.sv_num_used = gnss_aj_msg.sv_num_used;
    gnss_aj_msg_pub.diff_age = gnss_aj_msg.diff_age;
    gnss_aj_msg_pub.sol_age = gnss_aj_msg.sol_age;
    gnss_aj_msg_pub.ms = gnss_aj_msg.ms;
    gnss_aj_msg_pub.longitude = gnss_aj_msg.longitude;
    gnss_aj_msg_pub.latitude = gnss_aj_msg.latitude;
    gnss_aj_msg_pub.height = gnss_aj_msg.height;
    gnss_aj_msg_pub.ve = gnss_aj_msg.ve;
    gnss_aj_msg_pub.vn = gnss_aj_msg.vn;
    gnss_aj_msg_pub.vu = gnss_aj_msg.vu;
    gnss_aj_msg_pub.roll = gnss_aj_msg.roll;
    gnss_aj_msg_pub.pitch = gnss_aj_msg.pitch;
    gnss_aj_msg_pub.heading = gnss_aj_msg.heading;
    gnss_aj_msg_pub.std_longitude = gnss_aj_msg.std_longitude;
    gnss_aj_msg_pub.std_latitude = gnss_aj_msg.std_latitude;
    gnss_aj_msg_pub.std_height = gnss_aj_msg.std_height;
    gnss_aj_msg_pub.std_ve = gnss_aj_msg.std_ve;
    gnss_aj_msg_pub.std_vn = gnss_aj_msg.std_vn;
    gnss_aj_msg_pub.std_vu = gnss_aj_msg.std_vu;
    gnss_aj_msg_pub.std_roll = gnss_aj_msg.std_roll;
    gnss_aj_msg_pub.std_pitch = gnss_aj_msg.std_pitch;
    gnss_aj_msg_pub.std_heading = gnss_aj_msg.std_heading;
    gnss_aj_msg_pub.undulation = gnss_aj_msg.undulation;
    gnss_aj_msg_pub.baseline_length = gnss_aj_msg.baseline_length;

    for(int i=0;i<48;i++) {
        gnss_aj_msg_pub.chan_valid[i] = gnss_aj_msg.chan_valid[i];
        gnss_aj_msg_pub.chan_svid[i] = gnss_aj_msg.chan_svid[i];
        gnss_aj_msg_pub.chan_plllock[i] = gnss_aj_msg.chan_plllock[i];
        gnss_aj_msg_pub.chan_cn0[i] = gnss_aj_msg.chan_cn0[i];
        gnss_aj_msg_pub.chan_pseudoRange[i] = gnss_aj_msg.chan_pseudoRange[i];
        gnss_aj_msg_pub.chan_carrPhase[i] = gnss_aj_msg.chan_carrPhase[i];
    }

    gnss_aj_msg_pub.msg_cnt = gnss_aj_msg.msg_cnt;


    pub_gnss_aj.publish(gnss_aj_msg_pub);

    printf("\nReceived GnssMsg.\n");
}


void InsMsgHandler(InsMsg_t &ins_msg, double &timestamp) {

    static int32_t msg_cnt_delay = 0;
    int32_t msg_cnt_dfif = 0;

    ins_msg_pub.header.stamp = ros::Time::now();
    ins_msg_pub.header.frame_id = "base_link";

    ins_msg_pub.utime = (int64_t)(ins_msg.utime * 1e6);
    ins_msg_pub.longitude = ins_msg.longitude;
    ins_msg_pub.latitude = ins_msg.latitude;
    ins_msg_pub.height = ins_msg.height;
    ins_msg_pub.ve = ins_msg.ve;
    ins_msg_pub.vn = ins_msg.vn;
    ins_msg_pub.vu = ins_msg.vu;
    ins_msg_pub.roll = ins_msg.roll;
    ins_msg_pub.pitch = ins_msg.pitch;
    ins_msg_pub.yaw = ins_msg.yaw;
    ins_msg_pub.msg_cnt = ins_msg.msg_cnt;

    pub_ins.publish(ins_msg_pub);

    // msg_cnt_check
    msg_cnt_dfif = ins_msg.msg_cnt - msg_cnt_delay;
    if(msg_cnt_dfif!=1 && msg_cnt_dfif!=-255) {
        std::cout << "WARN: ins msg_cnt discontinuous, msg_cnt_dfif: " << msg_cnt_dfif << std::endl;
    }
    msg_cnt_delay = ins_msg.msg_cnt;
}

void WheelSpeedMsgHandler(WheelSpeedMsg_t &wheel_speed_msg, double &timestamp) {

    static int32_t msg_cnt_delay = 0;
    int32_t msg_cnt_dfif = 0;

    wheel_speed_msg_pub.header.stamp = ros::Time::now();
    wheel_speed_msg_pub.header.frame_id = "base_link";

    wheel_speed_msg_pub.utime = (int64_t)(wheel_speed_msg.utime * 1e6);
    wheel_speed_msg_pub.left_wheel_speed = wheel_speed_msg.left_wheel_speed;
    wheel_speed_msg_pub.right_wheel_speed = wheel_speed_msg.right_wheel_speed;
    wheel_speed_msg_pub.steering_angle = wheel_speed_msg.steering_angle;
    wheel_speed_msg_pub.msg_cnt = wheel_speed_msg.msg_cnt;

    pub_wheel_speed.publish(wheel_speed_msg_pub);

    // msg_cnt_check
    msg_cnt_dfif = wheel_speed_msg.msg_cnt - msg_cnt_delay;
    if(msg_cnt_dfif!=1 && msg_cnt_dfif!=-255) {
        std::cout << "WARN: wheel_speed msg_cnt discontinuous, msg_cnt_dfif: " << msg_cnt_dfif << std::endl;
    }
    msg_cnt_delay = wheel_speed_msg.msg_cnt;
}

void UwbMsgHandler(UwbMsg_t &uwb_msg, double &timestamp) {

    //std::cout<<"Get Uwb Type !!!\n";

    
    static int32_t msg_cnt_delay = 0;
    int32_t msg_cnt_dfif = 0;

    uwb_msg_pub.header.stamp = ros::Time::now();
    uwb_msg_pub.header.frame_id = "base_link";


    uwb_msg_pub.utime = (int64_t)(uwb_msg.time_stamp * 1e6);
        
    for(int i=0;i<8;i++) {
        uwb_msg_pub.anchor_serial[i] = uwb_msg.anchor_range[i].anchor_serial;
        uwb_msg_pub.range[i] = uwb_msg.anchor_range[i].range;
        uwb_msg_pub.blinkTxTime[i] = uwb_msg.TofParamter.blinkTxTime[i];
        uwb_msg_pub.blinkRxTime[i] = uwb_msg.TofParamter.blinkRxTime[i];
        uwb_msg_pub.respTxTime[i] = uwb_msg.TofParamter.respTxTime[i];
        uwb_msg_pub.respRxTimeStamp[i] = uwb_msg.TofParamter.respRxTimeStamp[i];
        uwb_msg_pub.carrierintegrator[i] = uwb_msg.TofParamter.carrierintegrator[i];

    }    

    std::cout<<"the uwb status:"<<uwb_msg.status<<std::endl;
    uwb_msg_pub.msg_cnt = uwb_msg.msg_cnt;

    pub_uwb.publish(uwb_msg_pub);

    // msg_cnt_check
    msg_cnt_dfif = uwb_msg.msg_cnt - msg_cnt_delay;
    if(msg_cnt_dfif!=1 && msg_cnt_dfif!=-255) {
        std::cout << "WARN: ins msg_cnt discontinuous, msg_cnt_dfif: " << msg_cnt_dfif << std::endl;
    }
    msg_cnt_delay = uwb_msg.msg_cnt;
}

void UwbTdoaPosMsgHandler(UwbTdoaPosMsg_t &uwb_tdoa_pos_msg, double &timestamp) {

    // std::cout << "Get UwbTdoaPos Type !!!" << std::endl;

    
    static int32_t msg_cnt_delay = 0;
    int32_t msg_cnt_dfif = 0;

    uwb_tdoa_pos_msg_pub.header.stamp = ros::Time::now();
    uwb_tdoa_pos_msg_pub.header.frame_id = "base_link";

    uwb_tdoa_pos_msg_pub.utime = (int64_t)(uwb_tdoa_pos_msg.utime * 1e6);

    uwb_tdoa_pos_msg_pub.tagID = uwb_tdoa_pos_msg.tagID;
    uwb_tdoa_pos_msg_pub.position_x = uwb_tdoa_pos_msg.position_x;
    uwb_tdoa_pos_msg_pub.position_y = uwb_tdoa_pos_msg.position_y;
    uwb_tdoa_pos_msg_pub.position_z = uwb_tdoa_pos_msg.position_z;

    uwb_tdoa_pos_msg_pub.msg_cnt = uwb_tdoa_pos_msg.msg_cnt;

    pub_uwb_tdoa_pos.publish(uwb_tdoa_pos_msg_pub);

    // // msg_cnt_check
    // msg_cnt_dfif = uwb_tdoa_pos_msg.msg_cnt - msg_cnt_delay;
    // if(msg_cnt_dfif!=1 && msg_cnt_dfif!=-255) {
    //     std::cout << "WARN: ins msg_cnt discontinuous, msg_cnt_dfif: " << msg_cnt_dfif << std::endl;
    // }
    // msg_cnt_delay = uwb_tdoa_pos_msg.msg_cnt;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ublox_node");

#ifdef SET_HIGH_PRIORITY
    pid_t pid = getpid();
    std::string cmd("sudo renice -10 " + std::to_string(pid));
    ROS_INFO("set high priority:%s\n",cmd.c_str());
    system(cmd.c_str());
#endif

	std::string port_name;
	int baudrate;
    Dgnss dgnss_;

	ros::param::get("~port_name", port_name);
	ros::param::get("~baudrate", baudrate);

    dgnss_.SetImuMsgCallback(boost::bind(&ImuMsgHandler, _1, _2));
    dgnss_.SetGnssMsgCallback(boost::bind(&GnssMsgHandler, _1, _2));
    dgnss_.SetGnssAjMsgCallback(boost::bind(&GnssAjMsgHandler, _1, _2));
    dgnss_.SetInsMsgCallback(boost::bind(&InsMsgHandler, _1, _2));
    dgnss_.SetWheelSpeedMsgCallback(boost::bind(&WheelSpeedMsgHandler, _1, _2));
    dgnss_.SetUwbMsgCallback(boost::bind(&UwbMsgHandler, _1, _2));
    dgnss_.SetUwbTdoaPosMsgCallback(boost::bind(&UwbTdoaPosMsgHandler, _1, _2));


	if (dgnss_.ValueInitOpenSerial(port_name.c_str(), baudrate) < 0)
	{
		ROS_ERROR("Failed to open dgnss tty port");
		return -1;
	}

	ros::NodeHandle n;
	pub_imu = n.advertise<ins_msg::ImuMsg>("/imu", 2);
	pub_imu_ros = n.advertise<sensor_msgs::Imu>("/imu_ros", 2);
    pub_gnss = n.advertise<ins_msg::GnssMsg>("/gnss", 2);
    pub_gnss_aj = n.advertise<ins_msg::GnssAjMsg>("/gnss_aj", 2);
    pub_ins = n.advertise<ins_msg::InsMsg>("/ins", 2);
    pub_wheel_speed = n.advertise<ins_msg::WheelSpeedMsg>("/wheel_speed", 2);
    pub_uwb =n.advertise<ins_msg::UwbMsg>("/uwb", 2);
    pub_uwb_tdoa_pos =n.advertise<ins_msg::UwbTdoaPosMsg>("/uwb_tdoa_pos", 2);

	while (ros::ok())
	{
        while(1) {
            dgnss_.ReadSerialPort();
            //sleep(0.001);
        }
	}
	return 0;
}


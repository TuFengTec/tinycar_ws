#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <thread>
#include <signal.h>
#include <math.h>
#include <iostream>

// socket can
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
// msgs
#include <geometry_msgs/Twist.h>
// #include <vd_msgs/ChassisInfo.h>
#include <crss_chassis_driver/ChassisInfo.h>

// #include <vd_msgs/SystemStatus.h>

// system status
bool can_node_start = true;
bool crss_system_start = true;

double      param_vc;
double      param_vel_max;
double      param_ang_max;

int socket_;

struct can_frame frame_rx;
struct can_frame frame_tx;

// vd_msgs::SystemStatus status;
crss_chassis_driver::ChassisInfo chassis;

ros::Publisher  motion_pub;     // chassis motion publisher
ros::Publisher  info_pub;       // chassis info publisher

void thread_receive()
{
    while(can_node_start){
        int nbyte = read(socket_, &frame_rx, sizeof(struct can_frame));
        if(nbyte < 0){
            usleep(10);
        }
        else {
            // printf("%03X | ", frame_rx.can_id);
            // for (int i=0; i < frame_rx.can_dlc; i++) printf("%02X ", frame_rx.data[i]);
            // printf("\n");

            uint16_t warning = 0;
            
            /* System status info */
            if(frame_rx.can_id == 0x30){
                chassis.WARMIGM = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                chassis.ERROR   = ((frame_rx.data[3] << 8) & 0xffff) | frame_rx.data[2];
                chassis.MODE    = frame_rx.data[4];
  
                info_pub.publish(chassis);
            }
            // Motion info
            if(frame_rx.can_id == 0x31){
                geometry_msgs::Twist real_motion;
                int16_t vel_cms         = 0;
                double  vel_ms          = 0;
                int16_t angle_deg_100   = 0;
                double  angle_rad       = 0;

                vel_cms         = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                angle_deg_100   = (((frame_rx.data[3] << 8) & 0xffff) | frame_rx.data[2]);
                

                vel_ms      = (double)vel_cms / param_vc;
                angle_rad   = (double)angle_deg_100 / 100.0 * (M_PI / 180.0);
                real_motion.linear.x = vel_ms;
                real_motion.angular.z = angle_rad;

                motion_pub.publish(real_motion);
            }
            // Ultrasonic info
            else if(frame_rx.can_id == 0x37){
                chassis.UL[0] = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                chassis.UL[1] = ((frame_rx.data[3] << 8) & 0xffff) | frame_rx.data[2];
                chassis.UL[2] = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
                chassis.UL[3] = ((frame_rx.data[7] << 8) & 0xffff) | frame_rx.data[6];
            }
            else if(frame_rx.can_id == 0x38){
                chassis.UL[4] = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                chassis.UL[5] = ((frame_rx.data[3] << 8) & 0xffff) | frame_rx.data[2];
                chassis.UL[6] = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
                chassis.UL[7] = ((frame_rx.data[7] << 8) & 0xffff) | frame_rx.data[6];
            }
            // Absolute encoder
            else if(frame_rx.can_id == 0x39){
                chassis.ENCODER = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
            }
            // Battery info
            else if(frame_rx.can_id == 0x3A){
                chassis.Vol = ((frame_rx.data[3] << 8) & 0xffff) | frame_rx.data[2];
                chassis.Curr = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
            }
            // Motor info
            else if(frame_rx.can_id == 0x42){
                chassis.VEL_WHEEL_L = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                chassis.ENC_WHEEL_L = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
            }
            else if(frame_rx.can_id == 0x43){
                chassis.VEL_WHEEL_R = ((frame_rx.data[1] << 8) & 0xffff) | frame_rx.data[0];
                chassis.ENC_WHEEL_R = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
            }
            else if(frame_rx.can_id == 0x52){
                chassis.CURR_WHEEL_L = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
            }
            else if(frame_rx.can_id == 0x53){
                chassis.CURR_WHEEL_R = ((frame_rx.data[5] << 8) & 0xffff) | frame_rx.data[4];
            }
            
            // chassis warning
            if(warning){
                ROS_WARN("data no update");
            }
        }
    }  
}

uint8_t self_cntr;
void ctrl_callback(const geometry_msgs::Twist msg)
{
    // if(status.enable_can){
    //     /* Can control disable handler */

    //     ROS_WARN("[CAN_NODE] Can send disable");
    //     return;
    // }
        
    float vel_plan = msg.linear.x;               //m/s
    float ang_plan = msg.angular.z;              //rad

    if(vel_plan >  param_vel_max) vel_plan =  param_vel_max;
    if(vel_plan < -param_vel_max) vel_plan = -param_vel_max;
    if(ang_plan >  param_ang_max) ang_plan =  param_ang_max;
    if(ang_plan < -param_ang_max) ang_plan = -param_ang_max;

    // ROS_INFO("[cmd_vel] VEL: %f ANGLE: %f", g_ctrl_vel_plan, g_ctrl_ang_plan);

    int16_t vel   = round(vel_plan * param_vc);
    int16_t angle = round(ang_plan * 180.0 / M_PI * 100.0);
    self_cntr += 16;

    frame_tx.can_id = 0x21;
    frame_tx.can_dlc = 8;
    frame_tx.data[0] = (vel) & 0xFF;
    frame_tx.data[1] = (vel >> 8) & 0xFF;
    frame_tx.data[2] = (angle) & 0xFF;
    frame_tx.data[3] = (angle >> 8) & 0xFF;
    frame_tx.data[7] = self_cntr;
    if (write(socket_, &frame_tx, sizeof(frame_tx)) != sizeof(frame_tx)) {
        ROS_ERROR("failed to send can packet");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_node");
    ROS_INFO("CHASSIS DRIVER START!");

    ros::NodeHandle nh;
    ros::Subscriber control_sub = nh.subscribe("/chassis/ctrl_motion", 100, ctrl_callback);     // obstacle detection
    motion_pub  = nh.advertise<geometry_msgs::Twist>("/chassis/real_motion", 100);              // chassis motion publisher
    info_pub    = nh.advertise<crss_chassis_driver::ChassisInfo>("/chassis/chassis_info", 100);             // chassis info publisher

    // Param init
    nh.param("/can_node/vc",        param_vc , 100.00);
    nh.param("/can_node/vel_max",   param_vel_max , 2.00);
    nh.param("/can_node/ang_max",   param_ang_max , 0.61);
    
    ROS_INFO("CAN NODE CONFIG: %.2f %.2f %.2f",param_vc, param_vel_max, param_ang_max);

    // Start socketcan
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Create socket failed");
        exit(-1);
    }
    strcpy(ifr.ifr_name, "can0");
    ROS_INFO("can port %s start", ifr.ifr_name);
    memset(&addr, 0, sizeof(addr));
    
    ioctl(socket_, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind can device failed\n");
            close(socket_);
            exit(-2);
        }
    int loopback = 0;
    setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // start tx thread
    std::thread chassis_receive(thread_receive);

    while (ros::ok()) {
        ros::spin();
        return 0;
    }

    ROS_INFO("can node shutdown...");
    can_node_start = false;

    ros::shutdown();
    return 0;
}

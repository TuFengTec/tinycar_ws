#include "pavo2_driver.h"
#include "std_srvs/Empty.h"
#include <dynamic_reconfigure/server.h>
#include <fstream>
#include <pavo2_ros/pavo2_ros_cfgConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


#define COMP_NODES (36000)
#define CIRCLE_ANGLE (28000.0)
#define START_ANGLE (4000)

#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace pavo2;
pavo2_driver *drv = NULL;

void publish_msg(ros::Publisher *pub,
                 std::vector<pavo_response_scan_t> &nodes_vec, ros::Time start,
                 double scan_time, std::string frame_id, bool inverted,
                 double angle_min, double angle_max, double min_range,
                 double max_range
                 ) 
{
  sensor_msgs::LaserScan scanMsg;
  size_t node_count = nodes_vec.size();
  int counts = node_count * ((angle_max - angle_min) / 280.0f);
  int angle_start = 140 + angle_min;
  int node_start = node_count * (angle_start / 280.0f);

  scanMsg.ranges.resize(counts);
  scanMsg.intensities.resize(counts);

  float range = 0.0;
  float intensity = 0.0;

  for (size_t i = 0; i < counts; i++) 
  {
    range = nodes_vec[node_start].distance * 0.002;
    intensity = nodes_vec[node_start].intensity;
    if ((range > max_range) || (range < min_range)) 
    {
        range = 0.0;
        intensity = 0.0;
    }
    if (!inverted) 
    {
        scanMsg.ranges[i] = range;
        scanMsg.intensities[i] = intensity;
        node_start = node_start + 1;
    } 
    else 
    {
        scanMsg.ranges[counts - 1 - i] = range;
        scanMsg.intensities[counts - 1 - i] = intensity;
        node_start = node_start + 1;
    }
  }
  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Radians((nodes_vec[0].angle-START_ANGLE)/100.0f-140);
  scanMsg.angle_max = Degree2Radians((nodes_vec[counts-1].angle-START_ANGLE)/100.0f-140);
  scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;

  //   std::cout<< " nodes_vec  angle "<< nodes_vec[counts-1].angle  <<  " end "<< nodes_vec[node_start].angle<<" counts "<<counts << std::endl;
  // scanMsg.angle_min = Degree2Radians(angle_min);
  // scanMsg.angle_max = Degree2Radians(angle_max);
  // scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;
  // std::cout<< " nodes_vec  angle "<< scanMsg.angle_max<< " nodes_vec  angle_min "<< scanMsg.angle_min   << std::endl;
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / (double)node_count;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;
  pub->publish(scanMsg);
}

void callback(pavo2_ros::pavo2_ros_cfgConfig &config, uint32_t level) 
{
  ROS_INFO("config lidar's param:angle_min:%.2f  angle_max:%.2f  "
           "range_min:%.2f  range_max:%.2f  frame_id:%s ",
           config.angle_min, 
           config.angle_max, 
           config.range_min,
           config.range_max, 
           config.frame_id.c_str());
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "pavo2_scan_node");
  std::string frame_id, lidar_ip, host_ip, scan_topic;
  int lidar_port, host_port;
  bool inverted, enable_motor,intensity,echo_mode;  
  int angle_resolution,motor_speed;
  int angle_start,angle_end;
  uint8_t motor_speed_get=0;
  double angle_min;
  double angle_max;
  double max_range, min_range;
  
  uint16_t angle_resolution_val;
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh("~");

  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<std::string>("scan_topic", scan_topic, "scan");
  nh_private.param<double>("angle_max", angle_max, 140.00);
  nh_private.param<double>("angle_min", angle_min, -140.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.10);
  nh_private.param<int>("angle_start", angle_start, 4000);
  nh_private.param<int>("angle_end", angle_end, 32000);
  nh_private.param<bool>("intensity", intensity, true);
  nh_private.param<bool>("inverted", inverted,false );
  nh_private.param<bool>("echo_mode", echo_mode, true);
  nh_private.param<int>("motor_speed", motor_speed, 50);
  nh_private.param<int>("angle_resolution", angle_resolution, 32);
 
  nh_private.param<std::string>("lidar_ip", lidar_ip, "10.10.10.121");
  nh_private.param<int>("lidar_port", lidar_port, 2368);
  nh_private.param<std::string>("host_ip", host_ip, "10.10.10.100");
  nh_private.param<int>("host_port", host_port, 2368);
 
  dynamic_reconfigure::Server<pavo2_ros::pavo2_ros_cfgConfig> server;
  dynamic_reconfigure::Server<pavo2_ros::pavo2_ros_cfgConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ros::Publisher scan_pub = nh_private.advertise<sensor_msgs::LaserScan>(scan_topic, 1000);
  std::vector<pavo_response_scan_t> scan_vec;
  drv=new pavo2_driver();
  if(NULL==drv)
  {
    ROS_INFO("new driver err");
  }
  ROS_INFO("IP:%s Port: %d",lidar_ip.c_str(),lidar_port);
  if(drv->pavo2_open(lidar_ip, lidar_port))
  {
    ROS_INFO("open socket succ");
  }
  else
  {
    ROS_INFO("open socket err");
  }
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;


  drv->get_motor_speed(motor_speed_get);
  sleep(1);

  ROS_INFO("laser speed %d ",motor_speed_get);


  ROS_INFO("angle_resolution: %d",angle_resolution);
  drv->set_angle_resolution(angle_resolution);
 
  // drv->set_motor_speed(motor_speed);
  ROS_INFO("angle_start:%d angle_end: %d",angle_start,angle_end);
  drv->set_angle_range(angle_start,angle_end);

  ROS_INFO("intensity:%d ",intensity);
  drv->set_intensity_mode(intensity);
  ROS_INFO("echo_mode:%d ",echo_mode);
  drv->set_echo_mode(echo_mode);
  sleep(1);
  
  drv->enable_data(true);
  ros::Rate rate(motor_speed);
  int count = 0;

  while (ros::ok()) 
  {
    start_scan_time = ros::Time::now();
    bool status = drv->get_scanned_data(scan_vec,150);

    count = scan_vec.size();
    if(count == 0)continue;
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();
    nh.getParam("angle_min", angle_min);
    nh.getParam("angle_max", angle_max);
    nh.getParam("range_min", min_range);
    nh.getParam("range_max", max_range);
    nh.getParam("frame_id", frame_id);

    max_range=100;
    publish_msg(&scan_pub, scan_vec, start_scan_time, scan_duration, frame_id,
                inverted, angle_min, angle_max, min_range, max_range
                );
    ros::spinOnce();
    rate.sleep();
  }
    delete drv;
}

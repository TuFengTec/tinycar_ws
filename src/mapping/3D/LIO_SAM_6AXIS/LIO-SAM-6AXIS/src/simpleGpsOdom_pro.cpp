
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <deque>
#include <mutex>
#include <queue>

#include "gpsTools.hpp"
#include "ros/ros.h"
#include "utility.h"


const std::string  imu_topic = "/convert/imu_data";

class GNSSOdom : public ParamServer {
 public:
  GNSSOdom(ros::NodeHandle &_nh) {
    nh = _nh;
    gpsSub = nh.subscribe(gpsTopic, 100, &GNSSOdom::GNSSCB, this,
                          ros::TransportHints().tcpNoDelay());
    gpsOdomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
    fusedPathPub = nh.advertise<nav_msgs::Path>("/gps_path", 100);

                            
    imuSub = nh.subscribe(imu_topic, 10, &GNSSOdom::imuHandler, this,
                          ros::TransportHints().tcpNoDelay());
  }

 private:
  
  void imuHandler(const sensor_msgs::Imu::ConstPtr &imu_raw) 
  {
    std::lock_guard<std::mutex> lock(mtxImu);
    Imu_Now = *imu_raw;
    getImu = true;
    
  }
  void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    // gps status
    // std::cout << "gps status: " << msg->status.status << std::endl;
    if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
      ROS_ERROR("POS LLA NAN...");
      return;
    }
    double gps_time = msg->header.stamp.toSec();
    Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
    std::cout << "LLA: " << lla.transpose() << std::endl;
    // if(!getImu){
    //   ROS_INFO("we have no  IMU  msg");
    //   return;
    // }
    if (!initXyz ||!getImu) {
      ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude, msg->altitude);

      gtools.lla_origin_ = lla;
      initXyz = true;
      return;
    }

    

    //  convert  LLA to XYZ
    Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
    Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
    // ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

    
    Eigen::Vector3d calib_enu = enu;
    
    {

      std::lock_guard<std::mutex> lock(mtxImu);

      double roll_t, pitch_t, yaw_t;
      tf::Matrix3x3(tf::Quaternion( Imu_Now.orientation.x,
                                    Imu_Now.orientation.y,
                                    Imu_Now.orientation.z,
                                    Imu_Now.orientation.w)).getRPY(roll_t, pitch_t, yaw_t);
      // double yaw_t2 = conver_frame(yaw_t);
      yawQuat = tf::createQuaternionMsgFromYaw(yaw_t);
      // ROS_INFO("GPS yawQuat : %f", yaw_t*180/M_PI);
    }
    

    // pub gps odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    // odom_msg.header.frame_id = odometryFrame;
     odom_msg.header.frame_id = mapFrame;
    odom_msg.child_frame_id = "gps";

    // ----------------- 1. use utm -----------------------
    //        odom_msg.pose.pose.position.x = utm_x - origin_utm_x;
    //        odom_msg.pose.pose.position.y = utm_y - origin_utm_y;
    //        odom_msg.pose.pose.position.z = msg->altitude - origin_al;

    // ----------------- 2. use enu -----------------------
    odom_msg.pose.pose.position.x = calib_enu(0);
    odom_msg.pose.pose.position.y = calib_enu(1);
    odom_msg.pose.pose.position.z = calib_enu(2);
    odom_msg.pose.covariance[0] = msg->position_covariance[0];
    odom_msg.pose.covariance[7] = msg->position_covariance[4];
    odom_msg.pose.covariance[14] = msg->position_covariance[8];
    odom_msg.pose.covariance[1] = lla[0];
    odom_msg.pose.covariance[2] = lla[1];
    odom_msg.pose.covariance[3] = lla[2];
    odom_msg.pose.covariance[4] = msg->status.status;
    // if (orientationReady_)
    odom_msg.pose.pose.orientation = yawQuat;
  
    gpsOdomPub.publish(odom_msg);

    // publish path
    // rospath.header.frame_id = odometryFrame;
    rospath.header.frame_id = mapFrame;
    rospath.header.stamp = msg->header.stamp;
    geometry_msgs::PoseStamped pose;
    pose.header = rospath.header;
    pose.pose.position.x = calib_enu(0);
    pose.pose.position.y = calib_enu(1);
    pose.pose.position.z = calib_enu(2);
    pose.pose.orientation.x = yawQuat.x;
    pose.pose.orientation.y = yawQuat.y;
    pose.pose.orientation.z = yawQuat.z;
    pose.pose.orientation.w = yawQuat.w;
    rospath.poses.push_back(pose);
    fusedPathPub.publish(rospath);
  }

  void ResetOrigin(Eigen::Vector3d &_lla) { gtools.lla_origin_ = _lla; }

  ros::NodeHandle nh;
  GpsTools gtools;

  ros::Publisher gpsOdomPub, fusedPathPub;
  ros::Subscriber gpsSub;
  ros::Subscriber imuSub;

  std::mutex mutexLock;
  std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

  std::mutex  mtxImu;
  sensor_msgs::Imu  Imu_Now;
  bool getImu= false;

  bool orientationReady_ = false;
  bool initXyz = false;
  bool firstYawInit = false;
  Eigen::Vector3d prevPos;
  double yaw = 0.0, prevYaw = 0.0;
  geometry_msgs::Quaternion yawQuat;
  nav_msgs::Path rospath;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam_6axis");
  ros::NodeHandle nh;
  GNSSOdom gps(nh);
  ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
  ros::spin();
  return 1;
}
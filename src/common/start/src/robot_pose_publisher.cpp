#include <thread>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>




class robot_pose_pub
{
public:

  robot_pose_pub()
  {
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom111", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("alined_cloud", 1);
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan",10, &robot_pose_pub::LaserCallBack, this);
    listener.setExtrapolationLimit(ros::Duration().fromSec(0.1));
  }

  void loop()
  {
    ros::Rate rate(100.0);
    while (ros::ok())
    {
      tf::StampedTransform transform;
      try
      {
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        // ros::Time::now()
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        continue;
      }

      nav_msgs::Odometry odom;
      // copy pose to odom msg
      odom.header.stamp = transform.stamp_;
      odom.header.frame_id = "map";
      odom.child_frame_id = "base_link";
      geometry_msgs::TransformStamped ts_msg;
      tf::transformStampedTFToMsg(transform, ts_msg);
      odom.pose.pose.position.x = ts_msg.transform.translation.x;
      odom.pose.pose.position.y = ts_msg.transform.translation.y;
      odom.pose.pose.position.z = ts_msg.transform.translation.z;
      odom.pose.pose.orientation = ts_msg.transform.rotation;

      odom_pub.publish(odom);
      // std::cout<<"pub odom!!!!!"<<std::endl;
      rate.sleep();
    }
  } 

  void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
      // std::cout<<"laserCallBack!!!!!"<<std::endl;
     laser_buffer.push_back(laser_msg);

      tf::StampedTransform transform;
     
      try
      {
        listener.waitForTransform("map", "laser_frame", ros::Time(0), ros::Duration(1.0));
        // ros::Time::now()
        listener.lookupTransform("map", "laser_frame", ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        return;
      }


      auto t_stamp = transform.stamp_.toSec();
      while (!laser_buffer.empty())
      {
          if (laser_buffer.front()->header.stamp.toSec() < t_stamp - 0.1)
          {
              // message too old
              laser_buffer.pop_front();
          }
          else if (laser_buffer.front()->header.stamp.toSec() > t_stamp + 0.1)
          {
              // message too new
              break;
              //return;
          }
          else
          {
              auto msg = laser_buffer.front();
              laser_buffer.pop_front();

              sensor_msgs::PointCloud2 cloud;
              projector_.projectLaser(*laser_msg, cloud);   

              // projector_.transformLaserScanToPointCloud("map", *laser_msg, cloud, listener);
              pcl::PointCloud<pcl::PointXYZ> rawCloud;
              pcl::fromROSMsg(cloud, rawCloud);
              cloud_pub.publish(cloud);
          }
      }
     
}
ros::NodeHandle nh;
ros::Subscriber laser_sub;
laser_geometry::LaserProjection projector_;
ros::Publisher cloud_pub;
ros::Publisher odom_pub;
tf::TransformListener listener;
std::deque<sensor_msgs::LaserScan::ConstPtr> laser_buffer;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_odom_converter");

  robot_pose_pub RPP;
  std::thread publish_thread(&robot_pose_pub::loop,&RPP);
  ros::spin();

  publish_thread.join();
  
  return 0;
}
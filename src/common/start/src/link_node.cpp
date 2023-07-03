#include "utils.h"

class minicarOdom
{
private:
	ros::NodeHandle nh;

	ros::Publisher pubOdom;
	ros::Subscriber subImu;
	ros::Subscriber subWheelVel;

	ros::Subscriber cmd_vel_sub;

	std::mutex imuMt;
	std::mutex wheelMt;

	ins_msg::ImuMsg imuMsg;
	ins_msg::ImuMsg imuMsg2;
	ins_msg::WheelSpeedMsg wheelMsg;
	ins_msg::WheelSpeedMsg wheelMsg2;

	double pose_x;
	double pose_y;
	double pose_th;
	double pose_vx;
	double pose_vy;
	double pose_vth;
	double dt;

	Upload_Data Send_Str;
	serial::Serial Robot_Serial; //声明串口对象

	tf::TransformBroadcaster odom_broadcaster;

public:
	minicarOdom() : nh("~")
	{
		subImu = nh.subscribe<ins_msg::ImuMsg>("/imu", 50, &minicarOdom::imuHandler, this);
		subWheelVel = nh.subscribe<ins_msg::WheelSpeedMsg>("/wheel_speed", 50, &minicarOdom::wheelHandler, this);
		pubOdom = nh.advertise<nav_msgs::Odometry>("/simple_odom", 5);

		cmd_vel_sub = nh.subscribe("/cmd_vel_boost", 100, &minicarOdom::cmd_velCallback, this);

		pose_x = pose_y = pose_th = pose_vx = pose_vy = pose_vth = dt;
		memset(&Send_Str, 0, sizeof(Send_Str));

		printf("size = %d\n",sizeof(Send_Str));
		/**open seril device**/
		try
		{
			Robot_Serial.setPort("/dev/ttyUSB2");
			Robot_Serial.setBaudrate(115200);
			serial::Timeout to = serial::Timeout::simpleTimeout(2000);
			Robot_Serial.setTimeout(to);
			Robot_Serial.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("[ZHOUXUEWEI] Unable to open port ");
		}
		if (Robot_Serial.isOpen())
		{
			ROS_INFO_STREAM("[ZHOUXUEWEI] Serial Port opened");
		}
		else
		{
		}
	}
	void imuHandler(const ins_msg::ImuMsg::ConstPtr &imu_raw)
	{
		std::lock_guard<std::mutex> lck(imuMt);

		imuMsg = *imu_raw;
	}
	void wheelHandler(const ins_msg::WheelSpeedMsg::ConstPtr &wheel_raw)
	{
		std::lock_guard<std::mutex> lck(wheelMt);

		wheelMsg = *wheel_raw;
	}

	void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
	{

		// y= y_min + (y_max-y_min)*(x-x_min)/(x_max - x_min)
		// /** process callback function msgs**/
		float radius = 0.0;

		if ((twist_aux.angular.z == 0) || (twist_aux.linear.x == 0))
		{
			Send_Str.Sensor_Str.ch1 = 127;
		}
		else
		{
			// radius = twist_aux.linear.x / twist_aux.angular.z;
			// Send_Str.Sensor_Str.Z_speed = atan(WHEEL_BASE / radius) * 57.3f;

			radius = twist_aux.linear.x / twist_aux.angular.z;
			double wheel_angle = atan(WHEEL_BASE / radius);  // -pi/4 pi/4  
			Send_Str.Sensor_Str.ch1 = (uint8_t) (0.0 + (0xFF-0x00)*(wheel_angle- Min_angle)/(Max_angle- Min_angle));
		}
		// ROS_INFO("radius=%f   angle=%f",radius, Send_Str.Sensor_Str.Z_speed);
		
		Send_Str.Sensor_Str.ch4 =(uint8_t)((255)-(0.0 +(0xFF-0x00)*(twist_aux.linear.x- Min_velx/(Max_velx-Min_velx))));


		Send_Str.Sensor_Str.Header1 = ch_head;
		Send_Str.Sensor_Str.Header2 = ch_head;
		Send_Str.Sensor_Str.Id = ch_id;
		Send_Str.Sensor_Str.Len = ch_len;
		// Send_Str.Sensor_Str.ch1 = 127; //200;
		
		// Send_Str.Sensor_Str.ch4 = 127;
		uint16_t crc  =  CRC_Table(Send_Str.buffer, 11);
		Send_Str.Sensor_Str.CRC1 = (uint8_t)crc&0xFF ;
		Send_Str.Sensor_Str.CRC2 = (uint8_t)(crc>>8)&0xFF ;
		Robot_Serial.write(Send_Str.buffer, sizeof(Send_Str.buffer));
	}

	void OdomThread()
	{

		ros::Rate rate(100);
		while (ros::ok())
		{
			ros::spinOnce();
			{
				std::lock_guard<std::mutex> lck(imuMt);
				imuMsg2 = imuMsg;
			}
			{
				std::lock_guard<std::mutex> lck(wheelMt);
				wheelMsg2 = wheelMsg;
			}
			pose_vx = (wheelMsg2.left_wheel_speed + wheelMsg2.right_wheel_speed) * 0.5;

			pose_vth = imuMsg2.gyro[2];
			dt = 0.01;
			/* Calculation tf and odom */
			double delta_x = (pose_vx * cos(pose_th) - pose_vy * sin(pose_th)) * dt;
			double delta_y = (pose_vx * sin(pose_th) + pose_vy * cos(pose_th)) * dt;
			double delta_th = pose_vth * dt;
			pose_x += delta_x;
			pose_y += delta_y;
			pose_th += delta_th;

			// PublisherOdom();     //no need odom from here

			rate.sleep();
		}
	}

	void PublisherOdom()
	{

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_th);

		// first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = pose_x;
		odom_trans.transform.translation.y = pose_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";

		// set the position
		odom.pose.pose.position.x = pose_x;
		odom.pose.pose.position.y = pose_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = pose_vx;
		odom.twist.twist.linear.y = pose_vy;
		odom.twist.twist.angular.z = pose_vth;

		if(pose_vx == 0)
		{
			memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
			memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
		}
		else
		{
			memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
			memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
		}

		// publish the message
		pubOdom.publish(odom);
	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "link node");

	minicarOdom MO;

	std::thread sendthread(&minicarOdom::OdomThread, &MO);

	ROS_INFO("\033[1;32m---->\033[0m link_node Started.");

	ros::spin();

	sendthread.join();

	return 0;
}

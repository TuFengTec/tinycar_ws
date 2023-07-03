#include "utils.h"
#include <optional>

class cmdRateBoost 
{
private:
	ros::NodeHandle nh;

	ros::Publisher pubBoostCmd;
	ros::Subscriber cmd_vel_sub;

	std::optional<geometry_msgs::Twist> new_cmdVel; 
	geometry_msgs::Twist pub_twist;
	std::mutex twistMt;
	double lastTimeStamp;

public:
	cmdRateBoost() : nh("~")
	{
		//pubBoostCmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel_boost", 5);
		pubBoostCmd = nh.advertise<geometry_msgs::Twist>("/chassis/ctrl_motion", 5);

		cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &cmdRateBoost::cmd_velCallback, this);


		pub_twist.linear.x = 0;
		pub_twist.linear.y = 0;
		pub_twist.linear.z = 0;

		pub_twist.angular.x = 0;
		pub_twist.angular.y = 0;
		pub_twist.angular.z = 0;
		lastTimeStamp = 0.0;
	}

	void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
	{
		std::lock_guard<std::mutex> lck(twistMt);
		new_cmdVel = twist_aux; 
		lastTimeStamp = ros::Time::now().toSec();
	}

	void boostThread()
	{

		ros::Rate rate(50);
		while (ros::ok())
		{
			ros::spinOnce();
			// PublisherOdom();     //no need odom from here
			rate.sleep();
			{
				std::lock_guard<std::mutex> lck(twistMt);
				if(new_cmdVel.has_value())
				{
					pub_twist = new_cmdVel.value();
					new_cmdVel.reset();
				}
			}
			double nowTimeStamp  = ros::Time::now().toSec();
			if(nowTimeStamp - lastTimeStamp > 3.0 )
			{
				continue;
			}
			pubBoostCmd.publish(pub_twist);
		}
	}

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cmdBoost");

	cmdRateBoost CRB;

	std::thread Boostthread(&cmdRateBoost::boostThread, &CRB);

	ROS_INFO("\033[1;32m---->\033[0m cmdBoost Started.");

	ros::spin();

	Boostthread.join();

	return 0;
}

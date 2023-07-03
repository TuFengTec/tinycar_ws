//如标题表明的那样，该节点只是为了测试图像的传递功能
//cam_test_subscriber.cpp:位于功能包的src文件夹下
//订阅图像话题，并显示图像
#include <ros/ros.h>                         //ros.h包含大部分通用的ROS头文件
#include <opencv2/highgui.hpp>               //opencvGUI组件
#include <opencv2/opencv.hpp>                //opencv.hppp包含大部分通用的OpenCV头文件
#include <image_transport/image_transport.h> //image_transport实现图像传输
#include <cv_bridge/cv_bridge.h>             //cv_bridge提供ROS对OpenCV格式的接口功能

//接收到订阅的消息后，会进入消息回调函数
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        //显示图像
        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);

        //等待30ms
        cv::waitKey(5);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    //初始化ROS节点，节点名：image_listener
    ros::init(argc, argv, "image_shower");

    //创建节点句柄
    ros::NodeHandle nh;

    //定义窗口
    namedWindow("view", cv::WINDOW_NORMAL);

    //图像消息句柄
    image_transport::ImageTransport it(nh);

    //创建一个Subscribe, 订阅名为“camera/image”的画图，注册回调函数imageCallback
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    //循环等待回调函数
    ros::spin();

    // 销毁显示窗口
    cv::destroyWindow("view");
}
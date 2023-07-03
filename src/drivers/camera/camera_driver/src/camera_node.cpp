//camera_node.cpp:位于功能包的src文件夹下
//重点在于将OpenCV和ROS联合使用，用OpenCV来开启摄像头，ROS自带的功能包进行图像传递
#include <ros/ros.h>                         //ros.h包含大部分通用的ROS头文件
#include <opencv2/opencv.hpp>               //opencv.hppp包含大部分通用的OpenCV头文件
#include <opencv2/highgui.hpp>               //opencvGUI组件
#include <opencv2/imgproc.hpp>               //引入cvtColor()
#include <image_transport/image_transport.h> //image_transport实现图像传输
#include <cv_bridge/cv_bridge.h>             //cv_bridge提供ROS对OpenCV格式的接口功能

#include "DHCameraROS.h"
//----------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    //ROS节点初始化，节点名:image_publisher
    ros::init(argc, argv, "image_publisher");

    //创建一个节点句柄，方便节点资源使用和管理
    ros::NodeHandle nh;

    //图像发送接口句柄，用于管理图像接口资源
    image_transport::ImageTransport it(nh);

    //创建一个图像信息发布器，发布名为“camera/image”的topic，消息类型为sensor_msgs::ImagePtr
    //发布图像消息队列的长度只能是1
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    //获取摄像机接口
    //VideoCapture cam("/dev/video0");

    DHCameraROS cam;

    //检测接口是否打开
    if (!cam.isOpened())
    {
        exit(0);
    }
    usleep(100);
    printf("摄像头开启正常\n");
    

    //设置循环频率
    ros::Rate loop_rate(10);      //单位为Hz

    //声明图像帧
    //cv::Mat src_image;
    CameraFrameData cfd;
    char img_file_name[128];
    int cnt = 0;
    while (nh.ok())
    {
        //从摄像头读取图像
        //src_image = cam.getImage(); 

        if(!cam.getImageData(cfd))
        {
            continue;
        }
        
        ros::Time t(cfd.Timestamp);
        cv::Mat src_image = cfd.toCvMat();

        if(src_image.rows==0)
            continue;

        cnt++;
        printf( "Camera: %4d -- %dx%d\n", cnt, (int)src_image.rows, (int)src_image.cols);
        
        //将OpenCV的Mat转变为sensor_msgs
        std_msgs::Header header;
        header.stamp = t;
        header.frame_id = "odom";
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", src_image.clone()).toImageMsg();

        //发布封装完毕的消息
        pub.publish(msg);

        //循环等待回调函数
        ros::spinOnce();

        //按照循环频率延时
        loop_rate.sleep();

        //printf("发布信息\n");
    }
}
//----------------------------------------------------------------------------------------------------
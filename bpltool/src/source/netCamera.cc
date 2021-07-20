/*
 * 新车的前后摄像头的驱动
 * 用Opencv拉流
 */
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <mutex>

/*
 * 安保机器人前后相机的ROS驱动，使用Opencv拉流的方法
 * 依赖于Opencv，cv_bridge，sensor_msgs
 * Cmake中添加以下依赖
 * find_package(OpenCV)
 * include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})
 * find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        )
 * Cmake中添加编译链接
 * add_executable(netCamera src/netCamera.cc)
 * target_link_libraries(netCamera
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})
 */

using namespace std;
using namespace cv;

mutex mut;
queue<Mat> imgQueueFront;
queue<Mat> imgQueueBack;

ros::Publisher pubImgFront;
ros::Publisher pubImgBack;

void getImageStreamFront();
void getImageStreamBack();

string frontTopic;
string backTopic;

thread* netCamGetThreadFront;
thread* netCamGetThreadBack;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "netCam");
    ros::NodeHandle nh("~");
    nh.getParam("frontTopic", frontTopic);
    nh.getParam("backTopic", backTopic);

    pubImgFront = nh.advertise<sensor_msgs::Image>(frontTopic, 1);
    pubImgBack = nh.advertise<sensor_msgs::Image>(backTopic, 1);

    ros::Rate loop(30);

    Mat img;

    if(frontTopic != ""){
        netCamGetThreadFront = new thread(getImageStreamFront);
        (*netCamGetThreadFront).detach();
    }

    if(backTopic != "") {
        netCamGetThreadBack = new thread(getImageStreamBack);
        (*netCamGetThreadBack).detach();
    }


    while (ros::ok())
    {
        if(!imgQueueFront.empty())
        {
            sensor_msgs::Image ROSImgFront = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgQueueFront.back()).toImageMsg();

            ROSImgFront.header.frame_id = "/map";
            ROSImgFront.header.stamp = ros::Time::now();
            pubImgFront.publish(ROSImgFront);
        }
        if(!imgQueueBack.empty())
        {
            sensor_msgs::Image ROSImgBack = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgQueueBack.back()).toImageMsg();

            ROSImgBack.header.frame_id = "/map";
            ROSImgBack.header.stamp = ros::Time::now();
            pubImgBack.publish(ROSImgBack);
        }
        loop.sleep();
    }

    delete netCamGetThreadFront;
    delete netCamGetThreadBack;
    netCamGetThreadFront = nullptr;
    netCamGetThreadBack = nullptr;
    return 0;
}

void getImageStreamFront()
{
    VideoCapture cap;
    Mat tempImg;
    string camAdd = "rtsp://admin:Aa123456@192.168.1.63/h264/ch1/sub/av_stream";
    try
    {
        cap.open(camAdd);
        cap.set(CAP_PROP_BUFFERSIZE, 1);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    }
    catch (int e)
    {
        mut.lock();
        cout << e << endl;
        mut.unlock();
    }
    while(1)
    {
        cap >> tempImg;
        if(tempImg.empty())
            continue;
        mut.lock();
        if(imgQueueFront.size() >= 5)
            imgQueueFront.pop();
        imgQueueFront.push(tempImg);
        mut.unlock();
        usleep(5000);//5ms
    }
}

void getImageStreamBack()
{
    VideoCapture cap;
    Mat tempImg;
    string camAdd = "rtsp://admin:Aa123456@192.168.1.64/h264/ch1/sub/av_stream";
    try
    {
        cap.open(camAdd);
        cap.set(CAP_PROP_BUFFERSIZE, 1);
        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    }
    catch (int e)
    {
        mut.lock();
        cout << e << endl;
        mut.unlock();
    }
    while(1)
    {
        cap >> tempImg;
        if(tempImg.empty())
            continue;
        mut.lock();
        if(imgQueueBack.size() >= 5)
            imgQueueBack.pop();
        imgQueueBack.push(tempImg);
        mut.unlock();
        usleep(5000);//5ms
    }
}


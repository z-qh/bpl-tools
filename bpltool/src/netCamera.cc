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


using namespace std;
using namespace cv;

mutex mut;
queue<Mat> imgQueueFront;
queue<Mat> imgQueueBack;

ros::Publisher pubImgFront;
ros::Publisher pubImgBack;

void getImageStreamFront()
{
    VideoCapture cap;
    Mat tempImg;
    string camAdd = "rtsp://admin:Aa123456@192.168.1.63/h264/ch1/main/av_stream";
    try
    {
        cap.open(0);
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
    string camAdd = "rtsp://admin:Aa123456@192.168.1.64/h264/ch1/main/av_stream";
    try
    {
        cap.open(camAdd);
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

int main(int argc, char** argv)
{

    ros::init(argc, argv, "netCam");
    ros::NodeHandle nh;

    pubImgFront = nh.advertise<sensor_msgs::Image>("/frontImg", 1);
    pubImgBack = nh.advertise<sensor_msgs::Image>("/backImg", 1);

    ros::Rate loop(30);

    Mat img;

    thread netCamGetThreadFront(getImageStreamFront);
    thread netCamGetThreadBack(getImageStreamBack);

    netCamGetThreadFront.detach();
    netCamGetThreadBack.detach();

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
    return 0;
}
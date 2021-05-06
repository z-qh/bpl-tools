#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"

#include <thread>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <mutex>


using namespace std;
using namespace cv;

mutex mut;
queue<Mat> imgQueue;

void getImageStream()
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
    while(cap.isOpened() == false)
    {
        mut.lock();
        cout << "open error, trying!!!" << endl;
        mut.unlock();
        cap.open(camAdd);
    }
    while(1)
    {
        cap >> tempImg;
        if(tempImg.empty())
            continue;
        mut.lock();
        if(imgQueue.size() >= 20)
            imgQueue.pop();
        imgQueue.push(tempImg);
        mut.unlock();
        usleep(10000);//10ms
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "netCam");
    ros::NodeHandle nh;

    ros::Rate loop(30);

    Mat img;

    thread netCamGetThread(getImageStream);

    netCamGetThread.detach();

    while (ros::ok())
    {
        if(!imgQueue.empty())
        {
            img = imgQueue.back();
            imshow("src", img);
            waitKey(10);
        }
        loop.sleep();
    }

    return 0;
}
/*
 * HID协议的继电器-USB模块
 * 为杨佳辉师兄而写
 */
#include "ros/ros.h"
#include "hidEledelay.hpp"

hid ele;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elerelay");
    ros::NodeHandle nh;

    ros::Rate loop(1);
    ele.connect();
    while (ros::ok())
    {
        if(ele.isopen)
            ele.disconnect();
        else
            ele.connect();
        loop.sleep();
    }

    ele.connect();
    return 0;

}
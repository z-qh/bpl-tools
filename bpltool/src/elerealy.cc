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
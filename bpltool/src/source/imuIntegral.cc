
#include "ros/ros.h"
#include "queue"
#include "imuIntegral.hpp"
#include "nav_msgs/Odometry.h"

using namespace std;

queue<sensor_msgs::Imu> imuQ;
nav_msgs::Odometry odom;
ros::Publisher imuPub;

imuIntegral IMU;

void subImuHandle(sensor_msgs::Imu msg)
{
    sensor_msgs::Imu tempImu = msg;
    IMU.integral(tempImu, 1.0 / 200.0);
    msg.header.frame_id = "map";
    imuPub.publish(msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imuIntegral");
    ros::NodeHandle nh;

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, subImuHandle);
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/imuIntegral", 1);
    imuPub = nh.advertise<sensor_msgs::Imu>("/imudata", 1);
    ros::Rate loop(200);
    int count = 0;
    odom.header.frame_id = "map";

    while(ros::ok())
    {
        count++;
        if(count >= 100)
        {
            count = 0;
            odom.pose.pose.position.x = IMU.P(0);
            odom.pose.pose.position.y = IMU.P(1);
            odom.pose.pose.position.z = IMU.P(2);
            odom.pose.pose.orientation.x = IMU.Q.x();
            odom.pose.pose.orientation.y = IMU.Q.y();
            odom.pose.pose.orientation.z = IMU.Q.z();
            odom.pose.pose.orientation.w = IMU.Q.w();
            odomPub.publish(odom);
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
/*
 * IMU的积分和预积分实验——未完成
 */
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

ros::Subscriber subImu;

class imudata{

public:
    geometry_msgs::Quaternion imu_ori;
    double imu_roll;
    double imu_pitch;
    double imu_yaw;


};

void subImuHandle(const sensor_msgs::Imu& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imutest");
    ros::NodeHandle nh;

    subImu = nh.subscribe("/imu/data", 1, subImuHandle);

    ros::spin();

    return 0;
}
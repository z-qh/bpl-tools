#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "gnss_driver/gps_navi_msg.h"
#include "tf/transform_datatypes.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <nav_msgs/Odometry.h>
#include "../include/imu_gnss/gps_ins_msg.h"
#include "sensor_msgs/Imu.h"

ros::Publisher pub;
ros::Publisher pub2;
void handle(imu_gnss::gps_ins_msg msg)
{
    nav_msgs::Odometry odom = msg.odom;
    odom.header.frame_id = "map";
    pub.publish(odom);
}
void handle2(sensor_msgs::Imu msg)
{
    auto temp = msg;
    temp.header.frame_id = "map";
    pub2.publish(temp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imuWatcher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<imu_gnss::gps_ins_msg>("/imu_gnss_state", 1, handle);
    pub = nh.advertise<nav_msgs::Odometry>("/imuWatcher", 1);

    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, handle2);
    pub2 = nh.advertise<sensor_msgs::Imu>("watchImu", 1);
    ros::spin();
}
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud.h"

#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar2image");
    ros::NodeHandle nh;

    ros::Subscriber subLidar = nh.subscribe<sensor_msgs::PointCloud>("/")
}
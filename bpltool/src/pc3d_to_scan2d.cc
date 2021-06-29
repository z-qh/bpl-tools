/*
 * 取三维激光点云最中间的线扫，当做二维激光数据使用
 */
#include "ros/ros.h"
#include "iostream"
#include "vector"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

ros::Subscriber pcSub;
ros::Publisher scanPub;

pcl::PointCloud<pcl::PointXYZI> originCloud;
pcl::PointCloud<pcl::PointXYZI> pubCloud;

void pcSubHandle(const sensor_msgs::PointCloud2& msg)
{
    pubCloud.clear();
    originCloud.clear();

    pcl::fromROSMsg(msg, originCloud);
    sensor_msgs::LaserScan laser;
    laser.header.frame_id = "map";
    laser.header.stamp = ros::Time::now();


    for(size_t i = 0; i < originCloud.points.size(); i++)
    {
        pcl::PointXYZI tempPoint = originCloud.points[i];
        double len = sqrt(tempPoint.x * tempPoint.x + tempPoint.y * tempPoint.y);
        double angle = atan(tempPoint.z / len) * 180.0 / M_PI;
        int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5));
        if(roundedAngle == 1)
        {
            pubCloud.points.push_back(tempPoint);
        }
    }

    sensor_msgs::PointCloud2 scanMsg;
    pcl::toROSMsg(pubCloud, scanMsg);
    scanMsg.header.frame_id = "map";
    scanMsg.header.stamp = ros::Time::now();
    scanPub.publish(scanMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcToScan");
    ros::NodeHandle nh;

    pcSub = nh.subscribe("/rslidar_points", 1, pcSubHandle);
    scanPub = nh.advertise<sensor_msgs::PointCloud2>("/pcScan", 1);

    ros::spin();
    return 0;
}
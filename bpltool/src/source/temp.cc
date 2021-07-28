/*
 * Tips，亿天一个小技巧
 *
 *vector里面嵌套非指针类型的点云将引发内存深浅拷贝错误，使用第二种方式请注意reset
 *vector<pcl::PointCloud<pcl::PointXYZI>> pointCloudVector(2);×
 *vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointCloudVector(2);√
 *
 *
 *直接使用点云，不需要对点云进行初始化，默认构造就好
 *
 *
 *使用点云智能指针，需要对点云进行初始化，像这样
 *pcl::PointCloud<pcl::PointXYZI>::Ptr YESS(new pcl::PointCloud<pcl::PointXYZI>());
 *
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
using namespace std;

//void test01();
//void test02();
//void test03();
//void test04();
//
//void testTimeKdtree();

void icp_test();

ros::Subscriber subOdom;
ros::Subscriber subMap;
ros::Publisher pubOdom;
ros::Publisher pubMap;

void handleOdom(const nav_msgs::Odometry msgs)
{
    nav_msgs::Odometry globalPose(msgs);
    globalPose.header.frame_id = "map";
    globalPose.header.stamp = ros::Time::now();
    globalPose.pose.pose.position.x = msgs.pose.pose.position.z;
    globalPose.pose.pose.position.y = msgs.pose.pose.position.x;
    globalPose.pose.pose.position.z = msgs.pose.pose.position.y;
    pubOdom.publish(globalPose);
}

void handleMap(const sensor_msgs::PointCloud2 inCloud)
{
    pcl::PointCloud<pcl::PointXYZI> globalPose;
    pcl::fromROSMsg(inCloud, globalPose);
    for(int i = 0; i < globalPose.size(); i++)
    {
        pcl::PointXYZI temp;
        temp = globalPose.points[i];
        globalPose.points[i].x = temp.z;
        globalPose.points[i].y = temp.x;
        globalPose.points[i].z = temp.y;
    }
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(globalPose, tempCloud);
    tempCloud.header.frame_id = "map";
    tempCloud.header.stamp = ros::Time::now();
    pubMap.publish(tempCloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odomTrans");
    ros::NodeHandle nh;
    subOdom = nh.subscribe<nav_msgs::Odometry>("/remove_obstacle_odom", 1, handleOdom);
    subMap = nh.subscribe<sensor_msgs::PointCloud2>("/node_position", 1, handleMap);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/hhh", 1);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/qqq", 1);


    ros::spin();

    return 0;
}






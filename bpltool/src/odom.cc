/*
 * 激光里程计——未完成
 */
#include "include/lo_header.h"

//订阅消息
ros::Subscriber subCornerSharp;//尖锐角点
ros::Subscriber subCornerLess;//普通角点
ros::Subscriber subSurfFlat;//平整平面点
ros::Subscriber subSurfLess;//普通平面点
ros::Subscriber subLidarPoints;//全部点云

//发布消息
ros::Publisher pubCornerLast;//最后一帧corner
ros::Publisher pubSurfLast;//最后一帧surf
ros::Publisher pubOdom;//里程计

void subCornerSharpHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
void subCornerLessHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
void subSurfFlatHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
void subSurfLessHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
void subLidarPointsHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    //初始化ROS
    ros::init(argc, argv, "loam_odom");
    ros::NodeHandle nh;

    subCornerSharp = nh.subscribe("/lidar_corner_sharp", 1, subCornerSharpHandle);
    subCornerLess = nh.subscribe("/lidar_corner", 1, subCornerLessHandle);
    subSurfFlat = nh.subscribe("/lidar_surf_flat", 1, subSurfFlatHandle);
    subSurfLess = nh.subscribe("/lidar_surf", 1, subSurfLessHandle);
    subLidarPoints = nh.subscribe("/lidar_cloud", 1, subLidarPointsHandle);

    ros::spin();
    return 0;
}
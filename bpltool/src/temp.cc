#include "include/loam_header.h"

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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Rate loop(1);

    while (ros::ok())
    {
        loop.sleep();
    }

    return 0;
}
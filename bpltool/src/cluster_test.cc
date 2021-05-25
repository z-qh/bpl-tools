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

#include "CVC.hpp"

#include "Eigen/Eigen"

#include "pcl/common/common.h"

#include <iostream>
#include <algorithm>
#include <iterator>

using namespace std;
/*
 * 两点之间绘制直线，100个点每米
 */
pcl::PointCloud<pcl::PointXYZI> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B)
{
    pcl::PointCloud<pcl::PointXYZI> result;
    result.clear();
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    double nums = distance * 100;
    for(int i = 0; i < nums; i++)
    {
        pcl::PointXYZI tempPoint;
        tempPoint.x = B.x + diffX / nums * i;
        tempPoint.y = B.y + diffY / nums * i;
        tempPoint.z = B.z + diffZ / nums * i;
        result.push_back(tempPoint);
    }
    return result;
}
/*
 * 根据XYZ最大最小值画框
 */
pcl::PointCloud<pcl::PointXYZI> createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max)
{
    pcl::PointCloud<pcl::PointXYZI> frame;
    frame.clear();
    pcl::PointXYZ p[8];
    //取出八个点坐标
    p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
    p[1].x = max[0];  p[1].y = max[1];  p[1].z = min[2];
    p[2].x = max[0];  p[2].y = min[1];  p[2].z = max[2];
    p[3].x = max[0];  p[3].y = min[1];  p[3].z = min[2];
    p[4].x = min[0];  p[4].y = max[1];  p[4].z = max[2];
    p[5].x = min[0];  p[5].y = max[1];  p[5].z = min[2];
    p[6].x = min[0];  p[6].y = min[1];  p[6].z = max[2];
    p[7].x = min[0];  p[7].y = min[1];  p[7].z = min[2];
    //绘制一共是二个线条
    frame += createLineCLoud(p[0], p[1]);
    frame += createLineCLoud(p[2], p[3]);
    frame += createLineCLoud(p[4], p[5]);
    frame += createLineCLoud(p[6], p[7]);

    frame += createLineCLoud(p[0], p[2]);
    frame += createLineCLoud(p[1], p[3]);
    frame += createLineCLoud(p[4], p[6]);
    frame += createLineCLoud(p[5], p[7]);

    frame += createLineCLoud(p[0], p[4]);
    frame += createLineCLoud(p[2], p[6]);
    frame += createLineCLoud(p[1], p[5]);
    frame += createLineCLoud(p[3], p[7]);

    return frame;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Rate loop(1);

    //读出PCD中的点云
    pcl::PointCloud<pcl::PointXYZI> fram105;
    pcl::io::loadPCDFile("/home/qh/frame105.pcd", fram105);

    //构建分类器
    vector<PointAPR> papr;
    calculateAPR(fram105, papr);
    unordered_map<int, Voxel> hvoxel;
    build_hash_table(papr, hvoxel);
    vector<int> cluster_index = CVC(hvoxel, papr);
    vector<int> cluster_id;
    most_frequent_value(cluster_index, cluster_id);

    //统计分割种类
    map<int, int> classes;
    for(int i = 0; i < cluster_index.size(); i++)
    {
        map<int, int>::iterator it = classes.find(cluster_index[i]) ;
        //如果不存在key则添加新的key，如果存在key则原有的值加一
        if(it == classes.end())
            classes.insert(make_pair(cluster_index[i], 1));
        else
            it->second++;
    }
    cout << classes.size() << endl;
    //去除点数量比较少的聚类
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); )
    {
        if(it->second <= 100)
            classes.erase(it++);
        else
            it++;
    }
    cout << classes.size() << endl;
    //保存点云，计算每个聚类的框框，然后画出来
    pcl::PointCloud<pcl::PointXYZI> saveCloud;
    //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
    {
        //保存聚类的点云
        pcl::PointCloud<pcl::PointXYZI> tempCloud;
        tempCloud.clear();
        for(int i = 0; i < cluster_index.size(); i++)
        {
            if(cluster_index[i] == it->first)
            {
                tempCloud.push_back(fram105[i]);
            }
        }
        //计算这部分点云的PCA并画框框
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(tempCloud, min, max);
        tempCloud += createFrameCloud(min, max);

        //将点云保存到总点云中
        saveCloud += tempCloud;
    }
    cout << saveCloud.size() << endl;

    //保存
    pcl::io::savePCDFileASCII("/home/qh/seg.pcd", saveCloud);

    while (ros::ok())
    {
        loop.sleep();
    }

    return 0;
}
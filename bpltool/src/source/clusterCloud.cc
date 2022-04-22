/*
 * CVC聚类并画框显示，针对单一文件处理和在线处理
 */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "CVC.hpp"

#include "Eigen/Eigen"

#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"

#include <iostream>
#include <algorithm>
#include <iterator>

using namespace std;

//launch 参数
string mode;
string inputTopic;
string outputTopic;
string pcdInputFile;
string pcdOutputFile;
string pcaPre;

ros::Subscriber subCloud;//订阅点云
ros::Publisher pubCloud;//发布点云
ros::Publisher pubOneCloud;

void subCloudHandle(const sensor_msgs::PointCloud2& msg);
void outlineHandle(string inputFile, string outputName);
pcl::PointCloud<pcl::PointXYZ> createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max);
pcl::PointCloud<pcl::PointXYZ> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    nh.getParam("mode", mode);
    transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if(mode.find("outline") != -1)
    {
        nh.getParam("pcdInputFile", pcdInputFile);
        nh.getParam("pcdOutputFile", pcdOutputFile);
        cout << "input" << pcdInputFile << endl;
        cout << "output" << pcdOutputFile << endl;
        outlineHandle(pcdInputFile, pcdOutputFile);
    }
    else if(mode.find("online") != -1)
    {
        nh.getParam("pcaPre", pcaPre);
        nh.getParam("inputTopic", inputTopic);
        nh.getParam("outputTopic", outputTopic);
        cout << "inputTopic" << inputTopic << endl;
        cout << "outputTopic" << outputTopic << endl;
        subCloud = nh.subscribe(inputTopic, 1, subCloudHandle);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);
        pubOneCloud = nh.advertise<sensor_msgs::PointCloud2>(pcaPre, 1);
        ros::spin();
    }
    else
    {
        cout << "invalid param" << endl;
        return 0;
    }
    return 0;
}

/*
 * 粗略确定点云中心
 */
pcl::PointXYZ getCloudCenter(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    double sumX, sumY, sumZ;
    for(int i = 0; i < cloud.size(); i++)
    {
        sumX += cloud.points[i].x;
        sumY += cloud.points[i].y;
        sumZ += cloud.points[i].z;
    }
    pcl::PointXYZ result;
    result.x = sumX / cloud.size();
    result.y = sumY / cloud.size();
    result.z = sumZ / cloud.size();
    cout << result.x << result.y << result.z << endl;
    return result;
}


/*
 * 两点之间绘制直线，100个点每米
 */
pcl::PointCloud<pcl::PointXYZ> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B)
{
    pcl::PointCloud<pcl::PointXYZ> result;
    result.clear();
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    double nums = distance * 100;
    for(int i = 0; i < nums; i++)
    {
        pcl::PointXYZ tempPoint;
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
pcl::PointCloud<pcl::PointXYZ> createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max)
{
    pcl::PointCloud<pcl::PointXYZ> frame;
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

/*
 * 离线单帧处理
 */
void outlineHandle(string inputFile, string outputName)
{
    //读出PCD中的点云
    pcl::PointCloud<pcl::PointXYZ> frame;
    pcl::io::loadPCDFile(inputFile, frame);
    //构建分类器
    vector<PointAPR> papr;
    calculateAPR(frame, papr);
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
    pcl::PointCloud<pcl::PointXYZ> saveCloud;
    //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
    {
        //保存聚类的点云
        pcl::PointCloud<pcl::PointXYZ> tempCloud;
        tempCloud.clear();
        for(int i = 0; i < cluster_index.size(); i++)
        {
            if(cluster_index[i] == it->first)
            {
                tempCloud.push_back(frame[i]);
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
    pcl::io::savePCDFileASCII(outputName, saveCloud);
}

/*
 * 在线处理回调
 */
void subCloudHandle(const sensor_msgs::PointCloud2& msg)
{
    pcl::PointCloud<pcl::PointXYZ> originCloud;
    pcl::fromROSMsg(msg, originCloud);

    //耗时统计
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //构建分类器
    vector<PointAPR> papr;
    calculateAPR(originCloud, papr);
    unordered_map<int, Voxel> hvoxel;
    build_hash_table(papr, hvoxel);
    vector<int> cluster_index = CVC(hvoxel, papr);
    vector<int> cluster_id;
    most_frequent_value(cluster_index, cluster_id);

    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " time cost: " << (time_used.count() * 1000) << " ms." << endl;

    //统计聚类结果
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

    //去除点数量比较少的聚类
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); )
    {
        if(it->second <= 500)
            classes.erase(it++);
        else
            it++;
    }
    cout << classes.size() << endl;
    //保存点云，计算每个聚类的框框，然后画出来
    pcl::PointCloud<pcl::PointXYZ> saveCloud;

    //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
    {
        //保存聚类的点云
        pcl::PointCloud<pcl::PointXYZ> tempCloud;
        tempCloud.clear();
        for(int i = 0; i < cluster_index.size(); i++)
        {
            if(cluster_index[i] == it->first)
            {
                tempCloud.push_back(originCloud[i]);
            }
        }

        //画框
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(tempCloud, min, max);
        tempCloud += createFrameCloud(min, max);

        //将点云保存到总点云中
        saveCloud += tempCloud;
    }

    //发布点云
    sensor_msgs::PointCloud2 ROSCloud;
    pcl::toROSMsg(saveCloud, ROSCloud);
    ROSCloud.header.frame_id = "map";
    ROSCloud.header.stamp = ros::Time::now();
    pubCloud.publish(ROSCloud);
}
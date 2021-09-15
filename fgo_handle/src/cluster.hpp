#include "CVC.hpp"
#include "map"
#include "vector"
#include "string"
#include "ros/ros.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "random"



using namespace std;

class cluster{
    class rgb{
    public:
        int r,g,b;
        rgb(int R, int G, int B):r(R),g(B),b(B){}
    };
    const rgb rgbTable[7] = {rgb(255, 0, 0), rgb(255, 165, 0), rgb(255, 255, 0), rgb(0, 255, 0), rgb(0, 0, 255),
                             rgb(255, 228, 196), rgb(139, 90, 0)};
private:
    std::default_random_engine rng;
    bool isCon = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lastCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nowCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud;
    map<int, int> lastClasses;
    map<int, int> nowClasses;
public:
    void clusterPoints(pcl::PointCloud<pcl::PointXYZI>& originCloud, map<int, int>& classes, pcl::PointCloud<pcl::PointXYZRGB>& showCloud)
    {
        //构建分类器
        vector<PointAPR> papr;
        calculateAPR(originCloud, papr);
        unordered_map<int, Voxel> hvoxel;
        build_hash_table(papr, hvoxel);
        vector<int> cluster_index = CVC(hvoxel, papr);
        vector<int> cluster_id;
        most_frequent_value(cluster_index, cluster_id);
        //统计聚类结果
        classes.clear();
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
            if(it->second <= 30)
                classes.erase(it++);
            else
                it++;
        }
        showCloud.clear();
        //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
        for(map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
        {
            //保存聚类的点云
            pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
            tempCloud.clear();
            static std::uniform_int_distribution<int> rand(1, 7);
            int colorIndex = rand(rng);
            for(int i = 0; i < cluster_index.size(); i++)
            {
                if(cluster_index[i] == it->first)
                {
                    pcl::PointXYZRGB tempP;
                    tempP.x = originCloud[i].x;
                    tempP.y = originCloud[i].y;
                    tempP.z = originCloud[i].z;
                    tempP.r = rgbTable[colorIndex].r;
                    tempP.g = rgbTable[colorIndex].g;
                    tempP.b = rgbTable[colorIndex].b;
                    tempCloud.push_back(tempP);
                }
            }
            //将点云保存到总点云中
            showCloud += tempCloud;
        }
    }
    void statusEstimate()
    {

    }
    cluster()
    {
        lastCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        nowCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        sourceCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        targetCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    void run(sensor_msgs::PointCloud2 msg, nav_msgs::Odometry::Ptr odom, sensor_msgs::PointCloud2& showMsg)
    {
        if(isCon) {
            *lastCloud = *nowCloud;
            lastClasses = nowClasses;
            *targetCloud = *sourceCloud;
        }
        pcl::fromROSMsg(msg, *nowCloud);
        vector<int> nonIndex;
        pcl::removeNaNFromPointCloud(*nowCloud, *nowCloud, nonIndex);

        //分类
        pcl::PointCloud<pcl::PointXYZRGB> show;
        clusterPoints(*nowCloud, nowClasses, show);
        pcl::toROSMsg(show, showMsg);
        showMsg.header = msg.header;
        if(!isCon){
            isCon = true;
            return;
        }
        //估计

    }
};



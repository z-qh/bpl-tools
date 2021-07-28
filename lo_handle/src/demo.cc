#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "iostream"
#include "string"

#include "feature.hpp"
#include "map.hpp"
#include "seg.hpp"
#include "trans.hpp"

using namespace std;

ros::Publisher TTTT;
ros::Publisher RRRR;
ros::Publisher EEEE;



//string sourceBagPath = "/home/qh/points.bag";
string sourceBagPath = "/media/qh/QH/Linuxbak/2020_9_16/2018-05-15-10-27-40_C.bag";


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lo_handle");
    ros::NodeHandle nh("~");
    nh.getParam("sourceBagPath", sourceBagPath);

    TTTT = nh.advertise<sensor_msgs::PointCloud2>("/TTTT", 1);
    RRRR = nh.advertise<nav_msgs::Odometry>("/RRRR", 1);
    EEEE = nh.advertise<nav_msgs::Odometry>("/EEEE", 1);

    ImageProjection IP;
    FeatureAssociation FA;
    TransformFusion TF;
    mapOptimization MAP(true);
    mapOptimization MAP2(false);

    rosbag::Bag bag;
    bag.open(sourceBagPath, rosbag::bagmode::Read);
    vector<string> topics{"/rslidar_points"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;

    sensor_msgs::PointCloud2 segCloud;
    detail_msgs::cloud_info segCloudInfo;
    sensor_msgs::PointCloud2 outCloud;

    sensor_msgs::PointCloud2 last_corner;
    sensor_msgs::PointCloud2 last_surf;
    sensor_msgs::PointCloud2 last_out;
    nav_msgs::Odometry odom;

    nav_msgs::Odometry mapOdom;
    nav_msgs::Odometry mapOdomNoLoop;

    ros::Rate loop(1);
    int frameCunt = 0;
    //耗时统计
    chrono::steady_clock::time_point t1;

    for(it = view.begin(); it != view.end(); it++)
    {
        string nowTopic = (*it).getTopic();
        if(nowTopic == topics[0])
        {
            static bool initFlag = true;
            if(initFlag) {
                t1 = chrono::steady_clock::now();
                cout << "file open success!" << endl;
                initFlag = false;
            }
            sensor_msgs::PointCloud2::ConstPtr tempCloud = (*it).instantiate<sensor_msgs::PointCloud2>();
            if(tempCloud == nullptr)
            {
                cout << "error" << endl;
                continue;
            }
            sensor_msgs::PointCloud2 rosCloud = *tempCloud;
            frameCunt++;
            IP.cloudHandler(rosCloud, segCloud, segCloudInfo, outCloud);
            FA.run(segCloud, segCloudInfo, outCloud, nullptr, last_corner, last_surf, last_out, odom);
            MAP.run(last_corner, last_surf, last_out, odom, nullptr, mapOdom);
            MAP2.run(last_corner, last_surf, last_out, odom, nullptr, mapOdomNoLoop);
            //TTTT.publish(segCloud);
            //TTTT.publish(last_corner);
            //RRRR.publish(odom);
            EEEE.publish(mapOdomNoLoop);
            RRRR.publish(mapOdom);
        }
    }

    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " time cost: " << (time_used.count() * 1000) << " ms." << endl;
    cout << " handle "<< frameCunt << " frames" << endl;
    cout << " pre frame uesd" << (time_used.count() * 1000.0 / frameCunt) << "ms" << endl;
    bag.close();
    return 0;
}


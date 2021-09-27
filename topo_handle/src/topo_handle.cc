#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "iostream"
#include "string"

#include "intersection.hpp"

#include "obstacle.hpp"

#include "topomap.hpp"

using namespace std;

string sourceBagPath = "/media/qh/QH/rosbagpack/kitti_odometry_00.bag";
string sourceBagPathTopic = "/kitti/velo/pointcloud";
string sourceBagPathOdomTopic = "/kitti/camera_left_poses";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "topo_handle");
    ros::NodeHandle nh("~");

    ros::Publisher groundCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/ground", 1);
    ros::Publisher nogroundCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/noground", 1);
    ros::Publisher showSegCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/showSegCloud", 1);
    ros::Publisher removeOBSCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/remove", 1);
    ros::Publisher topoMapGlobalPub = nh.advertise<sensor_msgs::PointCloud2>("/topomap", 1);
    ros::Publisher cloudLocalMapPub = nh.advertise<sensor_msgs::PointCloud2>("/localMap", 1);

    Intersection IS;
    obstacle OBS;
    topomap TOPOMAP;

    sensor_msgs::PointCloud2 IS_cluster_cloud;
    sensor_msgs::PointCloud2 IS_ground_cloud;
    sensor_msgs::PointCloud2 IS_noground_cloud;
    sensor_msgs::PointCloud2 IS_intersecction;
    nav_msgs::Odometry IS_cluster_odom;
    sensor_msgs::PointCloud2 IS_show_cloud;

    sensor_msgs::PointCloud2 OBS_remove_obstacle_cloud;
    nav_msgs::Odometry OBS_remove_odom;

    sensor_msgs::PointCloud2 topoMapGlobal;
    sensor_msgs::PointCloud2 cloud_local_map;


    rosbag::Bag bag;
    bag.open(sourceBagPath, rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back(sourceBagPathTopic);
    topics.push_back(sourceBagPathOdomTopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;

    int frameCunt = 0;
    //耗时统计
    chrono::steady_clock::time_point t1;
    chrono::steady_clock::time_point t2;
    chrono::duration<double> time_used;

    sensor_msgs::PointCloud2 BagCloud;
    nav_msgs::Odometry BagOdom;
    double cloudTime = -1;
    double odomTime = -1;
    for(it = view.begin(); it != view.end(); it++)
    {
        frameCunt++;

        sensor_msgs::PointCloud2ConstPtr tempPtrA = (*it).instantiate<sensor_msgs::PointCloud2>();
        if(tempPtrA != nullptr) {
            BagCloud = *tempPtrA;
            tempPtrA = nullptr;
            //cout << "cloud in " << frameCunt << " frame time: " << setprecision(14) << BagCloud.header.stamp.toSec() << endl;
            cloudTime = BagCloud.header.stamp.toSec();
        }

        nav_msgs::OdometryConstPtr tempPtrB = (*it).instantiate<nav_msgs::Odometry>();
        if(tempPtrB != nullptr) {
            BagOdom = *tempPtrB;
            tempPtrB = nullptr;
            //cout << "odom in " << frameCunt << " frame time: " << setprecision(14) << BagCloud.header.stamp.toSec() << endl;
            odomTime = BagOdom.header.stamp.toSec();
        }

        if(cloudTime == odomTime && odomTime != -1)
        {
            cloudTime = -1;
            odomTime = -1;
            /*
             * 分割
             */
            //cout << "IS input time cloud:" << setprecision(14) << BagCloud.header.stamp.toSec() << endl;
            //cout << "IS input time odom:" << setprecision(14) << BagOdom.header.stamp.toSec() << endl;
            t1 = chrono::steady_clock::now();//耗时统计
            IS.run(BagCloud, BagOdom,
                   IS_cluster_cloud,IS_intersecction,IS_cluster_odom,
                   IS_ground_cloud,IS_noground_cloud,IS_show_cloud);
            t2 = chrono::steady_clock::now();//耗时统计
            time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计

            //groundCloudPub.publish(IS_ground_cloud);
            nogroundCloudPub.publish(IS_noground_cloud);
            //showSegCloudPub.publish(IS_show_cloud);
            //getchar();
            //cout << frameCunt << " IS frame succeed : " << (time_used.count() * 1000) << " ms" << endl;
            /*
             * 去障
             */
            //cout << "OBS input time cloud:" << setprecision(14) << IS_cluster_cloud.header.stamp.toSec() << endl;
            //cout << "OBS input time odom:" << setprecision(14) << IS_cluster_odom.header.stamp.toSec() << endl;
            //cout << "OBS input time odom:" << setprecision(14) << IS_ground_cloud.header.stamp.toSec() << endl;
            t1 = chrono::steady_clock::now();//耗时统计
            OBS.run(IS_cluster_cloud, IS_ground_cloud, IS_cluster_odom,
                    OBS_remove_obstacle_cloud, OBS_remove_odom);
            t2 = chrono::steady_clock::now();//耗时统计
            time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计
            //show answers here
            //getchar();
            removeOBSCloudPub.publish(OBS_remove_obstacle_cloud);
            //cout << frameCunt << " OBS frame succeed : " << (time_used.count() * 1000) << " ms" << endl;
            /*
             *几何地图
             */
            //cout << "TOPOMAP input time cloud:" << setprecision(14) << OBS_remove_obstacle_cloud.header.stamp.toSec() << endl;
            //cout << "TOPOMAP input time odom:" << setprecision(14) << OBS_remove_odom.header.stamp.toSec() << endl;
            //cout << "TOPOMAP input time intersection:" << setprecision(14) << IS_intersecction.header.stamp.toSec() << endl;
            //cout << "TOPOMAP input size:" << OBS_remove_obstacle_cloud.data.size() << endl;
            t1 = chrono::steady_clock::now();//耗时统计
            TOPOMAP.run(OBS_remove_obstacle_cloud, OBS_remove_odom, IS_intersecction, topoMapGlobal, cloud_local_map, true);
            t2 = chrono::steady_clock::now();//耗时统计
            time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计
            //show answers here
            //getchar();
            topoMapGlobalPub.publish(topoMapGlobal);
            cloudLocalMapPub.publish(cloud_local_map);
            //cout << frameCunt << " TOPOMAP frame succeed : " << (time_used.count() * 1000) << " ms" << endl;
        }
    }

    bag.close();
    return 0;
}


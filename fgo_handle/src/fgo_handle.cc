#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "iostream"
#include "string"

#include "feature.hpp"
#include "map.hpp"
#include "seg.hpp"
#include "trans.hpp"
#include "scmap.hpp"
#include "cluster.hpp"


using namespace std;

ros::Publisher mapOdomPub;
ros::Publisher mapOdomNoLoopPub;
ros::Publisher SCmapOdomPub;
ros::Publisher odomTempPub;
ros::Publisher cloudTempPub;

//typedef sensor_msgs::PointCloud2ConstPtr SPCP;
std::vector<nav_msgs::Odometry> hisOdom;//qh add for debug
//qh add for debug
void wirte_odom_to_file(std::vector<nav_msgs::Odometry> msg, std::string path)
{
    std::ofstream file;
    file.open(path);
    if(!file)
        return;
    int odomSize = msg.size();
    file << odomSize << std::endl;
    for(int i = 0; i < odomSize; i++)
    {
        file << setprecision(12) << msg[i].pose.pose.position.x << " "
        << msg[i].pose.pose.position.y << " "
        << msg[i].pose.pose.position.z << " "
        << msg[i].pose.pose.orientation.x << " "
        << msg[i].pose.pose.orientation.y << " "
        << msg[i].pose.pose.orientation.z << " "
        << msg[i].pose.pose.orientation.w << " "
        << std::endl;
    }
    file.close();
}
//qh add for debug
void read_odom_from_file(std::vector<nav_msgs::Odometry>& msgs, std::string path)
{
    std::ifstream file;
    file.open(path);
    if(!file)
        return;
    msgs.clear();
    int odomSize = 0;
    file >> odomSize;
    for(int i = 0; i < odomSize; i++)
    {
        nav_msgs::Odometry tempOdom;
        file >> tempOdom.pose.pose.position.x 
        >> tempOdom.pose.pose.position.y
        >> tempOdom.pose.pose.position.z
        >> tempOdom.pose.pose.orientation.x 
        >> tempOdom.pose.pose.orientation.y 
        >> tempOdom.pose.pose.orientation.z 
        >> tempOdom.pose.pose.orientation.w; 
        msgs.push_back(tempOdom);
    }
    file.close();
}
//qh add for debug

string sourceBagPath = "/media/qh/QH/rosbagpack/kitti_odometry_00.bag";
string sourceBagPathTopic = "/kitti/velo/pointcloud";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fgo_handle");
    ros::NodeHandle nh("~");
    nh.getParam("sourceBagPath", sourceBagPath);
    nh.getParam("sourceBagPathTopic", sourceBagPathTopic);

    mapOdomPub = nh.advertise<nav_msgs::Odometry>("/mapOdomPub", 1);
    //mapOdomNoLoopPub = nh.advertise<nav_msgs::Odometry>("/mapOdomNoLoopPub", 1);
    //SCmapOdomPub = nh.advertise<nav_msgs::Odometry>("/SCmapOdomPub", 1);
    ros::Publisher clusterPub = nh.advertise<sensor_msgs::PointCloud2>("/Cluster", 1);

    odomTempPub = nh.advertise<nav_msgs::Odometry>("/odomTempPub", 1);
    cloudTempPub = nh.advertise<sensor_msgs::PointCloud2>("/cloudTempPub", 1);

    ImageProjection IP;
    FeatureAssociation FA;
    TransformFusion TF;
    mapOptimization MAP(false);
    cluster FGO;
    //mapOptimization MAP2(true);
    //SCmapOptimization SCMAP(true);

    rosbag::Bag bag;
    bag.open(sourceBagPath, rosbag::bagmode::Read);
    if(!bag.isOpen())
    {
        cout << "bag file load faild!" << endl;
        return 0;
    }
    vector<string> topics;
    topics.push_back(sourceBagPathTopic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;
    //地面分割
    sensor_msgs::PointCloud2 segCloud;
    detail_msgs::cloud_info segCloudInfo;
    sensor_msgs::PointCloud2 outCloud;
    sensor_msgs::PointCloud2 noground_cloud;
    //特征提取
    sensor_msgs::PointCloud2 last_corner;
    sensor_msgs::PointCloud2 last_surf;
    sensor_msgs::PointCloud2 last_out;
    nav_msgs::Odometry odom;
    //里程计
    nav_msgs::Odometry mapOdom;
    nav_msgs::Odometry mapOdomNoLoop;
    nav_msgs::Odometry SCmapOdom;
    //聚类
    sensor_msgs::PointCloud2 clusterCloud;
    nav_msgs::Odometry::Ptr FGOOdom(new nav_msgs::Odometry());

    int frameCunt = 0;//帧计数器
    //耗时统计
    chrono::steady_clock::time_point t1;
    //遍历bag
    for(it = view.begin(); it != view.end(); it++)
    {
        string nowTopic = (*it).getTopic();
        if(nowTopic == topics[0])
        {
            static bool initFlag = true;//初始化标志位，统计时间
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
            //分割地面并处理
            IP.cloudHandler(rosCloud, segCloud, segCloudInfo, outCloud, noground_cloud);
            cloudTempPub.publish(segCloud);
            //帧间里程计
            FA.run(segCloud, segCloudInfo, outCloud, nullptr, last_corner, last_surf, last_out, odom);
            odomTempPub.publish(odom);
            //雷达里程计
            MAP.run(last_corner, last_surf, last_out, odom, nullptr, mapOdom);
            //MAP2.run(last_corner, last_surf, last_out, odom, nullptr, mapOdomNoLoop);
            //SCMAP.run(last_corner, last_surf, last_out, odom, nullptr, SCmapOdom);
            //发布消息
            //mapOdomPub.publish(mapOdom);
            //mapOdomNoLoopPub.publish(mapOdomNoLoop);
            //SCmapOdomPub.publish(SCmapOdom);
            FGO.run(noground_cloud, FGOOdom, clusterCloud);
            clusterPub.publish(clusterCloud);
        }
    }
    wirte_odom_to_file(hisOdom, "/home/qh/B1HallCorrected.txt");
    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " time cost: " << (time_used.count() * 1000) << " ms." << endl;
    cout << " handle "<< frameCunt << " frames" << endl;
    cout << " pre frame uesd" << (time_used.count() * 1000.0 / frameCunt) << "ms" << endl;
    bag.close();
    return 0;
}


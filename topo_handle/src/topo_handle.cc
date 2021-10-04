#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "std_msgs/String.h"
#include "iostream"
#include "string"

#include "intersection.hpp"

#include "obstacle.hpp"

#include "topomap.hpp"
#include "recall.hpp"

using namespace std;

bool isRecall;

bool continueFlag = true;
void keyControlSubHandle(std_msgs::String cmd);
//string sourceBagPath = "/media/qh/QH/rosbagpack/kitti_odometry_00.bag";
//string sourceBagPathTopic = "/kitti/velo/pointcloud";
//string sourceBagPathOdomTopic = "/kitti/camera_left_poses";


string sourceBagPathTopic = "/os_cloud_node/points";
//recall
string sourceBagPath = "/media/qh/YES/2021-08-30-16-12-25L.bag";
//string sourceBagPath = "/media/qh/YES/2021-08-30-19-17-22L.bag";
//else
//string sourceBagPath = "/media/qh/YES/2021-08-30-18-06-30L.bag";

vector<pair<double,nav_msgs::Odometry>> odomList;
vector<int> odomStatueList;
void readFileOdom(){
    ifstream fileIn;
    if(isRecall){
        fileIn.open("/home/qh/robot_ws/map/2021-08-30-16-12-25L/gtFittingA.txt");
        // fileIn.open("/home/qh/robot_ws/map/2021-08-30-19-17-22L/gtFittingA.txt");
    }else{
        fileIn.open("/home/qh/robot_ws/map/2021-08-30-18-06-30L/gtFittingA.txt");
    }
    int odomSize = 0;
    double tempNone;
    fileIn >> tempNone >> tempNone >> tempNone;
    fileIn >> odomSize;
    odomList.resize(odomSize);
    odomStatueList.resize(odomSize);
    for(int i = 0; i < odomSize; i++){
        double R(0), P(0), Y(0);
        double time(0);
        int flag(0);
        fileIn >> odomList[i].first;
        odomList[i].second.header.stamp =  ros::Time().fromSec(odomList[i].first);
        odomList[i].second.header.frame_id = "camera_init";
        fileIn >>
        odomList[i].second.pose.pose.position.x >>
        odomList[i].second.pose.pose.position.y >>
        odomList[i].second.pose.pose.position.z;
        fileIn >> R >> P >> Y;
        fileIn >> flag;
        fileIn >> tempNone >> tempNone >> tempNone >> tempNone;
        tf::Quaternion tempQ = tf::createQuaternionFromRPY(R, P, Y);
        odomList[i].second.pose.pose.orientation.x = tempQ.x();
        odomList[i].second.pose.pose.orientation.y = tempQ.y();
        odomList[i].second.pose.pose.orientation.z = tempQ.z();
        odomList[i].second.pose.pose.orientation.w = tempQ.w();
        odomStatueList[i] = flag;
    }
    fileIn.close();
    cout << "odom size open succeed : " << odomSize << " poses!" << endl;
}
bool isSim(int state){
    if(state == 0 || state == -1 || state == 1)
        return true;
    else
        return false;
}

double valueNow = 0.40;

int posesCount = 30;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "topo_handle");
    ros::NodeHandle nh("~");
    nh.getParam("valueNow", valueNow);
    nh.getParam("isRecall", isRecall);


    readFileOdom();
    ros::Subscriber keyControlSub = nh.subscribe<std_msgs::String>("/keyboardControlCmd", 1, keyControlSubHandle);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher groundCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/ground", 1);
    ros::Publisher nogroundCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/noground", 1);
    ros::Publisher showSegCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/showSegCloud", 1);
    ros::Publisher removeOBSCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/remove", 1);
    ros::Publisher topoMapGlobalPub = nh.advertise<sensor_msgs::PointCloud2>("/topomap", 1);
    ros::Publisher cloudLocalMapPub = nh.advertise<sensor_msgs::PointCloud2>("/localMap", 1);
    ros::Publisher topoMapAllNodeCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/nodeCloud", 1);

    ros::Publisher oldNodePosiPub = nh.advertise<sensor_msgs::PointCloud2>("/oldNodePosi", 1);
    ros::Publisher newNodePosiPub = nh.advertise<sensor_msgs::PointCloud2>("/newNodePosi", 1);
    ros::Publisher recallCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/recallCloud", 1);

    Intersection IS;
    obstacle OBS;
    stringstream ss;
    ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << valueNow;
    string pathNow = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/"+ss.str()+"-15/";
    string pathLoad = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/"+ss.str()+"-15/";
    string pathLoadOut = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/"+ss.str()+"3.0-Recall/";
    topomap TOPOMAP(pathNow, valueNow, 15);
    topomapRecall TopoMapRecall(pathLoad, pathLoadOut, 3.0);
    cout << pathNow << endl;
    char keytemp = getchar();
    if(keytemp == 'q'){
        exit(0);
    }

    sensor_msgs::PointCloud2 IS_cluster_cloud;
    sensor_msgs::PointCloud2 IS_ground_cloud;
    sensor_msgs::PointCloud2 IS_noground_cloud;
    sensor_msgs::PointCloud2 IS_intersecction;
    nav_msgs::Odometry IS_cluster_odom;
    sensor_msgs::PointCloud2 IS_show_cloud;

    sensor_msgs::PointCloud2 OBS_remove_obstacle_cloud;
    nav_msgs::Odometry OBS_remove_odom;

    sensor_msgs::PointCloud2 topoMapGlobal;
    sensor_msgs::PointCloud2 topoMapAllNodeCloudMsg;
    pcl::PointCloud<PointType>::Ptr topoMapAllNodeCloud(new pcl::PointCloud<PointType>());
    bool genNewNodeFlag = false;
    sensor_msgs::PointCloud2 cloud_local_map;

    sensor_msgs::PointCloud2 oldNodePosi;
    sensor_msgs::PointCloud2 newNodePosi;
    sensor_msgs::PointCloud2 recallCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr recallAllCoud(new pcl::PointCloud<pcl::PointXYZI>());


    rosbag::Bag bag;
    bag.open(sourceBagPath, rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back(sourceBagPathTopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;

    int frameCunt = 0;
    //耗时统计
    chrono::steady_clock::time_point t1;
    chrono::steady_clock::time_point t2;
    chrono::duration<double> time_used;

    pcl::PointCloud<PointType>::Ptr reviceCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr lsCloud(new pcl::PointCloud<PointType>());

    sensor_msgs::PointCloud2 BagCloud;
    nav_msgs::Odometry BagOdom;
    double cloudTime = -1;
    for(it = view.begin(); it != view.end(); it++)
    {
        frameCunt++;
        while(!continueFlag){
            char key = getchar();
            if(key == 'q'){
                it = view.end();
                bag.close();
                reviceCloud->clear();
                lsCloud->clear();
                return 0;
            }
            cout << "waiting for action!" << endl;
        }

        sensor_msgs::PointCloud2ConstPtr tempPtrA = (*it).instantiate<sensor_msgs::PointCloud2>();
        if(tempPtrA != nullptr) {
            BagCloud = *tempPtrA;
            std_msgs::Header tempH = BagCloud.header;
            tempPtrA = nullptr;
            //cout << "cloud in " << frameCunt << " frame time: " << setprecision(14) << BagCloud.header.stamp.toSec() << endl;
            cloudTime = BagCloud.header.stamp.toSec();
            //getchar();
            //cout << "lidar time " << cloudTime << " pose time " << odomList[posesCount].first << endl;

            pcl::fromROSMsg(BagCloud, *reviceCloud);
            lsCloud->points.resize(reviceCloud->points.size());
            for(int i = 0; i < reviceCloud->points.size(); i++){
                lsCloud->points[i].x = reviceCloud->points[i].y;
                lsCloud->points[i].y = -reviceCloud->points[i].x;
                lsCloud->points[i].z = reviceCloud->points[i].z;
                lsCloud->points[i].intensity = reviceCloud->points[i].intensity;
            }
            pcl::toROSMsg(*lsCloud, BagCloud);
            BagCloud.header = tempH;
            BagCloud.header.frame_id = "camera_init";
            lsCloud->clear();
            reviceCloud->clear();
        }
        if(cloudTime != -1 &&
           abs(cloudTime - odomList[posesCount].first) < 0.15)
        {
            cloudTime = -1;
            /*
             * 分割
             */
            //cout << "IS input time cloud:" << setprecision(14) << BagCloud.header.stamp.toSec() << endl;
            //t1 = chrono::steady_clock::now();//耗时统计
            IS.run(BagCloud, odomList[posesCount].second,
                   IS_cluster_cloud,IS_intersecction,IS_cluster_odom,
                   IS_ground_cloud,IS_noground_cloud,IS_show_cloud);
            //t2 = chrono::steady_clock::now();//耗时统计
            //time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计

            groundCloudPub.publish(IS_ground_cloud);
            nogroundCloudPub.publish(IS_noground_cloud);
            showSegCloudPub.publish(IS_show_cloud);
            //getchar();
            //cout << frameCunt << " IS frame succeed : " << (time_used.count() * 1000) << " ms" << endl;
            /*
             * 去障
             */
            //cout << "OBS input time cluster cloud:" << setprecision(14) << IS_cluster_cloud.header.stamp.toSec() << endl;
            //cout << "OBS input time odom:" << setprecision(14) << IS_cluster_odom.header.stamp.toSec() << endl;
            //cout << "OBS input time ground cloud:" << setprecision(14) << IS_ground_cloud.header.stamp.toSec() << endl;
            //t1 = chrono::steady_clock::now();//耗时统计
            OBS.run(IS_cluster_cloud, IS_ground_cloud, IS_cluster_odom,
                    OBS_remove_obstacle_cloud, OBS_remove_odom);
            //t2 = chrono::steady_clock::now();//耗时统计
            //time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计
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
            //t1 = chrono::steady_clock::now();//耗时统计
            if(isRecall){
                sensor_msgs::PointCloud2 tempMsgs;
                bool isRecall = false;
                TopoMapRecall.run(OBS_remove_obstacle_cloud, OBS_remove_odom, IS_intersecction,
                                  oldNodePosi, newNodePosi, recallCloud, isRecall);
                if(isRecall){
                    isRecall = false;
                    pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());
                    pcl::fromROSMsg(recallCloud, *tempCloud);
                    *recallAllCoud += *tempCloud;
                    tempCloud->clear();
                }
                pcl::toROSMsg(*recallAllCoud, tempMsgs);
                tempMsgs.header.frame_id = "camera_init";

                oldNodePosiPub.publish(oldNodePosi);
                newNodePosiPub.publish(newNodePosi);
                recallCloudPub.publish(tempMsgs);
            }else{
                sensor_msgs::PointCloud2 tempMsgs;
                TOPOMAP.run(OBS_remove_obstacle_cloud, OBS_remove_odom, IS_intersecction, topoMapGlobal, cloud_local_map, isSim(odomStatueList[posesCount]), tempMsgs, genNewNodeFlag);
                if(genNewNodeFlag){
                    genNewNodeFlag = false;
                    pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());
                    pcl::fromROSMsg(tempMsgs, *tempCloud);
                    *topoMapAllNodeCloud += *tempCloud;
                    tempCloud->clear();
                }
                pcl::toROSMsg(*topoMapAllNodeCloud, topoMapAllNodeCloudMsg);
                topoMapAllNodeCloudMsg.header.frame_id = "camera_init";

                topoMapAllNodeCloudPub.publish(topoMapAllNodeCloudMsg);
                topoMapGlobalPub.publish(topoMapGlobal);
                cloudLocalMapPub.publish(cloud_local_map);
            }
            //t2 = chrono::steady_clock::now();//耗时统计
            //time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);//耗时统计
            //show answers here
            //getchar();
            //cout << frameCunt << " TOPOMAP frame succeed : " << (time_used.count() * 1000) << " ms" << endl;
            posesCount++;
        }
    }

    spinner.start();
    reviceCloud->clear();
    lsCloud->clear();
    bag.close();
    return 0;
}
void keyControlSubHandle(std_msgs::String cmd){
    if(cmd.data == "true")
        continueFlag = true;
    else
        continueFlag = false;
}


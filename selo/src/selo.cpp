#define PCL_NO_PRECOMPILE

#include <cmath>
#include <vector>
#include <string>
#include "odom/common.h"
#include "odom/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include "opencv2/opencv.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "queue"
#include "cvc/cvc.h"


using namespace std;


int N_SCANS = 0;
double MINIMUM_RANGE = 0.1;

double timeCloudRaw = 0;
double tiemOdomRaw = 0;

nav_msgs::Odometry laserOdomIn;

pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudPole(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudUpdate(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>());


queue<nav_msgs::Odometry> laserOdomQueue;
queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;

void publishCloud();
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
void handleCloudMsgs(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

void laserOdomHandler(nav_msgs::Odometry odom);
void handleOdomMsgs(nav_msgs::Odometry odom);

pcl::PointCloud<PointType>::Ptr mergeCloud(pcl::PointCloud<PointType>::Ptr dest, pcl::PointCloud<PointType>::Ptr source, vector<bool>&interest);
pcl::PointCloud<PointType>::Ptr mergeCloudAndUpdateInfo(pcl::PointCloud<PointType>::Ptr dest, pcl::PointCloud<PointType>::Ptr source, vector<bool>&interest);
vector<bool> getInterest(string file_name="/home/qh/kitti_data_interest.yaml");

ros::Publisher pubPoleCloud;
ros::Publisher pubUpdatedCloud;
ros::Publisher pubClusterCloud;

vector<bool> interest_label;
bool gt = false;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "selo_temp");
    ros::NodeHandle nh;

    nh.param<int>("scan_line", N_SCANS, 16);
    
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);
    nh.param<bool>("gt", gt, false);
    string odomTopic;
    if(gt){
        odomTopic = "/gt";
    }else{
        odomTopic = "/aft_mapped_to_init";
    }
    interest_label = getInterest();

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    ros::Subscriber subLaserOdom;
    subLaserOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, laserOdomHandler);

    pubPoleCloud = nh.advertise<sensor_msgs::PointCloud2>("/pole_cloud_registered", 10);
    pubUpdatedCloud = nh.advertise<sensor_msgs::PointCloud2>("/updated_cloud_registered", 10);
    pubClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud_registered", 10);
    ros::Rate loop(100);

    while(ros::ok()){
        ros::spinOnce();
        if( laserOdomQueue.empty() || laserCloudMsgQueue.empty()) continue;
        while(abs( laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec() ) > 0.05 ){
            if( laserOdomQueue.empty() || laserCloudMsgQueue.empty()) break;
            if( laserOdomQueue.front().header.stamp.toSec() < laserCloudMsgQueue.front()->header.stamp.toSec())laserOdomQueue.pop();
            else laserCloudMsgQueue.pop();
        }
        if( laserOdomQueue.empty() || laserCloudMsgQueue.empty()) continue;
        if( abs( laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec() ) < 0.05 ){
            
            handleCloudMsgs(laserCloudMsgQueue.front());
            handleOdomMsgs(laserOdomQueue.front());
            laserCloudMsgQueue.pop();
            laserOdomQueue.pop();
            Eigen::Matrix4f transN = Eigen::Matrix4f::Identity();
            Eigen::Quaternionf qN(laserOdomIn.pose.pose.orientation.w, 
                                    laserOdomIn.pose.pose.orientation.x,
                                    laserOdomIn.pose.pose.orientation.y,
                                    laserOdomIn.pose.pose.orientation.z);
            Eigen::Vector3f tN(laserOdomIn.pose.pose.position.x,laserOdomIn.pose.pose.position.y,laserOdomIn.pose.pose.position.z);
            transN.block<3,3>(0,0) = Eigen::Matrix3f(qN);
            transN.block<3,1>(0,3) = tN;
            if(gt){
                Eigen::Matrix4f gauss = Eigen::Matrix4f::Identity();
                gauss<<0,-1, 0, 0,
                    0, 0,-1, 0,
                    1, 0, 0, 0,
                    0, 0, 0, 1;
                transN = gauss.transpose() * transN * gauss;
            }
            pcl::transformPointCloud(*laserCloudIn, *laserCloudIn, transN);

            TicToc t_merge;
            mergeCloud(laserCloudPole, laserCloudIn, interest_label);
            printf("merge time %f ms\n", t_merge.toc());

            publishCloud();

        }

        loop.sleep();
    }
    if(!laserCloudPole->empty()) pcl::io::savePCDFileASCII("/home/qh/cluster.pcd", *laserCloudPole);
    return 0;

}

void removeClosedPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserOdomHandler(nav_msgs::Odometry odom){
    laserOdomQueue.push(odom);
    while(laserOdomQueue.size() > 10) laserOdomQueue.pop();
}

void handleOdomMsgs(nav_msgs::Odometry odom){
    tiemOdomRaw = odom.header.stamp.toSec();
    laserOdomIn = odom;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    laserCloudMsgQueue.push(laserCloudMsg);
    while(laserCloudMsgQueue.size() > 10) laserCloudMsgQueue.pop();
}

void handleCloudMsgs(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    timeCloudRaw = laserCloudMsg->header.stamp.toSec();
    vector<uint32_t> cloudLable;
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
    removeClosedPointCloud(*laserCloudIn, *laserCloudIn, MINIMUM_RANGE);
}
// 栅格化管理点云归并，距离已存点云相隔0.1m以上才会发生归并，这种归并主要发生在：
//    1.旧观测点因观测位置改变，区域点云得到了形状地补全，此栅格注册方式忠实原始点云信息，不进行下采样。如一棵大树在初次观测为一面圆弧，全方位观测后得到一个圆柱
//    2.新的观点因观测位置有变，区域点云进行了拓张，此栅格注册方法会完全保留新拓张点云。如街道上转向发生时，被建筑遮挡的物体注册到系统中并产生用于以后的维护的先验
//    3.旧观测点因环境变化，区域点云得到了形状补全。
//    需要改进：
//      关于类别的管理还不到位，目前只能是针对兴趣类别进行提取，兴趣类别太多提供给实例化维护的时候无法分辨类别，会导致实例化维护困难，
//      目前栅格依赖位姿，在激光里程计先验较差的情况下会发生较大的归并失败，需要处理这种异常
//      0.1米的栅格距离锁死导致近处细节点云无法更加细致的表达，远处的点云在更大误差的加持下脱离实体无法注册，可以使用半径渐增的圆环状栅格进行归并，需要根据当前坐标进行调整后面再改
//      大部分时间消耗在于index的过程，index使用map作为基础使用红黑树，也可以使用unorder_map，这两种方式都可以针对点云的索引进行优化实现，后面再搞

pcl::PointCloud<PointType>::Ptr mergeCloud(pcl::PointCloud<PointType>::Ptr dest, pcl::PointCloud<PointType>::Ptr source, vector<bool>&interest)
{   
    double lx=0.1,ly=0.1,lz=0.1;
    Eigen::Vector4f minpt,maxpt;
    pcl::getMinMax3D(*source, minpt, maxpt);

    map<int, std::pair<int, int> > dest_map; //key:voxel_i - val:dest_i,source_i
    
    int width = std::ceil( (maxpt(0)-minpt(0)) / lx );
    int len   = std::ceil( (maxpt(1)-minpt(1)) / ly );
    int hei   = std::ceil( (maxpt(2)-minpt(2)) / ly );
    int perh  = width * len;

    static double maxVol = 0;
    double nowVol = (maxpt(0)-minpt(0))*(maxpt(1)-minpt(1))*(maxpt(2)-minpt(2));
    if(nowVol > maxVol) maxVol = nowVol;
    TicToc index_time;
    for(size_t i = 0; i < dest->size(); ++i){
        auto&p=dest->points[i];
        if( p.x < minpt(0) || p.y < minpt(1) || p.z < minpt(2) || 
            p.x > maxpt(0) || p.y > maxpt(1) || p.z > maxpt(2) ) continue;
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh; 
        dest_map[ind] = std::make_pair(i, -1);
    }

    for(size_t i = 0; i < source->size(); ++i){
        auto&p=source->points[i];
        // if(!interest[p.label&0xFFFF]) continue;
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        auto it = dest_map.find(ind);
        if(it != dest_map.end()) it->second.second = i;
        else dest_map[ind] = std::make_pair(-1, i);
    }
    printf("\tindex time %f\n", index_time.toc());

    TicToc add_time;
    int add_count=0;
    for(auto&p:dest_map){
        if(p.second.second != -1){
            dest->push_back(source->points[p.second.second]);
            add_count++;
        }
    }
    printf("\tadd time %f num: %d\n", add_time.toc(), add_count);


    return dest;
}


//
pcl::PointCloud<PointType>::Ptr mergeCloud(pcl::PointCloud<PointType>::Ptr dest, pcl::PointCloud<PointType>::Ptr source)
{   
    double lx=0.1,ly=0.1,lz=0.1;
    Eigen::Vector4f minpt,maxpt;
    pcl::getMinMax3D(*source, minpt, maxpt);

    map<int, std::pair<int, int> > dest_map; //key:voxel_i - val:dest_i,source_i
    
    int width = std::ceil( (maxpt(0)-minpt(0)) / lx );
    int len   = std::ceil( (maxpt(1)-minpt(1)) / ly );
    int hei   = std::ceil( (maxpt(2)-minpt(2)) / ly );
    int perh  = width * len;

    static double maxVol = 0;
    double nowVol = (maxpt(0)-minpt(0))*(maxpt(1)-minpt(1))*(maxpt(2)-minpt(2));
    if(nowVol > maxVol) maxVol = nowVol;
    for(size_t i = 0; i < dest->size(); ++i){
        auto&p=dest->points[i];
        if( p.x < minpt(0) || p.y < minpt(1) || p.z < minpt(2) || 
            p.x > maxpt(0) || p.y > maxpt(1) || p.z > maxpt(2) ) continue;
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh; 
        dest_map[ind] = std::make_pair(i, -1);
    }

    for(size_t i = 0; i < source->size(); ++i){
        auto&p=source->points[i];
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        auto it = dest_map.find(ind);
        if(it != dest_map.end()) it->second.second = i;
        else dest_map[ind] = std::make_pair(-1, i);
    }

    int add_count=0;
    for(auto&p:dest_map){
        if(p.second.second != -1){
            dest->push_back(source->points[p.second.second]);
            add_count++;
        }
    }

    return dest;
}


pcl::PointCloud<PointType>::Ptr mergeCloudAndUpdateInfo(pcl::PointCloud<PointType>::Ptr dest, pcl::PointCloud<PointType>::Ptr source, vector<bool>&interest)
{   
    //update info
    laserCloudUpdate.reset(new pcl::PointCloud<PointType>());
    //merge
    double lx=0.1,ly=0.1,lz=0.1;
    Eigen::Vector4f minpt,maxpt;
    pcl::getMinMax3D(*source, minpt, maxpt);

    map<int, std::pair<int, int> > dest_map; //key:voxel_i - val:dest_i,source_i
    
    int width = std::ceil( (maxpt(0)-minpt(0)) / lx );
    int len   = std::ceil( (maxpt(1)-minpt(1)) / ly );
    int hei   = std::ceil( (maxpt(2)-minpt(2)) / ly );
    int perh  = width * len;

    static double maxVol = 0;
    double nowVol = (maxpt(0)-minpt(0))*(maxpt(1)-minpt(1))*(maxpt(2)-minpt(2));
    if(nowVol > maxVol) maxVol = nowVol;
    TicToc index_time;
    for(size_t i = 0; i < dest->size(); ++i){
        auto&p=dest->points[i];
        if( p.x < minpt(0) || p.y < minpt(1) || p.z < minpt(2) || 
            p.x > maxpt(0) || p.y > maxpt(1) || p.z > maxpt(2) ) continue;
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh; 
        dest_map[ind] = std::make_pair(i, -1);
    }

    for(size_t i = 0; i < source->size(); ++i){
        auto&p=source->points[i];
        // if(!interest[p.label&0xFFFF]) continue;
        int ind_x = std::floor(p.x-minpt(0) / lx);
        int ind_y = std::floor(p.y-minpt(1) / ly);
        int ind_z = std::floor(p.z-minpt(2) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        auto it = dest_map.find(ind);
        if(it != dest_map.end()) it->second.second = i;
        else dest_map[ind] = std::make_pair(-1, i);
    }
    printf("\tindex time %f\n", index_time.toc());

    TicToc add_time;
    int add_count=0;
    for(auto&p:dest_map){
        if(p.second.second != -1){
            dest->push_back(source->points[p.second.second]);
            laserCloudUpdate->push_back(source->points[p.second.second]);
            add_count++;
        }
    }
    printf("\tadd time %f num: %d\n", add_time.toc(), add_count);

    TicToc cluster_time;
    //cluster updatedcloud 
    // 基于高斯混合模型的城市道路场景物体约束点云配准
    // 根据局部地图、全局地图生成高斯混合模型，用于进行新点的实例化参数估计与数据关联，将新点关联到概率最大的实例中去，并根据优化后的位姿更新地图信息，以使得新的GMM是优化过后的点云注册而来
    // 在生成第一帧点云聚类、新添加的点过多、有聚类的点概率数值过小的时候，需要重新进行生成聚类维护信息
    // 数据关联方式就是GMM，但是聚类方式还得搞一下，
    // updateCloud;
}


void publishCloud(){
    sensor_msgs::PointCloud2 tmpMsgs;
    pcl::toROSMsg(*laserCloudPole, tmpMsgs);
    tmpMsgs.header.stamp = ros::Time().fromSec(timeCloudRaw);
    tmpMsgs.header.frame_id = "camera_init";
    pubPoleCloud.publish(tmpMsgs);

    pcl::toROSMsg(*laserCloudUpdate, tmpMsgs);
    tmpMsgs.header.stamp = ros::Time().fromSec(timeCloudRaw);
    tmpMsgs.header.frame_id = "camera_init";
    pubUpdatedCloud.publish(tmpMsgs);

    pcl::toROSMsg(*cluster_cloud, tmpMsgs);
    tmpMsgs.header.stamp = ros::Time().fromSec(timeCloudRaw);
    tmpMsgs.header.frame_id = "camera_init";
    pubClusterCloud.publish(tmpMsgs);

    
}

#include "yaml-cpp/yaml.h"

vector<bool> getInterest(string file_name){
    vector<int> ind;
    vector<bool> interest;
    vector<int> labels;
    vector<string> label_vals;
    YAML::Node config = YAML::LoadFile(file_name);

    int max_labels = 0;
    for(YAML::const_iterator it= config["labels"].begin(); it != config["labels"].end();++it){
        int now_label = it->first.as<int>();
        string now_label_val = it->second.as<string>();
        labels.push_back(now_label);
        label_vals.push_back(now_label_val);
        if( now_label > max_labels && now_label_val != "" ){
            max_labels = now_label; 
        }
        printf("%d : %s\n", now_label, now_label_val.c_str());
    }

    stringstream ss(config["interest"].as<string>());
    string info;
    while (getline(ss, info, ',')){
        ind.push_back(std::stoi(info));
    }

    std::sort(ind.begin(), ind.end());

    interest.resize(max_labels+1, false);
    printf("Total %d labels. Max LabelNum: %d. Thest are interested:\n", (int)labels.size(), max_labels);
    for(auto&p:ind){
        interest[p] = true;
        printf("%d:true\n", p);
    }

    return interest;
}

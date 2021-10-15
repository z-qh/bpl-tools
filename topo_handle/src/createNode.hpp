#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"


class createNode{
private:
    int bufferQueLength;
    deque<pcl::PointCloud<PointType>>laserCloudQue;
    deque<double>lidarTimeStampQue;
    deque<Eigen::Isometry3d>odometryQue;
    bool newlaserOdometry = false;
    bool newlaserCloud    = false;
    double laserCloudTime = 0;
    double laserOdomTime  = 0;
    pcl::PointCloud<PointType>::Ptr localMap;
    pcl::PointCloud<PointType>::Ptr arouncMapShow;
    int current_node_id = 0;
    int last_node_id    = -1;
    //////////////////////////////////////
    bool save_data_to_files;
    std::string node_save_path;

    node create_new_node(int& id, int sim, pcl::PointCloud<pcl::PointXYZI>& cloud)
    {
        int keyFrameIndex = bufferQueLength/2;
        Eigen::Isometry3d keyPose = odometryQue.at(keyFrameIndex);
        PointType nowPose;
        nowPose.x = keyPose.translation().x();
        nowPose.y = keyPose.translation().y();
        nowPose.z = keyPose.translation().z();
        nowPose.intensity = sim;
        double timeNow = lidarTimeStampQue.at(lidarTimeStampQue.size()/2);
        node topologicalNode(id, timeNow, nowPose, cloud, "");
        cout<<"create node id ===> "<<id<<std::endl;
        return topologicalNode;
    }


    bool createLocalMap()
    {
        while((int)laserCloudQue.size() > bufferQueLength)
        {
            laserCloudQue.pop_front();
            lidarTimeStampQue.pop_front();
        }
        while((int)odometryQue.size() > bufferQueLength)
        {
            odometryQue.pop_front();
        }
        if((int)laserCloudQue.size() == (int)odometryQue.size() && (int)laserCloudQue.size() == bufferQueLength)
        {
            localMap->clear();
            arouncMapShow->clear();
            Eigen::Isometry3d keyMatrix = odometryQue.at(odometryQue.size()/2).inverse();
            int queSize = laserCloudQue.size();
            pcl::PointCloud<PointType>::Ptr tmpMap(new pcl::PointCloud<PointType>() );
            for(int i=0; i<queSize; ++i)
            {
                *arouncMapShow += laserCloudQue.at(i);
                Eigen::Matrix4d transMatrix = keyMatrix.matrix();
                pcl::transformPointCloud(laserCloudQue.at(i), *tmpMap, transMatrix);
                *localMap += *tmpMap;
                tmpMap->clear();
            }
            return true;
        }
        std::cout << "数量" << laserCloudQue.size() << std::endl;//qh add for debug
        return false;
    }

public:
    createNode(std::string node_save_path_)
    {
        ////////////////////
        localMap.reset(new pcl::PointCloud<PointType>());
        arouncMapShow.reset(new pcl::PointCloud<PointType>());
        node_save_path = node_save_path_;
        bufferQueLength = 13;
        save_data_to_files = true;//是否保存文件

        /////////////////////
    }
    void run(node& nodeTemp, sensor_msgs::PointCloud2 remove_obstacle_cloud,
             nav_msgs::Odometry remove_obstacle_odom,
             int isSim,
             sensor_msgs::PointCloud2& localMsgs,
             bool& createFlag)
    {
        /////////////
        if(remove_obstacle_cloud.data.empty()){//初始的时候可能为空，因此判断{
            return;
        }
        /////////////
        ////////////////////
        laserOdomTime = remove_obstacle_odom.header.stamp.toSec();
        Eigen::Isometry3d currentPose = Eigen::Isometry3d::Identity();
        currentPose.rotate(Eigen::Quaterniond(remove_obstacle_odom.pose.pose.orientation.w,
                                              remove_obstacle_odom.pose.pose.orientation.x,
                                              remove_obstacle_odom.pose.pose.orientation.y,
                                              remove_obstacle_odom.pose.pose.orientation.z));
        currentPose.pretranslate(Eigen::Vector3d(remove_obstacle_odom.pose.pose.position.x,
                                                 remove_obstacle_odom.pose.pose.position.y,
                                                 remove_obstacle_odom.pose.pose.position.z));
        odometryQue.push_back(currentPose);
        newlaserOdometry = true;
        ///////////////////////
        pcl::PointCloud<PointType> LaserCloud;
        pcl::fromROSMsg(remove_obstacle_cloud, LaserCloud);
        laserCloudTime = remove_obstacle_cloud.header.stamp.toSec();
        ///////////////////////
        laserCloudQue.push_back(LaserCloud);
        lidarTimeStampQue.push_back(laserCloudTime);
        LaserCloud.clear();
        newlaserCloud = true;
        //////////////////////

        //////////////////////
        //////////////////////
        /////////////main handle/////////////////////
        if(newlaserCloud && newlaserOdometry)
        {
            newlaserCloud = false;
            newlaserOdometry = false;
            if(createLocalMap())
            {

                pcl::toROSMsg(*localMap, localMsgs);
                localMsgs.header.frame_id = "camera_init";
                //////////////////////create node
                if(true)
                {
                    nodeTemp = create_new_node(current_node_id, isSim, *localMap);
                    //node tmp_node = create_new_node(current_node_id, isSim, *localMap);
                    if(save_data_to_files)
                    {
                        //tmp_node.nodes_save_B(node_save_path);
                        createFlag = true;
                    }
                    last_node_id = current_node_id;
                    current_node_id++;
                }
            }
            localMap->clear();
        }
    }
};
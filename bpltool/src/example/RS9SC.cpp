#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "random"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "vector"

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "iostream"

#include "pcl/registration/icp.h"
#include "visualization_msgs/Marker.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/impl/pcl_base.hpp"
#include "nav_msgs/Path.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include "pcl/registration/transforms.h"
using namespace std;

//这里展示RS和SC检测回环的两种坐标系及的情况，

//这里用到kitti08的数据，其中真值作为对比展示，两个里程计都使用真值加上偏差
//使用kitti08中700-1500的数据，其中700-900段和1300和1500段发生反向的回环，点云仅使用XYZ信息
//数据存储在data下，使用相同的方式确定回环，认为在data中的101帧和710帧存在回环，利用两种方式计算并可视化

Eigen::Matrix4d odom2Trans(nav_msgs::Odometry& odom){
    Eigen::Vector3d tmpt(odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z);
    Eigen::Quaterniond tmpQ(odom.pose.pose.orientation.w,
                            odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y,
                            odom.pose.pose.orientation.z);
    Eigen::Matrix3d tmpR(tmpQ);
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3,3>(0,0) = tmpR;
    result.block<3,1>(0,3) = tmpt;
    return result;
}

nav_msgs::Odometry XYZRPY2Odom(double X,double Y, double Z, double Roll, double Pitch, double Yaw){
    nav_msgs::Odometry result;
    auto tmpQ = tf::createQuaternionFromRPY(Roll, Pitch, Yaw);
    result.pose.pose.position.x = X;
    result.pose.pose.position.y = Y;
    result.pose.pose.position.z = Z;
    result.pose.pose.orientation.x = tmpQ.x();
    result.pose.pose.orientation.y = tmpQ.y();
    result.pose.pose.orientation.z = tmpQ.z();
    result.pose.pose.orientation.w = tmpQ.w();
    return result;
}

void trans2XYZRPY(Eigen::Matrix4d& odom, double&X,double&Y,double&Z,double&Roll,double&Pitch,double&Yaw){
    Eigen::Matrix3d tmpR = odom.block<3,3>(0,0);
    Eigen::Quaterniond tmpQ(tmpR);
    tf::Quaternion tQ(tmpQ.x(),
                      tmpQ.y(),
                      tmpQ.z(),
                      tmpQ.w());
    tf::Matrix3x3(tQ).getRPY(Roll, Pitch, Yaw);
    X = odom(0,3);
    Y = odom(1,3);
    Z = odom(2,3);
}

void odom2XYZRPY(nav_msgs::Odometry& odom, double&X,double&Y,double&Z,double&Roll,double&Pitch,double&Yaw){
    tf::Quaternion tmpQ(odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w);
    tf::Matrix3x3(tmpQ).getRPY(Roll, Pitch, Yaw);
    X = odom.pose.pose.position.x;
    Y = odom.pose.pose.position.y;
    Z = odom.pose.pose.position.z;
}

bool detectLoop(pcl::PointCloud<pcl::PointXYZI>::Ptr his, pcl::PointXYZI& now, int& loopId){
    if( his->empty() ){
        loopId = -1;
        return false;
    }
    pcl::KdTreeFLANN<pcl::PointXYZI> kdPosiHis;
    kdPosiHis.setInputCloud(his);
    vector<int> indexs;
    vector<float> distances;
    kdPosiHis.radiusSearch(now, 5.0, indexs, distances);
    loopId = -1;
    for(auto& i : indexs){
        double timedif = now.intensity - his->points[i].intensity;
        if( timedif > 30.0){
            loopId = i;
            return true;
        }
    }
    return false;
}

nav_msgs::Odometry RSArrow;
nav_msgs::Odometry SCArrow;
nav_msgs::Odometry ALArrow;
nav_msgs::Odometry NRArrow;
sensor_msgs::PointCloud2 RS;
sensor_msgs::PointCloud2 SC;
sensor_msgs::PointCloud2 NR;

Eigen::Matrix4d RSrelativeLoop;
Eigen::Matrix4d NRrelativeLoop;
Eigen::Matrix4d SCrelativeLoop;

int main(int argc, char** argv){
    ros::init(argc, argv, "rs9sc");
    ros::NodeHandle nh;

    ros::Publisher landmarkRS = nh.advertise<nav_msgs::Odometry>("/RSviaualmarks", 10);
    ros::Publisher landmarkSC = nh.advertise<nav_msgs::Odometry>("/SCviaualmarks", 10);
    ros::Publisher landmarkAL = nh.advertise<nav_msgs::Odometry>("/ALviaualmarks", 10);
    ros::Publisher landmarkNR = nh.advertise<nav_msgs::Odometry>("/NRviaualmarks", 10);
    ros::Publisher RScloudPub = nh.advertise<sensor_msgs::PointCloud2>("/RScloud", 1);
    ros::Publisher SCcloudPub = nh.advertise<sensor_msgs::PointCloud2>("/SCcloud", 1);
    ros::Publisher NRcloudPub = nh.advertise<sensor_msgs::PointCloud2>("/NRcloud", 1);
    ros::Publisher GTPub = nh.advertise<nav_msgs::Path>("/gt", 10);
    ros::Publisher trajPub = nh.advertise<nav_msgs::Path>("/traj", 10);
    ros::Publisher RSPathPub = nh.advertise<nav_msgs::Path>("/RStraj", 10);
    ros::Publisher NRPathPub = nh.advertise<nav_msgs::Path>("/NRtraj", 10);
    ros::Publisher SCPathPub = nh.advertise<nav_msgs::Path>("/SCtraj", 10);

    vector<nav_msgs::Odometry> gtHis;
    vector<nav_msgs::Odometry> poseHis;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudHis;
    pcl::PointCloud<pcl::PointXYZI>::Ptr posiHis(new pcl::PointCloud<pcl::PointXYZI>());
    vector<nav_msgs::Odometry> RSAfter;
    vector<nav_msgs::Odometry> NRAfter;
    vector<nav_msgs::Odometry> SCAfter;

    string dataPath = "/home/qh/qh_ws/src/bpl-tools/bpltool/src/data/data.bag";
    rosbag::Bag bag;
    bag.open(dataPath, rosbag::bagmode::Read);
    if(!bag.isOpen())
    {
        cout << "bag file load faild!" << endl;
        return 0;
    }
    int GlobalLoopId = -1;
    int GlobalNearId = -1;
    vector<string> topics;
    topics.emplace_back("/gt");
    topics.emplace_back("/odom");
    topics.emplace_back("/laser");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;
    int Frame = 0;
    bool isLaser=false,isOdom=false,isGT=false;
    nav_msgs::Odometry gt, odom;
    pcl::PointCloud<pcl::PointXYZ>::Ptr thisCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(it = view.begin(); it != view.end() ; it++)
    {
        string nowTopic = (*it).getTopic();
        if(isGT && isLaser && isOdom){
            isGT = false;
            isLaser = false;
            isOdom = false;
            Frame++;
            // cout << Frame << endl;
            /*RS*/
            pcl::PointXYZI tmpPosi;
            tmpPosi.x = gt.pose.pose.position.x;
            tmpPosi.y = gt.pose.pose.position.y;
            tmpPosi.z = gt.pose.pose.position.z;
            tmpPosi.intensity = (float)((((int)gt.header.stamp.toSec()) % 1000000));
            int loopId = -1;
            bool isLoop = detectLoop(posiHis, tmpPosi, loopId);
            posiHis->push_back(tmpPosi);
            static bool firstLoop = true;
            if( isLoop && loopId != -1 && firstLoop){
                firstLoop = false;
                cout << " detect loop id" << loopId << " now id " << Frame << endl;
                GlobalLoopId = loopId;
                GlobalNearId = Frame;
                cout << " time diff " << odom.header.stamp.toSec() - gtHis[loopId].header.stamp.toSec() << endl;
                // NR
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr loop(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>());
                    auto loopT = odom2Trans(gtHis[loopId]);
                    auto nearT = odom2Trans(gt);
                    *loop = *cloudHis[loopId];
                    *near = *thisCloud;
                    cout << " loop " << loop->size() << endl;
                    cout << " near " << near->size() << endl;
                    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                    icp.setMaximumIterations(100);
                    icp.setInputSource(near);
                    icp.setInputTarget(loop);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
                    auto gauss = (loopT.inverse()*nearT).cast<float>();
                    icp.align(*unused_result, gauss);
                    std::cout << "[NR] ICP fit score: " << icp.getFitnessScore() << std::endl;
                    Eigen::Matrix4d relative = icp.getFinalTransformation().cast<double>();
                    NRrelativeLoop = relative.inverse();
                    Eigen::Matrix3d tmpR = NRrelativeLoop.block<3,3>(0,0);
                    auto tmpEuler = tmpR.eulerAngles(2,1,0);
                    cout << " NR " << endl;
                    cout << " Euler " << tmpEuler.transpose() << endl;
                    cout << " trans " << NRrelativeLoop.block<3,1>(0,3).transpose() << endl;
                    //
//                    *loop += *near;
                    *loop += *unused_result;
                    pcl::toROSMsg(*loop, NR);
                    NR.header.frame_id = "laser";
                    //
                    NRArrow.header.frame_id = "laser";
                    NRArrow.pose.pose.position.x = NRrelativeLoop(0,3);
                    NRArrow.pose.pose.position.y = NRrelativeLoop(1,3);
                    NRArrow.pose.pose.position.z = NRrelativeLoop(2,3);
                    Eigen::Quaterniond tmpQ(NRrelativeLoop.block<3,3>(0,0));
                    NRArrow.pose.pose.orientation.x = tmpQ.x();
                    NRArrow.pose.pose.orientation.y = tmpQ.y();
                    NRArrow.pose.pose.orientation.z = tmpQ.z();
                    NRArrow.pose.pose.orientation.w = tmpQ.w();
                }
                // RS
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr loop(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>());
                    auto loopT = odom2Trans(gtHis[loopId]);
                    auto nearT = odom2Trans(gt);
                    pcl::transformPointCloud(*cloudHis[loopId], *loop, loopT);
                    pcl::transformPointCloud(*thisCloud, *near, nearT);
                    cout << " loop " << loop->size() << endl;
                    cout << " near " << near->size() << endl;
                    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                    icp.setMaximumIterations(100);
                    icp.setInputSource(near);
                    icp.setInputTarget(loop);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
                    icp.align(*unused_result);
                    std::cout << "[RS] ICP fit score: " << icp.getFitnessScore() << std::endl;
                    Eigen::Matrix4d correctionCameraFrame = icp.getFinalTransformation().cast<double>();
                    Eigen::Matrix4d tWrong = nearT;
                    Eigen::Matrix4d tCorrect = correctionCameraFrame * tWrong;
                    Eigen::Matrix4d relative = tCorrect.inverse() * loopT;
                    RSrelativeLoop = relative;
                    Eigen::Matrix3d tmpR = RSrelativeLoop.block<3,3>(0,0);
                    auto tmpEuler = tmpR.eulerAngles(2,1,0);
                    cout << " RS " << endl;
                    cout << " Euler " << tmpEuler.transpose() << endl;
                    cout << " trans " << RSrelativeLoop.block<3,1>(0,3).transpose() << endl;
                    //
//                    *loop += *near;
                    *loop += *unused_result;
                    pcl::toROSMsg(*loop, RS);
                    RS.header.frame_id = "laser";
                    //
                    RSArrow.header.frame_id = "laser";
                    RSArrow.pose.pose.position.x = RSrelativeLoop(0,3);
                    RSArrow.pose.pose.position.y = RSrelativeLoop(1,3);
                    RSArrow.pose.pose.position.z = RSrelativeLoop(2,3);
                    Eigen::Quaterniond tmpQ(RSrelativeLoop.block<3,3>(0,0));
                    RSArrow.pose.pose.orientation.x = tmpQ.x();
                    RSArrow.pose.pose.orientation.y = tmpQ.y();
                    RSArrow.pose.pose.orientation.z = tmpQ.z();
                    RSArrow.pose.pose.orientation.w = tmpQ.w();
                }
                // SC
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr loop(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>());
                    auto loopT = odom2Trans(gtHis[loopId]);
                    auto nearT = odom2Trans(gt);
                    pcl::transformPointCloud(*cloudHis[loopId], *loop, loopT);
                    pcl::transformPointCloud(*thisCloud, *near, loopT);
                    cout << " loop " << loop->size() << endl;
                    cout << " near " << near->size() << endl;
                    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                    icp.setMaximumIterations(100);
                    icp.setInputSource(near);
                    icp.setInputTarget(loop);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
                    auto gauss = (loopT * ( loopT.inverse() * nearT ) * loopT.inverse()).cast<float>();
                    icp.align(*unused_result, gauss);
                    std::cout << "[SC] ICP fit score: " << icp.getFitnessScore() << std::endl;
                    Eigen::Matrix4d relative = icp.getFinalTransformation().cast<double>();
                    SCrelativeLoop = relative.inverse();
                    Eigen::Matrix3d tmpR = SCrelativeLoop.block<3,3>(0,0);
                    auto tmpEuler = tmpR.eulerAngles(2,1,0);
                    cout << " SC " << endl;
                    cout << " Euler " << tmpEuler.transpose() << endl;
                    cout << " trans " << SCrelativeLoop.block<3,1>(0,3).transpose() << endl;
                    //
//                    *loop += *near;
                    *loop += *unused_result;
                    pcl::toROSMsg(*loop, SC);
                    SC.header.frame_id = "laser";
                    //
                    {
                        SCArrow.header.frame_id = "laser";
                        SCArrow.pose.pose.position.x = SCrelativeLoop(0,3);
                        SCArrow.pose.pose.position.y = SCrelativeLoop(1,3);
                        SCArrow.pose.pose.position.z = SCrelativeLoop(2,3);
                        Eigen::Quaterniond tmpQ(SCrelativeLoop.block<3,3>(0,0));
                        SCArrow.pose.pose.orientation.x = tmpQ.x();
                        SCArrow.pose.pose.orientation.y = tmpQ.y();
                        SCArrow.pose.pose.orientation.z = tmpQ.z();
                        SCArrow.pose.pose.orientation.w = tmpQ.w();
                    }

                    Eigen::Matrix4d Align = loopT.inverse() * SCrelativeLoop * loopT;
                    //
                    {
                        ALArrow.header.frame_id = "laser";
                        ALArrow.pose.pose.position.x = Align(0,3);
                        ALArrow.pose.pose.position.y = Align(1,3);
                        ALArrow.pose.pose.position.z = Align(2,3);
                        Eigen::Quaterniond tmpQ(Align.block<3,3>(0,0));
                        ALArrow.pose.pose.orientation.x = tmpQ.x();
                        ALArrow.pose.pose.orientation.y = tmpQ.y();
                        ALArrow.pose.pose.orientation.z = tmpQ.z();
                        ALArrow.pose.pose.orientation.w = tmpQ.w();
                        Eigen::Matrix3d tmpR = Align.block<3,3>(0,0);
                        auto tmpEuler = tmpR.eulerAngles(2,1,0);
                        cout << " AL " << endl;
                        cout << " Euler " << tmpEuler.transpose() << endl;
                        cout << " trans " << Align.block<3,1>(0,3).transpose() << endl;
                    }
                }

            }
        }
        if(nowTopic == topics[0]){
            gt = *(*it).instantiate<nav_msgs::Odometry>();
            gtHis.push_back(gt);
            isGT = true;
        }
        if(nowTopic == topics[1]){
            odom = *(*it).instantiate<nav_msgs::Odometry>();
            poseHis.push_back(odom);
            isOdom = true;
        }
        if(nowTopic == topics[2]){
            sensor_msgs::PointCloud2 cloud;
            cloud = *(*it).instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(cloud, *thisCloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>());
            *tmpCloud = *thisCloud;
            cloudHis.push_back(tmpCloud);
            isLaser = true;
        }


    }

    for(auto p : poseHis){
        RSAfter.push_back(p);
        NRAfter.push_back(p);
        SCAfter.push_back(p);
    }
    // GTSAM to Optimize All Poses with One Loop
    if( 1 == 0)
    {
        //RS
        {
            gtsam::NonlinearFactorGraph gtSAMgraph;
            gtsam::Values initialEstimate;
            gtsam::Values optimizedEstimate;
            gtsam::ISAM2Params parameters;
            parameters.relinearizeThreshold = 0.01;
            parameters.relinearizeSkip = 1;
            auto *isam = new gtsam::ISAM2(parameters);
            gtsam::Values isamCurrentEstimate;
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
            gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
            gtsam::noiseModel::Base::shared_ptr robustNoiseModel;
            for(int i = 0; i < RSAfter.size(); i++){
                auto& p = RSAfter[i];
                if(i == 0){
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                                                    gtsam::Point3(x, y, z)), priorNoise));
                    initialEstimate.insert(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                }else{
                    auto& pLast = RSAfter[i-1];
                    double xLast,yLast,zLast,rollLast,pitchLast,yawLast;
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(pLast, xLast, yLast, zLast, rollLast, pitchLast, yawLast);
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);

                    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(rollLast, pitchLast, yawLast),
                                                         gtsam::Point3(xLast, yLast, zLast));
                    gtsam::Pose3 poseTo   = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                         gtsam::Point3(x, y, z));
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
                    initialEstimate.insert(i, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                    isam->update(gtSAMgraph, initialEstimate);
                    isam->update();
                    gtSAMgraph.resize(0);
                    initialEstimate.clear();
                }
            }
            {
                double x,y,z,roll,pitch,yaw;
                trans2XYZRPY(RSrelativeLoop, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 relative(gtsam::Rot3::RzRyRx(roll,pitch,yaw),gtsam::Point3(x,y,z));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(GlobalNearId, GlobalLoopId, relative, robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
            isam->update();
            isam->update();
            isam->update();

            isamCurrentEstimate = isam->calculateBestEstimate();
            for(int i = 0; i < RSAfter.size(); i++){
                gtsam::Pose3 a = isamCurrentEstimate.at<gtsam::Pose3>(i);
                double x,y,z,roll,pitch,yaw;
                x = a.translation().x();
                y = a.translation().y();
                z = a.translation().z();
                roll = a.rotation().roll();
                pitch = a.rotation().pitch();
                yaw = a.rotation().yaw();
                nav_msgs::Odometry after = XYZRPY2Odom(x,y,z,roll,pitch,yaw);
                RSAfter[i] = after;
            }
            delete isam;
        }
        cout << " RS Optimize " << endl;
        //NR
        {
            gtsam::NonlinearFactorGraph gtSAMgraph;
            gtsam::Values initialEstimate;
            gtsam::Values optimizedEstimate;
            gtsam::ISAM2Params parameters;
            parameters.relinearizeThreshold = 0.01;
            parameters.relinearizeSkip = 1;
            auto *isam = new gtsam::ISAM2(parameters);
            gtsam::Values isamCurrentEstimate;
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
            gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
            gtsam::noiseModel::Base::shared_ptr robustNoiseModel;
            for(int i = 0; i < NRAfter.size(); i++){
                auto& p = NRAfter[i];
                if(i == 0){
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                                                    gtsam::Point3(x, y, z)), priorNoise));
                    initialEstimate.insert(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                }else{
                    auto& pLast = NRAfter[i-1];
                    double xLast,yLast,zLast,rollLast,pitchLast,yawLast;
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(pLast, xLast, yLast, zLast, rollLast, pitchLast, yawLast);
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);

                    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(rollLast, pitchLast, yawLast),
                                                         gtsam::Point3(xLast, yLast, zLast));
                    gtsam::Pose3 poseTo   = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                         gtsam::Point3(x, y, z));
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
                    initialEstimate.insert(i, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                    isam->update(gtSAMgraph, initialEstimate);
                    isam->update();
                    gtSAMgraph.resize(0);
                    initialEstimate.clear();
                }
            }
            {
                double x,y,z,roll,pitch,yaw;
                trans2XYZRPY(NRrelativeLoop, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 relative(gtsam::Rot3::RzRyRx(roll,pitch,yaw),gtsam::Point3(x,y,z));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(GlobalNearId, GlobalLoopId, relative, robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
            isam->update();
            isam->update();
            isam->update();

            isamCurrentEstimate = isam->calculateBestEstimate();
            for(int i = 0; i < NRAfter.size(); i++){
                gtsam::Pose3 a = isamCurrentEstimate.at<gtsam::Pose3>(i);
                double x,y,z,roll,pitch,yaw;
                x = a.translation().x();
                y = a.translation().y();
                z = a.translation().z();
                roll = a.rotation().roll();
                pitch = a.rotation().pitch();
                yaw = a.rotation().yaw();
                nav_msgs::Odometry after = XYZRPY2Odom(x,y,z,roll,pitch,yaw);
                NRAfter[i] = after;
            }
            delete isam;
        }
        cout << " NR Optimize " << endl;
        //SC
        {
            gtsam::NonlinearFactorGraph gtSAMgraph;
            gtsam::Values initialEstimate;
            gtsam::Values optimizedEstimate;
            gtsam::ISAM2Params parameters;
            parameters.relinearizeThreshold = 0.01;
            parameters.relinearizeSkip = 1;
            auto *isam = new gtsam::ISAM2(parameters);
            gtsam::Values isamCurrentEstimate;
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
            gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
            gtsam::noiseModel::Base::shared_ptr robustNoiseModel;
            for(int i = 0; i < SCAfter.size(); i++){
                auto& p = SCAfter[i];
                if(i == 0){
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                                                    gtsam::Point3(x, y, z)), priorNoise));
                    initialEstimate.insert(0, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                }else{
                    auto& pLast = SCAfter[i-1];
                    double xLast,yLast,zLast,rollLast,pitchLast,yawLast;
                    double x,y,z,roll,pitch,yaw;
                    odom2XYZRPY(pLast, xLast, yLast, zLast, rollLast, pitchLast, yawLast);
                    odom2XYZRPY(p, x, y, z, roll, pitch, yaw);

                    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(rollLast, pitchLast, yawLast),
                                                         gtsam::Point3(xLast, yLast, zLast));
                    gtsam::Pose3 poseTo   = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                                         gtsam::Point3(x, y, z));
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
                    initialEstimate.insert(i, gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),gtsam::Point3(x, y, z)));
                    isam->update(gtSAMgraph, initialEstimate);
                    isam->update();
                    gtSAMgraph.resize(0);
                    initialEstimate.clear();
                }
            }
            {
                double x,y,z,roll,pitch,yaw;
                trans2XYZRPY(SCrelativeLoop, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 relative(gtsam::Rot3::RzRyRx(roll,pitch,yaw),gtsam::Point3(x,y,z));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(GlobalNearId, GlobalLoopId, relative, robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
            isam->update();
            isam->update();
            isam->update();

            isamCurrentEstimate = isam->calculateBestEstimate();
            for(int i = 0; i < SCAfter.size(); i++){
                gtsam::Pose3 a = isamCurrentEstimate.at<gtsam::Pose3>(i);
                double x,y,z,roll,pitch,yaw;
                x = a.translation().x();
                y = a.translation().y();
                z = a.translation().z();
                roll = a.rotation().roll();
                pitch = a.rotation().pitch();
                yaw = a.rotation().yaw();
                nav_msgs::Odometry after = XYZRPY2Odom(x,y,z,roll,pitch,yaw);
                SCAfter[i] = after;
            }
            delete isam;
        }
        cout << " SC Optimize " << endl;
    }

    nav_msgs::Path gtPath, trajPath;
    for(auto& p : gtHis){
        geometry_msgs::PoseStamped tmpP;
        tmpP.pose = p.pose.pose;
        tmpP.header = p.header;
        gtPath.poses.push_back(tmpP);
    }
    for(auto& p : poseHis){
        geometry_msgs::PoseStamped tmpP;
        tmpP.pose = p.pose.pose;
        tmpP.header = p.header;
        trajPath.poses.push_back(tmpP);
    }
    gtPath.header.frame_id = "laser";
    trajPath.header.frame_id = "laser";

    nav_msgs::Path RSAfterPath, NRAfterPath, SCAfterPath;
    for(auto&p:RSAfter){
        geometry_msgs::PoseStamped tmpP;
        tmpP.pose = p.pose.pose;
        tmpP.header = p.header;
        RSAfterPath.poses.push_back(tmpP);
    }
    for(auto&p:NRAfter){
        geometry_msgs::PoseStamped tmpP;
        tmpP.pose = p.pose.pose;
        tmpP.header = p.header;
        NRAfterPath.poses.push_back(tmpP);
    }
    for(auto&p:SCAfter){
        geometry_msgs::PoseStamped tmpP;
        tmpP.pose = p.pose.pose;
        tmpP.header = p.header;
        SCAfterPath.poses.push_back(tmpP);
    }
    RSAfterPath.header.frame_id = "laser";
    NRAfterPath.header.frame_id = "laser";
    SCAfterPath.header.frame_id = "laser";

    ros::Rate loop(10);
    while(ros::ok()){
        loop.sleep();
        RScloudPub.publish(RS);
        SCcloudPub.publish(SC);
        NRcloudPub.publish(NR);
        landmarkNR.publish(NRArrow);
        landmarkRS.publish(RSArrow);
        landmarkSC.publish(SCArrow);
        landmarkAL.publish(ALArrow);
        trajPub.publish(trajPath);
        GTPub.publish(gtPath);
        RSPathPub.publish(RSAfterPath);
        NRPathPub.publish(NRAfterPath);
        SCPathPub.publish(SCAfterPath);
    }
}


class Pose{
public:
    double x = 0;
    double y = 0;
    double z = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double qx = 0;
    double qy = 0;
    double qz = 0;
    double qw = 0;
    double time = 0;
};

void handledata(){
//    string sourceBagPath = "/home/qh/kitti/seq08-500-end.bag";
//    string sourceBagPathTopic = "/laser";
//    string sourceBagGt ="/gt";
    //    ofstream file;
//    file.open("/home/qh/qh_ws/src/bpl-tools/bpltool/src/data/gt.txt");
//    if(!file.is_open())
//    {
//        cout << "gt file load faild!" << endl;
//        return 0;
//    }
//    rosbag::Bag data;
//    data.open("/home/qh/qh_ws/src/bpl-tools/bpltool/src/data/data.bag", rosbag::bagmode::Append);
//    if(!data.isOpen())
//    {
//        cout << "data file load faild!" << endl;
//        return 0;
//    }
//    rosbag::Bag bag;
//    bag.open(sourceBagPath, rosbag::bagmode::Read);
//    if(!bag.isOpen())
//    {
//        cout << "bag file load faild!" << endl;
//        return 0;
//    }
//    vector<string> topics;
//    topics.push_back(sourceBagPathTopic);
//    topics.push_back(sourceBagGt);
//    rosbag::View view(bag, rosbag::TopicQuery(topics));
//    rosbag::View::iterator it;
//    //遍历bag
//    int cloudframe = 0;
//    int gtframe = 0;
//    for(it = view.begin(); it != view.end() ; it++)
//    {
//        string nowTopic = (*it).getTopic();
//        if(nowTopic == topics[0]){
//            cloudframe++;
//            if(cloudframe >= 200 && cloudframe < 1000){
//                sensor_msgs::PointCloud2::ConstPtr tempCloud = (*it).instantiate<sensor_msgs::PointCloud2>();
//                if(tempCloud == nullptr)
//                {
//                    cout << "error" << endl;
//                    continue;
//                }
//                sensor_msgs::PointCloud2 rosCloud = *tempCloud;
//                pcl::PointCloud<pcl::PointXYZ>::Ptr saveCloud(new pcl::PointCloud<pcl::PointXYZ>());
//                pcl::fromROSMsg(rosCloud, *saveCloud);
//                sensor_msgs::PointCloud2 saveMsg;
//                pcl::toROSMsg(*saveCloud, saveMsg);
//                saveMsg.header = rosCloud.header;
//                //data.write("/laser", saveMsg.header.stamp, saveMsg);
//            }
//        }
//        if(nowTopic == topics[1]){
//            gtframe++;
//            if(gtframe >= 200 && gtframe < 1000) {
//                nav_msgs::Odometry::ConstPtr tempOdom = (*it).instantiate<nav_msgs::Odometry>();
//                nav_msgs::Odometry saveMsg = *tempOdom;
//                double R,P,Y;
//                //data.write("/odom", saveMsg.header.stamp, saveMsg);
//                tf::Quaternion tmpQ(saveMsg.pose.pose.orientation.x,
//                                    saveMsg.pose.pose.orientation.y,
//                                    saveMsg.pose.pose.orientation.z,
//                                    saveMsg.pose.pose.orientation.w);
//                tf::Matrix3x3(tmpQ).getRPY(R, P, Y);
//                file << gtframe << " ";
//                file  << setprecision(4) << fixed
//                      << saveMsg.pose.pose.position.x << " "
//                      << saveMsg.pose.pose.position.y <<  " "
//                      << saveMsg.pose.pose.position.z << " "
//                      << R << " "
//                      << P << " "
//                      << Y << " "
//                      << saveMsg.pose.pose.orientation.x << " "
//                      << saveMsg.pose.pose.orientation.y << " "
//                      << saveMsg.pose.pose.orientation.z << " "
//                      << saveMsg.pose.pose.orientation.w << " "
//                      << setprecision(8) << saveMsg.header.stamp.toSec() << endl;
//            }
//        }
//    }
//    file.close();
//    data.close();

//    vector<Pose> poses;
//    vector<Eigen::Matrix4d> transforms;
//    ifstream GTfile("/home/qh/qh_ws/src/bpl-tools/bpltool/src/data/gt.txt");
//    // TO DO
//    while(GTfile.good()){
//        string infoStr;
//        getline(GTfile, infoStr);
//        if(infoStr.empty()) continue;
//        istringstream iss(infoStr);
//        Pose tp;
//        int id = 0;
//        iss >> id;
//        iss >> tp.x >> tp.y >> tp.z;
//        iss >> tp.roll >> tp.pitch >> tp.yaw;
//        iss >> tp.qx >> tp.qy >> tp.qz >> tp.qw;
//        iss >> tp.time;
//        poses.push_back(tp);
//        Eigen::Quaterniond tmpQ(tp.qw, tp.qx, tp.qy, tp.qz);
//        Eigen::Matrix3d tmpR(tmpQ);
//        Eigen::Vector3d tmpt(tp.x, tp.y, tp.z);
//        Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
//        tmpT.block<3,3>(0,0) = tmpR;
//        tmpT.block<3,1>(0,3) = tmpt;
//        transforms.push_back(tmpT);
//    }
//    cout << " poses " << poses.size() << endl;
//    cout << " trans " << transforms.size() << endl;
//    GTfile.close();
//
//    vector<Eigen::Matrix4d> relative;
//    vector<Eigen::Matrix4d> relativeBias;
//    vector<Eigen::Matrix4d> transformsBias;
//    default_random_engine rng;
//    normal_distribution<double> gauss(0, 2);
//    for(int i = 0; i < transforms.size(); i++){
////        char key = getchar();
////        if(key == 'q') exit(0);
//        Eigen::Matrix4d to = transforms[i];
//        if(i == 0 ){
//            transformsBias.push_back(to);
//        }
//        if(i != 0){
//            {
//                Eigen::Matrix4d from = transforms[i-1];
////                cout << "from" << fixed << setprecision(1) << from << endl;
////                cout << "to" << fixed << setprecision(1) << to << endl;
//                Eigen::Matrix4d rela = from.inverse() * to;
////                cout << "rela" << fixed << setprecision(1) << rela;
//                relative.push_back(rela);
//            }
//            {
//                Eigen::Matrix4d rela = relative[i-1];
//                Eigen::Matrix3d relativeR = rela.block<3,3>(0,0);
//                Eigen::Quaterniond relativeQ(relativeR);
//                tf::Quaternion tmpQ(relativeQ.x(), relativeQ.y(), relativeQ.z(), relativeQ.w());
//                // gauss(rng)
//                double Roll,Pitch,Yaw;
//                tf::Matrix3x3(tmpQ).getRPY(Roll, Pitch, Yaw);
//                double deltaYaw = gauss(rng);
//                double times = deltaYaw/Yaw;
//                cout << "Yaw " << Yaw << " delta Yaw " << deltaYaw << " pro " << endl;
//                deltaYaw /= times;
//                deltaYaw /= 15;
//                Yaw += deltaYaw;
//                auto biastmpQ = tf::createQuaternionFromRPY(Roll, Pitch, Yaw);
//                Eigen::Quaterniond biasQ(biastmpQ.w(), biastmpQ.x(), biastmpQ.y(), biastmpQ.z());
//                Eigen::Matrix3d biasR(biasQ);
//                rela.block<3,3>(0,0) = biasR;
////                cout << "relaBias" << fixed << setprecision(1) << rela << endl;
//                relativeBias.push_back(rela);
//            }
//            {
//                Eigen::Matrix4d last = transformsBias[i-1];
//                Eigen::Matrix4d rela = relativeBias[i-1];
//                Eigen::Matrix4d tmpT = last * rela;
////                cout << "poseBias" << fixed << setprecision(1) << tmpT << endl;
//                transformsBias.push_back(tmpT);
//            }
//        }
//    }
//    cout << "trans " << transforms.size() << endl;
//    cout << "transBias " << transformsBias.size() << endl;
//    cout << "rela " << relative.size() << endl;
//    cout << "relaBias " << relativeBias.size() << endl;
//
//    for(int i = 0; i < transformsBias.size(); i++){
//        Eigen::Matrix4d p = transformsBias[i];
//        double time = poses[i].time;
//        Eigen::Matrix3d tmpR = p.block<3,3>(0,0);
//        Eigen::Vector3d tmpt = p.block<3,1>(0,3);
//        Eigen::Quaterniond tmpQ(tmpR);
//        nav_msgs::Odometry tempOdom;
//        tempOdom.pose.pose.position.x = tmpt(0);
//        tempOdom.pose.pose.position.y = tmpt(1);
//        tempOdom.pose.pose.position.z = tmpt(2);
//        tempOdom.pose.pose.orientation.x = tmpQ.x();
//        tempOdom.pose.pose.orientation.y = tmpQ.y();
//        tempOdom.pose.pose.orientation.z = tmpQ.z();
//        tempOdom.pose.pose.orientation.w = tmpQ.w();
//        tempOdom.header.frame_id = "laser";
//        tempOdom.header.stamp = ros::Time().fromSec(time);
//        data.write("/odom", tempOdom.header.stamp, tempOdom);
//    }
//    data.close();
}




//#include "Base.h"
//
//using namespace std;
//
//bool comPairAZ(pair<double,int>a,pair<double,int>b){
//    return a.first > b.first;
//}
//
//void ExtractFeatures(pcl::PointCloud<PointType>::Ptr mCloud, pcl::PointCloud<PointType>::Ptr cornerCloud, pcl::PointCloud<PointType>::Ptr surfaceCloud)
//{
//    cornerCloud->clear();
//    surfaceCloud->clear();
//
//    vector<double> mPointRange;// point range
//    vector<double> mCurvature;// point curvature
//    vector<double> mNeighborPicked;//to pick bad points
//    vector<double> mCloudLabel;//to pick all surf points
//    vector<double> mPointColInd;//Point Column Index
//    vector<int> mStartRingIndex;//Per rings start-end index
//    vector<int> mEndRingIndex;//
//    vector<pair<double,int>> mCloudSmoothness;// Smoothness for sorting
//
//    cout << " start handle " << endl;
//    int pointNums = mCloud->size();
//    mPointRange.resize(pointNums);// Point Range
//    mCloudLabel.resize(pointNums);//to pick all surf points
//    mNeighborPicked.resize(pointNums);//to pick bad points
//    mPointColInd.resize(pointNums);// Point Column Index
//    mCloudSmoothness.resize(pointNums);// Smoothness for sorting
//    mStartRingIndex.resize(N_SCAN);// Per rings start-end index
//    mEndRingIndex.resize(N_SCAN);
//    mCurvature.resize(pointNums);
//    int tmpLastRing = mCloud->points.front().ring;
//    mStartRingIndex[tmpLastRing] = 0;
//    for(int i = 0; i < pointNums; ++i){
//        auto& p = mCloud->points[i];
//        if(p.ring != tmpLastRing){
//            mEndRingIndex[tmpLastRing] = i-1;
//            tmpLastRing = p.ring;
//            mStartRingIndex[tmpLastRing] = i;
//        }
//        mPointRange[i] = sqrt(pow(p.x,2)+pow(p.y,2)+pow(p.z,2));
//        double horizonAngle = atan2(p.x, p.y) * 180 / M_PI;
//        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
//        mPointColInd[i] = columnIdn;
//        if(mPointRange[i] < 3.0 || mPointRange[i] > 100.0 )
//            mNeighborPicked[i] = 1;
//        else
//            mNeighborPicked[i] = 1;
//
//        mCloudLabel[i] = 0;
//    }
//    mEndRingIndex[tmpLastRing] = pointNums-1;
//
//    cout << " Calculate smoothness " << endl;
//    // Calculate smoothness
//    for (int i = 5; i < pointNums - 5; ++i)
//    {
//        double diffRange = mPointRange[i-5] + mPointRange[i-4]
//                           + mPointRange[i-3] + mPointRange[i-2]
//                           + mPointRange[i-1] - mPointRange[i] * 10
//                           + mPointRange[i+1] + mPointRange[i+2]
//                           + mPointRange[i+3] + mPointRange[i+4]
//                           + mPointRange[i+5];
//
//        mCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;
//
//
//        mCloudSmoothness[i].first = mCurvature[i];// Smoothness for sorting
//        mCloudSmoothness[i].second = i;
//    }
//
//    cout << " mark occluded points and parallel beam points ref lego loam and loam " << endl;
//    // mark occluded points and parallel beam points ref lego loam and loam
//    for (int i = 5; i < pointNums - 6; ++i) {
//
//        auto &thisPoint = mCloud->points[i];
//        double depth1 = mPointRange[i];
//        double depth2 = mPointRange[i + 1];
//
//        int columnDiff = abs(int(mPointColInd[i + 1] - mPointColInd[i]));
//
//        // occluded points
//        if (columnDiff < 10) {//10 points range
//            if (depth1 - depth2 > 0.3) {
//                mNeighborPicked[i - 5] = 1;
//                mNeighborPicked[i - 4] = 1;
//                mNeighborPicked[i - 3] = 1;
//                mNeighborPicked[i - 2] = 1;
//                mNeighborPicked[i - 1] = 1;
//                mNeighborPicked[i] = 1;
//            } else if (depth2 - depth1 > 0.3) {
//                mNeighborPicked[i + 1] = 1;
//                mNeighborPicked[i + 2] = 1;
//                mNeighborPicked[i + 3] = 1;
//                mNeighborPicked[i + 4] = 1;
//                mNeighborPicked[i + 5] = 1;
//                mNeighborPicked[i + 6] = 1;
//            }
//        }
//        // parallel beam points
//        float diff1 = abs(float(mPointRange[i - 1] - mPointRange[i]));
//        float diff2 = abs(float(mPointRange[i + 1] - mPointRange[i]));
//
//        if (diff1 > 0.02 * mPointRange[i] && diff2 > 0.02 * mPointRange[i])
//            mNeighborPicked[i] = 1;
//    }
//
//    cout << " extract featrues " << endl;
//    // extract featrues
//    for (int i = 0; i < N_SCAN; i++)
//    {
//        for (int j = 0; j < 6; j++)
//        {
//            int sp = (mStartRingIndex[i] * (6 - j) + mEndRingIndex[i] * j) / 6;
//            int ep = (mStartRingIndex[i] * (5 - j) + mEndRingIndex[i] * (j + 1)) / 6 - 1;
//
//            if (sp >= ep)
//                continue;
//            // sort to find min and max serial points
//            sort(mCloudSmoothness.begin()+sp, mCloudSmoothness.begin()+ep, comPairAZ);
//            // pick large point
//            int largestPickedNum = 0;
//            for (int k = ep; k >= sp; k--)
//            {
//                int ind = mCloudSmoothness[k].second;
//                if (mNeighborPicked[ind] == 0 && mCurvature[ind] > edgeThreshold)
//                {
//                    largestPickedNum++;
//                    // pick one point
//                    if (largestPickedNum <= 20){
//                        mCloudLabel[ind] = 1;
//                        cornerCloud->push_back(mCloud->points[ind]);
//                    } else {
//                        break;
//                    }
//                    // pick surround points
//                    mNeighborPicked[ind] = 1;
//                    for (int l = 1; l <= 5; l++)
//                    {
//                        int columnDiff = abs(int(mPointColInd[ind + l] - mPointColInd[ind + l - 1]));
//                        if (columnDiff > 10)
//                            break;
//                        mNeighborPicked[ind + l] = 1;
//                    }
//                    for (int l = -1; l >= -5; l--)
//                    {
//                        int columnDiff = abs(int(mPointColInd[ind + l] - mPointColInd[ind + l + 1]));
//                        if (columnDiff > 10)
//                            break;
//                        mNeighborPicked[ind + l] = 1;
//                    }
//                }
//            }
//            // pick small point
//            for (int k = sp; k <= ep; k++)
//            {
//                int ind = mCloudSmoothness[k].second;
//                if (mNeighborPicked[ind] == 0 && mCurvature[ind] < surfThreshold)
//                {
//                    mCloudLabel[ind] = -1;
//                    // pick surf point
//                    mNeighborPicked[ind] = 1;
//                    // pick surround points
//                    for (int l = 1; l <= 5; l++) {
//                        int columnDiff = abs(int(mPointColInd[ind + l] - mPointColInd[ind + l - 1]));
//                        if (columnDiff > 10)
//                            break;
//                        mNeighborPicked[ind + l] = 1;
//                    }
//                    for (int l = -1; l >= -5; l--) {
//
//                        int columnDiff = abs(int(mPointColInd[ind + l] - mPointColInd[ind + l + 1]));
//                        if (columnDiff > 10)
//                            break;
//                        mNeighborPicked[ind + l] = 1;
//                    }
//                }
//            }
//
//            for (int k = sp; k <= ep; k++)
//            {
//                if (mCloudLabel[k] <= 0){
//                    surfaceCloud->push_back(mCloud->points[k]);
//                }
//            }
//        }
//
//    }
//
//
//}
//
//
//#include <pcl/filters/approximate_voxel_grid.h>
//#include <fast_gicp/gicp/fast_gicp.hpp>
//#include <fast_gicp/gicp/fast_vgicp.hpp>
//
//float downsample_resolution = 1.0;
//pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
//fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
//Eigen::Isometry3d GICPPoses = Eigen::Isometry3d::Identity();
//Eigen::Isometry3d STICPPoses = Eigen::Isometry3d::Identity();
//
//#include "pcl/registration/icp.h"
//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> sticp;
//
//
//ros::Publisher pubCorner;
//ros::Publisher pubSurf;
//ros::Publisher pubSemantic;
//
//ros::Publisher pubOdomGICP;
//ros::Publisher pubOdomSTICP;
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr lastIcpCloud(new pcl::PointCloud<pcl::PointXYZ>());
//
//void handle(const sensor_msgs::PointCloud2::ConstPtr &msg){
//    pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<PointType>::Ptr corner(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<PointType>::Ptr surf(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr icpCloud(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::fromROSMsg(*msg, *tmpCloud);
//
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//
//    //Semantic pick to remove dynamic
//    for(auto& point : tmpCloud->points){
//        double range = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
//        /*if( point.ring < param_laser_min_line_index ||
//            point.ring > param_laser_max_line_index ||
//            range < min_laser_range ||
//            range > max_laser_range ){
//            continue;
//        }*/
//        if( point.ring % 2 == 0) continue;
//        if( point.ring <= param_laser_min_line_index || point.ring >= param_laser_max_line_index) continue;
//
//        int label = labelClass[point.label];
//        if( label == 0 || label == 1 || label == 2 ) {
//            pcl::PointXYZ tmpPoint;
//            tmpPoint.x = point.x;
//            tmpPoint.y = point.y;
//            tmpPoint.z = point.z;
//            icpCloud->push_back(tmpPoint);
//        }
//    }
//    tmpCloud->clear();
//    voxelgrid.setInputCloud(icpCloud);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
//    voxelgrid.filter(*source);
//    auto t2 = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count();
//    cout << "remove dynamic " << t2*1000 << endl;
//
//    nav_msgs::Odometry GICPodomMsg;
//    nav_msgs::Odometry STICPodomMsg;
//
//    ///GICP
////    t1 = chrono::steady_clock::now();
////    static bool isFirst = true;
////    if(isFirst){
////        gicp.setInputSource(source);
////        isFirst = false;
////    }else{
////        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
////        gicp.setInputSource(source);
////        gicp.align(*aligned);
////        gicp.swapSourceAndTarget();
////        GICPPoses = GICPPoses * gicp.getFinalTransformation().cast<double>();
////        Pose robotPose(GICPPoses);
////        GICPodomMsg = Pose2RosMsg(robotPose);
////    }
////    GICPodomMsg.header.frame_id = "laser";
////    auto t3 = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count();
////    cout << "gicp " << t3*1000 << endl;
//
//
//    //LOAM Match
////    t1 = chrono::steady_clock::now();
////    ExtractFeatures(AllCloud, corner, surf);
////    auto t3 = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count();
////    cout << "extract " << t3*1000 << endl;
//
//
//    //stICP
//    t1 = chrono::steady_clock::now();
//    if(!source->empty() && !lastIcpCloud->empty()){
//        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
//        sticp.setInputSource(lastIcpCloud);
//        sticp.setInputTarget(source);
//        sticp.align(*aligned);
//        STICPPoses = STICPPoses * sticp.getFinalTransformation().inverse().cast<double>();
//        Pose tmp(STICPPoses);
//        STICPodomMsg = Pose2RosMsg(tmp);
//    }
//    STICPodomMsg.header.frame_id = "laser";
//    auto t5 = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count();
//    cout << "sticp " << t5*1000 << endl;
//
//
//    //pub the result to compare
//    sensor_msgs::PointCloud2 msgCorner,msgSurf,sourceCloud;
//    pcl::toROSMsg(*corner, msgCorner);
//    pcl::toROSMsg(*surf, msgSurf);
//    pcl::toROSMsg(*source, sourceCloud);
//    msgCorner.header.frame_id = "laser";
//    msgSurf.header.frame_id = "laser";
//    sourceCloud.header.frame_id = "laser";
//
//    pubSemantic.publish(sourceCloud);
//    pubCorner.publish(msgCorner);
//    pubSurf.publish(msgSurf);
//
//    pubOdomGICP.publish(GICPodomMsg);//GICP
//    pubOdomSTICP.publish(STICPodomMsg);//GICP
//
//    //update the cloud and free the memory
//    lastIcpCloud->clear();
//    *lastIcpCloud = *source;
//    icpCloud->clear();
//    corner->clear();
//    surf->clear();
//}
//
//
//int main(int argc, char** argv){
//    ros::init(argc, argv, "demo");
//    ros::NodeHandle nh;
//
//
//    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
//    gicp.setMaxCorrespondenceDistance(1.0);
//
////    sticp.setTransformationEpsilon(0.1);
////    sticp.setMaxCorrespondenceDistance(0.1);
//
//    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, handle);
//    pubCorner = nh.advertise<sensor_msgs::PointCloud2>("/corner", 1);
//    pubSurf = nh.advertise<sensor_msgs::PointCloud2>("/surf", 1);
//    pubSemantic = nh.advertise<sensor_msgs::PointCloud2>("/semantic", 1);
//
//    pubOdomGICP = nh.advertise<nav_msgs::Odometry>("/GICPodom", 10);
//    pubOdomSTICP = nh.advertise<nav_msgs::Odometry>("/STICPodom", 10);
//
//    ros::spin();
//    return 0;
//}
/***********^************************^**************/
/***********^************************^**************/
/***********|******test tracking*****|**************/
/***********|************************|**************/
/***********|************************|**************/


#include "Base.h"
#include "ClusterCVC.h"
#include "SemanticMark.h"


using namespace std;


semanticCloud createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max);
semanticCloud  createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B);

ros::Publisher pubSemantic;
ros::Publisher pubSMarks;

void handle(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<VelodynePointXYZILR> RawData;
    semanticCloudPtr semantic(new semanticCloud());

    pcl::fromROSMsg(*msg, RawData);

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // Semantic pick to remove dynamic
    for (auto &point : RawData.points) {
        int label = labelClass[point.label];
        if (label == 1 || label == 2) {
            PointType tmp;
            tmp.x = point.x;
            tmp.y = point.y;
            tmp.z = point.z;
            tmp.label = point.label;
            semantic->push_back(tmp);
        }
    }
    auto t2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    cout << "remove dynamic " << t2*1000 << endl;

    // Cluster for Extrack SMarks
    t1 = chrono::steady_clock::now();

    // Create the Cluster
    vector<cvc::PointAPR> papr;
    cvc::calculateAPR(*semantic, papr);
    unordered_map<int, cvc::Voxel> hvoxel;
    cvc::build_hash_table(papr, hvoxel);
    vector<int> cluster_index = CVC(hvoxel, papr);
    vector<int> cluster_id;
    cvc::most_frequent_value(cluster_index, cluster_id);
    // Count the Class number
    map<int, vector<int>> classes;
    for(int i = 0; i < cluster_index.size(); i++){
        auto it = classes.find(cluster_index[i]);
        if(it == classes.end()){
            vector<int> tmp;
            tmp.push_back(i);
            classes.insert(make_pair(cluster_index[i], tmp));
        }else{
            it->second.push_back(i);
        }
    }
    for(auto aclass = classes.begin(); aclass != classes.end(); ){
        if(aclass->second.size() < 30) classes.erase(aclass++);
        else aclass++;
    }


    // Smarks
    vector<lso::SemanticMark*> SMarks;
    for(auto& aclass : classes){
        semanticCloudPtr tempCloud(new semanticCloud());
        if(aclass.second.size() < 30) continue;
        for(auto index : aclass.second ){
            tempCloud->push_back(semantic->points[index]);
        }
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*tempCloud, min, max);
        Eigen::Vector3d tmpSize(max(0)-min(0),max(1)-min(1),max(2)-min(2));
        Pose tmpPoseture;
        lso::SemanticMark* tmpMark = new lso::SemanticMark(tempCloud, tmpPoseture, tmpSize);
        SMarks.push_back(tmpMark);
    }

    auto t3 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    cout << "extract SMakrs " << t3*1000 << endl;

    semanticCloudPtr result(new semanticCloud());
    cout << " class " << classes.size() << endl;
    for(auto& aclass : classes){
        semanticCloud tempCloud;
        if(aclass.second.size() < 30) continue;
        for(auto index : aclass.second ){
            tempCloud.push_back(semantic->points[index]);
        }
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(tempCloud, min, max);
        tempCloud += createFrameCloud(min, max);
        *result += tempCloud;
    }

    cout << " total " << result->points.size() << endl;
    sensor_msgs::PointCloud2 tmpMsgSemantic;

    pcl::toROSMsg(*semantic, tmpMsgSemantic);
    tmpMsgSemantic.header.frame_id = "laser";
    pubSemantic.publish(tmpMsgSemantic);

    sensor_msgs::PointCloud2 tmpMsgMarks;
    pcl::toROSMsg(*result, tmpMsgMarks);
    tmpMsgMarks.header.frame_id = "laser";
    pubSMarks.publish(tmpMsgMarks);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, handle);
    pubSemantic = nh.advertise<sensor_msgs::PointCloud2>("/semantic", 1);
    pubSMarks = nh.advertise<sensor_msgs::PointCloud2>("/marks", 1);

    ros::spin();
    return 0;
}

/*
 * 两点之间绘制直线，100个点每米
 */
semanticCloud createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B)
{
    semanticCloud result;
    result.clear();
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    double nums = distance * 100;
    for(int i = 0; i < nums; i++)
    {
        PointType tempPoint;
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
semanticCloud createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max)
{
    semanticCloud frame;
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




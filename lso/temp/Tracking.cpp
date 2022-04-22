#include <memory>

#include "Base.h"
#include "Frame.h"
#include "SemanticMark.h"
#include "ClusterCVC.h"
#include "Optimizer.h"

std_msgs::Header semanticMsgHeader;
bool isNewCloud = false ;
float downsample_resolution = 1.0;
pcl::ApproximateVoxelGrid<PointType> trackingFilter;
pcl::IterativeClosestPoint<PointType, PointType> sticp;
geometryCloudPtr semanticRawCloud(new geometryCloud());//Tracking
semanticCloudPtr semanticGroundCloud(new semanticCloud());//Ground
semanticCloudPtr semanticEnviCloud(new semanticCloud());//Extract object
lso::Frame::Ptr LastFrame;
lso::Frame::Ptr NowFrame;
lso::Map LocalMap;
ros::Publisher PubOdom;
ros::Publisher PubGeomotryMap;
ros::Publisher PubSemanticMarksMap;
void handleLaser(const sensor_msgs::PointCloud2::ConstPtr &msg);
void Tracking();
///////////////////////////////////////////////

int main(int argc, char** argv){
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh;

    trackingFilter.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, handleLaser);
    PubGeomotryMap = nh.advertise<sensor_msgs::PointCloud2>("/GeoMap", 1);
    PubSemanticMarksMap = nh.advertise<sensor_msgs::PointCloud2>("/MarksMap", 1);
    PubOdom = nh.advertise<nav_msgs::Path>("/path", 10);
    ros::Rate loop(30);
    while (ros::ok()){
        if(isNewCloud){
            Tracking();
            isNewCloud = false;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

void handleLaser(const sensor_msgs::PointCloud2::ConstPtr &msg){

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Clear the old cloud
    semanticRawCloud->clear();
    semanticGroundCloud->clear();
    semanticEnviCloud->clear();

    semanticMsgHeader = msg->header;
    pcl::PointCloud<VelodynePointXYZILR> RawData;
    semanticCloudPtr semantic(new semanticCloud());
    pcl::fromROSMsg(*msg, RawData);

    // Semantic pick to remove dynamic
    for (auto &point : RawData.points) {
        int label = labelClass[point.label];
        PointType tP;
        tP.label = point.label;
        tP.x = point.x;
        tP.y = point.y;
        tP.z = point.z;
        if( label == 0 ) {// Ground
            semanticGroundCloud->push_back(tP);
        }else if( label == 1 || label == 2 ) {// Static Object
            semanticEnviCloud->push_back(tP);
        }
    }

    *semanticRawCloud = *semanticEnviCloud + *semanticGroundCloud;
    //std::cout << " ground " << semanticGroundCloud->points.size()
    //          << " envi   " << semanticEnviCloud->points.size()
    //          << " all    " << semanticRawCloud->points.size()
    //          << std::endl;
    trackingFilter.setInputCloud(semanticRawCloud);
    trackingFilter.filter(*semanticRawCloud);
    //std::cout << " all    " << semanticRawCloud->points.size();
    isNewCloud = true;

    auto t2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "remove dynamic " << t2*1000 << std::endl;
}

void generateFrameAndSMarks();
void getInitialPose();
void registerMap();
void publishMapAndOdom();

void Tracking(){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    generateFrameAndSMarks();// Get the Frame and its Cloud and Smarks
    auto t3 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "generateFrameAndSMarks " << t3*1000 << std::endl;
    t1 = std::chrono::steady_clock::now();
    getInitialPose();// To Initialized Now Frame's Pose And its Smarks's Pose
    auto t4 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "getInitialPose " << t4*1000 << std::endl;
    t1 = std::chrono::steady_clock::now();
    Optimizer::PoseOptimization(NowFrame, LastFrame);// Optimize Now Frame Pose only Between Adjacent
    auto t5 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "PoseOptimization " << t5*1000 << std::endl;
    t1 = std::chrono::steady_clock::now();
    registerMap();// Update the Local Map
    auto t6 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "registerMap " << t6*1000 << std::endl;
    t1 = std::chrono::steady_clock::now();
    publishMapAndOdom();
    auto t7 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
    //std::cout << "publishMapAndOdom " << t7*1000 << std::endl;
}

bool NeedNewRefFrame(){
    double tmpDis = getDistanceBetween2Poses(LastFrame->mPose, NowFrame->mPose);
    if(tmpDis > 1.0) return true;
    else return false;
}

bool NeedNewKeyFrame(){
    return NeedNewRefFrame();
}

void getInitialPose(){
    //
    if(LastFrame == nullptr){
        LastFrame = NowFrame;
        return ;
    }
    sticp.setInputSource(LastFrame->mCloud);
    sticp.setInputTarget(NowFrame->mCloud);
    semanticCloudPtr tmpCloud(new semanticCloud());
    sticp.align(*tmpCloud);

    // Update Frame Pose
    Eigen::Isometry3d relativePse(sticp.getFinalTransformation().inverse().cast<double>());
    NowFrame->mPose.SetWorldPose(LastFrame->mPose.Posture * relativePse);
    //std::cout << "Front-End:" << relativePse(0,3) << " " << relativePse(1,3) << " " << relativePse(2,3) << std::endl;

    // Update SMarks Pose
    for( auto& mark : NowFrame->VisibleSMarks){
        mark->mPose.SetWorldPose(NowFrame->mPose.Position+mark->mSize, NowFrame->mPose.Orientation);
    }
    //std::cout << "Front-End:" << NowFrame->VisibleSMarks.size() << endl;
}

void generateFrameAndSMarks(){
    //Create the frame
    NowFrame = lso::Frame::Ptr(new lso::Frame(semanticRawCloud, semanticGroundCloud, semanticEnviCloud, semanticMsgHeader.stamp.toSec()));
            //std::make_shared<lso::Frame>;

    //Extract the Semantic Marks
    // Create the Cluster
    std::vector<cvc::PointAPR> papr;
    cvc::calculateAPR(*semanticEnviCloud, papr);
    std::unordered_map<int, cvc::Voxel> hvoxel;
    cvc::build_hash_table(papr, hvoxel);
    std::vector<int> cluster_index = CVC(hvoxel, papr);
    std::vector<int> cluster_id;
    cvc::most_frequent_value(cluster_index, cluster_id);
    // Count the Class number
    std::map<int, std::vector<int>> classes;
    for(int i = 0; i < cluster_index.size(); i++){
        auto it = classes.find(cluster_index[i]);
        if(it == classes.end()){
            std::vector<int> tmp;
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
    std::set<lso::SemanticMark::Ptr> SMarksSets;
    for(auto& aclass : classes){
        semanticCloudPtr tempCloud(new semanticCloud());
        if(aclass.second.size() < 30) continue;
        for(auto index : aclass.second ){
            tempCloud->push_back(semanticEnviCloud->points[index]);
        }
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*tempCloud, min, max);
        Eigen::Vector3d tmpSize(max(0)-min(0),max(1)-min(1),max(2)-min(2));
        for(auto& point : tempCloud->points){
            point.x -= tmpSize(0);
            point.y -= tmpSize(1);
            point.z -= tmpSize(2);
        }
        Pose tmpPoseture;
        lso::SemanticMark::Ptr tmpMark = std::make_shared<lso::SemanticMark>(tempCloud, tmpPoseture, tmpSize);
        SMarksSets.insert(tmpMark);
    }
    // Combine the Frame and Smarks
    NowFrame->VisibleSMarks = SMarksSets;
}

void registerMap(){
    if(!NeedNewKeyFrame()){
        return;
    }
    // Add New Key Frame
    lso::KeyFrame::Ptr tmpKF(new lso::KeyFrame(*NowFrame));
    LocalMap.PushFrame(tmpKF);

    std::cout<<"Add A New KeyFrame ======> " << tmpKF->Ind << std::endl;

    // Maintain the last time 30s
    LocalMap.MaintainKeepSec(semanticMsgHeader.stamp.toSec(), 30);

    // Update Ref Frame
    LastFrame = NowFrame;
}

void publishMapAndOdom(){
    sensor_msgs::PointCloud2 tmpMsg;
    LocalMap.GenerateGeometryMapCloudMsg(tmpMsg);
    tmpMsg.header.frame_id = "laser";
    PubGeomotryMap.publish(tmpMsg);

    LocalMap.GenerateSemanticMapCloudMsg(tmpMsg);
    tmpMsg.header.frame_id = "laser";
    PubSemanticMarksMap.publish(tmpMsg);

    nav_msgs::Path msgPath;
    LocalMap.GeneratePath(msgPath);
    msgPath.header.frame_id = "laser";
    PubOdom.publish(msgPath);
    std::cout << " Map " << LocalMap.KeyFrameNum << " " << LocalMap.GeoSize << std::endl;
}


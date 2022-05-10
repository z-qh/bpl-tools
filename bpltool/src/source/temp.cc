
#define PCL_NO_PRECOMPILE

#include "iostream"
#include "fstream"

#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "nav_msgs/Odometry.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint32_t, label, label)
)


struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time)
)


pcl::PointCloud<PointXYZIL>::Ptr transformPointCloud(pcl::PointCloud<PointXYZIL>::Ptr cloudIn, PointXYZIRPYT& transformIn){

    pcl::PointCloud<PointXYZIL>::Ptr cloudOut(new pcl::PointCloud<PointXYZIL>());

    PointXYZIL *pointFrom;
    PointXYZIL pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];

        float x1 = cos(transformIn.yaw) * pointFrom->x - sin(transformIn.yaw) * pointFrom->y;
        float y1 = sin(transformIn.yaw) * pointFrom->x + cos(transformIn.yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn.roll) * y1 - sin(transformIn.roll) * z1;
        float z2 = sin(transformIn.roll) * y1 + cos(transformIn.roll)* z1;

        pointTo.x = cos(transformIn.pitch) * x2 + sin(transformIn.pitch) * z2 + transformIn.x;
        pointTo.y = y2 + transformIn.y;
        pointTo.z = -sin(transformIn.pitch) * x2 + cos(transformIn.pitch) * z2 + transformIn.z;
        pointTo.intensity = pointFrom->intensity;
        pointTo.label = pointFrom->label;
        cloudOut->points[i] = pointTo;
    }
    

    return cloudOut;
}

void transRPY(pcl::PointCloud<PointXYZIRPYT>&pose){

    Eigen::Matrix4d gauss = Eigen::Matrix4d::Identity();
    gauss <<  -1, 0, 0, 0,
               0, 0, 1, 0,
               0, 1, 0, 0,
               0, 0, 0, 1;
    // gauss.block<3,3>(0,0).normalize();
    for(auto&p:pose.points){
        double R,P,Y;
        R = p.roll;
        P = p.pitch;
        Y = p.yaw;
        double x,y,z;
        x = p.x;
        y = p.y;
        z = p.z;
        Eigen::Vector3d posi(x, y, z);
        Eigen::Matrix3d ori(Eigen::AngleAxisd(Y,Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(P,Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(R,Eigen::Vector3d::UnitX()) );
        //
        //
        auto resR2 = gauss.block<3,3>(0,0) * ori * gauss.block<3,3>(0,0).transpose();
        auto test2 = gauss.block<3,3>(0,0) * posi + gauss.block<3,1>(0,3);
        //
        //
        x = test2(0);
        y = test2(1);
        z = test2(2);
        Y = resR2.eulerAngles(2,1,0)(0);
        P = resR2.eulerAngles(2,1,0)(1);
        R = resR2.eulerAngles(2,1,0)(2);
        //
        p.x = x;
        p.y = y;
        p.z = z;
        p.roll = R;
        p.pitch = P;
        p.yaw = Y;
    }
}

void pubEigenPose(ros::Publisher& pub, PointXYZIRPYT& pose){
    Eigen::Quaterniond tQ(Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX()));
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = pose.z;
    odom.pose.pose.orientation.x = tQ.x();
    odom.pose.pose.orientation.y = tQ.y();
    odom.pose.pose.orientation.z = tQ.z();
    odom.pose.pose.orientation.w = tQ.w();
    pub.publish(odom);
}

pcl::PointCloud<PointXYZIRPYT> readFile(string filePath){
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file(filePath);
    string header;
    getline(file, header);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        PointXYZIRPYT tmpP;
        int index;
        double time;
        double x,y,z;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

pcl::PointCloud<PointXYZIRPYT> readFileNoHead(string filePath){
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file(filePath);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        PointXYZIRPYT tmpP;
        int index;
        double time;
        double x,y,z;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

void generateDaquanLast(ros::Publisher& pubOdom, ros::Publisher& pubCloud){


    pcl::VoxelGrid<PointXYZIL> downSizeFilter;
    downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

    auto poses = readFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematic.txt");
    transRPY(poses);


    pcl::PointCloud<PointXYZIL>::Ptr allCloud(new pcl::PointCloud<PointXYZIL>());
    rosbag::Bag GNSS;
    GNSS.open("/home/qh/2021-01-19-11-10-53DaQuanL.bag", rosbag::bagmode::Read);
    sensor_msgs::PointCloud2 toPub;
    vector<string> topics;
    topics.push_back("/lslidar_point_cloud");
    rosbag::View VV(GNSS, rosbag::TopicQuery(topics));
    auto it = VV.begin();
    int poseCount = 0;
    int succeedCount = 0;
    vector<pcl::PointCloud<PointXYZIL>::Ptr> vec_cloud;
    for(it = VV.begin(); it != VV.end(); it++)
    {
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 tempMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            pcl::PointCloud<PointXYZIL>::Ptr tempCloud(new pcl::PointCloud<PointXYZIL>());
            pcl::fromROSMsg(tempMsg, *tempCloud);
            for(auto&p:tempCloud->points){
                float x = p.x;
                p.x = -p.y;
                p.y = x;
            }
            if(poseCount >= poses.size() ) break;
            if(abs(poses[poseCount].time - tempMsg.header.stamp.toSec()) < 0.02){
                auto transedTempCloud = transformPointCloud(tempCloud, poses[poseCount]);
                *allCloud += *transedTempCloud;
                downSizeFilter.setInputCloud(allCloud);
                downSizeFilter.filter(*allCloud);
                if(allCloud->size() > 200000){
                    cout << " SECTION ================== " << endl;
                    vec_cloud.push_back(allCloud);
                    allCloud.reset(new pcl::PointCloud<PointXYZIL>());
                }
                // char key = getchar();
                // if(key == 'q') break;
                // pcl::toROSMsg(*transedTempCloud, toPub);
                // toPub.header = tempMsg.header;
                // toPub.header.frame_id = "map";
                // pubCloud.publish(toPub);
                cout << poseCount << " Get Cloud " << transedTempCloud->size() << " " << allCloud->size() << endl;
                // pubEigenPose(pubOdom, poses[poseCount]);
                poseCount++;
                succeedCount++;
            }
            else if( tempMsg.header.stamp.toSec() - poses[poseCount].time > 0.05 ){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
            }
        }
    }
    vec_cloud.push_back(allCloud);
    allCloud.reset(new pcl::PointCloud<PointXYZIL>());

    int total_points = 0;
    for(auto&p:vec_cloud){
        total_points += p->points.size();
        *allCloud += *p;
    }
    cout << " all Cloud size : " << total_points * sizeof(PointXYZIL) / 1024.0 / 1024.0 << " MB " << endl;

    cout << "Succeed: " << succeedCount * 100.0 / poses.size() << "%" << endl;

    cout << " Total: " << total_points << endl;


    downSizeFilter.setInputCloud(allCloud);
    downSizeFilter.filter(*allCloud);
    pcl::io::savePCDFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematicDS.pcd", *allCloud);


}


void generateO1(ros::Publisher& pubOdom, ros::Publisher& pubCloud){
    auto poses = readFileNoHead("/home/qh/YES/add_ws/10/semantic_long_/RTKMAPafter.txt");
    // transRPY(poses);

    pcl::VoxelGrid<PointXYZIL> downSizeFilter;
    downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

    pcl::PointCloud<PointXYZIL>::Ptr allCloud(new pcl::PointCloud<PointXYZIL>());
    rosbag::Bag GNSS;

    GNSS.open("/home/qh/o1.bag", rosbag::bagmode::Read);

    sensor_msgs::PointCloud2 toPub;

    vector<string> topics;
    topics.push_back("/laser");
    rosbag::View VV(GNSS, rosbag::TopicQuery(topics));
    auto it = VV.begin();
    int poseCount = 0;
    int succeedCount = 0;
    vector<pcl::PointCloud<PointXYZIL>::Ptr> vec_cloud;
    for(it = VV.begin(); it != VV.end(); it++)
    {
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 tempMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            pcl::PointCloud<PointXYZIL>::Ptr tempCloud(new pcl::PointCloud<PointXYZIL>());
            pcl::fromROSMsg(tempMsg, *tempCloud);
            if(poseCount > 300 && poseCount < 590){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
                continue;
            }
            if(poseCount >= poses.size() ) break;
            if(abs(poses[poseCount].time - tempMsg.header.stamp.toSec()) < 0.02){
                auto transedTempCloud = transformPointCloud(tempCloud, poses[poseCount]);
                *allCloud += *transedTempCloud;
                downSizeFilter.setInputCloud(allCloud);
                downSizeFilter.filter(*allCloud);
                if(allCloud->size() > 200000){
                    cout << " SECTION ================== " << vec_cloud.size() << endl;
                    vec_cloud.push_back(allCloud);
                    allCloud.reset(new pcl::PointCloud<PointXYZIL>());
                }
                // char key = getchar();
                // if(key == 'q') break;
                // pcl::toROSMsg(*transedTempCloud, toPub);
                // toPub.header = tempMsg.header;
                // toPub.header.frame_id = "map";
                // pubCloud.publish(toPub);
                // cout << poseCount << " Get Cloud " << transedTempCloud->size() << " " << allCloud->size() << endl;
                // pubEigenPose(pubOdom, poses[poseCount]);
                poseCount++;
                succeedCount++;
            }
            else if( tempMsg.header.stamp.toSec() - poses[poseCount].time > 0.05 ){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
            }
        }
    }
    vec_cloud.push_back(allCloud);
    allCloud.reset(new pcl::PointCloud<PointXYZIL>());

    int total_points = 0;
    for(auto&p:vec_cloud){
        total_points += p->points.size();
        *allCloud += *p;
    }

    cout << " all Cloud size : " << total_points * sizeof(PointXYZIL) / 1024.0 / 1024.0 << " MB " << endl;
    cout << "Succeed: " << succeedCount * 100.0 / poses.size() << "%" << endl;
    cout << " Total: " << total_points << endl;


    downSizeFilter.setInputCloud(allCloud);
    downSizeFilter.filter(*allCloud);
    pcl::io::savePCDFile("/home/qh/YES/add_ws/10/semantic_long_/RTKMAPafterDS.pcd", *allCloud);


}

int main(int argc, char** argv){
    ros::init(argc, argv, "temp");
    ros::NodeHandle nh;

    ros::Publisher pubOdom = nh.advertise<nav_msgs::Odometry>("/temp", 1);
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("tempcloud", 1);
    // generateDaquanLast(pubOdom, pubCloud);
    generateO1(pubOdom, pubCloud);
    return 0;
}

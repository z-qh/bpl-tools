#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "nav_msgs/Odometry.h"
#include "gnss_driver/gps_navi_msg.h"
#include "sensor_msgs/PointCloud2.h"

#include "vector"
#include "string"

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/io.h"

#include "pcl/registration/ndt.h"
#include "pcl/registration/icp.h"
#include <map>

//////////////////////////////////////////////
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    double index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time) (double, index, index)
)

typedef PointXYZIRPYT  PointTypePose;
//////////////////////////////////////////////////
typedef pcl::PointXYZI PointType;

using namespace std;

bool getGlobalCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string filePath, vector<PointTypePose>& poses);
bool newCloud = false;
double laserCloudTime = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud;
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterTemp;
mutex muteTempCloud;
void subCloudHandle(const sensor_msgs::PointCloud2ConstPtr &msgs)
{
    laserCloudTime = msgs->header.stamp.toSec();
    muteTempCloud.lock();
    tempCloud->clear();
    pcl::fromROSMsg(*msgs, *tempCloud);
    vector<int> nanIndex;
    pcl::removeNaNFromPointCloud(*tempCloud, *tempCloud, nanIndex);
    newCloud = true;
    muteTempCloud.unlock();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap(new pcl::PointCloud<pcl::PointXYZI>());
vector<PointTypePose> globalPoses;

string locateCloudTopic = "/lslidar_points";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "relocateTim");
    ros::NodeHandle nh("~");
    nh.getParam("locateCloudTopic", locateCloudTopic);

    if(getGlobalCloud(globalMap, "/media/qh/Samsung USB/TIM/cloud.txt", globalPoses)){
        cout << "load file succeed !" << endl;
        pcl::io::savePCDFileASCII("/media/qh/Samsung USB/TIM/cloud.pcd", *globalMap);
    }else{
        cout << "load file filed !" << endl;
        return 0;
    }

    downSizeFilterTemp.setLeafSize(1.0f, 1.0f, 1.0f);
    ros::Subscriber subCloud = nh.subscribe<sensor_msgs::PointCloud2>(locateCloudTopic, 1, subCloudHandle);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    int tryTimes = 0;
    ros::Rate loop(20);
    while(ros::ok())
    {
        if(newCloud)
        {
            muteTempCloud.lock();
            //NDT match
            //ICP match
            muteTempCloud.unlock();
        }
        loop.sleep();
    }
    return 0;
}

bool NDTMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr nowCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr priorMap, PointTypePose& pose)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>ndt;
    ndt.setTransformationEpsilon(1);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
    ndt.setInputCloud(nowCloud);
    ndt.setInputTarget(priorMap);

    Eigen::AngleAxisf rot_x_init(0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisf rot_y_init(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisf rot_z_init(0, Eigen::Vector3d::UnitZ());

    Eigen::Translation3f t_init(0, 0, 0);
    Eigen::Matrix4f T_init = (t_init * rot_z_init * rot_y_init * rot_x_init).matrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZI>());

    ndt.align(*outCloud, T_init);
    if(ndt.hasConverged()){
        cout << "match score " << ndt.getFitnessScore() << endl;
        Eigen::Matrix4f tran = ndt.getFinalTransformation();
        pcl::transformPointCloud(*nowCloud, *nowCloud, tran);
        Eigen::Matrix3f R_final;
        R_final << tran(0,0),tran(0,1),tran(0,2),
        tran(1,0),tran(1,1),tran(1,2),
        tran(2,0),tran(2,1),tran(2,2);
        Eigen::Vector3f t_final(tran(0,3),tran(1,3),tran(2,3));
        pose.x = t_final(0);
        pose.y = t_final(1);
        pose.z = t_final(2);
        Eigen::Vector3f angle_final = R_final.eulerAngles(0, 1, 2);
        pose.roll = angle_final(0);
        pose.pitch = angle_final(1);
        pose.yaw = angle_final(2);
        return true;
    }else{
        return false;
    }
}

bool getGlobalCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string filePath, vector<PointTypePose>& poses)
{
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobal;
    downSizeFilter.setLeafSize(1.0f, 1.0f, 1.0f);
    downSizeFilterGlobal.setLeafSize(1.0f, 1.0f, 1.0f);
    ifstream file;
    file.open(filePath, ios::in);
    double lastX = 0, lastY = 0, lastZ = 0;
    double nowX = 0, nowY = 0, nowZ = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double poseTime = 0;
    double temp;
    int disThreshold = 10.0;
    bool isFirstFrame = true;
    int sumNums = 0;
    int pointsNums = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
    try{
        while(!file.eof()){
            file >> temp >> poseTime >> nowX >> nowY >> nowZ >> roll >> pitch >> yaw >> temp >> pointsNums;
            sumNums += pointsNums;
            PointTypePose tempPose;
            tempPose.time = poseTime;
            tempPose.x = nowX;
            tempPose.y = nowY;
            tempPose.z = nowZ;
            tempPose.roll = roll;
            tempPose.pitch = pitch;
            tempPose.yaw = yaw;
            poses.push_back(tempPose);
            tempCloud->points.resize(pointsNums);
            for(int i = 0; i < pointsNums; i++){
                double X,Y,Z,I;
                file >> X >> Y >> Z >> I;
                if(X*X + Y*Y + Z*Z < 2.5)
                    continue;
                tempCloud->points[i].x = X;
                tempCloud->points[i].y = Y;
                tempCloud->points[i].z = Z;
                tempCloud->points[i].intensity = I;
            }
            downSizeFilter.setInputCloud(tempCloud);
            downSizeFilter.filter(*tempCloud);

            *cloud += *tempCloud;
            if(isFirstFrame){
                lastX = nowX, lastY = nowY, lastZ = nowZ;
                isFirstFrame = false;
            }else{
                double dis = pow(nowX-lastX, 2) + pow(nowY-lastY, 2) + pow(nowZ-lastZ, 2);
                lastX = nowY, lastY = nowY, lastZ = nowZ;
                if(dis >= disThreshold){
                    downSizeFilterGlobal.setInputCloud(cloud);
                    downSizeFilterGlobal.filter(*cloud);
                }
            }
        }
    }
    catch(char *str)
    {
        cout << str << endl;
        file.close();
        return false;
    }
    std::cout << "before filter size " << sumNums * sizeof(pcl::PointXYZI)/1000/1000 << " MB." << endl;
    std::cout << "after  filter size " << cloud->points.size() * sizeof(pcl::PointXYZI)/1000/1000 << " MB." << endl;
    file.close();
    return true;
}
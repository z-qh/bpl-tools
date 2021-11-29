
#include "ros/ros.h"

#include "std_msgs/Header.h"

#include "iostream"
#include "sstream"
#include "fstream"
#include "map"

#include "Eigen/Eigen"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

struct VelodynePointXYZILR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZILR,
                                  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                  (uint16_t, label, label) (uint16_t, ring, ring) (float, time, time)
)

using namespace std;

class semanticProjection{
};


std::map<int,int> labelClass{//label-class 0:ground 1:struct 2:object 3:others
        {40, 0}, // "road"
        {48, 0}, // "sidewalk"
        {44, 0}, // "parking"
        {49, 0}, // "other-ground"
        {50, 1}, // "building"
        {52, 1}, // "other-structure"
        {71, 2}, // "trunk"
        {51, 2}, // "fence"
        {80, 2}, // "pole"
        {81, 2}, // "traffic-sign"
        {99, 2},// "other-object"
        {0 , 3}, // "unlabeled"
        {1 , 3}, // "outlier"
        {10, 3}, // "car"
        {11, 3}, // "bicycle"
        {13, 3}, // "bus"
        {15, 3}, // "motorcycle"
        {16, 3}, // "on-rails"
        {18, 3}, // "truck"
        {20, 3}, // "other-vehicle"
        {30, 3}, // "person"
        {31, 3}, // "bicyclist"
        {32, 3}, // "motorcyclist"
        {252, 3}, // "moving-car"
        {253, 3}, // "moving-bicyclist"
        {254, 3}, // "moving-person"
        {255, 3}, // "moving-motorcyclist"
        {256, 3}, // "moving-on-rails"
        {257, 3}, // "moving-bus"
        {258, 3}, // "moving-truck"
        {259, 3}, // "moving-other-vehicle"
        {70, 3}, // "vegetation"
        {60, 3}, // "lane-marking"
        {72, 3} // "terrain"
};

bool newDataFlag = false;
std_msgs::Header semanticMsgHeader;
pcl::PointCloud<VelodynePointXYZILR>::Ptr semanticRawCloud(new pcl::PointCloud<VelodynePointXYZILR>());
pcl::PointCloud<VelodynePointXYZILR>::Ptr semanticNoGroundCloud(new pcl::PointCloud<VelodynePointXYZILR>());
pcl::PointCloud<VelodynePointXYZILR>::Ptr semanticObjectCloud(new pcl::PointCloud<VelodynePointXYZILR>());
pcl::PointCloud<VelodynePointXYZILR>::Ptr semanticStructutreCloud(new pcl::PointCloud<VelodynePointXYZILR>());
int param_laser_max_line_index = 60;
int param_laser_min_line_index = 2;
double max_laser_range = 80.0;
double min_laser_range = 3.0;

void semanticRawHandle(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PointCloud<VelodynePointXYZILR> cloudTemp;
    pcl::fromROSMsg(*msg, cloudTemp);
    semanticMsgHeader = msg->header;
    newDataFlag = true;
    // for limit
    semanticRawCloud->clear();
    for(auto& point : cloudTemp.points){
        double range = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
        if( point.ring < param_laser_min_line_index ||
            point.ring > param_laser_max_line_index ||
            range < min_laser_range ||
            range > max_laser_range ){
            continue;
        }
        semanticRawCloud->push_back(point);
    }
}

void removeGround(){
    semanticNoGroundCloud->clear();
    semanticObjectCloud->clear();
    semanticStructutreCloud->clear();

    for(auto& thisPoint : semanticRawCloud->points){
        int label = labelClass[thisPoint.label];
        if( label != 0 ) semanticNoGroundCloud->push_back(thisPoint);
        if( label == 2 ) semanticObjectCloud->push_back(thisPoint);
        if( label == 1 ) semanticStructutreCloud->push_back(thisPoint);
    }

}

void getObject(){

}

void getBuilding(){

}

ros::Publisher pubSemanticNoGround;
ros::Publisher pubSemanticObject;
ros::Publisher pubSemanticStructure;
void publishCloud(){
    sensor_msgs::PointCloud2 tempMsg;

    pcl::toROSMsg(*semanticNoGroundCloud, tempMsg);
    tempMsg.header = semanticMsgHeader;
    pubSemanticNoGround.publish(tempMsg);

    pcl::toROSMsg(*semanticObjectCloud, tempMsg);
    tempMsg.header = semanticMsgHeader;
    pubSemanticObject.publish(tempMsg);

    pcl::toROSMsg(*semanticStructutreCloud, tempMsg);
    tempMsg.header = semanticMsgHeader;
    pubSemanticStructure.publish(tempMsg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic");
    ros::NodeHandle nh;

    ros::Subscriber subSemanticRaw = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 10, semanticRawHandle);
    //ros::Subscriber subMapOdom = nh.subscribe<nav_msgs::Odometry>("/odom")
    pubSemanticNoGround = nh.advertise<sensor_msgs::PointCloud2>("/semanticNoGround", 10);
    pubSemanticObject = nh.advertise<sensor_msgs::PointCloud2>("/semanticObject", 10);
    pubSemanticStructure = nh.advertise<sensor_msgs::PointCloud2>("/semanticStructure", 10);

    ros::Rate loop(30);
    while (ros::ok()){
        if(newDataFlag){
            newDataFlag = false;

            removeGround();
            publishCloud();
        }
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
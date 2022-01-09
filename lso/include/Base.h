#ifndef BASE_H_
#define BASE_H_

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "ros/ros.h"

#include "iostream"
#include "sstream"
#include "fstream"
#include "map"
#include "set"
#include "iterator"
#include "deque"

#include "Eigen/Eigen"

#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/filter.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/filters/approximate_voxel_grid.h"

#include "pcl/registration/icp.h"
#include "pcl/common/common.h"

class Pose{
public:
    Eigen::Vector3d Position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond Orientation = Eigen::Quaterniond::Identity();
    Eigen::Isometry3d Posture = Eigen::Isometry3d::Identity();
    Eigen::Vector3d RPY = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d Transform = Eigen::Matrix4d::Identity();
public:
    Pose(){};
    Pose(const Pose& pose_);
    Pose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_);
    Pose(const Eigen::Isometry3d& pose_);
    Pose(const Eigen::Vector3d &pos_, double R, double P, double Y);
    Pose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d& rotation_);
    Pose(const Eigen::Matrix4d &trans_);
    void SetWorldPose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_);
    void SetWorldPose(const Eigen::Isometry3d& pose_);
    void SetWorldPose(const Eigen::Vector3d &pos_, double R, double P, double Y);
    void SetWorldPose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d& rotation_);
    void SetWorldPose(const Eigen::Matrix4d &trans_);
    Pose Between(const Pose& pose_);
    friend std::ostream &operator<<(std::ostream& os, const Pose& p);
};

geometry_msgs::PoseStamped Pose2PoseStamped(Pose &p);
nav_msgs::Odometry Pose2RosMsg(Pose &p);
//Get relative pose distance
double getDistanceBetween2Poses(Pose&p1, Pose&p2);
//Eigen
void getRPYFromEigenQ(double&R, double&P, double&Y, const Eigen::Quaterniond&Q);
void getRPYFromEigenR(double&R, double&P, double&Y, const Eigen::Matrix3d&Rot);
//
///////////////////////////////////////////////
struct VelodynePointXYZILR
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    std::uint32_t label;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZILR,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(std::uint16_t, label, label) (std::uint16_t, ring, ring) (float, time, time)
)
////////////////////////////////////////////
typedef pcl::PointXYZL PointType;
typedef pcl::PointCloud<PointType>::Ptr semanticCloudPtr;
typedef pcl::PointCloud<PointType> semanticCloud;
typedef pcl::PointCloud<PointType>::Ptr geometryCloudPtr;
typedef pcl::PointCloud<PointType> geometryCloud;
///////////////////////////////////////////
extern std::map<int,int> labelClass;
///////////////////////////////////////////////
// Velodyne-64
extern int N_SCAN;
extern int Horizon_SCAN;
extern double ang_res_x;
extern int param_laser_min_line_index;
extern int param_laser_max_line_index;
extern double max_laser_range;
extern double min_laser_range;
extern double edgeThreshold;
extern double surfThreshold;
///////////////////////////////////////////////
#endif

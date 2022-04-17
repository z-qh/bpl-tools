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
    void SetPose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_);
    void SetPose(const Eigen::Isometry3d& pose_);
    void SetPose(const Eigen::Vector3d &pos_, double R, double P, double Y);
    void SetPose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d& rotation_);
    void SetPose(const Eigen::Matrix4d &trans_);
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
#endif

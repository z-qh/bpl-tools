#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "iostream"

#include "Eigen/Eigen"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

/*
 * Pose
 */
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
    friend std::ostream &operator<<(std::ostream& os, const Pose& p);
public:
    Pose(const Pose& pose_){
        Posture = pose_.Posture;
        Orientation = pose_.Orientation;
        Position = pose_.Position;
        RPY = pose_.RPY;
        Rotation = pose_.Rotation;
        Transform = pose_.Transform;
    }
    Pose(const Eigen::Isometry3d &pose_) {
        setPose(pose_);
    }
    Pose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
        setPose(pos_, R, P, Y);
    }
    Pose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
        setPose(pos_, ori_);
    }
    Pose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
        setPose(pos_, rotation_);
    }
    Pose(const Eigen::Matrix4d &trans_) {
        setPose(trans_);
    }
    void setPose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
        Position = pos_;
        RPY(0) = R;
        RPY(1) = P;
        RPY(2) = Y;
        Orientation = Eigen::Quaterniond(Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX()));
        Rotation = Eigen::Matrix3d(Orientation);
        Posture.setIdentity();
        Posture.pretranslate(Position);
        Posture.rotate(Rotation);
        Transform.block<3,3>(0, 0) = Rotation;
        Transform.block<3,1>(0, 3) = Position;
    }

    void setPose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
        Position = pos_;
        Orientation = ori_;
        Rotation = Eigen::Matrix3d(Orientation);
        RPY(2) = Rotation.eulerAngles(2,1,0)(0);
        RPY(1) = Rotation.eulerAngles(2,1,0)(1);
        RPY(0) = Rotation.eulerAngles(2,1,0)(2);
        Posture.setIdentity();
        Posture.pretranslate(Position);
        Posture.rotate(Rotation);
        Transform.block<3,3>(0, 0) = Rotation;
        Transform.block<3,1>(0, 3) = Position;
    }

    void setPose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
        Position = pos_;
        Rotation = rotation_;
        RPY(2) = Rotation.eulerAngles(2,1,0)(0);
        RPY(1) = Rotation.eulerAngles(2,1,0)(1);
        RPY(0) = Rotation.eulerAngles(2,1,0)(2);
        Posture.setIdentity();
        Posture.pretranslate(Position);
        Posture.rotate(Rotation);
        Transform.block<3,3>(0, 0) = Rotation;
        Transform.block<3,1>(0, 3) = Position;
    }

    void setPose(const Eigen::Isometry3d &pose_) {
        Position = pose_.translation();
        Rotation = pose_.rotation();
        RPY(2) = Rotation.eulerAngles(2,1,0)(0);
        RPY(1) = Rotation.eulerAngles(2,1,0)(1);
        RPY(0) = Rotation.eulerAngles(2,1,0)(2);
        Posture = pose_;
        Transform.block<3,3>(0, 0) = Rotation;
        Transform.block<3,1>(0, 3) = Position;
    }
    void setPose(const Eigen::Matrix4d &trans_) {
        Position = trans_.block<3, 1>(0, 3);
        Rotation = trans_.block<3, 3>(0, 0);
        RPY(2) = Rotation.eulerAngles(2,1,0)(0);
        RPY(1) = Rotation.eulerAngles(2,1,0)(1);
        RPY(0) = Rotation.eulerAngles(2,1,0)(2);
        Posture.setIdentity();
        Posture.pretranslate(Position);
        Posture.rotate(Rotation);
        Transform = trans_;
    }
    // A.Between(B) = A.inverse() * B,
    Pose Between(const Pose &pose_) {
        Pose res(Transform.inverse() * pose_.Transform);
        return res;
    }

    using Ptr = std::shared_ptr<Pose>;
    using ConstPtr = std::shared_ptr<const Pose>;
};
std::ostream &operator<<(std::ostream& os, const Pose& p){
    os << std::fixed << std::setprecision(1)
       << p.Position(0) << " " << p.Position(1) << " " << p.Position(2) << " "
       << p.RPY(0) << " " << p.RPY(1) << " " << p.RPY(2);
    return os;
}
geometry_msgs::PoseStamped Pose2PoseStamped(Pose &p);
nav_msgs::Odometry Pose2RosMsg(Pose &p);
double getDistanceBetween2Poses(Pose&p1, Pose&p2);
void getRPYFromEigenQ(double&R, double&P, double&Y, const Eigen::Quaterniond&Q);
void getRPYFromEigenR(double&R, double&P, double&Y, const Eigen::Matrix3d&Rot);
//
double getDistanceBetween2Poses(Pose&pose1, Pose&pose2){
    Eigen::Vector3d& p1 = pose1.Position;
    Eigen::Vector3d& p2 = pose2.Position;
    return sqrt(pow(p1(0)-p2(0), 2) + pow(p1(1)-p2(1), 2) + pow(p1(2)-p2(2), 2));
}

void getRPYFromEigenQ(double&R, double&P, double&Y, const Eigen::Quaterniond&Q){
    tf::Quaternion tmpTfQ(Q.x(), Q.y(), Q.z(), Q.w());
    tf::Matrix3x3(tmpTfQ).getRPY(R, P, Y);
}

void getRPYFromEigenR(double&R, double&P, double&Y, const Eigen::Matrix3d&Rot){
    Eigen::Quaterniond temQ(Rot);
    tf::Quaternion tmpTfQ(temQ.x(), temQ.y(), temQ.z(), temQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(R, P, Y);
}

geometry_msgs::PoseStamped Pose2PoseStamped(Pose &p){
    geometry_msgs::PoseStamped tp;
    tp.pose.position.x = p.Position(0);
    tp.pose.position.y = p.Position(1);
    tp.pose.position.z = p.Position(2);
    tp.pose.orientation.x = p.Orientation.x();
    tp.pose.orientation.y = p.Orientation.y();
    tp.pose.orientation.z = p.Orientation.z();
    tp.pose.orientation.w = p.Orientation.w();
    return tp;
}
//
#endif

#include "Base.h"

Pose::Pose(const Pose &p) {
    Posture = p.Posture;
    Orientation = p.Orientation;
    Position = p.Position;
    RPY = p.RPY;
    Rotation = p.Rotation;
}
Pose::Pose(const Eigen::Isometry3d &pose_) {
    SetPose(pose_);
}
Pose::Pose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
    SetPose(pos_, R, P, Y);
}
Pose::Pose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
    SetPose(pos_, ori_);
}
Pose::Pose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
    SetPose(pos_, rotation_);
}
Pose::Pose(const Eigen::Matrix4d &trans_) {
    SetPose(trans_);
}

void Pose::SetPose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
    Position = pos_;
    Eigen::Quaterniond tmpQ = Eigen::Quaterniond(Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
                                                 Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
                                                 Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX()));
    Orientation = tmpQ;
    Eigen::Matrix3d tmpR(tmpQ);
    Rotation = tmpR;
    Posture.pretranslate(pos_);
    Posture.rotate(tmpR);
    RPY(0) = R;
    RPY(1) = P;
    RPY(2) = Y;
    Transform.block<3,3>(0, 0) = Rotation;
    Transform.block<3,1>(0, 3) = Position;
}

void Pose::SetPose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
    Position = pos_;
    Orientation = ori_;
    Eigen::Matrix3d tmpR(ori_);
    Rotation = tmpR;
    Posture.pretranslate(pos_);
    Posture.rotate(Rotation);
    RPY(2) = Rotation.eulerAngles(2,1,0)(0);
    RPY(1) = Rotation.eulerAngles(2,1,0)(1);
    RPY(0) = Rotation.eulerAngles(2,1,0)(2);
    Transform.block<3,3>(0, 0) = Rotation;
    Transform.block<3,1>(0, 3) = Position;
}

void Pose::SetPose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
    Position = pos_;
    Rotation = rotation_;
    Eigen::Quaterniond tmpQ(rotation_);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(), tmpQ.y(), tmpQ.z(), tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture.pretranslate(pos_);
    Posture.rotate(rotation_);
    Transform.block<3,3>(0, 0) = Rotation;
    Transform.block<3,1>(0, 3) = Position;
}

void Pose::SetPose(const Eigen::Isometry3d &pose_) {
    Position = pose_.translation();
    Rotation = pose_.rotation();
    Eigen::Quaterniond tmpQ(Rotation);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(),tmpQ.y(),tmpQ.z(),tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture = pose_;
    Transform.block<3,3>(0, 0) = Rotation;
    Transform.block<3,1>(0, 3) = Position;
}
void Pose::SetPose(const Eigen::Matrix4d &trans_) {
    Position = trans_.block<3, 1>(0, 3);
    Rotation = trans_.block<3, 3>(0, 0);
    Eigen::Quaterniond tmpQ(Rotation);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(), tmpQ.y(), tmpQ.z(), tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture.pretranslate(Position);
    Posture.rotate(Rotation);
    Transform.block<3,3>(0, 0) = Rotation;
    Transform.block<3,1>(0, 3) = Position;
}

Pose Pose::Between(const Pose &pose_) {
    Pose res(Transform.inverse() * pose_.Transform);
    return res;
}

std::ostream &operator<<(std::ostream& os, const Pose& p){
    os << std::fixed << std::setprecision(1)
       << p.Position(0) << " " << p.Position(1) << " " << p.Position(2) << " "
       << p.RPY(0) << " " << p.RPY(1) << " " << p.RPY(2);
    return os;
}

nav_msgs::Odometry Pose2RosMsg(Pose &p){
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = p.Position(0);
    odom.pose.pose.position.y = p.Position(1);
    odom.pose.pose.position.z = p.Position(2);
    odom.pose.pose.orientation.x = p.Orientation.x();
    odom.pose.pose.orientation.y = p.Orientation.y();
    odom.pose.pose.orientation.z = p.Orientation.z();
    odom.pose.pose.orientation.w = p.Orientation.w();
    return odom;
}


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


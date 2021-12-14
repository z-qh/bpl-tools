#include "Base.h"

std::map<int,int> labelClass{//label-class 0:ground 1:struct 2:object 3:others
        {40, 0}, // "road"
        {48, 0}, // "sidewalk"
        {44, 0}, // "parking"
        {49, 0}, // "other-ground"
        {50, 1}, // "building"
        {52, 1}, // "other-structure"
        {51, 2}, // "fence"
        {80, 2}, // "pole"
        {81, 2}, // "traffic-sign"
        {99, 2}, // "other-object"
        {71, 2}, // "trunk"
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
///////////////////////////////////////////////
// Velodyne-64
int N_SCAN = 64;
int Horizon_SCAN = 1800;
double ang_res_x = 360.0 / Horizon_SCAN;
int param_laser_min_line_index = 2;
int param_laser_max_line_index = 60;
double max_laser_range = 80.0;
double min_laser_range = 3.0;
double edgeThreshold = 1.0;
double surfThreshold = 0.1;
///////////////////////////////////////////////
///////////////////////////////////////////////

Pose::Pose(const Pose &p) {
    Posture = p.Posture;
    Orientation = p.Orientation;
    Position = p.Position;
    RPY = p.RPY;
    Rotation = p.Rotation;
}
Pose::Pose(const Eigen::Isometry3d &pose_) {
    SetWorldPose(pose_);
}
Pose::Pose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
    SetWorldPose(pos_, R, P, Y);
}
Pose::Pose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
    SetWorldPose(pos_, ori_);
}
Pose::Pose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
    SetWorldPose(pos_, rotation_);
}
Pose::Pose(const Eigen::Matrix4d &trans_) {
    SetWorldPose(trans_);
}

void Pose::SetWorldPose(const Eigen::Vector3d &pos_, double R, double P, double Y) {
    Position = pos_;
    auto tmpTfQ = tf::createQuaternionFromRPY(R, P, Y);
    Eigen::Quaterniond tmpQ(tmpTfQ.w(), tmpTfQ.x(), tmpTfQ.y(), tmpTfQ.z());
    Orientation = tmpQ;
    Eigen::Matrix3d tmpR(tmpQ);
    Rotation = tmpR;
    Posture.pretranslate(pos_);
    Posture.rotate(tmpR);
    RPY(0) = R;
    RPY(1) = P;
    RPY(2) = Y;
}

void Pose::SetWorldPose(const Eigen::Vector3d &pos_, const Eigen::Quaterniond &ori_) {
    Position = pos_;
    Orientation = ori_;
    Eigen::Matrix3d tmpR(ori_);
    Rotation = tmpR;
    Posture.pretranslate(pos_);
    Posture.rotate(tmpR);
    tf::Quaternion tmpTfQ(ori_.x(), ori_.y(), ori_.z(), ori_.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
}

void Pose::SetWorldPose(const Eigen::Vector3d &pos_, const Eigen::Matrix3d &rotation_) {
    Position = pos_;
    Rotation = rotation_;
    Eigen::Quaterniond tmpQ(rotation_);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(), tmpQ.y(), tmpQ.z(), tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture.pretranslate(pos_);
    Posture.rotate(rotation_);
}

void Pose::SetWorldPose(const Eigen::Isometry3d &pose_) {
    Position = pose_.translation();
    Rotation = pose_.rotation();
    Eigen::Quaterniond tmpQ(Rotation);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(),tmpQ.y(),tmpQ.z(),tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture = pose_;
}
void Pose::SetWorldPose(const Eigen::Matrix4d &trans_) {
    Position = trans_.block<3, 1>(0, 3);
    Rotation = trans_.block<3, 3>(0, 0);
    Eigen::Quaterniond tmpQ(Rotation);
    Orientation = tmpQ;
    tf::Quaternion tmpTfQ(tmpQ.x(), tmpQ.y(), tmpQ.z(), tmpQ.w());
    tf::Matrix3x3(tmpTfQ).getRPY(RPY(0), RPY(1), RPY(2));
    Posture.pretranslate(Position);
    Posture.rotate(Rotation);
}

std::ostream &operator<<(std::ostream& os, Pose& p){
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


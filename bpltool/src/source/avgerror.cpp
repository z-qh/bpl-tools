#include "ros/ros.h"
#include "Eigen/Eigen"
#include "iostream"
#include "vector"
#include "string"
#include "fstream"

#include "nav_msgs/Odometry.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace std;

vector<pair<Eigen::Matrix4d,double>> readTumFile(string filePath){
    vector<pair<Eigen::Matrix4d,double>> result;
    ifstream file(filePath);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
        double time, X, Y, Z, W;
        ss >> time >> tmpT(0,3) >> tmpT(1,3) >> tmpT(2,3) >> X >> Y >> Z >> W;
        Eigen::Quaterniond tmpQ(W, X, Y, Z);
        Eigen::Matrix3d tmpR(tmpQ);
        tmpT.block<3,3>(0,0) = tmpR;
        result.push_back(make_pair(tmpT, time));
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

void saveTum(vector<pair<Eigen::Matrix4d,double>>&pose, string filePath){
    ofstream file(filePath);
    for(auto&p:pose){
        Eigen::Quaterniond tmpQ(p.first.block<3,3>(0,0));
        file << fixed << setprecision(6) << p.second << " "
             << p.first(0,3) << " "
             << p.first(1,3) << " "
             << p.first(2,3) << " "
             << tmpQ.x() << " "
             << tmpQ.y() << " "
             << tmpQ.z() << " "
             << tmpQ.w() << endl;
    }
    file.close();
}

vector<pair<Eigen::Matrix4d,double>> getRelative(vector<pair<Eigen::Matrix4d,double>>& pose){
    vector<pair<Eigen::Matrix4d,double>> result;
    result.push_back(pose.front());
    for(int i = 1; i < pose.size(); i++){
        auto last = pose[i-1];
        auto now = pose[i];
        auto relative = now.first * last.first.inverse();
        auto time = now.second;
        result.push_back(make_pair(relative, time));
    }
    return result;
}

vector<pair<Eigen::Matrix4d,double>> accRelative(vector<pair<Eigen::Matrix4d,double>>& pose){
    vector<pair<Eigen::Matrix4d,double>> result;
    result.push_back(pose.front());
    for(int i = 1; i < pose.size(); i++){
        auto relative = pose[i].first;
        auto next = relative * result.back().first;
        auto time = pose[i].second;
        result.push_back(make_pair(next, time));
    }
    return result;
}


void pubEigenPose(ros::Publisher& pub, Eigen::Matrix4d& pose){
    Eigen::Quaterniond tQ(pose.block<3,3>(0,0));
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = pose(0,3);
    odom.pose.pose.position.y = pose(1,3);
    odom.pose.pose.position.z = pose(2,3);
    odom.pose.pose.orientation.x = tQ.x();
    odom.pose.pose.orientation.y = tQ.y();
    odom.pose.pose.orientation.z = tQ.z();
    odom.pose.pose.orientation.w = tQ.w();
    pub.publish(odom);
}

vector<pair<Eigen::Matrix4d,double>> removeInitialBias(pair<Eigen::Matrix4d,double> start, vector<pair<Eigen::Matrix4d,double>>& pose){
//    auto relativeAfter = getRelative(pose);
//    relativeAfter.front() = start;
//    auto result = accRelative(relativeAfter);
//    return result;
    auto posiDiff = pose.front().first.block<3,1>(0,3) - start.first.block<3,1>(0,3);
    vector<pair<Eigen::Matrix4d,double>> result = pose;
    for(auto&p:result){
        p.first.block<3,1>(0,3) -= posiDiff;
    }
    return result;
}



/*vector<pair<Eigen::Matrix4d,double>>*/void removeAfterBias(vector<pair<Eigen::Matrix4d,double>>& pose, pair<Eigen::Matrix4d,double> end) {

    //
    Eigen::Vector3d posidiff = end.first.block<3,1>(0,3) - pose.back().first.block<3,1>(0,3);
    Eigen::Vector3d avgError = posidiff / pose.size() * 1.0;
    cout << "all error " << posidiff << endl;
    cout << "avg error " << avgError << endl;
    for(int i = 0; i < pose.size(); i++){
        Eigen::Vector3d thisError = avgError * i * 1.0;
        auto& p = pose[i];
        p.first.block<3,1>(0,3) += thisError;
    }
    return;

    //gtsam
    using namespace gtsam;
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    Values isamCurrentEstimate;
    gtsam::Vector Vector6(6);

    noiseModel::Diagonal::shared_ptr priorNoise;
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);

    noiseModel::Diagonal::shared_ptr odometryNoise;
    Vector6 << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

    gtsam::Vector Vector3(3);
    Vector3<<1e-6, 1e-6, 1e-6;
    noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
    double lastx,lasty,lastz,lastR,lastP,lastY;
    for(int i = 0; i < pose.size(); i++){
        auto&p=pose[i];
        double x,y,z,R,P,Y;
        x = p.first(0,3);
        y = p.first(1,3);
        z = p.first(2,3);
        Y = p.first.block<3,3>(0,0).eulerAngles(2,1,0)(0);
        P = p.first.block<3,3>(0,0).eulerAngles(2,1,0)(1);
        R = p.first.block<3,3>(0,0).eulerAngles(2,1,0)(2);
        if(i == 0){
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(Y, R, P),
                                                       Point3(z, x, y)), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(Y, R, P),
                                            Point3(z, x, y)));

            lastx = x;
            lasty = y;
            lastz = z;
            lastR = R;
            lastP = P;
            lastY = Y;
            isam->update(gtSAMgraph, initialEstimate);
            isam->update();
            gtSAMgraph.resize(0);
            initialEstimate.clear();
        }else{
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(lastY, lastR, lastP),
                                          Point3(lastz, lastx, lasty));
            gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(Y, R, P),
                                          Point3(z, x, y));
            gtSAMgraph.add(BetweenFactor<Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(i, Pose3(Rot3::RzRyRx(Y, R, P),Point3(z, x, y)));

            lastx = x;
            lasty = y;
            lastz = z;
            lastR = R;
            lastP = P;
            lastY = Y;
            isam->update(gtSAMgraph, initialEstimate);
            isam->update();
            gtSAMgraph.resize(0);
            initialEstimate.clear();
        }
    }
    {
        double x,y,z;
        x = end.first(0,3);
        y = end.first(1,3);
        z = end.first(2,3);
        gtsam::GPSFactor gps_factor(pose.size()-1, gtsam::Point3(z, x, y), gps_noise);
        gtSAMgraph.add(gps_factor);

        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        gtSAMgraph.resize(0);
        initialEstimate.clear();
    }
    //
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isamCurrentEstimate = isam->calculateEstimate();//得到正确的位姿估计
    cout << " Before "<< pose.size() << endl;
    cout << " After "<< isamCurrentEstimate.size() << endl;
    for(int i=0; i<isamCurrentEstimate.size()-1; ++i){
        Pose3 latestEstimate;
        latestEstimate = isamCurrentEstimate.at<Pose3>(i);
        double x = latestEstimate.translation().y();
        double y = latestEstimate.translation().z();
        double z = latestEstimate.translation().x();
        double roll  = latestEstimate.rotation().pitch();
        double pitch = latestEstimate.rotation().yaw();
        double yaw   = latestEstimate.rotation().roll();
        Eigen::Matrix3d tmR( Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        Eigen::Vector3d tmt(x,y,z);
        Eigen::Matrix4d tmT = Eigen::Matrix4d::Identity();
        tmT.block<3,3>(0,0) = tmR;
        tmT.block<3,1>(0,3) = tmt;
        pose[i+1].first = tmT;
    }
}

ros::Publisher pubGT;
ros::Publisher pubSUMA;
ros::Publisher pubAfter;
ros::Publisher pubS1;

void handle01();
void handle00();

int main(int argc, char** argv){
    ros::init(argc, argv, "avgerror");
    ros::NodeHandle nh("~");

    pubGT = nh.advertise<nav_msgs::Odometry>("/GT", 1);
    pubSUMA = nh.advertise<nav_msgs::Odometry>("/suma", 1);
    pubAfter = nh.advertise<nav_msgs::Odometry>("/after", 1);
    pubS1 = nh.advertise<nav_msgs::Odometry>("/s1", 1);

    handle00();
//    handle01();
    return 0;
}

void handle00(){
    auto s1 = readTumFile("/home/qh/add_ws/00/SaveS1");
    auto suma = readTumFile("/home/qh/add_ws/00/SaveS1SUAM");
    auto gt = readTumFile("/home/qh/add_ws/00/GPSTUM1");
    int s1I = 0;
    int afterI = 0;
    int sumaI = 0;
    int gtI = 0;
    pair<double,double> time1(1317617836, 1317617936);
    pair<double,double> time2(1317618036, 1317618135);
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedS1;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedS2;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedS3;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedSuma1;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedSuma2;
    for(auto & i : s1){
        double s1Time = i.second;
        if(s1Time < time1.first) BeforeAlignedS1.push_back(i);
        else if(s1Time > time1.second && s1Time < time2.first) BeforeAlignedS2.push_back(i);
        else if(s1Time > time2.second) BeforeAlignedS3.push_back(i);
    }
    for(auto&i:suma){
        double sumaTime = i.second;
        if(sumaTime < time1.second && sumaTime > time1.first) BeforeAlignedSuma1.push_back(i);
        if(sumaTime < time2.second && sumaTime > time2.first) BeforeAlignedSuma2.push_back(i);
    }
    cout << "s1     " << BeforeAlignedS1.size() << endl;
    cout << "s2     " << BeforeAlignedS2.size() << endl;
    cout << "s3     " << BeforeAlignedS3.size() << endl;
    cout << "suma1  " << BeforeAlignedSuma1.size() << endl;
    cout << "suma2  " << BeforeAlignedSuma2.size() << endl;
    //
    auto deSuma1 = removeInitialBias(BeforeAlignedS1.back(), BeforeAlignedSuma1);
    auto deSuma2 = removeInitialBias(BeforeAlignedS2.back(), BeforeAlignedSuma2);
    removeAfterBias(deSuma1, BeforeAlignedS2.front());
    removeAfterBias(deSuma2, BeforeAlignedS3.front());
    //

    vector<pair<Eigen::Matrix4d,double>> Afterligned;
    Afterligned.insert(Afterligned.end(), BeforeAlignedS1.begin(), BeforeAlignedS1.end());
    Afterligned.insert(Afterligned.end(), deSuma1.begin(), deSuma1.end());
    Afterligned.insert(Afterligned.end(), BeforeAlignedS2.begin(), BeforeAlignedS2.end());
    Afterligned.insert(Afterligned.end(), deSuma2.begin(), deSuma2.end());
    Afterligned.insert(Afterligned.end(), BeforeAlignedS3.begin(), BeforeAlignedS3.end());
    cout << "all    " << Afterligned.size() << endl;

    saveTum(Afterligned, "/home/qh/add_ws/00/s1SumaTUM");
    ros::Rate loop(30);
    while(ros::ok()){
        if(gtI<gt.size())
            pubEigenPose(pubGT, gt[gtI+=40].first);
        if(sumaI<suma.size())
            pubEigenPose(pubSUMA, suma[sumaI+=4].first);
        if(afterI<Afterligned.size())
            pubEigenPose(pubAfter, Afterligned[afterI+=4].first);
        if(s1I<s1.size())
            pubEigenPose(pubS1, s1[s1I+=2].first);
        loop.sleep();
    }
}

void handle01(){
    auto s1 = readTumFile("/home/qh/add_ws/01/SaveS1");
    auto suma = readTumFile("/home/qh/add_ws/01/SaveS1SUAM");
    auto gt = readTumFile("/home/qh/add_ws/01/GPSTUM1");
    int s1I = 0;
    int afterI = 0;
    int sumaI = 0;
    int gtI = 0;
    pair<double,double> time1(1317623680, 1317623730);
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedS1;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedS2;
    vector<pair<Eigen::Matrix4d,double>> BeforeAlignedSuma1;
//    for(auto & i : s1){
//        double s1Time = i.second;
//        if(s1Time < time1.first) BeforeAlignedS1.push_back(i);
//        else if(s1Time > time1.second) BeforeAlignedS2.push_back(i);
//    }
//    for(auto&i:suma){
//        double sumaTime = i.second;
//        if(sumaTime < time1.second && sumaTime > time1.first) BeforeAlignedSuma1.push_back(i);
//    }
    cout << "s1     " << BeforeAlignedS1.size() << endl;
    cout << "s2     " << BeforeAlignedS2.size() << endl;
    cout << "suma1  " << BeforeAlignedSuma1.size() << endl;
    //
//    auto deSuma1 = removeInitialBias(BeforeAlignedS1.back(), BeforeAlignedSuma1);
//    removeAfterBias(deSuma1, BeforeAlignedS2.front());
//    //
////
//    vector<pair<Eigen::Matrix4d,double>> Afterligned;
//    Afterligned.insert(Afterligned.end(), BeforeAlignedS1.begin(), BeforeAlignedS1.end());
//    Afterligned.insert(Afterligned.end(), deSuma1.begin(), deSuma1.end());
//    Afterligned.insert(Afterligned.end(), BeforeAlignedS2.begin(), BeforeAlignedS2.end());
//    cout << "all    " << Afterligned.size() << endl;

//    saveTum(Afterligned, "/home/qh/add_ws/01/s1SumaTUM");
    ros::Rate loop(20);
    while(ros::ok()){
        if(gtI<gt.size())
            pubEigenPose(pubGT, gt[gtI+=20].first);
        if(sumaI<suma.size())
            pubEigenPose(pubSUMA, suma[sumaI+=4].first);
//        if(afterI<Afterligned.size())
//            pubEigenPose(pubAfter, Afterligned[afterI+=4].first);
        if(s1I<s1.size())
            pubEigenPose(pubS1, s1[s1I+=2].first);
        loop.sleep();
    }
}


//杂乱工具处理不正确的坐标系
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "/home/qh/qh_ws/src/AutoCoordinateAlign.h"
#include "random"
#include "pcl/registration/icp.h"
#include "iostream"
#include "vector"
#include "string"
#include "fstream"
#include "sstream"
#include "iomanip"
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

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

using namespace std;

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

vector<Eigen::Matrix4d> readTrans(string filePath){
    vector<Eigen::Matrix4d> result;
    ifstream file;
    file.open(filePath);
    int count = 0;
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
        ss >> tmpT(0,0) >> tmpT(0,1) >> tmpT(0,2) >> tmpT(0,3);
        ss >> tmpT(1,0) >> tmpT(1,1) >> tmpT(1,2) >> tmpT(1,3);
        ss >> tmpT(2,0) >> tmpT(2,1) >> tmpT(2,2) >> tmpT(2,3);
        result.push_back(tmpT);
    }
    return result;
}

pcl::PointCloud<PointXYZIRPYT> readsumaFile(string filePath) {
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file;
    file.open(filePath);
    int count = 0;
    Eigen::Matrix4d Z_X_Y2XYZ = Eigen::Matrix4d::Identity();
    Z_X_Y2XYZ <<0,-1, 0, 0,
                0, 0,-1, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
        ss >> tmpT(0,0) >> tmpT(0,1) >> tmpT(0,2) >> tmpT(0,3);
        ss >> tmpT(1,0) >> tmpT(1,1) >> tmpT(1,2) >> tmpT(1,3);
        ss >> tmpT(2,0) >> tmpT(2,1) >> tmpT(2,2) >> tmpT(2,3);
        tmpT = Z_X_Y2XYZ.transpose() * tmpT * Z_X_Y2XYZ;
        PointXYZIRPYT tmpP;
        auto tmpt = tmpT.block<3,1>(0,3);
        auto tmprpy = tmpT.block<3,3>(0,0).eulerAngles(2,1,0);
        tmpP.intensity = count++;
        tmpP.x = tmpt(0);
        tmpP.y = tmpt(1);
        tmpP.z = tmpt(2);
        tmpP.yaw = tmprpy(0);
        tmpP.pitch = tmprpy(1);
        tmpP.roll = tmprpy(2);
        result.push_back(tmpP);
    }
    return result;
}

vector<Eigen::Matrix4d> getRelative(vector<Eigen::Matrix4d>&pose){
    vector<Eigen::Matrix4d> result;
    result.push_back(pose.front());
    for(int i = 0; i < pose.size()-1; ++i){
        auto& last=pose[i];
        auto& now=pose[i+1];
        auto tt = last*now.inverse();
        result.push_back(tt);
    }
    return result;
}
pcl::PointCloud<PointXYZIRPYT> TransToRPY(vector<Eigen::Matrix4d>& pose){
    pcl::PointCloud<PointXYZIRPYT> result;
    for(auto&p:pose){
        PointXYZIRPYT tmpP;
        tmpP.x = p(0,3);
        tmpP.y = p(1,3);
        tmpP.z = p(2,3);
        tmpP.yaw = p.block<3,3>(0,0).eulerAngles(2,1,0)(0);
        tmpP.pitch = p.block<3,3>(0,0).eulerAngles(2,1,0)(1);
        tmpP.roll = p.block<3,3>(0,0).eulerAngles(2,1,0)(2);
        result.push_back(tmpP);
    }
    return result;
}


vector<Eigen::Matrix4d> RPYTOTrans(pcl::PointCloud<PointXYZIRPYT>& pose){
    vector<Eigen::Matrix4d> result;
    for(auto&p:pose){
        Eigen::Quaterniond q(Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond TQ(q.w(), q.x(), q.y(), q.z());
        Eigen::Matrix3d TR(TQ);
        Eigen::Matrix4d TT = Eigen::Matrix4d::Identity();
        TT.block<3,3>(0,0) = TR;
        TT.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
        result.push_back(TT);
    }
    return result;
}

pcl::PointCloud<PointXYZIRPYT> readFile(string filePath){
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

pcl::PointCloud<PointXYZIRPYT> readFileWithFlag(string filePath){
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
        int flag;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw >> tmpP.intensity;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

double getTime(string& timeStamp){
    double thisTime = 0;
    while(timeStamp.front() == '0'){
        timeStamp.erase(timeStamp.begin());
    }
    if(timeStamp.size() != 16){
        cerr << timeStamp << " time stamp error " << endl;
        getchar();
        exit(0);
    }
    string secStr(timeStamp.begin(),timeStamp.end()-6);
    string msecStr(timeStamp.end()-6, timeStamp.end());
    thisTime = stod(secStr) + stod(msecStr) / 1000000.0;
    return thisTime;
}

pcl::PointCloud<PointXYZIRPYT> getOxfordGT(string fileName) {
    vector<nav_msgs::Odometry> GTOdom;
    pcl::PointCloud<PointXYZIRPYT> GTPoses;
    vector<Eigen::Isometry3d> transPoses;
    vector<string> posesTimes;
    Eigen::Vector3d it = Eigen::Vector3d::Zero();
    Eigen::Quaterniond iQ = Eigen::Quaterniond::Identity();
    Eigen::Isometry3d rader_to_lidar = Eigen::Isometry3d::Identity();
    ifstream lidarPosesFile(fileName);
    string head;
    getline(lidarPosesFile, head);
    Eigen::Isometry3d nowPose = Eigen::Isometry3d::Identity();
    nowPose.rotate(iQ);
    nowPose.pretranslate(it);
    while (lidarPosesFile.good()) {
        string poseStr;
        getline(lidarPosesFile, poseStr);
        string time, noneTime;
        string dX, dY, dZ, dR, dP, dYaw;
        istringstream iss(poseStr);
        getline(iss, time, ',');
        getline(iss, noneTime, ',');
        getline(iss, dX, ',');
        getline(iss, dY, ',');
        getline(iss, dZ, ',');
        getline(iss, dR, ',');
        getline(iss, dP, ',');
        getline(iss, dYaw, ',');
        if (lidarPosesFile.eof()) break;
        Eigen::Vector3d dt(stod(dX), stod(dY), stod(dZ));
        auto temQ = tf::createQuaternionFromRPY(stod(dR), stod(dP), stod(dYaw));
        Eigen::Quaterniond dQ(temQ.w(), temQ.x(), temQ.y(), temQ.z());
        Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
        dT.rotate(dQ);
        dT.pretranslate(dt);
        dT = rader_to_lidar.inverse() * dT * rader_to_lidar;
        nowPose = nowPose * dT;
        transPoses.push_back(nowPose);
        posesTimes.push_back(time);
        // Odom
        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = nowPose(0, 3);
        odom.pose.pose.position.y = nowPose(1, 3);
        odom.pose.pose.position.z = nowPose(2, 3);
        Eigen::Quaterniond odomQ(nowPose.matrix().block<3, 3>(0, 0));
        odom.pose.pose.orientation.x = odomQ.x();
        odom.pose.pose.orientation.y = odomQ.y();
        odom.pose.pose.orientation.z = odomQ.z();
        odom.pose.pose.orientation.w = odomQ.w();
        odom.header.frame_id = "camera_init";
        odom.header.stamp = ros::Time().fromSec(getTime(time));
        GTOdom.push_back(odom);
        // TXYZRPY
        PointXYZIRPYT pp;
        pp.time = getTime(time);
        pp.x = nowPose(0, 3);
        pp.y = nowPose(1, 3);
        pp.z = nowPose(2, 3);
        pp.yaw = nowPose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0)(0);
        pp.pitch = nowPose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0)(1);
        pp.roll = nowPose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0)(2);
        GTPoses.push_back(pp);
    }
    cout << "get gt " << transPoses.size() << endl;
    return GTPoses;
}


void saveXYZRPY(string filePath, pcl::PointCloud<PointXYZIRPYT>& pose){
    ofstream file;
    file.open(filePath);
    int count = 0;
    for(auto &p:pose){
        file << fixed
             << setprecision(8) << p.time << " "
             << setprecision(8) << p.x << " "
             << p.y << " "
             << p.z << " "
             << p.roll << " "
             << p.pitch << " "
             << p.yaw << endl;
    }
    file.close();
}

void saveXYZRPYWithFlag(string filePath, pcl::PointCloud<PointXYZIRPYT>& pose){
    ofstream file;
    file.open(filePath);
    int count = 0;
    for(auto &p:pose){
        file << fixed
             << setprecision(8) << p.time << " "
             << setprecision(8) << p.x << " "
             << p.y << " "
             << p.z << " "
             << p.roll << " "
             << p.pitch << " "
             << p.yaw << " "
             << (int)p.intensity << endl;
    }
    file.close();
}

void saveXYZRPY(string filePath, vector<double>&times, pcl::PointCloud<PointXYZIRPYT>& pose){
    for(int i = 0; i < pose.size(); ++i){
        pose[i].time = times[i];
    }
    saveXYZRPY(filePath, pose);
}


void saveXYZRPY(string filePath, vector<double>&times, vector<Eigen::Matrix4d>&trans){
    ofstream file;
    file.open(filePath);
    int count = 0;
    for(int i = 0; i < trans.size();++i){
        double time = times[i];
        auto&p=trans[i];
        double x,y,z,roll,pitch,yaw;
        x = p(0,3);
        y = p(1,3);
        z = p(2,3);
        yaw = p.block<3,3>(0,0).eulerAngles(2,1,0)(0);
        pitch = p.block<3,3>(0,0).eulerAngles(2,1,0)(1);
        roll = p.block<3,3>(0,0).eulerAngles(2,1,0)(2);
        file << fixed
             << setprecision(8) << time << " "
             << setprecision(8) << x << " "
             << y << " "
             << z << " "
             << roll << " "
             << pitch << " "
             << yaw << endl;
    }
}


void saveXYZRPY(string filePath, vector<pair<Eigen::Matrix4d,double>>&pose){
    vector<double> times;
    vector<Eigen::Matrix4d>trans;
    for(auto&p:pose){
        times.push_back(p.second);
        trans.push_back(p.first);
    }
    saveXYZRPY(filePath, times, trans);
}

void saveTrans(string filepath, vector<Eigen::Matrix4d>&trans){
    ofstream file(filepath);
    for(auto&p:trans){
        file << p(0,0) << " "
             << p(0,1) << " "
             << p(0,2) << " "
             << p(0,3) << " "
             << p(1,0) << " "
             << p(1,1) << " "
             << p(1,2) << " "
             << p(1,3) << " "
             << p(2,0) << " "
             << p(2,1) << " "
             << p(2,2) << " "
             << p(2,3) << endl;
    }
    file.close();
}



void saveTum(string filePath, vector<pair<Eigen::Matrix4d,double>>&pose){
    ofstream file(filePath);
    for(auto&p:pose){
        Eigen::Quaterniond tmpQ(p.first.block<3,3>(0,0));
        file << fixed << setprecision(8) << p.second << " "
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
void saveTum(string path, pcl::PointCloud<PointXYZIRPYT>& pose){
    ofstream file;
    file.open(path);
    for(auto&p:pose){
        Eigen::Quaterniond tq(Eigen::AngleAxisd(p.yaw,   Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(p.roll,  Eigen::Vector3d::UnitX()));
        file << fixed << setprecision(8) << p.time << " "
             << p.x << " " << p.y << " " << p.z << " "
             << tq.x() << " " << tq.y() << " " << tq.z() << " " << tq.w() << endl;
    }
    file.close();
}

void transRPY(pcl::PointCloud<PointXYZIRPYT>&pose){

    Eigen::Matrix4d gauss = Eigen::Matrix4d::Identity();
    gauss <<   0, 0,-1, 0,
               0,-1, 0, 0,
              -1, 0, 0, 0,
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

double convertTime(string thisTimeStamp){
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    double msec = 0;
    sscanf(thisTimeStamp.c_str(), "%d-%d-%d %d:%d:%lf",
           &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
           &tm.tm_hour, &tm.tm_min, &msec);
    tm.tm_sec = int(msec);
    msec -= int(msec);
    tm.tm_year -= 1900;
    tm.tm_mon--;
    return mktime(&tm)+msec;
}

void recoverTimeSumaGT(pcl::PointCloud<PointXYZIRPYT>& pose, string filePath){
    ifstream file(filePath);
    vector<double> times;
    int count = 0;
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        double thisTime = convertTime(info);
        if(count >= pose.points.size()) break;
        pose.points[count].time = thisTime;
        count ++;
    }
    cout << "Raw time " << pose.size() << endl;
    cout << "recover time " << count << endl;
}

void transFile01(){
    auto sumaNoTimeTrans = readTrans("/home/qh/add_ws/01/suma");
    auto suma = TransToRPY(sumaNoTimeTrans);
    recoverTimeSumaGT(suma, "/home/qh/kitti/01/timestampsRaw.txt");
    saveTum("/home/qh/add_ws/01/sumaTUM", suma);

    auto gtNoTime = readTrans("/home/qh/add_ws/01/gt");
    auto gt = TransToRPY(gtNoTime);
    recoverTimeSumaGT(gt, "/home/qh/kitti/01/timestampsRaw.txt");
    saveTum("/home/qh/add_ws/01/gtTUM", gt);

    auto s1 = readFile("/home/qh/add_ws/01/s1");
    transRPY(s1);
    saveTum("/home/qh/add_ws/01/s1TUM", s1);
}

void pubEigenPose(ros::Publisher& pub, Eigen::Matrix4d& pose){
    Eigen::Quaterniond tQ(pose.block<3,3>(0,0));
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = pose(0,3);
    odom.pose.pose.position.y = pose(1,3);
    odom.pose.pose.position.z = pose(2,3);
    odom.pose.pose.orientation.x = tQ.x();
    odom.pose.pose.orientation.y = tQ.y();
    odom.pose.pose.orientation.z = tQ.z();
    odom.pose.pose.orientation.w = tQ.w();
    pub.publish(odom);
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

vector<double> readTimeStampedRaw(string filename, int size, int start=0){
    ifstream file(filename);
    vector<double> times;
    int count = 0;
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        double thisTime = convertTime(info);
        if(count++ < start) continue;
        times.push_back(thisTime);
        if(times.size() >= size) break;
    }
    cout << "Raw time " << size << endl;
    cout << "recover time " << times.size() << endl;
    return times;
}

vector<PointXYZIRPYT> SAMHANDLE(vector<PointXYZIRPYT>&before,PointXYZIRPYT&final){
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
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);

    noiseModel::Diagonal::shared_ptr odometryNoise;
    Vector6 << 1e-3, 1e-3, 1e-3, 1e-8, 1e-8, 1e-6;
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

    gtsam::Vector Vector3(3);
    Vector3<<1e-6, 1e-6, 1e-6;
    noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
    double lastx,lasty,lastz,lastR,lastP,lastY;
    for(int i = 0; i < before.size(); i++){
        auto&p=before[i];
        double x,y,z,R,P,Y;
        x = p.x;
        y = p.y;
        z = p.z;
        Y = p.roll;
        P = p.pitch;
        R = p.yaw;
        if(i == 0){
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(R, P, Y),
                                                       Point3(x, y, z)), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(R, P, Y),
                                            Point3(x, y, z)));
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
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(lastR, lastP, lastY),
                                          Point3(lastx, lasty, lastz));
            gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(R, P, Y),
                                          Point3(x, y, z));
            gtSAMgraph.add(BetweenFactor<Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(i, Pose3(Rot3::RzRyRx(R, P, Y),Point3(x, y, z)));
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
        x = final.x;
        y = final.y;
        z = final.z;
        gtsam::GPSFactor gps_factor(before.size()-1, gtsam::Point3(x, y, z), gps_noise);
        gtSAMgraph.add(gps_factor);

        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
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
    cout << "SECTION ADJ Before "<< before.size() << endl;
    cout << "SECTION ADJ After  "<< isamCurrentEstimate.size() << endl;
    vector<PointXYZIRPYT> result;
    for(int i=0; i<isamCurrentEstimate.size(); ++i){
        Pose3 thisEstimate;
        thisEstimate = isamCurrentEstimate.at<Pose3>(i);
        float x = thisEstimate.translation().x();
        float y = thisEstimate.translation().y();
        float z = thisEstimate.translation().z();
        double roll  = thisEstimate.rotation().yaw();
        double pitch = thisEstimate.rotation().pitch();
        double yaw   = thisEstimate.rotation().roll();
        PointXYZIRPYT tmp;

        tmp.time = before[i].time;
        tmp.intensity = before[i].intensity;
        tmp.x = x;
        tmp.y = y;
        tmp.z = z;

        tmp.roll = roll;
        tmp.pitch = pitch;
        tmp.yaw = yaw;

        if(tmp.roll>M_PI)tmp.roll-=2*M_PI;
        if(tmp.pitch>M_PI)tmp.pitch-=2*M_PI;
        if(tmp.yaw>M_PI)tmp.yaw-=2*M_PI;
        if(tmp.roll<-M_PI)tmp.roll+=2*M_PI;
        if(tmp.pitch<-M_PI)tmp.pitch+=2*M_PI;
        if(tmp.yaw<-M_PI)tmp.yaw+=2*M_PI;

        result.push_back(tmp);
    }

    delete isam;
    isam = nullptr;
    return result;
}

void ADJERR(pcl::PointCloud<PointXYZIRPYT>&cloudKeyPoses6D){
    // SAVE SECTION
    vector<pair<vector<PointXYZIRPYT>, int>> loss_recover;
    int lastState = -1;
    for(int i = 0; i < cloudKeyPoses6D.points.size(); ++i){
        auto&p=cloudKeyPoses6D.points[i];
        int nowState = p.intensity;
        if(nowState != lastState){
            lastState = nowState;
            loss_recover.push_back(pair<vector<PointXYZIRPYT>,int>{vector<PointXYZIRPYT>(), nowState});
            loss_recover.back().first.push_back(p);
        }else{
            loss_recover.back().first.push_back(p);
        }
    }

    // ADJUST SECTION
    for(int i = 0; i < loss_recover.size(); ++i){
        if(loss_recover[i].second == 1) continue;
        auto&secNow =loss_recover[i].first;
        auto&secNext=loss_recover[i+1].first;
        auto&poseTo = secNext.front();
        auto&poseFrom = secNext.back();
        double xERR,yERR,zERR,rollERR,pitchERR,yawERR;

        auto ans = SAMHANDLE(secNow, poseTo);
        secNow = ans;
    }
    // DEBUG INFO AND SAVE
    cloudKeyPoses6D.clear();
    cout << " ALL " << cloudKeyPoses6D.points.size() << endl;
    cout << " ALL " << loss_recover.size() << " Sections" << endl;
    for(int i = 0; i < loss_recover.size(); ++i){
        cout << i+1 << " Section Size" << loss_recover[i].first.size() << " " << (loss_recover[i].second==1?"GNSS":"LOSS") << endl;
        for(auto&p:loss_recover[i].first){
            cloudKeyPoses6D.push_back(p);
        }
    }
}

vector<string> SEQNUMLIST{
        "00", "01", "02", "05", "08"
};

void recoverDaquanTime(pcl::PointCloud<PointXYZIRPYT>&poses,int start,int end){
    if(poses.size() != end-start+1){
        cout << " COUNT NOT MATCH PLEASE CHECK !" << endl;
        cout << start << " " << end << endl;
        return ;
    }
    vector<double> lidarTimes;
    string lidarTimesPath = "/home/qh/dlut/Daquan/timestamp";
    ifstream lidarTimesFile(lidarTimesPath);
    while(lidarTimesFile.good()){
        double timeStamp;
        lidarTimesFile >> timeStamp;
        if(lidarTimesFile.eof()) break;
        lidarTimes.push_back(timeStamp);
    }
    for(int i = start-1; i <= end-1; ++i){
        poses[i+1-start].time = lidarTimes[i];
    }
}

void recoverOxfordTime(pcl::PointCloud<PointXYZIRPYT>&poses,int start,int end){
    if(poses.size() != end-start+1){
        cout << " COUNT NOT MATCH PLEASE CHECK !" << endl;
        return ;
    }
    vector<string> lidarTimes;
    string lidarTimesPath = "/home/qh/oxford1/velodyne_left_timestamps";
    ifstream lidarTimesFile(lidarTimesPath);
    while(lidarTimesFile.good()){
        string timeStamp;
        int noneValue;
        lidarTimesFile >> timeStamp >> noneValue;
        if(lidarTimesFile.eof()) break;
        lidarTimes.push_back(timeStamp);
    }
    lidarTimesFile.close();
    cout << "get lidar Times " << lidarTimes.size() << endl;
    for(int i = start-1; i <= end-1; ++i){
        poses[i+1-start].time = getTime(lidarTimes[i]);
    }
}
void preHnadleaDquan(){

    Eigen::Matrix4d lsT = Eigen::Matrix4d::Identity();
    lsT << 0,-1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    auto suma_trans_1 = readTrans("/home/qh/add_ws/daquan/suma/1_174_320");
    for(auto&p:suma_trans_1) p = lsT * p * lsT.transpose();
    auto suma_1 = TransToRPY(suma_trans_1);
    recoverDaquanTime(suma_1, 174, 320);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma1.txt", suma_1);
    saveTum("/home/qh/add_ws/daquan/suma/suma1Tum.txt", suma_1);

    auto suma_trans_2 = readTrans("/home/qh/add_ws/daquan/suma/2_672_818");
    for(auto&p:suma_trans_2) p = lsT * p * lsT.transpose();
    auto suma_2 = TransToRPY(suma_trans_2);
    recoverDaquanTime(suma_2, 672, 818);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma2.txt", suma_2);
    saveTum("/home/qh/add_ws/daquan/suma/suma2Tum.txt", suma_2);

    auto suma_trans_3 = readTrans("/home/qh/add_ws/daquan/suma/3_1188_1784");
    for(auto&p:suma_trans_3) p = lsT * p * lsT.transpose();
    auto suma_3 = TransToRPY(suma_trans_3);
    recoverDaquanTime(suma_3, 1188, 1784);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma3.txt", suma_3);
    saveTum("/home/qh/add_ws/daquan/suma/suma3Tum.txt", suma_3);

    auto suma_trans_4 = readTrans("/home/qh/add_ws/daquan/suma/4_2116_2492");
    for(auto&p:suma_trans_4) p = lsT * p * lsT.transpose();
    auto suma_4 = TransToRPY(suma_trans_4);
    recoverDaquanTime(suma_4, 2116, 2492);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma4.txt", suma_4);
    saveTum("/home/qh/add_ws/daquan/suma/suma4Tum.txt", suma_4);

    auto suma_trans_5 = readTrans("/home/qh/add_ws/daquan/suma/5_3034_3772");
    for(auto&p:suma_trans_5) p = lsT * p * lsT.transpose();
    auto suma_5 = TransToRPY(suma_trans_5);
    recoverDaquanTime(suma_5, 3034, 3772);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma5.txt", suma_5);
    saveTum("/home/qh/add_ws/daquan/suma/suma5Tum.txt", suma_5);

    auto suma_trans_6 = readTrans("/home/qh/add_ws/daquan/suma/6_4550_5204");
    for(auto&p:suma_trans_6) p = lsT * p * lsT.transpose();
    auto suma_6 = TransToRPY(suma_trans_6);
    recoverDaquanTime(suma_6, 4550, 5204);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma6.txt", suma_6);
    saveTum("/home/qh/add_ws/daquan/suma/suma6Tum.txt", suma_6);

    auto suma_trans_7 = readTrans("/home/qh/add_ws/daquan/suma/7_5640_6286");
    for(auto&p:suma_trans_7) p = lsT * p * lsT.transpose();
    auto suma_7 = TransToRPY(suma_trans_7);
    recoverDaquanTime(suma_7, 5640, 6286);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma7.txt", suma_7);
    saveTum("/home/qh/add_ws/daquan/suma/suma7Tum.txt", suma_7);

    auto suma_trans_8 = readTrans("/home/qh/add_ws/daquan/suma/8_7474_8460");
    for(auto&p:suma_trans_8) p = lsT * p * lsT.transpose();
    auto suma_8 = TransToRPY(suma_trans_8);
    recoverDaquanTime(suma_8, 7474, 8460);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma8.txt", suma_8);
    saveTum("/home/qh/add_ws/daquan/suma/suma8Tum.txt", suma_8);

    auto suma_trans_9 = readTrans("/home/qh/add_ws/daquan/suma/9_9130_10098");
    for(auto&p:suma_trans_9) p = lsT * p * lsT.transpose();
    auto suma_9 = TransToRPY(suma_trans_9);
    recoverDaquanTime(suma_9, 9130, 10098);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma9.txt", suma_9);
    saveTum("/home/qh/add_ws/daquan/suma/suma9Tum.txt", suma_9);

    auto suma_trans_10 = readTrans("/home/qh/add_ws/daquan/suma/10_10228_11324");
    for(auto&p:suma_trans_10) p = lsT * p * lsT.transpose();
    auto suma_10 = TransToRPY(suma_trans_10);
    recoverDaquanTime(suma_10, 10228, 11324);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma10.txt", suma_10);
    saveTum("/home/qh/add_ws/daquan/suma/suma10Tum.txt", suma_10);

    auto suma_trans_11 = readTrans("/home/qh/add_ws/daquan/suma/11_11814_12240");
    for(auto&p:suma_trans_11) p = lsT * p * lsT.transpose();
    auto suma_11 = TransToRPY(suma_trans_11);
    recoverDaquanTime(suma_11, 11814, 12240);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma11.txt", suma_11);
    saveTum("/home/qh/add_ws/daquan/suma/suma11Tum.txt", suma_11);

    auto suma_trans_12 = readTrans("/home/qh/add_ws/daquan/suma/12_13480_14148");
    for(auto&p:suma_trans_12) p = lsT * p * lsT.transpose();
    auto suma_12 = TransToRPY(suma_trans_12);
    recoverDaquanTime(suma_12, 13480, 14148);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma12.txt", suma_12);
    saveTum("/home/qh/add_ws/daquan/suma/suma12Tum.txt", suma_12);

    auto suma_trans_13 = readTrans("/home/qh/add_ws/daquan/suma/13_14638_15314");
    for(auto&p:suma_trans_13) p = lsT * p * lsT.transpose();
    auto suma_13 = TransToRPY(suma_trans_13);
    recoverDaquanTime(suma_13, 14638, 15314);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma13.txt", suma_13);
    saveTum("/home/qh/add_ws/daquan/suma/suma13Tum.txt", suma_13);

    auto suma_trans_14 = readTrans("/home/qh/add_ws/daquan/suma/14_15684_16520");
    for(auto&p:suma_trans_14) p = lsT * p * lsT.transpose();
    auto suma_14 = TransToRPY(suma_trans_14);
    recoverDaquanTime(suma_14, 15684, 16520);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma14.txt", suma_14);
    saveTum("/home/qh/add_ws/daquan/suma/suma14Tum.txt", suma_14);

    auto suma_trans_15 = readTrans("/home/qh/add_ws/daquan/suma/15_17400_18296");
    for(auto&p:suma_trans_15) p = lsT * p * lsT.transpose();
    auto suma_15 = TransToRPY(suma_trans_15);
    recoverDaquanTime(suma_15, 17400, 18296);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma15.txt", suma_15);
    saveTum("/home/qh/add_ws/daquan/suma/suma15Tum.txt", suma_15);

    auto suma_trans_16 = readTrans("/home/qh/add_ws/daquan/suma/16_19334_21032");
    for(auto&p:suma_trans_16) p = lsT * p * lsT.transpose();
    auto suma_16 = TransToRPY(suma_trans_16);
    recoverDaquanTime(suma_16, 19334, 21032);
    saveXYZRPY("/home/qh/add_ws/daquan/suma/suma16.txt", suma_16);
    saveTum("/home/qh/add_ws/daquan/suma/suma16Tum.txt", suma_16);
}


pcl::PointCloud<PointXYZIRPYT> AlignSuma2Last(PointXYZIRPYT lastPose, pcl::PointCloud<PointXYZIRPYT> sumaNow){
    pcl::PointCloud<PointXYZIRPYT> result;
    Eigen::Matrix4d StartT = Eigen::Matrix4d::Identity();
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(lastPose.yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(lastPose.pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(lastPose.roll, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond TQ(q.w(), q.x(), q.y(), q.z());
        Eigen::Matrix3d TR(TQ);
        StartT.block<3,3>(0,0) = TR;
        StartT.block<3,1>(0,3) = Eigen::Vector3d(lastPose.x, lastPose.y, lastPose.z);
    }
    auto TT = RPYTOTrans(sumaNow);
    for(auto&p:TT){
        p = StartT * p;
    }
    result = TransToRPY(TT);
    for(int i = 0; i < sumaNow.size(); i++){
        result[i].time = sumaNow[i].time;
        result[i].intensity = -1;
    }
    return result;
}

//void handleOxfordAndSuma(){
//    vector<pcl::PointCloud<PointXYZIRPYT>> suma_ori(8, pcl::PointCloud<PointXYZIRPYT>());
//    suma_ori[0] = readFile("/home/qh/add_ws/10/suma/suma_1.txt");
//    suma_ori[1] = readFile("/home/qh/add_ws/10/suma/suma_2.txt");
//    suma_ori[2] = readFile("/home/qh/add_ws/10/suma/suma_3.txt");
//    suma_ori[3] = readFile("/home/qh/add_ws/10/suma/suma_4.txt");
//    suma_ori[4] = readFile("/home/qh/add_ws/10/suma/suma_5.txt");
//    suma_ori[5] = readFile("/home/qh/add_ws/10/suma/suma_6.txt");
//    suma_ori[6] = readFile("/home/qh/add_ws/10/suma/suma_7.txt");
//    suma_ori[7] = readFile("/home/qh/add_ws/10/suma/suma_8.txt");
//
//    auto RTKMAP_semantic = readFileWithFlag("/home/qh/add_ws/10/semantic_long_/RTKMAPafter.txt");
//
//    vector<pair<vector<PointXYZIRPYT>, int>> loss_recover_semantic;
//    vector<pair<vector<PointXYZIRPYT>, int>> loss_recover_normal;
//    //Semantic
//    {
//        int lastState = -1;
//        for(int i = 0; i < RTKMAP_semantic.points.size(); ++i){
//            auto&p=RTKMAP_semantic.points[i];
//            int nowState = p.intensity;
//            if(nowState != lastState){
//                lastState = nowState;
//                loss_recover_semantic.push_back(pair<vector<PointXYZIRPYT>,int>{vector<PointXYZIRPYT>(), nowState});
//                loss_recover_semantic.back().first.push_back(p);
//            }else{
//                loss_recover_semantic.back().first.push_back(p);
//            }
//        }
//        PointXYZIRPYT lastPose;
//        bool haveLast = false;
//        int lossGnssIndex = 0;
//        for(int i = 0; i < loss_recover_semantic.size()-1; ++i){
//            auto&p=loss_recover_semantic[i];
//            if(p.second == 1 && loss_recover_semantic[i+1].second==0){
//                lastPose = p.first.back();
//                haveLast = true;
//            }
//            if(p.second == 0 && haveLast && lossGnssIndex < suma_ori.size()){
//                haveLast = false;
//                p.first.clear();
//                auto handledSuma = AlignSuma2Last(lastPose, suma_ori[lossGnssIndex]);
//                lossGnssIndex++;
//                for(auto&a:handledSuma){
//                    p.first.push_back(a);
//                }
//            }
//        }
//        pcl::PointCloud<PointXYZIRPYT> rtk_suma_semantic_before;
//        pcl::PointCloud<PointXYZIRPYT> rtk_suma_semantic_after;
//        cout << "SEMANTIC BEFORE====================>" << endl;
//        for(int i = 0; i < loss_recover_semantic.size(); i++){
//            auto&p=loss_recover_semantic[i];
//            cout << "SECTION " << i << ": " << (p.second==0?"LOSS*************":"GNSS") << " SIZE: " << p.first.size() << endl;
//            for(auto&a:p.first){
//                rtk_suma_semantic_before.push_back(a);
//            }
//        }
//        // ADJUST SECTION
//        cout << "SEMANTIC ADJUST====================>" << endl;
//        for(int i = 0; i < loss_recover_semantic.size(); ++i){
//            if(loss_recover_semantic[i].second == 1) continue;
//            auto&secNow =loss_recover_semantic[i].first;
//            auto&secNext=loss_recover_semantic[i+1].first;
//            auto&poseTo = secNext.front();
//            auto&poseFrom = secNext.back();
//            auto ans = SAMHANDLE(secNow, poseTo);
//            secNow = ans;
//        }
//        cout << "SEMANTIC AFTER====================>" << endl;
//        for(int i = 0; i < loss_recover_semantic.size(); i++){
//            auto&p=loss_recover_semantic[i];
//            cout << "SECTION " << i << ": " << (p.second==0?"LOSS*************":"GNSS") << " SIZE: "<< p.first.size() << endl;
//            for(auto&a:p.first){
//                rtk_suma_semantic_after.push_back(a);
//            }
//        }
//        saveTum("/home/qh/add_ws/10/suma/suma_semanticTum", rtk_suma_semantic_before);
//        saveTum("/home/qh/add_ws/10/suma/suma_semantic_gtsamTum", rtk_suma_semantic_after);
//        saveXYZRPYWithFlag("/home/qh/add_ws/10/suma/suma_semantic.txt", rtk_suma_semantic_before);
//        saveXYZRPYWithFlag("/home/qh/add_ws/10/suma/suma_semantic_gtsam.txt", rtk_suma_semantic_after);
//    }
//}



void handleDaquanAndSuma(){
    vector<pcl::PointCloud<PointXYZIRPYT>> suma_ori(16, pcl::PointCloud<PointXYZIRPYT>());
    suma_ori[0] = readFile("/home/qh/add_ws/daquan/suma/suma1.txt");
    suma_ori[1] = readFile("/home/qh/add_ws/daquan/suma/suma2.txt");
    suma_ori[2] = readFile("/home/qh/add_ws/daquan/suma/suma3.txt");
    suma_ori[3] = readFile("/home/qh/add_ws/daquan/suma/suma4.txt");
    suma_ori[4] = readFile("/home/qh/add_ws/daquan/suma/suma5.txt");
    suma_ori[5] = readFile("/home/qh/add_ws/daquan/suma/suma6.txt");
    suma_ori[6] = readFile("/home/qh/add_ws/daquan/suma/suma7.txt");
    suma_ori[7] = readFile("/home/qh/add_ws/daquan/suma/suma8.txt");
    suma_ori[8] = readFile("/home/qh/add_ws/daquan/suma/suma9.txt");
    suma_ori[9] = readFile("/home/qh/add_ws/daquan/suma/suma10.txt");
    suma_ori[10] = readFile("/home/qh/add_ws/daquan/suma/suma11.txt");
    suma_ori[11] = readFile("/home/qh/add_ws/daquan/suma/suma12.txt");
    suma_ori[12] = readFile("/home/qh/add_ws/daquan/suma/suma13.txt");
    suma_ori[13] = readFile("/home/qh/add_ws/daquan/suma/suma14.txt");
    suma_ori[14] = readFile("/home/qh/add_ws/daquan/suma/suma15.txt");
    suma_ori[15] = readFile("/home/qh/add_ws/daquan/suma/suma16.txt");


    auto RTKMAP_semantic = readFileWithFlag("/home/qh/add_ws/daquan/RTKMAPsemantic_temp.txt");

    vector<pair<vector<PointXYZIRPYT>, int>> loss_recover_semantic;
    //Semantic
    {
        int lastState = -2;
        for(int i = 0; i < RTKMAP_semantic.points.size(); ++i){
            auto&p=RTKMAP_semantic.points[i];
            int nowState = p.intensity;
            if(nowState != lastState){
                lastState = nowState;
                loss_recover_semantic.push_back(pair<vector<PointXYZIRPYT>,int>{vector<PointXYZIRPYT>(), nowState});
                loss_recover_semantic.back().first.push_back(p);
            }else{
                loss_recover_semantic.back().first.push_back(p);
            }
        }
        PointXYZIRPYT lastPose;
        bool haveLast = false;
        int lossGnssIndex = 0;
        for(int i = 0; i < loss_recover_semantic.size()-1; ++i){
            auto&p=loss_recover_semantic[i];
            if(p.second == 0 && loss_recover_semantic[i+1].second==-1){
                lastPose = p.first.back();
                haveLast = true;
            }
            if(p.second == -1 && haveLast && lossGnssIndex < suma_ori.size()){
                haveLast = false;
                p.first.clear();
                auto handledSuma = AlignSuma2Last(lastPose, suma_ori[lossGnssIndex]);
                lossGnssIndex++;
                for(auto&a:handledSuma){
                    p.first.push_back(a);
                }
            }
        }
        pcl::PointCloud<PointXYZIRPYT> rtk_suma_semantic_before;
        pcl::PointCloud<PointXYZIRPYT> rtk_suma_semantic_after;
        cout << "SEMANTIC BEFORE====================>" << endl;
        for(int i = 0; i < loss_recover_semantic.size(); i++){
            auto&p=loss_recover_semantic[i];
            cout << "SECTION " << i << ": " << (p.second==-1?"LOSS*************":"GNSS") << " SIZE: " << p.first.size() << endl;
            for(auto&a:p.first){
                rtk_suma_semantic_before.push_back(a);
            }
        }
        // ADJUST SECTION
        cout << "SEMANTIC ADJUST====================>" << endl;
        for(int i = 0; i < loss_recover_semantic.size(); ++i){
            if(loss_recover_semantic[i].second == 0) continue;
            auto&secNow =loss_recover_semantic[i].first;
            auto&secNext=loss_recover_semantic[i+1].first;
            auto&poseTo = secNext.front();
            auto&poseFrom = secNext.back();
            auto ans = SAMHANDLE(secNow, poseTo);
            secNow = ans;
        }
        cout << "SEMANTIC AFTER====================>" << endl;
        for(int i = 0; i < loss_recover_semantic.size(); i++){
            auto&p=loss_recover_semantic[i];
            cout << "SECTION " << i << ": " << (p.second==-1?"LOSS*************":"GNSS") << " SIZE: "<< p.first.size() << endl;
            for(auto&a:p.first){
                rtk_suma_semantic_after.push_back(a);
            }
        }
        pcl::io::savePCDFileASCII("/home/qh/add_ws/daquan/test1.pcd", rtk_suma_semantic_before);
        pcl::io::savePCDFileASCII("/home/qh/add_ws/daquan/test2.pcd", rtk_suma_semantic_after);
        saveXYZRPYWithFlag("/home/qh/add_ws/daquan/suma_semantic_temp.txt", rtk_suma_semantic_before);
        saveXYZRPYWithFlag("/home/qh/add_ws/daquan/suma_semantic_gtsam_temp.txt", rtk_suma_semantic_after);
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "transTool");
    ros::NodeHandle nh;

    ros::Publisher pubGPS = nh.advertise<nav_msgs::Odometry>("/GPS", 1);
    ros::Publisher pubRTKMAP = nh.advertise<nav_msgs::Odometry>("/RTKMAP", 1);
    ros::Publisher pubGT = nh.advertise<nav_msgs::Odometry>("/GT", 1);

    ros::Publisher pubNormal = nh.advertise<nav_msgs::Odometry>("/normal", 1);
    ros::Publisher pubSemantic = nh.advertise<nav_msgs::Odometry>("/semantic", 1);


    handleDaquanAndSuma();

//    auto RTKMAP_semantic_ori = readFileWithFlag("/home/qh/add_ws/daquan/RTKMAPsemantic.txt");
//    auto RTKMAP_trans = RPYTOTrans(RTKMAP_semantic_ori);
    Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
    tmpT << -1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
//    for(auto&p:RTKMAP_trans){
//        p = tmpT * p * tmpT.transpose();
//    }
//    auto RTKMAP_semantic = TransToRPY(RTKMAP_trans);
//    for(int i = 0; i < RTKMAP_semantic.size(); ++i){
//        RTKMAP_semantic[i].time = RTKMAP_semantic_ori[i].time;
//        RTKMAP_semantic[i].intensity = RTKMAP_semantic_ori[i].intensity;
//    }
//    saveXYZRPYWithFlag("/home/qh/add_ws/daquan/RTKMAPsemantic_temp.txt", RTKMAP_semantic);
//    saveTum("/home/qh/add_ws/daquan/RTKMAPsemantic_tempTum.txt", RTKMAP_semantic);

    auto before_temp = readFileWithFlag("/home/qh/add_ws/daquan/suma_semantic_temp.txt");
    auto after_temp = readFileWithFlag("/home/qh/add_ws/daquan/suma_semantic_gtsam_temp.txt");

    auto before_trans = RPYTOTrans(before_temp);
    auto after_trans = RPYTOTrans(after_temp);
    for(auto&p:before_trans){
        p = tmpT.transpose() * p * tmpT;
    }
    for(auto&p:after_trans){
        p = tmpT.transpose() * p * tmpT;
    }

    auto before_res = TransToRPY(before_trans);
    auto after_res = TransToRPY(after_trans);
    for(int i = 0; i < before_res.size(); ++i){
        before_res[i].time = before_temp[i].time;
        before_res[i].intensity = before_temp[i].intensity;
    }
    for(int i = 0; i < after_res.size(); ++i){
        after_res[i].time = after_temp[i].time;
        after_res[i].intensity = after_temp[i].intensity;
    }

    saveXYZRPYWithFlag("/home/qh/add_ws/daquan/suma_semantic.txt", before_res);
    saveTum("/home/qh/add_ws/daquan/suma_semanticTum.txt", before_res);

    saveXYZRPYWithFlag("/home/qh/add_ws/daquan/suma_semantic_gtsam.txt", after_res);
    saveTum("/home/qh/add_ws/daquan/suma_semantic_gtsamTum.txt", after_res);


    return 0;
//


    int sumaI=0,gpsI=0;
    ros::Rate loop(20);
    while(ros::ok()){
//        if(gpsI<GPS.size())
//            pubEigenPose(pubGPS, GPS.points[gpsI+=1]);
//        if(rtkmapI<RTKMAP.size())
//            pubEigenPose(pubRTKMAP, RTKMAP.points[rtkmapI+=10]);
//        if(gtI<GT.size())
//            pubEigenPose(pubGT, GT[gtI+=10]);
//        if(gpsI<RTKMAP_now.points.size())
//            pubEigenPose(pubNormal, RTKMAP_now.points[gpsI+=5]);
//        if(sumaI<suma_1_ok.points.size())
//            pubEigenPose(pubSemantic, suma_1_ok.points[sumaI+=1]);
        loop.sleep();
    }
}

//杂乱工具处理不正确的坐标系
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "/home/qh/AutoCoordinateAlign.h"
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
    string head;
    getline(file, head);
    cout << head;
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
    string head;
    getline(file, head);
    cout << head;
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
             << setprecision(6) << p.time << " "
             << setprecision(6) << p.x << " "
             << p.y << " "
             << p.z << " "
             << p.roll << " "
             << p.pitch << " "
             << p.yaw << endl;
    }
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
             << setprecision(6) << time << " "
             << setprecision(6) << x << " "
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
void saveTum(string path, pcl::PointCloud<PointXYZIRPYT>& pose){
    ofstream file;
    file.open(path);
    for(auto&p:pose){
        Eigen::Quaterniond tq(Eigen::AngleAxisd(p.yaw,   Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(p.roll,  Eigen::Vector3d::UnitX()));
        file << fixed << setprecision(6) << p.time << " "
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
    cout << " Before "<< before.size() << endl;
    cout << " After "<< isamCurrentEstimate.size() << endl;
    vector<PointXYZIRPYT> result;
    for(int i=0; i<isamCurrentEstimate.size()-1; ++i){
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
        tmp.intensity = before[i].time;
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


int main(int argc, char** argv){
    ros::init(argc, argv, "transTool");
    ros::NodeHandle nh;

    ros::Publisher pubGPS = nh.advertise<nav_msgs::Odometry>("/GPS", 1);
    ros::Publisher pubRTKMAP = nh.advertise<nav_msgs::Odometry>("/RTKMAP", 1);
    ros::Publisher pubGT = nh.advertise<nav_msgs::Odometry>("/GT", 1);

    ros::Publisher pubNormal = nh.advertise<nav_msgs::Odometry>("/normal", 1);
    ros::Publisher pubSemantic = nh.advertise<nav_msgs::Odometry>("/semantic", 1);

    auto GNSS_sparse = readFile("/home/qh/add_ws/10/gt/GNSSsparse.txt");

    auto GT_XYZRPY = getOxfordGT("/home/qh/add_ws/10/gt/gt.csv");
    auto GT_trans  = RPYTOTrans(GT_XYZRPY);
    Eigen::Matrix4d gt2gps = Eigen::Matrix4d::Identity();
    gt2gps(1,1) = -1;
    gt2gps(2,2) = -1;
    for(auto&p:GT_trans){
        p = gt2gps * p * gt2gps.transpose();
    }
    auto GT = TransToRPY(GT_trans);
    for(int i = 0; i < GT.size(); i++){
        GT[i].time = GT_XYZRPY[i].time;
    }

    saveXYZRPY("/home/qh/add_ws/10/gt/gt.txt", GT);
    saveTum("/home/qh/add_ws/10/gt/gtTum.txt", GT);


    return 0;

    int gtI=0,rtkmapI=0,gpsI=0;
    ros::Rate loop(20);
    while(ros::ok()){
//        if(gpsI<GPS.size())
//            pubEigenPose(pubGPS, GPS.points[gpsI+=1]);
//        if(rtkmapI<RTKMAP.size())
//            pubEigenPose(pubRTKMAP, RTKMAP.points[rtkmapI+=10]);
//        if(gtI<GT.size())
//            pubEigenPose(pubGT, GT[gtI+=10]);
//        if(gpsI<GNSS_sparse.points.size())
//            pubEigenPose(pubNormal, GNSS_sparse.points[gpsI+=5]);
//        if(gtI<GT.points.size())
//            pubEigenPose(pubSemantic, GT.points[gtI+=5]);
        loop.sleep();
    }
}

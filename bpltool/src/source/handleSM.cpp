#define PCL_NO_PRECOMPILE

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "iostream"
#include "vector"
#include "string"
#include "fstream"
#include "pcl/registration/icp.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/io.h"

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

pcl::PointCloud<PointXYZIRPYT> TransToRPY(vector<pair<Eigen::Matrix4d,double>>& pose){
    pcl::PointCloud<PointXYZIRPYT> result;
    for(auto&it:pose){
        auto&p=it.first;
        PointXYZIRPYT tmpP;
        tmpP.time = it.second;
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

void transGPS(pcl::PointCloud<PointXYZIRPYT>&pose){
    for(auto&p:pose){
        float roll = p.roll;
        float pitch = p.pitch;
        float yaw = p.yaw;
        p.roll = yaw;
        p.pitch = pitch;
        p.yaw = -roll;
    }
}
vector<pair<Eigen::Matrix4d,double>> RPYTOTrans(pcl::PointCloud<PointXYZIRPYT>& pose){
    vector<pair<Eigen::Matrix4d,double>> result;
    for(auto&p:pose){
        Eigen::Quaterniond q(Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond TQ(q.w(), q.x(), q.y(), q.z());
        Eigen::Matrix3d TR(TQ);
        Eigen::Matrix4d TT = Eigen::Matrix4d::Identity();
        TT.block<3,3>(0,0) = TR;
        TT.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
        result.push_back(make_pair(TT,p.time));
    }
    return result;
}

pcl::PointCloud<PointXYZIRPYT> transPoseBySim3(pcl::PointCloud<PointXYZIRPYT>&in,Eigen::Matrix4d trans,double scale){
    trans.block<3,3>(0,0) = scale * trans.block<3,3>(0,0);
    vector<pair<Eigen::Matrix4d,double>> temp;
    auto s = RPYTOTrans(in);
    for(auto&p:s){
        p.first = trans * p.first;
        temp.push_back(p);
    }
    auto result = TransToRPY(temp);
    return result;
}

void saveXYZRPYAndPCD(pcl::PointCloud<PointXYZIRPYT>& pose,string path){
    ofstream file;
    file.open(path);
    for(auto&p:pose){
        file << fixed << setprecision(6) << p.time << " "
        << p.x << " "
        << p.y << " "
        << p.z << " "
        << p.roll << " "
        << p.pitch << " "
        << p.yaw << endl;
    }
    file.close();
    pcl::io::savePCDFileASCII(path+".pcd", pose);
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

void handleSM01(){
    auto s1 = readTumFile("/home/qh/add_ws/01/s1TUM");
    auto s1suma = readTumFile("/home/qh/add_ws/01/s1SumaTUM");
    auto GPS = readTumFile("/home/qh/add_ws/01/GPSTUM");
    auto s1_ = TransToRPY(s1);
    auto s1suma_ = TransToRPY(s1suma);
    auto GPS_ = TransToRPY(GPS);
    saveXYZRPYAndPCD(GPS_, "/home/qh/add_ws/01/SeqGPS");
    transGPS(GPS_);
    saveTum("/home/qh/add_ws/01/GPSTUM1", GPS_);
    {
        Eigen::Matrix4d TransS12GPS = Eigen::Matrix4d::Identity();
        TransS12GPS.block<3, 3>(0, 0) << 9.99967523e-01,8.38294032e-04,8.01558164e-03,
                -8.92196945e-04, 9.99976999e-01, 6.72355692e-03,
                -8.00976095e-03, -6.73049004e-03, 9.99945271e-01;
        TransS12GPS.block<3, 1>(0, 3) << -1.11137272, 0.16868055, 1.70381688;
        double scaleS12GPS = 1;
        scaleS12GPS = 1.0006454675307601;
        auto s1A = transPoseBySim3(s1_, TransS12GPS, scaleS12GPS);
        saveTum("/home/qh/add_ws/01/SaveS1", s1A);
        saveXYZRPYAndPCD(s1A, "/home/qh/add_ws/01/Seq01S1");
    }
    {
        Eigen::Matrix4d TransS1SUAM2GPS = Eigen::Matrix4d::Identity();
        TransS1SUAM2GPS.block<3, 3>(0, 0) << 0.9995949,0.02717158,0.00847061,
                                                            -0.02692655,0.99925051,-0.02780972,
                                                            -0.0092199,0.02757037,0.99957734;
        TransS1SUAM2GPS.block<3, 1>(0, 3) << -0.81216489,9.66546953,2.35144099;
        double scaleS1SUAM2GPS = 1.0000433939394828;
        auto s1sumaA = transPoseBySim3(s1suma_, TransS1SUAM2GPS, scaleS1SUAM2GPS);
        saveTum("/home/qh/add_ws/01/SaveS1SUAM", s1sumaA);
        saveXYZRPYAndPCD(s1sumaA, "/home/qh/add_ws/01/SeqS1SUAM");
    }
}


void handleSM00(){
    auto s1 = readTumFile("/home/qh/add_ws/00/s1TUM");
    auto s1suma = readTumFile("/home/qh/add_ws/00/s1SumaTUM");
    auto GPS = readTumFile("/home/qh/add_ws/00/GPSTUM");
    auto s1_ = TransToRPY(s1);
    auto s1suma_ = TransToRPY(s1suma);
    auto GPS_ = TransToRPY(GPS);
    saveXYZRPYAndPCD(GPS_, "/home/qh/add_ws/00/SeqGPS");
    transGPS(GPS_);
    saveTum("/home/qh/add_ws/00/GPSTUM1", GPS_);
    {
        Eigen::Matrix4d TransS12GPS = Eigen::Matrix4d::Identity();
        TransS12GPS.block<3, 3>(0, 0) << 8.51685906e-01, -2.53197319e-02,  5.23440568e-01,
                -5.23602632e-01,  2.68011528e-04,  8.51962565e-01,
                -2.17117518e-02, -9.99679368e-01, -1.30292166e-02;
        TransS12GPS.block<3, 1>(0, 3) << -0.71520503,  1.46723472, -0.10943889;
        double scaleS12GPS = 1;
        scaleS12GPS = 1.0186209393137995;
        auto s1A = transPoseBySim3(s1_, TransS12GPS, scaleS12GPS);
        saveTum("/home/qh/add_ws/00/SaveS1", s1A);
        saveXYZRPYAndPCD(s1A, "/home/qh/add_ws/00/Seq01S1");
    }
    {
        Eigen::Matrix4d TransS1SUAM2GPS = Eigen::Matrix4d::Identity();
        TransS1SUAM2GPS.block<3, 3>(0, 0) << 0.85163711, -0.02316394,  0.52361977,
                -0.52372726,  0.00163858,  0.85188442,
                -0.02059099, -0.99973034, -0.01073611;
        TransS1SUAM2GPS.block<3, 1>(0, 3) << -0.22594385,  0.52356885, -1.33316556;
        double scaleS1SUAM2GPS = 1.0177702872278596;
        auto s1sumaA = transPoseBySim3(s1suma_, TransS1SUAM2GPS, scaleS1SUAM2GPS);
        saveTum("/home/qh/add_ws/00/SaveS1SUAM", s1sumaA);
        saveXYZRPYAndPCD(s1sumaA, "/home/qh/add_ws/00/SeqS1SUAM");
    }
}


int main(){
    handleSM00();
}
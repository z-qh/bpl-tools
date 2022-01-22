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
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        Eigen::Matrix4d tmpT = Eigen::Matrix4d::Identity();
        ss >> tmpT(0,0) >> tmpT(0,1) >> tmpT(0,2) >> tmpT(0,3);
        ss >> tmpT(1,0) >> tmpT(1,1) >> tmpT(1,2) >> tmpT(1,3);
        ss >> tmpT(2,0) >> tmpT(2,1) >> tmpT(2,2) >> tmpT(2,3);
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
        ss >> tmpP.intensity >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

void save(string filePath, pcl::PointCloud<PointXYZIRPYT>& pose){
    ofstream file;
    file.open(filePath);
    file << "index time x y z roll pitch yaw"<< endl;
    int count = 0;
    for(auto &p:pose){
        file << fixed << count++ << " "
             << setprecision(6) << p.time << " "
             << setprecision(4) << p.x << " "
             << p.y << " "
             << p.z << " "
             << p.roll << " "
             << p.pitch << " "
             << p.yaw << endl;
    }
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

int main(int argc, char** argv){







}

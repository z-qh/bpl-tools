//将KITTI中的GPS打上时间戳转到东北天局部坐标系下
#include "iostream"
#include "fstream"
#include "vector"
#include "string"
#include "Eigen/Eigen"
#include "sstream"

#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "dirent.h"
#include "iomanip"

using namespace std;

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

void readTimes(vector<double>&times, string filePath){
    times.clear();
    ifstream file;
    file.open(filePath);
    while (file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        double time = convertTime(info);
        times.push_back(time);
    }
    file.close();
    cout << "get " << filePath << " " << times.size();
}

bool isFirstGPS = true;
double first_Xe = 0;
double first_Ye = 0;
double first_Ze = 0;
void BLH2ENU(double lat, double lon, double alt, double &x, double &y, double &z) {
    double sin_lat = sin(lat * M_PI / 180.0);
    double cos_lat = cos(lat * M_PI / 180.0);
    double sin_lon = sin(lon * M_PI / 180.0);
    double cos_lon = cos(lon * M_PI / 180.0);
    //经度(单位为°),纬度(单位为°),高度(单位m),东北天坐标系下坐标,ECEF->ENU(trans)
    lat = lat * M_PI / 180; // To rad.
    lon = lon * M_PI / 180;
    double f = 1 / 298.257223563; // WGS84
    double A_GNSS = 6378137.0;         // WGS84
    double B_GNSS = A_GNSS * (1 - f);
    double e = sqrt(A_GNSS * A_GNSS - B_GNSS * B_GNSS) / A_GNSS;
    double N = A_GNSS / sqrt(1 - e * e * sin(lat) * sin(lat));
    // To ECEF  地心站直角坐标系
    double Xe = (N + alt) * cos(lat) * cos(lon); //地心系下坐标(单位m)
    double Ye = (N + alt) * cos(lat) * sin(lon);
    double Ze = (N * (1 - (e * e)) + alt) * sin(lat);
    if(isFirstGPS){
        first_Xe = Xe;
        first_Ye = Ye;
        first_Ze = Ze;
        isFirstGPS = false;
        std::cout<<"xe "<<Xe<<" ye "<<Ye<<" ze "<<Ze<<std::endl;
        std::cout<<lat<<" lon "<<lon<<" alt "<<alt<<std::endl;
    }
    Xe -= first_Xe;
    Ye -= first_Ye;
    Ze -= first_Ze;
    // To ENU
    x = -Xe * sin_lon + Ye * cos_lon; //东北天坐标系下坐标
    y = -Xe * sin_lat * cos_lon - Ye * sin_lat * sin_lon + Ze * cos_lat;
    z = Xe * cos_lat * cos_lon + Ye * cos_lat * sin_lon + Ze * sin_lat;
}


Eigen::Matrix4d readIMU(string thisIMUFilePath){
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    ifstream thisIMUDataFile(thisIMUFilePath);
    double lat,lon,alt;
    thisIMUDataFile >> lat >> lon >> alt;
    double R,P,Y;
    thisIMUDataFile >> R >> P >> Y;
    double GPSX,GPSY,GPSZ;
    BLH2ENU(lat, lon, alt, GPSX, GPSY, GPSZ);
    //IMU Not Trans
    Eigen::Matrix3d tmpR(Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d tmpt(GPSX, GPSY, GPSZ);
    thisIMUDataFile.close();
    result.block<3,3>(0,0) = tmpR;
    result.block<3,1>(0,3) = tmpt;
    return result;
}


void readGPS(vector<pair<Eigen::Matrix4d,double>>&GPS, vector<double>&times, string IMUDataPath, double t1, double t2){
    GPS.clear();
    string IMUTempPath;
    double lastImuTime = 0;
    int ind = 0;
    while(true){
        stringstream ss;
        ss << fixed << setw(10) << setfill('0') << ind;
        IMUTempPath = IMUDataPath + "/" + ss.str() + ".txt";
        if( 0 == access(IMUTempPath.c_str(), 0) ){
            auto navData = readIMU(IMUTempPath);
            double thisImuTime = times[ind];
            if( thisImuTime < 10000 || thisImuTime <= lastImuTime ){
                cout << "get bad data abandon " << ind << fixed << lastImuTime << " " << thisImuTime << endl;
                ind++;
                continue;
            }
            lastImuTime = thisImuTime;
            if(thisImuTime < t1 || thisImuTime > t2+1.0) continue;
            GPS.push_back(make_pair(navData, thisImuTime));
        }else{
            break;
        }
        ind++;
    }
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

void transGPS(vector<pair<Eigen::Matrix4d,double>>& pose){

    Eigen::Matrix3d TransR = Eigen::Matrix3d::Identity();
    TransR << -1, 0, 0,
               0, 0,-1,
               0,-1, 0;
    for(auto&p:pose){
        p.first.block<3,3>(0,0) = TransR * p.first.block<3,3>(0,0) * TransR.transpose();
        p.first.block<3,1>(0,3) = TransR * p.first.block<3,1>(0,3);
    }
}

void readGPS01(){
    // seq01 0-1100 total 1101
    // 2011-10-03 14:34:18  1317623658
    // 2011-10-03 14:36:13  1317623773
    double timeStart = 1317623658;
    double timeEnd = 1317623773;
    string TimesPath = "/home/qh/kitti/01/oxts/timestamps.txt";
    string IMUDataPath = "/home/qh/kitti/01/oxts/data";
    vector<double> GPStimes;
    vector<pair<Eigen::Matrix4d,double>> gps;
    readTimes(GPStimes, TimesPath);
    readGPS(gps, GPStimes, IMUDataPath, timeStart, timeEnd);
    transGPS(gps);

    saveTum(gps, "/home/qh/add_ws/01/GPSTUM");
}




int main(){
//    readGPS01();
}

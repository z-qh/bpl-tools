#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "dirent.h"

#include <memory.h>
#include <iostream>
#include <ctime>

#include "iostream"
#include "sstream"
#include "Eigen/Eigen"

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "fstream"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "gnss_driver/gps_navi_msg.h"

#include "pcl/io/pcd_io.h"

struct VelodynePointXYZILR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZILR,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                           (uint32_t, label, label) (uint16_t, ring, ring) (float, time, time)
)
using namespace std;




class kitti{
public:
    /***************经纬高转enu坐标******************/
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
        double f        = 1 / 298.257223563; // WGS84
        double A_GNSS   = 6378137.0;         // WGS84
        double B_GNSS   = A_GNSS * (1 - f);
        double e        = sqrt(A_GNSS * A_GNSS - B_GNSS * B_GNSS) / A_GNSS;
        double N        = A_GNSS / sqrt(1 - e * e * sin(lat) * sin(lat));
        // To ECEF  地心站直角坐标系
        double Xe = (N + alt) * cos(lat) * cos(lon); //地心系下坐标(单位m)
        double Ye = (N + alt) * cos(lat) * sin(lon);
        double Ze = (N * (1 - (e * e)) + alt) * sin(lat);
        if(isFirstGPS){
            first_Xe    = Xe;
            first_Ye    = Ye;
            first_Ze    = Ze;
            isFirstGPS  = false;
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
    pair<sensor_msgs::Imu,gnss_driver::gps_navi_msg> readIMU(string thisIMUFilePath){
        sensor_msgs::Imu thisIMUData;
        gnss_driver::gps_navi_msg thisGNSSData;
        double noneValue;
        ifstream thisIMUDataFile(thisIMUFilePath);
        double lat,lon,alt;
        thisIMUDataFile >> lat >> lon >> alt;
        double R,P,Y;
        thisIMUDataFile >> R >> P >> Y;
        Eigen::Quaterniond Ori( Eigen::AngleAxisd(Y,Eigen::Vector3d::UnitZ())*
                                Eigen::AngleAxisd(P,Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(R,Eigen::Vector3d::UnitX()));
        Ori.normalize();
        double GPSX,GPSY,GPSZ;
        BLH2ENU(lat, lon, alt, GPSX, GPSY, GPSZ);
        //IMU Not Trans
        thisIMUData.orientation.x = Ori.x();
        thisIMUData.orientation.y = Ori.y();
        thisIMUData.orientation.z = Ori.z();
        thisIMUData.orientation.w = Ori.w();
        for(int i = 0; i < 5; i++) thisIMUDataFile >> noneValue;
        thisIMUDataFile >> thisIMUData.linear_acceleration.x
                        >> thisIMUData.linear_acceleration.y
                        >> thisIMUData.linear_acceleration.z;
        for(int i = 0; i < 3; i++) thisIMUDataFile >> noneValue;
        thisIMUDataFile >> thisIMUData.angular_velocity.x
                        >> thisIMUData.angular_velocity.y
                        >> thisIMUData.angular_velocity.z;
        thisIMUDataFile.close();
        thisGNSSData.latitude       = lat;
        thisGNSSData.longitude      = lon;
        thisGNSSData.elevation      = alt;
        thisGNSSData.PosStatus      = "KITTI";
        thisGNSSData.InsWorkStatus  = "KITTI";
        thisGNSSData.rollAngle      = R;
        thisGNSSData.pitchAngle     = P;
        thisGNSSData.yawAngle       = Y;

        return make_pair(thisIMUData, thisGNSSData);
    }
    static double convertTime(string thisTimeStamp){
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
    static double getLocalTime(double time){
        int sec = int(time);
        double msec = time - sec;
        sec = sec % 100000;
        return sec+msec;
    }

private:
    string calbiPath;
    Eigen::Matrix3d imu_laser_R;
    Eigen::Vector3d imu_laser_t;
    Eigen::Matrix4d imu_laser_T = Eigen::Matrix4d::Identity();
    void getAIMU2LaserMatrix(){
        ifstream calbiFile(calbiPath);
        string head;
        while(calbiFile.good()){
            string infoStr;
            getline(calbiFile, infoStr);
            if(infoStr.empty()) continue;
            istringstream iss(infoStr);
            iss >> head;
            iss >> imu_laser_R(0,0) >> imu_laser_R(0,1) >> imu_laser_R(0,2) >> imu_laser_t(0);
            iss >> imu_laser_R(1,0) >> imu_laser_R(1,1) >> imu_laser_R(1,2) >> imu_laser_t(1);
            iss >> imu_laser_R(2,0) >> imu_laser_R(2,1) >> imu_laser_R(2,2) >> imu_laser_t(2);
        }
        imu_laser_T.block<3,3>(0,0) = imu_laser_R;
        imu_laser_T.block<3,1>(0,3) = imu_laser_t;
        cout << head << imu_laser_R << endl;
        cout << head << imu_laser_t << endl;
    }

    string velodynePath;
    string labelPath;
    int odomFramesSize = 0; //Odom 点云、标签、图像的数量
    void getOdomFrames(){
        string label;
        int ind = 0;
        while(true){
            stringstream ss;
            ss << fixed << setw(6) << setfill('0') << ind;
            label = labelPath + "/" + ss.str() + ".label";
            if( 0 != access(label.c_str(), 0) ){
                odomFramesSize = ind;
                break;
            }
            ind++;
        }
        for(int i = 0; i < odomFramesSize; i++){
            stringstream ss;
            ss << fixed << setw(6) << setfill('0') << i;
            string velo = velodynePath + "/" + ss.str() + ".bin";
            if( 0 != access(velo.c_str(), 0) )
            {
                cerr << "data wrong" << endl;
                odomFramesSize = 0;
                return ;
            }
        }
        if(odomFramesSize == 0) return;
        cout << "odom " << odomFramesSize << endl;
    }

    string rawTimesPath;
    vector<double> rawLaserTimes;
    void getRawFrames(){
        ifstream rawTimesFile(rawTimesPath);
        while(rawTimesFile.good()){
            string timeStr;
            getline(rawTimesFile, timeStr);
            if(timeStr.empty()) continue;
            double time = convertTime(timeStr);
            rawLaserTimes.push_back(time);
        }
        rawTimesFile.close();
        cout << "raw " << rawLaserTimes.size() << endl;
    }

    string GTPath;
    vector<nav_msgs::Odometry> gtData;
    void getGTData(){
        ifstream GTfile(GTPath);
        Eigen::Matrix4d camera_to_laser = Eigen::Matrix4d::Identity();
        camera_to_laser = imu_laser_T;
        // TO DO
        while(GTfile.good()){
            string infoStr;
            getline(GTfile, infoStr);
            if(infoStr.empty()) continue;
            istringstream iss(infoStr);
            Eigen::Matrix4d tempT = Eigen::Matrix4d::Identity();
            iss >> tempT(0,0) >> tempT(0,1) >> tempT(0,2) >> tempT(0, 3);
            iss >> tempT(1,0) >> tempT(1,1) >> tempT(1,2) >> tempT(1, 3);
            iss >> tempT(2,0) >> tempT(2,1) >> tempT(2,2) >> tempT(2, 3);
            auto tempR2 = camera_to_laser.transpose() * tempT * camera_to_laser;
            Eigen::Matrix3d tempR = tempT.block<3,3>(0,0);
            Eigen::Quaterniond tempQ(tempR);
            nav_msgs::Odometry tempOdom;
            tempOdom.pose.pose.position.x = tempT(0, 3);
            tempOdom.pose.pose.position.y = tempT(1, 3);
            tempOdom.pose.pose.position.z = tempT(2, 3);
            tempQ.normalize();
            tempOdom.pose.pose.orientation.x = tempQ.x();
            tempOdom.pose.pose.orientation.y = tempQ.y();
            tempOdom.pose.pose.orientation.z = tempQ.z();
            tempOdom.pose.pose.orientation.w = tempQ.w();
            tempOdom.header.frame_id = "laser";
            gtData.push_back(tempOdom);
        }
        GTfile.close();
        cout << "gt " << gtData.size() << endl;
    }

    string IMUTimesPath;
    string IMUDataPath;
    vector<double> imuTimes;
    vector<pair<sensor_msgs::Imu,gnss_driver::gps_navi_msg>> imuData;
    void getIMUTimeAndData(){
        ifstream imuTimeFile(IMUTimesPath);
        while(imuTimeFile.good()){
            string timeStr;
            getline(imuTimeFile, timeStr);
            if(timeStr.empty()) continue;
            double time = convertTime(timeStr);
            imuTimes.push_back(time);
        }
        imuTimeFile.close();

        string IMUTempPath;
        int ind = 0;
        double lastImuTime = 0;
        while(true){
            stringstream ss;
            ss << fixed << setw(10) << setfill('0') << ind;
            IMUTempPath = IMUDataPath + "/" + ss.str() + ".txt";
            if( 0 == access(IMUTempPath.c_str(), 0) ){
                pair<sensor_msgs::Imu, gnss_driver::gps_navi_msg> navData;
                navData = readIMU(IMUTempPath);
                double thisImuTime = imuTimes[ind];
                if( thisImuTime < 10000 || thisImuTime <= lastImuTime ){
                    cout << "get bad data abandon " << ind << fixed << lastImuTime << " " << thisImuTime << endl;
                    ind++;
                    continue;
                }
                lastImuTime = thisImuTime;
                navData.first.header.stamp      = ros::Time().fromSec(thisImuTime);
                navData.second.header.stamp     = ros::Time().fromSec(thisImuTime);
                navData.first.header.frame_id   = "laser";
                navData.second.header.frame_id  = "laser";
                imuData.push_back(navData);
            }else{
                break;
            }
            ind++;
        }
        cout << "imu-tims " << imuTimes.size() << endl;
        cout << "imu-data " << imuData.size() << endl;
    }

    void save(){
        if(!bag.isOpen()){
            cerr << "bag not open please check!" << endl;
            return;
        }
        cout << " IMU " << endl;
        //IMU && GPS
        double lastImuTime = imuData[startImu].first.header.stamp.toSec() - 0.1;
        for(int i = startImu; i <= endImu; i++){
            double thisImuTime = imuData[i].first.header.stamp.toSec();
            if(thisImuTime < 10000 || thisImuTime <= lastImuTime){
                cout << " imu get bad time " << i << " " << lastImuTime << " " << thisImuTime << endl;
            }
            lastImuTime = thisImuTime;
            bag.write("/imu/data", imuData[i].first.header.stamp, imuData[i].first);
            bag.write("/gps", imuData[i].second.header.stamp, imuData[i].second);
        }
        // cout << " GT " << endl;
        //GT
        // for(int i = startOdom; i <= endOdom; i++){
        //     gtData[i].header.stamp = ros::Time().fromSec(rawLaserTimes[i+odom0InRaw]);
        //     bag.write("/gt", gtData[i].header.stamp, gtData[i]);
        // }

        cout << " Lidar " << endl;
        //label and lidar
        for(int i = startOdom; i <= endOdom ; i++){
            stringstream ss;
            ss << fixed << setw(6) << setfill('0') << i;
            string bin = velodynePath + "/" + ss.str() + ".bin";
            string label = labelPath + "/" + ss.str() + ".label";
            if( 0 == access(bin.c_str(), 0) && 0 == access(label.c_str(), 0) ){
                ifstream binfile;
                binfile.open(bin, ios::in | ios::binary);
                binfile.seekg(0, ios::beg);
                ifstream labelfile;
                labelfile.open(label, ios::in | ios::binary);
                labelfile.seekg(0, ios::beg);
                pcl::PointCloud<VelodynePointXYZILR>::Ptr points (new pcl::PointCloud<VelodynePointXYZILR>);
                for (int j = 0; binfile.good() && !binfile.eof() && labelfile.good() && !labelfile.eof(); j++) {
                    VelodynePointXYZILR point;
                    binfile.read((char *) &point.x, 3*sizeof(float));
                    binfile.read((char *) &point.intensity, sizeof(float));
                    double verticalAngle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180.0 / M_PI;
                    int rowIdn = round( (verticalAngle + 24.8) / 0.425 );
                    if(rowIdn < 0 || rowIdn >= 64) rowIdn = 64;
                    point.ring = rowIdn;
                    point.time = 0;
                    uint32_t labelValue = 0;
                    labelfile.read((char *) &labelValue, sizeof(uint32_t));
                    point.label = labelValue;
                    // float tmpX = point.x;
                    // point.x = point.y;
                    // point.y = -tmpX;
                    // point.z = point.z;
                    points->push_back(point);
                    if ((binfile.eof() && !labelfile.eof()) || (!binfile.eof() && labelfile.eof()))
                    {
                        cout << "wrong file " << endl;
                        getchar();
                    }

                }
                binfile.close();
                labelfile.close();

                sensor_msgs::PointCloud2 laserScan;
                pcl::toROSMsg(*points, laserScan);
                laserScan.header.frame_id = "laser";
                laserScan.header.stamp = ros::Time().fromSec(rawLaserTimes[i+odom0InRaw]);
                bag.write("/laser", laserScan.header.stamp, laserScan);

                cout << "get bin " << bin << endl;
                cout << "get label " << label << endl;
                cout << "size " << points->size() << " time " << rawLaserTimes[i+odom0InRaw] << endl;
                points->clear();
            }
            else{
                cout << "no bin or label " << bin  << " " << label << endl;
            }
        }
    }

    int odom0InRaw  = -1;
    int startOdom   = -1;
    int endOdom     = -1;
    int startImu    = -1;
    int endImu      = -1;
    void alignedTime(){
        bool exitFlag = false;
        if( odomFramesSize==0 ){
            cerr << "data odom wrong " << endl;
            exitFlag = true;
        }
        int startRaw        =-1;
        int endRaw          =-1;
        double startTime    = 0;
        double endTime      = 0;

        //we got the odom data len
        //we got the raw  data len
        //we must konw the first odom frame conrrest which raw data
        //we must konw the start odom frame and the end odom frame
        if(startOdom==-1)   startOdom   =0;
        if(endOdom==-1)     endOdom=odomFramesSize;
        if(odom0InRaw==-1)  odom0InRaw  =0;
        startRaw    = odom0InRaw + startOdom;
        endRaw      = odom0InRaw + endOdom;
        startTime   = rawLaserTimes[startRaw];
        endTime     = rawLaserTimes[endRaw];

        // save imu use imuData:startImu-endImu
        for(int i = 0; i < imuData.size(); i++){
            double t = imuData[i].first.header.stamp.toSec();
            if(t > startTime){
                startImu = i;
                break;
            }
        }
        for(int i = imuData.size()-1; i >= 0; i--){
            double t = imuData[i].first.header.stamp.toSec();
            if(t < endTime){
                endImu = i;
                break;
            }
        }

        // echo info
        cout << "title" << "\t\t" << "start" << "\t\t" << "end" << endl;
        cout << endl;
        cout << fixed << "raw" << "\t\t" << 0 << "\t\t" << rawLaserTimes.size()-1 << endl;
        cout << setprecision(3) << fixed
        << "raw" << "\t\t" << getLocalTime(rawLaserTimes.front()) << "\t" << getLocalTime(rawLaserTimes.back()) << endl;
        cout << endl;
        cout << fixed << "odom" << "\t\t" << startRaw << "\t\t" << endRaw << endl;
        cout << setprecision(3) << fixed
        << "odom" << "\t\t" << getLocalTime(rawLaserTimes[startRaw]) << "\t" << getLocalTime(rawLaserTimes[endRaw]) << endl;
        cout << endl;
        cout << fixed << "imu" << "\t\t" << startImu << "\t\t" << endImu << endl;
        cout << setprecision(3) << fixed
        << "imu" << "\t\t" << getLocalTime(imuData[startImu].first.header.stamp.toSec()) << "\t" << getLocalTime(imuData[endImu].first.header.stamp.toSec()) << endl;
    }
private:
    string pathBase;
    string saveName;
    rosbag::Bag bag;
public:
    kitti() = delete;
    kitti(string pathBase_, string saveName_,int odom0InRaw_=-1, int startOdom_=-1, int endOdom_=-1){
        //////////////////////////////////////////////
        pathBase    = pathBase_;
        saveName    = saveName_;
        odom0InRaw  = odom0InRaw_;
        startOdom   = startOdom_;
        endOdom     = endOdom_;
        ////////////////////////////////////////////
        velodynePath    = pathBase + "/velodyne";
        labelPath       = pathBase + "/labels";
        IMUDataPath     = pathBase + "/oxts/data";
        GTPath          = pathBase + "/poses.txt";
        rawTimesPath    = pathBase + "/timestampsRaw.txt";
        IMUTimesPath    = pathBase + "/oxts/timestamps.txt";
        calbiPath       = pathBase + "/calib.txt";
        ////////////////////////////////////////////
        if(0 == access(saveName.c_str(), 0)){
            cerr << "file already exist: " << saveName << endl;
            return;
        }
        bag.open(saveName, rosbag::bagmode::Write);
        /**************************/
        getAIMU2LaserMatrix();
        cout << "-----------------check----------------------" << endl;
        getOdomFrames();        //check the label and bin size no timestamp
        getIMUTimeAndData();    //check the imu data and imu timestamp
        getGTData();            //check the gt data no timestamp
        getRawFrames();         //check the raw data and laser timestamp
        /**************************/
        cout << "-----------------align----------------------" << endl;
        alignedTime();          //aligned the time between raw and odom relay on index
        /**************************/
        cout << "-----------------save-----------------------" << endl;
        save();
        /**************************/
        bag.close();
    }
};


int main(int argc, char** argv){
    // kitti seq00WithGNSS("/home/qh/kitti/00", "/home/qh/kitti/seq00.bag", 0, 0, 4540);
    kitti seq01WithGNSS("/home/qh/kitti/01", "/home/qh/kitti/seq01.bag", 0, 0, 1100);
    kitti seq02WithGNSS("/home/qh/kitti/02", "/home/qh/kitti/seq02.bag", 0, 0, 4660);
    kitti seq05WithGNSS("/home/qh/kitti/05", "/home/qh/kitti/seq05.bag", 0, 0, 2760);
    kitti seq06WithGNSS("/home/qh/kitti/06", "/home/qh/kitti/seq06.bag", 0, 0, 1100);
    kitti seq07WithGNSS("/home/qh/kitti/07", "/home/qh/kitti/seq07.bag", 0, 0, 1100);
    // 0 - 500 GT的高度有点问题还是抛弃吧
//    kitti seq08WithGNSS("/home/qh/kitti/08", "/home/qh/kitti/seq08.bag", 1100, 500, 4070);
    return 0;
}

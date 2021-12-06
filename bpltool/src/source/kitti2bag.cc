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
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "fstream"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

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
                                           (uint16_t, label, label) (uint16_t, ring, ring) (float, time, time)
)
using namespace std;

class kitti{
public:
    static sensor_msgs::Imu readIMU(string thisIMUFilePath){
        sensor_msgs::Imu thisIMUData;
        double noneValue;
        ifstream thisIMUDataFile(thisIMUFilePath);
        for(int i = 0; i < 3; i++) thisIMUDataFile >> noneValue;
        double R,P,Y;
        thisIMUDataFile >> R >> P >> Y;
        auto Ori = tf::createQuaternionFromRPY(R,P,Y);
        Ori = Ori.normalize();
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
        return thisIMUData;
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
        Eigen::Matrix3d camera_to_laser;
        camera_to_laser = imu_laser_R;
        // TO DO
        while(GTfile.good()){
            string infoStr;
            getline(GTfile, infoStr);
            if(infoStr.empty()) continue;
            istringstream iss(infoStr);
            Eigen::Matrix3d tempR;
            Eigen::Vector3d tempt;
            iss >> tempR(0,0) >> tempR(0,1) >> tempR(0,2) >> tempt(0);
            iss >> tempR(1,0) >> tempR(1,1) >> tempR(1,2) >> tempt(1);
            iss >> tempR(2,0) >> tempR(2,1) >> tempR(2,2) >> tempt(2);
            auto tempR2 = camera_to_laser.transpose() * tempR * camera_to_laser;
            auto tempt2 = camera_to_laser.transpose() * tempt;
            Eigen::Quaterniond tempQ(tempR2);
            nav_msgs::Odometry tempOdom;
            tempOdom.pose.pose.position.x = tempt2(0);
            tempOdom.pose.pose.position.y = tempt2(1);
            tempOdom.pose.pose.position.z = tempt2(2);
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
    vector<sensor_msgs::Imu> imuData;
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
                sensor_msgs::Imu imuDataTemp = readIMU(IMUTempPath);
                double thisImuTime = imuTimes[ind];
                if( thisImuTime < 10000 || thisImuTime <= lastImuTime ){
                    cout << "get bad data abandon " << ind << fixed << lastImuTime << " " << thisImuTime << endl;
                    ind++;
                    continue;
                }
                lastImuTime = thisImuTime;
                imuDataTemp.header.stamp = ros::Time().fromSec(thisImuTime);
                imuDataTemp.header.frame_id = "laser";
                imuData.push_back(imuDataTemp);
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
        //IMU
        double lastImuTime = imuData[startImu].header.stamp.toSec() - 0.1;
        for(int i = startImu; i <= endImu; i++){
            double thisImuTime = imuData[i].header.stamp.toSec();
            if(thisImuTime < 10000 || thisImuTime <= lastImuTime){
                cout << " imu get bad time " << i << " " << lastImuTime << " " << thisImuTime << endl;
                getchar();
                end();
                return;
            }
            lastImuTime = thisImuTime;
            bag.write("/imu/data", imuData[i].header.stamp, imuData[i]);
        }

        //GT
        for(int i = startOdom; i <= endOdom; i++){
            gtData[i].header.stamp = ros::Time().fromSec(rawLaserTimes[i+odom0InRaw]);
            bag.write("/gt", gtData[i].header.stamp, gtData[i]);
        }

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
//                char key = getchar();
//                if(key == 'q'){
//                    end();
//                    return;
//                }
                points->clear();
            }
            else{
                cout << "no bin or laebl " << bin  << " " << label << endl;
            }
        }

    }

    void end(){
        bag.close();
        exit(0);
    }

    int odom0InRaw = -1;
    int startOdom = -1;
    int endOdom = -1;
    int startImu = -1;
    int endImu = -1;
    void alignedTime(){
        //judge the dataset
        bool exitFlag = false;
        if( odomFramesSize==0 ){
            cerr << "data odom wrong " << endl;
            exitFlag = true;
        }
        int startRaw = -1;
        int endRaw = -1;
        double startTime = 0;
        double endTime = 0;
        //we got the odom data len
        //we got the raw  data len
        //we must konw the first odom frame conrrest which raw data
        //we must konw the start odom frame and the end odom frame
        if(startOdom==-1) startOdom=0;
        if(endOdom==-1) endOdom=odomFramesSize;
        if(odom0InRaw==-1) odom0InRaw=0;
        // save odom, gt, label time use rawLaserTimes:startRaw-endRaw
        startRaw = odom0InRaw + startOdom;
        endRaw = odom0InRaw + endOdom;

        // for get startImu and endImu
        startTime = rawLaserTimes[startRaw];
        endTime = rawLaserTimes[endRaw];
        // save imu use imuData:startImu-endImu
        for(int i = 0; i < imuData.size(); i++){
            double t = imuData[i].header.stamp.toSec();
            if(t > startTime){
                startImu = i;
                break;
            }
        }
        for(int i = imuData.size()-1; i >= 0; i--){
            double t = imuData[i].header.stamp.toSec();
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
        << "imu" << "\t\t" << getLocalTime(imuData[startImu].header.stamp.toSec()) << "\t" << getLocalTime(imuData[endImu].header.stamp.toSec()) << endl;
    }
private:
    string pathBase;
    string saveName;
    rosbag::Bag bag;
public:
    kitti() = delete;
    kitti(string pathBase_, string saveName_,int odom0InRaw_=-1, int startOdom_=-1, int endOdom_=-1){
        //////////////////////////////////////////////
        pathBase = pathBase_;
        saveName = saveName_;
        odom0InRaw = odom0InRaw_;
        startOdom = startOdom_;
        endOdom = endOdom_;
        ////////////////////////////////////////////
        velodynePath = pathBase + "/velodyne";
        labelPath = pathBase + "/labels";
        IMUDataPath = pathBase + "/oxts/data";
        GTPath = pathBase + "/poses.txt";
        rawTimesPath = pathBase + "/timestampsRaw.txt";
        IMUTimesPath = pathBase + "/oxts/timestamps.txt";
        calbiPath = pathBase + "/calib.txt";
        if( 0 == access(saveName.c_str(), 0)){
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

    //kitti seq01("/home/qh/kitti/01", "/home/qh/kitti/seq01.bag", 0, 0, 1100);

    // 0 - 500 高度有点问题还是抛弃吧
    //kitti seq08("/home/qh/kitti/08", "/home/qh/kitti/seq08.bag", 1100, 500, 4070);

    //
    kitti seq00("/home/qh/kitti/00", "/home/qh/kitti/seq00.bag", 0, 0, 4540);

    return 0;
}

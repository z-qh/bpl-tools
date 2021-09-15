#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>

#include "imu_gnss/gps_navi_msg.h"
#include "imu_gnss/gps_ins_msg.h"

#include <dirent.h>
#include <deque>
#include <fstream>

using namespace std;

class gnss{
private:
    string str = "/home/qh/robot_ws/gnss.txt";
    const int imuQueLength = 100;   //imu频率
    int imuPointerLast = -1;        //IMU最新一帧的索引值
    int imuPointerFront = 0;

    //这块包括每秒内imu的时间戳，三角数据，加速度，速度，位移。
    bool newImudata = false;
    bool firstImu(true);
    float first_imu_roll = 0;
    float first_imu_pitch = 0;
    float first_imu_yaw = 0;

    float first_imu_acc_x = 0;
    float first_imu_acc_y = 0;
    float first_imu_acc_z = 0;

    float first_imu_velo_x = 0;
    float first_imu_velo_y = 0;
    float first_imu_velo_z = 0;

    double imuTime[imuQueLength] = {0};     //时间戳

    float imuRoll[imuQueLength] = {0};      //IMU角度值
    float imuPitch[imuQueLength] = {0};
    float imuYaw[imuQueLength] = {0};

    float imuInsRoll[imuQueLength] = {0};      //IMU角度值
    float imuInsPitch[imuQueLength] = {0};
    float imuInsYaw[imuQueLength] = {0};

    float imuAccX[imuQueLength] = {0};      //IMU线加速度
    float imuAccY[imuQueLength] = {0};
    float imuAccZ[imuQueLength] = {0};

    float imuVeloX[imuQueLength] = {0};     //IMU线速度
    float imuVeloY[imuQueLength] = {0};
    float imuVeloZ[imuQueLength] = {0};


    float imuShiftX[imuQueLength] = {0};    //IMU位移量
    float imuShiftY[imuQueLength] = {0};
    float imuShiftZ[imuQueLength] = {0};
    //IMU速度
    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];
    //角度累计
    float imuAngularRotationX[imuQueLength] = {0};
    float imuAngularRotationY[imuQueLength] = {0};
    float imuAngularRotationZ[imuQueLength] = {0};

    geometry_msgs::Quaternion ImuOrien;
    /*******************************************************/

    /**********************************************************
        GPS相关变量
    **********************************************************/
    const int GNSSQueLength = 100;
    double GNSS_time[GNSSQueLength]={0};
    int GNSSPointerLast = -1;        //IMU最新一帧的索引值
    int GNSSPointerBack = -1;

    double gpsTimeCurrent;
    double lat,lon,alt;         //纬度，经度，海拔
    double last_lat, last_lon, last_alt;

    int last_status;
    int satNum;
    bool newGnssData(false);
    double northspeed = 0;
    double eastspeed = 0;
    double skyspeed = 0;
    //////////////////////
    int gnss_status;                 //GPS状态标志为当状态为4或者5时信号较好
    const int stateLengthQue = 3;
    std::deque<int>gnssStateQue;
    //////////////////////

    bool isFirstGPS(true);
    bool isFirstGNSS(true);
    double first_x,first_y,first_z;
    double Xd = 0,Yd = 0,Zd = 0;            //转换后的GNSS数据
    double last_Xd = 0, last_Yd = 0, last_Zd = 0;
    double sinLat,cosLat,sinLon,cosLon;     //起始点经纬度sin cos
    double first_Xe,first_Ye,first_Ze;

    ros::Publisher pubHighFreGPS;
    ros::Publisher pubGnssOdom;
    ros::Publisher pubImuGnssWithState;
    int num_count =0;
    double gnss_roll = 0, gnss_pitch=0, gnss_yaw = 0;
    double first_gnss_roll = 0, first_gnss_pitch = 0, first_gnss_yaw = 0;
    string state;
    //////////////////
    double imu_time_now=0;
    double gnss_time_first = 0;
    //////////////////

    static void toEulerAngle(const Eigen::Quaterniond&q, double &roll, double &pitch, double& yaw){
        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        yaw = atan2(siny_cosp, cosy_cosp);
    }
    //Kalman filter
    void KalmanFun(const pcl::PointXYZ &acc, pcl::PointXYZ &inPos, double dt,
                   pcl::PointXYZ &outPos, int status) {

        static bool isFirstKal(true);
        static Eigen::MatrixXd A(4, 4);
        static Eigen::MatrixXd B(4, 2);
        static Eigen::MatrixXd u(2, 1);
        static Eigen::MatrixXd Q(4, 4);
        static Eigen::MatrixXd H(2, 4);
        static Eigen::MatrixXd R(2, 2);
        static Eigen::MatrixXd X_pdct(4, 1);
        static Eigen::MatrixXd Pk_pdct(4, 4);
        static Eigen::MatrixXd K(4, 2);
        static Eigen::MatrixXd Z_meas(2, 1);
        static Eigen::MatrixXd X_evlt(4, 1);
        static Eigen::MatrixXd Pk_evlt(4, 4);
        if (isFirstKal) {

            X_pdct.setZero();
            Pk_pdct.setZero();
            K.setZero();
            Z_meas.setZero();
            X_evlt.setZero();
            Pk_evlt.setZero();
            isFirstKal = false;
        }
        A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

        B << 0.5 * dt * dt, 0, 0, 0.5 * dt * dt, dt, 0, 0, dt;

        u << acc.x, acc.y;

        Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                0.01;

        H << 1, 0, 0, 0, 0, 1, 0,
                0;
        if (status < 2) {
            R << 1, 0, 0, 1;
        } else if (status == 2) {
            R << 0.5, 0, 0, 0.5;
        } else {
            R << 0.01, 0, 0, 0.01;
        }
        X_pdct = A * X_evlt + B * u;
        Pk_pdct = A * Pk_evlt * A.transpose() + Q;

        Eigen::MatrixXd tmp(2, 2);
        tmp = H * Pk_pdct * H.transpose() + R;
        K = Pk_pdct * H.transpose() * tmp.inverse();

        Z_meas << inPos.x, inPos.y;
        X_evlt = X_pdct + K * (Z_meas - H * X_pdct);
        Pk_evlt = (Eigen::MatrixXd::Identity(4, 4) - K * H) * Pk_pdct;

        outPos.x = X_evlt(0, 0);
        outPos.y = X_evlt(1, 0);
        outPos.z = inPos.z;
    }
/***************经纬高转enu坐标******************/
    void BLH2ENU(double lat, double lon, double alt, double &x, double &y,
                 double &z, double sin_lat, double cos_lat, double sin_lon,
                 double cos_lon) {
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
    void AccumulateIMUShift()
    {
        float roll = imuRoll[imuPointerLast];
        float pitch = imuPitch[imuPointerLast];
        float yaw = imuYaw[imuPointerLast];
        float accX = imuAccX[imuPointerLast];
        float accY = imuAccY[imuPointerLast];
        float accZ = imuAccZ[imuPointerLast];

        // accX = accX + sin(pitch) * 9.81;
        // accY = accY - sin(roll) * cos(pitch) * 9.81;
        // accZ = accZ - cos(roll) * cos(pitch) * 9.81;

        //根据roll，pitch，yaw计算 出 世界坐标系下的加速度
        double x1 = accX;
        double y1 = cos(roll) * accY - sin(roll) * accZ;
        double z1 = sin(roll) * accY + cos(roll) * accZ;

        double x2 = cos(pitch) * x1 + sin(pitch) * z1;
        double y2 = y1;
        double z2 = -sin(pitch) * x1 + cos(pitch) * z1;

        accX = (cos(yaw) * x2 - sin(yaw) * y2);
        accY = (sin(yaw) * x2 + cos(yaw) * y2);
        accZ = z2;
        // std::cout<<"acc "<<accX<<" "<<accY<<" "<<accZ<<std::endl;
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;//前一帧
        double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];//imu测量周期
        //求每个imu时间点的位移与速度,两点之间视为匀加速直线运动
        //相当于预积分，要求IMU频率必须比激光频率高
        if (timeDiff < 0.1) {

            imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
            imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
            imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;
            // std::cout<<"vel x "<<imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2
            //          <<" y "<<imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2
            //          <<" z "<<imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2<<" "<<timeDiff<<std::endl;

            imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
            imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
            imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }
public:
    void ImuCallback(sensor_msgs::Imu imuIn){
        ImuOrien = imuIn.orientation;

        double roll,pitch,yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);        //在东北天坐标系下的角度值

        float accX = imuIn.linear_acceleration.x + sin(pitch) * 9.81;
        float accY = imuIn.linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accZ = imuIn.linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;

        //进来一个imu的数据，增加一次
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;

        imuTime[imuPointerLast] = imuIn.header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch ;
        imuYaw[imuPointerLast] = yaw;

        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;

        imuAngularVeloX[imuPointerLast] = imuIn.angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn.angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn.angular_velocity.z;
        AccumulateIMUShift();

        imu_time_now = imuIn.header.stamp.toSec();
        newImudata = true;
    }

    //输出激光坐标系下xyz
    void GNSSCallBack(imu_gnss::gps_navi_msg gnssMsgIn) {
        gpsTimeCurrent = gnssMsgIn.header.stamp.toSec();
        lat = gnssMsgIn.latitude;  //纬度
        lon = gnssMsgIn.longitude; //经度
        alt = gnssMsgIn.elevation;

        northspeed = gnssMsgIn.northspeed;
        eastspeed = gnssMsgIn.eastspeed;
        skyspeed = gnssMsgIn.skyspeed;

        state = gnssMsgIn.PosStatus;

        gnss_roll  = (gnssMsgIn.rollAngle) * M_PI / 180.0;
        gnss_pitch = gnssMsgIn.pitchAngle * M_PI / 180.0;
        gnss_yaw   = gnssMsgIn.yawAngle;
        gnss_yaw   = (90.0 - gnss_yaw) * M_PI / 180.0;
        if(lat == 0){
            lat = last_lat; lon = last_lon; alt = last_alt;
        }else{
            last_lat = lat; last_alt = alt; last_lon = lon;
        }

        GNSSPointerLast = (GNSSPointerLast + 1) % GNSSQueLength;
        if(isFirstGNSS){
            gnss_time_first = gpsTimeCurrent;
            std::cout<<"first init"<<std::endl;
            isFirstGNSS = false;
            sinLat = sin(lat * M_PI / 180.0);
            cosLat = cos(lat * M_PI / 180.0);
            sinLon = sin(lon * M_PI / 180.0);
            cosLon = cos(lon * M_PI / 180.0);

            first_gnss_roll     = gnss_roll;
            first_gnss_pitch    = gnss_pitch;
            first_gnss_yaw      = gnss_yaw;
        }
        BLH2ENU(lat, lon, alt, Xd, Yd, Zd, sinLat, cosLat, sinLon, cosLon);
        if(sqrt(pow((last_Xd-Xd),2) + pow((last_Yd-Yd),2) + pow((last_Zd-Zd),2)) > 5){
            Xd = last_Xd;   Yd = last_Yd;   Zd = last_Zd;
        }else{
            last_Xd = Xd;   last_Yd = Yd;   last_Zd = Zd;
        }
        if(state == "INS_RTKFIXED" || state == "INS_RTKFLOAT"){
            std::ofstream of(str,std::ios::app);
            if(!of.is_open()){
                std::cout<<"open file "<<str<<" error!"<<std::endl;
                return ;
            }
            string time_gnss = std::to_string(imuTime[imuPointerLast]);
            // of<<time_gnss<<std::endl;
            // of<<Xd<<" "<<Yd<<" "<<Zd<<" "<<gnss_roll<<" "<<gnss_pitch<<" "<<gnss_yaw + M_PI/2.0<<std::endl;
            of.close();
        }


        geometry_msgs::Quaternion qOut = tf::createQuaternionMsgFromRollPitchYaw(gnss_roll,
                                                                                 gnss_pitch, gnss_yaw);
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now().fromSec(imu_time_now);
        odom.header.stamp = gnssMsgIn.header.stamp;
        odom.header.frame_id = "camera_init";
        odom.pose.pose.position.x  = Xd;
        odom.pose.pose.position.y  = Yd;
        odom.pose.pose.position.z  = Zd;

        odom.pose.pose.orientation.x = qOut.x;
        odom.pose.pose.orientation.y = qOut.y;
        odom.pose.pose.orientation.z = qOut.z;
        odom.pose.pose.orientation.w = qOut.w;
        pubGnssOdom.publish(odom);
        newGnssData = true;
    }
};

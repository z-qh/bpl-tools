#include "ros/ros.h"
#include <fstream>
//这个文件需要以来一个ROS下的消息文件，这个消息文件和生成的头文件都放在include下
//这个依赖项是gnss_driver,xsens_imu_driver,serial这三个包
#include "include/gps_navi_msg.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <iostream>
bool isFirstGPS = true;
bool isFirstGNSS = true;

double first_Xe,first_Ye,first_Ze;
double lat,lon,alt;
double first_roll, first_pitch, first_yaw;
double sinLat,cosLat,sinLon,cosLon;

double NorthEastX,NorthEastY,NorthEastZ,NorthEastR,NorthEastP,NorthEastW;

std::ofstream gps_file;

ros::Publisher gps_odom_pub;
ros::Publisher pub_groundTruth;

geometry_msgs::Quaternion tempQuat;

nav_msgs::Odometry Odom;

ros::Time current_time, last_time;
pcl::PointCloud<pcl::PointXYZI>::Ptr groundTruth(new pcl::PointCloud<pcl::PointXYZI>());

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
void handle_gps_callback(const gnss_driver::gps_navi_msg& msg)
{
    //std::cout << msg.longitude << std::endl;
    //std::cout << msg.latitude << std::endl;

    if(msg.latitude==0 || msg.longitude==0)
        return;
    gps_file << msg.longitude << std::endl;
    gps_file << msg.latitude << std::endl;
    lat = msg.latitude;
    lon = msg.longitude;
    alt = msg.elevation;
    if(isFirstGNSS){
        std::cout<<"first init"<<std::endl;
        isFirstGNSS = false;
        sinLat = sin(lat * M_PI / 180.0);
        cosLat = cos(lat * M_PI / 180);
        sinLon = sin(lon * M_PI / 180);
        cosLon = cos(lon * M_PI / 180);
        first_roll = msg.rollAngle;
        first_pitch = msg.pitchAngle;
        first_yaw = -msg.yawAngle + 90;
    }
    BLH2ENU(lat, lon, alt, NorthEastX, NorthEastY, NorthEastZ, sinLat, cosLat, sinLon, cosLon);
    pcl::PointXYZI thisPoint;
    thisPoint.x = NorthEastX;
    thisPoint.y = NorthEastY;
    thisPoint.z = NorthEastZ;
    groundTruth->push_back(thisPoint);
    NorthEastR = (msg.rollAngle);
    NorthEastP = (msg.pitchAngle);
    NorthEastW = (90-(msg.yawAngle));
    std::cout<<"roll "<<NorthEastR<<" "<<NorthEastP<<" "<<NorthEastW<<std::endl;
    std::cout<<"msg roll "<<msg.rollAngle<<" "<<msg.pitchAngle<<" "<<msg.yawAngle<<std::endl;
//    NorthEastW = -msg.yawAngle*M_PI/180.0 + 90.0*M_PI/180.0;
    //tf::Matrix3x3(tf::Quaternion(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w)).getRPY(NorthEastR, NorthEastP,NorthEastW);
    tempQuat = tf::createQuaternionMsgFromRollPitchYaw(NorthEastR*M_PI/180.0, NorthEastP*M_PI/180.0, NorthEastW*M_PI/180.0);
    //std::cout << "X: " << NorthEastX << std::endl;
    //std::cout << "Y: " << NorthEastY << std::endl;
    //std::cout << "Z: " << NorthEastZ << std::endl;
    //std::cout << "roll: " << NorthEastR << std::endl;
    //std::cout << "pitch: " << NorthEastP << std::endl;
    //std::cout << "yaw: " << NorthEastW << std::endl;

    current_time = ros::Time::now();
    Odom.header.frame_id = "gps";
    Odom.header.stamp = current_time;
    Odom.pose.pose.orientation.x = tempQuat.x;
    Odom.pose.pose.orientation.y = tempQuat.y;
    Odom.pose.pose.orientation.z = tempQuat.z;
    Odom.pose.pose.orientation.w = tempQuat.w;
    std::cout<<tempQuat.x<<" y "<<tempQuat.y<<" "<<tempQuat.z<<" w "<<tempQuat.w<<" roll "<<NorthEastP<<" "<<NorthEastP<<std::endl;
    Odom.pose.pose.position.x = NorthEastX;
    Odom.pose.pose.position.y = NorthEastY;
    Odom.pose.pose.position.z = NorthEastZ;

    gps_odom_pub.publish(Odom);
    sensor_msgs::PointCloud2 tmpCloud;
    pcl::toROSMsg(*groundTruth,tmpCloud);
    tmpCloud.header.stamp = current_time;
    tmpCloud.header.frame_id = "gps";
    pub_groundTruth.publish(tmpCloud);
}

int main(int argc, char** argv)
{
    gps_file.open("/home/qh/nodelish.txt", std::ios::out);
    gps_file.precision(13);
    ros::init(argc, argv, "gnss_get_posi");
    ros::NodeHandle nh;
    //gnss_driver::gps_navi_msg m_navi;
    ros::Subscriber sub = nh.subscribe("/gps_navi", 1, handle_gps_callback);
    gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/qwe",1);
    pub_groundTruth = nh.advertise<sensor_msgs::PointCloud2>("/groundTruth",1);


    ros::spin();
    gps_file.close();

    return 0;
}
/*
 * Tips，亿天一个小技巧
 *
 *vector里面嵌套非指针类型的点云将引发内存深浅拷贝错误，使用第二种方式请注意reset
 *vector<pcl::PointCloud<pcl::PointXYZI>> pointCloudVector(2);×
 *vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointCloudVector(2);√
 *
 *
 *直接使用点云，不需要对点云进行初始化，默认构造就好
 *
 *
 *使用点云智能指针，需要对点云进行初始化，像这样
 *pcl::PointCloud<pcl::PointXYZI>::Ptr YESS(new pcl::PointCloud<pcl::PointXYZI>());
 *
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
using namespace std;

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint32_t, label, label)
)

typedef PointXYZIL PointSemantic;

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

typedef PointXYZIRPYT PointTypePose;


#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "gnss_driver/gps_navi_msg.h"
#include "tf/transform_datatypes.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

bool isFirstGPS = true;
bool isFirstGNSS = true;

double first_Xe,first_Ye,first_Ze;
double lat,lon,alt;
double first_roll, first_pitch, first_yaw;
double sinLat,cosLat,sinLon,cosLon;
double NorthEastX,NorthEastY,NorthEastZ,NorthEastR,NorthEastP,NorthEastW;


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
pcl::PointXYZ handle_gps_callback(gnss_driver::gps_navi_msgConstPtr msg)
{
    pcl::PointXYZ result;

    if(msg->latitude==0 || msg->longitude==0)
        return result;
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->elevation;
    if(isFirstGNSS){
        std::cout<<"first init"<<std::endl;
        isFirstGNSS = false;
        sinLat = sin(lat * M_PI / 180.0);
        cosLat = cos(lat * M_PI / 180);
        sinLon = sin(lon * M_PI / 180);
        cosLon = cos(lon * M_PI / 180);
        first_roll = msg->rollAngle;
        first_pitch = msg->pitchAngle;
        first_yaw = -msg->yawAngle + 90;
    }
    BLH2ENU(lat, lon, alt, NorthEastX, NorthEastY, NorthEastZ, sinLat, cosLat, sinLon, cosLon);
    pcl::PointXYZI thisPoint;
    thisPoint.x = NorthEastX;
    thisPoint.y = NorthEastY;
    thisPoint.z = NorthEastZ;
    NorthEastR = (msg->rollAngle);
    NorthEastP = (msg->pitchAngle);
    NorthEastW = (90-(msg->yawAngle));

    geometry_msgs::Quaternion tempQuat;
    tempQuat = tf::createQuaternionMsgFromRollPitchYaw(NorthEastR*M_PI/180.0, NorthEastP*M_PI/180.0, NorthEastW*M_PI/180.0);

    result.x = thisPoint.x;
    result.y = thisPoint.y;
    result.z = thisPoint.z;

    return result;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AAAA");
    ros::NodeHandle nh("~");

    while (ros::ok())
    {
        std::cout << "123" << std::endl;
    }
    return 0;
}








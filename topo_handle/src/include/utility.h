#pragma once

#define PCL_NO_PRECOMPILE
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


using namespace std;

// typedef pcl::PointXYZI  PointType;
// lslidar
//extern const int N_SCAN = 32;
//extern const int Horizon_SCAN = 2000;
//extern const float ang_res_x = 0.18;
//extern const float ang_res_y = 1.0;
//extern const float ang_bottom = 16.1;
//extern const int groundScanInd = 15;

//Ouster OS1-32
extern const string imuTopic = "/imu/data";
extern const string lidarTopic = "/os_cloud_node/points";
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 45.0/float(N_SCAN-1);
extern const float ang_bottom = 16.0+0.1;
extern const int groundScanInd = 15;

//VLP-16
//extern const int N_SCAN = 16;
//extern const int Horizon_SCAN = 1800;
//extern const float ang_res_x = 0.2;
//extern const float ang_res_y = 2.0;
//extern const float ang_bottom = 15.0+0.1;
//extern const int groundScanInd = 7;


extern const bool loopClosureEnableFlag = true;	        //闭环检测标志位
extern const double mappingProcessInterval = 0.2;      //4hz??  0.2

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 100;

extern const float sensorMountAngle = 0.0;
//extern const float segmentTheta = 1.0472;
extern const float segmentTheta = 45.0/180.0*M_PI;
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 500.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

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

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointXYZI PointType;
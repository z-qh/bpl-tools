#ifndef _FEATURE_H_
#define _FEATURE_H_

#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

struct smoothness_t {
    float value;
    size_t ind;
};
struct by_value {
    bool operator()(smoothness_t const& left, smoothness_t const& right) { return left.value < right.value; }
};

class FeatureExtracter {
public:
    FeatureExtracter();
    ~FeatureExtracter();
    pcl::PointCloud<pcl::PointXYZL>::Ptr get(pcl::PointCloud<pcl::PointXYZL>::Ptr inCloud, int ground_pick_ = 4, int surf_pick_ = 10, int corner_pick_ = 10);

private:
    void init();
    void reset();
    void labelComponents(int row, int col);

private:
    int N_SCAN;
    int Horizon_SCAN;
    int groundScanInd;
    float ang_res_x;
    float ang_res_y;
    float ang_up;
    float ang_bottom;
    float sensorMinimumRange;
    float sensorMountAngle;
    float segmentAlphaX;
    float segmentAlphaY;
    float segmentTheta;
    float edgeThreshold;
    float surfThreshold;

private:
    pcl::PointXYZL nanPoint;
    pcl::PointCloud<pcl::PointXYZL>::Ptr fullCloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr segmentedCloud;

    cv::Mat rangeMat;
    cv::Mat groundMat;
    cv::Mat clusterMat;
    int clusterCount;

    uint16_t* allPushedIndX; // array for tracking points of a segmented object
    uint16_t* allPushedIndY;

    uint16_t* queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t* queueIndY;
    std::vector<std::pair<int8_t, int8_t>> neighborIterator;

private:
    int32_t* startRingIndex;
    int32_t* endRingIndex;
    bool* segmentedCloudGroundFlag;
    uint32_t* segmentedCloudColInd;
    float* segmentedCloudRange;

private:
    std::vector<smoothness_t> cloudSmoothness;
    float* cloudCurvature;
    int* cloudNeighborPicked;

private:
    pcl::PointCloud<pcl::PointXYZL>::Ptr FeaturePoints;
};

#endif
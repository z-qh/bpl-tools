#include "ros/ros.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl/io/io.h"
#include "pcl_conversions/pcl_conversions.h"

#include <iostream>
#include <vector>
#include <string>

#include "pcl/registration/icp.h"

#include "Eigen/Eigen"

using namespace std;

pcl::PointCloud<pcl::PointXYZ> frame1;
pcl::PointCloud<pcl::PointXYZ> frame2;

void getCloudFromPCDFile(string fileName, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::io::loadPCDFile(fileName, cloud);
}

void loadCloud()
{
    int frame = 100;
    string FileNameBase = "/home/qh/frame";

    string loadFileName = FileNameBase + to_string(frame) + ".pcd";
    getCloudFromPCDFile(loadFileName, frame1);

    frame = 105;

    loadFileName = FileNameBase + to_string(frame) + ".pcd";
    getCloudFromPCDFile(loadFileName, frame2);
}

/*
 * PCL ICP 方法
 */
void LOAMHandleProcess()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);

    *src = frame1;
    *tgt = frame2;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setMaximumIterations(100);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);

    icp.align(*out);

    out->resize(tgt->size() + out->size());



}

int main(int argc, char** argv)
{
    //loadCloud();

    LOAMHandleProcess();

    return 0;
}
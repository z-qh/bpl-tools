#include "iostream"
#include "fstream"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"


using namespace std;

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr LOAM, GNSS, ALIGN;
    LOAM.reset(new pcl::PointCloud<pcl::PointXYZ>());
    GNSS.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ALIGN.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("/home/qh/add_ws/10/Align_GNSS.pcd", *GNSS);
    pcl::io::loadPCDFile("/home/qh/add_ws/10/Align_LOAM.pcd", *LOAM);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(GNSS);
    icp.setInputTarget(LOAM);
    icp.align(*ALIGN);

    pcl::io::savePCDFileASCII("/home/qh/add_ws/10/icp.pcd", *ALIGN);
}

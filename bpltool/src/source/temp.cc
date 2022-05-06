#include "iostream"
#include "fstream"

#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"


#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

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


pcl::PointCloud<PointXYZIL>::Ptr transformPointCloud(pcl::PointCloud<PointXYZIL>::Ptr cloudIn, PointXYZIRPYT& transformIn){

    pcl::PointCloud<PointXYZIL>::Ptr cloudOut(new pcl::PointCloud<PointXYZIL>());

    PointXYZIL *pointFrom;
    PointXYZIL pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn.yaw) * pointFrom->x - sin(transformIn.yaw) * pointFrom->y;
        float y1 = sin(transformIn.yaw) * pointFrom->x + cos(transformIn.yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn.roll) * y1 - sin(transformIn.roll) * z1;
        float z2 = sin(transformIn.roll) * y1 + cos(transformIn.roll)* z1;

        pointTo.x = cos(transformIn.pitch) * x2 + sin(transformIn.pitch) * z2 + transformIn.x;
        pointTo.y = y2 + transformIn.y;
        pointTo.z = -sin(transformIn.pitch) * x2 + cos(transformIn.pitch) * z2 + transformIn.z;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}


pcl::PointCloud<PointXYZIRPYT> readFile(string filePath){
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file(filePath);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        PointXYZIRPYT tmpP;
        int index;
        double time;
        double x,y,z;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

void generateDaquanLast(){
    auto poses = readFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematic.txt");
    pcl::PointCloud<PointXYZIL>::Ptr allCloud(new pcl::PointCloud<PointXYZIL>());
    pcl::PointCloud<PointXYZIL>::Ptr allCloudDS(new pcl::PointCloud<PointXYZIL>());
    rosbag::Bag GNSS;
    GNSS.open("", rosbag::bagmode::Read);

    vector<string> topics;
    topics.push_back("/lslidar_point_cloud");
    rosbag::View VV(GNSS, rosbag::TopicQuery(topics));
    auto it = VV.begin();
    int poseCount = 0;
    for(it = VV.begin(); it != VV.end(); it++)
    {
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 tempMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            pcl::PointCloud<PointXYZIL>::Ptr tempCloud(new pcl::PointCloud<PointXYZIL>());
            pcl::fromROSMsg(tempMsg, *tempCloud);
            if(poseCount >= poses.size() ) break;
            if(abs(poses[poseCount].time - tempMsg.header.stamp.toSec()) < 0.05){
                auto transedTempCloud = transformPointCloud(tempCloud, poses[poseCount]);
                *allCloud += *transedTempCloud;
            }
            else if( tempMsg.header.stamp.toSec() - poses[poseCount].time > 0.05 ){
                poseCount++;
            }
        }
    }
    pcl::VoxelGrid<PointXYZIL> downSizeFilter;
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.setInputCloud(allCloud);
    downSizeFilter.filter(*allCloudDS);

    pcl::io::savePCDFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematic.pcd", *allCloudDS);

}

int main(){
    
}

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"

ros::Publisher tempCloudPub;
sensor_msgs::PointCloud2 segTemp;

ros::Subscriber originCloudSuber;

pcl::PointCloud<pcl::PointXYZI> originCloud;
pcl::PointCloud<pcl::PointXYZI> groundCloud;

void originCloudSuberHandle(const sensor_msgs::PointCloud2& msg)
{

    /*
     * 清除之前的点
     */
    originCloud.clear();
    groundCloud.clear();

    /*
     * 接受点云并且去除无用点
     */
    pcl::fromROSMsg(msg, originCloud);
    pcl::Indices nanPointIndex;
    pcl::removeNaNFromPointCloud(originCloud, originCloud, nanPointIndex);

    /*
     * 下半上扫描的点提取地面点
     */
    for(int i = 0; i < originCloud.points.size(); i++)
    {
        pcl::PointXYZI tempPoint = originCloud.points[i];
        double len = sqrt(tempPoint.x * tempPoint.x + tempPoint.y * tempPoint.y);
        double angle = atan(tempPoint.z / len) * 180 / M_PI;
        if(angle > 0)
            continue;
        groundCloud.push_back(tempPoint);
    }

    std::cout << "total:" << originCloud.points.size() << std::endl;
    std::cout << "ground:" << groundCloud.points.size() << std::endl;
    pcl::toROSMsg(groundCloud, segTemp);
    segTemp.header.frame_id = "map";
    tempCloudPub.publish(segTemp);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_test_seg");
    ros::NodeHandle nh;
    originCloudSuber = nh.subscribe("/rslidar_points", 1, originCloudSuberHandle);

    tempCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/segTemp", 1);

    ros::spin();

    return 0;
}

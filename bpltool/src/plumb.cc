#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "vector"

#define SCAN_NUMS 16
#define SCAN_ANGLE 45

const double angleFactor = SCAN_ANGLE / SCAN_NUMS * 1.0;

using namespace std;

ros::Subscriber subPointCloud;

typedef pcl::PointXYZI PointType;
//提取其中的垂直特征
pcl::PointCloud<pcl::PointXYZ> extractPlumbFeature(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    //划分有序点云
    //水平角边界
    double startOrien = atan2(cloud.points.front().y, cloud.points.front().x);
    double endOrien = atan2(cloud.points.back().y, cloud.points.back().x);
    if (endOrien - startOrien > 3 * M_PI)
    {
        endOrien -= 2 * M_PI;
    }
    else if(endOrien - startOrien < M_PI )
    {
        endOrien += 2 * M_PI;
    }
    //按照相距起始角度从小到大的顺序存放每个点
    vector<pcl::PointCloud<pcl::PointXYZ>>  scans(SCAN_NUMS);
    for(int i = 0; i < cloud.size(); i++)
    {
        pcl::PointXYZ thisPoint = cloud[i];
        double verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        double verticalIndex = round(verticalAngle / angleFactor);
        scans[verticalIndex].push_back(thisPoint);
    }
    //线扫检验完整性


    //检验同一线扫是否按照顺序
    cout << "start angle:" << startOrien << " ";
    for(int i = SCAN_NUMS-1; i >= 0; i--)
    {
        double min = atan2(scans[i].points.front().x, scans[i].points.front().y) * 180 / M_PI;
        for(int j = 0; j < scans[i].size(); j++)
        {
            pcl::PointXYZ thisPoint = scans[i].points[j];
            double horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            if(horizonAngle < min)
            {
                min = horizonAngle;
                cout << endl << "min change" << endl;
            }
            if(j%20 == 0)
            {
                cout << horizonAngle << endl;
            }
        }
    }
    cout << "end angle:" << endOrien << endl;
    return cloud;

}
void subPointCloudHandle(const sensor_msgs::PointCloud2& msg)
{
    //接受去掉地面点的点云
    pcl::PointCloud<pcl::PointXYZI> inCloud;
    pcl::fromROSMsg(msg, inCloud);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plumn_test");
    ros::NodeHandle nh;
    subPointCloud = nh.subscribe("/seg_points", 1, subPointCloudHandle);

    ros::spin();

    return 0;

}

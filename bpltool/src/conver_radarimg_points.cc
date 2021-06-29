/*
 * 将Radar二维数据转成点云并发布
 * 利用opencv读想数值并转为3D点
 */

#include "ros/ros.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/common/common.h"
#include "sensor_msgs/PointCloud2.h"

using namespace std;
using namespace cv;

pcl::PointCloud<pcl::PointXYZ> radar;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_points");
    ros::NodeHandle nh;
    ros::Publisher radarPub = nh.advertise<sensor_msgs::PointCloud2>("/radar_point", 1);
    Mat src = imread("/home/qh/000001.png", 0);
    for (int row = 0; row < src.rows; row++)
    {
        uchar* point = src.ptr<uchar>(row);
        for (int col = 0; col < src.cols; col++)
        {
            if (point[col] > 50)
                point[col] = 255;
            else
                point[col] = 0;
        }
    }
    Mat structureElement = getStructuringElement(MORPH_RECT, Size(4, 4), Point(-1, -1));
    erode(src, src, structureElement);
    dilate(src, src, structureElement, Point(-1, -1), 1);
    double mid_x = src.rows / 2.0;
    double mid_y = src.cols / 2.0;
    for (int row = 0; row < src.rows; row++)
    {
        pcl::PointXYZ tempPoint;
        uchar* point = src.ptr<uchar>(row);
        for (int col = 0; col < src.cols; col++)
        {
            if (point[col] ==  255)
            {
                double x = row - mid_x;
                double y = col - mid_y;
                tempPoint.x = x * 10.0 / 576.0;
                tempPoint.y = y * 10.0 / 576.0;
                tempPoint.z = 1;
                radar.push_back(tempPoint);
            }
        }
    }
    radar.header.stamp = 11;
    radar.header.frame_id = "map";
    radarPub.publish(radar);
    cout << radar.points.size() << endl;
    ros::Rate loop(1);
    while (ros::ok())
    {
        imshow("src", src);
        waitKey(20);
        radarPub.publish(radar);
        loop.sleep();
    }

    return 0;
}
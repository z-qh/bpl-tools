#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Eigen>

typedef pcl::PointXYZ PointType;
using namespace std;

ros::Subscriber subPcaPointCloud;
ros::Publisher pubResutlCloud;
ros::Publisher marker_pub;


pcl::PointCloud<PointType> inCloud;

pcl::PointCloud<pcl::PointXYZ> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B)
{
    pcl::PointCloud<pcl::PointXYZ> result;
    result.clear();
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    double nums = distance * 200;
    for(int i = 0; i < nums; i++)
    {
        pcl::PointXYZ tempPoint;
        tempPoint.x = B.x + diffX / nums * i;
        tempPoint.y = B.y + diffY / nums * i;
        tempPoint.z = B.z + diffZ / nums * i;
        result.push_back(tempPoint);
    }
    return result;
}

void subPcaPointCloudHandle(const sensor_msgs::PointCloud2 msg)
{
    //转为pcl数据
    pcl::fromROSMsg(msg, inCloud);
    //去除无用点
    vector<int> index;
    pcl::removeNaNFromPointCloud(inCloud, inCloud, index);
    //PCA：计算主方向
    Eigen::Vector4f pcaCentroid;//质心
    pcl::compute3DCentroid(inCloud, pcaCentroid);//估计质心，齐次坐标，(c0,c1,c2,1)

    Eigen::Matrix3f covariance;//协方差
    pcl::computeCovarianceMatrixNormalized(inCloud, pcaCentroid, covariance);//归一化协方差矩阵

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPca = eigen_solver.eigenvectors();//特征向量
    Eigen::Vector3f eigenValuesPca = eigen_solver.eigenvalues();//特征值

    eigenVectorsPca.normalize();
    eigenValuesPca.normalize();

    int flag = 0;
    int min = eigenValuesPca(0);
    for(int i = 1; i < 3; i++)
    {
        if(eigenValuesPca(i) < min)
        {
            flag = i;
            min = eigenValuesPca(i);
        }
    }

    PointType O, A[3];
    pcl::PointCloud<PointType> arrow;
    O.x = 0, O.y = 0, O.z = 0;
    for(int i = 0; i < 3; i ++)
    {
        //if(i == flag)
        //    continue;
        A[i].x = eigenVectorsPca.col(i)(0)*eigenValuesPca(i);
        A[i].y = eigenVectorsPca.col(i)(1)*eigenValuesPca(i);
        A[i].z = eigenVectorsPca.col(i)(2)*eigenValuesPca(i);
        arrow += createLineCLoud(O, A[i]);
    }

    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(arrow, temp);
    temp.header.stamp = ros::Time::now();
    temp.header.frame_id = "map";
    pubResutlCloud.publish(temp);

    cout << eigenVectorsPca << endl;
    cout << eigenValuesPca << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca_test");
    ros::NodeHandle nh;

    subPcaPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pca_cloud", 1, subPcaPointCloudHandle);
    pubResutlCloud = nh.advertise<sensor_msgs::PointCloud2>("/pca_result", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/Orien", 1);

    ros::spin();

    return 0;
}
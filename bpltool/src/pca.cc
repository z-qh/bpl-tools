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
#include <algorithm>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Eigen>

typedef pcl::PointXYZ PointType;
using namespace std;

ros::Subscriber subPcaPointCloud;
ros::Publisher pubResutlCloud;
ros::Publisher marker_pub;

string mode;
string inputTopic;
string outputTopic;

pcl::PointCloud<pcl::PointXYZ> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B);
double getCloudMaxDis(const pcl::PointXYZ center, pcl::PointCloud<pcl::PointXYZ>& cloud);
pcl::PointXYZ getCloudCenter(const pcl::PointCloud<pcl::PointXYZ>& cloud);
pcl::PointCloud<pcl::PointXYZ> createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max);
void subPcaPointCloudHandle(const sensor_msgs::PointCloud2 msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pca_test");
    ros::NodeHandle nh("~");
    nh.getParam("mode", mode);
    transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    if(mode.find("online") != -1)
    {
        nh.getParam("inputTopic", inputTopic);
        nh.getParam("outputTopic", outputTopic);
        subPcaPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(inputTopic, 1, subPcaPointCloudHandle);
        pubResutlCloud = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);
        ros::spin();
    }
    else if(mode.find("outline") != -1)
    {

    }
    else
    {
        cout << "invalid param" << endl;
        return 0;
    }

    return 0;
}


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


double getCloudMaxDis(const pcl::PointXYZ center, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    double diss = 0;
    for(int i = 0; i < cloud.size(); i++)
    {
        double dissX = cloud.points[i].x - center.x;
        double dissY = cloud.points[i].y - center.y;
        double dissZ = cloud.points[i].z - center.z;
        double tempDiss = sqrt(dissX * dissX + dissY * dissY + dissZ * dissZ);
        if(tempDiss > diss)
        {
            diss = tempDiss;
        }
    }
    return diss;
}

pcl::PointXYZ getCloudCenter(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    double sumX, sumY, sumZ;
    for(int i = 0; i < cloud.size(); i++)
    {
        sumX += cloud.points[i].x;
        sumY += cloud.points[i].y;
        sumZ += cloud.points[i].z;
    }
    pcl::PointXYZ result;
    result.x = sumX / cloud.size();
    result.y = sumY / cloud.size();
    result.z = sumZ / cloud.size();
    cout << result.x << result.y << result.z << endl;
    return result;
}

pcl::PointCloud<pcl::PointXYZ> createFrameCloud(Eigen::Vector4f min, Eigen::Vector4f max)
{
    pcl::PointCloud<pcl::PointXYZ> frame;
    frame.clear();
    pcl::PointXYZ p[8];
    //取出八个点坐标
    p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
    p[1].x = max[0];  p[1].y = max[1];  p[1].z = min[2];
    p[2].x = max[0];  p[2].y = min[1];  p[2].z = max[2];
    p[3].x = max[0];  p[3].y = min[1];  p[3].z = min[2];
    p[4].x = min[0];  p[4].y = max[1];  p[4].z = max[2];
    p[5].x = min[0];  p[5].y = max[1];  p[5].z = min[2];
    p[6].x = min[0];  p[6].y = min[1];  p[6].z = max[2];
    p[7].x = min[0];  p[7].y = min[1];  p[7].z = min[2];
    //绘制一共是二个线条
    frame += createLineCLoud(p[0], p[1]);
    frame += createLineCLoud(p[2], p[3]);
    frame += createLineCLoud(p[4], p[5]);
    frame += createLineCLoud(p[6], p[7]);

    frame += createLineCLoud(p[0], p[2]);
    frame += createLineCLoud(p[1], p[3]);
    frame += createLineCLoud(p[4], p[6]);
    frame += createLineCLoud(p[5], p[7]);

    frame += createLineCLoud(p[0], p[4]);
    frame += createLineCLoud(p[2], p[6]);
    frame += createLineCLoud(p[1], p[5]);
    frame += createLineCLoud(p[3], p[7]);

    return frame;
}

void subPcaPointCloudHandle(const sensor_msgs::PointCloud2 msg)
{
    pcl::PointCloud<PointType> inCloud;
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

    //eigenVectorsPca.normalize();
    //eigenValuesPca.normalize();

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
    PointType O = getCloudCenter(inCloud);
    double dissFactor = getCloudMaxDis(O, inCloud);
    PointType orien[3];
    pcl::PointCloud<PointType> arrow;
    for(int i = 0; i < 3; i ++)
    {
        //if(i == flag)
        //    continue;
        orien[i].x = O.x + eigenVectorsPca.col(i)(0)*eigenValuesPca(i)*dissFactor;
        orien[i].y = O.y + eigenVectorsPca.col(i)(1)*eigenValuesPca(i)*dissFactor;
        orien[i].z = O.z + eigenVectorsPca.col(i)(2)*eigenValuesPca(i)*dissFactor;
        arrow += createLineCLoud(O, orien[i]);
        orien[i].x = O.x - eigenVectorsPca.col(i)(0)*eigenValuesPca(i)*dissFactor;
        orien[i].y = O.y - eigenVectorsPca.col(i)(1)*eigenValuesPca(i)*dissFactor;
        orien[i].z = O.z - eigenVectorsPca.col(i)(2)*eigenValuesPca(i)*dissFactor;
        arrow += createLineCLoud(O, orien[i]);
    }
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(arrow, min_pt, max_pt);

    arrow += createFrameCloud(min_pt, max_pt);

    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(arrow, temp);
    temp.header.stamp = ros::Time::now();
    temp.header.frame_id = "map";
    pubResutlCloud.publish(temp);
    cout << "center:" << O.x << O.y << O.z << endl;
    cout << eigenVectorsPca << endl;
    cout << eigenValuesPca << endl;
}


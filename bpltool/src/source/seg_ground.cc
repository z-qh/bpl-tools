/*
 * lego loam中的方法，分离地面点云和去除离群点
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <vector>
#include <cmath>
#include <deque>
#include <ctime>
#include <cfloat>
#include <string>

using namespace std;

#define PI 3.14159265

const int N_SCAN = 16;
const int Horizon_SCAN = 1800;
const float ang_bottom = 15.0+0.1;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const int groundScanInd = 7;
const float sensorMountAngle = 0.0;
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;
const float segmentTheta = 60.0/180.0*M_PI;

typedef pcl::PointXYZI PointType;

PointType nanPoint;

ros::Subscriber subLidarCloud;
ros::Publisher pubSegmentedCloud;
ros::Publisher pubSegmentedCloudPure;

//存放初始点云，全部点云，强度值点云
pcl::PointCloud<PointType>::Ptr laserCloudIn;
pcl::PointCloud<PointType>::Ptr fullCloud;
pcl::PointCloud<PointType>::Ptr fullInfoCloud;
pcl::PointCloud<PointType>::Ptr outlierCloud;
pcl::PointCloud<PointType>::Ptr segmentedCloud;
pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
pcl::PointCloud<PointType>::Ptr segmentedCLoudGround;

//距离矩阵，999999初始值未标记，距离值
cv::Mat rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
//地面点标记，1地面点，0初始值未标记，-1无效点
cv::Mat groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
//标签矩阵，-1地面点，0未标记，聚类值，999999无效点
cv::Mat labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

//初始角，终止角，角度差
double startOrientation;
double endOrientation;
double orientationDiff;

//分割过程的邻域查询，分割对象跟踪数组，广度优先搜索数组
std::vector<std::pair<uint8_t, uint8_t>> neighborIterator;
uint16_t *allPushedIndX;
uint16_t *allPushedIndY;
uint16_t *queueIndX;
uint16_t *queueIndY;
int labelCount;

void allocateMemory();
void resetParameters();
void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
void findStartEndAngle();
void projectPointCloud();
void groundRemoval();
void labelComponents(int row, int col);
void cloudSegmentation();
void publishCloud();
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

string inputTopic;
string pureTopic;
string groundTopic;

int main(int argc, char** argv)
{
    allocateMemory();
    ros::init(argc, argv, "seg_ground");
    ros::NodeHandle nh("~");
    nh.getParam("inputTopic", inputTopic);
    nh.getParam("pureTopic", pureTopic);
    nh.getParam("groundTopic", groundTopic);

    subLidarCloud = nh.subscribe(inputTopic, 1, cloudHandler);
    pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>(groundTopic, 1);
    pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>(pureTopic, 1);

    ros::spin();
    return 0;
}

//分配内存
void allocateMemory()
{
    //定义无效点
    nanPoint.x = 0;
    nanPoint.y = 0;
    nanPoint.z = 0;
    nanPoint.intensity = 0;

    //为PTR分配内存
    laserCloudIn.reset(new pcl::PointCloud<PointType>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCLoudGround.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

    std::pair<int8_t, int8_t> neighbor;

    //邻域数组初始化
    neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
}
//重置点云
void resetParameters()
{
    //清空点云
    laserCloudIn->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();
    segmentedCLoudGround->clear();

    //矩阵重置
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));

    labelCount = 1;

    //填充空点
    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
}
//将ROS转为PCL数据
void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    //去除无效数据
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
}
void findStartEndAngle()
{
    //初始和终止角
    startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,laserCloudIn->points[laserCloudIn->points.size() - 2].x) + 2 * M_PI;

    if (endOrientation - startOrientation > 3 * M_PI)
    {
        endOrientation -= 2 * M_PI;
    }
    else if (endOrientation - startOrientation < M_PI)
        endOrientation += 2 * M_PI;
    orientationDiff = endOrientation - startOrientation;
}
//处理点云索引
void projectPointCloud()
{
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        //点俯仰角与线索引
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;
        //点水平角与点索引
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
        //距离，排除距离较近的点
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < 0.1)
            continue;
        //距离矩阵
        rangeMat.at<float>(rowIdn, columnIdn) = range;
        //深度值记录行索引与线索引
        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        //记录到大点云中
        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range;
    }
}
//地面点去除
void groundRemoval()
{
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    //地面矩阵
    //-1, 不确定
    // 0, 初始值，计算后代表不接地
    // 1, 地面点
    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
        for (size_t i = 0; i < groundScanInd; ++i)
        {

            lowerInd = j + ( i )*Horizon_SCAN;
            upperInd = j + (i+1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1)
            {
                // 无效点
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }

            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10)
            {
                groundMat.at<int8_t>(i,j) = 1;
                groundMat.at<int8_t>(i+1,j) = 1;
            }
        }
    }
    // 提取地面点云
    // 标记不需要分割的点，地面点和无效点
    for (size_t i = 0; i < N_SCAN; ++i)
    {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
            if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX)
            {
                labelMat.at<int>(i,j) = -1;
            }
        }
    }
}
//聚类
void labelComponents(int row, int col)
{
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    bool lineCountFlag[N_SCAN] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    while(queueSize > 0){
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
        {
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                          rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                          rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta){

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }

    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;
    }
    if (feasibleSegment == true)
    {
        ++labelCount;
    }
    else
    {
        for (size_t i = 0; i < allPushedIndSize; ++i)
        {
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}
//点云分割
void cloudSegmentation()
{
    //分割过程
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i,j) == 0)
                labelComponents(i, j);

    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < N_SCAN; ++i)
    {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
            if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                if (labelMat.at<int>(i,j) == 999999)
                {
                    if (i > groundScanInd && j % 5 == 0)
                    {
                        outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        continue;
                    }
                    else
                    {
                        continue;
                    }
                }
                if (groundMat.at<int8_t>(i,j) == 1)
                {
                    if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                        continue;
                }
                segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                ++sizeOfSegCloud;
            }
        }
    }
}
//发布点云
void publishCloud()
{
    //将不含地面点的点云发布出去
    for (size_t i = 0; i < N_SCAN; ++i)
    {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
            if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999)
            {
                segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
            }
            if (labelMat.at<int>(i,j) == -1)
            {
                segmentedCLoudGround->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                segmentedCLoudGround->points.back().intensity = labelMat.at<int>(i,j);
            }
        }
    }
    // 2. 发布点云
    sensor_msgs::PointCloud2 laserCloudTemp;
    //地面的分割后点云
    pcl::toROSMsg(*segmentedCLoudGround, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header.frame_id = "map";
    pubSegmentedCloud.publish(laserCloudTemp);
    //不带地面的分割后点云
    pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
    laserCloudTemp.header.stamp = ros::Time::now();
    laserCloudTemp.header.frame_id = "map";
    pubSegmentedCloudPure.publish(laserCloudTemp);
}


void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    //耗时统计
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //将ROS转为PCL数据
    copyPointCloud(laserCloudMsg);
    //记录扫描初始角和终止角
    findStartEndAngle();
    //处理点云索引
    projectPointCloud();
    //地面点去除
    groundRemoval();
    //点云分割
    cloudSegmentation();
    //发布点云
    publishCloud();
    //重置参数
    resetParameters();

    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " time cost: " << (time_used.count() * 1000) << " ms." << endl;
}


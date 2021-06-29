#include <cmath>
#include <vector>

//pcl&ros
#include <pcl_conversions/pcl_conversions.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//ros
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <nav_msgs/Odometry.h>

typedef pcl::PointXYZI PointType;

using namespace std;

//扫描周期, velodyne频率10Hz，周期0.1s
const double scanPeriod = 0.1;

//初始化控制变量
const int systemDelay = 20;//弃用前20帧初始数据
int systemInitCount = 0;
bool systemInited = false;

//激光雷达线数
const int N_SCANS = 16;

//点云曲率, 40000为一帧点云中点的最大数量
float cloudCurvature[40000];
//曲率点对应的序号
int cloudSortInd[40000];
//点是否筛选过标志：0-未筛选过，1-筛选过
int cloudNeighborPicked[40000];
//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
int cloudLabel[40000];


ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;


//接收点云数据，velodyne雷达坐标系安装为x轴向前，y轴向左，z轴向上的右手坐标系
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (!systemInited) {//丢弃前20个点云数据
        systemInitCount++;
        if (systemInitCount >= systemDelay) {
            systemInited = true;
        }
        return;
    }

    //记录每个scan有曲率的点的开始和结束索引
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    //当前点云时间
    double timeScanCur = laserCloudMsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    //消息转换成pcl数据存放
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    std::vector<int> indices;
    //移除空点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    //点云点的数量
    int cloudSize = laserCloudIn.points.size();
    //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    //lidar scan结束点的旋转角，加2*pi使点云旋转周期为2*pi
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    //结束方位角与开始方位角差值控制在(PI,3*PI)范围，允许lidar不是一个圆周扫描
    //正常情况下在这个范围内：pi < endOri - startOri < 3*pi，异常则修正
    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }
    //lidar扫描线是否旋转过半
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++) {
        //坐标轴交换，velodyne lidar的坐标系也转换到z轴向前，x轴向左的右手坐标系
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        //计算点的仰角(根据lidar文档垂直角计算公式),根据仰角排列激光线号，velodyne每两个scan之间间隔2度
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID;
        //仰角四舍五入(加减0.5截断效果等于四舍五入)
        int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5));
        if (roundedAngle > 0){
            scanID = roundedAngle;
        }
        else {
            scanID = roundedAngle + (N_SCANS - 1);
        }
        //过滤点，只挑选[-15度，+15度]范围内的点,scanID属于[0,15]
        if (scanID > (N_SCANS - 1) || scanID < 0 ){
            count--;
            continue;
        }

        //该点的旋转角
        float ori = -atan2(point.x, point.y);
        if (!halfPassed) {//根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
            //确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;

            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        //-0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
        float relTime = (ori - startOri) / (endOri - startOri);
        //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point.intensity = scanID + scanPeriod * relTime;

        laserCloudScans[scanID].push_back(point);//将每个补偿矫正的点放入对应线号的容器
    }

    //获得有效范围内的点的数量
    cloudSize = count;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++) {//将所有的点按照线号从小到大放入一个容器
        *laserCloud += laserCloudScans[i];
    }
    int scanCount = -1;
    for (int i = 5; i < cloudSize - 5; i++) {//使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                      + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                      + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                      + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                      + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                      + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                      + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                      + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                      + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                      + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                      + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                      + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                      + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                      + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                      + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                      + laserCloud->points[i + 5].z;
        //曲率计算
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        //记录曲率点的索引
        cloudSortInd[i] = i;
        //初始时，点全未筛选过
        cloudNeighborPicked[i] = 0;
        //初始化为less flat点
        cloudLabel[i] = 0;

        //每个scan，只有第一个符合的点会进来，因为每个scan的点都在一起存放
        if (int(laserCloud->points[i].intensity) != scanCount) {
            scanCount = int(laserCloud->points[i].intensity);//控制每个scan只进入第一个点

            //曲率只取同一个scan计算出来的，跨scan计算的曲率非法，排除，也即排除每个scan的前后五个点
            if (scanCount > 0 && scanCount < N_SCANS) {
                scanStartInd[scanCount] = i + 5;
                scanEndInd[scanCount - 1] = i - 5;
            }
        }
    }
    //第一个scan曲率点有效点序从第5个开始，最后一个激光线结束点序size-5
    scanStartInd[0] = 5;
    scanEndInd.back() = cloudSize - 5;

    //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
    for (int i = 5; i < cloudSize - 6; i++) {//与后一个点差值，所以减6
        float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
        float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
        float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
        //计算有效曲率点与后一个点之间的距离平方和
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1) {//前提:两个点之间距离要大于0.1

            //点的深度
            float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                laserCloud->points[i].y * laserCloud->points[i].y +
                                laserCloud->points[i].z * laserCloud->points[i].z);

            //后一个点的深度
            float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                                laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                                laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

            //按照两点的深度的比例，将深度较大的点拉回后计算距离
            if (depth1 > depth2) {
                diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {//排除容易被斜面挡住的点
                    //该点及前面五个点（大致都在斜面上）全部置为筛选过
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            } else {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
        float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
        float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
        //与前一个点的距离平方和
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        //点深度的平方和
        float dis = laserCloud->points[i].x * laserCloud->points[i].x
                    + laserCloud->points[i].y * laserCloud->points[i].y
                    + laserCloud->points[i].z * laserCloud->points[i].z;

        //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
            cloudNeighborPicked[i] = 1;
        }
    }


    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    //将每条线上的点分入相应的类别：边沿点和平面点
    for (int i = 0; i < N_SCANS; i++) {
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        //将每个scan的曲率点分成6等份处理,确保周围都有点被选作特征点
        for (int j = 0; j < 6; j++) {
            //六等份起点：sp = scanStartInd + (scanEndInd - scanStartInd)*j/6
            int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
            //六等份终点：ep = scanStartInd - 1 + (scanEndInd - scanStartInd)*(j+1)/6
            int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;

            //按曲率从小到大冒泡排序
            for (int k = sp + 1; k <= ep; k++) {
                for (int l = k; l >= sp + 1; l--) {
                    //如果后面曲率点大于前面，则交换
                    if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
                        int temp = cloudSortInd[l - 1];
                        cloudSortInd[l - 1] = cloudSortInd[l];
                        cloudSortInd[l] = temp;
                    }
                }
            }

            //挑选每个分段的曲率很大和比较大的点
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSortInd[k];  //曲率最大点的点序

                //如果曲率大的点，曲率的确比较大，并且未被筛选过滤掉
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {//挑选曲率最大的前2个点放入sharp点集合
                        cloudLabel[ind] = 2;//2代表点曲率很大
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    } else if (largestPickedNum <= 20) {//挑选曲率最大的前20个点放入less sharp点集合
                        cloudLabel[ind] = 1;//1代表点曲率比较尖锐
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;//筛选标志置位

                    //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x
                                      - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                      - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                      - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                                      - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                      - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                      - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            //挑选每个分段的曲率很小比较小的点
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSortInd[k];

                //如果曲率的确比较小，并且未被筛选出
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1) {

                    cloudLabel[ind] = -1;//-1代表曲率很小的点
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {//只选最小的四个，剩下的Label==0,就都是曲率比较小的
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {//同样防止特征点聚集
                        float diffX = laserCloud->points[ind + l].x
                                      - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                      - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                      - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                                      - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                      - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                      - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            //将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别中
            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        //less flat点汇总
        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }

    cout << "full points nums:" << laserCloud->points.size() << endl;

    cout << "sharp points nums:" << cornerPointsSharp.points.size() << endl;

    //publich消除非匀速运动畸变后的所有的点
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = ros::Time::now();
    laserCloudOutMsg.header.frame_id = "map";
    pubLaserCloud.publish(laserCloudOutMsg);

    //publich消除非匀速运动畸变后的平面点和边沿点
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = ros::Time::now();
    cornerPointsSharpMsg.header.frame_id = "map";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = ros::Time::now();
    cornerPointsLessSharpMsg.header.frame_id = "map";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = ros::Time::now();
    surfPointsFlat2.header.frame_id = "map";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = ros::Time::now();
    surfPointsLessFlat2.header.frame_id = "map";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 2, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_2", 2);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);

    ros::spin();

    return 0;
}

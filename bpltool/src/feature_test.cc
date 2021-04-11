#include "include/lo_header.h"

//初始化过滤前几张点云
bool enoughFrameFlag = false;   //过滤掉前几帧数据
int frameNum = 0;               //统计来到的帧数量
//点云的角度
double startOrientation;    //第一个点角度
double endOrientation;      //最后一个点角度
double diffOrientation;     //第一个点到最后一个点角度差

//订阅话题
ros::Subscriber subLidarCloud;//订阅全局点云
//发布话题
ros::Publisher pubLidarCloudCorner;//发布角点
ros::Publisher pubLidarCloudSurf;//发布边缘点

//pcl这个点云指针需要这样指定一下指向类型，或者reset()一下
pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloudRaw(new pcl::PointCloud<pcl::PointXYZI>());//存储刚进来的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloudLineProcessed(new pcl::PointCloud<pcl::PointXYZI>());//存储分好线的点云
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarLine(16);//根据线号存储点云方便计算结算特征,vector嵌套要用指针，还要初始化
pcl::PointCloud<pcl::PointXYZI> lidarCloudCorner;//存储特征值比较大的点云
pcl::PointCloud<pcl::PointXYZI> lidarCloudSurf;//存储特征值比较小的点云
vector<vector<pair<int, double>>> cloudFeatureVal(16);//存储每一个线上的所有点的特征值
//清空所有点云
void clearAllPointCloud() {
    lidarCloudRaw->clear();
    lidarCloudLineProcessed->points.clear();
    for (size_t i = 0; i < lidarLine.size(); i++)
    {
        lidarLine[i]->points.clear();
    }
    lidarCloudCorner.points.clear();
    lidarCloudSurf.points.clear();
    cloudFeatureVal.clear();
    //pickOutPointIndex.clear();
}
//用于值对的排序，针对double排序，用在后面寻找最大小特征点的时候
bool strict_weak_ordering(const pair<int, double> a, const pair<int, double> b)
{
    return a.second < b.second;
}
//点云回调处理函数
void subLaserCloudHandle(const sensor_msgs::PointCloud2ConstPtr& lidarCloud)
{
    /*
     * 过滤掉前20帧点云
     */
    if(frameNum >= 20)
        enoughFrameFlag = true;
    if(enoughFrameFlag == false)
    {
        frameNum++;         //统计帧数
        return;
    }
    clearAllPointCloud();
    /*
     * 将ROS消息中的点拿出来并且去除无效点，避免出错
     */
    //转消息类型就是拿出来点的操作，转入的指针pcl会分配管理内存
    pcl::fromROSMsg(*lidarCloud, *lidarCloudRaw);//ros转pcl类型
    vector<int> cleanIndex;
    //点云中的点任一参数为NaN这个点都将去除，下面这种写法用不到index了
    pcl::removeNaNFromPointCloud(*lidarCloudRaw, *lidarCloudRaw, cleanIndex);//去除无效点

    /*
     * 首尾点的角度是全局的，每次回调更新一次
     * 寻找点云第一个点和最后一个点的角度
     * 雷达的坐标系是不会变的，这些点的角度都是建立在雷达坐标的基础上
     */
    startOrientation = -atan2(lidarCloudRaw->front().y, lidarCloudRaw->front().x);//第一个点在雷达下的角度
    endOrientation = -atan2(lidarCloudRaw->points.back().y, lidarCloudRaw->points.back().x) + 2 * PI;//最后一个点的角度
    if(endOrientation - startOrientation > 3 * PI)
        endOrientation -= 2 * PI;
    else if(endOrientation - startOrientation < M_PI)
        endOrientation += 2 * PI;
    diffOrientation = endOrientation - startOrientation;//计算角度差

    /*
     * 处理点云，分线存到点的intensity中去
     */
    for(pcl::PointXYZI tempPoint : lidarCloudRaw->points)
    {
        double angle = atan(tempPoint.z / sqrt(tempPoint.x * tempPoint.x + tempPoint.y * tempPoint.y)) * 180 / PI;
        int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5));
        int scanID;
        if(roundedAngle > 0)
            scanID = roundedAngle;
        else
            scanID = roundedAngle + 15;
        if(scanID > 15 || scanID < 0)
            continue;
        lidarLine[scanID]->push_back(tempPoint);//分线存进点云中去
        lidarCloudLineProcessed->points.push_back(tempPoint);
    }
    /*
     *针对分好线的点云，对每一线之间的点开始计算特征
     */
    //总共有16线，每一线之间进行计算特征
    for(size_t i = 0; i < 16; i++)
    {
        vector<pair<int, double>> thisLineFeatureVal;//存储本线的点的特征值
        thisLineFeatureVal.clear();
        pair<int, double> temppair;
        //统计本线内有多少点，弃用前后五个点
        int pointNum = lidarLine[i]->points.size();
        if(pointNum <= 100)
        {
            cloudFeatureVal.push_back(thisLineFeatureVal);
            continue;
        }
        for(size_t j = 5; j < pointNum - 5; j++)
        {
            double diffX =  lidarLine[i]->points[j-5].x + lidarLine[i]->points[j-4].x
                         +  lidarLine[i]->points[j-3].x + lidarLine[i]->points[j-2].x
                         +  lidarLine[i]->points[j-1].x - 10 * lidarLine[i]->points[j].x
                         +  lidarLine[i]->points[j+1].x + lidarLine[i]->points[j+2].x
                         +  lidarLine[i]->points[j+3].x + lidarLine[i]->points[j+4].x
                         +  lidarLine[i]->points[j+5].x;
            double diffY =  lidarLine[i]->points[j-5].y + lidarLine[i]->points[j-4].y
                         +  lidarLine[i]->points[j-3].y + lidarLine[i]->points[j-2].y
                         +  lidarLine[i]->points[j-1].y - 10 * lidarLine[i]->points[j].y
                         +  lidarLine[i]->points[j+1].y + lidarLine[i]->points[j+2].y
                         +  lidarLine[i]->points[j+3].y + lidarLine[i]->points[j+4].y
                         +  lidarLine[i]->points[j+5].y;
            double diffZ =  lidarLine[i]->points[i-5].z + lidarLine[i]->points[j-4].z
                         +  lidarLine[i]->points[j-3].z + lidarLine[i]->points[j-2].z
                         +  lidarLine[i]->points[j-1].z - 10 * lidarLine[i]->points[j].z
                         +  lidarLine[i]->points[j+1].z + lidarLine[i]->points[j+2].z
                         +  lidarLine[i]->points[j+3].z + lidarLine[i]->points[j+4].z
                         +  lidarLine[i]->points[j+5].z;
            //将特征值存储到本线的数组中
            temppair.first = j;
            temppair.second = diffX * diffX + diffY * diffY + diffZ * diffZ;
            thisLineFeatureVal.push_back(temppair);
        }
        //将所有线的点的特征存储起来
        cloudFeatureVal.push_back(thisLineFeatureVal);
    }
    //现在拿到了一个全部点云分好线的没啥用，lidarCloudLineProcessed
    //拿到了一个点云数组分好线存着的，lidarLine
    //拿到了一个二维数组存储点云特征值，cloudFeatureVal
    /*
     * 如果连个点之间角度很小但是距离太大，那么将这两个点以及周围的12个点全部标记上去除
     * 从第六个点开始到倒数第六个点,第一次比较是第六个点和第七个点，最后一次是倒数第六和倒数第七
     */
    for(size_t i = 0; i < lidarLine.size(); i++)
    {
        if(lidarLine[i]->points.size() < 100 || cloudFeatureVal[i].size() < 100)
            continue;
        for(size_t j = 5; j < lidarLine[i]->points.size() - 10; j++)
        {
            double diffX = lidarLine[i]->points[j].x - lidarLine[i]->points[j+1].x;
            double diffY = lidarLine[i]->points[j].y - lidarLine[i]->points[j+1].y;
            double diffZ = lidarLine[i]->points[j].z - lidarLine[i]->points[j+1].z;
            if(sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) > 0.1)
            {
                cloudFeatureVal[i][j-2].first = -1;
                cloudFeatureVal[i][j-1].first = -1;
                cloudFeatureVal[i][j].first = -1;
            }
        }
    }
    /*
     *针对每一线寻找其中的特征值最大的和最小的若干个点，存到corner和surf的点云中去
     *用了值对的排序
     */
    int lindeCount = 0;
    for(size_t i = 0; i < lidarLine.size(); i++)
    {
        int pointNum = lidarLine[i]->points.size();
        lindeCount += pointNum;
        int valNum = cloudFeatureVal[i].size();
        if(pointNum < 100 || valNum < 100)
            continue;
        //对pair类型的数组排序，针对double，升序
        sort(cloudFeatureVal[i].begin(), cloudFeatureVal[i].end(), strict_weak_ordering);

        if(valNum != cloudFeatureVal[i].size())
        {
            cout << "fashenshenmeshile " << endl;
        }
        //取排序好的中最小的10个点存到surf中去，还要防止这些点离得比较近
        for(int j = 0; j < 100; j++)
        {
            //将对应的点存到surf中去
            int tempPosi = cloudFeatureVal[i][j].first;
            if(tempPosi != -1)
                lidarCloudSurf.push_back(lidarLine[i]->points[tempPosi]);
            if(cloudFeatureVal[i][j].first > 5 && cloudFeatureVal[i][j].first < valNum-5)
            {
                cloudFeatureVal[i][cloudFeatureVal[i][j].first - 4].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first - 3].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first - 2].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first - 1].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first + 1].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first + 2].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first + 3].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][j].first + 4].first = -1;
            }
        }
        //取排序好的中最大的10个点存到surf中去，还要防止这些点离得比较近
        for(int j = 0; j < 100; j++)
        {
            //将对应的点存到corner中去
            int tempPosi = cloudFeatureVal[i][valNum-1-j].first;
            if(tempPosi != -1)
                lidarCloudCorner.push_back(lidarLine[i]->points[tempPosi]);
            if(cloudFeatureVal[i][valNum-1-j].first > 5 && cloudFeatureVal[i][valNum-1-j].first < valNum-5)
            {
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first - 4].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first - 3].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first - 2].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first - 1].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first + 1].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first + 2].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first + 3].first = -1;
                cloudFeatureVal[i][cloudFeatureVal[i][valNum-1-j].first + 4].first = -1;
            }
        }
    }
    /*
     *转换成ROS消息发布出去
     */
    sensor_msgs::PointCloud2 lidarCloudSurfPub;
    pcl::toROSMsg(lidarCloudSurf, lidarCloudSurfPub);
    lidarCloudSurfPub.header.frame_id = "map";
    lidarCloudSurfPub.header.stamp = lidarCloud->header.stamp;

    sensor_msgs::PointCloud2 lidarCloudCornerPub;
    pcl::toROSMsg(lidarCloudCorner, lidarCloudCornerPub);
    lidarCloudCornerPub.header.frame_id = "map";
    lidarCloudCornerPub.header.stamp = lidarCloud->header.stamp;

    pubLidarCloudCorner.publish(lidarCloudCornerPub);
    pubLidarCloudSurf.publish(lidarCloudSurfPub);

    //int totalCount = lidarCloudLineProcessed->points.size();
    //cout << "分好线的点云有" << lindeCount << endl;
    //cout << "总共点云有" << totalCount << endl;
    //int surfCount = lidarCloudSurf.points.size();
    //int cornerCount = lidarCloudCorner.points.size();
    //cout << "平面点有" << surfCount << endl;
    //cout << "角点有" << cornerCount << endl;
}


int main(int argc, char** argv)
{
    for(int i = 0; i < 16; i++)
    {
        lidarLine[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    ros::init(argc, argv, "loam_pick_feature");        //ros初始化
    ros::NodeHandle nh;                                         //ros句柄
    //订阅点云信息
    subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, subLaserCloudHandle);
    //发布点云信息
    pubLidarCloudCorner = nh.advertise<sensor_msgs::PointCloud2>("/corner", 1);
    pubLidarCloudSurf = nh.advertise<sensor_msgs::PointCloud2>("/surf", 1);
    //等待回调处理
    ros::spin();            //ctrl+c退出
    return 0;
}


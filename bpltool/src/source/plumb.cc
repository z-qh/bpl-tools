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
ros::Publisher pubPlumbCloud;

typedef pcl::PointXYZI PointType;

pcl::PointCloud<pcl::PointXYZ> extractPlumbFeature(const pcl::PointCloud<pcl::PointXYZ>& cloud);
void subPointCloudHandle(const sensor_msgs::PointCloud2& msg);
pcl::PointCloud<pcl::PointXYZ> cloudHandle(pcl::PointCloud<pcl::PointXYZ> cloud);

string inputTopic = "/pure_cloud";
string outputTopic = "/plumb_cloud";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plumn");
    ros::NodeHandle nh("~");
    nh.getParam("inputTopic", inputTopic);
    nh.getParam("outputTopic", outputTopic);

    subPointCloud = nh.subscribe(inputTopic, 1, subPointCloudHandle);
    pubPlumbCloud = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);

    ros::spin();

    return 0;

}
/*
 * 回调处理
 */
void subPointCloudHandle(const sensor_msgs::PointCloud2& msg)
{
    //接受去掉地面点的点云
    pcl::PointCloud<pcl::PointXYZ> inCloud;
    pcl::fromROSMsg(msg, inCloud);
    //extractPlumbFeature(inCloud);

    pcl::PointCloud<pcl::PointXYZ> tempCloud;

    //处理点云获取垂直特征
    tempCloud = cloudHandle(inCloud);
    //发布
    sensor_msgs::PointCloud2 tempMsgs;
    pcl::toROSMsg(tempCloud, tempMsgs);
    tempMsgs.header.stamp = ros::Time::now();
    tempMsgs.header.frame_id = "map";
    pubPlumbCloud.publish(tempMsgs);
}

/*
 * 提取其中的垂直特征
 */
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
        double verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180.0 / M_PI;
        int roundedAngle = int(verticalAngle + (verticalAngle<0.0?-0.5:+0.5));
        double scanID = 0;
        if (roundedAngle > 0){
            scanID = roundedAngle;
        }
        else {
            scanID = roundedAngle + (SCAN_NUMS - 1);
        }
        scans[scanID].push_back(thisPoint);
    }

    //按照顺序扫描
    for(int i = SCAN_NUMS-1; i >=0; i--)
    {
        if(scans[i].empty())
            continue;
        for(int j = 0; j < scans[i].size(); j++)
        {
            //找到第一个点
        }

    }

    return cloud;
}




//线扫检验完整性
//检验同一线扫是否按照顺序
/*
 * 验证通过，这种方式是可以上到下，从左到右扫描
 */
//    cout << "start angle:" << startOrien << " ";
//    for(int i = 0; i < SCAN_NUMS; i++)
//    {
//        if(scans[i].empty())
//            continue;
//        double max = atan2(scans[i].points.front().x, scans[i].points.front().y) * 180 / M_PI;
//        for(int j = 0; j < scans[i].size(); j+=(scans[i].size()/10))
//        {
//            pcl::PointXYZ thisPoint = scans[i].points[j];
//
//            double horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
//            cout << horizonAngle << endl;
//            if(horizonAngle > max)
//            {
//                max = horizonAngle;
//                cout << endl << "max change" << endl;
//            }
//        }
//    }
//    cout << "end angle:" << endOrien << endl;
//    for(int i = 0; i < SCAN_NUMS; i++)
//    {
//        if(scans[i].empty())
//            continue;
//        for(int j = 0; j < scans[i].size(); j++)
//        {
//        }
//    }

bool match(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    float diffX = p1.x - p2.x;
    float diffY = p1.y - p2.y;
    float diffZ = p1.z - p2.z;
    float diss = sqrt(pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2));
    float pitchAngle = atan2(diffZ, sqrt(pow(diffX, 2) + pow(diffY, 2))) * 180 / M_PI;
    if( diss < 0.5 && abs(pitchAngle) < 20)
        return true;
    else
        return false;
}

/*
 * 统计学方法提取信息，获取垂直特征
 */
pcl::PointCloud<pcl::PointXYZ> cloudHandle(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    //耗时统计
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //换个思路，按照水平角度来存储
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lines;
    map<int, int> linesIndex;
    int count = 0;
    for (int i = 0; i < cloud.size(); i++)
    {
        pcl::PointXYZ tempPoint = cloud.points[i];
        //角度
        float angle = atan2(tempPoint.y, tempPoint.x) * 180 / M_PI + 180;
        //角度的key，按照key存储，相当于临近水平角度的分在一起
        int angleIndex = round(angle);
        //查找已有的key
        map<int, int>::iterator it = linesIndex.find(angleIndex);
        //如果查到了，以值为索引存入数组
        if (it != linesIndex.end())
        {
            if (lines[it->second] == nullptr)
                lines[it->second].reset(new pcl::PointCloud<pcl::PointXYZ>());
            lines[it->second]->push_back(tempPoint);
        }
            //如果没查到说明有新的key，给赋新的值
        else
        {
            count++;
            lines.resize(count);
            lines[count - 1].reset(new pcl::PointCloud<pcl::PointXYZ>());
            linesIndex.insert(make_pair(angleIndex, count - 1));
        }
    }

    //忽略Z轴，计算到中心的距离，统计点之间到原点的方差，如果太大说明比较分散稀碎
    vector<int> pick(lines.size(), 0);
    vector<float> sigmaO(lines.size());//方差
    vector<float> meansO(lines.size());//均值
    vector<vector<int>> singlePick(lines.size());//单点选择数组
    for (int i = 0; i < lines.size(); i++)
    {
        //去掉的列这里还没给出
        if (lines[i]->empty())
            continue;
        float SO = 0;
        float SO2 = 0;
        //初始化
        singlePick[i].resize(lines[i]->size());
        //计算方差均值
        for (int j = 0; j < lines[i]->size(); j++)
        {
            float dis = sqrt(pow(lines[i]->points[j].x, 2) + pow(lines[i]->points[j].y, 2));
            SO += dis;
            SO2 += dis * dis;
        }
        meansO[i] = SO / lines[i]->size();
        sigmaO[i] = SO2 / lines[i]->size() - pow((SO / lines[i]->size()), 2);
        //数量很少或者方差很大就去掉，这个方差该怎么确定呢
        if (sigmaO[i] > 5 || lines[i]->size() < 30)
        {
            pick[i] = -1;
        }
        //cout << i << " S2:" << sigmaO[i] << " size:" << lines[i]->size() << endl;
    }

    //统计去除离散点之前的中心
    vector<pcl::PointXYZ> center(lines.size());
    for(int i = 0; i < lines.size(); i++)
    {
        //已经去掉的列和空列就不计算了
        if(lines[i]->empty() || pick[i] == -1)
            continue;
        float sumX = 0;
        float sumY = 0;
        float sumZ = 0;
        //中心均值
        for (int j = 0; j < lines[i]->size(); j++)
        {
            sumX += lines[i]->points[j].x;
            sumY += lines[i]->points[j].y;
            sumZ += lines[i]->points[j].z;
        }
        center[i].x = sumX / lines[i]->size();
        center[i].y = sumY / lines[i]->size();
        center[i].z = sumZ / lines[i]->size();
    }
    //统计点到Z中心的方差和均值，用于去除离散点
    vector<float> meansZ(lines.size());
    vector<float> sigmaZ(lines.size());
    for(int i = 0; i < lines.size(); i++)
    {
        //已经去掉的列和空列就不计算了
        if(lines[i]->empty() || pick[i] == -1)
            continue;
        float SC = 0;
        float SC2 = 0;
        //计算Z中心方差均值
        for (int j = 0; j < lines[i]->size(); j++)
        {
            float diffZ = abs(lines[i]->points[j].z - center[i].z);
            SC += diffZ;
            SC2 += diffZ * diffZ;
        }
        meansZ[i] = SC / lines[i]->size();
        sigmaZ[i] = SC2 / lines[i]->size() - pow((SC / lines[i]->size()), 2);

        //cout << i << " Y:" << sigmaO[i] << " size:" << lines[i]->size() << endl;
    }
    //去除相对原点的离散点，去除相对Z中心的离散点，并且纠正点云的中心点，
    for(int i = 0; i < lines.size(); i++)
    {
        //已经去掉的列和空列就不计算了
        if(lines[i]->empty() || pick[i] == -1)
            continue;
        float sumX = 0;
        float sumY = 0;
        float sumZ = 0;
        int outNums = 0;
        //纠正
        for (int j = 0; j < lines[i]->size(); j++)
        {
            float disO = sqrt(pow(lines[i]->points[j].x, 2) + pow(lines[i]->points[j].y, 2));
            float disZ = abs(lines[i]->points[j].z - center[i].z);
            //去除相对原点的离散点，去除相对Z中心的离散点
            if(disO < (meansO[i] - 2.0 * sqrt(sigmaO[i]))
               || disO > (meansO[i] + 2.0 * sqrt(sigmaO[i]))
               || disZ > (meansZ[i] + 1.0 * sqrt(sigmaZ[i]))
                    )
            {
                //cout << sigmaO[i] << endl;
                outNums++;
                singlePick[i][j] = -1;
                continue;
            }
            sumX += lines[i]->points[j].x;
            sumY += lines[i]->points[j].y;
            sumZ += lines[i]->points[j].z;
        }
        center[i].x = sumX / (lines[i]->size() - outNums);
        center[i].y = sumY / (lines[i]->size() - outNums);
        center[i].z = sumZ / (lines[i]->size() - outNums);
    }

    //遍历搜寻策略，利用中心点代表点云
    //vector<int> used(center.size(), 0);//使用过的矩阵
    vector<vector<int>> classIndex(1);//分类存储，类数未知
    int classNums = 0;
    //第一个点开头，单独做一类，并开始
    classIndex[classNums].push_back(0);
    for(int i = 0; i < center.size(); i++)
    {
        int j = (i + 1) % center.size();//后一个点
        //匹配此点到后面的点，匹配成功放到一起
        if(match(center[i], center[j]))
        {
            if(j != i+1)//回环
            {
                //回溯
                for(int k = 0; k < classIndex[classNums].size(); k++)
                    classIndex[classNums][k] = 0;
                //cout << "flash back, end!" << endl;
                break;
            }
            classIndex[classNums].push_back(j);
            //cout << "point: " << i << ", class: " << classNums << " add new，now: " << classIndex[classNums].size() << endl;
        }
            //不成功则另起一类，把后一个点放进去
        else
        {
            if(j != i+1)//回环
            {
                //cout << "no flash, end!" << endl;
                break;
            }
            classNums++;
            classIndex.resize(classNums+1);
            classIndex[classNums].push_back(j);
            //cout << "point: " << i  << ", new class, num: " << classNums << endl;
        }
    }

    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " plumb time cost: " << (time_used.count() * 1000) << " ms." << endl;

    pcl::PointCloud<pcl::PointXYZ> result;
    //存放中心点
    for(int i = 0; i < classIndex.size(); i++)
    {
        if(classIndex[i].size() <=2 )
            continue;
        for(int j = 0; j < classIndex[i].size(); j++)
        {
            pcl::PointXYZ temp;
            temp.x = center[classIndex[i][j]].x;
            temp.y = center[classIndex[i][j]].y;
            //temp.z = center[classIndex[i][j]].z;
            temp.z = 1.0;
            result.push_back(temp);
        }
    }
    //存放点云
    for(int i = 0; i < lines.size(); i++)
    {
        if(pick[i] == -1 || lines[i]->empty())
            continue;
        for( int j = 0; j < lines[i]->size(); j++)
        {
            pcl::PointXYZ temp;
            temp.x = lines[i]->points[j].x;
            temp.y = lines[i]->points[j].y;
            temp.z = lines[i]->points[j].z;
            result.push_back(temp);
        }
    }
    return result;

//    //按照calss上色对中心点染色
//    pcl::PointCloud<pcl::PointXYZRGB> Yes2;//显示辅助点云
//    string savefile = "/home/qh/Xpcd_test/bagPCD_1_save1.pcd";
//    string savefile2 = "/home/qh/Xpcd_test/bagPCD_1_save2.pcd";
//    pcl::PointCloud<pcl::PointXYZRGB> Yes;//上色辅助点云
//    int centerColor = 0;
//    for(int i = 0; i < classIndex.size(); i++)
//    {
//        centerColor = i % 2;
//        if(classIndex[i].size() <=2 )
//            continue;
//        for(int j = 0; j < classIndex[i].size(); j++)
//        {
//            pcl::PointXYZRGB temp;
//            temp.x = center[classIndex[i][j]].x;
//            temp.y = center[classIndex[i][j]].y;
//            //temp.z = center[classIndex[i][j]].z;
//            temp.z = 1.0;
//            temp.r = centerColor * 255;
//            temp.g = centerColor * 255;
//            temp.b = centerColor * 255;
//            Yes2.push_back(temp);
//        }
//    }
//    //看看分布
//    if(Yes2.empty())
//        return;
//    pcl::io::savePCDFile(savefile2, Yes2);
//    //cout << Yes2.size() << endl;
//
//    //上色看效果
//    int normalColor = 1;
//    for (int i = 0; i < lines.size(); i++)
//    {
//        normalColor = i % 2;
//        if (lines[i] == nullptr || lines[i]->empty())
//            continue;
//        //去除的列，数量太少或者方差过大
//        if(pick[i] == -1)
//            continue;
//        //染色
//        for (int j = 0; j < lines[i]->points.size(); j++)
//        {
//            pcl::PointXYZRGB temp;
//            temp.x = lines[i]->points[j].x;
//            temp.y = lines[i]->points[j].y;
//            temp.z = lines[i]->points[j].z;
//            //要去除的列染色
//            if (pick[i] == -1)
//            {
//                temp.r = 0;
//                temp.g = 255;
//                temp.b = 0;
//            }
//                //去除的离群点染色
//            else if(singlePick[i][j] == -1)
//            {
//                //continue;
//                temp.r = 255;
//                temp.g = 0;
//                temp.b = 0;
//            }
//                //正常列染色
//            else
//            {
//                //normalColor = 1;
//                temp.r = normalColor * 255;
//                temp.g = normalColor * 255;
//                temp.b = normalColor * 255;
//            }
//            Yes.push_back(temp);
//        }
//    }
//    if (Yes.empty())
//        return;
//    pcl::io::savePCDFile(savefile, Yes);
//    //cout << Yes.size() << endl;
}





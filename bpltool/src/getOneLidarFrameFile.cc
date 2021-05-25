#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <iostream>
#include <string>
#include <vector>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "pcl/io/io.h"
#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"


using namespace std;


int bagTopicFrameCount(string bagName, string topicName)
{
    //读取Bag文件
    rosbag::Bag bag;
    bag.open(bagName, rosbag::bagmode::Read);


    //设置需要访问的话题名字
    vector<string> topics;
    topics.push_back(topicName);

    //创建view，用于读取bag中的topic
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int nums = view.size();
    bag.close();
    return nums;
}

bool getOneFrameCloud(string bagName, string topicName, int frame,
                      pcl::PointCloud<pcl::PointXYZI>& inCloud)
{

    //读取Bag文件
    rosbag::Bag bag;
    bag.open(bagName, rosbag::bagmode::Read);

    //设置需要访问的话题名字
    vector<string> topics;
    topics.push_back(topicName);

    //创建view，用于读取bag中的topic
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    //迭代器的方式遍历，每个迭代器是一帧
    rosbag::View::iterator it = view.begin();

    for(int i = frame; i > 0; i--)
        it++;

    //取出对象，判断话题名是否正确
    auto m = *it;
    string nowTopic = m.getTopic();
    if(nowTopic == topicName)
    {
        sensor_msgs::PointCloud2::ConstPtr tempCloud = m.instantiate<sensor_msgs::PointCloud2>();
        if(tempCloud != nullptr)
        {
            sensor_msgs::PointCloud2 rosCloud;
            rosCloud = *tempCloud;
            pcl::fromROSMsg(rosCloud, inCloud);
        }
        else
        {
            cout << "the null message!" << endl;
        }
    }
    bag.close();
    return true;
}

void convertCloudToFile(string fileName,
                        pcl::PointCloud<pcl::PointXYZI>& inCloud)
{
    pcl::io::savePCDFileASCII(fileName, inCloud);
}
int main(int argc, char** argv)
{
    string fileName = "/home/qh/point.bag";
    string topicName = "/rslidar_points";
    string saveFileNameBase = "/home/qh/frame";

    cout << bagTopicFrameCount(fileName, topicName) << endl;

    pcl::PointCloud<pcl::PointXYZI> cloud;

    int frame = atoi(argv[1]);

    string saveFileName = saveFileNameBase + to_string(frame) + ".pcd";
    getOneFrameCloud(fileName, topicName, frame, cloud);
    cout << saveFileName << endl;
    convertCloudToFile(saveFileName, cloud);

    frame = atoi(argv[2]);
    saveFileName = saveFileNameBase + to_string(frame) + ".pcd";
    getOneFrameCloud(fileName, topicName, frame, cloud);
    cout << saveFileName << endl;
    convertCloudToFile(saveFileName, cloud);

    return 0;
}
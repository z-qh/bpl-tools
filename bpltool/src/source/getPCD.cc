/*
 * 获取点云pcd
 */
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "pcl/io/io.h"
#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"

using namespace std;

string mode;
string savePathHeader;
string inputTopic;
string bagPath;
string bagTopic;
int bagPosition;

int bagTopicFrameCount(string bagName, string topicName);
bool getOneFrameCloud(string bagName, string topicName, int frame,
                      pcl::PointCloud<pcl::PointXYZI>& inCloud);
void convertCloudToFile(string fileName,
                        pcl::PointCloud<pcl::PointXYZI>& inCloud);
class second
{
public:
    int get_flag;  //接收到键盘输入标志位
    int num;  //命名计数
    ros::NodeHandle nh;
    string  topic_name;
    string  outplace_name;
    second(ros::NodeHandle node_handle):get_flag(0),num(0)
    {
        nh = node_handle;
    }
    void subPCDTopic(ros::Subscriber* pcd)
    {
        int queue_size = 10;
        *pcd = nh.subscribe(topic_name, queue_size, &second::Callback, this);
    }
    void Callback(const sensor_msgs::PointCloud2ConstPtr& pcd_input)
    {
        if(get_flag)
        {
            get_flag=0;
            ++num;
            stringstream ss;
            ss << num;
            string num_c = ss.str();
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*pcd_input, cloud);
            try
            {
                pcl::io::savePCDFileASCII(outplace_name+"_"+ num_c +".pcd",cloud);
            }
            catch(const std::exception& e)
            {
                cerr << e.what() << endl;
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "getPCD");
    ros::NodeHandle nh("~");
    nh.getParam("mode", mode);

    transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    //在线模式-龙逆提供
    nh.getParam("savePathHeader", savePathHeader);
    if(mode.find("online") != -1)
    {
        nh.getParam("inputTopic", inputTopic);
        second s1(nh);
        string saveFileName = savePathHeader;
        s1.topic_name = inputTopic;
        s1.outplace_name = saveFileName;
        ros::Subscriber pcd;
        s1.subPCDTopic(&pcd);
        while (true)
        {
            char kb = getchar();
            if(kb=='q')
                break;
            else
                s1.get_flag = 1;
            ros::spinOnce();
        }
    }
    //离线模式
    else if(mode.find("outline") != -1)
    {
        nh.getParam("bagPath", bagPath);
        nh.getParam("bagTopic", bagTopic);
        nh.getParam("bagPosition", bagPosition);
        //统计bag信息并输出
        cout << bagTopicFrameCount(bagPath, bagTopic) << endl;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        int frame = bagPosition;
        //保存位置
        string saveFileName = savePathHeader + to_string(frame) + ".pcd";
        getOneFrameCloud(bagPath, bagTopic, frame, cloud);
        convertCloudToFile(saveFileName, cloud);
    }
    else
    {
        cout << "invaild param!" << endl;
        return 0;
    }

    return 0;
}

/*
 * 获取bag里帧数信息
 */
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

/*
 * 从bag里面指定话题名指定帧数读取pcd
 */
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

/*
 * 保存PCD文件
 */
void convertCloudToFile(string fileName,
                        pcl::PointCloud<pcl::PointXYZI>& inCloud)
{
    pcl::io::savePCDFileASCII(fileName, inCloud);
}







#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "iostream"
#include "string"

using namespace std;

string sourceBagPath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_handle");
    ros::NodeHandle nh("~");
    nh.getParam("sourceBagPath", sourceBagPath);

    rosbag::Bag bag;
    bag.open(sourceBagPath, rosbag::bagmode::Read);
    vector<string> topics{"/rslidar_points"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it;


    int frameCunt = 0;
    //耗时统计
    chrono::steady_clock::time_point t1;

    for(it = view.begin(); it != view.end(); it++)
    {
        string nowTopic = (*it).getTopic();
        if(nowTopic == topics[0])
        {
            
            
        }
    }

    //耗时统计
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << " time cost: " << (time_used.count() * 1000) << " ms." << endl;
    cout << " handle "<< frameCunt << " frames" << endl;
    cout << " pre frame uesd" << (time_used.count() * 1000.0 / frameCunt) << "ms" << endl;
    bag.close();
    return 0;
}


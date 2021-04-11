#include "include/lo_header.h"
#include "ros/ros.h"

ros::Subscriber subCloud;//订阅
ros::Publisher pubCloud;//发布


using namespace std;


void subCloudHandle(const sensor_msgs::PointCloud2& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ClusterNode");
    ros::NodeHandle nh;

    subCloud = nh.subscribe("/rslidar", 1, subCloudHandle);

    ros::Rate loop(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "pcl/filters/filter.h"
#include "iterator"

#include "unordered_map"

#include "algorithm"

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint32_t, label, label)
)

using namespace std;

bool newCloud = false;
pcl::PointCloud<PointXYZIL>::Ptr originCloud(new pcl::PointCloud<PointXYZIL>() );
pcl::PointCloud<pcl::PointXYZRGB>::Ptr showCloud(new pcl::PointCloud<pcl::PointXYZRGB>() );

class rgb{
public:
    int r,g,b;
    rgb(int r_, int g_, int b_):r(r_),g(g_),b(b_){}
    rgb():r(0),g(0),b(0){}
};

const unordered_map<int, rgb> labelToColor = {
        {0, rgb(0,0,0)},              //0-unlabeled-0
        {1, rgb(0,0,255)},            //1-outliers-1
        {10, rgb(245, 150, 100)},     //2-car-10
        {11, rgb(245, 230, 100)},     //3-bicycle-11
        {13, rgb(250, 80, 100)},      //4-bus-13
        {15, rgb(150, 60, 30)},       //5-motorcycle-15
        {16, rgb(255, 0, 0)},         //6-on-rails-16
        {18, rgb(180, 30, 80)},       //7-trunk-18
        {20, rgb(255, 0, 0)},         //8-other-vehicle-20
        {30, rgb(30, 30, 255)},       //9-person-30
        {31, rgb(200, 40, 255)},      //10-bicyclist-31
        {32, rgb(90, 30, 150)},       //11-motorcyclist-32
        {40, rgb(255, 0, 255)},       //12-road-40
        {44, rgb(255, 150, 255)},     //13-parking-44
        {48, rgb(75, 0, 75)},         //14-sidewalk-48
        {49, rgb(75, 0, 175)},        //15-other-ground-49
        {50, rgb(0, 200, 255)},       //16-building-50
        {51, rgb(50, 120, 255)},      //17-fence-51
        {52, rgb(0, 150, 255)},        //18-structure-52
        {71, rgb(0, 60, 135)},      //19-trunk-71
        {80, rgb(150, 240, 255)}        //20-pole-80
};

void subSematicCloudHandle(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    originCloud->clear();
    pcl::fromROSMsg(*msg, *originCloud);
    newCloud = true;
}

bool com(pair<int, int>A, pair<int, int>B)
{
    return A.second > B.second;
}

void handleSematicAndShow()
{
    showCloud->clear();
    unordered_map<int, vector<int>> info;
    int cloudSize = originCloud->points.size();
    for(int i = 0; i < cloudSize; i++)
    {
        auto it = info.find(originCloud->points[i].label);
        if(it != info.end())
            it->second.push_back(i);
        else
            info.insert(make_pair(originCloud->points[i].label, vector<int>()));
    }
    cout << "there are " << info.size() << " kinds of cluster!" << endl;
    vector<pair<int, int>> infoVec;
    for(auto it = info.begin(); it != info.end(); it++)
    {
        infoVec.push_back(make_pair(it->first, it->second.size()));
    }
    sort(infoVec.begin(), infoVec.end(), com);
    for(int i = 0; i < infoVec.size(); i++)
    {
        cout << "lable : " << infoVec[i].first << " has " << infoVec[i].second << " points!" << endl;
    }
    for(int i = 0; i < cloudSize; i++)
    {
        pcl::PointXYZRGB tempPoint;
        tempPoint.x = originCloud->points[i].x;
        tempPoint.y = originCloud->points[i].y;
        tempPoint.z = originCloud->points[i].z;
        auto it = labelToColor.find(originCloud->points[i].label);
        if(it != labelToColor.end())
        {
            tempPoint.r = it->second.r;
            tempPoint.g = it->second.g;
            tempPoint.b = it->second.b;
        }
        else
        {
            tempPoint.r = 0;
            tempPoint.g = 0;
            tempPoint.b = 0;
        }
        showCloud->push_back(tempPoint);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visSematic");
    ros::NodeHandle nh;

    originCloud->clear();
    showCloud->clear();

    ros::Subscriber subSematicCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 1, subSematicCloudHandle);
    ros::Publisher pubSematicCloud = nh.advertise<sensor_msgs::PointCloud2>("/sematic_cloud", 1);
    ros::Rate loop(10);
    while(ros::ok())
    {
        if(newCloud)
        {
            newCloud = false;
            handleSematicAndShow();
            sensor_msgs::PointCloud2 tempMsg;
            pcl::toROSMsg(*showCloud, tempMsg);
            tempMsg.header.stamp = ros::Time::now();
            tempMsg.header.frame_id = "map";
            pubSematicCloud.publish(tempMsg);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

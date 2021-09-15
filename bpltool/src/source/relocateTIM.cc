#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "nav_msgs/Odometry.h"
#include "gnss_driver/gps_navi_msg.h"
#include "sensor_msgs/PointCloud2.h"

#include "vector"
#include "string"

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "std_msgs/String.h"
#include <map>

//////////////////////////////////////////////
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    double index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time) (double, index, index)
)

typedef PointXYZIRPYT  PointTypePose;
//////////////////////////////////////////////////
typedef pcl::PointXYZI PointType;

using namespace std;

bool readCloudFromFile(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string filePath)
{
    ifstream file;
    file.open(filePath, ios::in);
    if(!file.good()){
        cout << "file open filed!" << endl;
        return false;
    }
    double temp;
    int pointsNum;
}

int main(int argc, char** argv)
{

    return 0;
}
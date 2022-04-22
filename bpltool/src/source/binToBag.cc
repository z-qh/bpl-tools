#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"
#include "gnss_driver/gps_navi_msg.h"
#include "sensor_msgs/Imu.h"

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

typedef PointXYZIL PointSemantic;

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

std::vector<uint> read_label_data(const std::string label_data_path)
{
    std::ifstream label_data_file(label_data_path, std::ifstream::in | std::ifstream::binary);
    label_data_file.seekg(0, std::ios::end);
    const size_t num_elements = label_data_file.tellg() / sizeof(uint);
    label_data_file.seekg(0, std::ios::beg);

    std::vector<uint> label_data_buffer(num_elements);
    label_data_file.read(reinterpret_cast<char*>(&label_data_buffer[0]), num_elements*sizeof(uint));
    return label_data_buffer;
}

string base = "00";

void get_frame_from_file(pcl::PointCloud<PointXYZIL>& cloud, string path, int count)
{
    std::stringstream lidar_data_path;
    lidar_data_path << path << "sequences/" + base + "/velodyne/"
                    << std::setfill('0') << std::setw(6) << count << ".bin";
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    std::stringstream label_data_path;
    label_data_path << path << "sequences/" + base + "/labels/"
                    << std::setfill('0') << std::setw(6) << count << ".label";
    std:vector<uint> label_data = read_label_data(label_data_path.str());
    cloud.clear();
    for (std::size_t i = 0; i < lidar_data.size() / 4; ++i)
    {
        PointXYZIL point;
        point.x = lidar_data[i * 4];
        point.y = lidar_data[i * 4 + 1];
        point.z = lidar_data[i * 4 + 2];
        float rate = 1.0;
        // float r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        // if (r > 35.0)  rate = r * r / 25.0 * 25.0;
        point.intensity = lidar_data[i * 4 + 3] * rate;
        point.label = label_data[i] & 0xffff;
        cloud.push_back(point);
    }
}


string bagPath = "/home/qh/bin2bag/gnss.bag";
string binPath = "/home/qh/bin2bag/";
string outPath = "/home/qh/bin2bag/test.bag";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bagToBin");
    ros::NodeHandle nh;

    nh.getParam("bagPath", bagPath);
    nh.getParam("binPath", binPath);
    nh.getParam("outPath", outPath);
    nh.getParam("base", base);

    vector<string> topics;
    topics.push_back("/gps_navi");
    topics.push_back("/imu/data");
    topics.push_back("/lslidar_point_cloud");

    rosbag::Bag sourceBag;
    sourceBag.open(bagPath, rosbag::bagmode::Read);
    if(!sourceBag.isOpen()){
        cout << "source bag not exists!" << endl;
        return 0;
    }
    rosbag::View sourceView(sourceBag, rosbag::TopicQuery(topics));
    auto sourceIt = sourceView.begin();

    rosbag::Bag targetBag;
    targetBag.open(outPath, rosbag::bagmode::Write);
    if(!targetBag.isOpen()){
        cout << "target bag path not exists!" << endl;
        return 0;
    }
    //rosbag::View targetView(targetBag, rosbag::TopicQuery(topics));
    //auto targetIt = targetView.begin();
    int cloudCount = 0;
    for(sourceIt = sourceView.begin(); sourceIt != sourceView.end(); sourceIt++) {
        string now_topic = sourceIt->getTopic();
        if(now_topic == topics[0])//GPS
        {
            gnss_driver::gps_navi_msg tempGps;
            tempGps = *(sourceIt->instantiate<gnss_driver::gps_navi_msg>());
            targetBag.write(topics[0], tempGps.header.stamp, tempGps);
        }
        if(now_topic == topics[1])//IMU
        {
            sensor_msgs::Imu tempIMU;
            tempIMU = *(sourceIt->instantiate<sensor_msgs::Imu>());
            targetBag.write(topics[1], tempIMU.header.stamp, tempIMU);
        }
        if(now_topic == topics[2])//PointCloud
        {
            sensor_msgs::PointCloud2 tempMsg;
            tempMsg = *(sourceIt->instantiate<sensor_msgs::PointCloud2>());

            pcl::PointCloud<PointXYZIL> tempCloud;
            get_frame_from_file(tempCloud, binPath, cloudCount);
            cloudCount++;
            sensor_msgs::PointCloud2 outMsg;
            pcl::toROSMsg(tempCloud, outMsg);
            outMsg.header = tempMsg.header;
            targetBag.write(topics[2], outMsg.header.stamp, outMsg);
            cout << "pointCloud :" << setw(6) << setfill('0') << cloudCount-1;
            cout << " time :" << setprecision(12) << outMsg.header.stamp.toSec() << endl;
        }
    }

    sourceBag.close();
    targetBag.close();
}
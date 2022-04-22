#include "iostream"
#include "fstream"

#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

using namespace std;

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



void PointCloud2LabelAndBin(pcl::PointCloud<PointXYZIL>&points, int count){
    static string saveBase = "/home/qh/YES/dlut/Daquan/";
    stringstream ssBin, ssLabel;
    ssBin << saveBase << "/bin/" << setw(6) << setfill('0') << count << ".bin";
    ssLabel << saveBase << "/labels/" << setw(6) << setfill('0') << count << ".label";
    cout << "SAVE BIN   " << ssBin.str() << " SIZE: " << points.size() << endl;
    cout << "SAVE LABEL " << ssLabel.str() << endl;
    ofstream binFile, labelFile;
    binFile.open(ssBin.str(), ios::binary);
    labelFile.open(ssLabel.str(), ios::binary);
    for(auto&p:points){
        binFile.write((char*)(&p.x), 3 * sizeof(float));
        binFile.write((char*)(&p.intensity), 1 * sizeof(float));
        labelFile.write((char*)(&p.label), 1*sizeof(int32_t));
    }
    labelFile.close();
    binFile.close();
}

void Daquan2BinLabel(){
    rosbag::Bag bag;
    vector<double> timestampeds;
    bag.open("/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag", rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back("/lslidar_point_cloud");
    rosbag::View bagView(bag, rosbag::TopicQuery(topics));
    int count = 0;
    for(auto it = bagView.begin(); it != bagView.end(); it++){
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 nowMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            timestampeds.push_back(nowMsg.header.stamp.toSec());
//            pcl::PointCloud<PointXYZIL> nowCloud;
//            pcl::fromROSMsg(nowMsg, nowCloud);
//            PointCloud2LabelAndBin(nowCloud, count++);
        }
    }
    ofstream stampFile("/home/qh/YES/dlut/Daquan/timestamp");
    for(auto&p:timestampeds){
        stampFile << fixed << setprecision(8) << p << endl;
    }
    stampFile.close();
}
int main(){
    Daquan2BinLabel();
}

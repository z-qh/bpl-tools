/*
 * 用于建筑物点云中的平面，可视化，残差设计与调整等
 */

#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "yaml-cpp/yaml.h"


using namespace std;

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint32_t, label, label)
)
typedef PointXYZIL PointType;

class TicToc{
public:
    TicToc(){tic();}
    void tic(){start = std::chrono::system_clock::now();}
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

///////////////////////////////////////////////////////////
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;
ros::Publisher pub_pcl_cluster, pub_pcl_interest;
image_transport::Publisher imgPub;
///////////////////////////////////////////////////////////
std::vector<bool> getInterest(std::string file_name);
pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel);
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec);
void process(const sensor_msgs::PointCloud2ConstPtr &rec);

int main(int argc, char** argv){
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    pub_pcl_cluster = nh.advertise<pcl::PointCloud<PointType>>("/plane/cluster_points", 1);
    pub_pcl_interest = nh.advertise<pcl::PointCloud<PointType>>("/plane/interest_points", 1);
    image_transport::ImageTransport it(nh);
    imgPub = it.advertise("/plane/img", 10);
    ros::Rate loop(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if (laserCloudMsgQueue.empty())
            continue;
        process(laserCloudMsgQueue.front());
        laserCloudMsgQueue.pop();
        loop.sleep();
    }
}

void line_extrac(pcl::PointCloud<PointType>::Ptr interest_points){

    
}

void plane_extrac(pcl::PointCloud<PointType>::Ptr interest_points){
    cv::Mat rangeMat;
    cv::Mat indMat;
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    // 先是kitt所以是64x1800
    rangeMat = cv::Mat(64, 1800, CV_32F, cv::Scalar::all(0));
    indMat = cv::Mat(64, 1800, CV_32S, cv::Scalar::all(0));
    // 投影 
    cloudSize = interest_points->points.size();
    for (size_t i = 0; i < cloudSize; ++i){
        PointType thisPoint;
        thisPoint.x = interest_points->points[i].x;
        thisPoint.y = interest_points->points[i].y;
        thisPoint.z = interest_points->points[i].z;
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        int rowIdn = (verticalAngle + 25.0) / (28.0/64.0);
        if (rowIdn < 0 || rowIdn >= 60)
            continue;
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        columnIdn = -round((horizonAngle-90.0)/(360.0/18000.0)) + 1800/2;
        if (columnIdn >= 1800)
            columnIdn -= 1800;
        if (columnIdn < 0 || columnIdn >= 1800)
            continue;
        rangeMat.at<float>(rowIdn, columnIdn) = range;
        indMat.at<int>(rowIdn, columnIdn) = i;
    }
    int label = 255;
    cv::Mat labelMat;
    int count = 0;
    labelMat = cv::Mat(64, 1800, CV_8U, cv::Scalar::all(0));
    for (size_t i = 0; i < 64; ++i){
        for (size_t j = 0; j < 1800; ++j){
            float dist = rangeMat.at<float>(i, j);
            int label = labelMat.at<int>(i, j);
            if(dist == 0 || label != 0) continue;
            int ind = indMat.at<int>(i, j);
            count++;
            vector<pair<int,int>> cap;
            static vector<pair<int,int>> dir{{-1,0},{0,-1},{+1,0},{0,+1}};
            // cap.emplace_back(i, j);
            // while(!cap.empty()){
            //     vector<pair<int, int>> tmp_cap;
            //     for(auto&ind:cap){
            //         labelMat.at<int>(ind.first, ind.second) = label;
            //         for(auto&d:dir){
            //             int si = ind.first+d.first;
            //             int sj = ind.second+d.second;
            //             if(si < 0 && sj < 0 && si >= 32 && sj >= 1800) continue;
            //             if(labelMat.at<int>(si, sj) != 0) continue;
            //             if(rangeMat.at<float>(si, sj) == 0) continue;
            //             tmp_cap.emplace_back(si, sj);
            //         }
            //     }
            //     cap.swap(tmp_cap);
            // }
            label--;
        }
    }
    std::cout << " img count " << count << std::endl;
    // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", indMat).toImageMsg();
    // imgPub.publish(img_msg);
    cv::imshow("yes", rangeMat);
    cv::waitKey(5);
}

void process(const sensor_msgs::PointCloud2ConstPtr &rec){
    TicToc time_rg;
    double timestamp = rec->header.stamp.toSec();
    pcl::PointCloud<PointType>::Ptr rec_point(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*rec, *rec_point);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*rec_point, *rec_point, indices);
    if(rec_point->points.empty()) return;
    static auto interest_labels = getInterest("/home/qh/kitti_data_interest.yaml");
    auto points_interest_serial = GetFilteredInterestSerial(rec_point, interest_labels);
    if( points_interest_serial!=nullptr && points_interest_serial->points.empty()) return;
    // 发布语义兴趣点
    pcl::PointCloud<PointType>::Ptr points_interest(new pcl::PointCloud<PointType>());
    for(auto&p:points_interest_serial->points) points_interest->push_back(p);
    points_interest->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_interest->header.frame_id = "map";
    pub_pcl_interest.publish(points_interest);
    printf("get interest %f ms\n", time_rg.toc());
    // 
    plane_extrac(points_interest_serial);

}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec)
{
    laserCloudMsgQueue.push(rec);
    while (laserCloudMsgQueue.size() > 10)
        laserCloudMsgQueue.pop();
}

std::vector<bool> getInterest(std::string file_name)
{
    std::vector<int> ind;
    std::vector<bool> interest;
    std::vector<int> labels;
    std::vector<std::string> label_vals;
    YAML::Node config = YAML::LoadFile(file_name);
    int max_labels = 0;
    for (YAML::const_iterator it = config["labels"].begin(); it != config["labels"].end(); ++it)
    {
        int now_label = it->first.as<int>();
        std::string now_label_val = it->second.as<std::string>();
        labels.push_back(now_label);
        label_vals.push_back(now_label_val);
        if (now_label > max_labels && now_label_val != "")
        {
            max_labels = now_label;
        }
        printf("%d : %s\n", now_label, now_label_val.c_str());
    }
    std::stringstream ss(config["interest"].as<std::string>());
    std::string info;
    while (getline(ss, info, ','))
    {
        ind.push_back(std::stoi(info));
    }
    std::sort(ind.begin(), ind.end());
    interest.resize(max_labels + 1, false);
    printf("Total %d labels. Max LabelNum: %d. Thest are interested:\n", static_cast<int>(labels.size()), max_labels);
    for (auto &p : ind)
    {
        interest[p] = true;
        printf("%d:true\n", p);
    }
    return interest;
}

pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel)
{
    int valid_labels = 0;
    vector<int> interestLabelId(interestLabel.size(), 0);
    for (int i = 0; i < interestLabel.size(); ++i)
    {
        if (interestLabel[i])
        {
            interestLabelId[i] = valid_labels;
            valid_labels++;
        }
    }
    if (raw_pcl_->points.size() > 0)
    {
        // step1:getInterestedLabels
        pcl::PointCloud<PointType>::Ptr interested;
        interested.reset(new pcl::PointCloud<PointType>());
        for (auto p : raw_pcl_->points)
        {
            if (interestLabel[p.label & 0xFFFF])
            {
                interested->points.push_back(p);
            }
            else
            {
            }
        }
        return interested;
    }
    else
    {
        return nullptr;
    }
}

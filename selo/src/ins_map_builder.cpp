#define PCL_NO_PRECOMPILE

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <memory>
#include <time.h>
#include <queue>

#include "dbscan/DBSCAN_paper.h"
#include "dbscan/DBSCAN_kdtree.h"
#include "dbscan/DBSCAN_flann.h"

#include "odom/tic_toc.h"
#include "yaml-cpp/yaml.h"

#include "odom/common.h"

#include "map/instance.h"
#include "map/ins_map.h"

double timestamp = 0;

std::vector<bool> interest_labels;

const std::vector<Eigen::Vector3d> color_tab{
    {1, 1, 1}, // white
    {1, 0, 0}, // red
    {0, 0, 1}, // blue
    {0, 1, 0}, // green
    {1, 1, 0}, // yellow
    {0, 1, 1} // sky
};

std::queue<nav_msgs::Odometry> laserOdomQueue;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;

std::string odomTopic = "/gt";
Eigen::Matrix4f gt2lidar = Eigen::Matrix4f::Identity();

ros::Publisher map_marker;
ros::Publisher marker_pub_box;
ros::Publisher map_cloud;
ros::Publisher pub_pcl_cluster, pub_pcl_cliped;


InsMapTest mapper;

void pubPoints(InstancesPtr &inss, ros::Publisher &pubCloud)
{
    pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
    for (auto&p:inss)
    {
        *tmpCloud += *(p->cloud);
    }
    tmpCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    tmpCloud->header.frame_id = "map";
    pubCloud.publish(tmpCloud);
}

void ClearAllMarker(ros::Publisher &marker_pub_box_)
{
    visualization_msgs::MarkerArray::Ptr clear_marker_array(new visualization_msgs::MarkerArray);
    visualization_msgs::Marker dummy_marker;
    dummy_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker_array->markers.push_back(dummy_marker);
    marker_pub_box_.publish(clear_marker_array);
}

void pushLine(InstancePtr ins1, InstancePtr ins2, int &marker_id, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_strip;
    line_strip.pose.orientation.w = 1.0;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time().now();
    line_strip.ns = "match";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.points.resize(3);
    geometry_msgs::Point p1, p2;
    p1.x = ins1->center[0];
    p1.y = ins1->center[1];
    p1.z = (ins1->max_height + ins1->min_height) / 2;
    p2.x = ins2->center[0];
    p2.y = ins2->center[1];
    p2.z = (ins2->max_height + ins2->min_height) / 2;
    line_strip.id = marker_id;
    marker_id++;
    line_strip.points[0] = p1;
    line_strip.points[1] = p2;
    line_strip.points[2] = p1;
    marker_array_box.markers.push_back(line_strip);
}

void pushBBox(InstancePtr ins, int &marker_id, int color, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_strip;
    line_strip.pose.orientation.w = 1.0;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time().now();
    line_strip.ns = "match";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.05;
    line_strip.color.r = color_tab[color](0);
    line_strip.color.g = color_tab[color](1);
    line_strip.color.b = color_tab[color](2);
    line_strip.color.a = 1.0;
    line_strip.points.resize(5);
    {
        geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
        p1.x = p5.x = ins->vertex1[0];
        p1.y = p5.y = ins->vertex1[1];
        p1.z = ins->min_height;
        p5.z = ins->max_height;
        p2.x = p6.x = ins->vertex2[0];
        p2.y = p6.y = ins->vertex2[1];
        p2.z = ins->min_height;
        p6.z = ins->max_height;
        p3.x = p7.x = ins->vertex3[0];
        p3.y = p7.y = ins->vertex3[1];
        p3.z = ins->min_height;
        p7.z = ins->max_height;
        p4.x = p8.x = ins->vertex4[0];
        p4.y = p8.y = ins->vertex4[1];
        p4.z = ins->min_height;
        p8.z = ins->max_height;
        line_strip.id = marker_id;
        line_strip.points[0] = p1;
        line_strip.points[1] = p2;
        line_strip.points[2] = p4;
        line_strip.points[3] = p3;
        line_strip.points[4] = p1;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p5;
        line_strip.points[1] = p6;
        line_strip.points[2] = p8;
        line_strip.points[3] = p7;
        line_strip.points[4] = p5;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p1;
        line_strip.points[1] = p5;
        line_strip.points[2] = p7;
        line_strip.points[3] = p3;
        line_strip.points[4] = p1;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p2;
        line_strip.points[1] = p6;
        line_strip.points[2] = p8;
        line_strip.points[3] = p4;
        line_strip.points[4] = p2;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
    }
}

void pubBBoxMarker(const std::vector<std::shared_ptr<Instance>> &instances, ros::Publisher marker_pub_box_, int color = 0)
{
    if (instances.size() > 0)
    {
        ClearAllMarker(marker_pub_box_);
        visualization_msgs::MarkerArray marker_array_box;
        int marker_id = 0;
        int instance_num = instances.size();
        for (int i = 0; i < instance_num; i++)
        {
            pushBBox(instances[i], marker_id, color, marker_array_box);
        }
        marker_pub_box_.publish(marker_array_box);
    }
}

pcl::PointCloud<PointType>::Ptr GetFilteredInterest(pcl::PointCloud<PointType>::Ptr raw_pcl_, std::vector<bool> &interestLabel)
{
    if (raw_pcl_->points.size() > 0)
    {
        // step1:getInterestedLabels
        pcl::PointCloud<PointType>::Ptr interested(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
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

void getDBSCANCluster(pcl::PointCloud<PointType>::Ptr in, std::vector<pcl::PointIndices> &cluster_indices)
{
    cluster_indices.clear();
    int point_number = in->points.size();
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.09, 0.09, 0.09);
    pcl::PointCloud<PointType>::Ptr downSizePoints(new pcl::PointCloud<PointType>());
    downSizePoints->clear();
    //当点云数目较多时，进行降采样算法
    if (false && point_number > 25000)
    {
        downSizeFilter.setInputCloud(in);
        downSizeFilter.filter(*downSizePoints);
    }
    else
    {
        *downSizePoints += *in;
    }
    int core_point_min_pts_param = 15; //判断不为噪声点的阈值
    double clusterTolerance = 0.1;     // dbscan聚类搜索半径
    int min_cluster_size_param = 20;   //聚类后点的数目最小值
    bool isDynamic = false;            //阈值是否动态调整
    pcl::KdTreeFLANN<PointType>::Ptr tree(new pcl::KdTreeFLANN<PointType>);
    tree->setInputCloud(downSizePoints);
    DBSCANPaperCluster<PointType> ec;
    ec.setCorePointMinPts(core_point_min_pts_param);
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(min_cluster_size_param);
    ec.setHorizonAngleResolution(0.18);
    ec.setVerticalAngleResolution(1);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downSizePoints);
    ec.setS(1.5);
    ec.setDynamic(isDynamic);
    ec.extract(cluster_indices); //聚类后所有的点的索引值存放在数组中
}

void getEulerCluster(pcl::PointCloud<PointType>::Ptr in, std::vector<pcl::PointIndices> &clusters)
{
    clusters.clear();
    int point_number = in->points.size();
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.09, 0.09, 0.09);
    pcl::PointCloud<PointType>::Ptr downSizePoints(new pcl::PointCloud<PointType>());
    downSizePoints->clear();
    //当点云数目较多时，进行降采样算法
    if (false && point_number > 25000)
    {
        downSizeFilter.setInputCloud(in);
        downSizeFilter.filter(*downSizePoints);
    }
    else
    {
        *downSizePoints += *in;
    }
    double clusterTolerance = 0.2; //欧式聚类搜索距离
    int minClusterSize = 30;       //最小聚类数量
    int maxClusterSize = 20000;    //最大聚类数量
    pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType>>(new pcl::search::KdTree<PointType>);
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downSizePoints);
    ec.extract(clusters);
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec)
{
    laserCloudMsgQueue.push(rec);
    while (laserCloudMsgQueue.size() > 10)
        laserCloudMsgQueue.pop();
}

void laserOdomHandler(nav_msgs::Odometry odom)
{
    if (odomTopic == "/gt")
    {
        Eigen::Matrix4f transN = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf qN(odom.pose.pose.orientation.w,
                              odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z);
        Eigen::Vector3f tN(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        transN.block<3, 3>(0, 0) = Eigen::Matrix3f(qN);
        transN.block<3, 1>(0, 3) = tN;
        transN = gt2lidar.transpose() * transN * gt2lidar;
        odom.pose.pose.orientation.w = Eigen::Quaternionf(transN.block<3,3>(0,0)).w();
        odom.pose.pose.orientation.x = Eigen::Quaternionf(transN.block<3,3>(0,0)).x();
        odom.pose.pose.orientation.y = Eigen::Quaternionf(transN.block<3,3>(0,0)).y();
        odom.pose.pose.orientation.z = Eigen::Quaternionf(transN.block<3,3>(0,0)).z();
        odom.pose.pose.position.x = transN(0, 3);
        odom.pose.pose.position.y = transN(1, 3);
        odom.pose.pose.position.z = transN(2, 3);
    }
    laserOdomQueue.push(odom);
    while (laserOdomQueue.size() > 10)
        laserOdomQueue.pop();
}

void BuildInsMap(const sensor_msgs::PointCloud2ConstPtr &rec, nav_msgs::Odometry odom)
{
    pcl::PointCloud<PointType>::Ptr rec_point(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*rec, *rec_point);
    timestamp = rec->header.stamp.toSec();
    // 车辆坐标系向 世界坐标系转换的 转换矩阵
    std::shared_ptr<Eigen::Matrix4d> transN = std::make_shared<Eigen::Matrix4d>();
    Eigen::Quaterniond qN(odom.pose.pose.orientation.w,
                            odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y,
                            odom.pose.pose.orientation.z);
    Eigen::Vector3d tN(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    transN->block<3, 3>(0, 0) = Eigen::Matrix3d(qN);
    transN->block<3, 1>(0, 3) = tN;

    if (rec_point->points.size() > 0)
    {
        // remove ground
        TicToc time_rg;
        pcl::PointCloud<PointType>::Ptr points_rm = GetFilteredInterest(rec_point, interest_labels);
        printf("remove ground %f ms\n", time_rg.toc());
        points_rm->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        points_rm->header.frame_id = "map";
        // pub_pcl_cliped.publish(points_rm);

        // cluster
        TicToc time_cluste;
        std::vector<pcl::PointIndices> clusters;
        getDBSCANCluster(points_rm, clusters);
        // getEulerCluster(points_rm, clusters);
        int counter = clusters.size();
        float cluster_id = 1;
        std::vector<pcl::PointCloud<PointType>::Ptr> points_vector;
        if (counter > 0)
        {
            points_vector.clear();
            for (int i = 0; i < counter; i++)
            {
                pcl::PointCloud<PointType>::Ptr temp_cluster(new pcl::PointCloud<PointType>);
                pcl::copyPointCloud(*points_rm, clusters[i], *temp_cluster);
                int points_size = temp_cluster->size();
                for (auto &p : temp_cluster->points)
                {
                    p.intensity = cluster_id;
                }
                cluster_id++;
                points_vector.push_back(temp_cluster);
            }
        }
        printf("cluster time %fms\n", time_cluste.toc());
        pcl::PointCloud<PointType>::Ptr pcl_cluster_ptr(new pcl::PointCloud<PointType>);
        for (int i = 0; i < counter; i++)
        {
            *pcl_cluster_ptr += *points_vector[i];
        }
        pcl_cluster_ptr->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        pcl_cluster_ptr->header.frame_id = "map";
        // pub_pcl_cluster.publish(pcl_cluster_ptr);

        // min_box
        TicToc box_time;
        std::vector<std::shared_ptr<Instance>> instances;
        for (int i = 0; i < counter; i++)
        {
            std::shared_ptr<Instance> out_obj(new Instance);
            out_obj->cloud = points_vector[i];
            instances.push_back(out_obj);
        }
        MinBoxInstanceBuilder instance_builder;
        InstanceBuilderOptions instance_builder_options;
        if (!instance_builder.Build(instance_builder_options, &instances))
        {
            return;
        }
        // printf("minbox time %fms\n", box_time.toc());

        // static int saveDirId = 0;
        // std::stringstream ss;
        // ss << std::fixed << saveDirId++;
        // std::string saveDir = "/home/qh/temp/"+ss.str()+"/";
        // if( 0 == access( saveDir.c_str(), 0)) SaveInstances(instances, saveDir);

        // tracker
        TicToc track_time;
        static int tracker_seq_num = 0;
        
        if (instances.size() > 0)
        {
            tracker_seq_num++;
            InsMapTestOptions map_option;
            map_option.velodyne_trans = *transN;
            mapper.Merge(instances, timestamp, map_option);

            auto allMap = mapper.GetGlobalMap();

            pubBBoxMarker(instances, marker_pub_box, 1);
            pubPoints(instances, pub_pcl_cliped);

            pubBBoxMarker(allMap, map_marker, 0);
            pubPoints(allMap, map_cloud);
        }
        printf("tracker time %fms\n", track_time.toc());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mot");
    ros::NodeHandle nh;

    interest_labels = getInterest("/home/qh/kitti_data_interest.yaml");

    timestamp = 0;
    gt2lidar << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    ros::Subscriber subLaserOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, laserOdomHandler);
    
    marker_pub_box  = nh.advertise<visualization_msgs::MarkerArray>("/selo/marker_box", 1);
    map_marker      = nh.advertise<visualization_msgs::MarkerArray>("/selo/map_marker", 1);
    map_cloud       = nh.advertise<pcl::PointCloud<PointType>>("/selo/map_points", 1);

    pub_pcl_cluster = nh.advertise<pcl::PointCloud<PointType>>("/selo/cluster_points", 1);
    pub_pcl_cliped = nh.advertise<pcl::PointCloud<PointType>>("/selo/interest_points", 1);

    ros::Rate loop(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
            continue;
        while (abs(laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec()) > 0.05)
        {
            if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
                break;
            if (laserOdomQueue.front().header.stamp.toSec() < laserCloudMsgQueue.front()->header.stamp.toSec())
                laserOdomQueue.pop();
            else
                laserCloudMsgQueue.pop();
        }
        if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
            continue;
        if (abs(laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec()) < 0.05)
        {
            BuildInsMap(laserCloudMsgQueue.front(), laserOdomQueue.front());
            laserCloudMsgQueue.pop();
            laserOdomQueue.pop();
        }
        loop.sleep();
    }
    std::string save_final_path = "/home/qh/temp/ins_map";
    auto final_instances = mapper.GetGlobalMap();
    SaveInstances(final_instances, save_final_path);
    return 0;
}
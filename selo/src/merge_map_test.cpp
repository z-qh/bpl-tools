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

using namespace std;

std::vector<bool> interest_labels;

const std::vector<Eigen::Vector3d> color_tab{
    {1, 1, 1}, // white
    {1, 0, 0}, // red
    {0, 0, 1}, // blue
    {0, 1, 0}, // green
    {1, 1, 0}, // yellow
    {0, 1, 1} // sky
};

ros::Publisher after_merge_map_marker;
ros::Publisher before_merge_map_marker;
ros::Publisher before_merge_map_cloud_puber;
ros::Publisher after_merge_map_cloud_puber;

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


bool MergeGlobalMapTest(InstancesPtr &inss);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mot");
    ros::NodeHandle nh;


    before_merge_map_marker = nh.advertise<visualization_msgs::MarkerArray>("/before_map_marker", 1);
    after_merge_map_marker  = nh.advertise<visualization_msgs::MarkerArray>("/after_map_marker", 1);

    before_merge_map_cloud_puber = nh.advertise<pcl::PointCloud<PointType>>("/before_map_points", 1);
    after_merge_map_cloud_puber  = nh.advertise<pcl::PointCloud<PointType>>("/after_map_points", 1);

    InstancesPtr load_inss;
    string load_str = "/home/qh/ins_map_temp/ins_map";
    LoadInstanaces(load_inss, load_str);

    

    // deep copy
    InstancesPtr merged_inss;
    for(auto&p:load_inss){
        InstancePtr tmp(new Instance());
        tmp->clone(*p);
        merged_inss.push_back(tmp);
    }

    MergeGlobalMapTest(merged_inss);

    ros::Rate loop(0.25);
    while (ros::ok())
    {   
        pubBBoxMarker(load_inss, before_merge_map_marker, 0);
        pubPoints(load_inss, before_merge_map_cloud_puber);

        pubBBoxMarker(merged_inss, after_merge_map_marker, 3);
        pubPoints(merged_inss, after_merge_map_cloud_puber);
        loop.sleep();
    }
    string save_path = "/home/qh/ins_map_temp/ins_map_merge";
    SaveInstances(merged_inss, save_path);

    return 0;
}

bool MergeGlobalMapTest(InstancesPtr &inss)
{
    int ini_size = inss.size();
    TicToc merge_time;
    InstancesPtr inss_;
    std::vector<bool> beenmerged_ind(inss.size(), false);
    pcl::PointCloud<pcl::PointXY>::Ptr instances_vertex_map(new pcl::PointCloud<pcl::PointXY>());
    pcl::KdTreeFLANN<pcl::PointXY> instances_vertex_map_kdtree;
    MinBoxInstanceBuilder instance_builder;
    InstanceBuilderOptions instance_builder_options;
    // 合并所有有交集的实例 阈值 0.5 用于合并大范围实例 边缘实例等 global中无重复
    if (inss.empty())
        return false;
    instances_vertex_map->clear();
    InstancesPtr instances_map;
    for (auto &p : inss)
    {
        pcl::PointXY tmpPoint1, tmpPoint2, tmpPoint3, tmpPoint4;
        tmpPoint1.x = p->vertex1(0);
        tmpPoint1.y = p->vertex1(1);
        tmpPoint2.x = p->vertex2(0);
        tmpPoint2.y = p->vertex2(1);
        tmpPoint3.x = p->vertex3(0);
        tmpPoint3.y = p->vertex3(1);
        tmpPoint4.x = p->vertex4(0);
        tmpPoint4.y = p->vertex4(1);
        instances_vertex_map->push_back(tmpPoint1);
        instances_vertex_map->push_back(tmpPoint2);
        instances_vertex_map->push_back(tmpPoint3);
        instances_vertex_map->push_back(tmpPoint4);
    }
    instances_vertex_map_kdtree.setInputCloud(instances_vertex_map);
    // 将当前Instance 归并到地图 global中
    for (int i = 0; i < inss.size(); ++i)
    {
        auto& p = inss[i];
        // 寻找半径内相交的点
        double search_radius = GetRectMaxRadius(p);
        pcl::PointXY nowPoint;
        nowPoint.x = p->center(0);
        nowPoint.y = p->center(1);
        std::vector<int> indexs;
        std::vector<float> distance2s;
        instances_vertex_map_kdtree.radiusSearch(nowPoint, search_radius, indexs, distance2s);
        // 获取当前instance 在 global 中能找到的碰撞的 instance
        std::set<int> now_friends;
        for (auto &e : indexs)
        {   
            std::cout << e << " ";
            int ind = e / 4;
            // 同一指针地址跳过计算
            if (isSame(inss[ind], p))
                continue;
            // 被吸收过了不参与计算
            if (beenmerged_ind[ind])
                continue;
            now_friends.insert(ind);
        }
        std::cout << std::endl;
        for (auto &e : now_friends)
        {
            if(beenmerged_ind[e]) continue;
            // 计算当前instance 和相交的 global instance 的重复区域
            auto n_p = Get2InssCollisionVolumePercent(p, inss[e]);
            if (n_p.second > 0.3)
            {
                std::cout << "map merge " << inss[e]->id << " in " << p->id << std::endl;
                mergeSource2Map(inss[e], p);
                // 更新吸收此Instance 的分布
                instance_builder.BuildInstance(instance_builder_options, p);
                beenmerged_ind[e] = true;
            }
        }
    }
    for(int i = 0; i < beenmerged_ind.size(); ++i){
        if( !beenmerged_ind[i] ) inss_.push_back(inss[i]);
    }
    printf("use time %f ms\n", merge_time.toc());
    inss.clear();
    inss = inss_;
    printf(" %d merge to %d\n", ini_size, inss_.size());
    return true;
}

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>



#include "odom/tic_toc.h"
#include "odom/common.h"

#include "map/instance.h"

typedef std::shared_ptr<Instance> InstancePtr;
typedef std::vector<std::shared_ptr<Instance>> InstancesPtr;

using namespace std;

const std::vector<Eigen::Vector3f> color_tab{
    {1, 1, 1}, // white
    {1, 0, 0}, // red
    {0, 0, 1}, // blue
    {0, 1, 0}, // green
    {1, 1, 0}, // yellow
    {0, 1, 1} // sky
};


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
    int line_points_size = ins->cloud_2dshape->size()+1;
    line_strip.points.resize(line_points_size);
    for(int i = 0; i < line_points_size; ++i)
    {   
        geometry_msgs::Point p;
        if(i == line_points_size-1)
        {
            p.x = ins->cloud_2dshape->points[0].x;
            p.y = ins->cloud_2dshape->points[0].y;
            p.z = ins->max_height;
            line_strip.points[i] = p;
            break;
        }
        p.x = ins->cloud_2dshape->points[i].x;
        p.y = ins->cloud_2dshape->points[i].y;
        p.z = ins->max_height;
        line_strip.points[i] = p;
    }
    line_strip.id = marker_id;
    marker_array_box.markers.push_back(line_strip);
    marker_id++;
    for(int i = 0; i < line_points_size; ++i)
    {   
        geometry_msgs::Point p;
        if(i == line_points_size-1)
        {
            p.x = ins->cloud_2dshape->points[0].x;
            p.y = ins->cloud_2dshape->points[0].y;
            p.z = ins->min_height;
            line_strip.points[i] = p;
            break;
        }
        p.x = ins->cloud_2dshape->points[i].x;
        p.y = ins->cloud_2dshape->points[i].y;
        p.z = ins->min_height;
        line_strip.points[i] = p;
    }
    line_strip.id = marker_id;
    marker_array_box.markers.push_back(line_strip);
    marker_id++;
    for(int i = 0; i < line_points_size-1; ++i)
    {   
        line_strip.points.resize(2);
        geometry_msgs::Point p1, p2;
        p1.x = p2.x = ins->cloud_2dshape->points[i].x;
        p1.y = p2.y = ins->cloud_2dshape->points[i].y;
        p1.z = ins->min_height;
        p2.z = ins->max_height;
        line_strip.points[0] = p1;
        line_strip.points[1] = p2;
        line_strip.id = marker_id;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
    }
}


pcl::PointCloud<PointType>::Ptr createLineCLoud(PointType A, PointType B, int id)
{
    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    result->clear();
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    double nums = distance * 100;
    for(int i = 0; i < nums; i++)
    {
        PointType tempPoint;
        tempPoint.x = B.x + diffX / nums * i;
        tempPoint.y = B.y + diffY / nums * i;
        tempPoint.z = B.z + diffZ / nums * i;
        tempPoint.intensity = id;
        result->push_back(tempPoint);
    }
    return result;
}

void pushBBoxCloud(InstancePtr ins, int marker_id, pcl::PointCloud<PointType>::Ptr cloud)
{
    visualization_msgs::Marker line_strip;
    int line_points_size = ins->cloud_2dshape->size();
    for(int i = 0; i < line_points_size; ++i)
    {   
        PointType p1, p2;
        p1 = ins->cloud_2dshape->points[i];
        p2 = ins->cloud_2dshape->points[(i+1)%line_points_size];
        p1.z = ins->max_height;
        p2.z = ins->max_height;
        auto this_line_cloud_up = createLineCLoud(p1, p2, marker_id);
        p1.z = ins->min_height;
        p2.z = ins->min_height;
        auto this_line_cloud_down = createLineCLoud(p1, p2, marker_id);
        *cloud += *this_line_cloud_up;
        *cloud += *this_line_cloud_down;
    }
    for(int i = 0; i < line_points_size; ++i)
    {   
        PointType p1, p2;
        p1 = ins->cloud_2dshape->points[i];
        p2 = ins->cloud_2dshape->points[i];
        p1.z = ins->max_height;
        p2.z = ins->min_height;
        auto this_vertical_line_cloud = createLineCLoud(p1, p2, marker_id);
        *cloud += *this_vertical_line_cloud;
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

pcl::PointCloud<PointType>::Ptr converBBox2Cloud(const std::vector<std::shared_ptr<Instance>> &instances)
{
    pcl::PointCloud<PointType>::Ptr bbox(new pcl::PointCloud<PointType>());
    if (instances.size() > 0)
    {
        visualization_msgs::MarkerArray marker_array_box;
        int marker_id = 0;
        int instance_num = instances.size();
        for (int i = 0; i < instance_num; i++)
        {
            pushBBoxCloud(instances[i], instances[i]->id , bbox);
        }
    }
    return bbox;
}

ros::Publisher ins_pub_map_cloud, ins_pub_map_shape_cloud;
ros::Publisher ins_pub_map_shape;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_ins_map");
    ros::NodeHandle nh;

    std::string prefix = std::string(argv[1]);

    ins_pub_map_shape  = nh.advertise<visualization_msgs::MarkerArray>("/ins_map/ins_shape", 1);
    ins_pub_map_shape_cloud = nh.advertise<pcl::PointCloud<PointType>>("/ins_map/ins_shape_cloud", 1); 
    ins_pub_map_cloud      = nh.advertise<pcl::PointCloud<PointType>>("/ins_map/ins_cloud", 1);
    
    InstancesPtr  instances;
    string path = "/home/qh/ins_map_temp/" + prefix;
    LoadInstanaces(instances, path);
    pcl::PointCloud<PointType>::Ptr shape_cloud(new pcl::PointCloud<PointType>());

    auto all_box_cloud = converBBox2Cloud(instances);

    if(!all_box_cloud->empty())
    {
        pcl::io::savePCDFileASCII("/home/qh/ins_map_temp/"+prefix+"_BBox.pcd", *all_box_cloud);
    }

    pcl::PointCloud<PointType>::Ptr origin_cloud(new pcl::PointCloud<PointType>());
    for (auto&p:instances)
    {
        *origin_cloud += *(p->cloud);
    }

    if(!origin_cloud->empty())
    {
        pcl::io::savePCDFileASCII("/home/qh/ins_map_temp/"+prefix+"_cloud.pcd", *origin_cloud);
    }

    ros::Rate loop(0.2);

    while(ros::ok())
    {
        pubBBoxMarker(instances, ins_pub_map_shape, 1);
        all_box_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        all_box_cloud->header.frame_id = "map";
        ins_pub_map_shape_cloud.publish(*all_box_cloud);
        pubPoints(instances, ins_pub_map_cloud);
        loop.sleep();
    }
    return 0;
}
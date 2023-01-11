#include "Scancontext.hpp"

#include "ros/ros.h"
#include "iostream"
#include "sstream"
#include "fstream"
#include "vector"
#include "string"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>

#include "nav_msgs/Odometry.h"

using namespace std;

std::vector<nav_msgs::Odometry> getOdomFromFile(string file){
    ifstream insFile(file);
    string tmp_data;
    vector<nav_msgs::Odometry> res_data;
    int data_count = 0;
    while(getline(insFile, tmp_data, '\n')){
        data_count++;
        if (data_count == 1) continue;
        stringstream ss(tmp_data);
        vector<string> sig_data;
        string tmp;
        while(getline(ss, tmp, ',')){
            sig_data.emplace_back(tmp);
        }
        if(sig_data.size() != 15){
            printf("Data Error!\n");
            exit(0);
        }
        double time = stod(sig_data[0]);
        double x = stod(sig_data[5]), y = stod(sig_data[6]), z = stod(sig_data[7]);
        double roll = stod(sig_data[12]), pitch = stod(sig_data[13]), yaw = stod(sig_data[14]);
        static double first_x, first_y, first_z; 
        if(data_count == 2){
            first_x = x;
            first_y = y;
            first_z = z;
        }
        
        Eigen::Matrix3d ori(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()) );
        Eigen::Quaterniond tmpQ(ori);
        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = x - first_x;
        odom.pose.pose.position.y = y - first_y;
        odom.pose.pose.position.z = z - first_z;
        odom.pose.pose.orientation.w = tmpQ.w();
        odom.pose.pose.orientation.x = tmpQ.x();
        odom.pose.pose.orientation.y = tmpQ.y();
        odom.pose.pose.orientation.z = tmpQ.z();
        res_data.emplace_back(odom);
    }
    printf("Load %d Datas!\n", int(res_data.size()));
    return res_data;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadBin(string& ssbin){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if( 0 != access(ssbin.c_str(), 0) ) return nullptr;
    ifstream binfile;
    binfile.open(ssbin, ios::in | ios::binary);
    // Check the file correction
    binfile.seekg(0, ios::end);
    int pointNums = binfile.tellg()/sizeof(float)/4;
    binfile.seekg(0, ios::beg);
    // load
    cloud->resize(pointNums);
    for (int j = 0; j < pointNums; j++) {
        pcl::PointXYZI point;
        binfile.read((char *) &point.x, 4*sizeof(float));
        point.x = point.x;
        point.y = point.y;
        point.z = point.z;
        cloud->points[j].x = point.x;
        cloud->points[j].y = point.y;
        cloud->points[j].z = point.z;
    }
    binfile.close();
    return cloud;
}

void handle(){
    vector<pair<string, Eigen::Matrix4d>> pair_data;
    ifstream file("/home/qh/YES/oxford/o1/pair.txt");
    string tmp_line;
    static double fx, fy, fz;
    while(getline(file, tmp_line, '\n')){
        stringstream ss(tmp_line);
        string bin_file;
        Eigen::Matrix4d t_pose = Eigen::Matrix4d::Identity();
        ss >> bin_file
           >> t_pose(0, 0) >> t_pose(0, 1) >> t_pose(0, 2) >> t_pose(0, 3)
           >> t_pose(1, 0) >> t_pose(1, 1) >> t_pose(1, 2) >> t_pose(1, 3)
           >> t_pose(2, 0) >> t_pose(2, 1) >> t_pose(2, 2) >> t_pose(2, 3);
        if(pair_data.empty()){
            printf("%f %f %f\n", t_pose(0, 3), t_pose(1, 3), t_pose(2, 3));
            fx=t_pose(0, 3), fy=t_pose(1, 3), fz=t_pose(2, 3);
        }
        pair_data.emplace_back(bin_file, t_pose);
        pair_data.back().second(0, 3) -= fx;
        pair_data.back().second(1, 3) -= fy;
        pair_data.back().second(2, 3) -= fz;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> down_size;
    down_size.setLeafSize(0.5, 0.5, 0.5);
    for(auto&per:pair_data){
        auto file = per.first;
        auto pose = per.second;
        auto pc = LoadBin(file);
        printf("%f %f %f\n", pose(0, 3), pose(1, 3), pose(2, 3));
        printf("Load %s: %d points\n", file.c_str(), pc->size());
        pcl::transformPointCloud(*pc, *pc, pose);
        *total_pc += *pc;
        down_size.setInputCloud(total_pc);
        down_size.filter(*total_pc);
    }
    if (!total_pc->empty())
        pcl::io::savePCDFile("/home/qh/123.pcd", *total_pc);

}

int main(int argc, char **argv){
    handle();
    exit(0);
    
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    
    ros::Publisher pubOdom = nh.advertise<nav_msgs::Odometry>("/oxfrod_odom", 1);
    auto odoms = getOdomFromFile("/home/qh/YES/oxford/o1/gps/ins.csv");
    
    ros::Rate loop(10);
    
    int pub_ind = 0;
    while(ros::ok()){
        pub_ind += 100;
        pub_ind = pub_ind % odoms.size();
        pubOdom.publish(odoms[pub_ind]);
        printf("Pub ing... \n");
        loop.sleep();
    }
    return 0;
}
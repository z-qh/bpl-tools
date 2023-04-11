
#include "../include/cloud_projection/feature.h"
#include "../include/io/file_reader.h"
#include "../include/odometry/odometry.h"
#include "../include/semantic_icp/semantic_icp.h"
#include "../include/semantic_icp/semantic_point_cloud.h"
#include "../include/utility.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

void test_semantic_matcher(int argc, char** argv) {
    typedef semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> SPC;
    typedef semanticicp::SemanticIterativeClosestPoint<pcl::PointXYZ, uint32_t> SICP;

    ros::init(argc, argv, "slo_test");
    ros::NodeHandle nh("~");

    ros::Publisher OdomPublisher = nh.advertise<nav_msgs::Odometry>("/sicp_odom", 1);
    ros::Publisher OriCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("/ori_cloud", 1);
    ros::Publisher FeatureCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("/feature_cloud", 1);

    // 创建filereader并调试
    FileReader FR("/home/qh/kitti/00/velodyne", "/home/qh/kitti/00/labels", FileReader::BIN_WITH_INTENSITY);
    FR.Debug_setPercent(0.0);
    // 创建FeatureExtractor并调试
    FeatureExtracter FE;
    // 创建semantic icp并调试
    auto begin = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();

    // first frame and target
    SPC::Ptr OriFrame(FileReader::toS(FR.get()));
    SPC::Ptr NextFrame;
    SPC::Ptr tmpFinal;
    Eigen::Isometry3d global_pose;
    global_pose.setIdentity();
    int count = 0;
    while (FR.good() && ros::ok()) {
        begin = std::chrono::steady_clock::now();
        auto ori_cloud = FR.get();
        ori_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        ori_cloud->header.frame_id = "map";
        OriCloudPublisher.publish(ori_cloud);
        auto feature_cloud = FE.get(ori_cloud);
        feature_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        feature_cloud->header.frame_id = "map";
        FeatureCloudPublisher.publish(feature_cloud);
        tmpFinal.reset(new SPC());
        NextFrame = SPC::Ptr(FileReader::toS(feature_cloud));
        SICP sicp;
        sicp.setInputSource(NextFrame);
        sicp.setInputTarget(OriFrame);
        sicp.align(tmpFinal);
        OriFrame = SPC::Ptr(FileReader::toS(ori_cloud));
        Eigen::Isometry3d tmpTrans(sicp.getFinalTransFormation().matrix());
        global_pose = global_pose * tmpTrans;
        ;
        end = std::chrono::steady_clock::now();
        printf("\033[33m Time:%5.2fs.\033[0m\n", chrono::duration<double>(end - begin).count());
        Eigen::Quaterniond tmpQ(global_pose.rotation());
        Eigen::Vector3d tmpt(global_pose.translation());
        nav_msgs::Odometry pubOdom;
        pubOdom.header.frame_id = "map";
        pubOdom.header.stamp = ros::Time::now();
        pubOdom.pose.pose.position.x = tmpt(0);
        pubOdom.pose.pose.position.y = tmpt(1);
        pubOdom.pose.pose.position.z = tmpt(2);
        pubOdom.pose.pose.orientation.w = tmpQ.w();
        pubOdom.pose.pose.orientation.x = tmpQ.x();
        pubOdom.pose.pose.orientation.y = tmpQ.y();
        pubOdom.pose.pose.orientation.z = tmpQ.z();
        OdomPublisher.publish(pubOdom);
    }
    printf("\033[32m Done.\033[0m\n");
}


void getaaaa(Odometry&test_odom, string&save_time_file){
    int total_opt_count=0;
    double total_opt_time=0;
    int frames = 0;
    total_opt_count = 0;
    total_opt_time = 0;
    frames = 0;
    while (test_odom.good()) {
        int n_opt_count = 0;
        double n_opt_time = 0;
        // test_odom.step(n_opt_count, n_opt_time, true);
        total_opt_count += n_opt_count;
        total_opt_time += n_opt_time;
        frames++;
        cout << "count: " << n_opt_count << endl;
        cout << "time : " << n_opt_time << " ms "<< endl;
        cout << "id   : " << frames << endl;
        continue;
    }
    if (save_time_file != "") {
        auto odom_result = test_odom.getTraj();
        std::ofstream file(save_time_file);
        file << "count : " << total_opt_count << endl;
        file << "time  : " << total_opt_time << endl;
        file << "frames: " << frames << endl;
        file.close();
    }
}

void test_time_opt(){

    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/161time.txt";
        test_odom.control(3000, 6600);
        cout << "start 161" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/162time.txt";
        test_odom.control(9200, 11500);
        cout << "start 162" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/163time.txt";
        test_odom.control(12500, 16000);
        cout << "start 163" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/164time.txt";
        test_odom.control(17000, 20200);
        cout << "start 164" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/165time.txt";
        test_odom.control(24900, 26500);
        cout << "start 165" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan17/bin", "/home/qh/YES/dlut/Daquan17/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/171time.txt";
        test_odom.control(1200, 8500);
        cout << "start 171" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan17/bin", "/home/qh/YES/dlut/Daquan17/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/172time.txt";
        test_odom.control(8500, 21000);
        cout << "start 172" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/dlut/Daquan19/bin", "/home/qh/YES/dlut/Daquan19/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/191time.txt";
        test_odom.control(17000, 17000);
        cout << "start 191" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/jgxy/jgxy1/bin", "/home/qh/YES/jgxy/jgxy1/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/j1time.txt";
        cout << "start j1" << endl;
        getaaaa(test_odom, save_time_file);
    }
    {
        Odometry test_odom("/home/qh/YES/jgxy/jgxy2/bin", "/home/qh/YES/jgxy/jgxy2/label_com", FileReader::BIN_WITH_INTENSITY, "");
        string save_time_file = "/home/qh/j2time.txt";
        cout << "start j2" << endl;
        getaaaa(test_odom, save_time_file);
    }

}

void test_semantic_odometry(int argc, char** argv) {

    ros::init(argc, argv, "slo_test");
    ros::NodeHandle nh("~");

    ros::Publisher OdomPublisher = nh.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher OriCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("sensor_cloud", 1);
    ros::Publisher FeatureCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("feature_cloud", 1);
    ros::Publisher MapCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("map_cloud", 1);
    ros::Publisher PathPublisher = nh.advertise<nav_msgs::Path>("path", 1);
    ros::Publisher LoopPublisher = nh.advertise<visualization_msgs::MarkerArray>("loop", 1);
    ros::Publisher LoopOldPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("loop/old", 1);
    ros::Publisher LoopCurPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>("loop/cur", 1);

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;

    aftMappedTrans.frame_id_ = "map";
    aftMappedTrans.child_frame_id_ = "aft_mapped";

    // Odometry test_odom("/home/qh/kitti/00/velodyne", "/home/qh/kitti/00/labels", FileReader::BIN_WITH_INTENSITY, "");
    // string save_poses_file = "";
    // Odometry test_odom("/home/qh/YES/dlut/Daquan16/bin", "/home/qh/YES/dlut/Daquan16/label_com", FileReader::BIN_WITH_INTENSITY, "");
    // string save_poses_file = "/home/qh/YES/dlut/Daquan16/odom/myresult";
    // seq161
    // test_odom.control(3000, 6600);
    // seq162
    // test_odom.control(9200, 11500);
    // seq163
    // test_odom.control(12500, 16000);
    // seq164
    // test_odom.control(17000, 20200);
    // seq165
    // test_odom.control(24900, 26500);


    // Odometry test_odom("/home/qh/YES/dlut/Daquan17/bin", 
    // "/home/qh/YES/dlut/Daquan17/label_com", FileReader::BIN_WITH_INTENSITY, "");
    // string save_poses_file = "/home/qh/YES/dlut/Daquan17/odom/myresult";
    // seq171
    // test_odom.control(1200, 8500);
    // seq172
    // test_odom.control(8500, 21000);
    // test_odom.control(10000, 15500);


    // Odometry test_odom("/home/qh/YES/dlut/Daquan19/bin", 
    // "/home/qh/YES/dlut/Daquan19/label_com", FileReader::BIN_WITH_INTENSITY, "");
    // string save_poses_file = "/home/qh/YES/dlut/Daquan19/odom/myresult";
    // // seq191
    // test_odom.control(17000, 20500);


    // Odometry test_odom("/home/qh/YES/jgxy/jgxy1/bin", 
    // "/home/qh/YES/jgxy/jgxy1/label_com", FileReader::BIN_WITH_INTENSITY, "");
    // string save_poses_file = "/home/qh/YES/jgxy/jgxy1/odom/myresult";


    Odometry test_odom("/home/qh/YES/jgxy/jgxy2/bin", 
    "/home/qh/YES/jgxy/jgxy2/label_com", FileReader::BIN_WITH_INTENSITY, "");
    string save_poses_file = "/home/qh/YES/jgxy/jgxy2/odom/myresult";

    while (ros::ok() && test_odom.good()) {
        test_odom.step();
        auto mapCloud = test_odom.getMap();
        auto oriCloud = test_odom.getCurrentCloud();
        auto featureCloud = test_odom.getCurrentFeatureCloud();
        auto poseNow = test_odom.getCurrentPose().matrix();
        auto traj = test_odom.getTraj();
        auto loop = test_odom.getLoop();
        auto loopCur = test_odom.getloopCur();
        auto loopOld = test_odom.getloopOld();
        auto time = test_odom.getCurrTime();
        printf("\033[32m Now frame========>> %d \033[0m\n", (int)(time*10));
        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = poseNow(0, 3);
        odom.pose.pose.position.y = poseNow(1, 3);
        odom.pose.pose.position.z = poseNow(2, 3);
        Eigen::Quaterniond tmpQ(poseNow.block<3, 3>(0, 0));
        odom.pose.pose.orientation.w = tmpQ.w();
        odom.pose.pose.orientation.x = tmpQ.x();
        odom.pose.pose.orientation.y = tmpQ.y();
        odom.pose.pose.orientation.z = tmpQ.z();
        OdomPublisher.publish(odom);

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time().now();
        for (auto& p : traj) {
            Eigen::Quaterniond tmpQ(p.rotation());
            geometry_msgs::PoseStamped tmpP;
            tmpP.pose.orientation.x = tmpQ.x();
            tmpP.pose.orientation.y = tmpQ.y();
            tmpP.pose.orientation.z = tmpQ.z();
            tmpP.pose.orientation.w = tmpQ.w();
            tmpP.pose.position.x = p.translation().x();
            tmpP.pose.position.y = p.translation().y();
            tmpP.pose.position.z = p.translation().z();
            path.poses.push_back(tmpP);
        }
        PathPublisher.publish(path);

        if (!loop.empty()) {
            visualization_msgs::MarkerArray markerArray;
            // loop nodes
            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = ros::Time().now();
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "loop_nodes";
            markerNode.id = 0;
            markerNode.pose.orientation.w = 1;
            markerNode.scale.x = 0.3;
            markerNode.scale.y = 0.3;
            markerNode.scale.z = 0.3;
            markerNode.color.r = 0;
            markerNode.color.g = 0.8;
            markerNode.color.b = 1;
            markerNode.color.a = 1;
            // loop edges
            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = ros::Time().now();
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "loop_edges";
            markerEdge.id = 1;
            markerEdge.pose.orientation.w = 1;
            markerEdge.scale.x = 0.1;
            markerEdge.color.r = 0.9;
            markerEdge.color.g = 0.9;
            markerEdge.color.b = 0;
            markerEdge.color.a = 1;
            for (auto it = loop.begin(); it != loop.end(); ++it) {
                int key_cur = it->first;
                int key_pre = it->second;
                geometry_msgs::Point p;
                p.x = traj[key_cur].translation().x();
                p.y = traj[key_cur].translation().y();
                p.z = traj[key_cur].translation().z();
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);
                p.x = traj[key_pre].translation().x();
                p.y = traj[key_pre].translation().y();
                p.z = traj[key_pre].translation().z();
                markerNode.points.push_back(p);
                markerEdge.points.push_back(p);
            }
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            LoopPublisher.publish(markerArray);
        }

        aftMappedTrans.stamp_ = ros::Time().now();
        aftMappedTrans.setRotation(tf::Quaternion(tmpQ.x(), tmpQ.y(), tmpQ.z(), tmpQ.w()));
        aftMappedTrans.setOrigin(tf::Vector3(poseNow(0, 3), poseNow(1, 3), poseNow(2, 3)));
        tfBroadcaster.sendTransform(aftMappedTrans);

        if (oriCloud) {
            oriCloud->header.frame_id = "map";
            oriCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            OriCloudPublisher.publish(oriCloud);
        }
        if (featureCloud) {
            featureCloud->header.frame_id = "map";
            featureCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            FeatureCloudPublisher.publish(featureCloud);
        }
        if (mapCloud) {
            mapCloud->header.frame_id = "map";
            mapCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            MapCloudPublisher.publish(mapCloud);
        }
        if (loopCur && loopOld) {
            loopCur->header.frame_id = "map";
            loopCur->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            loopOld->header.frame_id = "map";
            loopOld->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            LoopCurPublisher.publish(loopCur);
            LoopOldPublisher.publish(loopOld);
        }
        // printf("\033[32m Done.\033[0m\n");
    }

    if (save_poses_file != "") {
        auto odom_result = test_odom.getTraj();
        std::ofstream file(save_poses_file);
        for (auto& p : odom_result) {
            Eigen::Quaterniond tmpQ(p.rotation());
            auto tmpt = p.translation();
            file << setprecision(6) << std::fixed;
            file << tmpt.x() << " " << tmpt.y() << " " << tmpt.z() << " ";
            file << tmpQ.x() << " " << tmpQ.y() << " " << tmpQ.z() << " " << tmpQ.w() << endl;
        }
        file.close();
    }
}

int main(int argc, char** argv) {
    test_semantic_odometry(argc, argv);
    return 0;
}
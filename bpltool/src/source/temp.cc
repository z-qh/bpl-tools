
#define PCL_NO_PRECOMPILE

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

#include "nav_msgs/Odometry.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>

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


struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time)
)


pcl::PointCloud<PointXYZIL>::Ptr transformPointCloud(pcl::PointCloud<PointXYZIL>::Ptr cloudIn, PointXYZIRPYT& transformIn){

    pcl::PointCloud<PointXYZIL>::Ptr cloudOut(new pcl::PointCloud<PointXYZIL>());

    PointXYZIL *pointFrom;
    PointXYZIL pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];

        float x1 = cos(transformIn.yaw) * pointFrom->x - sin(transformIn.yaw) * pointFrom->y;
        float y1 = sin(transformIn.yaw) * pointFrom->x + cos(transformIn.yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn.roll) * y1 - sin(transformIn.roll) * z1;
        float z2 = sin(transformIn.roll) * y1 + cos(transformIn.roll)* z1;

        pointTo.x = cos(transformIn.pitch) * x2 + sin(transformIn.pitch) * z2 + transformIn.x;
        pointTo.y = y2 + transformIn.y;
        pointTo.z = -sin(transformIn.pitch) * x2 + cos(transformIn.pitch) * z2 + transformIn.z;
        pointTo.intensity = pointFrom->intensity;
        pointTo.label = pointFrom->label;
        cloudOut->points[i] = pointTo;
    }
    

    return cloudOut;
}

void transRPY(pcl::PointCloud<PointXYZIRPYT>&pose){

    Eigen::Matrix4d gauss = Eigen::Matrix4d::Identity();
    gauss <<  -1, 0, 0, 0,
               0, 0, 1, 0,
               0, 1, 0, 0,
               0, 0, 0, 1;
    // gauss.block<3,3>(0,0).normalize();
    for(auto&p:pose.points){
        double R,P,Y;
        R = p.roll;
        P = p.pitch;
        Y = p.yaw;
        double x,y,z;
        x = p.x;
        y = p.y;
        z = p.z;
        Eigen::Vector3d posi(x, y, z);
        Eigen::Matrix3d ori(Eigen::AngleAxisd(Y,Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(P,Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(R,Eigen::Vector3d::UnitX()) );
        //
        //
        auto resR2 = gauss.block<3,3>(0,0) * ori * gauss.block<3,3>(0,0).transpose();
        auto test2 = gauss.block<3,3>(0,0) * posi + gauss.block<3,1>(0,3);
        //
        //
        x = test2(0);
        y = test2(1);
        z = test2(2);
        Y = resR2.eulerAngles(2,1,0)(0);
        P = resR2.eulerAngles(2,1,0)(1);
        R = resR2.eulerAngles(2,1,0)(2);
        //
        p.x = x;
        p.y = y;
        p.z = z;
        p.roll = R;
        p.pitch = P;
        p.yaw = Y;
    }
}

void pubEigenPose(ros::Publisher& pub, PointXYZIRPYT& pose){
    Eigen::Quaterniond tQ(Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX()));
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = pose.z;
    odom.pose.pose.orientation.x = tQ.x();
    odom.pose.pose.orientation.y = tQ.y();
    odom.pose.pose.orientation.z = tQ.z();
    odom.pose.pose.orientation.w = tQ.w();
    pub.publish(odom);
}

pcl::PointCloud<PointXYZIRPYT> readFile(string filePath){
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file(filePath);
    string header;
    getline(file, header);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        PointXYZIRPYT tmpP;
        int index;
        double time;
        double x,y,z;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

pcl::PointCloud<PointXYZIRPYT> readFileNoHead(string filePath){
    pcl::PointCloud<PointXYZIRPYT> result;
    ifstream file(filePath);
    while(file.good()){
        string info;
        getline(file, info);
        if(file.eof()) break;
        stringstream ss(info);
        PointXYZIRPYT tmpP;
        int index;
        double time;
        double x,y,z;
        ss >> tmpP.time >> tmpP.x >> tmpP.y >> tmpP.z >> tmpP.roll >> tmpP.pitch >> tmpP.yaw;
        result.push_back(tmpP);
    }
    cout << filePath << " " << result.size() << endl;
    return result;
}

void generateDaquanLast(ros::Publisher& pubOdom, ros::Publisher& pubCloud){


    pcl::VoxelGrid<PointXYZIL> downSizeFilter;
    downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

    auto poses = readFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematic.txt");
    transRPY(poses);


    pcl::PointCloud<PointXYZIL>::Ptr allCloud(new pcl::PointCloud<PointXYZIL>());
    rosbag::Bag GNSS;
    GNSS.open("/home/qh/2021-01-19-11-10-53DaQuanL.bag", rosbag::bagmode::Read);
    sensor_msgs::PointCloud2 toPub;
    vector<string> topics;
    topics.push_back("/lslidar_point_cloud");
    rosbag::View VV(GNSS, rosbag::TopicQuery(topics));
    auto it = VV.begin();
    int poseCount = 0;
    int succeedCount = 0;
    vector<pcl::PointCloud<PointXYZIL>::Ptr> vec_cloud;
    for(it = VV.begin(); it != VV.end(); it++)
    {
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 tempMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            pcl::PointCloud<PointXYZIL>::Ptr tempCloud(new pcl::PointCloud<PointXYZIL>());
            pcl::fromROSMsg(tempMsg, *tempCloud);
            for(auto&p:tempCloud->points){
                float x = p.x;
                p.x = -p.y;
                p.y = x;
            }
            if(poseCount >= poses.size() ) break;
            if(abs(poses[poseCount].time - tempMsg.header.stamp.toSec()) < 0.02){
                auto transedTempCloud = transformPointCloud(tempCloud, poses[poseCount]);
                *allCloud += *transedTempCloud;
                downSizeFilter.setInputCloud(allCloud);
                downSizeFilter.filter(*allCloud);
                if(allCloud->size() > 200000){
                    cout << " SECTION ================== " << endl;
                    vec_cloud.push_back(allCloud);
                    allCloud.reset(new pcl::PointCloud<PointXYZIL>());
                }
                // char key = getchar();
                // if(key == 'q') break;
                // pcl::toROSMsg(*transedTempCloud, toPub);
                // toPub.header = tempMsg.header;
                // toPub.header.frame_id = "map";
                // pubCloud.publish(toPub);
                cout << poseCount << " Get Cloud " << transedTempCloud->size() << " " << allCloud->size() << endl;
                // pubEigenPose(pubOdom, poses[poseCount]);
                poseCount++;
                succeedCount++;
            }
            else if( tempMsg.header.stamp.toSec() - poses[poseCount].time > 0.05 ){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
            }
        }
    }
    vec_cloud.push_back(allCloud);
    allCloud.reset(new pcl::PointCloud<PointXYZIL>());

    int total_points = 0;
    for(auto&p:vec_cloud){
        total_points += p->points.size();
        *allCloud += *p;
    }
    cout << " all Cloud size : " << total_points * sizeof(PointXYZIL) / 1024.0 / 1024.0 << " MB " << endl;

    cout << "Succeed: " << succeedCount * 100.0 / poses.size() << "%" << endl;

    cout << " Total: " << total_points << endl;


    downSizeFilter.setInputCloud(allCloud);
    downSizeFilter.filter(*allCloud);
    pcl::io::savePCDFile("/home/qh/YES/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSimSematicDS.pcd", *allCloud);


}


void generateO1(ros::Publisher& pubOdom, ros::Publisher& pubCloud){
    auto poses = readFileNoHead("/home/qh/YES/add_ws/10/semantic_long_/RTKMAPafter.txt");
    // transRPY(poses);

    pcl::VoxelGrid<PointXYZIL> downSizeFilter;
    downSizeFilter.setLeafSize(0.5, 0.5, 0.5);

    pcl::PointCloud<PointXYZIL>::Ptr allCloud(new pcl::PointCloud<PointXYZIL>());
    rosbag::Bag GNSS;

    GNSS.open("/home/qh/o1.bag", rosbag::bagmode::Read);

    sensor_msgs::PointCloud2 toPub;

    vector<string> topics;
    topics.push_back("/laser");
    rosbag::View VV(GNSS, rosbag::TopicQuery(topics));
    auto it = VV.begin();
    int poseCount = 0;
    int succeedCount = 0;
    vector<pcl::PointCloud<PointXYZIL>::Ptr> vec_cloud;
    for(it = VV.begin(); it != VV.end(); it++)
    {
        string nowTopic = it->getTopic();
        if(nowTopic == topics[0])
        {
            sensor_msgs::PointCloud2 tempMsg = *(it->instantiate<sensor_msgs::PointCloud2>());
            pcl::PointCloud<PointXYZIL>::Ptr tempCloud(new pcl::PointCloud<PointXYZIL>());
            pcl::fromROSMsg(tempMsg, *tempCloud);
            if(poseCount > 300 && poseCount < 590){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
                continue;
            }
            if(poseCount >= poses.size() ) break;
            if(abs(poses[poseCount].time - tempMsg.header.stamp.toSec()) < 0.02){
                auto transedTempCloud = transformPointCloud(tempCloud, poses[poseCount]);
                *allCloud += *transedTempCloud;
                downSizeFilter.setInputCloud(allCloud);
                downSizeFilter.filter(*allCloud);
                if(allCloud->size() > 200000){
                    cout << " SECTION ================== " << vec_cloud.size() << endl;
                    vec_cloud.push_back(allCloud);
                    allCloud.reset(new pcl::PointCloud<PointXYZIL>());
                }
                // char key = getchar();
                // if(key == 'q') break;
                // pcl::toROSMsg(*transedTempCloud, toPub);
                // toPub.header = tempMsg.header;
                // toPub.header.frame_id = "map";
                // pubCloud.publish(toPub);
                // cout << poseCount << " Get Cloud " << transedTempCloud->size() << " " << allCloud->size() << endl;
                // pubEigenPose(pubOdom, poses[poseCount]);
                poseCount++;
                succeedCount++;
            }
            else if( tempMsg.header.stamp.toSec() - poses[poseCount].time > 0.05 ){
                cout << poseCount << " Get Anandon============== " << endl;
                poseCount++;
            }
        }
    }
    vec_cloud.push_back(allCloud);
    allCloud.reset(new pcl::PointCloud<PointXYZIL>());

    int total_points = 0;
    for(auto&p:vec_cloud){
        total_points += p->points.size();
        *allCloud += *p;
    }

    cout << " all Cloud size : " << total_points * sizeof(PointXYZIL) / 1024.0 / 1024.0 << " MB " << endl;
    cout << "Succeed: " << succeedCount * 100.0 / poses.size() << "%" << endl;
    cout << " Total: " << total_points << endl;


    downSizeFilter.setInputCloud(allCloud);
    downSizeFilter.filter(*allCloud);
    pcl::io::savePCDFile("/home/qh/YES/add_ws/10/semantic_long_/RTKMAPafterDS.pcd", *allCloud);


}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeFullZeroPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>());
    for(auto&p:in->points){
        if(p.x == 0 && p.y == 0 && p.z == 0){
            continue;
        }
        res->push_back(p);
    }
    return res;
} 


pcl::PointCloud<pcl::PointXYZL>::Ptr getCloudAndLabelBin(string bin, string label){
    ifstream binfile;
    binfile.open(bin, ios::in | ios::binary);
    binfile.seekg(0, ios::beg);
    ifstream labelfile;
    labelfile.open(label, ios::in | ios::binary);
    labelfile.seekg(0, ios::beg);
    pcl::PointCloud<pcl::PointXYZL>::Ptr points (new pcl::PointCloud<pcl::PointXYZL>);
    for (int j = 0; binfile.good() && !binfile.eof() && labelfile.good() && !labelfile.eof(); j++) {
        pcl::PointXYZL point;
        float none_use;
        binfile.read((char *) &point.x, 3*sizeof(float));
        binfile.read((char *) &none_use, sizeof(float));
        uint32_t labelValue = 0;
        labelfile.read((char *) &labelValue, sizeof(uint32_t));
        point.label = labelValue & 0xFFFF;
        points->push_back(point);
        if ((binfile.eof() && !labelfile.eof()) || (!binfile.eof() && labelfile.eof()))
        {
            cout << "wrong file " << endl;
            getchar();
        }

    }
    binfile.close();
    labelfile.close();
    return points;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr getCloudAndLabel(string bin, string label){

    pcl::PointCloud<pcl::PointXYZ>::Ptr Zpoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(bin, *Zpoints);
    auto opoints = removeFullZeroPointCloud(Zpoints);

    pcl::PointCloud<pcl::PointXYZL>::Ptr lpoints (new pcl::PointCloud<pcl::PointXYZL>);
    lpoints->resize(opoints->points.size());
    ifstream labelfile;
    filesystem::path lpath(label);
    std::uintmax_t label_size = filesystem::file_size(lpath) / sizeof(uint32_t);
    labelfile.open(label, ios::in | ios::binary);
    labelfile.seekg(0, ios::beg);
    if(opoints->points.size() != label_size){
        cout << "wrong file " << endl;
        cout << "pcd: " << opoints->points.size() << bin << endl;
        cout << "label: " << label_size << label << endl;
        getchar();
        exit(0);
    }
    for (int i = 0; i < label_size; ++i){
        uint32_t labelValue = 0;
        labelfile.read((char *) &labelValue, sizeof(uint32_t));
        lpoints->points[i].x = opoints->points[i].x;
        lpoints->points[i].y = opoints->points[i].y;
        lpoints->points[i].z = opoints->points[i].z;
        lpoints->points[i].label = labelValue;
    }
    labelfile.close();
    cout << "get " << lpoints->size() << " points" << endl;

    return lpoints;
}

void combinexyz_l(){
    vector<string> pcd_path_vec, lab_path_vec, des_paht_vec;
    // pcd_path_vec.push_back("/home/qh/YES/dlut/Daquan16/ip/com_pc");
    // lab_path_vec.push_back("/home/qh/YES/dlut/Daquan16/ip/label");
    // des_paht_vec.push_back("/home/qh/YES/dlut/Daquan16/ip");

    // pcd_path_vec.push_back("/home/qh/YES/dlut/Daquan16/nconv/com_pc");
    // lab_path_vec.push_back("/home/qh/YES/dlut/Daquan16/nconv/label");
    // des_paht_vec.push_back("/home/qh/YES/dlut/Daquan16/nconv");

    // pcd_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/ip/com_pc");
    // lab_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/ip/label");
    // des_paht_vec.push_back("/home/qh/YES/jgxy/jgxy1/ip");

    // pcd_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/nconv/com_pc");
    // lab_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/nconv/label");
    // des_paht_vec.push_back("/home/qh/YES/jgxy/jgxy1/nconv");

    // pcd_path_vec.push_back("/home/qh/YES/dlut/Daquan16/ori_pc");
    // lab_path_vec.push_back("/home/qh/YES/dlut/Daquan16/label_com");
    // des_paht_vec.push_back("/home/qh/YES/dlut/Daquan16/ori_spc");

    // pcd_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/ori_pc");
    // lab_path_vec.push_back("/home/qh/YES/jgxy/jgxy1/label_com");
    // des_paht_vec.push_back("/home/qh/YES/jgxy/jgxy1/ori_spc");


    pcd_path_vec.push_back("/home/qh/kitti/08/velodyne");
    lab_path_vec.push_back("/home/qh/kitti/08/labels");
    des_paht_vec.push_back("/home/qh/kitti/08/depth_com/label_ori_pc");
    

    for(int i = 0; i < pcd_path_vec.size(); ++i){
        string pcd_path = pcd_path_vec[i];
        string lab_path = lab_path_vec[i];
        string des_paht = des_paht_vec[i];
        for (auto& file : filesystem::directory_iterator(pcd_path))
        {
            // 如果文件后缀名为.pcd，则输出文件路径
            if (file.path().extension() == ".bin")
            {
                auto pcd_file = filesystem::path(file.path().string());
                int a = stoi(file.path().stem());
                cout << a << endl;
                if(a % 100 != 0) continue;
                auto lab_file = filesystem::path(lab_path + "/" + file.path().stem().string() + ".label");
                auto des_file = filesystem::path(des_paht + "/" + file.path().stem().string() + ".pcd");
                if(filesystem::exists(pcd_file) && filesystem::exists(lab_file) && !filesystem::exists(des_file)){
                    auto tmpCloud = getCloudAndLabelBin(pcd_file.string(), lab_file.string());
                    pcl::io::savePCDFile(des_file.string(), *tmpCloud);
                }else if(filesystem::exists(des_file)){
                    cout << "exist" << endl;
                    cout << "des: " << des_file.string() << endl;
                }
                else{
                    cout << "error" << endl;
                    cout << "pcd: " << pcd_file.string() << endl;
                    cout << "lab: " << lab_file.string() << endl;
                    cout << "des: " << des_file.string() << endl;
                }
                
            }
        }
    }

}

int main(int argc, char** argv){
    combinexyz_l();
    // rosbag::Bag Daquan16("/home/qh/YES/dlut/2021-01-16-DaQuan.bag", rosbag::BagMode::Read);
    return 0;
}

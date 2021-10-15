#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "gnss_driver/gps_navi_msg.h"
#include "tf/transform_datatypes.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include "../include/imu_gnss/gps_ins_msg.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"

#include "flexKeyboard.hpp"

using namespace std;

bool getOdomFronFile(vector<nav_msgs::Odometry>& odom, string filePath)
{
    ifstream file;
    file.open(filePath, ios::in);
    if(!file.good())
        return false;
    int odomSize = 0;
    file >> odomSize;
    odom.resize(odomSize);
    for(int i = 0; i < odomSize; i++){
        double time = 0, roll = 0, pitch = 0, yaw = 0, temp;
        file >> time >> odom[i].pose.pose.position.x
        >>odom[i].pose.pose.position.y
        >>odom[i].pose.pose.position.z
        >> roll >> pitch >> yaw >> temp;
        auto q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        odom[i].header.frame_id = "camera_init";
        odom[i].header.stamp = ros::Time().fromSec(time);
        odom[i].pose.pose.orientation = q;
    }
    file.close();
    cout << "size " << odom.size() << endl;
    return true;
}

string realPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/odomRealSematic.txt";
string simPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/odomSimSematic.txt";

ros::Publisher pubGT;
sensor_msgs::PointCloud2 tempMsgGT;

void pubGTThread()
{
    ros::Rate loop(1);
    while(ros::ok()){
        pubGT.publish(tempMsgGT);
        loop.sleep();
    }
}

int mai2n(int argc, char** argv)
{
    ros::init(argc, argv, "imuWatcher");
    ros::NodeHandle nh;
    char* key;
    flexKeyboard::flexKeyboard FK(key);

    pubGT = nh.advertise<sensor_msgs::PointCloud2>("/GT", 1);

    {
        ifstream fileGT;
        string outFilenameGT = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanLbak/ground_truth.txt";
        fileGT.open(outFilenameGT);
        int odomSizeGT  = 0;
        fileGT >> odomSizeGT;
        pcl::PointCloud<pcl::PointXYZ> cloudGT;
        cloudGT.resize(odomSizeGT);
        double R = 0, P = 0, Y = 0;
        for(int i = 0; i < odomSizeGT; i++)
        {
            fileGT >> R >> cloudGT[i].x >> cloudGT[i].z >> cloudGT[i].y >> R >> P >> Y >> R;
            cloudGT[i].z = -cloudGT[i].z;
        }
        fileGT.close();
        pcl::toROSMsg(cloudGT, tempMsgGT);
        tempMsgGT.header.frame_id = "camera_init";
        std::thread loopGT(pubGTThread);
        loopGT.detach();
    }

    ros::Publisher pubSim = nh.advertise<nav_msgs::Odometry>("/odomSim", 100);
    ros::Publisher pubReal = nh.advertise<nav_msgs::Odometry>("/odomReal", 100);

    vector<nav_msgs::Odometry> globalOdomSim;
    vector<nav_msgs::Odometry> globalOdomReal;

    getOdomFronFile(globalOdomSim, simPath);
    getOdomFronFile(globalOdomReal, realPath);

    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;


    aftMappedTrans.frame_id_ = "camera_init";
    aftMappedTrans.child_frame_id_ = "aft_mapped";

    ros::Rate* loop;
    loop = new ros::Rate(30);
    int fps = 30;
    bool state = false;
    int simCount = 0;
    int realCount = 0;
    while(ros::ok())
    {
        if(*key == '1' || *key == '!'){
            if(simCount<globalOdomSim.size()){
                if(simCount<globalOdomSim.size()) {
                    aftMappedTrans.stamp_ = globalOdomSim[simCount].header.stamp;
                    aftMappedTrans.setRotation(tf::Quaternion(-globalOdomSim[simCount].pose.pose.orientation.y,
                                                              -globalOdomSim[simCount].pose.pose.orientation.z,
                                                              globalOdomSim[simCount].pose.pose.orientation.x,
                                                              globalOdomSim[simCount].pose.pose.orientation.w));
                    aftMappedTrans.setOrigin(tf::Vector3(globalOdomSim[simCount].pose.pose.position.x,
                                                         globalOdomSim[simCount].pose.pose.position.y,
                                                         globalOdomSim[simCount].pose.pose.position.z));
                    tfBroadcaster.sendTransform(aftMappedTrans);
                    pubSim.publish(globalOdomSim[simCount++]);
                }
            }
        }else if(*key == '2' || *key == '@'){
            if(realCount<globalOdomReal.size()) {
                aftMappedTrans.stamp_ = globalOdomReal[realCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomReal[realCount].pose.pose.orientation.y,
                                                          -globalOdomReal[realCount].pose.pose.orientation.z,
                                                          globalOdomReal[realCount].pose.pose.orientation.x,
                                                          globalOdomReal[realCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomReal[realCount].pose.pose.position.x,
                                                     globalOdomReal[realCount].pose.pose.position.y,
                                                     globalOdomReal[realCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubReal.publish(globalOdomReal[realCount++]);
            }
        }else if(*key == flexKeyboard::KEYCODE_SPACE){
            state = !state;
        }else if(*key == '-' || *key == '_'){
            delete loop;
            fps = (fps-5)<5?5:(fps-5);
            cout << "fps: " << fps << endl;
            loop = new ros::Rate(fps);
        }else if(*key == '=' || *key == '+'){
            delete loop;
            fps = (fps+5)>50?50:(fps+5);
            cout << "fps: " << fps << endl;
            loop = new ros::Rate(fps);
        }else;
        if(state)
        {
            if(simCount<globalOdomSim.size()) {
                aftMappedTrans.stamp_ = globalOdomSim[simCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomSim[simCount].pose.pose.orientation.y,
                                                          -globalOdomSim[simCount].pose.pose.orientation.z,
                                                          globalOdomSim[simCount].pose.pose.orientation.x,
                                                          globalOdomSim[simCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomSim[simCount].pose.pose.position.x,
                                                     globalOdomSim[simCount].pose.pose.position.y,
                                                     globalOdomSim[simCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubSim.publish(globalOdomSim[simCount++]);
            }
            if(realCount<globalOdomReal.size()) {
                aftMappedTrans.stamp_ = globalOdomReal[realCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomReal[realCount].pose.pose.orientation.y,
                                                          -globalOdomReal[realCount].pose.pose.orientation.z,
                                                          globalOdomReal[realCount].pose.pose.orientation.x,
                                                          globalOdomReal[realCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomReal[realCount].pose.pose.position.x,
                                                     globalOdomReal[realCount].pose.pose.position.y,
                                                     globalOdomReal[realCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubReal.publish(globalOdomReal[realCount++]);
            }
        }
        loop->sleep();
    }
    return 0;
}
//nan
//none
//null

using namespace std;

#include "string"
#include "../../bpl-tools/topo_handle/src/include/dbscan/node_context_localMap.h"

void create_node_from_file_B(std::vector<node>& nodes, std::string& path)
{
    std::vector<std::string>file_name;
    std::vector<int>index;
    DIR *d = opendir(path.c_str());
    struct dirent *dp;
    while((dp = readdir(d)) != NULL)
    {
        if(dp->d_name[0] == '.')    {continue;}
        index.push_back(atoi(dp->d_name));
    }
    sort(index.begin(), index.end());
    closedir(d);
    int node_index = 0;
    if(index.size() > 0)
    {
        int node_number = index.size();
        nodes.resize(node_number);
        for(int i=0; i<node_number;++i)
        {
            node tmp_node;
            tmp_node.create_node_from_file_pose_only(path, index[i]);
            nodes[node_index] = tmp_node;
            node_index++;
        }
    }
    cout<<"number "<<nodes.size()<<endl;
}

void readTopomap(sensor_msgs::PointCloud2& msgs, string path){
    ifstream file;
    file.open(path);
    pcl::PointCloud<pcl::PointXYZI> tempCloud;
    while(!file.eof()){
        pcl::PointXYZI temPoint;
        file >> temPoint.x >> temPoint.y >> temPoint.z >> temPoint.intensity;
        tempCloud.push_back(temPoint);
    }
    pcl::toROSMsg(tempCloud, msgs);
    msgs.header.frame_id = "camera_init";
    cout << "size " << tempCloud.points.size() << endl;
}

void readTopomapRecall(sensor_msgs::PointCloud2& msgs, string path){
    ifstream file;
    file.open(path);
    if(!file.is_open()){
        cout << " file read error" << endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZI> tempCloud;
    while(!file.eof()){
        pcl::PointXYZI temPoint;
        int id;
        file >> id >> temPoint.x >> temPoint.y >> temPoint.z >> temPoint.intensity;
        tempCloud.push_back(temPoint);
    }
    pcl::toROSMsg(tempCloud, msgs);
    msgs.header.frame_id = "camera_init";
    cout << "size " << tempCloud.points.size() << endl;
}

void getNodeInfo(string pathin, string pathout){
    vector<node> states;
    vector<int> gnss;
    vector<int> nognss;
    std::vector<std::string>file_name;
    std::vector<int>index;
    DIR *d = opendir(pathin.c_str());
    struct dirent *dp;
    while((dp = readdir(d)) != NULL)
    {
        if(dp->d_name[0] == '.')    {continue;}
        index.push_back(atoi(dp->d_name));
    }
    sort(index.begin(), index.end());
    closedir(d);
    int max_id_in_file = INT_MIN;
    int success_number = 0;
    int node_index = 0;
    if(index.size() > 0)
    {
        int node_number = index.size();

        for(int i=0; i<node_number;++i)
        {
            node tmp_node;
            tmp_node.create_node_from_file_pose_only(pathin, index[i]);
            states.push_back(tmp_node);
        }
        cout<<endl;
    }


    bool isFirst{true};
    bool mode{true};
    int gnssSig = 0;
    int gnssSigLast = 0;
    bool waitFlag = false;
    double nowTime = 0;
    double startTime = 0;
    for(int i = 0; i < states.size(); i++){
        node& input_node = states[i];
        nowTime = input_node.time_stamp_;
        if(input_node.Global_Pose_.intensity == -1){
            gnssSigLast = gnssSig;
            gnssSig = 1;
            if(isFirst) {
                mode = true;
                gnssSigLast = gnssSig;
            }
        }else{
            gnssSigLast = gnssSig;
            gnssSig = 2;
            if(isFirst) {
                mode = false;
                gnssSigLast = gnssSig;
            }
        }
        if(input_node.Global_Pose_.intensity == 5){
            gnssSig = gnssSigLast;
        }
        if(gnssSig != gnssSigLast && !waitFlag){
            waitFlag = true;
            startTime = nowTime;
            cout << "start time " << startTime << endl;
        }

        if(waitFlag){
            cout << "now time " << nowTime << endl;
            if(nowTime - startTime > 5.0){
                waitFlag = false;
                mode = gnssSig==2?false:true;
            }
        }else if(gnssSig==2){
            mode = false;
        }else if(gnssSig==1){
            mode = true;
        }
        if(isFirst) isFirst = false;
        if(mode){
            nognss.push_back(i);
        }else{
            gnss.push_back(i);
        }
    }


    ofstream fileOut;
    fileOut.open(pathout);
    fileOut << "gnss: " << gnss.size() <<endl;
    for(auto i : gnss){
        fileOut << i << " ";
    }
    fileOut << endl;
    fileOut << "no gnss: " << nognss.size() << endl;
    for(auto i : nognss){
        fileOut << i << " ";
    }
    fileOut.close();
}

bool comMy(int a, int b){
    return a < b;
}

double dis(node&A,node&B){
    return sqrt(pow(A.Global_Pose_.x-B.Global_Pose_.x,2)
                +pow(A.Global_Pose_.y-B.Global_Pose_.y,2)
                +pow(A.Global_Pose_.z-B.Global_Pose_.z,2));
}

bool comPair(pair<int,double>A, pair<int,double>B)
{
    return A.second < B.second;
}

#include "pcl/kdtree/kdtree_flann.h"

vector<pair<pair<int,int>,double>> findATruthPair(int n){
    vector<pair<pair<int,int>,double>> result;
    pcl::KdTreeFLANN<pcl::PointXYZ> KDTree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>());
    ifstream fileIn;
    fileIn.open("/home/qh/robot_ws/map/2021-08-30-18-06-30L/gtFittingA.txt");
    int odomSize = 0;
    double tempNone;
    fileIn >> tempNone >> tempNone >> tempNone;
    fileIn >> odomSize;
    for(int i = 0; i < odomSize; i++){
        pcl::PointXYZ tempPoint;
        int flag(0);
        fileIn >> tempNone;
        fileIn >>
               tempPoint.x >>
               tempPoint.y >>
               tempPoint.z;
        fileIn >> tempNone >> tempNone >> tempNone;
        fileIn >> tempNone;
        fileIn >> tempNone >> tempNone >> tempNone >> tempNone;
        if(i>40)cloudTemp->push_back(tempPoint);
    }
    fileIn.close();
    cout << "odom size open succeed : " << odomSize << " poses!" << endl;
    KDTree.setInputCloud(cloudTemp);

    if(n == -1){
        for(int i = 0; i < cloudTemp->size(); i+=10){
            pcl::PointXYZ tempPoint;
            tempPoint.x = cloudTemp->points[i].x;
            tempPoint.y = cloudTemp->points[i].y;
            tempPoint.z = cloudTemp->points[i].z;
            vector<int> index;
            vector<float> indexDis;
            KDTree.radiusSearch(tempPoint, 5.0, index, indexDis);
            int countTemp = 0;
            for(int j = 0; j < index.size(); j++){
                if(index[j] - i > 1000) {
                    result.push_back( { {i, index[j]}, sqrt(indexDis[j]) });
                    cout << " " << i << " " << index[j] << " " << sqrt(indexDis[j]) << endl;
                    if(countTemp++>3) break;
                }
            }
        }
    }else{
        pcl::PointXYZ tempPoint;
        tempPoint.x = cloudTemp->points[n].x;
        tempPoint.y = cloudTemp->points[n].y;
        tempPoint.z = cloudTemp->points[n].z;
        vector<int> index;
        vector<float> indexDis;
        KDTree.radiusSearch(tempPoint, 5.0, index, indexDis);
        int countTemp = 0;
        for(int j = 0; j < index.size(); j++){
            if(index[j] - n > 500) {
                result.push_back({{n, index[j]}, sqrt(indexDis[j])});
                cout << " " << n << " " << index[j] << " " << sqrt(indexDis[j]) << endl;
                if(countTemp++>3)break;
            }
        }
    }

    return result;
}


int main (int argc, char ** argv){
    ros::init(argc, argv, "topomap");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/toppomap", 1);
    ros::Publisher pubrecall = nh.advertise<sensor_msgs::PointCloud2>("/toppomapRecall", 1);

    if(argv[1] == nullptr || argv[2] == nullptr){
        cout << " param 1 : build topo file name like N0.23 " << endl;
        cout << " param 2 : id to find to turth pair " << endl;
        exit(0);
    }

    sensor_msgs::PointCloud2 PubMsgs, PubMsgsRecall;
    //string fileName(argv[1]);
    //string topoNodePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/" + fileName + ".txt";
    //cout << topoNodePath << endl;
    //readTopomap(PubMsgs, topoNodePath);

    //string topoNodePathRecall = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/0.15-8.0-Recall.txt";
    //readTopomapRecall(PubMsgsRecall, topoNodePathRecall);
    auto truthPair = findATruthPair(stoi(argv[2]));
    cout << " get answer " << endl;
    string asdsad = "/home/qh/2loopSelf.txt";
    ofstream file;
    file.open(asdsad);
    for(auto n : truthPair) {
        file << n.first.first << " " << n.first.second << " " << n.second << endl;
    }
    file.close();
    return 0;


//    vector<node> sourceNode;
//
//    string pathSourceNode = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/nodeSparse/20-60-40/";
//    create_node_from_file_B(sourceNode, pathSourceNode);
//    pcl::PointCloud<pcl::PointXYZI> tempCloud;
//    for(int i =0 ;i < sourceNode.size(); i++){
//        if(i % 5 != 0) continue;
//        pcl::PointXYZI tempPoint;
//        cout <<i/5+1 << " " << sourceNode[i].id_ << endl;
//        tempPoint.x = sourceNode[i].Global_Pose_.x;
//        tempPoint.y = sourceNode[i].Global_Pose_.y;
//        tempPoint.z = sourceNode[i].Global_Pose_.z;
//        tempPoint.intensity = sourceNode[i].Global_Pose_.intensity;
//        tempCloud.push_back(tempPoint);
//    }
//    pcl::toROSMsg(tempCloud, PubMsgs);
    PubMsgs.header.frame_id = "camera_init";

    ros::Rate loop(1);
    while(ros::ok()){
        pub.publish(PubMsgs);
        pubrecall.publish(PubMsgsRecall);
        loop.sleep();
    }

    return 0;
}



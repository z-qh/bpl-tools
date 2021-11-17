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

void create_node_from_file_B(std::vector<node>& nodes, std::string& path, pcl::PointCloud<pcl::PointXYZ>& cloud)
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
        cloud.resize(node_number);
        for(int i=0; i<node_number;++i)
        {
            node tmp_node;
            tmp_node.create_node_from_file_pose_only(path, index[i]);
            nodes[node_index] = tmp_node;
            cloud.points[i].x = tmp_node.Global_Pose_.x;
            cloud.points[i].y = tmp_node.Global_Pose_.y;
            cloud.points[i].z = tmp_node.Global_Pose_.z;
            node_index++;
        }
    }
    cout<<"number "<<nodes.size()<<endl;
}

void readTopomap(sensor_msgs::PointCloud2& msgs, string path){
    if( access(path.c_str(), 0) != 0 )
        return;
    ifstream file;
    file.open(path);
    if(!file.is_open()){
        cout << " file read error" << endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZI> tempCloud;
    while(!file.eof()){
        pcl::PointXYZI temPoint;
        file >> temPoint.x >> temPoint.y >> temPoint.z >> temPoint.intensity;
        tempCloud.push_back(temPoint);
    }
    pcl::toROSMsg(tempCloud, msgs);
    msgs.header.frame_id = "camera_init";
    cout << "size " << tempCloud.points.size() << endl;
    string pathSave = path.substr(0, path.length() - 4);
    if( access((pathSave+".pcd").c_str(), 0) != 0 )
        pcl::io::savePCDFile(pathSave+".pcd", tempCloud);
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
    return A.second > B.second;
}

#include "pcl/kdtree/kdtree_flann.h"

bool comFirst(pair<int,double>A,pair<int,double>B){
    return A.first < B.first;
}

vector<pair<int, double>> findATruthPair(int n){
    vector<pair<int, double>> result;
    vector<node> nodeList;
    string nodeFilePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/nodeSparse/";
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree1;
    create_node_from_file_B(nodeList, nodeFilePath, *tempCloud);
    kdTree1.setInputCloud(tempCloud);

    pcl::PointXYZ tempPoint;
    tempPoint.x = nodeList[n].Global_Pose_.x;
    tempPoint.y = nodeList[n].Global_Pose_.y;
    tempPoint.z = nodeList[n].Global_Pose_.z;
    vector<int> index;
    vector<float> indexDis;

    kdTree1.radiusSearch(tempPoint,8,index,indexDis);

    for(int i = 0; i < index.size(); i++){
        if(index[i]-n>1000) result.push_back({index[i], sqrt(indexDis[i])});
    }
    sort(result.begin(), result.end(), comFirst);
    return result;
}


void readFileOdom(string filePathIn, pcl::PointCloud<pcl::PointXYZ>& cloud){
    ifstream fileIn;
    fileIn.open(filePathIn);

    int odomSize = 0;
    double tempNone;
    fileIn >> tempNone >> tempNone >> tempNone;
    fileIn >> odomSize;
    cloud.resize(odomSize);
    for(int i = 0; i < odomSize; i++){
        double R(0), P(0), Y(0);
        double time(0);
        int flag(0);
        fileIn >> time;
        fileIn >>
               cloud.points[i].x >>
               tempNone >>
               cloud.points[i].y;
        fileIn >> R >> P >> Y;
        fileIn >> flag;
        fileIn >> tempNone >> tempNone >> tempNone >> tempNone;
        tf::Quaternion tempQ = tf::createQuaternionFromRPY(R, P, Y);
    }
    fileIn.close();
    cout << "odom size open succeed : " << odomSize << " poses!" << endl;
}



int main456(int argc, char ** argv){
    ros::init(argc, argv, "topomap");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/toppomap", 1);
    ros::Publisher pubrecall = nh.advertise<sensor_msgs::PointCloud2>("/toppomapRecall", 1);
    ros::Publisher traj1 = nh.advertise<sensor_msgs::PointCloud2>("/traj1", 1);
    ros::Publisher traj2 = nh.advertise<sensor_msgs::PointCloud2>("/traj2", 1);
    ros::Publisher traj1Dis = nh.advertise<sensor_msgs::PointCloud2>("/traj1Dis", 1);

    sensor_msgs::PointCloud2 PubMsgs, PubMsgsRecall;
    
    if( argv[1] != nullptr)
    {
        vector<double> buidNValue = {};
        for(auto n : buidNValue){
            string tempBase(argv[1]);
            tempBase = tempBase.substr(0, tempBase.length()-4);
            stringstream ss;
            ss << fixed << setprecision(2) << n;
            string tempPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/" + tempBase + ss.str() + ".txt";
            cout << tempPath << endl;
            sensor_msgs::PointCloud2 msgsT;
            readTopomap(msgsT, tempPath);
        }

        string fileName(argv[1]);
        string topoNodePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/" + fileName + ".txt";
        cout << topoNodePath << endl;
        readTopomap(PubMsgs, topoNodePath);
    }

    //string topoNodePathRecall = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/0.15-8.0-Recall.txt";
    //readTopomapRecall(PubMsgsRecall, topoNodePathRecall);
    if(argv[2] != nullptr){
        auto truthPair = findATruthPair(stoi(argv[2]));
        for(auto n : truthPair) {
            cout << n.first << " " << n.second << endl;
        }
    }




    // pcl::PointCloud<pcl::PointXYZ> cloud1,cloud2;
    // pcl::PointCloud<pcl::PointXYZI> showCloud1;
    // readFileOdom("/home/qh/robot_ws/map/2021-08-30-16-12-25L/gtFittingA.txt", cloud1);
    // readFileOdom("/home/qh/robot_ws/map/2021-08-30-18-06-30L/gtFittingA.txt", cloud2);
    // sensor_msgs::PointCloud2 msgs1, msgs2, msgs1ShowDis;
    // pcl::toROSMsg(cloud1, msgs1);
    // pcl::toROSMsg(cloud2, msgs2);
    // msgs1.header.frame_id  ="camera_init";
    // msgs2.header.frame_id  ="camera_init";
    // {

    //     pcl::PointCloud<pcl::PointXY>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXY>());
    //     cloudPtr->resize(cloud2.size());
    //     for(int j = 0 ;j < cloud2.size(); j++){
    //         cloudPtr->points[j].x = cloud2.points[j].x;
    //         cloudPtr->points[j].y = cloud2.points[j].y;
    //     }
    //     pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    //     kdtree.setInputCloud(cloudPtr);
    //     vector<float> disList1;
    //     disList1.resize(cloud1.size());
    //     vector<int> maxList1Index;
    //     vector<pair<int, double>> maxList;
    //     for(int j = 0; j < cloud1.size(); j++){
    //         const pcl::PointXYZ& currentP = cloud1.points[j];
    //         pcl::PointXY p;
    //         p.x = currentP.x;
    //         p.y = currentP.y;
    //         vector<int> indexClosets(1);
    //         vector<float> indexClosetsDisSquare(1);
    //         kdtree.nearestKSearch(p, 1, indexClosets, indexClosetsDisSquare);
    //         disList1[j] = indexClosetsDisSquare.front();
    //         maxList1Index.push_back(j);
    //         maxList.push_back({indexClosets.front(), sqrt(indexClosetsDisSquare.front())});
    //     }
    //     sort(maxList.begin(), maxList.end(), comPair);
    //     for(int j = 0; j < maxList.size(); j++){
    //         const auto& n  = maxList[j];
    //         cout << " 2 id " << maxList1Index[j];
    //         cout << " max id " << n.first;
    //         cout << " the dis " << n.second << endl;
    //         if(n.second < 3.5){
    //             cout << " more than 3.5m total " << j << endl;
    //             break;
    //         }
    //     }
    //     showCloud1.resize(disList1.size());
    //     for(int j = 0; j < disList1.size(); j++){
    //         showCloud1.points[j].x = cloud1.points[j].x;
    //         showCloud1.points[j].y = cloud1.points[j].y;
    //         showCloud1.points[j].z = 0;
    //         showCloud1.points[j].intensity = disList1[j];
    //     }
    //     pcl::toROSMsg(showCloud1, msgs1ShowDis);
    //     msgs1ShowDis.header.frame_id = "camera_init";
    // }

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
        // traj1Dis.publish(msgs1ShowDis);
        // traj1.publish(msgs1);
        // traj2.publish(msgs2);
        pub.publish(PubMsgs);
        pubrecall.publish(PubMsgsRecall);
        loop.sleep();
    }

    return 0;
}

#include "sensor_msgs/LaserScan.h"


void readFileOdomDQ(string filePathIn, pcl::PointCloud<pcl::PointXYZI>& cloud){
    ifstream fileIn;
    fileIn.open(filePathIn);

    int odomSize = 0;
    double tempNone;
    fileIn >> odomSize;
    cloud.resize(odomSize);
    for(int i = 0; i < odomSize; i++){
        double R(0), P(0), Y(0);
        double time(0);
        int flag(0);
        fileIn >> time;
        fileIn >>
               cloud.points[i].x >>
               cloud.points[i].y >>
               cloud.points[i].z;
        fileIn >> R >> P >> Y;
        fileIn >> flag;
        cloud.points[i].intensity = flag;
        tf::Quaternion tempQ = tf::createQuaternionFromRPY(R, P, Y);
    }
    fileIn.close();
    cout << "odom size open succeed : " << odomSize << " poses!" << endl;
}


int main123(int argc, char** argv){
    ros::init(argc, argv, "dqea");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/DQ", 1);

    pcl::PointCloud<pcl::PointXYZI> tempCloud;
    readFileOdomDQ("/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomRealSematic.txt", tempCloud);
    sensor_msgs::PointCloud2 tempCloudMsgs;
    pcl::toROSMsg(tempCloud, tempCloudMsgs);
    tempCloudMsgs.header.frame_id = "camera_init";

    ros::Rate loop(1);
    while(ros::ok()){
        pub.publish(tempCloudMsgs);
        loop.sleep();
    }

    return 0;
}

// int main(int argc, char** argv){
//     ros::init(argc, argv, "bagConvert");
//     ros::NodeHandle nh;

//     rosbag::Bag allFile;
//     allFile.open("/media/qh/YES/JGXY.bag", rosbag::bagmode::Read);
//     rosbag::Bag outFile;
//     outFile.open("/media/qh/YES/JGXYCalibration.bag", rosbag::bagmode::Write);
//     vector<string> topics{"/gps_navi", "/os_cloud_node/points", "/imu/data"};
//     rosbag::View view(allFile, rosbag::TopicQuery(topics));
//     rosbag::View::iterator it;
//     ros::Time nowTime(0);
//     int cloudCount = 0;
//     for(it = view.begin(); it != view.end() && ros::ok(); it++){
//         if(it->getTopic() == "/gps_navi"){
//             gnss_driver::gps_navi_msg::Ptr tempPtrA = (*it).instantiate<gnss_driver::gps_navi_msg>();
//             if(nowTime.toSec() != 0){
//                 outFile.write("/gps_navi", nowTime, tempPtrA);
//             }
//         }else if(it->getTopic() == "/os_cloud_node/points"){
//             cloudCount++;
//             sensor_msgs::PointCloud2::Ptr tempPtrA = (*it).instantiate<sensor_msgs::PointCloud2>();
//             nowTime = tempPtrA->header.stamp;
//             outFile.write("/os_cloud_node/points", tempPtrA->header.stamp, tempPtrA);
//         }else if(it->getTopic() == "/imu/data"){
//             sensor_msgs::Imu::Ptr tempPtrA = (*it).instantiate<sensor_msgs::Imu>();
//             outFile.write("/imu/data", tempPtrA->header.stamp, tempPtrA);
//         }
//         if(cloudCount > 300){
//             break;
//         }
//     }
//     allFile.close();
//     outFile.close();
//     return 0;
// }

int main(int argc, char** argv){

    cout << "123" << endl;
    // #pragma omp parallel for num_threads(3)
    for(int i = 0; i < 20; i++){
        cout << i;
    }
    return 1;
}

#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"

#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"

string pathBase;
bool loadPCD = false;

class buildTopo{
private:
    bool isFirst{true};
    bool mode{true};
    int gnssSig = 0;
    int gnssSigLast = 0;
    bool waitFlag = false;
    double nowTime = 0;
    double startTime = 0;
    double judgeLoopScore = 0;
    double buildMapSimScore = 0;
    double disForDetech = 0;
    //////////////////////////////////////
    bool save_data_to_files = false;
    std::string node_save_path;;
    //////////////////////////////////////
public:
    vector<node>nodes;
private:
    int current_node_id = 0;
    int last_node_id = -1;
    //////////////////////////////////////
    double distanceFromPreNode = 0.0;      //距离上一个拓扑节点的间距
    Eigen::Vector3d PreviousNodePosition = Eigen::Vector3d::Zero();     //上一个拓扑节点key的index
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistroyKeyPoses;
    Eigen::MatrixXd sc_current, sc_last;
    //////////////////////////////////////
    //////////////////////////////////////////////////////////
    std::vector<Eigen::MatrixXd>global_scanContext;                 //存储全局描述子
    pcl::PointCloud<PointType>::Ptr globalKeyPose3d;
    //////////////////////////////////////////////////////////
    pcl::PointCloud<PointType>::Ptr pubGlobalNodePosition;  //存储每个节点的全局位置
    //////////////////////////////////////////////////////////
    unordered_map<int,int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
    unordered_map<int,int>allNodeIndexId;           //第一个为节点的id，第二个为在nodes中的索引
    //////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////
    int loopClosureFromKDTree(PointType currentPose, node& scNode, pcl::PointCloud<pcl::PointXYZI>& scCloud)
    {
        if(global_scanContext.size() > 1)
        {
            int result = (int)scanContextKnnSearch(currentPose, scNode, scCloud);
            if(result == -1)
            {
                return -1;
            }else
                return result;
        }
        return -1;
    }
    //通过kd树搜索，返回闭环节点id
    int scanContextKnnSearch(PointType currentPose, node& currNode, pcl::PointCloud<pcl::PointXYZI>& currCloud)
    {
        double min_dist = 10000000;
        int loop_index = -1;
        //使用位置进行KD树搜索
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, disForDetech, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for(int j  = 0; j < pointSearchIndLoop.size(); j++){
            int n = pointSearchIndLoop[j];
            if(currNode.time_stamp_ - nodes[n].time_stamp_ < 60.0){
                continue;
            }else{
                double scoreNow = node::getScore(nodes[n], currNode, currCloud);
                if(scoreNow < judgeLoopScore){
                    return n;
                }
            }
        }
        return -1;
    }


    double getScoreFromLastNode(Eigen::MatrixXd& sc_current, Eigen::MatrixXd& sc_last)
    {
        double score = distDirectSC(sc_current, sc_last);
        return score;
    }

public:
    buildTopo(std::string node_save_path_, double buildMapSimScore_, double disForDetech_ = 15.0)
    {
        ////////////////////
        kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
        pubGlobalNodePosition.reset(new pcl::PointCloud<PointType>());
        /////////////////////
        disForDetech = disForDetech_;
        judgeLoopScore = buildMapSimScore_;//描述子余弦距离
        buildMapSimScore = buildMapSimScore_;//todo
        /////////////////////
        node_save_path = node_save_path_;
        save_data_to_files = true;//是否保存文件
        /////////////////////

    }

    void runS(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud)
    {
        if(isFirst) {
            isFirst = false;
            ////////////////////////////////////////////
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
            input_node.Global_Pose_.intensity = mode;
            nodes.push_back(input_node);

            sc_current = input_node.scanContext_;
            sc_last = sc_current;

            PointType thisPose;
            thisPose.x = input_node.Global_Pose_.x;
            thisPose.y = input_node.Global_Pose_.y;
            thisPose.z = input_node.Global_Pose_.z;
            thisPose.intensity = current_node_id;
            globalKeyPose3d->push_back(thisPose);

            PreviousNodePosition(0) = thisPose.x;
            PreviousNodePosition(1) = thisPose.y;
            PreviousNodePosition(2) = thisPose.z;

            global_scanContext.push_back(input_node.scanContext_);

            if(save_data_to_files)
            {
                string file_path = node_save_path;
                nodes[current_node_id].nodes_save_B(file_path);
            }

            last_node_id = current_node_id;
            current_node_id++;
            return ;
            ///////////////////////////////////////////////
        }
        else {
            Eigen::Vector3d currentNodePosition;
            currentNodePosition(0) = input_node.Global_Pose_.x;
            currentNodePosition(1) = input_node.Global_Pose_.y;
            currentNodePosition(2) = input_node.Global_Pose_.z;
            distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                            + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                            + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

            sc_current = input_node.scanContext_;
            double similarityScore = getScoreFromLastNode(sc_current, sc_last);

            PointType currentPose = input_node.Global_Pose_;

            double score = similarityScore;
            if(score>=buildMapSimScore) {
                int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                //relocal succeed!
                if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex)) {    
                    sc_last = sc_current;
                    return;
                }else {//relocalization failed!
                    global_scanContext.push_back(input_node.scanContext_);

                    nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                    input_node.Global_Pose_.intensity = mode;
                    nodes.push_back(input_node);
                    sc_last = input_node.scanContext_;

                    PointType thisPose;
                    thisPose.x = input_node.Global_Pose_.x;
                    thisPose.y = input_node.Global_Pose_.y;
                    thisPose.z = input_node.Global_Pose_.z;
                    globalKeyPose3d->push_back(thisPose);

                    PreviousNodePosition(0) = thisPose.x;
                    PreviousNodePosition(1) = thisPose.y;
                    PreviousNodePosition(2) = thisPose.z;

                    sc_last = sc_current;
                    if(save_data_to_files)
                    {
                        string file_path = node_save_path;
                        nodes[current_node_id].nodes_save_B(file_path);
                    }

                    last_node_id = current_node_id;
                    current_node_id++;
                    return ;
                }
            }
        }
    }

 };

void read_nodes_from_files_B(std::vector<node>& nodes, std::string& path, int preLoadSize = -1)
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
    if(index.size() > 0)
    {
        int node_number = index.size();
        if(preLoadSize != -1){
            node_number = preLoadSize;
        }
        nodes.resize(node_number);
        for(int i=0; i<node_number;++i)
        {
            nodes[i].create_node_from_file_B(path, index[i]);
        }
    }
}

vector<node> nodeList;
vector<pcl::PointCloud<pcl::PointXYZI>> cloudList;

void init(int preLoadCtrl = -1){
    string loadCloudPath = pathBase + "/cloudSparse/";
    std::vector<std::string>file_nameList;
    {
        std::vector<int>index;
        DIR *d = opendir(loadCloudPath.c_str());
        struct dirent *dp;
        while((dp = readdir(d)) != NULL)
        {
            if(dp->d_name[0] == '.')    {continue;}
            index.push_back(atoi(dp->d_name));
        }
        sort(index.begin(), index.end());
        for(auto n : index){
            stringstream ss;
            ss << n;
            file_nameList.push_back(loadCloudPath + ss.str()+".pcd");
        }
    }

    string loadNodePath = pathBase + "/nodeSparse/";
    read_nodes_from_files_B(nodeList, loadNodePath, preLoadCtrl);
    
    int cloudSizePre = file_nameList.size();
    if(preLoadCtrl != -1){
        cloudSizePre = preLoadCtrl;
        cout << "--------------pre load-------------------" << endl;
        cout << " pre load " << cloudSizePre * 100.0 / file_nameList.size() << "%" << endl; 
    } 
    cloudList.resize(cloudSizePre);
    //for(int i = 0; i < cloudSizePre; i++){
    //    if(i%10==0) cout << std::fixed << setprecision(1) << "load " << i * 100.0 / cloudSizePre << "% " << endl;
    //    pcl::io::loadPCDFile(file_nameList[i], cloudList[i]);
    //}
    cout << "all node " << nodeList.size() << " cloud size " << cloudList.size() << endl;
    cout << " get all node and cloud push any key to continue" << endl;
}

int nowThread = 0;
int nowIndex = 0;
int MaxThread = 0;

void buildS(double K_){
    double buildValue = K_;
    stringstream ss;
    ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
    string savePath = pathBase + "/buildS/S" + ss.str() + "/";
    if(0 == access(savePath.c_str(), 0)){
        nowThread--;
        return ;
    }else if( 0 != access(savePath.c_str(), 0)){
        mkdir(savePath.c_str(), 0777);
    }
    buildTopo buildTopoMap(savePath, buildValue, 15.0);
    int nodesSize = nodeList.size();
    for(int j = 0; j < nodesSize; j++){
        buildTopoMap.runS(nodeList[j], cloudList[j]);
    }
    vector<node>& resultNodeList = buildTopoMap.nodes;

    string fileResult = pathBase + "/buildS/S" + ss.str() + ".txt";
    ofstream fileout;
    fileout.open(fileResult);
    for(const node& n : resultNodeList){
        fileout << n.Global_Pose_.x << " "
                << n.Global_Pose_.y << " "
                << n.Global_Pose_.z << " " << endl;
    }
    fileout.close();
    string pcdFile = pathBase + "/buildS/S" + ss.str() + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    for(const node& n : resultNodeList){
        pcl::PointXYZ tempP;
        tempP.x = n.Global_Pose_.x;
        tempP.y = n.Global_Pose_.y;
        tempP.z = n.Global_Pose_.z;
        tempCloud.push_back(tempP);
    }
    pcl::io::savePCDFileASCII(pcdFile, tempCloud);
    nowThread--;
}



int main(int argc, char** argv){
    if(argv[1] != nullptr){
        pathBase = string(argv[1]);
        std::cout << "get pathBase " << pathBase << std::endl;
    }else{
        std::cerr << " main param error please check pathbase !" << std::endl;
        return 0;
    }
    if(argv[2] != nullptr){
        MaxThread = stoi(argv[2]);
        std::cout << "get thread num " << MaxThread << std::endl;
    }else{
        std::cerr << " main param error please check thread num !" << std::endl;
        return 0;
    }
    if(argv[3] != nullptr && string(argv[3]) == "true"){
        std::cout << " load pcd " << std::endl;
        loadPCD = true;
    }
    // pathBase = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL";
    // pathBase = "/home/qh/robot_ws/map/2021-08-30-18-06-30L";
    // MaxThread = 6;

    ros::init(argc, argv, "recallS");
    ros::NodeHandle nh("~");
    init();

    vector<double> simBuildTopoValue{
    0.60,  0.63,  0.65,  0.68,  0.70,  0.73,  0.75,  0.78,  0.55,  0.58,
    0.35,  0.38,  0.40,  0.43,  0.45,  0.48,  0.50,  0.53, 
    0.30,  0.33,  
    0.10,  0.13, 
    0.15,  0.18,  0.20,  0.23,  0.25,  0.28
    };

    ros::Rate loop(1);
    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < simBuildTopoValue.size()){
            cout << " add a new thread S " << simBuildTopoValue[nowIndex] << endl;
            nowThread++;
            thread* tempT(new thread(buildS, simBuildTopoValue[nowIndex]));
            nowIndex++;
        }
        if(nowIndex == simBuildTopoValue.size() && nowThread == 0)
        {
            break;
        }
        loop.sleep();
    }
    cout << " end " << endl;
    return 0;
}



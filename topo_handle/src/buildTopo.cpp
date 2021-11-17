#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"

#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"

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
    double buildMapDisThres;
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
    int loopClosureFromKDTreeDis(PointType currentPose, node& scNode, pcl::PointCloud<pcl::PointXYZI>& scCloud){
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, buildMapDisThres, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        if(pointSearchIndLoop.empty()) return -1;
        else return pointSearchIndLoop.front();
    }
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

        // cout << " ----now id------ " << currNode.id_ << endl;
        // cout << " ----search------ " << pointSearchIndLoop.size() << " points ";
        // for(auto n : pointSearchIndLoop){
        //     cout << nodes[n].id_ << " ";
        // }
        // cout << endl;
        // cout << " ----time now---- " << currNode.time_stamp_  << endl;

        for(int j  = 0; j < pointSearchIndLoop.size(); j++){
            int n = pointSearchIndLoop[j];
            //cout << "id " << nodes[n].id_ << endl;
            //cout << "time  " << nodes[n].time_stamp_  << endl;
            //cout << "time diff " << currNode.time_stamp_ - nodes[n].time_stamp_ << endl;
            if(currNode.time_stamp_ - nodes[n].time_stamp_ < 60.0){
                continue;
            }else{
                double scoreNow = node::getScore(nodes[n], currNode, currCloud);
                //cout << "---->time ok get id " << nodes[n].id_ << " score " << scoreNow << " dis " << sqrt(pointSearchSqDisLoop[j]) << endl;
                if(scoreNow < judgeLoopScore){
                    //cout << " get loop ! " << endl;
                    //getchar();
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
    buildTopo(std::string node_save_path_, double buildMapSimScore_, double buildMapDisThres_ = 15.0, double disForDetech_ = 20.0, double gnssSimThres = 0.35 )
    {
        ////////////////////
        kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
        pubGlobalNodePosition.reset(new pcl::PointCloud<PointType>());
        /////////////////////
        disForDetech = disForDetech_;
        judgeLoopScore = gnssSimThres;//描述子余弦距离
        buildMapSimScore = buildMapSimScore_;//todo
        buildMapDisThres = buildMapDisThres_;
        /////////////////////
        node_save_path = node_save_path_;
        save_data_to_files = true;//是否保存文件
        /////////////////////

    }
    void run(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud)
    {
        nowTime = input_node.time_stamp_;
        if(input_node.Global_Pose_.intensity == 1
         ||input_node.Global_Pose_.intensity == 0
         ||input_node.Global_Pose_.intensity == -1){
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
        }

        if(waitFlag){
            if(nowTime - startTime > 5.0){
                waitFlag = false;
                mode = gnssSig==2?false:true;
            }
        }else if(gnssSig==2){
            mode = false;
        }else if(gnssSig==1){
            mode = true;
        }

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
        }else {

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

            if(!mode) {//GNSS good
                if(distanceFromPreNode>=buildMapDisThres) {
                    int loopClosureIndex = loopClosureFromKDTreeDis(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                    //relocal succeed!
                    if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) ) {
                        PreviousNodePosition(0) = currentPose.x;
                        PreviousNodePosition(1) = currentPose.y;
                        PreviousNodePosition(2) = currentPose.z;
                        return;
                    }else {//relocalization failed!
                        //search sim node
                        int loopClosureIndexSim = loopClosureFromKDTree(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                        if(loopClosureIndexSim != -1 && nodeKeyIndex.count(loopClosureIndexSim) ) {
                            PreviousNodePosition(0) = currentPose.x;
                            PreviousNodePosition(1) = currentPose.y;
                            PreviousNodePosition(2) = currentPose.z;
                            return;
                        }
                        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(input_node.scanContext_);
                        std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
                        global_scanContext.push_back(input_node.scanContext_);

                        input_node.Global_Pose_.intensity = mode;
                        nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
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
            }else {
                double score = similarityScore;
                if(score>=buildMapSimScore) {
                    int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                    //relocal succeed!
                    if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) ) {
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

    void runD(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud){
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
        }else {
            Eigen::Vector3d currentNodePosition;
            currentNodePosition(0) = input_node.Global_Pose_.x;
            currentNodePosition(1) = input_node.Global_Pose_.y;
            currentNodePosition(2) = input_node.Global_Pose_.z;
            distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                            + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                            + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

            sc_current = input_node.scanContext_;

            PointType currentPose = input_node.Global_Pose_;

            if(distanceFromPreNode>=buildMapDisThres) {
                int loopClosureIndex = loopClosureFromKDTreeDis(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                //relocal succeed!
                if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) ) {
                    PreviousNodePosition(0) = currentPose.x;
                    PreviousNodePosition(1) = currentPose.y;
                    PreviousNodePosition(2) = currentPose.z;
                    return;
                }else {//relocalization failed!
                    //search sim node
                    int loopClosureIndexSim = loopClosureFromKDTree(currentPose, input_node, input_cloud);//返回值为在nodes中的索引
                    if(loopClosureIndexSim != -1 && nodeKeyIndex.count(loopClosureIndexSim) ) {
                        PreviousNodePosition(0) = currentPose.x;
                        PreviousNodePosition(1) = currentPose.y;
                        PreviousNodePosition(2) = currentPose.z;
                        return;
                    }

                    global_scanContext.push_back(input_node.scanContext_);

                    input_node.Global_Pose_.intensity = mode;
                    nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
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

bool NState = false;
bool DState = false;
bool SState = false;


void init(int preLoadCtrl = -1){
    string loadCloudPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/cloudSparse/";
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

    string loadNodePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/nodeSparse/";
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

//normal
int buildN(){
    vector<double> simBuildTopoValue{
        0.23,  0.25,  0.28,  0.30, 
        0.33,  0.35,  0.38,  0.40,
        0.43,  0.45,  0.48,  0.50,
        0.53,  0.55,  0.58,  0.60,
        0.63,  0.65,  0.68,  0.70,
        0.73,  0.75,  0.78,  0.80,
        0.83,  0.85,  0.88,  0.90,
        0.93,  0.95,  0.98
        };

    for(double k : simBuildTopoValue){
        double buildValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N" + ss.str() + "/";
        if(0 == access(savePath.c_str(), 0)){
            rmdir(savePath.c_str());
            mkdir(savePath.c_str(), 0777);
        }else if( 0 != access(savePath.c_str(), 0)){
            mkdir(savePath.c_str(), 0777);
        }
        buildTopo buildTopoMap(savePath, buildValue, 15.0, 15.0);
        int nodesSize = nodeList.size();
        // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=2){
            //char key = getchar();
            //if(key == 'q') exit(0);
            chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            // double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            // double nowPercent = j * 1.0 / nodesSize;
            // double restPercent = 1.0 - nowPercent;
            // double time_rest = time_used / nowPercent * restPercent;
            // cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.run(nodeList[j], cloudList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const node& n : resultNodeList){
            if(n.Global_Pose_.intensity == 1){
                nognss.push_back(n.id_);
                fileout << n.Global_Pose_.x << " "
                     << n.Global_Pose_.y << " "
                     << n.Global_Pose_.z << " " << 0 << endl;
            }else{
                gnss.push_back(n.id_);
                fileout << n.Global_Pose_.x << " "
                     << n.Global_Pose_.y << " "
                     << n.Global_Pose_.z << " " << 1 << endl;
            }
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "gnss " << gnss.size() << endl;
        for(const auto& n : gnss){
            fileout << n << " ";
        }
        fileout << endl;
        fileout << "nognss " << nognss.size() << endl;
        for(const auto& n : nognss){
            fileout << n << " ";
        }
        fileout.close();
    }
    NState = true;
}
//只按照相似度建图
int buildS(){
    vector<double> simBuildTopoValue{
        0.23,  0.25,  0.28,  0.30, 
        0.33,  0.35,  0.38,  0.40,
        0.43,  0.45,  0.48,  0.50,
        0.53,  0.55,  0.58,  0.60,
        0.63,  0.65,  0.68,  0.70,
        0.73,  0.75,  0.78,  0.80,
        0.83,  0.85,  0.88,  0.90,
        0.93,  0.95,  0.98
        };

    for(double k : simBuildTopoValue){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "/";
        if(0 == access(savePath.c_str(), 0)){
            rmdir(savePath.c_str());
            mkdir(savePath.c_str(), 0777);
        }else if( 0 != access(savePath.c_str(), 0)){
            mkdir(savePath.c_str(), 0777);
        }
        buildTopo buildTopoMap(savePath, buildValue, 0, 20.0, 0.35);
        int nodesSize = nodeList.size();
        // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=2){
            //char key = getchar();
            //if(key == 'q') exit(0);
            // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            // double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            // double nowPercent = j * 1.0 / nodesSize;
            // double restPercent = 1.0 - nowPercent;
            // double time_rest = time_used / nowPercent * restPercent;
            // cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.runS(nodeList[j], cloudList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const node& n : resultNodeList){
            fileout << n.Global_Pose_.x << " "
                    << n.Global_Pose_.y << " "
                    << n.Global_Pose_.z << " " << 0 << endl;
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "id: " << resultNodeList.size() << endl;
        for(const auto& n : resultNodeList){
            fileout << n.id_ << " ";
        }
        fileout.close();
    }
    SState = true;
}

//只按照固定距离建图
int buildD(){
    vector<double> tab{ 
        // 30.0,  25.0,  20.0,  15.0,  10.0,  5.0,
        // 27.5,  22.5,  17.5,  12.5,  7.50,  2.5,
        // 3.0,  1.0,  0.5
        1.5,  2.0,  3.5,  4.0,  4.5,  5.5,  6.0,  6.5,  7.0,  8.0,  8.5,  9.0
        };
    // vector<double> tab{ 30.0};

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "/";
        if(0 == access(savePath.c_str(), 0)){
            rmdir(savePath.c_str());
            mkdir(savePath.c_str(), 0777);
        }else if( 0 != access(savePath.c_str(), 0)){
            mkdir(savePath.c_str(), 0777);
        }
        buildTopo buildTopoMap(savePath, 0, buildValue, 20.0, 0.35);
        int nodesSize = nodeList.size();
        // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=2){
            //char key = getchar();
            //if(key == 'q') exit(0);
            // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            // double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            // double nowPercent = j * 1.0 / nodesSize;
            // double restPercent = 1.0 - nowPercent;
            // double time_rest = time_used / nowPercent * restPercent;
            // cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.runD(nodeList[j], cloudList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const auto& n : resultNodeList){
            fileout << n.Global_Pose_.x << " "
                    << n.Global_Pose_.y << " "
                    << n.Global_Pose_.z << " " << 0 << endl;
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "id: " << resultNodeList.size() << endl;
        for(const auto& n : resultNodeList){
            fileout << n.id_ << " ";
        }
        fileout.close();
    }
    DState = true;
}

//daquan
int main4(){
    for(int i = 0; i < 1; i++){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double k = 0.2;
        double buildValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/node/" + ss.str() + "/";
        buildTopo buildTopoMap(savePath, 0.2, 15.0, 16.0, 0.3);
        for(int j = 0; j < nodeList.size(); j++){
            //char key = getchar();
            //if(key == 'q') exit(0);
            if(nodeList[j].Global_Pose_.intensity==0) nodeList[j].Global_Pose_.intensity = 2;
            buildTopoMap.run(nodeList[j], cloudList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/node/" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(int i = 0; i < resultNodeList.size(); i++){
            node n = resultNodeList[i];
            if(n.Global_Pose_.intensity == 1){
                nognss.push_back(n.id_);
                fileout << n.Global_Pose_.x << " "
                        << n.Global_Pose_.y << " "
                        << n.Global_Pose_.z << " " << 0 << endl;
            }else{
                gnss.push_back(n.id_);
                fileout << n.Global_Pose_.x << " "
                        << n.Global_Pose_.y << " "
                        << n.Global_Pose_.z << " " << 1 << endl;
            }
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/node/" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "gnss " << gnss.size() << endl;
        for(const auto& n : gnss){
            fileout << n << " ";
        }
        fileout << endl;
        fileout << "nognss " << nognss.size() << endl;
        for(const auto& n : nognss){
            fileout << n << " ";
        }
        fileout.close();
    }
}

int main(int argc, char** argv){
    if(argv[1] == nullptr)  init(-1);
    else                    init(stoi(argv[1]));


    cout << " end " << endl;
    return 0;
}
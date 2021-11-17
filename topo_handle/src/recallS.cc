#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"
#include "thread"
#include "fstream"

#define _T_P 0
#define _F_P 1
#define _T_N 2
#define _F_P_F_N 3
#define _n_c_F_N 4

string pathBase;
bool loadPCD = false;

class recogPair{
public:
    int state = -1;//0-TP   1-FP    2-TN    3-FN
    PointType priorP;
    PointType nowP;
    
};

std::ofstream& operator<<(std::ofstream& fo, recogPair& pair){
    fo << pair.state << " " 
    << pair.nowP.x << " " << pair.nowP.y << " " << pair.nowP.z << " "
    << pair.priorP.x << " " << pair.priorP.y << " " << pair.priorP.z;
    return fo;
}
std::ifstream& operator>>(std::ifstream & fi, recogPair& pair){
    fi >> pair.state
    >> pair.nowP.x >> pair.nowP.y >> pair.nowP.z
    >> pair.priorP.x >> pair.priorP.y >> pair.priorP.z;
    return fi;
}

class recallTopo{
private:
    //////////////////////////////////////
    double truthDisThres = 0;
    double sameThres;
    double findCandidateDis;
    //////////////////////////////////////
    std::string node_load_path;
    //////////////////////////////////////
    vector<node>oldNodes;
    //////////////////////////////////////
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistroyKeyPoses;
    //////////////////////////////////////
    bool create_history_nodes{true};
    //////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalKeyPose3d;
    //////////////////////////////////////////////////////////
    void read_nodes_from_files_B(std::vector<node>& nodes, std::string& path)
    {
        std::vector<std::string>file_name;
        std::vector<int>index;
        DIR *d = opendir(path.c_str());
        struct dirent *dp;
        while((dp = readdir(d)) != nullptr)
        {
            if(dp->d_name[0] == '.')    {continue;}
            index.push_back(atoi(dp->d_name));
        }
        sort(index.begin(), index.end());
        closedir(d);
        int max_id_in_file = INT_MIN;
        int success_number = 0;
        int node_index = 0;
        if(!index.empty())
        {
            int node_number = index.size();
            nodes.resize(node_number);
            for(int i=0; i<node_number;++i)
            {
                node tmp_node;
                tmp_node.create_node_from_file_B(path, index[i]);
                nodes[node_index] = tmp_node;
                max_id_in_file = std::max(max_id_in_file, (int)tmp_node.id_);
                pcl::PointXYZ tmp_global_pose;

                tmp_global_pose.x = tmp_node.Global_Pose_.x;
                tmp_global_pose.y = tmp_node.Global_Pose_.y;
                tmp_global_pose.z = tmp_node.Global_Pose_.z;

                globalKeyPose3d->push_back(tmp_global_pose);

                PointType node_position;
                node_position = nodes[node_index].Global_Pose_;
                node_position.intensity = nodes[node_index].id_;

                success_number++;
                node_index++;
            }
        }
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
    }


    static bool comWithDistanceOldNode(PointType a, PointType&  b){
        return a.z < b.z;
    }

    void getCandidateSim(node& nowNode, vector<PointType>& tempSortKey, pcl::PointCloud<pcl::PointXYZI>& currCloud){
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        pcl::PointXYZ tempPoint;
        tempPoint.x = nowNode.Global_Pose_.x;
        tempPoint.y = nowNode.Global_Pose_.y;
        tempPoint.z = nowNode.Global_Pose_.z;
        kdtreeHistroyKeyPoses->radiusSearch(tempPoint, findCandidateDis, pointSearchIndLoop, pointSearchSqDisLoop);
        tempSortKey.clear();
        for(int n = 0; n < pointSearchIndLoop.size(); n++){
            PointType tempRes;
            tempRes.x = pointSearchIndLoop[n];
            tempRes.y = sqrt(pointSearchSqDisLoop[n]);
            float score = node::getScore(oldNodes[pointSearchIndLoop[n]], nowNode, currCloud);
            tempRes.z = score;
            tempSortKey.push_back(tempRes);
        }
        sort(tempSortKey.begin(), tempSortKey.end(), comWithDistanceOldNode);
    }

    void getTruth(node& nowNode, vector<PointType>& tempSortKey){
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        pcl::PointXYZ tempPoint;
        tempPoint.x = nowNode.Global_Pose_.x;
        tempPoint.y = nowNode.Global_Pose_.y;
        tempPoint.z = nowNode.Global_Pose_.z;
        kdtreeHistroyKeyPoses->radiusSearch(tempPoint, truthDisThres, pointSearchIndLoop, pointSearchSqDisLoop);
        tempSortKey.clear();
        tempSortKey.resize(pointSearchIndLoop.size());
        for(int n = 0; n < pointSearchIndLoop.size(); n++){
            tempSortKey[n].x = pointSearchIndLoop[n];
            tempSortKey[n].y = sqrt(pointSearchSqDisLoop[n]);
            tempSortKey[n].z = -1;
        }
    }


public:
    int TP_1 = 0, TP_N = 0;
    int TN_1 = 0, TN_N = 0;
    int FP_1 = 0, FP_N = 0;
    int FN_1 = 0, FN_N = 0;
    int ncFN = 0, FPFN = 0;
    vector<recogPair> pairList;
    recallTopo(std::string node_load_path_,
               double sameThres_,                   //Similarity threshold
               double truthDisThres_ = 3.0,         //Truth conditional distance
               double findCandidateDis_ = 20.0      //Candidate point distance
               )
    {
        ////////////////////
        kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        globalKeyPose3d.reset(new pcl::PointCloud<pcl::PointXYZ>());
        /////////////////////
        sameThres = sameThres_;//描述子余弦距离
        truthDisThres = truthDisThres_;
        findCandidateDis = findCandidateDis_;
        /////////////////////
        node_load_path = node_load_path_;   
        create_history_nodes = true;//是否从已有文件创建节点
        /////////////////////
        if(create_history_nodes)
        {
            string file_path = node_load_path;
            read_nodes_from_files_B(oldNodes, file_path);
            // cout << " get old node " << oldNodes.size() << endl;
        }
        /////////////////////
    }

private:
    
    void Invincible(node& nowNode, vector<PointType>&truth,vector<PointType>&sim)
    {
	    //判断P（针对“当前节点与sim.front()这条识别出来的连线” 来说）
        if(!sim.empty() && sim.front().z < sameThres){  //这个P
            if(!truth.empty() && sim.front().x == truth.front().x)  //是对的
            {
                TP_1++;
                recogPair tempP;
                tempP.state = _T_P;
                tempP.priorP = oldNodes[(int)sim.front().x].Global_Pose_;
                tempP.nowP = nowNode.Global_Pose_;
                pairList.push_back(tempP);
            }
            else{  //是错的
                FP_1++;
                recogPair tempP;
                tempP.state = _F_P;
                tempP.priorP = oldNodes[(int)sim.front().x].Global_Pose_;
                tempP.nowP = nowNode.Global_Pose_;
                pairList.push_back(tempP);
            }
        }
        //判断N
        if(!truth.empty()){ //当前节点有对应真值，那么“当前节点与truth.front()这条连线”应该是有的。如果没有连，则是错误的“没连”
            if(!sim.empty() && sim.front().z < sameThres && sim.front().x != truth.front().x){ //情况1，与别的先验节点连上了
                FN_1++; 
                FPFN++;
                recogPair tempP;
                tempP.state = _F_P_F_N;
                tempP.priorP = oldNodes[(int)sim.front().x].Global_Pose_;
                tempP.nowP = nowNode.Global_Pose_;
                pairList.push_back(tempP);
            }
            else if(!sim.empty() && sim.front().z > sameThres || sim.empty()){ //情况2，谁都没连
                FN_1++;
                ncFN++; 
                recogPair tempP;
                tempP.state = _n_c_F_N;
                tempP.priorP = oldNodes[(int)truth.front().x].Global_Pose_;
                tempP.nowP = nowNode.Global_Pose_;
                pairList.push_back(tempP);
            }
        }
        else{ //当前节点没有对应真值，即“当前节点就不应该连线”
            if(!sim.empty() && sim.front().z > sameThres || sim.empty()){  //如果确实没连线
                TN_1++;  //正确的“没连”
                recogPair tempP;
                tempP.state = _T_N;
                tempP.nowP = nowNode.Global_Pose_;
                pairList.push_back(tempP);
            }
        }
    }


public:
    void run(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud){

        //truth point search in truth distance
        vector<PointType> resultDis;//first index second dis
        getTruth(input_node, resultDis);

        //sim search for candidate Point to find Most similar TOP1/TOP10 for Accuracy and fast search 50m
        vector<PointType> resultSim;//first index second dis
        getCandidateSim(input_node, resultSim, input_cloud);

        Invincible(input_node, resultDis,resultSim);
   }
};

void read_nodes_from_files_B(std::vector<node>& nodes, std::string& path)
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
    int max_id_in_file = INT_MIN;
    int success_number = 0;
    int node_index = 0;
    if(index.size() > 0)
    {
        int node_number = index.size();
        nodes.resize(node_number);

        for(int i=0; i<node_number;++i)
        {
            node tmp_node;
            tmp_node.create_node_from_file_B(path, index[i]);
            nodes[node_index] = tmp_node;
            max_id_in_file = std::max(max_id_in_file, (int)tmp_node.id_);

            success_number++;
            node_index++;
        }
    }
}

vector<node> nodeList1;
vector<pcl::PointCloud<pcl::PointXYZI>> cloudList1;

void init(){
    string sourceNodePath1 = pathBase + "/nodeSparse/";
    read_nodes_from_files_B(nodeList1, sourceNodePath1);

    string loadCloudPath1 = pathBase + "/cloudSparse/";
    std::vector<std::string>file_nameList1;
    {
        std::vector<int>index;
        DIR *d = opendir(loadCloudPath1.c_str());
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
            file_nameList1.push_back(loadCloudPath1 + ss.str()+".pcd");
        }
    }
    cloudList1.resize(file_nameList1.size());
    int cloudSize = file_nameList1.size();
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(int n = 0; n < file_nameList1.size(); n++){
        // pcl::io::loadPCDFile(file_nameList1[n], cloudList1[n]);
        // if(n % 50 == 0){
        //     cout << " load " << n * 100.0 / cloudSize << "% " << endl;
        // }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    cout << "time " << time_used << " s" << endl;
    cout << " cloud size" << file_nameList1.size() << " node size " << nodeList1.size() << endl;
   
}

int nowThread = 0;
int nowIndex = 0;
int MaxThread = 0;

void callBack(double K_, double thres_){
    double buildValue = K_;
    stringstream ss;
    ss << std::fixed << setprecision(2) << buildValue;
    string loadPath = pathBase + "/buildS/S" + ss.str() + "/";
    recallTopo recallTopoMap(loadPath, thres_);
    int nodesSize = nodeList1.size();

    stringstream ss2;
    ss2 << fixed << setprecision(2) << "S" << thres_ << "R";  
    string nowDir = pathBase + "/recogS/" + ss2.str();
    if( 0 != access(nowDir.c_str(), 0) ){
        mkdir(nowDir.c_str(), 0777);
    }
    string fileResult = nowDir + "/S" + ss.str() + "RecallInfo.txt";
    if( 0 == access(fileResult.c_str(), 0) ){
        nowThread--;
        return;
    }

    for(int j = 0; j < nodesSize; j+=10){
        recallTopoMap.run(nodeList1[j], cloudList1[j]);
    }

    ofstream fileout;
    fileout.open(fileResult);
    int TP1 = recallTopoMap.TP_1; int TP10 = recallTopoMap.TP_N;
    int FP1 = recallTopoMap.FP_1; int FP10 = recallTopoMap.FP_N;
    int TN1 = recallTopoMap.TN_1; int TN10 = recallTopoMap.TN_N;
    int FN1 = recallTopoMap.FN_1; int FN10 = recallTopoMap.FN_N;
    double T1acc = ( TP1 + TN1 ) * 1.0 / ( TP1 + TN1 + FP1 + FN1 ) * 1.0;
    double T10acc = ( TP10 + TN10 ) * 1.0 / ( TP10 + TN10 + FP10 + FN10 ) * 1.0;
    double T1pre = ( TP1 ) * 1.0 / ( TP1 + FP1 ) * 1.0;
    double T10pre = ( TP10 ) * 1.0 / ( TP10 + FP10 ) * 1.0;
    double T1recall = ( TP1 ) * 1.0 / ( TP1 + FN1 ) * 1.0;
    double T10recall = ( TP10 ) * 1.0 / ( TP10 + FN10 ) * 1.0;
    fileout << "TP: " << TP1 << endl;
    fileout << "FP: " << FP1 << endl;
    fileout << "TN: " << TN1 << endl;
    fileout << "FN: " << FN1 << endl;
    fileout << "FPFN: " << recallTopoMap.FPFN << endl;
    fileout << "ncFN: " << recallTopoMap.ncFN << endl;
    fileout << "accuracy: " << (T1acc)  << endl;
    fileout << "precision: " << (T1pre) << endl;
    fileout << "recall: " << (T1recall) << endl;
    fileout << "FPFN: " << (recallTopoMap.FPFN) << endl;
    fileout << "ncFN: " << (recallTopoMap.ncFN) << endl;
    for(auto n : recallTopoMap.pairList){
        fileout << n;
        fileout << endl;
    }
    fileout.close();
    nowThread--;
}

bool comZA(pair<double,double>A, pair<double,double>B){
    return ((A.first*A.second)>(B.first*B.second));
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

    vector<double> thresTab{
        0.90,  0.80,  0.70,  0.60,  0.50,  0.42,  0.38,  0.33,  0.35,  0.30,  0.28,  0.25,  0.23,  0.15,  0.10
    };

    vector<double> buildValue{
        0.60,  0.63,  0.65,  0.68,  0.70,  0.73,  0.75,  0.78,  0.55,  0.58,
        0.35,  0.38,  0.40,  0.43,  0.45,  0.48,  0.50,  0.53, 
        0.30,  0.33,  
        0.10,  0.13, 
        0.15,  0.18,  0.20,  0.23,  0.25,  0.28
        };


    vector<pair<double, double>> paramList;

    for(int i = 0; i < thresTab.size(); i++){
        for(int j = 0; j < buildValue.size(); j++){
            paramList.push_back( {thresTab[i], buildValue[j]} );
        }
    }

    sort(paramList.begin(), paramList.end(), comZA);
    for(const auto& n : paramList){
        cout << n.second << " " << n.first << endl;
    }

    ros::Rate loop(1);

    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < paramList.size()){
            cout << nowIndex << " add a new thread S " << paramList[nowIndex].second << " thres " << paramList[nowIndex].first << endl;
            nowThread++;
            thread* tempT(new thread(callBack, paramList[nowIndex].second, paramList[nowIndex].first));
            nowIndex++;
        }
        if(nowIndex == paramList.size() && nowThread == 0)
        {
            cout << " end " << endl;
            break;
        }
        loop.sleep();
    }

    return 0;

    double R,P,Y;
    double QX,QY,QZ,QW;
    tf::Quaternion tempQ = tf::createQuaternionFromRPY(R,P,Y);
}

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
    std::vector<Eigen::MatrixXd>global_scanContext;                 //存储全局描述子
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalKeyPose3d;
    //////////////////////////////////////////////////////////
    unordered_map<int, int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
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
                nodeKeyIndex.insert({node_index, tmp_node.id_});    //index node id
                max_id_in_file = std::max(max_id_in_file, (int)tmp_node.id_);
                pcl::PointXYZ tmp_global_pose;

                tmp_global_pose.x = tmp_node.Global_Pose_.x;
                tmp_global_pose.y = tmp_node.Global_Pose_.y;
                tmp_global_pose.z = tmp_node.Global_Pose_.z;

                globalKeyPose3d->push_back(tmp_global_pose);        //存储全局地图
                global_scanContext.push_back(nodes[node_index].scanContext_); //存储拓扑节点中的描述子

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
               double sameThres_,         //Similarity threshold
               double truthDisThres_ = 3.0,               //Truth conditional distance
               double findCandidateDis_ = 50.0    //Candidate point distance
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
    vector<PointType> whitchIsSame(vector<PointType>& candidate) const{
        vector<PointType> result;
        for(PointType n : candidate){
            if( n.z < sameThres )
                result.push_back(n);
        }
        return result;
    }
    vector<PointType> whichIsTruth(vector<PointType>& candidate) const{
        vector<PointType> result;
        for(PointType n : candidate){
            if(n.y < truthDisThres)
                result.push_back(n);
        }
        return result;
    }
    
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


    void top1Judge(vector<PointType>&truth,vector<PointType>&sim){
        if(!sim.empty() && sim.front().z < sameThres){
            if(!truth.empty() && sim.front().x == truth.front().x)
            {
                TP_1++;
            }
            else{
                FP_1++;
            }
        }
        else{
            if(!truth.empty()){
                FN_1++;
            }
            else{
                TN_1++;
            }
        }
    }

    void topNJudge(vector<PointType>&truth,vector<PointType>&sim,int N = 10){
        vector<PointType> topN;
        for(int i = 0; i < N && i < sim.size(); i++){
            topN.push_back(sim[i]);
        }
        vector<PointType> simN = whitchIsSame(topN);// top10中相似度满足余弦距离阈值的
        vector<PointType> truthN = whichIsTruth(simN);// top10中相似度满足余弦距离阈值，也满足真值距离的
        //阳，仅相似度判断，相似度TOP1的点满足相似度阈值条件
        if(!simN.empty()){
            if(!truthN.empty() && truthN.front().x == simN.front().x)
            {
                TP_N++;
            }
            else{
                FP_N++;
            }
        }
        //阴，仅相似度判断，相似度TOP1的点不满足相似度阈值条件
        else{
            if(!truth.empty()){
                FN_N++;
            }
            else{
                TN_N++;
            }
        }

    }


public:
    void run(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud){

        // cout << " get new node ! " << endl;
        //truth point search in truth distance
        vector<PointType> resultDis;//first index second dis
        getTruth(input_node, resultDis);
        // cout << " truth search " << resultDis.size() << endl;

        //sim search for candidate Point to find Most similar TOP1/TOP10 for Accuracy and fast search 50m
        vector<PointType> resultSim;//first index second dis
        getCandidateSim(input_node, resultSim, input_cloud);
        // cout << " sim search " << resultSim.size() << endl;

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
    string sourceNodePath1 = "/home/robot/dlut/map/2021-08-30-16-12-25L/nodeSparse/";
    read_nodes_from_files_B(nodeList1, sourceNodePath1);

    string loadCloudPath1 = "/home/robot/dlut/map/2021-08-30-16-12-25L/cloudSparse/";
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
        if(n % 50 == 0){
            cout << " load " << n * 100.0 / cloudSize << "% " << endl;
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    cout << "time " << time_used << " s" << endl;
    cout << " cloud size 2 " << file_nameList1.size() << " node size 2 " << nodeList1.size() << endl;
   
}


int nowThread = 0;
int nowIndex = 0;
int MaxThread = 12;

void disCallBack1(double K_, double thres_){
    double buildValue = K_;
    stringstream ss;
    ss << std::fixed << setprecision(2) << buildValue;
    string loadPath = "/home/robot/dlut/map/2021-08-30-18-06-30L/buildN/N" + ss.str() + "/";
    recallTopo recallTopoMap(loadPath, thres_);
    int nodesSize = nodeList1.size();

    stringstream ss2;
    ss2 << fixed << setprecision(2) << "N" << thres_ << "R";  
    string nowDir = "/home/robot/dlut/map/2021-08-30-16-12-25L/recogN/" + ss2.str();
    if( 0 != access(nowDir.c_str(), 0) ){
        mkdir(nowDir.c_str(), 0777);
    }
    string fileResult = nowDir + "/N" + ss.str() + "RecallInfo.txt";
    if( 0 == access(fileResult.c_str(), 0) ){
        return;
    }

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(int j = 0; j < nodesSize; j+=10){
        //char key = getchar();
        //if(key == 'q') exit(0);
        // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        // double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
        // double nowPercent = j * 1.0 / nodesSize;
        // double restPercent = 1.0 - nowPercent;
        // double time_rest = time_used / nowPercent * restPercent;
        // std::cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s " << std::endl;
        // recallTopoMap.run(nodeList1[j], cloudList1[j]);
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

bool comparePair(pair<double,double>A, pair<double,double>B){
    return ((A.first*A.second)>(B.first*B.second));
}

void test(){
    usleep(200000);
    nowThread--;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "recallD");
    ros::NodeHandle nh("~");
    
    init();

    vector<double> thresTab{
        0.90,  0.80,  0.70,  0.60,  0.50,  0.42,  0.38,  0.33,  0.35,  0.30,  0.28,  0.25,  0.23,  0.15,  0.10
    };

    vector<double> buildValue{
        0.50,  0.53,  0.55,  0.58,  
        0.60,  0.63,  0.65,  0.68,
        0.70,  0.73,  0.75,  0.78,
        0.80,  0.83,  0.85,  0.88,
        0.90
        };


    vector<pair<double, double>> paramList;

    for(int i = 0; i < thresTab.size(); i++){
        for(int j = 0; j < buildValue.size(); j++){
            paramList.push_back( {thresTab[i], buildValue[j]} );
        }
    }

    sort(paramList.begin(), paramList.end(), comparePair);
    for(const auto& n : paramList){
        cout << n.second << " " << n.first << endl;
    }

    ros::Rate loop(1);

    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < paramList.size()){
            cout << nowIndex << " add a new thread D " << paramList[nowIndex].second << " thres " << paramList[nowIndex].first << endl;
            nowThread++;
            // thread* tempT(new thread(test));
            thread* tempT(new thread(disCallBack1, paramList[nowIndex].second, paramList[nowIndex].first));
            nowIndex++;
        }
        if(nowIndex == paramList.size() && nowThread == 0)
        {
            cout << " endl " << endl;
            break;
        }
        loop.sleep();
    }

    return 0;
}

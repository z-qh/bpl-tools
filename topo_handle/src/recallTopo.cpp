#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"
#include "thread"

#define _T_P 11
#define _F_P 01
#define _T_N 10
#define _F_N 00

using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;;

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
            //opt here to save time acctully the result is same as not 
            if(score < sameThres){
                break;
            }
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
        sort(tempSortKey.begin(), tempSortKey.end(), comWithDistanceOldNode);
    }


public:
    int TP_1 = 0, TP_N = 0;
    int TN_1 = 0, TN_N = 0;
    int FP_1 = 0, FP_N = 0;
    int FN_1 = 0, FN_N = 0;
    recallTopo(std::string node_load_path_,
               double sameThres_,         //Similarity threshold
               double truthDisThres_ = 5.0,               //Truth conditional distance
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
void Invincible(vector<PointType>&truth,vector<PointType>&sim)
    {
	    //判断P（针对“当前节点与sim.front()这条识别出来的连线” 来说）
        if(!sim.empty() && sim.front().z < sameThres){  //这个P
            if(!truth.empty() && sim.front().x == truth.front().x)  //是对的
            {
                TP_1++;
            }
            else{  //是错的
                FP_1++;
            }
        }
        //判断N
        if(!truth.empty()){ //当前节点有对应真值，那么“当前节点与truth.front()这条连线”应该是有的。如果没有连，则是错误的“没连”
            if(!sim.empty() && sim.front().z < sameThres && sim.front().x != truth.front().x){ //情况1，与别的先验节点连上了
                FN_1++; 
            }
            else if(!sim.empty() && sim.front().z > sameThres || sim.empty()){ //情况2，谁都没连
                FN_1++; 
            }
        }
        else{ //当前节点没有对应真值，即“当前节点就不应该连线”
            if(!sim.empty() && sim.front().z > sameThres || sim.empty()){  //如果确实没连线
                TN_1++;  //正确的“没连”
            }
        }
    }
    void top1Judge(vector<PointType>&truth,vector<PointType>&sim){
        //阳，仅相似度判断，相似度TOP1的点满足相似度阈值条件
        if(sim.front().z < sameThres){
            //真，仅距离判断，TOP1的点小于真值距离，真阳TP+1
            if(sim.front().y < truthDisThres)
            {
                TP_1++;
            }
            //假，仅距离判断，TOP1的点大于真值距离，假阳FP+1
            else{
                FP_1++;
            }
        }
        //阴，仅相似度判断，相似度TOP1的点不满足相似度阈值条件
        else{
            //真，仅距离判断，真值距离内有先验拓扑点，真阴TN+1
            if(!truth.empty()){
                FN_1++;
            }
            //假，仅距离判断，真值距离内无先验拓扑点，假阴FN+1
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
            //真，仅距离判断，TOP1的点小于真值距离，真阳TP+1
            if(!truthN.empty())
            {
                TP_N++;
            }
            //假，仅距离判断，TOP1的点大于真值距离，假阳FP+1
            else{
                FP_N++;
            }
        }
        //阴，仅相似度判断，相似度TOP1的点不满足相似度阈值条件
        else{
            //真，仅距离判断，真值距离内有先验拓扑点，真阴TN+1
            if(!truth.empty()){
                FN_N++;
            }
            //假，仅距离判断，真值距离内无先验拓扑点，假阴FN+1
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

        Invincible(resultDis,resultSim);
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
vector<node> nodeList3;
vector<pcl::PointCloud<pcl::PointXYZI>> cloudList1;
vector<pcl::PointCloud<pcl::PointXYZI>> cloudList3;

void init(){
    string sourceNodePath1 = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/nodeSparse/";
    string sourceNodePath3 = "/home/qh/robot_ws/map/2021-08-30-19-17-22L/nodeSparse/";
    read_nodes_from_files_B(nodeList1, sourceNodePath1);
    read_nodes_from_files_B(nodeList3, sourceNodePath3);


    string loadCloudPath1 = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/cloudSparse/";
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

    string loadCloudPath3 = "/home/qh/robot_ws/map/2021-08-30-19-17-22L/cloudSparse/";
    std::vector<std::string>file_nameList3;
    {
        std::vector<int>index;
        DIR *d = opendir(loadCloudPath3.c_str());
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
            file_nameList3.push_back(loadCloudPath3 + ss.str()+".pcd");
        }
    }
    cloudList1.resize(file_nameList1.size());
    cloudList3.resize(file_nameList3.size());

    for(int n = 0; n < file_nameList1.size(); n++){
        //pcl::io::loadPCDFile(s, cloudList1[n]);
    }
    for(int n = 0; n < file_nameList3.size(); n++){
        //pcl::io::loadPCDFile(s, cloudList3[n]);
    }

    cout << " cloud size 1 " << file_nameList1.size() << " node size 1 " << nodeList1.size() << endl;
    cout << " cloud size 3 " << file_nameList3.size() << " node size 3 " << nodeList3.size() << endl;
    
}

bool D1State = false;
bool D3State = false;
bool S1State = false;
bool S3State = false;
bool N1State = false;
bool N3State = false;

void disCallBack1(){
    vector<double> tab{ 
        30.0,  25.0,  20.0,  15.0,  10.0,  5.0,
        27.5,  22.5,  17.5,  12.5,  7.50,  2.5,
        3.0,  1.0,  0.5
        };

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.35, 5.0, 50.0+buildValue);
        int nodesSize = nodeList1.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList1[j], cloudList1[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/D" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    D1State = true;
}

void disCallBack3(){
    vector<double> tab{ 
        30.0,  25.0,  20.0,  15.0,  10.0,  5.0,
        27.5,  22.5,  17.5,  12.5,  7.50,  2.5,
        3.0,  1.0,  0.5
        };

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.35, 5.0, 50.0+buildValue);
        int nodesSize = nodeList3.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList3[j], cloudList3[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-19-17-22L/node/D" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    D3State = true;
}

void normalCallBack1(){
    string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N0.25/";
    vector<double> simRecallTopoValue{
        //0.23,  0.25,  0.28,  0.30, 
        //0.33,  0.35,  
        0.38,  0.40,
        0.43,  0.45,  0.48,  0.50,
        0.53,  0.55,  0.58,  0.60,
        0.63,  0.65,  0.68,  0.70,
        0.73,  0.75,  0.78,  0.80,
        0.83,  0.85,  0.88,  0.90,
        0.93,  0.95,  0.98
    };
    for(double k : simRecallTopoValue){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double recallValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << recallValue;
        recallTopo recallTopoMap(loadPath, recallValue);
        int nodesSize = nodeList1.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList1[j], cloudList1[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/N" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    N1State = true;
}

void normalCallBack3(){
    string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N0.25/";
    vector<double> simRecallTopoValue{
            0.23,  0.25,  0.28,  0.30,
            0.33,  0.35,  0.38,  0.40
    };
    for(double k : simRecallTopoValue){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double recallValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << recallValue;
        recallTopo recallTopoMap(loadPath, recallValue);
        int nodesSize = nodeList3.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList3[j], cloudList3[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-19-17-22L/node/F" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    N3State = true;
}

void simCallBack1(){
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
        double callBackValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << callBackValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.35);
        int nodesSize = nodeList1.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList1[j], cloudList1[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/S" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    S1State = true;
}

void simCallBack3(){
    vector<double> simBuildTopoValue{
            0.23,  0.25,  0.28,  0.30,
            0.33,  0.35,  0.38,  0.40,
            0.43,  0.45,  0.48,  0.50,
            0.53,     0.55,  0.58,  0.60,
            0.63,  0.65,  0.68,  0.70,
            0.73,  0.75,  0.78,  0.80,
            0.83,  0.85,  0.88,  0.90,
            0.93,  0.95,  0.98
    };
    for(double k : simBuildTopoValue){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double callBackValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << callBackValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.35);
        int nodesSize = nodeList3.size();
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j+=3){
            //char key = getchar();
            //if(key == 'q') exit(0);
            //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            //double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            //double nowPercent = j * 1.0 / nodesSize;
            //double restPercent = 1.0 - nowPercent;
            //double time_rest = time_used / nowPercent * restPercent;
            //cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            recallTopoMap.run(nodeList3[j], cloudList3[j]);
        }

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-19-17-22L/node/S" + ss.str() + "RecallInfo.txt";
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
        fileout << "TP1: " << TP1 << " TP10: " << TP10 << endl;
        fileout << "FP1: " << FP1 << " FP10: " << FP10 << endl;
        fileout << "TN1: " << TN1 << " TN10: " << TN10 << endl;
        fileout << "FN1: " << FN1 << " FN10: " << FN10 << endl;
        fileout << "Top1-accuracy: " << (T1acc) << " Top10-accuracy: " << (T10acc) << endl;
        fileout << "Top1-precision: " << (T1pre) << " Top10-precision: " << (T10pre) << endl;
        fileout << "Top1-recall: " << (T1recall) << " Top10-recall: " << (T10recall) << endl;
        fileout.close();
    }
    S3State = true;
}

int main(int argc, char** argv){
    init();
    // thread recallD1(disCallBack1);
    // thread recallD3(disCallBack3);

    // thread recallS1(simCallBack1);
    // thread recallS3(simCallBack3);

    // recallD1.detach();
    // recallD3.detach();

    // recallS1.detach();
    // recallS3.detach();

    char k = 0;
    while(!N1State || !N3State || !S1State || !S3State || !D1State || !D3State){
        k = getchar();
        cout << "N1 " << N1State << endl;
        cout << "Ne " << N3State << endl;
        cout << "D1 " << D1State << endl;
        cout << "D3 " << D3State << endl;
        cout << "S1 " << S1State << endl;
        cout << "S3 " << S3State << endl;
    }

    return 0;
}

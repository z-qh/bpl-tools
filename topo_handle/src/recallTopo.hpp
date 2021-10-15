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
        tempSortKey.resize(pointSearchIndLoop.size());
        for(int n = 0; n < pointSearchIndLoop.size(); n++){
            tempSortKey[n].x = pointSearchIndLoop[n];
            tempSortKey[n].y = sqrt(pointSearchSqDisLoop[n]);
            float score = node::getScore(oldNodes[pointSearchIndLoop[n]], nowNode, currCloud);
            tempSortKey[n].z = score;
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
                TN_1++;
            }
            //假，仅距离判断，真值距离内无先验拓扑点，假阴FN+1
            else{
                FN_1++;
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
                TN_N++;
            }
            //假，仅距离判断，真值距离内无先验拓扑点，假阴FN+1
            else{
                FN_N++;
            }
        }

    }


public:
    void run(node& input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud){

        //truth point search in truth distance
        vector<PointType> resultDis;//first index second dis
        getTruth(input_node, resultDis);


        //sim search for candidate Point to find Most similar TOP1/TOP10 for Accuracy and fast search 50m
        vector<PointType> resultSim;//first index second dis
        getCandidateSim(input_node, resultSim, input_cloud);

        top1Judge(resultDis,resultSim);
        topNJudge(resultDis,resultSim);
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
    for(const auto& s : file_nameList1){
        pcl::PointCloud<pcl::PointXYZI>tempCloud;
        pcl::io::loadPCDFile(s, tempCloud);
        cloudList1.push_back(tempCloud);
    }
    for(const auto& s : file_nameList3){
        pcl::PointCloud<pcl::PointXYZI>tempCloud;
        pcl::io::loadPCDFile(s, tempCloud);
        cloudList3.push_back(tempCloud);
    }
}

bool D1State = false;
bool D3State = false;
bool S1State = false;
bool S3State = false;
bool N1State = false;
bool N3State = false;

void disCallBack1(){
    vector<double> tab{0.5, 1.0, 3.0, 10.0, 15.0, 30.0};

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node2/D" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.30);
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

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node2/D" + ss.str() + "RecallInfo.txt";
        ofstream fileout;
        fileout.open(fileResult);
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
        fileout.close();
    }
    D1State = true;
}

void disCallBack3(){
    vector<double> tab{0.5, 1.0, 3.0, 10.0, 15.0, 30.0};

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string loadPath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "/";
        recallTopo recallTopoMap(loadPath, 0.30);
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
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
        fileout.close();
    }
    D3State = true;
}

void normalCallBack1(){
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

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-16-12-25L/node/F" + ss.str() + "RecallInfo.txt";
        ofstream fileout;
        fileout.open(fileResult);
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
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
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
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
        recallTopo recallTopoMap(loadPath, 0.30);
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
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
        fileout.close();
    }
    S1State = true;
}

void simCallBack3(){
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
        recallTopo recallTopoMap(loadPath, 0.30);
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
        fileout << "TP1 : " << recallTopoMap.TP_1 << "TP10 : " << recallTopoMap.TP_N << endl;
        fileout << "FP1 : " << recallTopoMap.FP_1 << "FP10 : " << recallTopoMap.FP_N << endl;
        fileout << "TN1 : " << recallTopoMap.TN_1 << "TN10 : " << recallTopoMap.TN_N << endl;
        fileout << "FN1 : " << recallTopoMap.FN_1 << "FN10 : " << recallTopoMap.FN_N << endl;
        fileout.close();
    }
    S3State = true;
}

int main(int argc, char** argv){
    init();
    thread recallD1(disCallBack1);
    thread recallD3(disCallBack3);
    thread recallN1(normalCallBack1);
    thread recallN3(normalCallBack3);
    thread recallS1(simCallBack1);
    thread recallS3(simCallBack3);

    recallD1.detach();
    recallD3.detach();
    recallN1.detach();
    recallN3.detach();
    recallS1.detach();
    recallS3.detach();

    char k = 0;
    while(!N1State || !N3State || !S1State || !S3State || !D1State || !D3State){
        k = getchar();
        cout << k << endl;
        cout << "N1 " << N1State << endl;
        cout << "Ne " << N3State << endl;
        cout << "D1 " << D1State << endl;
        cout << "D3 " << D3State << endl;
        cout << "S1 " << S1State << endl;
        cout << "S3 " << S3State << endl;
    }

    return 0;
}

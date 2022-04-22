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

mutex mutMap;

map<pair<int,int>, double> allMap1;
map<pair<int,int>, double> allMap2;
map<pair<int,int>, double> allMap3;
map<pair<int,int>, double> allMapo1;
map<pair<int,int>, double> allMapo2;

void loadMap(map<pair<int,int>, double>& buff, string filePath){
    ifstream file(filePath);
    if(!file.is_open()) return;
    while(file.good()){
        string info;
        getline(file, info);
        if(info.empty()) continue;
        istringstream iss(info);
        pair<int,int> source2now;
        double score;
        iss >> source2now.first >> source2now.second >> score;
        buff.insert(make_pair(source2now, score));
    }
    file.close();
    cout << "load buff " << buff.size() << endl;
}

void saveMap(map<pair<int,int>, double>& buff, string filePath){
    ofstream file(filePath);
    if(buff.empty()){
        file.close();
        return;
    }
    for(auto& p : buff){
        file << p.first.first << " " << p.first.second << " " << p.second << endl;
    }
    file.close();
    cout << "save buff " << buff.size() << endl;
}

using namespace std;

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

class myRecog{
public:
    static void getNewJudge(vector<recogPair>&result, node& old, node& now, int state){
        recogPair tmp;
        tmp.state = state;
        tmp.nowP = now.Global_Pose_;
        tmp.priorP = old.Global_Pose_;
        result.push_back(tmp);
    }
    static void read_nodes_from_files_B(string& path,
                                        vector<node>& nodes,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr Pose3d,
                                        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
                                        int start=-1, int end=-1, int pick=1)
    {
        vector<string>file_name;
        vector<int>index;
        DIR *d = opendir(path.c_str());
        struct dirent *dp;
        while((dp = readdir(d)) != nullptr)
        {
            if(dp->d_name[0] == '.')    {continue;}
            index.push_back(atoi(dp->d_name));
        }
        sort(index.begin(), index.end());
        closedir(d);
        int node_index = 0;
        int node_number = index.size();
        start = (start==-1?0:start);
        end = (end==-1?node_number:end);
        if(!index.empty())
        {
            for(int i=0; i<node_number;i+=pick)
            {
                if(i < start || i > end ) continue;
                node tmp_node;
                tmp_node.create_node_from_file_B(path, index[i]);
                nodes.push_back(tmp_node);
                pcl::PointXYZ tmp_global_pose;

                tmp_global_pose.x = tmp_node.Global_Pose_.x;
                tmp_global_pose.y = tmp_node.Global_Pose_.y;
                tmp_global_pose.z = tmp_node.Global_Pose_.z;

                Pose3d->push_back(tmp_global_pose);
                node_index++;
            }
        }
        kdtree->setInputCloud(Pose3d);
        cout << "get " << nodes.size() << " nodes" << endl;
        cout << "first " << start << " " << nodes.front().cloudPath_ << endl;
        cout << "end   " << end << " " << nodes.back().cloudPath_ << endl;
    }
private:
    //////////////////////////////////////
    double truthDisThres;
    double simThres;
    double disCandidate;
    //////////////////////////////////////
    std::string node_load_path;
    vector<node>oldNodes;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeOldPoses;
    pcl::PointCloud<pcl::PointXYZ>::Ptr oldPose3d;
    //////////////////////////////////////
    std::string node_recog_Path;
    vector<node>recogNodes;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeRecogPoses;
    pcl::PointCloud<pcl::PointXYZ>::Ptr recogPose3d;
    //////////////////////////////////////

    //////////////////////////////////////////////////////////
    vector<pair<int,int>> matchPairs;// piror - recog
    vector<int> matchFlags;//1 truth 2 closest false
    void getMatchPair(){
        for(int recogJ = 0; recogJ < recogNodes.size(); recogJ++){
            node& recogNode = recogNodes[recogJ];
            pcl::PointXYZ recogPosi;
            recogPosi.x = recogNode.Global_Pose_.x;
            recogPosi.y = recogNode.Global_Pose_.y;
            recogPosi.z = recogNode.Global_Pose_.z;
            vector<int> tmpIndexs;
            vector<float> tmpDistances;
            kdtreeOldPoses->radiusSearch(recogPosi, truthDisThres, tmpIndexs, tmpDistances);
            if(!tmpIndexs.empty()){
                for(auto tmpPriorI : tmpIndexs){
                    matchPairs.push_back(make_pair(tmpPriorI, recogJ));
                    matchFlags.push_back(1);
                }
            }else{
                kdtreeOldPoses->nearestKSearch(recogPosi, 1, tmpIndexs, tmpDistances);
                if(tmpIndexs.size() != 1){
                    cerr << " what happend! " << endl;
                    getchar();
                    exit(0);
                }
                matchPairs.push_back(make_pair(tmpIndexs.front(), recogJ));
                matchFlags.push_back(2);
            }
        }
        cout << "get " << matchPairs.size() << " pairs" << endl;
        //for(int i = 0; i < matchPairs.size(); i++){
        //    auto& p = matchPairs[i];
        //    auto& f = matchFlags[i];
        //    cout << oldNodes[p.first].id_ << " " << recogNodes[p.second].id_ << " " << f << endl;
        //    getchar();
        //}
    }
    //////////////////////////////////////////////////////////
public:
    int TP = 0;
    int FP = 0;
    int TN = 0;
    int FN = 0;
    int FPFN = 0;
    int ncFN = 0;
    vector<recogPair> pairList;
private:
    map<pair<int,int>,double>* hisMap;
    void checkPairs(){
        for(int i = 0; i < matchPairs.size(); i++){
            int tmpFlag = matchFlags[i];
            auto& nowPair = matchPairs[i];
            node& priorNode = oldNodes[nowPair.first];
            node& recogNode = recogNodes[nowPair.second];

            double simScore = 0;
            if(hisMap != nullptr ){
                pair<int,int> tmpBuff(priorNode.id_, recogNode.id_);
                auto it = hisMap->find(tmpBuff);
                if(it != hisMap->end()){
                    simScore = it->second;
                }else{
                    simScore = node::getScoreFile(priorNode, recogNode);
                    mutMap.lock();
                    hisMap->insert(make_pair(tmpBuff, simScore));
                    mutMap.unlock();
                }
            }else{
                simScore = node::getScoreFile(priorNode, recogNode);
            }

            if(simScore <= simThres && tmpFlag == 1){
                //TP
                getNewJudge(pairList, priorNode, recogNode, _T_P);
                TP++;
            }
            if(simScore > simThres && tmpFlag == 1){
                //FN
                getNewJudge(pairList, priorNode, recogNode, _n_c_F_N);
                FN++;
            }
            if(simScore <= simThres && tmpFlag == 2){
                //FP
                getNewJudge(pairList, priorNode, recogNode, _F_P);
                FP++;
            }
            if(simScore > simThres && tmpFlag == 2){
                //TN
                getNewJudge(pairList, priorNode, recogNode, _T_N);
                TN++;
            }
        }
    }

public:
    myRecog(string recog, string load, double simThres_, double truthDisThres_ = 3.0, double candidate_ = 20.0, double start_=-1, double end_=-1, int pick=1,
            map<pair<int,int>,double>* hisMap_= nullptr){
        //////////////////init mem
        kdtreeOldPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        kdtreeRecogPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        oldPose3d.reset(new pcl::PointCloud<pcl::PointXYZ>());
        recogPose3d.reset(new pcl::PointCloud<pcl::PointXYZ>());
        //////////////////param
        hisMap = hisMap_;
        disCandidate = candidate_;
        truthDisThres = truthDisThres_;
        simThres = simThres_;
        node_load_path = load;
        node_recog_Path = recog;
        ////////////////load prior and recog
        read_nodes_from_files_B(node_load_path, oldNodes, oldPose3d, kdtreeOldPoses);
        read_nodes_from_files_B(node_recog_Path, recogNodes, recogPose3d, kdtreeRecogPoses, start_, end_, pick);
        ////////////////get match pair
        getMatchPair();
        ////////////////check pair
        checkPairs();
    }
};



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

    void getCandidateSim(node& nowNode, vector<PointType>& tempSortKey){
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
            float score = 0;

            if(hisMap != nullptr ){
                pair<int,int> tmpBuff(oldNodes[pointSearchIndLoop[n]].id_, nowNode.id_);
                auto it = hisMap->find(tmpBuff);
                if(it != hisMap->end()){
                    score = it->second;
                }else{
                    score = node::getScoreFile(oldNodes[pointSearchIndLoop[n]], nowNode);
                    mutMap.lock();
                    hisMap->insert(make_pair(tmpBuff, score));
                    mutMap.unlock();
                }
            }else{
                score = node::getScoreFile(oldNodes[pointSearchIndLoop[n]], nowNode);
            }

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


    map<pair<int,int>,double>* hisMap;
public:
    int TP_1 = 0, TP_N = 0;
    int TN_1 = 0, TN_N = 0;
    int FP_1 = 0, FP_N = 0;
    int FN_1 = 0, FN_N = 0;
    int ncFN = 0, FPFN = 0;
    vector<recogPair> pairList;
    recallTopo(std::string node_load_path_,
               double sameThres_,                   //Similarity threshold
               map<pair<int,int>,double>* hisBuff= nullptr,
               double truthDisThres_ = 5.0,         //Truth conditional distance
               double findCandidateDis_ = 30.0      //Candidate point distance
    )
    {
        hisMap = hisBuff;
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
    bool judgeTop(vector<PointType>&sim,vector<PointType>&truth){
        bool result = false;
        for(int i = 0; i < 3 && i < truth.size(); i++){
            result = ( result || (sim.front().x == truth[i].x) );
        }
        return result;
    }

    void Invincible(node& nowNode, vector<PointType>&truth,vector<PointType>&sim)
    {
        //如果此点找到了先验为0的，就说明先验和当前是完全相同的点，SC为0距离同样为0，这时就看看此点第二相似度的先验点的匹配情况，
//        if( !truth.empty() && !sim.empty() && sim.front().z == 0 && truth.front().y <= 0.01) {
//            //如果有第二相似的，也有第二近的，那么通过下面原有的方法判断TPFN
//            if(sim.size() >= 2 && truth.size() >= 2 ){
//                sim[0] = sim[1];
//                truth[0] = truth[1];
//            }
//            else return;
//        }
        //判断P（针对“当前节点与sim.front()这条识别出来的连线” 来说）
        if(!sim.empty() && sim.front().z < sameThres){  //这个P
            if(!truth.empty() && judgeTop(sim, truth))  //是对的
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
            if(!sim.empty() && sim.front().z < sameThres && !judgeTop(sim, truth)){ //情况1，与别的先验节点连上了
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
    void run(node input_node){
        //truth point search in truth distance
        vector<PointType> resultDis;//first index second dis
        getTruth(input_node, resultDis);

        //sim search for candidate Point to find Most similar TOP1/TOP10 for Accuracy and fast search 50m
        vector<PointType> resultSim;//first index second dis
        getCandidateSim(input_node, resultSim);

        Invincible(input_node, resultDis,resultSim);
    }
};


class test{
public:
    string loadPathBase;
    string recogPathBase;
    int pick = 1;
    int start = -1;
    int end = -1;
    double truthThres = 1.0;
    double disCandidate = 40.0;

    string mode;
    double buildValue;
    double recogValue;

    map<pair<int,int>,double>* hisBuff;
    vector<node>* nodeList;
};

int nowThread = 0;
int nowIndex = 0;
int MaxThread = 6;

void recogOne(test param){
    stringstream ss;
    stringstream ss2;
    string loadPath;
    string recogPath;
    string totalDir;
    string nowDir;
    string fileResult;
    recogPath = param.loadPathBase + "/nodeSparse/";
    if(param.mode == "recogD") {
        ss << std::fixed << setprecision(1) << param.buildValue;
        loadPath = param.loadPathBase + "/buildD/D" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "D" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/recogD";
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/DRecallInfo" + ss.str() +".txt";
    }
    if(param.mode == "recogS") {
        ss << std::fixed << setprecision(2) << param.buildValue;
        loadPath = param.loadPathBase + "/buildS/S" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "S" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/recogS";
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/SRecallInfo.txt";
    }
    if(param.mode == "N") {
        ss << std::fixed << setprecision(2) << param.buildValue;
        loadPath = param.loadPathBase + "/buildN/N" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "N" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/testN" + ss.str();
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/NRecallInfo.txt";
    }

    if( 0 != access(totalDir.c_str(), 0) ){
        mkdir(totalDir.c_str(), 0777);
    }
    if( 0 != access(nowDir.c_str(), 0) ){
        mkdir(nowDir.c_str(), 0777);
    }
    if( 0 == access(fileResult.c_str(), 0) ){
        nowThread--;
        return;
    }

    myRecog recallT(recogPath, loadPath, param.recogValue,
                   param.truthThres, param.disCandidate, param.start, param.end, param.pick,
                   param.hisBuff);

    ofstream fileout;
    fileout.open(fileResult);
    int TP = recallT.TP;
    int FP = recallT.FP;
    int TN = recallT.TN;
    int FN = recallT.FN;
    double acc = ( TP + TN ) * 1.0 / ( TP + TN + FP + FN ) * 1.0;
    double pre = ( TP ) * 1.0 / ( TP + FP ) * 1.0;
    double rec = ( TP ) * 1.0 / ( TP + FN ) * 1.0;
    fileout << "TP: " << TP << endl;
    fileout << "FP: " << FP << endl;
    fileout << "TN: " << TN << endl;
    fileout << "FN: " << FN << endl;
    fileout << "accuracy: " << (acc)  << endl;
    fileout << "precision: " << (pre) << endl;
    fileout << "recall: " << (rec) << endl;
    //for(auto n : recallT.pairList){
    //    fileout << n;
    //    fileout << endl;
    //}
    fileout.close();
    nowThread--;
}

void recogO(test param){
    double buildValue = param.buildValue;
    stringstream ss;
    stringstream ss2;
    string loadPath;
    string totalDir;
    string nowDir;
    string fileResult;
//    if(param.mode == "recogD") {
//        ss << std::fixed << setprecision(1) << buildValue;
//        loadPath = param.loadPathBase + "/buildD/D" + ss.str() + "/";
//        ss2 << fixed << setprecision(2) << "D" << param.recogValue << "R";
//        totalDir = param.recogPathBase + "/recogD";
//        nowDir = totalDir + "/" + ss2.str();
//        fileResult = nowDir + "/DRecallInfo" + ss.str() +".txt";
//    }
//    if(param.mode == "recogN") {
//        ss << std::fixed << setprecision(2) << buildValue;
//        loadPath = param.loadPathBase + "/buildN/N" + ss.str() + "/";
//        ss2 << fixed << setprecision(2) << "N" << param.recogValue << "R";
//        totalDir = param.recogPathBase + "/recogN";
//        nowDir = totalDir + "/" + ss2.str();
//        fileResult = nowDir + "/NRecallInfo" + ss.str() +".txt";
//    }
//    if(param.mode == "recogS") {
//        ss << std::fixed << setprecision(2) << buildValue;
//        loadPath = param.loadPathBase + "/buildS/S" + ss.str() + "/";
//        ss2 << fixed << setprecision(2) << "S" << param.recogValue << "R";
//        totalDir = param.recogPathBase + "/recogS";
//        nowDir = totalDir + "/" + ss2.str();
//        fileResult = nowDir + "/SRecallInfo" + ss.str() +".txt";
//    }
    if(param.mode == "testD") {
        ss << std::fixed << setprecision(1) << buildValue;
        loadPath = param.loadPathBase + "/buildD/D" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "D" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/testD";
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/DRecallInfo.txt";
    }
    if(param.mode == "testS") {
        ss << std::fixed << setprecision(2) << buildValue;
        loadPath = param.loadPathBase + "/buildS/S" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "S" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/testS";
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/SRecallInfo.txt";
    }
    if(param.mode == "testN") {
        ss << std::fixed << setprecision(2) << buildValue;
        loadPath = param.loadPathBase + "/buildN/N" + ss.str() + "/";
        ss2 << fixed << setprecision(2) << "N" << param.recogValue << "R";
        totalDir = param.recogPathBase + "/testN";
        nowDir = totalDir + "/" + ss2.str();
        fileResult = nowDir + "/NRecallInfo.txt";
    }

    recallTopo recallTopoMap(loadPath, param.recogValue, param.hisBuff);
    int nodesSize = param.nodeList->size();

    if( 0 != access(totalDir.c_str(), 0) ){
        mkdir(totalDir.c_str(), 0777);
    }
    if( 0 != access(nowDir.c_str(), 0) ){
        mkdir(nowDir.c_str(), 0777);
    }
    if( 0 == access(fileResult.c_str(), 0) ){
        nowThread--;
        return;
    }
    param.start = param.start==-1?0:param.start;
    param.end = param.end==-1?param.nodeList->size():param.end;

    for(int j = param.start; j < param.end; j+=param.pick){
        recallTopoMap.run((*param.nodeList)[j]);
    }

    ofstream fileout;
    fileout.open(fileResult);
    int TP1 = recallTopoMap.TP_1; int TP10 = recallTopoMap.TP_N;
    int FP1 = recallTopoMap.FP_1; int FP10 = recallTopoMap.FP_N;
    int TN1 = recallTopoMap.TN_1; int TN10 = recallTopoMap.TN_N;
    int FN1 = recallTopoMap.FN_1; int FN10 = recallTopoMap.FN_N;
    double T1acc = ( TP1 + TN1 ) * 1.0 / ( TP1 + TN1 + FP1 + FN1 ) * 1.0;
    double T1pre = ( TP1 ) * 1.0 / ( TP1 + FP1 ) * 1.0;
    double T1recall = ( TP1 ) * 1.0 / ( TP1 + FN1 ) * 1.0;
    fileout << "TP: " << TP1 << endl;
    fileout << "FP: " << FP1 << endl;
    fileout << "TN: " << TN1 << endl;
    fileout << "FN: " << FN1 << endl;
    fileout << "FPFN: " << recallTopoMap.FPFN << endl;
    fileout << "ncFN: " << recallTopoMap.ncFN << endl;
    fileout << "accuracy: " << (T1acc)  << endl;
    fileout << "precision: " << (T1pre) << endl;
    fileout << "recall: " << (T1recall) << endl;
    for(auto n : recallTopoMap.pairList){
        fileout << n;
        fileout << endl;
    }
    fileout.close();
    nowThread--;
}

vector<node> o1Nodes;
vector<node> o2Nodes;
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


int main(int argc, char** argv){


    int o1Size,o2Size,m1,m2,m3;
    //loadMap(allMap1, "/home/qh/robot_ws/map/2021-08-30-16-12-25L/k.map");
    //loadMap(allMap2, "/home/qh/robot_ws/map/2021-08-30-18-06-30L/k.map");
    //loadMap(allMap3, "/home/qh/robot_ws/map/2021-08-30-19-17-22L/k.map");
    //loadMap(allMapo1, "/home/qh/robot_ws/map/o1/o1.map");
    loadMap(allMapo2, "/home/qh/robot_ws/map/o2/o2.map");
    o1Size = allMapo1.size();
    o2Size = allMapo2.size();
    m1 = allMap1.size();
    m2 = allMap2.size();
    m3 = allMap3.size();
    cout << "load buff 1 " << m1 << endl;
    cout << "load buff 2 " << m2 << endl;
    cout << "load buff 3 " << m3 << endl;
    cout << "load buff o1 " << o1Size << endl;
    cout << "load buff o2 " << o2Size << endl;



    ros::init(argc, argv, "recallD");
    ros::NodeHandle nh("~");


    string o1_pathBase = "/home/qh/robot_ws/map/o1";
    string o2_pathBase = "/home/qh/robot_ws/map/o2";

    string o1NodesPath = o1_pathBase+"/nodeSparse/";
    string o2NodesPath = o2_pathBase+"/nodeSparse/";
    read_nodes_from_files_B(o1Nodes, o1NodesPath);
    read_nodes_from_files_B(o2Nodes, o2NodesPath);


    vector<double> thresTab{
        0.90,  0.80,  0.70,  0.60,  0.50,  0.42,  0.38,  0.33,  0.35,  0.30,
        0.28,  0.27,  0.26,
        0.25,  0.24,
        0.23,  0.22,  0.21,  0.20,  0.19,  0.18,  0.17,  0.16,
        0.15,  0.14,  0.13,  0.12,
        0.10
    };

    vector<double> buildDTab{
            0.2, 0.5, 0.6, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 7, 9, 10, 15, 20, 30
    };

    vector<double> buildSTab{
            0.10, 0.13, 0.15, 0.18, 0.20, 0.23, 0.25, 0.28, 0.30, 0.35, 0.33, 0.38, 0.40, 0.43, 0.45, 0.48, 0.50, 0.53,
            0.60, 0.63, 0.65, 0.68, 0.70, 0.73, 0.75, 0.78, 0.55, 0.58
    };

    vector<double> buildNTab{
            0.10,  0.20, 0.30, 0.40,
            0.50,  0.53, 0.55, 0.58,
            0.60,  0.63, 0.65, 0.68,
            0.70,  0.73, 0.75, 0.78,
            0.80,  0.83, 0.85, 0.88,
            0.90
    };

    vector<test> paramList;


//    //recogD
//    for(auto thres : thresTab) {
//        for(auto build : buildDTab){
//            test paramTest;
//            paramTest.loadPathBase = o1_pathBase;
//            paramTest.recogPathBase = o1_pathBase;
//            paramTest.buildValue = build;
//            paramTest.recogValue = thres;
//            paramTest.mode = "recogD";
//            paramTest.hisBuff = &allMapo1;
//            paramTest.nodeList = &o1Nodes;
//            paramList.push_back(paramTest);
//        }
//    }
//
//    //recogS
//    for(auto thres : thresTab) {
//        for(auto build : buildSTab){
//            test paramTest;
//            paramTest.loadPathBase = o1_pathBase;
//            paramTest.recogPathBase = o1_pathBase;
//            paramTest.buildValue = build;
//            paramTest.recogValue = thres;
//            paramTest.mode = "recogS";
//            paramTest.hisBuff = &allMapo1;
//            paramTest.nodeList = &o1Nodes;
//            paramList.push_back(paramTest);
//        }
//    }

    //recogN
//    for(auto thres : thresTab) {
//        for(auto build : buildNTab){
//            test paramTest;
//            paramTest.loadPathBase = o1_pathBase;
//            paramTest.recogPathBase = o1_pathBase;
//            paramTest.buildValue = build;
//            paramTest.recogValue = thres;
//            paramTest.mode = "recogN";
//            paramTest.hisBuff = &allMapo1;
//            paramTest.nodeList = &o1Nodes;
//            paramList.push_back(paramTest);
//        }
//    }

    double bestD = 5.0;
    double bestN = 0.53;
    double bestS = 0.30;
    //testD
    for(auto thres : thresTab)
    {
        test paramTest;
        paramTest.loadPathBase = o1_pathBase;
        paramTest.recogPathBase = o2_pathBase;
        paramTest.buildValue = bestD;
        paramTest.recogValue = thres;
        paramTest.mode = "testD";
        paramTest.hisBuff = &allMapo2;
        paramTest.nodeList = &o2Nodes;
        paramList.push_back(paramTest);
    }
    //testS
    for(auto thres : thresTab)
    {
        test paramTest;
        paramTest.loadPathBase = o1_pathBase;
        paramTest.recogPathBase = o2_pathBase;
        paramTest.buildValue = bestS;
        paramTest.recogValue = thres;
        paramTest.mode = "testS";
        paramTest.hisBuff = &allMapo2;
        paramTest.nodeList = &o2Nodes;
        paramList.push_back(paramTest);
    }

    //testN
    for(auto thres : thresTab)
    {
        test paramTest;
        paramTest.loadPathBase = o1_pathBase;
        paramTest.recogPathBase = o2_pathBase;
        paramTest.buildValue = bestN;
        paramTest.recogValue = thres;
        paramTest.mode = "testN";
        paramTest.hisBuff = &allMapo2;
        paramTest.nodeList = &o2Nodes;
        paramList.push_back(paramTest);
    }


    ros::Rate loop(30);

    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < paramList.size()){
            auto& paramTemp = paramList[nowIndex];
            cout << nowIndex << " add a new thread" << paramTemp.mode << "" << paramTemp.buildValue << " " << paramTemp.recogValue << endl;
            nowThread++;
            //thread* tempT(new thread(recogOne, paramList[nowIndex]));
            thread* tempT(new thread(recogO, paramList[nowIndex]));
            nowIndex++;
        }
        if(nowIndex == paramList.size() && nowThread == 0)
        {
            cout << " end " << endl;
            break;
        }
        loop.sleep();
    }

//    if(allMap1.size() - m1 != 0)
//        saveMap(allMap1, "/home/qh/robot_ws/map/2021-08-30-16-12-25L/k.map");
//
//    if(allMap2.size() - m2 != 0)
//        saveMap(allMap2, "/home/qh/robot_ws/map/2021-08-30-18-06-30L/k.map");
//
//    if(allMap3.size() - m3 != 0)
//        saveMap(allMap3, "/home/qh/robot_ws/map/2021-08-30-19-17-22L/k.map");
//
//    if(allMapo1.size() - o1Size != 0)
//        saveMap(allMapo1, "/home/qh/robot_ws/map/o1/o1.map");

    if(allMapo2.size() - o2Size != 0)
        saveMap(allMapo2, "/home/qh/robot_ws/map/o2/o2.map");

    cout << "add 1 " << allMap1.size() - m1 << endl;
    cout << "add 2 " << allMap2.size() - m2 << endl;
    cout << "add 3 " << allMap3.size() - m3 << endl;
    cout << "add o1 " << allMapo1.size() - o1Size << endl;
    cout << "add o2 " << allMapo2.size() - o2Size << endl;

    return 0;
}
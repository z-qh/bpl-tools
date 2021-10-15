#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"
#include "thread"


int main(int argc, char** argv){

    extern std::vector<float>dataNeibor;
    string cloudPathBase = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/pair";
    string ohterPathCloud = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/pair";

    vector<pcl::PointCloud<pcl::PointXYZI>> sourceCloud;
    vector<pcl::PointCloud<pcl::PointXYZI>> sourceCloud2;
    vector<node> newNode;
    vector<node> newNode2;

    {
        stringstream ss;
        ss << std::fixed << stoi(argv[1]);
        cloudPathBase += ss.str() + "/.cloud/";
    }{
        stringstream ss;
        ss << std::fixed << stoi(argv[1])+1;
        ohterPathCloud += ss.str() + "/.cloud/";
    }

    std::vector<std::string>file_name1;
    std::vector<std::string>file_name2;
    {
        std::vector<int>index;
        DIR *d = opendir(cloudPathBase.c_str());
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
            file_name1.push_back(cloudPathBase + ss.str()+".pcd");
        }
    }
    cout << endl;
    {
        std::vector<int>index;
        DIR *d = opendir(ohterPathCloud.c_str());
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
            file_name2.push_back(ohterPathCloud + ss.str()+".pcd");
        }
    }

    cout << "file get ready" << endl;

    KeyMat globalScanContextRingKey;                                //存储全局描述子的ringkey, 要进行搜索的树
    std::unique_ptr<InvKeyTree>scanContextTree;                     //用于构建描述子搜索树

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    isSaveCloud = false;
    for(int i = 0; i < file_name1.size(); i++){
        node tempNode(file_name1[i]);
        tempNode.id_ = i;
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(tempNode.scanContext_);
        std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
        if(i!=0)globalScanContextRingKey.push_back(polarcontext_vkeys_vec);
        newNode.push_back(tempNode);
    }

    scanContextTree.reset();
    scanContextTree = std::make_unique<InvKeyTree>(PC_NUM_RING, globalScanContextRingKey, 10);


    for(int i = 0; i < file_name2.size(); i++){
        node tempNode(file_name2[i]);
        tempNode.id_ = i;
        newNode2.push_back(tempNode);
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    cout << file_name1.size() + file_name2.size() << " fps " << time_used << "s " << endl;
    getchar();

    //knn search
    t1 = chrono::steady_clock::now();

    vector<pair<int,double>> KnnRes;
    double minScore = node::getCandidateFromKeyRing(newNode, scanContextTree, newNode[0], KnnRes);
    cout << "knn search result " << minScore << endl;
    for(auto n : KnnRes){
        cout << " knn id " << n.first << " knn score " << n.second << endl;
    }
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    cout << file_name1.size() + file_name2.size() << " knn " << time_used << "s " << endl;

    // std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);                  //存放索引值数组
    // std::vector<float> distance_sqrt(NUM_CANDIDATES_FROM_TREE);                       //存放距离值数组
    // nanoflann::KNNResultSet<float>knnsearchResult(NUM_CANDIDATES_FROM_TREE);          //
    // knnsearchResult.init(&candidate_indexes[0], &distance_sqrt[0]);
    // scanContextTree->index->findNeighbors(knnsearchResult,&currRingKey[0],nanoflann::SearchParams(NUM_CANDIDATES_FROM_TREE));

    t1 = chrono::steady_clock::now();
    for(int i = 0; i < newNode.size();i++){
        double scoreNow = node::getScore(newNode[0], newNode[i]);
        auto scoreDirect = distanceBtnScanContext(newNode[0].scanContext_, newNode[i].scanContext_);
        cout  << newNode[0].id_ << " " <<newNode[i].id_ << " score is " <<  scoreNow << " " << scoreDirect.first << endl;
    }
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    cout << file_name1.size() + file_name2.size() << " fps " << time_used << "s " << endl;

    cout << endl;
    cout << endl;
    for(int i = 0; i < newNode2.size();i+=5){
        cout  << newNode[0].id_ << " " <<newNode2[i].id_ << " score is " << node::getScore(newNode[0], newNode2[i]) << endl;
    }
}

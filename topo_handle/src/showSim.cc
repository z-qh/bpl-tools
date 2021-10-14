#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"
#include "thread"

extern std::vector<float>dataNeibor;

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
    cout << "load succeed \n";
}


double getNowAndHisScore(node& nowNodeOld, node& nowNodeNew){
    double min_dist = 10000000;
    int nn_align = 0, nn_idx = 0;   //nn_align为描述子旋转的角度值， nn_idx为匹配成功的索引值
    int loop_index = -1;

    for(int i = 0; i < SHIFTSIZE; i++){
        Eigen::MatrixXd currentContextShift = nowNodeOld.vertices_.scanContext_8[i];
        Eigen::MatrixXd scanContextCandidate = nowNodeNew.vertices_.scanContext_8.front();
        std::pair<double, int> sc_dist_result = distanceBtnScanContext(currentContextShift, scanContextCandidate);
        double candidate_dist = sc_dist_result.first;       //余弦距离
        if(candidate_dist < min_dist)
        {
            min_dist = candidate_dist;                      //两个描述子之间的余弦距离
        }
    }
    return min_dist;
}

double distance(node& nodeOld, node& nodeNew){
    PointType& A = nodeOld.vertices_.Global_Pose_;
    PointType& B = nodeNew.vertices_.Global_Pose_;
    return sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)+pow(A.z-B.z,2));
}

int main1(){
    string sourcePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/nodeSparse/";
    vector<node> nodeList;
    read_nodes_from_files_B(nodeList, sourcePath);
    for(int i = 40; i < nodeList.size(); i++){
        int nowI = i;
        char k1 = getchar();
        if(k1 == 'n')   continue;
        double score = 0, score_last = 0;
        double dis = 0, dis_last = 0;
        for(int j = i; j < nodeList.size(); j++){
            char k2 = getchar();
            if(k2 == 'n') break;
            cout << " now I " << nodeList[i].id_  << " nowJ " << nodeList[j].id_ << "----------------" << endl;
            dis_last = dis;
            score_last = score;
            score = getNowAndHisScore(nodeList[i], nodeList[j]);
            dis =  distance(nodeList[i], nodeList[j]);
            cout << "between score is " << score << " add "<<std::fixed<<setprecision(2)<<score-score_last << endl;
            cout << "between dis   is " << dis << " add "<<std::fixed<<setprecision(2) << dis-dis_last <<" " <<(dis-dis_last)/(score-score_last) <<endl;
        }

    }
}

template<typename T>
MatrixXd makeScancontext_B(T &_scan, int PC_NUM_RING, int PC_NUM_SECTOR, double PC_MAX_RADIUS ){
    double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
    int num_pts_scan_number = _scan->points.size();
    // main
    const int NO_POINT = -1000;
    //矩阵的维度为20*60
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);//PC_NUM_RING = 20, PC_NUM_SECTOR = 60 Ones为全1矩阵
    float value_angle,value_range;
    int ring_idx, sctor_idx;
    // std::cout<<"points size is "<<_scan->points.size()<<std::endl;
#pragma omp parallel for num_threads(3)
    for(auto data:_scan->points){
        auto point = data;
        point.y += 20;   //保证z值大于0
        value_range = std::sqrt(point.x*point.x + point.z*point.z);
        value_angle = xy2theta(point.x,point.z);
        if(value_range > PC_MAX_RADIUS){
            continue;
        }
        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((value_range/PC_MAX_RADIUS)*PC_NUM_RING))),1);//0还是1
        sctor_idx = std::max( std::min(PC_NUM_SECTOR, int(ceil((value_angle / 360.0) * PC_NUM_SECTOR ))), 1);
        if(desc(ring_idx-1,sctor_idx-1)<point.y)
            desc(ring_idx-1,sctor_idx-1) = point.y;
    }
    for(int row_idx = 0; row_idx < desc.rows(); ++row_idx)
        for(int col_idx = 0; col_idx < desc.cols(); ++col_idx)
            if(desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;
    return desc;
}


string cloudPathBase = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/pair";
string ohterPathCloud = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/pair";

int main(int argc, char** argv){
    vector<pcl::PointCloud<pcl::PointXYZI>> sourceCloud;
    vector<pcl::PointCloud<pcl::PointXYZI>> sourceCloud2;
    vector<node> newNode;
    vector<node> newNode2;

    int paramCount = 5;
    while(argv[paramCount]!= nullptr){
        dataNeibor.push_back(stof(argv[paramCount]));
        cout << stof(argv[paramCount]) << endl;
        paramCount++;
    }
    SHIFTSIZE = dataNeibor.size();
    //ring kd tree
    KeyMat ringKeyKdTree;
    KeyMat ringKeyKdTreeSearchData;
    std::unique_ptr<InvKeyTree> ringKeyKdTreeSearcher;

    {
        stringstream ss;
        ss << std::fixed << stoi(argv[1]);
        cloudPathBase += ss.str() + "/.cloud/";
    }{
        stringstream ss;
        ss << std::fixed << stoi(argv[1])+1;
        ohterPathCloud += ss.str() + "/.cloud/";
    }
    int lenSize = stoi(argv[2]);
    int widthSize = stoi(argv[3]);
    double mDis = stof(argv[4]);

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

    for(const auto& s : file_name1){
        pcl::PointCloud<pcl::PointXYZI> tempCloud;
        pcl::io::loadPCDFile(s,tempCloud);
        sourceCloud.push_back(tempCloud);
    }
    for(const auto& s : file_name2){
        pcl::PointCloud<pcl::PointXYZI> tempCloud;
        pcl::io::loadPCDFile(s,tempCloud);
        sourceCloud2.push_back(tempCloud);
    }

    for(int n = 0; n < file_name1.size(); n++){
        node tempNode;
        tempNode.id_ = n;
        pcl::PointCloud<pcl::PointXYZI>& tempCloud = sourceCloud[n];
        stringstream ss;
        ss << fixed << setprecision(0) << lenSize << "-" << widthSize << "-" <<mDis;
        for(float shift : dataNeibor)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZI>());
            tmpCloud->clear();
            for(auto point:tempCloud.points)
            {
                point.z += shift;
                tmpCloud->push_back(point);
            }
            tempNode.vertices_.scanContext_8.push_back(makeScancontext_B(tmpCloud, lenSize, widthSize, mDis));
        }
        newNode.push_back(tempNode);

        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(tempNode.vertices_.scanContext_8.front());
        std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
        if(n!=0) ringKeyKdTree.push_back(polarcontext_vkeys_vec);

    }

    ringKeyKdTreeSearchData.assign(ringKeyKdTree.begin(), ringKeyKdTree.end());
    ringKeyKdTreeSearcher = std::make_unique<InvKeyTree>(lenSize, ringKeyKdTreeSearchData, 10);

    for(int n = 0; n < file_name2.size(); n++){
        node tempNode;
        tempNode.id_ = n;
        pcl::PointCloud<pcl::PointXYZI>& tempCloud = sourceCloud2[n];
        stringstream ss;
        ss << fixed << setprecision(0) << lenSize << "-" << widthSize << "-" <<mDis;
        for(float shift : dataNeibor)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZI>());
            tmpCloud->clear();
            for(auto point:tempCloud.points)
            {
                point.z += shift;
                tmpCloud->push_back(point);
            }
            tempNode.vertices_.scanContext_8.push_back(makeScancontext_B(tmpCloud, lenSize, widthSize, mDis));
        }
        newNode2.push_back(tempNode);
    }
    stringstream ss;
    ss << fixed << setprecision(0) << lenSize << "-" << widthSize << "-" <<mDis;
    cout << ss.str() << endl;
    getchar();
    for(int i = 0; i < newNode.size();i++){
        cout  << newNode[0].id_ << " " <<newNode[i].id_ << " score is1 " << getNowAndHisScore(newNode[0], newNode[i]) << endl;
    }
    cout << endl;
    cout << endl;
    for(int i = 0; i < newNode2.size();i++){
        cout  << newNode[0].id_ << " " <<newNode2[i].id_ << " score is1 " << getNowAndHisScore(newNode[0], newNode2[i]) << endl;
    }


    vector<int> top10;
    std::vector<size_t>candidata_indexes(1);//top 10
    std::vector<float>out_dists_sqrt(1);//top 10
    nanoflann::KNNResultSet<float>knnsearch_result(1);//top 10
    knnsearch_result.init(&candidata_indexes[0],&out_dists_sqrt[0]);

    {
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(newNode[0].vertices_.scanContext_8.front());
        std::vector<float> currentKey = eig2stdvec(ringkey);
        if(ringKeyKdTreeSearcher->index->findNeighbors(knnsearch_result, &currentKey[0], nanoflann::SearchParams(1)))
            top10.push_back(candidata_indexes[0]);
    }
    for(int i = 0; i < SHIFTSIZE; i++){
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(newNode[0].vertices_.scanContext_8[i]);
        std::vector<float> currentKey = eig2stdvec(ringkey);
        if(ringKeyKdTreeSearcher->index->findNeighbors(knnsearch_result, &currentKey[0], nanoflann::SearchParams(1)))
            top10.push_back(candidata_indexes[0]);
    }


    for(int i = 0; i < top10.size() ; i++){
        cout << top10[i] << " " << getNowAndHisScore(newNode[0], newNode[top10[i]]) << endl;
    }
}


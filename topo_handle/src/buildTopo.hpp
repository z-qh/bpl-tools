#include "include/utility.h"
#include "include/dbscan/node_context_localMap.h"

#include "cmath"
#include "unordered_set"
#include "unordered_map"
#include "deque"

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
    unordered_map<int, int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
    unordered_map<int,int>allNodeIndexId;           //第一个为节点的id，第二个为在nodes中的索引
    //////////////////////////////////////////////////////////

    int loopClosureFromKDTree(PointType currentPose, node& scNode)
    {

        std::unordered_map<int, int>possibleNodeIndex;
        possibleNodeIndex.clear();
        if(global_scanContext.size() > 1)
        {
            int result = (int)scanContextKnnSearch(currentPose,scNode);
            if(result == -1)
            {
                return -1;
            }else
                return result;
        }
        return -1;
    }
    //通过kd树搜索，返回闭环节点id
    int scanContextKnnSearch(PointType currentPose, node& currNode)
    {

        double min_dist = 10000000;
        int nn_align = 0, nn_idx = 0;   //nn_align为描述子旋转的角度值， nn_idx为匹配成功的索引值
        int loop_index = -1;
        // //knn search
        // std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);                  //存放索引值数组
        // std::vector<float> distance_sqrt(NUM_CANDIDATES_FROM_TREE);                       //存放距离值数组
        // nanoflann::KNNResultSet<float>knnsearchResult(NUM_CANDIDATES_FROM_TREE);          //
        // knnsearchResult.init(&candidate_indexes[0], &distance_sqrt[0]);
        // scanContextTree->index->findNeighbors(knnsearchResult,&currRingKey[0],nanoflann::SearchParams(NUM_CANDIDATES_FROM_TREE));
        //使用位置进行KD树搜索
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, disForDetech, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        unordered_set<int>proposeIndex;
        //可能存在闭环的拓扑节点
        for(int i=0; i<pointSearchIndLoop.size(); ++i)
        {
            proposeIndex.insert(pointSearchIndLoop[i]);
        }

        for(int i = 0; i < SHIFTSIZE; i++){
            Eigen::MatrixXd currentContextShift = currNode.vertices_.scanContext_8[i];
            for(const auto& index:proposeIndex)
            {
                if(nodeKeyIndex[index] == last_node_id)
                    continue;
                Eigen::MatrixXd scanContextCandidate = global_scanContext[index];
                std::pair<double, int> sc_dist_result = distanceBtnScanContext(currentContextShift, scanContextCandidate);
                double candidate_dist = sc_dist_result.first;       //余弦距离
                int candidate_align = sc_dist_result.second;        //偏移量
                if(candidate_dist < min_dist)
                {
                    min_dist = candidate_dist;                      //两个描述子之间的余弦距离
                    nn_align = candidate_align;
                    nn_idx = index;    //搜索到的描述子的索引值
                }
            }
        }
        cout<<"KnnSearch min_dist "<<min_dist<<" index is "<<nn_idx<<" node id "<<(int)nodeKeyIndex[nn_idx]<<" last_node_id "<<last_node_id<<" rotation "<<nn_align<<std::endl;
        //设定余弦距离最小值
        if ( min_dist < judgeLoopScore)
        {
            loop_index = nodeKeyIndex[nn_idx];
        }

        if(loop_index!=-1 )
        {
            cout<<"min_dist "<<min_dist<<endl;
            return nn_idx;
        }else
            return -1;

    }


    double getScoreFromLastNode(Eigen::MatrixXd& sc_current, Eigen::MatrixXd& sc_last)
    {
        double score = distDirectSC(sc_current, sc_last);
        return score;
    }

public:
    buildTopo(std::string node_save_path_, double buildMapSimScore_, double buildMapDisThres_ = 15.0, double disForDetech_ = 50.0, double gnssSimThres = 0.35 )
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
    void run(node& input_node)
    {
        nowTime = input_node.time_stamp_;
        cout << "input sig state" << input_node.vertices_.Global_Pose_.intensity << endl;
        if(input_node.vertices_.Global_Pose_.intensity == 1
         ||input_node.vertices_.Global_Pose_.intensity == 0
         ||input_node.vertices_.Global_Pose_.intensity == -1){
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
        if(input_node.vertices_.Global_Pose_.intensity == 5){
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

        if(isFirst) {
            cout << "mode " << mode << endl;
            isFirst = false;
            ////////////////////////////////////////////
            node tmp_node = input_node;
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
            tmp_node.vertices_.Global_Pose_.intensity = mode;
            nodes.push_back(tmp_node);

            sc_current = tmp_node.vertices_.scanContext_8.front();
            sc_last = sc_current;

            PointType thisPose;
            thisPose.x = tmp_node.vertices_.Global_Pose_.x;
            thisPose.y = tmp_node.vertices_.Global_Pose_.y;
            thisPose.z = tmp_node.vertices_.Global_Pose_.z;
            thisPose.intensity = current_node_id;
            globalKeyPose3d->push_back(thisPose);

            PreviousNodePosition(0) = thisPose.x;
            PreviousNodePosition(1) = thisPose.y;
            PreviousNodePosition(2) = thisPose.z;

            global_scanContext.push_back(tmp_node.vertices_.scanContext_8.front());

            cout << "build new node" << current_node_id << endl;
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

            cout << "mode " << mode << endl;

            Eigen::Vector3d currentNodePosition;
            currentNodePosition(0) = input_node.vertices_.Global_Pose_.x;
            currentNodePosition(1) = input_node.vertices_.Global_Pose_.y;
            currentNodePosition(2) = input_node.vertices_.Global_Pose_.z;
            distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                            + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                            + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

            sc_current = input_node.vertices_.scanContext_8.front();
            double similarityScore = getScoreFromLastNode(sc_current, sc_last);

            cout << "distance " << distanceFromPreNode << " score " << similarityScore << endl;
            PointType currentPose = input_node.vertices_.Global_Pose_;

            if(!mode) {//GNSS good
                if(distanceFromPreNode>=buildMapDisThres) {
                    int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node);//返回值为在nodes中的索引
                    //relocal succeed!
                    if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) && nodeKeyIndex[loopClosureIndex] != last_node_id) {
                        return;
                    }else {//relocalization failed!
                        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(input_node.vertices_.scanContext_8.front());
                        std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
                        global_scanContext.push_back(input_node.vertices_.scanContext_8.front());

                        node tmp_node = input_node;
                        tmp_node.vertices_.Global_Pose_.intensity = mode;
                        nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                        nodes.push_back(tmp_node);
                        sc_last = tmp_node.vertices_.scanContext_8.front();

                        PointType thisPose;
                        thisPose.x = tmp_node.vertices_.Global_Pose_.x;
                        thisPose.y = tmp_node.vertices_.Global_Pose_.y;
                        thisPose.z = tmp_node.vertices_.Global_Pose_.z;
                        globalKeyPose3d->push_back(thisPose);

                        PreviousNodePosition(0) = thisPose.x;
                        PreviousNodePosition(1) = thisPose.y;
                        PreviousNodePosition(2) = thisPose.z;

                        sc_last = sc_current;
                        cout << "build new node" << current_node_id << endl;
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
                    int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node);//返回值为在nodes中的索引
                    //relocal succeed!
                    if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) && nodeKeyIndex[loopClosureIndex] != last_node_id) {
                        return;
                    }else {//relocalization failed!
                        global_scanContext.push_back(input_node.vertices_.scanContext_8.front());

                        node tmp_node = input_node;
                        nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                        tmp_node.vertices_.Global_Pose_.intensity = mode;
                        nodes.push_back(tmp_node);
                        sc_last = tmp_node.vertices_.scanContext_8.front();

                        PointType thisPose;
                        thisPose.x = tmp_node.vertices_.Global_Pose_.x;
                        thisPose.y = tmp_node.vertices_.Global_Pose_.y;
                        thisPose.z = tmp_node.vertices_.Global_Pose_.z;
                        globalKeyPose3d->push_back(thisPose);

                        PreviousNodePosition(0) = thisPose.x;
                        PreviousNodePosition(1) = thisPose.y;
                        PreviousNodePosition(2) = thisPose.z;

                        sc_last = sc_current;
                        cout << "build new node" << current_node_id << endl;
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

    void runS(node& input_node)
    {
        if(isFirst) {
            isFirst = false;
            ////////////////////////////////////////////
            node tmp_node = input_node;
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
            tmp_node.vertices_.Global_Pose_.intensity = mode;
            nodes.push_back(tmp_node);

            sc_current = tmp_node.vertices_.scanContext_8.front();
            sc_last = sc_current;

            PointType thisPose;
            thisPose.x = tmp_node.vertices_.Global_Pose_.x;
            thisPose.y = tmp_node.vertices_.Global_Pose_.y;
            thisPose.z = tmp_node.vertices_.Global_Pose_.z;
            thisPose.intensity = current_node_id;
            globalKeyPose3d->push_back(thisPose);

            PreviousNodePosition(0) = thisPose.x;
            PreviousNodePosition(1) = thisPose.y;
            PreviousNodePosition(2) = thisPose.z;

            global_scanContext.push_back(tmp_node.vertices_.scanContext_8.front());

            cout << "build new node" << current_node_id << endl;
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
            currentNodePosition(0) = input_node.vertices_.Global_Pose_.x;
            currentNodePosition(1) = input_node.vertices_.Global_Pose_.y;
            currentNodePosition(2) = input_node.vertices_.Global_Pose_.z;
            distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                            + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                            + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

            sc_current = input_node.vertices_.scanContext_8.front();
            double similarityScore = getScoreFromLastNode(sc_current, sc_last);

            cout << "distance " << distanceFromPreNode << " score " << similarityScore << endl;
            PointType currentPose = input_node.vertices_.Global_Pose_;

            double score = similarityScore;
            if(score>=buildMapSimScore) {
                int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node);//返回值为在nodes中的索引
                //relocal succeed!
                if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) && nodeKeyIndex[loopClosureIndex] != last_node_id) {
                    return;
                }else {//relocalization failed!
                    global_scanContext.push_back(input_node.vertices_.scanContext_8.front());

                    node tmp_node = input_node;
                    nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                    tmp_node.vertices_.Global_Pose_.intensity = mode;
                    nodes.push_back(tmp_node);
                    sc_last = tmp_node.vertices_.scanContext_8.front();

                    PointType thisPose;
                    thisPose.x = tmp_node.vertices_.Global_Pose_.x;
                    thisPose.y = tmp_node.vertices_.Global_Pose_.y;
                    thisPose.z = tmp_node.vertices_.Global_Pose_.z;
                    globalKeyPose3d->push_back(thisPose);

                    PreviousNodePosition(0) = thisPose.x;
                    PreviousNodePosition(1) = thisPose.y;
                    PreviousNodePosition(2) = thisPose.z;

                    sc_last = sc_current;
                    cout << "build new node" << current_node_id << endl;
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

    void runD(node& input_node){
        if(isFirst) {
            isFirst = false;
            ////////////////////////////////////////////
            node tmp_node = input_node;
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
            tmp_node.vertices_.Global_Pose_.intensity = mode;
            nodes.push_back(tmp_node);

            sc_current = tmp_node.vertices_.scanContext_8.front();
            sc_last = sc_current;

            PointType thisPose;
            thisPose.x = tmp_node.vertices_.Global_Pose_.x;
            thisPose.y = tmp_node.vertices_.Global_Pose_.y;
            thisPose.z = tmp_node.vertices_.Global_Pose_.z;
            thisPose.intensity = current_node_id;
            globalKeyPose3d->push_back(thisPose);

            PreviousNodePosition(0) = thisPose.x;
            PreviousNodePosition(1) = thisPose.y;
            PreviousNodePosition(2) = thisPose.z;

            global_scanContext.push_back(tmp_node.vertices_.scanContext_8.front());

            cout << "build new node" << current_node_id << endl;
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
            currentNodePosition(0) = input_node.vertices_.Global_Pose_.x;
            currentNodePosition(1) = input_node.vertices_.Global_Pose_.y;
            currentNodePosition(2) = input_node.vertices_.Global_Pose_.z;
            distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                            + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                            + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

            sc_current = input_node.vertices_.scanContext_8.front();
            double similarityScore = node::getScore(nodes.back(), input_node);

            cout << "distance " << distanceFromPreNode << " score " << similarityScore << endl;
            PointType currentPose = input_node.vertices_.Global_Pose_;

            if(distanceFromPreNode>=buildMapDisThres) {
                int loopClosureIndex = loopClosureFromKDTree(currentPose, input_node);//返回值为在nodes中的索引
                //relocal succeed!
                if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) && nodeKeyIndex[loopClosureIndex] != last_node_id) {
                    return;
                }else {//relocalization failed!
                    global_scanContext.push_back(input_node.vertices_.scanContext_8.front());

                    node tmp_node = input_node;
                    tmp_node.vertices_.Global_Pose_.intensity = mode;
                    nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                    nodes.push_back(tmp_node);
                    sc_last = tmp_node.vertices_.scanContext_8.front();

                    PointType thisPose;
                    thisPose.x = tmp_node.vertices_.Global_Pose_.x;
                    thisPose.y = tmp_node.vertices_.Global_Pose_.y;
                    thisPose.z = tmp_node.vertices_.Global_Pose_.z;
                    globalKeyPose3d->push_back(thisPose);

                    PreviousNodePosition(0) = thisPose.x;
                    PreviousNodePosition(1) = thisPose.y;
                    PreviousNodePosition(2) = thisPose.z;

                    sc_last = sc_current;
                    cout << "build new node" << current_node_id << endl;
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
    if(index.size() > 0)
    {
        int node_number = index.size();
        nodes.resize(node_number);
        for(int i=0; i<node_number;++i)
        {
            node tmp_node;
            tmp_node.create_node_from_file_B(path, index[i]);
            nodes[i] = tmp_node;
        }
    }
    cout<<"number "<<nodes.size()<<" "<<index.size();
}

//normal
int buildN(){
    vector<node> nodeList;
    string loadPath = "/home/qh/robot_ws/map/allNode/2/";
    read_nodes_from_files_B(nodeList, loadPath);

    for(int i = 0; i < 11; i++){
        double k = 0.20 + i * 0.01;
        double buildValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N" + ss.str() + "/";
        buildTopo buildTopoMap(savePath, buildValue, 15.0);
        int nodesSize = nodeList.size();
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j++){
            //char key = getchar();
            //if(key == 'q') exit(0);
            chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            double nowPercent = j * 1.0 / nodesSize;
            double restPercent = 1.0 - nowPercent;
            double time_rest = time_used / nowPercent * restPercent;
            cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.run(nodeList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/N" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const node& n : resultNodeList){
            if(n.vertices_.Global_Pose_.intensity == 1){
                nognss.push_back(n.id_);
                fileout << n.vertices_.Global_Pose_.x << " "
                     << n.vertices_.Global_Pose_.y << " "
                     << n.vertices_.Global_Pose_.z << " " << 0 << endl;
            }else{
                gnss.push_back(n.id_);
                fileout << n.vertices_.Global_Pose_.x << " "
                     << n.vertices_.Global_Pose_.y << " "
                     << n.vertices_.Global_Pose_.z << " " << 1 << endl;
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
        fileout << endl;
        cout << "gnss " << gnss.size() << endl;
        cout << "nognss " << nognss.size() << endl;
        fileout.close();
    }
}
//只按照相似度建图
int buildS(){
    vector<node> nodeList;
    string loadPath = "/home/qh/robot_ws/map/allNode/2/";
    read_nodes_from_files_B(nodeList, loadPath);

    for(int i = 0; i < 10; i++){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double k = 0.23 + i * 0.03;
        double buildValue = k;
        stringstream ss;
        ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "/";
        buildTopo buildTopoMap(savePath, buildValue);
        int nodesSize = nodeList.size();
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j++){
            //char key = getchar();
            //if(key == 'q') exit(0);
            chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            double nowPercent = j * 1.0 / nodesSize;
            double restPercent = 1.0 - nowPercent;
            double time_rest = time_used / nowPercent * restPercent;
            cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.runS(nodeList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const node& n : resultNodeList){
            fileout << n.vertices_.Global_Pose_.x << " "
                    << n.vertices_.Global_Pose_.y << " "
                    << n.vertices_.Global_Pose_.z << " " << 0 << endl;
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/S" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "id: " << endl;
        for(const auto& n : resultNodeList){
            fileout << n.id_ << " ";
        }
        fileout.close();
    }
}

//只按照固定距离建图
int buildD(){
    vector<node> nodeList;
    string loadPath = "/home/qh/robot_ws/map/allNode/2/";
    read_nodes_from_files_B(nodeList, loadPath);

    vector<double> tab{0.5, 1.0, 3.0, 10.0, 15.0, 30.0};

    for(double k : tab){
        //char key = getchar();
        //if(key == 'q') exit(0);
        double buildValue = k;
        stringstream ss;
        ss << std::fixed << setprecision(1) << buildValue;
        string savePath = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "/";
        buildTopo buildTopoMap(savePath, 0.20, buildValue,15.0,0.23);
        int nodesSize = nodeList.size();
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for(int j = 0; j < nodesSize; j++){
            //char key = getchar();
            //if(key == 'q') exit(0);
            chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
            double nowPercent = j * 1.0 / nodesSize;
            double restPercent = 1.0 - nowPercent;
            double time_rest = time_used / nowPercent * restPercent;
            cout << nowPercent * 100  << "% "  << " rest:" << time_rest << "s ";
            buildTopoMap.runD(nodeList[j]);
        }
        cout << savePath << endl;
        vector<node>& resultNodeList = buildTopoMap.nodes;

        string fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(const auto& n : resultNodeList){
            fileout << n.vertices_.Global_Pose_.x << " "
                    << n.vertices_.Global_Pose_.y << " "
                    << n.vertices_.Global_Pose_.z << " " << 0 << endl;
        }
        fileout.close();
        fileResult = "/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/D" + ss.str() + "Info.txt";
        fileout.open(fileResult);
        fileout << "id: " << endl;
        for(const auto& n : resultNodeList){
            fileout << n.id_ << " ";
        }
        fileout.close();
    }
}

//daquan
int main4(){
    vector<node> nodeList;
    string loadPath = "/home/qh/robot_ws/map/allNode/4/";
    read_nodes_from_files_B(nodeList, loadPath);

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
            if(nodeList[j].vertices_.Global_Pose_.intensity==0) nodeList[j].vertices_.Global_Pose_.intensity = 2;
            buildTopoMap.runS(nodeList[j]);
        }
        vector<node>& resultNodeList = buildTopoMap.nodes;

        vector<int> gnss;
        vector<int> nognss;
        string fileResult = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/node/" + ss.str() + ".txt";
        ofstream fileout;
        fileout.open(fileResult);
        for(int i = 0; i < resultNodeList.size(); i++){
            node n = resultNodeList[i];
            if(n.vertices_.Global_Pose_.intensity == 1){
                nognss.push_back(n.id_);
                fileout << n.vertices_.Global_Pose_.x << " "
                        << n.vertices_.Global_Pose_.y << " "
                        << n.vertices_.Global_Pose_.z << " " << 0 << endl;
            }else{
                gnss.push_back(n.id_);
                fileout << n.vertices_.Global_Pose_.x << " "
                        << n.vertices_.Global_Pose_.y << " "
                        << n.vertices_.Global_Pose_.z << " " << 1 << endl;
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
        fileout << endl;
        cout << "gnss " << gnss.size() << endl;
        cout << "nognss " << nognss.size() << endl;
        fileout.close();
    }
}

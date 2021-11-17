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
double BestDis = 2;

class buildTopo{
public:
    bool isFirst{true};
    double disForDetech = 0;
    double W_sc = 0.6, W_dis = 0.4, alpha = 0.05;
    double finalThres = 0;
    //////////////////////////////////////
    bool save_data_to_files = false;
    std::string node_save_path;;
    //////////////////////////////////////
public:
    vector<node>nodes;
private:
    int current_node_id = 0;
    int last_node_id = -1;
    double D_th;
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
    vector<PointType> disFromClosestNearb(PointType currentPose, node& currNode, pcl::PointCloud<pcl::PointXYZI>& currCloud){
        vector<PointType> result;
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, disForDetech, pointSearchIndLoop, pointSearchSqDisLoop);
        for(int j  = 0; j < pointSearchIndLoop.size(); j++){
            int n = pointSearchIndLoop[j];
            if(currNode.time_stamp_ - nodes[n].time_stamp_ < 30.0){
                continue;
            }else{
                PointType tempPoint;
                tempPoint.x = n;
                tempPoint.y = sqrt(pointSearchSqDisLoop[j]);
                result.push_back(tempPoint);
            }
        }
        return result;
    }
    //////////////////////////////////////////////////////////
    static bool comSimAZ(PointType& A, PointType& B){
        return A.z < B.z;
    }
    //通过kd树搜索，返回闭环节点id
    vector<PointType> scanContextKnnSearch(PointType currentPose, node& currNode, pcl::PointCloud<pcl::PointXYZI>& currCloud)
    {
        vector<PointType> result;
        //使用位置进行KD树搜索
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, disForDetech, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        for(int j  = 0; j < pointSearchIndLoop.size(); j++){
            int n = pointSearchIndLoop[j];
            if(currNode.time_stamp_ - nodes[n].time_stamp_ < 30.0){
                continue;
            }else{
                PointType tempPoint;
                tempPoint.x = n;
                tempPoint.y = sqrt(pointSearchSqDisLoop[j]);
                double scoreNow = node::getScore(nodes[n], currNode, currCloud);
                tempPoint.z = scoreNow;
                result.push_back(tempPoint);
            }
        }
        sort(result.begin(), result.end(), comSimAZ);
        if(!result.empty()) return result;
        else{
            PointType tenmPoint;
            tenmPoint.x = -1, tenmPoint.y = -1, tenmPoint.z = -1;
            result.push_back(tenmPoint);
            return result;
        }
    }


    double getScoreFromLastNode(Eigen::MatrixXd& sc_current, Eigen::MatrixXd& sc_last)
    {
        double score = distDirectSC(sc_current, sc_last);
        return score;
    }

public:
    buildTopo(std::string node_save_path_, double finalThres_, double D_th_ = 2.0, double disForDetech_ = 8.0)
    {
        ////////////////////
        kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
        pubGlobalNodePosition.reset(new pcl::PointCloud<PointType>());
        /////////////////////
        disForDetech = disForDetech_;
        finalThres = finalThres_;
        D_th = D_th_;
        /////////////////////
        node_save_path = node_save_path_;
        save_data_to_files = false;//是否保存文件
        /////////////////////

    }
    void run(node input_node, pcl::PointCloud<pcl::PointXYZI> input_cloud)
    {
        if(isFirst) {
            isFirst = false;
            ////////////////////////////////////////////
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
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
            PointType currentPose = input_node.Global_Pose_;
            sc_current = input_node.scanContext_;
            bool isBuildANewNode = true;
            // double W_sc = 0.6, W_dis = 0.4, alpha = 0.2;
            // double D_th = BestDis
            // last node score 
            {
                double similarityScoreFromLastNode = getScoreFromLastNode(sc_current, sc_last);
                double D_sc = similarityScoreFromLastNode;
                distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
                                              + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
                                              + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));
                double D_now = distanceFromPreNode;
                double finalScore1 = 0;
                double y = exp(-alpha * pow((D_now - D_th), 2));
                if(D_now > 6) y = 1.0;
                finalScore1 = W_sc * D_sc + W_dis * y;
                if(finalScore1 < finalThres)
                    return;
            }
            // 10m closest history node score
            {
                vector<PointType> hisNearby = disFromClosestNearb(currentPose, input_node, input_cloud);//first his node id, second his node dis
                
                for(const auto& n : hisNearby){
                    double scoreNow = node::getScore(nodes[n.x], input_node, input_cloud);
                    double D_now = n.y;
                    double D_sc = scoreNow;
                    double finalScore2 = 0;
                    double y = exp(-alpha * pow((D_now - D_th), 2));
                    if(D_now > 6) y = 1.0;
                    finalScore2 = W_sc * D_sc + W_dis * y;
                    if(finalScore2 < finalThres)
                        return;
                }
            }

            if(isBuildANewNode){
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
    // cout << "get all node and cloud push any key to continue" << endl;
}

bool comZA(double A,double B){
    return A > B;
}

int nowThread = 0;
int nowIndex = 0;
int MaxThread = 1;



//normal
void buildN(double K_){
    double buildValue = K_;
    stringstream ss;
    ss << setprecision(2) << std::left << setfill('0') <<  setw(4) << buildValue;
    string savePath = pathBase + "/buildN/N" + ss.str() + "/";
    // if(0 == access(savePath.c_str(), 0)){
    //     nowThread--;
    //     return ;
    // }else if( 0 != access(savePath.c_str(), 0)){
    //     mkdir(savePath.c_str(), 0777);
    // }
    buildTopo buildTopoMap(savePath, buildValue, BestDis);
    int nodesSize = nodeList.size();

    for(int j = 0; j < nodesSize; j++){
        buildTopoMap.run(nodeList[j], cloudList[j]);
    }
    vector<node>& resultNodeList = buildTopoMap.nodes;

    string fileResult = pathBase + "/buildN/N" + ss.str() + ".txt";
    ofstream fileout;
    fileout.open(fileResult);
    for(const node& n : resultNodeList){
        fileout << n.Global_Pose_.x << " "
                << n.Global_Pose_.y << " "
                << n.Global_Pose_.z << " " << endl;
    }
    fileout.close();
    string pcdFile = pathBase + "/buildN/N" + ss.str() + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    for(const node& n : resultNodeList){
        pcl::PointXYZ tempP;
        tempP.x = n.Global_Pose_.x;
        tempP.y = n.Global_Pose_.y;
        tempP.z = n.Global_Pose_.z;
        tempCloud.push_back(tempP);
    }
    if(!tempCloud.empty()) pcl::io::savePCDFileASCII(pcdFile, tempCloud);
    nowThread--;
}


int main(int argc, char** argv){
    double buidlAValue = 0;
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
    if(argv[4] != nullptr){
        BestDis = stof(argv[4]);
        
        std::cout << " best dis " << BestDis << std::endl;
    }
    if(argv[5] != nullptr){
        buidlAValue = stof(argv[5]);
        std::cout << " best value " << buidlAValue << std::endl;
    }

    // pathBase = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL";
    // pathBase = "/home/qh/robot_ws/map/2021-08-30-18-06-30L";
    // MaxThread = 6;


    ros::init(argc, argv, "recallD");
    ros::NodeHandle nh("~");
    
    shiftNeiborA.push_back(4);
    shiftNeiborA.push_back(5);
    shiftNeiborA.push_back(6);
    shiftNeiborInc.push_back(0.1);
    shiftNeiborInc.push_back(0.3);
    shiftNeiborInc.push_back(0.5);
    shiftNeiborInc.push_back(0.7);
    shiftNeiborInc.push_back(0.9);
    
    init();

    vector<double> simBuildTopoValue;
    // {
    //     // 0.10,
    //     // 0.20,
    //     // 0.30,
    //     // 0.40,
    //     // 0.50,  0.53, 0.55, 0.58,
    //     // 0.60,  0.63, 0.65, 0.68,
    //     // 0.70,  0.73, 0.75, 0.78,
    //     // 0.80,  0.83, 0.85, 0.88,
    //     // 0.90
    //     // 0.60
    //     };
    simBuildTopoValue.push_back(buidlAValue);

    double timeAll = 0;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    ros::Rate loop(10);
    sort(simBuildTopoValue.begin(), simBuildTopoValue.end(), comZA);
    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < simBuildTopoValue.size()){
            cout << nowIndex << " add a new thread Sth " << simBuildTopoValue[nowIndex] << endl;
            nowThread++;
            thread* tempT(new thread(buildN, simBuildTopoValue[nowIndex]));
            nowIndex++;
        }
        if(nowIndex == simBuildTopoValue.size() && nowThread == 0)
        {
            cout << " endl " << endl;
            break;
        }
        loop.sleep();
    }


   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
   chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
   timeAll = time_used.count() * 1000;
   cout << pathBase << endl;
   cout << fixed << setprecision(3) << "handle time cost: " << (timeAll * 1000) << " ms. " << endl;
   cout << fixed << setprecision(3) << "frame " << nodeList.size() << " poses." << endl;
   cout << fixed << setprecision(3) << "per " << timeAll/nodeList.size() << " ms." << endl;

    return 0;
}
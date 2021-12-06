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

class buildTopo{
private:
    bool isFirst{true};
    double buidlMapDis = 0;
    //////////////////////////////////////
    bool save_data_to_files = false;
    std::string node_save_path;;
    //////////////////////////////////////
public:
    vector<node>nodes;
private:
    int current_node_id = 0;
    int last_node_id = -1;
    ////////////////////////////////////// 
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistroyKeyPoses;
    //////////////////////////////////////////////////////////
    pcl::PointCloud<PointType>::Ptr globalKeyPose3d;
    //////////////////////////////////////////////////////////
    unordered_map<int,int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
    //////////////////////////////////////////////////////////
    int loopClosureFromKDTreeDis(PointType currentPose){
        std::vector<int>pointSearchIndLoop;
        std::vector<float>pointSearchSqDisLoop;
        kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
        kdtreeHistroyKeyPoses->radiusSearch(currentPose, buidlMapDis, pointSearchIndLoop, pointSearchSqDisLoop);
        if(pointSearchIndLoop.empty()) return -1;
        else return pointSearchIndLoop.front();
    }


public:
    buildTopo(std::string node_save_path_, double buildMapDisDis)
    {
        ////////////////////
        kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
        /////////////////////
        buidlMapDis = buildMapDisDis;//todo
        /////////////////////
        node_save_path = node_save_path_;
        save_data_to_files = true;//是否保存文件
        /////////////////////
    }

    void runD(node input_node){
        if(isFirst) {
            isFirst = false;
            ////////////////////////////////////////////
            nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
            nodes.push_back(input_node);

            PointType thisPose;
            thisPose.x = input_node.Global_Pose_.x;
            thisPose.y = input_node.Global_Pose_.y;
            thisPose.z = input_node.Global_Pose_.z;
            thisPose.intensity = current_node_id;
            globalKeyPose3d->push_back(thisPose);

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
            PointType currentPose = input_node.Global_Pose_;

            int loopClosureIndex = loopClosureFromKDTreeDis(currentPose);//返回值为在nodes中的索引
            if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex) ) {
                return;
            }else{
                nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
                nodes.push_back(input_node);

                PointType thisPose;
                thisPose.x = input_node.Global_Pose_.x;
                thisPose.y = input_node.Global_Pose_.y;
                thisPose.z = input_node.Global_Pose_.z;
                globalKeyPose3d->push_back(thisPose);

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
            nodes[i].create_node_from_file_B(path, index[i]);
        }
    }
}

vector<node> nodeList;


void init(){
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
    read_nodes_from_files_B(nodeList, loadNodePath);

    int cloudSizePre = file_nameList.size();
    cout << "all node " << nodeList.size() << endl;
}

int nowThread = 0;
int nowIndex = 0;
int MaxThread = 0;

void buildD(double K_){
    double buildValue = K_;
    stringstream ss;
    ss << fixed << setprecision(1) << buildValue;
    string savePath = pathBase + "/buildD/D" + ss.str() + "/";
    if(0 == access(savePath.c_str(), 0)){
        nowThread--;
        return ;
    }else if( 0 != access(savePath.c_str(), 0)){
        mkdir(savePath.c_str(), 0777);
    }
    buildTopo buildTopoMap(savePath, buildValue);
    for(int j = 0; j < nodeList.size(); j++){
        buildTopoMap.runD(nodeList[j]);
    }
    vector<node>& resultNodeList = buildTopoMap.nodes;

    string fileResult = pathBase + "/buildD/D" + ss.str() + ".txt";
    ofstream fileout;
    fileout.open(fileResult);
    for(const node& n : resultNodeList){
        fileout << n.Global_Pose_.x << " "
                << n.Global_Pose_.y << " "
                << n.Global_Pose_.z << " " << endl;
    }
    fileout.close();
    string pcdFile = pathBase + "/buildD/D" + ss.str() + ".pcd";
    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    for(const node& n : resultNodeList){
        pcl::PointXYZ tempP;
        tempP.x = n.Global_Pose_.x;
        tempP.y = n.Global_Pose_.y;
        tempP.z = n.Global_Pose_.z;
        tempCloud.push_back(tempP);
    }
    pcl::io::savePCDFileASCII(pcdFile, tempCloud);
    nowThread--;
}
bool comB(double A,double B){
    return A>B;
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

    ros::init(argc, argv, "buildD");
    ros::NodeHandle nh("~");
    init();

    cout << " getstart " << endl;

    vector<double> disTable{
        0.2, 0.5, 0.6, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 7, 8, 9
    };

    sort(disTable.begin(), disTable.end(), comB);
    for(auto n : disTable){
        cout << n << endl;
    }

    ros::Rate loop(1);

    while(ros::ok()){
        if(nowThread<MaxThread&&nowIndex < disTable.size()){
            cout << nowIndex << " add a new thread D " << fixed << setprecision(2) << disTable[nowIndex] << endl;
            nowThread++;
            thread* tempT(new thread(buildD, disTable[nowIndex]));
            nowIndex++;
        }
        if(nowIndex == disTable.size() && nowThread == 0)
        {
            cout << " end " << endl;
            break;
        }
        loop.sleep();
    }

    cout << " end " << endl;
    return 0;
}
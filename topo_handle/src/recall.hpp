// #include "include/utility.h"
// #include "include/dbscan/node_context_localMap.h"

// #include "cmath"
// #include "unordered_set"
// #include "unordered_map"
// #include "deque"


// class topomapRecall{
// private:
//     int bufferQueLength;
//     double scoreBetweenScanContext;
//     double scoreThreshold;
//     deque<pcl::PointCloud<PointType>>laserCloudQue;
//     deque<double>lidarTimeStampQue;
//     deque<Eigen::Isometry3d>odometryQue;
//     bool newlaserOdometry = false;
//     bool newlaserCloud    = false;
//     double laserCloudTime = 0;
//     double laserOdomTime  = 0;
//     pcl::PointCloud<PointType>::Ptr localMap;
//     pcl::PointCloud<PointType>::Ptr arouncMapShow;
//     //////////////////////////////////////
//     bool save_data_to_files;
//     std::string node_save_path;
//     std::string node_load_path;
//     //////////////////////////////////////
//     vector<node>oldNodes;
//     vector<node>newNodes;
//     std::vector<float>dataNeibor{-2, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0};
//     double distanceThreshold;

//     //////////////////////////////////////
//     double distanceFromPreNode = 0.0;      //距离上一个拓扑节点的间距
//     Eigen::Vector3d PreviousNodePosition = Eigen::Vector3d::Zero();     //上一个拓扑节点key的index
//     Eigen::Isometry3d latestNodePose = Eigen::Isometry3d::Identity();   //上一个拓扑节点的旋转平移矩阵（世界坐标系下）
//     pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistroyKeyPoses;
//     pcl::KdTreeFLANN<PointType> distanceKDTree;
//     pcl::PointCloud<PointType>::Ptr allOldPose;
//     Eigen::MatrixXd sc_current, sc_last;
//     unordered_set<int> recalledNode;
//     //////////////////////////////////////
//     bool relocation{false};
//     bool create_history_nodes{false};
//     //////////////////////////////////////////////////////////
//     std::vector<Eigen::MatrixXd>global_scanContext;                 //存储全局描述子
//     KeyMat globalScanContextRingKey;                                //存储全局描述子的ringkey
//     std::unique_ptr<InvKeyTree>scanContextTree;                     //用于构建描述子搜索树
//     pcl::PointCloud<PointType>::Ptr globalKeyPose3d;
//     Eigen::Isometry3d relativeMeasurementsToLastNode = Eigen::Isometry3d::Identity();   //当前节点与上一个节点相对旋转平移
//     Eigen::Isometry3d currentNodePose = Eigen::Isometry3d::Identity();                  //当前节点在世界坐标系下的旋转平移
//     //////////////////////////////////////////////////////////
//     pcl::PointCloud<PointType>::Ptr pubGlobalNodePosition;  //存储每个节点的全局位置
//     pcl::PointCloud<PointType>::Ptr pubGlobalNodePositionNew;  //存储每个节点的全局位置

//     //////////////////////////////////////////////////////////
//     //////////////////////////////////////////////////////////
//     //////////////////////////////////////////////////////////
//     unordered_map<int, int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
//     unordered_map<int,int>allNodeIndexId;           //第一个为节点的id，第二个为在nodes中的索引
//     //////////////////////////////////////////////////////////
//     //////////////////////////////////////////////////////////
//     void read_nodes_from_files(std::vector<node>& nodes, std::string& path)
//     {
//         std::vector<std::string>file_name;
//         std::vector<int>index;
//         DIR *d = opendir(path.c_str());
//         struct dirent *dp;
//         while((dp = readdir(d)) != NULL)
//         {
//             if(dp->d_name[0] == '.')    {continue;}
//             index.push_back(atoi(dp->d_name));
//         }
//         sort(index.begin(), index.end());
//         closedir(d);
//         int max_id_in_file = INT_MIN;
//         int success_number = 0;
//         int node_index = 0;
//         if(index.size() > 0)
//         {
//             int node_number = index.size();
//             nodes.resize(node_number);
//             for(int i=0;i<node_number; ++i)
//             {
//                 node tmp_node;
//                 tmp_node.create_node_from_file(path, index[i]);
//                 allNodeIndexId.insert({tmp_node.id_, i});// node id ,index
//             }

//             for(int i=0; i<node_number;++i)
//             {
//                 node tmp_node;
//                 tmp_node.create_node_from_file(path, index[i]);
//                 //std::cout<<tmp_node.id_<<" ";
//                 nodes[node_index] = tmp_node;
//                 nodeKeyIndex.insert({node_index, tmp_node.id_});    //index node id
//                 max_id_in_file = std::max(max_id_in_file, (int)tmp_node.id_);
//                 PointType tmp_global_pose;

//                 tmp_global_pose = tmp_node.vertices_.Global_Pose_;
//                 tmp_global_pose.intensity = node_index;
//                 globalKeyPose3d->push_back(tmp_global_pose);        //存储全局地图
//                 global_scanContext.push_back(nodes[node_index].vertices_.scanContext_); //存储拓扑节点中的描述子
//                 globalScanContextRingKey.push_back(nodes[node_index].vertices_.scanContextRingKey_);

//                 PointType node_position;
//                 node_position = nodes[node_index].vertices_.Global_Pose_;
//                 node_position.intensity = nodes[node_index].id_;

//                 pubGlobalNodePosition->push_back(node_position);

//                 success_number++;
//                 node_index++;
//             }
//             //cout<<endl;
//         }
//         for(auto n:nodes){
//             PointType tempPoint;
//             tempPoint.x = n.vertices_.Global_Pose_.x;
//             tempPoint.y = n.vertices_.Global_Pose_.y;
//             tempPoint.z = n.vertices_.Global_Pose_.z;
//             allOldPose->push_back(tempPoint);
//         }
//         distanceKDTree.setInputCloud(allOldPose);
//         //cout<<endl;
//         //cout<<"number "<<nodes.size()<<" "<<node_index<<endl;
//         //cout<<"nodes in file number is "<<index.size()<<endl;
//         //cout<<"global "<<global_scanContext.size()<<endl;
//         //std::cout<<"success number is "<<success_number<<std::endl;
//         //std::cout<<std::endl;
//     }

//     int loopClosureFromKDTree(Eigen::MatrixXd& sc)
//     {
//         Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
//         std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
//         int globalRingKeyNumber = globalScanContextRingKey.size();
//         std::unordered_map<int, int>possibleNodeIndex;
//         possibleNodeIndex.clear();
//         if(globalRingKeyNumber > 1)
//         {
//             KeyMat scanContextRingKeyToSearch;      //要进行搜索的树
//             scanContextRingKeyToSearch.clear();
//             scanContextRingKeyToSearch.assign(globalScanContextRingKey.begin(),
//                                               globalScanContextRingKey.end()-1);
//             scanContextTree.reset();
//             scanContextTree = std::make_unique<InvKeyTree>(PC_NUM_RING, scanContextRingKeyToSearch, 10);
//             int result = (int)scanContextKnnSearch(polarcontext_vkeys_vec, sc);
//             if(result == -1)
//             {
//                 return -1;
//             }else
//                 return result;
//         }
//         return -1;
//     }
//     //通过kd树搜索，返回闭环节点id
//     int scanContextKnnSearch(std::vector<float>& currRingKey, Eigen::MatrixXd& currScanContext)
//     {
//         //当前局部地图位置
//         int keyFrameIndex = bufferQueLength/2;
//         Eigen::Isometry3d keyPose = odometryQue.at(keyFrameIndex);
//         PointType currentPose;
//         currentPose.x = keyPose.translation().x();
//         currentPose.y = keyPose.translation().y();
//         currentPose.z = keyPose.translation().z();

//         double min_dist = 10000000;
//         int nn_align = 0, nn_idx = 0;   //nn_align为描述子旋转的角度值， nn_idx为匹配成功的索引值
//         int loop_index = -1;
//         // //knn search
//         // std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);                  //存放索引值数组
//         // std::vector<float> distance_sqrt(NUM_CANDIDATES_FROM_TREE);                       //存放距离值数组
//         // nanoflann::KNNResultSet<float>knnsearchResult(NUM_CANDIDATES_FROM_TREE);          //
//         // knnsearchResult.init(&candidate_indexes[0], &distance_sqrt[0]);
//         // scanContextTree->index->findNeighbors(knnsearchResult,&currRingKey[0],nanoflann::SearchParams(NUM_CANDIDATES_FROM_TREE));
//         //使用位置进行KD树搜索
//         std::vector<int>pointSearchIndLoop;
//         std::vector<float>pointSearchSqDisLoop;
//         kdtreeHistroyKeyPoses->setInputCloud(globalKeyPose3d);
//         //recall distance is 3.0M
//         kdtreeHistroyKeyPoses->radiusSearch(currentPose, distanceThreshold, pointSearchIndLoop, pointSearchSqDisLoop, 0);

//         unordered_set<int>proposeIndex;
//         //可能存在闭环的拓扑节点

// //        cout << "near points :" ;
//         for(int i=0; i<pointSearchIndLoop.size(); ++i)
//         {
// //            cout << " ************************" << endl;
// //            cout << pointSearchIndLoop[i] << "| " << endl;
// //            cout << globalKeyPose3d->points[pointSearchIndLoop[i]].x << " "
// //            << globalKeyPose3d->points[pointSearchIndLoop[i]].y << " "
// //            << globalKeyPose3d->points[pointSearchIndLoop[i]].z << " " << endl;
// //            cout << "score is " << distanceBtnScanContext(currScanContext, global_scanContext[pointSearchIndLoop[i]]).first << endl;
// //            cout << " ************************" << endl;
//             auto it = recalledNode.find(pointSearchIndLoop[i]);
//             if(it != recalledNode.end()){
//                 continue;
//             }
//             proposeIndex.insert(pointSearchIndLoop[i]);
//         }



//         for(int i = 0; i<dataNeibor.size(); ++i)
//         {
//             float shift = dataNeibor[i];
//             pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
//             tmpCloud->clear();
//             for(auto point:localMap->points)
//             {
//                 point.z += shift;
//                 tmpCloud->push_back(point);
//             }
//             Eigen::MatrixXd currentContextShift = makeScancontext(tmpCloud);
//             for(auto index:proposeIndex)
//             {
//                 Eigen::MatrixXd scanContextCandidate = global_scanContext[index];
//                 std::pair<double, int> sc_dist_result = distanceBtnScanContext(currentContextShift, scanContextCandidate);
//                 double candidate_dist = sc_dist_result.first;       //余弦距离
//                 int candidate_align = sc_dist_result.second;        //偏移量

//                 if(candidate_dist < min_dist)
//                 {
//                     min_dist = candidate_dist;                      //两个描述子之间的余弦距离
//                     nn_align = candidate_align;
//                     nn_idx = index;    //搜索到的描述子的索引值
//                 }
//             }
//         }

//         //设定余弦距离最小值
//         if(min_dist < scoreBetweenScanContext)
//         {
//             loop_index = nodeKeyIndex[nn_idx];
//         }
//         //registerIndex的作用是为了避免当前帧多个数据匹配到同一个历史帧上
//         if(loop_index!=-1 )
//         {
//             //cout << setprecision(5) << setw(6)<<"min_dist "<<min_dist<<endl;
//             return nn_idx;
//         }else{
//             //cout << setprecision(5) << setw(6)<<"min_dist "<<min_dist<<endl;
//             return -1;
//         }

//     }


//     bool createLocalMap()
//     {
//         while((int)laserCloudQue.size() > bufferQueLength)
//         {
//             laserCloudQue.pop_front();
//             lidarTimeStampQue.pop_front();
//         }
//         while((int)odometryQue.size() > bufferQueLength)
//         {
//             odometryQue.pop_front();
//         }
//         if((int)laserCloudQue.size() == (int)odometryQue.size() && (int)laserCloudQue.size() == bufferQueLength)
//         {
//             localMap->clear();
//             arouncMapShow->clear();
//             Eigen::Isometry3d keyMatrix = odometryQue.at(odometryQue.size()/2).inverse();
//             int queSize = laserCloudQue.size();
//             pcl::PointCloud<PointType>::Ptr tmpMap(new pcl::PointCloud<PointType>() );
//             for(int i=0; i<queSize; ++i)
//             {
//                 *arouncMapShow += laserCloudQue.at(i);
//                 Eigen::Matrix4d transMatrix = keyMatrix.matrix();
//                 pcl::transformPointCloud(laserCloudQue.at(i), *tmpMap, transMatrix);
//                 *localMap += *tmpMap;
//                 tmpMap->clear();
//             }
//             return true;
//         }
//         //std::cout << "数量" << laserCloudQue.size() << std::endl;//qh add for debug
//         return false;
//     }
// public:
//     topomapRecall(std::string node_load_path_, std::string node_save_path_, double distanceThreshold_ = 3.0, double similityThres = 0.20 )
//     {
//         ////////////////////
//         kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
//         globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
//         localMap.reset(new pcl::PointCloud<PointType>());
//         arouncMapShow.reset(new pcl::PointCloud<PointType>());
//         pubGlobalNodePosition.reset(new pcl::PointCloud<PointType>());
//         pubGlobalNodePositionNew.reset(new pcl::PointCloud<PointType>());
//         allOldPose.reset(new pcl::PointCloud<PointType>());
//         node_save_path = node_save_path_;
//         node_load_path = node_load_path_;


//         relocation = true;
//         bufferQueLength = 13;

//         scoreBetweenScanContext = similityThres;//描述子余弦距离
//         save_data_to_files = true;//是否保存文件

//         distanceThreshold = distanceThreshold_;

//         create_history_nodes = true;//是否从已有文件创建节点
//         /////////////////////
//         if(create_history_nodes)
//         {
//             string file_path = node_load_path;
//             read_nodes_from_files(oldNodes, file_path);
//             //cout << "old Node nums " << oldNodes.size() << endl;
//         }
//         /////////////////////
//     }
//     void run(sensor_msgs::PointCloud2 remove_obstacle_cloud,
//              nav_msgs::Odometry remove_obstacle_odom,
//              //输出
//              sensor_msgs::PointCloud2& oldNode_position_,
//              sensor_msgs::PointCloud2& newNode_position_,
//              sensor_msgs::PointCloud2& cloud_local_map,
//              bool& recallFlag)
//     {
//         /////////////
//         if(remove_obstacle_cloud.data.empty()){//初始的时候可能为空，因此判断{
//             return;
//         }
//         /////////////
//         ////////////////////
//         laserOdomTime = remove_obstacle_odom.header.stamp.toSec();
//         Eigen::Isometry3d currentPose = Eigen::Isometry3d::Identity();
//         currentPose.rotate(Eigen::Quaterniond(remove_obstacle_odom.pose.pose.orientation.w,
//                                               remove_obstacle_odom.pose.pose.orientation.x,
//                                               remove_obstacle_odom.pose.pose.orientation.y,
//                                               remove_obstacle_odom.pose.pose.orientation.z));
//         currentPose.pretranslate(Eigen::Vector3d(remove_obstacle_odom.pose.pose.position.x,
//                                                  remove_obstacle_odom.pose.pose.position.y,
//                                                  remove_obstacle_odom.pose.pose.position.z));
//         odometryQue.push_back(currentPose);
//         newlaserOdometry = true;
//         ///////////////////////
//         pcl::PointCloud<PointType> LaserCloud;
//         pcl::fromROSMsg(remove_obstacle_cloud, LaserCloud);
//         laserCloudTime = remove_obstacle_cloud.header.stamp.toSec();
//         ///////////////////////
//         laserCloudQue.push_back(LaserCloud);
//         lidarTimeStampQue.push_back(laserCloudTime);
//         LaserCloud.clear();
//         newlaserCloud = true;
//         //////////////////////

//         //////////////////////
//         //////////////////////
//         /////////////main handle/////////////////////
//         if(newlaserCloud && newlaserOdometry)
//         {
//             newlaserCloud = false;
//             newlaserOdometry = false;
//             if(createLocalMap())
//             {
//                 ///////////////////////pub to show
//                 pcl::toROSMsg(*arouncMapShow, cloud_local_map);
//                 cloud_local_map.header = remove_obstacle_cloud.header;

//                 //////////////////////recallrelocation
//                 if(true)
//                 {
//                     sc_current = makeScancontext(localMap);
//                     int loopClosureIndex = loopClosureFromKDTree(sc_current);
//                     if(loopClosureIndex != -1)//重定位成功
//                     {
//                         if(!nodeKeyIndex.count(loopClosureIndex))
//                         {
//                             return;
//                         }
//                         int loopNodeId = nodeKeyIndex[loopClosureIndex];
//                         auto it = recalledNode.find(loopClosureIndex);
//                         if(it == recalledNode.end()){
//                             recalledNode.insert(loopClosureIndex);
//                         }else{
//                             localMap->clear();
//                             return;
//                         }
//                         //cout << setprecision(5) << setw(6)<<"succeed "<<loopNodeId<<" index "<<loopClosureIndex<<std::endl;
//                         newNodes.push_back(oldNodes[loopClosureIndex]);
//                         PointType thisPose;
//                         thisPose.x = oldNodes[loopClosureIndex].vertices_.Global_Pose_.x;
//                         thisPose.y = oldNodes[loopClosureIndex].vertices_.Global_Pose_.y;
//                         thisPose.z = oldNodes[loopClosureIndex].vertices_.Global_Pose_.z;
//                         PointType tempPose;
//                         int keyFrameIndex = bufferQueLength/2;
//                         Eigen::Isometry3d keyPose = odometryQue.at(keyFrameIndex);
//                         tempPose.x = keyPose.translation().x();
//                         tempPose.y = keyPose.translation().y();
//                         tempPose.z = keyPose.translation().z();
//                         double minDisTemp = sqrt(pow(thisPose.x-tempPose.x,2) + pow(thisPose.y-tempPose.y,2) + pow(thisPose.z-tempPose.z,2));
//                         //cout << setprecision(5) << setw(6) << "min----> " << loopClosureIndex << " " << minDisTemp << endl;
//                         pubGlobalNodePositionNew->push_back(thisPose);
//                         if(save_data_to_files) {
//                             string file_path = node_save_path;
//                             oldNodes[loopClosureIndex].nodes_save(file_path);
//                         }
//                         recallFlag = true;
//                         localMap->clear();
//                         return;
//                     }
//                     else//重定位失败
//                     {
//                         recallFlag = false;
//                         //std::cout<<"lost" << endl;
//                     }
//                 }

//             }
//             localMap->clear();
//         }
//         /////////////main handle/////////////////////
//         sensor_msgs::PointCloud2 pubNodePositionTmp_;
//         pcl::toROSMsg(*pubGlobalNodePosition, pubNodePositionTmp_);
//         pubNodePositionTmp_.header.stamp = remove_obstacle_cloud.header.stamp;
//         pubNodePositionTmp_.header.frame_id = remove_obstacle_cloud.header.frame_id;
//         oldNode_position_ = pubNodePositionTmp_;

//         pcl::toROSMsg(*pubGlobalNodePositionNew, pubNodePositionTmp_);
//         pubNodePositionTmp_.header.stamp = remove_obstacle_cloud.header.stamp;
//         pubNodePositionTmp_.header.frame_id = remove_obstacle_cloud.header.frame_id;
//         newNode_position_ = pubNodePositionTmp_;
//     }
// };
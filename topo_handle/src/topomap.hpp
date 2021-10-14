// #include "include/utility.h"
// #include "include/dbscan/node_context_localMap.h"

// #include "cmath"
// #include "unordered_set"
// #include "unordered_map"
// #include "deque"

// //double
// int64_t __NaN=0xFFF8000000000000,__Infinity=0x7FF0000000000000,__Neg_Infinity=0xFFF0000000000000;
// const double NaN=*((double *)&__NaN),Infinity=*((double *)&__Infinity),Neg_Infinity=*((double *)&__Neg_Infinity);

// bool IsNaN(double dat)
// {
//     int64_t & ref=*(int64_t *)&dat;
//     return (ref&0x7FF0000000000000) == 0x7FF0000000000000 && (ref&0xfffffffffffff)!=0;
// }

// class topomap{
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
//     double weight_distance, weight_similarity, weight_intersection;
//     bool save_data_to_files;
//     std::string node_save_path;;
//     //////////////////////////////////////
//     vector<node>nodes;
//     int current_node_id = 0;
//     int last_node_id    = -1;
//     std::vector<float>dataNeibor{-2, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0};
//     double distanceThreshold;
//     double scanContextThreshold;
//     double distanceInterThreshold;
//     //////////////////////////////////////
//     double distanceFromPreNode = 0.0;      //距离上一个拓扑节点的间距
//     Eigen::Vector3d PreviousNodePosition = Eigen::Vector3d::Zero();     //上一个拓扑节点key的index
//     Eigen::Isometry3d latestNodePose = Eigen::Isometry3d::Identity();   //上一个拓扑节点的旋转平移矩阵（世界坐标系下）
//     pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistroyKeyPoses;
//     Eigen::MatrixXd sc_current, sc_last;
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
//     //////////////////////////////////////////////////////////
//     Eigen::Vector3d lastIntersection    = Eigen::Vector3d::Zero();
//     Eigen::Vector3d currentIntersection = Eigen::Vector3d::Zero();
//     bool newIntersection = false;
//     double distanceFromPreInter = 0;
//     //////////////////////////////////////////////////////////
//     int node_number = 0;
//     int node_success = 0;
//     int removeId = 0;
//     int successFreqTh = 0;
//     //////////////////////////////////////////////////////////
//     unordered_map<int, int>nodeKeyIndex;            //第一个为在nodes中的索引，第二个为节点的id
//     unordered_map<int,int>allNodeIndexId;           //第一个为节点的id，第二个为在nodes中的索引
//     //////////////////////////////////////////////////////////
//     double cost_time = 0;
//     double current_time = 0;
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
//                 if(tmp_node.success_call_freq_ >successFreqTh || tmp_node.id_ >= removeId)
//                 {
//                     std::cout<<tmp_node.id_<<" ";
//                     nodes[node_index] = tmp_node;

//                     nodeKeyIndex.insert({node_index, tmp_node.id_});    //index node id

//                     max_id_in_file = std::max(max_id_in_file, (int)tmp_node.id_);
//                     PointType tmp_global_pose;

//                     tmp_global_pose = tmp_node.vertices_.Global_Pose_;
//                     tmp_global_pose.intensity = node_index;
//                     globalKeyPose3d->push_back(tmp_global_pose);        //存储全局地图
//                     global_scanContext.push_back(nodes[node_index].vertices_.scanContext_); //存储拓扑节点中的描述子
//                     globalScanContextRingKey.push_back(nodes[node_index].vertices_.scanContextRingKey_);

//                     PointType node_position;
//                     node_position = nodes[node_index].vertices_.Global_Pose_;
//                     node_position.intensity = nodes[node_index].id_;

//                     pubGlobalNodePosition->push_back(node_position);

//                     success_number++;
//                     node_index++;
//                 }else{
//                     // remove_redundant_nodes(path,index,i);
//                 }
//             }
//             cout<<endl;
//             nodes.resize(node_index);
//             current_node_id = max_id_in_file+1;
//             // last_node_id = max_id_in_file;
//             last_node_id = -1;
//         }
//         for(auto n:nodes)
//         {
//             cout<<n.id_<<" ";
//         }
//         cout<<endl;
//         cout<<"number "<<nodes.size()<<" "<<node_index<<endl;
//         cout<<"nodes in file number is "<<index.size()<<endl;
//         cout<<"global "<<global_scanContext.size()<<endl;
//         std::cout<<"success number is "<<success_number<<std::endl;

//         std::cout<<std::endl;
//         relocation = true;
//     }
//     bool creataEdge(std::vector<node>&nodes, int father_id, int child_id)
//     {
//         Edge edge;
//         edge.father_id_ = nodes[father_id].id_;
//         edge.child_id_  = nodes[child_id].id_;
//         Eigen::Isometry3d fatherMatrix  = nodes[father_id].vertices_.globalTransformMatrix_;
//         Eigen::Isometry3d childMatrix   = nodes[child_id].vertices_.globalTransformMatrix_;

//         edge.keyTransformMatrix_ = fatherMatrix.inverse()*childMatrix;
//         nodes[father_id].as_father_edges_.push_back(edge);
//         nodes[child_id].as_child_edges_.push_back(edge);
//         return true;
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
//         kdtreeHistroyKeyPoses->radiusSearch(currentPose, 8.0, pointSearchIndLoop, pointSearchSqDisLoop, 0);

//         unordered_set<int>proposeIndex;
//         //可能存在闭环的拓扑节点
//         for(int i=0; i<pointSearchIndLoop.size(); ++i)
//         {
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
//             for(auto index:proposeIndex
//             )
//             {
//                 if(nodeKeyIndex[index] == last_node_id)
//                     continue;
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
//         cout<<"KnnSearch min_dist "<<min_dist<<" index is "<<nn_idx<<" node id "<<(int)nodeKeyIndex[nn_idx]<<" last_node_id "<<last_node_id<<" rotation "<<nn_align<<std::endl;
//         if(nodeKeyIndex[nn_idx] == last_node_id || std::fabs(nodes[nn_idx].time_stamp_ - lidarTimeStampQue.at(lidarTimeStampQue.size()/2))<=30)
//         {
//             return -1;
//         }
//         //设定余弦距离最小值
//         if(min_dist < scoreBetweenScanContext)
//         {
//             loop_index = nodeKeyIndex[nn_idx];
//         }
//         //registerIndex的作用是为了避免当前帧多个数据匹配到同一个历史帧上
//         if(loop_index!=-1 )
//         {
//             cout<<"min_dist "<<min_dist<<endl;
//             cout<<"global "<<global_scanContext.size()<<" "<<globalKeyPose3d->points.size()<<endl;
//             cout<<"scanContextKnnSearch "<<loop_index<<" return is "<<globalKeyPose3d->points[nn_idx].intensity<<endl;
//             return nn_idx;
//         }else
//             return -1;

//     }
//     //上一个节点如果有孩子节点，从孩子节点中使用描述符判断是否有相同的闭环点
//     int loopClouserFromChild(Eigen::MatrixXd& sc)
//     {
//         int father_node_id = last_node_id;
//         int father_id = findIndexInNodes(nodes, father_node_id);
//         if(father_id == -1)
//         {
//             std::cout<<"loopClouserFromChild father id == -1"<<std::endl;
//             return -1;
//         }
//         std::cout<<"father id "<<father_node_id<<" "<<father_node_id<<std::endl;
//         int asFatherEdgeNumber = nodes[father_id].as_father_edges_.size();
//         if(asFatherEdgeNumber<=0 || !isfinite(asFatherEdgeNumber))
//         {
//             return -1;
//         }
//         std::unordered_map<int, double>possibleChildIndex; //key为可能的节点的index，value为个数
//         possibleChildIndex.clear();
//         for(int index=0; index<dataNeibor.size(); ++index)
//         {
//             float shift = dataNeibor[index];
//             pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
//             tmpCloud->clear();
//             for(auto point:localMap->points)
//             {
//                 point.z += shift;
//                 tmpCloud->push_back(point);
//             }
//             Eigen::MatrixXd currentContextShift = makeScancontext(tmpCloud);

//             for(int i=0; i<asFatherEdgeNumber; ++i)
//             {
//                 int child_id = nodes[father_id].as_father_edges_[i].child_id_;
//                 int child_index_in_nodes = findIndexInNodes(nodes, child_id);
//                 if(child_index_in_nodes ==-1)
//                     continue;

//                 auto result = distanceBtnScanContext(currentContextShift, nodes[child_index_in_nodes].vertices_.scanContext_);
//                 //first为余弦距离,second为偏移量
//                 std::cout<<"result "<<result.first<<" "<<result.second<<" shift "<<shift<<" id "<<child_id<<std::endl;
//                 if(result.first< scoreBetweenScanContext)
//                 {
//                     if(possibleChildIndex.count(child_id))
//                     {
//                         double last_score = possibleChildIndex[child_id];
//                         possibleChildIndex[child_id] = std::min(result.first, last_score);
//                     }else
//                         possibleChildIndex[child_id] = result.first;
//                 }
//             }
//         }
//         if(possibleChildIndex.empty())
//             return -1;
//         double min_score = INT_MAX;             //找最小的分数
//         int childLoopClouserIndex = -1;         //闭环点id
//         for(auto &data:possibleChildIndex)
//         {
//             if(data.second < min_score)
//             {
//                 min_score = data.second;
//                 childLoopClouserIndex = data.first;
//             }
//         }
//         if(min_score > scoreBetweenScanContext)
//             return -1;
//         else{
//             std::cout<<"child loop clouser index is "<<childLoopClouserIndex<<std::endl;
//             return childLoopClouserIndex;
//         }
//     }
//     int findIndexInNodes(std::vector<node>& nodes, int& node_id)
//     {
//         int father_id = -1;
//         for(auto nodeIndex:nodeKeyIndex)
//         {
//             if(nodeIndex.second == node_id)
//             {
//                 father_id = nodeIndex.first;
//                 break;
//             }
//         }
//         if(father_id == -1)
//         {
//             std::cout<<"father id == -1 "<<node_id<<std::endl;
//             return -1;
//         }
//         return father_id;
//     }
//     node create_new_node(int& id)
//     {
//         node topologicalNode;
//         topologicalNode.id_ = id;
//         topologicalNode.time_stamp_ = lidarTimeStampQue.at(lidarTimeStampQue.size()/2);
//         // int localMapNumber = 0;
//         topologicalNode.vertices_.point_num_                = localMap->points.size();
//         topologicalNode.vertices_.localTransformMatrix_     = relativeMeasurementsToLastNode;
//         topologicalNode.vertices_.globalTransformMatrix_    = currentNodePose;
//         topologicalNode.vertices_.Global_Pose_.x            = currentNodePose.translation().x();
//         topologicalNode.vertices_.Global_Pose_.y            = currentNodePose.translation().y();
//         topologicalNode.vertices_.Global_Pose_.z            = currentNodePose.translation().z();
//         topologicalNode.vertices_.scanContext_              = global_scanContext.back();
//         topologicalNode.vertices_.scanContextRingKey_       = globalScanContextRingKey.back();
//         *(topologicalNode.vertices_.laserCloud_)               += *localMap;
//         latestNodePose = currentNodePose;
//         sc_last = global_scanContext.back();
//         cout<<"create node id ===> "<<id<<std::endl;
//         return topologicalNode;
//     }
//     void constructNode(Eigen::MatrixXd& sc_current)
//     {
//         int keyFrameIndex = bufferQueLength/2;
//         Eigen::Isometry3d keyPose = odometryQue.at(keyFrameIndex);
//         currentNodePose = keyPose;
//         relativeMeasurementsToLastNode = latestNodePose.inverse()*keyPose;

//         Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc_current);
//         std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
//         global_scanContext.push_back(sc_current);
//         globalScanContextRingKey.push_back(polarcontext_vkeys_vec);
//     }
//     //构造节点
//     void constructNode()
//     {
//         int keyFrameIndex = bufferQueLength/2;
//         Eigen::Isometry3d keyPose = odometryQue.at(keyFrameIndex);
//         currentNodePose = keyPose;                                          //global pose
//         relativeMeasurementsToLastNode = latestNodePose.inverse()*keyPose;  //relative pose

//         Eigen::MatrixXd sc_node = makeScancontext(localMap);
//         Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc_node);
//         std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);
//         global_scanContext.push_back(sc_node);
//         globalScanContextRingKey.push_back(polarcontext_vkeys_vec);
//         // PointType currentPose;
//         // currentPose.x = keyPose.translation().x();
//         // currentPose.y = keyPose.translation().y();
//         // currentPose.z = keyPose.translation().z();
//         // globalKeyPose3d->push_back(currentPose);
//     }
//     double getScoreFromLastNode(Eigen::MatrixXd& sc_current, Eigen::MatrixXd& sc_last)
//     {
//         // std::pair<double, int>sc_result = distanceBtnScanContext(sc_current, sc_last);  //返回值first为余弦距离，second为最小偏移量
//         // return sc_result.first;
//         double score = distDirectSC(sc_current, sc_last);
//         return score;
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
//         std::cout << "数量" << laserCloudQue.size() << std::endl;//qh add for debug
//         return false;
//     }
// public:
//     topomap(std::string node_save_path_, double scoreThreshold_ = 0.2, double distanceThreshold_ = 20.0 )
//     {
//         ////////////////////
//         kdtreeHistroyKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
//         globalKeyPose3d.reset(new pcl::PointCloud<PointType>());
//         localMap.reset(new pcl::PointCloud<PointType>());
//         arouncMapShow.reset(new pcl::PointCloud<PointType>());
//         pubGlobalNodePosition.reset(new pcl::PointCloud<PointType>());
//         node_save_path = node_save_path_;//"/home/qh/robot_ws/map/2021-08-30-18-06-30L/node/0.70-15/";//路径
//         bufferQueLength = 13;
//         weight_distance = 0.25;
//         weight_intersection = 0.4;
//         weight_similarity = 0.5;
//         scoreBetweenScanContext = 0.30;//描述子余弦距离
//         save_data_to_files = true;//是否保存文件
//         scoreThreshold = scoreThreshold_;//todo
//         scanContextThreshold = 0.4;
//         distanceThreshold = distanceThreshold_;
//         distanceInterThreshold = 4.0;
//         create_history_nodes = false;//是否从已有文件创建节点
//         removeId = 0;
//         successFreqTh = 0;
//         /////////////////////
//         if(create_history_nodes)
//         {
//             string file_path = node_save_path;
//             read_nodes_from_files(nodes, file_path);
//         }
//         /////////////////////

//     }
//     void run(sensor_msgs::PointCloud2 remove_obstacle_cloud,
//              nav_msgs::Odometry remove_obstacle_odom,
//              sensor_msgs::PointCloud2 intersection,
//              //输出
//              sensor_msgs::PointCloud2& node_position_,
//              sensor_msgs::PointCloud2& cloud_local_map,
//              bool isSim,
//              sensor_msgs::PointCloud2& newNopeCloudAround,
//              bool& genNewNodeFlag)
//     {
//         /////////////
//         if(remove_obstacle_cloud.data.empty())//初始的时候可能为空，因此判断
//             return;
//         /////////////
//         static int nodeCountLast = current_node_id;
//         if(current_node_id != nodeCountLast){//generate new node !
//             nodeCountLast = current_node_id;
//             pcl::toROSMsg(*arouncMapShow, newNopeCloudAround);
//             newNopeCloudAround.header.frame_id = "camera_init";
//             genNewNodeFlag = true;
//         }else{
//             genNewNodeFlag = false;
//         }
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
//         if(intersection.data.empty())//地面分析未到位，初始的时候会经常不到位
//             return;
//         pcl::PointCloud<PointType>::Ptr pose(new pcl::PointCloud<PointType>());
//         pcl::fromROSMsg(intersection, *pose);
//         if(pose->points.size()<1)
//             return;
//         currentIntersection(0) = pose->points[0].x;
//         currentIntersection(1) = pose->points[0].y;
//         currentIntersection(2) = pose->points[0].z;
//         newIntersection = true;
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

//                 //////////////////////
//                 //switch signal area to no signal area and reverse
//                 static bool isSimLast = isSim;
//                 static bool waitBuffer = false;
//                 static double waitTime = 5.0;
//                 static double lastTime = 0;
//                 static bool mode = true;


//                 if(isSim != isSimLast && !waitBuffer){
//                     isSimLast = isSim;
//                     waitBuffer = true;
//                     lastTime = remove_obstacle_cloud.header.stamp.toSec();
//                 }
//                 if(waitBuffer){//waitbuff mode no change
//                     if(remove_obstacle_cloud.header.stamp.toSec() - lastTime > waitTime){
//                         waitBuffer = false;
//                     }
//                 }

//                 if(!waitBuffer && isSim){//no sig mode is 1
//                     mode = true;
//                 }else if(!waitBuffer && !isSim){// sig mode is 2
//                     mode = false;
//                 }
//                 cout << "sig state:" << mode << endl;

//                 if(relocation)
//                 {
//                     node_number++;
//                     relocation = false;
//                     sc_current = makeScancontext(localMap);
//                     int loopClosureIndex = loopClosureFromKDTree(sc_current);
//                     if(loopClosureIndex != -1)//重定位成功
//                     {
//                         if(!nodeKeyIndex.count(loopClosureIndex))
//                         {
//                             cout << "error! please stop the process!" << endl;
//                             return;
//                         }
//                         int loopNodeId = nodeKeyIndex[loopClosureIndex];
//                         std::cout<<"*******************************************"<<std::endl;
//                         std::cout<<"relocation success the id is "<<loopNodeId<<" index "<<loopClosureIndex<<std::endl;
//                         std::cout<<"*******************************************"<<std::endl;
//                         latestNodePose = nodes[loopClosureIndex].vertices_.globalTransformMatrix_;
//                         nodes[loopClosureIndex].success_call_freq_++;
//                         if(save_data_to_files)
//                         {
//                             string file_path = node_save_path;
//                             nodes[loopClosureIndex].nodes_save(file_path);
//                         }
//                         last_node_id = loopNodeId;
//                         sc_last = sc_current;
//                         localMap->clear();
//                         node_success++;
//                         return;
//                     }
//                     else//重定位失败
//                     {
//                         constructNode();
//                         node tmp_node = create_new_node(current_node_id);
//                         tmp_node.vertices_.Global_Pose_.intensity = mode;
//                         nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
//                         nodes.push_back(tmp_node);
//                         sc_last = tmp_node.vertices_.scanContext_;
//                         PointType thisPose;
//                         thisPose.x = tmp_node.vertices_.Global_Pose_.x;
//                         thisPose.y = tmp_node.vertices_.Global_Pose_.y;
//                         thisPose.z = tmp_node.vertices_.Global_Pose_.z;
//                         thisPose.intensity = current_node_id;
//                         globalKeyPose3d->push_back(thisPose);
//                         thisPose.intensity = mode;// qh add for debug
//                         pubGlobalNodePosition->push_back(thisPose);
//                         PreviousNodePosition(0) = thisPose.x;
//                         PreviousNodePosition(1) = thisPose.y;
//                         PreviousNodePosition(2) = thisPose.z;
//                         last_node_id = current_node_id;
//                         current_node_id++;
//                         localMap->clear();
//                         return;
//                     }
//                 }

//                 if(nodes.empty())
//                 {
//                     constructNode();
//                     //创建新节点并给latestNodePose赋值
//                     node tmp_node = create_new_node(current_node_id);
//                     tmp_node.vertices_.Global_Pose_.intensity = mode;
//                     nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
//                     nodes.push_back(tmp_node);
//                     sc_last = tmp_node.vertices_.scanContext_;
//                     PointType thisPose;
//                     thisPose.x = tmp_node.vertices_.Global_Pose_.x;
//                     thisPose.y = tmp_node.vertices_.Global_Pose_.y;
//                     thisPose.z = tmp_node.vertices_.Global_Pose_.z;
//                     thisPose.intensity = current_node_id;
//                     globalKeyPose3d->push_back(thisPose);
//                     thisPose.intensity = mode;// qh add for debug
//                     pubGlobalNodePosition->push_back(thisPose);

//                     PreviousNodePosition(0) = thisPose.x;
//                     PreviousNodePosition(1) = thisPose.y;
//                     PreviousNodePosition(2) = thisPose.z;

//                     last_node_id = current_node_id;
//                     current_node_id++;
//                     localMap->clear();
//                     return;
//                 }
//                 else
//                 {
//                     Eigen::Vector3d currentNodePosition = odometryQue.at(odometryQue.size()/2).matrix().block<3,1>(0,3);
//                     distanceFromPreNode = std::sqrt((currentNodePosition(0) - PreviousNodePosition(0)) * (currentNodePosition(0) - PreviousNodePosition(0))
//                                                     + (currentNodePosition(1) - PreviousNodePosition(1)) * (currentNodePosition(1) - PreviousNodePosition(1))
//                                                     + (currentNodePosition(2) - PreviousNodePosition(2)) * (currentNodePosition(2) - PreviousNodePosition(2)));

//                     distanceFromPreInter = std::sqrt((currentIntersection(0) - lastIntersection(0)) * (currentIntersection(0) - lastIntersection(0))
//                                                      + (currentIntersection(1) - lastIntersection(1)) * (currentIntersection(1) - lastIntersection(1))
//                                                      + (currentIntersection(2) - lastIntersection(2)) * (currentIntersection(2) - lastIntersection(2)));

//                     sc_current = makeScancontext(localMap);
//                     double similarityScore = getScoreFromLastNode(sc_current, sc_last);
//                     cout << setprecision(6) << "distanceFromPreNode" << distanceFromPreNode << endl;
//                     cout << setprecision(6) << "similarityScore" << similarityScore << endl;
//                     double score = 0;
//                     if(!mode) {//GNSS good
//                         if(distanceFromPreNode>=distanceThreshold)
//                         {
//                             std::cout<<"\ndistance "<<distanceFromPreNode <<" similarityScore "<<similarityScore<<" distanceFromPreInter "<<distanceFromPreInter<<" score "<<score<<std::endl;
//                             lastIntersection = currentIntersection;
//                             node_number++;
//                             constructNode(sc_current);
//                             //需要构建节点
//                             //在孩子节点上重定位
//                             int child_id =  loopClouserFromChild(sc_current);       //返回的是孩子节点的id
// //                            if(child_id != -1)
//                             if(false)
//                             {
//                                 int child_id_index = findIndexInNodes(nodes, child_id);
//                                 if(child_id_index!= -1 && nodeKeyIndex.count(child_id_index))
//                                 {
//                                     if(child_id != last_node_id)
//                                     {
//                                         std::cout<<"loopClosureFromChild find success the father id is [ "<<
//                                                  last_node_id<<" ] the child id is [ "<<child_id<<" ]"<<std::endl;
//                                         node_success++;
//                                         std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                                         //child_id 检测到的闭环节点id
//                                         latestNodePose = nodes[child_id_index].vertices_.globalTransformMatrix_;
//                                         last_node_id = child_id;

//                                         sc_last = sc_current;
//                                         localMap->clear();
//                                         Eigen::Vector3d currentNodePosition = odometryQue.at(odometryQue.size()/2).matrix().block<3,1>(0,3);
//                                         PreviousNodePosition(0) = currentNodePosition(0);
//                                         PreviousNodePosition(1) = currentNodePosition(1);
//                                         PreviousNodePosition(2) = currentNodePosition(2);

//                                         nodes[child_id_index].success_call_freq_++;
//                                         if(save_data_to_files)
//                                         {
//                                             string file_path = node_save_path;
//                                             nodes[child_id_index].nodes_save(file_path);
//                                         }
//                                         double average_time = cost_time/(node_number*1.0);
//                                         std::cout<<"cost time "<<cost_time<<" average_time "<<average_time<<std::endl;
//                                         return;
//                                     }
//                                 }
//                             }
//                             //全局拓扑节点重定位
//                             int loopClosureIndex = loopClosureFromKDTree(sc_current);//返回值为在nodes中的索引
//                             double average_time = cost_time/(node_number*1.0);
//                             std::cout<<"cost time "<<cost_time<<" average_time "<<average_time<<std::endl;

//                             if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex))
//                             {

//                                 int loopNodeId = nodeKeyIndex[loopClosureIndex];
//                                 if(loopNodeId != last_node_id)
//                                 {
//                                     std::cout<<"loopClosureFromKDTree find success the father id is [ "
//                                              <<last_node_id<<" ] the child id is [ "<<loopNodeId<<" ]"<<std::endl;
//                                     node_success++;
//                                     std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                                     latestNodePose = nodes[loopClosureIndex].vertices_.globalTransformMatrix_;

//                                     sc_last = sc_current;

//                                     int father_id = findIndexInNodes(nodes, last_node_id);
//                                     if(father_id == -1)
//                                     {
//                                         std::cout<<"loopClosureFromKD father id == -1"<<std::endl;
//                                         return;
//                                     }
//                                     int child_number = nodes[father_id].as_father_edges_.size();

//                                     Eigen::Vector3d currentNodePosition = odometryQue.at(odometryQue.size()/2).matrix().block<3,1>(0,3);
//                                     PreviousNodePosition(0) = currentNodePosition(0);
//                                     PreviousNodePosition(1) = currentNodePosition(1);
//                                     PreviousNodePosition(2) = currentNodePosition(2);

//                                     nodes[loopClosureIndex].success_call_freq_++;
//                                     if(save_data_to_files)
//                                     {
//                                         string file_path = node_save_path;
//                                         nodes[loopClosureIndex].nodes_save(file_path);
//                                     }
//                                     bool create_now_edge(true);     //判断两个节点之间是否已经有边
//                                     if(child_number == 0)
//                                     {
//                                         create_now_edge = true;
//                                     }else{
//                                         for(int i=0; i<child_number; ++i)
//                                         {
//                                             if(nodes[father_id].as_father_edges_[i].child_id_ == loopNodeId)
//                                             {
//                                                 create_now_edge = false;
//                                                 break;
//                                             }
//                                         }
//                                     }
//                                     if(!create_now_edge)
//                                     {
//                                         last_node_id = loopNodeId;
//                                         localMap->clear();
//                                         return;
//                                     }else{
//                                         //创建新的边，父节点为last_node_id，子节点为loopClosureIndex
//                                         creataEdge(nodes, father_id, loopClosureIndex);
//                                         if(save_data_to_files)
//                                         {
//                                             string file_path = node_save_path;
//                                             nodes[father_id].nodes_save(file_path);
//                                             nodes[loopClosureIndex].nodes_save(file_path);
//                                         }
//                                         cout<<"create new edge father => "<<last_node_id<<" child "<<loopNodeId<<endl;
//                                         last_node_id = loopNodeId;
//                                         return;
//                                     }
//                                 }
//                             }

//                             std::cout<<"relocation error, create new node !!"<<std::endl;
//                             //重定位失败，构建新的节点
//                             node tmp_node = create_new_node(current_node_id);
//                             tmp_node.vertices_.Global_Pose_.intensity = mode;
//                             nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
//                             nodes.push_back(tmp_node);
//                             PointType thisPose;
//                             thisPose.x = tmp_node.vertices_.Global_Pose_.x;
//                             thisPose.y = tmp_node.vertices_.Global_Pose_.y;
//                             thisPose.z = tmp_node.vertices_.Global_Pose_.z;
//                             thisPose.intensity = current_node_id;
//                             globalKeyPose3d->push_back(thisPose);
//                             thisPose.intensity = mode;// qh add for debug
//                             pubGlobalNodePosition->push_back(thisPose);
//                             sc_last = tmp_node.vertices_.scanContext_;

//                             PreviousNodePosition(0) = thisPose.x;
//                             PreviousNodePosition(1) = thisPose.y;
//                             PreviousNodePosition(2) = thisPose.z;
//                             //创建节点边
//                             int father_id = findIndexInNodes(nodes, last_node_id);
//                             if(father_id == -1)
//                             {
//                                 std::cout<<"new node father id == -1 "<<last_node_id<<" nodes size "<<nodes.size()<<std::endl;
//                                 return ;
//                             }
//                             if(last_node_id!=-1 && last_node_id != tmp_node.id_)
//                             {
//                                 creataEdge(nodes, father_id, (int)(nodes.size()-1));
//                                 if(save_data_to_files)
//                                 {
//                                     string file_path = node_save_path;
//                                     nodes[father_id].nodes_save(file_path);
//                                     nodes[(int)(nodes.size()-1)].nodes_save(file_path);
//                                 }
//                                 std::cout<<"create edge and the father is "<<last_node_id<<" child id is "<<tmp_node.id_<<std::endl;
//                             }
//                             // std::cout<<"relative:\n"<<relativeMeasurementsToLastNode.matrix()<<std::endl;
//                             // std::cout<<"edge: \n"<<nodes[tmp_node.id_].as_child_edges_[0].keyTransformMatrix_.matrix()<<std::endl;
//                             last_node_id = current_node_id;
//                             current_node_id++;
//                             localMap->clear();
//                             std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                         }
//                     }else{//no GNSS signal
//                         //score = std::exp((similarityScore - scanContextThreshold)/(scanContextThreshold*1.0));
//                         score = similarityScore;
//                         if(score>=scoreThreshold)
//                         {
//                             std::cout<<"\ndistance "<<distanceFromPreNode <<" similarityScore "<<similarityScore<<" distanceFromPreInter "<<distanceFromPreInter<<" score "<<score<<std::endl;
//                             lastIntersection = currentIntersection;
//                             node_number++;
//                             constructNode(sc_current);
//                             //需要构建节点
//                             //在孩子节点上重定位
//                             int child_id =  loopClouserFromChild(sc_current);       //返回的是孩子节点的id
//                             if(child_id != -1)
//                             {
//                                 int child_id_index = findIndexInNodes(nodes, child_id);
//                                 if(child_id_index!= -1 && nodeKeyIndex.count(child_id_index))
//                                 {
//                                     if(child_id != last_node_id)
//                                     {
//                                         std::cout<<"loopClosureFromChild find success the father id is [ "<<
//                                                  last_node_id<<" ] the child id is [ "<<child_id<<" ]"<<std::endl;
//                                         node_success++;
//                                         std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                                         //child_id 检测到的闭环节点id
//                                         latestNodePose = nodes[child_id_index].vertices_.globalTransformMatrix_;
//                                         last_node_id = child_id;

//                                         sc_last = sc_current;
//                                         localMap->clear();
//                                         Eigen::Vector3d currentNodePosition = odometryQue.at(odometryQue.size()/2).matrix().block<3,1>(0,3);
//                                         PreviousNodePosition(0) = currentNodePosition(0);
//                                         PreviousNodePosition(1) = currentNodePosition(1);
//                                         PreviousNodePosition(2) = currentNodePosition(2);

//                                         nodes[child_id_index].success_call_freq_++;
//                                         if(save_data_to_files)
//                                         {
//                                             string file_path = node_save_path;
//                                             nodes[child_id_index].nodes_save(file_path);
//                                         }
//                                         double average_time = cost_time/(node_number*1.0);
//                                         std::cout<<"cost time "<<cost_time<<" average_time "<<average_time<<std::endl;
//                                         return;
//                                     }
//                                 }
//                             }
//                             //全局拓扑节点重定位
//                             int loopClosureIndex = loopClosureFromKDTree(sc_current);//返回值为在nodes中的索引
//                             double average_time = cost_time/(node_number*1.0);
//                             std::cout<<"cost time "<<cost_time<<" average_time "<<average_time<<std::endl;

//                             if(loopClosureIndex != -1 && nodeKeyIndex.count(loopClosureIndex))
//                             {

//                                 int loopNodeId = nodeKeyIndex[loopClosureIndex];
//                                 if(loopNodeId != last_node_id)
//                                 {
//                                     std::cout<<"loopClosureFromKDTree find success the father id is [ "
//                                              <<last_node_id<<" ] the child id is [ "<<loopNodeId<<" ]"<<std::endl;
//                                     node_success++;
//                                     std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                                     latestNodePose = nodes[loopClosureIndex].vertices_.globalTransformMatrix_;

//                                     sc_last = sc_current;

//                                     int father_id = findIndexInNodes(nodes, last_node_id);
//                                     if(father_id == -1)
//                                     {
//                                         std::cout<<"loopClosureFromKD father id == -1"<<std::endl;
//                                         return;
//                                     }
//                                     int child_number = nodes[father_id].as_father_edges_.size();

//                                     Eigen::Vector3d currentNodePosition = odometryQue.at(odometryQue.size()/2).matrix().block<3,1>(0,3);
//                                     PreviousNodePosition(0) = currentNodePosition(0);
//                                     PreviousNodePosition(1) = currentNodePosition(1);
//                                     PreviousNodePosition(2) = currentNodePosition(2);

//                                     nodes[loopClosureIndex].success_call_freq_++;
//                                     if(save_data_to_files)
//                                     {
//                                         string file_path = node_save_path;
//                                         nodes[loopClosureIndex].nodes_save(file_path);
//                                     }
//                                     bool create_now_edge(true);     //判断两个节点之间是否已经有边
//                                     if(child_number == 0)
//                                     {
//                                         create_now_edge = true;
//                                     }else{
//                                         for(int i=0; i<child_number; ++i)
//                                         {
//                                             if(nodes[father_id].as_father_edges_[i].child_id_ == loopNodeId)
//                                             {
//                                                 create_now_edge = false;
//                                                 break;
//                                             }
//                                         }
//                                     }
//                                     if(!create_now_edge)
//                                     {
//                                         last_node_id = loopNodeId;
//                                         localMap->clear();
//                                         return;
//                                     }else{
//                                         //创建新的边，父节点为last_node_id，子节点为loopClosureIndex
//                                         creataEdge(nodes, father_id, loopClosureIndex);
//                                         if(save_data_to_files)
//                                         {
//                                             string file_path = node_save_path;
//                                             nodes[father_id].nodes_save(file_path);
//                                             nodes[loopClosureIndex].nodes_save(file_path);
//                                         }
//                                         cout<<"create new edge father => "<<last_node_id<<" child "<<loopNodeId<<endl;
//                                         last_node_id = loopNodeId;
//                                         return;
//                                     }
//                                 }
//                             }

//                             std::cout<<"relocation error, create new node !!"<<std::endl;
//                             //重定位失败，构建新的节点
//                             node tmp_node = create_new_node(current_node_id);
//                             tmp_node.vertices_.Global_Pose_.intensity = mode;
//                             nodeKeyIndex.insert({(int)nodes.size(), current_node_id});
//                             nodes.push_back(tmp_node);
//                             PointType thisPose;
//                             thisPose.x = tmp_node.vertices_.Global_Pose_.x;
//                             thisPose.y = tmp_node.vertices_.Global_Pose_.y;
//                             thisPose.z = tmp_node.vertices_.Global_Pose_.z;
//                             thisPose.intensity = current_node_id;
//                             globalKeyPose3d->push_back(thisPose);
//                             //qh add for debug
//                             thisPose.intensity = mode;//qh add for debug
//                             pubGlobalNodePosition->push_back(thisPose);
//                             sc_last = tmp_node.vertices_.scanContext_;

//                             PreviousNodePosition(0) = thisPose.x;
//                             PreviousNodePosition(1) = thisPose.y;
//                             PreviousNodePosition(2) = thisPose.z;
//                             //创建节点边
//                             int father_id = findIndexInNodes(nodes, last_node_id);
//                             if(father_id == -1)
//                             {
//                                 std::cout<<"new node father id == -1 "<<last_node_id<<" nodes size "<<nodes.size()<<std::endl;
//                                 return ;
//                             }
//                             if(last_node_id!=-1 && last_node_id != tmp_node.id_)
//                             {
//                                 creataEdge(nodes, father_id, (int)(nodes.size()-1));
//                                 if(save_data_to_files)
//                                 {
//                                     string file_path = node_save_path;                                    nodes[father_id].nodes_save(file_path);
//                                     nodes[(int)(nodes.size()-1)].nodes_save(file_path);
//                                 }
//                                 std::cout<<"create edge and the father is "<<last_node_id<<" child id is "<<tmp_node.id_<<std::endl;
//                             }
//                             // std::cout<<"relative:\n"<<relativeMeasurementsToLastNode.matrix()<<std::endl;
//                             // std::cout<<"edge: \n"<<nodes[tmp_node.id_].as_child_edges_[0].keyTransformMatrix_.matrix()<<std::endl;
//                             last_node_id = current_node_id;
//                             current_node_id++;
//                             localMap->clear();
//                             std::cout<<"node number "<<node_number<<" success "<<node_success<<std::endl;
//                         }
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
//         node_position_ = pubNodePositionTmp_;
//     }
// };
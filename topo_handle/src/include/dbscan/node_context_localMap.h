#ifndef _NCLT_NODE_CONTEXT_LOCALMAP_H_
#define _NCLT_NODE_CONTEXT_LOCALMAP_H_
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <utility>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <omp.h>
using namespace Eigen;
using namespace nanoflann;
using std::atan2;
using std::cos;
using std::sin;
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;;

const float  LIDAR_HEIGHT = 20.0;             //雷达安装高度保证点云z轴数据大于0
const int    PC_NUM_RING = 40;              // 20 in the original paper (IROS 18) 40， 80
const int    PC_NUM_SECTOR = 180;            // 60 in the original paper (IROS 18)
const double PC_MAX_RADIUS = 80.0;          // 80 meter 激光点最远距离
const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

 // tree
const int    NUM_EXCLUDE_RECENT = 50; // simply just keyframe gap, but node position distance-based exclusion is ok. 
const int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)

// loop thres
const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
// const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

// config 
// 重新构建kd树的频率
const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
int          tree_making_period_conter = 0;
const int row = 3;
const int column = 3;
class Vertice{
public:
    Vertice(){
        localTransformMatrix_  = Eigen::Isometry3d::Identity();
        globalTransformMatrix_ = Eigen::Isometry3d::Identity();
        laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    };
    size_t point_num_;       //该帧点云中点云的数目
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;//该帧点云
    pcl::PointXYZI Global_Pose_;     //每个定点的全局位姿
    Eigen::MatrixXd scanContext_;
    std::vector<float> scanContextRingKey_;
    Eigen::Isometry3d localTransformMatrix_;     //上对于上一帧的旋转平移矩阵声明为3d，其实为4*4矩阵
    Eigen::Isometry3d globalTransformMatrix_;    //全局坐标系下的RT
};
//边 类
class Edge{
public:
    Edge(){
        keyTransformMatrix_ = Eigen::Isometry3d::Identity();
    }
    double father_id_;  //边父节点
    double child_id_;   //子节点
    // Eigen::Matrix4d keyHomogeneousMatrix;   //基准点相对于上一阵的齐次矩阵
    Eigen::Isometry3d keyTransformMatrix_;
};
//创建节点
class node{
public:
    node(){
        as_father_edges_.clear();
        as_child_edges_.clear();
        father_intersection_edges_.clear();
        child_intersection_edges_.clear();
    };
    ~node(){
        /*
        for(auto c:key_node_)
            c.reset();
        key_node_.clear();*/
    };
    bool create_node(int newId){
        std::cout<<"create_node id "<<newId<<std::endl;
    }
    bool nodes_save(std::string& path);
    bool create_node_from_file(std::string path, int index);

    double id_ = 0;                          //节点id，即节点在整体拓扑地图中的id

    double time_stamp_;                     //记录当前节点创建时的时间
    double call_freq_ = 0;                  //被call的次数
    double success_call_freq_ = 0;          //成功匹配的次数

    Vertice vertices_;                      //该节点中存储的点云及相对于全局的坐标
    std::vector<Edge>as_father_edges_;      //作为父节点的边
    std::vector<Edge>as_child_edges_;       //作为子节点的边
    
    bool Intersection_=false;
    double intersection_id_ = 0;
    std::vector<Edge>father_intersection_edges_;        //节点如果是路口的话，创建的边
    std::vector<Edge>child_intersection_edges_;         //节点如果是路口的话，创建的边
};
bool node::nodes_save(std::string &path){
    std::string fileName;
    fileName = path + std::to_string((int)id_) + ".txt"; 
    std::ofstream of(fileName);
    if(!of.is_open()){
        std::cout<<"open file "<<fileName<<" error!"<<std::endl;
        return false;
    }
    //节点编号
    of<<id_<<std::endl;
    //
    of<<success_call_freq_<<std::endl;
    //存储以该点为父节点的边
    of<<as_father_edges_.size()<<std::endl;//数量
    for(auto as_father_edge:as_father_edges_){
        of<<as_father_edge.father_id_<<" "<<as_father_edge.child_id_<<std::endl;//编号
        of<<as_father_edge.keyTransformMatrix_.matrix()<<std::endl;//两个边之间的变换阵
    }
    //存储以该点为子节点的边
    of<<as_child_edges_.size()<<std::endl;//数量
    for(auto as_child_edge:as_child_edges_){
        of<<as_child_edge.father_id_<<" "<<as_child_edge.child_id_<<std::endl;//编号
        of<<as_child_edge.keyTransformMatrix_.matrix()<<std::endl;//两个边之间的变换阵
    }
    //位置
    of<<std::fixed<<std::setprecision(9)<<vertices_.Global_Pose_.x<<" "<<vertices_.Global_Pose_.y<<" "<<vertices_.Global_Pose_.z<<std::endl;
    //点云数量
    of<<vertices_.point_num_<<std::endl;
    
    //变换阵
    of << vertices_.localTransformMatrix_.matrix()<< std::endl << "龙逆1" << std::endl
    << vertices_.scanContext_<< std::endl << "龙逆2" << std::endl
    << vertices_.globalTransformMatrix_.matrix()<<std::endl;
    //
    of<<vertices_.scanContextRingKey_.size()<<std::endl;
    
    for(auto c:vertices_.scanContextRingKey_){
        of<<c<<" ";
    }
    of<<std::endl;
    // for(auto c:vertices_.laserCloud_->points){
    //     of<<c.x<<" "<<c.y<<" "<<c.z<<" "<<c.intensity<<std::endl;
    // }
    
    of.close();
    return true;
}
bool node::create_node_from_file(std::string path, int index){
    std::string fileName;
    fileName = path + std::to_string((int)index) + ".txt"; 
    std::ifstream read_file(fileName);
    if(!read_file.is_open()){
        std::cout<<"open file "<<fileName<<" error!"<<std::endl;
        return false;
    }
    read_file>>id_;
    if(read_file.eof()){
        std::cout<<"The "<<fileName<<" is empty, read file error!"<<std::endl;
        return false;
    }
    read_file>>success_call_freq_;
    int father_edge_number;
    read_file>>father_edge_number;
    as_father_edges_.resize(father_edge_number);
    if(father_edge_number > 0){
        for(int i=0;i<father_edge_number;++i){
            read_file>>as_father_edges_[i].father_id_>>as_father_edges_[i].child_id_;
            for(int row = 0;row<4;++row){
                for(int col=0; col<4;++col){
                    double data;
                    read_file>>data;
                    as_father_edges_[i].keyTransformMatrix_(row,col) = data;
                }
            }
        }
    }
    int child_edge_number;
    read_file>>child_edge_number;
    as_child_edges_.resize(child_edge_number);
    if(child_edge_number > 0){
        for(int i=0;i<child_edge_number;++i){
            read_file>>as_child_edges_[i].father_id_>>as_child_edges_[i].child_id_;
            for(int row = 0;row<4;++row){
                for(int col = 0;col<4;++col){
                    double data;
                    read_file>>data;
                    as_child_edges_[i].keyTransformMatrix_(row,col) = data;
                }
            }
        }
    }
    
    read_file>>vertices_.Global_Pose_.x>>vertices_.Global_Pose_.y>>vertices_.Global_Pose_.z;
    read_file>>vertices_.point_num_;
    for(int row = 0;row<4;++row){
        for(int col = 0;col<4;++col){
            double data;
            read_file>>data;
            vertices_.localTransformMatrix_(row,col) = data;
            // read_file>>vertices_[i].homogenousMatrix(row,col);
        }
    }
    vertices_.scanContext_ = Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    for(int row = 0; row<PC_NUM_RING; ++row){
        for(int col = 0; col<PC_NUM_SECTOR; ++col){
            read_file>>vertices_.scanContext_(row,col);
        }
    }
    for(int row = 0;row<4;++row){
        for(int col = 0;col<4;++col){
            double data;
            read_file>>data;
            vertices_.globalTransformMatrix_(row,col) = data;
            // read_file>>vertices_[i].GlobalHomogenousMatrix(row,col);
        }
    }
    int ringKeyNumber;
    read_file>>ringKeyNumber;
    vertices_.scanContextRingKey_.resize(ringKeyNumber);
    for(int index = 0;index<ringKeyNumber; ++index){
        read_file>>vertices_.scanContextRingKey_[index];
    }
    vertices_.laserCloud_->clear();
    // for(int i = 0; i<vertices_.point_num_; ++i){
    //     pcl::PointXYZI thisPoint;
    //     read_file>>thisPoint.x >>  thisPoint.y >> thisPoint.z >> thisPoint.intensity;
    //     vertices_.laserCloud_->push_back(thisPoint);
    // }
    
    read_file.close();
    return true;
}
/////////////////////////////////////////////
//创建关键节点
float rad2deg(float radians){
    return radians * 180.0 / M_PI;
}

float deg2rad(float degrees){
    return degrees * M_PI / 180.0;
}
/*输入坐标x,y输出与x轴的夹角
*/
float xy2theta(const float&_x, const float& _y){
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
}
/*构建描述子
* input:  一帧激光扫描数据
* output: 该帧激光对应的描述子
*/
template<typename T>
MatrixXd makeScancontext(T &_scan){
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
        point.y += LIDAR_HEIGHT;   //保证z值大于0
        // point.z += 0.4;             //让车在路中央？？？
        //正常坐标系下
        // value_range = std::sqrt(point.x*point.x + point.y*point.y);
        // value_angle = xy2theta(point.x,point.y);
        //loam坐标系下，z轴为正常坐标系下的x轴，x轴为正常坐标系下的y轴
        value_range = std::sqrt(point.x*point.x + point.z*point.z);
        value_angle = xy2theta(point.x,point.z);
        if(value_range > PC_MAX_RADIUS){//测量距离不能大于设定范围
            continue;
        }
        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((value_range/PC_MAX_RADIUS)*PC_NUM_RING))),1);//0还是1
        sctor_idx = std::max( std::min(PC_NUM_SECTOR, int(ceil((value_angle / 360.0) * PC_NUM_SECTOR ))), 1);
        // std::cout<<"ring_idx "<<ring_idx<<" sctor_idx "<<sctor_idx<<std::endl;
        // if(desc(ring_idx,sctor_idx)<point.z)
        //     desc(ring_idx,sctor_idx) = point.z;
        //loam坐标系下y轴为正常坐标系下的z轴
        if(desc(ring_idx-1,sctor_idx-1)<point.y)
            desc(ring_idx-1,sctor_idx-1) = point.y;
    }
    for(int row_idx = 0; row_idx < desc.rows(); ++row_idx)
        for(int col_idx = 0; col_idx < desc.cols(); ++col_idx)
            if(desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;//空白区域点值为0

    return desc;
}
//计算scan context中每一行得到RingKey RingKey为该行中所有z的平均值
// RingKey为（row,1）的向量，列向量
MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd &_desc ){
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for(int row_idx = 0; row_idx < _desc.rows(); row_idx++){
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        int all_occupancied = 0;
        for(int col_idx = 0;col_idx<curr_row.cols();++col_idx){
            if(curr_row(0, col_idx)!=0){
                all_occupancied++;
            }
        }
        //invariant_key(row_idx, 0) = curr_row.mean();//求取平均值
        invariant_key(row_idx, 0) = all_occupancied/(curr_row.cols()*1.0);//求取平均值
    }

    return invariant_key;
} // makeRingkeyFromScancontext
//将描述子计算成一个行向量
// Sector key为扇形区域  描述子组成一个1*n的向量
MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc){
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for(int col_idx = 0; col_idx < _desc.cols(); col_idx++){
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
}
//将矩阵进行旋转
MatrixXd circshift(MatrixXd &mat, int _shift){
    if(_shift<0){
        return mat;
    }
    if(_shift == 0){
        MatrixXd afterCircShift(mat);
        return afterCircShift;
    }
    MatrixXd afterCircShift = MatrixXd::Zero(mat.rows(),mat.cols());
    for(int col_idx = 0; col_idx<mat.cols(); ++col_idx){
        int newLocation = (col_idx+_shift)%mat.cols();
        afterCircShift.col(newLocation) = mat.col(col_idx);
    }
    return afterCircShift;
}
//Matirx转换到vector
std::vector<float> eig2stdvec(MatrixXd _inputMat){
    std::vector<float>resultVec(_inputMat.data(), _inputMat.data() + _inputMat.size());
    return resultVec;
}
//计算两个描述子之间的距离  对应论文中公式（5）余弦相似度
//计算两个列向量上的余弦相似度
double distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2){
    int num_effective_cols = 0;
    double sum_similarity = 0;
    for(int col_idx = 0; col_idx<_sc1.cols(); ++col_idx){
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        //norm返回向量的二范数
        if(col_sc1.norm() == 0 || col_sc2.norm()==0){
            continue;
        }
        double sector_similarity = col_sc1.dot(col_sc2)/(col_sc1.norm()*col_sc2.norm()); //1-sector_similarity为余弦相似度
        sum_similarity = sum_similarity + sector_similarity;
        num_effective_cols++;  
    }
    double sc_sim = sum_similarity/(num_effective_cols*1.0);
    //double sc_sim = sum_similarity/num_effective_cols;
    return std::move(1.0-sc_sim);
}
//计算两个描述子之间的最小平移量
int fastAlignUsingVKey(MatrixXd& _vkey1, MatrixXd& _vkey2){
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    //shift_idx 为平移的量
    // int min_shift_idx = 0, max_shift_idx = 
    for(int shift_idx = 0; shift_idx<_vkey1.cols(); ++shift_idx){

        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);//将一位矩阵平移
        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;//平移后的向量，将两者相减
        double cur_diff_norm = vkey_diff.norm();//norm为计算二范数

        if(cur_diff_norm<min_veky_diff_norm){
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    return argmin_vkey_shift;
}
/*计算两个描述子之间的余弦距离
* 返回值first为余弦距离，second为最小偏移量
*/
std::pair<double,int>distanceBtnScanContext(MatrixXd &sc_1, MatrixXd &sc_2){
    // 1. fast align using variant key (not in original IROS18)
    //快速配准 判断平移多少，
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc_1); //输出1*n维矩阵
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc_2);
    int argmin_vkey_shift = fastAlignUsingVKey(vkey_sc1,vkey_sc2);      //查找两个描述子之间最小平移量
    const int SEARCH_RADIUS = round(0.5*SEARCH_RATIO*sc_1.cols());      //0.5*0.1*列数
    std::vector<int>shift_idx_search_space{argmin_vkey_shift};
    for(int i=0;i<SEARCH_RADIUS+1;++i){
        shift_idx_search_space.push_back((argmin_vkey_shift+i+sc_1.cols())%sc_1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift-i+sc_1.cols())%sc_1.cols());
    }
    std::sort(shift_idx_search_space.begin(),shift_idx_search_space.end());
    //2. 余弦距离匹配
    int argmin_shift = 0;
    double minScanContextDist = 10000000;
    for(int shift:shift_idx_search_space){
        MatrixXd sc2_shifted = circshift(sc_2, shift);
        double curScanContextDist = distDirectSC(sc_1,sc2_shifted);
        if(curScanContextDist<minScanContextDist){
            argmin_shift = shift;
            minScanContextDist = curScanContextDist;
        }
    }
    return std::make_pair(minScanContextDist,argmin_shift);
}
/*存储关键帧的描述子
 *_scan 为当前帧的激光数据
 *polarcontexts 为存储的所有的描述子
 *polarcontext_invkeys_mat 为描述子的ringkey(n*1)从矩阵转换到vector存储类型
*/
template <typename T>
void makeAndSaveScanContextAndKeys(T _scan, std::vector<Eigen::MatrixXd>& polarcontexts, KeyMat &polarcontext_invkeys_mat){
    
    Eigen::MatrixXd sc = makeScancontext(_scan);    //

    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    //Eigen::MatrixXd sectorKey = makeSectorkeyFromScancontext(sc);
    std::vector<float>polarcontext_vkeys_vec = eig2stdvec(ringkey);

    polarcontexts.push_back(sc);
    polarcontext_invkeys_mat.push_back(polarcontext_vkeys_vec);
}
/*
探索闭环节点，利用ringkey创建kd树搜索
利用列向量的余弦相似度的平均值判断差异，差异最小的两个点为闭环匹配对
polarcontexts 描述子集合   polarcontext_invkeys_mat，描述子对应的ringkey的集合
polarcontext_invkeys_to_search  矩阵转换为vector存储
polarcontext_tree  依据描述子创建的kd树的集合
*/
std::pair<int,float>detectLoopClosureID(std::vector<Eigen::MatrixXd>& polarcontexts, KeyMat &polarcontext_invkeys_mat, 
                                        KeyMat& polarcontext_invkeys_to_search,
                                        std::unique_ptr<InvKeyTree>& polarcontext_tree){
    int loop_id{-1};    //
    auto curr_key = polarcontext_invkeys_mat.back();    //key值
    auto curr_desc = polarcontexts.back();              //描述子
    if(polarcontext_invkeys_mat.size()<NUM_EXCLUDE_RECENT+1){
        std::pair<int,float>result{loop_id,0.0};
        return result;
    }
    if(tree_making_period_conter%TREE_MAKING_PERIOD_ == 0){
        polarcontext_invkeys_to_search.clear();
        polarcontext_invkeys_to_search.assign(polarcontext_invkeys_mat.begin(),polarcontext_invkeys_mat.end()-NUM_EXCLUDE_RECENT);
        polarcontext_tree.reset();
        polarcontext_tree = std::make_unique<InvKeyTree>(PC_NUM_RING, polarcontext_invkeys_to_search,10);
    }
    tree_making_period_conter++;
    double min_dist = 1000000;
    int nn_align = 0, nn_index = 0;//align为要移动的距离，index为搜索结果的索引值
    std::vector<size_t>candidata_indexes(NUM_CANDIDATES_FROM_TREE);
    std::vector<float>out_dists_sqrt(NUM_CANDIDATES_FROM_TREE);

    nanoflann::KNNResultSet<float>knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    knnsearch_result.init(&candidata_indexes[0],&out_dists_sqrt[0]);
    polarcontext_tree->index->findNeighbors(knnsearch_result, &curr_key[0],nanoflann::SearchParams(10));
    //依据余弦距离，查找最优匹配对，找余弦距离最小值
    for(int index = 0;index<NUM_CANDIDATES_FROM_TREE; ++index){
        MatrixXd polarcontext_candidata = polarcontexts[candidata_indexes[index]];  //搜索到的描述子
        std::pair<double, int>sc_dist_result = distanceBtnScanContext(curr_desc,polarcontext_candidata);
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;
        if(candidate_dist < min_dist){
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_index = candidata_indexes[index];
        }
    }
    /* 
     * 闭环检测阈值检查
     */
    //两个描述子之间的余弦距离要小于SC_DIST_THRES
    if(min_dist < SC_DIST_THRES){
        loop_id = nn_index; //闭环索引值
    
        // std::cout.precision(3); 
        std::cout<<"[Loop found] Nearest distance: "<<min_dist<<" btn "<< polarcontexts.size()-1<<" and "<<nn_index<<"."<<std::endl;
        std::cout<<"[Loop found] yaw diff: "<< nn_align * PC_UNIT_SECTORANGLE<<" deg."<<std::endl;
    }else{
        std::cout.precision(3); 
        std::cout<< "[Not loop] Nearest distance: "<< min_dist<< " btn "<< polarcontexts.size()-1<< " and "<< nn_index<< "."<<std::endl;
        std::cout<< "[Not loop] yaw diff: "<< nn_align * PC_UNIT_SECTORANGLE<< " deg."<<std::endl;
    }
    float yaw_diff_rad = deg2rad(nn_align*PC_UNIT_SECTORANGLE);
    std::pair<int,float>result{loop_id,yaw_diff_rad};
    return result;
}
#endif
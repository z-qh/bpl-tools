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
const int    PC_NUM_RING = 20;              // 20 in the original paper (IROS 18) 40， 80
const int    PC_NUM_SECTOR = 60;            // 60 in the original paper (IROS 18)
const double PC_MAX_RADIUS = 50.0;          // 80 meter 激光点最远距离
const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

// loop thres
const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
template<typename T>
MatrixXd makeScancontext(T &_scan);
std::pair<double,int>distanceBtnScanContext(MatrixXd &sc_1, MatrixXd &sc_2);

std::vector<float>dataNeibor{0,-0.3,0.3,-1,1,-4,4};
int SHIFTSIZE = dataNeibor.size();
// config
// 重新构建kd树的频率
const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
int          tree_making_period_conter = 0;

class Vertice{
public:
    Vertice(){
    };
    size_t point_num_;       //该帧点云中点云的数目
    pcl::PointXYZI Global_Pose_;     //每个定点的全局位姿
    std::vector<Eigen::MatrixXd> scanContext_8;
};

//创建节点
class node{
private:
    void createNode(int& id, double& Time, pcl::PointXYZI& Pose, pcl::PointCloud<pcl::PointXYZI>&Cloud){
        time_stamp_ = Time;
        id_ = id;
        vertices_.Global_Pose_ = Pose;
        vertices_.point_num_ = Cloud.points.size();
        for(int i = 0; i<dataNeibor.size(); ++i)
        {
            float shift = dataNeibor[i];
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZI>());
            tmpCloud->clear();
            for(auto point:Cloud.points)
            {
                point.z += shift;
                tmpCloud->push_back(point);
            }
            vertices_.scanContext_8.push_back(makeScancontext(tmpCloud));
        }
    }
public:
    double id_ = 0;                          //节点id，即节点在整体拓扑地图中的id
    double time_stamp_{};                     //记录当前节点创建时的时间
    Vertice vertices_;                      //该节点中存储的点云及相对于全局的坐标
public:
    static double getScore(node& nowNodeOld, node& nowNodeNew){
        double min_dist = 10000;
        for(int i = 0; i < SHIFTSIZE; i++){
            Eigen::MatrixXd& scanContextCandidate = nowNodeNew.vertices_.scanContext_8[i];
            Eigen::MatrixXd& currentContextShift = nowNodeOld.vertices_.scanContext_8[0];
            std::pair<double, int> sc_dist_result = distanceBtnScanContext(currentContextShift, scanContextCandidate);
            double candidate_dist = sc_dist_result.first;       //余弦距离
            if(candidate_dist < min_dist)
            {
                min_dist = candidate_dist;                      //两个描述子之间的余弦距离
            }
        }
        return min_dist;
    }
public:
    node(){
        id_ = -1;
        time_stamp_ = -1;
        vertices_.Global_Pose_.x = -1;
        vertices_.Global_Pose_.y = -1;
        vertices_.Global_Pose_.z = -1;
        vertices_.Global_Pose_.intensity = -1;
        vertices_.scanContext_8.clear();
        vertices_.point_num_ = -1;
    }
    node(int& id, double& Time, pcl::PointXYZI& Pose, pcl::PointCloud<pcl::PointXYZI>&Cloud_){
        createNode(id, Time, Pose, Cloud_);
    };

    bool nodes_save_B(std::string& path);
    bool create_node_from_file_B(std::string path, int index);
};
////////////////////////////////////////////////
bool node::nodes_save_B(std::string &path){
    std::string fileName;
    fileName = path + std::to_string((int)id_) + ".txt";
    std::ofstream of(fileName);
    if(!of.is_open()){
        std::cout<<"open file "<<fileName<<" error!"<<std::endl;
        return false;
    }
    //存储以该点为父节点的边
    of<<id_<<std::endl;
    of<<std::fixed<<std::setprecision(9)<<vertices_.Global_Pose_.x<<" "<<vertices_.Global_Pose_.y<<" "<<vertices_.Global_Pose_.z<<" "<<vertices_.Global_Pose_.intensity<<std::endl;
    of<<vertices_.point_num_<<std::endl;
    for(const auto& n : vertices_.scanContext_8){
        of <<n<<"\n";
    }
    of<<std::fixed<<std::setprecision(17)<<time_stamp_<<endl;
    of.close();
    return true;
}
bool node::create_node_from_file_B(std::string path, int index){
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

    read_file>>vertices_.Global_Pose_.x>>vertices_.Global_Pose_.y>>vertices_.Global_Pose_.z>>vertices_.Global_Pose_.intensity;
    read_file>>vertices_.point_num_;

    vertices_.scanContext_8.clear();
    for(int i = 0; i < SHIFTSIZE; i++){
        Eigen::MatrixXd tempSc = Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
        for(int row = 0; row<PC_NUM_RING; ++row){
            for(int col = 0; col<PC_NUM_SECTOR; ++col){
                read_file>>tempSc(row,col);
            }
        }
        vertices_.scanContext_8.push_back(tempSc);
    }

    read_file>>time_stamp_;
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

#endif
#define PCL_NO_PRECOMPILE

#ifndef _INS_MAP_H_
#define _INS_MAP_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <deque>
#include <unordered_set>
#include <unordered_map>
#include <assert.h>
#include <algorithm>
#include <exception>
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cmath>
#include <tuple>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/qhull.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <opencv2/opencv.hpp>

#include "basic.h"
#include "odom/common.h"
#include "odom/tic_toc.h"

const float precision = 1E-2;

typedef pcl::PointCloud<PointType> PolygonType;

using SeqId = uint32_t;

class alignas(16) Instance
{
public:
    Instance(){}
    Instance(pcl::PointCloud<PointType>::Ptr cloud_, int id_)
    {
        this->cloud = cloud_;
        this->id = id_;
        BuildPolygonBoundariesFromCloud();
        this->update = false;
    }
    // 从其他实例深拷贝
    void clone(const Instance &rhs)
    {
        *this = rhs;
        pcl::copyPointCloud<PointType, PointType>(*(rhs.cloud), *cloud);
        pcl::copyPointCloud<PointType, PointType>(*(rhs.cloud_2dshape), *cloud_2dshape);
    }
    // 检查初始化
    bool init()
    {
        if (this->cloud == nullptr)
            return false;
        if (this->cloud->empty())
            return false;
        if (this->id == -1)
            return false;
        return BuildPolygonBoundariesFromCloud();
    }
    // 实例标签
    int id = -1;
    // 是否更新外观
    bool update = true;
    // 几何点云
    pcl::PointCloud<PointType>::Ptr cloud;
    // 外观点云 凸包
    pcl::PointCloud<PointType>::Ptr cloud_2dshape;
    Eigen::Vector2f shape;
    // BBox信息 凸包
    pcl::PolygonMesh convex_mesh;
    // 平面中心
    float area = 0;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    // 均匀高度
    float min_height = 0;
    float max_height = 0;
    float height = 0;
    // 矩形边界
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    // 首先计算边界 确定中心 对点云做 Z轴大距离下采样 XY轴小分辨率下采样 后确定形状
    // 针对退化方向保留一个确定的宽度
    bool BuildPolygonBoundariesFromCloud()
    {
        Eigen::Vector4f min_pt_, max_pt_;
        Eigen::Vector4f distribute;
        GetCloudMinMax3D(this->cloud, min_pt_, max_pt_, distribute);
        Eigen::Vector3f center((min_pt_[0] + max_pt_[0]) / 2, //初始化center点
                               (min_pt_[1] + max_pt_[1]) / 2,
                               (min_pt_[2] + max_pt_[2]) / 2);
        // 确保边界宽度为0.05
        float epslin = 1e-1;
        for (int i = 0; i < 3; i++)
        {
            if (max_pt_[i] - min_pt_[i] < epslin)
            {
                max_pt_[i] = center[i] + epslin / 2;
                min_pt_[i] = center[i] - epslin / 2;
            }
        }
        // 中心
        this->center = Eigen::Vector3f((max_pt_[0] + min_pt_[0]) / 2,
                                       (max_pt_[1] + min_pt_[1]) / 2,
                                       (max_pt_[2] + min_pt_[2]) / 2);
        // 点云高度简化为平均高度模型
        this->min_height = min_pt_[2];
        this->max_height = max_pt_[2];
        this->height = max_pt_[2] - min_pt_[2];
        this->min_pt = min_pt_;
        this->max_pt = max_pt_;

        pcl::PointCloud<PointType>::Ptr pcd_xy(new pcl::PointCloud<PointType>()); //将点云投影到地平面上,z坐标全部换成目标障碍物的最小z坐标
        // 使用voxelgrid下采样
        pcl::VoxelGrid<PointType> donw_sample_XYY2XY;
        Eigen::Vector3f donw_sample_leaf_size;
        donw_sample_leaf_size(0) = abs(distribute(0)) > 0.1 ? abs(distribute(0)) : 0.1;
        donw_sample_leaf_size(1) = abs(distribute(1)) > 0.1 ? abs(distribute(1)) : 0.1;
        donw_sample_leaf_size(2) = this->height;

        // donw_sample_XYY2XY.setLeafSize(donw_sample_leaf_size(0), donw_sample_leaf_size(1), donw_sample_leaf_size(2));
        // donw_sample_XYY2XY.setInputCloud(this->cloud);
        // donw_sample_XYY2XY.filter(*pcd_xy);
        *pcd_xy = *(this->cloud);
        // 将点云投影到地面上
        for (auto &p : pcd_xy->points)
        {
            p.z = 0;
        }
        // 点云数量太小
        if (pcd_xy->size() < 4)
            return false;
        // 分析凸包点云主次方向大小
        this->shape = pca_2d(pcd_xy);
        if(this->shape(1) < 0.5 || this->shape(0) < 0.01) return false;
        // 对2d_shape 求凹/凸包
        pcl::ConvexHull<PointType> convex_hull;
        convex_hull.setDimension(2);
        convex_hull.setComputeAreaVolume(true);
        convex_hull.setInputCloud(pcd_xy);
        pcl::PolygonMesh mesh_resutlt;
        try
        {
            convex_hull.reconstruct(mesh_resutlt);
        }
        catch (std::exception &e)
        {
            for (auto &p : pcd_xy->points)
            {
                printf("\033[31m(%f,%f)\033[0m\n", p.x, p.y);
            }
        }
        pcl::PointCloud<PointType>::Ptr plane_hull(new pcl::PointCloud<PointType>());
        pcl::fromPCLPointCloud2<PointType>(mesh_resutlt.cloud, *plane_hull);
        this->cloud_2dshape = plane_hull;
        this->convex_mesh = mesh_resutlt;
        this->area = convex_hull.getTotalArea();
        this->update = false;
        return true;
    }
    // 获取点云边界 以及XY上的均值和协方差
    static void GetCloudMinMax3D(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Vector4f &min_pt_, Eigen::Vector4f &max_pt_, Eigen::Vector4f &distribute)
    {
        int point_size = cloud->size();
        float x_sum = 0, y_sum = 0, x_sum_sq = 0, y_sum_sq = 0;
        min_pt_[0] = min_pt_[1] = min_pt_[2] = FLT_MAX;
        max_pt_[0] = max_pt_[1] = max_pt_[2] = -FLT_MAX;
        for (size_t i = 0; i < point_size; ++i)
        {
            x_sum += cloud->points[i].x;
            y_sum += cloud->points[i].y;
            x_sum_sq += cloud->points[i].x * cloud->points[i].x;
            y_sum_sq += cloud->points[i].y * cloud->points[i].y;
            min_pt_[0] = std::min(min_pt_[0], cloud->points[i].x);
            max_pt_[0] = std::max(max_pt_[0], cloud->points[i].x);
            min_pt_[1] = std::min(min_pt_[1], cloud->points[i].y);
            max_pt_[1] = std::max(max_pt_[1], cloud->points[i].y);
            min_pt_[2] = std::min(min_pt_[2], cloud->points[i].z);
            max_pt_[2] = std::max(max_pt_[2], cloud->points[i].z);
        }
        // 均值 方差
        x_sum -= (min_pt_(0) + max_pt_(0)) * 0.5 * point_size;
        y_sum -= (min_pt_(1) + max_pt_(1)) * 0.5 * point_size;
        distribute(0) = x_sum / point_size;
        distribute(1) = y_sum / point_size;
        distribute(2) = (x_sum_sq - x_sum * distribute(0) / (point_size - 1));
        distribute(3) = (y_sum_sq - y_sum * distribute(1) / (point_size - 1));
    }
};

typedef std::shared_ptr<Instance> InstancePtr;
typedef std::vector<std::shared_ptr<Instance>> InstancesPtr;

// 基础保存
bool SaveInstance(std::shared_ptr<Instance> ins, std::string &saveFilePathDir);
// 基础加载
std::shared_ptr<Instance> LoadInstanace(std::string &loadFileName);
// 保存
bool SaveInstances(std::vector<std::shared_ptr<Instance>> &instances, std::string &saveFilePathDir);
// 目录 后缀
std::vector<std::string> GetFiles(const char *src_dir, const char *ext);
// 加载
int LoadInstanaces(std::vector<std::shared_ptr<Instance>> &instances, std::string &loadFilePathDir);
// 变换点云
void TransformPointCloud(const Eigen::Matrix4f &trans_mat, pcl::PointCloud<PointType> &cloud_in_out);

// bfs based component analysis
void ConnectComponentAnalysis(const std::vector<std::vector<int>> &graph, std::vector<std::vector<int>> *components);

// 计算两个凸包相交区域占比
std::pair<float, float> Get2InssCollisionVolumePercent(InstancePtr &ins_a, InstancePtr &ins_b);

// 计算两个聚类距离
float ComputeDistance(InstancePtr new_ins, InstancePtr map_ins);

// 计算多个聚类之间的距离关系
void ComputeAssociateMatrix(InstancesPtr &new_inss, InstancesPtr &map_inss, Eigen::MatrixXf &association_mat);

// 计算关联矩阵
void ComputeConnectedComponents(const Eigen::MatrixXf &association_mat, const float connected_threshold, std::vector<std::vector<int>> &new_ins_components, std::vector<std::vector<int>> &map_ins_components);

// 最小化代价匹配二部图
void AssignMapInssToNewInss(const Eigen::MatrixXf &association_mat,
                            const float assign_distance_maximum,
                            std::vector<std::pair<int, int>> &assignments,
                            std::vector<int> &unassigned_new_inss,
                            std::vector<int> &unassigned_map_inss);

// 匹配连接部分
void MatchInComponents(const Eigen::MatrixXf &association_mat,
                       const std::vector<int> &new_inss_component,
                       const std::vector<int> &map_inss_component,
                       std::vector<std::pair<int, int>> &sub_assignments,
                       std::vector<int> &sub_unassigned_new_inss,
                       std::vector<int> &sub_unassigned_map_inss,
                       float s_match_distance_maximum_);

// 匹配两簇 Instances
std::vector<std::pair<int, int>> Match2Instances(InstancesPtr &new_inss, InstancesPtr &map_inss, float s_match_distance_maximum_);

// 获取 无匹配的 New Instance 暂时保留下来
InstancesPtr UnassignedInstance(InstancesPtr &new_inss, std::vector<std::pair<int, int>> &assignments);

// 检索 匹配合并了的 instanc 需要 match_info 因此删除 没匹配和并 的 instanc 防止重复重建 重复吸附到 global 删除match信息
bool RemoveNoneMatchNoneMergeInstance(InstancesPtr &inss, std::vector<std::pair<int, int>> &match);

// 合并点云
void mergeSource2Map(InstancePtr new_ins, InstancePtr map_ins);

// 利用栅格基于点云密度合并Instance，将Map_k转到L_k+1下，在L_k+1坐标系下将Map_k与P_k+1合并成Map_k+1
void mergeSource2Map(InstancePtr new_ins, InstancePtr map_ins, bool no_downsample);

// 同一块内存的共享指针
bool isSamePtr(InstancePtr &a, InstancePtr &b);

// 将Instance点云转到给定坐标下
void TransformInstance(InstancePtr ins, const Eigen::Matrix4f &pose);

// 将Instances转到给定坐标下
void TransformInstances(const std::vector<std::shared_ptr<Instance>> &instances, const Eigen::Matrix4f &pose);

// 深度搜索连接
void dfs_connect(Eigen::MatrixXf &mat, int id, std::vector<bool> &visited, std::vector<int> &connect);

class InsMapTestOptions
{
public:
    Eigen::Matrix4f velodyne_trans;
public:
    InsMapTestOptions(){velodyne_trans = Eigen::Matrix4f::Identity();}
    InsMapTestOptions(Eigen::Matrix4f pose) : velodyne_trans(pose) {}
};

class MapConfig
{
public:
    float visible_search_radius = 100.0;
    float match_distance_maximum = 4.0;
    float merge_map_search_radius = 300;
public:
    MapConfig(){}
};

class InsMapTest
{
public:
    InsMapTest()
    {
        instances_center_map.reset(new pcl::PointCloud<PointType>());
        instances_vertex_map.reset(new pcl::PointCloud<PointType>());
        instances_map_id = 0;
        global_map_manager.clear();
    }

    // merge new instanc into maps
    bool Merge(InstancesPtr &instances, float timestamp, InsMapTestOptions option);

    // get local global instanc
    InstancesPtr GetLocalGlobalMap() const;

    // get local instancs
    InstancesPtr GetLocalMap() const;

    // get global instanc
    InstancesPtr GetGlobalMap() const;

private:
    // initial the map
    bool Initialize(InstancesPtr &instances, float timestamp, InsMapTestOptions option);
    // 获取所有 local instanc
    void CollectVisibleLocalInstances(InstancesPtr &local_instances);
    // 获取所有 temp instanc
    void CollectVisibleTempInstances(InstancesPtr &temp_instances);
    // 获取当前帧一定范围内的 global instanc 使用 kdtree 范围搜索 后面需要加遮挡判断等
    void CollectVisibleGlobalInstances(InstancesPtr &global_instances);

    // 记录当前位姿
    void RecordPose(Eigen::Matrix4f &pose, float timestamp);

    // 匹配两簇 Instances
    std::vector<std::pair<int, int>> InsMapTest::Match2Instances(const InstancesPtr &new_inss,const InstancesPtr &map_inss, float s_match_distance_maximum_);

    // 删除时间久远的local
    void DeleteFarAwayLocalMap();

    // 将与 global 匹配的 new instanc 进行归拢
    bool MergeGlobalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &map_inss, std::vector<std::pair<int, int>> &assignments);

    // 将与 loca 匹配的 new instanc 进行添加
    bool MergeLocalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &local_inss, std::vector<std::pair<int, int>> &assignments);

    // 重建更新点云后的Instances
    bool RebuildInstances(InstancesPtr &ins);

    // 归拢 Instances 中的重复的 Instance 针对输入进行处理
    bool MergeSameInstancesIndividual(InstancesPtr &inss);

    // 归拢 Instances 中的重复的 Instance 并记录到地图 返回新注册的 instanc
    InstancesPtr MergeSameInstancesToMap(InstancesPtr &inss_map, InstancesPtr &inss_local);

    // 根据新注册更新的 global 整顿 global 中的其他 待完成多棱柱后开搞
    bool MergeGlobalMap(InstancesPtr &new_reg_inss);

private:
    // ID-Uint32 低十六位0X0000FFFF记录localmap中的id 高十六位0XFFFF0000记录tempmap中的id 负数记录globalmap中的id
    int instances_global_id = -1;
    int instances_local_id = 0;
    int instances_temp_id = 65536;

    std::unordered_map<InstancePtr, float> temp_map_manager;
    std::unordered_map<InstancePtr, float> local_map_manager;
    std::unordered_map<InstancePtr, int> global_map_manager;

    std::deque<std::tuple<InstancePtr, InstancePtr, float, float>> match_record;
    

    pcl::PointCloud<PointType>::Ptr instances_center_map;
    pcl::KdTreeFLANN<PointType> instances_center_map_kdtree;

    pcl::PointCloud<PointType>::Ptr instances_vertex_map;
    pcl::KdTreeFLANN<PointType> instances_vertex_map_kdtree;

    std::vector<std::pair<float, Eigen::Matrix4f>> trajectory;

    float now_timestamp;
    Eigen::Matrix4f now_pose;
    Eigen::Vector3f now_posi;

    MapConfig map_config;

    bool first_run = true;
};

#endif //_INS_MAP_H_

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

#include "instance.h"
#include "hungarian_bigraph_matcher.h"

typedef std::shared_ptr<Instance> InstancePtr;
typedef std::vector<std::shared_ptr<Instance>> InstancesPtr;

// 获取中心与边界
void getMinMax3DAndDummyCenter(pcl::PointCloud<PointType>::Ptr &cloud,
                               Eigen::Vector4f &min_pt,
                               Eigen::Vector4f &max_pt,
                               Eigen::Vector4f &center)
{
    long long sum_x = 0, sum_y = 0, sum_z = 0;
    min_pt = Eigen::Vector4f(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
    max_pt = Eigen::Vector4f(-FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (auto &p : cloud->points)
    {
        sum_x += static_cast<int>(p.x);
        sum_y += static_cast<int>(p.y);
        sum_z += static_cast<int>(p.z);
        if (p.x < min_pt(0))
            min_pt(0) = p.x;
        if (p.y < min_pt(1))
            min_pt(1) = p.y;
        if (p.z < min_pt(2))
            min_pt(2) = p.z;
        if (p.x > max_pt(0))
            max_pt(0) = p.x;
        if (p.y > max_pt(1))
            max_pt(1) = p.y;
        if (p.z > max_pt(2))
            max_pt(2) = p.z;
    }
    center(0) = sum_x / cloud->size();
    center(1) = sum_y / cloud->size();
    center(2) = sum_z / cloud->size();
}
// 变换点云
void TransformPointCloud(const Eigen::Matrix4d &trans_mat, pcl::PointCloud<PointType> &cloud_in_out)
{
    for (std::size_t i = 0; i < cloud_in_out.size(); ++i)
    {
        PointType &p = cloud_in_out.points[i];
        Eigen::Vector4d v(p.x, p.y, p.z, 1);
        v = trans_mat * v;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
    }
}

// bfs based component analysis
void ConnectComponentAnalysis(const std::vector<std::vector<int>> &graph, std::vector<std::vector<int>> *components)
{
    int no_item = graph.size();
    std::vector<int> visited;
    visited.resize(no_item, 0);
    std::queue<int> que;
    std::vector<int> component;
    components->clear();

    for (int i = 0; i < no_item; ++i)
    {
        if (visited[i])
        {
            continue;
        }
        component.push_back(i);
        que.push(i);
        visited[i] = 1;
        while (!que.empty())
        {
            int id = que.front();
            que.pop();
            for (size_t j = 0; j < graph[id].size(); ++j)
            {
                int nb_id = graph[id][j];
                if (visited[nb_id] == 0)
                {
                    component.push_back(nb_id);
                    que.push(nb_id);
                    visited[nb_id] = 1;
                }
            }
        }
        components->push_back(component);
        component.clear();
    }
}

// 计算两个聚类距离
float ComputeDistance(InstancePtr new_ins, InstancePtr map_ins)
{
    // Compute distance for given new_ins & map_ins
    float location_distance = (new_ins->center - map_ins->center).cast<float>().norm(); //距离差异

    float result_distance = location_distance; //各个差异*权值
    return result_distance;
}

// 计算多个聚类之间的距离关系
void ComputeAssociateMatrix(InstancesPtr &new_inss, InstancesPtr &map_inss, Eigen::MatrixXf &association_mat)
{
    // Compute matrix of association distance
    for (size_t i = 0; i < new_inss.size(); ++i)
    {
        for (size_t j = 0; j < map_inss.size(); ++j)
        {
            association_mat(i, j) = ComputeDistance(new_inss[i], map_inss[j]);
        }
    }
}

// 计算关联矩阵
void ComputeConnectedComponents(const Eigen::MatrixXf &association_mat, const float connected_threshold, std::vector<std::vector<int>> &new_ins_components, std::vector<std::vector<int>> &map_ins_components)
{
    // Compute connected components within given threshold
    int no_new_ins = association_mat.rows();
    int no_map_ins = association_mat.cols();
    std::vector<std::vector<int>> nb_graph;
    nb_graph.resize(no_new_ins + no_map_ins);
    for (int i = 0; i < no_new_ins; i++)
    {
        for (int j = 0; j < no_map_ins; j++)
        {
            if (association_mat(i, j) <= connected_threshold)
            {
                nb_graph[i].push_back(no_new_ins + j);
                nb_graph[j + no_new_ins].push_back(i);
            }
        }
    }

    std::vector<std::vector<int>> components;
    ConnectComponentAnalysis(nb_graph, &components);
    new_ins_components.clear();
    new_ins_components.resize(components.size());
    map_ins_components.clear();
    map_ins_components.resize(components.size());
    for (size_t i = 0; i < components.size(); i++)
    {
        for (size_t j = 0; j < components[i].size(); j++)
        {
            int id = components[i][j];
            if (id < no_new_ins)
            {
                new_ins_components[i].push_back(id);
            }
            else
            {
                id -= no_new_ins;
                map_ins_components[i].push_back(id);
            }
        }
    }
}

// 最小化代价匹配二部图
void AssignMapInssToNewInss(const Eigen::MatrixXf &association_mat,
                            const double assign_distance_maximum,
                            std::vector<std::pair<int, int>> &assignments,
                            std::vector<int> &unassigned_new_inss,
                            std::vector<int> &unassigned_map_inss)
{
    // Assign map_ins to new_ins with null setup
    std::vector<int> new_inss_idx;
    std::vector<int> map_inss_idx;
    int no_new_inss = association_mat.rows();
    int no_map_inss = association_mat.cols();
    // build cost
    std::vector<std::vector<double>> cost(no_new_inss + no_map_inss);
    for (int i = 0; i < no_new_inss; ++i)
    {
        cost[i].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            cost[i][j] = association_mat(i, j);
        }
    }
    for (int i = 0; i < no_map_inss; ++i)
    {
        cost[i + no_new_inss].resize(no_map_inss);
        for (int j = 0; j < no_map_inss; ++j)
        {
            if (j == i)
            {
                cost[i + no_new_inss][j] = assign_distance_maximum * 1.2f;
            }
            else
            {
                cost[i + no_new_inss][j] = 999999.0f;
            }
        }
    }

    HungarianOptimizer hungarian_optimizer(cost);
    hungarian_optimizer.minimize(&new_inss_idx, &map_inss_idx);

    int assignments_num = 0;
    std::vector<bool> new_inss_used(no_new_inss + no_map_inss, false);
    std::vector<bool> map_inss_used(no_map_inss, false);
    for (size_t i = 0; i < new_inss_idx.size(); ++i)
    {
        if (new_inss_idx[i] < 0 || new_inss_idx[i] >= no_new_inss || map_inss_idx[i] < 0 ||
            map_inss_idx[i] >= no_map_inss)
        {
            continue;
        }
        if (association_mat(new_inss_idx[i], map_inss_idx[i]) <
            assign_distance_maximum)
        {
            assignments[assignments_num++] = std::make_pair(new_inss_idx[i], map_inss_idx[i]);
            new_inss_used[new_inss_idx[i]] = true;
            map_inss_used[map_inss_idx[i]] = true;
        }
    }
    assignments.resize(assignments_num);
    unassigned_new_inss.resize(association_mat.rows());
    int unassigned_new_inss_num = 0;
    for (int i = 0; i < association_mat.rows(); ++i)
    {
        if (new_inss_used[i] == false)
        {
            unassigned_new_inss[unassigned_new_inss_num++] = i;
        }
    }
    unassigned_new_inss.resize(unassigned_new_inss_num);
    unassigned_map_inss.resize(association_mat.cols());
    int unassigned_map_inss_num = 0;
    for (int i = 0; i < association_mat.cols(); ++i)
    {
        if (map_inss_used[i] == false)
        {
            unassigned_map_inss[unassigned_map_inss_num++] = i;
        }
    }
    unassigned_map_inss.resize(unassigned_map_inss_num);
}

// 匹配连接部分
void MatchInComponents(const Eigen::MatrixXf &association_mat,
                       const std::vector<int> &new_inss_component,
                       const std::vector<int> &map_inss_component,
                       std::vector<std::pair<int, int>> &sub_assignments,
                       std::vector<int> &sub_unassigned_new_inss,
                       std::vector<int> &sub_unassigned_map_inss,
                       double s_match_distance_maximum_)
{
    sub_assignments.clear();
    sub_unassigned_new_inss.clear();
    sub_unassigned_map_inss.clear();
    // A. failed to match if either components is empty
    if (new_inss_component.empty())
    {
        for (size_t i = 0; i < map_inss_component.size(); ++i)
        {
            sub_unassigned_map_inss.push_back(map_inss_component[i]);
        }
    }
    if (map_inss_component.empty())
    {
        for (size_t i = 0; i < new_inss_component.size(); ++i)
        {
            sub_unassigned_new_inss.push_back(new_inss_component[i]);
        }
    }
    if (new_inss_component.empty() || map_inss_component.empty())
        return;
    // B. if components perfectly match
    if (new_inss_component.size() == 1 && map_inss_component.size() == 1)
    {
        int new_ins_id = new_inss_component[0];
        int map_ins_id = map_inss_component[0];
        if (association_mat(new_ins_id, map_ins_id) <= s_match_distance_maximum_)
        {
            sub_assignments.push_back(std::make_pair(new_ins_id, map_ins_id));
        }
        else
        {
            sub_unassigned_map_inss.push_back(map_ins_id);
            sub_unassigned_new_inss.push_back(new_ins_id);
        }
        return;
    }
    // C. multi instanc match
    std::vector<int> new_inss_local2global;
    std::vector<int> map_inss_local2global;
    std::vector<std::pair<int, int>> local_assignments;
    std::vector<int> local_unassigned_new_inss;
    std::vector<int> local_unassigned_map_inss;
    Eigen::MatrixXf local_association_mat(new_inss_component.size(), map_inss_component.size());
    new_inss_local2global.resize(new_inss_component.size());
    map_inss_local2global.resize(map_inss_component.size());
    for (size_t i = 0; i < new_inss_component.size(); ++i)
    {
        new_inss_local2global[i] = new_inss_component[i];
    }
    for (size_t i = 0; i < map_inss_component.size(); ++i)
    {
        map_inss_local2global[i] = map_inss_component[i];
    }
    for (size_t i = 0; i < new_inss_component.size(); ++i)
    {
        for (size_t j = 0; j < map_inss_component.size(); ++j)
        {
            int new_ins_id = new_inss_component[i];
            int map_ins_id = map_inss_component[j];
            local_association_mat(i, j) = association_mat(new_ins_id, map_ins_id);
        }
    }
    local_assignments.resize(local_association_mat.cols());
    local_unassigned_new_inss.assign(local_association_mat.rows(), -1);
    local_unassigned_map_inss.assign(local_association_mat.cols(), -1);
    AssignMapInssToNewInss(local_association_mat, s_match_distance_maximum_, local_assignments, local_unassigned_new_inss, local_unassigned_map_inss);
    for (size_t i = 0; i < local_assignments.size(); ++i)
    {
        int global_new_ins_id = new_inss_local2global[local_assignments[i].first];
        int global_map_ins_id = map_inss_local2global[local_assignments[i].second];
        sub_assignments.push_back(
            std::make_pair(global_new_ins_id, global_map_ins_id));
    }
    for (size_t i = 0; i < local_unassigned_new_inss.size(); ++i)
    {
        int global_new_ins_id = new_inss_local2global[local_unassigned_new_inss[i]];
        sub_unassigned_new_inss.push_back(global_new_ins_id);
    }
    for (size_t i = 0; i < local_unassigned_map_inss.size(); ++i)
    {
        int global_map_ins_id = map_inss_local2global[local_unassigned_map_inss[i]];
        sub_unassigned_map_inss.push_back(global_map_ins_id);
    }
}

// 匹配两簇 Instances
std::vector<std::pair<int, int>> Match2Instances(InstancesPtr &new_inss, InstancesPtr &map_inss, double s_match_distance_maximum_)
{
    std::vector<std::pair<int, int>> assignments;
    std::vector<int> unassigned_new_inss;
    std::vector<int> unassigned_map_inss;
    // A. computing association matrix
    Eigen::MatrixXf association_mat(new_inss.size(), map_inss.size());
    ComputeAssociateMatrix(new_inss, map_inss, association_mat);

    // B. computing connected components
    std::vector<std::vector<int>> new_inss_components;
    std::vector<std::vector<int>> map_inss_components;
    ComputeConnectedComponents(association_mat, s_match_distance_maximum_, new_inss_components, map_inss_components);

    // C. matching each sub-graph
    assignments.clear();
    unassigned_new_inss.clear();
    unassigned_map_inss.clear();
    for (size_t i = 0; i < new_inss_components.size(); i++)
    {
        std::vector<std::pair<int, int>> sub_assignments;
        std::vector<int> sub_unassigned_new_inss;
        std::vector<int> sub_unassigned_map_inss;
        MatchInComponents(association_mat,
                          new_inss_components[i], map_inss_components[i],
                          sub_assignments,
                          sub_unassigned_new_inss, sub_unassigned_map_inss,
                          s_match_distance_maximum_);
        for (size_t j = 0; j < sub_assignments.size(); ++j)
        {
            int new_ins_id = sub_assignments[j].first;
            int map_ins_id = sub_assignments[j].second;
            assignments.push_back(sub_assignments[j]);
            float association_score = association_mat(new_ins_id, map_ins_id);
            // (*instances)[map_ins_id]->association_score = association_score;
        }
        for (size_t j = 0; j < sub_unassigned_new_inss.size(); ++j)
        {
            unassigned_new_inss.push_back(sub_unassigned_new_inss[j]);
        }
        for (size_t j = 0; j < sub_unassigned_map_inss.size(); ++j)
        {
            unassigned_map_inss.push_back(sub_unassigned_map_inss[j]);
        }
    }

    return assignments;
}

// 获取 无匹配的 New Instance 暂时保留下来
InstancesPtr UnassignedInstance(InstancesPtr &new_inss, std::vector<std::pair<int, int>> &assignments)
{
    InstancesPtr unassignments_new;
    int assignment_number = assignments.size();
    std::vector<bool> unassignments_new_idx(new_inss.size(), false);
    for (int i = 0; i < assignment_number; ++i)
    {
        unassignments_new_idx[assignments[i].first] = true;
    }
    for (int i = 0; i < new_inss.size(); ++i)
    {
        if (!unassignments_new_idx[i])
        {
            unassignments_new.push_back(new_inss[i]);
        }
    }
    return unassignments_new;
}

// 检索 匹配合并了的 instanc 需要 match_info 因此删除 没匹配和并 的 instanc 防止重复重建 重复吸附到 global
bool RemoveNoneMatchNoneMergeInstance(InstancesPtr &inss,  std::vector<std::pair<int, int>> &match)
{
    if (inss.empty() || match.empty())
    {
        inss.clear();
        return true;
    }
    InstancesPtr inss_;
    inss_.swap(inss);
    for (auto &p : match)
    {   
        if( !inss_[p.second]->update ) continue;
        inss.push_back( inss_[p.second] );
    }
    match.clear();
    return true;
}

//利用栅格基于点云密度合并Instance，将Map_k转到L_k+1下，在L_k+1坐标系下将Map_k与P_k+1合并成Map_k+1
void mergeSource2Map(InstancePtr new_ins, InstancePtr map_ins)
{
    pcl::PointCloud<PointType>::Ptr &source = new_ins->cloud;
    pcl::PointCloud<PointType>::Ptr &map = map_ins->cloud;
    int map_origin_points_size = map->size();
    double lx = 0.5, ly = 0.5, lz = 0.5;
    Eigen::Vector4f minpt, maxpt, dummy_center;
    std::map<int, std::pair<int, int>> dest_map; // key:voxel_i - val:map_i, source_i

    // edge and center decentralization
    std::vector<double> new_edge_x{new_ins->vertex1(0), new_ins->vertex2(0), new_ins->vertex3(0), new_ins->vertex4(0)};
    std::vector<double> new_edge_y{new_ins->vertex1(1), new_ins->vertex2(1), new_ins->vertex3(1), new_ins->vertex4(1)};
    minpt(0) = *std::min_element(new_edge_x.begin(), new_edge_x.end());
    minpt(1) = *std::min_element(new_edge_y.begin(), new_edge_y.end());
    maxpt(0) = *std::max_element(new_edge_x.begin(), new_edge_x.end());
    maxpt(1) = *std::max_element(new_edge_y.begin(), new_edge_y.end());
    minpt(2) = new_ins->min_height;
    maxpt(2) = new_ins->max_height;
    dummy_center(0) = (minpt(0) + maxpt(0)) / 2;
    dummy_center(1) = (minpt(1) + maxpt(1)) / 2;
    dummy_center(2) = (minpt(2) + maxpt(2)) / 2;

    minpt -= dummy_center;
    maxpt -= dummy_center;
    // grid
    int width = std::ceil((maxpt(0) - minpt(0)) / lx);
    int len = std::ceil((maxpt(1) - minpt(1)) / ly);
    int hei = std::ceil((maxpt(2) - minpt(2)) / lz);
    int perh = width * len;

    // decentralization register
    for (size_t i = 0; i < map->size(); ++i)
    {
        auto &p = map->points[i];
        if (p.x < minpt(0) || p.y < minpt(1) || p.z < minpt(2) ||
            p.x > maxpt(0) || p.y > maxpt(1) || p.z > maxpt(2))
            continue;

        int ind_x = std::floor((p.x - dummy_center(0) - minpt(0)) / lx);
        int ind_y = std::floor((p.y - dummy_center(1) - minpt(1)) / ly);
        int ind_z = std::floor((p.z - dummy_center(2) - minpt(2)) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        dest_map[ind] = std::make_pair(i, -1);
    }

    for (size_t i = 0; i < source->size(); ++i)
    {
        auto p = source->points[i];
        int ind_x = std::floor((p.x - dummy_center(0) - minpt(0)) / lx);
        int ind_y = std::floor((p.y - dummy_center(1) - minpt(1)) / ly);
        int ind_z = std::floor((p.z - dummy_center(2) - minpt(2)) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        auto it = dest_map.find(ind);
        if (it == dest_map.end())
        {
            map->push_back(p);
            dest_map[ind] = std::make_pair(-1, i);
        }
    }
    int map_merge_size = map->size();
    if (map_merge_size != map_origin_points_size)
        map_ins->update = true;
}

// 计算三角面积
double GetConvexHullArea(Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3)
{
    return std::abs(0.5 * (p1(0) * p2(1) + p2(0) * p3(1) + p3(0) * p1(1) - p1(0) * p3(1) - p2(0) * p1(1) - p3(0) * p2(1)));
}

// 计算矩形面积 vector size : 4
double GetRectangleArea(std::vector<Eigen::Vector2d> &rect)
{
    if (rect.size() != 4)
        return -1;
    return 2.0 * GetConvexHullArea(rect[0], rect[1], rect[2]);
}

// ab cd 两线平行 vector size : 2

bool parallel(std::vector<Eigen::Vector2d> &l1, std::vector<Eigen::Vector2d> &l2)
{
    double v_x = (l1[1](1) - l1[0](1)) * (l2[1](0) - l2[0](0)) - (l2[1](1) - l2[0](1)) * (l1[1](0) - l1[0](0));
    if (v_x <= 0.1 && v_x >= -0.1)
        return true;
    return false;
}

// vec(p1, p2) x vec(p1, p) : (b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y)
double GetCross(Eigen::Vector2d &a, Eigen::Vector2d &b, Eigen::Vector2d &c)
{
    return (b(0) - a(0)) * (c(1) - a(1)) - (c(0) - a(0)) * (b(1) - a(1));
}

// def parallel(ax, ay, bx, by, cx, cy, dx, dy):
//     return (by - ay) * (dx - cx) - (dy - cy) * (bx - ax)
// ax=33.890000 
// ay=13.530000 
// bx=41.300000
// by=8.790000 
// cx=50.050000
// cy=22.470000 
// dx=42.640000
// dy=27.200000

//
// 判断点在矩形内
bool IsPointInMatrix(Eigen::Vector2d &p, std::vector<Eigen::Vector2d> &rect1)
{
    return GetCross(rect1[0], rect1[1], p) * GetCross(rect1[2], rect1[3], p) >= 0 && GetCross(rect1[1], rect1[2], p) * GetCross(rect1[3], rect1[0], p) >= 0;
}

// 计算线段交点 vector size : 2 重合相交无交点 顶点相交无交点 交叉相交一交点
bool GetIntersection(std::vector<Eigen::Vector2d> &l1, std::vector<Eigen::Vector2d> &l2, Eigen::Vector2d &intersect_point)
{
    if (parallel(l1, l2))
        return false;
    // assert(parallel(l1, l2) == false);
    intersect_point = Eigen::Vector2d(-1, -1);

    double cross_12_13 = (l1[1](0) - l1[0](0)) * (l2[0](1) - l1[0](1)) - (l2[0](0) - l1[0](0)) * (l1[1](1) - l1[0](1));
    double cross_12_14 = (l1[1](0) - l1[0](0)) * (l2[1](1) - l1[0](1)) - (l2[1](0) - l1[0](0)) * (l1[1](1) - l1[0](1));
    double cross_34_31 = (l2[1](0) - l2[0](0)) * (l1[0](1) - l2[0](1)) - (l1[0](0) - l2[0](0)) * (l2[1](1) - l2[0](1));
    double cross_34_32 = (l2[1](0) - l2[0](0)) * (l1[1](1) - l2[0](1)) - (l1[1](0) - l2[0](0)) * (l2[1](1) - l2[0](1));

    if (cross_12_13 * cross_12_14 <= 0 && cross_34_31 * cross_34_32 <= 0)
    {
        if (cross_12_14 == 0) // 12 coincide 14 the corss is 4
        {
            return false;
        }
        double lambda = abs(cross_12_13 / cross_12_14);
        double x = l2[0](0) + (lambda / (lambda + 1.0)) * (l2[1](0) - l2[0](0));
        double y = l2[0](1) + (lambda / (lambda + 1.0)) * (l2[1](1) - l2[0](1));
        intersect_point(0) = x;
        intersect_point(1) = y;
        return true;
    }
    return false;
}

// 计算两矩形之间的重复面积 vector size : 4 ; return size : 3 s1 s2 sameAreaS abcd inclock
bool Get2RectangleImpactArea(std::vector<Eigen::Vector2d> &rect1, std::vector<Eigen::Vector2d> &rect2, std::vector<double> &result)
{
    double s1 = GetRectangleArea(rect1);
    double s2 = GetRectangleArea(rect2);
    result = std::vector<double>{s1, s2, -1};
    if (s1 <= 0 || s2 <= 0)
        return false;
    // 4 edges ans 4 edges : 4 x 2
    std::vector<std::vector<Eigen::Vector2d>> edges1(4, std::vector<Eigen::Vector2d>(2, Eigen::Vector2d::Zero()));
    std::vector<std::vector<Eigen::Vector2d>> edges2(edges1);
    edges1[0][0] = rect1[0];
    edges1[0][1] = rect1[1];
    edges1[1][0] = rect1[1];
    edges1[1][1] = rect1[2];
    edges1[2][0] = rect1[2];
    edges1[2][1] = rect1[3];
    edges1[3][0] = rect1[3];
    edges1[3][1] = rect1[0];
    // assert(parallel(edges1[0], edges1[2]) == true);
    // assert(parallel(edges1[1], edges1[3]) == true);
    edges2[0][0] = rect2[0];
    edges2[0][1] = rect2[1];
    edges2[1][0] = rect2[1];
    edges2[1][1] = rect2[2];
    edges2[2][0] = rect2[2];
    edges2[2][1] = rect2[3];
    edges2[3][0] = rect2[3];
    edges2[3][1] = rect2[0];
    // assert(parallel(edges2[0], edges2[2]) == true);
    // assert(parallel(edges2[1], edges2[3]) == true);
    if (!parallel(edges1[1], edges1[3]) || !parallel(edges1[0], edges1[2]))
    {
        std::cerr << "\033[31m"
                  << "parallel1 error here"
                  << "\033[0m" << std::endl;
        for (auto &p : rect1)
            std::cout << std::fixed << "(" << p(0) << "," << p(1) << ") ";
        std::cout << std::endl;
        exit(0);
    }
    if (!parallel(edges2[0], edges2[2]) || !parallel(edges2[1], edges2[3]))
    {
        std::cerr << "\033[31m"
                  << "parallel2 error here"
                  << "\033[0m" << std::endl;
        for (auto &p : rect2)
            std::cout << std::fixed << "(" << p(0) << "," << p(1) << ") ";
        std::cout << std::endl;
        exit(0);
    }
    // get intersections
    std::set<std::pair<double, double>> intersections_set;
    std::vector<Eigen::Vector2d> intersections;
    // std::cout << "intersection" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            Eigen::Vector2d intersection;
            if (!GetIntersection(edges1[i], edges2[j], intersection))
                continue;
            // std::cout << intersection(0) << " " << intersection(1) << std::endl;
            auto intersection_pair = std::make_pair(intersection(0), intersection(1));
            auto it = intersections_set.find(intersection_pair);
            if (it != intersections_set.end())
                continue;
            intersections_set.insert(intersection_pair);
            intersections.push_back(intersection);
        }
    }
    // get another inside points
    std::vector<Eigen::Vector2d> inside_points;
    for (int i = 0; i < 4; ++i)
    {
        if (IsPointInMatrix(rect1[i], rect2))
        {
            inside_points.push_back(rect1[i]);
        }
        if (IsPointInMatrix(rect2[i], rect1))
        {
            inside_points.push_back(rect2[i]);
        }
    }
    // convex vertex
    std::set<std::pair<double, double>> vertex_points_set;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_vertex(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &p : intersections)
    {
        std::pair<double, double> tmpPair = std::make_pair(p(0), p(1));
        if (vertex_points_set.find(tmpPair) != vertex_points_set.end())
            continue;
        vertex_points_set.insert(tmpPair);
        pcl::PointXYZ tmpPoint;
        tmpPoint.x = p(0);
        tmpPoint.y = p(1);
        convex_vertex->push_back(tmpPoint);
    }
    for (auto &p : inside_points)
    {
        std::pair<double, double> tmpPair = std::make_pair(p(0), p(1));
        if (vertex_points_set.find(tmpPair) != vertex_points_set.end())
            continue;
        vertex_points_set.insert(tmpPair);
        pcl::PointXYZ tmpPoint;
        tmpPoint.x = p(0);
        tmpPoint.y = p(1);
        convex_vertex->push_back(tmpPoint);
    }
    // std::cout << "points" << std::endl;
    // for(auto&p:convex_vertex->points){
    // std::cout << p.x << " " << p.y << std::endl;
    // }
    if (convex_vertex->size() < 3)
        return false;
    // get overlapping area convex hull
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(convex_vertex);
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    pcl::PolygonMesh mesh_out;
    convex_hull.reconstruct(mesh_out);
    double convex_area = convex_hull.getTotalArea();
    result[2] = convex_area;
    return true;
}

bool UpXVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a(0) < b(0);
}
bool UpYVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a(1) < b(1);
}
bool DownYVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a(1) > b(1);
}

void GetOrderedRect(std::vector<Eigen::Vector2d> &rect)
{
    std::sort(rect.begin(), rect.end(), UpXVector2);
    std::sort(rect.begin(), rect.begin() + 2, DownYVector2);
    std::sort(rect.begin() + 2, rect.end(), UpYVector2);
}
// 获取两个 Instance的 碰撞体积
std::pair<double, double> Get2InssCollisionVolumePercent(InstancePtr &ins1, InstancePtr &ins2)
{
    // z len
    std::pair<double, double> percent{0, 0};
    double overlap_z;
    Eigen::Vector2d ins1_z, ins2_z;
    ins1_z = Eigen::Vector2d{ins1->min_height, ins1->max_height};
    ins2_z = Eigen::Vector2d{ins2->min_height, ins2->max_height};
    double overlap_z_min = std::max(ins1_z(0), ins2_z(0));
    double overlap_z_max = std::min(ins1_z(1), ins2_z(1));
    if (overlap_z_max <= overlap_z_min)
    {
        overlap_z = 0;
        return percent;
    }
    overlap_z = overlap_z_max - overlap_z_min;

    // rect area
    std::vector<Eigen::Vector2d> ins1_rect, ins2_rect;
    ins1_rect.push_back(Eigen::Vector2d{ins1->vertex1[0], ins1->vertex1[1]});
    ins1_rect.push_back(Eigen::Vector2d{ins1->vertex2[0], ins1->vertex2[1]});
    ins1_rect.push_back(Eigen::Vector2d{ins1->vertex4[0], ins1->vertex4[1]});
    ins1_rect.push_back(Eigen::Vector2d{ins1->vertex3[0], ins1->vertex3[1]});

    ins2_rect.push_back(Eigen::Vector2d{ins2->vertex1[0], ins2->vertex1[1]});
    ins2_rect.push_back(Eigen::Vector2d{ins2->vertex2[0], ins2->vertex2[1]});
    ins2_rect.push_back(Eigen::Vector2d{ins2->vertex4[0], ins2->vertex4[1]});
    ins2_rect.push_back(Eigen::Vector2d{ins2->vertex3[0], ins2->vertex3[1]});
    std::vector<double> overlap;
    GetOrderedRect(ins1_rect);
    GetOrderedRect(ins2_rect);
    if (!Get2RectangleImpactArea(ins1_rect, ins2_rect, overlap))
    {
        percent = {0, 0};
        return percent;
    }
    double ins1_vol = overlap[0] * (ins1_z(1) - ins1_z(0));
    double ins2_vol = overlap[1] * (ins2_z(1) - ins2_z(0));
    double overlap_vol = overlap[2] * overlap_z;
    if (overlap[2] > overlap[1] + 0.1 || overlap[2] > overlap[0] + 0.1)
    {
        std::cerr << "\033[31m"
                  << "convex hull error here"
                  << "\033[0m" << std::endl;
        std::cout << "s1:" << overlap[0] << " s2:" << overlap[1] << " same:" << overlap[2] << std::endl;
        for (auto &p : ins1_rect)
            std::cout << std::fixed << "(" << p(0) << "," << p(1) << ") ";
        std::cout << std::endl;
        for (auto &p : ins2_rect)
            std::cout << std::fixed << "(" << p(0) << "," << p(1) << ") ";
        std::cout << std::endl;
    }
    percent = {overlap_vol / ins1_vol, overlap_vol / ins2_vol};
    return percent;
}

double GetDistance2AB(double &x1, double &y1, double &x2, double &y2)
{
    return (x1 - x2) * (y1 - y2);
}

double GetRectMaxRadius(InstancePtr &ins)
{
    double s1 = GetDistance2AB(ins->center(0), ins->center(1), ins->vertex1(0), ins->vertex1(1));
    double s2 = GetDistance2AB(ins->center(0), ins->center(1), ins->vertex2(0), ins->vertex2(1));
    double s3 = GetDistance2AB(ins->center(0), ins->center(1), ins->vertex3(0), ins->vertex3(1));
    double max = max = s1 > s2 ? s1 : s2;
    max = max > s3 ? max : s3;
    return max;
}

bool isSame(InstancePtr &a, InstancePtr &b)
{
    return !(a.owner_before(b) || b.owner_before(a));
}


class InsMapTestOptions
{
public:
    Eigen::Matrix4d velodyne_trans;

public:
    InsMapTestOptions()
    {
        velodyne_trans = Eigen::Matrix4d::Identity();
    }
    InsMapTestOptions(Eigen::Matrix4d pose) : velodyne_trans(pose) {}
};

class MapConfig
{
public:
    double search_radius = 100.0;
    double match_distance_maximum = 4.0;

public:
    MapConfig()
    {
    }
};

class InsMapTest
{
public:
    InsMapTest()
    {
        instances_center_map.reset(new pcl::PointCloud<PointType>());
        instances_vertex_map.reset(new pcl::PointCloud<pcl::PointXY>());
        instances_map_id = 0;
        global_map_manager.clear();
    }
    // merge new instanc into maps
    bool Merge(InstancesPtr &instances, double timestamp, InsMapTestOptions option)
    {
        std::cout << "##########################" << std::endl;
        // 合并新Instances中的同类
        MergeSameInstancesIndividual(instances);
        if (first_run)
        {
            first_run = false;
            return Initialize(instances, timestamp, option);
            std::cout << "First Run ##########################" << std::endl;
        }
        // 记录当前位姿
        RecordPose(option.velodyne_trans, timestamp);
        // 将Instances转移到世界坐标系下
        TransformInstances(instances, option.velodyne_trans);
        // 获取可见Instances位于世界坐标系下

        InstancesPtr local_map_instances, global_map_instances;
        CollectVisibleGlobalInstances(global_map_instances);
        CollectVisibleLocalInstances(local_map_instances);
        // 将 new instanc 与 global可见的 instanc 进行匹配
        auto matchInfo = Match2Instances(instances, global_map_instances, map_config.match_distance_maximum);
        std::cout << "global matched " << matchInfo.size() << std::endl;
        // 获取没有和 global 匹配的 new instanc
        auto unassignments_global_new = UnassignedInstance(instances, matchInfo);
        // 和 local 进行匹配
        auto local_match_info = Match2Instances(unassignments_global_new, local_map_instances, map_config.match_distance_maximum);
        std::cout << "local  matched " << local_match_info.size() << std::endl;

        // 获取 既没有和 global 也没有和 local 匹配的 new instance
        auto unassignments_global_local_new = UnassignedInstance(unassignments_global_new, local_match_info);
        std::cout << "none   matched " << unassignments_global_local_new.size() << std::endl;
        
        // 既没和 global 匹配上 也没和 local 匹配上的 new instance 就作为临时地图存在历史地图中
        for (auto &p : unassignments_global_local_new)
        {
            p->id = instances_local_id--;
            local_map_manager.insert({p, now_timestamp});
        }

        // 将已匹配 global 的 new instanc 合并进入 global
        MergeGlobalMatchedInstances(instances, global_map_instances, matchInfo);

        // 将已匹配 local  的 new instanc 添加进 global 删除 local 里面的对应
        MergeLocalMatchedInstances(unassignments_global_new, local_map_instances, local_match_info);

        // 检索 匹配合并了的 instanc 需要 match_info 因此删除 没匹配和并 的 instanc 防止重复重建 重复吸附到 global
        RemoveNoneMatchNoneMergeInstance(global_map_instances, matchInfo);
        RemoveNoneMatchNoneMergeInstance(local_map_instances, local_match_info);

        // 重建已合并的 global instanc 以更新
        RebuildInstances(global_map_instances);
        RebuildInstances(local_map_instances);
        std::cout << "rebuild global: " << global_map_instances.size() << std::endl;
        std::cout << "rebuild local : " << local_map_instances.size() << std::endl;

        // 当前新合并的 global instanc 之间 进行 归拢
        auto map_add_new = MergeSameInstancesToMap(global_map_instances, local_map_instances);

        // 根据当前位置 整顿整张地图 使用kdtree管理所有instances的 2d vertex 阈值 0.5 用于合并大型建筑和边缘重合实例 防止当前instances 和global 有重复
        // MergeGlobalMap(map_add_new);

        // 删除5s之前的临时Instances 该注册进去的删除也无妨已经被记录了
        DeleteFarAwayLocalMap();

        std::cout << "now global map size: " << global_map_manager.size() << std::endl;
        std::cout << "now local  map size: " << local_map_manager.size() << std::endl;
        std::cout << "##########################" << std::endl;
        return true;
    }
    // get local global instanc
    InstancesPtr GetLocalGlobalMap() const
    {
        InstancesPtr global_local_map;
        for (auto &p : global_map_manager)
        {
            global_local_map.push_back(p.first);
        }
        for (auto &p : local_map_manager)
        {
            global_local_map.push_back(p.first);
        }
        return global_local_map;
    }
    // get local instancs
    InstancesPtr GetLocalMap() const
    {
        InstancesPtr local_map;
        for (auto &p : local_map_manager)
        {
            local_map.push_back(p.first);
        }
        return local_map;
    }
    // get global instanc
    InstancesPtr GetGlobalMap() const
    {
        InstancesPtr global_map;
        for (auto &p : global_map_manager)
        {
            global_map.push_back(p.first);
        }
        return global_map;
    }
    

private:
    // initial the map
    bool Initialize(InstancesPtr &instances, double timestamp, InsMapTestOptions option)
    {
        // 记录当前位姿
        RecordPose(option.velodyne_trans, timestamp);
        // 将Instances转移到世界坐标系下
        TransformInstances(instances, option.velodyne_trans);
        // 初始化不需要匹配直接将Instances记录到Map中
        for (auto &p : instances)
        {
            std::cout << "\033[32m"
                      << "add new instance to map " << instances_map_id << "\033[0m" << std::endl;
            p->id = instances_map_id;
            global_map_manager.insert({p, instances_map_id});
            instances_map_id++;
        }
        return true;
    }

    // 获取所有 local instanc 这都是之前没和 global 匹配上的 匹配上的都删了 不用和 global 去重
    void CollectVisibleLocalInstances(InstancesPtr &local_instances)
    {
        for (auto &p : local_map_manager)
        {
            local_instances.push_back(p.first);
        }
    }

    // 获取当前帧可见的 global instanc kdtree 范围搜索 后面需要加遮挡判断等
    void CollectVisibleGlobalInstances(InstancesPtr &global_instances)
    {
        if (global_map_manager.empty())
            return;
        global_instances.clear();
        int inss_number = global_map_manager.size();
        InstancesPtr instances_map;
        instances_center_map->clear();
        for (auto &p : global_map_manager)
        {
            instances_map.push_back(p.first);
        }
        // map
        for (int i = 0; i < inss_number; ++i)
        {
            PointType tmpP;
            tmpP.x = instances_map[i]->center[0];
            tmpP.y = instances_map[i]->center[1];
            tmpP.z = instances_map[i]->center[2];
            instances_center_map->push_back(tmpP);
        }
        instances_center_map_kdtree.setInputCloud(instances_center_map);
        PointType nowPoint;
        nowPoint.x = now_posi(0);
        nowPoint.y = now_posi(1);
        nowPoint.z = now_posi(2);
        std::vector<int> inds;
        std::vector<float> dis2s;
        instances_center_map_kdtree.radiusSearch(nowPoint, map_config.search_radius, inds, dis2s);
        for (auto &p : inds)
        {
            global_instances.push_back(instances_map[p]);
        }
    }

    // 记录当前位姿
    void RecordPose(Eigen::Matrix4d &pose, double timestamp)
    {
        now_pose = pose;
        now_posi = pose.block<3, 1>(0, 3);
        now_timestamp = timestamp;
        trajectory.push_back(std::make_pair(timestamp, pose));
    }

    // 将Instances转到给定坐标下
    void TransformInstances(const std::vector<std::shared_ptr<Instance>> &instances, const Eigen::Matrix4d &pose)
    {
        int instances_number = instances.size();
        for (int i = 0; i < instances_number; ++i)
        {
            TransformInstance(instances[i], pose);
        }
    }

    // 将Instance转到给定坐标下
    void TransformInstance(InstancePtr ins, const Eigen::Matrix4d &pose)
    {
        // Transform instance with given pose
        Eigen::Vector3d &dir = ins->direction;
        dir = (pose * Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3);
        Eigen::Vector3d &center = ins->center;
        center = (pose * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
        Eigen::Vector3d &vertex1 = ins->vertex1;
        vertex1 = (pose * Eigen::Vector4d(vertex1(0), vertex1(1), vertex1(2), 1)).head(3);
        Eigen::Vector3d &vertex2 = ins->vertex2;
        vertex2 = (pose * Eigen::Vector4d(vertex2(0), vertex2(1), vertex2(2), 1)).head(3);
        Eigen::Vector3d &vertex3 = ins->vertex3;
        vertex3 = (pose * Eigen::Vector4d(vertex3(0), vertex3(1), vertex3(2), 1)).head(3);
        Eigen::Vector3d &vertex4 = ins->vertex4;
        vertex4 = (pose * Eigen::Vector4d(vertex4(0), vertex4(1), vertex4(2), 1)).head(3);
        TransformPointCloud(pose, *(ins->cloud));
        TransformPointCloud(pose, ins->polygon);
    }

    // 删除时间久远的local
    void DeleteFarAwayLocalMap()
    {
        if (local_map_manager.empty())
            return;
        int before_delete_number = local_map_manager.size();
        for (auto it = local_map_manager.begin(); it != local_map_manager.end();)
        {
            if (now_timestamp - it->second >= 5.0)
            {
                local_map_manager.erase(it++);
            }
            else
            {
                it++;
            }
        }
        int after_delete_number = local_map_manager.size();
        std::cout << "delete " << before_delete_number - after_delete_number << " nomatched" << std::endl;
    }

    // 将与 global 匹配的 new instanc 进行归拢
    bool MergeGlobalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &map_inss, std::vector<std::pair<int, int>> &assignments)
    {
        // 合并过程发生在两个Instance 之间 A->B A为new一定不在global中 B作为 global 必然在 只需要向 global 合并即可
        if (assignments.size() > 0)
        {
            int assignments_number = assignments.size();
            for (int i = 0; i < assignments_number; ++i)
            {
                std::cout << "global matched merge " << new_inss[assignments[i].first]->id << " in " << map_inss[assignments[i].second]->id << std::endl;
                mergeSource2Map(new_inss[assignments[i].first], map_inss[assignments[i].second]);
            }
        }
        return true;
    }

    // 将与 loca 匹配的 new instanc 进行添加
    bool MergeLocalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &local_inss, std::vector<std::pair<int, int>> &assignments)
    {
        // 合并过程发生在两个Instance 之间 A->B A为new一定不在global中 B local_global 必然不在
        if (assignments.size() > 0)
        {
            int assignments_number = assignments.size();
            for (int i = 0; i < assignments_number; ++i)
            {
                std::cout << "local matched merge " << new_inss[assignments[i].first]->id << " in " << local_inss[assignments[i].second]->id << std::endl;
                mergeSource2Map(new_inss[assignments[i].first], local_inss[assignments[i].second]);
                // 删除在 local 里的对应
                auto it_local  = local_map_manager.find(local_inss[assignments[i].second]);
                if( it_local != local_map_manager.end() ){
                    local_map_manager.erase(it_local);
                }
                else{
                    std::cout << "not in local you wen ti " << local_inss[assignments[i].second]->id << std::endl;
                    exit(0);
                }
                // 加入 global
                auto it_global = global_map_manager.find(local_inss[assignments[i].second]);
                if (it_global == global_map_manager.end())
                {
                    std::cout << "\033[32m"
                              << "add new instance to map " << instances_map_id << "\033[0m" << std::endl;
                    local_inss[assignments[i].second]->id = instances_map_id;
                    global_map_manager.insert({local_inss[assignments[i].second], instances_map_id});
                    instances_map_id++;
                }
                else{
                    std::cout << " already in global you wen ti " << (it_global->first)->id << std::endl;
                    exit(0);
                }
            }
        }
        return true;
    }

    // 重建更新点云后的Instances
    bool RebuildInstances(InstancesPtr &ins)
    {
        for (auto &p : ins)
        {
            instance_builder.BuildInstance(instance_builder_options, p);
        }
        return true;
    }

    // 归拢 Instances 中的重复的 Instance 针对输入进行处理
    bool MergeSameInstancesIndividual(InstancesPtr &inss)
    {
        if (inss.empty())
            return false;
        InstancesPtr inss_;
        inss_.swap(inss);
        int inss_number = inss_.size();
        std::vector<bool> merged_ins_ind(inss_number, false);
        for (int i = 0; i < inss_number; ++i)
        {
            if (merged_ins_ind[i])
                continue;
            for (int j = 0; j < inss_number; ++j)
            {
                if (i == j)
                    continue;
                if (merged_ins_ind[j])
                    continue;
                auto n_p = Get2InssCollisionVolumePercent(inss_[i], inss_[j]);
                if (n_p.second > 0.6)
                {
                    std::cout << "individual merge " << inss_[j]->id << " in " << inss_[i]->id << " " << std::endl;
                    mergeSource2Map(inss_[j], inss_[i]);
                    merged_ins_ind[j] = true;
                    instance_builder.BuildInstance(instance_builder_options, inss_[i]);
                    j = 0;
                }
            }
        }
        for (int i = 0; i < inss_number; ++i)
        {
            if (!merged_ins_ind[i])
            {
                inss.push_back(inss_[i]);
            }
        }
        std::cout << "input before gather up: " << inss_number << std::endl;
        std::cout << "input after  gather up: " << inss.size() << std::endl;
        return true;
    }

    // 归拢 Instances 中的重复的 Instance 并记录到地图 返回新注册的 instanc
    InstancesPtr MergeSameInstancesToMap(InstancesPtr &inss_map, InstancesPtr &inss_local)
    {
        std::cout << "update map  :";
        for(auto&p:inss_map){
            std::cout << p->id << " ";
        }
        std::cout << std::endl;

        std::cout << "update local:";
        for(auto&p:inss_local){
            std::cout << p->id << " ";
        }
        std::cout << std::endl;

        InstancesPtr inss, inss_;
        inss.resize(inss_map.size() + inss_local.size());
        std::merge(inss_map.begin(), inss_map.end(), inss_local.begin(), inss_local.end(), inss.begin());
        // 存在 global_local 中的 Instanc 如果只吸收了其他而没被吸收则最终保留 被吸收的需要去除 一个Instance只会被吸收一次 一个Instance可多次吸收其他或吸收或原始的Instance
        if (inss.empty())
            return inss_;
        inss_.swap(inss);
        int inss_number = inss_.size();
        std::vector<bool> merged_ins_ind(inss_number, false);
        for (int i = 0; i < inss_number; ++i)
        {
            if (merged_ins_ind[i])
                continue;
            for (int j = 0; j < inss_number; ++j)
            {
                if (i == j)
                    continue;
                if (merged_ins_ind[j])
                    continue;
                auto n_p = Get2InssCollisionVolumePercent(inss_[i], inss_[j]);
                if (n_p.second > 0.6)
                {
                    std::cout << "new merge " << inss_[j]->id << " in " << inss_[i]->id << std::endl;
                    mergeSource2Map(inss_[j], inss_[i]);
                    merged_ins_ind[j] = true;
                    instance_builder.BuildInstance(instance_builder_options, inss_[i]);
                    j = 0;
                }
            }
        }
        for (int i = 0; i < inss_number; ++i)
        {
            // 没被吸收的传出去
            if (!merged_ins_ind[i])
            {
                inss.push_back(inss_[i]);
            }
            // delete the merged ins from the global map
            // 被吸收的删掉
            if (merged_ins_ind[i])
            {
                // 整顿 add new 彼此之间
                auto it_global = global_map_manager.find(inss_[i]);
                if (it_global == global_map_manager.end())
                {
                    std::cout << " not in global you wen ti " << inss_[i]->id << std::endl;
                    exit(0);
                }
                std::cout << "\033[34m"
                          << "add delete instance from map " << (it_global->first)->id << "\033[0m" << std::endl;
                global_map_manager.erase(it_global);
            }
        }
        std::cout << "map before gather up: " << inss_number << std::endl;
        std::cout << "map after  gather up: " << inss.size() << std::endl;
        return inss;
    }

    // 根据新注册更新的 global 整顿 global 中的其他 待完成多棱柱后开搞
    bool MergeGlobalMap(InstancesPtr &inss)
    {
        // 合并所有有交集的实例 阈值 0.5 用于合并大范围实例 边缘实例等 global中无重复
        if (global_map_manager.empty())
            return false;
        instances_vertex_map->clear();
        InstancesPtr instances_map;
        for (auto &p : global_map_manager)
        {
            pcl::PointXY tmpPoint1, tmpPoint2, tmpPoint3, tmpPoint4;
            tmpPoint1.x = p.first->vertex1(0);
            tmpPoint1.y = p.first->vertex1(1);
            tmpPoint2.x = p.first->vertex2(0);
            tmpPoint2.y = p.first->vertex2(1);
            tmpPoint3.x = p.first->vertex3(0);
            tmpPoint3.y = p.first->vertex3(1);
            tmpPoint4.x = p.first->vertex4(0);
            tmpPoint4.y = p.first->vertex4(1);
            instances_vertex_map->push_back(tmpPoint1);
            instances_vertex_map->push_back(tmpPoint2);
            instances_vertex_map->push_back(tmpPoint3);
            instances_vertex_map->push_back(tmpPoint4);
            instances_map.push_back(p.first);
        }
        instances_vertex_map_kdtree.setInputCloud(instances_vertex_map);
        // 将当前Instance 归并到地图 global中
        for (auto &p : inss)
        {
            // 寻找半径内相交的点
            double search_radius = GetRectMaxRadius(p);
            pcl::PointXY nowPoint;
            nowPoint.x = p->center(0);
            nowPoint.y = p->center(1);
            std::vector<int> indexs;
            std::vector<float> distance2s;
            instances_vertex_map_kdtree.radiusSearch(nowPoint, search_radius, indexs, distance2s);
            // 获取当前instance 在 global 中能找到的碰撞的 instance
            std::set<int> now_friends;
            for (auto &e : indexs)
            {
                int ind = e / 4;
                // 同一指针地址跳过计算
                if (isSame(instances_map[ind], p))
                    continue;
                now_friends.insert(ind);
            }
            for (auto &e : now_friends)
            {
                // 计算当前instance 和相交的 global instance 的重复区域
                auto n_p = Get2InssCollisionVolumePercent(p, instances_map[e]);
                if (n_p.first > 0.5)
                {
                    std::cout << "map merge " << p->id << " in " << instances_map[e]->id << std::endl;
                    mergeSource2Map(p, instances_map[e]);
                    // 更新吸收此Instance 的分布
                    instance_builder.BuildInstance(instance_builder_options, instances_map[e]);
                    // 删除已经归并的Instance
                    auto it = global_map_manager.find(p);
                    if (it != global_map_manager.end())
                    {
                        // 整顿 add new 和 global
                        std::cout << "\033[34m"
                                  << "map delete instance from map " << (it->first)->id << "\033[0m" << std::endl;
                        global_map_manager.erase(it);
                    }
                    // 此Instance 已经被吸收 下一个
                    break;
                }
            }
        }
        return true;
    }

private:
    std::unordered_map<InstancePtr, double> local_map_manager;
    int instances_local_id = -1;

    std::unordered_map<InstancePtr, int> global_map_manager;
    int instances_map_id = 0;

    MinBoxInstanceBuilder instance_builder;
    InstanceBuilderOptions instance_builder_options;

    pcl::PointCloud<PointType>::Ptr instances_center_map;
    pcl::KdTreeFLANN<PointType> instances_center_map_kdtree;

    pcl::PointCloud<pcl::PointXY>::Ptr instances_vertex_map;
    pcl::KdTreeFLANN<pcl::PointXY> instances_vertex_map_kdtree;

    std::vector<std::pair<double, Eigen::Matrix4d>> trajectory;

    double now_timestamp;
    Eigen::Matrix4d now_pose;
    Eigen::Vector3d now_posi;

    MapConfig map_config;

    bool first_run = true;
};

#endif //_INS_MAP_H_

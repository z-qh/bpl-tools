#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "map/instance.h"
#include "map/hungarian_bigraph_matcher.h"

using namespace std;

typedef shared_ptr<Instance> InstancePtr;
typedef vector<shared_ptr<Instance>> InstancesPtr;

ros::Publisher marker_pub_box_0, marker_pub_box_1, marker_pub_box_2;
ros::Publisher cloud_pub_0, cloud_pub_1, cloud_pub_2;

double s_match_distance_maximum_ = 4.0f;

void ClearAllMarker(ros::Publisher &marker_pub_box_)
{
    visualization_msgs::MarkerArray::Ptr clear_marker_array(new visualization_msgs::MarkerArray);
    visualization_msgs::Marker dummy_marker;
    dummy_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker_array->markers.push_back(dummy_marker);
    marker_pub_box_.publish(clear_marker_array);
}

void pushLine(InstancePtr ins1, InstancePtr ins2, int &marker_id, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_strip;
    line_strip.pose.orientation.w = 1.0;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time().now();
    line_strip.ns = "match";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.points.resize(3);
    geometry_msgs::Point p1, p2;
    p1.x = ins1->center[0];
    p1.y = ins1->center[1];
    p1.z = (ins1->max_height + ins1->min_height) / 2;
    p2.x = ins2->center[0];
    p2.y = ins2->center[1];
    p2.z = (ins2->max_height + ins2->min_height) / 2;
    line_strip.id = marker_id;
    marker_id++;
    line_strip.points[0] = p1;
    line_strip.points[1] = p2;
    line_strip.points[2] = p1;
    marker_array_box.markers.push_back(line_strip);
}
void pushBBox(InstancePtr ins, int &marker_id, double color, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_strip;
    line_strip.pose.orientation.w = 1.0;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time().now();
    line_strip.ns = "match";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.color.r = 0.5;
    line_strip.color.g = color;
    line_strip.color.b = 0;
    line_strip.color.a = 1.0;
    line_strip.points.resize(5);
    {
        geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
        p1.x = p5.x = ins->vertex1[0];
        p1.y = p5.y = ins->vertex1[1];
        p1.z = ins->min_height;
        p5.z = ins->max_height;
        p2.x = p6.x = ins->vertex2[0];
        p2.y = p6.y = ins->vertex2[1];
        p2.z = ins->min_height;
        p6.z = ins->max_height;
        p3.x = p7.x = ins->vertex3[0];
        p3.y = p7.y = ins->vertex3[1];
        p3.z = ins->min_height;
        p7.z = ins->max_height;
        p4.x = p8.x = ins->vertex4[0];
        p4.y = p8.y = ins->vertex4[1];
        p4.z = ins->min_height;
        p8.z = ins->max_height;
        line_strip.id = marker_id;
        line_strip.points[0] = p1;
        line_strip.points[1] = p2;
        line_strip.points[2] = p4;
        line_strip.points[3] = p3;
        line_strip.points[4] = p1;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p5;
        line_strip.points[1] = p6;
        line_strip.points[2] = p8;
        line_strip.points[3] = p7;
        line_strip.points[4] = p5;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p1;
        line_strip.points[1] = p5;
        line_strip.points[2] = p7;
        line_strip.points[3] = p3;
        line_strip.points[4] = p1;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
        line_strip.id = marker_id;
        line_strip.points[0] = p2;
        line_strip.points[1] = p6;
        line_strip.points[2] = p8;
        line_strip.points[3] = p4;
        line_strip.points[4] = p2;
        marker_array_box.markers.push_back(line_strip);
        marker_id++;
    }
}

void pubMatchedBBoxMarker(InstancesPtr &inss1, InstancesPtr &inss2, vector<pair<int, int>> &assignments, ros::Publisher &marker_pub_box_)
{
    if (assignments.size() > 0)
    {
        ClearAllMarker(marker_pub_box_);
        visualization_msgs::MarkerArray marker_array_box;
        int marker_id = 0;
        int assign_number = assignments.size();
        for (int i = 0; i < assign_number; i++)
        {
            double color = i * 1.0 / assign_number;
            pushBBox(inss1[assignments[i].first], marker_id, color, marker_array_box);
            pushBBox(inss2[assignments[i].second], marker_id, color, marker_array_box);
            pushLine(inss1[assignments[i].first], inss2[assignments[i].second], marker_id, marker_array_box);
        }
        marker_pub_box_.publish(marker_array_box);
    }
}

void pubBBoxMarker(std::vector<std::shared_ptr<Instance>> &instances, ros::Publisher &marker_pub_box_)
{
    if (instances.size() > 0)
    {
        ClearAllMarker(marker_pub_box_);
        visualization_msgs::MarkerArray marker_array_box;
        int marker_id = 0;
        int instance_num = instances.size();
        for (int i = 0; i < instance_num; i++)
        {
            double color = i * 1.0 / instance_num;
            pushBBox(instances[i], marker_id, color, marker_array_box);
        }
        marker_pub_box_.publish(marker_array_box);
    }
}

void pubAssignmentedPoints(InstancesPtr &inss, ros::Publisher &pubCloud, vector<pair<int, int>> &assignments, string mod = "left")
{
    pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
    int assignment_number = assignments.size();
    for (int i = 0; i < assignment_number; ++i)
    {
        if (mod == "left")
            *tmpCloud += *(inss[assignments[i].first]->cloud);
        if (mod == "right")
            *tmpCloud += *(inss[assignments[i].second]->cloud);
    }
    tmpCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    tmpCloud->header.frame_id = "map";
    pubCloud.publish(tmpCloud);
}

void pubPoints(InstancesPtr &inss, ros::Publisher &pubCloud)
{
    pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>());
    for (auto&p:inss)
    {
        *tmpCloud += *(p->cloud);
    }
    tmpCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    tmpCloud->header.frame_id = "map";
    pubCloud.publish(tmpCloud);
}

//计算两个聚类距离
float ComputeDistance(InstancePtr ins1, InstancePtr ins2)
{
    // Compute distance for given ins1 & ins2
    float location_distance = (ins1->center - ins2->center).cast<float>().norm(); //距离差异

    float result_distance = location_distance; //各个差异*权值
    return result_distance;
}

//计算多个聚类之间的距离关系
void ComputeAssociateMatrix(InstancesPtr &inss1, InstancesPtr &inss2, Eigen::MatrixXf &association_mat)
{
    // Compute matrix of association distance
    for (size_t i = 0; i < inss1.size(); ++i)
    {
        for (size_t j = 0; j < inss2.size(); ++j)
        {
            association_mat(i, j) = ComputeDistance(inss1[i], inss2[j]);
        }
    }
}
// bfs based component analysis
void ConnectedComponentAnalysis(const std::vector<std::vector<int>> &graph, std::vector<std::vector<int>> *components)
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


//计算关联矩阵
void ComputeConnectedComponents(const Eigen::MatrixXf &association_mat, const float connected_threshold, std::vector<std::vector<int>> &ins1_components, std::vector<std::vector<int>> &ins2_components)
{
    // Compute connected components within given threshold
    int no_ins1 = association_mat.rows();
    int no_ins2 = association_mat.cols();
    std::vector<std::vector<int>> nb_graph;
    nb_graph.resize(no_ins1 + no_ins2);
    for (int i = 0; i < no_ins1; i++)
    {
        for (int j = 0; j < no_ins2; j++)
        {
            if (association_mat(i, j) <= connected_threshold)
            {
                nb_graph[i].push_back(no_ins1 + j);
                nb_graph[j + no_ins1].push_back(i);
            }
        }
    }

    std::vector<std::vector<int>> components;
    ConnectedComponentAnalysis(nb_graph, &components);
    ins1_components.clear();
    ins1_components.resize(components.size());
    ins2_components.clear();
    ins2_components.resize(components.size());
    for (size_t i = 0; i < components.size(); i++)
    {
        for (size_t j = 0; j < components[i].size(); j++)
        {
            int id = components[i][j];
            if (id < no_ins1)
            {
                ins1_components[i].push_back(id);
            }
            else
            {
                id -= no_ins1;
                ins2_components[i].push_back(id);
            }
        }
    }
}

void AssignInss2ToInss1(const Eigen::MatrixXf &association_mat,
                        const double assign_distance_maximum,
                        std::vector<std::pair<int, int>> &assignments,
                        std::vector<int> &unassigned_inss1,
                        std::vector<int> &unassigned_inss2)
{
    // Assign ins2 to ins1 with null setup
    std::vector<int> inss1_idx;
    std::vector<int> inss2_idx;
    int no_inss1 = association_mat.rows();
    int no_inss2 = association_mat.cols();
    // build cost
    std::vector<std::vector<double>> cost(no_inss1 + no_inss2);
    for (int i = 0; i < no_inss1; ++i)
    {
        cost[i].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            cost[i][j] = association_mat(i, j);
        }
    }
    for (int i = 0; i < no_inss2; ++i)
    {
        cost[i + no_inss1].resize(no_inss2);
        for (int j = 0; j < no_inss2; ++j)
        {
            if (j == i)
            {
                cost[i + no_inss1][j] = assign_distance_maximum * 1.2f;
            }
            else
            {
                cost[i + no_inss1][j] = 999999.0f;
            }
        }
    }

    HungarianOptimizer hungarian_optimizer(cost);
    hungarian_optimizer.minimize(&inss1_idx, &inss2_idx);

    int assignments_num = 0;
    std::vector<bool> inss1_used(no_inss1 + no_inss2, false);
    std::vector<bool> inss2_used(no_inss2, false);
    for (size_t i = 0; i < inss1_idx.size(); ++i)
    {
        if (inss1_idx[i] < 0 || inss1_idx[i] >= no_inss1 || inss2_idx[i] < 0 ||
            inss2_idx[i] >= no_inss2)
        {
            continue;
        }
        if (association_mat(inss1_idx[i], inss2_idx[i]) <
            assign_distance_maximum)
        {
            assignments[assignments_num++] = std::make_pair(inss1_idx[i], inss2_idx[i]);
            inss1_used[inss1_idx[i]] = true;
            inss2_used[inss2_idx[i]] = true;
        }
    }
    assignments.resize(assignments_num);
    unassigned_inss1.resize(association_mat.rows());
    int unassigned_inss1_num = 0;
    for (int i = 0; i < association_mat.rows(); ++i)
    {
        if (inss1_used[i] == false)
        {
            unassigned_inss1[unassigned_inss1_num++] = i;
        }
    }
    unassigned_inss1.resize(unassigned_inss1_num);
    unassigned_inss2.resize(association_mat.cols());
    int unassigned_inss2_num = 0;
    for (int i = 0; i < association_mat.cols(); ++i)
    {
        if (inss2_used[i] == false)
        {
            unassigned_inss2[unassigned_inss2_num++] = i;
        }
    }
    unassigned_inss2.resize(unassigned_inss2_num);
}

void MatchInComponents(const Eigen::MatrixXf &association_mat,
                       const std::vector<int> &inss1_component,
                       const std::vector<int> &inss2_component,
                       std::vector<std::pair<int, int>> &sub_assignments,
                       std::vector<int> &sub_unassigned_inss1,
                       std::vector<int> &sub_unassigned_inss2)
{
    sub_assignments.clear();
    sub_unassigned_inss1.clear();
    sub_unassigned_inss2.clear();
    // A. failed to match if either components is empty
    if (inss1_component.empty())
    {
        for (size_t i = 0; i < inss2_component.size(); ++i)
        {
            sub_unassigned_inss2.push_back(inss2_component[i]);
        }
    }
    if (inss2_component.empty())
    {
        for (size_t i = 0; i < inss1_component.size(); ++i)
        {
            sub_unassigned_inss1.push_back(inss1_component[i]);
        }
    }
    if (inss1_component.empty() || inss2_component.empty())
        return;
    // B. if components perfectly match
    if (inss1_component.size() == 1 && inss2_component.size() == 1)
    {
        int ins1_id = inss1_component[0];
        int ins2_id = inss2_component[0];
        if (association_mat(ins1_id, ins2_id) <= s_match_distance_maximum_)
        {
            sub_assignments.push_back(std::make_pair(ins1_id, ins2_id));
        }
        else
        {
            sub_unassigned_inss2.push_back(ins2_id);
            sub_unassigned_inss1.push_back(ins1_id);
        }
        return;
    }
    // C. multi instances match
    std::vector<int> inss1_local2global;
    std::vector<int> inss2_local2global;
    std::vector<std::pair<int, int>> local_assignments;
    std::vector<int> local_unassigned_inss1;
    std::vector<int> local_unassigned_inss2;
    Eigen::MatrixXf local_association_mat(inss1_component.size(), inss2_component.size());
    inss1_local2global.resize(inss1_component.size());
    inss2_local2global.resize(inss2_component.size());
    for (size_t i = 0; i < inss1_component.size(); ++i)
    {
        inss1_local2global[i] = inss1_component[i];
    }
    for (size_t i = 0; i < inss2_component.size(); ++i)
    {
        inss2_local2global[i] = inss2_component[i];
    }
    for (size_t i = 0; i < inss1_component.size(); ++i)
    {
        for (size_t j = 0; j < inss2_component.size(); ++j)
        {
            int ins1_id = inss1_component[i];
            int ins2_id = inss2_component[j];
            local_association_mat(i, j) = association_mat(ins1_id, ins2_id);
        }
    }
    local_assignments.resize(local_association_mat.cols());
    local_unassigned_inss1.assign(local_association_mat.rows(), -1);
    local_unassigned_inss2.assign(local_association_mat.cols(), -1);
    AssignInss2ToInss1(local_association_mat, s_match_distance_maximum_, local_assignments, local_unassigned_inss1, local_unassigned_inss2);
    for (size_t i = 0; i < local_assignments.size(); ++i)
    {
        int global_ins1_id = inss1_local2global[local_assignments[i].first];
        int global_ins2_id = inss2_local2global[local_assignments[i].second];
        sub_assignments.push_back(
            std::make_pair(global_ins1_id, global_ins2_id));
    }
    for (size_t i = 0; i < local_unassigned_inss1.size(); ++i)
    {
        int global_ins1_id = inss1_local2global[local_unassigned_inss1[i]];
        sub_unassigned_inss1.push_back(global_ins1_id);
    }
    for (size_t i = 0; i < local_unassigned_inss2.size(); ++i)
    {
        int global_ins2_id = inss2_local2global[local_unassigned_inss2[i]];
        sub_unassigned_inss2.push_back(global_ins2_id);
    }
}

std::vector<std::pair<int, int>> Match2Instances(InstancesPtr &inss1, InstancesPtr &inss2)
{
    std::vector<std::pair<int, int>> assignments;
    std::vector<int> unassigned_inss1;
    std::vector<int> unassigned_inss2;
    // A. computing association matrix
    Eigen::MatrixXf association_mat(inss1.size(), inss2.size());
    ComputeAssociateMatrix(inss1, inss2, association_mat);

    // B. computing connected components
    std::vector<std::vector<int>> inss1_components;
    std::vector<std::vector<int>> inss2_components;
    ComputeConnectedComponents(association_mat, s_match_distance_maximum_, inss1_components, inss2_components);

    // C. matching each sub-graph
    assignments.clear();
    unassigned_inss1.clear();
    unassigned_inss2.clear();
    for (size_t i = 0; i < inss1_components.size(); i++)
    {
        std::vector<std::pair<int, int>> sub_assignments;
        std::vector<int> sub_unassigned_inss1;
        std::vector<int> sub_unassigned_inss2;
        MatchInComponents(association_mat, inss1_components[i], inss2_components[i], sub_assignments, sub_unassigned_inss1, sub_unassigned_inss2);
        for (size_t j = 0; j < sub_assignments.size(); ++j)
        {
            int ins1_id = sub_assignments[j].first;
            int ins2_id = sub_assignments[j].second;
            assignments.push_back(sub_assignments[j]);
            float association_score = association_mat(ins1_id, ins2_id);
            // (*instances)[ins2_id]->association_score = association_score;
        }
        for (size_t j = 0; j < sub_unassigned_inss1.size(); ++j)
        {
            unassigned_inss1.push_back(sub_unassigned_inss1[j]);
        }
        for (size_t j = 0; j < sub_unassigned_inss2.size(); ++j)
        {
            unassigned_inss2.push_back(sub_unassigned_inss2[j]);
        }
    }

    for (auto &p : assignments)
    {
        cout << p.first << " " << p.second << " " << association_mat(p.first, p.second) << endl;
    }
    cout << " Total " << assignments.size() << " matchs!" << endl;
    return assignments;
}

//利用栅格管理点云密度，将Map_k转到L_k+1下，在L_k+1坐标系下将Map_k与P_k+1合并成Map_k+1
void mergeSource2Map(pcl::PointCloud<PointType>::Ptr map, pcl::PointCloud<PointType>::Ptr source)
{
    double lx = 0.01, ly = 0.01, lz = 0.01;
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D(*source, minpt, maxpt);

    std::map<int, std::pair<int, int>> dest_map; // key:voxel_i - val:map_i, source_i

    int width = std::ceil((maxpt(0) - minpt(0)) / lx);
    int len = std::ceil((maxpt(1) - minpt(1)) / ly);
    int hei = std::ceil((maxpt(2) - minpt(2)) / lz);
    int perh = width * len;
    for (size_t i = 0; i < map->size(); ++i)
    {
        auto &p = map->points[i];
        if (p.x < minpt(0) || p.y < minpt(1) || p.z < minpt(2) ||
            p.x > maxpt(0) || p.y > maxpt(1) || p.z > maxpt(2))
            continue;
        int ind_x = std::floor((p.x - minpt(0)) / lx);
        int ind_y = std::floor((p.y - minpt(1)) / ly);
        int ind_z = std::floor((p.z - minpt(2)) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        dest_map[ind] = std::make_pair(i, -1);
    }

    for (size_t i = 0; i < source->size(); ++i)
    {
        auto p = source->points[i];
        int ind_x = std::floor((p.x - minpt(0)) / lx);
        int ind_y = std::floor((p.y - minpt(1)) / ly);
        int ind_z = std::floor((p.z - minpt(2)) / lz);
        int ind = ind_x + ind_y * width + ind_z * perh;
        auto it = dest_map.find(ind);
        if (it == dest_map.end())
        {
            map->push_back(p);
            dest_map[ind] = std::make_pair(-1, i);
        }
    }
}

bool Merge2Instances(InstancesPtr &inss1, InstancesPtr &inss2, std::vector<std::pair<int, int>> &assignments)
{
    if (assignments.size() > 0)
    {
        int assignments_number = assignments.size();
        for (int i = 0; i < assignments_number; ++i)
        {
            mergeSource2Map(inss1[assignments[i].first]->cloud, inss2[assignments[i].second]->cloud);
        }
    }
    return true;
}

void deleteUnassignedInstance(InstancesPtr &inss1, InstancesPtr &inss2, std::vector<std::pair<int, int>> &assignments)
{
    std::vector<std::pair<int, int>> assignments_;
    InstancesPtr inss1_, inss2_;
    assignments_.swap(assignments);
    inss1_.swap(inss1);
    inss2_.swap(inss2);
    int assignment_number = assignments_.size();
    for (int i = 0; i < assignment_number; ++i)
    {
        inss1.push_back(inss1_[assignments_[i].first]);
        inss2.push_back(inss2_[assignments_[i].second]);
        assignments.push_back(make_pair(i, i));
    }
    inss1_.clear();
    inss2_.clear();
    assignments_.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mot");
    ros::NodeHandle nh;

    cloud_pub_0 = nh.advertise<pcl::PointCloud<PointType>>("/cloud_0", 1);
    cloud_pub_1 = nh.advertise<pcl::PointCloud<PointType>>("/cloud_1", 1);
    cloud_pub_2 = nh.advertise<pcl::PointCloud<PointType>>("/cloud_2", 1);

    marker_pub_box_0 = nh.advertise<visualization_msgs::MarkerArray>("/marker_box_0", 1);
    marker_pub_box_1 = nh.advertise<visualization_msgs::MarkerArray>("/marker_box_1", 1);
    marker_pub_box_2 = nh.advertise<visualization_msgs::MarkerArray>("/marker_box_2", 1);

    vector<shared_ptr<Instance>> instances1, instances2, instances0;
    string d0 = "/home/qh/temp/0/";
    string d1 = "/home/qh/temp/1/";
    string d2 = "/home/qh/temp/2/";

    LoadInstanaces(instances0, d0);
    LoadInstanaces(instances1, d1);
    LoadInstanaces(instances2, d2);

    auto matchInfo = Match2Instances(instances0, instances2);

    deleteUnassignedInstance(instances0, instances2, matchInfo);
    
    cout << matchInfo.size() << endl;
    Merge2Instances(instances0, instances2, matchInfo);

    ros::Rate loop(3);
    while (ros::ok())
    {
        pubMatchedBBoxMarker(instances0, instances2, matchInfo, marker_pub_box_0);
    
        pubPoints(instances0, cloud_pub_0);
        pubPoints(instances2, cloud_pub_2);

        // pubBBoxMarker(instances2, marker_pub_box_2);
        loop.sleep();
    }

    return 0;
}
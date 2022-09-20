#include "ins_map.h"



// 基础保存
bool SaveInstance(std::shared_ptr<Instance> ins, std::string &saveFilePathDir)
{
    std::stringstream ss;
    ss << std::fixed << ins->id;
    if (saveFilePathDir.back() != '/')
        saveFilePathDir += "/";
    std::string saveFileName = saveFilePathDir + ss.str() + ".ins";
    // if(0 == access(saveFileName.c_str(), 0)) return true;
    std::ofstream file(saveFileName);
    if (!file.good())
        return false;
    else
    {
        file << std::fixed << std::setprecision(5) <<
            // id
            ins->id << std::endl
             <<
            // minheight maxheigh heigh
            ins->min_height << " " << ins->max_height << " " << ins->height << std::endl
             <<
            // center
            ins->center[0] << " " << ins->center[1] << " " << ins->center[2] << std::endl
             <<
            // rect bound
            ins->min_pt(0) << " " << ins->min_pt(1) << " " << ins->min_pt(2) << std::endl
             << ins->max_pt(0) << " " << ins->max_pt(1) << " " << ins->max_pt(2) << std::endl
             <<
            // shape cloud
            ins->cloud_2dshape->size() << std::endl;
        for (auto &p : ins->cloud_2dshape->points)
        {
            file << p.x << " " << p.y << " " << p.z << std::endl;
        }
        file << ins->cloud->size() << std::endl;
        for (auto &p : ins->cloud->points)
        {
            file << p.x << " " << p.y << " " << p.z << " " << p.intensity << " " << p.label << std::endl;
        }
    }
    file.close();
    return true;
}
// 基础加载
std::shared_ptr<Instance> LoadInstanace(std::string &loadFileName)
{
    std::ifstream file(loadFileName);
    std::shared_ptr<Instance> ins(new Instance);
    if (!file.good())
    {
        printf("\033[31m%s file not exits\033[0m\n", loadFileName.c_str());
        return nullptr;
    }
    else
    {
        int cloud_size, cloud_2d_size;
        file >>
            // id
            ins->id >>
            // height
            ins->min_height >> ins->max_height >> ins->height >>
            // center
            ins->center[0] >> ins->center[1] >> ins->center[2] >>
            // rect bound
            ins->min_pt(0) >> ins->min_pt(1) >> ins->min_pt(2) >>
            ins->max_pt(0) >> ins->max_pt(1) >> ins->max_pt(2) >>
            // point cloud
            cloud_2d_size;
        pcl::PointCloud<PointType>::Ptr cloud_2d(new pcl::PointCloud<PointType>());
        for (int i = 0; i < cloud_2d_size; ++i)
        {
            PointType p;
            file >> p.x >> p.y >> p.z;
            cloud_2d->push_back(p);
        }
        file >> cloud_size;
        pcl::PointCloud<PointType>::Ptr cloud_3d(new pcl::PointCloud<PointType>());
        for (int i = 0; i < cloud_size; ++i)
        {
            PointType p;
            file >> p.x >> p.y >> p.z >> p.intensity >> p.label;
            cloud_3d->push_back(p);
        }
        ins->cloud = cloud_3d;
        ins->cloud_2dshape = cloud_2d;
        ins->update = true;
    }
    file.close();
    if (!ins->init())
        return nullptr;
    return ins;
}
// 保存
bool SaveInstances(std::vector<std::shared_ptr<Instance>> &instances, std::string &saveFilePathDir)
{
    for (auto &p : instances)
    {
        if (!SaveInstance(p, saveFilePathDir))
        {
            return false;
        }
    }
    printf("Save %d Instances!\n", instances.size());
    return true;
}

// 目录 后缀
std::vector<std::string> GetFiles(const char *src_dir, const char *ext)
{
    std::vector<std::string> result;
    std::string directory(src_dir);
    std::string m_ext(ext);

    // 目录句柄
    DIR *dir = opendir(src_dir);
    if (dir == NULL)
    {
        printf("%s is not a directory or not exist!\n", directory.c_str());
        exit(0);
    }

    // dirent存储文件的各种属性
    struct dirent *d_ent = NULL;

    // linux 去掉 "."和".."
    char dot[3] = ".";
    char dotdot[6] = "..";

    // 一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
    while ((d_ent = readdir(dir)) != NULL)
    {
        // 忽略 "." 和 ".."
        if ((strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0))
        {
            // d_type可以看到当前的东西的类型 DT_DIR代表当前都到的是目录 usr/include/dirent.h中定义
            if (d_ent->d_type != DT_DIR)
            {
                std::string d_name(d_ent->d_name);
                if (strcmp(d_name.c_str() + d_name.length() - m_ext.length(), m_ext.c_str()) == 0)
                {
                    // 绝对路径
                    std::string absolutePath = directory + std::string("/") + std::string(d_ent->d_name);
                    if (directory[directory.length() - 1] == '/')
                        absolutePath = directory + std::string(d_ent->d_name);
                    result.push_back(absolutePath);
                }
            }
        }
    }
    // 排序
    sort(result.begin(), result.end());
    closedir(dir);
    return result;
}
// 加载
int LoadInstanaces(std::vector<std::shared_ptr<Instance>> &instances, std::string &loadFilePathDir)
{
    instances.clear();
    int id = 0;
    auto loadFileNames = GetFiles(loadFilePathDir.c_str(), ".ins");
    for (auto &p : loadFileNames)
    {
        auto pttr = LoadInstanace(p);
        if (pttr != nullptr)
            instances.push_back(pttr);
    }
    printf("Load %d Instances!\n", instances.size());
    return instances.size();
}


// 变换点云
void TransformPointCloud(const Eigen::Matrix4f &trans_mat, pcl::PointCloud<PointType> &cloud_in_out)
{
    for (std::size_t i = 0; i < cloud_in_out.size(); ++i)
    {
        PointType &p = cloud_in_out.points[i];
        Eigen::Vector4f v(p.x, p.y, p.z, 1);
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

// 计算两个凸包相交区域占比
std::pair<float, float> Get2InssCollisionVolumePercent(const InstancePtr &ins_a, const InstancePtr &ins_b)
{
    std::vector<PointType> ins_a_ver, ins_b_ver;
    for (auto &p : ins_a->cloud_2dshape->points)
        ins_a_ver.push_back(p);
    for (auto &p : ins_b->cloud_2dshape->points)
        ins_b_ver.push_back(p);
    if (gjk(ins_a_ver, ins_b_ver))
    {
        float coll_area = polygon_collision_area(ins_a_ver, ins_b_ver);
        if ( (coll_area-ins_a->area) > precision || (coll_area-ins_b->area) > precision)
        {
            printf("\033[31merror here 501 %d %d\033[0m\n", ins_a->id, ins_b->id);
            std::string saveDir = "/home/qh/ins_map_temp";
            SaveInstance(ins_a, saveDir);
            SaveInstance(ins_b, saveDir);
            exit(0);
        }
        return std::pair<float, float>(coll_area / ins_a->area, coll_area / ins_b->area);
    }

    return std::pair<float, float>(0, 0);
}
// 计算两个聚类距离
float ComputeDistance(const InstancePtr new_ins, const InstancePtr map_ins)
{
    // Compute distance for given new_ins & map_ins
    // float location_distance = (new_ins->center - map_ins->center).norm(); //距离差异

    pcl::PointCloud<PointType>::Ptr total2dshapes(new pcl::PointCloud<PointType>());
    *total2dshapes = *new_ins->cloud_2dshape + *map_ins->cloud_2dshape;
    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    convex_hull.setInputCloud(total2dshapes);
    pcl::PolygonMesh mesh_result;
    convex_hull.reconstruct(mesh_result);
    float S_total = convex_hull.getTotalArea();
    
    auto S_overlap_rate = Get2InssCollisionVolumePercent(new_ins, map_ins);
    float S_overlap = S_overlap_rate.second * map_ins->area;

    float result_distance = sqrt(S_total / ( new_ins->area + map_ins->area - S_overlap ));
    result_distance = (new_ins->center - map_ins->center).norm();
    return result_distance;
}
// 计算多个聚类之间的距离关系
void ComputeAssociateMatrix(const InstancesPtr &new_inss, const InstancesPtr &map_inss, Eigen::MatrixXf &association_mat)
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
                            const float assign_distance_maximum,
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
    std::vector<std::vector<float>> cost(no_new_inss + no_map_inss);
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
                       float s_match_distance_maximum_)
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
// 检索 匹配合并了的 instanc 需要 match_info 因此删除 没匹配和并 的 instanc 防止重复重建 重复吸附到 global 删除match信息
bool RemoveNoneMatchNoneMergeInstance(InstancesPtr &inss, std::vector<std::pair<int, int>> &match)
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
        if (!inss_[p.second]->update)
            continue;
        inss.push_back(inss_[p.second]);
    }
    match.clear();
    return true;
}
// 合并点云
void mergeSource2Map(InstancePtr new_ins, InstancePtr map_ins)
{
    pcl::PointCloud<PointType>::Ptr &source = new_ins->cloud;
    pcl::PointCloud<PointType>::Ptr &map = map_ins->cloud;
    pcl::PointCloud<PointType>::Ptr merged(new pcl::PointCloud<PointType>());
    *merged += *source;
    *merged += *map;
    pcl::VoxelGrid<PointType> donw_sample_filter;
    donw_sample_filter.setLeafSize(.2f, .2f, .2f);
    donw_sample_filter.setInputCloud(merged);
    donw_sample_filter.filter(*map);
    map_ins->update = true;
}
// 利用栅格基于点云密度合并Instance，将Map_k转到L_k+1下，在L_k+1坐标系下将Map_k与P_k+1合并成Map_k+1
void mergeSource2Map(InstancePtr new_ins, InstancePtr map_ins, bool no_downsample)
{
    pcl::PointCloud<PointType>::Ptr &source = new_ins->cloud;
    pcl::PointCloud<PointType>::Ptr &map = map_ins->cloud;
    int map_origin_points_size = map->size();
    float lx = 0.5, ly = 0.5, lz = 0.5;
    Eigen::Vector4f minpt, maxpt;
    Eigen::Vector3f dummy_center;
    std::map<int, std::pair<int, int>> dest_map; // key:voxel_i - val:map_i, source_i

    minpt = new_ins->min_pt;
    maxpt = new_ins->max_pt;
    dummy_center = new_ins->center;

    minpt(0) -= dummy_center(0);
    minpt(1) -= dummy_center(1);
    minpt(2) -= dummy_center(2);
    maxpt(0) -= dummy_center(0);
    maxpt(1) -= dummy_center(1);
    maxpt(2) -= dummy_center(2);
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
// 同一块内存的共享指针
bool isSamePtr(InstancePtr &a, InstancePtr &b)
{
    return !(a.owner_before(b) || b.owner_before(a));
}
// 将Instance点云转到给定坐标下
void TransformInstance(InstancePtr ins, const Eigen::Matrix4f &pose)
{
    // Transform instance with given pose
    TransformPointCloud(pose, *(ins->cloud));
    ins->init();
}
// 将Instances转到给定坐标下
void TransformInstances(const std::vector<std::shared_ptr<Instance>> &instances, const Eigen::Matrix4f &pose)
{
    int instances_number = instances.size();
    for (int i = 0; i < instances_number; ++i)
    {
        TransformInstance(instances[i], pose);
    }
}
// 深度搜索连接
void dfs_connect(Eigen::MatrixXf &mat, int id, std::vector<bool> &visited, std::vector<int> &connect)
{
    visited[id] = true;
    for (size_t i = id + 1; i < mat.cols(); ++i)
    {
        if (mat(id, i) != 0 && !visited[i])
        {
            connect.push_back(i);
            dfs_connect(mat, i, visited, connect);
        }
    }
}
// 初始化
bool InsMapTest::Initialize(InstancesPtr &instances, float timestamp, InsMapTestOptions option)
{
    // 记录当前位姿
    RecordPose(option.velodyne_trans, timestamp);
    // 初始化不需要匹配直接将Instances记录到Map中
    for (auto &p : instances)
    {   
        // printf("\033[32m add new instance to local map %d \033[0m\n", instances_map_id);
        p->id = instances_local_id;
        local_map_manager.insert({p, timestamp});
        instances_local_id++;
    }
    return true;
}

// merge new instanc into maps
/*修改之前的流程*************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 * 1.MergeSameInstancesIndividual 将输入的聚类的实例剪枝 主要是避免较差的聚类的不稳定性 聚类较差时候两个类别之间甚至有重叠
 * 2.Initialize 将第一帧的实例直接加入到全局地图中去 后续根据第一帧地图作为参考进行建图
 * 3.RecordPose 记录当前位姿 旧流程中位姿仅用于提供先验搜索范围 
 * 4.CollectVisibleGlobalInstances 获取当前位姿视野内(100M)的全局已注册实例(中心点查找) 根据中心点是一种粗略查找方式 实例体积有可能很大
 * 5.CollectVisibleLocalInstances 获取当前位姿视野内(100M)的临时未注册实例(中心店查找) 这里算是一种半精确查找 未能获取匹配的临时实例体积通常较小
 * 6.Match2Instances 将新一帧的实例与全局已经注册实例进行数据关联 数据关联方式就是二分图匹配用匈牙利算法 无向图的权重是实例中心点距离 这里匹配方法是一种传统的粗略的数据关联方式
 * 7.Match2Instances 将新一帧的未能与全局已注册实例进行匹配的实例与临时未注册实例匹配 数据关联方式同上
 * 8.UnassignedInstance 未能与全局已注册实例匹配 也未能与临时实例匹配的新实例 作为临时实例加入到临时地图中去等待检索
 * 9.MergeGlobalMatchedInstances 将与全局已注册实例匹配的新实例合并进全局已注册实例中 删除新实例 合并方式是点云合并+下采样+重建包围框
 * 10.MergeLocalMatchedInstances  将与局部喂注册实例匹配的新实例合并进临时实例中 删除临时地图中的对应临时实例 将合并后的实例注册到全局中 合并方式同上
 * 11.RemoveNoneMatchNoneMergeInstance 删除已经新一帧的合并信息以防止后续步骤误用
 * 12.RebuildInstances 对已经注册进全局的实例 对发生合并的全局实例 重新生成包围框
 * 13.MergeSameInstancesToMap + RebuildInstances 已经注册进全局的实例 发生合并的全局实例 在一起进行碰撞检测并合并 用于地图的补充与连接 合并后更新包围框
 * 14.MergeGlobalMap 在大范围内(300M)连接地图 检查范围中全局已注册实例的碰撞与重叠情况 合并这些实例并更新 根据碰撞检测矩阵计算连通图并进行合并更新
 * 15.DeleteFarAwayLocalMap 删除时间久远的临时地图
 * 整个过程中有许多可以改进的地方 如权重计算的方式 数据关联的方式 地图范围统计的方式 新一帧进行数据关联后的跟踪维护方式 点云合并的方法等
 **************************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 */

// 因为BA过程的加入，整个流程都要改，改之前的流程在selo哪里，改之后的流程下面大致讨论一下，主要加入维护局部地图的部分，根据匹配的情况和可见性来适时进行BA
/*修改之后的流程*************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 * 局部地图是一个多实例的集合 维护各个实例匹配的信息 建立一个无实例的包围框地图用于检测合并 不更改实例信息以完成BA 满足一定条件发生BA并将可靠的实例注册到全局地图中
 * 实例需要有存活时间 即在局部地区中连续观测到的时间 每一次匹配如果有这个实例的匹配信息 则他的可观测次数自增 每一次匹配所有局部地图中的实例存活时间自增
 * 发生BA的时间 应该是第固定几帧发生一次BA 位姿变化大也发生BA
 * 1.MergeSameInstancesIndividual 将输入的聚类的实例剪枝 主要是避免较差的聚类的不稳定性 聚类较差时候两个类别之间甚至有重叠
 * 2.Initialize 将第一帧的实例直接加入到局部地图中去 后续根据局部地图作为参考进行建图 
 * 3.RecordPose 记录当前位姿 也得记录每一个实例对应的帧 或者记录每一个实例对应的位姿
 * X.获取全局地图中的附近点 记录到局部地图中去
 * 4.Match2Instances 将新一帧的实例与局部地图实例进行数据关联 分两类 一类是重叠面积较小 一类是重叠面积较大 用数据关联成功的新实例更新局部地图包围框 
 * 5.Match2Instances 将新一帧的未能与局部地图实例进行匹配的实例与临时实例匹配 同样分为两类 数据关联方式同上 用数据关联成功的实例更新局部地图包围框
 * 6.计算第一类匹配中的BA信息并记录 每20帧进行一次BA 转角超过90°发生BA 距离超过200M发生BA
 * 7.维护局部地图中的实例 记录可视时长 生存时长 可观测占比 优化位姿 优化误差等
 * 8.BA优化 之前记录了N个重叠较高的实例匹配对 知道每个实例的位姿 根据这些信息进行BA 将匹配的这些实例不合并地加入到全局地图中去
 * 
 * 
 * 
 **************************************************************************************************************************************************************
 **************************************************************************************************************************************************************
 */
bool InsMapTest::Merge(InstancesPtr &instances, float timestamp, InsMapTestOptions option)
{
    // 初始化
    TicToc merge_individual;
    MergeSameInstancesIndividual(instances);
    if(first_run){
        first_run = false;
        return Initialize(instances, timestamp, option);
        printf("First Run########################\n");
    }

    // 记录当前帧位姿
    RecordPose(option.velodyne_trans, timestamp);

    // 获取全局地图先验地图 
    TicToc search_map;
    InstancesPtr global_map_instances, local_map_instances, temp_map_instances;
    CollectVisibleGlobalInstances(global_map_instances);
    CollectVisibleLocalInstances(local_map_instances);
    CollectVisibleTempInstances(temp_map_instances);

    // 将新实例与局部地图匹配 未能与局部地图匹配的与临时地图匹配 未能匹配的加入临时地图
    auto local_match_info = Match2Instances(instances, local_map_instances, map_config.match_distance_maximum);
    auto local_unassignments = UnassignedInstance(instances, local_match_info);
    auto temp_match_info = Match2Instances(local_unassignments, temp_map_instances, map_config.match_distance_maximum);
    auto unassigments = UnassignedInstance(local_unassignments, temp_match_info);
    for(auto&&p : unassigments){
        temp_map_manager.insert({p, now_timestamp});
    }

    // 维护记录匹配情况







}

// get local global instanc
InstancesPtr InsMapTest::GetLocalGlobalMap() const
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
InstancesPtr InsMapTest::GetLocalMap() const
{
    InstancesPtr local_map;
    for (auto &p : local_map_manager)
    {
        local_map.push_back(p.first);
    }
    return local_map;
}
// get global instanc
InstancesPtr InsMapTest::GetGlobalMap() const
{
    InstancesPtr global_map;
    for (auto &p : global_map_manager)
    {
        global_map.push_back(p.first);
    }
    return global_map;
}



// 获取所有 local instanc
void InsMapTest::CollectVisibleLocalInstances(InstancesPtr &local_instances)
{
    for (auto &p : local_map_manager)
    {
        local_instances.push_back(p.first);
    }
}

// 获取所有 temp instanc
void InsMapTest::CollectVisibleTempInstances(InstancesPtr &temp_instances)
{
    for(auto & p: temp_map_manager){
        temp_instances.push_back(p.first);
    }
}

// 获取当前帧一定范围内的 global instanc 使用 kdtree 范围搜索 后面需要加遮挡判断等
void InsMapTest::CollectVisibleGlobalInstances(InstancesPtr &global_instances)
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
    instances_center_map_kdtree.radiusSearch(nowPoint, map_config.visible_search_radius, inds, dis2s);
    for (auto &p : inds)
    {
        global_instances.push_back(instances_map[p]);
    }
}

// 记录当前位姿
void InsMapTest::RecordPose(Eigen::Matrix4f &pose, float timestamp)
{
    now_pose = pose;
    now_posi = pose.block<3, 1>(0, 3);
    now_timestamp = timestamp;
    trajectory.push_back(std::make_pair(timestamp, pose));
}


// 匹配两簇 Instances
std::vector<std::pair<int, int>> InsMapTest::Match2Instances(const InstancesPtr &new_inss,const InstancesPtr &map_inss, float s_match_distance_maximum_)
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

// 删除时间久远的local
void InsMapTest::DeleteFarAwayLocalMap()
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
    // printf("delete %d nomatched\n", before_delete_number - after_delete_number);
}

// 将与 global 匹配的 new instanc 进行归拢
bool InsMapTest::MergeGlobalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &map_inss, std::vector<std::pair<int, int>> &assignments)
{
    // 合并过程发生在两个Instance 之间 A->B A为new一定不在global中 B作为 global 必然在 只需要向 global 合并即可
    if ( !assignments.empty() )
    {
        int assignments_number = assignments.size();
        for (int i = 0; i < assignments_number; ++i)
        {
            // printf("global matched merge %d in %d\n", new_inss[assignments[i].first]->id, map_inss[assignments[i].second]->id);
            mergeSource2Map(new_inss[assignments[i].first], map_inss[assignments[i].second]);
            if (map_inss[assignments[i].second]->update)
            {
                if (!map_inss[assignments[i].second]->init())
                {
                    printf("\033[31merror here 815\033[0m\n");
                    auto it_global = global_map_manager.find(map_inss[assignments[i].second]);
                    if(it_global != global_map_manager.end())
                    {
                        global_map_manager.erase(it_global);
                    }
                    else{
                        printf("\033[31merror here 822\033[0m\n");
                        exit(0);
                    }

                }
            }
        }
    }
    return true;
}
// 将与 loca 匹配的 new instanc 进行添加
bool InsMapTest::MergeLocalMatchedInstances(InstancesPtr &new_inss, InstancesPtr &local_inss, std::vector<std::pair<int, int>> &assignments)
{
    // 合并过程发生在两个Instance 之间 A->B A为new一定不在global中 B local_global 必然不在
    if ( ! assignments.empty() )
    {
        int assignments_number = assignments.size();
        for (int i = 0; i < assignments_number; ++i)
        {
            // printf("local matched merge %d in %d\n", new_inss[assignments[i].first]->id, local_inss[assignments[i].second]->id);
            mergeSource2Map(new_inss[assignments[i].first], local_inss[assignments[i].second]);
            bool shape_success = true;
            if (local_inss[assignments[i].second]->update)
            {
                if (!local_inss[assignments[i].second]->init())
                {
                    printf("\033[31merror here 849\033[0m\n");
                    shape_success = false;
                }
            }
            local_inss[assignments[i].second]->init();
            // 删除在 local 里的对应
            auto it_local = local_map_manager.find(local_inss[assignments[i].second]);
            if (it_local != local_map_manager.end())
            {
                local_map_manager.erase(it_local);
            }
            else
            {
                printf("\033[31mnot in local you wen ti %d\033[0m\n", local_inss[assignments[i].second]->id);
                exit(0);
            }
            // 加入 global
            if(!shape_success) 
            {
                printf("\033[31mcause error abandon\033[0m\n");
                continue;
            }
            auto it_global = global_map_manager.find(local_inss[assignments[i].second]);
            if (it_global == global_map_manager.end())
            {
                // printf("\033[32madd new instance to map %d\033[0m\n", instances_map_id);
                local_inss[assignments[i].second]->id = instances_map_id;
                global_map_manager.insert({local_inss[assignments[i].second], instances_map_id});
                instances_map_id++;
            }
            else
            {
                printf("\033[31malready in global you wen ti %d\033[0m\n", (it_global->first)->id);
                exit(0);
            }
        }
    }
    return true;
}
// 重建更新点云后的Instances
bool InsMapTest::RebuildInstances(InstancesPtr &ins)
{
    for (auto &p : ins)
    {
        if (p->update)
        {
            p->init();
        }
    }
    return true;
}
// 归拢 Instances 中的重复的 Instance 针对输入进行处理
bool InsMapTest::MergeSameInstancesIndividual(InstancesPtr &inss)
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
            if (n_p.second > 0)
            {
                // printf("individual merge %d in %d\n", inss_[j]->id, inss_[i]->id);
                mergeSource2Map(inss_[j], inss_[i]);
                merged_ins_ind[j] = true;
                if (inss_[i]->update)
                {
                    if (!inss_[i]->init())
                    {
                        merged_ins_ind[i] = true;
                        break;
                    }
                }
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
    // printf("input before gather up:  %d\n", inss_number);
    // printf("input after  gather up:  %d\n", inss.size());
    return true;
}
// 归拢 Instances 中的重复的 Instance 并记录到地图 返回新注册的 instanc
InstancesPtr InsMapTest::MergeSameInstancesToMap(InstancesPtr &inss_map, InstancesPtr &inss_local)
{
    // printf("update map: ");
    // for (auto &p : inss_map)
    // {
    //     printf("%d ", p->id);
    // }
    // printf("\n");
    // printf("update local: ");
    // for (auto &p : inss_local)
    // {
    //     printf("%d ", p->id);
    // }
    // printf("\n");

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
            if (n_p.second > .2f)
            {
                // printf("new merge %d in %d\n", inss_[j]->id, inss_[i]->id);
                mergeSource2Map(inss_[j], inss_[i]);
                merged_ins_ind[j] = true;
                if (inss_[i]->update)
                {
                    if (!inss_[i]->init())
                    {
                        merged_ins_ind[i] = true;
                        break;
                    }
                }
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
                printf("\033[31mnot in global you wen ti %d\033[0m\n", inss_[i]->id);
                exit(0);
            }
            // printf("\033[34madd delete instance from map %d\033[0m\n", (it_global->first)->id);
            global_map_manager.erase(it_global);
        }
    }
    // printf("map before gather up:  %d\n", inss_number);
    // printf("map after  gather up:  %d\n", inss.size());
    return inss;
}
// 根据新注册更新的 global 整顿 global 中的其他 待完成多棱柱后开搞
bool InsMapTest::MergeGlobalMap(InstancesPtr &new_reg_inss)
{
    if (global_map_manager.empty())
        return true;
    // 首先将一个较大范围内的Instanc的 2d shape points添加进去
    InstancesPtr around_instances;
    for (auto &p : global_map_manager)
    {
        float dis = (now_posi - p.first->center).norm();
        if (dis < map_config.merge_map_search_radius)
        {
            PointType this_p;
            this_p.x = p.first->center(0);
            this_p.y = p.first->center(1);
            this_p.z = p.first->center(2);
            this_p.intensity = around_instances.size();
            around_instances.push_back(p.first);
        }
    }
    size_t vertices_size = around_instances.size();
    // 计算碰撞的无向图邻接矩阵和连通图 对称阵 两个相容顶点无连接
    Eigen::MatrixXf adjacency_matrix = Eigen::MatrixXf::Zero(vertices_size, vertices_size);
    for (size_t i = 0; i < vertices_size; ++i)
    {
        for (size_t j = i + 1; j < vertices_size; ++j)
        {
            std::vector<PointType> ins_a_ver, ins_b_ver;
            for (auto &p : around_instances[i]->cloud_2dshape->points)
                ins_a_ver.push_back(p);
            for (auto &p : around_instances[j]->cloud_2dshape->points)
                ins_b_ver.push_back(p);
            adjacency_matrix(i, j) = gjk(ins_a_ver, ins_b_ver);
        }
    }
    // 计算连通图
    std::vector<bool> visited(vertices_size, false);
    std::vector<std::vector<int>> connected;
    for (size_t i = 0; i < vertices_size; ++i)
    {
        std::vector<int> this_connect;
        if (!visited[i])
        {
            this_connect.push_back(i);
            dfs_connect(adjacency_matrix, i, visited, this_connect);
        }
        if (this_connect.size() > 1)
        {
            connected.push_back(this_connect);
        }
    }
    // 根据连通图归并并统计被吸收的instanc
    std::vector<bool> been_merged_inss(vertices_size, false);
    for (auto &this_connect : connected)
    {
        for (size_t i = 1; i < this_connect.size(); ++i)
        {
            // printf("map merge %d in %d\n", around_instances[this_connect[i]]->id, around_instances[this_connect.front()]->id);
            mergeSource2Map(around_instances[this_connect[i]], around_instances[this_connect.front()]);
            if (around_instances[this_connect.front()]->update)
            {
                if (!around_instances[this_connect.front()]->init())
                {
                    printf("\033[31merror here 1094\033[0m\n");
                }
            }
            been_merged_inss[this_connect[i]] = true;
        }
    }
    // 删除地图中已经被吸收的
    for (size_t i = 0; i < vertices_size; ++i)
    {
        if (!been_merged_inss[i])
            continue;
        auto it_global = global_map_manager.find(around_instances[i]);
        if (it_global != global_map_manager.end())
        {
            // printf("\033[34mmap delete instance from map %d\033[0m\n", (it_global->first)->id);
            global_map_manager.erase(it_global);
        }
        else
        {
            printf("\033[31mnot in global you wen ti %d\033[0m\n", around_instances[i]->id);
            exit(0);
        }
    }

    return true;
}



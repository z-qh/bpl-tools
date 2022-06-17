#define PCL_NO_PRECOMPILE

#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <exception>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/qhull.h>

#include <pcl/filters/voxel_grid.h>

#include "odom/common.h"
#include "gjk2d.h"

const float EPSILON = 1e-6;

typedef pcl::PointCloud<PointType> PolygonType;
using SeqId = uint32_t;

class InstanceBuilderOptions
{
public:
    Eigen::Vector3f ref_center;
};

class alignas(16) Instance
{
public:
    Instance()
    {
    }
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

public:
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

// 文件相关
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

#endif // _INSTANCE_H_

#define PCL_NO_PRECOMPILE

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/search/organized.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/qhull.h>

#include <pcl/filters/voxel_grid.h>

#include "map/instance.h"
#include "map/gjk2d.h"

using namespace std;

// 矩形相关
// 计算三角面积
double GetConvexHullArea(Eigen::Vector2f &p1, Eigen::Vector2f &p2, Eigen::Vector2f &p3)
{
    return std::abs(0.5 * (p1(0) * p2(1) + p2(0) * p3(1) + p3(0) * p1(1) - p1(0) * p3(1) - p2(0) * p1(1) - p3(0) * p2(1)));
}

// 计算矩形面积 vector size : 4
double GetRectangleArea(std::vector<Eigen::Vector2f> &rect)
{
    if (rect.size() != 4)
        return -1;
    return 2.0 * GetConvexHullArea(rect[0], rect[1], rect[2]);
}

// ab cd 两线平行 vector size : 2
bool parallel(std::vector<Eigen::Vector2f> &l1, std::vector<Eigen::Vector2f> &l2)
{
    return (l1[1](1) - l1[0](1)) * (l2[1](0) - l2[0](0)) - (l2[1](1) - l2[0](1)) * (l1[1](0) - l1[0](0)) == 0;
}

// vec(p1, p2) x vec(p1, p) : (b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y)
double GetCross(Eigen::Vector2f &a, Eigen::Vector2f &b, Eigen::Vector2f &c)
{
    return (b(0) - a(0)) * (c(1) - a(1)) - (c(0) - a(0)) * (b(1) - a(1));
}

// 判断点在矩形内
bool IsPointInMatrix(Eigen::Vector2f &p, std::vector<Eigen::Vector2f> &rect1)
{
    return GetCross(rect1[0], rect1[1], p) * GetCross(rect1[2], rect1[3], p) >= 0 && GetCross(rect1[1], rect1[2], p) * GetCross(rect1[3], rect1[0], p) >= 0;
}

// 计算线段交点 vector size : 2 重合相交无交点 顶点相交无交点 交叉相交一交点
bool GetIntersection(std::vector<Eigen::Vector2f> &l1, std::vector<Eigen::Vector2f> &l2, Eigen::Vector2f &intersect_point)
{
    assert(parallel(l1, l2) == false);
    intersect_point = Eigen::Vector2f(-1, -1);

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
bool Get2RectangleImpactArea(std::vector<Eigen::Vector2f> &rect1, std::vector<Eigen::Vector2f> &rect2, std::vector<double> &result)
{
    double s1 = GetRectangleArea(rect1);
    double s2 = GetRectangleArea(rect2);
    result = std::vector<double>{s1, s2, -1};
    if (s1 <= 0 || s2 <= 0)
        return false;
    // 4 edges ans 4 edges : 4 x 2
    std::vector<std::vector<Eigen::Vector2f>> edges1(4, std::vector<Eigen::Vector2f>(2, Eigen::Vector2f::Zero()));
    std::vector<std::vector<Eigen::Vector2f>> edges2(edges1);
    edges1[0][0] = rect1[0];
    edges1[0][1] = rect1[1];
    edges1[1][0] = rect1[1];
    edges1[1][1] = rect1[2];
    edges1[2][0] = rect1[2];
    edges1[2][1] = rect1[3];
    edges1[3][0] = rect1[3];
    edges1[3][1] = rect1[0];
    assert(parallel(edges1[0], edges1[2]) == true && parallel(edges1[1], edges1[3]) == true);
    edges2[0][0] = rect2[0];
    edges2[0][1] = rect2[1];
    edges2[1][0] = rect2[1];
    edges2[1][1] = rect2[2];
    edges2[2][0] = rect2[2];
    edges2[2][1] = rect2[3];
    edges2[3][0] = rect2[3];
    edges2[3][1] = rect2[0];
    assert(parallel(edges2[0], edges2[2]) == true && parallel(edges2[1], edges2[3]) == true);
    // get intersections
    std::set<std::pair<double, double>> intersections_set;
    std::vector<Eigen::Vector2f> intersections;
    std::cout << "intersection" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            Eigen::Vector2f intersection;
            if (!GetIntersection(edges1[i], edges2[j], intersection))
                continue;
            std::cout << intersection(0) << " " << intersection(1) << std::endl;
            auto intersection_pair = std::make_pair(intersection(0), intersection(1));
            auto it = intersections_set.find(intersection_pair);
            if (it != intersections_set.end())
                continue;
            intersections_set.insert(intersection_pair);
            intersections.push_back(intersection);
        }
    }
    // get another inside points
    std::vector<Eigen::Vector2f> inside_points;
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
    std::cout << " points " << std::endl;
    for (auto &p : convex_vertex->points)
    {
        std::cout << p.x << " " << p.y << std::endl;
    }
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
bool UpXVector2(const Eigen::Vector2f &a, const Eigen::Vector2f &b)
{
    return a(0) < b(0);
}
bool UpYVector2(const Eigen::Vector2f &a, const Eigen::Vector2f &b)
{
    return a(1) < b(1);
}
bool DownYVector2(const Eigen::Vector2f &a, const Eigen::Vector2f &b)
{
    return a(1) > b(1);
}

void GetOrderedRect(std::vector<Eigen::Vector2f> &rect)
{
    std::sort(rect.begin(), rect.end(), UpXVector2);
    std::sort(rect.begin(), rect.begin() + 2, DownYVector2);
    std::sort(rect.begin() + 2, rect.end(), UpYVector2);
    for (auto &p : rect)
    {
        printf("(%f,%f  )", p(0), p(1));
    }
    printf("\n");
}

void testRect()
{

    vector<Eigen::Vector2f> rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8, rect9;
    // points
    rect1.push_back(Eigen::Vector2f{-1.0, 1.0});
    rect1.push_back(Eigen::Vector2f{1.0, 1.0});
    rect1.push_back(Eigen::Vector2f{1.0, -1.0});
    rect1.push_back(Eigen::Vector2f{-1.0, -1.0});

    rect2.push_back(Eigen::Vector2f{-10, -8});
    rect2.push_back(Eigen::Vector2f{-8, -8});
    rect2.push_back(Eigen::Vector2f{-8, -7});
    rect2.push_back(Eigen::Vector2f{-10, -7});

    rect3.push_back(Eigen::Vector2f{0.0, 1.0});
    rect3.push_back(Eigen::Vector2f{1.0, 2.0});
    rect3.push_back(Eigen::Vector2f{2.0, 1.0});
    rect3.push_back(Eigen::Vector2f{1.0, 0.0});

    rect4.push_back(Eigen::Vector2f{-1.414, 0});
    rect4.push_back(Eigen::Vector2f{0, 1.414});
    rect4.push_back(Eigen::Vector2f{1.414, 0});
    rect4.push_back(Eigen::Vector2f{0, -1.414});

    rect5.push_back(Eigen::Vector2f{-1, 0.5});
    rect5.push_back(Eigen::Vector2f{1, 0.5});
    rect5.push_back(Eigen::Vector2f{1, 1.5});
    rect5.push_back(Eigen::Vector2f{-1, 1.5});

    //
    // std::vector<double> result;
    // if( Get2RectangleImpactArea(rect1, rect2, result) )
    //     printf("area2 %f %f %f\n", result[0], result[1], result[2]);
    // if( Get2RectangleImpactArea(rect1, rect3, result) )
    //     printf("area3 %f %f %f\n", result[0], result[1], result[2]);
    std::vector<double> result;
    if (Get2RectangleImpactArea(rect1, rect4, result))
        printf("area4 %f %f %f\n", result[0], result[1], result[2]);
    // if( Get2RectangleImpactArea(rect1, rect5, result) )
    //     printf("area5 %f %f %f\n", result[0], result[1], result[2]);
    std::vector<Eigen::Vector2f> l1{{0, 1}, {0, 2}};
    std::vector<Eigen::Vector2f> l2{{1, 0}, {2, 0}};
    std::cout << parallel(l1, l2) << std::endl;

    //
    // rect6.push_back(Eigen::Vector2f{-1, 0.5});//a
    // rect6.push_back(Eigen::Vector2f{1, 0.5});//b
    // rect6.push_back(Eigen::Vector2f{1, 1.5});//c
    // rect6.push_back(Eigen::Vector2f{-1,1.5});//d

    // rect7.push_back(Eigen::Vector2f{-1, 0.5});//a
    // rect7.push_back(Eigen::Vector2f{-1,1.5});//d
    // rect7.push_back(Eigen::Vector2f{1, 0.5});//b
    // rect7.push_back(Eigen::Vector2f{1, 1.5});//c

    // rect8.push_back(Eigen::Vector2f{-1, 0.5});//a
    // rect8.push_back(Eigen::Vector2f{-1,1.5});//d
    // rect8.push_back(Eigen::Vector2f{1, 1.5});//c
    // rect8.push_back(Eigen::Vector2f{1, 0.5});//b

    // rect9.push_back(Eigen::Vector2f{1, 0.5});//b
    // rect9.push_back(Eigen::Vector2f{1, 1.5});//c
    // rect9.push_back(Eigen::Vector2f{-1, 0.5});//a
    // rect9.push_back(Eigen::Vector2f{-1,1.5});//d

    // GetOrderedRect(rect6);
    // GetOrderedRect(rect7);
    // GetOrderedRect(rect8);
    // GetOrderedRect(rect9);

    std::vector<double> test1result;
    vector<Eigen::Vector2f> test1, test2;
    test1 = vector<Eigen::Vector2f>{
        {22.424697, -4.982507}, {22.358124, -11.449210}, {23.965817, -11.465761}, {24.032391, -4.999058}};
    test2 = vector<Eigen::Vector2f>{
        {22.013256, -8.972094}, {22.432433, -11.001574}, {22.960191, -10.892572}, {22.541014, -8.863091}};
    GetOrderedRect(test1);
    GetOrderedRect(test2);
    Get2RectangleImpactArea(test1, test2, test1result);
    for (auto &p : test1result)
    {
        cout << p << " ";
    }
    cout << endl;

    char key;
    key = getchar();
    vector<shared_ptr<Instance>> inss1, inss2;
    std::string dddd1 = "/home/qh/ins_map_temp/ins_map";
    LoadInstanaces(inss1, dddd1);
    getchar();
}

// 凸包相关

typedef std::shared_ptr<Instance> InstancePtr;
typedef std::vector<std::shared_ptr<Instance>> InstancesPtr;

// 获取点云边界 以及XY上的均值和协方差
void GetCloudMinMax3D(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, Eigen::Vector4f &distribute)
{
    int point_size = cloud->size();
    float x_sum = 0, y_sum = 0, x_sum_sq = 0, y_sum_sq = 0;
    min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
    max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
    for (size_t i = 0; i < point_size; ++i)
    {
        x_sum += cloud->points[i].x;
        y_sum += cloud->points[i].y;
        x_sum_sq += cloud->points[i].x * cloud->points[i].x;
        y_sum_sq += cloud->points[i].y * cloud->points[i].y;
        min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
        max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
        min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
        max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
        min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
        max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
    // 均值 方差
    x_sum -= (min_pt(0) + max_pt(0)) * 0.5 * point_size;
    y_sum -= (min_pt(1) + max_pt(1)) * 0.5 * point_size;
    distribute(0) = x_sum / point_size;
    distribute(1) = y_sum / point_size;
    distribute(2) = (x_sum_sq - x_sum * distribute(0) / (point_size - 1));
    distribute(3) = (y_sum_sq - y_sum * distribute(1) / (point_size - 1));
}

// 首先计算边界 确定中心 对点云做 Z轴大距离下采样 XY轴小分辨率下采样
pcl::PolygonMesh ComputePolygonBoundaries(InstancePtr &ins)
{
    cout << "ID" << ins->id << endl;
    pcl::PolygonMesh mesh_resutlt;
    if (ins->cloud->size() <= 30)
        return mesh_resutlt; //数量不足
    Eigen::Vector4f min_pt, max_pt;
    Eigen::Vector4f distribute;
    GetCloudMinMax3D(ins->cloud, min_pt, max_pt, distribute);
    cout << "dist " << distribute(0) << " " << distribute(1) << endl;
    Eigen::Vector3f center((min_pt[0] + max_pt[0]) / 2, //初始化center点
                           (min_pt[1] + max_pt[1]) / 2,
                           (min_pt[2] + max_pt[2]) / 2);
    // 处理边界退化
    float epslin = 1e-1;
    for (int i = 0; i < 3; i++)
    {
        if (max_pt[i] - min_pt[i] < epslin)
        {
            max_pt[i] = center[i] + epslin / 2;
            min_pt[i] = center[i] - epslin / 2;
        }
    }
    // 中心
    ins->center = Eigen::Vector3f((max_pt[0] + min_pt[0]) / 2,
                                  (max_pt[1] + min_pt[1]) / 2,
                                  (max_pt[2] + min_pt[2]) / 2);
    // 点云高度简化为平均高度模型
    ins->min_height = static_cast<double>(min_pt[2]);
    ins->max_height = static_cast<double>(max_pt[2]);
    ins->height = static_cast<double>(max_pt[2] - min_pt[2]);

    pcl::PointCloud<PointType>::Ptr pcd_xy(new pcl::PointCloud<PointType>()); //将点云投影到地平面上,z坐标全部换成目标障碍物的最小z坐标
    // 使用voxelgrid下采样
    pcl::VoxelGrid<PointType> donw_sample_XYY2XY;
    Eigen::Vector3f donw_sample_leaf_size;
    donw_sample_leaf_size(0) = abs(distribute(0)) > 0.3 ? abs(distribute(0)) : 0.3;
    donw_sample_leaf_size(1) = abs(distribute(1)) > 0.3 ? abs(distribute(1)) : 0.3;
    donw_sample_leaf_size(2) = ins->height;

    donw_sample_XYY2XY.setLeafSize(donw_sample_leaf_size(0), donw_sample_leaf_size(1), donw_sample_leaf_size(2));
    donw_sample_XYY2XY.setInputCloud(ins->cloud);
    donw_sample_XYY2XY.filter(*pcd_xy);
    // 将点云投影到地面上
    for (auto &p : pcd_xy->points)
    {
        p.z = min_pt(2);
    }
    ins->cloud_2dshape = pcd_xy;
    pcl::io::savePCDFileASCII("/home/qh/ins_map_temp/" + std::to_string(ins->id) + "_filtered.pcd", *(ins->cloud));
    pcl::io::savePCDFileASCII("/home/qh/ins_map_temp/" + std::to_string(ins->id) + "_origin.pcd", *(ins->cloud_2dshape));
    // 对2d_shape 求凸包 求凹包
    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(pcd_xy);
    convex_hull.reconstruct(mesh_resutlt);
    return mesh_resutlt;
}

// 保留在内的多边形点
void polygon_internals_test(std::vector<PointType> &vertices1, PointType point, std::vector<PointType> &internals)
{
    size_t vertices1_size = vertices1.size() - 1;
    if (vertices1_size < 3)
        return;
    float nCurCrossProduct = 0, nLastValue = 0;
    for (int i = 0; i < vertices1_size; i++)
    {
        PointType vU = subtract(point, vertices1[i]);
        int nNextIndex = (i + 1) % vertices1_size;
        PointType vV = subtract(vertices1[nNextIndex], vertices1[i]);
        nCurCrossProduct = cross_product(vU, vV);
        if (i > 0 && nCurCrossProduct * nLastValue <= 0)
        {
            return;
        }
        nLastValue = nCurCrossProduct;
    }
    internals.push_back(point);
    cout << "get inter " << point.x << " " << point.y << endl;
}

// 线段切割多边形, 用线段ab切割多边形p, 保留交点
void polygon_cut_ls_test(std::vector<PointType> &vertices, PointType l_a, PointType l_b, std::vector<PointType> &intersection)
{
    size_t vertices_size = vertices.size() - 1; // a-b-c-d-a
    if (vertices_size < 3)
        return;
    for (int i = 0; i < vertices_size; i++)
    {
        if (sig(cross(l_a, l_b, vertices[i])) != sig(cross(l_a, l_b, vertices[i + 1])))
        {
            PointType this_inter;
            is_line_cross(l_a, l_b, vertices[i], vertices[i + 1], this_inter);
            if (is_point_online(l_a, l_b, vertices[i], vertices[i + 1], this_inter))
            {
                cout << "get intersec " << this_inter.x << " " << this_inter.y << endl;
                this_inter.z = 0;
                intersection.push_back(this_inter);
            }
            else
            {
                cout << "get intersec abandon " << this_inter.x << " " << this_inter.y << endl;
                cout << l_a.x << " " << l_a.y << " " << endl;
                cout << l_b.x << " " << l_b.y << " " << endl;
                cout << vertices[i].x << " " << vertices[i].y << " " << endl;
                cout << vertices[i + 1].x << " " << vertices[i + 1].y << " " << endl;
            }
        }
    }
}

// 求两多边形重复区域
float polygon_collision_area_test(std::vector<PointType> &vertices1, std::vector<PointType> &vertices2)
{
    if (vertices1.size() < 3 || vertices2.size() < 3)
        return 0;
    // 首先将两个多边形循环
    vertices1.push_back(vertices1.front());
    vertices2.push_back(vertices2.front());
    size_t vertices1_size = vertices1.size() - 1;
    size_t vertices2_size = vertices2.size() - 1;
    // 计算交点和内点
    std::vector<PointType> intersections, internals;
    for (size_t i = 0; i < vertices1_size; ++i)
    {
        polygon_cut_ls_test(vertices2, vertices1[i], vertices1[i + 1], intersections);
        polygon_internals_test(vertices2, vertices1[i], internals);
    }
    for (size_t i = 0; i < vertices2_size; ++i)
    {
        polygon_internals_test(vertices1, vertices2[i], internals);
    }
    pcl::PointCloud<PointType>::Ptr hull_cloud(new pcl::PointCloud<PointType>());
    for (auto &p : intersections)
        hull_cloud->push_back(p);
    for (auto &p : internals)
        hull_cloud->push_back(p);
    cout << hull_cloud->points.size() << endl;
    for (auto &p : hull_cloud->points)
    {
        cout << "(" << p.x << "," << p.y << ")," << p.z << endl;
    }
    // 计算交点-内点组成的convexhull计算面积
    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setInputCloud(hull_cloud);
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    pcl::PolygonMesh mesh_resutlt;
    convex_hull.reconstruct(mesh_resutlt);
    float coll_area = convex_hull.getTotalArea();

    // 最后去除最重复的最后一个元素
    vertices1.pop_back();
    vertices2.pop_back();
    return coll_area;
}

void testpca(){
    Eigen::MatrixXf X(2,3);
    X << 0,0,0,0,0,0;
    auto shape = pca_2d(X);
    cout << shape << endl;
}

void dfs_connect_test(Eigen::MatrixXf &mat, int id, std::vector<bool> &visited, std::vector<int> &connect)
{
    visited[id] = true;
    for(size_t i = id+1; i < mat.cols(); ++i)
    {
        if( mat(id, i) != 0 && !visited[i] )
        {
            connect.push_back(i);
            dfs_connect_test(mat, i, visited, connect);
        }
    }
}


void test_connect_test()
{   
    size_t vertices_size = 10;
    Eigen::MatrixXf adjacency_matrix = Eigen::MatrixXf::Zero(vertices_size, vertices_size);
    adjacency_matrix(0,1) = 1;
    adjacency_matrix(1,2) = 1;
    adjacency_matrix(1,3) = 1;
    adjacency_matrix(3,7) = 1;
    adjacency_matrix(5,6) = 1;
    adjacency_matrix(7,8) = 1;
    std::vector<bool> visited(vertices_size, false);
    std::vector<std::vector<int>> connected;
    for( size_t i = 0; i < vertices_size; ++i)
    {
        std::vector<int> this_connect;
        if(!visited[i])
        {   
            this_connect.push_back(i);
            dfs_connect_test(adjacency_matrix, i, visited, this_connect);
        }
        if( this_connect.size() > 1 )
        {
            cout << " find a connect :";
            for(auto&p:this_connect) cout << " " << p;
            cout << endl;
            connected.push_back(this_connect);
        }
    }
}

int main()
{
    // test_connect_test();


    string dddd1 = "/home/qh/ins_map_temp/-1002.ins";
    string dddd2 = "/home/qh/ins_map_temp/-1027.ins";
    auto insA = LoadInstanace(dddd1);
    auto insB = LoadInstanace(dddd2);
    if(insA == nullptr) return 0;
    if(insB == nullptr) return 0;
    cout << insA->shape(0) << " " << insA->shape(1) << endl;
    cout << insB->shape(0) << " " << insB->shape(1) << endl;
    std::vector<PointType> ins_a_ver, ins_b_ver;
    cout << insA->cloud_2dshape->size() << endl;
    for (auto &p : insA->cloud_2dshape->points)
    {
        cout << "(" << p.x << "," << p.y << ")," << endl;
        ins_a_ver.push_back(p);
    }
    cout << insB->cloud_2dshape->size() << endl;
    for (auto &p : insB->cloud_2dshape->points)
    {
        cout << "(" << p.x << "," << p.y << ")," << endl;
        ins_b_ver.push_back(p);
    }

    if(polygon_collision_area(ins_a_ver, ins_b_ver)) cout << " coll !!!!!"  << endl;
    cout << "##################" << endl;
    float area = polygon_collision_area_test(ins_a_ver, ins_b_ver);
    cout << insA->area << endl;
    cout << insB->area << endl;
    cout << "area " << area << endl;

    if( area-insA->area > 1E-2 ) cout << " 123 " << endl;

    return 0;
}
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

#include "map/instance.h"

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
    return (l1[1](1) - l1[0](1)) * (l2[1](0) - l2[0](0)) - (l2[1](1) - l2[0](1)) * (l1[1](0) - l1[0](0)) == 0;
}

// vec(p1, p2) x vec(p1, p) : (b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y)
double GetCross(Eigen::Vector2d &a, Eigen::Vector2d &b, Eigen::Vector2d &c)
{
    return (b(0) - a(0)) * (c(1) - a(1)) - (c(0) - a(0)) * (b(1) - a(1));
}

// 判断点在矩形内
bool IsPointInMatrix(Eigen::Vector2d &p, std::vector<Eigen::Vector2d> &rect1)
{
    return GetCross(rect1[0], rect1[1], p) * GetCross(rect1[2], rect1[3], p) >= 0 && GetCross(rect1[1], rect1[2], p) * GetCross(rect1[3], rect1[0], p) >= 0;
}

// 计算线段交点 vector size : 2 重合相交无交点 顶点相交无交点 交叉相交一交点
bool GetIntersection(std::vector<Eigen::Vector2d> &l1, std::vector<Eigen::Vector2d> &l2, Eigen::Vector2d &intersect_point)
{
    assert(parallel(l1, l2) == false);
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
bool Get2RectangleImpactArea(std::vector<Eigen::Vector2d> &rect1, std::vector<Eigen::Vector2d> &rect2, std::vector<double>& result)
{
    double s1 = GetRectangleArea(rect1);
    double s2 = GetRectangleArea(rect2);
    result = std::vector<double>{s1, s2, -1};
    if (s1 <= 0 || s2 <= 0)
        return false;
    // 4 edges ans 4 edges : 4 x 2
    std::vector<std::vector<Eigen::Vector2d>> edges1(4, std::vector<Eigen::Vector2d>(2, Eigen::Vector2d::Zero()));
    std::vector<std::vector<Eigen::Vector2d>> edges2(edges1);
    edges1[0][0] = rect1[0]; edges1[0][1] = rect1[1];
    edges1[1][0] = rect1[1]; edges1[1][1] = rect1[2];
    edges1[2][0] = rect1[2]; edges1[2][1] = rect1[3];
    edges1[3][0] = rect1[3]; edges1[3][1] = rect1[0];
    assert(parallel(edges1[0], edges1[2]) == true && parallel(edges1[1], edges1[3]) == true);
    edges2[0][0] = rect2[0]; edges2[0][1] = rect2[1];
    edges2[1][0] = rect2[1]; edges2[1][1] = rect2[2];
    edges2[2][0] = rect2[2]; edges2[2][1] = rect2[3];
    edges2[3][0] = rect2[3]; edges2[3][1] = rect2[0];
    assert(parallel(edges2[0], edges2[2]) == true && parallel(edges2[1], edges2[3]) == true);
    // get intersections
    std::set<std::pair<double, double>> intersections_set;
    std::vector<Eigen::Vector2d> intersections;
    std::cout << "intersection" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            Eigen::Vector2d intersection;
            if(!GetIntersection(edges1[i], edges2[j], intersection)) continue;
            std::cout << intersection(0) << " " << intersection(1) << std::endl;
            auto intersection_pair = std::make_pair(intersection(0), intersection(1));
            auto it = intersections_set.find(intersection_pair);
            if (it != intersections_set.end()) continue;
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
    std::cout << " points " << std::endl;
    for(auto&p:convex_vertex->points){
        std::cout << p.x << " " << p.y << std::endl;
    }
    if(convex_vertex->size() < 3) return false;
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
bool UpXVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b){
    return a(0) < b(0);
}
bool UpYVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b){
    return a(1) < b(1);
}
bool DownYVector2(const Eigen::Vector2d &a, const Eigen::Vector2d &b){
    return a(1) > b(1);
}

void GetOrderedRect(std::vector<Eigen::Vector2d> &rect){
    std::sort(rect.begin(), rect.end(), UpXVector2);
    std::sort(rect.begin(), rect.begin()+2, DownYVector2);
    std::sort(rect.begin()+2, rect.end(), UpYVector2);
    for(auto&p:rect){
        printf("(%f,%f  )", p(0), p(1));
    }
    printf("\n");
}

using namespace std;
int main()
{
    vector<Eigen::Vector2d> rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8, rect9;
    // points
    rect1.push_back(Eigen::Vector2d{-1.0, 1.0});
    rect1.push_back(Eigen::Vector2d{1.0, 1.0});
    rect1.push_back(Eigen::Vector2d{1.0, -1.0});
    rect1.push_back(Eigen::Vector2d{-1.0, -1.0});

    rect2.push_back(Eigen::Vector2d{-10, -8});
    rect2.push_back(Eigen::Vector2d{-8, -8});
    rect2.push_back(Eigen::Vector2d{-8, -7});
    rect2.push_back(Eigen::Vector2d{-10,-7});

    rect3.push_back(Eigen::Vector2d{0.0, 1.0});
    rect3.push_back(Eigen::Vector2d{1.0, 2.0});
    rect3.push_back(Eigen::Vector2d{2.0, 1.0});
    rect3.push_back(Eigen::Vector2d{1.0, 0.0});

    rect4.push_back(Eigen::Vector2d{-1.414, 0});
    rect4.push_back(Eigen::Vector2d{0, 1.414});
    rect4.push_back(Eigen::Vector2d{ 1.414, 0});
    rect4.push_back(Eigen::Vector2d{0,-1.414});

    rect5.push_back(Eigen::Vector2d{-1, 0.5});
    rect5.push_back(Eigen::Vector2d{1, 0.5});
    rect5.push_back(Eigen::Vector2d{1, 1.5});
    rect5.push_back(Eigen::Vector2d{-1,1.5});

    
    //
    // std::vector<double> result;
    // if( Get2RectangleImpactArea(rect1, rect2, result) )
    //     printf("area2 %f %f %f\n", result[0], result[1], result[2]);   
    // if( Get2RectangleImpactArea(rect1, rect3, result) )
    //     printf("area3 %f %f %f\n", result[0], result[1], result[2]);   
    std::vector<double> result;
    if( Get2RectangleImpactArea(rect1, rect4, result) )
        printf("area4 %f %f %f\n", result[0], result[1], result[2]);   
    // if( Get2RectangleImpactArea(rect1, rect5, result) )
    //     printf("area5 %f %f %f\n", result[0], result[1], result[2]);
    std::vector<Eigen::Vector2d> l1{{0, 1}, {0, 2}};
    std::vector<Eigen::Vector2d> l2{{1, 0}, {2, 0}};    
    std::cout << parallel(l1, l2) << std::endl;

    //
    // rect6.push_back(Eigen::Vector2d{-1, 0.5});//a
    // rect6.push_back(Eigen::Vector2d{1, 0.5});//b
    // rect6.push_back(Eigen::Vector2d{1, 1.5});//c
    // rect6.push_back(Eigen::Vector2d{-1,1.5});//d

    // rect7.push_back(Eigen::Vector2d{-1, 0.5});//a
    // rect7.push_back(Eigen::Vector2d{-1,1.5});//d
    // rect7.push_back(Eigen::Vector2d{1, 0.5});//b
    // rect7.push_back(Eigen::Vector2d{1, 1.5});//c

    // rect8.push_back(Eigen::Vector2d{-1, 0.5});//a
    // rect8.push_back(Eigen::Vector2d{-1,1.5});//d
    // rect8.push_back(Eigen::Vector2d{1, 1.5});//c
    // rect8.push_back(Eigen::Vector2d{1, 0.5});//b

    // rect9.push_back(Eigen::Vector2d{1, 0.5});//b
    // rect9.push_back(Eigen::Vector2d{1, 1.5});//c    
    // rect9.push_back(Eigen::Vector2d{-1, 0.5});//a
    // rect9.push_back(Eigen::Vector2d{-1,1.5});//d

    // GetOrderedRect(rect6);
    // GetOrderedRect(rect7);
    // GetOrderedRect(rect8);
    // GetOrderedRect(rect9);
    
    std::vector<double> test1result;
    vector<Eigen::Vector2d> test1, test2;
    test1 = vector<Eigen::Vector2d>{ 
        {22.424697,-4.982507}, {22.358124,-11.449210}, {23.965817,-11.465761}, {24.032391,-4.999058}
        };
    test2 = vector<Eigen::Vector2d>{ 
        {22.013256,-8.972094}, {22.432433,-11.001574}, {22.960191,-10.892572}, {22.541014,-8.863091}
        };
    GetOrderedRect(test1);
    GetOrderedRect(test2);
    Get2RectangleImpactArea(test1, test2, test1result);
    for(auto&p:test1result){
        cout << p << " ";
    }
    cout << endl;

    char key;
    key = getchar();
    vector<shared_ptr<Instance>> inss1, inss2;
    string dddd1 = "/home/qh/temp/ins_map";
    LoadInstanaces(inss1, dddd1);
    getchar();

    return 0;
}
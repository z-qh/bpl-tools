#define PCL_NO_PRECOMPILE

#ifndef _GJK2D_H_
#define _GJK2D_H_

#include <iostream>
#include <vector>
#include <exception>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/qhull.h>

#include <stdlib.h>
#include <float.h>

#include "odom/common.h"

// 作差向量
PointType subtract(PointType a, PointType b);
// 取反
PointType negate(PointType v);
// 垂直向量
PointType perpendicular(const PointType &v);
// 点乘
float dot_product(const PointType &a, const PointType &b);
// 取模
float length_squared(const PointType &v);
// 向量混合积计算指向民科夫斯基空间中原点的垂直法向量
PointType triple_product(const PointType &a, const PointType &b, const PointType &c);
// 计算大致中心用作GJK单纯形搜索的初始方向
PointType average_point(const std::vector<PointType> &vertices);
// 获取沿某个方向上的最远顶点
size_t index_of_furthest_oint(const std::vector<PointType> &vertices, PointType dir);
// 民科夫斯基和
PointType support(const std::vector<PointType> &vertices1, const std::vector<PointType> &vertices2, PointType dir);
bool gjk(const std::vector<PointType> &vertices1, const std::vector<PointType> &vertices2);
// 叉乘 OAxOB
float cross(const PointType &o, const PointType &a, const PointType &b);
float cross_product(const PointType &vTarget1, const PointType &vTarget2);
// 符号函数
int sig(float d);
// 凸多边形面积 a-b-c-d-a
float area(std::vector<PointType> &vertices);
// 线条交点
bool is_line_cross(const PointType &a, const PointType &b, const PointType &c, const PointType &d, PointType &p);
// 线切割多边形, 用直线ab切割多边形p, 保留交点
void polygon_cut(std::vector<PointType> &vertices, PointType l_a, PointType l_b, std::vector<PointType> &intersection);
// 计算线段交点 vector size : 2 重合相交无交点 顶点相交无交点 交叉相交一交点
bool is_point_online(const PointType &a, const PointType &b, const PointType &c, const PointType &d, const PointType &intersect_point);
// 线段切割多边形, 用线段ab切割多边形p, 保留交点
void polygon_cut_ls(std::vector<PointType> &vertices, PointType l_a, PointType l_b, std::vector<PointType> &intersection);
// 保留在内的多边形点
void polygon_internals(std::vector<PointType> &vertices1, PointType point, std::vector<PointType> &internals);
// 求两多边形重复区域
float polygon_collision_area(std::vector<PointType> &vertices1, std::vector<PointType> &vertices2);
// pcd 判断是否过于密集
Eigen::MatrixXf pca_2d(Eigen::MatrixXf &X);
Eigen::MatrixXf pca_2d(std::vector<PointType> &vertices);
Eigen::MatrixXf pca_2d(pcl::PointCloud<PointType>::Ptr cloud);
Eigen::MatrixXf pca_2d(pcl::PointCloud<PointType> &cloud);

#endif // GJK2D

#define PCL_NO_PRECOMPILE

#include "gjk2d.h"



const float eps = 1E-3;
// 基础点处理方法
// 作差向量
PointType subtract(PointType a, PointType b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

// 取反
PointType negate(PointType v)
{
    v.x = -v.x;
    v.y = -v.y;
    return v;
}

// 垂直向量
PointType perpendicular(const PointType &v)
{
    PointType p;
    p.x = v.y;
    p.y = -v.x;
    return p;
}

// 点乘
float dot_product(const PointType &a, const PointType &b)
{
    return a.x * b.x + a.y * b.y;
}

// 取模
float length_squared(const PointType &v)
{
    return v.x * v.x + v.y * v.y;
}

// 向量混合积计算指向民科夫斯基空间中原点的垂直法向量
PointType triple_product(const PointType &a, const PointType &b, const PointType &c)
{
    PointType r;
    float ac = a.x * c.x + a.y * c.y;
    float bc = b.x * c.x + b.y * c.y;
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

// 计算大致中心用作GJK单纯形搜索的初始方向
PointType average_point(const std::vector<PointType> &vertices)
{
    size_t count = vertices.size();
    PointType avg;
    avg.x = 0;
    avg.y = 0;
    for (size_t i = 0; i < count; i++)
    {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}

// 获取沿某个方向上的最远顶点
size_t index_of_furthest_oint(const std::vector<PointType> &vertices, PointType dir)
{
    float maxProduct = dot_product(dir, vertices[0]);
    size_t count = vertices.size();
    size_t index = 0;
    for (size_t i = 1; i < count; i++)
    {
        float product = dot_product(dir, vertices[i]);
        if (product > maxProduct)
        {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

// 民科夫斯基和
PointType support(const std::vector<PointType> &vertices1, const std::vector<PointType> &vertices2, PointType dir)
{
    // 沿任意方向获取第一个实体的最远点
    size_t i = index_of_furthest_oint(vertices1, dir);
    // 沿相反方向获取第二个实体的最远点
    size_t j = index_of_furthest_oint(vertices2, negate(dir));
    // 减去两个点以查看实体是否重叠——民科夫斯基和
    return subtract(vertices1[i], vertices2[j]);
}

bool gjk(const std::vector<PointType> &vertices1, const std::vector<PointType> &vertices2)
{
    size_t index = 0; // 单纯形当前顶点的索引
    PointType a, b, c, dir, ao, ab, ac, abperp, acperp, simplex[3];
    PointType position1 = average_point(vertices1); // 简单中心代替重力中心
    PointType position2 = average_point(vertices2); //
    // 第一个实体中心到第二个实体中心的初始方向
    dir = subtract(position1, position2);
    // 如果初始方向为零则将其设置为任意轴（X轴）
    if ((dir.x == 0) && (dir.y == 0))
        dir.x = 1.0f;
    // 将第一个支撑点设置为新单纯形的初始点
    a = simplex[0] = support(vertices1, vertices2, dir);
    if (dot_product(a, dir) <= 0)
        return false; // 没有碰撞
    dir = negate(a);  // 下一个搜索方向始终朝向原点
    while (1)
    {
        a = simplex[++index] = support(vertices1, vertices2, dir);
        if (dot_product(a, dir) <= 0)
            return false; // 无碰撞
        ao = negate(a);   // 从a到原点，即-a
        // 单纯形有2个点 (直线段，还不是三角形)
        if (index < 2)
        {
            b = simplex[0];
            ab = subtract(b, a);              // 向量AB
            dir = triple_product(ab, ao, ab); // 垂直AB指向原点
            if (length_squared(dir) == 0)
                dir = perpendicular(ab);
            continue;
        }
        b = simplex[1];
        c = simplex[0];
        ab = subtract(b, a); // 向量AB
        ac = subtract(c, a); // 向量AC
        acperp = triple_product(ab, ac, ac);
        if (dot_product(acperp, ao) >= 0)
        {
            dir = acperp; // 新方向垂直AC朝向原点
        }
        else
        {
            abperp = triple_product(ac, ab, ab);
            if (dot_product(abperp, ao) < 0)
                return true;         // 有碰撞
            simplex[0] = simplex[1]; // 交换第一个元素(C点)
            dir = abperp;            // 新方向垂直于AB朝向原点
        }
        simplex[1] = simplex[2]; // 交换B点
        --index;
    }
    return false;
}

// 叉乘 OAxOB
float cross(const PointType &o, const PointType &a, const PointType &b)
{
    return (a.x - o.x) * (b.y - o.y) - (b.x - o.x) * (a.y - o.y);
}

float cross_product(const PointType &vTarget1, const PointType &vTarget2)
{
    return vTarget1.x * vTarget2.y - vTarget2.x * vTarget1.y;
}

// 符号函数
int sig(float d)
{
    return (d > eps) - (d < -eps);
}

// 凸多边形面积 a-b-c-d-a
float area(std::vector<PointType> &vertices)
{
    size_t vertices_size = vertices.size() - 1;
    if (vertices_size < 3)
        return 0;
    float area_res = 0;
    for (size_t i = 0; i < vertices_size; i++)
    {
        area_res += vertices[i].x * vertices[i + 1].y - vertices[i].y * vertices[i + 1].x;
    }
    return area_res / 2.0;
}

// 线条交点
bool is_line_cross(const PointType &a, const PointType &b, const PointType &c, const PointType &d, PointType &p)
{
    float s1, s2;
    s1 = cross(a, b, c);
    s2 = cross(a, b, d);
    if (sig(s1) == 0 && sig(s2) == 0)
        return false;
    if (sig(s2 - s1) == 0)
        return false;
    p.x = (c.x * s2 - d.x * s1) / (s2 - s1);
    p.y = (c.y * s2 - d.y * s1) / (s2 - s1);
    return true;
}

// 线切割多边形, 用直线ab切割多边形p, 保留交点
void polygon_cut(std::vector<PointType> &vertices, PointType l_a, PointType l_b, std::vector<PointType> &intersection)
{
    size_t vertices_size = vertices.size() - 1; // a-b-c-d-a
    if (vertices_size < 3)
        return;
    for (int i = 0; i < vertices_size; i++)
    {
        if (sig(cross(l_a, l_b, vertices[i])) > 0)
        {
            intersection.push_back(vertices[i]);
        }
        if (sig(cross(l_a, l_b, vertices[i])) != sig(cross(l_a, l_b, vertices[i + 1])))
        {
            PointType this_inter;
            this_inter.z = 0;
            is_line_cross(l_a, l_b, vertices[i], vertices[i + 1], this_inter);
            intersection.push_back(this_inter);
        }
    }
}

// 计算线段交点 vector size : 2 重合相交无交点 顶点相交无交点 交叉相交一交点
bool is_point_online(const PointType &a, const PointType &b, const PointType &c, const PointType &d, const PointType &intersect_point)
{
    if (
        std::min(c.x, d.x) < intersect_point.x &&
        std::max(c.x, d.x) > intersect_point.x &&
        std::min(c.y, d.y) < intersect_point.y &&
        std::max(c.y, d.y) > intersect_point.y &&
        std::min(a.x, b.x) < intersect_point.x &&
        std::max(a.x, b.x) > intersect_point.x &&
        std::min(a.y, b.y) < intersect_point.y &&
        std::max(a.y, b.y) > intersect_point.y &&
        sig(cross(intersect_point, a, b)) == 0 &&
        sig(cross(intersect_point, c, d)) == 0)
        return true;
    return false;
}

// 线段切割多边形, 用线段ab切割多边形p, 保留交点
void polygon_cut_ls(std::vector<PointType> &vertices, PointType l_a, PointType l_b, std::vector<PointType> &intersection)
{
    size_t vertices_size = vertices.size() - 1; // a-b-c-d-a
    if (vertices_size < 3)
        return;
    for (int i = 0; i < vertices_size; i++)
    {
        if (sig(cross(l_a, l_b, vertices[i])) != sig(cross(l_a, l_b, vertices[i + 1])))
        {
            PointType this_inter;
            this_inter.z = 0;
            is_line_cross(l_a, l_b, vertices[i], vertices[i + 1], this_inter);
            if (is_point_online(l_a, l_b, vertices[i], vertices[i + 1], this_inter))
                intersection.push_back(this_inter);
        }
    }
}

// 保留在内的多边形点
void polygon_internals(std::vector<PointType> &vertices1, PointType point, std::vector<PointType> &internals)
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
}

// 求两多边形重复区域
float polygon_collision_area(std::vector<PointType> &vertices1, std::vector<PointType> &vertices2)
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
        polygon_cut_ls(vertices2, vertices1[i], vertices1[i + 1], intersections);
        polygon_internals(vertices2, vertices1[i], internals);
    }
    for (size_t i = 0; i < vertices2_size; ++i)
    {
        polygon_internals(vertices1, vertices2[i], internals);
    }
    pcl::PointCloud<PointType>::Ptr hull_cloud(new pcl::PointCloud<PointType>());
    for (auto &p : intersections)
        hull_cloud->push_back(p);
    for (auto &p : internals)
        hull_cloud->push_back(p);
    if (hull_cloud->size() < 3)
        return 0;
    // 计算交点-内点组成的convexhull计算面积
    pcl::ConvexHull<PointType> convex_hull;
    convex_hull.setInputCloud(hull_cloud);
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    pcl::PolygonMesh mesh_resutlt;
    try
    {
        convex_hull.reconstruct(mesh_resutlt);
    }
    catch (std::exception &e)
    {
        for (auto &p : hull_cloud->points)
        {
            printf("\033[31m(%f,%f)\033[0m\n", p.x, p.y);
        }
    }
    float coll_area = convex_hull.getTotalArea();

    // 最后去除最重复的最后一个元素
    vertices1.pop_back();
    vertices2.pop_back();
    return coll_area;
}

// pcd 判断是否过于密集
Eigen::MatrixXf pca_2d(Eigen::MatrixXf &X)
{
    // 计算每一维度均值, 去中心化
    Eigen::MatrixXf meanval = X.rowwise().mean(); //每一列的矩阵，列降维
    for (size_t i = 0; i < X.cols(); i++)
    {
        X.col(i) -= meanval;
    }
    // 协方差
    Eigen::MatrixXf C = X * X.transpose() / (X.cols()-1);
    if(C.determinant() == 0 ) return Eigen::Vector2f::Zero();
    // 特征值分解
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(C);
	Eigen::MatrixXf vec = eig.eigenvectors();
	Eigen::MatrixXf val = eig.eigenvalues();
    return val;
}

Eigen::MatrixXf pca_2d(std::vector<PointType> &vertices)
{
    size_t vertices_size = vertices.size();
    Eigen::MatrixXf X(2, vertices_size);
    for(size_t i = 0; i < vertices_size; ++i)
    {
        X(0, i) = vertices[i].x;
        X(1, i) = vertices[i].y;
    }
    return pca_2d(X);
}

Eigen::MatrixXf pca_2d(pcl::PointCloud<PointType>::Ptr cloud)
{
    size_t point_size = cloud->size();
    Eigen::MatrixXf X(2, point_size);
    for(size_t i = 0; i < point_size; ++i)
    {
        X(0, i) = cloud->points[i].x;
        X(1, i) = cloud->points[i].y;
    }
    return pca_2d(X);
}

Eigen::MatrixXf pca_2d(pcl::PointCloud<PointType> &cloud)
{
    size_t point_size = cloud.size();
    Eigen::MatrixXf X(2, point_size);
    for(size_t i = 0; i < point_size; ++i)
    {
        X(0, i) = cloud.points[i].x;
        X(1, i) = cloud.points[i].y;
    }
    return pca_2d(X);
}


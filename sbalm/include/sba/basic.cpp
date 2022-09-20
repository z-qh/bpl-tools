#define PCL_NO_PRECOMPILE

#include "basic.h"




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



HungarianOptimizer::HungarianOptimizer(const std::vector<std::vector<float>>& costs)
    : state_(nullptr) {
  width_ = costs.size();

  if (width_ > 0) {
    height_ = costs[0].size();
  } else {
    height_ = 0;
  }

  matrix_size_ = std::max(width_, height_);
  max_cost_ = 0;

  // Generate the expanded cost matrix by adding extra 0-valued elements in
  // order to make a square matrix.  At the same time, find the greatest cost
  // in the matrix (used later if we want to maximize rather than minimize the
  // overall cost.)
  costs_.resize(matrix_size_);
  for (int row = 0; row < matrix_size_; ++row) {
    costs_[row].resize(matrix_size_);
  }

  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      if ((row >= width_) || (col >= height_)) {
        costs_[row][col] = 0;
      } else {
        costs_[row][col] = costs[row][col];
        max_cost_ = std::max(max_cost_, costs_[row][col]);
      }
    }
  }

  // Initially, none of the cells of the matrix are marked.
  marks_.resize(matrix_size_);
  for (int row = 0; row < matrix_size_; ++row) {
    marks_[row].resize(matrix_size_);
    for (int col = 0; col < matrix_size_; ++col) {
      marks_[row][col] = NONE;
    }
  }

  stars_in_col_.resize(matrix_size_);

  rows_covered_.resize(matrix_size_);
  cols_covered_.resize(matrix_size_);

  preimage_.resize(matrix_size_ * 2);
  image_.resize(matrix_size_ * 2);

  uncov_col_.resize(matrix_size_);
  uncov_row_.resize(matrix_size_);
}

// Find an assignment which maximizes the total cost.
// Return an array of pairs of integers.  Each pair (i, j) corresponds to
// assigning agent i to task j.
void HungarianOptimizer::maximize(std::vector<int>* preimage,
                                  std::vector<int>* image) {
  // Find a maximal assignment by subtracting each of the
  // original costs from max_cost_  and then minimizing.
  for (int row = 0; row < width_; ++row) {
    for (int col = 0; col < height_; ++col) {
      costs_[row][col] = max_cost_ - costs_[row][col];
    }
  }
  minimize(preimage, image);
}

// Find an assignment which minimizes the total cost.
// Return an array of pairs of integers.  Each pair (i, j) corresponds to
// assigning agent i to task j.
void HungarianOptimizer::minimize(std::vector<int>* preimage,
                                  std::vector<int>* image) {
  do_munkres();
  find_assignments(preimage, image);
}

// Convert the final cost matrix into a set of assignments of agents -> tasks.
// Return an array of pairs of integers, the same as the return values of
// Minimize() and Maximize()
void HungarianOptimizer::find_assignments(std::vector<int>* preimage,
                                          std::vector<int>* image) {
  preimage->clear();
  image->clear();
  for (int row = 0; row < width_; ++row) {
    for (int col = 0; col < height_; ++col) {
      if (is_starred(row, col)) {
        preimage->push_back(row);
        image->push_back(col);
        break;
      }
    }
  }
  // TODO(user)
  // result_size = std::min(width_, height_);
  // CHECK image.size() == result_size
  // CHECK preimage.size() == result_size
}

// Find a column in row 'row' containing a star, or return
// kHungarianOptimizerColNotFound if no such column exists.
int HungarianOptimizer::find_star_in_row(int row) const {
  for (int col = 0; col < matrix_size_; ++col) {
    if (is_starred(row, col)) {
      return col;
    }
  }

  return kHungarianOptimizerColNotFound;
}

// Find a row in column 'col' containing a star, or return
// kHungarianOptimizerRowNotFound if no such row exists.
int HungarianOptimizer::find_star_in_col(int col) const {
  if (!col_contains_star(col)) {
    return kHungarianOptimizerRowNotFound;
  }

  for (int row = 0; row < matrix_size_; ++row) {
    if (is_starred(row, col)) {
      return row;
    }
  }

  // NOTREACHED
  return kHungarianOptimizerRowNotFound;
}

// Find a column in row containing a prime, or return
// kHungarianOptimizerColNotFound if no such column exists.
int HungarianOptimizer::find_prime_in_row(int row) const {
  for (int col = 0; col < matrix_size_; ++col) {
    if (is_primed(row, col)) {
      return col;
    }
  }

  return kHungarianOptimizerColNotFound;
}

// Remove the prime marks from every cell in the matrix.
void HungarianOptimizer::clear_primes() {
  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      if (is_primed(row, col)) {
        marks_[row][col] = NONE;
      }
    }
  }
}

// Uncovery ever row and column in the matrix.
void HungarianOptimizer::clear_covers() {
  for (int x = 0; x < matrix_size_; x++) {
    uncover_row(x);
    uncover_col(x);
  }
}

// Find the smallest uncovered cell in the matrix.
float HungarianOptimizer::find_smallest_uncovered() {
  float minval = std::numeric_limits<float>::max();
  uncov_col_.clear();
  uncov_row_.clear();

  for (int i = 0; i < matrix_size_; ++i) {
    if (!row_covered(i)) {
      uncov_row_.push_back(i);
    }
    if (!col_covered(i)) {
      uncov_col_.push_back(i);
    }
  }

  for (size_t row = 0; row < uncov_row_.size(); ++row) {
    for (size_t col = 0; col < uncov_col_.size(); ++col) {
      minval = std::min(minval, costs_[uncov_row_[row]][uncov_col_[col]]);
    }
  }

  return minval;
}

// Find an uncovered zero and store its co-ordinates in (zeroRow, zeroCol)
// and return true, or return false if no such cell exists.
bool HungarianOptimizer::find_zero(int* zero_row, int* zero_col) {
  uncov_col_.clear();
  uncov_row_.clear();

  for (int i = 0; i < matrix_size_; ++i) {
    if (!row_covered(i)) {
      uncov_row_.push_back(i);
    }
    if (!col_covered(i)) {
      uncov_col_.push_back(i);
    }
  }
  if (uncov_row_.empty() || uncov_col_.empty()) {
    return false;
  }

  for (size_t i = 0; i < uncov_row_.size(); ++i) {
    for (size_t j = 0; j < uncov_col_.size(); ++j) {
      if (costs_[uncov_row_[i]][uncov_col_[j]] == 0) {
        *zero_row = uncov_row_[i];
        *zero_col = uncov_col_[j];
        return true;
      }
    }
  }
  return false;
}

// Print the matrix to stdout (for debugging.)
void HungarianOptimizer::print_matrix() {
  for (int row = 0; row < matrix_size_; ++row) {
    for (int col = 0; col < matrix_size_; ++col) {
      printf("%g ", costs_[row][col]);

      if (is_starred(row, col)) {
        printf("*");
      }

      if (is_primed(row, col)) {
        printf("'");
      }
    }
    printf("\n");
  }
}

//  Run the Munkres algorithm!
void HungarianOptimizer::do_munkres() {
  int max_iter = 1000;
  int iter_num = 0;
  state_ = &HungarianOptimizer::reduce_rows;
  while (state_ != NULL && iter_num < max_iter) {
    // while (state_ != NULL) {
    (this->*state_)();
    ++iter_num;
  }
  if (iter_num >= max_iter) {
    check_star();
  }
}

void HungarianOptimizer::check_star() {
  for (int row = 0; row < width_; ++row) {
    int star_col = -1;
    bool is_single = true;
    for (int col = 0; col < height_; ++col) {
      if (is_starred(row, col)) {
        if (star_col == -1) {
          star_col = col;
        } else {
          is_single = false;
          break;
        }
      }
    }
    if (!is_single) {
      for (int col = 0; col < height_; ++col) {
        unstar(row, col);
      }
    }
  }
}
// Step 1.
// For each row of the matrix, find the smallest element and subtract it
// from every element in its row.  Go to Step 2.
void HungarianOptimizer::reduce_rows() {
  for (int row = 0; row < matrix_size_; ++row) {
    float min_cost = costs_[row][0];
    for (int col = 1; col < matrix_size_; ++col) {
      min_cost = std::min(min_cost, costs_[row][col]);
    }
    for (int col = 0; col < matrix_size_; ++col) {
      costs_[row][col] -= min_cost;
    }
  }
  state_ = &HungarianOptimizer::star_zeroes;
}

// Step 2.
// Find a zero (Z) in the matrix.  If there is no starred zero in its row
// or column, star Z.  Repeat for every element in the matrix.  Go to step 3.
// of the CPU - the next slowest step takes 0.6%.  I can't think of a way
// of speeding it up though.
void HungarianOptimizer::star_zeroes() {
  // Since no rows or columns are covered on entry to this step, we use the
  // covers as a quick way of marking which rows & columns have stars in them.
  for (int row = 0; row < matrix_size_; ++row) {
    if (row_covered(row)) {
      continue;
    }

    for (int col = 0; col < matrix_size_; ++col) {
      if (col_covered(col)) {
        continue;
      }

      if (costs_[row][col] == 0) {
        star(row, col);
        cover_row(row);
        cover_col(col);
        break;
      }
    }
  }

  clear_covers();
  state_ = &HungarianOptimizer::cover_starred_zeroes;
}

// Step 3.
// Cover each column containing a starred zero.  If all columns are
// covered, the starred zeros describe a complete set of unique assignments.
// In this case, terminate the algorithm.  Otherwise, go to step 4.
void HungarianOptimizer::cover_starred_zeroes() {
  int num_covered = 0;

  for (int col = 0; col < matrix_size_; ++col) {
    if (col_contains_star(col)) {
      cover_col(col);
      num_covered++;
    }
  }

  if (num_covered >= matrix_size_) {
    state_ = NULL;
    return;
  }
  state_ = &HungarianOptimizer::prime_zeroes;
}

// Step 4.
// Find a noncovered zero and prime it.  If there is no starred zero in the
// row containing this primed zero, Go to Step 5.  Otherwise, cover this row
// and uncover the column containing the starred zero. Continue in this manner
// until there are no uncovered zeros left, then go to Step 6.

void HungarianOptimizer::prime_zeroes() {
  // This loop is guaranteed to terminate in at most matrix_size_ iterations,
  // as find_zero() returns a location only if there is at least one uncovered
  // zero in the matrix.  Each iteration, either one row is covered or the
  // loop terminates.  Since there are matrix_size_ rows, after that many
  // iterations there are no uncovered cells and hence no uncovered zeroes,
  // so the loop terminates.
  for (;;) {
    int zero_row = 0;
    int zero_col = 0;
    if (!find_zero(&zero_row, &zero_col)) {
      // No uncovered zeroes.
      state_ = &HungarianOptimizer::augment_path;
      return;
    }

    prime(zero_row, zero_col);
    int star_col = find_star_in_row(zero_row);

    if (star_col != kHungarianOptimizerColNotFound) {
      cover_row(zero_row);
      uncover_col(star_col);
    } else {
      preimage_[0] = zero_row;
      image_[0] = zero_col;
      state_ = &HungarianOptimizer::make_augmenting_path;
      return;
    }
  }
}

// Step 5.
// Construct a series of alternating primed and starred zeros as follows.
// Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote
// the starred zero in the column of Z0 (if any). Let Z2 denote the primed
// zero in the row of Z1 (there will always be one).  Continue until the
// series terminates at a primed zero that has no starred zero in its column.
// unstar each starred zero of the series, star each primed zero of the
// series, erase all primes and uncover every line in the matrix.  Return to
// Step 3.
void HungarianOptimizer::make_augmenting_path() {
  bool done = false;
  int count = 0;

  /* Note: this loop is guaranteed to terminate within matrix_size_ iterations
  // because:
  // 1) on entry to this step, there is at least 1 column with no starred zero
  //    (otherwise we would have terminated the algorithm already.)
  // 2) each row containing a star also contains exactly one primed zero.
  // 4) each column contains at most one starred zero.
  //
  // Since the path_ we construct visits primed and starred zeroes alternately,
  // and terminates if we reach a primed zero in a column with no star, our
  // path_ must either contain matrix_size_ or fewer stars (in which case the
  // loop iterates fewer than matrix_size_ times), or it contains more.  In
  // that case, because (1) implies that there are fewer than
  // matrix_size_ stars, we must have visited at least one star more than once.
  // Consider the first such star that we visit more than once; it must have
  // been reached immediately after visiting a prime in the same row.  By (2),
  // this prime is unique and so must have also been visited more than once.
  // Therefore, that prime must be in the same column as a star that has been
  // visited more than once, contradicting the assumption that we chose the
  // first multiply visited star, or it must be in the same column as more
  // than one star, contradicting (3).  Therefore, we never visit any star
  // more than once and the loop terminates within matrix_size_ iterations.
  */

  while (!done) {
    // First construct the alternating path...
    int row = find_star_in_col(image_[count]);

    if (row != kHungarianOptimizerRowNotFound) {
      count++;
      preimage_[count] = row;
      image_[count] = image_[count - 1];
    } else {
      done = true;
    }

    if (!done) {
      int col = find_prime_in_row(preimage_[count]);
      count++;
      preimage_[count] = preimage_[count - 1];
      image_[count] = col;
    }
  }

  // Then modify it.
  for (int i = 0; i <= count; ++i) {
    int row = preimage_[i];
    int col = image_[i];

    if (is_starred(row, col)) {
      unstar(row, col);
    } else {
      star(row, col);
    }
  }

  clear_covers();
  clear_primes();
  state_ = &HungarianOptimizer::cover_starred_zeroes;
}

// Step 6
// Add the smallest uncovered value in the matrix to every element of each
// covered row, and subtract it from every element of each uncovered column.
// Return to Step 4 without altering any stars, primes, or covered lines.
void HungarianOptimizer::augment_path() {
  float minval = find_smallest_uncovered();

  for (int row = 0; row < matrix_size_; ++row) {
    if (row_covered(row)) {
      for (int c = 0; c < matrix_size_; ++c) {
        costs_[row][c] += minval;
      }
    }
  }
  for (int col = 0; col < matrix_size_; ++col) {
    if (!col_covered(col)) {
      for (int r = 0; r < matrix_size_; ++r) {
        costs_[r][col] -= minval;
      }
    }
  }
  state_ = &HungarianOptimizer::prime_zeroes;
}


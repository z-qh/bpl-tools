/*
 * 用于建筑物点云中的平面，可视化，残差设计与调整等
 */

#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "yaml-cpp/yaml.h"
#include "CVC.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <queue>
#include <signal.h>
#include <list>
#include <climits>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/core/auto_differentiation.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"


using namespace std;

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (uint32_t, label, label)
)
typedef PointXYZIL PointType;

class TicToc{
public:
    TicToc(){tic();}
    void tic(){start = std::chrono::system_clock::now();}
    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


//线的参数
class LineParameter{
public:
    static void sortEigenVectorByValues(Eigen::Vector3d& eigenValues, Eigen::Matrix3d& eigenVectors) {
        vector<tuple<double, Eigen::Vector3d>> eigenValueAndVector;
        int sz = eigenValues.size();
        for (int i = 0; i < sz; ++i)
            eigenValueAndVector.emplace_back(tuple<double, Eigen::Vector3d>(eigenValues[i], eigenVectors.col(i)));
        
        // // 使用标准库中的sort，按从小到大排序
        static auto comp = [&](const tuple<double, Eigen::Vector3d>& a, const tuple<double, Eigen::Vector3d>& b)->bool
        {
            return std::get<0>(a) < std::get<0>(b);
        };
        std::sort(eigenValueAndVector.begin(), eigenValueAndVector.end(), comp);
        
        for (int i = 0; i < sz; ++i) {
            eigenValues[i] = std::get<0>(eigenValueAndVector[i]); // 排序后的特征值
            eigenVectors.col(i) = std::get<1>(eigenValueAndVector[i]); // 排序后的特征向量
        }
    }

public:
    pcl::PointCloud<PointType>::Ptr ori_cloud = nullptr;
    Eigen::Vector3d A = Eigen::Vector3d::Zero(), D = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_vec = Eigen::Vector3d::Zero(), min_vec = Eigen::Vector3d::Zero();
    double max_dis=-FLT_MAX, min_dis=FLT_MAX;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    LineParameter() = delete;
    LineParameter(pcl::PointCloud<PointType>::Ptr cloud){
        ori_cloud = cloud;
        fit_line_parameter(cloud);
    }
    LineParameter(const LineParameter& lp){
        this->A = lp.A;
        this->D = lp.D;
        this->ori_cloud = lp.ori_cloud;
        this->min_dis = lp.min_dis;
        this->max_dis = lp.max_dis;
        this->center = lp.center;
    }
    double DisToPoint(Eigen::Vector3d&x){
        double f = (x-A).norm();
        double s = (x-A).dot(D);
        return  sqrt(f*f - s*s);
    }
    using Ptr = std::shared_ptr<LineParameter>;
    friend std::ostream& operator<<(LineParameter&line, std::ostream& out);
private:
    void fit_line_parameter(pcl::PointCloud<PointType>::Ptr cloud){
        if(cloud == nullptr || cloud->empty())return;
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
        for(auto&p:cloud->points){
            a += Eigen::Vector3d(p.x, p.y, p.z);
        }
        a /= cloud->size();
        center = a;
        A = a;
        Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
        for(auto&p:cloud->points){
            Eigen::Vector3d x(p.x, p.y, p.z);
            // Eigen::Vector3d y = x - a;
            S += (x-a).transpose()*(x-a)*Eigen::Matrix3d::Identity() - (x-a)*(x-a).transpose();
        }
        Eigen::EigenSolver<Eigen::Matrix3d> solver(S);
        Eigen::Vector3d eigenValues = solver.pseudoEigenvalueMatrix().diagonal();
        Eigen::Matrix3d eigenVectors= solver.pseudoEigenvectors();
        sortEigenVectorByValues(eigenValues, eigenVectors);
        D = eigenVectors.col(0);
        D.normalize();
        // cout << "center: " << A[0] << " " << A[1] << " " << A[2] << endl;
        // cout << "direct: " << D[0] << " " << D[1] << " " << D[2] << endl;
        double a0 = A[0] + D[0] * (A[2]-0.f) / D[2];
        double a1 = A[1] + D[1] * (A[2]-0.f) / D[2];
        double a2 = 0.f;
        A[0]=a0, A[1]=a1, A[2]=a2;
        for(auto&p:cloud->points){
            Eigen::Vector3d pointx(p.x, p.y, p.z);
            double td = (pointx-A).dot(D);
            if(td < min_dis) min_dis = td, min_vec=pointx;
            if(td > max_dis) max_dis = td, max_vec=pointx;
        }
        // cout << "min dis :" << min_dis << endl;
        // cout << "max dis :" << max_dis << endl;
    }
};
std::ostream& operator<<(std::ostream& out, LineParameter&line){
    out << "C:  " << line.center[0] << " " << line.center[1] << " " << line.center[2] << endl;
    out << "A:  " << line.A[0] << " " << line.A[1] << " " << line.A[2] << endl;
    out << "D:  " << line.D[0] << " " << line.D[1] << " " << line.D[2] << endl;
    out << "V1: " << line.min_vec[0] << " " << line.min_vec[1] << " " << line.min_vec[2] << endl;
    out << "V2: " << line.max_vec[0] << " " << line.max_vec[1] << " " << line.max_vec[2] << endl;
    return out;
}

// 柱面拟合残差
class CylinderFitFactor{
public:
    CylinderFitFactor(Eigen::Vector3d X_):X(X_){}
    template<typename T>
    bool operator()(const T*D, const T*A, const T*r, T*residual) const
    {
        Eigen::Matrix<T,3,1> x{T(X[0]), T(X[1]), T(X[2])};
        Eigen::Matrix<T,3,1> d{D[0], D[1], D[2]};
        Eigen::Matrix<T,3,1> a{A[0], A[1], A[2]};
        // T f = (x-a).transpose()*(x-a); 
        // T s = d.transpose()*(x-a);
        // T l = f - s * s - r[0];
        // residual[0] = T(l);
        T f = (x-a).norm();
        T s = (x-a).dot(d) / d.norm();
        residual[0] = T(sqrt(f*f - s*s) - r[0]);
        // Eigen::Vector3d aaa;
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d X_){
        return (new ceres::AutoDiffCostFunction<CylinderFitFactor, 1, 3, 3, 1>(
            new CylinderFitFactor(X_)
        ));
    }
    Eigen::Vector3d X;
};

// 柱面拟合残差
class CylinderLineFitFactor{
public:
    CylinderLineFitFactor(Eigen::Vector3d X_, LineParameter::Ptr line_):X(X_), line(line_){}
    template<typename T>
    bool operator()(const T*r, T*residual) const
    {
        Eigen::Matrix<T,3,1> x{T(X[0]), T(X[1]), T(X[2])};
        Eigen::Matrix<T,3,1> d{T(line->D[0]), T(line->D[1]), T(line->D[2])};
        Eigen::Matrix<T,3,1> a{T(line->A[0]), T(line->A[1]), T(line->A[2])};
        // T f = (x-a).transpose()*(x-a); 
        // T s = d.transpose()*(x-a);
        // T l = f - s * s - r[0];
        // residual[0] = T(l);
        T f = (x-a).norm();
        T s = (x-a).dot(d) / d.norm();
        residual[0] = T(sqrt(f*f - s*s) - r[0]);
        // Eigen::Vector3d aaa;
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d X_, LineParameter::Ptr line_){
        return (new ceres::AutoDiffCostFunction<CylinderLineFitFactor, 1, 1>(
            new CylinderLineFitFactor(X_, line_)
        ));
    }
    Eigen::Vector3d X;
    LineParameter::Ptr line;
};

// 这种方式拟合会发散, 加了中心之后就不发散了 
class CylinderParameter{
public:
    static pcl::PointCloud<PointType>::Ptr generateCylinderCloud(CylinderParameter& cylinder){
        Eigen::Quaterniond oriq = cylinder.Q;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if(cylinder.min_dis == FLT_MAX || cylinder.max_dis == -FLT_MAX) return cloud;
        const static double theta_d = (PI/30.0);
        const static double line_d = 0.1;
        for(double theta = -PI; theta < PI; theta+=theta_d){
            Eigen::Vector3d randp;
            randp[0] = cylinder.r * cos(theta);
            randp[1] = cylinder.r * sin(theta);
            for(double dis = cylinder.min_dis; dis < cylinder.max_dis; dis+=line_d){
                randp[2] = dis;
                PointType tmp;
                tmp.x = randp[0];tmp.y = randp[1];tmp.z = randp[2];
                cloud->points.emplace_back(tmp);
            }
        }
        for(auto&p:cloud->points){
            Eigen::Vector3d np(p.x, p.y, p.z);
            Eigen::Vector3d cp = oriq * np + cylinder.A;
            p.x = cp[0], p.y = cp[1], p.z=cp[2];
        }
        return cloud;
    }
    static Eigen::Quaterniond fromtwovectors(Eigen::Vector3d to, Eigen::Vector3d from=Eigen::Vector3d::UnitZ()){
        Eigen::Vector3d w = to.cross(from);
        Eigen::Quaterniond q(1.0f+to.dot(from), w[0], w[1], w[2]);
        q.normalize();
        return q;
    }
public:
    bool valid=false;
    Eigen::Vector3d D = Eigen::Vector3d::Zero(), A = Eigen::Vector3d::Zero();
    Eigen::Quaterniond Q = Eigen::Quaterniond::Identity();
    double r=0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    pcl::PointCloud<PointType>::Ptr ori_cloud = nullptr;
    Eigen::Vector3d max_vec = Eigen::Vector3d::Zero(), min_vec = Eigen::Vector3d::Zero();
    double min_dis=FLT_MAX, max_dis=-FLT_MAX;
    CylinderParameter() = delete;
    CylinderParameter(const CylinderParameter& c){
        this->D = c.D;
        this->A = c.A;
        this->r = c.r;
        this->Q = c.Q;
        this->min_dis = c.min_dis;
        this->max_dis = c.max_dis;
        this->ori_cloud = c.ori_cloud;
        this->center = c.center;
        this->max_vec = c.max_vec;
        this->min_vec = c.min_vec;
        this->valid = c.valid;
    }
    CylinderParameter(const pcl::PointCloud<PointType>::Ptr cloud, LineParameter::Ptr line){
        if(cloud->empty()) return ;
        ori_cloud = cloud;
        fitting_cylinder(cloud, line);
    }
    using Ptr = std::shared_ptr<CylinderParameter>;
    friend std::ostream& operator<<(std::ostream& out, CylinderParameter&cylinder);
public:
    Eigen::Vector3d translateA(Eigen::Matrix3d rot_, Eigen::Vector3d t_){
        Eigen::Quaterniond q(Q.matrix().transpose() * rot_ * Q.matrix());
        Eigen::Vector3d d = q * Eigen::Vector3d::UnitZ();
        d.normalize();
        // Eigen::Vector3d c = rot_ * center + t_;
        // Eigen::Vector3d ma_vec = rot_ * max_vec + t_;
        // Eigen::Vector3d mi_vec = rot_ * min_vec + t_;
        Eigen::Vector3d a = rot_ * A + t_;
        double a0 = a[0] + d[0] * (a[2]-0.f) / d[2];
        double a1 = a[1] + d[1] * (a[2]-0.f) / d[2];
        double a2 = 0.f;
        a[0]=a0, a[1]=a1, a[2]=a2;
        // double dis1 = (ma_vec-a).dot(d);
        // double dis2 = (mi_vec-a).dot(d);
        // double ma_dis = max(dis1, dis2);
        // double mi_dis = min(dis1, dis2);
        return a;
    }
    CylinderParameter::Ptr translate(Eigen::Matrix3d rot_, Eigen::Vector3d t_){
        // translate error here 
        CylinderParameter::Ptr ans(new CylinderParameter(*this));
        Eigen::Vector3d d = rot_ * D;
        d.normalize();
        Eigen::Quaterniond q(fromtwovectors(d));
        // Eigen::Quaterniond q(Q.matrix().transpose() * rot_ * Q.matrix());
        // Eigen::Vector3d d = q * Eigen::Vector3d::UnitZ();
        d.normalize();
        Eigen::Vector3d c = rot_ * center + t_;
        Eigen::Vector3d ma_vec = rot_ * max_vec + t_;
        Eigen::Vector3d mi_vec = rot_ * min_vec + t_;
        Eigen::Vector3d a;
        double a0 = c[0] + d[0] * (c[2]-0.f) / d[2];
        double a1 = c[1] + d[1] * (c[2]-0.f) / d[2];
        double a2 = 0.f;
        a[0]=a0, a[1]=a1, a[2]=a2;
        double dis1 = (ma_vec-a).dot(d);
        double dis2 = (mi_vec-a).dot(d);
        double ma_dis = max(dis1, dis2);
        double mi_dis = min(dis1, dis2);
        ans->A = a;
        ans->center = c;
        ans->D = d;
        ans->max_dis = ma_dis;
        ans->min_dis = mi_dis;
        ans->max_vec = ma_vec;
        ans->min_vec = mi_vec;
        ans->Q = q;
        ans->r = r;
        return ans;
    }
private:
    void fitting_cylinder(const pcl::PointCloud<PointType>::Ptr cloud, LineParameter::Ptr preline){
        // cout << "start fitting " << endl;
        ceres::Problem::Options problem_option;
        ceres::Problem problem(problem_option);
        double parameter[7] = {0,0,1, 0,0,0, 0};
        problem.AddParameterBlock(parameter, 3);
        problem.AddParameterBlock(parameter+3, 3);
        problem.AddParameterBlock(parameter+6, 1);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(1.5);
        // Eigen::Vector3d tmpcenter = Eigen::Vector3d::Zero();
        for(size_t i = 0; i < cloud->size(); ++i){
            Eigen::Vector3d pointx(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            // tmpcenter += pointx;
            // ceres::CostFunction * cost_function = CylinderFitFactor::Create(pointx);
            // problem.AddResidualBlock(cost_function, loss_function, parameter, parameter+3, parameter+6);

            ceres::CostFunction *cost_function = CylinderLineFitFactor::Create(pointx, preline);
            problem.AddResidualBlock(cost_function, loss_function, parameter+6);
        }
        // tmpcenter /= cloud->size();
        // parameter[3] = tmpcenter[0];
        // parameter[4] = tmpcenter[1];
        // parameter[5] = tmpcenter[2];
        ceres::Solver::Options option;
        option.linear_solver_type = ceres::DENSE_QR;
        option.max_num_iterations = 40;
        option.minimizer_progress_to_stdout = false;
        option.check_gradients = false;
        option.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(option, &problem, &summary);
        // cout << summary.FullReport() << endl;
        // D = Eigen::Vector3d(parameter[0], parameter[1], parameter[2]);
        // D.normalize();
        // r = parameter[6];
        // A = Eigen::Vector3d(parameter[3], parameter[4], parameter[5]);
        // 
        this->center = preline->center;
        this->D = preline->D;
        this->A = preline->A;
        this->r = parameter[6];
        this->Q = fromtwovectors(D);
        // cout << "center: " << endl << A << endl;
        // cout << "dir :   " << endl << D << endl;
        // cout << "radius: " << endl << r << endl;
        // cout << "end " << endl;
        // 找方向向量上的最小点最大点用以确定轮廓
        for(size_t i = 0; i < cloud->size(); ++i){
            Eigen::Vector3d pointx(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            double tmp_v = (pointx-A).dot(D);
            if(tmp_v < min_dis) min_dis=tmp_v,min_vec=pointx;
            if(tmp_v > max_dis) max_dis=tmp_v,max_vec=pointx;
        }
        // cout << "min: " << min_dis << endl;
        // cout << "max: " << max_dis << endl << endl;

        cout << "final cost " << summary.final_cost << endl;
        if(summary.final_cost<0.2) cout << "#############" << endl;
        else {
            cout << "!!!!!!!!!!!!" << endl; 
            static int bad_id = 1;
            static string pathBase = "/home/qh/ins_map_temp/badcylinder/";
            string log_file_name, shape_cloud_file_name, ori_cloud_file_name;
            log_file_name = pathBase + to_string(bad_id) + ".log";
            shape_cloud_file_name = pathBase + to_string(bad_id) + "shape.pcd";
            ori_cloud_file_name = pathBase + to_string(bad_id) + "ori.pcd";
            auto shape_cloud = CylinderParameter::generateCylinderCloud(*this);
            auto ori_cloud = this->ori_cloud;
            pcl::io::savePCDFileASCII(shape_cloud_file_name, *shape_cloud);
            pcl::io::savePCDFileASCII(ori_cloud_file_name, *ori_cloud);
        }
    }
};

std::ostream& operator<<(std::ostream& out, CylinderParameter&cylinder){
    out << "R  " << cylinder.r << endl;
    out << "C  " << cylinder.center[0] << " " << cylinder.center[1] << " " << cylinder.center[2] << endl;
    out << "A  " << cylinder.A[0] << " " << cylinder.A[1] << " " << cylinder.A[2] << endl;
    out << "D  " << cylinder.D[0] << " " << cylinder.D[1] << " " << cylinder.D[2] << endl;
    out << "V1 " << cylinder.min_vec[0] << " " << cylinder.min_vec[1] << " " << cylinder.min_vec[2] << endl;
    out << "V2 " << cylinder.max_vec[0] << " " << cylinder.max_vec[1] << " " << cylinder.max_vec[2] << endl;
    return out;
}
//点到线的残差
class LidarLineFactor{
public:
    LidarLineFactor(Eigen::Vector3d curr_point, LineParameter line, double s_):X(curr_point),A(line.A),D(line.D),s(s_){}
    template<typename T>
    bool operator()(const T*q, const T*t, T*residual) const
    {
        Eigen::Matrix<T,3,1> x{T(X[0]), T(X[1]), T(X[2])};
        Eigen::Matrix<T,3,1> a{T(A[0]), T(A[1]), T(A[2])};
        Eigen::Matrix<T,3,1> d{T(D[0]), T(D[1]), T(D[2])};
        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        Eigen::Matrix<T,3,1> t_last_curr{T(s)*t[0], T(s)*t[1], T(s)*t[2]};
        Eigen::Matrix<T,3,1> lx;
        lx = q_last_curr * x + t_last_curr;
        // Eigen::Matrix<T,3,1> dv = (lx-a) - (lx-a).transpose()*d*d;
        // residual[0] = dv[0] / dv.norm();
        // residual[1] = dv[1] / dv.norm();
        // residual[2] = dv[2] / dv.norm();
        T f = (lx-a).norm();
        T s = (lx-a).dot(d);
        residual[0] = T(sqrt(f*f - s*s));
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const LineParameter line_, double s_){
        // return (new ceres::AutoDiffCostFunction<LidarLineFactor, 3, 4, 3>(
        return (new ceres::AutoDiffCostFunction<LidarLineFactor, 1, 4, 3>(
            new LidarLineFactor(curr_point_, line_, s_)
        ));
    }
    Eigen::Vector3d X,A,D;
    double s;
};
// 点到柱面的残差
class LidarCylinderFactor{
public:
    LidarCylinderFactor(Eigen::Vector3d curr_point, CylinderParameter cylinder, double s_)
    :P(curr_point), A(cylinder.A), D(cylinder.D), R(cylinder.r), s(s_){}
    template<typename T>
    bool operator()(const T*q, const T*t, T*residual) const
    {
        Eigen::Quaternion<T> q_lsat_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        Eigen::Matrix<T,3,1> t_last_curr{T(s)*t[0], T(s)*t[1], T(s)*t[2]};
        Eigen::Matrix<T,3,1> cp{T(P[0]), T(P[1]), T(P[2])};
        Eigen::Matrix<T,3,1> lp;
        lp = q_lsat_curr * cp + t_last_curr;
        Eigen::Matrix<T,3,1> x{lp[0]-T(A[0]), lp[1]-T(A[1]), lp[2]-T(A[2])};
        Eigen::Matrix<T,3,1> d{T(D[0]), T(D[1]), T(D[2])};
        T f = x.norm();
        T s = x.dot(d);
        T l = T(sqrt(f*f- s*s) - T(R));
        residual[0] = T(l);
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const CylinderParameter cylinder_, double s_){
        return (new ceres::AutoDiffCostFunction<LidarCylinderFactor, 1, 4, 3>(
            new LidarCylinderFactor(curr_point_, cylinder_, s_)
        ));
    }
    Eigen::Vector3d P,A,D;
    double R;
    double s;
};

class Pole{
public:
    static double disFromTwoCylinder(CylinderParameter::Ptr&a, CylinderParameter::Ptr&b){
        double s1 = sqrt(pow((a->A-b->A).norm(), 2) - pow((a->A-b->A).dot(b->D),2)) * (b->max_dis-b->min_dis);
        double s2 = sqrt(pow((b->A-a->A).norm(), 2) - pow((b->A-a->A).dot(a->D),2)) * (a->max_dis-a->min_dis);
        return s1+s2;
    }
    static double dirFromTwoCylinder(CylinderParameter::Ptr&a,CylinderParameter::Ptr&b){
        return acos(a->A.dot(b->A));
    }
    static double rFromTwoClinder(CylinderParameter::Ptr&a,CylinderParameter::Ptr&b){
        return abs(a->r - b->r) / b->r;
    }
public:
    CylinderParameter::Ptr cylinder = nullptr;
    int id = 0;
    double timestamped = 0;
    int survival_time = 0;
    int visiable_time = 0;
    int assocate_count = 0;
    Pole() = delete;
    Pole(CylinderParameter::Ptr c, int id_, double time){
        id = id_;
        timestamped = time;
        cylinder = c;
    }
    Pole(const Pole& p){
        id = p.id;
        timestamped = p.timestamped;
        survival_time = p.survival_time;
        assocate_count = p.assocate_count;
        visiable_time = p.visiable_time;
    }
    using Ptr = std::shared_ptr<Pole>;
};

class PoseType{
public:
    int odom_id;
    shared_ptr<Eigen::Isometry3d> odom;
    PoseType() = delete;
    PoseType(int id_, shared_ptr<Eigen::Isometry3d> o_):odom_id(id_), odom(o_){};
    PoseType(const PoseType& p){
        this->odom_id = p.odom_id;
        this->odom = p.odom;
    } 
    using Ptr = shared_ptr<PoseType>;
};

class PoleConstraint{
public:
    int pole_id;
    Eigen::Vector3d pre_center;
    shared_ptr<vector<PoseType::Ptr>> pose_vec;
    shared_ptr<vector<CylinderParameter::Ptr>> measurement_vec;
    PoleConstraint() = delete;
    PoleConstraint(int p_id):pole_id(p_id){
        pre_center = Eigen::Vector3d::Zero();
        pose_vec.reset(new vector<PoseType::Ptr>());
        measurement_vec.reset(new vector<CylinderParameter::Ptr>());
    }
    PoleConstraint(const PoleConstraint& pc){
        this->pole_id = pc.pole_id;
        this->pose_vec = pc.pose_vec;
        this->measurement_vec = pc.measurement_vec;
    }
    Eigen::Vector3d getCenter() const 
    {
        Eigen::Vector3d ans(pre_center);
        ans /= measurement_vec->size();
        return ans;
    }
    using Ptr = shared_ptr<PoleConstraint>;
};




class CylinderMapManager{
public:
    static double disFromTwoPosi(Eigen::Vector3d lposi, Eigen::Vector3d nposi){
        return (lposi - nposi).norm();
    }
public:
    unordered_map<int, Pole::Ptr> global_map;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreePoleCenterMap;
    pcl::PointCloud<PointType>::Ptr PoleCenterCloud;
    list<list<pair<PoseType::Ptr, CylinderParameter::Ptr>>> BAAssocate; 
    list<list<int>> BAAssocateInd;
    int used_id;
    int odom_id;
    bool initial = false;
    CylinderMapManager(){
        odom_id = -1;
        used_id = 0;
        kdtreePoleCenterMap.reset(new pcl::KdTreeFLANN<PointType>());
        PoleCenterCloud.reset(new pcl::PointCloud<PointType>());
    }


    void addNewLandmark(std::vector<CylinderParameter::Ptr>& landmarks, double timestamp, nav_msgs::Odometry odom){
        list<pair<PoseType::Ptr, CylinderParameter::Ptr>> nowBAAssocate;
        list<int> nowBAAssocateInd;
        shared_ptr<Eigen::Isometry3d> pose(new Eigen::Isometry3d(Eigen::Isometry3d::Identity()));
        Eigen::Vector3d nposi(odom.pose.pose.position.x, 
                              odom.pose.pose.position.y, 
                              odom.pose.pose.position.z);
        pose->translate(nposi);
        Eigen::Matrix3d nori(Eigen::Quaterniond(odom.pose.pose.orientation.w,
                                                odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z));
        pose->rotate(nori);
        PoseType::Ptr nPoseType(new PoseType(odom_id--, pose));

        pcl::PointCloud<PointType>::Ptr nowPoleCenterCloud(new pcl::PointCloud<PointType>());
        // 删除比较差的map点
        cout << "remove bad mp" << endl;

        for(unordered_map<int, Pole::Ptr>::iterator it = global_map.begin(); it != global_map.end();){
            // no match and too old
            if(it->second->assocate_count == 0 && timestamp -it->second->timestamped > 10.0f){
                global_map.erase(it++);
                continue;
            }
            // no match ans too faraway
            if(it->second->assocate_count == 0 && disFromTwoPosi(it->second->cylinder->center, nposi) > 80.0){
                global_map.erase(it++);
                continue;
            }
            // too few visiable frames 
            if(it->second->survival_time * 1.0 / it->second->visiable_time < 0.2){
                global_map.erase(it++);
                continue;
            }
            it->second->survival_time++;
            it++;
        }
        PoleCenterCloud->clear();
        cout << "get new map cloud" << endl;
        for(auto&p:global_map){
            PointType np;
            np.x = p.second->cylinder->A[0];
            np.y = p.second->cylinder->A[1];
            np.z = p.second->cylinder->A[2];
            np.intensity = p.first;
            PoleCenterCloud->emplace_back(np);
        }
        if(!PoleCenterCloud->empty()) kdtreePoleCenterMap->setInputCloud(PoleCenterCloud);
        // 找对应
        cout << "get assocate mp" << endl;
        for(auto&l:landmarks){
            PointType np;
            np.x = l->A[0];
            np.y = l->A[1];
            np.z = l->A[2];
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            if(!PoleCenterCloud->empty()) kdtreePoleCenterMap->radiusSearch(np, 2.0, pointSearchInd, pointSearchSqDis);
            if(pointSearchInd.empty()){
                // 找不到对应则加入地图中
                Pole::Ptr newPole(new Pole(l, used_id, timestamp));
                global_map.insert({used_id, newPole});
                used_id++;
            }else{
                int min_s_id = -1;
                Pole::Ptr min_s_pole = nullptr;
                double min_s = FLT_MAX;
                for(auto&ind:pointSearchInd){
                    double tmps = Pole::disFromTwoCylinder(l, global_map[PoleCenterCloud->points[ind].intensity]->cylinder);
                    if(tmps < min_s){
                        min_s = tmps;
                        min_s_pole = global_map[PoleCenterCloud->points[ind].intensity];
                        min_s_id = PoleCenterCloud->points[ind].intensity;
                    }
                }
                if(min_s_pole == nullptr){
                    Pole::Ptr newPole(new Pole(l, used_id, timestamp));
                    global_map.insert({used_id, newPole});
                    used_id++;
                    continue;
                }
                // 对比已关联的质量，去除质量比较差的关联，保留质量比较好的
                double dir_angle = Pole::dirFromTwoCylinder(l, min_s_pole->cylinder);
                double r_dis = Pole::rFromTwoClinder(l, min_s_pole->cylinder);
                if(r_dis > 0.2 || dir_angle > 0.35){
                    Pole::Ptr newPole(new Pole(l, used_id, timestamp));
                    global_map.insert({used_id, newPole});
                    used_id++;
                    continue;
                }else{
                    // 记录BA参数
                    nowBAAssocate.emplace_back(nPoseType, global_map[min_s_id]->cylinder);
                    nowBAAssocateInd.emplace_back(min_s_id);
                    // 已经关联的更新cylinder以及Pole关联参数，
                    min_s_pole->assocate_count++;
                    min_s_pole->visiable_time++;
                    global_map[min_s_id]->cylinder = l;
                }
            }
        }
        if(!nowBAAssocate.empty()) BAAssocate.emplace_back(move(nowBAAssocate));
        if(!nowBAAssocateInd.empty()) BAAssocateInd.emplace_back(move(nowBAAssocateInd));
        cout << "add end " << endl;
    }
    // 因为同一个Pole关联后会更新cylinder，于是这里返回的cylinder，但是还是需要Pole的信息以表示是否是同一个pole，然后也需要不同的cylinder提供观测。
    vector<PoleConstraint::Ptr> getBAAssocate(){
        unordered_map<int, PoleConstraint::Ptr> pole_constraint_dic;
        vector<PoleConstraint::Ptr> ans;
        auto itframeba = BAAssocate.begin();
        auto itframebaid = BAAssocateInd.begin(); 
        for(; itframeba != BAAssocate.end() && itframebaid != BAAssocateInd.end(); itframeba++,itframebaid++){
            auto itpba = itframeba->begin();
            auto itpbaid = itframebaid->begin();
            // 查这一对BA的ID是否在Map中存在，如果因为劣质导致删除那么就不能提供BA，ID对应唯一的Pole，但是不能用Pole中的Cylinder， 只能用以前存起来的Cylinder
            for(; itpba != itframeba->end() && itpbaid != itframebaid->end(); itpba++, itpbaid++){
                if(global_map.count(*itpbaid) == 1){
                    if(pole_constraint_dic.count(*itpbaid) != 1) pole_constraint_dic[*itpbaid] = PoleConstraint::Ptr(new PoleConstraint(*itpbaid));
                    auto &ba_constraint = pole_constraint_dic[*itpbaid];
                    ba_constraint->pole_id = *itpbaid;
                    ba_constraint->pose_vec->emplace_back(itpba->first);
                    ba_constraint->measurement_vec->emplace_back(itpba->second);
                    ba_constraint->pre_center += itpba->second->A;
                }
            }
        }
        for(auto&p:pole_constraint_dic){
            ans.emplace_back(p.second);
        }
        return ans;
    }
    void updatepose(){
        auto baassocate = this->getBAAssocate();
        using BlockSolverTpye = g2o::BlockSolverPL<6, 3>;
        using LinearSolverType = g2o::LinearSolverDense<BlockSolverTpye::PoseMatrixType>;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverTpye>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);
        g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
        cameraOffset->setOffset();
        cameraOffset->setId(0);
        optimizer.addParameter(cameraOffset);
        int vertex_cylinder_id = 1;
        int vertex_pose_id = baassocate.size()+1;
        unordered_set<int> pose_vertex_id_dic, cylinder_vertex_id_dic;
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                auto&pose=*(*perba->pose_vec)[i];
                // auto&cylinder=(*perba->measurement_vec)[i];
                if(pose_vertex_id_dic.count(pose.odom_id) != 1){
                    // 位姿是顶点
                    g2o::VertexSE3 * vSE3_w = new g2o::VertexSE3();
                    vSE3_w->setEstimate(g2o::SE3Quat(pose.odom->rotation().cast<double>(),pose.odom->translation().cast<double>()));
                    vSE3_w->setId(vertex_pose_id);
                    vSE3_w->setFixed(false);
                    optimizer.addVertex(vSE3_w);
                    pose.odom_id = vertex_pose_id;
                    vertex_pose_id++;
                }
            }
            if(cylinder_vertex_id_dic.count(perba->pole_id) == 1) continue;
            if(perba->measurement_vec->empty()) continue;
            Eigen::Vector3d avg_posi_world = perba->getCenter();
            // 路标点是顶点
            g2o::VertexPointXYZ * vMP_w = new g2o::VertexPointXYZ();
            vMP_w->setEstimate(avg_posi_world.cast<double>());
            vMP_w->setId(vertex_cylinder_id);
            vMP_w->setFixed(false);
            optimizer.addVertex(vMP_w);
            perba->pole_id = vertex_cylinder_id;
            vertex_cylinder_id++;
        }
        int edge_id = vertex_pose_id + 1;
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                auto&pose=*(*perba->pose_vec)[i];
                auto&cylinder=(*perba->measurement_vec)[i];
                g2o::EdgeSE3PointXYZ * edge_pose_cylinder = new g2o::EdgeSE3PointXYZ();
                edge_pose_cylinder->setId(edge_id);
                edge_pose_cylinder->setParameterId(0, 0);
                edge_pose_cylinder->setVertex(pose.odom_id, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(pose.odom_id)));
                edge_pose_cylinder->setVertex(perba->pole_id, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(perba->pole_id)));
                auto local_measurement = cylinder->translateA(pose.odom->rotation().transpose(),(-1)*pose.odom->rotation().transpose()*pose.odom->translation());
                edge_pose_cylinder->setMeasurement(local_measurement.cast<double>());

            }
        }

        
        
        BAAssocate.clear();
        BAAssocateInd.clear();
    }
};


// get BA
CylinderMapManager cylinderManager;
vector<PointType> odompath;


pcl::PointCloud<PointType>::Ptr createLineCLoud(PointType A, PointType B)
{
    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    double diffX = A.x - B.x;
    double diffY = A.y - B.y;
    double diffZ = A.z - B.z;
    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    const static double line_d = 1 / 0.1;
    double nums = distance * line_d;
    for(int i = 0; i < nums; i++)
    {
        PointType tempPoint;
        tempPoint.x = B.x + diffX / nums * i;
        tempPoint.y = B.y + diffY / nums * i;
        tempPoint.z = B.z + diffZ / nums * i;
        result->push_back(tempPoint);
    }
    return result;
}

void exitSavedata(){
    // save ba graph 
    cout << "save data" << endl;
    TicToc showBatime;
    auto baassocate = cylinderManager.getBAAssocate();
    string saveBase = "/home/qh/ins_map_temp/cylinderMap/";
    pcl::PointCloud<PointType>::Ptr path_node(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr assline_cylinder(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr assline(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr assline_ori(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr assline_lcc(new pcl::PointCloud<PointType>());
    
    for(auto&perba:baassocate){
        for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
            auto&pose=*(*perba->pose_vec)[i];
            auto&cylinder=(*perba->measurement_vec)[i];
            int label_id = perba->pole_id;
            PointType cc, oc;
            cc.x = cylinder->center[0],cc.y = cylinder->center[1],cc.z = cylinder->center[2];
            oc.x = pose.odom->translation()[0],oc.y = pose.odom->translation()[1],oc.z = pose.odom->translation()[2];
            // printf("get cc %f,%f,%f  oc %f,%f,%f\n", cc.x, cc.y, cc.z, oc.x, oc.y, oc.z);
            *assline += *createLineCLoud(cc, oc);
            *assline_cylinder += *CylinderParameter::generateCylinderCloud(*cylinder);
            *assline_ori += *(cylinder->ori_cloud);
            PointType lcc;
            lcc.x=cylinder->A[0],lcc.y=cylinder->A[1],lcc.z=cylinder->A[2];
            lcc.intensity = label_id, lcc.label = label_id;
            // lcc.intensity = cylinder->
            assline_lcc->emplace_back(lcc);
        }
    }
    for(auto&p:odompath){
        path_node->emplace_back(p);   
    }
    if(!path_node->empty()) pcl::io::savePCDFileASCII(saveBase+"path_node.pcd", *path_node);
    if(!assline_cylinder->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_cylinder.pcd", *assline_cylinder);
    if(!assline->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate.pcd", *assline);
    if(!assline_ori->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_ori.pcd", *assline_ori);
    if(!assline_lcc->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_lcc.pcd", *assline_lcc);
    printf("show Ba time %f\n", showBatime.toc());
    cout << "data saved" << endl;
}



///////////////////////////////////////////////////////////
std::string odomTopic = "/gt";
Eigen::Matrix4d gt2lidar = Eigen::Matrix4d::Identity();
std::queue<nav_msgs::Odometry> laserOdomQueue;
ros::Publisher line_pub, cylinder_pub;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;
ros::Publisher pub_pcl_cluster_frame, pub_pcl_interest, pub_pcl_map, pub_ba_map;
image_transport::Publisher imgPub;
std::vector<bool>  interest_labels;
std::vector<pcl::PointCloud<PointType>::Ptr> last_cluster_cloud, now_cluster_cloud;
std::vector<LineParameter::Ptr> last_cluster_parameter_line, now_cluster_parameter_line;
std::vector<CylinderParameter::Ptr> now_cluster_parameter_cylinder, last_cluster_parameter_cylinder;
std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudArray;
///////////////////////////////////////////////////////////
std::vector<bool> getInterest(std::string file_name);
pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel);
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec);
void process(const sensor_msgs::PointCloud2ConstPtr &rec, nav_msgs::Odometry& odom);
void laserOdomHandler(nav_msgs::Odometry odom);
int main(int argc, char** argv){
    ros::init(argc, argv, "pole");
    ros::NodeHandle nh;
    gt2lidar << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    ros::Subscriber subLaserOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, laserOdomHandler);
    line_pub = nh.advertise<pcl::PointCloud<PointType>>("/pole/line", 1);
    cylinder_pub = nh.advertise<pcl::PointCloud<PointType>>("/pole/cylinder", 1);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    pub_pcl_cluster_frame = nh.advertise<pcl::PointCloud<PointType>>("/pole/cluster_frame", 1);
    pub_ba_map = nh.advertise<pcl::PointCloud<PointType>>("pole/ba_map", 1);
    pub_pcl_interest = nh.advertise<pcl::PointCloud<PointType>>("/pole/interest_points", 1);
    pub_pcl_map = nh.advertise<pcl::PointCloud<PointType>>("/pole/map", 1);
    image_transport::ImageTransport it(nh);
    imgPub = it.advertise("/pole/img", 10);
    interest_labels = getInterest("/home/qh/qh_ws/src/bpl-tools/bpltool/src/source/kitti_data_interest_pole.yaml");
    ros::Rate loop(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
            continue;
        while (abs(laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec()) > 0.05)
        {
            if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
                break;
            if (laserOdomQueue.front().header.stamp.toSec() < laserCloudMsgQueue.front()->header.stamp.toSec())
                laserOdomQueue.pop();
            else
                laserCloudMsgQueue.pop();
        }
        if (laserOdomQueue.empty() || laserCloudMsgQueue.empty())
            continue;
        if (abs(laserOdomQueue.front().header.stamp.toSec() - laserCloudMsgQueue.front()->header.stamp.toSec()) < 0.05)
        {
            process(laserCloudMsgQueue.front(), laserOdomQueue.front());
            laserCloudMsgQueue.pop();
            laserOdomQueue.pop();
            last_cluster_cloud.swap(now_cluster_cloud);
            last_cluster_parameter_line.swap(now_cluster_parameter_line);
            last_cluster_parameter_cylinder.swap(now_cluster_parameter_cylinder);
        }
        loop.sleep();
    }
    // exitSavedata();
    return 0;
}



void line_extrac(pcl::PointCloud<PointType>::Ptr interest_points){
    
}

// 用来优化的位姿参数
static double parameters[7] = {0,0,0,1, 0,0,0};
// 世界系位姿
static Eigen::Quaterniond q_w_curr(parameters[3], parameters[0], parameters[1], parameters[2]); 
static Eigen::Vector3d t_w_curr(parameters[4], parameters[5], parameters[6]);
static Eigen::Matrix3d r_w_curr(q_w_curr);
static Eigen::Matrix4d T_w_curr = Eigen::Matrix4d::Identity();
void pointAssociateToMap(PointType &pi, PointType &po)
{
	Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
	Eigen::Vector3d point_w = r_w_curr * point_curr + t_w_curr;
	po.x = point_w.x();
	po.y = point_w.y();
	po.z = point_w.z();
	po.intensity = pi.intensity;
    po.label = pi.label;
	//po->intensity = 1.0;
}

void pointAssociateToT(PointType &pi, Eigen::Matrix3d r, Eigen::Vector3d t)
{
	Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
	Eigen::Vector3d point_w = r * point_curr + t;
	pi.x = point_w.x();
	pi.y = point_w.y();
	pi.z = point_w.z();
	pi.intensity = pi.intensity;
    pi.label = pi.label;
}


void process(const sensor_msgs::PointCloud2ConstPtr &rec, nav_msgs::Odometry& odom){
    TicToc time_rg;
    PointType odom_point;
    odom_point.x = odom.pose.pose.position.x;
    odom_point.y = odom.pose.pose.position.y;
    odom_point.z = odom.pose.pose.position.z;
    odompath.emplace_back(odom_point);
    double timestamp = rec->header.stamp.toSec();
    Eigen::Quaterniond tmp_q_w_curr(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    Eigen::Matrix3d tmp_r_w_curr(tmp_q_w_curr);
    Eigen::Vector3d tmp_t_w_curr(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    pcl::PointCloud<PointType>::Ptr rec_point(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*rec, *rec_point);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*rec_point, *rec_point, indices);
    if(rec_point->points.empty()) return;
    auto points_interest_serial = GetFilteredInterestSerial(rec_point, interest_labels);
    if( points_interest_serial!=nullptr && points_interest_serial->points.empty()) return;

    // 发布语义兴趣点
    pcl::PointCloud<PointType>::Ptr points_interest(new pcl::PointCloud<PointType>());
    for(auto&p:points_interest_serial->points) points_interest->emplace_back(p);
    points_interest->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_interest->header.frame_id = "map";
    pub_pcl_interest.publish(points_interest);

    // cluster
    //构建分类器
    vector<PointAPR> papr;
    calculateAPR(*points_interest_serial, papr);
    unordered_map<int, Voxel> hvoxel;
    build_hash_table(papr, hvoxel);
    vector<int> cluster_index = CVC(hvoxel, papr);
    vector<int> cluster_id;
    most_frequent_value(cluster_index, cluster_id);
    //统计聚类结果
    unordered_map<int, int> classes;
    for(size_t i = 0; i < cluster_index.size(); i++)
    {
        unordered_map<int, int>::iterator it = classes.find(cluster_index[i]) ;
        if(it == classes.end())
            classes.insert(make_pair(cluster_index[i], 1));
        else
            it->second++;
    }
    //去除点数量比较少的聚类
    for(unordered_map<int, int>::iterator it = classes.begin(); it != classes.end(); )
    {
        if(it->second <= 30)
            classes.erase(it++);
        else
            it++;
    }
    std::cout << "cluster: " << classes.size() << std::endl;
    //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
    vector<pcl::PointCloud<PointType>::Ptr> cluster_cloud;
    vector<LineParameter::Ptr> cluster_parameter;
    vector<CylinderParameter::Ptr> cluster_parameter2;
    vector<pcl::PointCloud<PointType>::Ptr> cluster_cloud_frame;
    int cluster_label = 0;
    for(unordered_map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
    {
        //保存聚类的点云
        pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());
        for(size_t i = 0; i < cluster_index.size(); i++)
        {
            if(cluster_index[i] == it->first)
            {
                points_interest_serial->points[i].label = cluster_label;
                // pointAssociateToT(points_interest_serial->points[i], tmp_r_w_curr, tmp_t_w_curr);
                tempCloud->emplace_back(points_interest_serial->points[i]);
            }
        }
        cluster_cloud.emplace_back(tempCloud);
        //计算这部分点云的PCA并画框框
        Eigen::Vector4d min, max;
        LineParameter::Ptr line(new LineParameter(tempCloud));
        CylinderParameter::Ptr cylinder(new CylinderParameter(tempCloud, line));
        cluster_label++;
        cluster_parameter.emplace_back(line);
        cluster_parameter2.emplace_back(cylinder);
        cluster_cloud_frame.emplace_back(tempCloud);
    }
    // 更新当前的点云与参数
    now_cluster_cloud.swap(cluster_cloud);
    now_cluster_parameter_line.swap(cluster_parameter);
    now_cluster_parameter_cylinder.swap(cluster_parameter2);


    // 将当前点云变换到世界系下
    pcl::PointCloud<PointType>::Ptr points_cluster_frame(new pcl::PointCloud<PointType>());
    for(auto&p:cluster_cloud_frame) *points_cluster_frame += *p;
    for(auto&p:points_cluster_frame->points){
        pointAssociateToT(p, tmp_r_w_curr, tmp_t_w_curr);
    }
    points_cluster_frame->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_cluster_frame->header.frame_id = "map";
    pub_pcl_cluster_frame.publish(points_cluster_frame);
    laserCloudArray.emplace_back(points_cluster_frame);

    // 柱面可视化参数
    pcl::PointCloud<PointType>::Ptr visual_cloud_cylinder(new pcl::PointCloud<PointType>());
    for(auto&cylinder:now_cluster_parameter_cylinder){
        auto cylinder_w = cylinder->translate(tmp_r_w_curr, tmp_t_w_curr);
        auto shape_cloud = CylinderParameter::generateCylinderCloud(*cylinder_w);
        *visual_cloud_cylinder += *shape_cloud;
    }
    visual_cloud_cylinder->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    visual_cloud_cylinder->header.frame_id = "map";
    cylinder_pub.publish(visual_cloud_cylinder);

    cout << "find match" << endl;
    // cylinderManager.addNewLandmark(now_cluster_parameter_cylinder, timestamp, odom);
    cout << "get match " << endl;
    auto baassocate = cylinderManager.getBAAssocate();
    cout << "get ba assocate " << baassocate.size() << endl;


    {
        // 作为ODOM基准 计算位姿 这个行不通， 约束太少了，只有联合多帧才有意义
        // 发布距离后点云 利用上一帧位姿变换点云，此处不仅ODOM基准不够好，针织转换也是有错误的（上一帧位姿转本帧点云）
        // {
        //     pcl::PointCloud<PointType>::Ptr points_cluster_frame(new pcl::PointCloud<PointType>());
        //     for(auto&p:cluster_cloud_frame) *points_cluster_frame += *p;
        //     for(auto&pi:points_cluster_frame->points){
        //         pointAssociateToMap(pi, pi);
        //     }
        //     points_cluster_frame->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        //     points_cluster_frame->header.frame_id = "map";
        //     pub_pcl_cluster_frame.publish(points_cluster_frame);
        //     laserCloudArray.emplace_back(points_cluster_frame);
        //     bool ok = true;
        //     ceres::Problem::Options problem_option;
        //     ceres::Problem problem(problem_option);
        //     ceres::LocalParameterization * qparamtion = new ceres::EigenQuaternionParameterization();
        //     problem.AddParameterBlock(parameters, 4, qparamtion);
        //     problem.AddParameterBlock(parameters+4, 3);
        //     ceres::LossFunction * loss_function = new ceres::HuberLoss(1.0);

        //     // per point to line
        //     if(!now_cluster_cloud.empty() && !last_cluster_cloud.empty() && ok){
        //         pcl::KdTreeFLANN<PointType>::Ptr kdtreeLastMap(new pcl::KdTreeFLANN<PointType>());
        //         pcl::PointCloud<PointType>::Ptr last_center_cloud(new pcl::PointCloud<PointType>());
        //         for(auto&l:last_cluster_parameter){
        //             PointType tmpp;
        //             tmpp.x=l->A[0];
        //             tmpp.y=l->A[1];
        //             tmpp.z=l->A[2];
        //             last_center_cloud->emplace_back(tmpp);
        //         }
        //         kdtreeLastMap->setInputCloud(last_center_cloud);
        //         for(size_t i = 0; i < now_cluster_cloud.size(); ++i){
        //             // cout << "for cluster " << i << endl;
        //             // LineParameter::Ptr now_cluster_line = now_cluster_parameter[i];
        //             for(size_t j = 0; j < now_cluster_cloud[i]->size(); ++j){
        //                 Eigen::Vector3d pointx(now_cluster_cloud[i]->points[j].x, now_cluster_cloud[i]->points[j].y, now_cluster_cloud[i]->points[j].z);
        //                 PointType pointSel;
        //                 pointSel.x=now_cluster_cloud[i]->points[j].x;
        //                 pointSel.y=now_cluster_cloud[i]->points[j].y;
        //                 pointSel.z=now_cluster_cloud[i]->points[j].z;
        //                 std::vector<int> pointSearchInd;
        //                 std::vector<float> pointSearchSqDis;
        //                 kdtreeLastMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 
        //                 if(pointSearchInd.empty()) continue;
        //                 int smInd = -1;
        //                 double smDis=FLT_MAX;
        //                 for(size_t k = 0; k < pointSearchInd.size(); ++k){
        //                     double tmpDis = last_cluster_parameter[pointSearchInd[k]]->DisToPoint(pointx);
        //                     if(tmpDis < smDis){
        //                         smDis = tmpDis;
        //                         smInd = pointSearchInd[k];
        //                     }
        //                 }
        //                 // cout << "c " << i << " p " << j << " find " << smInd << " dis " << smDis << endl;
        //                 if(smDis >= 0 && smDis < 1.0){
        //                     ceres::CostFunction *cost_function = LidarLineFactor::Create(pointx, *last_cluster_parameter[smInd], 1.0);
        //                     problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
        //                 }
        //             }
        //         }
        //         ceres::Solver::Options options;
        //         options.linear_solver_type = ceres::DENSE_QR;
        //         options.max_num_iterations = 40;
        //         options.minimizer_progress_to_stdout = false;
        //         options.check_gradients = false;
        //         options.gradient_check_relative_precision = 1e-4;
        //         ceres::Solver::Summary summary;
        //         ceres::Solve(options, &problem, &summary);

        //         cout << summary.FullReport() << endl;
        //     }
        //     // 更新上一帧位姿
        //     Eigen::Quaterniond q_last_curr(parameters[3], parameters[0], parameters[1], parameters[2]); 
        //     Eigen::Vector3d t_last_curr(parameters[4], parameters[5], parameters[6]);
        //     Eigen::Matrix3d r_last_curr(q_last_curr);
        //     Eigen::Matrix4d T_last_curr = Eigen::Matrix4d::Identity();
        //     T_last_curr.block<3,3>(0,0) = r_last_curr;
        //     T_last_curr.block<3,1>(0,3) = t_last_curr;
        //     T_w_curr = T_last_curr.inverse() * T_w_curr;
        //     q_w_curr = Eigen::Quaterniond(T_w_curr.block<3,3>(0,0));
        //     r_w_curr = Eigen::Matrix3d(q_w_curr);
        //     t_w_curr = Eigen::Vector3d(T_w_curr.block<3,1>(0,3));
        //     Eigen::Vector3d euler = r_w_curr.eulerAngles(2,1,0);
        //     cout << "world posi " << t_w_curr[0] << " " << t_w_curr[1] << " " << t_w_curr[2] << endl;
        //     cout << "world ori  " << euler[0] << " " << euler[1] << " " << euler[2] << endl;
        //     cout << "pose " ;
        //     for(int i = 0; i < 7; ++i){
        //         cout << parameters[i] << " ";
        //         parameters[i] = 0.0f;
        //     }
        //     cout << endl;
        //     parameters[3] = 1.0f;
        // }
    }

    printf("get semantic constraint %f ms\n", time_rg.toc());
    static int frameCount = 0;
    frameCount++;

    // 固定帧数 发布点云地图 减少开支
    cout << "frame count " << frameCount << endl;
    if (frameCount % 10 == 0)
    {
        pcl::PointCloud<PointType> laserCloudMap;
        for (size_t i = 0; i < laserCloudArray.size(); i++)
        {
            laserCloudMap += *laserCloudArray[i];
        }
        laserCloudMap.header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        laserCloudMap.header.frame_id = "map";
        pub_pcl_map.publish(laserCloudMap);
    }

    // 发布tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tmp_q;
    transform.setOrigin(tf::Vector3(tmp_t_w_curr(0),
                                    tmp_t_w_curr(1),
                                    tmp_t_w_curr(2)));
    tmp_q.setW(tmp_q_w_curr.w());
    tmp_q.setX(tmp_q_w_curr.x());
    tmp_q.setY(tmp_q_w_curr.y());
    tmp_q.setZ(tmp_q_w_curr.z());
    transform.setRotation(tmp_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(timestamp), "map", "odom"));


    if (frameCount % 20== 0){
        TicToc showBatime;
        pcl::PointCloud<PointType>::Ptr showBaCloud(new pcl::PointCloud<PointType>());
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                auto&pose=*(*perba->pose_vec)[i];
                auto&cylinder=(*perba->measurement_vec)[i];
                *showBaCloud += *CylinderParameter::generateCylinderCloud(*cylinder);
                PointType cc, oc;
                cc.x = cylinder->center[0],cc.y = cylinder->center[1],cc.z = cylinder->center[2];
                oc.x = pose.odom->translation()[0],oc.y = pose.odom->translation()[1],oc.z = pose.odom->translation()[2];
                // printf("get cc %f,%f,%f  oc %f,%f,%f\n", cc.x, cc.y, cc.z, oc.x, oc.y, oc.z);
                *showBaCloud += *createLineCLoud(cc, oc);
            }
        }
        printf("show Ba time %f\n", showBatime.toc());
        showBaCloud->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        showBaCloud->header.frame_id = "map";
        pub_ba_map.publish(showBaCloud);
    }
}



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec)
{
    laserCloudMsgQueue.push(rec);
    while (laserCloudMsgQueue.size() > 10)
        laserCloudMsgQueue.pop();
}

void laserOdomHandler(nav_msgs::Odometry odom)
{
    if (odomTopic == "/gt")
    {
        Eigen::Matrix4d transN = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond qN(odom.pose.pose.orientation.w,
                              odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z);
        Eigen::Vector3d tN(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        transN.block<3, 3>(0, 0) = Eigen::Matrix3d(qN);
        transN.block<3, 1>(0, 3) = tN;
        transN = gt2lidar.transpose() * transN * gt2lidar;
        Eigen::Quaterniond tmpQ(transN.block<3, 3>(0, 0));
        odom.pose.pose.orientation.w = tmpQ.w();
        odom.pose.pose.orientation.x = tmpQ.x();
        odom.pose.pose.orientation.y = tmpQ.y();
        odom.pose.pose.orientation.z = tmpQ.z();
        odom.pose.pose.position.x = transN(0, 3);
        odom.pose.pose.position.y = transN(1, 3);
        odom.pose.pose.position.z = transN(2, 3);
    }
    laserOdomQueue.push(odom);
    while (laserOdomQueue.size() > 10)
        laserOdomQueue.pop();
}


std::vector<bool> getInterest(std::string file_name)
{
    std::vector<int> ind;
    std::vector<bool> interest;
    std::vector<int> labels;
    std::vector<std::string> label_vals;
    YAML::Node config = YAML::LoadFile(file_name);
    int max_labels = 0;
    for (YAML::const_iterator it = config["labels"].begin(); it != config["labels"].end(); ++it)
    {
        int now_label = it->first.as<int>();
        std::string now_label_val = it->second.as<std::string>();
        labels.emplace_back(now_label);
        label_vals.emplace_back(now_label_val);
        if (now_label > max_labels && now_label_val != "")
        {
            max_labels = now_label;
        }
        printf("%d : %s\n", now_label, now_label_val.c_str());
    }
    std::stringstream ss(config["interest"].as<std::string>());
    std::string info;
    while (getline(ss, info, ','))
    {
        ind.emplace_back(std::stoi(info));
    }
    std::sort(ind.begin(), ind.end());
    interest.resize(max_labels + 1, false);
    printf("Total %d labels. Max LabelNum: %d. Thest are interested:\n", static_cast<int>(labels.size()), max_labels);
    for (auto &p : ind)
    {
        interest[p] = true;
        printf("%d:true\n", p);
    }
    return interest;
}

pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel)
{
    int valid_labels = 0;
    vector<int> interestLabelId(interestLabel.size(), 0);
    for (size_t i = 0; i < interestLabel.size(); ++i)
    {
        if (interestLabel[i])
        {
            interestLabelId[i] = valid_labels;
            valid_labels++;
        }
    }
    if (raw_pcl_->points.size() > 0)
    {
        // step1:getInterestedLabels
        pcl::PointCloud<PointType>::Ptr interested;
        interested.reset(new pcl::PointCloud<PointType>());
        for (auto p : raw_pcl_->points)
        {
            if (interestLabel[p.label & 0xFFFF])
            {
                interested->points.emplace_back(p);
            }
            else
            {
            }
        }
        return interested;
    }
    else
    {
        return nullptr;
    }
}

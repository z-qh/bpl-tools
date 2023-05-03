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
#include "nav_msgs/Path.h"


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
    double timestamp = 0;
    pcl::PointCloud<PointType>::Ptr ori_cloud = nullptr;
    Eigen::Vector3d A = Eigen::Vector3d::Zero(), D = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_vec = Eigen::Vector3d::Zero(), min_vec = Eigen::Vector3d::Zero();
    double max_dis=-FLT_MAX, min_dis=FLT_MAX;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    LineParameter() = delete;
    LineParameter(pcl::PointCloud<PointType>::Ptr cloud, double time){
        timestamp = time;
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
        const static double theta_d = (PI/6.0);
        const static double line_d = 0.1;
        int label_color = 0;
        if(!cylinder.valid) label_color = 255;
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
            p.label = label_color;
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
    CylinderParameter(pcl::PointCloud<PointType>::Ptr cloud, LineParameter::Ptr line){
        if(cloud->empty()) return ;
        ori_cloud = cloud;
        fitting_cylinder(cloud, line);
    }
    using Ptr = std::shared_ptr<CylinderParameter>;
    friend std::ostream& operator<<(std::ostream& out, CylinderParameter&cylinder);
public:
    Eigen::Vector3d translateA(Eigen::Matrix3d rot_, Eigen::Vector3d t_){
        Eigen::Vector3d d = rot_ * D;
        d.normalize();
        Eigen::Vector3d c = rot_ * center + t_;
        Eigen::Vector3d a;
        double a0 = c[0] + d[0] * (c[2]-0.f) / d[2];
        double a1 = c[1] + d[1] * (c[2]-0.f) / d[2];
        double a2 = 0.f;
        a[0]=a0, a[1]=a1, a[2]=a2;
        return a;
    }
    void translate(Eigen::Matrix3d rot_, Eigen::Vector3d t_){
        Eigen::Vector3d d = rot_ * D;
        d.normalize();
        Eigen::Quaterniond q(fromtwovectors(d));
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
        this->A = a;
        this->center = c;
        this->D = d;
        this->max_dis = ma_dis;
        this->min_dis = mi_dis;
        this->max_vec = ma_vec;
        this->min_vec = mi_vec;
        this->Q = q;
        this->r = r;
        this->valid = valid;
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
        trans.block<3,3>(0,0) = rot_;
        trans.block<3,1>(0,3) = t_;
        pcl::transformPointCloud(*ori_cloud, *ori_cloud, trans);
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
        if(summary.final_cost < 0.5) valid = true;

        // cout << "final cost " << summary.final_cost << endl;
        // if(summary.final_cost<0.5) cout << "#############" << endl;
        // else {
        //     cout << "!!!!!!!!!!!!" << endl; 
        //     static int bad_id = 1;
        //     static string pathBase = "/home/qh/ins_map_temp/badcylinder/";
        //     string log_file_name, shape_cloud_file_name, ori_cloud_file_name;
        //     log_file_name = pathBase + to_string(bad_id) + ".log";
        //     shape_cloud_file_name = pathBase + to_string(bad_id) + "shape.pcd";
        //     ori_cloud_file_name = pathBase + to_string(bad_id) + "ori.pcd";
        //     auto shape_cloud = CylinderParameter::generateCylinderCloud(*this);
        //     auto ori_cloud = this->ori_cloud;
        //     // if(!shape_cloud->empty()) pcl::io::savePCDFileASCII(shape_cloud_file_name, *shape_cloud);
        //     // if(!ori_cloud->empty()) pcl::io::savePCDFileASCII(ori_cloud_file_name, *ori_cloud);
        //     ofstream file(log_file_name);
        //     file << summary.FullReport() << endl;
        //     file.close();
        //     bad_id++;
        // }
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
    static double disFromTwoCylinder(CylinderParameter::Ptr a, CylinderParameter::Ptr b){
        double s1 = sqrt(pow((a->A-b->A).norm(), 2) - pow((a->A-b->A).dot(b->D),2)) * (b->max_dis-b->min_dis);
        double s2 = sqrt(pow((b->A-a->A).norm(), 2) - pow((b->A-a->A).dot(a->D),2)) * (a->max_dis-a->min_dis);
        return s1+s2;
    }
    static double dirFromTwoCylinder(CylinderParameter::Ptr a,CylinderParameter::Ptr b){
        return acos(a->A.dot(b->A));
    }
    static double rFromTwoClinder(CylinderParameter::Ptr a,CylinderParameter::Ptr b){
        return abs(a->r - b->r) / b->r;
    }
public:
    CylinderParameter::Ptr cylinder = nullptr;
    shared_ptr<vector<CylinderParameter::Ptr>> hcylinder;
    int id = 0;
    double timestamped = 0;
    int visiable_count = 0;
    int assocate_count = 0;
    shared_ptr<Eigen::Vector3d> SumA;
    Pole() = delete;
    Pole(CylinderParameter::Ptr c, int id_, double time){
        id = id_;
        timestamped = time;
        cylinder = c;
        hcylinder = std::make_shared<vector<CylinderParameter::Ptr>>();
        hcylinder->emplace_back(c);
        SumA = std::make_shared<Eigen::Vector3d>(c->A);
    }
    Pole(const Pole& p){
        this->id = p.id;
        this->timestamped = p.timestamped;
        this->assocate_count = p.assocate_count;
        this->visiable_count = p.visiable_count;
        this->hcylinder = p.hcylinder;
    }
    Eigen::Vector3d getAvgA() const 
    {
        Eigen::Vector3d ans(*SumA);
        ans /= hcylinder->size();
        return ans;
    }
    void updateMeasurement(CylinderParameter::Ptr c){
        *SumA += c->A;
        hcylinder->emplace_back(c);
        cylinder = c;
    }
    void update_clear(Eigen::Vector3d newA){
        *SumA = newA;
        this->hcylinder->clear();
        this->hcylinder->emplace_back(cylinder);
        // avoid accidental delet
        this->visiable_count = 1;
        this->assocate_count = 1;
    }
    using Ptr = std::shared_ptr<Pole>;
};

class PoseType{
public:
    int odom_id;
    double timestamp;
    shared_ptr<Eigen::Matrix4d> odom;
    PoseType() = delete;
    PoseType(int id_, double time, shared_ptr<Eigen::Matrix4d> o_):odom_id(id_), timestamp(time), odom(o_){};
    PoseType(const PoseType& p){
        this->odom_id = p.odom_id;
        this->odom = p.odom;
    } 
    using Ptr = shared_ptr<PoseType>;
};

class PoleConstraint{
public:
    int pole_id;
    shared_ptr<vector<PoseType::Ptr>> pose_vec;
    shared_ptr<vector<CylinderParameter::Ptr>> measurement_vec;
    PoleConstraint() = delete;
    PoleConstraint(int p_id):pole_id(p_id){
        pose_vec = std::make_shared<vector<PoseType::Ptr>>();
        measurement_vec = std::make_shared<vector<CylinderParameter::Ptr>>();
    }
    PoleConstraint(const PoleConstraint& pc){
        this->pole_id = pc.pole_id;
        this->pose_vec = pc.pose_vec;
        this->measurement_vec = pc.measurement_vec;
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
    map<int, PoseType::Ptr, std::greater<int>> trajectory;
    PoseType::Ptr lastPoseType = nullptr;
    int used_id;
    bool initial = false;
    std::vector<int> trajPathPoseId;
    nav_msgs::Path::Ptr trajPath;
    CylinderMapManager(){
        used_id = 0;
        kdtreePoleCenterMap.reset(new pcl::KdTreeFLANN<PointType>());
        PoleCenterCloud.reset(new pcl::PointCloud<PointType>());
        trajPath.reset(new nav_msgs::Path());
        trajPath->header.frame_id = "map";
    }


    void addNewLandmark(std::vector<CylinderParameter::Ptr>& landmarks, double timestamp, PoseType::Ptr nPoseType, double range){
        list<pair<PoseType::Ptr, CylinderParameter::Ptr>> nowBAAssocate;
        list<int> nowBAAssocateInd;
        auto&tT = *nPoseType->odom;
        Eigen::Vector3d nposi(tT.block<3,1>(0,3));
        Eigen::Matrix3d nori(tT.block<3,3>(0,0));
        Eigen::Quaterniond nquat(nori);
        trajectory[nPoseType->odom_id] = nPoseType;
        lastPoseType = nPoseType;

        trajPathPoseId.emplace_back(nPoseType->odom_id);

        // 统计当前帧的点云分布KDtree用于删除不良关联
        // cout << "remove bad mp" << endl;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeLocalPoleCenterMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr LocalPoleCenterCloud(new pcl::PointCloud<PointType>());
        for(auto&p:landmarks){
            PointType ltmp;
            ltmp.x = p->A[0],ltmp.y = p->A[1],ltmp.z = p->A[2];
            LocalPoleCenterCloud->emplace_back(ltmp);
        }
        if(!LocalPoleCenterCloud->empty()) kdtreeLocalPoleCenterMap->setInputCloud(LocalPoleCenterCloud);

        // 删除比较差的map点
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
            if(it->second->assocate_count * 1.0 / it->second->visiable_count < 0.2){
                global_map.erase(it++);
                continue;
            }

            // dis limit
            // double tdis = (it->second->cylinder->A - nposi).norm();
            // if(tdis < range) it->second->visiable_count++;

            // kdtree limit
            // if(LocalPoleCenterCloud->empty()){
            //     it->second->visiable_count++;
            // }else{
            //     PointType htmp;
            //     htmp.x=it->second->cylinder->A[0],htmp.y=it->second->cylinder->A[1],htmp.z=it->second->cylinder->A[2];
            //     std::vector<int> pointSearchInd;
            //     std::vector<float> pointSearchSqDis;
            //     kdtreeLocalPoleCenterMap->radiusSearch(htmp, 5.0, pointSearchInd, pointSearchSqDis);
            //     if(!pointSearchInd.empty()){
            //         it->second->visiable_count++;
            //     }
            // }

            it++;
        }
        PoleCenterCloud->clear();
        // cout << "get new map cloud" << endl;
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
        // cout << "get assocate mp" << endl;
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
                Pole::Ptr newPole = std::make_shared<Pole>(l, used_id, timestamp);
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
                    Pole::Ptr newPole = std::make_shared<Pole>(l, used_id, timestamp);
                    global_map.insert({used_id, newPole});
                    used_id++;
                    continue;
                }
                // 对比已关联的质量，去除质量比较差的关联，保留质量比较好的
                double dir_angle = Pole::dirFromTwoCylinder(l, min_s_pole->cylinder);
                double r_dis = Pole::rFromTwoClinder(l, min_s_pole->cylinder);
                if(r_dis > 0.2 || dir_angle > 0.35){
                    Pole::Ptr newPole = std::make_shared<Pole>(l, used_id, timestamp);
                    global_map.insert({used_id, newPole});
                    used_id++;
                    continue;
                }else{
                    // 记录BA参数
                    nowBAAssocate.emplace_back(nPoseType, min_s_pole->cylinder);
                    nowBAAssocateInd.emplace_back(min_s_id);
                    // 已经关联的更新cylinder以及Pole关联参数，
                    min_s_pole->assocate_count++;
                    min_s_pole->visiable_count++;
                    min_s_pole->updateMeasurement(l);
                }
            }
        }
        if(!nowBAAssocate.empty()) BAAssocate.emplace_back(nowBAAssocate);
        if(!nowBAAssocateInd.empty()) BAAssocateInd.emplace_back(nowBAAssocateInd);
        // cout << "add end " << endl;
    }
    
    
    vector<PoleConstraint::Ptr> getBAAssocate(){
        unordered_map<int, PoleConstraint::Ptr> pole_constraint_dic;
        vector<PoleConstraint::Ptr> ans;
        auto itframeba = BAAssocate.begin();
        auto itframebaid = BAAssocateInd.begin(); 
        for(; itframeba != BAAssocate.end() && itframebaid != BAAssocateInd.end(); itframeba++,itframebaid++){
            auto itpba = itframeba->begin();
            auto itpbaid = itframebaid->begin();
            for(; itpba != itframeba->end() && itpbaid != itframebaid->end(); itpba++, itpbaid++){
                if(global_map.count(*itpbaid) == 1){
                    if(pole_constraint_dic.count(*itpbaid) != 1) 
                        pole_constraint_dic[*itpbaid] = std::make_shared<PoleConstraint>(*itpbaid);
                    auto &ba_constraint = pole_constraint_dic[*itpbaid];
                    ba_constraint->pole_id = *itpbaid;
                    ba_constraint->pose_vec->emplace_back(itpba->first);
                    ba_constraint->measurement_vec->emplace_back(itpba->second);
                }
            }
        }
        for(auto&p:pole_constraint_dic){
            ans.emplace_back(p.second);
        }
        return ans;
    }
    void updatepose(){
        // cout << "######################updatepose########################" << endl;
        if(lastPoseType == nullptr) return;
        // cout << "get BA Assocate " << endl;
        auto baassocate = this->getBAAssocate();
        // cout << "Configuration optimize Tools " << endl;
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
        int vertex_pole_id = 1;
        int vertex_pose_id = baassocate.size()+1;
        unordered_map<int, int> poseid2vertexid;
        unordered_map<int, int> vertexid2poseid;
        unordered_map<int, int> vertexid2poleid;
        unordered_map<int, int> poleid2vertexid;
        int last_pose_idf_id = 0;
        int last_pose_vertex_id = 0;
        // cout << "Start add constraint" << endl;
        // cout << "Start add vertex" << endl;
        int edge_count = 0;
        unordered_set<int> pose_vertex_id_dic, cylinder_vertex_id_dic;
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                PoseType&pose(*(*perba->pose_vec)[i]);
                if(pose_vertex_id_dic.count(pose.odom_id) != 1){
                    // 位姿是顶点
                    g2o::VertexSE3 * vSE3_w = new g2o::VertexSE3();
                    Eigen::Matrix3d tR = (*pose.odom).block<3,3>(0,0);
                    Eigen::Vector3d tt = (*pose.odom).block<3,1>(0,3);
                    vSE3_w->setEstimate(g2o::SE3Quat(tR, tt));
                    vSE3_w->setId(vertex_pose_id);
                    vSE3_w->setFixed(false);
                    optimizer.addVertex(vSE3_w);
                    vertexid2poseid[vertex_pose_id] = pose.odom_id;
                    poseid2vertexid[pose.odom_id] = vertex_pose_id;
                    if(last_pose_idf_id > pose.odom_id){
                        last_pose_idf_id = pose.odom_id;
                        last_pose_vertex_id = vertex_pose_id;
                    }
                    vertex_pose_id++;
                    // cout << "\t get a pose vertex " << vertex_pose_id-1 << endl;
                }
                edge_count++;
            }
            if(cylinder_vertex_id_dic.count(perba->pole_id) == 1) continue;
            if(perba->measurement_vec->empty()) continue;
            Eigen::Vector3d avg_posi_world = global_map[perba->pole_id]->getAvgA();
            // 路标点是顶点
            g2o::VertexPointXYZ * vMP_w = new g2o::VertexPointXYZ();
            vMP_w->setEstimate(avg_posi_world);
            vMP_w->setId(vertex_pole_id);
            vMP_w->setFixed(false);
            optimizer.addVertex(vMP_w);
            vertexid2poleid[vertex_pole_id] = perba->pole_id;
            poleid2vertexid[perba->pole_id] = vertex_pole_id;
            vertex_pole_id++;
            // cout << "\t get a cylinder vertex " << vertex_pole_id-1 << endl;
        }
        // 为每一帧BA添加 edge
        int edge_id = vertex_pose_id + 1;
        // cout << "Start add edge : " << edge_count << endl;
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                auto&pose=*(*perba->pose_vec)[i];
                auto&cylinder=(*perba->measurement_vec)[i];
                int pose_vertex_id = poseid2vertexid[pose.odom_id];
                int pole_vertex_id = poleid2vertexid[perba->pole_id];
                // cout << "\t get a edge " << edge_id << " v1: " << pose_vertex_id << " v2: " << pole_vertex_id << endl;
                g2o::EdgeSE3PointXYZ * edge_pose_cylinder = new g2o::EdgeSE3PointXYZ();
                edge_pose_cylinder->setId(edge_id);
                edge_pose_cylinder->setParameterId(0, 0);
                edge_pose_cylinder->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(pose_vertex_id)));
                edge_pose_cylinder->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pole_vertex_id)));
                Eigen::Matrix3d tR = (*pose.odom).block<3,3>(0,0);
                Eigen::Vector3d tt = (*pose.odom).block<3,1>(0,3);
                Eigen::Vector3d local_measurement = cylinder->translateA(tR.transpose(),(-1)*tR.transpose()*tt);
                edge_pose_cylinder->setMeasurement(local_measurement);
                optimizer.addEdge(edge_pose_cylinder);
                edge_id++;
            }
        }

        // cout << "**************optimization begin***********" << endl;
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        // cout << "**************optimization end***********" << endl;
        // cout << "get ans after optimize " << endl;
        for(int landmark_id = 1; landmark_id < vertex_pole_id; ++landmark_id){
            g2o::VertexPointXYZ* landmark_res = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(landmark_id));
            auto pole_cons_to_update = global_map[vertexid2poleid[landmark_id]];
            pole_cons_to_update->update_clear(landmark_res->estimate());
        }
        for(int pose_id = baassocate.size()+1; pose_id < vertex_pose_id; ++pose_id){
            g2o::VertexSE3* pose_res = static_cast<g2o::VertexSE3*>(optimizer.vertex(pose_id));
            (*trajectory[vertexid2poseid[pose_id]]->odom).block<3,3>(0,0) = pose_res->estimate().rotation();
            (*trajectory[vertexid2poseid[pose_id]]->odom).block<3,1>(0,3) = pose_res->estimate().translation();
            if(last_pose_vertex_id == pose_id){
                (*lastPoseType->odom).block<3,3>(0,0) = pose_res->estimate().rotation();
                (*lastPoseType->odom).block<3,1>(0,3) = pose_res->estimate().translation();
            }
        }
        // cout << "clear the memory !" << endl;
        // cout << "clear ba assocate cache !" << endl;
        BAAssocate.clear();
        // cout << "clear ba id cache!" << endl;
        BAAssocateInd.clear();
        // cout << "end" << endl;
    }
    pcl::PointCloud<PointType>::Ptr getPoseVec(){
        pcl::PointCloud<PointType>::Ptr ans(new pcl::PointCloud<PointType>());
        for(auto&p:trajectory){
            PointType tmp;
            auto&tT = *p.second->odom;
            tmp.x = tT(0, 3);
            tmp.y = tT(1, 3);
            tmp.z = tT(2, 3);
            tmp.intensity = 0.0f;
            tmp.label = 0;
            ans->emplace_back(tmp);
        }
        return ans;
    }

    nav_msgs::Path::Ptr getPath(){
        for(size_t i = trajPath->poses.size(); i < trajPathPoseId.size(); ++i){
            auto todom = trajectory[trajPathPoseId[i]];
            geometry_msgs::PoseStamped tposes;
            tposes.header.stamp = ros::Time().fromSec(todom->timestamp);
            tposes.header.frame_id = "map";
            auto&tT=*(todom->odom);
            tposes.pose.position.x = tT(0,3);
            tposes.pose.position.y = tT(1,3);
            tposes.pose.position.z = tT(2,3);
            Eigen::Quaterniond tquat(tT.block<3,3>(0,0));
            tposes.pose.orientation.w = tquat.w();
            tposes.pose.orientation.x = tquat.x();
            tposes.pose.orientation.y = tquat.y();
            tposes.pose.orientation.z = tquat.z();
            trajPath->poses.emplace_back(tposes);
        }
        return trajPath;        
    }

    pcl::PointCloud<PointType>::Ptr getLandMark(){
        pcl::PointCloud<PointType>::Ptr lCloud(new pcl::PointCloud<PointType>());
        for(auto&p:global_map){
            PointType tmp;
            auto avgA = p.second->getAvgA();
            tmp.x = avgA[0];
            tmp.y = avgA[1];
            tmp.z = avgA[2];
            lCloud->emplace_back(tmp);
        }
        return lCloud;
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
    // pcl::PointCloud<PointType>::Ptr path_node(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr assline_cylinder(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr assline(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr assline_ori(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr assline_lcc(new pcl::PointCloud<PointType>());
    
    // for(auto&perba:baassocate){
    //     for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
    //         auto&pose=*(*perba->pose_vec)[i];
    //         auto&cylinder=(*perba->measurement_vec)[i];
    //         int label_id = perba->pole_id;
    //         PointType cc, oc;
    //         cc.x = cylinder->center[0],cc.y = cylinder->center[1],cc.z = cylinder->center[2];
    //         oc.x = pose.odom->translation()[0],oc.y = pose.odom->translation()[1],oc.z = pose.odom->translation()[2];
    //         // printf("get cc %f,%f,%f  oc %f,%f,%f\n", cc.x, cc.y, cc.z, oc.x, oc.y, oc.z);
    //         *assline += *createLineCLoud(cc, oc);
    //         *assline_cylinder += *CylinderParameter::generateCylinderCloud(*cylinder);
    //         *assline_ori += *(cylinder->ori_cloud);
    //         PointType lcc;
    //         lcc.x=cylinder->A[0],lcc.y=cylinder->A[1],lcc.z=cylinder->A[2];
    //         lcc.intensity = label_id, lcc.label = label_id;
    //         // lcc.intensity = cylinder->
    //         assline_lcc->emplace_back(lcc);
    //     }
    // }
    // for(auto&p:odompath){
    //     path_node->emplace_back(p);   
    // }
    // if(!path_node->empty()) pcl::io::savePCDFileASCII(saveBase+"path_node.pcd", *path_node);
    // if(!assline_cylinder->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_cylinder.pcd", *assline_cylinder);
    // if(!assline->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate.pcd", *assline);
    // if(!assline_ori->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_ori.pcd", *assline_ori);
    // if(!assline_lcc->empty()) pcl::io::savePCDFileASCII(saveBase+"BA_assocate_lcc.pcd", *assline_lcc);
    // printf("show Ba time %f\n", showBatime.toc());

    // auto before_odom_cloud = cylinderManager.getPoseVec();
    // cout << "update pose start" << endl;
    // cylinderManager.updatepose();
    // cout << "update pose end  " << endl;
    // auto after_odom_cloud = cylinderManager.getPoseVec();
    // cout << "save update result" << endl;
    // if(!before_odom_cloud->empty()) pcl::io::savePCDFileASCII(saveBase+"before_odom.pcd", *before_odom_cloud);
    // if(!after_odom_cloud->empty()) pcl::io::savePCDFileASCII(saveBase+"after_odom.pcd", *after_odom_cloud);
    cout << "data saved" << endl;
}



///////////////////////////////////////////////////////////
std::string odomTopic = "/gt";
Eigen::Matrix4d gt2lidar = Eigen::Matrix4d::Identity();
std::queue<nav_msgs::Odometry> laserOdomQueue;
ros::Publisher line_pub, cylinder_pub, landmark_pub, path_pub, old_path_pub;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;
ros::Publisher pub_pcl_cluster_frame, pub_pcl_interest, pub_pcl_map, pub_ba_map;
image_transport::Publisher imgPub;
std::vector<bool>  interest_labels;
pcl::VoxelGrid<PointType> downSizeFilterCorner;
// std::vector<pcl::PointCloud<PointType>::Ptr> last_cluster_cloud, now_cluster_cloud;
std::vector<LineParameter::Ptr> last_cluster_parameter_line, now_cluster_parameter_line;
std::vector<CylinderParameter::Ptr> now_cluster_parameter_cylinder, last_cluster_parameter_cylinder;
// std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudArray;
std::vector<PoseType::Ptr> laserOdomArray;
///////////////////////////////////////////////////////////
std::vector<bool> getInterest(std::string file_name);
pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel, double&feel_range);
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec);
void process(const sensor_msgs::PointCloud2ConstPtr &rec, nav_msgs::Odometry& odom);
void laserOdomHandler(nav_msgs::Odometry odom);
int main(int argc, char** argv){
    ros::init(argc, argv, "pole");
    ros::NodeHandle nh("~");
    nh.getParam("odomTopic", odomTopic);
    downSizeFilterCorner.setLeafSize(1.f, 1.f, 1.f);
    gt2lidar << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    ros::Subscriber subLaserOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, laserOdomHandler);
    line_pub = nh.advertise<pcl::PointCloud<PointType>>("/pole/line", 1);
    cylinder_pub = nh.advertise<pcl::PointCloud<PointType>>("/pole/cylinder", 1);
    landmark_pub = nh.advertise<pcl::PointCloud<PointType>>("/pole/landmark", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/pole/path", 10);
    old_path_pub = nh.advertise<nav_msgs::Path>("/pole/path_old", 10);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    pub_pcl_cluster_frame = nh.advertise<pcl::PointCloud<PointType>>("/pole/cluster_frame", 1);
    pub_ba_map = nh.advertise<pcl::PointCloud<PointType>>("pole/ba_map", 1);
    pub_pcl_interest = nh.advertise<pcl::PointCloud<PointType>>("/pole/interest_points", 1);
    pub_pcl_map = nh.advertise<pcl::PointCloud<PointType>>("/pole/map", 1);
    image_transport::ImageTransport it(nh);
    imgPub = it.advertise("/pole/img", 10);
    interest_labels = getInterest("/home/qh/qh_ws/src/bpl-tools/bpltool/src/source/kitti_data_interest_pole.yaml");
    cout << "odom topic " << odomTopic << endl;
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
            static int frame_count = 1;
            process(laserCloudMsgQueue.front(), laserOdomQueue.front());
            frame_count++;
            // if(frame_count >= 100) break;
            laserCloudMsgQueue.pop();
            laserOdomQueue.pop();
            // last_cluster_cloud.swap(now_cluster_cloud);
            last_cluster_parameter_line.swap(now_cluster_parameter_line);
            last_cluster_parameter_cylinder.swap(now_cluster_parameter_cylinder);
        }
        loop.sleep();
    }
    exitSavedata();
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
    Eigen::Matrix4d tmp_T_w_curr = Eigen::Matrix4d::Identity();
    tmp_T_w_curr.block<3,3>(0,0) = tmp_r_w_curr;
    tmp_T_w_curr.block<3,1>(0,3) = tmp_t_w_curr;
    pcl::PointCloud<PointType>::Ptr rec_point(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*rec, *rec_point);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*rec_point, *rec_point, indices);
    // laserCloudArray.emplace_back(rec_point);
    shared_ptr<Eigen::Matrix4d> nlaserOdom = std::make_shared<Eigen::Matrix4d>(Eigen::Matrix4d::Identity());
    *nlaserOdom = tmp_T_w_curr;
    static int odom_id = -1;
    PoseType::Ptr nPoseType = std::make_shared<PoseType>(odom_id--, timestamp, nlaserOdom);
    laserOdomArray.emplace_back(nPoseType);
    if(rec_point->points.empty()) return;


    // 发布语义兴趣点
    pcl::PointCloud<PointType>::Ptr points_interest(new pcl::PointCloud<PointType>());
    for(auto&p:rec_point->points) {
        PointType pp(p);
        pp.label = p.label & 0xFF;
        pointAssociateToT(pp, tmp_r_w_curr, tmp_t_w_curr);
        points_interest->emplace_back(pp);
    }
    // points_interest->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_interest->header.stamp = pcl_conversions::toPCL(ros::Time().now());
    points_interest->header.frame_id = "map";
    pub_pcl_interest.publish(points_interest);

    double feel_range = -DBL_MAX;
    auto points_interest_serial = GetFilteredInterestSerial(rec_point, interest_labels, feel_range);
    if( points_interest_serial!=nullptr && points_interest_serial->points.empty()) return;

    

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
    // std::cout << "cluster: " << classes.size() << std::endl;
    //每一个key都要计算一波点云，计算一波框框，然后存到点云中去
    vector<LineParameter::Ptr> cluster_parameter;
    vector<CylinderParameter::Ptr> cluster_parameter2;
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
                tempCloud->emplace_back(points_interest_serial->points[i]);
            }
        }
        // 参数花
        LineParameter::Ptr line = std::make_shared<LineParameter>(tempCloud, timestamp);
        CylinderParameter::Ptr cylinder = std::make_shared<CylinderParameter>(tempCloud, line);
        if(!cylinder->valid) continue;
        cluster_label++;
        cluster_parameter.emplace_back(line);
        cluster_parameter2.emplace_back(cylinder);
    }
    // 更新当前的点云与参数
    now_cluster_parameter_line.swap(cluster_parameter);
    now_cluster_parameter_cylinder.swap(cluster_parameter2);


    // 将当前点云变换到世界系下
    pcl::PointCloud<PointType>::Ptr points_cluster_frame(new pcl::PointCloud<PointType>());
    for(auto&p:points_interest_serial->points) points_cluster_frame->emplace_back(p);
    for(auto&p:points_cluster_frame->points){
        pointAssociateToT(p, tmp_r_w_curr, tmp_t_w_curr);
    }
    points_cluster_frame->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_cluster_frame->header.frame_id = "map";
    pub_pcl_cluster_frame.publish(points_cluster_frame);

    // 柱面可视化参数
    pcl::PointCloud<PointType>::Ptr visual_cloud_cylinder(new pcl::PointCloud<PointType>());
    for(auto&cylinder:now_cluster_parameter_cylinder){
        cylinder->translate(tmp_r_w_curr, tmp_t_w_curr);
        auto shape_cloud = CylinderParameter::generateCylinderCloud(*cylinder);
        *visual_cloud_cylinder += *shape_cloud;
    }
    visual_cloud_cylinder->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    visual_cloud_cylinder->header.frame_id = "map";
    cylinder_pub.publish(visual_cloud_cylinder);

    // cout << "find match" << endl;
    cylinderManager.addNewLandmark(now_cluster_parameter_cylinder, timestamp, nPoseType, feel_range);
    // cout << "get match " << endl;
    auto baassocate = cylinderManager.getBAAssocate();
    // cout << "get ba assocate " << baassocate.size() << endl;

    // Old path
    static nav_msgs::Path tpath;
    geometry_msgs::PoseStamped tposes;
    tposes.header.frame_id = "map";
    tposes.header.stamp = ros::Time().fromSec(timestamp);
    auto&tT=*(nPoseType->odom);
    tposes.pose.position.x = tT(0,3);
    tposes.pose.position.y = tT(1,3);
    tposes.pose.position.z = tT(2,3);
    Eigen::Quaterniond tquat(tT.block<3,3>(0,0));
    tposes.pose.orientation.w = tquat.w();
    tposes.pose.orientation.x = tquat.x();
    tposes.pose.orientation.y = tquat.y();
    tposes.pose.orientation.z = tquat.z();
    tpath.poses.emplace_back(tposes);
    tpath.header.frame_id = "map";
    old_path_pub.publish(tpath);

    // New path
    auto path_odom_optimize = cylinderManager.getPath();
    path_pub.publish(*path_odom_optimize);

    auto landmarks_cloud = cylinderManager.getLandMark();
    landmarks_cloud->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    landmarks_cloud->header.frame_id = "map";
    landmark_pub.publish(landmarks_cloud);

    static int frameCount = 0;
    frameCount++;

    cout << "frame count " << frameCount << endl;

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

    if (frameCount % 50 == 0){
        TicToc optimizeTime;
        cylinderManager.updatepose();
        cout << "optimize time : " << optimizeTime.toc() << endl;
    }

    if (frameCount % 5 == 0 && true){
        TicToc showBatime;
        pcl::PointCloud<PointType>::Ptr showBaCloud(new pcl::PointCloud<PointType>());
        for(auto&perba:baassocate){
            for(size_t i = 0; i < perba->measurement_vec->size(); ++i){
                auto&pose=*(*perba->pose_vec)[i];
                auto&cylinder=(*perba->measurement_vec)[i];
                *showBaCloud += *CylinderParameter::generateCylinderCloud(*cylinder);
                PointType cc, oc;
                cc.x = cylinder->center[0],cc.y = cylinder->center[1],cc.z = cylinder->center[2];
                auto&tT = *(pose.odom);
                oc.x = tT(0, 3),oc.y = tT(1, 3),oc.z = tT(2, 3);
                // printf("get cc %f,%f,%f  oc %f,%f,%f\n", cc.x, cc.y, cc.z, oc.x, oc.y, oc.z);
                *showBaCloud += *createLineCLoud(cc, oc);
            }
        }
        printf("show Ba time %f\n", showBatime.toc());
        showBaCloud->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
        showBaCloud->header.frame_id = "map";
        pub_ba_map.publish(showBaCloud);
    }

    printf("get semantic constraint %f ms\n", time_rg.toc());
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
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel, double&feel_range)
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
                double tdis = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
                feel_range = max(feel_range, tdis);
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
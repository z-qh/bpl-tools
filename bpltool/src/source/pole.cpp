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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



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
    static void sortEigenVectorByValues(Eigen::Vector3f& eigenValues, Eigen::Matrix3f& eigenVectors) {
        vector<tuple<float, Eigen::Vector3f>> eigenValueAndVector;
        int sz = eigenValues.size();
        for (int i = 0; i < sz; ++i)
            eigenValueAndVector.emplace_back(tuple<float, Eigen::Vector3f>(eigenValues[i], eigenVectors.col(i)));
        
        // // 使用标准库中的sort，按从小到大排序
        static auto comp = [&](const tuple<float, Eigen::Vector3f>& a, const tuple<float, Eigen::Vector3f>& b)->bool
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
    pcl::PointCloud<PointType>::Ptr ori_cloud;
    Eigen::Vector3f A = Eigen::Vector3f::Zero(), D = Eigen::Vector3f::Zero();
    LineParameter() = delete;
    LineParameter(pcl::PointCloud<PointType>::Ptr cloud){
        ori_cloud = cloud;
        fit_line_parameter();
    }
    LineParameter(const LineParameter& lp){
        this->A = lp.A;
        this->D = lp.D;
        this->ori_cloud = lp.ori_cloud;
    }
    float DisToPoint(Eigen::Vector3f&x){
        float f = (x-A).transpose()*(x-A);
        float s = (D.transpose()*(x-A));
        return  f - s * s;
    }
    using Ptr = std::shared_ptr<LineParameter>;
    friend std::ostream& operator<<(LineParameter&line, std::ostream& out);
private:
    void fit_line_parameter(){
        if(ori_cloud == nullptr || ori_cloud->empty())return;
        Eigen::Vector3f a = Eigen::Vector3f::Zero();
        for(auto&p:ori_cloud->points){
            a += Eigen::Vector3f(p.x, p.y, p.z);
        }
        a /= ori_cloud->size();
        A = a;
        Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
        for(auto&p:ori_cloud->points){
            Eigen::Vector3f x(p.x, p.y, p.z);
            Eigen::Vector3f y = x - a;
            S += (x-a).transpose()*(x-a)*Eigen::Matrix3f::Identity() - (x-a)*(x-a).transpose();
        }
        Eigen::EigenSolver<Eigen::Matrix3f> solver(S);
        Eigen::Vector3f eigenValues = solver.pseudoEigenvalueMatrix().diagonal();
        Eigen::Matrix3f eigenVectors= solver.pseudoEigenvectors();
        sortEigenVectorByValues(eigenValues, eigenVectors);
        // cout << eigenValues << endl;
        // cout << eigenVectors<< endl;
        D = eigenVectors.col(0);
        D.normalize();
    }
};
std::ostream& operator<<(std::ostream& out, LineParameter&line){
    out << "center: " << line.A[0] << " " << line.A[1] << " " << line.A[2] << endl;
    out << "direct: " << line.D[0] << " " << line.D[1] << " " << line.D[2] << endl;
    return out;
}

// 柱面拟合残差
class CylinderFitFactor{
public:
    CylinderFitFactor(Eigen::Vector3f X_):X(X_){}
    template<typename T>
    bool operator()(const T*D, const T*A, const T*r, T*residual) const
    {
        Eigen::Matrix<T,3,1> x{T(X[0]), T(X[1]), T(X[2])};
        Eigen::Matrix<T,3,1> d{D[0], D[1], D[2]};
        Eigen::Matrix<T,3,1> a{A[0], A[1], A[2]};
        T f = (x-a).transpose()*(x-a); 
        T s = d.transpose()*(x-a);
        residual[0] = T(f - s * s - r[0]);
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3f X_){
        return (new ceres::AutoDiffCostFunction<CylinderFitFactor, 1, 3, 3, 1>(
            new CylinderFitFactor(X_)
        ));
    }
    Eigen::Vector3f X;
};

// 这种方式拟合会发散, 加了中心之后就不发散了 
class CylinderParameter{
public:
    Eigen::Vector3f D, A;
    float r;
    CylinderParameter() = delete;
    CylinderParameter(const CylinderParameter& c){
        this->D = c.D;
        this->A = c.A;
        this->r = c.r;
    }
    CylinderParameter(const pcl::PointCloud<PointType>::Ptr cloud){
        if(cloud->empty()) return ;
        fitting_cylinder(cloud);
    }
    using Ptr = std::shared_ptr<CylinderParameter>;
private:
    void fitting_cylinder(const pcl::PointCloud<PointType>::Ptr cloud){
        // cout << "start fitting " << endl;
        ceres::Problem::Options problem_option;
        ceres::Problem problem(problem_option);
        double parameter[7] = {0,0,1, 0,0,0, 0};
        problem.AddParameterBlock(parameter, 3);
        problem.AddParameterBlock(parameter+3, 3);
        problem.AddParameterBlock(parameter+6, 1);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for(size_t i = 0; i < cloud->size(); ++i){
            Eigen::Vector3f pointx(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            center += pointx;
            ceres::CostFunction * cost_function = CylinderFitFactor::Create(pointx);
            problem.AddResidualBlock(cost_function, loss_function, parameter, parameter+3, parameter+6);
        }
        center /= cloud->size();
        parameter[3] = center[0];
        parameter[4] = center[1];
        parameter[5] = center[2];
        ceres::Solver::Options option;
        option.linear_solver_type = ceres::DENSE_QR;
        option.max_num_iterations = 40;
        option.minimizer_progress_to_stdout = false;
        option.check_gradients = false;
        option.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(option, &problem, &summary);
        // cout << summary.FullReport() << endl;
        D = Eigen::Vector3f(parameter[0], parameter[1], parameter[2]);
        D.normalize();
        r = parameter[6];
        A = Eigen::Vector3f(parameter[3], parameter[4], parameter[5]);
        // cout << "center: " << endl << A << endl;
        // cout << "dir :   " << endl << D << endl;
        // cout << "radius: " << endl << r << endl;
        // cout << "end " << endl;
    }
};
//点到线的残差
class LidarLineFactor{
public:
    LidarLineFactor(Eigen::Vector3f curr_point, LineParameter line, float s_):X(curr_point),A(line.A),D(line.D),s(s_){}
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
        T f = (lx-a).transpose()*(lx-a);
        T s = (d.transpose()*(lx-a));
        residual[0] = T(f - s * s);
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3f curr_point_, const LineParameter line_, float s_){
        // return (new ceres::AutoDiffCostFunction<LidarLineFactor, 3, 4, 3>(
        return (new ceres::AutoDiffCostFunction<LidarLineFactor, 1, 4, 3>(
            new LidarLineFactor(curr_point_, line_, s_)
        ));
    }
    Eigen::Vector3f X,A,D;
    float s;
};
// 点到柱面的残差
class LidarCylinderFactor{
public:
    LidarCylinderFactor(Eigen::Vector3f curr_point, CylinderParameter cylinder, float s_)
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
        T f = x.transpose() * x;
        T s = d.transpose() * x;
        residual[0] = T(f - s * s - T(R));
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3f curr_point_, const CylinderParameter cylinder_, float s_){
        return (new ceres::AutoDiffCostFunction<LidarCylinderFactor, 1, 4, 3>(
            new LidarCylinderFactor(curr_point_, cylinder_, s_)
        ));
    }
    Eigen::Vector3f P,A,D;
    float R;
    float s;
};




///////////////////////////////////////////////////////////
ros::Publisher line_pub, cylinder_pub;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudMsgQueue;
ros::Publisher pub_pcl_cluster_frame, pub_pcl_interest;
image_transport::Publisher imgPub;
std::vector<bool>  interest_labels;
std::vector<pcl::PointCloud<PointType>::Ptr> last_cluster_cloud, now_cluster_cloud;
std::vector<LineParameter::Ptr> last_cluster_parameter, now_cluster_parameter;
///////////////////////////////////////////////////////////
std::vector<bool> getInterest(std::string file_name);
pcl::PointCloud<PointType>::Ptr GetFilteredInterestSerial(
    pcl::PointCloud<PointType>::Ptr raw_pcl_, vector<bool> &interestLabel);
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec);
void process(const sensor_msgs::PointCloud2ConstPtr &rec);
int main(int argc, char** argv){
    ros::init(argc, argv, "pole");
    ros::NodeHandle nh;
    line_pub = nh.advertise<visualization_msgs::MarkerArray>("/pole/line", 1);
    cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("/pole/cylinder", 1);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser", 1, laserCloudHandler);
    pub_pcl_cluster_frame = nh.advertise<pcl::PointCloud<PointType>>("/pole/cluster_frame", 1);
    pub_pcl_interest = nh.advertise<pcl::PointCloud<PointType>>("/pole/interest_points", 1);
    image_transport::ImageTransport it(nh);
    imgPub = it.advertise("/pole/img", 10);
    interest_labels = getInterest("/home/qh/qh_ws/src/bpl-tools/bpltool/src/source/kitti_data_interest_pole.yaml");
    ros::Rate loop(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if (laserCloudMsgQueue.empty())
            continue;
        process(laserCloudMsgQueue.front());
        laserCloudMsgQueue.pop();
        last_cluster_cloud.swap(now_cluster_cloud);
        last_cluster_parameter.swap(now_cluster_parameter);
        loop.sleep();
    }
}

void line_extrac(pcl::PointCloud<PointType>::Ptr interest_points){

}

// 用来优化的位姿参数
static double parameters[7] = {0,0,0,1, 0,0,0};
// 世界系位姿
static Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters); 
static Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4); 
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

Eigen::Quaternionf fromtwovectors(Eigen::Vector3f to, Eigen::Vector3f from=Eigen::Vector3f::UnitX()){
    Eigen::Vector3f w = to.cross(from);
    Eigen::Quaternionf q(1.0f+to.dot(from), w[0], w[1], w[2]);
    q.normalize();
    return q;
}

void ClearAllMarker(ros::Publisher &marker_pub_box_)
{
    visualization_msgs::MarkerArray::Ptr clear_marker_array(new visualization_msgs::MarkerArray);
    visualization_msgs::Marker dummy_marker;
    dummy_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker_array->markers.push_back(dummy_marker);
    marker_pub_box_.publish(clear_marker_array);
}

void pushShape(std::vector<LineParameter::Ptr>&parameters, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_shape;
    line_shape.header.frame_id = "map";
    line_shape.header.stamp = ros::Time().now();
    line_shape.ns = "parameter";
    line_shape.type = visualization_msgs::Marker::ARROW;
    line_shape.scale.x = 2.5;
    line_shape.scale.y = 0.25;
    line_shape.scale.z = 0.25;
    line_shape.color.r = 1.0;
    line_shape.color.g = 0.0;
    line_shape.color.b = 0.0;
    line_shape.color.a = 1.0;
    int line_points_size = parameters.size();
    int marker_id = 1;
    for(auto&p:parameters){
        line_shape.pose.position.x = p->A[0];
        line_shape.pose.position.y = p->A[1];
        line_shape.pose.position.z = p->A[2];
        auto q = fromtwovectors(p->D);
        line_shape.pose.orientation.x = q.x();
        line_shape.pose.orientation.y = q.y();
        line_shape.pose.orientation.z = q.z();
        line_shape.pose.orientation.w = q.w();
        line_shape.id = marker_id++;
        marker_array_box.markers.push_back(line_shape);
    }
}

void pushShape(std::vector<CylinderParameter::Ptr>&parameters, visualization_msgs::MarkerArray &marker_array_box)
{
    visualization_msgs::Marker line_shape;
    line_shape.header.frame_id = "map";
    line_shape.header.stamp = ros::Time().now();
    line_shape.ns = "parameter";
    line_shape.type = visualization_msgs::Marker::ARROW;
    line_shape.scale.x = 2.5;
    line_shape.color.r = 1.0;
    line_shape.color.g = 0.0;
    line_shape.color.b = 0.0;
    line_shape.color.a = 1.0;
    int line_points_size = parameters.size();
    int marker_id = 1;
    for(auto&p:parameters){
        line_shape.scale.y = p->r + 0.2;
        line_shape.scale.z = p->r + 0.2;
        line_shape.pose.position.x = p->A[0];
        line_shape.pose.position.y = p->A[1];
        line_shape.pose.position.z = p->A[2];
        auto q = fromtwovectors(p->D);
        line_shape.pose.orientation.x = q.x();
        line_shape.pose.orientation.y = q.y();
        line_shape.pose.orientation.z = q.z();
        line_shape.pose.orientation.w = q.w();
        line_shape.id = marker_id++;
        marker_array_box.markers.push_back(line_shape);
    }
}

void process(const sensor_msgs::PointCloud2ConstPtr &rec){
    TicToc time_rg;
    double timestamp = rec->header.stamp.toSec();
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
    printf("get interest %f ms\n", time_rg.toc());
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
    map<int, int> classes;
    for(int i = 0; i < cluster_index.size(); i++)
    {
        map<int, int>::iterator it = classes.find(cluster_index[i]) ;
        if(it == classes.end())
            classes.insert(make_pair(cluster_index[i], 1));
        else
            it->second++;
    }
    //去除点数量比较少的聚类
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); )
    {
        if(it->second <= 10)
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
    for(map<int, int>::iterator it = classes.begin(); it != classes.end(); it++)
    {
        //保存聚类的点云
        pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());
        for(int i = 0; i < cluster_index.size(); i++)
        {
            if(cluster_index[i] == it->first)
            {
                points_interest_serial->points[i].label = cluster_label;
                tempCloud->emplace_back(points_interest_serial->points[i]);
            }
        }
        cluster_cloud.emplace_back(tempCloud);
        //计算这部分点云的PCA并画框框
        Eigen::Vector4f min, max;
        LineParameter::Ptr line(new LineParameter(tempCloud));
        CylinderParameter::Ptr cylinder(new CylinderParameter(tempCloud));
        cluster_label++;
        cluster_parameter.emplace_back(line);
        cluster_parameter2.emplace_back(cylinder);
        cluster_cloud_frame.emplace_back(tempCloud);
    }
    now_cluster_cloud.swap(cluster_cloud);
    now_cluster_parameter.swap(cluster_parameter);


    // 可视化参数化
    ClearAllMarker(line_pub);
    visualization_msgs::MarkerArray marker_array_line;
    pushShape(cluster_parameter, marker_array_line);
    line_pub.publish(marker_array_line);

    ClearAllMarker(cylinder_pub);
    visualization_msgs::MarkerArray marker_array_cylinder;
    pushShape(cluster_parameter2, marker_array_cylinder);
    cylinder_pub.publish(marker_array_cylinder);


    //发布距离后点云
    pcl::PointCloud<PointType>::Ptr points_cluster_frame(new pcl::PointCloud<PointType>());
    for(auto&p:cluster_cloud_frame) *points_cluster_frame += *p;
    points_cluster_frame->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timestamp));
    points_cluster_frame->header.frame_id = "map";
    pub_pcl_cluster_frame.publish(points_cluster_frame);

    // 更新上一帧位姿
    q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters); 
    t_w_curr = Eigen::Map<Eigen::Vector3d> (parameters + 4); 
    // 计算位姿
    bool ok = true;
    ceres::Problem::Options problem_option;
    ceres::Problem problem(problem_option);
    ceres::LocalParameterization * qparamtion = new ceres::EigenQuaternionParameterization();
    problem.AddParameterBlock(parameters, 4, qparamtion);
    problem.AddParameterBlock(parameters+4, 3);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);

    // per point to line
    if(!now_cluster_cloud.empty() && !last_cluster_cloud.empty() && ok){
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeLastMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr last_center_cloud(new pcl::PointCloud<PointType>());
        for(auto&l:last_cluster_parameter){
            PointType tmpp;
            tmpp.x=l->A[0];
            tmpp.y=l->A[1];
            tmpp.z=l->A[2];
            last_center_cloud->emplace_back(tmpp);
        }
        kdtreeLastMap->setInputCloud(last_center_cloud);
        for(size_t i = 0; i < now_cluster_cloud.size(); ++i){
            // cout << "for cluster " << i << endl;
            // LineParameter::Ptr now_cluster_line = now_cluster_parameter[i];
            for(size_t j = 0; j < now_cluster_cloud[i]->size(); ++j){
                Eigen::Vector3f pointx(now_cluster_cloud[i]->points[j].x, now_cluster_cloud[i]->points[j].y, now_cluster_cloud[i]->points[j].z);
                PointType pointSel;
                pointSel.x=now_cluster_cloud[i]->points[j].x;
                pointSel.y=now_cluster_cloud[i]->points[j].y;
                pointSel.z=now_cluster_cloud[i]->points[j].z;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeLastMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 
                if(pointSearchInd.empty()) continue;
                int smInd = -1;
                float smDis=FLT_MAX;
                for(size_t k = 0; k < pointSearchInd.size(); ++k){
                    float tmpDis = last_cluster_parameter[pointSearchInd[k]]->DisToPoint(pointx);
                    if(tmpDis < smDis){
                        smDis = tmpDis;
                        smInd = pointSearchInd[k];
                    }
                }
                // cout << "c " << i << " p " << j << " find " << smInd << " dis " << smDis << endl;
                if(smDis >= 0 && smDis < 1.0){
                    ceres::CostFunction *cost_function = LidarLineFactor::Create(pointx, *last_cluster_parameter[smInd], 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                }
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 40;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        cout << summary.FullReport() << endl;
    }
    for(size_t i = 0; i < now_cluster_cloud.size(); ++i){
        for(size_t j = 0; j < now_cluster_cloud[i]->size(); ++j){
            pointAssociateToMap(&now_cluster_cloud[i]->points[j], &now_cluster_cloud[i]->points[j]);
        }
    }



    cout << "pose " ;
    for(int i = 0; i < 7; ++i){
        cout << parameters[i] << " ";
    }
    cout << endl;

}



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &rec)
{
    laserCloudMsgQueue.push(rec);
    while (laserCloudMsgQueue.size() > 10)
        laserCloudMsgQueue.pop();
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
    for (int i = 0; i < interestLabel.size(); ++i)
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

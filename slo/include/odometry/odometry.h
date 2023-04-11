#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "../cloud_projection/feature.h"
#include "../io/file_reader.h"
#include "../semantic_icp/semantic_icp.h"
#include "../semantic_icp/semantic_point_cloud.h"
#include "../utility.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

std::map<std::pair<int, int>, Eigen::Isometry3d> load_cache(string path);

void dump_cache(std::map<std::pair<int, int>, Eigen::Isometry3d> cache, string path);


struct LidarPlaneNormFactor {

    LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_)
        : curr_point(curr_point_)
        , plane_unit_norm(plane_unit_norm_)
        , negative_OA_dot_norm(negative_OA_dot_norm_) {}

    template <typename T> bool operator()(const T* q, const T* t, T* residual) const {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
            new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
};

class Odometry {
private:
    string cache;
    std::map<std::pair<int, int>, Eigen::Isometry3d> smatch_cache; 
private:
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2* isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;

private:
    // feature list
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> FeatureCloudKeyFrames;
    pcl::VoxelGrid<pcl::PointXYZL> downSave;
    
    // local map
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudFeatureFromMap;
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserCloudFeatureFromMapDS;
    pcl::VoxelGrid<pcl::PointXYZL> localMapFilter;
    int laserCloudFeatureFromMapDSNum;
    pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtreeSurroundingKeyPoses;
    pcl::VoxelGrid<pcl::PointXYZL> downSizeFilterSurroundingKeyPoses;
    map<uint32_t, pcl::PointCloud<pcl::PointXYZL>> laserCloudMapContainer;

    // pose
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudKeyPoses3D;
    std::vector<Eigen::Isometry3d> keyPoses6D;
    std::vector<double> keyPoses6DTime;

    Eigen::Isometry3d poseLocalMatch;
    Eigen::Isometry3d posePrevious;
    Eigen::Isometry3d poseAfterMapped;
    Eigen::Isometry3d local_pose_increasement;
    double pose_parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr;
    Eigen::Map<Eigen::Vector3d> t_w_curr;
    Eigen::Quaterniond q_wmap_wodom;
    Eigen::Vector3d t_wmap_wodom;
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;

    double timeLaserInfoCur;
    double timeLaserInfoPre;

    // loop closure
    bool aLoopIsClosed;
    pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr loopKdTree;
    pcl::VoxelGrid<pcl::PointXYZL> loopFilter;
    map<int, int> loopIndexContainer;
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> OriCloudKeyFrames;

private:
    pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtreeFeatureFromMap;

private:
    semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>::Ptr OldFrame;
    semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>::Ptr NewFrame;

private:
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserOriCloudLast;
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserOriCloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr laserFeatureCloud;

    pcl::PointCloud<pcl::PointXYZL>::Ptr loopCur;
    pcl::PointCloud<pcl::PointXYZL>::Ptr loopOld;
public:
    pcl::PointCloud<pcl::PointXYZL>::Ptr getloopCur();
    pcl::PointCloud<pcl::PointXYZL>::Ptr getloopOld();

private:
    FeatureExtracter FE;

private:
    FileReader FR;

private:
    pcl::PointCloud<pcl::PointXYZL>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr incloud, Eigen::Matrix4d trans);

private:
    void init();
    void reset();

public:
    Odometry(string bin_p, string label_p, bool intensity, string cache_);
    ~Odometry();
    bool step();
    bool good();
    pcl::PointCloud<pcl::PointXYZL>::Ptr getMap();
    pcl::PointCloud<pcl::PointXYZL>::Ptr getCurrentCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr getCurrentFeatureCloud();
    Eigen::Isometry3d getCurrentPose();
    std::vector<Eigen::Isometry3d> getTraj();
    std::map<int, int> getLoop();
    void control(int min_, int max_);
    double getCurrTime();
};

#endif

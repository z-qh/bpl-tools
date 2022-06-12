#define PCL_NO_PRECOMPILE

#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <dirent.h>

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
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/qhull.h>

#include "odom/common.h"

typedef pcl::PointCloud<PointType> PolygonType;
using SeqId = uint32_t;

class InstanceBuilderOptions
{
public:
    Eigen::Vector3d ref_center;
};

class alignas(16) ModelConfigs
{
public:
    std::string name;
    std::string version;
    std::string matcher_method;
    std::string filter_method;

    int track_cached_history_size_maximum;
    int track_consecutive_invisible_maximum;
    double track_visible_ratio_minimum;
    int collect_age_minimum;
    int collect_consecutive_invisible_maximum;
    int acceleration_noise_maximum;
    double speed_noise_maximum;
    double match_distance_maximum;
    double location_distance_weight;
    double direction_distance_weight;
    double bbox_size_distance_weight;
    double point_num_distance_weight;
    double histogram_distance_weight;
    int histogram_bin_size;
    bool use_adaptive;
    double measurement_noise;
    int initial_velocity_noise;
    int xy_propagation_noise;
    int z_propagation_noise;
    double breakdown_threshold_maximum;
};

template <typename PointT>
void GetCloudMinMax3D(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f *min_point, Eigen::Vector4f *max_point)
{
    Eigen::Vector4f &min_pt = *min_point;
    Eigen::Vector4f &max_pt = *max_point;
    min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
    max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
    if (cloud->is_dense) //是否包含Inf/NaN
    {
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
            max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
            min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
            max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
            min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
            max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
        }
    }
    else
    {
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (!pcl_isfinite(cloud->points[i].x) ||
                !pcl_isfinite(cloud->points[i].y) ||
                !pcl_isfinite(cloud->points[i].z))
            {
                continue;
            }
            min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
            max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
            min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
            max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
            min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
            max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
        }
    }
}

class alignas(16) Instance
{
public:
    Instance();
    // deep copy
    void clone(const Instance &rhs);

    // instance id per frame
    bool update = true;
    int id = 0;
    // point cloud of the instance
    pcl::PointCloud<PointType>::Ptr cloud;
    // convex hull of the instance
    PolygonType polygon;

    // oriented boundingbox information
    // main direction
    Eigen::Vector3d direction = Eigen::Vector3d(1, 0, 0);
    // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
    double theta = 0.0;
    // ground center of the instance (cx, cy, z_min)
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    // size of the oriented bbox, length is the size in the main direction
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
    // shape feature used for tracking
    std::vector<float> shape_features;

    // tracking information
    int instance_id = 0;

    // age of the tracked instance
    double instance_time = 0.0;
    double latest_instance_time = 0.0;
    double timestamp = 0.0;

    // stable anchor_point during time, e.g., barycenter
    Eigen::Vector3d anchor_point;

    // noise covariance matrix for uncertainty of position and velocity
    Eigen::Matrix3d position_uncertainty;

    // modeling uncertainty from sensor level tracker
    Eigen::Matrix4d state_uncertainty = Eigen::Matrix4d::Identity();
    // Tailgating (trajectory of instances)
    std::vector<Eigen::Vector3d> drops;
    // CIPV
    bool b_cipv = false;
    // local lidar track id
    int local_lidar_track_id = -1;
    // local radar track id
    int local_radar_track_id = -1;
    // local camera track id
    int local_camera_track_id = -1;

    // local lidar track ts
    double local_lidar_track_ts = -1;
    // local radar track ts
    double local_radar_track_ts = -1;
    // local camera track ts
    double local_camera_track_ts = -1;

    Eigen::Vector3d vertex1;
    Eigen::Vector3d vertex2;
    Eigen::Vector3d vertex3;
    Eigen::Vector3d vertex4;
    double min_height;
    double max_height;
    
};

bool SaveInstance(std::shared_ptr<Instance> ins, std::string &saveFilePathDir, int saveFileId);

std::shared_ptr<Instance> LoadInstanace(std::string &loadFileName);

bool SaveInstances(std::vector<std::shared_ptr<Instance>> &instances, std::string &saveFilePathDir);

int LoadInstanaces(std::vector<std::shared_ptr<Instance>> &instances, std::string &loadFilePathDir);

// Sensor single frame instances.
class SensorInstances
{
public:
    SensorInstances()
    {
        sensor2world_pose = Eigen::Matrix4d::Zero();
        sensor2world_pose_static = Eigen::Matrix4d::Zero();
    }

    // std::string ToString() const;

    // Transmit error_code to next subnode.
    // common::ErrorCode error_code = common::ErrorCode::OK;

    // SensorType sensor_type = SensorType::UNKNOWN_SENSOR_TYPE;
    // std::string sensor_id;
    double timestamp = 0.0;
    SeqId seq_num = 0;
    std::vector<std::shared_ptr<Instance>> instances;
    Eigen::Matrix4d sensor2world_pose;
    Eigen::Matrix4d sensor2world_pose_static;
    // LaneInstancesPtr lane_instances;

    uint32_t cipv_index = -1;
    uint32_t cipv_track_id = -1;

    // sensor particular supplements, default nullptr
    // RadarFrameSupplementPtr radar_frame_supplement = nullptr;
    // CameraFrameSupplementPtr camera_frame_supplement = nullptr;
};

class BaseInstanceBuilder
{
public:
    BaseInstanceBuilder() {}
    virtual ~BaseInstanceBuilder() {}

    virtual bool Init() = 0;

    virtual bool Build(const InstanceBuilderOptions &options, std::vector<std::shared_ptr<Instance>> *instances) = 0;

    virtual std::string name() const = 0;

protected:
    virtual void SetDefaultValue(pcl::PointCloud<PointType>::Ptr cloud,
                                 std::shared_ptr<Instance> obj,
                                 Eigen::Vector4f *min_pt,
                                 Eigen::Vector4f *max_pt)
    {
        GetCloudMinMax3D<PointType>(cloud, min_pt, max_pt);       //获得目标点云的最大box值,即最小XYZ和最大XYZ,分别存在min_pt max_pt里面
        Eigen::Vector3f center(((*min_pt)[0] + (*max_pt)[0]) / 2, //初始化center点
                               ((*min_pt)[1] + (*max_pt)[1]) / 2,
                               ((*min_pt)[2] + (*max_pt)[2]) / 2);

        // handle degeneration case
        float epslin = 1e-3;
        for (int i = 0; i < 3; i++)
        {
            if ((*max_pt)[i] - (*min_pt)[i] < epslin)
            {
                (*max_pt)[i] = center[i] + epslin / 2;
                (*min_pt)[i] = center[i] - epslin / 2;
            }
        }

        // length
        obj->length = static_cast<double>((*max_pt)[0] - (*min_pt)[0]); //长-X
        // width
        obj->width = static_cast<double>((*max_pt)[1] - (*min_pt)[1]); //宽-Y
        if (obj->length - obj->width < 0)                              //约定最长的边为长,所以长小于宽时交换
        {
            float tmp = obj->length;
            obj->length = obj->width;
            obj->width = tmp;
            obj->direction = Eigen::Vector3d(0.0, 1.0, 0.0);
        }
        else
        {
            obj->direction = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
        // height
        obj->height = static_cast<double>((*max_pt)[2] - (*min_pt)[2]); //高
        // center
        obj->center = Eigen::Vector3d(((*max_pt)[0] + (*min_pt)[0]) / 2, // obj的center点填入
                                      ((*max_pt)[1] + (*min_pt)[1]) / 2,
                                      ((*max_pt)[2] + (*min_pt)[2]) / 2);
        // polygon
        if (cloud->size() < 4) //点云个数小于4时候,就把minbox的边界点定义为minmax3d
        {
            obj->polygon.points.resize(4);
            obj->polygon.points[0].x = static_cast<double>((*min_pt)[0]);
            obj->polygon.points[0].y = static_cast<double>((*min_pt)[1]);
            obj->polygon.points[0].z = static_cast<double>((*min_pt)[2]);

            obj->polygon.points[1].x = static_cast<double>((*max_pt)[0]);
            obj->polygon.points[1].y = static_cast<double>((*min_pt)[1]);
            obj->polygon.points[1].z = static_cast<double>((*min_pt)[2]);

            obj->polygon.points[2].x = static_cast<double>((*max_pt)[0]);
            obj->polygon.points[2].y = static_cast<double>((*max_pt)[1]);
            obj->polygon.points[2].z = static_cast<double>((*min_pt)[2]);

            obj->polygon.points[3].x = static_cast<double>((*min_pt)[0]);
            obj->polygon.points[3].y = static_cast<double>((*max_pt)[1]);
            obj->polygon.points[3].z = static_cast<double>((*min_pt)[2]);
        }
    }
};

class MinBoxInstanceBuilder : public BaseInstanceBuilder
{
public:
    MinBoxInstanceBuilder() : BaseInstanceBuilder() {}
    virtual ~MinBoxInstanceBuilder() {}

    bool Init() override { return true; }

    bool Build(const InstanceBuilderOptions &options,
               std::vector<std::shared_ptr<Instance>> *instances) override;
    std::string name() const override { return "MinBoxInstanceBuilder"; }

public:
    void BuildInstance(InstanceBuilderOptions options,
                       std::shared_ptr<Instance> instance);

    void ComputePolygon2dxy(std::shared_ptr<Instance> obj);

    double ComputeAreaAlongOneEdge(std::shared_ptr<Instance> obj,
                                   size_t first_in_point, Eigen::Vector3d *center,
                                   double *lenth, double *width,
                                   Eigen::Vector3d *dir);

    void ReconstructPolygon(const Eigen::Vector3d &ref_ct,
                            std::shared_ptr<Instance> obj);

    void ComputeGeometricFeature(const Eigen::Vector3d &ref_ct,
                                 std::shared_ptr<Instance> obj);

    void TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloud,
                             const std::vector<int> &indices,
                             pcl::PointCloud<PointType> *trans_cloud);
};

template <typename PointInT>
class ConvexHull2DXY : public pcl::ConvexHull<PointInT>
{
public:
    typedef boost::shared_ptr<pcl::ConvexHull<PointInT>> Ptr;
    typedef boost::shared_ptr<const pcl::ConvexHull<PointInT>> ConstPtr;
    typedef pcl::PointCloud<PointInT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    using pcl::ConvexHull<PointInT>::reconstruct;
    using pcl::ConvexHull<PointInT>::compute_area_;
    using pcl::ConvexHull<PointInT>::total_area_;
    using pcl::ConvexHull<PointInT>::total_volume_;
    using pcl::ConvexHull<PointInT>::qhull_flags;

    ConvexHull2DXY() = default;
    virtual ~ConvexHull2DXY() = default;

    void Reconstruct2dxy(PointCloudPtr hull,
                         std::vector<pcl::Vertices> *polygons)
    {
        hull->header = input_->header;
        if (!initCompute() || input_->points.empty() || indices_->empty())
        {
            hull->points.clear();
            return;
        }

        PerformReconstruction2dxy(hull, polygons, true);

        hull->width = static_cast<uint32_t>(hull->points.size());
        hull->height = 1;
        hull->is_dense = true;

        deinitCompute();
    }

    void PerformReconstruction2dxy(PointCloudPtr hull,
                                   std::vector<pcl::Vertices> *polygons,
                                   bool fill_polygon_data = false)
    {
        int dimension = 2;

        // True if qhull should free points in qh_freeqhull() or reallocation
        // boolT ismalloc = True;//PCL1.10 qhull.h has no True = 1
        boolT ismalloc = 1;
        // output from qh_produce_output(), use NULL to skip qh_produce_output()
        FILE *outfile = NULL;

#ifndef HAVE_QHULL_2011
        if (compute_area_)
        {
            outfile = stderr;
        }
#endif

        // option flags for qhull, see qh_opt.htm
        const char *flags = qhull_flags.c_str();
        // error messages from qhull code
        FILE *errfile = stderr;

        // Array of coordinates for each point
        coordT *points = reinterpret_cast<coordT *>(
            calloc(indices_->size() * dimension, sizeof(coordT)));
        if (points == NULL)
        {
            hull->points.resize(0);
            hull->width = hull->height = 0;
            polygons->resize(0);
            return;
        }

        // Build input data, using appropriate projection
        int j = 0;
        for (size_t i = 0; i < indices_->size(); ++i, j += dimension)
        {
            points[j + 0] = static_cast<coordT>(input_->points[(*indices_)[i]].x);
            points[j + 1] = static_cast<coordT>(input_->points[(*indices_)[i]].y);
        }

        // Compute convex hull
        int exitcode =
            qh_new_qhull(dimension, static_cast<int>(indices_->size()), points,
                         ismalloc, const_cast<char *>(flags), outfile, errfile);
#ifdef HAVE_QHULL_2011
        if (compute_area_)
        {
            qh_prepare_output();
        }
#endif

        // 0 if no error from qhull or it doesn't find any vertices
        if (exitcode != 0 || qh num_vertices == 0)
        {
            PCL_ERROR(
                "[pcl::%s::performReconstrution2D] "
                "ERROR: qhull was unable to compute "
                "a convex hull for the given point "
                "cloud (%lu)!\n",
                getClassName().c_str(), indices_->size());

            hull->points.resize(0);
            hull->width = hull->height = 0;
            polygons->resize(0);

            qh_freeqhull(!qh_ALL);
            int curlong, totlong;
            qh_memfreeshort(&curlong, &totlong);
            return;
        }

        // Qhull returns the area in volume for 2D
        if (compute_area_)
        {
            total_area_ = qh totvol;
            total_volume_ = 0.0;
        }

        int num_vertices = qh num_vertices;
        hull->points.resize(num_vertices);
        memset(&hull->points[0], static_cast<int>(hull->points.size()),
               sizeof(PointInT));

        vertexT *vertex;
        int i = 0;

        std::vector<std::pair<int, Eigen::Vector4f>,
                    Eigen::aligned_allocator<std::pair<int, Eigen::Vector4f>>>
            idx_points(num_vertices);
        idx_points.resize(hull->points.size());
        memset(&idx_points[0], static_cast<int>(hull->points.size()),
               sizeof(std::pair<int, Eigen::Vector4f>));

        FORALLvertices
        {
            hull->points[i] = input_->points[(*indices_)[qh_pointid(vertex->point)]];
            idx_points[i].first = qh_pointid(vertex->point);
            ++i;
        }

        // Sort
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*hull, centroid);
        for (size_t j = 0; j < hull->points.size(); j++)
        {
            idx_points[j].second[0] = hull->points[j].x - centroid[0];
            idx_points[j].second[1] = hull->points[j].y - centroid[1];
        }

        std::sort(idx_points.begin(), idx_points.end(), pcl::comparePoints2D);

        polygons->resize(1);
        (*polygons)[0].vertices.resize(hull->points.size());

        for (int j = 0; j < static_cast<int>(hull->points.size()); j++)
        {
            hull->points[j] = input_->points[(*indices_)[idx_points[j].first]];
            (*polygons)[0].vertices[j] = static_cast<unsigned int>(j);
        }

        qh_freeqhull(!qh_ALL);
        int curlong, totlong;
        qh_memfreeshort(&curlong, &totlong);

        hull->width = static_cast<uint32_t>(hull->points.size());
        hull->height = 1;
        hull->is_dense = false;
        return;
    }

    std::string getClassName() const { return ("ConvexHull2DXY"); }

protected:
    using pcl::PCLBase<PointInT>::input_;
    using pcl::PCLBase<PointInT>::indices_;
    using pcl::PCLBase<PointInT>::initCompute;
    using pcl::PCLBase<PointInT>::deinitCompute;
};

#endif // _INSTANCE_H_

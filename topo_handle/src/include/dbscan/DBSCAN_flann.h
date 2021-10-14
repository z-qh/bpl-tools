#ifndef DBSCAN_FLANN_H
#define DBSCAN_FLANN_H

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "nanoflann.hpp"
#include "nanoflannutils.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <ctime>
#include <omp.h>
using namespace nanoflann;
#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClustersFlann (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}
template <typename PointT>
class DBSCANFlannCluster {
public:
    DBSCANFlannCluster(){
        srand((unsigned int)time(0));
    }
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    typedef typename pcl::KdTreeFLANN<PointT>::Ptr KdFLANNPtr;
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdFLANNPtr tree) {
        search_method_ = tree;
    }
    void extract(std::vector<pcl::PointIndices>& cluster_indices){
        std::vector<int> nn_indices;        //index
        std::vector<float> nn_distances;    //distance
        int N = input_cloud_->points.size();
        std::vector<bool> is_noise(N, false);
        std::vector<int> types(N, UN_PROCESSED);
        double time = ros::Time::now().toSec();
        PointCloud<float>cloud;
        
        generateRandomPointCloud(cloud,N, input_cloud_);
        // std::cout<<"input_cloud_ size "<<input_cloud_->points.size()<<" cloud "<<cloud.pts.size()<<std::endl;

        int noise_number = 0;
        int min_number = 0;
        typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>,3>my_kd_tree_t;
        my_kd_tree_t index(3,cloud,KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();         //完成构建kd树
        int pointNumber = input_cloud_->points.size();
        
        for(int i=0; i<pointNumber; ++i){
            if (types[i] == PROCESSED || is_noise[i]) {	//已经聚类过就不在聚类
                continue;
            }
            PointT thisPoint = input_cloud_->points[i];
            double depth = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
            float theta_res = max(horizon_angle_resolution_, vertical_angle_resolution_);
            // float threshold = max(s_*(depth*tan(theta_res/180.0*M_PI)),eps_);
            float threshold = max(s_*(horizon_angle_resolution_*M_PI/180.0)*depth, eps_);
            float query_pt[3]={thisPoint.x, thisPoint.y, thisPoint.z};
            std::vector<std::pair<size_t,float>> indices_dists;

            nanoflann::SearchParams params;
            params.sorted = false;
            double time = ros::Time::now().toSec();
            const size_t nMatches = index.radiusSearch(&query_pt[0], threshold, indices_dists, params);
            // std::cout<<"cost time-- "<<std::to_string(ros::Time::now().toSec() - time)<<"match number "<<nMatches<<std::endl;
            int nnSize = nMatches;
            if(nnSize < minPts_){
                is_noise[i] = true;
                noise_number++;
                continue;
            }
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            for(size_t tmp_index = 0; tmp_index<nMatches; ++tmp_index){
                if(indices_dists[tmp_index].first !=i){
                    seed_queue.push_back(indices_dists[tmp_index].first);
                    types[indices_dists[tmp_index].first] = PROCESSING;
                }
            }
            // for every point near the chosen core point.
            int sq_idx = 0;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {//if this point is noise or PROCESSED no need to check neighbors
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                // nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                PointT thisPoint = input_cloud_->points[cloud_index];
                double depth = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
                float theta_res = max(horizon_angle_resolution_, vertical_angle_resolution_);
                double threshold = max(s_*(depth*tan(theta_res/180.0*M_PI)),eps_);
                // double threshold = max((depth*sin(theta_res/2.0/180.0*M_PI)*2),eps_);
                // double threshold = max((depth*sin(ang_res_y/2.0/180.0*M_PI)*2),eps_);    //16线激光使用该阈值，ang_res_y=2

                std::vector<std::pair<size_t,float>> indices_dists;
                nanoflann::SearchParams params;
                params.sorted = false;
                float query_pt[3]={thisPoint.x, thisPoint.y, thisPoint.z};
                const size_t nMatches = index.radiusSearch(&query_pt[0], threshold, indices_dists, params);


                int nnSize = nMatches;
                // nn_size = radiusSearch(cloud_index, threshold, nn_indices, nn_distances);
                if (nnSize >= minPts_) {
                    for(size_t tmp_index = 0; tmp_index<nMatches; ++tmp_index){
                        if(types[indices_dists[tmp_index].first] == UN_PROCESSED){
                            seed_queue.push_back(indices_dists[tmp_index].first);
                            types[indices_dists[tmp_index].first] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // std::cout<<"r size "<<r.indices.size()<<std::endl;
                // These two lines should not be needed: (can anyone confirm?) -FF
                //Remove duplicate indexes

                // std::sort (r.indices.begin (), r.indices.end ());
                // r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
                
                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
            // if(seed_queue.size()>=max_pts_per_cluster_){
            //     std::cout<<"seed_queue size is "<<seed_queue.size()<<" max "<<max_pts_per_cluster_<<endl;
            // }
            // if(seed_queue.size()<min_pts_per_cluster_){
            //     min_number++;
            //     // std::cout<<"seed_queue size is "<<seed_queue.size()<<" max "<<min_pts_per_cluster_  <<endl;
            // }
        }
        index.freeIndex(index);
        // std::cout<<"noise number is "<<noise_number<<" less min_pts "<<min_number<<std::endl;
        // std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClustersFlann);

    }
    //设定搜索半径
    void setClusterTolerance(double tolerance) {
        eps_ = tolerance; 
    }
    //设定最小聚类数目
    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }
    //设定最大聚类数目
    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    //设定最小核心点数目
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }
    //水平方向的分辨率
    void setHorizonAngleResolution(float angle_resolution){
        horizon_angle_resolution_ = angle_resolution;
    }
    //设定垂直方向上的分辨率
    void setVerticalAngleResolution(float angle_resolution){
        vertical_angle_resolution_ = angle_resolution;
    }

protected:
    PointCloudPtr input_cloud_;
    
    double eps_ {0.0};                                              //搜索半径
    float horizon_angle_resolution_;                                //三维激光水平方向角分辨率
    float vertical_angle_resolution_;                               //三维激光垂直方向角分辨率
    int minPts_ {1}; // not including the point itself.             //以点为半径进行搜索时，半径内数目小于该点，则设为噪声点
    int min_pts_per_cluster_ {1};                                   //每类点数目最小值
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};     //每类点数目最大值
    float s_{1.5};

    KdFLANNPtr search_method_;

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);
        k_sqr_distances.push_back(0);
        int size = input_cloud_->points.size();
        double radius_square = radius * radius;
        for (int i = 0; i < size; i++) {
            if (i == index) {
                continue;
            }
            double distance_x = input_cloud_->points[i].x - input_cloud_->points[index].x;
            double distance_y = input_cloud_->points[i].y - input_cloud_->points[index].y;
            double distance_z = input_cloud_->points[i].z - input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
            if (distance_square <= radius_square) {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));
            }
        }
        return k_indices.size();
    }
}; // class DBSCANCluster

#endif // DBSCAN_H

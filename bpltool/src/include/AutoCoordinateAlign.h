#ifndef _AUTOCOORDINATEALIGN_H_
#define _AUTOCOORDINATEALIGN_H_

#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <cmath>
#include <time.h>
#include <utility>
#include <stdlib.h>
#include <time.h> 
#include <unordered_set>
template <typename T>
T add(T a, T b){
	T c;
	c.x = a.x+b.x;
	c.y = a.y+b.y;
	c.z = a.z+b.z;
	return std::move(c);
}
template <typename T>
T subtraction(T a, T b){
	T c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return std::move(c);
}
//pcl::copyPointCloud(*source,*target)
class AutoCoordinateAlign{
public:
    AutoCoordinateAlign(){
        source_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        target_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        correspondences_source_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        correspondences_target_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        current_iterate_ = 0;
    }
    using PointCloudPtr = typename pcl::PointCloud<pcl::PointXYZ>::Ptr;
    void setMaximumIterations(int iterate){	iterate_ = std::max(iterate,50);}
	void setMaxCorrespondenceDistance(double threshould){threshould_ = threshould;}
	void setfitnessEpsilon(double fitnessEpsilon){fitnessEpsilon_ = fitnessEpsilon;}
	double getFitnessScore(){	return fitnessScore_;}
	bool hasCoverged(){	return hasCoverged_;}
	Eigen::Matrix4d getFinalTransformation(){	return finalTransformation_;}
    void setInputCloud(PointCloudPtr& cloud_source);
    void setInputTarget(PointCloudPtr& cloud_target);
    bool calNearestPointPairs();
    bool autoAlign();

    void reset(){
        source_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        target_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        correspondences_source_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        correspondences_target_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        finalTransformation_ = Eigen::Matrix4d::Identity(); 
    }
    int current_iterate_;
private:
	PointCloudPtr source_points_;	//源点云
	PointCloudPtr target_points_;	//目标点云
	PointCloudPtr correspondences_source_;
	PointCloudPtr correspondences_target_;

    
	int iterate_;
	double threshould_;//设定的点对之间的距离
	double fitnessEpsilon_;
	double fitnessScore_;
	bool hasCoverged_ = false;
	int correspondences_minnumber = 30;
	Eigen::Matrix4d finalTransformation_ = Eigen::Matrix4d::Identity(); 
};
void AutoCoordinateAlign::setInputCloud(PointCloudPtr& cloud_source){
    source_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    correspondences_source_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_source,*source_points_);
}
void AutoCoordinateAlign::setInputTarget(PointCloudPtr& cloud_target){
    target_points_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    correspondences_target_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_target,*target_points_);
}
bool AutoCoordinateAlign::calNearestPointPairs(){
    correspondences_source_->clear();
	correspondences_target_->clear();
    int pairs_number = source_points_->points.size();
    if(pairs_number >0 && pairs_number!=(int)target_points_->points.size()){
        std::cout<<"error the source points number is not equals to the target points number"<<std::endl;
        return false;
    }
    pcl::copyPointCloud(*source_points_,*correspondences_source_);
    pcl::copyPointCloud(*target_points_,*correspondences_target_);
    // std::cout<<"source_points size "<<correspondences_source_->points.size()<<" target "<<correspondences_target_->points.size()<<std::endl;
    return true;
}
bool AutoCoordinateAlign::autoAlign(){
    current_iterate_ = 0;
    if(!calNearestPointPairs()){
        return false;
    }
    while(current_iterate_< 1){
        pcl::PointXYZ  source_center,target_center;
        source_center.x = 0;source_center.y = 0;source_center.z = 0;
        target_center.x = 0;target_center.y = 0;target_center.z = 0;
        
        for(size_t i=0;i<correspondences_source_->points.size();++i){
            source_center = add(source_center,correspondences_source_->points[i]);
            target_center = add(target_center,correspondences_target_->points[i]);
        }
        source_center.x = source_center.x/((int)correspondences_source_->points.size()*1.0);	//求质心
        source_center.y = source_center.y/((int)correspondences_source_->points.size()*1.0);
        source_center.z = source_center.z/((int)correspondences_source_->points.size()*1.0);
        target_center.x = target_center.x/((int)correspondences_source_->points.size()*1.0);
        target_center.y = target_center.y/((int)correspondences_source_->points.size()*1.0);
        target_center.z = target_center.z/((int)correspondences_source_->points.size()*1.0);
        // std::cout<<"source center is x "<<source_center.x<<" y "<<source_center.y<<" z "<<source_center.z<<
        // " target center x "<<target_center.x<<" y "<<target_center.y<<" z "<<target_center.z<<std::endl;
        //点集去除质心后的坐标
        pcl::PointCloud<pcl::PointXYZ>::Ptr remove_center_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr remove_center_target(new pcl::PointCloud<pcl::PointXYZ>());

        remove_center_source->resize(correspondences_source_->points.size());
        remove_center_target->resize(correspondences_source_->points.size());

        for(size_t i=0; i<correspondences_source_->points.size();++i){
            remove_center_source->points[i] = subtraction(correspondences_source_->points[i],source_center);	//去除中心点的源点云
            remove_center_target->points[i] = subtraction(correspondences_target_->points[i],target_center);	//去除中心点的目标点云
        }
        int count = correspondences_source_->points.size();
        // for(int i= 0;i<count;++i){
        //     std::cout<<"source is x "<<remove_center_source->points[i].x<<" y "<<remove_center_source->points[i].y<<" z "<<remove_center_source->points[i].z<<
        //     " target x "<<remove_center_target->points[i].x<<" y "<<remove_center_target->points[i].y<<" z "<<remove_center_target->points[i].z<<std::endl;
        // }
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for(int i=0;i<remove_center_source->points.size();++i){
            pcl::PointXYZ p = (remove_center_source->points[i]);
            pcl::PointXYZ q = (remove_center_target->points[i]);
            //协方差矩阵为 原点云*目标点云的转置
            W += Eigen::Vector3d(p.x,p.y,p.z)*(Eigen::Vector3d(q.x,q.y,q.z).transpose());
            // W += Eigen::Vector3d(q.x,q.y,q.z)*(Eigen::Vector3d(p.x,p.y,p.z).transpose());
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d R = V*(U.transpose());
        // std::cout<<"R\n"<<R<<std::endl;
        // std::cout<<"target x"<<target_center.x<<" y "<<target_center.y<<" z "<<target_center.z<<std::endl;
        // std::cout<<"source x"<<source_center.x<<" y "<<source_center.y<<" z "<<source_center.z<<std::endl;
        Eigen::Vector3d testsource = R * Eigen::Vector3d(source_center.x,source_center.y,source_center.z);
        // std::cout<<"testsource x "<<testsource(0)<<" y "<<testsource(1)<<" z "<<testsource(2)<<std::endl;
        Eigen::Vector3d T = Eigen::Vector3d(target_center.x,target_center.y,target_center.z)-R*Eigen::Vector3d(source_center.x,source_center.y,source_center.z);

        finalTransformation_.block<3, 3>(0, 0) = std::move(R);   //R
        finalTransformation_.block<3, 1>(0, 3) = std::move(T);	  //t
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr testSource(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*correspondences_source_,*testSource,finalTransformation_);
        double distance_sum = 0;
        for(int i=0;i<testSource->points.size();++i){
            distance_sum += sqrt((testSource->points[i].x - correspondences_target_->points[i].x )*(testSource->points[i].x - correspondences_target_->points[i].x)+
                                (testSource->points[i].y - correspondences_target_->points[i].y )*(testSource->points[i].y - correspondences_target_->points[i].y)+
                                (testSource->points[i].z - correspondences_target_->points[i].z )*(testSource->points[i].z - correspondences_target_->points[i].z));
        }
        distance_sum = distance_sum/((int)testSource->points.size()*1.0);
        return true;
	    
        if(distance_sum<fitnessEpsilon_){
            fitnessScore_ = distance_sum;
            hasCoverged_ = true;
            break;
        }
    }
}
#endif

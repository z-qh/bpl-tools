#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include "data_type.h"
#include "LidarFrame.h"
#include "Geometry.h"
#include "Parameter.h"

#include "opencv2/opencv.hpp"

#include "pcl/registration/icp_nl.h"
#include "pcl/filters/voxel_grid.h"

/*
 * LidarOdometry
 */
class LidarOdometry{
public:

public:
    LidarFrame::Ptr LastFrame;
    LidarFrame::Ptr nowFrame;
public:
    LidarOdometry() = delete;
    LidarOdometry(const Parameter& param){

    }
    /*
     * set new frame and get the new pose
     */
    bool setNewInputLidarFrame(LidarFrame::Ptr input_frame){
        if(!LastFrame){
            LastFrame = input_frame;
            return true;
        }else{
            semanticCloudPtr oldCloud(new semanticCloud());
            semanticCloudPtr newCloud(new semanticCloud());
            semanticCloudPtr unuse(new semanticCloud());
            pcl::VoxelGrid<PointSemantic>downSizeFilter;
            downSizeFilter.setLeafSize(3, 3, 3);
            downSizeFilter.setInputCloud(LastFrame->mCloud);
            downSizeFilter.filter(*oldCloud);
            downSizeFilter.setInputCloud(input_frame->mCloud);
            downSizeFilter.filter(*newCloud);

            pcl::IterativeClosestPointNonLinear<PointSemantic,PointSemantic> icp;
            icp.setMaximumIterations(100);
            icp.setInputSource(oldCloud);
            icp.setInputTarget(newCloud);
            icp.align(*unuse);
            Eigen::Matrix4d betweenT = icp.getFinalTransformation();
            Eigen::Matrix4d newT = betweenT * LastFrame->mPose.Transform;
            input_frame->mPose.setPose(newT);

            return true;
        }
    }

public:
    using Ptr = std::shared_ptr<LidarOdometry>;
    using ConstPtr = std::shared_ptr<const LidarOdometry>;
};



#endif
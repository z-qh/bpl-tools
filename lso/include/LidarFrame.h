#ifndef FRAME_H_
#define FRAME_H_

#include "Geometry.h"
#include "data_type.h"
#include "ClusterCVC.h"


/*
 * LidarFrame
 */
class LidarFrame {
public:
    semanticCloudPtr mCloud;
public:
    Pose mPose;
    double mTime = 0;

public:
    LidarFrame() = delete;
    LidarFrame(semanticCloudPtr lidar_cloud, double time){
        mCloud = lidar_cloud;
        mTime = time;
        instantiation();
    }
    bool instantiation(){
        if(mCloud->empty()) return false;


        return true;
    }
public:
    using Ptr = std::shared_ptr<LidarFrame>;
    using ConstPtr = std::shared_ptr<const LidarFrame>;
};
#endif

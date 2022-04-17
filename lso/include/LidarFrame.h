#ifndef FRAME_H_
#define FRAME_H_

#include "Geometry.h"
#include "data_type.h"


/*
 * Frame
 */
class Frame {
public:
    semanticCloudPtr mCloud;// Cloud for tracking Down sampled
    semanticCloudPtr mGroundCloud;// Ground Cloud
    semanticCloudPtr mEnviCloud;// Environment Cloud
public:
    Pose mPose;// Posture
    double mTime = 0;// Time

public:
    Frame(){

    }
public:
    using Ptr = std::shared_ptr<Frame>;
    using ConstPtr = std::shared_ptr<const Frame>;
};
#endif

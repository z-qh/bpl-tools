#ifndef SEMANTICMARK_H_
#define SEMANTICMARK_H_

#include "Base.h"

namespace lso{
    /*
     * SemanticMark
     */
    class SemanticMark{
    public:
        int mSemanticFlag = 0;// Semantic Label
        Eigen::Vector3d mSize = Eigen::Vector3d::Zero();// Clouds Cluster Size
        Pose mPose;// Posture

    public:
        semanticCloudPtr mCloud;// Geometry-Semantic Cloud

    public:
        SemanticMark(){
            mCloud.reset(new semanticCloud());
        }
        // Initialize
        SemanticMark(const semanticCloudPtr& cloud_, const Pose &pose_, const Eigen::Vector3d &size_)
        {
            mCloud.reset(new semanticCloud());
            *mCloud = *cloud_;// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            mPose = pose_;
            mSemanticFlag = mCloud->points.front().label;
            mSize = size_;
        }
        SemanticMark(const SemanticMark& mark){
            mCloud.reset(new semanticCloud());
            *mCloud = *mark.mCloud;// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            mPose = mark.mPose;
            mSemanticFlag = mark.mSemanticFlag;
            mSize = mark.mSize;
        }

    public:
        using Ptr = std::shared_ptr<SemanticMark>;
        using ConstPtr = std::shared_ptr<const SemanticMark>;
    };

}

#endif


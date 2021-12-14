#ifndef FRAME_H_
#define FRAME_H_

#include "SemanticMark.h"
#include "Base.h"



namespace lso {
    /*
     * Frame
     */
    class Frame {
    public:
        semanticCloudPtr mCloud;// Cloud for tracking Down sampled
        geometryCloudPtr mGroundCloud;// Ground Cloud
        geometryCloudPtr mEnviCloud;// Environment Cloud
    public:
        Pose mPose;// Posture
        double mTime = 0;// Time

    public:
        std::set<lso::SemanticMark::Ptr> VisibleSMarks;// Relative Semantic Marks

    public:
        Frame(){
            mCloud.reset(new semanticCloud());
            mGroundCloud.reset(new geometryCloud());
            mEnviCloud.reset(new geometryCloud());
        }
        Frame(const Frame &frame){
            mCloud.reset(new semanticCloud());
            mGroundCloud.reset(new geometryCloud());
            mEnviCloud.reset(new geometryCloud());
            *mCloud = *(frame.mCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mGroundCloud = *(frame.mGroundCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mEnviCloud = *(frame.mEnviCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            mPose = frame.mPose;
            mTime = frame.mTime;
            VisibleSMarks = frame.VisibleSMarks;
        }
        Frame(semanticCloudPtr rawCloud_, semanticCloudPtr groundCloud_, semanticCloudPtr enviCloud_, double time_){
            mCloud.reset(new semanticCloud());
            mGroundCloud.reset(new geometryCloud());
            mEnviCloud.reset(new geometryCloud());
            *mCloud = *rawCloud_;// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mGroundCloud = *groundCloud_;// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mEnviCloud = *enviCloud_;// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            mTime = time_;
        }

    public:
        void AddVisibleSMark(SemanticMark *SMark);
        void EraseVisibleSMark(SemanticMark *SMark);
        using Ptr = std::shared_ptr<Frame>;
        using ConstPtr = std::shared_ptr<const Frame>;
    };
}

namespace lso{
    /*
     * KeyFrame
     */
    class KeyFrame : public Frame {
    public:
        int Ind = 0;// Global index

    public:
        KeyFrame(const KeyFrame &KF){
            *mCloud = *(KF.mCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mGroundCloud = *(KF.mGroundCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mEnviCloud = *(KF.mEnviCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            mPose = KF.mPose;
            mTime = KF.mTime;
            VisibleSMarks = KF.VisibleSMarks;
        }
        KeyFrame(const Frame &frame){
            *mCloud = *(frame.mCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mGroundCloud = *(frame.mGroundCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            *mEnviCloud = *(frame.mEnviCloud);// Deep Copy to Reduce Problem , Should be Normal Copy to Reduce Time
            VisibleSMarks = frame.VisibleSMarks;
            mPose = frame.mPose;
            mTime = frame.mTime;
            VisibleSMarks = frame.VisibleSMarks;
        }
        using Ptr = std::shared_ptr<KeyFrame>;
        using ConstPtr = std::shared_ptr<const KeyFrame>;
    };
}
#endif

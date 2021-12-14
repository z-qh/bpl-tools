#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "Base.h"
#include "SemanticMark.h"
#include "Frame.h"
#include "Map.h"


namespace Optimizer{

    /*
     * Optimize The new Frame Pose between adjacent frames
     */
    void PoseOptimization(const lso::Frame::Ptr& now_, const lso::Frame::Ptr last_){

    }

    /*
     * Optimize Global SMarks Posture and Global Frames
     */
    void GlobalPoseAndMarkOptimization(const lso::Map::Ptr map_){

    }

    /*
     * Optimize Local Smarks Posture and Frame Pose
     */
    void LocalPoseAndMarkOptimization(const lso::Map::Ptr localMap_){

    }

    /*
     * Optimize the New Frame Pose in Local Map
     */
    void PoseOptimizationLocal(const lso::Map::Ptr localMap_, const lso::Frame::Ptr now_){

    }

    /*
     * Optimize the Loop Closure
     */
    void LoopOptimization(const lso::Map::Ptr map_, const lso::Frame::Ptr loop_){

    }
}

#endif
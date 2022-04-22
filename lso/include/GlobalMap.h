#ifndef GLOBAL_MAP_H_
#define GLOBAL_MAP_H_

#include "map"

#include "Geometry.h"
#include "LidarFrame.h"
#include "data_type.h"



class GlobalMap{
public:
    int KeyFrameNum = 0;
    std::map<LidarFrame::Ptr, int> KeyFrames;
public:
    GlobalMap(){

    }

public:
    using Ptr = std::shared_ptr<GlobalMap>;
    using ConstPtr = std::shared_ptr<const GlobalMap>;
};


#endif
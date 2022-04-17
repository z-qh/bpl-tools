#ifndef LOCAL_MAP_H_
#define LOCAL_MAP_H_

#include "map"
#include "deque"
#include "iterator"

#include "Geometry.h"
#include "LidarFrame.h"
#include "data_type.h"

/*
 * LocalMap
 */

class LocalMap{
public:
    int KeyFrameID;
    int KeyFrameCount;
    std::map<LidarFrame::Ptr, int> KeyFramesMap;
    std::deque<LidarFrame::Ptr> KeyFramesList;
public:
    LocalMap(){

    }

    bool insert(LidarFrame::Ptr new_frame){
        auto it = KeyFramesMap.find(new_frame);
        if(it == KeyFramesMap.end()){
            KeyFramesMap.insert(std::make_pair(new_frame, KeyFrameID));
            KeyFramesList.push_back(new_frame);
            KeyFrameID++;
            KeyFrameCount++;
        }else{
            return false;
        }
    }

    bool removeOld(){
        LidarFrame::Ptr oldFrame = KeyFramesList.front();
        auto it = KeyFramesMap.find(oldFrame);
        if(it != KeyFramesMap.end()){
            KeyFramesMap.erase(it);
            KeyFrameCount--;
        }else{
            KeyFrameCount--;
        }
        KeyFramesList.pop_front();
    }

public:
    using Ptr = std::shared_ptr<LocalMap>;
    using ConstPtr = std::shared_ptr<const LocalMap>;
};

#endif
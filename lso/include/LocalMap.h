#ifndef LSO_MAP_H_
#define LSO_MAP_H_

#include "Geometry.h"
#include "Frame.h"
#include "data_type.h"


namespace lso{
    class Map{
    public:
        int KeyFrameNum = 0;
        map<int, Frame::Ptr> KeyFrames;
    public:
        Map(){

        }

    public:
        using Ptr = std::shared_ptr<Map>;
        using ConstPtr = std::shared_ptr<const Map>;
    };
}

#endif
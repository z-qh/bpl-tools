#ifndef PARAMETER_H
#define PARAMETER_H

#include "unistd.h"

#include "iostream"
#include "string"
#include "sstream"
#include "ctime"
#include "tinyxml2.h"
/*
 * Parameter
 */
class Parameter{
public:
    /*
     * from kitti timestamps get the times
     */
    static double convertTime(const std::string& thisTimeStamp){
        struct tm tm;
        memset(&tm, 0, sizeof(tm));
        double msec = 0;
        sscanf(thisTimeStamp.c_str(), "%d-%d-%d %d:%d:%lf",
               &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
               &tm.tm_hour, &tm.tm_min, &msec);
        tm.tm_sec = int(msec);
        msec -= int(msec);
        tm.tm_year -= 1900;
        tm.tm_mon--;
        return mktime(&tm)+msec;
    }

public:
    std::string DataBinPath;
    std::string DataLabelPath;

public:
    Parameter(){
        DataBinPath = "/home/qh/kitti/01/velodyne";
        DataLabelPath = "/home/qh/kitti/01/labels";
    }

    using Ptr = std::shared_ptr<Parameter>;
    using ConstPtr = std::shared_ptr<const Parameter>;

};



#endif
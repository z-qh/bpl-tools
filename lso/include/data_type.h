#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_

#include "unistd.h"
#include "sys/types.h"
#include "pcl/point_cloud.h"
#include "pcl/point_cloud.h"

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, label, label)
)

typedef PointXYZIL PointSemantic;
typedef PointXYZIL PointType;
using semanticCloud = pcl::PointCloud<PointSemantic>;
using semanticCloudPtr = semanticCloud::Ptr;

#endif
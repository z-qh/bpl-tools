#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <memory>
#include <time.h>

#include "mot/types.h"
#include "mot/pcl_clip/pcl_clip.h"
#include "mot/remove_ground/remove_ground.h"
#include "mot/cluster/region_growing_segmentation.h"
#include "mot/cluster/EuclideanCluster.h"
#include "mot/cluster/dbscan.h"
#include "mot/min_box/min_box.h"
#include "mot/hm_tracker/hm_tracker.h"
#include "mot/object.h"


int main(){
    
}
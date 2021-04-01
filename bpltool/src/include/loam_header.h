#include "ros/ros.h"                        //ros核心头文件
#include "sensor_msgs/Imu.h"                //传感器-IMU
#include "sensor_msgs/PointCloud2.h"        //传感器-点云
#include "nav_msgs/Odometry.h"              //导航-里程计
#include "opencv2/imgproc.hpp"              //ros-opencv-图像处理
#include "pcl/point_cloud.h"                //pcl-点云
#include "pcl/point_types.h"                //pcl-点类型
#include "pcl_ros/point_cloud.h"            //pcl-ros-点云
#include "pcl_conversions/pcl_conversions.h"//pcl-转换
#include "pcl/filters/filter.h"             //pcl-滤波器
#include "pcl/filters/voxel_grid.h"         //pcl-体素
#include "pcl/kdtree/kdtree_flann.h"        //pcl-KD树-最邻近匹配
#include "pcl/common/common.h"              //pcl-common
#include "pcl/registration/icp.h"           //pcl-icp匹配
#include <vector>                           //vector
#include <iostream>                         //标准输入输出流
#include <cmath>                            //数学函数库
#include <algorithm>                        //常用算法
#include <queue>                            //队列
#include <deque>                            //双向队列
#include <fstream>                          //文件流
#include <ctime>                            //时间
#include <cfloat>                           //float限制
#include <iterator>                         //迭代器
#include <sstream>                          //字符串流
#include <string>                           //字符串
#include <limits>                           //系统限制
#include <iomanip>                          //操纵器
#include <array>                            //内置数组
#include <thread>                           //多线程
#include <mutex>                            //线程锁

#define PI 3.14159265

using namespace std;
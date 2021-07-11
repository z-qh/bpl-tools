#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;
//创建pub对象，全局变量，分配内存
ros::Publisher map_puber;
//障碍的数量和椭圆圈的数量
int obs_num, cir_num;
//用来衡量地图尺寸的变量，地图尺寸，起始点，分辨率，地图频率
double x_size, y_size, z_size, init_x, init_y, resolution, sense_rate;
//用来圈定各种物体的尺寸的变量，障碍物的参数，椭圆的参数
double x_l, x_h, y_l, y_h, w_l, w_h, h_l, h_h, w_c_l, w_c_h;
//地图创建标志位
bool has_map  = false;
//用来作为发布消息的媒介
sensor_msgs::PointCloud2 globalMap_pcd;
//用来保存生成好的3维的地图，不过这地图用点云来表示仅仅是为了方便ros之间传输三维地图
pcl::PointCloud<pcl::PointXYZ> cloudMap;
//这个也是地图，是上个对象的共享数据实体，这里是我猜得，因为赋值方式是共享
pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
//用于进行障碍物之间是否冲突进行搜索的中间量
vector<int> pointIdxSearch;
vector<float> pointSquaredDistance;

void RandomMapGenerate()
{
  //设置随时数生成器对象，基础随机数，详情查阅C++相关标准
  random_device rd;
  default_random_engine eng(rd());
  //均匀分布的对象，生成障碍物的随机数
  uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(x_l, x_h );
  uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(y_l, y_h );
  uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(w_l, w_h);
  uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(h_l, h_h);
  //均匀分布的对象，生成椭圈的圆心的参数的随机数
  uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(x_l + 1.0, x_h - 1.0);
  uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(y_l + 1.0, y_h - 1.0);
  uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(w_c_l , w_c_h);
  //均匀分布的对象，生成随机的欧拉角的随机参数
  uniform_real_distribution<double> rand_roll = uniform_real_distribution<double>(- M_PI,     + M_PI);
  uniform_real_distribution<double> rand_pitch = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
  uniform_real_distribution<double> rand_yaw = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
  //圆圈是椭圆，这里是椭圆的两个半轴的随机数
  uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
  uniform_real_distribution<double> rand_num = uniform_real_distribution<double>(0.0, 1.0);
  //中间变量
  pcl::PointXYZ pt_random;

  //首先画一些椭圆圈，根据指定的椭圆圈数量画圈
  for(int i = 0; i < cir_num; i ++)
  {
    //定义一个椭圆的集合
    double x0, y0, z0, R;
    std::vector<Vector3d> circle_set;
    //随机生成一些点和半径
    x0 = rand_x_circle(eng);
    y0 = rand_y_circle(eng);
    z0 = rand_h(eng) / 2.0;
    R = rand_r_circle(eng);
    //如果这些随意的点距离初始点太近的话，就舍弃掉
    if(sqrt( pow(x0-init_x, 2) + pow(y0-init_y, 2) ) < 2.0 ) 
      continue;
    //生成椭圆的两个参数
    double a, b;
    a = rand_ellipse_c(eng);
    b = rand_ellipse_c(eng);
    double x, y, z;
    Vector3d pt3, pt3_rot;
    //根据一定的角度间隔，生成以原点圆心的椭圆的很多点，可以理解为先生成一个模板椭圆
    for(double theta = -M_PI; theta < M_PI; theta += 0.025)
    {
      //计算椭圆上的坐标，并且保存到集合中去
      x = a * cos(theta) * R;
      y = b * sin(theta) * R;
      z = 0;
      pt3 << x, y, z;
      circle_set.push_back(pt3);
    }
    //定义随机三维旋转矩阵
    Matrix3d Rot;
    //欧拉角
    double roll, pitch, yaw;
    double alpha, beta, gama;
    //随机生成欧拉角
    roll = rand_roll(eng); // alpha
    pitch = rand_pitch(eng); // beta
    yaw = rand_yaw(eng); // gama
    alpha = roll;
    beta = pitch;
    gama = yaw;
    //有一半的旋转矩阵俯仰角和航偏角是九十度
    double p = rand_num(eng);
    if(p < 0.5)
    {
      beta = M_PI / 2.0;
       gama = M_PI / 2.0;
    }
    //用生成好的俯仰角产生旋转矩阵
    Rot << cos(alpha) * cos(gama) - cos(beta) * sin(alpha) * sin(gama), - cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama), sin(alpha) * sin(beta),
            cos(gama) * sin(alpha) + cos(alpha) * cos(beta) * sin(gama), cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), - cos(alpha) * sin(beta),        
            sin(beta) * sin(gama), cos(gama) * sin(beta), cos(beta);
    //针对模板椭圆进行随机旋转和平移，对应的参数在之前已经生成好了
    for(auto pt: circle_set)
    {
      pt3_rot = Rot * pt;
      pt_random.x = pt3_rot(0) + x0 + 0.001;
      pt_random.y = pt3_rot(1) + y0 + 0.001;
      pt_random.z = pt3_rot(2) + z0 + 0.001;
      //保存在地面或者地上的点，地下的点没有意义
      //这个就是真正的地图了
      if(pt_random.z >= 0.0)
        cloudMap.points.push_back(pt_random);
    }
  }
  //设置kdtreeMap空的标志位清零
  bool is_kdtree_empty = false;
  //如果生成的地图是有效的，那么将他用share的方式传入kdtreeMap中去
  if(cloudMap.points.size() > 0)
    kdtreeMap.setInputCloud( cloudMap.makeShared() ); 
  else
    is_kdtree_empty = true;

  //随机生成一些障碍
  for(int i = 0; i < obs_num; i ++)
  {
    //随机生成障碍物的参数
    double x, y, w, h; 
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    //如果距离原点特别近的话，那就舍弃掉
    //if(sqrt( pow(x - init_x, 2) + pow(y - init_y, 2) ) < 2.0 ) 
    if(sqrt( pow(x - init_x, 2) + pow(y - init_y, 2) ) < 0.8 ) 
      continue;
    //设置一个点搜索点的冲突
    pcl::PointXYZ searchPoint(x, y, (h_l + h_h)/2.0);
    pointIdxSearch.clear();
    pointSquaredDistance.clear();
    //如果生成的参数并没和之前的存在的点重合并且距离也合适的话，那么就使用，否则抛弃掉
    if(is_kdtree_empty == false)
    {
      if (kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0)
      {
        if(sqrt(pointSquaredDistance[0]) < 1.0)
          continue;
      }
    }
    //根据分辨率对点的位置进行整定，小于等于
    x = floor(x/resolution) * resolution + resolution / 2.0;
    y = floor(y/resolution) * resolution + resolution / 2.0;
    //对障碍的宽度进行整定，大于等于
    int widNum = ceil(w/resolution);
    //根据之前整定好的点作为中心生成障碍，方方的障碍物
    for(int r = -widNum/2.0; r < widNum/2.0; r ++)
    {
      for(int s = -widNum/2.0; s < widNum/2.0; s ++)
      {
        //随机生成高度
        h = rand_h(eng);
        int heiNum = 2.0 * ceil(h/resolution);
        for(int t = 0; t < heiNum; t ++ ){
          pt_random.x = x + (r+0.0) * resolution + 0.001;
          pt_random.y = y + (s+0.0) * resolution + 0.001;
          pt_random.z = (t+0.0) * resolution * 0.5 + 0.001;
          //存入地图的点云中
          cloudMap.points.push_back( pt_random );
        }
      }
    }
  }
  //设置地图点云的属性
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  //置位地图生成标志位
  has_map = true;
  //将点云转换为ros类型从而发布
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
}
//发布地图消息到ROS中去
void pubSensedPoints()
{
  if( !has_map ) return;
  map_puber.publish(globalMap_pcd);
}
//主函数，此程序的主要作用是随机生成一个点云类型的地图，然后上面有一些椭圆，还有一些障碍
//发布并维护这个地图，这个地图是不能被修改的，维护地图消息的频率是1Hz
int main (int argc, char** argv) 
{
  ros::init (argc, argv, "rand_map");
  ros::NodeHandle n("~");
  //实例化ROStopic发布者对象
  map_puber = n.advertise<sensor_msgs::PointCloud2>("/global_map", 1); 
  //地图起始参数,默认为0,0
  n.param("init_state_x", init_x, 0.0);
  n.param("init_state_y", init_y, 0.0);
  //rosparam地图尺寸，默认为20,20,10
  n.param("map/x_size", x_size, 20.0);
  n.param("map/y_size", y_size, 20.0);
  n.param("map/z_size", z_size, 10.0 );
  //障碍数量，默认为30，需要更换的，请根据自己修改或者launch中加载
  n.param("map/obs_num", obs_num, 50);
  //椭圆环数量，默认为30，这个主要是给无人机用的，完全可以屏蔽掉
  n.param("map/circle_num", cir_num, 10);
  //地图分辨率,默认为0.2
  n.param("map/resolution", resolution, 0.2);
  //障碍物的宽度，默认0.1,0.7
  n.param("ObstacleShape/lower_rad", w_l, 0.1);
  n.param("ObstacleShape/upper_rad", w_h, 0.7);
  //障碍物和圆环的高度，默认2.0,5.0
  n.param("ObstacleShape/lower_hei", h_l, 2.0);
  n.param("ObstacleShape/upper_hei", h_h, 5.0);
  //圆环半径范围，默认0.6,2.0
  n.param("CircleShape/lower_circle_rad", w_c_l, 0.6);
  n.param("CircleShape/upper_circle_rad", w_c_h, 2.0);
  //地图的发布的频率,默认0.5
  n.param("sensing/rate", sense_rate, 0.5);
  //地图的边缘点，也就是地图坐标上下限
  x_l = - x_size / 2.0;
  x_h = + x_size / 2.0;
  y_l = - y_size / 2.0;
  y_h = + y_size / 2.0;
  //随机生成地图，地图存在cloudMap中，后转换为globalMap_pcd进行发布
  //map的frame_id为world，是一串点云消息，rviz请修改点云显示类型和尺寸查看
  RandomMapGenerate();
  //地图发布的频率是1Hz
  //发布的话题名称是/random_complex_scene/global_map
  ros::Rate loop(sense_rate);
  while(ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop.sleep();
  }
}

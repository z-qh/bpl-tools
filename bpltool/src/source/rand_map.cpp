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

ros::Publisher map_puber;

int main (int argc, char** argv)
{
    ros::init (argc, argv, "rand_map");
    ros::NodeHandle n("~");

    map_puber = n.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}

/*
 * 生成固定形状点云地图
 */
void RandomMapGenerateExample()
{
    vector<int> pointIdxSearch;
    vector<float> pointSquaredDistance;
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
    int obs_num, cir_num;
    double x_size, y_size, z_size, init_x, init_y, resolution;
    double x_l, x_h, y_l, y_h, w_l, w_h, h_l, h_h, w_c_l, w_c_h;
    random_device rd;
    default_random_engine eng(rd());
    init_x = 0.0;
    init_y = 0.0;
    x_size = 20.0;
    y_size = 20.0;
    z_size = 10.0;
    obs_num = 50;
    cir_num = 10;
    resolution = 0.2;
    w_l = 0.1;
    w_h = 0.7;
    h_l = 2.0;
    h_h = 5.0;
    w_c_l = 0.6;
    w_c_h = 2.0;
    x_l = - x_size / 2.0;
    x_h = + x_size / 2.0;
    y_l = - y_size / 2.0;
    y_h = + y_size / 2.0;

    uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(x_l, x_h );
    uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(y_l, y_h );
    uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(w_l, w_h);
    uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(h_l, h_h);

    uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(x_l + 1.0, x_h - 1.0);
    uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(y_l + 1.0, y_h - 1.0);
    uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(w_c_l , w_c_h);

    uniform_real_distribution<double> rand_roll = uniform_real_distribution<double>(- M_PI,     + M_PI);
    uniform_real_distribution<double> rand_pitch = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
    uniform_real_distribution<double> rand_yaw = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);

    uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
    uniform_real_distribution<double> rand_num = uniform_real_distribution<double>(0.0, 1.0);

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
        //根据一定的角度间隔，生成以原点圆心的椭圆的点
        for(double theta = -M_PI; theta < M_PI; theta += 0.025)
        {
            //计算椭圆上的坐标，并且保存到集合中去
            x = a * cos(theta) * R;
            y = b * sin(theta) * R;
            z = 0;
            pt3 << x, y, z;
            circle_set.push_back(pt3);
        }

        Matrix3d Rot;

        double roll, pitch, yaw;
        double alpha, beta, gama;
        //随机生成欧拉角
        roll = rand_roll(eng); // alpha
        pitch = rand_pitch(eng); // beta
        yaw = rand_yaw(eng); // gama
        alpha = roll;
        beta = pitch;
        gama = yaw;

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
        //针对模板椭圆进行随机旋转和平移
        for(auto pt: circle_set)
        {
            pt3_rot = Rot * pt;
            pt_random.x = pt3_rot(0) + x0 + 0.001;
            pt_random.y = pt3_rot(1) + y0 + 0.001;
            pt_random.z = pt3_rot(2) + z0 + 0.001;
            //保存在地面或者地上的点，
            if(pt_random.z >= 0.0)
                cloudMap.points.push_back(pt_random);
        }
    }
    //设置kdtreeMap空的标志位清零
    bool is_kdtree_empty = false;
    //用share的方式传入kdtreeMap中去
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
        //根据分辨率对点的位置进行整定
        x = floor(x/resolution) * resolution + resolution / 2.0;
        y = floor(y/resolution) * resolution + resolution / 2.0;
        //对障碍的宽度进行整定
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
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";
    map_puber.publish(globalMap_pcd);
}


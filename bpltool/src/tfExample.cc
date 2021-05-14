#include "ros/ros.h"

/*
 * tf2一共有几个包，分别是tf2，tf2_geometry_msgs，tf2_ros，tf2_sensor_msgs
 * tf_bullet，tf2_eigen，tf2_kdl，tf2_msgs，tf2_py，tf2_tools
 * tf2提供了基础的数据类型LinearMath，数据类型的转换utils
 * tf2_ros提供了ros下的订阅和发布
 * tf2_geometry_msgs提供了geometry_msgs之间数据类型的转换
 * tf2_sensor_msgs提供了doTransform方法对点云进行转换
 * tf2_eigen提供了和eigen之间的数据转换
 *
 */
#include "geometry_msgs/TransformStamped.h"             //坐标转换信息用于发布

#include "tf2_ros/static_transform_broadcaster.h"       //静态坐标发布
#include "tf2_ros/transform_broadcaster.h"              //动态坐标发布
#include "tf2_ros/transform_listener.h"                 //坐标接收

#include "tf2/utils.h"                                  //常用数据类型转换函数，依赖tf2、tf2_geometry_msgs

#include "tf2/LinearMath/Vector3.h"                     //含有tf2::Vector3数据类型
#include "tf2/LinearMath/Transform.h"                   //含有tf2::Transform数据类型
#include "tf2/LinearMath/Matrix3x3.h"                   //含有tf2::Matrix3x3数据类型
#include "tf2/LinearMath/Quaternion.h"                  //含有tf2::Quaternion数据类型
#include "tf2/transform_datatypes.h"                    //含有tf2::Stamped<T>数据类型


#include "tf2_eigen/tf2_eigen.h"

#include "Eigen/Eigen"
#include "random"
#include "vector"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/PointCloud2.h"

pcl::PointCloud<pcl::PointXYZ> cloud;
sensor_msgs::PointCloud2 pubCloud;
ros::Publisher pub;

//记录转换
geometry_msgs::TransformStamped tfInMapAndChild;
geometry_msgs::TransformStamped tfInMapAndChild2;

int count = 0;

void genRandomCircle()
{
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::uniform_real_distribution<double> normalRandRad(1,2);
    //生成一个标准的圆形，总共有360个点，半径随机0-3之间
    std::vector<Eigen::Vector3d> circle;
    double rad = normalRandRad(rng);
    for(int i = 0; i < 360; i++)
    {
        Eigen::Vector3d tempPoint;
        tempPoint(0) = sin(i*M_PI/180.0) * rad;
        tempPoint(1) = cos(i*M_PI/180.0) * rad;
        tempPoint(2) = 0;
        circle.push_back(tempPoint);
    }
    //生成一个随机的旋转
    std::uniform_real_distribution<double> normalRandRoll(0,60);
    std::uniform_real_distribution<double> normalRandPitch(0,60);
    std::uniform_real_distribution<double> normalRandYaw(0,60);
    //生成一个起点坐标
    std::uniform_real_distribution<double> normalX(0,1);
    std::uniform_real_distribution<double> normalY(0,1);
    std::uniform_real_distribution<double> normalZ(0,1);
    //计算角度值
    double roll = normalRandRoll(rng)*M_PI/180;
    double pitch = normalRandPitch(rng)*M_PI/180;
    double yaw = normalRandYaw(rng)*M_PI/180;

    double X = normalX(rng);
    double Y = normalX(rng);
    double Z = normalX(rng);

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    Eigen::Matrix3d rot;
    tf2::Matrix3x3 rotTf(quat);

    rot << rotTf[0][0], rotTf[0][1], rotTf[0][2],
            rotTf[1][0], rotTf[1][1], rotTf[1][2],
            rotTf[2][0], rotTf[2][1], rotTf[2][2];
    
    //进行旋转
    for(int i = 0; i < circle.size(); i++)
    {
        Eigen::Vector3d tempPoint = circle[i];
        Eigen::Vector3d rotPoint = rot * tempPoint;
        pcl::PointXYZ pclPoint(rotPoint.x()+X, rotPoint.y()+Y, rotPoint.z()+Z);
        cloud.push_back(pclPoint);
    }
}

void trans()
{
    pcl::toROSMsg(cloud, pubCloud);
    pubCloud.header.frame_id = "child2";
    pubCloud.header.stamp = ros::Time::now();
}

void changePosi()
{
    count++;
    if(count >= 180)
        count = 0;
    int time = count / 10 % 9;
    tfInMapAndChild2.transform.translation.x = time;
    tfInMapAndChild2.transform.translation.y = 0;
    tfInMapAndChild2.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(time * 60 / 3.1415926535, 0, 0);
    //quat.setRPY(0, 0, 0);
    tfInMapAndChild2.transform.rotation.x = quat.x();
    tfInMapAndChild2.transform.rotation.y = quat.y();
    tfInMapAndChild2.transform.rotation.z = quat.z();
    tfInMapAndChild2.transform.rotation.w = quat.w();
    tfInMapAndChild2.header.stamp = ros::Time::now();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfExample");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/tfCloud", 1);

    //静态广播对象，必须放在初始化后面
    tf2_ros::StaticTransformBroadcaster tfPub;
    tf2_ros::TransformBroadcaster tfPub2;

    ros::Rate loop(10   );

    //广播变换前设置头信息，变换信息包括旋转四元数和平移三维向量
    tfInMapAndChild.header.frame_id = "map";
    tfInMapAndChild.header.stamp = ros::Time::now();
    tfInMapAndChild.child_frame_id = "child";
    tfInMapAndChild.transform.translation.x = 5;
    tfInMapAndChild.transform.translation.y = 5;
    tfInMapAndChild.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tfInMapAndChild.transform.rotation.x = quat.x();
    tfInMapAndChild.transform.rotation.y = quat.y();
    tfInMapAndChild.transform.rotation.z = quat.z();
    tfInMapAndChild.transform.rotation.w = quat.w();

    tfInMapAndChild2.header.frame_id = "map";
    tfInMapAndChild2.header.stamp = ros::Time::now();
    tfInMapAndChild2.child_frame_id = "child2";


    genRandomCircle();


    //发布变换
    tfPub.sendTransform(tfInMapAndChild);

    while (ros::ok())
    {
        changePosi();
        tfInMapAndChild2.header.stamp = ros::Time::now();
        tfPub2.sendTransform(tfInMapAndChild2);
        trans();
        pub.publish(pubCloud);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;

}
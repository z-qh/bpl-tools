#include "iostream"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/nonlinear/ISAM2.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tf.h"

#include "random"

//构建位姿估计问题，将真值加入随机偏置作为观测值
//在根据测量时间序列优化所有帧上的二维位姿，所有位姿数量不超过1000帧
namespace GPS2D{

    using  namespace std;

    gtsam::Pose2 generateGaussNoise()
    {
        static default_random_engine e;
        static normal_distribution<double> gaussRandom(0,1);
        //angle
        double x = gaussRandom(e);
        double y = gaussRandom(e);
        double theta = gaussRandom(e) / 180.0 * M_PI;

        gtsam::Pose2 result(x, y, theta);
        return result;
    }

    vector<gtsam::Pose2> generateGPSMeasurementSet(int n = 360, double r = 20, double density = 5)
    {
        double C = 2 * M_PI * r / density;
        vector<gtsam::Pose2> result;
        // tarjectory a circle
        for(double i = 0; i < C; i++){
            double theta = i / C * n / 180.0 * M_PI;
            gtsam::Pose2 noise = generateGaussNoise();
            double y = sin(theta) * r + noise.y();
            double x = cos(theta) * r + noise.x();
            theta = (i / C * n + 90.0) / 180.0 * M_PI + noise.theta();
            gtsam::Pose2 thisPose(x, y, theta);
            result.push_back(thisPose);
        }
        return result;
    }

    vector<nav_msgs::Odometry> convertPoseToMsg(const vector<gtsam::Pose2>& pose)
    {
        vector<nav_msgs::Odometry> result;
        for(int i = 0; i < pose.size(); i++)
        {
            nav_msgs::Odometry tempOdom;
            tempOdom.pose.pose.position.x = pose[i].x();
            tempOdom.pose.pose.position.y = pose[i].y();
            tempOdom.pose.pose.position.z = 0;
            auto quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, pose[i].theta());
            tempOdom.pose.pose.orientation.x = quat.x;
            tempOdom.pose.pose.orientation.y = quat.y;
            tempOdom.pose.pose.orientation.z = quat.z;
            tempOdom.pose.pose.orientation.w = quat.w;
            tempOdom.header.stamp = ros::Time::now();
            tempOdom.header.frame_id = "map";
            result.push_back(tempOdom);
        }
        return result;
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "factorGraph");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/before", 100);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Odometry>("/after", 100);


    return 0;
}

//构建ceres求解轨迹优化问题，N点+1点
//问题描述为前面的N点为六自由度的位姿，最后的1点为六自由度的位姿
//前N点为LO数据，后1点为GNSS数据等于位姿真实值，如何优化前面N点的位姿


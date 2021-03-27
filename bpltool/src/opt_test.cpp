#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <random>
#include <nav_msgs/Path.h>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
using namespace std;

//发布消息接受消息
ros::Publisher odomPub;
ros::Subscriber pathSub;
ros::Publisher pathPub;
ros::Publisher optPub;

//随机数生成器
default_random_engine rng(time(0));
normal_distribution<double> gauss(0, 0.1);

//CostFunction损失函数
class DeltaDis: public ceres::SizedCostFunction<3,3,3>
{
private:
    Eigen::Vector3d Deltadis;
public:
    DeltaDis() = delete;
    DeltaDis(Eigen::Vector3d deltadis)
    {
        Deltadis = deltadis;
    }
    virtual ~DeltaDis(){}
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);

        Eigen::Map<Eigen::Matrix<double,3,1>> residual(residuals);
        residual = Deltadis - (Pi - Pj);
        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> jacobians_pose_i(jacobians[0]);
                jacobians_pose_i.setZero();
                jacobians_pose_i(0,0) = -1.0;
                jacobians_pose_i(1,1) = -1.0;
                jacobians_pose_i(2,2) = -1.0;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> jacobians_pose_j(jacobians[1]);
                jacobians_pose_j.setZero();
                jacobians_pose_j(0,0) = 1.0;
                jacobians_pose_j(1,1) = 1.0;
                jacobians_pose_j(2,2) = 1.0;
            }
        }
    }
};

bool solveOptimizationProblem(ceres::Problem* problem)
{
    if(problem == nullptr)
        return false;
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    cout << summary.FullReport() << endl;
    return summary.IsSolutionUsable();
}

vector<Eigen::Vector3d> runOptimation(vector<Eigen::Vector3d>& path)
{
    vector<Eigen::Vector3d> result;
    ceres::Problem problem;
    uint N = path.size();
    //第一次的时候轨迹数量不够，跳过，从第二次开始优化
    if(N <= 1)
        return result;
    //建立新的内存区域放置待优化的内容
    double **pose_s = new double*[N];
    for(uint i = 0; i < N; i++)
    {
        pose_s[i] = new double[3];
    }
    for(uint i = 0; i < N; i++)
    {
        pose_s[i][0] = path[i][0];
        pose_s[i][1] = path[i][1];
        pose_s[i][2] = path[i][2];
    }
    Eigen::Vector3d deldis;
    deldis[0] = 0.1 + fabs(gauss(rng)/5);
    deldis[1] = 0.0;
    deldis[2] = 0.0;
    //为优化问题添加优化参数，损失函数，
    for(uint i = 1; i < N; i++)
    {
        problem.AddParameterBlock(pose_s[i-1], 3);
        problem.AddParameterBlock(pose_s[i], 3);
        //这里的观测数据，应该是有高斯噪声在里面的一个接近真值的随机数
        //但是这里的效果还是不是很好
        DeltaDis* deldisf = new DeltaDis(deldis);
        problem.AddResidualBlock(deldisf, nullptr, pose_s[i-1],pose_s[i]);
    }
    //解优化问题
    solveOptimizationProblem(&problem);
    for(uint i = 0; i < N; i++)
    {
        Eigen::Vector3d temp;
        temp[0] = pose_s[i][0];
        temp[1] = pose_s[i][1];
        temp[2] = pose_s[i][2];
        result.push_back(temp);
    }
    for(uint i = 0; i < N; i++)
    {
        delete[] pose_s[i];
    }
    delete[] pose_s;
    return result;
}

void handleOriginPath(const nav_msgs::Path& originPath)
{
    nav_msgs::Path optPath;
    optPath.header.frame_id = "map";
    vector<Eigen::Vector3d> pathData;
    vector<Eigen::Vector3d> optPathData;
    for(geometry_msgs::PoseStamped posei : originPath.poses)
    {
        Eigen::Vector3d tempPose;
        tempPose[0] = posei.pose.position.x;
        tempPose[1] = posei.pose.position.y;
        tempPose[2] = posei.pose.position.z;
        pathData.push_back(tempPose);
    }
    //优化所有时刻的pose，没有滑动窗口
    optPathData = runOptimation(pathData);
    for(Eigen::Vector3d posei : optPathData)
    {
        geometry_msgs::PoseStamped tempPose;
        tempPose.pose.position.x = posei[0];
        tempPose.pose.position.y = posei[1];
        tempPose.pose.position.z = posei[2];
        tempPose.pose.orientation.x = 0;
        tempPose.pose.orientation.y = 0;
        tempPose.pose.orientation.z = 0;
        tempPose.pose.orientation.w = 0;
        optPath.poses.push_back(tempPose);
    }
    optPub.publish(optPath);
}

void generatePosi(nav_msgs::Odometry& p, double x_bias, double y_bias, nav_msgs::Path& path)
{
    p.header.stamp = ros::Time::now();
    p.pose.pose.position.x += 0.1 + fabs(x_bias);
    p.pose.pose.position.y = y_bias;

    geometry_msgs::PoseStamped tempPose;
    tempPose.header.stamp = ros::Time::now();
    tempPose.pose.position.x = p.pose.pose.position.x;
    tempPose.pose.position.y = p.pose.pose.position.y;
    tempPose.pose.position.z = p.pose.pose.position.z;
    tempPose.pose.orientation.x = p.pose.pose.orientation.x;
    tempPose.pose.orientation.y = p.pose.pose.orientation.y;
    tempPose.pose.orientation.z = p.pose.pose.orientation.z;
    tempPose.pose.orientation.w = p.pose.pose.orientation.w;
    path.poses.push_back(tempPose);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "opt_test");
    ros::NodeHandle nh;

    odomPub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    pathSub = nh.subscribe("/originPath", 1, handleOriginPath);
    pathPub  = nh.advertise<nav_msgs::Path>("/originPath", 1);
    optPub = nh.advertise<nav_msgs::Path>("optPath", 1);
    ros::Rate rate(1);

    geometry_msgs::Quaternion tempQuat;
    tempQuat = tf::createQuaternionMsgFromYaw(M_PI / 2);

    nav_msgs::Odometry odomPosi;
    odomPosi.header.frame_id = "map";
    odomPosi.header.stamp = ros::Time::now();
    odomPosi.pose.pose.position.x = -1;
    odomPosi.pose.pose.position.y = 0;
    odomPosi.pose.pose.position.z = 0;
    odomPosi.pose.pose.orientation.x = 0;
    odomPosi.pose.pose.orientation.y = 0;
    odomPosi.pose.pose.orientation.z = 0;
    odomPosi.pose.pose.orientation.w = 0;

    nav_msgs::Path originPath;
    originPath.header.frame_id = "map";
    originPath.header.stamp = ros::Time::now();
    while(ros::ok())
    {
        generatePosi(odomPosi, gauss(rng), gauss(rng), originPath);
        odomPub.publish(odomPosi);
        pathPub.publish(originPath);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
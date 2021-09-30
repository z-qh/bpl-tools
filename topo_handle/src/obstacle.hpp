#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
#include <limits>
#include <deque>
#include <queue>

#include "Eigen/Geometry"
#include "Eigen/Core"

#include "include/dbscan/IOU.h"

#include "std_msgs/Header.h"

#include "include/utility.h"

class obstacle{
private:
    //////////////////////////
    PointType max_point;
    PointType min_point;
    PointType center;
    //////////////////////////
    pcl::PointCloud<PointType>::Ptr PointCloud_withLabel;
    pcl::PointCloud<PointType>::Ptr label_value;
    pcl::PointCloud<PointType>::Ptr removeObstacle;
    pcl::PointCloud<PointType>::Ptr dynamicObstacle;        //存储动态障碍
    unordered_set<int>intensity_set;

    float IOUThreshold;

    //////////////////////////
    //pcl显示
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //////////////////////////
    vector<vector<Eigen::Vector3f>>bboxT;    //boundingbox的位置
    vector<vector<Eigen::Quaternionf>>bboxQ; //boundingbox的朝向
    vector<vector<Eigen::Vector3f>>whd;      //boundingbox的宽高深
    //////////////////////////
    deque<pcl::PointCloud<PointType>>laserCloudInqueue;
    deque<std_msgs::Header>laserDataHead;
    deque<sensor_msgs::PointCloud2>groundMsg;
    //////////////////////////
    PointType nanPoint;
    pcl::PointCloud<PointType>::Ptr fullCloudWithLabelAndIndex;
    //////////////////////////
    static bool cmp_x(pair<PointType,PointType> pointa, pair<PointType,PointType> pointb){
        return pointa.first.x<pointb.first.x;
    }
    static bool cmp_x_inv(pair<PointType,PointType>pointa, pair<PointType,PointType> pointb){
        return pointa.first.x>pointb.first.x;
    }

    queue<pcl::PointCloud<PointType>::Ptr>centerPointCloudsque;	//聚类中心点数据  使用队列存储
    queue<vector<vector<PointType>>>RectangleVertexque;			//存储矩形顶点队列
    queue<vector<Eigen::Vector3f>>bboxTque;
    queue<vector<Eigen::Quaternionf>>bboxQque;
    queue<vector<Eigen::Vector3f>>whdque;
    unordered_set<int>label_set;	//保存的类

    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    //////////////////////////////////////
    bool newLaserOdometry = false;
    bool newLaserPointCloud = false;

    const int queLength = 2;
    deque<sensor_msgs::PointCloud2> lidarPointCloudsQueue;
    deque<Eigen::Isometry3d>poseQueue;

    Eigen::Quaterniond pubOrientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    pubPosition    = Eigen::Vector3d::Zero();
    deque<nav_msgs::Odometry>odomMsgIn;
    //////////////////////////////////////
    void remove_dynamic_obstacles(sensor_msgs::PointCloud2& remove_obstacle_cloud_){
        if(laserCloudInqueue.size() != queLength)
            return;
        /////////main handle//////////////
        segmentedCloud->clear();
        std::fill(fullCloudWithLabelAndIndex->points.begin(),fullCloudWithLabelAndIndex->points.end(), nanPoint);
        removeObstacle.reset(new pcl::PointCloud<PointType>());
        label_set.clear();
        vector<pcl::PointCloud<PointType>::Ptr>centerPointClouds(2);	//每个聚类的中心点,由于有两帧数据所以
        vector<pcl::PointCloud<PointType>::Ptr>pointCloudsKtime(2);       //存储两帧点云数据
        vector<vector<vector<PointType>>>RectangleVertex(2);	//第一个vector表示两帧数据  第二个为每个聚类  第三个为每个聚类中存放的数据 存放矩形四个点及边长的长
        for(int i=0;i<2;++i){
            pointCloudsKtime[i].reset(new pcl::PointCloud<PointType>());
            centerPointClouds[i].reset(new pcl::PointCloud<PointType>());
            bboxQ[i].clear();
            bboxT[i].clear();
            whd[i].clear();
        }
        *pointCloudsKtime[0] +=laserCloudInqueue[0];
        *pointCloudsKtime[1] +=laserCloudInqueue[1];

        int timeStemp = 0;
        for(; timeStemp<2; ++timeStemp)
        {
            int cloudLabelSize = pointCloudsKtime[timeStemp]->points.size();
            unordered_map<int, pcl::PointCloud<PointType>::Ptr>labelPointCloudRemove;//key为点云类别标签，value为对应类别的点云数据

            for(int i=0; i<cloudLabelSize; ++i){
                PointType thisPoint;
                thisPoint.x = pointCloudsKtime[timeStemp]->points[i].x;
                thisPoint.y = pointCloudsKtime[timeStemp]->points[i].y;
                thisPoint.z = pointCloudsKtime[timeStemp]->points[i].z;
                thisPoint.intensity = pointCloudsKtime[timeStemp]->points[i].intensity;

                int label = pointCloudsKtime[timeStemp]->points[i].intensity;
                auto iter = labelPointCloudRemove.find(label);

                if(iter == labelPointCloudRemove.end()){
                    pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
                    labelPointCloudRemove[label] = tmp;
                }
                thisPoint.z = 0;
                labelPointCloudRemove[label]->push_back(thisPoint);	//计算框和交并比时所使用的点云
            }
            for(int j=0; j<labelPointCloudRemove.size(); ++j){
                Eigen::Vector4f pcaCentroid = Eigen::Vector4f::Identity();
                pcl::compute3DCentroid(*labelPointCloudRemove[j], pcaCentroid);//计算该类点云的中心点
                // std::cout<<"center "<<pcaCentroid(0)<<" "<<pcaCentroid(1)<<" "<<pcaCentroid(2)<<" "<<pcaCentroid(3)<<std::endl;
                Eigen::Matrix3f covariance;//使用特征值分解进行PCA主方向分析
                pcl::computeCovarianceMatrixNormalized(*labelPointCloudRemove[j],pcaCentroid,covariance);//计算协方差矩阵
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//SelfAdjointEigenSolver分解特征值和特征向量，特征值从小到大存储
                Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
                Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
                // 获得点云的主方向，点云三维方向向量

                // eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
                // eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
                eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));

                Eigen::Matrix4f value = Eigen::Matrix4f::Identity();//将RT存储在4*4矩阵
                value.block<3,3>(0,0) = eigenVectorsPCA;		//旋转矩阵  pca的方向矩阵
                value.block<3,1>(0,3) = pcaCentroid.head<3>();	//平移矩阵

                //获得pca方向的逆
                Eigen::Matrix4f value_inverse = Eigen::Matrix4f::Identity();
                value_inverse.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //RT  R矩阵的逆
                value_inverse.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -RT*t
                pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*labelPointCloudRemove[j], *transformedCloud, value_inverse);	//将当前类的点云转换到原点  原始点云的中心点转换到三维坐标原点

                Eigen::Vector4f min_p1, max_p1;//存放转换到原点的点云中的xyz的最小值和最大值
                pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
                Eigen::Vector3f c1={0.5f*(min_p1(0) + max_p1(0)),0.5f*(min_p1(1) + max_p1(1)),0.5f*(min_p1(2) + max_p1(2))};//转换到原点后的聚类中心点
                Eigen::Vector3f tmp_whd = {(max_p1(0) - min_p1(0)),(max_p1(1) - min_p1(1)),(max_p1(2) - min_p1(2))};//各个轴上的值 长宽深
                Eigen::Vector3f c;	//原始聚类点云的中心点
                Eigen::Affine3f tm_inv_aff(value);
                pcl::transformPoint(c1, c, tm_inv_aff);
                PointType centerPoint;
                centerPoint.x = c(0);   centerPoint.y = c(1);   centerPoint.z = c(2);
                centerPoint.intensity = centerPointClouds[timeStemp]->points.size();
                centerPointClouds[timeStemp]->push_back(centerPoint);	//存储聚类中心，强度值存储的是在centerPointClouds[timeStemp]中的索引值
                whd[timeStemp].push_back(tmp_whd);                      //存储该boundingbox长宽深
                Eigen::Quaternionf tmp_bboxQ(value.block<3,3>(0, 0));
                Eigen::Vector3f    tmp_bboxT(c);                        //原始点云聚类中心
                bboxT[timeStemp].push_back(tmp_bboxT);                  //存储该boundingbox相对于原点的平移矩阵
                bboxQ[timeStemp].push_back(tmp_bboxQ);                  //存储该boundingbox相对于原点的旋转四元数

                pcl::PointCloud<PointType>::Ptr transVertexCloud(new pcl::PointCloud<PointType>());   //原点坐标系下点云bounding box的顶点坐标
                pcl::PointCloud<PointType>::Ptr VertexCloud(new pcl::PointCloud<PointType>());        //原始点云坐标系下顶点坐标

                transVertexCloud->points.resize(8);
                transVertexCloud->points[0].x = max_p1(0);//构建转换到原点坐标系下bounding box的顶点坐标
                transVertexCloud->points[0].y = max_p1(1);
                transVertexCloud->points[0].z = max_p1(2);

                transVertexCloud->points[1].x = max_p1(0);
                transVertexCloud->points[1].y = max_p1(1);
                transVertexCloud->points[1].z = min_p1(2);

                transVertexCloud->points[2].x = max_p1(0);
                transVertexCloud->points[2].y = min_p1(1);
                transVertexCloud->points[2].z = max_p1(2);

                transVertexCloud->points[3].x = min_p1(0);
                transVertexCloud->points[3].y = max_p1(1);
                transVertexCloud->points[3].z = max_p1(2);

                transVertexCloud->points[4].x = max_p1(0);
                transVertexCloud->points[4].y = min_p1(1);
                transVertexCloud->points[4].z = min_p1(2);

                transVertexCloud->points[5].x = min_p1(0);
                transVertexCloud->points[5].y = max_p1(1);
                transVertexCloud->points[5].z = min_p1(2);

                transVertexCloud->points[6].x = min_p1(0);
                transVertexCloud->points[6].y = min_p1(1);
                transVertexCloud->points[6].z = max_p1(2);

                transVertexCloud->points[7].x = min_p1(0);
                transVertexCloud->points[7].y = min_p1(1);
                transVertexCloud->points[7].z = min_p1(2);

                pcl::transformPointCloud(*transVertexCloud,*VertexCloud,tm_inv_aff);        //将边界框顶点坐标转换到原始点云位置
                pcl::PointCloud<PointType>::Ptr bottomVertexCloud(new pcl::PointCloud<PointType>());
                bottomVertexCloud->push_back(VertexCloud->points[0]);   //去重后的矩形框坐标
                for(int k=1; k<8; ++k){
                    bool hasRecv = false;
                    for(int n=0;n<bottomVertexCloud->points.size();++n){
                        if(VertexCloud->points[k].x == bottomVertexCloud->points[n].x &&
                           VertexCloud->points[k].y == bottomVertexCloud->points[n].y &&
                           VertexCloud->points[k].z == bottomVertexCloud->points[n].z){
                            hasRecv = true;
                            break;
                        }
                    }
                    if(!hasRecv)
                        bottomVertexCloud->push_back(VertexCloud->points[k]);
                }
                //计算bounding box交并比时需要按顺时针或者逆时针输入顶点坐标，所以需要对坐标进行排序 将坐标与中心点进行比较，从而进行排序
                //分象限排序的目的是，在一个象限中，可能会出现多个顶点，此时需要按照x轴进行排序

                //bottomVertexCloud中存储的时经过去重后的转换到原始点云下的边界框的顶点坐标
                vector<pair<PointType, PointType>>testVertexCloud;    //testVertexCloud排序使用
                //寻找底边框
                pcl::PointCloud<PointType>::Ptr testBottomVertex(new pcl::PointCloud<PointType>());

                testVertexCloud.clear();
                //排序出差，应该减去中心点后排序
                //为了计算IOU需要将顶点坐标按照逆时针或者顺时针排序，接下来的一段代码的作用就是用于排序的
                //第三象限
                for(auto point : bottomVertexCloud->points){
                    if(point.x<c(0) && point.y<c(1)){
                        PointType thisPoint;
                        thisPoint.x = point.x - c(0);
                        thisPoint.y = point.y - c(1);
                        thisPoint.z = point.z - c(2);
                        thisPoint.z = 0;
                        testVertexCloud.push_back(make_pair(thisPoint,point));
                        // testVertexCloud.push_back(point);
                    }
                }
                if(testVertexCloud.size()>1){
                    sort(testVertexCloud.begin(),testVertexCloud.end(),cmp_x);
                }
                for(auto c:testVertexCloud)
                    testBottomVertex->push_back(c.second);	//将原始点云边界框顶点坐标经过排序后存放

                //第四象限
                testVertexCloud.clear();
                for(auto point : bottomVertexCloud->points){
                    if(point.x>c(0) && point.y<c(1)){
                        PointType thisPoint;
                        thisPoint.x = point.x - c(0);
                        thisPoint.y = point.y - c(1);
                        thisPoint.z = point.z - c(2);

                        thisPoint.z = 0;
                        testVertexCloud.push_back(make_pair(thisPoint,point));
                    }
                }
                if(testVertexCloud.size()>1){
                    sort(testVertexCloud.begin(),testVertexCloud.end(),cmp_x);
                }
                for(auto c:testVertexCloud)
                    testBottomVertex->push_back(c.second);

                //第一象限
                testVertexCloud.clear();
                for(auto point : bottomVertexCloud->points){
                    if(point.x>c(0) && point.y>c(1)){
                        PointType thisPoint;
                        thisPoint.x = point.x - c(0);
                        thisPoint.y = point.y - c(1);
                        thisPoint.z = point.z - c(2);

                        thisPoint.z = 0;
                        testVertexCloud.push_back(make_pair(thisPoint,point));
                    }
                }
                if(testVertexCloud.size()>1){
                    sort(testVertexCloud.begin(),testVertexCloud.end(),cmp_x_inv);
                }
                for(auto c:testVertexCloud)
                    testBottomVertex->push_back(c.second);

                //第二象限
                testVertexCloud.clear();
                for(auto point : bottomVertexCloud->points){
                    if(point.x<c(0) && point.y>c(1)){
                        PointType thisPoint;
                        thisPoint.x = point.x - c(0);
                        thisPoint.y = point.y - c(1);
                        thisPoint.z = point.z - c(2);
                        thisPoint.z = 0;
                        testVertexCloud.push_back(make_pair(thisPoint,point));
                    }
                }
                if(testVertexCloud.size()>1){
                    sort(testVertexCloud.begin(),testVertexCloud.end(),cmp_x_inv);
                }
                for(auto c:testVertexCloud)
                    testBottomVertex->push_back(c.second);


                vector<PointType>tmp_vertex;
                for(auto point:testBottomVertex->points)
                    tmp_vertex.push_back(point);
                //最后一个为聚类边界框各个轴上的距离
                PointType tmp_point;
                tmp_point.x = tmp_whd(0);
                tmp_point.y = tmp_whd(1);
                tmp_point.z = tmp_whd(2);
                tmp_point.intensity = j;	//intensity表示聚类的类别
                tmp_vertex.push_back(tmp_point);
                RectangleVertex[timeStemp].push_back(tmp_vertex);

            }
            bboxTque.push(bboxT[timeStemp]);	bboxQque.push(bboxQ[timeStemp]);
            whdque.push(whd[timeStemp]);		centerPointCloudsque.push(centerPointClouds[timeStemp]);
            RectangleVertexque.push(RectangleVertex[timeStemp]);
        }//end timestamp
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalCenterPoints(new pcl::KdTreeFLANN<PointType>());

        kdtreeGlobalCenterPoints->setInputCloud(centerPointClouds[1]);	//k+1帧构建kd树进行查找
        int centerPointsSizeKtime = centerPointClouds[0]->points.size();

        int tmp_count = 0;
        for(int index=0; index<centerPointsSizeKtime; ++index){
            PointType pointSearchRadius = RectangleVertex[0][index][4];

            float SearchRadius = max(max(pointSearchRadius.x,pointSearchRadius.y),pointSearchRadius.z);//搜索时长度为xyz最大值

            vector<int>pointSearchInd;
            vector<float>pointSearchSqDis;
            kdtreeGlobalCenterPoints->radiusSearch(centerPointClouds[0]->points[index],SearchRadius,pointSearchInd,pointSearchSqDis,0);

            for(int k=0; k<(int)pointSearchInd.size(); ++k){

                int id = pointSearchInd[k];
                int idInRect = centerPointClouds[1]->points[id].intensity;
                vector<vector<int>>tmp_box_1= {{RectangleVertex[0][index][0].x,RectangleVertex[0][index][0].y},
                                               {RectangleVertex[0][index][1].x,RectangleVertex[0][index][1].y},
                                               {RectangleVertex[0][index][2].x,RectangleVertex[0][index][2].y},
                                               {RectangleVertex[0][index][3].x,RectangleVertex[0][index][3].y}};//第k帧的聚类的boungding box 二维

                vector<vector<int>>tmp_box_2 = {{RectangleVertex[1][idInRect][0].x,RectangleVertex[1][idInRect][0].y},
                                                {RectangleVertex[1][idInRect][1].x,RectangleVertex[1][idInRect][1].y},
                                                {RectangleVertex[1][idInRect][2].x,RectangleVertex[1][idInRect][2].y},
                                                {RectangleVertex[1][idInRect][3].x,RectangleVertex[1][idInRect][3].y}};//第k+1帧bounding box 二维
                proposalType box1(tmp_box_1);
                proposalType box2(tmp_box_2);
                float result1 = IOU(box1, box2, 0);
                float result2 = IOU(box1, box2 ,1);//计算交并比
                float result3 = IOU(box1, box2 ,2);
                // std::cout<<k<<" result "<<result1<<" "<<result2<<" "<<result3<<std::endl;
                if(result1>=IOUThreshold){//根据计算结果判断是否保存该点云数据
                    tmp_count++;
                    if(!label_set.count(pointSearchRadius.intensity))
                        label_set.insert(pointSearchRadius.intensity);
                    break;
                }
            }
        }
        pcl::PointCloud<PointType>::Ptr pubCloud(new pcl::PointCloud<PointType>());
        for(auto point:pointCloudsKtime[0]->points){
            if(label_set.count(point.intensity)){
                PointType push_point;
                // push_point.x = point.y;
                // push_point.y = point.z;
                // push_point.z = point.x;
                push_point.x = point.x;
                push_point.y = point.y;
                push_point.z = point.z;
                push_point.intensity = point.intensity;

                pubCloud->push_back(push_point);
            }
        }

        pcl::PointCloud<PointType>::Ptr tempGround(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(groundMsg.front(), *tempGround);
        //*pubCloud += *tempGround;

        //////////trans//////////////////////
        Eigen::Isometry3d nowPoseTemp = Eigen::Isometry3d::Identity();
        nowPoseTemp.rotate(Eigen::Quaterniond(odomMsgIn.front().pose.pose.orientation.w,
                                               odomMsgIn.front().pose.pose.orientation.x,
                                               odomMsgIn.front().pose.pose.orientation.y,
                                               odomMsgIn.front().pose.pose.orientation.z));
        nowPoseTemp.pretranslate(Eigen::Vector3d(odomMsgIn.front().pose.pose.position.x,
                                                 odomMsgIn.front().pose.pose.position.y,
                                                 odomMsgIn.front().pose.pose.position.z));

        Eigen::Matrix4d transMatrix = nowPoseTemp.matrix();
        pcl::transformPointCloud(*pubCloud, *pubCloud, transMatrix);

        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*pubCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = lidarPointCloudsQueue.front().header.stamp;
        laserCloudTemp.header.frame_id = lidarPointCloudsQueue.front().header.frame_id;
        remove_obstacle_cloud_ = laserCloudTemp;

        tempGround->clear();
        pubCloud->clear();
        /////////main handle//////////////
    }


public:
    obstacle()
    {
        //
        bboxT.resize(2);
        bboxQ.resize(2);
        whd.resize(2);
        //
        IOUThreshold = 0.8;//0.8
        //
        PointCloud_withLabel.reset(new pcl::PointCloud<PointType>());
        label_value.reset(new pcl::PointCloud<PointType>());
        removeObstacle.reset(new pcl::PointCloud<PointType>());
        dynamicObstacle.reset(new pcl::PointCloud<PointType>());
        fullCloudWithLabelAndIndex.reset(new pcl::PointCloud<PointType>());
        fullCloudWithLabelAndIndex->points.resize(N_SCAN * Horizon_SCAN);
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        //
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;
        std::fill(fullCloudWithLabelAndIndex->points.begin(),fullCloudWithLabelAndIndex->points.end(), nanPoint);
        //
        center.x = 0;
        center.y = 0;
        center.z = 0;
        max_point.x = std::numeric_limits<float>::lowest();
        max_point.y = std::numeric_limits<float>::lowest();
        max_point.z = std::numeric_limits<float>::lowest();
        min_point.x = std::numeric_limits<float>::max();
        min_point.y = std::numeric_limits<float>::max();
        min_point.z = std::numeric_limits<float>::max();
        //
    }
    void run(sensor_msgs::PointCloud2 cluster_cloud,
             sensor_msgs::PointCloud2 ground_cloud,
             nav_msgs::Odometry cluster_odom,
             //输出
             sensor_msgs::PointCloud2& remove_obstacle_cloud,
             nav_msgs::Odometry& OBS_remove_odom)
    {
        //
        sensor_msgs::PointCloud2 tmpMsgA = ground_cloud;
        groundMsg.push_back(tmpMsgA);
        if(groundMsg.size() > queLength)
            groundMsg.pop_front();
        //
        sensor_msgs::PointCloud2 tmpMsg = cluster_cloud;
        lidarPointCloudsQueue.push_back(tmpMsg);
        if(lidarPointCloudsQueue.size() > queLength)
            lidarPointCloudsQueue.pop_front();
        newLaserPointCloud = true;
        //
        Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = cluster_odom.pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        //-pitch, -yaw, roll
        //roll     pitch   yaw
        geometry_msgs::Quaternion qOut = tf::createQuaternionMsgFromRollPitchYaw(roll, -pitch, -yaw);
        current_pose.rotate(Eigen::Quaterniond(qOut.w, qOut.x, qOut.y, qOut.z));
        current_pose.pretranslate(Eigen::Vector3d(cluster_odom.pose.pose.position.z,
                                                  cluster_odom.pose.pose.position.x,
                                                  cluster_odom.pose.pose.position.y));
        poseQueue.push_back(current_pose);
        odomMsgIn.push_back(cluster_odom);
        if(poseQueue.size()>queLength){
            poseQueue.pop_front();
            odomMsgIn.pop_front();
        }
        newLaserOdometry = true;
        //
        if(newLaserPointCloud && newLaserOdometry && poseQueue.size() == queLength && lidarPointCloudsQueue.size() == queLength)
        {
            newLaserOdometry = false;
            newLaserPointCloud = false;
            if((lidarPointCloudsQueue.front().header.stamp.toSec() - odomMsgIn.front().header.stamp.toSec())>=0.2)
                return;
            if((odomMsgIn.front().header.stamp.toSec() - lidarPointCloudsQueue.front().header.stamp.toSec())>=0.2)
                return;
            pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
            Eigen::Isometry3d currentPose   = poseQueue[1];
            Eigen::Isometry3d lastPose      = poseQueue[0];
            Eigen::Isometry3d relativePose  = lastPose.inverse() * currentPose;
            Eigen::Matrix4d lastMatrix      = lastPose.matrix();
            Eigen::Matrix3d rotationMatrix  = lastMatrix.block<3,3>(0,0);
            pubPosition     = lastMatrix.block<3,1>(0,3);
            pubOrientation  = rotationMatrix;
            pubOrientation.normalize();
            //////////////////////////////////////////
            for(int i = 0; i < queLength; i++)
            {
                laserCloudIn->clear();
                pcl::fromROSMsg(lidarPointCloudsQueue[i], *laserCloudIn);
                laserDataHead.push_back(lidarPointCloudsQueue[i].header);
                intensity_set.clear();

                int cloudSize = laserCloudIn->points.size();
                PointCloud_withLabel->clear();
                for(int j = 0; j < cloudSize; j++)
                {
                    PointType thisPoint = laserCloudIn->points[j];
                    if( !intensity_set.count(thisPoint.intensity) && thisPoint.intensity >= 0)
                        intensity_set.insert(thisPoint.intensity);
                    if(thisPoint.intensity >= 0)
                        PointCloud_withLabel->push_back(thisPoint);
                }

                pcl::PointCloud<PointType>::Ptr tmpLaserCloud(new pcl::PointCloud<PointType>());
                if( i== 1)
                {
                    //利用两帧数据之间的里程计信息，将两帧点云对齐
                    pcl::transformPointCloud(*PointCloud_withLabel, *tmpLaserCloud, relativePose.matrix());
                }
                else
                {
                    *tmpLaserCloud += *PointCloud_withLabel;
                }
                laserCloudInqueue.push_back(*tmpLaserCloud);
                tmpLaserCloud->clear();
            }
            laserCloudIn->clear();
            ///////////////
            remove_dynamic_obstacles(remove_obstacle_cloud);
            OBS_remove_odom = odomMsgIn.front();
            ///////////////
            for(int i = 0; i < laserCloudInqueue.size(); i++)
                laserCloudInqueue[i].clear();
            laserCloudInqueue.clear();
        }

    }
};


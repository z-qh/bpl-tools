#include "include/utility.h"
#include "include/dbscan/DBSCAN_paper.h"
#include "include/dbscan/DBSCAN_kdtree.h"
#include "include/dbscan/DBSCAN_flann.h"

class Intersection{
private:
/*
通过对地面点进行pca分析，判断是否为路口或者拐角区域
实现对三维点云数据聚类 剔除了离群点
*/
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
/////////地面点分割相关/////
    cv::Mat rangeMat;
    cv::Mat labelMat;
    cv::Mat groundMat;
    pcl::PointCloud<PointType>::Ptr fullLaserCloud;
    pcl::PointCloud<PointType>::Ptr groundLaserCloud;
    PointType nanPoint;
    pcl::VoxelGrid<PointType>downSizeFilter;
    pcl::VoxelGrid<PointType>gorundCloudDownSizeFilter;
    pcl::PointCloud<PointType>::Ptr groundInLine[groundScanInd];
    std::vector<pair<int8_t, int8_t>>neighbors;
    std_msgs::Header cloudHeader;

    nav_msgs::Odometry odomIn;
/////////////////////////////////////////////////
    Eigen::Isometry3d currentPose = Eigen::Isometry3d::Identity();
    bool recognizeIntersection = false;
    float valueFactor = 1.5;
    float groundAngle = 5.0;
    std::deque<bool>is_Intersection;        //存储是否为路口点状态，当连续一段时间内都为路口才判断成功
    float intersectionThreshold;            //判断是否为路口阈值
    int intersectionLength = 10;
/////////////////////////////////////////////////
    bool newLaserOdometry = false;
    bool newLaserCloud    = false;
    double timeLaserOdometry = 0;
    double timeLaserCloud    = 0;
    std::mutex mtx;
////////////////////////
//segmentation
    uint16_t *queueIndX;
    uint16_t *queueIndY;
    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;
    int labelCount = 1;
///////////////////////
    bool isDownSize;
    int downSizeThreshold;
    double clusterTolerance=0.1;            //dbscan聚类搜索半径
    int min_cluster_size_param = 20;        //聚类后点的数目最小值
    int core_point_min_pts_param = 15;      //判断不为噪声点的阈值
    double carLength = 2.5;
    double maxLength = 100.0;
    bool isDynamic = false;
/////////////////////////////////////////////////
private:
    void resetParameters(){
        laserCloudIn->clear();
        groundLaserCloud->clear();
        labelCount = 1;
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));

        std::fill(fullLaserCloud->points.begin(), fullLaserCloud->points.end(), nanPoint);
        for(int i=0; i<groundScanInd; ++i){
            groundInLine[i].reset(new pcl::PointCloud<PointType>());
        }
    }
    void groundExtract(sensor_msgs::PointCloud2& intersectionCloud_,
                       sensor_msgs::PointCloud2& ground_cloud_){
        int lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        for(size_t j=0; j<Horizon_SCAN; ++j){
            for(size_t i=0; i<groundScanInd; ++i){
                lowerInd = j + i * Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if(fullLaserCloud->points[lowerInd].intensity == -1 ||
                   fullLaserCloud->points[upperInd].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                diffX = fullLaserCloud->points[upperInd].x - fullLaserCloud->points[lowerInd].x;
                diffY = fullLaserCloud->points[upperInd].y - fullLaserCloud->points[lowerInd].y;
                diffZ = fullLaserCloud->points[upperInd].z - fullLaserCloud->points[lowerInd].z;
                //angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY))*180.0/M_PI;//qh add for debug
                angle = atan2(diffY, sqrt(diffX*diffX + diffZ*diffZ))*180.0/M_PI;
                if(fabs(angle)<=groundAngle){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        for(size_t i = 0; i<N_SCAN; ++i){
            for(size_t j=0; j<Horizon_SCAN; ++j){
                if(groundMat.at<int8_t>(i,j) == 1){
                    groundLaserCloud->push_back(fullLaserCloud->points[j + i*Horizon_SCAN]);
                    if(i<groundScanInd){
                        PointType thisPoint = fullLaserCloud->points[j + i*Horizon_SCAN];
                        thisPoint.y=0;
                        groundInLine[i]->push_back(thisPoint);
                    }
                }
                if(groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        //对每条线进行聚类
        pcl::PointCloud<PointType>::Ptr clusterground(new pcl::PointCloud<PointType>());
        for(int i=0;i<groundScanInd;++i){
            if(groundInLine[i]->points.size()>10){
                pcl::KdTreeFLANN<PointType>::Ptr tree(new pcl::KdTreeFLANN<PointType>);
                tree->setInputCloud(groundInLine[i]);
                std::vector<pcl::PointIndices> cluster_indices;

                DBSCANFlannCluster<PointType> ec;
                ec.setCorePointMinPts(8);
                ec.setClusterTolerance(0.1);
                ec.setMinClusterSize(10);
                ec.setHorizonAngleResolution(ang_res_x);
                ec.setVerticalAngleResolution(ang_res_y);
                ec.setMaxClusterSize(50000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(groundInLine[i]);
                ec.extract(cluster_indices);//聚类后所有的点的索引值存放在数组中
                pcl::PointCloud<PointType>::Ptr cloud_clustered(new pcl::PointCloud<PointType>());
                for(auto cluster = cluster_indices.begin(); cluster!= cluster_indices.end(); ++cluster){
                    float max_x = -9999, max_y = -9999, min_x = 9999, min_y = 9999;
                    for(auto index=cluster->indices.begin(); index!=cluster->indices.end(); ++index){
                        PointType thisPoint;
                        thisPoint.x = groundInLine[i]->points[*index].x;
                        thisPoint.y = groundInLine[i]->points[*index].y;
                        thisPoint.z = groundInLine[i]->points[*index].z;
                        thisPoint.intensity = i;
                        if(max_x < thisPoint.z){
                            max_x = thisPoint.z;
                        }
                        if(max_y < thisPoint.x){
                            max_y = thisPoint.x;
                        }
                        if(min_x > thisPoint.z){
                            min_x = thisPoint.z;
                        }
                        if(min_y > thisPoint.x){
                            min_y = thisPoint.x;
                        }
                        cloud_clustered->push_back(thisPoint);
                    }
                    if(min_x!=9999 && min_y != 9999 && max_x !=-9999 && min_y!=-9999 &&
                       sqrt((max_x - min_x)*(max_x - min_x) + (max_y - min_y)*(max_y - min_y)) >=carLength
                       && sqrt((max_x - min_x)*(max_x - min_x) + (max_y - min_y)*(max_y - min_y))< maxLength){
                        *clusterground += *cloud_clustered;
                    }
                    cloud_clustered->clear();
                }
            }
        }

        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*clusterground, laserCloudTemp);
        laserCloudTemp.header = cloudHeader;
        ground_cloud_ = laserCloudTemp;

        //降采样和不降采样可以选择，降采样可以使密度相差不是很大
        pcl::PointCloud<PointType>::Ptr afterDownSize(new pcl::PointCloud<PointType>());
        gorundCloudDownSizeFilter.setInputCloud(clusterground);
        gorundCloudDownSizeFilter.filter(*afterDownSize);
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*afterDownSize, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*afterDownSize, pcaCentroid, covariance);
        //SelfAdjointEigenSolver分解特征值和特征向量时，特征值从小到大存储，特征向量按列存储col
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
        // std::cout<<"values "<<eigenValuesPCA(0)<<" "<<eigenValuesPCA(1)<<" "<<eigenValuesPCA(2)<<std::endl;
        //1.5可以进行调整
        // if(eigenValuesPCA(1)*valueFactor>eigenValuesPCA(2) && eigenValuesPCA(2)<120){
        if(eigenValuesPCA(1)*valueFactor>eigenValuesPCA(2)){
            is_Intersection.push_back(true);
            if(is_Intersection.size() > intersectionLength){
                is_Intersection.pop_front();
                int count_true = 0;
                for(int k=0; k<intersectionLength; ++k){
                    if(is_Intersection[k] == true)
                        count_true++;
                }
               if((float)(count_true/(intersectionLength*1.0))>= (intersectionThreshold)){
                    pcl::PointCloud<PointType>intersectionCloud;
                    intersectionCloud.clear();
                    PointType thisPose;
                    thisPose.x = currentPose.translation().x();
                    thisPose.y = currentPose.translation().y();
                    thisPose.z = currentPose.translation().z();
                    intersectionCloud.push_back(thisPose);
                    //输出位姿点
                   sensor_msgs::PointCloud2 laserCloudTemp;
                   pcl::toROSMsg(intersectionCloud, laserCloudTemp);
                   laserCloudTemp.header = cloudHeader;
                   intersectionCloud_ = laserCloudTemp;

                    intersectionCloud.clear();
                    for(int k=0; k<intersectionLength; ++k){
                        is_Intersection[k]=false;
                    }
                }
            }
        }else{
            is_Intersection.push_back(false);
            if(is_Intersection.size()>intersectionLength)
                is_Intersection.pop_front();
        }
        clusterground->clear();
        afterDownSize->clear();
    }
    void projectionPointCloud()
    {
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index;
        PointType thisPoint;
        int cloudSize = laserCloudIn->points.size();

        //qh add for debug ouster
        for(int i=0; i < cloudSize; i++)
        {
            PointType TTTT = laserCloudIn->points[i];
            laserCloudIn->points[i].x = TTTT.y;
            laserCloudIn->points[i].y = TTTT.z;
            laserCloudIn->points[i].z = TTTT.x;
            laserCloudIn->points[i].intensity = TTTT.intensity;
        }

        //qh add for debug lslidar
//        for(int i=0; i < cloudSize; i++)
//        {
//            pcl::PointXYZI TTTT = laserCloudIn->points[i];
//            laserCloudIn->points[i].x = TTTT.x;
//            laserCloudIn->points[i].y = TTTT.z;
//            laserCloudIn->points[i].z = -TTTT.y;
//            laserCloudIn->points[i].intensity = TTTT.intensity;
//        }

        //这里针对velodyne的雷达处理//qh comment for debug
        for(int i=0; i<cloudSize; ++i){
            thisPoint = laserCloudIn->points[i];
            verticalAngle = std::atan2(thisPoint.y, sqrt(thisPoint.x*thisPoint.x + thisPoint.z*thisPoint.z))*180/M_PI;
            rowIdn = (verticalAngle + ang_bottom)/ang_res_y;
            if(rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
            horizonAngle = atan2(thisPoint.z, thisPoint.x)*180.0 / M_PI;
            columnIdn = -round((horizonAngle - 90.0)/ang_res_x) + Horizon_SCAN/2;
            if(columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            if(columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
            rangeMat.at<float>(rowIdn,columnIdn) = range;
            thisPoint.intensity = (float)rowIdn + (float)columnIdn/10000.0;
            index = columnIdn + rowIdn * Horizon_SCAN;
            fullLaserCloud->points[index] = thisPoint;
        }
    }
    //三维点云快速分割
    void labelComponents(int row, int col){
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        //queueSize未处理点的数目
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];//
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;//后移
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;//标记类别
            for (auto iter = neighbors.begin(); iter != neighbors.end(); ++iter){
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;
                //寻找最大的
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                // alpha代表角度分辨率，
                // X方向上角度分辨率是segmentAlphaX(rad)
                // Y方向上角度分辨率是segmentAlphaY(rad)
                if ((*iter).first == 0)//水平方向上的邻点
                    alpha = ang_res_x/180.0*M_PI;//水平方向上的分辨率
                else
                    alpha = ang_res_y/180.0*M_PI;
                // 通过下面的公式计算这两点之间是否有平面特征
                // atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));
                //如果夹角大于60°，则将这个邻点纳入到局部特征中，该邻点可以用来配准使用
                float angleThreshold = 10/180.0*M_PI;
                if (angle > angleThreshold){//1.0472（π/3）

                    queueIndX[queueEndInd] = thisIndX;//将当前点加入队列中
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;//角度大于60°，为同一个标签
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }
        bool feasibleSegment = false;
        int verticalThreshold = 5,verticalLineNum = 3;
        //当邻点数目达到30后，则该帧雷达点云的几何特征配置成功
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
            //如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数，若竖直方向上被统计的数目大于3，为有效聚类
        else if (allPushedIndSize >= verticalThreshold){//segmentValidPointNum = 5
            int lineCount = 0;
            for (size_t i = 0; i <N_SCAN ; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= verticalLineNum)
                feasibleSegment = true;
        }

        if (feasibleSegment == true){
            ++labelCount;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }
    //点云分割聚类
    void cloudSegmentation(sensor_msgs::PointCloud2& cluster_cloud,
                           sensor_msgs::PointCloud2& noground_cloud_){
        for(int i=0; i<N_SCAN; ++i){
            for(int j=0; j<Horizon_SCAN; ++j){
                if(labelMat.at<int>(i,j) == 0){
                    labelComponents(i,j);
                }
            }
        }
        //获取非地面点
        pcl::PointCloud<PointType>::Ptr withOutGroundPoints(new pcl::PointCloud<PointType>());
        withOutGroundPoints->clear();
        for(size_t i=0; i<N_SCAN; ++i){
            for(size_t j=0; j<Horizon_SCAN; ++j){
                if(labelMat.at<int>(i,j)>0 && labelMat.at<int>(i,j)!=999999){
                    PointType thisPoint;
                    thisPoint.x = fullLaserCloud->points[j + i*Horizon_SCAN].x;
                    thisPoint.y = fullLaserCloud->points[j + i*Horizon_SCAN].y;
                    thisPoint.z = fullLaserCloud->points[j + i*Horizon_SCAN].z;
                    thisPoint.intensity = labelMat.at<int>(i,j);
                    withOutGroundPoints->push_back(thisPoint);
                }
            }
        }


        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*withOutGroundPoints, laserCloudTemp);
        laserCloudTemp.header = cloudHeader;
        noground_cloud_ = laserCloudTemp;

        //对非地面点云进行dbscan聚类
        int withOutGroundNumber = withOutGroundPoints->points.size();

        pcl::PointCloud<PointType>::Ptr downSizePoints(new pcl::PointCloud<PointType>());
        downSizePoints->clear();
        //当点云数目较多时，进行降采样算法
        if(isDownSize && withOutGroundNumber>downSizeThreshold){
            downSizeFilter.setInputCloud(withOutGroundPoints);
            downSizeFilter.filter(*downSizePoints);
        }else{
            *downSizePoints += *withOutGroundPoints;
        }
        //将三维点云进行聚类
        pcl::KdTreeFLANN<PointType>::Ptr tree(new pcl::KdTreeFLANN<PointType>);
        tree->setInputCloud(downSizePoints);
        std::vector<pcl::PointIndices> cluster_indices;
        DBSCANPaperCluster<PointType> ec;
        ec.setCorePointMinPts(core_point_min_pts_param);
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(min_cluster_size_param);
        ec.setHorizonAngleResolution(0.18);
        ec.setVerticalAngleResolution(1);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(downSizePoints);
        ec.setS(1.5);
        ec.setDynamic(isDynamic);   //阈值是否动态调整
        ec.extract(cluster_indices);//聚类后所有的点的索引值存放在数组中

        pcl::PointCloud<PointType>::Ptr cloud_clustered(new pcl::PointCloud<PointType>());
        int label = 0;
        for(auto cluster = cluster_indices.begin(); cluster!= cluster_indices.end(); ++cluster,++label){
            for(auto index=cluster->indices.begin(); index!=cluster->indices.end(); ++index){
                PointType thisPoint;
                thisPoint.x = downSizePoints->points[*index].x;
                thisPoint.y = downSizePoints->points[*index].y;
                thisPoint.z = downSizePoints->points[*index].z;
                thisPoint.intensity = label;
                cloud_clustered->push_back(thisPoint);
            }
        }

        pcl::toROSMsg(*cloud_clustered, laserCloudTemp);
        laserCloudTemp.header = cloudHeader;
        cluster_cloud =  laserCloudTemp;//点云的强度代表了种类

        downSizePoints->clear();
        withOutGroundPoints->clear();
        cloud_clustered->clear();
    }

public:
    static bool comparePairByMyself(pair<double, vector<int>> p1, pair<double, vector<int>> p2)
    {
        return p1.second.size() > p2.second.size();
    }
public:
    void analyzeCloud(const sensor_msgs::PointCloud2 result, sensor_msgs::PointCloud2& show_seg_cloud_)
    {
        pcl::PointCloud<PointType> sourceCloud;
        pcl::fromROSMsg(result, sourceCloud);
        int cloudSize = sourceCloud.points.size();
        //统计种类信息
        map<double, vector<int>> infMap;//key-count
        for(int i = 0; i < cloudSize; i++)
        {
            PointType tempP = sourceCloud.points[i];
            map<double, vector<int>>::iterator it = infMap.find(tempP.intensity);
            if(it != infMap.end())
                it->second.push_back(i);
            else
                infMap.insert(make_pair(tempP.intensity, vector<int>()));
        }
        cout << "total : " << infMap.size() << " kinds cloud seg!" << endl;
        vector<pair<double, vector<int>>> infVec;
        for(auto pairI : infMap)
        {
            infVec.push_back(pairI);
        }
        sort(infVec.begin(), infVec.end(), comparePairByMyself);
        cout << "and they are: " << endl;
        if(cloudSize < 5)
            for(int i = 0; i < cloudSize; i++)
            {
                cout << i << " kind has: " << infVec[i].second.size() << " points!" << endl;
            }
        else
            for(int i = 0; i < 5; i++)
            {
                cout << i << " kind has: " << infVec[i].second.size()<< " points!" << endl;
            }
        pcl::PointCloud<pcl::PointXYZRGB> showCloud;
        int colorCount = 0;
        for(auto pairI : infMap)
        {
            colorCount++;
            if(colorCount > 5)
                colorCount = 0;
            double R(colorCount*40), G(colorCount*40), B(colorCount*40);
            for(int k : pairI.second)
            {
                pcl::PointXYZRGB tempP;
                tempP.x  = sourceCloud[k].x;
                tempP.y  = sourceCloud[k].y;
                tempP.z  = sourceCloud[k].z;
                tempP.r = R;
                tempP.g = G;
                tempP.b = B;
                showCloud.push_back(tempP);
            }
        }
        pcl::toROSMsg(showCloud, show_seg_cloud_);
        show_seg_cloud_.header = cloudHeader;
    }

public:
    Intersection()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        //参数配置
        valueFactor = 1.7;
        groundAngle = 10;
        isDownSize = false;
        min_cluster_size_param = 30;
        core_point_min_pts_param = 8;
        intersectionThreshold = 0.7;
        intersectionLength = 7;
        downSizeThreshold = 25000;
        isDynamic = true;
        clusterTolerance = 0.1;
        carLength = 3.0;
        maxLength = 100.0;

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        groundLaserCloud.reset(new pcl::PointCloud<PointType>());
        fullLaserCloud.reset(new pcl::PointCloud<PointType>());
        fullLaserCloud->points.resize(N_SCAN * Horizon_SCAN);

        pair<int8_t,int8_t>neigbhor;
        neigbhor.first = -1;    neigbhor.second = 0;    neighbors.push_back(neigbhor);
        neigbhor.first = 0;     neigbhor.second = 1;    neighbors.push_back(neigbhor);
        neigbhor.first = 0;     neigbhor.second = -1;   neighbors.push_back(neigbhor);
        neigbhor.first = 1;     neigbhor.second = 0;    neighbors.push_back(neigbhor);

        queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
        allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

        downSizeFilter.setLeafSize(0.09,0.09,0.09);
        gorundCloudDownSizeFilter.setLeafSize(0.07,0.07,0.07);

        resetParameters();
    }
    void run(sensor_msgs::PointCloud2 input_point_cloud,
        nav_msgs::Odometry aft_mapped_to_init,
        //输出
        sensor_msgs::PointCloud2& cluster_cloud,
        sensor_msgs::PointCloud2& intersection,
        nav_msgs::Odometry& odom_cluster,
        sensor_msgs::PointCloud2& ground_cloud,
        sensor_msgs::PointCloud2& noground_cloud,
        sensor_msgs::PointCloud2& show_seg_cloud)
    {
        //
        cloudHeader = input_point_cloud.header;
        pcl::fromROSMsg(input_point_cloud, *laserCloudIn);
        timeLaserCloud = input_point_cloud.header.stamp.toSec();
        newLaserCloud = true;
        //
        odomIn = aft_mapped_to_init;
        currentPose = Eigen::Isometry3d::Identity();
        currentPose.rotate(Eigen::Quaterniond(odomIn.pose.pose.orientation.w,
                                              odomIn.pose.pose.orientation.x,
                                              odomIn.pose.pose.orientation.y,
                                              odomIn.pose.pose.orientation.z));
        currentPose.pretranslate(Eigen::Vector3d(odomIn.pose.pose.position.x,
                                                 odomIn.pose.pose.position.y,
                                                 odomIn.pose.pose.position.z));
        timeLaserOdometry = aft_mapped_to_init.header.stamp.toSec();
        newLaserOdometry = true;
        //
        if(newLaserOdometry && newLaserCloud)
        {
            newLaserOdometry = false;
            newLaserCloud = false;
            projectionPointCloud();
            groundExtract(intersection, ground_cloud);
            cloudSegmentation(cluster_cloud, noground_cloud);
            //analyzeCloud(cluster_cloud, show_seg_cloud);
            resetParameters();
        }
        //输出位姿
        odom_cluster = odomIn;

    }
};

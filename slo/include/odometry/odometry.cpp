#include "odometry.h"

std::map<std::pair<int, int>, Eigen::Isometry3d> load_cache(string path) {
    std::map<std::pair<int, int>, Eigen::Isometry3d> ans;
    ifstream file(path);
    while (file.good()) {
        string tmpLine;
        getline(file, tmpLine);
        if (file.eof())
            break;
        stringstream ss(tmpLine);
        int sourceTime, TargetTime;
        Eigen::Vector3d tmpt = Eigen::Vector3d::Zero();
        Eigen::Quaterniond tmpQ = Eigen::Quaterniond::Identity();
        ss >> sourceTime >> TargetTime;
        ss >> tmpt(0) >> tmpt(1) >> tmpt(2);
        ss >> tmpQ.x() >> tmpQ.y() >> tmpQ.z() >> tmpQ.w();
        Eigen::Isometry3d tmpT = Eigen::Isometry3d::Identity();
        tmpT.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(tmpQ);
        tmpT.matrix().block<3, 1>(0, 3) = tmpt;
        ans.insert({{sourceTime, TargetTime}, tmpT});
    }
    file.close();
    cout << "Load Cache " << ans.size() << endl;
    return ans;
}

void dump_cache(std::map<std::pair<int, int>, Eigen::Isometry3d> cache, string path) {
    ofstream file(path);
    for (auto& p : cache) {
        file << setprecision(4) << std::fixed;
        file << p.first.first << " " << p.first.second << " ";
        Eigen::Quaterniond tmpQ(p.second.rotation().matrix());
        Eigen::Vector3d tmpt(p.second.translation());
        file << tmpt(0) << " " << tmpt(1) << " " << tmpt(2) << " ";
        file << tmpQ.x() << " " << tmpQ.y() << " " << tmpQ.z() << " " << tmpQ.w() << std::endl;
    }
    file.close();
    cout << "Dump Cache " << cache.size() << endl;
}

Odometry::Odometry(string bin_p, string label_p, bool intensity, string cache_)
    : FR(FileReader(bin_p, label_p, intensity))
    , q_w_curr(pose_parameters)
    , t_w_curr(pose_parameters + 4)
    , cache(cache_) {
    if (cache != "") {
        smatch_cache = load_cache(cache);
    }
    init();
    reset();
}

Odometry::~Odometry() {
    if (cache != "") {
        dump_cache(smatch_cache, cache);
    }
}

void Odometry::init() {
    // factor noise
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);
    // factor isam
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    // feature
    laserCloudFeatureFromMap.reset(new pcl::PointCloud<pcl::PointXYZL>());
    laserCloudFeatureFromMapDS.reset(new pcl::PointCloud<pcl::PointXYZL>());
    localMapFilter.setLeafSize(0.4, 0.4, 0.4);
    laserCloudFeatureFromMapDSNum = 0;
    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>());
    downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);
    downSave.setLeafSize(0.4, 0.4, 0.4);

    // pose
    cloudKeyPoses3D.reset(new pcl::PointCloud<pcl::PointXYZL>());

    poseLocalMatch.setIdentity();
    poseAfterMapped.setIdentity();
    local_pose_increasement.setIdentity();

    q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
    t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

    q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
    t_wodom_curr = Eigen::Vector3d(0, 0, 0);

    timeLaserInfoCur = 0;
    timeLaserInfoPre = 0;

    // loop closure
    aLoopIsClosed = false;
    loopKdTree.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>());
    loopFilter.setLeafSize(0.4, 0.4, 0.4);

    kdtreeFeatureFromMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>());
}

void Odometry::reset() {}
bool Odometry::step(bool test){
    
}
bool Odometry::step() {
    if (!loopIndexContainer.empty()) {
    }
    auto ori_cloud = FR.get();
    timeLaserInfoPre = timeLaserInfoCur;
    timeLaserInfoCur = FR.getTime();
    laserOriCloudLast = laserOriCloud;
    laserOriCloud = ori_cloud;
    auto global_feature_cloud = FE.get(ori_cloud, 10, 20, 20);
    laserFeatureCloud = global_feature_cloud;
    // 1 local match to update pose guess
    auto localMatch_start = std::chrono::steady_clock::now();
    if (!OldFrame) {
        OldFrame = FileReader::toS(ori_cloud);
        local_pose_increasement.setIdentity();
    } else {
        if (cache != "" && smatch_cache.find({(int)(timeLaserInfoCur * 10), (int)(timeLaserInfoPre * 10)}) != smatch_cache.end()) {
            auto it = smatch_cache.find({timeLaserInfoCur, timeLaserInfoPre});
            local_pose_increasement = it->second;
        } else {
            auto local_feature_cloud = FE.get(ori_cloud, 4, 20, 20);
            NewFrame = FileReader::toS(local_feature_cloud);
            semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>::Ptr tmpLocalFinal;
            tmpLocalFinal.reset(new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>());
            semanticicp::SemanticIterativeClosestPoint<pcl::PointXYZ, uint32_t> local_matcher;
            local_matcher.setInputSource(NewFrame);
            local_matcher.setInputTarget(OldFrame);
            local_matcher.align(tmpLocalFinal);
            local_pose_increasement = Eigen::Isometry3d(local_matcher.getFinalTransFormation().matrix());
            if (cache != "")
                smatch_cache.insert({{(int)(timeLaserInfoCur * 10), (int)(timeLaserInfoPre * 10)}, local_pose_increasement});
        }
        OldFrame = FileReader::toS(ori_cloud);
        poseLocalMatch = poseLocalMatch * local_pose_increasement;
        q_wodom_curr = Eigen::Quaterniond(poseLocalMatch.rotation());
        t_wodom_curr = Eigen::Vector3d(poseLocalMatch.translation());
    }
    auto localMatch_end = std::chrono::steady_clock::now();
    // printf("\033[32m local match :%5.2fms.\033[0m\n", chrono::duration<double>(localMatch_end - localMatch_start).count() * 1000);
    // 2 mapping refine and update map

    // 2.1 update guess
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

    // 2.2 extract local map
    auto extractMap_start = std::chrono::steady_clock::now();
    if (!cloudKeyPoses3D->points.empty()) {
        laserCloudFeatureFromMap->clear();
        auto t1 = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZL>::Ptr surroundingKeyPoses(new pcl::PointCloud<pcl::PointXYZL>());
        pcl::PointCloud<pcl::PointXYZL>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<pcl::PointXYZL>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), 50.0, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for (auto& pt : surroundingKeyPosesDS->points) {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.label = cloudKeyPoses3D->points[pointSearchInd[0]].label;
        }
        int numPoses = cloudKeyPoses3D->points.size();
        for (int i = numPoses - 1; i >= 0; --i) {
            if (timeLaserInfoCur - keyPoses6DTime[i] < 10.0) {
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            } else {
                break;
            }
        }
        auto t2 = std::chrono::steady_clock::now();
        for (int i = 0; i < (int)surroundingKeyPosesDS->points.size(); ++i) {
            int thisKeyInd = surroundingKeyPosesDS->points[i].label;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
                *laserCloudFeatureFromMap += laserCloudMapContainer[thisKeyInd];
            } else {
                pcl::PointCloud<pcl::PointXYZL> laserCloudFeatureTemp =
                    *transformPointCloud(FeatureCloudKeyFrames[thisKeyInd], keyPoses6D[thisKeyInd].matrix());
                *laserCloudFeatureFromMap += laserCloudFeatureTemp;
                laserCloudMapContainer[thisKeyInd] = laserCloudFeatureTemp;
            }
        }
        auto t3 = std::chrono::steady_clock::now();
        localMapFilter.setInputCloud(laserCloudFeatureFromMap);
        localMapFilter.filter(*laserCloudFeatureFromMapDS);
        laserCloudFeatureFromMapDSNum = laserCloudFeatureFromMapDS->points.size();
        if (laserCloudMapContainer.size() > 1000) {
            laserCloudMapContainer.clear();
        }
        auto t4 = std::chrono::steady_clock::now();

        // printf("\033[34m extract key :%5.2fms.\033[0m\n"
        //        "\033[34m accumul map :%5.2fms.\033[0m\n"
        //        "\033[34m downsiz map :%5.2fms.\033[0m\n",
        //        chrono::duration<double>(t2 - t1).count() * 1000, chrono::duration<double>(t3 - t2).count() * 1000,
        //        chrono::duration<double>(t4 - t3).count() * 1000);
    }
    auto extractMap_end = std::chrono::steady_clock::now();
    // printf("\033[33m extract map :%5.2fms.\033[0m\n", chrono::duration<double>(extractMap_end - extractMap_start).count() * 1000);

    // 2.3 scan map match
    auto mapMatch_start = std::chrono::steady_clock::now();
    if (laserCloudFeatureFromMapDSNum > 50) {
        kdtreeFeatureFromMap->setInputCloud(laserCloudFeatureFromMapDS);
        for (int iterCount = 0; iterCount < 10; iterCount++) {

            ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;

            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(pose_parameters, 4, q_parameterization);
            problem.AddParameterBlock(pose_parameters + 4, 3);

            int surf_num = 0;
            for (int i = 0; i < (int)laserFeatureCloud->points.size(); i++) {
                pcl::PointXYZL& pointOri = laserFeatureCloud->points[i];
                static auto pointAssociateToMap = [&](pcl::PointXYZL& pi) -> pcl::PointXYZL {
                    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
                    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
                    pcl::PointXYZL po;
                    po.x = point_w[0];
                    po.y = point_w[1];
                    po.z = point_w[2];
                    po.label = pi.label;
                    return po;
                };
                pcl::PointXYZL pointSel = pointAssociateToMap(pointOri);
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtreeFeatureFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0) {
                    for (int j = 0; j < 5; j++) {
                        matA0(j, 0) = laserCloudFeatureFromMapDS->points[pointSearchInd[j]].x;
                        matA0(j, 1) = laserCloudFeatureFromMapDS->points[pointSearchInd[j]].y;
                        matA0(j, 2) = laserCloudFeatureFromMapDS->points[pointSearchInd[j]].z;
                    }

                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    // Ax + By + Cz + 1 = 0
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++) {
                        uint32_t thisLabel = laserCloudFeatureFromMapDS->points[pointSearchInd[j]].label;
                        if (thisLabel != pointSel.label && (thisLabel == 17 || thisLabel == 19 || thisLabel == 20)) {
                            continue;
                        }
                        // (x0, y0, z0) Ax + By + Cz + D = 0  = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
                        if (fabs(norm(0) * laserCloudFeatureFromMapDS->points[pointSearchInd[j]].x +
                                 norm(1) * laserCloudFeatureFromMapDS->points[pointSearchInd[j]].y +
                                 norm(2) * laserCloudFeatureFromMapDS->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
                            planeValid = false;
                            break;
                        }
                        if (laserCloudFeatureFromMapDS->points[pointSearchInd[j]].label != pointSel.label) {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (planeValid) {
                        ceres::CostFunction* cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        problem.AddResidualBlock(cost_function, loss_function, pose_parameters, pose_parameters + 4);
                        surf_num++;
                    }
                    else{
                        // pointOri.x = 0;
                        // pointOri.y = 0;
                        // pointOri.z = 0;
                        // pointOri.label = 0;
                    }
                }
                else{
                    // pointOri.x = 0;
                    // pointOri.y = 0;
                    // pointOri.z = 0;
                    // pointOri.label = 0;
                }
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            options.num_threads = 4;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }
    }

    // 2.4 update pose
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
    poseAfterMapped.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(q_w_curr);
    poseAfterMapped.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(t_w_curr);
    auto mapMatch_end = std::chrono::steady_clock::now();
    // printf("\033[32m map match :%5.2fms.\033[0m\n", chrono::duration<double>(mapMatch_end - mapMatch_start).count() * 1000);

    // 3 save key frames and factor
    if (cloudKeyPoses3D->points.empty()) {
        gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(poseAfterMapped.matrix()), priorNoise));
        initialEstimate.insert(0, Pose3(poseAfterMapped.matrix()));
    } else {
        gtsam::Pose3 poseFrom = Pose3(keyPoses6D.back().matrix());
        gtsam::Pose3 poseTo = Pose3(poseAfterMapped.matrix());
        gtSAMgraph.add(
            BetweenFactor<Pose3>(cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(poseAfterMapped.matrix()));
    }
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();
    Pose3 latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
    pcl::PointXYZL thisPose3D;
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.label = cloudKeyPoses3D->points.size();
    cloudKeyPoses3D->push_back(thisPose3D);

    Eigen::Isometry3d thisPose6D;
    thisPose6D.setIdentity();
    thisPose6D.matrix() = latestEstimate.matrix();
    keyPoses6D.push_back(thisPose6D);
    keyPoses6DTime.push_back(timeLaserInfoCur);

    FeatureCloudKeyFrames.push_back(laserFeatureCloud);
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
        downSave.setInputCloud(laserOriCloud);
        downSave.filter(*cloud_temp);
        OriCloudKeyFrames.push_back(cloud_temp);
    }

    // 4 loop detection
    auto loopDetection_start = std::chrono::steady_clock::now();
    int loopKeyPre = -1;
    int loopKeyCur = cloudKeyPoses3D->points.size() - 1;
    if (cloudKeyPoses3D->points.size() != 0) {
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchDisLoop;
        loopKdTree->setInputCloud(cloudKeyPoses3D);
        loopKdTree->radiusSearch(cloudKeyPoses3D->points.back(), 20.0, pointSearchIndLoop, pointSearchDisLoop);

        loopKeyPre = -1;
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            if (abs(keyPoses6DTime[id] - timeLaserInfoCur) > 30.0) {
                loopKeyPre = id;
                break;
            }
        }
    }
    if (loopKeyPre != -1 && loopKeyCur != loopKeyPre) {
        // extract cloud
        pcl::PointCloud<pcl::PointXYZL>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZL>());
        pcl::PointCloud<pcl::PointXYZL>::Ptr prevKeyframeCloud(new pcl::PointCloud<pcl::PointXYZL>());
        {
            *cureKeyframeCloud += *transformPointCloud(laserOriCloud, keyPoses6D.back().matrix());
            pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
            loopFilter.setInputCloud(cureKeyframeCloud);
            loopFilter.filter(*cloud_temp);
            *cureKeyframeCloud = *cloud_temp;
            int cloudSize = keyPoses6D.size();
            for (int i = -25; i <= 25; ++i) {
                int keyNear = loopKeyPre + i;
                if (keyNear < 0 || keyNear >= cloudSize)
                    continue;
                *prevKeyframeCloud += *transformPointCloud(OriCloudKeyFrames[keyNear], keyPoses6D[keyNear].matrix());
            }
            if (prevKeyframeCloud->empty() != true) {
                pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
                loopFilter.setInputCloud(prevKeyframeCloud);
                loopFilter.filter(*cloud_temp);
                *prevKeyframeCloud = *cloud_temp;
            }
        }
        // ICP Settings
        static pcl::IterativeClosestPoint<pcl::PointXYZL, pcl::PointXYZL> icp;
        icp.setMaxCorrespondenceDistance(20.0 * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<pcl::PointXYZL>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZL>());
        icp.align(*unused_result);

        if (icp.hasConverged() && icp.getFitnessScore() < 0.3) {
            loopCur = cureKeyframeCloud;
            loopOld = prevKeyframeCloud;
            Eigen::Isometry3d correctionLidarFrame(icp.getFinalTransformation().cast<double>().matrix());
            Eigen::Isometry3d tWrong(keyPoses6D[loopKeyCur].matrix());
            Eigen::Isometry3d tCorrect(correctionLidarFrame * tWrong);

            gtsam::Pose3 poseFrom = Pose3(tCorrect.matrix());
            gtsam::Pose3 poseTo = Pose3(keyPoses6D[loopKeyPre].matrix());
            gtsam::Vector Vector6(6);
            float noiseScore = icp.getFitnessScore();
            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
            constraintNoise = noiseModel::Diagonal::Variances(Vector6);
            gtSAMgraph.add(BetweenFactor<Pose3>(loopKeyCur, loopKeyPre, poseFrom.between(poseTo), constraintNoise));
            isam->update(gtSAMgraph);
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            gtSAMgraph.resize(0);
            aLoopIsClosed = true;

            loopIndexContainer[loopKeyCur] = loopKeyPre;
            // printf("\033[36m loop detection %d---%d, score:%f.\033[0m\n", loopKeyCur, loopKeyPre, icp.getFitnessScore());
        }
    }

    auto loopDetection_end = std::chrono::steady_clock::now();
    // printf("\033[32m loop detection :%5.2fms.\033[0m\n", chrono::duration<double>(loopDetection_end - loopDetection_start).count() * 1000);

    // 5 loop closure to correct  pose
    auto loopClosure_start = std::chrono::steady_clock::now();
    if (aLoopIsClosed) {
        laserCloudMapContainer.clear();
        Pose3 latestEstimate;
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isamCurrentEstimate = isam->calculateEstimate();
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            keyPoses6D[i] = Eigen::Isometry3d(isamCurrentEstimate.at<Pose3>(i).matrix());
        }
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

        poseAfterMapped = Eigen::Isometry3d(latestEstimate.matrix());

        q_w_curr = Eigen::Quaterniond(poseAfterMapped.rotation());
        t_w_curr = Eigen::Vector3d(poseAfterMapped.translation());

        poseLocalMatch = poseAfterMapped;

        q_wmap_wodom.setIdentity();
        t_wmap_wodom.setZero();

        aLoopIsClosed = false;
    }
    auto loopClosure_end = std::chrono::steady_clock::now();
    // printf("\033[32m loop closure :%5.2fms.\033[0m\n", chrono::duration<double>(loopClosure_end - loopClosure_start).count() * 1000);
    return true;
}

bool Odometry::good() { return FR.good(); }

pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::transformPointCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr incloud, Eigen::Matrix4d trans) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr ans(new pcl::PointCloud<pcl::PointXYZL>());
    int cloudSize = incloud->points.size();
    ans->resize(cloudSize);
    for (int i = 0; i < cloudSize; ++i) {
        auto& pointFrom = incloud->points[i];
        Eigen::Vector4d pointFromVec(pointFrom.x, pointFrom.y, pointFrom.z, 1);
        Eigen::Vector4d pointToVec = trans * pointFromVec;
        ans->points[i].label = pointFrom.label;
        ans->points[i].x = pointToVec[0];
        ans->points[i].y = pointToVec[1];
        ans->points[i].z = pointToVec[2];
    }
    return ans;
}

std::map<int, int> Odometry::getLoop() { return loopIndexContainer; }

std::vector<Eigen::Isometry3d> Odometry::getTraj() { return keyPoses6D; }

pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::getMap() { return laserCloudFeatureFromMapDS; }

pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::getCurrentCloud() { return transformPointCloud(laserOriCloud, poseAfterMapped.matrix()); }
pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::getCurrentFeatureCloud() { return transformPointCloud(laserFeatureCloud, poseAfterMapped.matrix()); }
Eigen::Isometry3d Odometry::getCurrentPose() { return poseAfterMapped; }

pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::getloopCur() { return loopCur; }
pcl::PointCloud<pcl::PointXYZL>::Ptr Odometry::getloopOld() { return loopOld; }
void Odometry::control(int min_, int max_){ FR.control(min_, max_);}
double Odometry::getCurrTime(){return timeLaserInfoCur;}
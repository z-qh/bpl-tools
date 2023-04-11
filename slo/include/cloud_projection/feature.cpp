#include "feature.h"

FeatureExtracter::FeatureExtracter() {
    N_SCAN = 32;
    Horizon_SCAN = 2048;
    groundScanInd = 15;
    ang_up = 22.5;
    ang_bottom = -22.5;
    ang_res_x = 360.0 / float(Horizon_SCAN);
    ang_res_y = (ang_up - ang_bottom) / float(N_SCAN - 1);
    sensorMinimumRange = 8.0;
    sensorMountAngle = 0.0;
    segmentAlphaX = ang_res_x / 180.0 * M_PI;
    segmentAlphaY = ang_res_y / 180.0 * M_PI;
    segmentTheta = 20.0 / 180.0 * M_PI; // decrese this value may improve accuracy
    edgeThreshold = 1.0;
    surfThreshold = 0.1;
    init();
    reset();
}

FeatureExtracter::~FeatureExtracter() {
    delete allPushedIndX;
    delete allPushedIndY;
    delete queueIndX;
    delete queueIndY;
    delete startRingIndex;
    delete endRingIndex;
    delete segmentedCloudGroundFlag;
    delete segmentedCloudColInd;
    delete segmentedCloudRange;
    delete cloudCurvature;
    delete cloudNeighborPicked;
}

void FeatureExtracter::init() {

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.label = 0;

    fullCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());
    segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    clusterMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    clusterCount = 1;
    allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    startRingIndex = new int32_t[N_SCAN];
    endRingIndex = new int32_t[N_SCAN];
    segmentedCloudGroundFlag = new bool[N_SCAN * Horizon_SCAN];
    segmentedCloudColInd = new uint32_t[N_SCAN * Horizon_SCAN];
    segmentedCloudRange = new float[N_SCAN * Horizon_SCAN];

    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
    cloudCurvature = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];

    FeaturePoints.reset(new pcl::PointCloud<pcl::PointXYZL>());
}

void FeatureExtracter::reset() {
    fullCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());
    fullCloud->points.resize(N_SCAN * Horizon_SCAN);
    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZL>());

    rangeMat.setTo(FLT_MAX);
    groundMat.setTo(0);
    clusterMat.setTo(0);
    clusterCount = 1;

    std::memset(allPushedIndX, 0, N_SCAN * Horizon_SCAN * sizeof(uint16_t));
    std::memset(allPushedIndY, 0, N_SCAN * Horizon_SCAN * sizeof(uint16_t));
    std::memset(queueIndX, 0, N_SCAN * Horizon_SCAN * sizeof(uint16_t));
    std::memset(queueIndY, 0, N_SCAN * Horizon_SCAN * sizeof(uint16_t));

    std::memset(startRingIndex, 0, N_SCAN * sizeof(int32_t));
    std::memset(endRingIndex, 0, N_SCAN * sizeof(int32_t));
    std::memset(segmentedCloudGroundFlag, false, N_SCAN * Horizon_SCAN * sizeof(bool));
    std::memset(segmentedCloudColInd, 0, N_SCAN * Horizon_SCAN * sizeof(uint32_t));
    std::memset(segmentedCloudRange, 0, N_SCAN * Horizon_SCAN * sizeof(float));

    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
    std::memset(cloudCurvature, 0, N_SCAN * Horizon_SCAN * sizeof(float));
    std::memset(cloudNeighborPicked, 0, N_SCAN * Horizon_SCAN * sizeof(int));

    FeaturePoints.reset(new pcl::PointCloud<pcl::PointXYZL>());
}

void FeatureExtracter::labelComponents(int row, int col) {
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    int fromLabel, thisLabel;
    bool lineCountFlag[N_SCAN] = {false};
    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;
    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    while (queueSize > 0) {
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        fromLabel = fullCloud->points[fromIndX * Horizon_SCAN + fromIndY].label;
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        clusterMat.at<int>(fromIndX, fromIndY) = clusterCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter) {
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            thisLabel = fullCloud->points[thisIndX * Horizon_SCAN + thisIndY].label;
            //  index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (clusterMat.at<int>(thisIndX, thisIndY) != 0)
                continue;
            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;
            angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));
            if (angle > segmentTheta && thisLabel == fromLabel) {
                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;
                clusterMat.at<int>(thisIndX, thisIndY) = clusterCount;
                lineCountFlag[thisIndX] = true;
                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }
    // check if this segment is valid size > 30
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30) {
        feasibleSegment = true;
    } else if (allPushedIndSize >= 5 && (fromLabel == 17 || fromLabel == 19 || fromLabel == 20)) {
        feasibleSegment = true;
    }
    if (fromLabel == 0)
        feasibleSegment = false;
    // segment is valid, mark these points
    if (feasibleSegment == true) {
        ++clusterCount;
    } else { // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i) {
            clusterMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

pcl::PointCloud<pcl::PointXYZL>::Ptr FeatureExtracter::get(pcl::PointCloud<pcl::PointXYZL>::Ptr inCloud, int ground_pick_, int surf_pick_, int corner_pick_) {
    reset();
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSizeIn;
    cloudSizeIn = inCloud->points.size();
    for (size_t i = 0; i < cloudSizeIn; ++i) {
        pcl::PointXYZL thisPoint = inCloud->points[i];
        // find the row and column index in the iamge for this point
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + abs(ang_bottom)) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;
        rangeMat.at<float>(rowIdn, columnIdn) = range;
        index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
    // groundMat
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
        for (size_t i = 0; i < groundScanInd; ++i) {
            lowerInd = j + (i)*Horizon_SCAN;
            upperInd = j + (i + 1) * Horizon_SCAN;
            // no img point or not ground label
            if (fullCloud->points[lowerInd].label == 0 || fullCloud->points[upperInd].label == 0 || fullCloud->points[lowerInd].label != 1) {
                // no info to check, invalid points
                groundMat.at<int8_t>(i, j) = -1;
                continue;
            }
            // 计算两线相邻点之间组成向量的垂直角度，小于10就认为是地面点
            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10) {
                groundMat.at<int8_t>(i, j) = 1;
                groundMat.at<int8_t>(i + 1, j) = 1;
                fullCloud->points[upperInd].label = 1;
            }
        }
    }
    for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX) {
                clusterMat.at<int>(i, j) = -1;
            }
        }
    }
    // extract segmented cloud
    for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (clusterMat.at<int>(i, j) == 0) {
                labelComponents(i, j);
            }
        }
    }
    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < N_SCAN; ++i) {
        startRingIndex[i] = sizeOfSegCloud - 1 + 5;

        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (clusterMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
                // majority of ground points are skipped
                if (groundMat.at<int8_t>(i, j) == 1) {
                    if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
                        continue;
                }
                // mark ground points so they will not be considered as edge features
                // later
                segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
                // mark the points' column index for marking occlusion later
                segmentedCloudColInd[sizeOfSegCloud] = j;
                // save range info
                segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
                // save seg cloud
                segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                // size of seg cloud
                ++sizeOfSegCloud;
            }
        }
        endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    }
    // calculate Smoothness
    int cloudSizeS = segmentedCloud->points.size();
    for (int i = 5; i < cloudSizeS - 5; i++) {
        float diffRange = segmentedCloudRange[i - 5] + segmentedCloudRange[i - 4] + segmentedCloudRange[i - 3] + segmentedCloudRange[i - 2] +
                          segmentedCloudRange[i - 1] - segmentedCloudRange[i] * 10 + segmentedCloudRange[i + 1] + segmentedCloudRange[i + 2] +
                          segmentedCloudRange[i + 3] + segmentedCloudRange[i + 4] + segmentedCloudRange[i + 5];
        cloudCurvature[i] = diffRange * diffRange;
        cloudNeighborPicked[i] = 0;
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
    // mark Occluded point
    for (int i = 5; i < cloudSizeS - 6; ++i) {
        float depth1 = segmentedCloudRange[i];
        float depth2 = segmentedCloudRange[i + 1];
        int columnDiff = std::abs(int(segmentedCloudColInd[i + 1] - segmentedCloudColInd[i]));
        if (columnDiff < 10) {
            if (depth1 - depth2 > 0.3) {
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        float diff1 = std::abs(float(segmentedCloudRange[i - 1] - segmentedCloudRange[i]));
        float diff2 = std::abs(float(segmentedCloudRange[i + 1] - segmentedCloudRange[i]));
        if (diff1 > 0.02 * segmentedCloudRange[i] && diff2 > 0.02 * segmentedCloudRange[i]) {
            cloudNeighborPicked[i] = 1;
        }
    }
    // Extract plane points
    for (int i = 0; i < N_SCAN; i++) {
        for (int j = 0; j < 6; j++) {
            int sp = (startRingIndex[i] * (6 - j) + endRingIndex[i] * j) / 6;
            int ep = (startRingIndex[i] * (5 - j) + endRingIndex[i] * (j + 1)) / 6 - 1;
            if (sp >= ep)
                continue;
            std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());
            int smallestGroundPickNum = 0;
            int smallestNGroundPickNum = 0;
            int smallestPolePickNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness[k].ind;
                // 没遮挡，曲率小，非地面点
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold && segmentedCloudGroundFlag[ind] == false &&
                    segmentedCloud->points[ind].label != 0) {
                    if (smallestNGroundPickNum >= surf_pick_) {
                        break;
                    }
                    FeaturePoints->push_back(segmentedCloud->points[ind]);
                    smallestNGroundPickNum++;
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
                // 没遮挡，曲率小，地面点
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold && segmentedCloudGroundFlag[ind] == true &&
                    segmentedCloud->points[ind].label == 1) {
                    if (smallestGroundPickNum >= ground_pick_) {
                        break;
                    }
                    FeaturePoints->push_back(segmentedCloud->points[ind]);
                    smallestGroundPickNum++;
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
                // polelike，非地面点
                if (segmentedCloudGroundFlag[ind] == false &&
                    (segmentedCloud->points[ind].label == 17 || segmentedCloud->points[ind].label == 19 || segmentedCloud->points[ind].label == 20)) {
                    FeaturePoints->push_back(segmentedCloud->points[ind]);
                    smallestPolePickNum++;
                    cloudNeighborPicked[ind] = 1;
                }
            }
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold && segmentedCloud->points[ind].label != 0) {
                    if (largestPickedNum >= corner_pick_) {
                        break;
                    }
                    FeaturePoints->push_back(segmentedCloud->points[ind]);
                    largestPickedNum++;
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segmentedCloudColInd[ind + l] - segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
        }
    }
    return FeaturePoints;
}


//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <vector>
//#include <fstream>
//#include <string>
//#include <random>
//
//#define SCAN_NUMS 16
//#define SCAN_ANGLE 45
//
//using namespace std;

//void test01()
//{
//    string fileName = "/home/qh/Xpcd_test/bagPCD_1.pcd";
//    string savefile = "/home/qh/Xpcd_test/bagPCD_1_save.pcd";
//
//    pcl::PointCloud<pcl::PointXYZ> incloud;
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::io::loadPCDFile(fileName, incloud);
//
//    //转到右手系
//    for (int i = 0; i < incloud.size(); i++)
//    {
//        pcl::PointXYZ temp;
//        temp.x = incloud.points[i].x;
//        temp.y = incloud.points[i].y;
//        temp.z = incloud.points[i].z;
//        cloud.push_back(temp);
//    }
//
//    /*
//    * 划分有序点云
//    */
//    //起始角与终止角
//    double startAngle = atan2(cloud.points.front().y, cloud.points.front().x);
//    double endAngle = atan2(cloud.points.back().y, cloud.points.back().x);
//    if (endAngle - startAngle > 3 * M_PI)
//        endAngle -= 2 * M_PI;
//    else if (endAngle - startAngle < M_PI)
//        endAngle += 2 * M_PI;
//    //按照线号存放点云线扫，每一线扫点云有序
//    vector<pcl::PointCloud<pcl::PointXYZ>> scans(SCAN_NUMS);
//    for (int i = 0; i < cloud.size(); i++)
//    {
//        pcl::PointXYZ thisPoint = cloud.points[i];
//        double scanAngle = atan2(thisPoint.z, sqrt(pow(thisPoint.x, 2) + pow(thisPoint.y, 2))) * 180 / M_PI;
//        int roundAngle = int(scanAngle + (scanAngle < 0.0 ? -0.5 : 0.5));
//        int scanID = 0;
//        if (roundAngle > 0)
//            scanID = roundAngle;
//        else
//            scanID = roundAngle + SCAN_NUMS - 1;
//        scans[scanID].push_back(thisPoint);
//    }
//
//    //构建KD-Tree
//    pcl::PointCloud<pcl::PointXYZ>::Ptr kdCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    vector<int> searchIndex;
//    vector<float> searchDis;
//
//    //存放曲率与索引初始化
//    vector<vector<float>> curv(SCAN_NUMS);
//    //vector<vector<int>> curvIndex(SCAN_NUMS);
//    for (int i = 0; i < SCAN_NUMS; i++)
//    {
//        curv[i].resize(scans[i].size());
//        //curvIndex.resize(scans[i].size());
//    }
//    //按照顺序扫描，记录每个点的曲率和索引
//    for (int i = 0; i < SCAN_NUMS; i++)
//    {
//        if (scans[i].empty())
//            continue;
//        for (int j = 0; j < scans[i].size(); j++)
//        {
//            //每线首尾五个点不扫描
//            if (j < 5 || j >= scans[i].size() - 5)
//            {
//                curv[i][j] = -1;
//                //curvIndex[i][j] = j;
//                continue;
//            }
//            float diffX = scans[i].points[j - 5].x + scans[i].points[j - 4].x
//                          + scans[i].points[j - 3].x + scans[i].points[j - 2].x
//                          + scans[i].points[j - 1].x - 10 * scans[i].points[j].x
//                          + scans[i].points[j + 1].x + scans[i].points[j + 2].x
//                          + scans[i].points[j + 3].x + scans[i].points[j + 4].x
//                          + scans[i].points[j + 5].x;
//            float diffY = scans[i].points[j - 5].y + scans[i].points[j - 4].y
//                          + scans[i].points[j - 3].y + scans[i].points[j - 2].y
//                          + scans[i].points[j - 1].y - 10 * scans[i].points[j].y
//                          + scans[i].points[j + 1].y + scans[i].points[j + 2].y
//                          + scans[i].points[j + 3].y + scans[i].points[j + 4].y
//                          + scans[i].points[j + 5].y;
//            float diffZ = scans[i].points[j - 5].z + scans[i].points[j - 4].z
//                          + scans[i].points[j - 3].z + scans[i].points[j - 2].z
//                          + scans[i].points[j - 1].z - 10 * scans[i].points[j].z
//                          + scans[i].points[j + 1].z + scans[i].points[j + 2].z
//                          + scans[i].points[j + 3].z + scans[i].points[j + 4].z
//                          + scans[i].points[j + 5].z;
//            curv[i][j] = diffX * diffX + diffY * diffY + diffZ * diffZ;
//        }
//    }
//    //剔除中间断片，断层的
//    for (int i = 0; i < SCAN_NUMS; i++)
//    {
//        continue;
//        for (int j = 0; j < scans[i].size(); j++)
//        {
//            if (curv[i][j] == -1)
//                continue;
//            float diffX = scans[i].points[j + 1].x - scans[i].points[j].x;
//            float diffY = scans[i].points[j + 1].y - scans[i].points[j].y;
//            float diffZ = scans[i].points[j + 1].z - scans[i].points[j].z;
//            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
//            //如果两个点距离比较远，那么不是发生了断层，就是发生了断片
//            if (diff > 0.1)
//            {
//                float depth1 = sqrt(scans[i].points[j].x * scans[i].points[j].x +
//                                    scans[i].points[j].y * scans[i].points[j].y +
//                                    scans[i].points[j].z * scans[i].points[j].z);
//
//                float depth2 = sqrt(scans[i].points[j + 1].x * scans[i].points[j + 1].x +
//                                    scans[i].points[j + 1].y * scans[i].points[j + 1].y +
//                                    scans[i].points[j + 1].z * scans[i].points[j + 1].z);
//
//                if (depth1 > depth2)
//                {
//                    diffX = scans[i].points[j + 1].x - scans[i].points[j].x * depth2 / depth1;
//                    diffY = scans[i].points[j + 1].y - scans[i].points[j].y * depth2 / depth1;
//                    diffZ = scans[i].points[j + 1].z - scans[i].points[j].z * depth2 / depth1;
//
//                    //排除容易被斜面挡住的点
//                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
//                    {
//                        //该点及前面五个点（大致都在斜面上）全部置为筛选过
//                        curv[i][j - 5] = -1;
//                        curv[i][j - 4] = -1;
//                        curv[i][j - 3] = -1;
//                        curv[i][j - 2] = -1;
//                        curv[i][j - 1] = -1;
//                    }
//                }
//                else
//                {
//                    diffX = scans[i].points[j + 1].x * depth1 / depth2 - scans[i].points[j].x;
//                    diffY = scans[i].points[j + 1].y * depth1 / depth2 - scans[i].points[j].y;
//                    diffZ = scans[i].points[j + 1].z * depth1 / depth2 - scans[i].points[j].z;
//
//                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
//                    {
//                        curv[i][j + 5] = -1;
//                        curv[i][j + 4] = -1;
//                        curv[i][j + 3] = -1;
//                        curv[i][j + 2] = -1;
//                        curv[i][j + 1] = -1;
//                    }
//                }
//            }
//        }
//    }
//    //按照曲率上色，完事拉出去看看
//    pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
//    for (int i = 0; i < SCAN_NUMS; i++)
//    {
//        if (scans[i].size() == 0)
//            continue;
//        for (int j = 0; j < scans[i].size(); j++)
//        {
//            pcl::PointXYZRGB tempPoint;
//            tempPoint.x = scans[i][j].x;
//            tempPoint.y = scans[i][j].y;
//            tempPoint.z = scans[i][j].z;
//            if (curv[i][j] == -1)
//            {
//                tempPoint.r = 255;
//                tempPoint.g = 0;
//                tempPoint.b = 0;
//            }
//            else
//            {
//                tempPoint.r = ((curv[i][j] * 0.05) > 255 ? 255 : (curv[i][j] * 0.05));
//                tempPoint.g = ((curv[i][j] * 0.05) > 255 ? 255 : (curv[i][j] * 0.05));
//                tempPoint.b = ((curv[i][j] * 0.05) > 255 ? 255 : (curv[i][j] * 0.05));
//            }
//            colorCloud.push_back(tempPoint);
//        }
//    }
//    if (colorCloud.empty())
//        return ;
//    pcl::io::savePCDFileASCII(savefile, colorCloud);
//    cout << colorCloud.size() << endl;
//}
//
//////按照顺序扫描，线上一些曲率相似并且没得断片的点记录下来，
////for (int i = SCAN_NUMS - 1; i > 0; i--)
////{
////  if (scans[i].empty())
////    continue;
////  for (int j = 0; j < scans[i].size(); j++)
////  {
////
////  }
////}
//
//bool match(pcl::PointXYZ p1, pcl::PointXYZ p2)
//{
//    float diffX = p1.x - p2.x;
//    float diffY = p1.y - p2.y;
//    float diffZ = p1.z - p2.z;
//    float diss = sqrt(pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2));
//    float pitchAngle = atan2(diffZ, sqrt(pow(diffX, 2) + pow(diffY, 2))) * 180 / M_PI;
//    if( diss < 0.5 && abs(pitchAngle) < 20)
//        return true;
//    else
//        return false;
//}


//void test02()
//{
//    string fileName = "/home/qh/Xpcd_test/XTEST1.pcd";
//    string savefile = "/home/qh/Xpcd_test/XTEST1_save1.pcd";
//    string savefile2 = "/home/qh/Xpcd_test/XTEST1_save2.pcd";
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::io::loadPCDFile(fileName, cloud);
//    pcl::PointCloud<pcl::PointXYZRGB> Yes2;//显示辅助点云
//    pcl::PointCloud<pcl::PointXYZRGB> Yes;//上色辅助点云
//
//    //耗时统计
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//
//    //换个思路，按照水平角度来存储
//    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lines;
//    map<int, int> linesIndex;
//    int count = 0;
//    for (int i = 0; i < cloud.size(); i++)
//    {
//        pcl::PointXYZ tempPoint = cloud.points[i];
//        //角度
//        float angle = atan2(tempPoint.y, tempPoint.x) * 180 / M_PI + 180;
//        //角度的key，按照key存储，相当于临近水平角度的分在一起
//        int angleIndex = round(angle);
//        //查找已有的key
//        map<int, int>::iterator it = linesIndex.find(angleIndex);
//        //如果查到了，以值为索引存入数组
//        if (it != linesIndex.end())
//        {
//            if (lines[it->second] == nullptr)
//                lines[it->second].reset(new pcl::PointCloud<pcl::PointXYZ>());
//            lines[it->second]->push_back(tempPoint);
//        }
//        //如果没查到说明有新的key，给赋新的值
//        else
//        {
//            count++;
//            lines.resize(count);
//            lines[count - 1].reset(new pcl::PointCloud<pcl::PointXYZ>());
//            linesIndex.insert(make_pair(angleIndex, count - 1));
//        }
//    }
//
//    //忽略Z轴，计算到中心的距离，统计点之间到原点的方差，如果太大说明比较分散稀碎
//    vector<int> pick(lines.size(), 0);
//    vector<float> sigmaO(lines.size());//方差
//    vector<float> meansO(lines.size());//均值
//    vector<vector<int>> singlePick(lines.size());//单点选择数组
//    for (int i = 0; i < lines.size(); i++)
//    {
//        //去掉的列这里还没给出
//        if (lines[i]->empty())
//            continue;
//        float SO = 0;
//        float SO2 = 0;
//        //初始化
//        singlePick[i].resize(lines[i]->size());
//        //计算方差均值
//        for (int j = 0; j < lines[i]->size(); j++)
//        {
//            float dis = sqrt(pow(lines[i]->points[j].x, 2) + pow(lines[i]->points[j].y, 2));
//            SO += dis;
//            SO2 += dis * dis;
//        }
//        meansO[i] = SO / lines[i]->size();
//        sigmaO[i] = SO2 / lines[i]->size() - pow((SO / lines[i]->size()), 2);
//        //数量很少或者方差很大就去掉，这个方差该怎么确定呢
//        if (sigmaO[i] > 1 || lines[i]->size() < 30)
//        {
//            pick[i] = -1;
//        }
//        cout << i << " S2:" << sigmaO[i] << " size:" << lines[i]->size() << endl;
//    }
//
//    //统计去除离散点之前的中心
//    vector<pcl::PointXYZ> center(lines.size());
//    for(int i = 0; i < lines.size(); i++)
//    {
//        //已经去掉的列和空列就不计算了
//        if(lines[i]->empty() || pick[i] == -1)
//            continue;
//        float sumX = 0;
//        float sumY = 0;
//        float sumZ = 0;
//        //中心均值
//        for (int j = 0; j < lines[i]->size(); j++)
//        {
//            sumX += lines[i]->points[j].x;
//            sumY += lines[i]->points[j].y;
//            sumZ += lines[i]->points[j].z;
//        }
//        center[i].x = sumX / lines[i]->size();
//        center[i].y = sumY / lines[i]->size();
//        center[i].z = sumZ / lines[i]->size();
//    }
//    //统计点到Z中心的方差和均值，用于去除离散点
//    vector<float> meansZ(lines.size());
//    vector<float> sigmaZ(lines.size());
//    for(int i = 0; i < lines.size(); i++)
//    {
//        //已经去掉的列和空列就不计算了
//        if(lines[i]->empty() || pick[i] == -1)
//            continue;
//        float SC = 0;
//        float SC2 = 0;
//        //计算Z中心方差均值
//        for (int j = 0; j < lines[i]->size(); j++)
//        {
//            float diffZ = abs(lines[i]->points[j].z - center[i].z);
//            SC += diffZ;
//            SC2 += diffZ * diffZ;
//        }
//        meansZ[i] = SC / lines[i]->size();
//        sigmaZ[i] = SC2 / lines[i]->size() - pow((SC / lines[i]->size()), 2);
//
//        cout << i << " Y:" << sigmaO[i] << " size:" << lines[i]->size() << endl;
//    }
//    //去除相对原点的离散点，去除相对Z中心的离散点，并且纠正点云的中心点，
//    for(int i = 0; i < lines.size(); i++)
//    {
//        //已经去掉的列和空列就不计算了
//        if(lines[i]->empty() || pick[i] == -1)
//            continue;
//        float sumX = 0;
//        float sumY = 0;
//        float sumZ = 0;
//        int outNums = 0;
//        //纠正
//        for (int j = 0; j < lines[i]->size(); j++)
//        {
//            float disO = sqrt(pow(lines[i]->points[j].x, 2) + pow(lines[i]->points[j].y, 2));
//            float disZ = abs(lines[i]->points[j].z - center[i].z);
//            //去除相对原点的离散点，去除相对Z中心的离散点
//            if(disO < (meansO[i] - 2.0 * sqrt(sigmaO[i]))
//            || disO > (meansO[i] + 2.0 * sqrt(sigmaO[i]))
//            || disZ > (meansZ[i] + 1.0 * sqrt(sigmaZ[i]))
//            )
//            {
//                //cout << sigmaO[i] << endl;
//                outNums++;
//                singlePick[i][j] = -1;
//                continue;
//            }
//            sumX += lines[i]->points[j].x;
//            sumY += lines[i]->points[j].y;
//            sumZ += lines[i]->points[j].z;
//        }
//        center[i].x = sumX / (lines[i]->size() - outNums);
//        center[i].y = sumY / (lines[i]->size() - outNums);
//        center[i].z = sumZ / (lines[i]->size() - outNums);
//    }
//
////    for(int i = 0; i < center.size(); i++)
////    {
////        if(lines[i]->empty() || pick[i] == -1)
////            continue;
////        pcl::PointXYZRGB temp;
////        temp.x = center[i].x;
////        temp.y = center[i].y;
////        temp.z = center[i].z;
////        temp.r = 0;
////        temp.g = 255;
////        temp.b = 0;
////        Yes2.push_back(temp);
////    }
////    if(Yes2.empty())
////        return;
////    pcl::io::savePCDFile(savefile2, Yes2);
//
//    //遍历搜寻策略，利用中心点代表点云
//    //vector<int> used(center.size(), 0);//使用过的矩阵
//    vector<vector<int>> calssIndex(1);//分类存储，类数未知
//    int classNums = 0;
//    //第一个点开头，单独做一类，并开始
//    calssIndex[classNums].push_back(0);
//    for(int i = 0; i < center.size(); i++)
//    {
//        int j = (i + 1) % center.size();//后一个点
//        //匹配此点到后面的点，匹配成功放到一起
//        if(match(center[i], center[j]))
//        {
//            if(j != i+1)//回环
//            {
//                //回溯
//                for(int k = 0; k < calssIndex[classNums].size(); k++)
//                    calssIndex[classNums][k] = 0;
//                cout << "flash back, end!" << endl;
//                break;
//            }
//            calssIndex[classNums].push_back(j);
//            cout << "point: " << i << ", class: " << classNums << " add new，now: " << calssIndex[classNums].size() << endl;
//        }
//        //不成功则另起一类，把后一个点放进去
//        else
//        {
//            if(j != i+1)//回环
//            {
//                cout << "no flash, end!" << endl;
//                break;
//            }
//            classNums++;
//            calssIndex.resize(classNums+1);
//            calssIndex[classNums].push_back(j);
//            cout << "point: " << i  << ", new class, num: " << classNums << endl;
//        }
//    }
//
//    //耗时统计
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "handle time cost: " << (time_used.count() * 1000) << " ms." << endl;
//
//    //按照calss上色对中心点染色
//    int centerColor = 0;
//    for(int i = 0; i < calssIndex.size(); i++)
//    {
//        centerColor = i % 2;
//        if(calssIndex[i].size() <=2 )
//            continue;
//        for(int j = 0; j < calssIndex[i].size(); j++)
//        {
//            pcl::PointXYZRGB temp;
//            temp.x = center[calssIndex[i][j]].x;
//            temp.y = center[calssIndex[i][j]].y;
//            temp.z = center[calssIndex[i][j]].z;
//            //temp.z = 1.0;
//            temp.r = centerColor * 255;
//            temp.g = centerColor * 255;
//            temp.b = centerColor * 255;
//            Yes2.push_back(temp);
//        }
//    }
//    //看看分布
//    if(Yes2.empty())
//        return;
//    pcl::io::savePCDFile(savefile2, Yes2);
//    //cout << Yes2.size() << endl;
//
//    //上色看效果
//    int normalColor = 1;
//    for (int i = 0; i < lines.size(); i++)
//    {
//        normalColor = i % 2;
//        if (lines[i] == nullptr || lines[i]->empty())
//            continue;
//        //去除的列，数量太少或者方差过大
//        if(pick[i] == -1)
//            continue;
//        //染色
//        for (int j = 0; j < lines[i]->points.size(); j++)
//        {
//            pcl::PointXYZRGB temp;
//            temp.x = lines[i]->points[j].x;
//            temp.y = lines[i]->points[j].y;
//            temp.z = lines[i]->points[j].z;
//            //要去除的列染色
//            if (pick[i] == -1)
//            {
//                temp.r = 0;
//                temp.g = 255;
//                temp.b = 0;
//            }
//            //去除的离群点染色
//            else if(singlePick[i][j] == -1)
//            {
//                //continue;
//                temp.r = 255;
//                temp.g = 0;
//                temp.b = 0;
//            }
//            //正常列染色
//            else
//            {
//                //normalColor = 1;
//                temp.r = normalColor * 255;
//                temp.g = normalColor * 255;
//                temp.b = normalColor * 255;
//            }
//            Yes.push_back(temp);
//        }
//    }
//    if (Yes.empty())
//        return;
//    pcl::io::savePCDFile(savefile, Yes);
//    //cout << Yes.size() << endl;
//
//    //找到最多的一列
//    //int max = 0;
//    //int maxIndex = -1;
//    //for(int i = 0; i < lines.size(); i++)
//    //{
//    //  if (lines[i] == nullptr || lines[i]->empty())
//    //    continue;
//    //  if (lines[i]->size() > max)
//    //  {
//    //    maxIndex = i;
//    //    max = lines[i]->size();
//    //  }
//    //}
//    //if (maxIndex == -1)
//    //  return;
//    //pcl::io::savePCDFile(savefile, *lines[maxIndex]);
//    //cout << max << endl;
//
//    //
//
//
//
//}
//

//#include "fstream"
//#include <pcl/surface/convex_hull.h>
//#include <pcl/visualization/pcl_visualizer.h>

//pcl::PointCloud<pcl::PointXYZ> createLineCLoud(pcl::PointXYZ A, pcl::PointXYZ B)
//{
//    pcl::PointCloud<pcl::PointXYZ> result;
//    result.clear();
//    double diffX = A.x - B.x;
//    double diffY = A.y - B.y;
//    double diffZ = A.z - B.z;
//    double distance = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
//    double nums = distance * 100;
//    for(int i = 0; i < nums; i++)
//    {
//        pcl::PointXYZ tempPoint;
//        tempPoint.x = B.x + diffX / nums * i;
//        tempPoint.y = B.y + diffY / nums * i;
//        tempPoint.z = B.z + diffZ / nums * i;
//        result.push_back(tempPoint);
//    }
//    return result;
//}

//void test03()
//{
//    string filename = "/home/qh/input.txt";
//    string savefilename = "/home/qh/result.pcd";
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr result;
//
//    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
//    result.reset(new pcl::PointCloud<pcl::PointXYZ>());
//
//    ifstream ifile;
//    ifile.open(filename);
//    int num  = 0;
//    ifile >> num;
//    for(int i = 0; i < num; i++)
//    {
//        pcl::PointXYZ temp;
//        ifile >> temp.x >> temp.y >> temp.z;
//        cloud->push_back(temp);
//    }
//
//    pcl::ConvexHull<pcl::PointXYZ> hull;
//    hull.setInputCloud(cloud);
//    hull.setDimension(3);
//    hull.setComputeAreaVolume(true);
//    vector<pcl::Vertices> poly;
//    hull.reconstruct(*result, poly);
//
//    if(result->empty())
//        return;
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
//    viewer->setBackgroundColor(255,255,255);
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 0);
//    viewer->addPointCloud(cloud, color_handler, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(result, 255, 0, 0);
//    viewer->addPointCloud(result, color_handlerK, "point");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");
//
//    viewer->addPolygon<pcl::PointXYZ>(result, 0, 0, 255, "polyline");
//
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//    }
//}


//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>

//void test04()
//{
//    string filename = "/home/qh/input2.txt";
//    string savefilename = "/home/qh/result.ply";
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr result;
//
//    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
//    result.reset(new pcl::PointCloud<pcl::PointXYZ>());
//
//    ifstream ifile;
//    ifile.open(filename);
//    int num  = 0;
//    ifile >> num;
//    for(int i = 0; i < num; i++)
//    {
//        pcl::PointXYZ temp;
//        ifile >> temp.x >> temp.y >> temp.z;
//        cloud->push_back(temp);
//    }
//
//    //法线估计对象
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    //存储估计的法线
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    //定义kd树指针
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud(cloud);
//    n.setInputCloud(cloud);
//    n.setSearchMethod(tree);
//    n.setKSearch(20);
//    //估计法线存储到其中
//    n.compute(*normals);//Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_width_normals(new pcl::PointCloud<pcl::PointNormal>);
//    //链接字段
//    pcl::concatenateFields(*cloud, *normals, *cloud_width_normals);
//    //定义搜索树对象
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//    //点云构建搜索树
//    tree2->setInputCloud(cloud_width_normals);
//    //定义三角化对象
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    //存储最终三角化的网络模型
//    pcl::PolygonMesh triangles;//设置连接点之间的最大距离，（即是三角形最大边长）
//    gp3.setSearchRadius(200.0f);
//    //设置各种参数值
//    gp3.setMu(2.5f);
//    gp3.setMaximumNearestNeighbors(100);
//    gp3.setMaximumSurfaceAngle(M_PI_4);
//    gp3.setMinimumAngle(M_PI / 18);
//    gp3.setMaximumAngle(2 * M_PI / 3);
//    gp3.setNormalConsistency(false);
//    //设置搜索方法和输入点云
//    gp3.setInputCloud(cloud_width_normals);
//    gp3.setSearchMethod(tree2);
//    //执行重构，结果保存在triangles中
//    gp3.reconstruct(triangles);
//    pcl::io::savePLYFileBinary(savefilename, triangles);
//
//    // 显示结果图
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MAP3D MESH"));
//    ////设置背景;
//    viewer->setBackgroundColor(0, 0, 0);
//    //设置显示的网格
//    viewer->addPolygonMesh(triangles, "my");
//    //viewer->initCameraParameters();
//    while (!viewer->wasStopped()) {
//        viewer->spin();
//    }
//    std::cout << "success" << std::endl;
//
//
//}


//void testTimeKdtree()
//{
//    string file = "/home/qh/bagPCD_2.pcd";
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::io::loadPCDFile(file, *cloud);
//    for(int i = 0; i < 3; i++)
//    {
//        *cloud += *cloud;
//    }
//    cout << cloud->size() << endl;
//
//
//    //耗时统计
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//
//    kdtree.setInputCloud(cloud);
//
//    pcl::PointXYZ searchPoint;
//    searchPoint.x = 0;
//    searchPoint.y = 0;
//    searchPoint.z = 0;
//    int K = 10;
//    std::vector<int> pointIdxNKNSearch(K);
//    std::vector<float> pointNKNSquaredDistance(K);
//
//    //kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//
//    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//    {
//        std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
//                  << " " << cloud->points[ pointIdxNKNSearch[i] ].y
//                  << " " << cloud->points[ pointIdxNKNSearch[i] ].z
//                  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
//    }
//
//
//
//    //耗时统计
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "kdtree time cost: " << (time_used.count() * 1000) << " ms." << endl;
//}
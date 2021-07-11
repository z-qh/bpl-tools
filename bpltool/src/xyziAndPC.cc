/*
 * 将自定义的XYZI信息转成PCD用于其它软件查看
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <algorithm>

#include "pcl_ros/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

using namespace std;

struct XYZIFileType {
    float x;
    float y;
    float z;
    float i;
};

void convertPcdToXYZIFile(string inFile, string outFile);
void convertXYZIFileToPcd(string inFileName, string outFileName);

string mode;
string xyziPath;
string pcdPath;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xyziAndPC");
    ros::NodeHandle nh("~");

    nh.getParam("mode", mode);
    transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    nh.getParam("pcdPath", pcdPath);
    nh.getParam("xyziPath", xyziPath);

    if(mode.find("topcd") != -1)
    {
        convertXYZIFileToPcd(xyziPath, pcdPath);
    }
    else if(mode.find("toxyzi") != -1)
    {
        convertPcdToXYZIFile(pcdPath, xyziPath);
    }
    else
        cout << "invalid param" << endl;

    return 0;
}

/*
 * 获取XYZI包中的点云数量
 */
int getXYZIFileData(vector< XYZIFileType>& cloud, string fileName)
{
    cloud.clear();
    ifstream ifile;
    ifile.open(fileName, ios::binary);
    //统计文件大小
    ifile.seekg(0, ios::end);
    int fileSize = ifile.tellg() / 1024 / 1024;
    int fileStep = fileSize / 200;
    int readCount = 0;
    float temp[4];
    //读点云数据
    ifile.seekg(0);
    while (ifile.peek() != EOF)
    {
        ifile.read((char*)temp, sizeof(temp));
        if (readCount < fileStep)
        {
            readCount++;
            continue;
        }
        readCount = 0;
        XYZIFileType point;
        point.x = temp[0];
        point.y = temp[1];
        point.z = temp[2];
        point.i = temp[3];
        cloud.push_back(point);
    }
    ifile.close();
    return cloud.size();
}

/*
 * 将XYZI转为PCD
 */
void convertXYZIFileToPcd(string inFileName, string outFileName)
{
    vector<XYZIFileType> data;
    int nums =  getXYZIFileData(data, inFileName);

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = nums;
    cloud.height = 1;
    cloud.is_dense = true;
    for (int i = 0; i < data.size(); i++)
    {
        pcl::PointXYZI tempPoint;
        tempPoint.x = data[i].x;
        tempPoint.y = data[i].y;
        tempPoint.z = data[i].z;
        tempPoint.intensity = data[i].i;
        cloud.points.push_back(tempPoint);
    }
    pcl::io::savePCDFileASCII(outFileName, cloud);
}

/*
 * 将PCD转为XYZI包
 */
void convertPcdToXYZIFile(string inFile, string outFile)
{
    cout << "Yes" << endl;
}



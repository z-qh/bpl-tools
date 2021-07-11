/*
 * 将自定义的XYZI信息转成PCD用于其它软件查看
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

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

int getXYZIFileData(vector< XYZIFileType>& cloud, string fileName = "C:/Users/10399/Desktop/points.xyzi")
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

void convertXYZIFileToPcd(string outFileName = "C:/Users/10399/Desktop/points.pcd", string intFileName = "C:/Users/10399/Desktop/points.xyzi")
{
    vector<XYZIFileType> data;
    int nums =  getXYZIFileData(data, intFileName);

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


int main()
{
    convertXYZIFileToPcd();
    return 0;
}

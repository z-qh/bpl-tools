#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include "../semantic_icp/semantic_point_cloud.h"
#include "algorithm"
#include "filesystem"
#include "fstream"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "string"
#include "vector"

using namespace std;

class FileReader {
public:
    FileReader(string bin_path_, string label_path_, bool intensity_ = false);
    void reset();
    pcl::PointCloud<pcl::PointXYZL>::Ptr get();
    double getTime();
    bool good();
    void control(int min_=-1, int max_=-1);
public:
    void Debug_setPercent(float percent);

private:
    filesystem::path bin_path;
    filesystem::path label_path;
    vector<string> bin_files;
    vector<string> label_files;
    uint32_t output_count = 0;
    bool intensity = false;
    int control_min=-1;
    int control_max=-1;

public:
    static bool BIN_WITH_INTENSITY;
    static bool BIN_NO_INTENSITY;
    static map<int, int> labelMap;

public:
    static semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>::Ptr toS(pcl::PointCloud<pcl::PointXYZL>::Ptr pclCloud);
};

#endif
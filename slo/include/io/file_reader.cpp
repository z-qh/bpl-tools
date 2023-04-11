#include "file_reader.h"
using namespace std;

bool FileReader::BIN_WITH_INTENSITY = true;
bool FileReader::BIN_NO_INTENSITY = false;

map<int, int> FileReader::labelMap = {
    {00, 0},  //"unlabeled"
    {01, 0},  //"outlier"
    {10, 0},  //"car"
    {11, 0},  //"bicycle"
    {13, 0},  //"bus"
    {15, 0},  //"motorcycle"
    {16, 0},  //"on-rails"
    {18, 0},  //"truck"
    {20, 0},  //"other-vehicle"
    {30, 0},  //"person"
    {31, 0},  //"bicyclist"
    {32, 0},  //"motorcyclist"
    {40, 1},  //"road"
    {44, 1},  //"parking"
    {48, 1},  //"sidewalk"
    {49, 1},  //"other-ground"
    {50, 12}, //"building"
    {51, 13}, //"fence"
    {52, 14}, //"other-structure"
    {60, 15}, //"lane-marking"
    {70, 16}, //"vegetation"
    {71, 17}, //"trunk"
    {72, 18}, //"terrain"
    {80, 19}, //"pole"
    {81, 20}, //"traffic-sign"
    {99, 21}, //"other-object"
    {252, 0}, //"moving-car"
    {253, 0}, //"moving-bicyclist"
    {254, 0}, //"moving-person"
    {255, 0}, //"moving-motorcyclist"
    {256, 0}, //"moving-on-rails"
    {257, 0}, //"moving-bus"
    {258, 0}, //"moving-truck"
    {259, 0}, //"moving-other-vehicle"
};

FileReader::FileReader(string bin_path_, string label_path_, bool intensity_)
    : bin_path(bin_path_)
    , label_path(label_path_)
    , intensity(intensity_) {
    static auto getFileName = [](string& path_) -> string {
        string s = filesystem::path(path_).filename().string();
        return s.substr(0, s.find_last_of('.'));
    };
    if (!filesystem::exists(bin_path) || !filesystem::exists(label_path)) {
        printf("\033[31m The file directory does not exist!\033[0m\n");
        exit(0);
    }
    for (auto& p : filesystem::directory_iterator(bin_path)) {
        if (p.path().extension() == ".bin") {
            bin_files.emplace_back(p.path().string());
        }
    }
    for (auto& p : filesystem::directory_iterator(label_path)) {
        if (p.path().extension() == ".label") {
            label_files.emplace_back(p.path().string());
        }
    }
    if (bin_files.size() != label_files.size()) {
        printf("\033[31m The number of file .bin and .label is inconsistent!\033[0m\n");
        exit(0);
    }
    sort(bin_files.begin(), bin_files.end());
    sort(label_files.begin(), label_files.end());
    for (auto i = 0; i < bin_files.size(); ++i) {
        auto bin = getFileName(bin_files[i]);
        auto label = getFileName(label_files[i]);
        if (bin != label) {
            printf("\033[31m File name mismatch!\033[0m\n");
            exit(0);
        }
    }
    printf("\033[32m Load %d bins and labels!\033[0m\n", label_files.size());
}
void FileReader::reset() { output_count = 0; }
pcl::PointCloud<pcl::PointXYZL>::Ptr FileReader::get() {
    int pointChannleNum = 3;
    if (intensity)
        pointChannleNum = 4;
    if (output_count >= bin_files.size()) {
        return nullptr;
    }
    if(control_min != -1 || control_max != -1){
        if(output_count < control_min || output_count >= control_max){
            return nullptr;
        }
    }
    pcl::PointCloud<pcl::PointXYZL>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZL>());
    string bin = bin_files[output_count];
    string label = label_files[output_count];
    output_count++;
    ifstream binf(bin, ios::in | ios::binary), labelf(label, ios::in | ios::binary);
    binf.seekg(0, ios::end);
    labelf.seekg(0, ios::end);
    int pointNums = binf.tellg() / sizeof(float) / pointChannleNum;
    int labelNums = labelf.tellg() / sizeof(int32_t);
    if (pointNums != labelNums) {
        printf("\033[31m %s label matching error!\033[0m\n", bin.c_str());
        binf.close();
        labelf.close();
        exit(0);
    }
    binf.seekg(0, ios::beg);
    labelf.seekg(0, ios::beg);
    outCloud->resize(pointNums);
    for (int i = 0; i < pointNums; ++i) {
        auto& point = outCloud->points[i];
        binf.read((char*)&point.x, pointChannleNum * sizeof(float));
        labelf.read((char*)&point.label, sizeof(uint32_t));
        point.label = labelMap[point.label & 0xffff];
    }
    binf.close();
    labelf.close();
    return outCloud;
}
semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>::Ptr FileReader::toS(pcl::PointCloud<pcl::PointXYZL>::Ptr pclCloud) {
    typedef pcl::PointCloud<pcl::PointXYZL> InCloud;
    typedef pcl::PointCloud<pcl::PointXYZ> OutCloud;
    typedef semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> SPC;
    if (pclCloud == nullptr)
        return nullptr;
    std::vector<uint32_t> labels;
    std::map<uint32_t, OutCloud::Ptr> map;
    SPC::Ptr semanticCloud(new SPC());
    for (auto& p : pclCloud->points) {
        if (FileReader::labelMap[p.label] == 0)
            continue;
        if (map.find(p.label) == map.end()) {
            OutCloud::Ptr cloud(new OutCloud());
            cloud->emplace_back(p.x, p.y, p.z);
            map[p.label] = cloud;
            labels.emplace_back(p.label);
        } else {
            OutCloud::Ptr cloud = map[p.label];
            cloud->emplace_back(p.x, p.y, p.z);
        }
    }
    for (uint32_t l : labels) {
        OutCloud::Ptr cloud = map[l];
        semanticCloud->addSemanticCloud(l, cloud);
    }
    return semanticCloud;
}
bool FileReader::good() { 
    if (output_count >= bin_files.size()){
        return false;
    }
    if(control_min != -1 || control_max != -1){
        if(output_count >= control_max) return false;
        if(output_count < control_min) return false;
    }
    return true;
}

void FileReader::Debug_setPercent(float percent) { output_count = std::round(percent * bin_files.size()); }

double FileReader::getTime() { return output_count * 0.1; }

void FileReader::control(int min_, int max_) {
    control_min = min_;
    control_max = max_;
    if(control_min >= 0 && control_max <= bin_files.size()){
        output_count = control_min;
    }else{
        printf("control error!\n");
        exit(0);
    }
}
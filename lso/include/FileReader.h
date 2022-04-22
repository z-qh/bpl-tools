#ifndef FILE_READER_H_
#define FILE_READER_H_

#include "filesystem"

#include "iostream"
#include "string"
#include "sstream"

#include "vector"
#include "algorithm"

#include "LidarFrame.h"
#include "Geometry.h"
#include "data_type.h"
#include "Parameter.h"

/*
 * FileReader
 */
class FileReader{
public:
    FileReader() = delete;
    std::vector<std::string> BinFileList,LabelFileList;
    int FileCount,FileSize;
    FileReader(const Parameter& param){
        BinFileList = getDirectoryList(param.DataBinPath);
        LabelFileList = getDirectoryList(param.DataLabelPath);
        FileCount = 0;
        FileSize = BinFileList.size();
        std::cout << "FileReader.h-L31:" << std::endl;
        std::cout << BinFileList.front() << " - " << BinFileList.back() << std::endl;
        std::cout << LabelFileList.front() << " - " << BinFileList.back() << std::endl;
    }
public:
    bool isGood(){
        if(!BinFileList.empty() && !LabelFileList.empty()
           && LabelFileList.size() == BinFileList.size()
           && FileCount < FileSize){
            return true;
        }else{
            return false;
        }
    }
    std::vector<std::string> getDirectoryList(const std::string& directory){
        std::vector<std::string> list;
        typedef std::filesystem::directory_iterator diter;
        for(diter it = diter(directory); it != diter(); ++it){
            list.push_back(it->path().string());
        }
        sort(list.begin(), list.end());
        return list;
    }
    semanticCloudPtr getNextData(){
        return getBinAndLabel(FileCount++);
    }
    semanticCloudPtr getBinAndLabel(int FileId){
        semanticCloudPtr points(new semanticCloud());
        ifstream labelFile,binFile;
        binFile.open(BinFileList[FileId], std::ios::binary);
        labelFile.open(LabelFileList[FileId], std::ios::binary);
        int binFileSize, labelFileSize;
        binFile.seekg(0, std::ios::end);
        labelFile.seekg(0, std::ios::end);
        binFileSize = binFile.tellg() / sizeof(float) / 4;
        labelFileSize = labelFile.tellg() / sizeof(int32_t);
        if(binFileSize != labelFileSize) return points;
        for(int i = 0; i < binFileSize; ++i){
            PointSemantic point;
            binFile.read((char*)(&point.x), 3 * sizeof(float));
            binFile.read((char*)(&point.intensity), 1 * sizeof(float));
            labelFile.read((char*)(&point.label), 1 * sizeof(int32_t));
            points->push_back(point);
        }
        return points;
    }
    bool end(){
        return FileCount>=FileSize;
    }

public:
    using Ptr = std::shared_ptr<FileReader>;
    using ConstPtr = std::shared_ptr<const FileReader>;
};

#endif
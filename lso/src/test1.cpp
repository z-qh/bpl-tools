#include "ClusterCVC.h"
#include "Geometry.h"
#include "LidarFrame.h"
#include "GlobalMap.h"
#include "LocalMap.h"
#include "LidarOdometry.h"
#include "Parameter.h"
#include "FileReader.h"

#include "iostream"

using namespace std;

int main(){
    Parameter::Ptr globalParameter(new Parameter());
    FileReader::Ptr globalFileReader(new FileReader(*globalParameter));
    LidarOdometry::Ptr globalLidarOdometry(new LidarOdometry(*globalParameter));
    LocalMap::Ptr localMap(new LocalMap());
    cout << sizeof(globalParameter) << endl;
    cout << sizeof(globalFileReader) << endl;
    for(; globalFileReader->isGood();){
        if(globalFileReader->FileCount > 100) break;
        LidarFrame::Ptr nowFrame(new LidarFrame(globalFileReader->getNextData(), 0));
        globalLidarOdometry->setNewInputLidarFrame(nowFrame);
        localMap->insert(nowFrame);
        while(localMap->KeyFrameCount > 10){
            localMap->removeOld();
        }
    }

    return 0;
}
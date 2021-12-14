#ifndef LSO_MAP_H_
#define LSO_MAP_H_

#include "Base.h"
#include "SemanticMark.h"
#include "Frame.h"


namespace lso{
    class Map{
    public:
        std::size_t GeoSize = 0;
        std::size_t MarkSize = 0;
        int MarkNum = 0;
        int KeyFrameNum = 0;
        std::set<lso::SemanticMark::Ptr> Marks;
        std::deque<std::pair<KeyFrame::Ptr, int>> KeyFrames;
        float downsample_resolution = 1.0;
        pcl::ApproximateVoxelGrid<PointType> GeometryCloudFilter;
        std::deque<Pose> Path;

    public:
        Map(){
            GeometryCloudFilter.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);// Init the DownSampleFilter
        }
        //
        void GenerateSemanticMapCloudMsg(sensor_msgs::PointCloud2& msg){
            semanticCloudPtr Cloud(new semanticCloud());
            for(const lso::SemanticMark::Ptr& mark : Marks){
                semanticCloudPtr tmp(new semanticCloud());
                pcl::transformPointCloud(*(mark->mCloud), *tmp, mark->mPose.Posture.matrix());
                *Cloud += *tmp;
            }
            pcl::toROSMsg(*Cloud, msg);
        }

        // Add Frame and SMarks to Map
        void PushFrame(lso::KeyFrame::Ptr &tKF){
            KeyFrames.emplace_back(tKF, tKF->Ind);
            Path.push_back(tKF->mPose);
            for(const lso::SemanticMark::Ptr& mark : tKF->VisibleSMarks){
                Marks.insert(mark);
                MarkNum++;
                MarkSize += mark->mCloud->size();
            }
            KeyFrameNum++;
            GeoSize += tKF->mCloud->size();
        }

        void MaintainKeepSec(double now_, double keep_){
            double thre = now_ - keep_;
            for(auto itf = KeyFrames.begin(); itf != KeyFrames.end();){
                if(itf->first->mTime < thre){
                    for(auto imk = itf->first->VisibleSMarks.begin(); imk != itf->first->VisibleSMarks.end();){
                        MarkNum--;
                        MarkSize -= (*imk)->mCloud->size();
                        itf->first->VisibleSMarks.erase(imk++);
                    }
                    KeyFrameNum--;
                    GeoSize -= itf->first->mCloud->size();
                    KeyFrames.erase(itf++);
                    Path.pop_front();
                }else{
                    return ;
                }
            }
        }

        // Delete Such Frame Use Shared Ptr
        void RemoveKeyFrame(lso::KeyFrame::Ptr &kf){
            auto it = std::find(KeyFrames.begin(), KeyFrames.end(), std::make_pair(kf, kf->Ind));
            // Delete the Frame's SMarks and Frame
            if(it != KeyFrames.end()){
                for(lso::SemanticMark::Ptr mk : it->first->VisibleSMarks){
                    RemoveSMarks(mk);
                }
                KeyFrameNum--;
                GeoSize -= it->first->mCloud->size();
                KeyFrames.erase(it);
                Path.pop_front();
            }
        }

        // Delete Such SMarks
        void RemoveSMarks(lso::SemanticMark::Ptr &mk){
            auto it = std::find(Marks.begin(), Marks.end(), mk);
            if(it != Marks.end()) {
                MarkNum--;
                MarkSize -= (*it)->mCloud->size();
                Marks.erase(it);
            }
        }

        //
        void GenerateGeometryMapCloudMsg(sensor_msgs::PointCloud2& msg){
            semanticCloudPtr Cloud(new semanticCloud());
            for(auto& frame : KeyFrames){
                semanticCloudPtr tmp(new semanticCloud());
                pcl::transformPointCloud(*(frame.first->mCloud), *tmp, frame.first->mPose.Posture.matrix());
                *Cloud += *tmp;
            }
            pcl::toROSMsg(*Cloud, msg);
        }

        //
        void GeneratePath(nav_msgs::Path &msg){
            msg.poses.clear();
            for( auto& to : Path){
                msg.poses.push_back(Pose2PoseStamped(to));
            }
        }

    public:
        using Ptr = std::shared_ptr<Map>;
    };
}

#endif
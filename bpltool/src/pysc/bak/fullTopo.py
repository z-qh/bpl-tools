import time

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2

import Base
import open3d
import math
from collections import defaultdict
import pickle

if __name__ == "__main__":
    # # get pose and time x y z
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan/liosave/sam2.txt")

    # # get Bag to find boundary
    # acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
    #                                          bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
    #                                          lidar_topic="/lslidar_point_cloud",
    #                                          acc_full_topo_info_path="/home/qh/YES/dlut/Daquan/acc_full_topo_info.pkl",
    #                                          load=False)
    # acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=None,
    #                                          bag_file=None,
    #                                          lidar_topic=None,
    #                                          acc_full_topo_info_path="/home/qh/YES/dlut/Daquan/acc_full_topo_info.pkl",
    #                                          load=True)

    # # generate acc full voxel map
    # map = Base.GetVoxelMap(cloud_path="/home/qh/YES/dlut/Daquan/liosave/GlobalMapDS.pcd",
    #                        path="/home/qh/YES/dlut/Daquan/acc_global_voxel_map.pkl",
    #                        x_res=1.0, y_res=1.0, z_res=1.5)
    # map = Base.GetVoxelMap(cloud_path=None,
    #                        path="/home/qh/YES/dlut/Daquan/acc_global_voxel_map.pkl",
    #                        x_res=1.0, y_res=1.0, z_res=1.5)

    # # generate acc full topo node
    # accfulltopo = Base.GetAccFullTopo(accmap=map,
    #                                   topo_info=acc_full_topo_info,
    #                                   acc_fulltopo_path="/home/qh/YES/dlut/Daquan/acc_full_topo.pkl",
    #                                   load=False)
    accfulltopo = Base.GetAccFullTopo(accmap=None,
                                      topo_info=None,
                                      acc_fulltopo_path="/home/qh/YES/dlut/Daquan/acc_full_topo.pkl",
                                      load=True)
    # # get acc full topo connect mat

    # acc_full_topo_connect = Base.GetFullTopoConnectInfo(acc_full_topo=accfulltopo,
    #                                                     connec_path="/home/qh/YES/dlut/Daquan/acc_connect.pkl",
    #                                                     load=False)
    # acc_full_topo_connect = Base.GetFullTopoConnectInfo(full_topo=None,
    #                                                     connect_path="/home/qh/YES/dlut/Daquan/acc_connect.pkl",
    #                                                     load=True)

    # # generate appearance full topo
    # app_full_topo = Base.GetAppearanceFullTopoInfo(pose_vec=pose_vec_data,
    #                                                bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
    #                                                lidar_topic="/lslidar_point_cloud",
    #                                                appearance_full_topo_info_path="/home/qh/YES/dlut/Daquan/app_full_topo.pkl",
    #                                                load=False)
    app_full_topo = Base.GetAppearanceFullTopoInfo(pose_vec=None,
                                                   bag_file=None,
                                                   lidar_topic=None,
                                                   appearance_full_topo_info_path="/home/qh/YES/dlut/Daquan/app_full_topo.pkl",
                                                   load=True)
    # # get appearance full topo connect mat
    app_full_topo_connect = Base.GetFullTopoConnectInfo(full_topo=app_full_topo,
                                                        connect_path="/home/qh/YES/dlut/Daquan/app_connect.pkl",
                                                        load=False)
    # acc_full_topo_connect = Base.GetFullTopoConnectInfo(acc_full_topo=None,
    #                                                     connec_path="/home/qh/YES/dlut/Daquan/acc_connect.pkl",
    #                                                     load=True)


    print(123)

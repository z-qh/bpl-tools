import collections
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2

from scipy.spatial.transform import Rotation
import Base
import open3d
import math
from collections import defaultdict
import pickle
from numpy import trapz
import glob

if __name__ == "__main__":
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
                                                 lidar_topic="/lslidar_point_cloud")
    map19 = Base.GetVoxelMap(path="/home/qh/YES/dlut/Daquan19")
    acc_full_topo = Base.GetAccFullTopo(accmap=map,
                                        topo_info=acc_full_topo_info)

    # aa = pickle.load(open("/home/qh/YES/dlut/Daquan19/vis/19-600TonoNodeAcc.pkl", "rb"))
    # Base.plot_multiple_sc(aa.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/19-600Acc.png")
    # bb = pickle.load(open("/home/qh/YES/dlut/Daquan19/vis/19-600TopoNode.pkl", "rb"))
    # Base.plot_multiple_sc(bb.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/19-600.png")

    # top_, gdis_ = 5, 5.0
    # sim_map_list = [0.90]
    # acc_pr_path_16 = "/home/qh/YES/dlut/Daquan16/acc_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    # app_pr_path_16 = "/home/qh/YES/dlut/Daquan16/app_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    # acc_pr_path_17 = "/home/qh/YES/dlut/Daquan17/acc_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    # app_pr_path_17 = "/home/qh/YES/dlut/Daquan17/app_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    #
    # acc_pr_load16 = pickle.load(open(acc_pr_path_16, "rb"))
    # app_pr_load16 = pickle.load(open(app_pr_path_16, "rb"))
    # acc_pr_load17 = pickle.load(open(acc_pr_path_17, "rb"))
    # app_pr_load17 = pickle.load(open(app_pr_path_17, "rb"))
    #
    # acc_pr = []
    # app_pr = []
    # acc_pr.append(acc_pr_load16)
    # acc_pr.append(acc_pr_load17)
    # app_pr.append(app_pr_load16)
    # app_pr.append(app_pr_load17)
    #
    # plot_muliti_pr2(acc_pr, app_pr, row_size_=1, save_path="/home/qh/12345.png")
    print(123)

    # """ Test Ground Seg
    if not VoxelMap.GroundSegInitFlag:
        VoxelMap.SetParameter(nscan=32, hscan=2000,
                              low_ang=-16.5, up_ang=10.0,
                              ground_ind=15, min_range=5.0,
                              segment_theta_d=30.0)
    bin_file = "/home/qh/YES/dlut/Daquan19/bin/000800.bin"
    if not os.path.isfile(bin_file):
        print("Not Found Bin File")
        exit(0)
    bin_points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
    ups = VoxelMap.GetSelectRay(bin_points, n_sec=10)
    pcd_ray_end = open3d.geometry.PointCloud()
    pcd_ray_end.points = open3d.utility.Vector3dVector(ups)
    pcd_ray_end.paint_uniform_color([0, 0, 0])
    delete_dic = collections.defaultdict(int)
    for ray_ind in range(0, ups.shape[0]):
        ray = ups[ray_ind, :]
        walks_vo = VoxelMap.walk_voxels(start=[0, 0, 0], end=ray, voxel_size=0.1)
        for vv in range(0, len(walks_vo)-5):
            delete_dic[walks_vo[vv]] = 2
    delete_points = np.zeros((len(delete_dic), 3), dtype=np.float32)
    for ind, key in enumerate(delete_dic):
        delete_points[ind, :] = VoxelMap.CenterVoxel(np.array(key), 0.1)
    pcd_delete_voxel = open3d.geometry.PointCloud()
    pcd_delete_voxel.points = open3d.utility.Vector3dVector(delete_points)
    pcd_delete_voxel.paint_uniform_color([0, 0, 1.0])
    open3d.visualization.draw_geometries([pcd_ray_end, pcd_delete_voxel])

    print(123)
    # """
    input_file = "/home/qh/YES/dlut/Daquan19/liosave/GlobalMap.pcd"
    output_file = "/home/qh/YES/dlut/Daquan19/liosave/GlobalMapDS1.pcd"

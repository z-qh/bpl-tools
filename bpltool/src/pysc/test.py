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
    Base.SetParameter(nscan=64, hscan=1800,
                      low_ang=-24.8, up_ang=2.0,
                      ground_ind=60, min_range=5.0)
    pose_vec_file = "/home/qh/YES/kitti/08/poses.txt"
    pose_vec = np.loadtxt(pose_vec_file)
    bin_dir = "/home/qh/YES/kitti/08/velodyne"
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    gt_2_laser = [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]
    gt_2_laser_T = np.array((gt_2_laser), dtype=np.float64).reshape((4, 4))
    bin_info = []
    for ind in range(500, len(bin_to_use)):
        T_ = np.row_stack((pose_vec[ind].reshape((3, 4)), np.array([0, 0, 0, 1])))
        T_ = np.dot(np.dot(gt_2_laser_T.transpose(), T_), gt_2_laser_T)
        p_ = T_[0:3, 3:4]
        r_ = T_[0:3, 0:3]
        bin_file = bin_to_use[ind]
        bin_info.append((0, ind, bin_file, p_, r_))
        local_points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, :3]
        local_points = local_points[~np.isnan(local_points).any(axis=1)]
        select_points = Base.GetSelectRay(local_points, n_sec=10, surf_threshold=0.2)
        print(123)
        print(123)
    map_seq08 = Base.VoxelMap("/home/qh/kitti/08/GlobalMapDS1.pcd",
                              save_path="/home/qh/kitti/08",
                              bin_list=bin_info)

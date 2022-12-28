import collections
import time
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import Base
import open3d
import numba
import math
from collections import defaultdict
import pickle
import glob
import time as ttime

"""
尝试使用kITTI的数据来完成Voxelmap的构建，来验证去除动态障碍的能力

"""


N_SCAN = 64
Horizon_SCAN = 1800
low_angle = -24.8
up_angle = 2.0
ang_res_y = (up_angle - low_angle) / (N_SCAN - 1)
ang_res_x = 360.0 / Horizon_SCAN


@numba.jit(nopython=True)
def GetInRange(points):
    points = points[:, 0:3]
    point_size = points.shape[0]
    ans = np.zeros((point_size, 3), dtype=np.float32)
    ind = 0
    for i in range(point_size):
        this_point = points[i, :]
        vertical_angle = np.arctan2(this_point[2], np.sqrt(this_point[0] ** 2 + this_point[1] ** 2)) * 180 / np.pi
        row_idn = int((vertical_angle - low_angle) / ang_res_y)
        if row_idn < 20 or row_idn >= 55:
            continue
        horizon_angle = np.arctan2(this_point[0], this_point[1]) * 180 / np.pi
        column_idn = int(-round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN / 2)
        if column_idn >= Horizon_SCAN:
            column_idn -= Horizon_SCAN
        if column_idn < 0 or column_idn >= Horizon_SCAN:
            continue
        range_ = np.linalg.norm(this_point)
        if range_ < 5 or range_ > 80:
            continue
        ans[ind, :] = points[i, :]
        ind += 1
    return ans[0:ind, :]


def handleCloud():
    # """
    # 将位姿和点云帧一一对应
    pose_vec_file = "/home/qh/YES/kitti/07/poses.txt"
    pose_vec = np.loadtxt(pose_vec_file)
    bin_dir = "/home/qh/YES/kitti/07/velodyne"
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    bin_info = []
    gt_2_laser = [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]
    gt_2_laser_T = np.array((gt_2_laser), dtype=np.float64).reshape((4, 4))
    # 500 - 4071, 500 - 2200, 2200 -
    # for ind in range(500, 1000):
    for ind in range(0, len(bin_to_use)):
        T_ = np.row_stack((pose_vec[ind].reshape((3, 4)), np.array([0, 0, 0, 1])))
        T_ = np.dot(np.dot(gt_2_laser_T.transpose(), T_), gt_2_laser_T)
        p_ = T_[0:3, 3:4]
        r_ = T_[0:3, 0:3]
        bin_file = bin_to_use[ind]
        bin_info.append((ind, bin_file, p_, r_))
    # """
    proc_count = 0
    start_time = ttime.time()
    voxel_map = collections.defaultdict(int)
    for ind, bin_file, p_, r_ in bin_info:
        l_points = GetInRange(np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4)))
        l_points = l_points[~np.isnan(l_points).any(axis=1)]
        g_points = Base.TransPointCloud(l_points, r_, p_)
        for p_ind in range(g_points.shape[0]):
            voxel = Base.Posi2Ind(g_points[p_ind, :], voxel_size=0.1)
            voxel_map[voxel] = 1
        proc_count += 1
        if proc_count % 100 == 0:
            print("process {:.2f}% use {:.2f}s".format(proc_count / len(bin_info) * 100, ttime.time() - start_time))
            start_time = ttime.time()
    print("123")
    all_points = np.zeros((len(voxel_map), 3), dtype=np.float32)
    start_time = ttime.time()
    for i, voxel in enumerate(voxel_map):
        all_points[i, :] = Base.CenterVoxel(np.array(voxel), voxel_size=0.1)
        if proc_count % 100000 == 0:
            print("process {:.2f}% use {:.2f}s".format(i / len(voxel_map) * 100, ttime.time() - start_time))
            start_time = ttime.time()
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(all_points)
    open3d.io.write_point_cloud("/home/qh/kitti/07/GlobalMapDS.pcd", pcd)


def GetVoxelMap():
    Base.SetParameter(nscan=64, hscan=1800,
                      low_ang=-24.8, up_ang=2.0,
                      ground_ind=60, min_range=5.0)
    pose_vec_file = "/home/qh/YES/kitti/07/poses.txt"
    pose_vec = np.loadtxt(pose_vec_file)
    bin_dir = "/home/qh/YES/kitti/07/velodyne"
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    gt_2_laser = [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]
    gt_2_laser_T = np.array((gt_2_laser), dtype=np.float64).reshape((4, 4))
    bin_info = []
    for ind in range(0, len(bin_to_use)):
    # for ind in range(500, 1000):
        T_ = np.row_stack((pose_vec[ind].reshape((3, 4)), np.array([0, 0, 0, 1])))
        T_ = np.dot(np.dot(gt_2_laser_T.transpose(), T_), gt_2_laser_T)
        p_ = T_[0:3, 3:4]
        r_ = T_[0:3, 0:3]
        bin_file = bin_to_use[ind]
        bin_info.append((0, ind, bin_file, p_, r_))

    Base.SetParameter(nscan=64, hscan=1800,
                      low_ang=-24.8, up_ang=2.0,
                      ground_ind=50, min_range=5.0,
                      segment_theta_d=10.0,
                      down_lines=10, up_lines=55)
    map_seq05 = Base.VoxelMap("/home/qh/kitti/07/GlobalMapDS.pcd",
                              save_path="/home/qh/kitti/07",
                              bin_list=bin_info)


if __name__ == "__main__":
    # handleCloud()
    GetVoxelMap()
    print(123)

import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pylab as p
import rosbag
import sensor_msgs.point_cloud2 as pc2
import Base
import open3d
import math
from collections import defaultdict
import pickle
from scipy.spatial.transform import Rotation
import glob
import collections
import time as ttime
import numba
import BA


# 使用时间戳进行二分查找 找到某个戳对应的bin将位姿对应起来
def BinFind(pose_vec, x, l, r):
    if r >= l:
        mid = int(l + (r - l) / 2)
        if abs(pose_vec[mid, 0] - x) < 0.05:
            return mid
        elif pose_vec[mid, 0] > x:
            return BinFind(pose_vec, x, l, mid - 1)
        else:
            return BinFind(pose_vec, x, mid + 1, r)
    else:
        return -1


# 大圈数据 第一行数据格式 第二行数据量 time x y z qw qx qy qz
def GetBinList(bin_dir, bin_times_file, pose_vec_file, save_path=None):
    if save_path is not None and os.path.isfile(save_path):
        bin_info = pickle.load(open(save_path, "rb"))
        return bin_info
    bin_info = []
    bin_times = np.loadtxt(bin_times_file)
    pose_vec = np.loadtxt(pose_vec_file, skiprows=2)
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    for i in range(0, len(bin_to_use), 1):
        bin_file = bin_to_use[i]
        bin_id = int(bin_file.split('/')[-1].split('.')[-2])
        now_time = bin_times[bin_id]
        pose_ind = BinFind(pose_vec, now_time, 0, pose_vec.shape[0] - 1)
        if pose_ind == -1:
            continue
        p_ = pose_vec[pose_ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[pose_ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        bin_info.append((now_time, pose_ind, bin_file, p_, r_))
    if save_path is not None:
        pickle.dump(bin_info, open(save_path, "wb"))
    return bin_info


# 获取某个数据包的bin和位姿，利用总体点云地图PCD来建立voxelmap并删除动态障碍
def GetAccMap(path, s=0, e=-1):
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))[s:e]
        Base.SetParameter(nscan=32, hscan=2000,
                          low_ang=-16.5, up_ang=10,
                          ground_ind=15, min_range=5.0,
                          segment_theta_d=40.0,
                          down_lines=-1, up_lines=-1)
        map1x = Base.VoxelMap2(save_path=os.path.join(path, "orb"), bin_list=bin_list, voxel_size=0.05)
        return map1x
    else:
        print("Error!")
        exit(0)


class GeometryMap:
    def __init__(self, cloud_path, bin_list, vs=0.05):
        self.cloud_map = open3d.io.read_point_cloud(cloud_path)
        self.kdtree = open3d.geometry.KDTreeFlann(self.cloud_map)
        self.points = np.array(self.cloud_map.points)
        self.bin_list = bin_list

    def getAroundPoints(self, ind):
        if 0 <= ind < len(self.bin_list):
            bin_p_ = self.bin_list[ind][3]
            bin_r_ = self.bin_list[ind][4]
            serach_radius = 200
            _, p_ind, _ = self.kdtree.search_radius_vector_3d(np.array(bin_p_), serach_radius)
            tmp_point = self.points[p_ind, :]
            tmp_point = Base.TransInvPointCloud(tmp_point, bin_r_, bin_p_)
            return tmp_point
        else:
            print("out of range")
            return None

    def getDepthImg(self, ind):
        points = self.getAroundPoints(ind)
        if points is not None:
            return 1

    def getPBAImg(self, ind):
        points = self.getAroundPoints(ind)
        if points is not None:
            return 1


def generate_data():
    map16 = GetAccMap("/home/qh/YES/dlut/Daquan16", s=100, e=400)
    map17 = GetAccMap("/home/qh/YES/dlut/Daquan17", s=100, e=400)
    map19 = GetAccMap("/home/qh/YES/dlut/Daquan19", s=0, e=300)


def genimg():
    bin_list_16 = pickle.load(open("/home/qh/YES/dlut/Daquan16/orb/bin_info.pkl", "rb"))
    LM16 = GeometryMap("/home/qh/YES/dlut/Daquan16/orb/VM2_pcd.pcd", bin_list_16)
    for ind in range(0, 300):
        aa = LM16.getAroundPoints(ind)
        a = BA.do_range_projection(points=aa)
        a.save("/home/qh/YES/dlut/Daquan16/orb/imgs/{:d}.png".format(ind))

    bin_list_17 = pickle.load(open("/home/qh/YES/dlut/Daquan17/orb/bin_info.pkl", "rb"))
    LM17 = GeometryMap("/home/qh/YES/dlut/Daquan17/orb/VM2_pcd.pcd", bin_list_17)
    for ind in range(0, 300):
        aa = LM17.getAroundPoints(ind)
        a = BA.do_range_projection(points=aa)
        a.save("/home/qh/YES/dlut/Daquan17/orb/imgs/{:d}.png".format(ind))

    bin_list_19 = pickle.load(open("/home/qh/YES/dlut/Daquan19/orb/bin_info.pkl", "rb"))
    LM19 = GeometryMap("/home/qh/YES/dlut/Daquan19/orb/VM2_pcd.pcd", bin_list_19)
    for ind in range(0, 300):
        aa = LM19.getAroundPoints(ind)
        a = BA.do_range_projection(points=aa)
        a.save("/home/qh/YES/dlut/Daquan19/orb/imgs/{:d}.png".format(ind))


if __name__ == "__main__":
    genimg()
    print(123)
    "/home/qh/"

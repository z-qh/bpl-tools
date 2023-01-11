import os
import time
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
import time as ttime

"""
本文件主要完成：
1.Oxford两圈的数据生成、障碍物去除、拓扑数据的生成
2.多阈值拓扑地图生成
3.整定累积和外观的最优参数
4.检验最终的PR曲线
某些绘图需求也在这里面实现
"""
fix_array = np.array([5735936.253250, 620062.035182, -109.767389, 0, 0, 0])
o1_start_time, o1_end_time = 1547121050529479, 1547122996303616
o2_start_time, o2_end_time = 1547123576678646, 1547125731737581


def xyzrpy2se3(xyzrpy):
    if len(xyzrpy) != 6:
        print("XYZRPY ERROR!")
        exit(0)
    T_ = np.eye(4)
    T_[0:3, 0:3] = Rotation.from_euler("zyx", xyzrpy[5:2:-1], degrees=False).as_matrix()
    T_[0:3, 3:4] = xyzrpy[0:3].reshape((3, 1))
    return T_


def get_ins_pose(ins_file):
    time_stamp, abs_poses = [], []
    with open(ins_file) as f:
        lines = f.readlines()
        for i in range(1, len(lines)):
            data = lines[i].strip("\n").split(",")
            time = float(data[0])
            xyzrpy = np.array([float(v) for v in data[5:8]] + [float(v) for v in data[-3:]]) - fix_array
            time_stamp.append(time)
            abs_poses.append(xyzrpy2se3(xyzrpy))
    return time_stamp, abs_poses


def GetBinList(bin_dir, pose_file, start_time=-1, end_time=-1, save_path=None):
    if save_path is not None and os.path.isfile(save_path):
        bin_info = pickle.load(open(save_path, "rb"))
        return bin_info
    bin_info = []
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    bin_times = np.zeros((len(bin_to_use)), dtype=np.float64)
    for i in range(len(bin_to_use)):
        bin_times[i] = float(os.path.basename(bin_to_use[i]).split(".")[0])
    pose_time, pose_vec = get_ins_pose(pose_file)
    ind_count = 0
    if start_time == -1:
        start_time = bin_times[0]
    if end_time == -1:
        end_time = bin_times[-1]
    bin_dic = {}
    for i in range(len(pose_time)):
        p_time = pose_time[i]
        if p_time < start_time or p_time > end_time:
            continue
        pose = pose_vec[i]
        closest_bin_ind = np.argmin(np.abs(bin_times - p_time))
        if abs(bin_times[closest_bin_ind] / 1E6 - p_time / 1E6) > 0.04:
            continue
        if closest_bin_ind in bin_dic:
            continue
        bin_dic[closest_bin_ind] = 1
        closest_bin = bin_to_use[closest_bin_ind]
        r_ = pose[0:3, 0:3]
        p_ = pose[0:3, 3:4]
        bin_info.append((p_time / 1E6, ind_count, closest_bin, p_, r_))
        ind_count += 1
    if save_path is not None and len(bin_info) != 0:
        pickle.dump(bin_info, open(save_path, "wb"))
    return bin_info


def showPosi(poses):
    x, y, z = [], [], []
    for pose in poses:
        x.append(pose[0, 3])
        y.append(pose[1, 3])
        z.append(pose[2, 3])
    ax = plt.axes(projection="3d")
    ax.scatter3D(x, y, z, color="red")
    ax.set_zlim((-400, 600))
    plt.show()


def showPose(poses):
    axis_list = []
    for pose in poses:
        axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
        axis_pcd.transform(pose)
        axis_list.append(axis_pcd)
    open3d.visualization.draw_geometries(axis_list)


# 获取某个数据包的bin和位姿，利用总体点云地图PCD来建立voxelmap并删除动态障碍
def GetAccFullTopo(path, s, e):
    # path = "/home/dlut/zqh/oxford/o1"
    acc_full_topo_path = os.path.join(path, "acc_full_topo.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "left_bin"),
                              pose_file=os.path.join(path, "gps/ins.csv"),
                              save_path=os.path.join(path, "bin_info.pkl"),
                              start_time=s, end_time=e)
        Base.SetParameter(nscan=32, hscan=900,
                          low_ang=-30.67, up_ang=10.67,
                          ground_ind=20, min_range=5.0,
                          segment_theta_d=40.0,
                          down_lines=-1, up_lines=-1)
        map1x = Base.VoxelMap2(save_path=path, bin_list=bin_list)
    else:
        print("ERROR when get DataSet!")
        return None
    acc_full_topo = []
    handle_size = len(bin_list)
    report_size = handle_size // 50 if handle_size // 50 != 0 else 1
    start_time = ttime.time()
    for n_time, ind, bin_file, p_, r_ in bin_list:
        points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        points = points[~np.isnan(points).any(axis=1)]
        boundary = Base.getBound(points)
        topo_cloud = map1x.GetAreaPointCloud(p_, boundary)
        topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), topo_cloud, ch=3)
        acc_full_topo.append(tmp_topo_node)
        if ind % report_size == 0:
            print("Gen Acc Topo Node {:.2f}% Cost {:.2f}s".format(ind / handle_size * 100, ttime.time() - start_time))
            start_time = ttime.time()
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save Acc Topo Node!")
    return acc_full_topo


# # 生成appearance-based的完整关键帧节点
def GetAppFullTopo(path, s, e):
    app_full_topo_path = os.path.join(path, "app_full_topo.pkl")
    if os.path.isfile(app_full_topo_path):
        app_full_topo = pickle.load(open(app_full_topo_path, "rb"))
        return app_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "left_bin"),
                              pose_file=os.path.join(path, "gps/ins.csv"),
                              save_path=os.path.join(path, "bin_info.pkl"),
                              start_time=s, end_time=e)
    else:
        print("ERROR when get DataSet!")
        return None
    app_full_topo = []
    for n_time, ind, bin_file, p_, r_ in bin_list:
        points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        points = points[~np.isnan(points).any(axis=1)]
        boundary = Base.getBound(points)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), points, ch=3)
        app_full_topo.append(tmp_topo_node)
    if len(app_full_topo) != 0:
        pickle.dump(app_full_topo, open(app_full_topo_path, "wb"))
        print("Save Acc Topo Node!")
    return app_full_topo


if __name__ == "__main__":
    # """
    # o1 累积
    # acc_o1_full_topo = GetAccFullTopo("/home/dlut/zqh/oxford/o1", o1_start_time, o1_end_time)
    # del acc_o1_full_topo
    # save_path = "/home/dlut/zqh/oxford/o1/acc_sim_mat.pkl"
    # acc_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path)

    # o1 外观
    app_o1_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    del app_o1_full_topo
    # save_path = "/home/dlut/zqh/oxford/o1/app_sim_mat.pkl"
    # app_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path)
    # """

    # """
    # o2 累积
    # acc_o2_full_topo = GetAccFullTopo("/home/dlut/zqh/oxford/o2", o2_start_time, o2_end_time)
    # del acc_o2_full_topo
    # save_path = "/home/dlut/zqh/oxford/o2/acc_sim_mat.pkl"
    # acc_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path, acc_o2_full_topo)

    # o2 外观
    # app_o2_full_topo = GetAppFullTopo("/home/dlut/zqh/oxford/o2", o2_start_time, o2_end_time)
    # del app_o2_full_topo
    # save_path = "/home/dlut/zqh/oxford/o2/app_sim_mat.pkl"
    # app_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path, app_o2_full_topo)
    # """

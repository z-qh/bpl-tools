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
from scipy.stats import gaussian_kde

"""
本文件主要完成：
1.三圈大圈地图生成、障碍物去除、拓扑数据的生成
2.多阈值拓扑地图生成
3.整定累积和外观的最优参数
4.检验最终的PR曲线
某些绘图需求也在这里面实现
"""


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


# 获取点云bin的路径和对应的位姿 打包成一一对应输出
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
def GetAccFullTopo(path):
    # path = "/home/qh/YES/dlut/Daquan19"
    acc_full_topo_path = os.path.join(path, "acc_full_topo.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        Base.SetParameter(nscan=32, hscan=2000,
                          low_ang=-16.5, up_ang=10,
                          ground_ind=15, min_range=5.0,
                          segment_theta_d=40.0,
                          down_lines=-1, up_lines=-1)
        map1x = Base.VoxelMap2(save_path=path, bin_list=bin_list, voxel_size=0.2)
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


def GetAccFullTopoRemoveNone16(path, RD=False):
    # path = "/home/qh/YES/dlut/Daquan19"
    acc_full_topo_path = os.path.join(path, "acc_full_topo_rn_rd.pkl" if RD else "acc_full_topo_rn.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        if RD:
            map1x = Base.VoxelMap4_Load(save_path=path, voxel_size=0.2)
        else:
            map1x = Base.VoxelMap3_Load(save_path=path, voxel_size=0.2)
    else:
        print("ERROR when get DataSet!")
        return None
    acc_full_topo = []
    # bin_list = bin_list[21400:24000]
    bin_list = bin_list[17300:17700] + bin_list[18750:19200] + bin_list[11400:12500]
    for n_time, ind, bin_file, p_, r_ in bin_list:
        print('\r', ind, end="")
        boundary = (-80, 80, -80, 80, -5, 5)
        topo_cloud = map1x.GetAreaPointCloud(p_, boundary)
        topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
        if not RD:
            topo_cloud = Base.rmmm(topo_cloud, -0.5)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), topo_cloud, ch=3)
        acc_full_topo.append(tmp_topo_node)
        # Base.plot_multiple_sc(tmp_topo_node.SCs)
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # open3d.visualization.draw_geometries([pcd])
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save RN Acc Topo Node!", len(acc_full_topo))
    return acc_full_topo


def GetAccFullTopoRemoveNone17(path, RD=False):
    # path = "/home/qh/YES/dlut/Daquan19"
    acc_full_topo_path = os.path.join(path, "acc_full_topo_rn_rd.pkl" if RD else "acc_full_topo_rn.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        if RD:
            map1x = Base.VoxelMap4_Load(save_path=path, voxel_size=0.2)
        else:
            map1x = Base.VoxelMap3_Load(save_path=path, voxel_size=0.2)
    else:
        print("ERROR when get DataSet!")
        return None
    acc_full_topo = []
    # bin_list = bin_list[19900:22200]
    bin_list = bin_list[10500:11000] + bin_list[16300:16400] + bin_list[14900:15200]
    for n_time, ind, bin_file, p_, r_ in bin_list:
        print('\r', ind, end="")
        boundary = (-80, 80, -80, 80, -5, 5)
        topo_cloud = map1x.GetAreaPointCloud(p_, boundary)
        topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
        if not RD:
            topo_cloud = Base.rmmm(topo_cloud, -0.5)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), topo_cloud, ch=3)
        acc_full_topo.append(tmp_topo_node)
        # Base.plot_multiple_sc(tmp_topo_node.SCs)
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # open3d.visualization.draw_geometries([pcd])
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save RN Acc Topo Node!", len(acc_full_topo))
    return acc_full_topo


# 获取某个数据包的bin和位姿，利用总体点云地图PCD, 不删除动态障碍物
def GetAccFullTopoRemoveNone19(path, RD=False):
    # path = "/home/qh/YES/dlut/Daquan19"
    acc_full_topo_path = os.path.join(path, "acc_full_topo_rn_rd.pkl" if RD else "acc_full_topo_rn.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        if RD:
            map1x = Base.VoxelMap4_Load(save_path=path, voxel_size=0.2)
        else:
            map1x = Base.VoxelMap3_Load(save_path=path, voxel_size=0.2)
    else:
        print("ERROR when get DataSet!")
        return None
    acc_full_topo = []
    # bin_list = bin_list[18500:20500]
    bin_list = bin_list[9000:9800] + bin_list[17100:17500] + bin_list[14100:14300]
    for n_time, ind, bin_file, p_, r_ in bin_list:
        print("\r", ind, end="")
        boundary = (-80, 80, -80, 80, -5, 5)
        topo_cloud = map1x.GetAreaPointCloud(p_, boundary)
        topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
        if not RD:
            topo_cloud = Base.rmmm(topo_cloud, -0.5)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), topo_cloud, ch=3)
        acc_full_topo.append(tmp_topo_node)
        # Base.plot_multiple_sc(tmp_topo_node.SCs)
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # open3d.visualization.draw_geometries([pcd])
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save RN Acc Topo Node!")
    return acc_full_topo


# 生成appearance-based的完整关键帧节点
def GetAppFullTopo(path):
    app_full_topo_path = os.path.join(path, "app_full_topo.pkl")
    if os.path.isfile(app_full_topo_path):
        app_full_topo = pickle.load(open(app_full_topo_path, "rb"))
        return app_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
    else:
        print("ERROR when get DataSet!")
        return None
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
    """ 显示画图三圈轨迹
    pose_vec_data16 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan16/liosave/sam2.txt")
    pose_vec_data17 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan17/liosave/sam2.txt")
    pose_vec_data19 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    """

    """
    # # Completion the half connect matrix 补全上三角矩阵
    Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/acc_sim_matrn.pkl")
    Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/acc_sim_mat.pkl")
    Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/app_sim_mat.pkl")
    """

    """
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))

    bin_list19 = GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan19", "bin"),
                            bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan19", "timestamp"),
                            pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan19", "liosave/sam2.txt"),
                            save_path=os.path.join("/home/qh/YES/dlut/Daquan19", "bin_info.pkl"))

    bin_list16 = GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan16", "bin"),
                            bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan16", "timestamp"),
                            pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan16", "liosave/sam2.txt"),
                            save_path=os.path.join("/home/qh/YES/dlut/Daquan16", "bin_info.pkl"))

    bin_list17 = GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan17", "bin"),
                            bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan17", "timestamp"),
                            pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan17", "liosave/sam2.txt"),
                            save_path=os.path.join("/home/qh/YES/dlut/Daquan17", "bin_info.pkl"))

    posi_19 = np.zeros((0, 3), dtype=np.float32)
    for now_time, pose_ind, bin_file, p_, r_ in bin_list19:
        posi_19 = np.vstack((posi_19, p_.reshape(-1)))
    posi_16 = np.zeros((0, 3), dtype=np.float32)
    for now_time, pose_ind, bin_file, p_, r_ in bin_list16:
        posi_16 = np.vstack((posi_16, p_.reshape(-1)))
    posi_17 = np.zeros((0, 3), dtype=np.float32)
    for now_time, pose_ind, bin_file, p_, r_ in bin_list17:
        posi_17 = np.vstack((posi_17, p_.reshape(-1)))

    ax.scatter(posi_19[:, 0], posi_19[:, 1], marker="o", s=10, c="red", alpha=0.5)
    ax.scatter(posi_16[:, 0], posi_16[:, 1], marker=".", s=10, c="green", alpha=0.5)
    ax.scatter(posi_17[:, 0], posi_17[:, 1], marker="*", s=10, c="blue", alpha=0.5)
    for i in range(posi_17.shape[0]):
        if i % 100 == 0:
            plt.text(posi_17[i, 0] + 0.2, posi_17[i, 1] + 0.2, '{:d}'.format(i), fontsize=12, c="blue")
    for i in range(posi_19.shape[0]):
        if i % 100 == 0:
            plt.text(posi_19[i, 0] + 0.2, posi_19[i, 1] + 0.2, '{:d}'.format(i), fontsize=12, c="red")
    for i in range(posi_16.shape[0]):
        if i % 100 == 0:
            plt.text(posi_16[i, 0] + 0.2, posi_16[i, 1] + 0.2, '{:d}'.format(i), fontsize=12, c="green")
    ax.set_aspect(1)
    plt.show()
    plt.close()
    """

    # """
    # 19 RN
    # acc_full_topo19_rn = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19")
    # save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat_rn.pkl"
    # acc_full_topo19_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, None)
    # Base.TopoConnectCompletion(save_path)

    # acc_full_topo16_rn = GetAccFullTopoRemoveNone16("/home/qh/YES/dlut/Daquan16")
    # save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19_rn.pkl"
    # acc_full_topo16_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, acc_full_topo16_rn)

    # acc_full_topo17_rn = GetAccFullTopoRemoveNone17("/home/qh/YES/dlut/Daquan17")
    # save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat_to19_rn.pkl"
    # acc_full_topo17_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, acc_full_topo17_rn)
    
    # RN RD
    # acc_full_topo19_rn_rd = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19", RD=True)
    # save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat_rn_rd.pkl"
    # acc_full_topo19_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, None)
    # Base.TopoConnectCompletion(save_path)

    # acc_full_topo16_rn_rd = GetAccFullTopoRemoveNone16("/home/qh/YES/dlut/Daquan16", RD=True)
    # save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19_rn_rd.pkl"
    # acc_full_topo16_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, acc_full_topo16_rn_rd)

    # acc_full_topo17_rn_rd = GetAccFullTopoRemoveNone17("/home/qh/YES/dlut/Daquan17", RD=True)
    # save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat_to19_rn_rd.pkl"
    # acc_full_topo17_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, acc_full_topo17_rn_rd)
    # """

    """
    # 19 累积
    acc_full_topo19 = GetAccFullTopo("/home/qh/YES/dlut/Daquan19")
    save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat.pkl"
    acc_full_topo19_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
    # 19 外观
    app_full_topo19 = GetAppFullTopo("/home/qh/YES/dlut/Daquan19")
    save_path = "/home/qh/YES/dlut/Daquan19/app_sim_mat.pkl"
    app_full_topo19_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
    """

    """
    # Daqaun16 用于验证测试 首先生成 fullnode 然后针对 累积19 外观19 生成相似度矩阵
    # 16 累积
    acc_full_topo16 = GetAccFullTopo("/home/qh/YES/dlut/Daquan16")
    save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19.pkl"
    acc_full_topo16_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo16)
    # 16 外观
    app_full_topo16 = GetAppFullTopo("/home/qh/YES/dlut/Daquan16")
    save_path = "/home/qh/YES/dlut/Daquan16/app_sim_mat_to19.pkl"
    app_full_topo16_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo16)
    """

    """
    # Daqaun17 用于验证测试 首先生成 fullnode 然后针对 累积19 外观19 生成相似度矩阵

    # 17 累积
    acc_full_topo17 = GetAccFullTopo("/home/qh/YES/dlut/Daquan17")
    save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat.pkl"
    acc_full_topo17_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo17)

    # 17 外观
    app_full_topo17 = GetAppFullTopo("/home/qh/YES/dlut/Daquan17")
    save_path = "/home/qh/YES/dlut/Daquan17/app_sim_mat.pkl"
    app_full_topo17_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo17)
    """

    """
    # ACC 建图整定
    acc_full_topo19 = GetAccFullTopo("/home/qh/YES/dlut/Daquan19")
    save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat.pkl"
    acc_full_topo19_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
    set_path = "/home/qh/YES/dlut/Daquan19"
    acc_sim_list = [0.80, 0.83, 0.85, 0.88, 0.90, 0.92, 0.95]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    acc_pr_dic_save = os.path.join(set_path, "topo_map/acc_total_pr.pkl")
    acc_pr_dic = {}
    for acc_sim_thre in zip(acc_sim_list):
        acc_sim = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(acc_sim_thre))
        acc_sim_fig = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.png".format(acc_sim_thre))
        acc_pr = os.path.join(set_path, "topo_map/acc_sim_{:.2f}_pr.pkl".format(acc_sim_thre))
        acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo19,
                                             sim_mat=acc_full_topo19_sim_mat,
                                             sim_threshold=acc_sim_thre,
                                             path=acc_sim)
        print("Save Acc Sim:{:.2f}".format(acc_sim_thre))
        Base.ShowTopoMap(acc_full_topo19, acc_tmp_topo, path=acc_sim_fig, vis=False)

        if os.path.isfile(acc_pr):
            tmp_acc_pr_list = pickle.load(open(acc_pr, "rb"))
        else:
            tmp_acc_pr_list = []
            for sim_recall in sim_recall_list:
                
                acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                                        base_topo=acc_tmp_topo,
                                                        full_topo=acc_full_topo19,
                                                        sim_mat=acc_full_topo19_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(acc_tmp_topo),
                                                        info="Daquan19.BuildSim{:.2f}.TopoNum{:d}".format(acc_sim_thre, len(acc_tmp_topo)))

                tmp_acc_pr_list.append(acc_tmp_pr)
            pickle.dump(tmp_acc_pr_list, open(acc_pr, "wb"))
        acc_pr_dic[acc_sim_thre] = tmp_acc_pr_list
        print("ACC PR:\n" + str(tmp_acc_pr_list))
    if not os.path.isfile(acc_pr_dic_save):
        pickle.dump(acc_pr_dic, open(acc_pr_dic_save, "wb"))
    Base.plot_muliti_pr_acc(acc_pr_dic)
    """

    """ #APP 建图整定
    app_full_topo19 = GetAppFullTopo("/home/qh/YES/dlut/Daquan19")
    save_path = "/home/qh/YES/dlut/Daquan19/app_sim_mat.pkl"
    app_full_topo19_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
    set_path = "/home/qh/YES/dlut/Daquan19"
    app_sim_list = [0.55, 0.58, 0.60, 0.62, 0.65, 0.68, 0.70, 0.80, 0.85, 0.90]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    app_pr_dic_save = os.path.join(set_path, "topo_map/app_total_pr.pkl")
    app_pr_dic = {}
    for app_sim_thre in app_sim_list:
        app_sim = os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(app_sim_thre))
        app_sim_fig = os.path.join(set_path, "topo_map/app_sim_{:.2f}.png".format(app_sim_thre))
        app_pr = os.path.join(set_path, "topo_map/app_sim_{:.2f}_pr.pkl".format(app_sim_thre))
        app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_full_topo19,
                                             sim_mat=app_full_topo19_sim_mat,
                                             sim_threshold=app_sim_thre,
                                             path=app_sim)
        print("Save App Sim:{:.2f}".format(app_sim_thre))
        Base.ShowTopoMap(app_full_topo19, app_tmp_topo, path=app_sim_fig, vis=False)

        if os.path.isfile(app_pr):
            tmp_app_pr_list = pickle.load(open(app_pr, "rb"))
        else:
            tmp_app_pr_list = []
            for sim_recall in sim_recall_list:
                app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                                        base_topo=app_tmp_topo,
                                                        full_topo=app_full_topo19,
                                                        sim_mat=app_full_topo19_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(app_tmp_topo),
                                                        info="Daquan19.BuildSim{:.2f}.TopoNum{:d}".format(app_sim_thre, len(app_tmp_topo)))

                tmp_app_pr_list.append(app_tmp_pr)
            pickle.dump(tmp_app_pr_list, open(app_pr, "wb"))
        app_pr_dic[app_sim_thre] = tmp_app_pr_list
        print("APP PR:\n" + str(tmp_app_pr_list))
    if not os.path.isfile(app_pr_dic_save):
        pickle.dump(app_pr_dic, open(app_pr_dic_save, "wb"))
    Base.plot_muliti_pr_app(app_pr_dic)
    """

    """
    # RN 建图和整定
    acc_full_topo19_rn = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19")
    save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat_rn.pkl"
    acc_full_topo19_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, None)
    set_path = "/home/qh/YES/dlut/Daquan19"
    rn_acc_sim_list = [0.3, 0.4, 0.5, 0.6, 0.65, 0.68, 0.70, 0.73, 0.75, 0.78, 0.80, 0.83, 0.85, 0.88, 0.90]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    rn_acc_pr_dic_save = os.path.join(set_path, "topo_map/rn_acc_total_pr.pkl")
    rn_acc_pr_dic = {}
    for rn_acc_sim_thre in rn_acc_sim_list:
        rn_acc_sim = os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.pkl".format(rn_acc_sim_thre))
        rn_acc_sim_fig = os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.png".format(rn_acc_sim_thre))
        rn_acc_pr = os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}_pr.pkl".format(rn_acc_sim_thre))
        rn_acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo19_rn,
                                                sim_mat=acc_full_topo19_rn_sim_mat,
                                                sim_threshold=rn_acc_sim_thre,
                                                path=rn_acc_sim)
        print("Save Rn Acc Sim:{:.2f}".format(rn_acc_sim_thre))
        Base.ShowTopoMap(acc_full_topo19_rn, rn_acc_tmp_topo, path=rn_acc_sim_fig, vis=False)

        if os.path.isfile(rn_acc_pr):
            tmp_rn_acc_pr_list = pickle.load(open(rn_acc_pr, "rb"))
            rn_acc_pr_dic[rn_acc_sim_thre] = tmp_rn_acc_pr_list
        else:
            tmp_rn_acc_pr_list = []
            for rn_sim_recall in sim_recall_list:
                rn_acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn,
                                                           base_topo=rn_acc_tmp_topo,
                                                           full_topo=acc_full_topo19_rn,
                                                           sim_mat=acc_full_topo19_rn_sim_mat,
                                                           sim=rn_sim_recall,
                                                           top=1, gdis=3.0, topo_num=len(rn_acc_tmp_topo),
                                                           info="Daquan19RN.BuildSim{:.2f}.TopoNum{:d}".format(
                                                               rn_acc_sim_thre, len(rn_acc_tmp_topo)))
                tmp_rn_acc_pr_list.append(rn_acc_tmp_pr)
            pickle.dump(tmp_rn_acc_pr_list, open(rn_acc_pr, "wb"))
            rn_acc_pr_dic[rn_acc_sim_thre] = tmp_rn_acc_pr_list
            # Base.plot_pr(tmp_rn_acc_pr_list)
        print("RN ACC PR:\n" + str(tmp_rn_acc_pr_list))
    if not os.path.isfile(rn_acc_pr_dic_save):
        pickle.dump(rn_acc_pr_dic, open(rn_acc_pr_dic_save, "wb"))
    Base.plot_muliti_pr_acc(rn_acc_pr_dic)
    """

    """
    # RN RD 建图和整定
    Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/acc_sim_mat_rn_rd.pkl")
    acc_full_topo19_rn_rd = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19", RD=True)
    save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat_rn_rd.pkl"
    acc_full_topo19_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, None)
    set_path = "/home/qh/YES/dlut/Daquan19"
    rn_acc_sim_list = [0.3, 0.4, 0.5, 0.6, 0.65, 0.68, 0.70, 0.73, 0.75, 0.78, 0.80, 0.83, 0.85, 0.88, 0.90]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    rn_acc_pr_dic_save = os.path.join(set_path, "topo_map/rn_rd_acc_total_pr.pkl")
    rn_acc_pr_dic = {}
    for rn_acc_sim_thre in rn_acc_sim_list:
        rn_acc_sim = os.path.join(set_path, "topo_map/rn_rd_acc_sim_{:.2f}.pkl".format(rn_acc_sim_thre))
        rn_acc_sim_fig = os.path.join(set_path, "topo_map/rn_rd_acc_sim_{:.2f}.png".format(rn_acc_sim_thre))
        rn_acc_pr = os.path.join(set_path, "topo_map/rn_rd_acc_sim_{:.2f}_pr.pkl".format(rn_acc_sim_thre))
        rn_acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo19_rn_rd,
                                                sim_mat=acc_full_topo19_rn_rd_sim_mat,
                                                sim_threshold=rn_acc_sim_thre,
                                                path=rn_acc_sim)
        print("Save Rn Acc Sim:{:.2f}".format(rn_acc_sim_thre))
        Base.ShowTopoMap(acc_full_topo19_rn_rd, rn_acc_tmp_topo, path=rn_acc_sim_fig, vis=False)

        if os.path.isfile(rn_acc_pr):
            tmp_rn_acc_pr_list = pickle.load(open(rn_acc_pr, "rb"))
            rn_acc_pr_dic[rn_acc_sim_thre] = tmp_rn_acc_pr_list
        else:
            tmp_rn_acc_pr_list = []
            for rn_sim_recall in sim_recall_list:
                rn_acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn_rd,
                                                           base_topo=rn_acc_tmp_topo,
                                                           full_topo=acc_full_topo19_rn_rd,
                                                           sim_mat=acc_full_topo19_rn_rd_sim_mat,
                                                           sim=rn_sim_recall,
                                                           top=1, gdis=3.0, topo_num=len(rn_acc_tmp_topo),
                                                           info="Daquan19RN.BuildSim{:.2f}.TopoNum{:d}".format(
                                                               rn_acc_sim_thre, len(rn_acc_tmp_topo)))
                tmp_rn_acc_pr_list.append(rn_acc_tmp_pr)
            pickle.dump(tmp_rn_acc_pr_list, open(rn_acc_pr, "wb"))
            rn_acc_pr_dic[rn_acc_sim_thre] = tmp_rn_acc_pr_list
            # Base.plot_pr(tmp_rn_acc_pr_list)
        print("RN ACC PR:\n" + str(tmp_rn_acc_pr_list))
    if not os.path.isfile(rn_acc_pr_dic_save):
        pickle.dump(rn_acc_pr_dic, open(rn_acc_pr_dic_save, "wb"))
    Base.plot_muliti_pr_acc(rn_acc_pr_dic)
    """

    """ 16 acc
    if os.path.isfile("/home/qh/YES/dlut/acc_16_val.pkl"):
        acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/acc_16_val.pkl", "rb"))
    else:
        acc_full_topo19 = GetAccFullTopo("/home/qh/YES/dlut/Daquan19")
        save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat.pkl"
        acc_full_topo19_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
        acc_full_topo16 = GetAccFullTopo("/home/qh/YES/dlut/Daquan16")
        save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19.pkl"
        acc_full_topo16_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo16)
        set_path = "/home/qh/YES/dlut/Daquan19"
        best_acc = 0.90
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        acc_topo = pickle.load(open(os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(best_acc)), "rb"))
        acc_16_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            acc_16_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                                   base_topo=acc_topo,
                                                   full_topo=acc_full_topo16,
                                                   sim_mat=acc_full_topo16_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(acc_topo),
                                                   info="ACC16")
            if acc_16_pr['precision'] == 0 and acc_16_pr['recall'] == 0:
                continue
            acc_16_pr_list.append(acc_16_pr)
            print(acc_16_pr)
        pickle.dump(acc_16_pr_list, open("/home/qh/YES/dlut/acc_16_val.pkl", "wb"))
        del acc_full_topo19, acc_full_topo19_sim_mat
        del acc_full_topo16, acc_full_topo16_sim_mat
    # Base.plot_pr(acc_16_pr_list)
    """

    """ 16 app
    if os.path.isfile("/home/qh/YES/dlut/app_16_val.pkl"):
        app_16_pr_list = pickle.load(open("/home/qh/YES/dlut/app_16_val.pkl", "rb"))
    else:
        app_full_topo19 = GetAppFullTopo("/home/qh/YES/dlut/Daquan19")
        save_path = "/home/qh/YES/dlut/Daquan19/app_sim_mat.pkl"
        app_full_topo19_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
        app_full_topo16 = GetAppFullTopo("/home/qh/YES/dlut/Daquan16")
        save_path = "/home/qh/YES/dlut/Daquan16/app_sim_mat_to19.pkl"
        app_full_topo16_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo16)
        set_path = "/home/qh/YES/dlut/Daquan19"
        best_app = 0.85
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        app_topo = pickle.load(open(os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(best_app)), "rb"))
        app_16_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            app_16_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                                   base_topo=app_topo,
                                                   full_topo=app_full_topo16,
                                                   sim_mat=app_full_topo16_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(app_topo),
                                                   info="APP16")
            if app_16_pr['precision'] == 0 and app_16_pr['recall'] == 0:
                continue
            app_16_pr_list.append(app_16_pr)
            print(app_16_pr)
        pickle.dump(app_16_pr_list, open("/home/qh/YES/dlut/app_16_val.pkl", "wb"))
        del app_full_topo19, app_full_topo19_sim_mat
        del app_full_topo16, app_full_topo16_sim_mat
    # Base.plot_pr(app_16_pr_list)
    """

    """ 17 acc
    if os.path.isfile("/home/qh/YES/dlut/acc_17_val.pkl"):
        acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/acc_17_val.pkl", "rb"))
    else:
        acc_full_topo19 = GetAccFullTopo("/home/qh/YES/dlut/Daquan19")
        save_path = "/home/qh/YES/dlut/Daquan19/acc_sim_mat.pkl"
        acc_full_topo19_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
        acc_full_topo17 = GetAccFullTopo("/home/qh/YES/dlut/Daquan17")
        save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat_to19.pkl"
        acc_full_topo17_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo17)
        set_path = "/home/qh/YES/dlut/Daquan19"
        best_acc = 0.90
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        acc_topo = pickle.load(open(os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(best_acc)), "rb"))
        acc_17_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            acc_17_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                                   base_topo=acc_topo,
                                                   full_topo=acc_full_topo17,
                                                   sim_mat=acc_full_topo17_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(acc_topo),
                                                   info="ACC17")
            if acc_17_pr['precision'] == 0 and acc_17_pr['recall'] == 0:
                continue
            acc_17_pr_list.append(acc_17_pr)
            print(acc_17_pr)
        pickle.dump(acc_17_pr_list, open("/home/qh/YES/dlut/acc_17_val.pkl", "wb"))
        del acc_full_topo19, acc_full_topo19_sim_mat
        del acc_full_topo17, acc_full_topo17_sim_mat
    # Base.plot_pr(acc_17_pr_list)
    """

    """ 17 app
    if os.path.isfile("/home/qh/YES/dlut/app_17_val.pkl"):
        app_17_pr_list = pickle.load(open("/home/qh/YES/dlut/app_17_val.pkl", "rb"))
    else:
        app_full_topo19 = GetAppFullTopo("/home/qh/YES/dlut/Daquan19")
        save_path = "/home/qh/YES/dlut/Daquan19/app_sim_mat.pkl"
        app_full_topo19_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
        app_full_topo17 = GetAppFullTopo("/home/qh/YES/dlut/Daquan17")
        save_path = "/home/qh/YES/dlut/Daquan17/app_sim_mat_to19.pkl"
        app_full_topo17_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo17)
        set_path = "/home/qh/YES/dlut/Daquan19"
        best_app = 0.85
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        app_topo = pickle.load(open(os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(best_app)), "rb"))
        app_17_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            app_17_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                                   base_topo=app_topo,
                                                   full_topo=app_full_topo17,
                                                   sim_mat=app_full_topo17_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(app_topo),
                                                   info="APP17")
            if app_17_pr['precision'] == 0 and app_17_pr['recall'] == 0:
                continue
            app_17_pr_list.append(app_17_pr)
            print(app_17_pr)
        pickle.dump(app_17_pr_list, open("/home/qh/YES/dlut/app_17_val.pkl", "wb"))
        del app_full_topo19, app_full_topo19_sim_mat
        del app_full_topo17, app_full_topo17_sim_mat
    # Base.plot_pr(app_17_pr_list)
    """

    # """ 16 rn acc
    if os.path.isfile("/home/qh/YES/dlut/rn_acc_16_val.pkl"):
        rn_acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_16_val.pkl", "rb"))
    else:
        acc_full_topo19_rn = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19")
        acc_full_topo16_rn = GetAccFullTopoRemoveNone16("/home/qh/YES/dlut/Daquan16")
        save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19_rn.pkl"
        acc_full_topo16_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, acc_full_topo16_rn)
        set_path = "/home/qh/YES/dlut/Daquan19"
        rn_best_acc = 0.75
        rn_sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60] + np.arange(0.62, 0.92, 0.005).tolist() + [0.95, 0.98, 0.99]
        # rn_sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        rn_acc_topo = pickle.load(
            open(os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.pkl".format(rn_best_acc)), "rb"))
        rn_acc_16_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in rn_sim_recall_list:
            rn_acc_16_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn,
                                                      base_topo=rn_acc_topo,
                                                      full_topo=acc_full_topo16_rn,
                                                      sim_mat=acc_full_topo16_rn_sim_mat,
                                                      sim=sim_recall,
                                                      top=top_, gdis=gdis_, topo_num=len(rn_acc_topo),
                                                      info="RN")
            if rn_acc_16_pr['precision'] == 0 and rn_acc_16_pr['recall'] == 0:
                continue
            rn_acc_16_pr_list.append(rn_acc_16_pr)
            print(rn_acc_16_pr)
        pickle.dump(rn_acc_16_pr_list, open("/home/qh/YES/dlut/rn_acc_16_val.pkl", "wb"))
        del acc_full_topo19_rn
        del acc_full_topo16_rn, acc_full_topo16_rn_sim_mat
    Base.plot_pr(rn_acc_16_pr_list)
    # """

    # """ 17 rn acc
    if os.path.isfile("/home/qh/YES/dlut/rn_acc_17_val.pkl"):
        rn_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_17_val.pkl", "rb"))
    else:
        acc_full_topo19_rn = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19")
        acc_full_topo17_rn = GetAccFullTopoRemoveNone17("/home/qh/YES/dlut/Daquan17")
        save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat_to19_rn.pkl"
        acc_full_topo17_rn_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn, save_path, acc_full_topo17_rn)
        set_path = "/home/qh/YES/dlut/Daquan19"
        rn_best_acc = 0.75
        rn_sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60] + np.arange(0.62, 0.92, 0.005).tolist() + [0.95, 0.98,
                                                                                                             0.99]
        rn_acc_topo = pickle.load(
            open(os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.pkl".format(rn_best_acc)), "rb"))
        rn_acc_17_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in rn_sim_recall_list:
            rn_acc_17_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn,
                                                      base_topo=rn_acc_topo,
                                                      full_topo=acc_full_topo17_rn,
                                                      sim_mat=acc_full_topo17_rn_sim_mat,
                                                      sim=sim_recall,
                                                      top=top_, gdis=gdis_, topo_num=len(rn_acc_topo),
                                                      info="RN")
            if rn_acc_17_pr['precision'] == 0 and rn_acc_17_pr['recall'] == 0:
                continue
            rn_acc_17_pr_list.append(rn_acc_17_pr)
            print(rn_acc_17_pr)
        pickle.dump(rn_acc_17_pr_list, open("/home/qh/YES/dlut/rn_acc_17_val.pkl", "wb"))
        del acc_full_topo19_rn
        del acc_full_topo17_rn, acc_full_topo17_rn_sim_mat
    Base.plot_pr(rn_acc_17_pr_list)
    # """

    # """ 16 RN RD
    if os.path.isfile("/home/qh/YES/dlut/rn_rd_acc_16_val.pkl"):
        rn_rd_acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_rd_acc_16_val.pkl", "rb"))
    else:
        acc_full_topo19_rn_rd = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19", RD=True)
        acc_full_topo16_rn_rd = GetAccFullTopoRemoveNone16("/home/qh/YES/dlut/Daquan16", RD=True)
        save_path = "/home/qh/YES/dlut/Daquan16/acc_sim_mat_to19_rn_rd.pkl"
        acc_full_topo16_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, acc_full_topo16_rn_rd)
        set_path = "/home/qh/YES/dlut/Daquan19"
        rn_best_acc = 0.90
        rn_sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60] + np.arange(0.62, 0.92, 0.005).tolist() + [0.95, 0.98,
                                                                                                             0.99]
        rn_acc_topo = pickle.load(
            open(os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.pkl".format(rn_best_acc)), "rb"))
        rn_rd_acc_16_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in rn_sim_recall_list:
            rn_acc_16_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn_rd,
                                                      base_topo=rn_acc_topo,
                                                      full_topo=acc_full_topo16_rn_rd,
                                                      sim_mat=acc_full_topo16_rn_rd_sim_mat,
                                                      sim=sim_recall,
                                                      top=top_, gdis=gdis_, topo_num=len(rn_acc_topo),
                                                      info="RN")
            if rn_acc_16_pr['precision'] == 0 and rn_acc_16_pr['recall'] == 0:
                continue
            rn_rd_acc_16_pr_list.append(rn_acc_16_pr)
            print(rn_acc_16_pr)
        pickle.dump(rn_rd_acc_16_pr_list, open("/home/qh/YES/dlut/rn_rd_acc_16_val.pkl", "wb"))
        del acc_full_topo19_rn_rd
        del acc_full_topo16_rn_rd, acc_full_topo16_rn_rd_sim_mat
    Base.plot_pr(rn_rd_acc_16_pr_list)
    # """

    # """ 17 RN RD
    if os.path.isfile("/home/qh/YES/dlut/rn_rd_acc_17_val.pkl"):
        rn_rd_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_rd_acc_17_val.pkl", "rb"))
    else:
        acc_full_topo19_rn_rd = GetAccFullTopoRemoveNone19("/home/qh/YES/dlut/Daquan19", RD=True)
        acc_full_topo17_rn_rd = GetAccFullTopoRemoveNone17("/home/qh/YES/dlut/Daquan17", RD=True)
        save_path = "/home/qh/YES/dlut/Daquan17/acc_sim_mat_to19_rn_rd.pkl"
        acc_full_topo17_rn_rd_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19_rn_rd, save_path, acc_full_topo17_rn_rd)
        set_path = "/home/qh/YES/dlut/Daquan19"
        rn_best_acc = 0.90
        rn_sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60] + np.arange(0.62, 0.92, 0.005).tolist() + [0.95, 0.98,
                                                                                                             0.99]
        rn_acc_topo = pickle.load(
            open(os.path.join(set_path, "topo_map/rn_acc_sim_{:.2f}.pkl".format(rn_best_acc)), "rb"))
        rn_rd_acc_17_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in rn_sim_recall_list:
            rn_acc_17_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19_rn_rd,
                                                      base_topo=rn_acc_topo,
                                                      full_topo=acc_full_topo17_rn_rd,
                                                      sim_mat=acc_full_topo17_rn_rd_sim_mat,
                                                      sim=sim_recall,
                                                      top=top_, gdis=gdis_, topo_num=len(rn_acc_topo),
                                                      info="RN")
            if rn_acc_17_pr['precision'] == 0 and rn_acc_17_pr['recall'] == 0:
                continue
            rn_rd_acc_17_pr_list.append(rn_acc_17_pr)
            print(rn_acc_17_pr)
        pickle.dump(rn_rd_acc_17_pr_list, open("/home/qh/YES/dlut/rn_rd_acc_17_val.pkl", "wb"))
        del acc_full_topo19_rn_rd
        del acc_full_topo17_rn_rd, acc_full_topo17_rn_rd_sim_mat
    Base.plot_pr(rn_rd_acc_17_pr_list)
    # """

    # """ RN RD 对比
    if True:
        row_size, col_size = 1, 1
        fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
        title_label = "VS"
        rn_acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_16_val.pkl", "rb"))
        rn_rd_acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_rd_acc_16_val.pkl", "rb"))
        ax1 = plt.subplot(row_size, col_size, 1)
        ax1.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        x, y, area, num = Base.GetPrData(rn_acc_16_pr_list)
        f1 = max([2 * x[i] * y[i] / (x[i] + y[i]) for i in range(len(x))])
        plt.plot(x, y, lw="2", color="#fddf8b", label="raw {:.4f} {:.4f}".format(area, f1), alpha=0.9)
        plt.scatter(x, y, marker="o", s=10, color="black")
        x, y, area, num = Base.GetPrData(rn_rd_acc_16_pr_list)
        f1 = max([2 * x[i] * y[i] / (x[i] + y[i]) for i in range(len(x))])
        plt.plot(x, y, lw="2", color="#0d5b26", label="remove dynamic {:.4f} {:.4f}".format(area, f1), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax1.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("Daquan 16", fontsize=15)
        plt.show()
        plt.close()
    if True:
        row_size, col_size = 1, 1
        fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
        title_label = "VS"
        rn_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_17_val.pkl", "rb"))
        rn_rd_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_rd_acc_17_val.pkl", "rb"))
        ax2 = plt.subplot(row_size, col_size, 1)
        ax2.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        x, y, area, num = Base.GetPrData(rn_acc_17_pr_list)
        f1 = max([2 * x[i] * y[i] / (x[i] + y[i]) for i in range(len(x))])
        plt.plot(x, y, lw="2", color="#fddf8b", label="raw {:.4f} {:.4f}".format(area, f1), alpha=0.9)
        plt.scatter(x, y, marker="o", s=10, color="black")
        f1 = max([2 * x[i] * y[i] / (x[i] + y[i]) for i in range(len(x))])
        x, y, area, num = Base.GetPrData(rn_rd_acc_17_pr_list)
        plt.plot(x, y, lw="2", color="#0d5b26", label="remove dynamic {:.4f} {:.4f}".format(area, f1), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax2.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("Daquan 17", fontsize=15)
        # plt.savefig("/home/qh/YES/dlut/compare.png", dpi=600, transparent=True)
        plt.show()
        plt.close()
    # """

    """ 细节展示
    acc_full_topo19 = GetAccFullTopo("/home/qh/YES/dlut/Daquan19")
    app_full_topo19 = GetAppFullTopo("/home/qh/YES/dlut/Daquan19")
    acc_topo_map = pickle.load(
        open(os.path.join("/home/qh/YES/dlut/Daquan19", "topo_map/acc_sim_{:.2f}.pkl".format(0.90)), "rb"))
    app_topo_map = pickle.load(
        open(os.path.join("/home/qh/YES/dlut/Daquan19", "topo_map/app_sim_{:.2f}.pkl".format(0.85)), "rb"))
    #
    if True:
        fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
        points = np.zeros((3, 0), dtype=np.float32)
        id_list = []
        for ind in range(len(acc_topo_map)):
            node = acc_full_topo19[acc_topo_map[ind]]
            points = np.column_stack((points, node.position))
            id_list.append(ind)
            x = node.position.flatten()[1]
            y = node.position.flatten()[0]
            # if ind % 50 == 0:
            # if 1550 < ind < 1600:
            # if (1536<ind<1550 or 2116<ind<2120) and ind%1==0 :
            #     plt.text(x + 0.2, y + 0.2, '{:d}'.format(ind), fontsize=12)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(points.transpose())
        kdtree = open3d.geometry.KDTreeFlann(pcd)

        # for in []:
        #     now_p = acc_full_topo19[acc_topo_map[ind]].position
        #     aa = kdtree.search_knn_vector_3d(now_p, 3)

        ax.scatter(points[1], points[0], marker="o", s=50, c=id_list, cmap="jet", zorder=5)
        dedic = {1201: 1202, 1202: 1203, 1203: 1204, 1204: 1205,
                 2117: 2118, 2585: 2586}
        for iind in range(len(acc_topo_map)-1):
            if not iind in dedic:
                plt.plot(points[1, iind:iind+2], points[0, iind:iind+2],
                         linewidth=2, markersize=2, color='gray', alpha=0.5, zorder=0)
        exdic = {1201: 674, 1202: 686, 1203: 690, 1204: 693, 1205: 724,
                 2117: 1537, 1548: 2585, 1549: 2586, 1578:2118}
        for iind, key in enumerate(exdic):
            tmx = [acc_full_topo19[acc_topo_map[key]].position.flatten()[1],
                   acc_full_topo19[acc_topo_map[exdic[key]]].position.flatten()[1]]
            tmy = [acc_full_topo19[acc_topo_map[key]].position.flatten()[0],
                   acc_full_topo19[acc_topo_map[exdic[key]]].position.flatten()[0]]
            plt.plot(tmx, tmy, linewidth=2, markersize=2, color="gray", alpha=0.5, zorder=1)
        ax.set_aspect(1)
        ax.legend(loc='upper left')
        # ax.get_xaxis().set_visible(False)
        # ax.get_yaxis().set_visible(False)
        plt.xlim(-640, -520), plt.ylim(-120, -80)
        # plt.xlim(-920, -850), plt.ylim(70, 220)
        # plt.savefig(path, dpi=600, transparent=True)
        plt.show()
        plt.close()

    #
    if True:
        fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
        points = np.zeros((3, 0), dtype=np.float32)
        id_list = []
        for ind in range(len(app_topo_map)):
            node = app_full_topo19[app_topo_map[ind]]
            points = np.column_stack((points, node.position))
            id_list.append(ind)
            x = node.position.flatten()[1]
            y = node.position.flatten()[0]
            # if ind % 50 == 0:
            # if 1550 < ind < 1600:
            # if (1536<ind<1550 or 2116<ind<2120) and ind%1==0 :
            #     plt.text(x + 0.2, y + 0.2, '{:d}'.format(ind), fontsize=12)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(points.transpose())
        kdtree = open3d.geometry.KDTreeFlann(pcd)

        # for in []:
        #     now_p = app_full_topo19[app_topo_map[ind]].position
        #     aa = kdtree.search_knn_vector_3d(now_p, 3)

        ax.scatter(points[1], points[0], marker="o", s=50, c=id_list, cmap="jet", zorder=5)
        dedic = {}
        for iind in range(len(app_topo_map)-1):
            if not iind in dedic:
                plt.plot(points[1, iind:iind+2], points[0, iind:iind+2],
                         linewidth=2, markersize=2, color='gray', alpha=0.5, zorder=0)
                # continue
        exdic = {}
        for iind, key in enumerate(exdic):
            tmx = [app_full_topo19[app_topo_map[key]].position.flatten()[1],
                   app_full_topo19[app_topo_map[exdic[key]]].position.flatten()[1]]
            tmy = [app_full_topo19[app_topo_map[key]].position.flatten()[0],
                   app_full_topo19[app_topo_map[exdic[key]]].position.flatten()[0]]
            plt.plot(tmx, tmy, linewidth=2, markersize=2, color="gray", alpha=0.5, zorder=1)
        ax.set_aspect(1)
        ax.legend(loc='upper left')
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)
        plt.xlim(-640, -520), plt.ylim(-120, -80)
        # plt.xlim(-920, -850), plt.ylim(70, 220)
        # plt.savefig(path, dpi=600, transparent=True)
        plt.show()
        plt.close()

    # Base.ShowTopoMapByDensity(acc_full_topo19, acc_topo_map, path="/home/qh/毕业/实验资料/Daquan19-OUR拓扑地图-密度颜色-100米比例尺.png")
    # Base.ShowTopoMapByDensity(app_full_topo19, app_topo_map, path="/home/qh/毕业/实验资料/Daquan19-APP拓扑地图-密度颜色-100米比例尺.png")
    """

    """ 对比图
    row_size, col_size = 1, 1
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "VS"
    if False:
        app_16_pr_list = pickle.load(open("/home/qh/YES/dlut/app_16_val.pkl", "rb"))
        acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/acc_16_val.pkl", "rb"))
        rn_acc_16_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_16_val.pkl", "rb"))
        ax1 = plt.subplot(row_size, col_size, 1)
        ax1.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        # x, y, area, num = Base.GetPrData(app_16_pr_list)
        # plt.plot(x, y, lw="2", color="#52b9d8", label="app {:d} {:.5f}".format(num, area), alpha=0.9)
        # plt.scatter(x, y, marker="^", s=10, color="black")
        x, y, area, num = Base.GetPrData(rn_acc_16_pr_list)
        plt.plot(x, y, lw="2", color="#fddf8b", label="raw".format(num, area), alpha=0.9)
        # plt.plot(x, y, lw="2", color="#fddf8b", label="rn acc {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="o", s=10, color="black")
        x, y, area, num = Base.GetPrData(acc_16_pr_list)
        plt.plot(x, y, lw="2", color="#0d5b26", label="remove dynamic".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax1.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("Daquan 16", fontsize=15)
    if True:
        app_17_pr_list = pickle.load(open("/home/qh/YES/dlut/app_17_val.pkl", "rb"))
        acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/acc_17_val.pkl", "rb"))
        rn_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_acc_17_val.pkl", "rb"))
        # rn_rd_acc_17_pr_list = pickle.load(open("/home/qh/YES/dlut/rn_rd_acc_17_val.pkl", "rb"))
        ax2 = plt.subplot(row_size, col_size, 1)
        ax2.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        # x, y, area, num = Base.GetPrData(app_17_pr_list)
        # plt.plot(x, y, lw="2", color="#52b9d8", label="app {:d} {:.5f}".format(num, area), alpha=0.9)
        # plt.scatter(x, y, marker="^", s=10, color="black")
        x, y, area, num = Base.GetPrData(rn_acc_17_pr_list)
        plt.plot(x, y, lw="2", color="#fddf8b", label="raw".format(num, area), alpha=0.9)
        # plt.plot(x, y, lw="2", color="#fddf8b", label="rn acc {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="o", s=10, color="black")
        x, y, area, num = Base.GetPrData(acc_17_pr_list)
        plt.plot(x, y, lw="2", color="#0d5b26", label="remove dynamic".format(num, area), alpha=0.9)
        # plt.plot(x, y, lw="2", color="#0d5b26", label="acc {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax2.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("Daquan 17", fontsize=15)
    plt.savefig("/home/qh/YES/dlut/compare.png", dpi=600, transparent=True)
    plt.show()
    plt.close()
    print(123)
    """

import os
import time
import matplotlib.pyplot as plt
import numpy as np
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
    # path = "/home/dlut/zqh/dlut/Daquan19"
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
    pose_vec_data16 = Base.GetPoseVec("/home/dlut/zqh/dlut/Daquan16/liosave/sam2.txt")
    pose_vec_data17 = Base.GetPoseVec("/home/dlut/zqh/dlut/Daquan17/liosave/sam2.txt")
    pose_vec_data19 = Base.GetPoseVec("/home/dlut/zqh/dlut/Daquan19/liosave/sam2.txt")
    """

    """
    # # Completion the half connect matrix 补全上三角矩阵
    Base.TopoConnectCompletion("/home/dlut/zqh/dlut/Daquan19/acc_connect.pkl")
    Base.TopoConnectCompletion("/home/dlut/zqh/dlut/Daquan19/app_connect.pkl")
    """

    # """
    # Daquan19 的 累积19 和 外观19 的完整拓扑节点生成 并生成相似度矩阵
    # 19 累积
    acc_full_topo19 = GetAccFullTopo("/home/dlut/zqh/dlut/Daquan19")
    # save_path = "/home/dlut/zqh/dlut/Daquan19/acc_sim_mat.pkl"
    # acc_full_topo19_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
    # 19 外观
    app_full_topo19 = GetAppFullTopo("/home/dlut/zqh/dlut/Daquan19")
    # save_path = "/home/dlut/zqh/dlut/Daquan19/app_sim_mat.pkl"
    # app_full_topo19_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
    # """

    # """
    # Daqaun16 用于验证测试 首先生成 fullnode 然后针对 累积19 外观19 生成相似度矩阵
    # 16 累积
    acc_full_topo16 = GetAccFullTopo("/home/dlut/zqh/dlut/Daquan16")
    # save_path = "/home/dlut/zqh/dlut/Daquan16/acc_sim_mat.pkl"
    # acc_full_topo16_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo16)
    # 16 外观
    app_full_topo16 = GetAppFullTopo("/home/dlut/zqh/dlut/Daquan16")
    # save_path = "/home/dlut/zqh/dlut/Daquan16/app_sim_mat.pkl"
    # app_full_topo16_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo16)
    # """

    # """
    # Daqaun17 用于验证测试 首先生成 fullnode 然后针对 累积19 外观19 生成相似度矩阵

    # 17 累积
    acc_full_topo17 = GetAccFullTopo("/home/dlut/zqh/dlut/Daquan17")
    # save_path = "/home/dlut/zqh/dlut/Daquan17/acc_sim_mat.pkl"
    # acc_full_topo17_sim_mat = Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo17)

    # 17 外观
    app_full_topo17 = GetAppFullTopo("/home/dlut/zqh/dlut/Daquan17")
    # save_path = "/home/dlut/zqh/dlut/Daquan17/app_sim_mat.pkl"
    # app_full_topo17_sim_mat = Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo17)
    # """

    """
    # 并生成若干阈值拓扑地图 并找到最优参数 进行检验
    set_path = "/home/dlut/zqh/dlut/Daquan19"
    sim_list = [0.70, 0.75, 0.78, 0.80, 0.83, 0.85, 0.87, 0.88, 0.89, 0.90,
                0.91, 0.92, 0.93, 0.94, 0.95]
    sim_recall_list = [0.1, 0.2, 0.3, 0.4, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    top_, gdis_ = 10, 5.0
    for sim_thre in sim_list:
        acc_sim = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(sim_thre))
        app_sim = os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(sim_thre))
        acc_sim_fig = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.png".format(sim_thre))
        app_sim_fig = os.path.join(set_path, "topo_map/app_sim_{:.2f}.png".format(sim_thre))
        acc_pr = os.path.join(set_path, "topo_map/acc_sim_{:.2f}_pr.pkl".format(sim_thre))
        app_pr = os.path.join(set_path, "topo_map/app_sim_{:.2f}_pr.pkl".format(sim_thre))
        acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo19,
                                             sim_mat=acc_full_topo19_sim_mat,
                                             sim_threshold=sim_thre,
                                             path=acc_sim)
        app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_full_topo19,
                                             sim_mat=app_full_topo19_sim_mat,
                                             sim_threshold=sim_thre,
                                             path=app_sim)
        print("Save Sim:{:.2f}".format(sim_thre))
        Base.ShowTopoMap(acc_full_topo19, acc_tmp_topo, path=acc_sim_fig, vis=False)
        Base.ShowTopoMap(app_full_topo19, app_tmp_topo, path=app_sim_fig, vis=False)

        tmp_acc_pr_list = []
        tmp_app_pr_list = []
        for sim_recall in sim_recall_list:
            acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                                    base_topo=acc_tmp_topo,
                                                    full_topo=acc_full_topo19,
                                                    sim_mat=acc_full_topo19_sim_mat,
                                                    sim=sim_recall,
                                                    top=top_, gdis=gdis_)
            app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                                    base_topo=app_tmp_topo,
                                                    full_topo=app_full_topo19,
                                                    sim_mat=app_full_topo19_sim_mat,
                                                    sim=sim_recall,
                                                    top=top_, gdis=gdis_)
            tmp_acc_pr_list.append(acc_tmp_pr)
            tmp_app_pr_list.append(app_tmp_pr)
        pickle.dump(tmp_acc_pr_list, open(acc_pr, "wb"))
        pickle.dump(tmp_app_pr_list, open(app_pr, "wb"))
    best_acc = 0.90
    best_app = 0.90
    acc_topo = pickle.load(open(os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(best_acc))))
    app_topo = pickle.load(open(os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(best_app))))
    acc_16_pr_list = []
    acc_17_pr_list = []
    app_16_pr_list = []
    app_17_pr_list = []
    for sim_recall in sim_recall_list:
        acc_16_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                               base_topo=acc_topo,
                                               full_topo=acc_full_topo16,
                                               sim_mat=acc_full_topo16_sim_mat,
                                               sim=sim_recall,
                                               top=top_, gdis=gdis_)
        acc_17_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo19,
                                               base_topo=acc_topo,
                                               full_topo=acc_full_topo17,
                                               sim_mat=acc_full_topo17_sim_mat,
                                               sim=sim_recall,
                                               top=top_, gdis=gdis_)
        app_16_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                               base_topo=app_topo,
                                               full_topo=app_full_topo16,
                                               sim_mat=app_full_topo16_sim_mat,
                                               sim=sim_recall,
                                               top=top_, gdis=gdis_)
        app_17_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo19,
                                               base_topo=app_topo,
                                               full_topo=app_full_topo17,
                                               sim_mat=app_full_topo17_sim_mat,
                                               sim=sim_recall,
                                               top=top_, gdis=gdis_)
        acc_16_pr_list.append(acc_16_pr)
        acc_17_pr_list.append(acc_16_pr)
        app_16_pr_list.append(app_16_pr)
        app_17_pr_list.append(app_17_pr)
    pickle.dump(acc_16_pr_list, open("/home/dlut/zqh/dlut/acc_16_val.pkl", "wb"))
    pickle.dump(acc_17_pr_list, open("/home/dlut/zqh/dlut/acc_17_val.pkl", "wb"))
    pickle.dump(app_16_pr_list, open("/home/dlut/zqh/dlut/app_16_val.pkl", "wb"))
    pickle.dump(app_17_pr_list, open("/home/dlut/zqh/dlut/app_17_val.pkl", "wb"))
    """

    print(123)

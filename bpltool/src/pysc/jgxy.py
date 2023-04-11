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
1.经管学院两圈地图生成、障碍物去除、拓扑数据的生成
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
        pose_ind = np.argmin(abs(pose_vec[:, 0] - now_time))
        if abs(now_time - pose_vec[:, 0][pose_ind]) > 0.05:
            continue
        p_ = pose_vec[pose_ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[pose_ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        bin_info.append((now_time, pose_ind, bin_file, p_, r_))
    if save_path is not None:
        pickle.dump(bin_info, open(save_path, "wb"))
    return bin_info


# 获取某个数据包的bin和位姿，利用总体点云地图PCD来建立voxelmap并删除动态障碍
def GetAccFullTopo(path):
    # path = "/home/qh/YES/jgxy/jgxy1"
    acc_full_topo_path = os.path.join(path, "acc_full_topo.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))[::500]
        Base.SetParameter(nscan=32, hscan=2048,
                          low_ang=-22.5, up_ang=22.5,
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
        Base.plot_multiple_sc(tmp_topo_node.SCs)
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
        # open3d.visualization.draw_geometries([pcd, axis_pcd])
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save Acc Topo Node!")
    return acc_full_topo


def GetAccFullTopoRemoveNone(path):
    # path = "/home/qh/YES/jgxy/jgxy1"
    acc_full_topo_path = os.path.join(path, "acc_full_topo_rn.pkl")
    if os.path.isfile(acc_full_topo_path):
        acc_full_topo = pickle.load(open(acc_full_topo_path, "rb"))
        return acc_full_topo
    if os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        Base.SetParameter(nscan=32, hscan=2048,
                          low_ang=-22.5, up_ang=22.5,
                          ground_ind=15, min_range=5.0,
                          segment_theta_d=40.0,
                          down_lines=-1, up_lines=-1)
        map1x = Base.VoxelMap3_Load(save_path=path)
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
            print(
                "Gen RN Acc Topo Node {:.2f}% Cost {:.2f}s".format(ind / handle_size * 100, ttime.time() - start_time))
            start_time = ttime.time()
        # Base.plot_multiple_sc(tmp_topo_node.SCs)
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
        # open3d.visualization.draw_geometries([pcd, axis_pcd])
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
    app_full_topo = []
    for n_time, ind, bin_file, p_, r_ in bin_list:
        points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        points = points[~np.isnan(points).any(axis=1)]
        boundary = Base.getBound(points)
        tmp_topo_node = Base.genTopoSC(Base.TopoNode(ind, p_, r_, boundary, n_time), points, ch=3)
        app_full_topo.append(tmp_topo_node)
    if len(app_full_topo) != 0:
        pickle.dump(app_full_topo, open(app_full_topo_path, "wb"))
        print("Save APP Topo Node!")
    return app_full_topo


if __name__ == "__main__":

    # acc_full_j1_rn = GetAccFullTopoRemoveNone("/home/qh/YES/jgxy/jgxy1")
    # save_path = "/home/qh/YES/jgxy/jgxy1/acc_sim_mat_rn.pkl"
    # acc_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(acc_full_j1, save_path)

    # acc_full_j2_rn = GetAccFullTopoRemoveNone("/home/qh/YES/jgxy/jgxy2")
    # save_path = "/home/qh/YES/jgxy/jgxy1/acc_sim_mat_rn.pkl"
    # acc_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(acc_full_j1, save_path)

    # JGXY1 ACC
    # acc_full_j1 = GetAccFullTopo("/home/qh/YES/jgxy/jgxy1")
    # save_path = "/home/qh/YES/jgxy/jgxy1/acc_sim_mat.pkl"
    # acc_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(acc_full_j1, save_path)

    # JGXY1 APP
    # app_full_j1 = GetAppFullTopo("/home/qh/YES/jgxy/jgxy1")
    # save_path = "/home/qh/YES/jgxy/jgxy1/app_sim_mat.pkl"
    # app_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(app_full_j1, save_path)

    # JGXY2 ACC
    # acc_full_j2 = GetAccFullTopo("/home/qh/YES/jgxy/jgxy2")
    # save_path = "/home/qh/YES/jgxy/jgxy2/acc_sim_mat.pkl"
    # acc_full_topo_jgxy2_sim_mat = Base.GetSimMatrixTo19(acc_full_j2, save_path)

    # JGXY2 APP
    # app_full_j2 = GetAppFullTopo("/home/qh/YES/jgxy/jgxy2")
    # save_path = "/home/qh/YES/jgxy/jgxy2/app_sim_mat.pkl"
    # app_full_topo_jgxy2_sim_mat = Base.GetSimMatrixTo19(app_full_j2, save_path)
    print(123)

    """
    # # Completion the half connect matrix 补全上三角矩阵
    # Base.TopoConnectCompletion("/home/qh/YES/jgxy/jgxy1/acc_sim_rnmat.pkl")
    Base.TopoConnectCompletion("/home/qh/YES/jgxy/jgxy1/acc_sim_mat.pkl")
    Base.TopoConnectCompletion("/home/qh/YES/jgxy/jgxy1/app_sim_mat.pkl")
    """

    """
    # ACC 建图整定
    acc_full_j1 = GetAccFullTopo("/home/qh/YES/jgxy/jgxy1")
    save_path = "/home/qh/YES/jgxy/jgxy1/acc_sim_mat.pkl"
    acc_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(acc_full_j1, save_path)
    set_path = "/home/qh/YES/jgxy/jgxy1"
    acc_sim_list = [0.40, 0.50, 0.60, 0.65, 0.70, 0.73, 0.75, 0.78, 0.80, 0.83, 0.85, 0.88, 0.90, 0.92, 0.95]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    acc_pr_dic_save = os.path.join(set_path, "topo_map/acc_total_pr.pkl")
    acc_pr_dic = {}
    for acc_sim_thre in acc_sim_list:
        acc_sim = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(acc_sim_thre))
        acc_sim_fig = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.png".format(acc_sim_thre))
        acc_pr = os.path.join(set_path, "topo_map/acc_sim_{:.2f}_pr.pkl".format(acc_sim_thre))
        acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_j1,
                                             sim_mat=acc_full_topo_jgxy1_sim_mat,
                                             sim_threshold=acc_sim_thre,
                                             path=acc_sim)
        print("Save Acc Sim:{:.2f}".format(acc_sim_thre))
        Base.ShowTopoMap(acc_full_j1, acc_tmp_topo, path=acc_sim_fig, vis=False)

        if os.path.isfile(acc_pr):
            tmp_acc_pr_list = pickle.load(open(acc_pr, "rb"))
        else:
            tmp_acc_pr_list = []
            for sim_recall in sim_recall_list:
                acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_j1,
                                                        base_topo=acc_tmp_topo,
                                                        full_topo=acc_full_j1,
                                                        sim_mat=acc_full_topo_jgxy1_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(acc_tmp_topo),
                                                        info="JGXY1.BuildSim{:.2f}.TopoNum{:d}".format(acc_sim_thre, len(acc_tmp_topo)))

                tmp_acc_pr_list.append(acc_tmp_pr)
            pickle.dump(tmp_acc_pr_list, open(acc_pr, "wb"))
        acc_pr_dic[acc_sim_thre] = tmp_acc_pr_list
        print("ACC PR:\n" + str(tmp_acc_pr_list))
    if not os.path.isfile(acc_pr_dic_save):
        pickle.dump(acc_pr_dic, open(acc_pr_dic_save, "wb"))
    Base.plot_muliti_pr_acc(acc_pr_dic)
    """

    """ APP 建图整定
    app_full_j1 = GetAppFullTopo("/home/qh/YES/jgxy/jgxy1")
    save_path = "/home/qh/YES/jgxy/jgxy1/app_sim_mat.pkl"
    app_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(app_full_j1, save_path)
    set_path = "/home/qh/YES/jgxy/jgxy1"
    app_sim_list = [0.40, 0.45, 0.50, 0.55, 0.58, 0.60, 0.62, 0.65, 0.68, 0.70, 0.75, 0.80, 0.85, 0.90]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    app_pr_dic_save = os.path.join(set_path, "topo_map/app_total_pr.pkl")
    app_pr_dic = {}
    for app_sim_thre in app_sim_list:
        app_sim = os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(app_sim_thre))
        app_sim_fig = os.path.join(set_path, "topo_map/app_sim_{:.2f}.png".format(app_sim_thre))
        app_pr = os.path.join(set_path, "topo_map/app_sim_{:.2f}_pr.pkl".format(app_sim_thre))
        app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_full_j1,
                                             sim_mat=app_full_topo_jgxy1_sim_mat,
                                             sim_threshold=app_sim_thre,
                                             path=app_sim)
        print("Save App Sim:{:.2f}".format(app_sim_thre))
        Base.ShowTopoMap(app_full_j1, app_tmp_topo, path=app_sim_fig, vis=False)

        if os.path.isfile(app_pr):
            tmp_app_pr_list = pickle.load(open(app_pr, "rb"))
        else:
            tmp_app_pr_list = []
            for sim_recall in sim_recall_list:
                app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_j1,
                                                        base_topo=app_tmp_topo,
                                                        full_topo=app_full_j1,
                                                        sim_mat=app_full_topo_jgxy1_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(app_tmp_topo),
                                                        info="JGXY1.BuildSim{:.2f}.TopoNum{:d}".format(app_sim_thre, len(app_tmp_topo)))

                tmp_app_pr_list.append(app_tmp_pr)
            pickle.dump(tmp_app_pr_list, open(app_pr, "wb"))
        app_pr_dic[app_sim_thre] = tmp_app_pr_list
        print("APP PR:\n" + str(tmp_app_pr_list))
    if not os.path.isfile(app_pr_dic_save):
        pickle.dump(app_pr_dic, open(app_pr_dic_save, "wb"))
    Base.plot_muliti_pr_app(app_pr_dic)
    """

    # """ J2 acc
    if os.path.isfile("/home/qh/YES/jgxy/acc_J2_val.pkl"):
        acc_J2_pr_list = pickle.load(open("/home/qh/YES/jgxy/acc_J2_val.pkl", "rb"))
    else:
        acc_full_j1 = GetAccFullTopo("/home/qh/YES/jgxy/jgxy1")
        save_path = "/home/qh/YES/jgxy/jgxy1/acc_sim_mat.pkl"
        acc_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(acc_full_j1, save_path)
        acc_full_j2 = GetAccFullTopo("/home/qh/YES/jgxy/jgxy2")
        save_path = "/home/qh/YES/jgxy/jgxy2/acc_sim_mat_toJ1.pkl"
        acc_full_topo_jgxy2_sim_mat = Base.GetSimMatrixTo19(acc_full_j2, save_path)
        set_path = "/home/qh/YES/jgxy/jgxy1"
        best_acc = 0.80
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        acc_topo = pickle.load(open(os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(best_acc)), "rb"))
        acc_J2_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            acc_J2_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_j1,
                                                   base_topo=acc_topo,
                                                   full_topo=acc_full_j2,
                                                   sim_mat=acc_full_topo_jgxy2_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(acc_topo),
                                                   info="ACCJ2")
            if acc_J2_pr['precision'] == 0 and acc_J2_pr['recall'] == 0:
                continue
            acc_J2_pr_list.append(acc_J2_pr)
            print(acc_J2_pr)
        pickle.dump(acc_J2_pr_list, open("/home/qh/YES/jgxy/acc_J2_val.pkl", "wb"))
        del acc_full_j1, acc_full_topo_jgxy1_sim_mat
        del acc_full_j2, acc_full_topo_jgxy2_sim_mat
    # Base.plot_pr(acc_J2_pr_list)
    # """

    # """ J2 app
    if os.path.isfile("/home/qh/YES/jgxy/app_J2_val.pkl"):
        app_J2_pr_list = pickle.load(open("/home/qh/YES/jgxy/app_J2_val.pkl", "rb"))
    else:
        app_full_j1 = GetAppFullTopo("/home/qh/YES/jgxy/jgxy1")
        save_path = "/home/qh/YES/jgxy/jgxy1/app_sim_mat.pkl"
        app_full_topo_jgxy1_sim_mat = Base.GetSimMatrixTo19(app_full_j1, save_path)
        app_full_j2 = GetAppFullTopo("/home/qh/YES/jgxy/jgxy2")
        save_path = "/home/qh/YES/jgxy/jgxy2/app_sim_mat_toJ1.pkl"
        app_full_topo_jgxy2_sim_mat = Base.GetSimMatrixTo19(app_full_j2, save_path)
        set_path = "/home/qh/YES/jgxy/jgxy1"
        best_app = 0.75
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        app_topo = pickle.load(open(os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(best_app)), "rb"))
        app_J2_pr_list = []
        top_, gdis_ = 10, 5.0
        for sim_recall in sim_recall_list:
            app_J2_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_j1,
                                                   base_topo=app_topo,
                                                   full_topo=app_full_j2,
                                                   sim_mat=app_full_topo_jgxy2_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(app_topo),
                                                   info="APPJ2")
            if app_J2_pr['precision'] == 0 and app_J2_pr['recall'] == 0:
                continue
            app_J2_pr_list.append(app_J2_pr)
            print(app_J2_pr)
        pickle.dump(app_J2_pr_list, open("/home/qh/YES/jgxy/app_J2_val.pkl", "wb"))
        del app_full_j1, app_full_topo_jgxy1_sim_mat
        del app_full_j2, app_full_topo_jgxy2_sim_mat
    # Base.plot_pr(app_J2_pr_list)
    # """

    # """ 对比图
    row_size, col_size = 1, 1
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "VS"
    if True:
        app_J2_pr_list = pickle.load(open("/home/qh/YES/jgxy/app_J2_val.pkl", "rb"))
        acc_J2_pr_list = pickle.load(open("/home/qh/YES/jgxy/acc_J2_val.pkl", "rb"))
        ax1 = plt.subplot(row_size, col_size, 1)
        ax1.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        x, y, area, num = Base.GetPrData(app_J2_pr_list)
        plt.plot(x, y, lw="2", color="#52b9d8", label="app {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="^", s=10, color="black")
        # x, y, area, num = Base.GetPrData(rn_acc_J2_pr_list)
        # plt.plot(x, y, lw="2", color="#fddf8b", label="rn acc {:d} {:.5f}".format(num, area), alpha=0.9)
        # plt.scatter(x, y, marker="o", s=10, color="black")
        x, y, area, num = Base.GetPrData(acc_J2_pr_list)
        plt.plot(x, y, lw="2", color="#0d5b26", label="acc {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax1.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("JGXY2", fontsize=15)
    plt.savefig("/home/qh/YES/jgxy/compare.png", dpi=600, transparent=True)
    plt.show()
    plt.close()
    print(123)
    # """


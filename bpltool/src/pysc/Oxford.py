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
def GetAccFullTopoRemoveNone(path, s, e):
    # path = "/home/qh/YES/oxford/o1"
    acc_full_topo_path = os.path.join(path, "acc_full_topo_rn.pkl")
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
    if len(acc_full_topo) != 0:
        pickle.dump(acc_full_topo, open(acc_full_topo_path, "wb"))
        print("Save RN Acc Topo Node!")
    return acc_full_topo


# 获取某个数据包的bin和位姿，利用总体点云地图PCD来建立voxelmap并删除动态障碍
def GetAccFullTopo(path, s, e):
    # path = "/home/qh/YES/oxford/o1"
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

    # acc_o1_full_topo_rn = GetAccFullTopoRemoveNone("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    # acc_o2_full_topo_rn = GetAccFullTopoRemoveNone("/home/qh/YES/oxford/o2", o1_start_time, o1_end_time)

    """
    # o1 累积
    # acc_o1_full_topo = GetAccFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    # save_path = "/home/qh/YES/oxford/o1/acc_sim_mat.pkl"
    # acc_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path)

    # o1 外观
    # app_o1_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    # save_path = "/home/qh/YES/oxford/o1/app_sim_mat.pkl"
    # app_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path)
    """

    """
    # o2 累积
    # acc_o2_full_topo = GetAccFullTopo("/home/qh/YES/oxford/o2", o2_start_time, o2_end_time)
    # save_path = "/home/qh/YES/oxford/o2/acc_sim_mat.pkl"
    # acc_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path, acc_o2_full_topo)

    # o2 外观
    # app_o2_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o2", o2_start_time, o2_end_time)
    # save_path = "/home/qh/YES/oxford/o2/app_sim_mat.pkl"
    # app_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path, app_o2_full_topo)
    """

    """
    # ACC 建图整定
    acc_o1_full_topo = GetAccFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    save_path = "/home/qh/YES/oxford/o1/acc_sim_mat.pkl"
    acc_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path)
    set_path = "/home/qh/YES/oxford/o1"
    acc_sim_list = [0.40, 0.50, 0.60, 0.65, 0.70, 0.73, 0.75, 0.78, 0.80, 0.83, 0.85, 0.88, 0.90, 0.92, 0.95]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    acc_pr_dic_save = os.path.join(set_path, "topo_map/acc_total_pr.pkl")
    acc_pr_dic = {}
    for acc_sim_thre in acc_sim_list:
        acc_sim = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(acc_sim_thre))
        acc_sim_fig = os.path.join(set_path, "topo_map/acc_sim_{:.2f}.png".format(acc_sim_thre))
        acc_pr = os.path.join(set_path, "topo_map/acc_sim_{:.2f}_pr.pkl".format(acc_sim_thre))
        acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_o1_full_topo,
                                             sim_mat=acc_full_topo_o1_sim_mat,
                                             sim_threshold=acc_sim_thre,
                                             path=acc_sim)
        print("Save Acc Sim:{:.2f}".format(acc_sim_thre))
        Base.ShowTopoMap(acc_o1_full_topo, acc_tmp_topo, path=acc_sim_fig, vis=False)

        if os.path.isfile(acc_pr):
            tmp_acc_pr_list = pickle.load(open(acc_pr, "rb"))
        else:
            tmp_acc_pr_list = []
            for sim_recall in sim_recall_list:
                acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_o1_full_topo,
                                                        base_topo=acc_tmp_topo,
                                                        full_topo=acc_o1_full_topo,
                                                        sim_mat=acc_full_topo_o1_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(acc_tmp_topo),
                                                        info="Oxford1.BuildSim{:.2f}.TopoNum{:d}".format(acc_sim_thre, len(acc_tmp_topo)))

                tmp_acc_pr_list.append(acc_tmp_pr)
            pickle.dump(tmp_acc_pr_list, open(acc_pr, "wb"))
        acc_pr_dic[acc_sim_thre] = tmp_acc_pr_list
        print("ACC PR:\n" + str(tmp_acc_pr_list))
    if not os.path.isfile(acc_pr_dic_save):
        pickle.dump(acc_pr_dic, open(acc_pr_dic_save, "wb"))
    Base.plot_muliti_pr_acc(acc_pr_dic)
    """  # 0.88

    """ APP 建图整定
    app_o1_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
    save_path = "/home/qh/YES/oxford/o1/app_sim_mat.pkl"
    app_full_topo_o1_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path)
    set_path = "/home/qh/YES/oxford/o1"
    app_sim_list = [0.40, 0.45, 0.50, 0.55, 0.58, 0.60, 0.62, 0.65, 0.68, 0.70, 0.75, 0.80, 0.81, 0.82, 0.83, 0.85, 0.90]
    sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    app_pr_dic_save = os.path.join(set_path, "topo_map/app_total_pr.pkl")
    app_pr_dic = {}
    for app_sim_thre in app_sim_list:
        app_sim = os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(app_sim_thre))
        app_sim_fig = os.path.join(set_path, "topo_map/app_sim_{:.2f}.png".format(app_sim_thre))
        app_pr = os.path.join(set_path, "topo_map/app_sim_{:.2f}_pr.pkl".format(app_sim_thre))
        app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_o1_full_topo,
                                             sim_mat=app_full_topo_o1_sim_mat,
                                             sim_threshold=app_sim_thre,
                                             path=app_sim)
        print("Save App Sim:{:.2f}".format(app_sim_thre))
        Base.ShowTopoMap(app_o1_full_topo, app_tmp_topo, path=app_sim_fig, vis=False)

        if os.path.isfile(app_pr):
            tmp_app_pr_list = pickle.load(open(app_pr, "rb"))
        else:
            tmp_app_pr_list = []
            for sim_recall in sim_recall_list:
                app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_o1_full_topo,
                                                        base_topo=app_tmp_topo,
                                                        full_topo=app_o1_full_topo,
                                                        sim_mat=app_full_topo_o1_sim_mat,
                                                        sim=sim_recall,
                                                        top=1, gdis=3.0, topo_num=len(app_tmp_topo),
                                                        info="Oxford1.BuildSim{:.2f}.TopoNum{:d}".format(app_sim_thre,
                                                                                                       len(app_tmp_topo)))

                tmp_app_pr_list.append(app_tmp_pr)
            pickle.dump(tmp_app_pr_list, open(app_pr, "wb"))
        app_pr_dic[app_sim_thre] = tmp_app_pr_list
        print("APP PR:\n" + str(tmp_app_pr_list))
    if not os.path.isfile(app_pr_dic_save):
        pickle.dump(app_pr_dic, open(app_pr_dic_save, "wb"))
    Base.plot_muliti_pr_app(app_pr_dic)
    """  # 0.83

    """ O2 acc
    if os.path.isfile("/home/qh/YES/oxford/acc_o2_val.pkl"):
        acc_o2_pr_list = pickle.load(open("/home/qh/YES/oxford/acc_o2_val.pkl", "rb"))
    else:
        acc_o1_full_topo = GetAccFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
        acc_o2_full_topo = GetAccFullTopo("/home/qh/YES/oxford/o2", o2_start_time, o2_end_time)
        save_path = "/home/qh/YES/oxford/o2/acc_sim_mat_toO1.pkl"
        acc_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(acc_o1_full_topo, save_path, acc_o2_full_topo)
        set_path = "/home/qh/YES/oxford/o1"
        best_acc = 0.88
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        acc_topo = pickle.load(open(os.path.join(set_path, "topo_map/acc_sim_{:.2f}.pkl".format(best_acc)), "rb"))
        acc_o2_pr_list = []
        top_, gdis_ = 10, 8.0
        for sim_recall in sim_recall_list:
            acc_o2_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_o1_full_topo,
                                                   base_topo=acc_topo,
                                                   full_topo=acc_o2_full_topo,
                                                   sim_mat=acc_full_topo_o2_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(acc_topo),
                                                   info="ACCo2")
            if acc_o2_pr['precision'] == 0 and acc_o2_pr['recall'] == 0:
                continue
            acc_o2_pr_list.append(acc_o2_pr)
            print(acc_o2_pr)
        pickle.dump(acc_o2_pr_list, open("/home/qh/YES/oxford/acc_o2_val.pkl", "wb"))
        del acc_o1_full_topo
        del acc_o2_full_topo, acc_full_topo_o2_sim_mat
    # Base.plot_pr(acc_o2_pr_list)
    """

    """ o2 app
    if os.path.isfile("/home/qh/YES/oxford/app_o2_val.pkl"):
        app_o2_pr_list = pickle.load(open("/home/qh/YES/oxford/app_o2_val.pkl", "rb"))
    else:
        app_o1_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o1", o1_start_time, o1_end_time)
        app_o2_full_topo = GetAppFullTopo("/home/qh/YES/oxford/o2", o2_start_time, o2_end_time)
        save_path = "/home/qh/YES/oxford/o2/app_sim_mat_toO1.pkl"
        app_full_topo_o2_sim_mat = Base.GetSimMatrixTo19(app_o1_full_topo, save_path, app_o2_full_topo)
        set_path = "/home/qh/YES/oxford/o1"
        best_app = 0.83
        sim_recall_list = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
        app_topo = pickle.load(open(os.path.join(set_path, "topo_map/app_sim_{:.2f}.pkl".format(best_app)), "rb"))
        app_o2_pr_list = []
        top_, gdis_ = 10, 8.0
        for sim_recall in sim_recall_list:
            app_o2_pr = Base.GetPrecisionAndRecall(full_base_topo=app_o1_full_topo,
                                                   base_topo=app_topo,
                                                   full_topo=app_o2_full_topo,
                                                   sim_mat=app_full_topo_o2_sim_mat,
                                                   sim=sim_recall,
                                                   top=top_, gdis=gdis_, topo_num=len(app_topo),
                                                   info="APPo2")
            if app_o2_pr['precision'] == 0 and app_o2_pr['recall'] == 0:
                continue
            app_o2_pr_list.append(app_o2_pr)
            print(app_o2_pr)
        pickle.dump(app_o2_pr_list, open("/home/qh/YES/oxford/app_o2_val.pkl", "wb"))
        del app_o1_full_topo
        del app_o2_full_topo, app_full_topo_o2_sim_mat
    # Base.plot_pr(app_o2_pr_list)
    """

    """
    row_size, col_size = 1, 1
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "VS"
    if True:
        app_o2_pr_list = pickle.load(open("/home/qh/YES/oxford/app_o2_val.pkl", "rb"))
        acc_o2_pr_list = pickle.load(open("/home/qh/YES/oxford/acc_o2_val.pkl", "rb"))
        ax1 = plt.subplot(row_size, col_size, 1)
        ax1.set_aspect('equal', adjustable='box')
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        x, y, area, num = Base.GetPrData(app_o2_pr_list)
        plt.plot(x, y, lw="2", color="#52b9d8", label="app {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="^", s=10, color="black")
        # x, y, area, num = Base.GetPrData(rn_acc_o2_pr_list)
        # plt.plot(x, y, lw="2", color="#fddf8b", label="rn acc {:d} {:.5f}".format(num, area), alpha=0.9)
        # plt.scatter(x, y, marker="o", s=10, color="black")
        x, y, area, num = Base.GetPrData(acc_o2_pr_list)
        plt.plot(x, y, lw="2", color="#0d5b26", label="acc {:d} {:.5f}".format(num, area), alpha=0.9)
        plt.scatter(x, y, marker="v", s=10, color="black")
        ax1.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        plt.title("oxford2", fontsize=15)
    plt.savefig("/home/qh/YES/oxford/compare.png", dpi=600, transparent=True)
    plt.show()
    plt.close()
    print(123)
    """


a, b = get_ins_pose("/home/qh/YES/oxford/o1/gps/ins.csv")
traj_len = 0
for i in range(1, len(b)):
    traj_len += np.linalg.norm(b[i-1][0:3, 3]-b[i][0:3, 3])
print("traj", traj_len)
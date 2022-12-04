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




def plot_muliti_pr2(acc_pr_list, app_pr_list, save_path=None, row_size_=2, title=None, vis=True):
    parameter_size = len(acc_pr_list)
    row_size = row_size_
    col_size = int(math.ceil(parameter_size / row_size))
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    linestyles = ["solid", "dotted", "dashdot"]
    titles = ["Extra dataset1 validation", "Extra dataset2 validation"]
    for ind in range(len(acc_pr_list)):
        acc_pr = acc_pr_list[ind]
        app_pr = app_pr_list[ind]
        title_label = "ours-method VS appearance-based-method"
        if title is not None:
            title_label = title
        fig.suptitle(title_label, fontsize=25)
        for i, key in enumerate(acc_pr):
            # # ACC
            row = i // col_size + 1
            col = i - (row - 1) * col_size + 1
            ax = plt.subplot(row_size, col_size, ind + 1)
            ax.set_aspect('equal', adjustable='box')
            if col == 1:
                plt.ylabel("precision", fontsize=16)
            plt.xlim(0, 1.1)
            plt.ylim(0, 1.1)
            recall = []
            precision = []
            sim = []
            recall.append(1.0)
            precision.append(0.0)
            sim.append(0.0)
            acc_vertex_size = 0
            for pr in acc_pr[key]:
                if pr[1] == 0 or pr[2] == 0:
                    continue
                sim.append(pr[0])
                precision.append(pr[1])
                recall.append(pr[2])
                acc_vertex_size = pr[3]
            recall.append(0.0)
            precision.append(1.0)
            sim.append(1.0)
            acc_area = -trapz(precision, recall, dx=0.001)
            plt.plot(recall, precision, lw="2", color="black", label="ours", alpha=1.0, linestyle="dashed")
            plt.scatter(recall, precision, marker="o", s=10, color="black")
            # for a, b, c in zip(recall, precision, sim):
            #     plt.text(a, b, c, ha='center', va='bottom', fontsize=10)
            # # APP
            recall = []
            precision = []
            sim = []
            recall.append(1.0)
            precision.append(0.0)
            sim.append(0.0)
            app_vertex_size = 0
            for pr in app_pr[key]:
                if pr[1] == 0 or pr[2] == 0:
                    continue
                sim.append(pr[0])
                precision.append(pr[1])
                recall.append(pr[2])
                app_vertex_size = pr[3]
            recall.append(0.0)
            precision.append(1.0)
            sim.append(1.0)
            app_area = -trapz(precision, recall, dx=0.001)
            plt.plot(recall, precision, lw="2", color="gray", label="appearance-based", alpha=1.0, linestyle="solid")
            plt.scatter(recall, precision, marker="o", s=10, color="gray", alpha=0.9)
            # for a, b, c in zip(recall, precision, sim):
            #     plt.text(a, b, c, ha='center', va='bottom', fontsize=10)
            ax.legend(loc="best")
            plt.xlabel("recall", fontsize=16)
            detail = "\nours vertex size:{:d}\nappearance-based vertex:{:d}\ncount reduce {:.1f}%\n\nours area under " \
                     "curve:{:.2f}%\nappearance-based area under curve:{:.2f}%\nperformance improved by {:.2}%".\
                format(acc_vertex_size, app_vertex_size, (app_vertex_size - acc_vertex_size) / app_vertex_size * 100.0, acc_area*100, app_area*100, (acc_area-app_area)/app_area*100)
            plt.text(0.5, 0.4, detail, ha="center", fontsize=12)
            plt.title(titles[ind], fontsize=15)
        for i in range(row_size * col_size):
            if i < parameter_size:
                continue
            else:
                ax = plt.subplot(row_size, col_size, i + 1)
                ax.axis('off')
    if save_path is not None:
        plt.savefig(save_path, dpi=1200, transparent=True)
    if vis:
        plt.show()
    plt.close()


if __name__ == "__main__":
    # pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    # acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
    #                                              bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
    #                                              lidar_topic="/lslidar_point_cloud")
    # map = Base.GetVoxelMap(cloud_path="/home/qh/YES/dlut/Daquan19/liosave/GlobalMapDS.pcd",
    #                        path="/home/qh/YES/dlut/Daquan19/acc_global_voxel_map.pkl",
    #                        x_res=1.0, y_res=1.0, z_res=1.5)
    # acc_full_topo = Base.GetAccFullTopo(accmap=map,
    #                                     topo_info=acc_full_topo_info)

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

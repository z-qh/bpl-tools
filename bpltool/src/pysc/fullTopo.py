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

"""
本文件主要完成：
数据包voxelmap的生成，
累积方法拓扑节点数据生成，描述子相似度矩阵，若干阈值拓扑地图的生成
外观方法的拓扑节点数据生成，描述子相似度矩阵，若干阈值拓扑地图的生成
某些绘图需求也在这里面实现
"""


# 保存截图用函数
def ShowTopoMap2(full_topo, topo_nodes_ind, path=None, vis=True):
    points = np.zeros((3, 0), dtype=np.float32)
    for ind in range(len(topo_nodes_ind)):
        node = full_topo[topo_nodes_ind[ind]]
        points = np.column_stack((points, node.position))
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    ax.scatter(points[1], points[0], marker="o", s=49, color="darkgrey")
    # ax.plot(points[1], points[0], marker="o", s=1, color="black")
    ax.set_aspect(1)
    ax.axis('off')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    detail = "Vertex size: {:d}".format(len(topo_nodes_ind))
    plt.title(detail, fontsize=15)
    if path is not None:
        plt.savefig(path, dpi=1200, transparent=True)
    if vis:
        plt.show()
    plt.close()


# 保存截图用函数
def plot_trajectory2(traj_list, save_path=None):
    # traj : time x y z tw tx ty tz
    color_list = ["black", "black", "black", "black", "black", "black"]
    linestyles = ["solid", "dotted", "dashdot"]
    titles = ["Origin dataset", "Extra dataset1", "Extra dataset2"]
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    x_max, x_min = 0.0, 0.0
    for ind in range(len(traj_list)):
        traj = traj_list[ind]
        posix = []
        posiy = []
        mileage = 0
        for i in range(traj.shape[0]):
            posix.append(traj[i, 1])
            posiy.append(traj[i, 2])
            x_max = max(x_max, traj[i, 1])
            x_min = min(x_min, traj[i, 1])
            if i == 0:
                continue
            else:
                mileage += np.linalg.norm(traj[i - 1, 1:4] - traj[i, 1:4])
        mile_msg = ", {:.2f}Km".format(mileage / 1000.0)
        ax.plot(posiy, posix, linestyle=linestyles[ind],
                linewidth=1.0,
                color=color_list[ind],
                label=titles[ind] + mile_msg,
                alpha=1.0)
        ax.set_aspect(1)
        # ax.axis('off')
        ax.legend(loc='best')
        # ax.get_xaxis().set_visible(False)
        # ax.get_yaxis().set_visible(False)
    if save_path is not None:
        plt.text(0, 0, "xMax:{:.2f} xMin:{:.2f}".format(x_max, x_min))
        plt.savefig(save_path, dpi=1200, transparent=True)
    plt.show()
    plt.close()


if __name__ == "__main__":
    """ 显示画图三圈轨迹
    pose_vec_data16 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan16/liosave/sam2.txt")
    pose_vec_data17 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan17/liosave/sam2.txt")
    pose_vec_data19 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    traj_list = []
    traj_list.append(pose_vec_data19)
    traj_list.append(pose_vec_data16)
    traj_list.append(pose_vec_data17)
    plot_trajectory2(traj_list, "/home/qh/traj.png")

    # acc_full_topo = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    # acc_sim = "/home/qh/YES/dlut/Daquan19/topo_map/acc_sim_{:.2f}.pkl".format(0.90)
    # acc_tmp_topo = Base.GenTopoNodeBySim(path=acc_sim)
    # ShowTopoMap2(acc_full_topo, acc_tmp_topo, path="/home/qh/123.png")
    """

    """
    # # Completion the half connect matrix 补全上三角矩阵
    # Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/acc_connect.pkl")
    # Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/app_connect.pkl")
    """

    """
    # Daquan19 的 累积19 和 外观19 的生成 并生成相似度矩阵 并生成若干阈值拓扑地图
    # # get pose and time x y z
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")

    # # get Bag to find boundary
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan19/acc_full_topo_info.pkl")
    # # generate acc full voxel map
    map19 = Base.GetVoxelMap(path="/home/qh/YES/dlut/Daquan19")
    # # generate acc full topo node
    acc_full_topo = Base.GetAccFullTopo(accmap=map19,
                                        topo_info=acc_full_topo_info,
                                        acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    # # get acc full topo connect mat
    acc_full_topo_connect = Base.GetFullTopoConnectInfo(full_topo=acc_full_topo,
                                                        connect_path="/home/qh/YES/dlut/Daquan19/acc_connect.pkl")
    # # generate appearance full topo
    app_full_topo = Base.GetAppFullTopo(pose_vec=pose_vec_data,
                                                   bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
                                                   lidar_topic="/lslidar_point_cloud",
                                                   app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl", ch=5)
    # # get appearance full topo connect mat
    app_full_topo_connect = Base.GetFullTopoConnectInfo(full_topo=app_full_topo,
                                                        connect_path="/home/qh/YES/dlut/Daquan19/app_connect.pkl")
    # # generate acc sim topo
    # 0.30, 0.40, 0.50, 0.60, 0.65, 这些参数离大谱 点很少
    sim_list = [0.30, 0.40, 0.50, 0.60, 0.65,
                0.70, 0.75, 0.78, 0.80, 0.83, 0.85, 0.87, 0.90,
                0.91, 0.92, 0.93, 0.94, 0.95]
    sim_list = [0.90]
    for sim_thre in sim_list:
        acc_sim = "/home/qh/YES/dlut/Daquan19/topo_map/acc_sim_{:.2f}.pkl".format(sim_thre)
        app_sim = "/home/qh/YES/dlut/Daquan19/topo_map/app_sim_{:.2f}.pkl".format(sim_thre)
        acc_sim_fig = "/home/qh/YES/dlut/Daquan19/topo_map/acc_sim_{:.2f}.png".format(sim_thre)
        app_sim_fig = "/home/qh/YES/dlut/Daquan19/topo_map/app_sim_{:.2f}.png".format(sim_thre)
        acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo,
                                             full_topo_connect=acc_full_topo_connect,
                                             sim_threshold=sim_thre,
                                             path=acc_sim)
        app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_full_topo,
                                             full_topo_connect=app_full_topo_connect,
                                             sim_threshold=sim_thre,
                                             path=app_sim)
        print("Save Sim:{:.2f}".format(sim_thre))
        Base.ShowTopoMap(acc_full_topo, acc_tmp_topo, path=acc_sim_fig, vis=False)
        Base.ShowTopoMap(app_full_topo, app_tmp_topo, path=app_sim_fig, vis=False)
        ShowTopoMap2(acc_full_topo, acc_tmp_topo, path="/home/qh/123.png")
    """

    """
    # Daqaun16 用于验证测试 首先生成fullnode 然后针对 累积19 外观19 生成相似度矩阵
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan16/liosave/sam2.txt")
    # # generate acc full topo
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-16-DaQuan.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan16/acc_full_topo_info.pkl")
    acc_map = Base.GetVoxelMap(path="/home/qh/YES/dlut/Daquan16")
    acc_full_topo = Base.GetAccFullTopo(accmap=acc_map,
                                        topo_info=acc_full_topo_info,
                                        acc_fulltopo_path="/home/qh/YES/dlut/Daquan16/acc_full_topo.pkl")
    # # 累积 connect to 19
    full_topo_base_acc = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    acc_full_topo_connect_acc = Base.GetFullTopoConnectInfo(full_topo=acc_full_topo, full_topo_base=full_topo_base_acc,
                                                            connect_path="/home/qh/YES/dlut/Daquan16/acc_connect16.pkl")
    # generate app full topo
    app_full_topo = Base.GetAppFullTopo(pose_vec=pose_vec_data,
                                                   bag_file="/home/qh/YES/dlut/2021-01-16-DaQuan.bag",
                                                   lidar_topic="/lslidar_point_cloud",
                                                   app_full_topo_path="/home/qh/YES/dlut/Daquan16/app_full_topo.pkl", ch=4)
    # # 外观 connect to 19
    full_topo_base_app = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    app_full_topo_connect_app = Base.GetFullTopoConnectInfo(full_topo=app_full_topo, full_topo_base=full_topo_base_app,
                                                            connect_path="/home/qh/YES/dlut/Daquan16/app_connect16.pkl")
    """

    """
    # Daqaun17 用于验证测试 首先生成fullnode 然后针对 累积19 外观19 生成相似度矩阵
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan17/liosave/sam2.txt")
    # # generate acc full topo
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-17-11-12-10.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan17/acc_full_topo_info.pkl")
    acc_map = Base.GetVoxelMap(path="/home/qh/YES/dlut/Daquan17")
    acc_full_topo = Base.GetAccFullTopo(accmap=acc_map,
                                        topo_info=acc_full_topo_info,
                                        acc_fulltopo_path="/home/qh/YES/dlut/Daquan17/acc_full_topo.pkl")
    # # 累积 connect to 19
    full_topo_base_acc = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    acc_full_topo_connect_acc = Base.GetFullTopoConnectInfo(full_topo=acc_full_topo, full_topo_base=full_topo_base_acc,
                                                            connect_path="/home/qh/YES/dlut/Daquan17/acc_connect17.pkl")
    # generate app full topo
    app_full_topo = Base.GetAppFullTopo(pose_vec=pose_vec_data,
                                                   bag_file="/home/qh/YES/dlut/2021-01-17-11-12-10.bag",
                                                   lidar_topic="/lslidar_point_cloud",
                                                   app_full_topo_path="/home/qh/YES/dlut/Daquan17/app_full_topo.pkl", ch=4)
    # # 外观 connect to 19
    full_topo_base_app = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    app_full_topo_connect_app = Base.GetFullTopoConnectInfo(full_topo=app_full_topo, full_topo_base=full_topo_base_app,
                                                            connect_path="/home/qh/YES/dlut/Daquan17/app_connect17.pkl")
    """

    """
    # acc_full_topo_connect_16 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan16/acc_connect16.pkl")
    # app_full_topo_connect_16 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan16/app_connect16.pkl")
    # acc_full_topo_connect_17 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/acc_connect17.pkl")
    # app_full_topo_connect_17 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/app_connect17.pkl")
    """

    print(123)

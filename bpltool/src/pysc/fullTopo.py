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

if __name__ == "__main__":
    """
    pose_vec_data16 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan16/liosave/sam2.txt")
    pose_vec_data17 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan17/liosave/sam2.txt")
    pose_vec_data19 = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    traj_list = []
    traj_list.append(pose_vec_data16)
    traj_list.append(pose_vec_data17)
    traj_list.append(pose_vec_data19)
    Base.plot_trajectory(traj_list, "/home/qh/YES/dlut/traj.png")
    """

    """
    # # Completion the half connect matrix
    # Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/acc_connect.pkl")
    # Base.TopoConnectCompletion("/home/qh/YES/dlut/Daquan19/app_connect.pkl")
    """

    """
    # Daquan19 的 A19 和 B19 的生成
    # # get pose and time x y z
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")

    # # get Bag to find boundary
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan19/acc_full_topo_info.pkl")
    # # generate acc full voxel map
    map = Base.GetVoxelMap(cloud_path="/home/qh/YES/dlut/Daquan19/liosave/GlobalMapDS.pcd",
                           path="/home/qh/YES/dlut/Daquan19/acc_global_voxel_map.pkl",
                           x_res=1.0, y_res=1.0, z_res=1.5)
    # # generate acc full topo node
    acc_full_topo = Base.GetAccFullTopo(accmap=map,
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
        # Base.ShowTopoMap(acc_full_topo, acc_tmp_topo, path=acc_sim_fig, vis=False)
        # Base.ShowTopoMap(app_full_topo, app_tmp_topo, path=app_sim_fig, vis=False)
    """

    """
    # Daqaun16 用于验证测试 首先生成fullnode 然后针对A19 B19生成相似度矩阵
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan16/liosave/sam2.txt")
    # # generate acc full topo
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-16-DaQuan.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan16/acc_full_topo_info.pkl")
    acc_map = Base.GetVoxelMap(cloud_path="/home/qh/YES/dlut/Daquan16/liosave/GlobalMapDS.pcd",
                               path="/home/qh/YES/dlut/Daquan16/acc_global_voxel_map.pkl",
                               x_res=1.0, y_res=1.0, z_res=1.5)
    acc_full_topo = Base.GetAccFullTopo(accmap=acc_map,
                                        topo_info=acc_full_topo_info,
                                        acc_fulltopo_path="/home/qh/YES/dlut/Daquan16/acc_full_topo.pkl")
    # # connect to 19
    # full_topo_base_acc = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    # acc_full_topo_connect_acc = Base.GetFullTopoConnectInfo(full_topo=acc_full_topo, full_topo_base=full_topo_base_acc,
    #                                                         connect_path="/home/qh/YES/dlut/Daquan16/acc_connect16.pkl")
    # generate app full topo
    # app_full_topo = Base.GetAppFullTopo(pose_vec=pose_vec_data,
    #                                                bag_file="/home/qh/YES/dlut/2021-01-16-DaQuan.bag",
    #                                                lidar_topic="/lslidar_point_cloud",
    #                                                app_full_topo_path="/home/qh/YES/dlut/Daquan16/app_full_topo.pkl", ch=4)
    # # connect to 19
    # full_topo_base_app = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    # app_full_topo_connect_app = Base.GetFullTopoConnectInfo(full_topo=app_full_topo, full_topo_base=full_topo_base_app,
    #                                                         connect_path="/home/qh/YES/dlut/Daquan16/app_connect16.pkl")
    """

    """
    # Daqaun17 用于验证测试 首先生成fullnode 然后针对A19 B19生成相似度矩阵
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan17/liosave/sam2.txt")
    # # generate acc full topo
    acc_full_topo_info = Base.GetAccFullTopoInfo(pose_vec=pose_vec_data,
                                                 bag_file="/home/qh/YES/dlut/2021-01-17-11-12-10.bag",
                                                 lidar_topic="/lslidar_point_cloud",
                                                 acc_full_topo_info_path="/home/qh/YES/dlut/Daquan17/acc_full_topo_info.pkl")
    acc_map = Base.GetVoxelMap(cloud_path="/home/qh/YES/dlut/Daquan17/liosave/GlobalMapDS.pcd",
                               path="/home/qh/YES/dlut/Daquan17/acc_global_voxel_map.pkl",
                               x_res=1.0, y_res=1.0, z_res=1.5)
    acc_full_topo = Base.GetAccFullTopo(accmap=acc_map,
                                        topo_info=acc_full_topo_info,
                                        acc_fulltopo_path="/home/qh/YES/dlut/Daquan17/acc_full_topo.pkl")
    # # connect to 19
    # full_topo_base_acc = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    # acc_full_topo_connect_acc = Base.GetFullTopoConnectInfo(full_topo=acc_full_topo, full_topo_base=full_topo_base_acc,
    #                                                         connect_path="/home/qh/YES/dlut/Daquan17/acc_connect17.pkl")
    # generate app full topo
    # app_full_topo = Base.GetAppFullTopo(pose_vec=pose_vec_data,
    #                                                bag_file="/home/qh/YES/dlut/2021-01-17-11-12-10.bag",
    #                                                lidar_topic="/lslidar_point_cloud",
    #                                                app_full_topo_path="/home/qh/YES/dlut/Daquan17/app_full_topo.pkl", ch=4)
    # # connect to 19
    # full_topo_base_app = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    # app_full_topo_connect_app = Base.GetFullTopoConnectInfo(full_topo=app_full_topo, full_topo_base=full_topo_base_app,
    #                                                         connect_path="/home/qh/YES/dlut/Daquan17/app_connect17.pkl")
    """

    """
    # acc_full_topo_connect_16 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan16/acc_connect16.pkl")
    # app_full_topo_connect_16 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan16/app_connect16.pkl")
    # acc_full_topo_connect_17 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/acc_connect17.pkl")
    # app_full_topo_connect_17 = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/app_connect17.pkl")
    """


    print(123)

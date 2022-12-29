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
import os
import time as ttime


def getPR(top_=10, gdis_=3.0):
    """
    进行验证， 目前有 Daquan17 对 Daquan19 的相似度矩阵 用来检验阴阳 并且TopoMap是有位姿的可以检验真假
    参数整定之后最好的参数是 0.87 0.90 生成的地图
    """

    sim_map_list = [0.90]
    sim_recall_list = [0.1, 0.2, 0.3, 0.4, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    acc_pr_path = "/home/qh/YES/dlut/Daquan17/acc_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    app_pr_path = "/home/qh/YES/dlut/Daquan17/app_prTop{:d}G{:.1f}.pkl".format(top_, gdis_)
    accPR = {}
    appPR = {}
    if os.path.isfile(acc_pr_path) and os.path.isfile(app_pr_path):
        acc_pr_load = pickle.load(open(acc_pr_path, "rb"))
        app_pr_load = pickle.load(open(app_pr_path, "rb"))
        for sim_map in sim_map_list:
            accPR[sim_map] = acc_pr_load[sim_map]
            appPR[sim_map] = app_pr_load[sim_map]
        return accPR, appPR
    acc_full_tpop_base = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    app_full_topo_base = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    acc_full_topo = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan17/acc_full_topo.pkl")
    acc_full_topo_connect = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/acc_connect17.pkl")
    app_full_topo = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan17/app_full_topo.pkl")
    app_full_topo_connect = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan17/app_connect17.pkl")
    for sim_map in sim_map_list:
        start_time = ttime.time()
        if accPR.get(sim_map) is None:
            acc_sim_map_path = "/home/qh/YES/dlut/Daquan19/topo_map/acc_sim_{:.2f}.pkl".format(sim_map)
            tmp_acc_pr_list = []
            acc_tmp_topo = Base.GenTopoNodeBySim(sim_threshold=sim_map, path=acc_sim_map_path)
            for sim_recall in sim_recall_list:
                acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_tpop_base,
                                                        base_topo=acc_tmp_topo,
                                                        full_topo=acc_full_topo,
                                                        connect=acc_full_topo_connect, sim=sim_recall,
                                                        top=top_, gdis=gdis_)
                tmp_acc_pr_list.append(acc_tmp_pr)
            accPR[sim_map] = tmp_acc_pr_list
        if appPR.get(sim_map) is None:
            app_sim_map_path = "/home/qh/YES/dlut/Daquan19/topo_map/app_sim_{:.2f}.pkl".format(sim_map)
            tmp_app_pr_list = []
            app_tmp_topo = Base.GenTopoNodeBySim(sim_threshold=sim_map,
                                                 path=app_sim_map_path)
            for sim_recall in sim_recall_list:
                app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo_base,
                                                        base_topo=app_tmp_topo,
                                                        full_topo=app_full_topo,
                                                        connect=app_full_topo_connect, sim=sim_recall,
                                                        top=top_, gdis=gdis_)
                tmp_app_pr_list.append(app_tmp_pr)
            appPR[sim_map] = tmp_app_pr_list
        print("Get {:.2f}-TopoMap get Recall Cost:{:.2f}s".format(sim_map, ttime.time() - start_time))
    if not os.path.isfile(acc_pr_path):
        pickle.dump(accPR, open(acc_pr_path, "wb"))
    if not os.path.isfile(app_pr_path):
        pickle.dump(appPR, open(app_pr_path, "wb"))
    return accPR, appPR


if __name__ == "__main__":
    top, gids = 5, 5.0
    accpr, apppr = getPR(top, gids)
    Base.plot_muliti_pr(accpr, apppr, row_size_=1,
                        title="Validation2: ours VS appearance-based-method",
                        save_path="/home/qh/YES/dlut/Daquan17/TestTop{:d}G{:.1f}.png".format(top, gids))

    print("123")

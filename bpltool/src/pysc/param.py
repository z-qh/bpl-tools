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
import time as ttime


def getPR():
    """
    进行参数整定， 自己对自己进行参数整定 ours 和 appearance-base
    """
    # generate acc sim topo
    top_list = [10, 5, 3, 1]
    gdis_list = [3.0, 2.0, 1.0]
    top_, gdis_ = 10, 3.0
    sim_map_list = [0.70, 0.75, 0.78, 0.83, 0.85, 0.87, 0.90, 0.95]
    sim_recall_list = [0.1, 0.2, 0.3, 0.4, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99]
    acc_pr_path = "/home/qh/YES/dlut/Daquan19/acc_pr.pkl"
    app_pr_path = "/home/qh/YES/dlut/Daquan19/app_pr.pkl"
    accPR = {}
    appPR = {}
    if os.path.isfile(acc_pr_path) and os.path.isfile(app_pr_path):
        acc_pr_load = pickle.load(open(acc_pr_path, "rb"))
        app_pr_load = pickle.load(open(app_pr_path, "rb"))
        for sim_map in sim_map_list:
            accPR[sim_map] = acc_pr_load[sim_map]
            appPR[sim_map] = app_pr_load[sim_map]
        return accPR, appPR
    acc_full_topo = Base.GetAccFullTopo(acc_fulltopo_path="/home/qh/YES/dlut/Daquan19/acc_full_topo.pkl")
    acc_full_topo_connect = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan19/acc_connect.pkl")
    app_full_topo = Base.GetAppFullTopo(app_full_topo_path="/home/qh/YES/dlut/Daquan19/app_full_topo.pkl")
    app_full_topo_connect = Base.GetFullTopoConnectInfo(connect_path="/home/qh/YES/dlut/Daquan19/app_connect.pkl")
    for sim_map in sim_map_list:
        start_time = ttime.time()
        if accPR.get(sim_map) is None:
            acc_sim = "/home/qh/YES/dlut/Daquan19/topo_map/acc_sim_{:.2f}.pkl".format(sim_map)
            tmp_acc_pr_list = []
            acc_tmp_topo = Base.GenTopoNodeBySim(full_topo=acc_full_topo,
                                                 full_topo_connect=acc_full_topo_connect,
                                                 sim_threshold=sim_map,
                                                 path=acc_sim)
            for sim_recall in sim_recall_list:
                acc_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=acc_full_topo,
                                                        base_topo=acc_tmp_topo,
                                                        full_topo=acc_full_topo,
                                                        connect=acc_full_topo_connect, sim=sim_recall,
                                                        top=top_, gdis=gdis_)
                tmp_acc_pr_list.append(acc_tmp_pr)
            accPR[sim_map] = tmp_acc_pr_list
        if appPR.get(sim_map) is None:
            app_sim = "/home/qh/YES/dlut/Daquan19/topo_map/app_sim_{:.2f}.pkl".format(sim_map)
            tmp_app_pr_list = []
            app_tmp_topo = Base.GenTopoNodeBySim(full_topo=app_full_topo,
                                                 full_topo_connect=app_full_topo_connect,
                                                 sim_threshold=sim_map,
                                                 path=app_sim)
            for sim_recall in sim_recall_list:
                app_tmp_pr = Base.GetPrecisionAndRecall(full_base_topo=app_full_topo,
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
    accPR, appPR = getPR()

    Base.plot_muliti_pr(accPR, appPR,save_path="/home/qh/YES/dlut/Daquan19/ParameterSetting2.png")
    print("0.90")

import collections
import os
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
import glob

pre_fix = "/home/qh/YES/dlut/"


def handleDaquan19():
    acc_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan19/acc_sim_mat.pkl")
    Base.GetSimMatrixTo19(acc_full_topo19, save_path, None)
    del acc_full_topo19
    app_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan19/app_sim_mat.pkl")
    Base.GetSimMatrixTo19(app_full_topo19, save_path, None)
    del acc_full_topo19


def handleDaquan16():
    acc_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/acc_full_topo.pkl", "rb")))
    acc_full_topo16 = pickle.load(open(os.path.join(pre_fix, "Daquan16/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan16/acc_sim_mat_to19.pkl")
    Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo16)
    del acc_full_topo19, acc_full_topo16

    app_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/app_full_topo.pkl", "rb")))
    app_full_topo16 = pickle.load(open(os.path.join(pre_fix, "Daquan16/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan16/app_sim_mat_to19.pkl")
    Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo16)
    del app_full_topo19, app_full_topo16


def handleDaquan17():
    acc_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/acc_full_topo.pkl", "rb")))
    acc_full_topo17 = pickle.load(open(os.path.join(pre_fix, "Daquan17/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan17/acc_sim_mat_to19.pkl")
    Base.GetSimMatrixTo19(acc_full_topo19, save_path, acc_full_topo17)
    del acc_full_topo19, acc_full_topo17

    app_full_topo19 = pickle.load(open(os.path.join(pre_fix, "Daquan19/app_full_topo.pkl", "rb")))
    app_full_topo17 = pickle.load(open(os.path.join(pre_fix, "Daquan17/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "Daquan17/app_sim_mat_to19.pkl")
    Base.GetSimMatrixTo19(app_full_topo19, save_path, app_full_topo17)
    del app_full_topo19, app_full_topo17


def handleOxford1():
    acc_o1_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford1/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "oxford1/acc_sim_mat.pkl")
    Base.GetSimMatrixTo19(acc_o1_full_topo, save_path)
    del acc_o1_full_topo

    app_o1_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford1/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "oxford1/app_sim_mat.pkl")
    Base.GetSimMatrixTo19(app_o1_full_topo, save_path)
    del app_o1_full_topo


def handleOxford2():
    acc_o1_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford1/acc_full_topo.pkl", "rb")))
    acc_o2_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford2/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "oxford1/acc_sim_mat_toO1.pkl")
    Base.GetSimMatrixTo19(acc_o1_full_topo, save_path, acc_o2_full_topo)
    del acc_o1_full_topo, acc_o2_full_topo

    app_o1_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford1/app_full_topo.pkl", "rb")))
    app_o2_full_topo = pickle.load(open(os.path.join(pre_fix, "oxford2/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "oxford1/app_sim_mat_toO1.pkl")
    Base.GetSimMatrixTo19(app_o1_full_topo, save_path, app_o2_full_topo)
    del app_o1_full_topo, app_o2_full_topo


def handleJGXY1():
    acc_full_j1 = pickle.load(open(os.path.join(pre_fix, "jgxy1/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "jgxy1/acc_sim_mat.pkl")
    Base.GetSimMatrixTo19(acc_full_j1, save_path, None)
    del acc_full_j1

    app_full_j1 = pickle.load(open(os.path.join(pre_fix, "jgxy1/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "jgxy1/app_sim_mat.pkl")
    Base.GetSimMatrixTo19(app_full_j1, save_path, None)
    del app_full_j1


def handleJGXY2():
    acc_full_j1 = pickle.load(open(os.path.join(pre_fix, "jgxy1/acc_full_topo.pkl", "rb")))
    acc_full_j2 = pickle.load(open(os.path.join(pre_fix, "jgxy2/acc_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "jgxy2/acc_sim_mat_toJ1.pkl")
    Base.GetSimMatrixTo19(acc_full_j1, save_path, acc_full_j2)
    del acc_full_j1, acc_full_j2

    app_full_j1 = pickle.load(open(os.path.join(pre_fix, "jgxy1/app_full_topo.pkl", "rb")))
    app_full_j2 = pickle.load(open(os.path.join(pre_fix, "jgxy2/app_full_topo.pkl", "rb")))
    save_path = os.path.join(pre_fix, "jgxy2/app_sim_mat_toJ1.pkl")
    Base.GetSimMatrixTo19(app_full_j1, save_path, app_full_j2)
    del app_full_j1, app_full_j2


if __name__ == "__main__":
    handleDaquan19()
    handleDaquan16()
    handleDaquan17()
    handleOxford1()
    handleOxford2()
    handleJGXY1()
    handleJGXY2()

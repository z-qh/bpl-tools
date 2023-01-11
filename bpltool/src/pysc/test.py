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

if __name__ == "__main__":
    dir = "/home/qh/YES/oxford/2019-01-10-12-32-52-radar-oxford-10k_Velodyne_HDL-32E_Left_Pointcloud/2019-01-10-12-32-52-radar-oxford-10k/velodyne_left"
    bin_files = sorted(glob.glob(dir + "/*.bin"))
    for bin_file in bin_files:
        pc = np.fromfile(bin_file, dtype=np.float32).reshape((4, -1))[0:3, :].transpose()
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(pc)
        open3d.visualization.draw_geometries([pcd])
        print(123)
    print(123)




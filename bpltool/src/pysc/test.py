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
    bag_path_16 = "/home/qh/YES/dlut/2021-01-17-11-12-10.bag"
    topic = "/lslidar_point_cloud"
    bag16 = rosbag.Bag(bag_path_16)
    path_save_bin = "/home/qh/YES/dlut/Daquan17/bin"
    bag_data = bag16.read_messages(topic)
    ind = 0
    f = open("/home/qh/YES/dlut/Daquan17/timestamp", "w")
    info = bag16.get_type_and_topic_info()
    print("Total {:d} Frames".format(info.topics["/lslidar_point_cloud"].message_count))
    handle_size = info.topics["/lslidar_point_cloud"].message_count
    report_size = handle_size // 100 if handle_size // 100 != 0 else 1
    start_time = time.time()
    for topic, msg, t, in bag_data:
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar)).astype(np.float32)
        points = points[~np.isnan(points).any(axis=1)]
        bin_file = os.path.join(path_save_bin, "{:06d}.bin".format(ind))
        points.tofile(bin_file)
        f.write("{:f}\n".format(t.to_sec()))
        if ind % report_size == 0:
            print("Gen Bin {:.2f}% Cost {:.2f}s".format(ind / handle_size * 100, time.time()-start_time))
            start_time = time.time()
        ind += 1
    f.close()
    print(123)

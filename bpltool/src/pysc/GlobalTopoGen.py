import math
import pickle
from collections import defaultdict
import os
import numpy
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

from open3d import *


class CloudPiece:
    def __init__(self, i_, j_, cloud_):
        self.idi = i_
        self.idj = j_
        self.data = cloud_

    def InsertPoint(self, point):
        self.data = np.row_stack(self.data, point)


class KeyFrame:
    def __init__(self, time_, x_, y_, trans_):
        self.time = time_
        self.x = x_
        self.y = y_
        self.trans = trans_


class VoxelMap:
    def Posi2Ind(self, poi_x, poi_y):
        ind_i = math.floor(poi_y / self.y_resolution)
        ind_j = math.floor(poi_x / self.x_resolution)
        return ind_i, ind_j

    def Ind2PosiRange(self, ind_i, ind_j):
        poi_x_min = self.x_resolution * ind_j
        poi_x_max = poi_x_min + self.x_resolution
        poi_y_min = self.y_resolution * ind_i
        poi_y_max = poi_y_min + self.y_resolution
        return poi_x_min, poi_x_max, poi_y_min, poi_y_max

    def Posi2Key(self, ind_i, ind_j):
        return "{:d} {:d}".format(ind_i, ind_j)

    def Key2Ind(self, key: str):
        ind_i = key.split()[0]
        ind_j = key.split()[1]
        return ind_i, ind_j

    def __init__(self, full_cloud=None, trajectory=None, x_res=0.2, y_res=0.2):
        self.x_resolution = 0.2
        self.y_resolution = 0.2
        self.voxel_dict = defaultdict(None)
        self.key_frame = []
        self.range_x = 0
        self.range_y = 0
        if not full_cloud is None and not trajectory is None:
            self.InsertFullCloud(full_cloud)
            self.key_frame = trajectory
            pickle.dump(self.voxel_dict, open("global_voxel_map.pkl", "wb"))
            pickle.dump(self.key_frame, open("global_trajectory.pkl", "wb"))
        elif os.path.isfile("global_voxel_map.pkl") and os.path.isfile("global_voxel_map.pkl"):
            self.voxel_dict = pickle.load(open("global_voxel_map.pkl", "rb"))
            self.key_frame = pickle.load(open("global_voxel_map.pkl", "rb"))
        else:
            print("Nothing to Do, Just For Test")

    def InsertFullCloud(self, point_cloud):
        # point_cloud : x y z i l... as a row
        point_size = point_cloud.shape[0]
        for point_i in range(point_size):
            point = point_cloud[point_i, :]
            now_x = point[0]
            now_y = point[1]
            ind_i, ind_j = self.Posi2Ind(now_x, now_y)
            key = self.Posi2Key(ind_i, ind_j)
            if self.voxel_dict[key] is None:
                self.voxel_dict[key] = CloudPiece(ind_i, ind_j, point)
            else:
                self.voxel_dict[key].InsertPoint(point)



def BinFind(time_vec, x, l, r):
    if r >= l:
        mid = int(l + (r - l) / 2)
        if abs(time_vec[mid].time - x) < 0.05:
            return mid
        elif time_vec[mid].time > x:
            return BinFind(time_vec, x, l, mid - 1)
        else:
            return BinFind(time_vec, x, mid + 1, r)
    else:
        return -1

if __name__ == "__main__":
    # 加载位姿 只保存 math.time X Z Y roll pitch yaw
    trajectory = []
    pose_file = "/home/qh/YES/dlut/gtFittingA.txt"
    pose_vec = np.loadtxt(pose_file, skiprows=1)
    pose_vec = np.delete(pose_vec, [7], 1)
    for data in pose_vec:
        trajectory.append(KeyFrame(data[0], data[1], data[3], data[1:7]))
    # 生成全局地图
    global_map = np.array(())
    bag_file = "/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag"
    bag = rosbag.Bag(bag_file)
    bag_clouds = bag.read_messages("/lslidar_point_cloud")
    count = 1
    for topic, msg, rtime in bag_clouds:
        time = rtime.to_sec()
        ind = BinFind(trajectory, time, 0, len(trajectory) - 1)
        if ind == -1:
            continue
        transT = trajectory[ind].trans
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar))
        TransposeT(transT, points)
        global_map = np.row_stack((global_map, points))
        if count > 10:
            break

    # 可视化累积点云
    full_cloud = open3d.geometry.PointCloud()
    full_cloud.points = open3d.utility.Vector3dVector(global_map)
    full_cloud.paint_uniform_color([1, 1, 1])

    vis = open3d.visualization.Visualizer()
    vis.create_window()
    render_option = open3d.visualization.RenderOption = vis.get_render_option()
    render_option.background_color = np.array([0, 0, 0])
    render_option.point_size = 1.0
    vis.add_geometry(full_cloud)
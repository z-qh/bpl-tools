import collections
import math
import sys
import numpy as np
from numba import cuda
import numba
import open3d
import pickle
import os
import rosbag
import sensor_msgs.point_cloud2 as pc2
import time as ttime
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import Base
import glob
import cv2

N_SCAN = 0
Horizon_SCAN = 0
low_angle = 0
up_angle = 0
sensor_minimum_range = 0
ground_scan_ind = 0

ang_res_x = 0
ang_res_y = 0
segment_alpha_x = 0
segment_alpha_y = 0
segment_theta = 0
segment_valid_point_num = 0
segment_valid_line_num = 0
mount_angle = 0
GroundSegInitFlag = False


def SetParameter(nscan=32, hscan=2000,
                 low_ang=-16.5, up_ang=10,
                 ground_ind=15, min_range=5.0,
                 segment_theta_d=30.0):
    global segment_alpha_x, segment_alpha_y, low_angle, up_angle, N_SCAN, Horizon_SCAN
    global ang_res_x, ang_res_y, ground_scan_ind, segment_theta
    global segment_valid_point_num, segment_valid_line_num, sensor_minimum_range, mount_angle
    global GroundSegInitFlag
    N_SCAN = nscan
    Horizon_SCAN = hscan
    low_angle = low_ang
    up_angle = up_ang
    ang_res_x = 360.0 / Horizon_SCAN
    ang_res_y = (up_angle - low_angle) / (N_SCAN - 1)
    ground_scan_ind = ground_ind
    sensor_minimum_range = min_range

    segment_alpha_x = ang_res_x / 180.0 * np.pi
    segment_alpha_y = ang_res_y / 180.0 * np.pi
    segment_valid_point_num = 5
    segment_valid_line_num = 3
    segment_theta = segment_theta_d / 180.0 * np.pi
    mount_angle = 0
    GroundSegInitFlag = True


@numba.jit(nopython=True)
def labelComponents(queue_ind_x, queue_ind_y,
                    all_pushed_ind_x, all_pushed_ind_y,
                    label_mat, range_mat,
                    row, col, label_count):
    neighbors = [(-1, 0), (0, 1), (0, -1), (1, 0)]
    line_count_flag = np.zeros((N_SCAN,), dtype=np.int8)
    queue_ind_x[0] = row
    queue_ind_y[0] = col
    queue_size = 1
    queue_start_ind = 0
    queue_end_ind = 1
    all_pushed_ind_x[0] = row
    all_pushed_ind_y[0] = col
    all_pushed_ind_size = 1
    while queue_size > 0:
        from_ind_x = queue_ind_x[queue_start_ind]
        from_ind_y = queue_ind_y[queue_start_ind]
        queue_size -= 1
        queue_start_ind += 1
        label_mat[from_ind_x, from_ind_y] = label_count
        for near in neighbors:
            this_ind_x = from_ind_x + near[0]
            this_ind_y = from_ind_y + near[1]
            if this_ind_x < 0 or this_ind_x >= N_SCAN:
                continue
            if this_ind_y < 0:
                this_ind_y = Horizon_SCAN - 1
            if this_ind_y >= Horizon_SCAN:
                this_ind_y = 0
            if label_mat[this_ind_x, this_ind_y] != 0:
                continue
            d1 = max(range_mat[from_ind_x, from_ind_y], range_mat[this_ind_x, this_ind_y])
            d2 = min(range_mat[from_ind_x, from_ind_y], range_mat[this_ind_x, this_ind_y])
            if near[0] == 0:
                alpha = segment_alpha_x
            else:
                alpha = segment_alpha_y
            angle_ = np.arctan2(d2 * np.sin(alpha), (d1 - d2 * np.cos(alpha)))
            if angle_ > segment_theta:
                queue_ind_x[queue_end_ind] = this_ind_x
                queue_ind_y[queue_end_ind] = this_ind_y
                queue_size += 1
                queue_end_ind += 1
                label_mat[this_ind_x, this_ind_y] = label_count
                line_count_flag[this_ind_x] = 1
                all_pushed_ind_x[all_pushed_ind_size] = this_ind_x
                all_pushed_ind_y[all_pushed_ind_size] = this_ind_y
                all_pushed_ind_size += 1
    feasible_segment = False
    if all_pushed_ind_size >= 30:
        feasible_segment = True
    elif all_pushed_ind_size >= segment_valid_point_num:
        line_count = 0
        for i_ in range(N_SCAN):
            if line_count_flag[i_] == 1:
                line_count += 1
        if line_count >= segment_valid_line_num:
            feasible_segment = True
    if feasible_segment:
        label_count += 1
    else:
        for i_ in range(all_pushed_ind_size):
            label_mat[all_pushed_ind_x[i_], all_pushed_ind_y[i_]] = 999999


@numba.jit(nopython=True)
def GetSelectRay(local_points, n_sec=6, surf_threshold=0.1):
    if not GroundSegInitFlag:
        print("Please Init First!")
        return
    #
    start_orientation = -np.arctan2(local_points[0, 1], local_points[0, 0])
    end_orientation = -np.arctan2(local_points[-1, 1], local_points[-1, 0]) + 2 * np.pi
    if end_orientation - start_orientation > 3 * np.pi:
        end_orientation -= 2 * np.pi
    elif end_orientation - start_orientation < np.pi:
        end_orientation += 2 * np.pi
    orientation_diff = end_orientation - start_orientation
    #
    range_mat = np.full((N_SCAN, Horizon_SCAN), fill_value=np.nan, dtype=np.float32)
    ground_mat = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.int8)
    label_mat = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.int32)
    full_cloud = np.full((N_SCAN * Horizon_SCAN, 3), fill_value=np.nan, dtype=np.float32)
    cloud_size = local_points.shape[0]
    for i in range(cloud_size):
        this_point = local_points[i, :]
        vertical_angle = np.arctan2(this_point[2], np.sqrt(this_point[0] ** 2 + this_point[1] ** 2)) * 180 / np.pi
        row_idn = int((vertical_angle - low_angle) / ang_res_y)
        if row_idn < 0 or row_idn >= N_SCAN:
            continue
        horizon_angle = np.arctan2(this_point[0], this_point[1]) * 180 / np.pi
        column_idn = int(-round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN / 2)
        if column_idn >= Horizon_SCAN:
            column_idn -= Horizon_SCAN
        if column_idn < 0 or column_idn >= Horizon_SCAN:
            continue
        range_ = np.linalg.norm(this_point)
        if range_ < sensor_minimum_range:
            continue
        range_mat[row_idn, column_idn] = range_
        index = column_idn + row_idn * Horizon_SCAN
        full_cloud[index, :] = this_point
    #
    for j_ in range(Horizon_SCAN):
        for i_ in range(ground_scan_ind):
            low_ind = j_ + i_ * Horizon_SCAN
            up_ind = j_ + (i_ + 1) * Horizon_SCAN
            if np.isnan(full_cloud[low_ind]).any() or np.isnan(full_cloud[up_ind]).any():
                # no info to check, invalid points
                ground_mat[i_, j_] = -1
                continue
            diff = full_cloud[up_ind] - full_cloud[low_ind]
            angle_ = np.arctan2(diff[2], np.sqrt(diff[0] ** 2 + diff[1] ** 2)) * 180 / np.pi
            if abs(angle_ - mount_angle) <= 10:
                ground_mat[i_, j_] = 1
                ground_mat[i_ + 1, j_] = 1
    for i_ in range(N_SCAN):
        for j_ in range(Horizon_SCAN):
            if ground_mat[i_, j_] == 1 or np.isnan(range_mat[i_, j_]):
                label_mat[i_, j_] = -1
    #
    queue_ind_x = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int16)
    queue_ind_y = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int16)
    all_pushed_ind_x = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int16)
    all_pushed_ind_y = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int16)
    label_count = 1
    for i_ in range(N_SCAN):
        for j_ in range(Horizon_SCAN):
            if label_mat[i_, j_] == 0:
                labelComponents(queue_ind_x, queue_ind_y,
                                all_pushed_ind_x, all_pushed_ind_y,
                                label_mat, range_mat, i_, j_, label_count)
    size_of_seg_cloud = 0
    start_ring_index = np.zeros((N_SCAN,), dtype=np.int32)
    end_ring_index = np.zeros((N_SCAN,), dtype=np.int32)
    segmented_cloud_ground_flag = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int8)
    segmented_cloud_col_ind = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int32)
    segmented_cloud_range = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.float32)
    segmented_cloud_index = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.int32)
    for i_ in range(N_SCAN):
        start_ring_index[i_] = size_of_seg_cloud - 1 + 5
        for j_ in range(Horizon_SCAN):
            if label_mat[i_, j_] > 0 or ground_mat[i_, j_] == 1:
                if label_mat[i_, j_] == 999999:
                    if i_ > ground_scan_ind and j_ % 5 == 0:
                        # NonGroundPoints
                        continue
                if ground_mat[i_, j_] == 1:
                    if j_ % 5 != 0 and 5 < j_ < Horizon_SCAN - 5:
                        continue
                segmented_cloud_ground_flag[size_of_seg_cloud] = ground_mat[i_, j_]
                segmented_cloud_col_ind[size_of_seg_cloud] = j_
                segmented_cloud_range[size_of_seg_cloud] = range_mat[i_, j_]
                segmented_cloud_index[size_of_seg_cloud] = i_ * Horizon_SCAN + j_
                size_of_seg_cloud += 1
        end_ring_index[i_] = size_of_seg_cloud - 1 - 5
    #
    cloud_curvature = np.zeros((N_SCAN * Horizon_SCAN,), dtype=np.float32)
    cloud_neighbor_picked = np.zeros((N_SCAN * Horizon_SCAN), dtype=np.float32)
    for i_ in range(5, size_of_seg_cloud - 5):
        dif_range = sum(segmented_cloud_range[i_ - 5:i_ + 6]) - segmented_cloud_range[i_] * 11
        cloud_curvature[i_] = dif_range ** 2
    for i_ in range(5, size_of_seg_cloud - 6):
        d1 = segmented_cloud_range[i_]
        d2 = segmented_cloud_range[i_ + 1]
        column_dif = abs(int(segmented_cloud_col_ind[i_ + 1] - segmented_cloud_col_ind[i_]))
        if column_dif < 10:
            if d1 - d2 > 0.3:
                cloud_neighbor_picked[i_ - 5:i_ + 1] = 1
            elif d2 - d1 > 0.3:
                cloud_neighbor_picked[i_ + 1:i_ + 7] = 1
        df1 = abs(segmented_cloud_range[i_ - 1] - segmented_cloud_range[i_])
        df2 = abs(segmented_cloud_range[i_ + 1] - segmented_cloud_range[i_])
        if df1 > 0.02 * segmented_cloud_range[i_] and df2 > 0.02 * segmented_cloud_range[i_]:
            cloud_neighbor_picked[i_] = 1
    #
    # none_ground_points = np.zeros((N_SCAN*Horizon_SCAN, 3), dtype=np.float32)
    # none_ground_points_size = 0
    # ground_point = np.zeros((N_SCAN*Horizon_SCAN, 3), dtype=np.float32)
    # ground_point_size = 0
    # for i_ in range(N_SCAN):
    #     for j_ in range(Horizon_SCAN):
    #         if label_mat[i_, j_] > 0 and label_mat[i_, j_] != 999999:
    #             none_ground_points[none_ground_points_size, :] = full_cloud[i_*Horizon_SCAN+j_, :]
    #             none_ground_points_size += 1
    #         if ground_mat[i_, j_] == 1:
    #             ground_point[ground_point_size, :] = full_cloud[i_*Horizon_SCAN+j_, :]
    #             ground_point_size += 1
    # return none_ground_points[:none_ground_points_size, :], ground_point[:ground_point_size, :]
    #
    per_sec_szie = 15
    ray_mat = np.zeros((N_SCAN * n_sec * per_sec_szie, 3), dtype=np.float32)
    ray_size = 0
    for i_ in range(N_SCAN):
        for j_ in range(n_sec):
            sp = int((start_ring_index[i_] * (6 - j_) + end_ring_index[i_] * j_) / n_sec)
            ep = int((start_ring_index[i_] * (5 - j_) + end_ring_index[i_] * (j_ + 1)) / (n_sec - 1))
            if sp >= ep:
                continue
            sort_ed_curvature = np.argsort(cloud_curvature[sp:ep + 1])
            s_ind = 0
            for k_ in sort_ed_curvature:
                k_ += sp
                if s_ind >= per_sec_szie:
                    break
                if cloud_neighbor_picked[k_] == 0 and 0 < cloud_curvature[k_] < surf_threshold and \
                        segmented_cloud_ground_flag[k_] != 1:
                    ray_mat[ray_size, :] = full_cloud[segmented_cloud_index[k_]]
                    ray_size += 1
                    s_ind += 1
    return ray_mat[0:ray_size]


@numba.jit(nopython=True)
def CalculateNormal(points):
    mean = np.zeros((3,), dtype=np.float32)
    for i in range(points.shape[1]):
        mean[i] = points[:, i].mean()
    x = points - mean
    x = np.dot(x.transpose(), x)
    if np.linalg.det(x) == 0:
        return None
    if np.linalg.matrix_rank(x) != 3:
        return None
    if not np.all(np.linalg.eigvals(x) > 0):
        return None
    eig_v, eig_m = np.linalg.eig(x)
    for v in eig_v:
        if not np.isreal(v):
            return None
    if eig_v[2] > eig_v[1] * 0.5:
        return None
    return eig_m[:, 2].astype(np.float32)


@numba.jit(nopython=True)
def DisAngle(norm, ray_point):
    ray = ray_point / np.linalg.norm(ray_point)
    ang_dis = np.degrees(np.arccos(np.dot(norm, ray)))
    if ang_dis > 90:
        return 180 - ang_dis
    return ang_dis


@numba.jit(nopython=True)
def Posi2Ind(point, voxel_size):
    return int(point[0] // voxel_size), int(point[1] // voxel_size), int(point[2] // voxel_size)


def CenterVoxel(voxle_ind, voxel_size):
    return (voxle_ind * voxel_size + (voxle_ind + 1) * voxel_size) / 2


def visitor(voxel, voxel_map: collections.defaultdict, delete_voxel_map: collections.defaultdict):
    if voxel not in voxel_map:
        return True
    voxel_map.pop(voxel)
    delete_voxel_map[voxel] = 2
    return True


@numba.jit(nopython=True)
def walk_voxels(start, end, voxel_size):
    direction = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    s_x, s_y, s_z = x, y, z = Posi2Ind((start[0], start[1], start[2]), voxel_size)
    e_x, e_y, e_z = Posi2Ind((end[0], end[1], end[2]), voxel_size)
    voxel_list = [(s_x, s_y, s_z)]
    # visitor((s_x, s_y, s_z), voxel_map, delete_voxel_map)
    if direction == (0, 0, 0):
        return voxel_list
    if (s_x, s_y, s_z) == (e_x, e_y, e_z):
        return voxel_list
    if direction[0] == 0:
        t_delta_x = None
        step_x = None
        t_max_x = math.inf
        max_multi_x = math.inf
    else:
        step_x = 1 if direction[0] > 0 else -1
        t_delta_x = step_x * voxel_size / direction[0]
        t_max_x = t_delta_x * (1 - (step_x * start[0] / voxel_size) % 1)
        max_multi_x = (e_x - s_x) * step_x
        if step_x == -1 and t_max_x == t_delta_x and s_x != e_x:
            x -= 1
            s_x -= 1
            max_multi_x -= 1
    if direction[1] == 0:
        t_delta_y = None
        step_y = None
        t_max_y = math.inf
        max_multi_y = math.inf
    else:
        step_y = 1 if direction[1] > 0 else -1
        t_delta_y = step_y * voxel_size / direction[1]
        t_max_y = t_delta_y * (1 - (step_y * start[1] / voxel_size) % 1)
        max_multi_y = (e_y - s_y) * step_y
        if step_y == -1 and t_max_y == t_delta_y and s_y != e_y:
            y -= 1
            s_y -= 1
            max_multi_y -= 1
    if direction[2] == 0:
        t_delta_z = None
        step_z = None
        t_max_z = math.inf
        max_multi_z = math.inf
    else:
        step_z = 1 if direction[2] > 0 else -1
        t_delta_z = step_z * voxel_size / direction[2]
        t_max_z = t_delta_z * (1 - (step_z * start[2] / voxel_size) % 1)
        max_multi_z = (e_z - s_z) * step_z
        if step_z == -1 and t_max_z == t_delta_z and s_z != e_z:
            z -= 1
            s_z -= 1
            max_multi_z -= 1
    # visitor((x, y, z), voxel_map, delete_voxel_map)
    voxel_list.append((x, y, z))
    if (x, y, z) == (e_x, e_y, e_z):
        return voxel_list
    t_max_x_start = t_max_x
    t_max_y_start = t_max_y
    t_max_z_start = t_max_z
    multi_x = multi_y = multi_z = 0
    while True:
        min_val = min(t_max_x, t_max_y, t_max_z)
        stepped_x = stepped_y = stepped_z = False
        if min_val == t_max_x:
            multi_x += 1
            x = s_x + multi_x * step_x
            t_max_x = t_max_x_start + multi_x * t_delta_x
            stepped_x = True
        if min_val == t_max_y:
            multi_y += 1
            y = s_y + multi_y * step_y
            t_max_y = t_max_y_start + multi_y * t_delta_y
            stepped_y = True
        if min_val == t_max_z:
            multi_z += 1
            z = s_z + multi_z * step_z
            t_max_z = t_max_z_start + multi_z * t_delta_z
            stepped_z = True
        if ((stepped_x and stepped_y) or (stepped_y and stepped_z) or (stepped_x and stepped_z)) \
                and (step_x == 1 or step_y == 1 or step_z == 1) and (step_x == -1 or step_y == -1 or step_z == -1):
            add_x, add_y, add_z = x, y, z
            if stepped_x:
                if step_x < 0:
                    if multi_x > max_multi_x + 1:
                        break
                    add_x += 1
                elif multi_x > max_multi_x:
                    break
            if stepped_y:
                if step_y < 0:
                    if multi_y > max_multi_y + 1:
                        break
                    add_y += 1
                elif multi_y > max_multi_y:
                    break
            if stepped_z:
                if step_z < 0:
                    if multi_z > max_multi_z + 1:
                        break
                    add_z += 1
                elif multi_z > max_multi_z:
                    break
            # if not visitor((add_x, add_y, add_z), voxel_map, delete_voxel_map):
            #     break
            voxel_list.append((add_x, add_y, add_z))
        if stepped_x and multi_x > max_multi_x:
            break
        if stepped_y and multi_y > max_multi_y:
            break
        if stepped_z and multi_z > max_multi_z:
            break
        # if not visitor((x, y, z), voxel_map, delete_voxel_map):
        #     break
        voxel_list.append((x, y, z))
    return voxel_list


class VoxelMap:
    def __init__(self, origin_pcd_file, save_path, bin_list, voxel_size=0.1):
        # On DS PCD
        self.origin_pcd = None
        self.origin_kd_tree = None
        self.origin_point_cloud = None
        # File
        self.voxel_map = None
        self.delete_voxel_map = None
        self.pcd = None
        self.delete_pcd = None
        self.point_cloud = None
        self.kd_tree = None
        self.ray_list = None
        # Parameter
        self.voxel_size = voxel_size
        self.save_path = save_path
        self.origin_pcd_file = origin_pcd_file
        self.voxel_map_file = os.path.join(save_path, "VoxelMap_voxelmap.pkl")
        self.pcd_file = os.path.join(save_path, "VoxelMap_pcd.pcd")
        self.delete_pcd_file = os.path.join(save_path, "VoxelMap_badpcd.pcd")
        self.ray_file = os.path.join(save_path, "VoxelMap_ray.pkl")
        self.bin_list = bin_list
        if os.path.isfile(self.pcd_file) and os.path.isfile(self.voxel_map_file):
            print("Load PCD and Map from File, Delete It to Regenerate!")
            print(self.pcd_file)
            self.GetData()
        # 1 Load Raw Data
        self.GetOriginData()
        # 2 Do Ray Project
        self.DoRayProject()
        # 3 Save Voxel Map and PCD for Kdtree
        self.SaveVoxelMap()

    def GetData(self):
        self.pcd = open3d.io.read_point_cloud(self.pcd_file)
        self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
        self.point_cloud = np.array(self.pcd.points)

    def GetOriginData(self):
        if os.path.isfile(self.origin_pcd_file):
            self.origin_pcd = open3d.io.read_point_cloud(self.origin_pcd_file)
            self.origin_kd_tree = open3d.geometry.KDTreeFlann(self.origin_pcd)
            self.origin_point_cloud = np.array(self.origin_pcd.points)
            if os.path.isfile(self.voxel_map_file):
                self.voxel_map = pickle.load(open(self.voxel_map_file, "rb"))
                return
            self.voxel_map = collections.defaultdict(int)
            start_time = ttime.time()
            for i in range(self.origin_point_cloud.shape[0]):
                p = self.origin_point_cloud[i, :]
                self.voxel_map[Posi2Ind(p, self.voxel_size)] = 1
                if i % 1000000 == 0:
                    print("Bulid Voxel Map {:.2f}% Cost {:.2f}s".format(i / self.origin_point_cloud.shape[0] * 100,
                                                                        ttime.time() - start_time))
                    start_time = ttime.time()
        else:
            print("No Origin Data Found!")

    def SaveVoxelMap(self):
        if not os.path.isfile(self.voxel_map_file):
            pickle.dump(self.voxel_map, open(self.voxel_map_file, "wb"))
            print("Save Voxel Map!")
        if not os.path.isfile(self.pcd_file):
            voxel_map_size = len(self.voxel_map)
            self.point_cloud = np.zeros((voxel_map_size, 3), dtype=np.float32)
            start_time = ttime.time()
            for i, key in enumerate(self.voxel_map):
                self.point_cloud[i, :] = CenterVoxel(np.array(key), self.voxel_size)
                if i % 100000 == 0:
                    print("Build New Kdtree Map {:.2f}% Cost{:.2f}s".format(i / voxel_map_size * 100,
                                                                            ttime.time() - start_time))
                    start_time = ttime.time()
            self.pcd = open3d.geometry.PointCloud()
            self.pcd.points = open3d.utility.Vector3dVector(self.point_cloud)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            open3d.io.write_point_cloud(self.pcd_file, self.pcd)
            print("Save PCD, Gen KD-Tree And PointCloud For Using Out")
        if not os.path.isfile(self.delete_pcd_file):
            if self.delete_voxel_map is None:
                print("Please Do Ray Handle!")
                return
            bad_voxel_map_size = len(self.delete_voxel_map)
            delete_points = np.zeros((bad_voxel_map_size, 3), dtype=np.float32)
            start_time = ttime.time()
            for i, key in enumerate(self.delete_voxel_map):
                delete_points[i, :] = CenterVoxel(np.array(key), self.voxel_size)
                if i % 10000 == 0:
                    print("Get delete Voxel: {:.2f}% Cost {:.2f}s".format(i / bad_voxel_map_size * 100,
                                                                          ttime.time() - start_time))
                    start_time = ttime.time()
            self.delete_pcd = open3d.geometry.PointCloud()
            self.delete_pcd.points = open3d.utility.Vector3dVector(delete_points)
            open3d.io.write_point_cloud(self.delete_pcd_file, self.delete_pcd)
            print("Save Bad PCD")

    def GetAreaPointCloud(self, center, boundary):
        serach_radius = max(boundary)
        _, p_ind, _ = self.kd_tree.search_radius_vector_3d(np.array(center), serach_radius)
        tmp_point = self.point_cloud[p_ind, :]
        # bb = self.origin_kd_tree.search_radius_vector_3d(np.array(center), serach_radius)
        # cc = self.origin_point_cloud[bb[1], :]
        return tmp_point

    def DoRayProject(self):
        # Get RayList
        ray_count = 0
        if os.path.isfile(self.ray_file):
            print("Load Rays, Delete It to Regenerate")
            print(self.ray_file)
            self.ray_list = pickle.load(open(self.ray_file, "rb"))
        else:
            print("Get Rays ...")
            bin_size = len(self.bin_list)
            start_time = ttime.time()
            self.ray_list = []
            for bin_ind in range(0, bin_size):
                bin_file = self.bin_list[bin_ind][2]
                bin_p_ = self.bin_list[bin_ind][3]
                bin_r_ = self.bin_list[bin_ind][4]
                local_points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, :3]
                local_points = local_points[~np.isnan(local_points).any(axis=1)]

                # """
                if not GroundSegInitFlag:
                    SetParameter(nscan=32, hscan=2000,
                                 low_ang=-16.5, up_ang=10,
                                 ground_ind=15, min_range=5.0,
                                 segment_theta_d=40.0)
                select_points = GetSelectRay(local_points, n_sec=10, surf_threshold=0.2)
                # bin_pcd = open3d.geometry.PointCloud()
                # bin_pcd.points = open3d.utility.Vector3dVector(local_points)
                # bin_kd_tree = open3d.geometry.KDTreeFlann(bin_pcd)
                # select_points = self.GetNormFilter(bin_kd_tree, local_points, select_points, adis=60)
                select_points = Base.TransPointCloud(select_points, bin_r_, bin_p_)
                # """
                # tmp_pcd_show = open3d.geometry.PointCloud()
                # tmp_pcd_show.points = open3d.utility.Vector3dVector(grounds)
                # open3d.visualization.draw_geometries([tmp_pcd_show])
                """
                global_points = Base.TransPointCloud(local_points, bin_r_, bin_p_)
                select_points = self.GetSelectRayPointsGlobal(global_points, local_points, lines=32, sec=50, adis=60)
                """
                """
                bin_pcd = open3d.geometry.PointCloud()
                bin_pcd.points = open3d.utility.Vector3dVector(local_points)
                bin_kd_tree = open3d.geometry.KDTreeFlann(bin_pcd)
                select_points = self.GetSelectRayPointsLocal(bin_kd_tree, local_points, lines=32, sec=50, adis=45)
                select_points = Base.TransPointCloud(select_points, bin_r_, bin_p_)
                """
                self.ray_list.append((bin_p_, select_points))
                ray_count += select_points.shape[0]
                if bin_ind % 200 == 0:
                    print("Get Valid Ray {:.2f}% Cost {:.2f}s Get {:d} Rays".format(bin_ind / bin_size * 100,
                                                                                    ttime.time() - start_time,
                                                                                    ray_count))
                    start_time = ttime.time()
            if len(self.ray_list) != 0:
                pickle.dump(self.ray_list, open(self.ray_file, "wb"))
        # Check Every Ray
        if os.path.isfile(self.delete_pcd_file):
            print("Already Rayed! Delete the file to restart:")
            print(self.delete_pcd_file)
            self.pcd = open3d.io.read_point_cloud(self.delete_pcd_file)
            return
        handle_frame_count = 0
        total_frame = len(self.ray_list)
        start_time = ttime.time()
        self.delete_voxel_map = collections.defaultdict(int)
        for ray_center, ray_mat in self.ray_list:
            for ind in range(ray_mat.shape[0]):
                vis_voxel = walk_voxels(ray_center.reshape((3,)), ray_mat[ind, :], self.voxel_size)
                for v_ind in range(0, len(vis_voxel) - 5):
                    visitor(vis_voxel[v_ind], self.voxel_map, self.delete_voxel_map)
            handle_frame_count += 1
            if handle_frame_count % 200 == 0:
                print("Handle Ray {:.2f}% Cost {:.2f}s".format(handle_frame_count / total_frame * 100,
                                                               ttime.time() - start_time))
                start_time = ttime.time()
        return

    def GetSelectRayPointsGlobal(self, global_points, local_points, lines=32, sec=60, adis=10):
        ray_points = np.zeros((lines * sec, 3), dtype=np.float32)
        ind = 0
        scan_size = global_points.shape[0] // lines
        sec_size = scan_size // sec
        for scan_ind in range(lines):
            for sec_ind in range(sec):
                if int(scan_ind * scan_size + sec_size * sec_ind) >= global_points.shape[0]:
                    break
                g_point = global_points[int(scan_ind * scan_size + sec_size * sec_ind), :]
                l_point = local_points[int(scan_ind * scan_size + sec_size * sec_ind), :]
                _, near_ind, _ = self.origin_kd_tree.search_radius_vector_3d(g_point, self.voxel_size * math.sqrt(3))
                near_points = self.origin_point_cloud[near_ind]
                if near_points.shape[0] < 3:
                    continue
                p_norm = CalculateNormal(near_points)
                if p_norm is None:
                    continue
                dis = DisAngle(p_norm, l_point)
                if dis > adis:
                    continue
                ray_points[ind, :] = g_point
                ind += 1
        return ray_points[0:ind, :]

    def GetSelectRayPointsLocal(self, local_kd_tree, local_points, lines=32, sec=60, adis=10):
        ray_points = np.zeros((lines * sec, 3), dtype=np.float32)
        ind = 0
        scan_size = local_points.shape[0] // lines
        sec_size = scan_size // sec
        for scan_ind in range(lines):
            for sec_ind in range(sec):
                if int(scan_ind * scan_size + sec_size * sec_ind) >= local_points.shape[0]:
                    break
                l_point = local_points[int(scan_ind * scan_size + sec_size * sec_ind), :]
                _, near_ind, _ = local_kd_tree.search_radius_vector_3d(l_point, self.voxel_size * math.sqrt(3))
                near_points = local_points[near_ind]
                if near_points.shape[0] < 3:
                    continue
                p_norm = CalculateNormal(near_points)
                if p_norm is None:
                    continue
                dis = DisAngle(p_norm, l_point)
                if dis > adis:
                    continue
                ray_points[ind, :] = l_point
                ind += 1
        return ray_points[0:ind, :]

    def GetNormFilter(self, local_kd_tree, local_points, filter_, adis=10):
        ray_points = np.zeros((filter_.shape[0], 3), dtype=np.float32)
        ind = 0
        for p_ind in range(filter_.shape[0]):
            l_point = filter_[p_ind, :]
            _, near_id, _ = local_kd_tree.search_radius_vector_3d(l_point, self.voxel_size * math.sqrt(3) * 3)
            near_points = local_points[near_id]
            if near_points.shape[0] < 3:
                continue
            p_norm = CalculateNormal(near_points)
            if p_norm is None:
                continue
            dis = DisAngle(p_norm, l_point)
            if dis > adis:
                continue
            ray_points[ind, :] = l_point
            ind += 1
        return ray_points[0:ind, :]


def GetBinList(bin_dir, bin_times_file, pose_vec_file, save_path=None):
    if save_path is not None and os.path.isfile(save_path):
        bin_info = pickle.load(open(save_path, "rb"))
        return bin_info
    bin_info = []
    bin_times = np.loadtxt(bin_times_file)
    pose_vec = np.loadtxt(pose_vec_file, skiprows=2)
    bin_to_use = sorted(glob.glob(bin_dir + '/*.bin'))
    for bin_file in bin_to_use:
        bin_id = int(bin_file.split('/')[-1].split('.')[-2])
        now_time = bin_times[bin_id]
        pose_ind = Base.BinFind(pose_vec, now_time, 0, pose_vec.shape[0] - 1)
        if pose_ind == -1:
            continue
        p_ = pose_vec[pose_ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[pose_ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        bin_info.append((now_time, pose_ind, bin_file, p_, r_))
    if save_path is not None:
        pickle.dump(bin_info, open(save_path, "wb"))
    return bin_info


def VisRayHandle(map: VoxelMap):
    center_pcd = open3d.geometry.PointCloud()
    ray_points_list = np.zeros((0, 3), dtype=np.float32)
    cneter_points = np.zeros((0, 3), dtype=np.float32)
    line_list = []
    points_size = 0
    for ray_ind in range(300, len(map.ray_list), 10):
        ray_center, ray_mat = map.ray_list[ray_ind]
        if ray_ind > 400:
            break
        ray_points_list = np.row_stack((ray_points_list, ray_center.reshape((1, 3))))
        ray_points_list = np.row_stack((ray_points_list, ray_mat))
        cneter_points = np.row_stack((cneter_points, ray_center.reshape((1, 3))))
        for i in range(1, ray_mat.shape[0] + 1):
            line_list.append([points_size, points_size + i])
        points_size += ray_mat.shape[0] + 1
    center_pcd.points = open3d.utility.Vector3dVector(cneter_points)
    center_pcd.paint_uniform_color([0, 0, 0])
    lines_pcd = open3d.geometry.LineSet()
    lines_pcd.points = open3d.utility.Vector3dVector(ray_points_list)
    lines_pcd.lines = open3d.utility.Vector2iVector(line_list)

    delete_pcd = map.delete_pcd
    axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
    # open3d.visualization.draw_geometries([map.pcd, axis_pcd, lines_pcd, center_pcd])
    # open3d.visualization.draw_geometries([map.pcd, axis_pcd, delete_pcd, center_pcd])
    open3d.visualization.draw_geometries([axis_pcd, delete_pcd])


if __name__ == "__main__":
    bin_list = GetBinList(bin_dir="/home/qh/YES/dlut/Daquan19/bin",
                          bin_times_file="/home/qh/YES/dlut/Daquan19/timestamp",
                          pose_vec_file="/home/qh/YES/dlut/Daquan19/liosave/sam2.txt",
                          save_path="/home/qh/YES/dlut/Daquan19/bin_info.pkl")
    map19 = VoxelMap(origin_pcd_file="/home/qh/YES/dlut/Daquan19/liosave/GlobalMapDS1.pcd",
                     save_path="/home/qh/YES/dlut/Daquan19/",
                     bin_list=bin_list)
    # open3d.visualization.draw_geometries([lines_pcd, center_pcd])

    VisRayHandle(map19)
    print(123)
    #

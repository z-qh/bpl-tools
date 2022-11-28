import collections
import math
import sys
import time

import numpy as np
from numba import cuda
import numba
import open3d
import matplotlib.pyplot as plt
import pickle
import os
import rosbag
import sensor_msgs.point_cloud2 as pc2
import time as ttime
from scipy.spatial.transform import Rotation
from scipy.spatial.kdtree import KDTree

global_down_sample_size = 0.5

np.set_printoptions(suppress=True, precision=5, threshold=sys.maxsize, linewidth=sys.maxsize)


# -------------------------------- Scan Context --------------------------------
def xy2theta(x, y):
    if x >= 0 and y >= 0:
        theta = 180 / np.pi * np.arctan(y / x)
    elif x < 0 and y >= 0:
        theta = 180 - ((180 / np.pi) * np.arctan(y / (-x)))
    elif x < 0 and y < 0:
        theta = 180 + ((180 / np.pi) * np.arctan(y / x))
    else:  # x >= 0 and y < 0:
        theta = 360 - ((180 / np.pi) * np.arctan((-y) / x))
    return theta


def pt2rs(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    z = point[2]
    if x == 0.0:
        x = 0.001
    if y == 0.0:
        y = 0.001
    theta = xy2theta(x, y)
    faraway = np.sqrt(x * x + y * y)
    idx_ring = np.divmod(faraway, gap_ring)[0]
    idx_sector = np.divmod(theta, gap_sector)[0]
    if idx_ring >= num_ring:
        idx_ring = num_ring - 1
    return int(idx_ring), int(idx_sector)


def ptcloud2sc(ptcloud, num_sector, num_ring, min_length, max_length, lidar_hei):
    num_points = ptcloud.shape[0]
    gap_ring = max_length / num_ring
    gap_sector = 360 / num_sector
    enough_large = 1000
    sc_storage = np.zeros([enough_large, num_ring, num_sector])
    sc_counter = np.zeros([num_ring, num_sector])
    for pt_idx in range(num_points):
        point = ptcloud[pt_idx, :]
        lenght = np.sqrt(point[0] ** 2 + point[1] ** 2)
        if lenght < min_length or lenght > max_length:
            continue
        point_height = point[2] + lidar_hei
        idx_ring, idx_sector = pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
        if sc_counter[idx_ring, idx_sector] >= enough_large:
            continue
        sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
        sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1
    sc = np.amax(sc_storage, axis=0)
    return sc


def load_velo_scan(cloud_data, path, channle):
    if cloud_data is not None:
        ptcloud_xyz = cloud_data.reshape((-1, channle))
        return ptcloud_xyz[:, 0:3]
    elif path is not None:
        scan = np.fromfile(path, dtype=np.float32).reshape((-1, channle))
        ptcloud_xyz = scan[:, 0:3]
        return ptcloud_xyz
    else:
        print("Neither numpy data nor cloud path !")


def genSCs(cloud_data, path, channle, ring_res, sector_res, min_dis, max_dis, lidar_hei,
           downcell_size, viz=False):
    ptcloud_xyz = load_velo_scan(cloud_data, path, channle)
    print("The number of original points: " + str(ptcloud_xyz.shape))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(ptcloud_xyz)
    downpcd = pcd.voxel_down_sample(voxel_size=downcell_size)
    ptcloud_xyz_downed = np.asarray(downpcd.points)
    print("The number of downsampled points: " + str(ptcloud_xyz_downed.shape))
    scan_context_array = []
    if viz:
        open3d.visualization.draw_geometries([downpcd])
    for res in range(len(sector_res)):
        num_sector = sector_res[res]
        num_ring = ring_res[res]
        sc = ptcloud2sc(ptcloud_xyz_downed, num_sector, num_ring, min_dis, max_dis, lidar_hei)
        scan_context_array.append(sc)
    return scan_context_array


def plot_multiple_sc(SCs, save_path=None):
    if len(SCs) == 1:
        fig, axes = plt.subplots(nrows=1)
        axes.set_title('Cloud Descriptor')
        axes.imshow(SCs[0], cmap="binary")  # gray
        axes.invert_yaxis()
        fig.gca().set_aspect(1)
    else:
        num_res = len(SCs)
        fig, axes = plt.subplots(nrows=num_res)
        axes[0].set_title('Scan Contexts with multiple resolutions')
        for ax, res in zip(axes, range(num_res)):
            ax.imshow(SCs[res], cmap="gray")
            ax.invert_yaxis()
    if save_path is not None:
        plt.savefig(save_path, dpi=600)
    plt.show()


# The higher the score, the more similar the scene
def SimilarityOrigin(sc1, sc2):
    num_sectors = sc1.shape[1]
    # repeate to move 1 columns
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        # Shift
        one_step = 1  # const
        sc1 = np.roll(sc1, one_step, axis=1)  # columne shift
        # compare
        sum_of_cos_sim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if (~np.any(col_j_1) or ~np.any(col_j_2)):
                continue
            # calc sim
            cos_similarity = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cos_sim = sum_of_cos_sim + cos_similarity
            num_col_engaged = num_col_engaged + 1
        # devided by num_col_engaged: So, even if there are many columns that are excluded from the calculation, we
        # can get high scores if other columns are well fit.
        sim_for_each_cols[i] = sum_of_cos_sim / num_col_engaged
    sim = max(sim_for_each_cols)
    return sim


def SimilaritySimple(sc1, sc2):
    hei = sc1.shape[0]
    wid = sc1.shape[1]
    temp_mat = np.zeros((wid, wid), dtype=np.float32)
    temp_vec = np.zeros((wid, 1), dtype=np.float32)
    val = np.zeros((1, 1), dtype=np.float32)
    for i in range(wid):
        for j in range(wid):
            dot = 0
            norm1 = 0
            norm2 = 0
            for k in range(hei):
                dot += sc1[k, i] * sc2[k, j]
                norm1 += sc1[k, i] ** 2
                norm2 += sc2[k, j] ** 2
            if i < j:
                x_ = j - i
            elif i > j:
                x_ = j + wid - i
            else:
                x_ = 0
            y_ = j
            if norm1 == 0 or norm2 == 0:
                temp_mat[x_, y_] = math.nan
            else:
                temp_mat[x_, y_] = dot / (math.sqrt(norm1) * math.sqrt(norm2))
    for i in range(wid):
        valid_line = 0
        for j in range(wid):
            if math.isnan(temp_mat[i, j]):
                continue
            temp_vec[i, 0] += temp_mat[i, j]
            valid_line += 1
        temp_vec[i, 0] /= valid_line
    val[0, 0] = -1
    for i in range(wid):
        if temp_vec[i, 0] > val[0, 0]:
            val[0, 0] = temp_vec[i, 0]


@numba.jit(nopython=True)
def SimilarityNumba(sc1, sc2):
    num_rings = sc1.shape[0]
    num_sectors = sc1.shape[1]
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        sc1 = np.roll(sc1.transpose(), num_rings).transpose()
        sum_of_cos_sim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if ~np.any(col_j_1) or ~np.any(col_j_2):
                continue
            cos_similarity = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cos_sim = sum_of_cos_sim + cos_similarity
            num_col_engaged = num_col_engaged + 1
        sim_for_each_cols[i] = sum_of_cos_sim / num_col_engaged
    sim = max(sim_for_each_cols)
    return sim


COLUMN_SIZE: int = 80
GRID_BLOCKX_SEIZ: int = 1024
GRID_BLOCKY_SIEZ: int = 1024
BLOCK_THREAD_SIZE: int = 1024
BLOCK_THREAD_VOXEL_LEN: int = int(math.sqrt(BLOCK_THREAD_SIZE))


# The size of the calculated matrix determined here
# 512  x 512  x 1024 =  268435456 = 16384^2 : 16384 topo nodes
# 1024 x 1024 x 1024 = 1073741824 = 32768^2 : 32768 topo nodes

@cuda.jit
def UpperTriangleDotArea(data, dest, xs, xe, ys, ye):
    # data 1W X 40 X 80
    # dest 1W X 1W
    # need < 1W
    # -------------- INDEX
    tx = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.x + cuda.threadIdx.x // BLOCK_THREAD_VOXEL_LEN
    ty = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.y + cuda.threadIdx.x % BLOCK_THREAD_VOXEL_LEN
    if tx < xs or tx > xe or ty < ys or ty > ye:
        return
    if tx > ty:
        return
    if tx == ty:
        dest[tx, ty] = 1.0
        return
    # -------------- CAL PROCESS
    hei = data[tx].shape[0]
    wid = data[tx].shape[1]
    temp_mat = cuda.local.array((COLUMN_SIZE, COLUMN_SIZE), numba.types.float32)
    temp_vec = cuda.local.array((COLUMN_SIZE, 1), numba.types.float32)
    temp_out = cuda.local.array((1, 1), numba.types.float32)
    for mat_ind_m in range(wid):
        for mat_ind_n in range(wid):
            dot = 0
            norm1 = 0
            norm2 = 0
            for k in range(hei):
                dot += data[tx][k, mat_ind_m] * data[ty][k, mat_ind_n]
                norm1 += data[tx][k, mat_ind_m] ** 2
                norm2 += data[ty][k, mat_ind_n] ** 2
            if mat_ind_m < mat_ind_n:
                x_ = mat_ind_n - mat_ind_m
            elif mat_ind_m > mat_ind_n:
                x_ = mat_ind_n + wid - mat_ind_m
            else:
                x_ = 0
            y_ = mat_ind_n
            if norm1 == 0 or norm2 == 0:
                temp_mat[x_, y_] = math.nan
            else:
                temp_mat[x_, y_] = dot / (math.sqrt(norm1) * math.sqrt(norm2))
    for i in range(wid):
        valid_line = 0
        for j in range(wid):
            if math.isnan(temp_mat[i, j]):
                continue
            temp_vec[i, 0] += temp_mat[i, j]
            valid_line += 1
        temp_vec[i, 0] /= valid_line
    temp_out[0, 0] = -1
    for i in range(wid):
        if temp_vec[i, 0] > temp_out[0, 0]:
            temp_out[0, 0] = temp_vec[i, 0]
    dest[tx, ty] = temp_out[0, 0]
    cuda.syncthreads()


class ScanContext:
    viz = False
    downcell_size = 1.0
    lidar_height = 2.0
    # sector_res = np.array([45, 90, 180, 360, 720])
    # ring_res = np.array([10, 20, 40, 80, 160])
    sector_res = np.array([80])
    ring_res = np.array([40])
    min_dis = 5
    max_dis = 100
    SCs = []

    def __init__(self, cloud_data=None, path=None, channle=3):
        self.SCs = genSCs(cloud_data, path, channle, self.ring_res, self.sector_res, self.min_dis, self.max_dis,
                          self.lidar_height,
                          self.downcell_size)


# -------------------------------- Scan Context --------------------------------


# -------------------------------- Down Sample  --------------------------------
def DownSamplePointCloud(input, output, ds):
    # input = "/home/qh/YES/dlut/Daquan/liosave/GlobalMap.pcd"
    # output = "/home/qh/YES/dlut/Daquan/liosave/GlobalMapDS.pcd"
    point_cloud = open3d.io.read_point_cloud(input)
    point_cloud_ds = point_cloud.voxel_down_sample(voxel_size=ds)
    open3d.io.write_point_cloud(output, point_cloud_ds)


# -------------------------------- Down Sample  --------------------------------


# -------------------------------- Topo Node --------------------------------

@numba.jit(nopython=True)
def TransPointCloud(point_cloud, r_, t_):
    return (np.dot(r_, point_cloud.transpose()[0:3, :]) + t_).transpose()


@numba.jit(nopython=True)
def TransInvPointCloud(point_cloud, r_, t_):
    return (np.dot(r_.transpose(), point_cloud.transpose()[0:3, :] - t_.astype(np.float32))).transpose()


class TopoNode:
    id = -1
    position = np.zeros((3, 1), dtype=np.float64)
    boundary = (0, 0, 0, 0, 0, 0)
    rotation = np.zeros((3, 3), dtype=np.float64)
    time = -1
    SCs = []
    edge = []

    def __init__(self, id_, p_, r_, bound_, time_):
        self.id = id_
        self.position = p_
        self.rotation = r_
        self.boundary = bound_
        self.time = time_


def getBound(point_cloud):
    minx = min(point_cloud[:, 0])
    maxx = max(point_cloud[:, 0])
    miny = min(point_cloud[:, 1])
    maxy = max(point_cloud[:, 1])
    minz = min(point_cloud[:, 2])
    maxz = max(point_cloud[:, 2])
    return minx, maxx, miny, maxy, minz, maxz


def genTopoSC(topo_node, point_cloud, ch):
    topo_node.SCs = ScanContext(point_cloud, path=None, channle=ch).SCs
    return topo_node


# -------------------------------- Topo Node --------------------------------

# -------------------------------- Voxel Map --------------------------------
class CloudPiece:
    def __init__(self, i_, j_, k_, cloud_):
        self.idi = i_
        self.idj = j_
        self.idk = k_
        self.data = cloud_
        self.size = 1

    def InsertPoint(self, point):
        self.data = np.row_stack((self.data, point))

    def __len__(self):
        return self.data.shape[0]

    def __str__(self):
        return "{:d} {:d} {:d} ".format(self.idi, self.idj, self.idk) + str(self.data)


def Posi2Ind(poi_x, poi_y, poi_z, x_resolution, y_resolution, z_resolution):
    ind_i = math.floor(poi_y / y_resolution)
    ind_j = math.floor(poi_x / x_resolution)
    ind_k = math.floor(poi_z / z_resolution)
    return ind_i, ind_j, ind_k


def Ind2PosiRange(ind_i, ind_j, ind_k, x_resolution, y_resolution, z_resolution):
    poi_x_min = x_resolution * ind_j
    poi_x_max = poi_x_min + x_resolution
    poi_y_min = y_resolution * ind_i
    poi_y_max = poi_y_min + y_resolution
    poi_z_min = z_resolution * ind_k
    poi_z_max = poi_z_min + z_resolution
    return poi_x_min, poi_x_max, poi_y_min, poi_y_max, poi_z_min, poi_z_max


def Ind2Key(ind_i, ind_j, ind_k):
    return "{:d} {:d} {:d}".format(ind_i, ind_j, ind_k)


def CreateVoxelFromFullCloud(mapdic, point_cloud, x_resolution, y_resolution, z_resolution):
    # point_cloud : x y z i l... as a row
    point_size = point_cloud.shape[0]
    start_time = ttime.time()
    used_time = 0
    for point_i in range(point_size):
        point = point_cloud[point_i, :]
        now_x = point[0]
        now_y = point[1]
        now_z = point[2]
        ind_i, ind_j, ind_k = Posi2Ind(now_x, now_y, now_z, x_resolution, y_resolution, z_resolution)
        key = Ind2Key(ind_i, ind_j, ind_k)
        voxel = mapdic.get(key)
        if voxel is None:
            mapdic[key] = point
        if point_i % 100000 == 0:
            used_time += ttime.time() - start_time
            print("Build Voxel:{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(point_i / point_size * 100,
                                                                          ttime.time() - start_time,
                                                                          used_time))
            start_time = ttime.time()


class VoxelMap:
    def __init__(self, full_cloud, x_res, y_res, z_res):
        self.x_resolution = x_res
        self.y_resolution = y_res
        self.z_resolution = z_res
        self.voxel_dict = dict()
        self.boundary = [0, 0, 0, 0, 0, 0]
        self.boundary_ind = [0, 0, 0, 0, 0, 0]
        if full_cloud is not None:
            CreateVoxelFromFullCloud(self.voxel_dict, full_cloud, self.x_resolution,
                                     self.y_resolution, self.z_resolution)
            self.boundary = getBound(full_cloud)
            self.boundary_ind[0:5:2] = Posi2Ind(self.boundary[0], self.boundary[2],
                                                self.boundary[4], self.x_resolution,
                                                self.y_resolution, self.z_resolution)
            self.boundary_ind[1:6:2] = Posi2Ind(self.boundary[1], self.boundary[3],
                                                self.boundary[5], self.x_resolution,
                                                self.y_resolution, self.z_resolution)


def GetAreaPointCloud(voxel_dict, center, boundary, x_resolution, y_resolution, z_resolution):
    tmp_points = np.zeros((0, 3), dtype=np.float32)
    voxel_size_x = int(math.ceil(max(abs(boundary[0]), abs(boundary[1])) / x_resolution))
    voxel_size_y = int(math.ceil(max(abs(boundary[2]), abs(boundary[3])) / y_resolution))
    voxel_size_z = int(math.ceil(max(abs(boundary[4]), abs(boundary[5])) / z_resolution))
    now_i, now_j, now_k = Posi2Ind(center[0], center[1], center[2],
                                   x_resolution,
                                   y_resolution,
                                   z_resolution)
    max_x_y = max(voxel_size_x, voxel_size_y)
    radius2 = max(voxel_size_x / x_resolution, voxel_size_y / y_resolution) ** 2
    for k in range(now_k - voxel_size_z, now_k + voxel_size_z):
        for addi in range(-max_x_y, max_x_y):
            for addj in range(-max_x_y, max_x_y):
                i = now_i + addi
                j = now_j + addj
                if ((addi * x_resolution) ** 2 + (addj * y_resolution) ** 2) > radius2:
                    continue
                key = "{:d} {:d} {:d}".format(i, j, k)
                voxel = voxel_dict.get(key)
                if voxel is not None:
                    tmp_points = np.row_stack((tmp_points, voxel))
    return tmp_points


def GetVoxelMap(cloud_path=None, path=None, x_res=0.5, y_res=0.5, z_res=0.5):
    if cloud_path is not None:
        pcd_load = open3d.io.read_point_cloud(cloud_path)
        points_xyz = np.asarray(pcd_load.points, dtype=np.float32)
        res = VoxelMap(points_xyz, x_res, y_res, z_res)
        pickle.dump(res, open(path, "wb"))
    elif path is not None:
        start_time = ttime.time()
        res = pickle.load(open(path, "rb"))
        print("Load Voxel Map Cost {:.2f}s".format(ttime.time() - start_time))
    else:
        print("ERROR when get parameter!")
        res = None
    return res


# -------------------------------- Voxel Map --------------------------------


# -------------------------------- Ours Method ------------------------------
def GetPoseVec(pose_file=None, skip=2):
    # "/home/qh/YES/dlut/Daquan/liosave/sam2.txt"
    # time x y z tw tx ty tz
    pose_vec = np.loadtxt(pose_file, skiprows=skip)
    return pose_vec


def BinFind(pose_vec, x, l, r):
    if r >= l:
        mid = int(l + (r - l) / 2)
        if abs(pose_vec[mid, 0] - x) < 0.05:
            return mid
        elif pose_vec[mid, 0] > x:
            return BinFind(pose_vec, x, l, mid - 1)
        else:
            return BinFind(pose_vec, x, mid + 1, r)
    else:
        return -1


def GetAccFullTopoInfo(pose_vec=None,
                       bag_file=None,
                       lidar_topic=None,
                       acc_full_topo_info_path=None,
                       load=False):
    if load and acc_full_topo_info_path is not None:
        accfullTopoInfo = pickle.load(open(acc_full_topo_info_path, "rb"))
        return accfullTopoInfo
    accfullTopoInfo = []
    if bag_file is None or lidar_topic is None:
        print("Bag File Not Exist !")
        return
    bag = rosbag.Bag(bag_file)
    bag_clouds = bag.read_messages(lidar_topic)
    bag_count_info = bag.get_message_count(lidar_topic)
    count = 0
    start_time = ttime.time()
    used_time = 0
    for topic, msg, rtime in bag_clouds:
        now_time = rtime.to_sec()
        ind = BinFind(pose_vec, now_time, 0, pose_vec.shape[0] - 1)
        if ind == -1:
            continue
        lidar = pc2.read_points(msg)
        p_ = pose_vec[ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        points = np.array(list(lidar))
        boundary = getBound(points)
        accfullTopoInfo.append(TopoNode(count, p_, r_, boundary, now_time))
        count += 1
        if count % 100 == 0:
            used_time += ttime.time() - start_time
            print("GetTopoInfo :{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(count / bag_count_info * 100,
                                                                           ttime.time() - start_time,
                                                                           used_time))
            start_time = ttime.time()
    pickle.dump(accfullTopoInfo, open(acc_full_topo_info_path, "wb"))
    return accfullTopoInfo


def GetAccFullTopo(accmap=None, topo_info=None, acc_fulltopo_path=None, load=False):
    if load and acc_fulltopo_path is not None:
        accfullTopo = pickle.load(open(acc_fulltopo_path, "rb"))
        return accfullTopo
    if accmap is None or topo_info is None:
        print("Get Data Error")
        return
    accfullTopo = []
    start_time = ttime.time()
    used_time = 0
    count = 0
    for ind in range(0, len(topo_info)):
        count += 1
        now_topo_info = topo_info[ind]
        topo_cloud = GetAreaPointCloud(accmap.voxel_dict, now_topo_info.position, now_topo_info.boundary,
                                       accmap.x_resolution, accmap.y_resolution, accmap.z_resolution)
        topo_cloud = TransInvPointCloud(topo_cloud, now_topo_info.rotation, now_topo_info.position)
        if count % 100 == 0:
            used_time += ttime.time() - start_time
            print("BulidTopo :{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(count / len(topo_info) * 100,
                                                                         ttime.time() - start_time,
                                                                         used_time))
            start_time = ttime.time()
        accfullTopo.append(genTopoSC(now_topo_info, topo_cloud, ch=3))
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
        # open3d.visualization.draw_geometries([pcd, axis_pcd])
        # plot_multiple_sc(accfullTopo[-1].SCs, save_path=None)
    pickle.dump(accfullTopo, open(acc_fulltopo_path, "wb"))
    return accfullTopo


def GetFullTopoConnectInfo(full_topo, connect_path=None, load=False):
    if load and connect_path is not None:
        start_time = ttime.time()
        acc_connect_info = pickle.load(open(connect_path, "rb"))
        print("load connect info :{:.2f}s".format(ttime.time() - start_time))
        return acc_connect_info
    start_time = ttime.time()
    total_line = len(full_topo)
    np_data = np.zeros((total_line, 40, 80), dtype=np.float32)
    sc_sc_sim = np.zeros((total_line, total_line), dtype=np.float32)
    for i in (range(total_line)):
        np_data[i] = full_topo[i].SCs[0]
    np_data_gpu = cuda.to_device(np_data)
    gpu_res = cuda.to_device(sc_sc_sim)
    UpperTriangleDotArea[(GRID_BLOCKX_SEIZ, GRID_BLOCKY_SIEZ), BLOCK_THREAD_SIZE] \
        (np_data_gpu, gpu_res, 0, total_line-1, 0, total_line-1)
    cuda.synchronize()
    gpu_ans = gpu_res.copy_to_host()
    pickle.dump(gpu_ans, open(connect_path, "wb"))
    print("GPU 1060 solver connect info :{:.2f}s".format(ttime.time() - start_time))
    return gpu_ans


def GenAccTopoNodeBySim(acc_full_topo, acc_full_topo_connect, sim_threshold):
    topo_node_ind = []
    for ind in range(len(acc_full_topo)):
        now_node = acc_full_topo[ind]
        if len(topo_node_ind) == 0:
            topo_node_ind.append(ind)
            continue
        pass_sim = acc_full_topo_connect[ind, topo_node_ind]
        max_pass_sim = min(pass_sim)
        if max_pass_sim < sim_threshold:
            topo_node_ind.append(ind)
    return topo_node_ind


# -------------------------------- Ours Method ------------------------------


# -------------------------------- Appearance Method ------------------------------

def GetAppearanceFullTopoInfo(pose_vec=None,
                              bag_file=None,
                              lidar_topic=None,
                              appearance_full_topo_info_path=None,
                              load=False):
    if load and appearance_full_topo_info_path is not None:
        appearance_fullTopoInfo = pickle.load(open(appearance_full_topo_info_path, "rb"))
        return appearance_fullTopoInfo
    appearance_fullTopoInfo = []
    if bag_file is None or lidar_topic is None:
        print("Bag File Not Exist !")
        return
    bag = rosbag.Bag(bag_file)
    bag_clouds = bag.read_messages(lidar_topic)
    bag_count_info = bag.get_message_count(lidar_topic)
    count = 0
    start_time = ttime.time()
    used_time = 0
    for topic, msg, rtime in bag_clouds:
        now_time = rtime.to_sec()
        ind = BinFind(pose_vec, now_time, 0, pose_vec.shape[0] - 1)
        if ind == -1:
            continue
        lidar = pc2.read_points(msg)
        p_ = pose_vec[ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        points = np.array(list(lidar))
        boundary = getBound(points)
        appearance_fullTopoInfo.append(genTopoSC(TopoNode(count, p_, r_, boundary, now_time), points, ch=5))
        count += 1
        if count % 100 == 0:
            used_time += ttime.time() - start_time
            print("GetTopoInfo :{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(count / bag_count_info * 100,
                                                                           ttime.time() - start_time,
                                                                           used_time))
            start_time = ttime.time()
    pickle.dump(appearance_fullTopoInfo, open(appearance_full_topo_info_path, "wb"))
    return appearance_fullTopoInfo


def GenAppTopoNodeBySim(app_full_topo, app_full_topo_connect, sim_threshold):
    topo_node_ind = []
    for ind in range(len(app_full_topo)):
        now_node = app_full_topo[ind]
        if len(topo_node_ind) == 0:
            topo_node_ind.append(ind)
            continue
        pass_sim = app_full_topo_connect[ind, topo_node_ind]
        max_pass_sim = min(pass_sim)
        if max_pass_sim < sim_threshold:
            topo_node_ind.append(ind)
    return topo_node_ind


# -------------------------------- Appearance Method ------------------------------

# -------------------------------- Density Method ------------------------------

def TopoNodeDistance(t1, t2):
    return (t1.position - t2.position).norm()


def GenTopoNodeByDensity(app_full_topo, density):
    topo_node_ind = []
    tree_data = np.zeros((0,3), dtype=np.float32)
    for ind in range(len(app_full_topo)):
        now_node = app_full_topo[ind]
        if len(topo_node_ind) == 0:
            topo_node_ind.append(ind)
            tree_data = np.row_stack((tree_data, now_node.position))
            continue
        tree = KDTree(data=tree_data)
        pass_min_dis = tree.query(now_node.position)
        if pass_min_dis > density:
            topo_node_ind.append(ind)
            tree_data = np.row_stack((tree_data, now_node.position))
    return topo_node_ind

# -------------------------------- Density Method ------------------------------

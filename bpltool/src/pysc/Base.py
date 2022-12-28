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
from scipy.spatial.kdtree import KDTree
import matplotlib.pyplot as plt
import collections
import glob

global_down_sample_size = 0.2

np.set_printoptions(suppress=True, precision=5, threshold=sys.maxsize, linewidth=sys.maxsize)


# -------------------------------- Scan Context --------------------------------
# 描述子生成支撑函数，计算点的角度
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

# 描述子提取支撑函数，计算点的行列索引
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

# 描述子生成支撑函数 提取描述子矩阵
def ptcloud2sc(ptcloud, num_sector, num_ring, min_length, max_length, lidar_hei):
    num_points = ptcloud.shape[0]
    gap_ring = max_length / num_ring
    gap_sector = 360 / num_sector
    enough_large = 1000
    sc_storage = np.zeros([enough_large, num_ring, num_sector])
    sc_counter = np.zeros([num_ring, num_sector])
    for pt_idx in range(num_points):
        point = ptcloud[pt_idx, :]
        if True in np.isnan(point):
            continue
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

# 描述子提取支撑函数，获取bin点云或者整理现有点云
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

# 生成给定参数和点云的描述子
def genSCs(cloud_data, path, channle, ring_res, sector_res, min_dis, max_dis, lidar_hei,
           downcell_size, viz=False):
    ptcloud_xyz = load_velo_scan(cloud_data, path, channle)
    # print("The number of original points: " + str(ptcloud_xyz.shape))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(ptcloud_xyz)
    downpcd = pcd.voxel_down_sample(voxel_size=downcell_size)
    ptcloud_xyz_downed = np.asarray(downpcd.points)
    # print("The number of downsampled points: " + str(ptcloud_xyz_downed.shape))
    scan_context_array = []
    if viz:
        open3d.visualization.draw_geometries([downpcd])
    for res in range(len(sector_res)):
        num_sector = sector_res[res]
        num_ring = ring_res[res]
        sc = ptcloud2sc(ptcloud_xyz_downed, num_sector, num_ring, min_dis, max_dis, lidar_hei)
        scan_context_array.append(sc)
    return scan_context_array

# 画多个描述子的图像，通常SCs只有一个描述子
def plot_multiple_sc(SCs, save_path=None, vis=True):
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
        plt.savefig(save_path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()


# 使用numpy的言简意赅计算相似度版本函数
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

# 模拟使用最简单的函数而不是numpy函数来计算相似度
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
        temp_vec[i, 0] = 0.0
        for j in range(wid):
            if math.isnan(temp_mat[i, j]):
                continue
            temp_vec[i, 0] += temp_mat[i, j]
            valid_line += 1
        if valid_line == 0:
            temp_vec[i, 0] = 0
        else:
            temp_vec[i, 0] /= valid_line
    val[0, 0] = -1
    for i in range(wid):
        if temp_vec[i, 0] > val[0, 0]:
            val[0, 0] = temp_vec[i, 0]

# 使用Numba加速计算两个描述子之间的相似度
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

# 计算相似度矩阵需要的CUDA和kernel的大小和参数
COLUMN_SIZE: int = 80
GRID_BLOCKX_SEIZ: int = 1024
GRID_BLOCKY_SIEZ: int = 1024
BLOCK_THREAD_SIZE: int = 1024
BLOCK_THREAD_VOXEL_LEN: int = int(math.sqrt(BLOCK_THREAD_SIZE))

# The size of the calculated matrix determined here
# 512  x 512  x 1024 =  268435456 = 16384^2 : 16384 topo nodes
# 1024 x 1024 x 1024 = 1073741824 = 32768^2 : 32768 topo nodes

# 计算一个描述子列表自己和自己的相似度矩阵，输入是数据
# 需要指定输出矩阵的需要计算的位置
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
        temp_vec[i, 0] = 0.0
        for j in range(wid):
            if math.isnan(temp_mat[i, j]):
                continue
            temp_vec[i, 0] += temp_mat[i, j]
            valid_line += 1
        if valid_line == 0:
            temp_vec[i, 0] = 0
        else:
            temp_vec[i, 0] /= valid_line
    temp_out[0, 0] = -1
    for i in range(wid):
        if temp_vec[i, 0] > temp_out[0, 0]:
            temp_out[0, 0] = temp_vec[i, 0]
    dest[tx, ty] = temp_out[0, 0]
    cuda.syncthreads()

# 给定两个描述子的集合，计算一个相似度矩阵，是一个上三角矩阵
# 输入是两个参数和全零的相似度矩阵作为结果来保存，两个矩阵分别要计算的起始和终止位置
@cuda.jit
def UpperTriangleDotArea2(data1, data2, dest, xs, xe, ys, ye):
    # data 1W X 40 X 80
    # dest 1W X 1W
    # need < 1W
    # -------------- INDEX
    tx = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.x + cuda.threadIdx.x // BLOCK_THREAD_VOXEL_LEN
    ty = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.y + cuda.threadIdx.x % BLOCK_THREAD_VOXEL_LEN
    if tx < xs or tx > xe or ty < ys or ty > ye:
        return
    # -------------- CAL PROCESS
    hei = 40
    wid = 80
    temp_mat = cuda.local.array((COLUMN_SIZE, COLUMN_SIZE), numba.types.float32)
    temp_vec = cuda.local.array((COLUMN_SIZE, 1), numba.types.float32)
    temp_out = cuda.local.array((1, 1), numba.types.float32)
    for mat_ind_m in range(wid):
        for mat_ind_n in range(wid):
            dot = 0
            norm1 = 0
            norm2 = 0
            for k in range(hei):
                dot += data1[tx][k, mat_ind_m] * data2[ty][k, mat_ind_n]
                norm1 += data1[tx][k, mat_ind_m] ** 2
                norm2 += data2[ty][k, mat_ind_n] ** 2
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
        temp_vec[i, 0] = 0.0
        for j in range(wid):
            if math.isnan(temp_mat[i, j]):
                continue
            temp_vec[i, 0] += temp_mat[i, j]
            valid_line += 1
        if valid_line == 0:
            temp_vec[i, 0] = 0
        else:
            temp_vec[i, 0] /= valid_line
    temp_out[0, 0] = -1
    for i in range(wid):
        if temp_vec[i, 0] > temp_out[0, 0]:
            temp_out[0, 0] = temp_vec[i, 0]
    dest[tx, ty] = temp_out[0, 0]
    cuda.syncthreads()

# 描述子基础数据结构
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

    def __init__(self, channle, cloud_data=None, path=None):
        self.SCs = genSCs(cloud_data, path, channle, self.ring_res, self.sector_res, self.min_dis, self.max_dis,
                          self.lidar_height,
                          self.downcell_size)


# -------------------------------- Scan Context --------------------------------


# -------------------------------- Down Sample  --------------------------------
# 点云的下采样
def DownSamplePointCloud(input, output, ds=global_down_sample_size):
    # input = "/home/qh/YES/dlut/Daquan/liosave/GlobalMap.pcd"
    # output = "/home/qh/YES/dlut/Daquan/liosave/GlobalMapDS.pcd"
    point_cloud = open3d.io.read_point_cloud(input)
    point_cloud_ds = point_cloud.voxel_down_sample(voxel_size=ds)
    open3d.io.write_point_cloud(output, point_cloud_ds)


# -------------------------------- Down Sample  --------------------------------


# -------------------------------- Topo Node --------------------------------

# 按照旋转矩阵和平移矩阵，旋转平移点云
@numba.jit(nopython=True)
def TransPointCloud(point_cloud, r_, t_):
    return (np.dot(r_.astype(np.float64),
                   point_cloud.transpose()[0:3, :].astype(np.float64)) + t_.astype(np.float64)) \
        .astype(np.float32).transpose()

# 按照旋转矩阵和平移矩阵，逆旋转平移点云
@numba.jit(nopython=True)
def TransInvPointCloud(point_cloud, r_, t_):
    return (np.dot(r_.transpose().astype(np.float64),
                   point_cloud.transpose()[0:3, :].astype(np.float64)) - t_.astype(np.float64)) \
        .astype(np.float32).transpose()

# 拓扑点基本数据结构 有位姿 边界 时间戳 描述子 ID等信息
class TopoNode:
    id = -1
    position = np.zeros((3, 1), dtype=np.float64)
    boundary = (0, 0, 0, 0, 0, 0)
    rotation = np.zeros((3, 3), dtype=np.float64)
    time = -1
    SCs = []

    def __init__(self, id_, p_, r_, bound_, time_):
        self.id = id_
        self.position = p_
        self.rotation = r_
        self.boundary = bound_
        self.time = time_

# 获取点云的边界
def getBound(point_cloud):
    minx = min(point_cloud[:, 0])
    maxx = max(point_cloud[:, 0])
    miny = min(point_cloud[:, 1])
    maxy = max(point_cloud[:, 1])
    minz = min(point_cloud[:, 2])
    maxz = max(point_cloud[:, 2])
    return minx, maxx, miny, maxy, minz, maxz

# 完善拓扑点的描述子信息，拓扑点已有位置和时间戳等信息
def genTopoSC(topo_node, point_cloud, ch=3):
    topo_node.SCs = ScanContext(channle=ch, cloud_data=point_cloud, path=None).SCs
    return topo_node


# -------------------------------- Topo Node --------------------------------

# -------------------------------- Voxel Map --------------------------------
# 激光投影参数
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
down_l = -1
up_l = -1

# 激光投影参数
def SetParameter(nscan=32, hscan=2000,
                 low_ang=-16.5, up_ang=10.0,
                 ground_ind=15, min_range=5.0,
                 segment_theta_d=30.0,
                 down_lines=-1, up_lines=-1):
    global segment_alpha_x, segment_alpha_y, low_angle, up_angle, N_SCAN, Horizon_SCAN
    global ang_res_x, ang_res_y, ground_scan_ind, segment_theta
    global segment_valid_point_num, segment_valid_line_num, sensor_minimum_range, mount_angle
    global GroundSegInitFlag
    global down_l, up_l
    N_SCAN = nscan
    Horizon_SCAN = hscan
    down_l = down_lines
    up_l = up_lines
    if down_lines == -1:
        down_l = 0
    if up_lines == -1:
        up_l = N_SCAN
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

# 地面点分割
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

# 对地面点进行分割，只获取非地面店中平滑的激光点作为射线
@numba.jit(nopython=True)
def GetSelectRay(local_points, n_sec=6, surf_threshold=0.1):
    if not GroundSegInitFlag:
        print("Please Init First!")
        return
    range_mat = np.full((N_SCAN, Horizon_SCAN), fill_value=np.nan, dtype=np.float32)
    ground_mat = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.int8)
    label_mat = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.int32)
    full_cloud = np.full((N_SCAN * Horizon_SCAN, 3), fill_value=np.nan, dtype=np.float32)
    cloud_size = local_points.shape[0]
    for i in range(cloud_size):
        this_point = local_points[i, :]
        vertical_angle = np.arctan2(this_point[2], np.sqrt(this_point[0] ** 2 + this_point[1] ** 2)) * 180 / np.pi
        row_idn = int((vertical_angle - low_angle) / ang_res_y)
        if row_idn < down_l or row_idn >= up_l:
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
    per_sec_szie = 20
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

# 仅仅使用分区域的方法来完成点云射线的下采样，0.5就是两倍下采样，需要用到激光本身参数
@numba.jit(nopython=True)
def GetSelectRayFull(local_points, down_sample=0.5):
    full_cloud = np.full((N_SCAN * Horizon_SCAN, 3), fill_value=np.nan, dtype=np.float32)
    cloud_size = local_points.shape[0]
    ans_cloud = np.zeros((N_SCAN * Horizon_SCAN, 3), dtype=np.float32)
    for i in range(cloud_size):
        this_point = local_points[i, :]
        vertical_angle = np.arctan2(this_point[2], np.sqrt(this_point[0] ** 2 + this_point[1] ** 2)) * 180 / np.pi
        row_idn = int((vertical_angle - low_angle) / ang_res_y)
        if row_idn < down_l or row_idn >= up_l:
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
        index = column_idn + row_idn * Horizon_SCAN
        full_cloud[index, :] = this_point
    step = int(1 / down_sample)
    # any_nan_count = np.sum(np.any(np.isnan(full_cloud), axis=1))
    ans_count = 0
    for i_ in range(N_SCAN):
        for j_ in range(0, Horizon_SCAN, step):
            index = j_ + i_ * Horizon_SCAN
            if np.any(np.isnan(full_cloud[index, :])):
                continue
            ans_cloud[ans_count, :] = full_cloud[index, :]
            ans_count += 1
    return ans_cloud[0:ans_count, :]

# 计算若干点之间平面的法向量
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

# 点找到对应的栅格
@numba.jit(nopython=True)
def Posi2Ind(point, voxel_size):
    return int(point[0] // voxel_size), int(point[1] // voxel_size), int(point[2] // voxel_size)

# 用栅格的中心点来代替真实的激光点，完成点云的下采样
def CenterVoxel(voxle_ind, voxel_size):
    return (voxle_ind * voxel_size + (voxle_ind + 1) * voxel_size) / 2

# 光线的遍历算法，找到线段对应的触碰到的栅格
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

# 使用非占据空间来进行动态障碍剔除，需要对射线进行筛选，帅选后的射线比较稀疏，存在一些不统一的问题，泛用性差
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
            del self.origin_pcd
            del self.origin_kd_tree
            del self.origin_point_cloud
            print("Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(self.voxel_map) / 1024 / 1024))
        else:
            print("No Origin Data Found!")

    def SaveVoxelMap(self):
        if not os.path.isfile(self.voxel_map_file):
            pickle.dump(self.voxel_map, open(self.voxel_map_file, "wb"))
            print("Save Voxel Map!")
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
            del self.delete_pcd
            del self.delete_voxel_map
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
                    print("Please Set Seg Parameter First!")
                    exit(0)
                    # SetParameter(nscan=32, hscan=2000,
                    #              low_ang=-16.5, up_ang=10,
                    #              ground_ind=15, min_range=5.0,
                    #              segment_theta_d=40.0,
                    #              down_lines=-1, up_lines=-1)
                # select_points = GetSelectRay(local_points, n_sec=10, surf_threshold=0.5)
                select_points = GetSelectRayFull(local_points, down_sample=0.5)
                select_points = TransPointCloud(select_points, bin_r_, bin_p_)
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
            self.delete_pcd = open3d.io.read_point_cloud(self.delete_pcd_file)
            return
        total_rays_count = 0
        for ray_center, ray_mat in self.ray_list:
            total_rays_count += ray_mat.shape[0]
        print("Check Rays Total: {:d} Total Frame: {:d}".format(total_rays_count, len(self.ray_list)))
        handle_frame_count = 0
        total_frame = len(self.ray_list)
        start_time = ttime.time()
        self.delete_voxel_map = collections.defaultdict(int)
        tmp_full_delete_voxel_map = collections.defaultdict(int)
        for ray_center, ray_mat in self.ray_list:
            for ind in range(ray_mat.shape[0]):
                vis_voxel = walk_voxels(ray_center.reshape((3,)), ray_mat[ind, :], self.voxel_size)
                for v_ind in range(0, len(vis_voxel) - 5):
                    tmp_full_delete_voxel_map[vis_voxel[v_ind]] = 1
            handle_frame_count += 1
            if handle_frame_count % 200 == 0:
                print("Handle Ray {:.2f}% Cost {:.2f}s".format(handle_frame_count / total_frame * 100,
                                                               ttime.time() - start_time))
                start_time = ttime.time()
        start_time = ttime.time()
        handle_voxel_count = 0
        for ind, voxel in enumerate(tmp_full_delete_voxel_map):
            if voxel in self.voxel_map:
                self.voxel_map.pop(voxel)
                self.delete_voxel_map[voxel] = 1
            handle_voxel_count += 1
            if handle_voxel_count % 2000000 == 0:
                print("Handle Voxel {:.2f}% Cost {:.2f}s".format(
                    handle_voxel_count / len(tmp_full_delete_voxel_map) * 100,
                    ttime.time() - start_time))
                start_time = ttime.time()
        print("Tmp Total Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(tmp_full_delete_voxel_map) / 1024 / 1024))
        print("Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(self.voxel_map) / 1024 / 1024))
        print("Delete Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(self.delete_voxel_map) / 1024 / 1024))
        del self.ray_list
        del tmp_full_delete_voxel_map
        return

# 使用比例法，统计每个栅格击中和击穿的计数，通过计数来删除击中占比小的栅格，来删除动态障碍
class VoxelMap2:
    def __init__(self):
# -------------------------------- Voxel Map --------------------------------

# -------------------------------- Functions --------------------------------
# 获取点云bin的路径和对应的位姿，打包成一一对应输出
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
        pose_ind = BinFind(pose_vec, now_time, 0, pose_vec.shape[0] - 1)
        if pose_ind == -1:
            continue
        p_ = pose_vec[pose_ind, 1:4].reshape((3, 1))
        r_ = Rotation.from_quat(pose_vec[pose_ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        bin_info.append((now_time, pose_ind, bin_file, p_, r_))
    if save_path is not None:
        pickle.dump(bin_info, open(save_path, "wb"))
    return bin_info

# 获取某个数据包的bin和位姿，利用总体点云地图PCD来建立voxelmap并删除动态障碍
def GetVoxelMap(path=None):
    # path = "/home/qh/YES/dlut/Daquan19"
    if path is not None and os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        map1x = VoxelMap(origin_pcd_file=os.path.join(path, "liosave/GlobalMapDS1.pcd"),
                         save_path=path,
                         bin_list=bin_list)
        return map1x
    print("ERROR when get parameter!")
    return None

# 获取某个数据包的bin和位姿，利用点云bin和位姿来建立voxelmap并统计栅格击中和击穿的字数来删除动态障碍
def GetVoxelMap2(path=None):
    if path is not None and os.path.isdir(path):
        bin_list = GetBinList(bin_dir=os.path.join(path, "bin"),
                              bin_times_file=os.path.join(path, "timestamp"),
                              pose_vec_file=os.path.join(path, "liosave/sam2.txt"),
                              save_path=os.path.join(path, "bin_info.pkl"))
        map1x = VoxelMap2(save_path=path,
                          bin_list=bin_list)
        return map1x
    print("ERROR when get parameter!")
    return None


# 给定若干条轨迹列表，画出每条轨迹和位姿点
def plot_trajectory(traj_list, save_path=None):
    # traj : time x y z tw tx ty tz
    color_list = ["gray", "red", "gold", "green", "blue", "pink"]
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    for ind in range(len(traj_list)):
        traj = traj_list[ind]
        posix = []
        posiy = []
        for i in range(traj.shape[0]):
            posix.append(traj[i, 1])
            posiy.append(traj[i, 2])
        ax.scatter(posiy, posix, marker="o", s=10,
                   color=color_list[ind],
                   label="SEQ{:d}".format(ind),
                   alpha=0.6)
        ax.set_aspect(1)
        ax.legend(loc='best')
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)
    if save_path is not None:
        plt.savefig(save_path, dpi=600, transparent=True)
    plt.show()
    plt.close()

# 给定位姿文件，获取特定形式的位姿，Daquan的三个文件都是如此组织的
def GetPoseVec(pose_file=None, skip=2):
    # "/home/qh/YES/dlut/Daquan/liosave/sam2.txt"
    # time x y z tw tx ty tz
    pose_vec = np.loadtxt(pose_file, skiprows=skip)
    return pose_vec

# 使用时间戳进行二分查找 找到某个戳对应的bin将位姿对应起来
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

# 使用Bag信息，获取每一帧点云的边界，和位姿时间戳等信息，建立完成的关键帧的信息，为后面填塞描述子提供基础
def GetAccFullTopoInfo(pose_vec=None,
                       bag_file=None,
                       lidar_topic=None,
                       acc_full_topo_info_path=None):
    if acc_full_topo_info_path is not None and os.path.isfile(acc_full_topo_info_path):
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
        points = points[~np.isnan(points).any(axis=1)]
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

# 使用前面已经有了关键信息的描述子信息列表，用voxelmap地图来完成点云的累积，再填充描述子
def GetAccFullTopo(accmap=None, topo_info=None, acc_fulltopo_path=None):
    if acc_fulltopo_path is not None and os.path.isfile(acc_fulltopo_path):
        start_time = ttime.time()
        accfullTopo = pickle.load(open(acc_fulltopo_path, "rb"))
        print("Load Acc Full Topo: {:.2f}s".format(ttime.time() - start_time))
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
        topo_cloud = accmap.GetAreaPointCloud(now_topo_info.position, now_topo_info.boundary)
        topo_cloud = TransInvPointCloud(topo_cloud, now_topo_info.rotation, now_topo_info.position)
        accfullTopo.append(genTopoSC(now_topo_info, topo_cloud, ch=3))
        if count % 100 == 0:
            used_time += ttime.time() - start_time
            print("BulidTopo :{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(count / len(topo_info) * 100,
                                                                         ttime.time() - start_time,
                                                                         used_time))
            start_time = ttime.time()
        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(topo_cloud)
        # axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
        # open3d.visualization.draw_geometries([pcd, axis_pcd])
        # plot_multiple_sc(accfullTopo[-1].SCs, save_path=None)
    if acc_fulltopo_path is not None:
        pickle.dump(accfullTopo, open(acc_fulltopo_path, "wb"))
    return accfullTopo

# 对完整的拓扑节点列表计算相似度矩阵
def GetFullTopoConnectInfo(full_topo=None, full_topo_base=None, connect_path=None):
    if connect_path is not None and os.path.isfile(connect_path):
        start_time = ttime.time()
        acc_connect_info = pickle.load(open(connect_path, "rb"))
        print("load connect info :{:.2f}s".format(ttime.time() - start_time))
        return acc_connect_info
    if full_topo_base is None and full_topo is not None:
        start_time = ttime.time()
        total_line = len(full_topo)
        np_data = np.zeros((total_line, 40, 80), dtype=np.float32)
        sc_sc_sim = np.zeros((total_line, total_line), dtype=np.float32)
        for i in (range(total_line)):
            np_data[i] = full_topo[i].SCs[0]
        np_data_gpu = cuda.to_device(np_data)
        gpu_res = cuda.to_device(sc_sc_sim)
        UpperTriangleDotArea[(GRID_BLOCKX_SEIZ, GRID_BLOCKY_SIEZ), BLOCK_THREAD_SIZE] \
            (np_data_gpu, gpu_res, 0, total_line - 1, 0, total_line - 1)
        cuda.synchronize()
        gpu_ans = gpu_res.copy_to_host()
        if connect_path is not None:
            pickle.dump(gpu_ans, open(connect_path, "wb"))
        print("GPU 1060 solver connect info :{:.2f}s".format(ttime.time() - start_time))
        return gpu_ans
    if full_topo is not None and full_topo_base is not None:
        start_time = ttime.time()
        total_line1 = len(full_topo)
        total_line2 = len(full_topo_base)
        np_data1 = np.zeros((total_line1, 40, 80), dtype=np.float32)
        np_data2 = np.zeros((total_line2, 40, 80), dtype=np.float32)
        sc_sc_sim = np.zeros((total_line1, total_line2), dtype=np.float32)
        for i in range(total_line1):
            np_data1[i] = full_topo[i].SCs[0]
        for i in range(total_line2):
            np_data2[i] = full_topo_base[i].SCs[0]
        np_data1_gpu = cuda.to_device(np_data1)
        np_data2_gpu = cuda.to_device(np_data2)
        gpu_res = cuda.to_device(sc_sc_sim)
        UpperTriangleDotArea2[(GRID_BLOCKX_SEIZ, GRID_BLOCKY_SIEZ), BLOCK_THREAD_SIZE] \
            (np_data1_gpu, np_data2_gpu, gpu_res, 0, total_line1 - 1, 0, total_line2 - 1)
        cuda.synchronize()
        gpu_ans = gpu_res.copy_to_host()
        pickle.dump(gpu_ans, open(connect_path, "wb"))
        print("GPU 1060 slover cvonnect info :{:.2f}s".format(ttime.time() - start_time))
        return gpu_ans

# 补全上三角矩阵
def TopoConnectCompletion(path):
    data = pickle.load(open(path, "rb"))
    data_size = data.shape[0]
    for i in range(0, data_size):
        for j in range(0, i):
            data[i, j] = data[j, i]
    pickle.dump(data, open(path, "wb"))

# 根据相似度矩阵和描述子列表来生成指定阈值的拓扑地图
def GenTopoNodeBySim(full_topo=None, full_topo_connect=None, sim_threshold=0, path=None):
    if path is not None and os.path.isfile(path):
        start_time = ttime.time()
        topo_node_ind = pickle.load(open(path, "rb"))
        print("Load topo map: {:.2f}s".format(ttime.time() - start_time))
        return topo_node_ind
    if full_topo is None or full_topo_connect is None:
        print("Param Wrong!")
        return None
    print("Not Find Load Gen {:.2f}".format(sim_threshold))
    topo_node_ind = []
    start_time = ttime.time()
    for ind in range(len(full_topo)):
        if len(topo_node_ind) == 0:
            topo_node_ind.append(ind)
            continue
        pass_sim = full_topo_connect[ind, topo_node_ind]
        max_pass_sim = max(pass_sim)
        if max_pass_sim < sim_threshold:
            topo_node_ind.append(ind)
        if ind % 5000 == 0:
            print("Gen Topo Map : {:.2f}% Cost:{:.2f}s".format(ind / len(full_topo) * 100, ttime.time() - start_time))
    if path is not None:
        pickle.dump(topo_node_ind, open(path, "wb"))
    return topo_node_ind

# 生成appearance-based的完整关键帧节点
def GetAppFullTopo(pose_vec=None,
                   bag_file=None,
                   lidar_topic=None,
                   app_full_topo_path=None,
                   ch=5):
    if app_full_topo_path is not None and os.path.isfile(app_full_topo_path):
        start_time = ttime.time()
        appearance_fullTopoInfo = pickle.load(open(app_full_topo_path, "rb"))
        print("Load App Full Topo: {:.2f}s".format(ttime.time() - start_time))
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
        appearance_fullTopoInfo.append(genTopoSC(TopoNode(count, p_, r_, boundary, now_time), points, ch=ch))
        count += 1
        if count % 100 == 0:
            used_time += ttime.time() - start_time
            print("GetTopo :{:.2f}% Cost:{:.2f}s Total:{:.2f}s".format(count / bag_count_info * 100,
                                                                       ttime.time() - start_time,
                                                                       used_time))
            start_time = ttime.time()
    pickle.dump(appearance_fullTopoInfo, open(app_full_topo_path, "wb"))
    return appearance_fullTopoInfo


# 计算两个拓扑节点之间的距离
def TopoNodeDistance(t1, t2):
    return np.linalg.norm(t1.position - t2.position)

# 生成density-based的拓扑地图
def GenTopoNodeByDensity(app_full_topo, density):
    topo_node_ind = []
    tree_data = np.zeros((0, 3), dtype=np.float32)
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
# -------------------------------- Functions --------------------------------

# -------------------------------- Show Topo Map ------------------------------
# 画出某一条拓扑地图
def ShowTopoMap(full_topo, topo_nodes_ind, path=None, vis=True):
    points = np.zeros((3, 0), dtype=np.float32)
    for ind in range(len(topo_nodes_ind)):
        node = full_topo[topo_nodes_ind[ind]]
        points = np.column_stack((points, node.position))
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    ax.scatter(points[1], points[0], marker="o", s=10, color="black", label='Vertex')
    ax.set_aspect(1)
    ax.legend(loc='upper left')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    if path is not None:
        plt.savefig(path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()


# -------------------------------- Show Topo Map ------------------------------

# -------------------------------- Precision And Recall ------------------------------
# 给定检验参数-真值距离和topK，遍历 full_topo 中的拓扑节点进行检验 需要full_base_topo的原因是base_topo只记录了节点ID（省内存）
def GetPrecisionAndRecall(full_base_topo, base_topo, full_topo, connect, sim, top=10, gdis=3.0):
    tp, fp, tn, fn = 0, 0, 0, 0
    base_topo = np.array(base_topo)
    topo_connect = connect[:, base_topo]
    his_points = []
    for hind in base_topo:
        hnode = full_base_topo[hind]
        his_points.append(hnode.position)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(his_points)
    tree = open3d.geometry.KDTreeFlann(pcd)
    for now_node_id in range(len(full_topo)):
        now_node = full_topo[now_node_id]
        sim_ok = np.where(topo_connect[now_node_id] > sim)[0]
        sim_ok_val = topo_connect[now_node_id, sim_ok]
        sort_sim_ok = (-sim_ok_val).argsort()
        max_sim = sort_sim_ok[0:top]
        global_sim_id = base_topo[sim_ok[max_sim]].reshape((1, -1))
        sim_size = global_sim_id.shape[1]
        k, ind, d = tree.search_knn_vector_3d(now_node.position, 1)
        min_dis = math.sqrt(d[0])
        if sim_size == 0 and min_dis < gdis:
            fn += 1
        elif sim_size == 0 and min_dis > gdis:
            tn += 1
        else:
            tpflag = False
            for i in global_sim_id[0]:
                hnode = full_base_topo[i]
                dis = TopoNodeDistance(now_node, hnode)
                if dis < gdis:
                    tp += 1
                    tpflag = True
                    break
            if not tpflag:
                fp += 1
    if tp == 0:
        return sim, 0, 0, len(base_topo), top, gdis
    return sim, tp / (tp + fp), tp / (tp + fn), len(base_topo), top, gdis

# 绘制PR列表中的数据为曲线
def plot_pr(pr_list, path=None, vis=True):
    recall = []
    precision = []
    sim = []
    for pr in pr_list:
        sim.append(pr[0])
        precision.append(pr[1])
        recall.append(pr[2])
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(8, 4.5))
    ax.plot(recall, precision, lw="5", color="black")
    plt.xlim(0, 1.1)
    plt.ylim(0, 1.1)
    ax.set_aspect('equal', adjustable='box')
    if path is not None:
        plt.savefig(path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()

# 绘制多条PR曲线
def plot_muliti_pr(acc_pr, app_pr, save_path=None, row_size_=2, title=None, vis=True):
    parameter_size = len(acc_pr)
    row_size = row_size_
    col_size = int(math.ceil(parameter_size / row_size))
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "ours-method VS appearance-based-method"
    if title is not None:
        title_label = title
    fig.suptitle(title_label, fontsize=25)
    for i, key in enumerate(acc_pr):
        # # ACC
        row = i // col_size + 1
        col = i - (row - 1) * col_size + 1
        ax = plt.subplot(row_size, col_size, i + 1)
        ax.set_aspect('equal', adjustable='box')
        if col == 1:
            plt.ylabel("precision", fontsize=16)
        plt.xlim(0, 1.1)
        plt.ylim(0, 1.1)
        recall = []
        precision = []
        sim = []
        recall.append(1.0)
        precision.append(0.0)
        sim.append(0.0)
        acc_vertex_size = 0
        for pr in acc_pr[key]:
            if pr[1] == 0 or pr[2] == 0:
                continue
            sim.append(pr[0])
            precision.append(pr[1])
            recall.append(pr[2])
            acc_vertex_size = pr[3]
        recall.append(0.0)
        precision.append(1.0)
        sim.append(1.0)
        plt.plot(recall, precision, lw="2", color="lime", label="ours", alpha=0.9)
        plt.scatter(recall, precision, marker="o", s=10, color="black")
        # for a, b, c in zip(recall, precision, sim):
        #     plt.text(a, b, c, ha='center', va='bottom', fontsize=10)
        # # APP
        recall = []
        precision = []
        sim = []
        recall.append(1.0)
        precision.append(0.0)
        sim.append(0.0)
        app_vertex_size = 0
        for pr in app_pr[key]:
            if pr[1] == 0 or pr[2] == 0:
                continue
            sim.append(pr[0])
            precision.append(pr[1])
            recall.append(pr[2])
            app_vertex_size = pr[3]
        recall.append(0.0)
        precision.append(1.0)
        sim.append(1.0)
        plt.plot(recall, precision, lw="2", color="silver", label="appearance-based", alpha=0.9)
        plt.scatter(recall, precision, marker="o", s=10, color="gray", alpha=0.9)
        # for a, b, c in zip(recall, precision, sim):
        #     plt.text(a, b, c, ha='center', va='bottom', fontsize=10)
        ax.legend(loc="best")
        plt.xlabel("recall", fontsize=16)
        detail = "\nours vertex size:{:d}\nappearance-based vertex:{:d}\ncount reduce {:.1f}%". \
            format(acc_vertex_size, app_vertex_size, (app_vertex_size - acc_vertex_size) / app_vertex_size * 100.0)
        plt.text(0.5, 0.4, detail, ha="center", fontsize=12)
        plt.title("Threshold: {:.2f}".format(key), fontsize=15)

    for i in range(row_size * col_size):
        if i < parameter_size:
            continue
        else:
            ax = plt.subplot(row_size, col_size, i + 1)
            ax.axis('off')
    if save_path is not None:
        plt.savefig(save_path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()

# -------------------------------- Precision And Recall ------------------------------

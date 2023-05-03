import math
import sys
import numpy as np
from numba import cuda
import numba
import open3d
import pickle
import os
import time as ttime
import matplotlib.pyplot as plt
import collections
from scipy.integrate import trapz
from matplotlib.patches import FancyArrowPatch

global_down_sample_size = 0.2

np.set_printoptions(suppress=True, precision=5, threshold=sys.maxsize, linewidth=sys.maxsize)


# -------------------------------- Scan Context --------------------------------
# 描述子生成支撑函数，计算点的角度
@numba.jit(nopython=True)
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
@numba.jit(nopython=True)
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
    if idx_sector >= num_sector:
        idx_sector = num_sector - 1
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
    # downpcd = open3d.geometry.voxel_down_sample(pcd, voxel_size=downcell_size)
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

    def __init__(self, channle, cloud_data=None, path=None, lidar_height=0):
        self.lidar_height = 2.0 if lidar_height == 0 else lidar_height
        self.SCs = genSCs(cloud_data, path, channle, self.ring_res, self.sector_res, self.min_dis, self.max_dis,
                          self.lidar_height,
                          self.downcell_size)


# -------------------------------- Scan Context --------------------------------


# -------------------------------- Down Sample  --------------------------------
# 点云的下采样
def DownSamplePointCloud(input, output, ds=global_down_sample_size):
    point_cloud = open3d.io.read_point_cloud(input)
    point_cloud_ds = open3d.geometry.voxel_down_sample(point_cloud, voxel_size=ds)
    # point_cloud_ds = point_cloud.voxel_down_sample(voxel_size=ds)
    open3d.io.write_point_cloud(output, point_cloud_ds)


# -------------------------------- Down Sample  --------------------------------


# -------------------------------- Topo Node --------------------------------

# 按照旋转矩阵和平移矩阵，旋转平移点云
@numba.jit(nopython=True)
def TransPointCloud(points, r_, t_):
    T_ = np.eye(4, dtype=np.float64)
    T_[0:3, 0:3] = r_
    T_[0:3, 3:4] = t_
    return np.dot(T_.astype(np.float64),
                  np.vstack((points.transpose().astype(np.float64),
                             np.ones((1, points.shape[0]),
                                     dtype=np.float64))))[0:3, :].transpose()


# 按照旋转矩阵和平移矩阵，逆旋转平移点云
@numba.jit(nopython=True)
def TransInvPointCloud(points, r_, t_):
    T_ = np.eye(4)
    T_[0:3, 0:3] = r_
    T_[0:3, 3:4] = t_
    T_ = np.linalg.inv(T_)
    return np.dot(T_.astype(np.float64),
                  np.vstack((points.transpose().astype(np.float64),
                             np.ones((1, points.shape[0]),
                                     dtype=np.float64))))[0:3, :].transpose()


@numba.jit(nopython=True)
def rmmm(points, hl):
    mask = points[:, 2] > hl
    points = points[~mask]
    return points


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
def genTopoSC(topo_node, point_cloud, ch=3, lh=0):
    topo_node.SCs = ScanContext(channle=ch, cloud_data=point_cloud, path=None, lidar_height=lh).SCs
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
    if not GroundSegInitFlag:
        print("Please Init First!")
        return
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
            handle_size = self.origin_point_cloud.shape[0]
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i in range(self.origin_point_cloud.shape[0]):
                p = self.origin_point_cloud[i, :]
                self.voxel_map[Posi2Ind(p, self.voxel_size)] = 1
                if i % report_size == 0:
                    print("Bulid Voxel Map {:.2f}% Cost {:.2f}s".format(i / self.origin_point_cloud.shape[0] * 100,
                                                                        ttime.time() - start_time))
                    start_time = ttime.time()
            del self.origin_pcd
            del self.origin_kd_tree
            del self.origin_point_cloud
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
            handle_size = bad_voxel_map_size
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i, key in enumerate(self.delete_voxel_map):
                delete_points[i, :] = CenterVoxel(np.array(key), self.voxel_size)
                if i % report_size == 0:
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
            handle_size = voxel_map_size
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i, key in enumerate(self.voxel_map):
                self.point_cloud[i, :] = CenterVoxel(np.array(key), self.voxel_size)
                if i % report_size == 0:
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
            handle_size = bin_size
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
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
                if bin_ind % report_size == 0:
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
        self.delete_voxel_map = collections.defaultdict(int)
        tmp_full_delete_voxel_map = collections.defaultdict(int)
        handle_size = total_frame
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        for ray_center, ray_mat in self.ray_list:
            for ind in range(ray_mat.shape[0]):
                vis_voxel = walk_voxels(ray_center.reshape((3,)), ray_mat[ind, :], self.voxel_size)
                for v_ind in range(0, len(vis_voxel) - 5):
                    tmp_full_delete_voxel_map[vis_voxel[v_ind]] = 1
            handle_frame_count += 1
            if handle_frame_count % report_size == 0:
                print("Handle Ray {:.2f}% Cost {:.2f}s".format(handle_frame_count / total_frame * 100,
                                                               ttime.time() - start_time))
                start_time = ttime.time()
        start_time = ttime.time()
        handle_voxel_count = 0
        handle_size = len(tmp_full_delete_voxel_map)
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        for ind, voxel in enumerate(tmp_full_delete_voxel_map):
            if voxel in self.voxel_map:
                self.voxel_map.pop(voxel)
                self.delete_voxel_map[voxel] = 1
            handle_voxel_count += 1
            if handle_voxel_count % report_size == 0:
                print("Handle Voxel {:.2f}% Cost {:.2f}s".format(
                    handle_voxel_count / handle_size * 100,
                    ttime.time() - start_time))
                start_time = ttime.time()
        print("Tmp Total Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(tmp_full_delete_voxel_map) / 1024 / 1024))
        print("Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(self.voxel_map) / 1024 / 1024))
        print("Delete Voxel Map Cost Mem {:.2f} MB".format(sys.getsizeof(self.delete_voxel_map) / 1024 / 1024))
        del self.ray_list
        del tmp_full_delete_voxel_map
        return


# 点云投影后保留要求范围中的点
@numba.jit(nopython=True)
def GetInRange(points, min_dis, max_dis, d_l, u_l):
    if not GroundSegInitFlag:
        print("Please Init First!")
        return
    points = points[:, 0:3]
    point_size = points.shape[0]
    ans = np.zeros((point_size, 3), dtype=np.float32)
    ind = 0
    for i in range(point_size):
        this_point = points[i, :]
        vertical_angle = np.arctan2(this_point[2], np.sqrt(this_point[0] ** 2 + this_point[1] ** 2)) * 180 / np.pi
        row_idn = int((vertical_angle - low_angle) / ang_res_y)
        if row_idn < d_l or row_idn >= u_l:
            continue
        horizon_angle = np.arctan2(this_point[0], this_point[1]) * 180 / np.pi
        column_idn = int(-round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN / 2)
        if column_idn >= Horizon_SCAN:
            column_idn -= Horizon_SCAN
        if column_idn < 0 or column_idn >= Horizon_SCAN:
            continue
        range_ = np.linalg.norm(this_point)
        if range_ < min_dis or range_ > max_dis:
            continue
        ans[ind, :] = points[i, :]
        ind += 1
    return ans[0:ind, :]


# 点云投影后保留要求范围中的点
@numba.jit(nopython=True)
def GetInRange(points, min_dis, max_dis):
    points = points[:, 0:3]
    point_size = points.shape[0]
    ans = np.zeros((point_size, 3), dtype=np.float32)
    ind = 0
    for i in range(point_size):
        this_point = points[i, :]
        range_ = np.linalg.norm(this_point)
        if range_ < min_dis or range_ > max_dis:
            continue
        ans[ind, :] = points[i, :]
        ind += 1

    return points[::5, :]



# 使用比例法，统计每个栅格击中和击穿的计数，通过计数来删除击中占比小的栅格，来删除动态障碍
class VoxelMap2:
    def __init__(self, save_path, bin_list, hit_thres=0.05, voxel_size=0.1):
        self.pcd = None
        self.kd_tree = None
        self.voxel_map = None
        self.delete_voxel_map = None
        self.point_cloud = None
        self.bin_list = bin_list
        self.voxel_size = voxel_size
        self.hit_thres = hit_thres
        self.raw_voxem_map_file = os.path.join(save_path, "VM2_raw.pkl")
        self.voxel_map_file = os.path.join(save_path, "VM2_vm.pkl")
        self.delete_voxel_map_file = os.path.join(save_path, "VM2_badvm.pkl")
        self.pcd_file = os.path.join(save_path, "VM2_pcd.pcd")
        self.bad_pcd_file = os.path.join(save_path, "VM2_badpcd.pcd")
        if os.path.isfile(self.pcd_file):
            print("Load Map Data, Delete It to Regenerate!")
            print(self.pcd_file)
            self.pcd = open3d.io.read_point_cloud(self.pcd_file)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
        elif os.path.isfile(self.raw_voxem_map_file):
            print("Load Raw Data, Delete It to Regenerate!")
            print(self.raw_voxem_map_file)
            self.voxel_map = pickle.load(open(self.raw_voxem_map_file, "rb"))
            self.DeterminVoxelMap()
            self.SaveData()
        else:
            print("Cal Hit And Cross To Remove Dynamic Points")
            if not GroundSegInitFlag:
                print("Please Set Seg Parameter First!")
                exit(0)
            self.GetHitVoxelMap()
            self.GetCrossVoxelMap()
            self.DeterminVoxelMap()
            self.SaveData()

    def GetHitVoxelMap(self):
        self.voxel_map = collections.defaultdict(list)
        handle_size = len(self.bin_list)
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        proc_count = 0
        for _, _, bin_file, bin_p, bin_r in self.bin_list:
            l_points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
            l_points = GetInRange(l_points, min_dis=5.0, max_dis=80.0)
            g_points = TransPointCloud(l_points, bin_r, bin_p)
            for i in range(g_points.shape[0]):
                point = g_points[i, :]
                voxel = Posi2Ind(point, voxel_size=self.voxel_size)
                if voxel in self.voxel_map:
                    self.voxel_map[voxel][0] += 1
                else:
                    self.voxel_map[voxel] = [1, 0]
            proc_count += 1
            if proc_count % report_size == 0:
                print(
                    "Hit Voxel {:.2f}% Cost {:.2f}s".format(proc_count / handle_size * 100, ttime.time() - start_time))
                start_time = ttime.time()

    def GetCrossVoxelMap(self):
        handle_size = len(self.bin_list)
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        proc_count = 0
        for _, _, bin_file, bin_p, bin_r in self.bin_list:
            l_points = np.fromfile(bin_file, dtype=np.float32).reshape((-1, 4))[:, 0:3]
            l_points = GetInRange(l_points, min_dis=5.0, max_dis=80.0)
            g_points = TransPointCloud(l_points, bin_r, bin_p)
            for i in range(g_points.shape[0]):
                point = g_points[i, :]
                cross_voxels = walk_voxels(bin_p.reshape((3,)), point, voxel_size=self.voxel_size)
                for v_ind in range(0, len(cross_voxels) - 5):
                    voxel = cross_voxels[v_ind]
                    if voxel in self.voxel_map:
                        self.voxel_map[voxel][1] += 1
            proc_count += 1
            if proc_count % report_size == 0:
                print("Cross Voxel {:.2f}% Cost {:.2f}s".format(proc_count / handle_size * 100,
                                                                ttime.time() - start_time))
                start_time = ttime.time()

    def DeterminVoxelMap(self):
        pickle.dump(self.voxel_map, open(self.raw_voxem_map_file, "wb"))
        self.delete_voxel_map = collections.defaultdict(list)
        handle_size = len(self.voxel_map)
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        for i, voxel in enumerate(self.voxel_map):
            hit, cross = self.voxel_map[voxel][0], self.voxel_map[voxel][1]
            hit_percent = hit / (hit + cross)
            if hit != 0 and hit_percent < self.hit_thres:
                self.delete_voxel_map[voxel] = [hit, cross]
            if i % report_size == 0:
                print("DeterMin Voxel {:.2f}% Cost {:.2f}s".format(i / handle_size * 100, ttime.time() - start_time))
                start_time = ttime.time()
        handle_size = len(self.delete_voxel_map)
        report_size = handle_size // 50 if handle_size // 50 != 0 else 1
        start_time = ttime.time()
        for i, voxel in enumerate(self.delete_voxel_map):
            if voxel in self.voxel_map:
                self.voxel_map.pop(voxel)
            if i % report_size == 0:
                print("Handle Bad Voxel {:.2f}% Cost {:.2f}s".format(i / handle_size * 100, ttime.time() - start_time))
                start_time = ttime.time()

    def SaveData(self):
        if self.voxel_map is not None and len(self.voxel_map) != 0:
            pickle.dump(self.voxel_map, open(self.voxel_map_file, "wb"))
            final_points = np.zeros((len(self.voxel_map), 3), dtype=np.float32)
            handle_size = len(self.voxel_map)
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i, voxel in enumerate(self.voxel_map):
                final_points[i, :] = CenterVoxel(np.array(voxel), voxel_size=self.voxel_size)
                if i % report_size == 0:
                    print("Build Map {:.2f}% Cost {:.2f}s".format(i / handle_size * 100, ttime.time() - start_time))
                    start_time = ttime.time()
            del self.voxel_map
            self.pcd = open3d.geometry.PointCloud()
            self.pcd.points = open3d.utility.Vector3dVector(final_points)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
            open3d.io.write_point_cloud(self.pcd_file, self.pcd)

        if self.delete_voxel_map is not None and len(self.delete_voxel_map) != 0:
            pickle.dump(self.delete_voxel_map, open(self.delete_voxel_map_file, "wb"))
            delete_points = np.zeros((len(self.delete_voxel_map), 3), dtype=np.float32)
            for i, voxel in enumerate(self.delete_voxel_map):
                delete_points[i, :] = CenterVoxel(np.array(voxel), voxel_size=self.voxel_size)
            del self.delete_voxel_map
            delete_pcd = open3d.geometry.PointCloud()
            delete_pcd.points = open3d.utility.Vector3dVector(delete_points)
            open3d.io.write_point_cloud(self.bad_pcd_file, delete_pcd)

    def GetAreaPointCloud(self, center, boundary):
        serach_radius = max(boundary)
        _, p_ind, _ = self.kd_tree.search_radius_vector_3d(np.array(center), serach_radius)
        tmp_point = self.point_cloud[p_ind, :]
        return tmp_point


# 不删除动态障碍物的地图
class VoxelMap3_Load:
    def __init__(self, save_path, voxel_size=0.1):
        self.pcd = None
        self.kd_tree = None
        self.point_cloud = None
        self.voxel_size = voxel_size
        self.voxel_map = None
        self.raw_voxem_map_file = os.path.join(save_path, "VM2_raw.pkl")
        self.raw_pcd_file = os.path.join(save_path, "VM3_raw.pcd")
        if os.path.isfile(self.raw_pcd_file):
            print("Load Raw Pcd File, Delete It to Regenerate!")
            self.pcd = open3d.io.read_point_cloud(self.raw_pcd_file)
            self.pcd = self.pcd.voxel_down_sample(voxel_size=1.0)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
        elif os.path.isfile(self.raw_voxem_map_file):
            print("Load Raw Data, Delete It to Regenerate!")
            print(self.raw_voxem_map_file)
            self.voxel_map = pickle.load(open(self.raw_voxem_map_file, "rb"))
            raw_points = np.zeros((len(self.voxel_map), 3), dtype=np.float32)
            handle_size = len(self.voxel_map)
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i, voxel in enumerate(self.voxel_map):
                raw_points[i, :] = CenterVoxel(np.array(voxel), voxel_size=self.voxel_size)
                if i % report_size == 0:
                    print("Handle Raw Voxel {:.2f}% Cost {:.2f}s".format(i / handle_size * 100,
                                                                         ttime.time() - start_time))
                    start_time = ttime.time()
            del self.voxel_map
            self.pcd = open3d.geometry.PointCloud()
            self.pcd.points = open3d.utility.Vector3dVector(raw_points)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
            open3d.io.write_point_cloud(self.raw_pcd_file, self.pcd)
        else:
            print("No Raw Voxel Data!")
            exit(0)

    def GetAreaPointCloud(self, center, boundary):
        serach_radius = max(boundary)
        _, p_ind, _ = self.kd_tree.search_radius_vector_3d(np.array(center), serach_radius)
        tmp_point = self.point_cloud[p_ind, :]
        return tmp_point


# 不删除动态障碍物的地图
class VoxelMap4_Load:
    def __init__(self, save_path, voxel_size=0.1):
        self.pcd = None
        self.kd_tree = None
        self.point_cloud = None
        self.voxel_size = voxel_size
        self.voxel_map = None
        self.raw_voxem_map_file = os.path.join(save_path, "VM2_vm.pkl")
        self.raw_pcd_file = os.path.join(save_path, "VM2_pcd.pcd")
        if os.path.isfile(self.raw_pcd_file):
            print("Load Raw Pcd 4, Delete It to Regenerate!")
            self.pcd = open3d.io.read_point_cloud(self.raw_pcd_file)
            self.pcd = self.pcd.voxel_down_sample(voxel_size=1.0)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
        elif os.path.isfile(self.raw_voxem_map_file):
            print("Load Raw Data, Delete It to Regenerate!")
            print(self.raw_voxem_map_file)
            self.voxel_map = pickle.load(open(self.raw_voxem_map_file, "rb"))
            raw_points = np.zeros((len(self.voxel_map), 3), dtype=np.float32)
            handle_size = len(self.voxel_map)
            report_size = handle_size // 50 if handle_size // 50 != 0 else 1
            start_time = ttime.time()
            for i, voxel in enumerate(self.voxel_map):
                raw_points[i, :] = CenterVoxel(np.array(voxel), voxel_size=self.voxel_size)
                if i % report_size == 0:
                    print("Handle Raw Voxel {:.2f}% Cost {:.2f}s".format(i / handle_size * 100,
                                                                         ttime.time() - start_time))
                    start_time = ttime.time()
            del self.voxel_map
            self.pcd = open3d.geometry.PointCloud()
            self.pcd.points = open3d.utility.Vector3dVector(raw_points)
            self.kd_tree = open3d.geometry.KDTreeFlann(self.pcd)
            self.point_cloud = np.array(self.pcd.points)
            open3d.io.write_point_cloud(self.raw_pcd_file, self.pcd)
        else:
            print("No Raw Voxel Data!")
            exit(0)

    def GetAreaPointCloud(self, center, boundary):
        serach_radius = max(boundary)
        _, p_ind, _ = self.kd_tree.search_radius_vector_3d(np.array(center), serach_radius)
        tmp_point = self.point_cloud[p_ind, :]
        return tmp_point


# -------------------------------- Voxel Map --------------------------------


# -------------------------------- Functions --------------------------------


# 对完整的拓扑节点列表计算相似度矩阵
def GetSimMatrixTo19(full_topo_19, connect_path, full_topo_another=None):
    if os.path.isfile(connect_path):
        sim_matrix = pickle.load(open(connect_path, "rb"))
        return sim_matrix
    cuda.select_device(0)
    if full_topo_another is None:
        start_time = ttime.time()
        total_line = len(full_topo_19)
        np_data = np.zeros((total_line, 40, 80), dtype=np.float32)
        sc_sc_sim = np.zeros((total_line, total_line), dtype=np.float32)
        for i in (range(total_line)):
            np_data[i] = full_topo_19[i].SCs[0]
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
    if full_topo_another is not None:
        start_time = ttime.time()
        total_line1 = len(full_topo_another)
        total_line2 = len(full_topo_19)
        np_data1 = np.zeros((total_line1, 40, 80), dtype=np.float32)
        np_data2 = np.zeros((total_line2, 40, 80), dtype=np.float32)
        sc_sc_sim = np.zeros((total_line1, total_line2), dtype=np.float32)
        for i in range(total_line1):
            np_data1[i] = full_topo_another[i].SCs[0]
        for i in range(total_line2):
            np_data2[i] = full_topo_19[i].SCs[0]
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


@numba.jit(nopython=True)
def CompleteUpTriangle(data):
    data_size = data.shape[0]
    for i in range(0, data_size):
        for j in range(0, i):
            data[j, i] = data[i, j]


@numba.jit(nopython=True)
def CompleteDownTriangle(data):
    data_size = data.shape[0]
    for i in range(0, data_size):
        for j in range(0, i):
            data[i, j] = data[j, i]


# 补全上三角矩阵
def TopoConnectCompletion(path):
    data = pickle.load(open(path, "rb"))
    if data[0, 1] == 0:
        print("Complete Up Triangle!")
        CompleteUpTriangle(data)
    elif data[1, 0] == 0:
        print("Complete Down Triangle!")
        CompleteDownTriangle(data)
    else:
        print("Already Complete!")
    pickle.dump(data, open(path, "wb"))


# 根据相似度矩阵和描述子列表来生成指定阈值的拓扑地图
def GenTopoNodeBySim(full_topo=None, sim_mat=None, sim_threshold=0, path=None):
    if path is not None and os.path.isfile(path):
        start_time = ttime.time()
        topo_node_ind = pickle.load(open(path, "rb"))
        print("Load topo map: {:.2f}s {:d}".format(ttime.time() - start_time, len(topo_node_ind)))
        return topo_node_ind
    if full_topo is None or sim_mat is None:
        print("Param Wrong!")
        return None
    print("Not Find Load Gen {:.2f}".format(sim_threshold))
    topo_node_ind = []
    handle_size = len(full_topo)
    report_size = handle_size // 50 if handle_size // 50 != 0 else 1
    start_time = ttime.time()
    for ind in range(len(full_topo)):
        if len(topo_node_ind) == 0:
            topo_node_ind.append(ind)
            continue
        pass_sim = sim_mat[ind, topo_node_ind]
        max_pass_sim = max(pass_sim)
        if max_pass_sim < sim_threshold:
            topo_node_ind.append(ind)
        if ind % report_size == 0:
            print("Gen Topo Map : {:.2f}% Cost:{:.2f}s".format(ind / handle_size * 100, ttime.time() - start_time))
    if path is not None:
        pickle.dump(topo_node_ind, open(path, "wb"))
    return topo_node_ind


# 计算两个拓扑节点之间的距离
def TopoNodeDistance(t1, t2):
    return np.linalg.norm(t1.position - t2.position)


# -------------------------------- Functions --------------------------------

# -------------------------------- Precision And Recall ------------------------------
# 给定检验参数-真值距离和topK，遍历 full_topo 中的拓扑节点进行检验 需要full_base_topo的原因是base_topo只记录了节点ID（省内存）
def GetPrecisionAndRecall(full_base_topo, base_topo, full_topo, sim_mat, sim, top=10, gdis=3.0, info="", topo_num=-1):
    tp, fp, tn, fn = 0, 0, 0, 0
    base_topo = np.array(base_topo)
    topo_connect = sim_mat[:, base_topo]
    his_points = []
    for hind in base_topo:
        hnode = full_base_topo[hind]
        his_points.append(hnode.position)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(his_points)
    tree = open3d.geometry.KDTreeFlann(pcd)
    for now_node_ind in range(len(full_topo)):
        # 先找先验地图中
        # 先找先验地图中相似度满足阈值的，再找其中最相近的前N个，
        # 没有满足阈值的：这个点范围内没有先验地图点为TN，有先验地图点为FN
        # 相似度最大的N个：如果任一在范围内TP，都不在范围内FP
        now_node = full_topo[now_node_ind]
        sim_ok = np.where(topo_connect[now_node_ind] > sim)[0]
        sim_ok_val = topo_connect[now_node_ind, sim_ok]
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
        precision = 0
        recall = 0
    else:
        precision = tp / (tp + fp)
        recall = tp / (tp + fn)
    return {
        "info": info,
        "recall_sim": sim,
        "precision": precision,
        "recall": recall,
        "top": top,
        "turth dis": gdis,
        "topo num": topo_num
    }


# -------------------------------- Precision And Recall ------------------------------


# -------------------------------- Show Fun --------------------------------
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


# 画出某一条拓扑地图
def ShowTopoMap(full_topo, topo_nodes_ind, path=None, vis=True):
    if path is not None and os.path.isfile(path):
        return None
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    points = np.zeros((3, 0), dtype=np.float32)
    id_list = []
    for ind in range(len(topo_nodes_ind)):
        node = full_topo[topo_nodes_ind[ind]]
        points = np.column_stack((points, node.position))
        id_list.append(ind)
    ax.scatter(points[1], points[0], marker="o", s=3, color="black", label='Vertex')
    plt.plot(points[1], points[0], linewidth=1, markersize=1, color='gray', alpha=0.5, zorder=0, label="edge")
    ax.set_aspect(1)
    ax.legend(loc='upper left')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    if path is not None:
        plt.savefig(path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()


def ShowTopoMapByDensity(full_topo, topo_nodes_ind, path=None, vis=True):
    points = np.zeros((3, 0), dtype=np.float32)
    for ind in range(len(topo_nodes_ind)):
        node = full_topo[topo_nodes_ind[ind]]
        points = np.column_stack((points, node.position))
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points.transpose())
    # open3d.visualization.draw_geometries([pcd])
    kdtree = open3d.geometry.KDTreeFlann(pcd)
    density = []
    for ind in range(len(topo_nodes_ind)):
        node = full_topo[topo_nodes_ind[ind]]
        now_p = node.position
        aa = kdtree.search_radius_vector_3d(now_p, 10.0)
        density.append(aa[0] / 10.0)
    cmap = "jet"
    sc = ax.scatter(points[1], points[0], c=density, cmap=cmap)
    plt.plot(points[1], points[0], linewidth=1, markersize=1, color='gray', alpha=0.5, zorder=-1)
    cbar = plt.colorbar(sc)
    cbar.set_label('Density')
    scale_length = 100  # 比例尺长度为 10 米
    scale_pos_x = -1200  # 比例尺位置的 x 坐标
    scale_pos_y = -200  # 比例尺位置的 y 坐标

    # 绘制比例尺线段
    arrow_style = FancyArrowPatch(
        (scale_pos_x, scale_pos_y),  # 起点
        (scale_pos_x + scale_length, scale_pos_y),  # 终点
        arrowstyle='|-|',  # 箭头样式，表示为竖线和横线
        mutation_scale=4,  # 箭头大小
        linewidth=2,  # 线宽
        color='black',  # 线条颜色
        linestyle='solid'  # 线型
    )
    plt.gca().add_patch(arrow_style)
    plt.axis('equal')
    if path is not None:
        plt.savefig(path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()


# 绘制PR列表中的数据为曲线
def plot_pr(pr_list, path=None, vis=True):
    recall = []
    precision = []
    sim = []
    for pr in pr_list:
        sim.append(pr['recall_sim'])
        precision.append(pr['precision'])
        recall.append(pr['recall'])
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


def plot_acc_app_pr(acc_pr, app_pr, path=None, vis=True):
    acc_recall, app_recall = [], []
    acc_precision, app_precision = [], []
    acc_sim, app_sim = [], []
    for pr in acc_pr:
        acc_sim.append(pr['recall_sim'])
        acc_precision.append(pr['precision'])
        acc_recall.append(pr['recall'])
    for pr in app_pr:
        app_sim.append(pr['recall_sim'])
        app_precision.append(pr['precision'])
        app_recall.append(pr['recall'])
    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(8, 4.5))
    ax.plot(acc_recall, acc_precision, lw="5", color="green", label="acc")
    ax.plot(app_recall, app_precision, lw="5", color="blue", label="app")
    ax.legend(loc="best")
    plt.xlim(0, 1.1)
    plt.ylim(0, 1.1)
    ax.set_aspect('equal', adjustable='box')
    if path is not None:
        plt.savefig(path, dpi=600, transparent=True)
    if vis:
        plt.show()
    plt.close()


# 绘制多条PR曲线
def plot_muliti_pr_acc(acc_pr_dic, save_path=None, row_size_=2, title=None, vis=True):
    parameter_size = len(acc_pr_dic)
    row_size = row_size_
    col_size = int(math.ceil(parameter_size / row_size))
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "ours-method VS appearance-based-method"
    if title is not None:
        title_label = title
    fig.suptitle(title_label, fontsize=25)
    for i, key in enumerate(acc_pr_dic):
        # # ACC
        if True:
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
            topo_num = 0
            for pr in acc_pr_dic[key]:
                if pr['precision'] == 0 or pr['recall'] == 0:
                    continue
                sim.append(pr['recall_sim'])
                precision.append(pr['precision'])
                recall.append(pr['recall'])
                topo_num = pr['topo num']
            recall.append(0.0)
            precision.append(1.0)
            sim.append(1.0)
            auc = trapz(precision, recall, dx=0.00001)
            plt.plot(recall, precision, lw="2", color="lime", label="ours {:d} {:.5f}".format(topo_num, auc), alpha=0.9)
            plt.scatter(recall, precision, marker="o", s=10, color="black")
            ax.legend(loc="best")
            plt.xlabel("recall", fontsize=16)
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


def plot_muliti_pr_app(app_pr_dic, save_path=None, row_size_=2, title=None, vis=True):
    parameter_size = len(app_pr_dic)
    row_size = row_size_
    col_size = int(math.ceil(parameter_size / row_size))
    fig, ax = plt.subplots(row_size, col_size, figsize=(24, 13.5))
    title_label = "ours-method VS appearance-based-method"
    if title is not None:
        title_label = title
    fig.suptitle(title_label, fontsize=25)
    for i, key in enumerate(app_pr_dic):
        # # APP
        if True:
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
            topo_num = 0
            for pr in app_pr_dic[key]:
                if pr['precision'] == 0 or pr['recall'] == 0:
                    continue
                sim.append(pr['recall_sim'])
                precision.append(pr['precision'])
                recall.append(pr['recall'])
                topo_num = pr['topo num']
            recall.append(0.0)
            precision.append(1.0)
            sim.append(1.0)
            auc = trapz(precision, recall, dx=0.00001)
            plt.plot(recall, precision, lw="2", color="silver", label="a-b {:d} {:.5f}".format(topo_num, auc),
                     alpha=0.9)
            plt.scatter(recall, precision, marker="o", s=10, color="gray", alpha=0.9)
            ax.legend(loc="best")
            plt.xlabel("recall", fontsize=16)
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


def GetPrData(pr_list):
    recall = []
    precision = []
    sim = []
    recall.append(1.0)
    precision.append(0.0)
    sim.append(0.0)
    topo_num = 0
    for pr in pr_list:
        if pr['precision'] == 0 or pr['recall'] == 0:
            continue
        sim.append(pr['recall_sim'])
        precision.append(pr['precision'])
        recall.append(pr['recall'])
        topo_num = pr['topo num']
    recall.append(0.0)
    precision.append(1.0)
    sim.append(1.0)
    auc = trapz(precision, recall, dx=0.00001)
    return recall, precision, auc, topo_num

# -------------------------------- Show Fun --------------------------------

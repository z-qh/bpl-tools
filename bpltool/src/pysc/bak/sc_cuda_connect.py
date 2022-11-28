import math
import pickle
import time
import numpy as np
import sys
from global_judge_generate import Vertex, Posi
from make_sc_example import ScanContext

from numba import cuda
import numba

BLOCK_DIM = 80
BLOCK_SIZE = 1

np.set_printoptions(suppress=True, precision=3, threshold=sys.maxsize, linewidth=sys.maxsize)


@cuda.jit
def AccMat2VecCuda(mat_, vec_):
    wid = mat_.shape[1]
    tx = cuda.threadIdx.x + cuda.blockIdx.x * cuda.blockDim.x
    if tx >= BLOCK_DIM: return
    valid_count = 0
    vec_[tx, 0] = 0
    for i in range(wid):
        if math.isnan(mat_[tx, i]):
            continue
        vec_[tx, 0] += mat_[tx, i]
        valid_count += 1
    vec_[tx, 0] /= valid_count


@cuda.jit
def SimCalCuda(sc1, sc2, temp):
    hei = sc1.shape[0]
    tx = cuda.threadIdx.x + cuda.blockIdx.x * cuda.blockDim.x
    ty = cuda.threadIdx.y + cuda.blockIdx.y * cuda.blockDim.y
    if tx >= BLOCK_DIM or ty >= BLOCK_DIM: return
    dot = 0
    norm1 = 0
    norm2 = 0
    for i in range(hei):
        dot += sc1[i, tx] * sc2[i, ty]
        norm1 += sc1[i, tx] ** 2
        norm2 += sc2[i, ty] ** 2
    if tx < ty:
        x_ = ty - tx
    elif tx > ty:
        x_ = ty + BLOCK_DIM - tx
    else:
        x_ = 0
    y_ = ty
    if norm1 == 0 or norm2 == 0:
        temp[x_, y_] = math.nan
    else:
        temp[x_, y_] = dot / (math.sqrt(norm1) * math.sqrt(norm2))
    cuda.syncthreads()


def testSim(sc1, sc2, temp_mat, temp_vec, val):
    hei = sc1.shape[0]
    wid = sc1.shape[1]
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
                x_ = j + BLOCK_DIM - i
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
        if temp_vec[i, 0] > val[0, 0]: val[0, 0] = temp_vec[i, 0]


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


def distance_sc(sc1, sc2):
    num_sectors = sc1.shape[1]
    # repeate to move 1 columns
    sim_for_each_cols = np.zeros(num_sectors, dtype=np.float32)
    sim_mat = np.zeros((num_sectors, num_sectors), dtype=np.float32)
    for i in range(num_sectors):
        # Shift
        one_step = 1  # const
        sc1 = np.roll(sc1, one_step, axis=1)  # columne shift
        # compare
        sum_of_cos_sim = 0
        num_col_engaged = 0
        if i == 43:
            print(123)
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if not np.any(col_j_1) or not np.any(col_j_2):
                continue
            # calc sim
            cos_similarity = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cos_sim = sum_of_cos_sim + cos_similarity
            num_col_engaged = num_col_engaged + 1
            sim_mat[i, j] = cos_similarity
        sim_for_each_cols[i] = sum_of_cos_sim / num_col_engaged
    sim = max(sim_for_each_cols)
    return sim, sim_for_each_cols, sim_mat


@cuda.jit
def UpperTriangleDot(data, dest, need):
    # data 1W X 40 X 80
    # dest 1W X 1W
    # need < 1W
    tx = cuda.threadIdx.x + cuda.blockIdx.x * cuda.blockDim.x
    ty = cuda.threadIdx.y + cuda.blockIdx.y * cuda.blockDim.y
    if tx >= need or ty >= need or tx >= ty:
        return
    temp_mat = cuda.local.array((BLOCK_DIM, BLOCK_DIM), numba.types.float32)
    temp_vec = cuda.local.array((BLOCK_DIM, BLOCK_SIZE), numba.types.float32)
    temp_out = cuda.local.array((BLOCK_SIZE, BLOCK_SIZE), numba.types.float32)
    hei = data[tx].shape[0]
    wid = data[tx].shape[1]
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
                x_ = mat_ind_n + BLOCK_DIM - mat_ind_m
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


GRID_BLOCKX_SEIZ: int = 512
GRID_BLOCKY_SIEZ: int = 512
BLOCK_THREAD_SIZE: int = 1024
BLOCK_THREAD_VOXEL_LEN: int = int(math.sqrt(BLOCK_THREAD_SIZE))


@cuda.jit
def UpperTriangleDotArea(data, dest, xs, xe, ys, ye):
    # data 1W X 40 X 80
    # dest 1W X 1W
    # need < 1W
    ########### INDEX
    tx = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.x + cuda.threadIdx.x // BLOCK_THREAD_VOXEL_LEN
    ty = BLOCK_THREAD_VOXEL_LEN * cuda.blockIdx.y + cuda.threadIdx.x % BLOCK_THREAD_VOXEL_LEN
    if tx < xs or tx > xe or ty < ys or ty > ye:
        return
    if tx > ty:
        return
    if tx == ty:
        dest[tx, ty] = 1.0
        return
    ############ CAL PROCESS
    temp_mat = cuda.local.array((BLOCK_DIM, BLOCK_DIM), numba.types.float32)
    temp_vec = cuda.local.array((BLOCK_DIM, BLOCK_SIZE), numba.types.float32)
    temp_out = cuda.local.array((BLOCK_SIZE, BLOCK_SIZE), numba.types.float32)
    hei = data[tx].shape[0]
    wid = data[tx].shape[1]
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
                x_ = mat_ind_n + BLOCK_DIM - mat_ind_m
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


if __name__ == "__main__":
    time_vertex_dic_raw = pickle.load(open("time_vertex_dic_raw.pkl", "rb"))
    total_line = len(time_vertex_dic_raw)
    print("Load Vertex {:d}".format(total_line))
    start = time.time()
    sc_sc_sim = np.zeros((total_line, total_line), dtype=np.float32)
    sc_des_list_cuda = []
    sc_des_list = []
    ###################################################################
    for i, key_i in (enumerate(time_vertex_dic_raw.keys())):
        sc_des_list.append(time_vertex_dic_raw[key_i].sc.SCs[0])
    ###################################################################
    # tmp_mat = cuda.to_device(np.zeros((80, 80), dtype=np.float32).reshape((80, 80)))
    # tmp_vec = cuda.to_device(np.zeros((80, 1), dtype=np.float32).reshape((80, 1)))
    # for i, key_i in (enumerate(time_vertex_dic_raw.keys())):
    #     tmp_cuda_sc = cuda.to_device(time_vertex_dic_raw[key_i].sc.SCs[0])
    #     sc_des_list_cuda.append(tmp_cuda_sc)
    # for i in range(total_line):
    #     for j in range(i + 1, total_line):
    #         SimCalCuda[(BLOCK_DIM, BLOCK_DIM), (BLOCK_SIZE, BLOCK_SIZE)](sc_des_list_cuda[i], sc_des_list_cuda[j], tmp_mat)
    #         AccMat2VecCuda[(BLOCK_DIM, 1), (BLOCK_SIZE, 1)](tmp_mat, tmp_vec)
    #         sc_sc_sim[i][j] = max(tmp_vec.copy_to_host())[0]
    #     if i % 1 == 0:
    #         end = time.time()
    #         print("###############{:.2f} % use {:.2f}".format(i / total_line, end - start))
    ###################################################################
    np_data = np.zeros((total_line, 40, 80), dtype=np.float32)
    for i, key_i in (enumerate(time_vertex_dic_raw.keys())):
        np_data[i] = time_vertex_dic_raw[key_i].sc.SCs[0]
    np_data_gpu = cuda.to_device(np_data)
    gpu_res = cuda.to_device(sc_sc_sim)
    ###################################################################
    # UpperTriangleDot[(1024, 1024, 1), (32, 1, 1)](np_data_gpu, gpu_res, total_line)
    # cuda.synchronize()
    # gpu_ans = gpu_res.copy_to_host()
    ##################################################################
    UpperTriangleDotArea[(GRID_BLOCKX_SEIZ, GRID_BLOCKY_SIEZ), BLOCK_THREAD_SIZE](np_data_gpu, gpu_res, 0, total_line - 1, 0, total_line - 1)
    cuda.synchronize()
    gpu_ans = gpu_res.copy_to_host()  #
    ###################################################################
    # a = np_data_gpu[0]
    # b = np_data_gpu[1]
    # tmp_mat = cuda.to_device(np.zeros((BLOCK_DIM, BLOCK_DIM), dtype=np.float32))
    # tmp_vec = cuda.to_device(np.zeros((BLOCK_DIM, BLOCK_SIZE), dtype=np.float32))
    # tmp_out = cuda.to_device(np.zeros((BLOCK_SIZE, BLOCK_SIZE), dtype=np.float32))
    # SimCalCudaParal[(1,1,1), (1,1,1)](a, b, tmp_mat, tmp_vec, tmp_out)
    # cuda.synchronize()
    # aans = tmp_out.copy_to_host()
    # a_ = sc_des_list[0]
    # b_ = sc_des_list[1]
    # ans = SimilarityNumba(a_, b_)
    ###################################################################
    # a_ = sc_des_list[0]
    # b_ = sc_des_list[1]
    # ans, ans_vec, ans_mat = distance_sc(a_, b_)
    # tmp_mat = np.zeros((BLOCK_DIM, BLOCK_DIM), dtype=np.float32)
    # tmp_vec = np.zeros((BLOCK_DIM, BLOCK_SIZE), dtype=np.float32)
    # tmp_out = np.zeros((BLOCK_SIZE, BLOCK_SIZE), dtype=np.float32)
    # testSim(a_, b_, tmp_mat, tmp_vec, tmp_out)
    ###################################################################
    end = time.time()
    pickle.dump(gpu_ans, open("/home/qh/vertex_connect.pkl", "wb"))
    print("Total Cost: {:.4f} sec".format(end - start))

# import sys
# import numpy as np
# import pickle
# np.set_printoptions(suppress=True, precision=4, threshold=sys.maxsize, linewidth=sys.maxsize)
# data = pickle.load(open("/home/qh/vertex_connect.pkl", "rb"))
# aa = data[20:30, 20:30]

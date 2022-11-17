import math
import pickle
import time
import numpy as np
from global_judge_generate import Vertex, Posi
from make_sc_example import ScanContext

from numba import cuda
import numba

BLOCK_DIM = 80
BLOCK_SIZE = 1

np.set_printoptions(suppress=True, precision=3)

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
    if norm1 == 0 or norm2 == 0: temp[x_, y_] = math.nan
    else: temp[x_, y_] = dot / (math.sqrt(norm1) * math.sqrt(norm2))
    cuda.syncthreads()

@cuda.jit
def UpperTriangleDot(si, sj, ei, ej, data):
    return





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


if __name__ == "__main__":
    time_vertex_dic_raw = pickle.load(open("/home/qh/topo_ws/time_vertex_dic_raw.pkl", "rb"))
    total_line = len(time_vertex_dic_raw)
    print("Load Vertex {:d}".format(total_line))
    start = time.time()
    sc_sc_sim = np.zeros((total_line, total_line), dtype=np.float32)
    sc_des_list_cuda = []
    sc_des_list = []
    tmp_mat = cuda.to_device(np.zeros((80, 80), dtype=np.float32).reshape((80, 80)))
    tmp_vec = cuda.to_device(np.zeros((80, 1), dtype=np.float32).reshape((80, 1)))
    for i, key_i in (enumerate(time_vertex_dic_raw.keys())):
        tmp_cuda_sc = cuda.to_device(time_vertex_dic_raw[key_i].sc.SCs[0])
        sc_des_list.append(time_vertex_dic_raw[key_i].sc.SCs[0])
        sc_des_list_cuda.append(tmp_cuda_sc)
    for i in range(total_line):
        for j in range(i + 1, total_line):
            SimCalCuda[(BLOCK_DIM, BLOCK_DIM), (BLOCK_SIZE, BLOCK_SIZE)](sc_des_list_cuda[i], sc_des_list_cuda[j], tmp_mat)
            AccMat2VecCuda[(BLOCK_DIM, 1), (BLOCK_SIZE, 1)](tmp_mat, tmp_vec)
            sc_sc_sim[i][j] = max(tmp_vec.copy_to_host())[0]
        if i % 1 == 0:
            end = time.time()
            print("###############{:.2f} % use {:.2f}".format(i / total_line, end - start))
    end = time.time()
    print("Total Cost: {:.4f} sec".format(end - start))


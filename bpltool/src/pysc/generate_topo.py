import math
import pickle
import time
import numpy as np
import sys
import random
from global_judge_generate import Vertex, Posi
from make_sc_example import ScanContext

np.set_printoptions(suppress=True, precision=4, threshold=sys.maxsize, linewidth=sys.maxsize)


def dist(a, b):
    return math.sqrt((a.position.x - b.position.x) ** 2 + (a.position.y - b.position.y) ** 2)


if __name__ == "__main__":
    data_connect = pickle.load(open("vertex_connect_full.pkl", "rb"))
    data_list = pickle.load(open("vertex_list.pkl", "rb"))
    data_size = len(data_list)
    topo_list = []
    drop_list = []
    topo_vertex_dict = {}
    last_valid = 0
    for i in range(data_size):
        last_node = i - 1
        next_node = i + 1
        if i == 0:
            last_node = 0
        if i == data_size - 1:
            next_node = data_size - 1
        hasSim = False
        if len(topo_list) != 0:
            max_sim_ = max(data_connect[i, topo_list])
            j_list = np.where(data_connect[i] == max_sim_)[0]
            dis_min_j = np.argmin(abs(j_list - i))
            j = j_list[dis_min_j]
        else:
            max_sim_ = 0
            j = -1
        if max_sim_ > 0.6:
            hasSim = True
            data_list[i].father = last_node
            data_list[i].child = next_node
            data_list[i].near = j
        else:
            hasSim = False
        if not hasSim:
            data_list[i].valid = True
            # if dist(data_list[i], data_list[last_valid]) > 10:
            #     data_list[i].father = i
            # else:
            data_list[i].father = last_valid
            if not data_list[last_node].valid:
                if dist(data_list[data_list[last_node].near], data_list[i]) < 10:
                    data_list[i].near = data_list[last_node].near
                    print("get a sim edge in {:2d}-{:2d}".format(i, data_list[last_node].near))
            topo_list.append(i)
            topo_vertex_dict[i] = data_list[i]
            last_valid = i
            print("add new {:d}".format(i))
        else:
            data_list[i].valid = False
        if i % 100 == 0:
            print("process : {:.2f}%".format(i / data_size * 100))
    pickle.dump(topo_vertex_dict, open("topo/topo_vertex_dict_0.6.pkl", "wb"))
    print(123)

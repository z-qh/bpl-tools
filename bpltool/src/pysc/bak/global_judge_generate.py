import math
import pickle
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import make_sc_example
import Distance_SC
np.set_printoptions(suppress=True)


class Posi:
    def __init__(self, x_, y_, z_):
        self.x = x_
        self.y = y_
        self.z = z_


class Vertex:
    def __init__(self, cloud_data: np.ndarray, time: float, posi_: Posi):
        self.timestamp = time
        cloud_data = cloud_data[:, 0:4]
        self.sc = make_sc_example.ScanContext(cloud_data)
        self.valid = False
        self.father = -1
        self.child = -1
        self.near = -1
        self.position = posi_


time_pose_dict = {}
time_vertex_dic = {}
time_vec = []
compact_vertex = []
topo_vertex = []
drop_vertex = []

def BinFind(x, l, r):
    if r >= l:
        mid = int(l + (r - l) / 2)
        if abs(time_vec[mid] - x) < 0.05:
            return mid
        elif time_vec[mid] > x:
            return BinFind(x, l, mid - 1)
        else:
            return BinFind(x, mid + 1, r)
    else:
        return -1

def Distance(a:Vertex, b:Vertex):
    return math.sqrt((a.position.x-b.position.x)**2+(a.position.y-b.position.y)**2)


if __name__ == "__main__":
    # 加载位姿
    pose_file = "/home/qh/YES/dlut/gtFittingA.txt"
    pose_vec = np.loadtxt(pose_file, skiprows=1)
    pose_vec = np.delete(pose_vec, [4, 5, 6, 7], 1)
    for data in pose_vec:
        time_pose_dict[data[0]] = Posi(data[1], data[3], 0)
        time_vec.append(data[0])
    # 生成描述子
    count = 1
    bag_file = "/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag"
    bag = rosbag.Bag(bag_file)
    info = bag.get_type_and_topic_info()
    bag_clouds = bag.read_messages("/lslidar_point_cloud")
    for topic, msg, rtime in bag_clouds:
        time = rtime.to_sec()
        ind = BinFind(time, 0, len(time_vec) - 1)
        if ind == -1:
            continue
        posi = time_pose_dict[time_vec[ind]]
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar))
        time_vertex_dic[time] = Vertex(points, time, posi)
        compact_vertex.append(time)
        count += 1
        # if count > 10:
        #     break
        print("############## {:d} MkSC by{:.2f} in Posi:{:.2f}, {:.2f}, {:.2f}".format(count, time, posi.x, posi.y, posi.z))
    pickle.dump(time_vertex_dic, open("/home/qh/time_vertex_dic_raw.pkl", "wb"))
    print("save sc raw data")
    print("###############################################3")
    print("###############################################3")
    # 全局描述子判别与连接边
    last_time = compact_vertex[0]
    for pc_time_ind in range(len(compact_vertex)):
        last_time = -1
        next_time = -1
        simTime = -1
        pc_time = -1
        simFlag = False
        if pc_time_ind == 0:
            last_time = compact_vertex[pc_time_ind]
            next_time = compact_vertex[pc_time_ind+1]
        elif pc_time_ind == len(compact_vertex)-1:
            last_time = compact_vertex[pc_time_ind-1]
            next_time = compact_vertex[pc_time_ind]
        else:
            last_time = compact_vertex[pc_time_ind-1]
            next_time = compact_vertex[pc_time_ind+1]
        pc_time = compact_vertex[pc_time_ind]
        for pcand_time in topo_vertex:
            simlity = Distance_SC.Similarity(time_vertex_dic[pc_time].sc.SCs[0], time_vertex_dic[pcand_time].sc.SCs[0])
            if simlity > 0.7:
                simFlag = True
                simTime = time_vertex_dic[pcand_time].timestamp
                if Distance(time_vertex_dic[pc_time], time_vertex_dic[pcand_time]) < 5:
                    break
        time_vertex_dic[pc_time].father = last_time
        time_vertex_dic[pc_time].child = next_time
        if simFlag is True:
            time_vertex_dic[pc_time].near = simTime
            time_vertex_dic[pc_time].valid = False
            drop_vertex.append(pc_time)
        else:
            time_vertex_dic[pc_time].valid = True
            topo_vertex.append(pc_time)
    print(123)
    # 保存对象以节省CPU
    # time_vertex_dic, topo_vertex, drop_vertex
    pickle.dump(time_vertex_dic, open("/home/qh/time_vertex_dic.pkl", "wb"))
    pickle.dump(topo_vertex, open("/home/qh/topo_vertex.pkl", "wb"))
    pickle.dump(drop_vertex, open("/home/qh/drop_vertex.pkl", "wb"))



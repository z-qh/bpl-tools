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


def GetAreaPointCloud2(voxel_dict, boundary, x_resolution, y_resolution, z_resolution):
    tmp_points = np.zeros((0, 3), dtype=np.float32)
    sy = int(math.ceil(boundary[0] / x_resolution))
    ey = int(math.ceil(boundary[1] / x_resolution))
    sx = int(math.ceil(boundary[2] / y_resolution))
    ex = int(math.ceil(boundary[3] / y_resolution))
    sz = int(math.ceil(boundary[4] / z_resolution))
    ez = int(math.ceil(boundary[5] / z_resolution))
    for k in range(sz, ez):
        for i in range(sx, ex):
            for j in range(sy, ey):
                key = "{:d} {:d} {:d}".format(i, j, k)
                voxel = voxel_dict.get(key)
                if voxel is not None:
                    tmp_points = np.row_stack((tmp_points, voxel))
    return tmp_points


# # 为了画比较容易理解的流程图
def A():
    global point_cloud1, point_cloud2, sc1, asc1, sc2, asc2, point_cloudmid, acc_cloud, bound3, bound2, bound1
    accmap = Base.GetVoxelMap(cloud_path=None,
                              path="/home/qh/YES/dlut/Daquan19/acc_global_voxel_map.pkl",
                              x_res=1.0, y_res=1.0, z_res=1.5)
    pose_vec_data = Base.GetPoseVec("/home/qh/YES/dlut/Daquan19/liosave/sam2.txt")
    bag = rosbag.Bag("/home/qh/YES/dlut/2021-01-19-11-10-53DaQuanL.bag")
    bag_clouds = bag.read_messages("/lslidar_point_cloud")
    traj = np.zeros((0, 3), dtype=np.float32)
    count = 0
    for topic, msg, rtime in bag_clouds:
        now_time = rtime.to_sec()
        ind = Base.BinFind(pose_vec_data, now_time, 0, pose_vec_data.shape[0] - 1)
        if ind == -1:
            continue
        if count < 6094:
            count += 1
            continue
        if count > 6490:
            break
        lidar = pc2.read_points(msg)
        p_ = pose_vec_data[ind, 1:4].reshape((3, 1))
        traj = np.row_stack((traj, p_.reshape((1, 3))))
        r_ = Rotation.from_quat(pose_vec_data[ind, [5, 6, 7, 4]]).as_matrix().astype(np.float64)
        points = np.array(list(lidar))
        boundary = Base.getBound(points)
        if count == 6094:
            point_cloud1 = points[:, 0:3]
            sc1 = Base.genTopoSC(Base.TopoNode(count, p_, r_, boundary, now_time), points, ch=5)
            topo_cloud = Base.GetAreaPointCloud(accmap.voxel_dict, p_, boundary, accmap.x_resolution,
                                                accmap.y_resolution, accmap.z_resolution)
            topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
            asc1 = Base.genTopoSC(Base.TopoNode(count, p_, r_, boundary, now_time), topo_cloud)
        if count == 6292:
            point_cloudmid = points[:, 0:3]
        if count == 6490:
            point_cloud2 = points[:, 0:3]
            sc2 = Base.genTopoSC(Base.TopoNode(count, p_, r_, boundary, now_time), points, ch=5)
            topo_cloud = Base.GetAreaPointCloud(accmap.voxel_dict, p_, boundary, accmap.x_resolution,
                                                accmap.y_resolution, accmap.z_resolution)
            topo_cloud = Base.TransInvPointCloud(topo_cloud, r_, p_)
            asc2 = Base.genTopoSC(Base.TopoNode(count, p_, r_, boundary, now_time), topo_cloud)
        count += 1
    return point_cloud1, point_cloud2, point_cloudmid, sc1, sc2, asc1, asc2


if __name__ == "__main__":
    # pc1, pc2, pcmid, sc1, sc2, asc1, asc2 = A()
    # pcd1 = open3d.geometry.PointCloud()
    # pcd1.points = open3d.utility.Vector3dVector(pc1)
    # open3d.io.write_point_cloud("/home/qh/YES/dlut/Daquan19/vis/pc1.pcd", pcd1)
    # pcd2 = open3d.geometry.PointCloud()
    # pcd2.points = open3d.utility.Vector3dVector(pc2)
    # open3d.io.write_point_cloud("/home/qh/YES/dlut/Daquan19/vis/pc2.pcd", pcd2)
    # pcdmid = open3d.geometry.PointCloud()
    # pcdmid.points = open3d.utility.Vector3dVector(pcmid)
    # open3d.io.write_point_cloud("/home/qh/YES/dlut/Daquan19/vis/pcmid.pcd", pcdmid)
    # pickle.dump(sc1, open("/home/qh/YES/dlut/Daquan19/vis/sc1.pkl", "wb"))
    # pickle.dump(sc2, open("/home/qh/YES/dlut/Daquan19/vis/sc2.pkl", "wb"))
    # pickle.dump(asc1, open("/home/qh/YES/dlut/Daquan19/vis/asc1.pkl", "wb"))
    # pickle.dump(asc2, open("/home/qh/YES/dlut/Daquan19/vis/asc2.pkl", "wb"))
    # Base.plot_multiple_sc(sc1.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/sc1.png", vis=False)
    # Base.plot_multiple_sc(sc2.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/sc2.png", vis=False)
    # Base.plot_multiple_sc(asc1.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/asc1.png", vis=False)
    # Base.plot_multiple_sc(asc2.SCs, save_path="/home/qh/YES/dlut/Daquan19/vis/asc2.png", vis=False)
    # sc1 = pickle.load(open("/home/qh/YES/dlut/Daquan19/vis/sc1.pkl", "rb"))
    # sc2 = pickle.load(open("/home/qh/YES/dlut/Daquan19/vis/sc2.pkl", "rb"))
    # bound_all = (
    #     min(sc1.position[0] + sc1.boundary[0], sc2.position[0] + sc2.boundary[0]),
    #     max(sc1.position[0] + sc1.boundary[1], sc2.position[0] + sc2.boundary[1]),
    #     min(sc1.position[1] + sc1.boundary[2], sc2.position[1] + sc2.boundary[2]),
    #     max(sc1.position[1] + sc1.boundary[3], sc2.position[1] + sc2.boundary[3]),
    #     min(sc1.position[2] + sc1.boundary[4], sc2.position[2] + sc2.boundary[4]),
    #     max(sc1.position[2] + sc1.boundary[5], sc2.position[2] + sc2.boundary[5])
    # )
    #
    # accmap = Base.GetVoxelMap(cloud_path=None,
    #                           path="/home/qh/YES/dlut/Daquan19/acc_global_voxel_map.pkl",
    #                           x_res=1.0, y_res=1.0, z_res=1.5)
    # aa = Base.GetAreaPointCloud(accmap.voxel_dict, sc1.position, sc1.boundary, accmap.x_resolution, accmap.y_resolution, accmap.z_resolution)
    # bb = Base.GetAreaPointCloud(accmap.voxel_dict, sc2.position, sc2.boundary, accmap.x_resolution, accmap.y_resolution, accmap.z_resolution)
    # #
    # pcd = open3d.geometry.PointCloud()
    # pcd.points = open3d.utility.Vector3dVector(aa)
    # # open3d.io.write_point_cloud("/home/qh/YES/dlut/Daquan19/vis/acc1.pcd", pcd)
    # pcd2 = open3d.geometry.PointCloud()
    # pcd2.points = open3d.utility.Vector3dVector(bb)
    # open3d.io.write_point_cloud("/home/qh/YES/dlut/Daquan19/vis/acc2.pcd", pcd2)
    # # axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=[0, 0, 0])
    # open3d.visualization.draw_geometries([pcd])

    pcd = open3d.io.read_point_cloud("/home/qh/YES/dlut/Daquan19/vis/acc2.pcd")
    open3d.visualization.draw_geometries([pcd])

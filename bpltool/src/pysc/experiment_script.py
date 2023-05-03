import os
import numpy as np
import open3d as o3d
import glob
import Daquan


def visOxfordDynamic():
    bin_to_use = sorted(glob.glob("/home/qh/YES/oxford/o1/left_bin" + '/*.bin'))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)
    for i in range(0, len(bin_to_use), 2):
        f = bin_to_use[i]
        points = np.fromfile(f, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        pointcloud.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(pointcloud)
        if i % 2000 == 0:
            a = input()
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        vis.poll_events()
        vis.update_renderer()
        print("\r", i, end="")
    # O1: 0-7000帧


def visDaquan():
    # [18500:20500]
    bin_list19 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan19", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan19", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan19", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan19", "bin_info.pkl"))

    # [21400:24000]
    bin_list16 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan16", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan16", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan16", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan16", "bin_info.pkl"))
    # [19900:22200]
    bin_list17 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan17", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan17", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan17", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan17", "bin_info.pkl"))

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)
    for i in range(0, len(bin_list16), 1):
        f = bin_list16[i][2]
        points = np.fromfile(f, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        pointcloud.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(pointcloud)
        if i % 100 == 0:
            a = input()
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        vis.poll_events()
        vis.update_renderer()
        print("\r", i, end="")

if __name__ == "__main__":
    visDaquan()

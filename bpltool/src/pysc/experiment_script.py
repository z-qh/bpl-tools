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
    bin_list19 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan19", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan19", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan19", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan19", "bin_info.pkl"))[18500:20500]

    bin_list16 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan16", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan16", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan16", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan16", "bin_info.pkl"))[21400:24000]

    bin_list17 = Daquan.GetBinList(bin_dir=os.path.join("/home/qh/YES/dlut/Daquan17", "bin"),
                                   bin_times_file=os.path.join("/home/qh/YES/dlut/Daquan17", "timestamp"),
                                   pose_vec_file=os.path.join("/home/qh/YES/dlut/Daquan17", "liosave/sam2.txt"),
                                   save_path=os.path.join("/home/qh/YES/dlut/Daquan17", "bin_info.pkl"))[19900:22200]

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)
    for i in range(0, len(bin_list16), 1):
        # if 18500 <= i < 20500:
            # if 17300 <= i < 17700 or 18750 <= i < 19200 or 11400 <= i < 12500: #16
            # if 10050 <= i < 11000 or 16200 <= i < 16500 or 14900 <= i < 15250:#17
            # print("\r", i, end="")
        # else:
        #     continue
        f = bin_list16[i][2]
        points = np.fromfile(f, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        pointcloud.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(pointcloud)
        # if i % 1000 == 0:
        #     a = input()
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        vis.poll_events()
        vis.update_renderer()
        print("\r", i, end="")
    # 9K-11K + 16K-19K


if __name__ == "__main__":
    visDaquan()

import os
import numpy as np
import open3d as o3d
import glob


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
    bin_to_use = sorted(glob.glob("/home/qh/YES/dlut/Daquan19/bin" + '/*.bin'))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)
    for i in range(16000, len(bin_to_use), 2):
        f = bin_to_use[i]
        points = np.fromfile(f, dtype=np.float32).reshape((-1, 4))[:, 0:3]
        pointcloud.points = o3d.utility.Vector3dVector(points)
        vis.update_geometry(pointcloud)
        if i % 1000 == 0:
            a = input()
        if to_reset:
            vis.reset_view_point(True)
            to_reset = False
        vis.poll_events()
        vis.update_renderer()
        print("\r", i, end="")
    # 9K-11K + 16K-19K


if __name__ == "__main__":
    visDaquan()

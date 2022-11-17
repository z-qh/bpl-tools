import math
import os
import numpy as np
import matplotlib.pyplot as plt
import open3d.cuda.pybind.utility
from mpl_toolkits.mplot3d import Axes3D
from open3d import *


def createCycle(radius, origin=[0, 0, 0], pointsize_per=100):
    point_size = int(2.0 * radius * np.pi * pointsize_per)
    cir = np.zeros((0, 3), dtype=np.float32)
    for i in range(point_size):
        angle = i / point_size * np.pi * 2
        x = radius * np.cos(angle) + origin[0]
        y = radius * np.sin(angle) + origin[1]
        z = origin[2]
        tmp = np.array([x, y, z]).astype(np.float32)
        cir = np.vstack((cir, tmp))
    return cir


def createLine(p1=[0, 0, 0], p2=[1, 1, 1], pointsize_per=100):
    a = np.array(p1)
    b = np.array(p2)
    point_size = int(np.linalg.norm(a - b) * pointsize_per)
    line = np.zeros((0, 3), dtype=np.float32)
    for i in range(point_size):
        percent = i / point_size
        x = (p2[0] - p1[0]) * percent + p1[0]
        y = (p2[1] - p1[1]) * percent + p1[1]
        z = (p2[2] - p1[2]) * percent + p1[2]
        tmp = np.array([x, y, z]).astype(np.float32)
        line = np.vstack((line, tmp))
    return line


def createVoxel(cloud_data):
    maxh = 6
    minh = -2
    cloud = np.zeros((0, 3), dtype=np.float32)
    maxdis = 100
    mindis = 5
    max_rings = 40
    rings = 20
    vecs = 80

    for i in range(rings):
        r = ((maxdis - mindis) / max_rings * i) + mindis
        c1 = createCycle(r, [0, 0, maxh], pointsize_per=100)
        c2 = createCycle(r, [0, 0, minh], pointsize_per=100)
        cloud = np.vstack((cloud, c1))
        cloud = np.vstack((cloud, c2))
    for i in range(vecs):
        angle = i / vecs * np.pi * 2
        sr = ((maxdis - mindis) / max_rings * 0) + mindis
        er = ((maxdis - mindis) / max_rings * (rings-1)) + mindis
        p1x = np.cos(angle) * sr
        p1y = np.sin(angle) * sr
        p1z = maxh
        p2x = np.cos(angle) * er
        p2y = np.sin(angle) * er
        p2z = maxh
        cloud = np.vstack((cloud, createLine([p1x, p1y, p1z], [p2x, p2y, p2z], 100)))
        p3x = np.cos(angle) * sr
        p3y = np.sin(angle) * sr
        p3z = minh
        p4x = np.cos(angle) * er
        p4y = np.sin(angle) * er
        p4z = minh
        cloud = np.vstack((cloud, createLine([p3x, p3y, p3z], [p4x, p4y, p4z], 100)))
        for j in range(rings):
            r = ((maxdis - mindis) / max_rings * j) + mindis
            pax = np.cos(angle) * r
            pay = np.sin(angle) * r
            paz = maxh
            pbz = minh
            cloud = np.vstack((cloud, createLine([pax, pay, paz], [pax, pay, pbz], 100)))
    return cloud


if __name__ == "__main__":
    ptcloud_xyz = np.fromfile("data/007229.bin", dtype=np.float32).reshape((-1, 4))[:, 0:3].astype(np.float32)
    cloud_pcd = open3d.geometry.PointCloud()
    cloud_pcd.points = open3d.utility.Vector3dVector(ptcloud_xyz)
    cloud_pcd.paint_uniform_color([1, 1, 1])

    # voxel_xyz = createVoxel(ptcloud_xyz)
    # cloud_voxel = open3d.geometry.PointCloud()
    # cloud_voxel.points = open3d.utility.Vector3dVector(voxel_xyz)
    # cloud_voxel.paint_uniform_color([1.0, 0, 0])

    open3d.io.write_point_cloud("sense.pcd", cloud_pcd)
    # open3d.io.write_point_cloud("voxle.pcd", cloud_voxel)

    vis = open3d.visualization.Visualizer()
    vis.create_window()
    render_option = open3d.visualization.RenderOption = vis.get_render_option()
    render_option.background_color = np.array([0, 0, 0])
    render_option.point_size = 1.0
    vis.add_geometry(cloud_pcd)
    # vis.add_geometry(cloud_voxel)
    vis.run()

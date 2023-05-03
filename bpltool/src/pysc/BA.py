import numpy as np
import os
import glob
from PIL import Image

import cv2
import matplotlib.pyplot as plt
import pandas as pd
from random import sample


def do_range_projection(proj_fov_up=15.0, proj_fov_down=-16.0, proj_W=2000, proj_H=200, points=None):
    # laser parameters
    fov_up = proj_fov_up / 180.0 * np.pi  # field of view up in rad
    fov_down = proj_fov_down / 180.0 * np.pi  # field of view down in rad
    fov = abs(fov_down) + abs(fov_up)  # get field of view total in rad

    # get depth of all points
    depth = np.linalg.norm(points[:, :3], 2, axis=1)

    # get scan components
    scan_x = points[:, 0]
    scan_y = points[:, 1]
    scan_z = points[:, 2]

    # get angles of all points
    yaw = -np.arctan2(scan_y, scan_x)
    pitch = np.arcsin(scan_z / depth)

    # get projections in image coords
    proj_x = 0.5 * (yaw / np.pi + 1.0)  # in [0.0, 1.0]
    proj_y = 1.0 - (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]

    # scale to image size using angular resolution
    proj_x *= proj_W  # in [0.0, W]
    proj_y *= proj_H  # in [0.0, H]

    # round and clamp for use as index
    proj_x = np.floor(proj_x)
    proj_x = np.minimum(proj_W - 1, proj_x)
    proj_x = np.maximum(0, proj_x).astype(np.int32)  # in [0,W-1]
    proj_x = np.copy(proj_x)  # store a copy in orig order

    proj_y = np.floor(proj_y)
    proj_y = np.minimum(proj_H - 1, proj_y)
    proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
    proj_y = np.copy(proj_y)  # stope a copy in original order

    # copy of depth in original order
    # unproj_range = np.copy(depth)

    # order in decreasing depth
    indices = np.arange(depth.shape[0])
    order = np.argsort(depth)[::-1]
    depth = depth[order]
    indices = indices[order]
    points = points[order]
    proj_y = proj_y[order]
    proj_x = proj_x[order]

    # ------------------------  深度图  ---------------------------
    proj_range = np.full((proj_H, proj_W), -1, dtype=np.float32)
    # assing to images
    proj_range[proj_y, proj_x] = depth * 255
    proj_range[proj_range < 0] = 0
    proj_range = np.around(proj_range).astype(np.int16)
    img = Image.fromarray(proj_range, 'I;16')
    # img.show()
    return img


if __name__ == '__main__':
    # segmention()

    file_path = "/home/qh/kitti/00/velodyne/000000.bin"
    bin_files = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    do_range_projection(points=bin_files)

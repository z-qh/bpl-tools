################################################################################
#
# Copyright (c) 2017 University of Oxford
# Authors:
#  Dan Barnes (dbarnes@robots.ox.ac.uk)
#
# This work is licensed under the Creative Commons
# Attribution-NonCommercial-ShareAlike 4.0 International License.
# To view a copy of this license, visit
# http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to
# Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
#
###############################################################################

from typing import AnyStr
import numpy as np
import os
import cv2
import pcl_ros

# Hard coded configuration to simplify parsing code
hdl32e_range_resolution = 0.002  # m / pixel
hdl32e_minimum_range = 1.0
hdl32e_elevations = np.array([-0.1862, -0.1628, -0.1396, -0.1164, -0.0930,
                              -0.0698, -0.0466, -0.0232, 0., 0.0232, 0.0466, 0.0698,
                              0.0930, 0.1164, 0.1396, 0.1628, 0.1862, 0.2094, 0.2327,
                              0.2560, 0.2793, 0.3025, 0.3259, 0.3491, 0.3723, 0.3957,
                              0.4189, 0.4421, 0.4655, 0.4887, 0.5119, 0.5353])[:, np.newaxis]
hdl32e_base_to_fire_height = 0.090805
hdl32e_cos_elevations = np.cos(hdl32e_elevations)
hdl32e_sin_elevations = np.sin(hdl32e_elevations)


def load_velodyne_binary(velodyne_bin_path: AnyStr):
    ext = os.path.splitext(velodyne_bin_path)[1]
    if ext != ".bin":
        raise RuntimeError("Velodyne binary pointcloud file should have `.bin` extension but had: {}".format(ext))
    if not os.path.isfile(velodyne_bin_path):
        raise FileNotFoundError("Could not find velodyne bin example: {}".format(velodyne_bin_path))
    data = np.fromfile(velodyne_bin_path, dtype=np.float32)
    ptcld = data.reshape((4, -1))
    return ptcld


def load_velodyne_raw(velodyne_raw_path: AnyStr):
    ext = os.path.splitext(velodyne_raw_path)[1]
    if ext != ".png":
        raise RuntimeError("Velodyne raw file should have `.png` extension but had: {}".format(ext))
    if not os.path.isfile(velodyne_raw_path):
        raise FileNotFoundError("Could not find velodyne raw example: {}".format(velodyne_raw_path))
    example = cv2.imread(velodyne_raw_path, cv2.IMREAD_GRAYSCALE)
    intensities, ranges_raw, angles_raw, timestamps_raw = np.array_split(example, [32, 96, 98], 0)
    ranges = np.ascontiguousarray(ranges_raw.transpose()).view(np.uint16).transpose()
    ranges = ranges * hdl32e_range_resolution
    angles = np.ascontiguousarray(angles_raw.transpose()).view(np.uint16).transpose()
    angles = angles * (2. * np.pi) / 36000
    approximate_timestamps = np.ascontiguousarray(timestamps_raw.transpose()).view(np.int64).transpose()
    return ranges, intensities, angles, approximate_timestamps


class Point:
    x: float = 0
    y: float = 0
    z: float = 0
    intensity: float = 0

    def __init__(self, x_: float = 0, y_: float = 0, z_: float = 0, i_: float = 0):
        self.x = x_
        self.y = y_
        self.z = z_
        self.intensity = i_
    def __str__(self):
        return "{:.7f} {:.7f} {:.7f} {:.4f}".format(self.x, self.y, self.z, self.intensity)


class PointCloud:
    cloud = []

    def __init__(self):
        self.cloud.clear()

    def clear(self):
        self.cloud.clear()

    def __len__(self):
        return len(self.cloud)

    def append(self, point_: Point):
        self.cloud.append(point_)

    def push_back(self, point_: Point):
        self.cloud.append(point_)

    def saveASCII(self, path):
        head = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 " \
               "4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {:d}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 " \
               "0\nPOINTS {:d}\nDATA ascii\n".format(len(self.cloud), len(self.cloud))
        file = open(path, "w")
        file.write(head)
        for point in self.cloud:
            file.write(str(point))
            file.write("\n")
        file.close()


def velodyne_raw_to_pointcloud(ranges: np.ndarray, intensities: np.ndarray, angles: np.ndarray):
    valid = ranges > hdl32e_minimum_range
    z = hdl32e_sin_elevations * ranges - hdl32e_base_to_fire_height
    xy = hdl32e_cos_elevations * ranges
    x = np.sin(angles) * xy
    y = -np.cos(angles) * xy

    xf = x[valid].reshape(-1)
    yf = y[valid].reshape(-1)
    zf = z[valid].reshape(-1)
    intensityf = intensities[valid].reshape(-1).astype(np.float32)
    ptcld = np.stack((xf, yf, zf, intensityf), 0)
    cloudTmp = PointCloud()
    size = len(ptcld[3])
    for index in range(size):
        thisPoint = Point(ptcld[0][index], ptcld[1][index], ptcld[2][index], ptcld[3][index])
        cloudTmp.push_back(thisPoint)
    return cloudTmp

def velodyne_raw_to_np(ranges: np.ndarray, intensities: np.ndarray, angles: np.ndarray):
    valid = ranges > hdl32e_minimum_range
    z = hdl32e_sin_elevations * ranges - hdl32e_base_to_fire_height
    xy = hdl32e_cos_elevations * ranges
    x = np.sin(angles) * xy
    y = -np.cos(angles) * xy

    xf = x[valid].reshape(-1)
    yf = y[valid].reshape(-1)
    zf = z[valid].reshape(-1)
    intensityf = intensities[valid].reshape(-1).astype(np.float32)
    ptcld = np.stack((xf, yf, zf, intensityf), 0)
    size = len(ptcld[3])
    lidar = []
    for index in range(size):
        linestr_convert = list([ptcld[0][index], ptcld[1][index], ptcld[2][index], ptcld[3][index]])
        lidar.append(linestr_convert)
    return np.array(lidar)

def pcd2bin(pcd:PointCloud):
    lidar = []
    for p in pcd.cloud:
        app = list([p.x,p.y,p.z,p.intensity])
        lidar.append(app)
    return np.array(lidar)

def saveALLPNG2PCD():
    file = open("/home/qh/YES/oxford/2019-01-10-11-46-21-radar-oxford-10k/velodyne_left_timestamps", "r")
    fileSourceBase = "/home/qh/YES/oxford/2019-01-10-11-46-21-radar-oxford-10k/velodyne_left"
    fileDestBase = "/home/qh/YES/oxford/2019-01-10-11-46-21-radar-oxford-10k/left_bin"
    lines = file.readlines()
    total = 0
    for line in lines:
        timeStamp = line.strip("\n").split(" ")[0]
        if(len(timeStamp) != 16):
            print(line)
        fileSourceTmp = fileSourceBase + "/" + timeStamp + ".png"
        fileDestTemp = fileDestBase + "/" + timeStamp + ".pcd"
        velodyne_file_new = fileDestBase + "/" + timeStamp + ".bin"
        if not os.path.isfile(fileSourceTmp):
            print("file not exist " + fileSourceTmp)
            exit(0)
        print("get data" + fileSourceTmp)
        total += 1
        ranges, intensities, angles, approximate_timestamps = load_velodyne_raw(fileSourceTmp)
        pcd = velodyne_raw_to_pointcloud(ranges, intensities, angles)
        # pcd.saveASCII(fileDestTemp)

        bin = velodyne_raw_to_np(ranges,intensities, angles)
        bin = bin.reshape(-1, 4).astype(np.float32)
        bin.tofile(velodyne_file_new)

    print(total)



if __name__ == "__main__":
    saveALLPNG2PCD()

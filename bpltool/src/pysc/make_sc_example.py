import os
import numpy as np
import matplotlib.pyplot as plt
import open3d.cuda.pybind.utility
from mpl_toolkits.mplot3d import Axes3D
from open3d import *

np.set_printoptions(suppress=True, precision=3, threshold=sys.maxsize, linewidth=sys.maxsize)


class kitti_vlp_database:
    def __init__(self, bin_dir):
        self.bin_dir = bin_dir
        self.bin_files = os.listdir(bin_dir);
        self.bin_files.sort()
        self.num_bins = len(self.bin_files)


class ScanContext:
    # static variables 
    viz = 0
    downcell_size = 0.5
    kitti_lidar_height = 2.0
    # sector_res = np.array([45, 90, 180, 360, 720])
    # ring_res = np.array([10, 20, 40, 80, 160])
    sector_res = np.array([80])
    ring_res = np.array([40])
    min_length = 5
    max_length = 100

    def __init__(self, cloud_data=None, path=None, channle=3):
            self.scancontexts = self.genSCs(cloud_data, path, channle)

    def load_velo_scan(self, cloud_data, path, channle):
        if not cloud_data is None:
            scan = cloud_data.reshape((-1, 4))
            ptcloud_xyz = scan[:, :-1]
            return ptcloud_xyz
        if not path is None:
            scan = np.fromfile(path, dtype=np.float32).reshape((-1, channle))
            ptcloud_xyz = scan[:, 0:3]
            return ptcloud_xyz

    def xy2theta(self, x, y):
        if (x >= 0 and y >= 0):
            theta = 180 / np.pi * np.arctan(y / x)
        if (x < 0 and y >= 0):
            theta = 180 - ((180 / np.pi) * np.arctan(y / (-x)))
        if (x < 0 and y < 0):
            theta = 180 + ((180 / np.pi) * np.arctan(y / x))
        if (x >= 0 and y < 0):
            theta = 360 - ((180 / np.pi) * np.arctan((-y) / x))
        return theta

    def pt2rs(self, point, gap_ring, gap_sector, num_ring, num_sector):
        x = point[0]
        y = point[1]
        z = point[2]
        if (x == 0.0):
            x = 0.001
        if (y == 0.0):
            y = 0.001
        theta = self.xy2theta(x, y)
        faraway = np.sqrt(x * x + y * y)
        idx_ring = np.divmod(faraway, gap_ring)[0]
        idx_sector = np.divmod(theta, gap_sector)[0]
        if (idx_ring >= num_ring):
            idx_ring = num_ring - 1  # python starts with 0 and ends with N-1
        return int(idx_ring), int(idx_sector)

    def ptcloud2sc(self, ptcloud, num_sector, num_ring, min_length, max_length):
        num_points = ptcloud.shape[0]
        gap_ring = max_length / num_ring
        gap_sector = 360 / num_sector
        enough_large = 1000
        sc_storage = np.zeros([enough_large, num_ring, num_sector])
        sc_counter = np.zeros([num_ring, num_sector])

        for pt_idx in range(num_points):
            point = ptcloud[pt_idx, :]
            lenght = np.sqrt(point[0] ** 2 + point[1] ** 2)
            if lenght < min_length or lenght > max_length:
                continue
            point_height = point[2] + ScanContext.kitti_lidar_height
            idx_ring, idx_sector = self.pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
            if sc_counter[idx_ring, idx_sector] >= enough_large:
                continue
            sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
            sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1
        sc = np.amax(sc_storage, axis=0)

        return sc

    def genSCs(self, cloud_data, path, channle):
        ptcloud_xyz = self.load_velo_scan(cloud_data, path, channle)
        print("The number of original points: " + str(ptcloud_xyz.shape))
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(ptcloud_xyz)
        downpcd = pcd.voxel_down_sample(voxel_size=ScanContext.downcell_size)
        ptcloud_xyz_downed = np.asarray(downpcd.points)
        print("The number of downsampled points: " + str(ptcloud_xyz_downed.shape))
        # draw_geometries([downpcd])
        if (ScanContext.viz):
            open3d.visualization.draw_geometries([downpcd])
        self.SCs = []
        for res in range(len(ScanContext.sector_res)):
            num_sector = ScanContext.sector_res[res]
            num_ring = ScanContext.ring_res[res]
            sc = self.ptcloud2sc(ptcloud_xyz_downed, num_sector, num_ring, ScanContext.min_length,
                                 ScanContext.max_length)
            self.SCs.append(sc)

    def plot_multiple_sc(self, fig_idx=1):
        if fig_idx == 1:
            fig, axes = plt.subplots(nrows=fig_idx)
            axes.set_title('Cloud Descriptor')
            axes.imshow(self.SCs[0], cmap="binary")
            # axes.imshow(self.SCs[0], cmap="gray")
            axes.invert_yaxis()
            fig.gca().set_aspect(1)
        else:
            num_res = len(ScanContext.sector_res)
            fig, axes = plt.subplots(nrows=num_res)
            axes[0].set_title('Scan Contexts with multiple resolutions')
            for ax, res in zip(axes, range(num_res)):
                ax.imshow(self.SCs[res], cmap="gray")
                ax.invert_yaxis()
        plt.savefig("007229.png", dpi=1200)
        plt.show()


if __name__ == "__main__":
    sc = ScanContext(path="data/007229.bin", channle=4)
    sc.plot_multiple_sc()
    # bin_dir = './data/'
    # bin_db = kitti_vlp_database(bin_dir)

    # for bin_idx in range(bin_db.num_bins):
    #     bin_file_name = bin_db.bin_files[bin_idx]
    #     bin_path = bin_db.bin_dir + bin_file_name
    #     sc = ScanContext(path=os.path.join(bin_dir,bin_file_name), channle=4)
    #     sc.plot_multiple_sc()
    #     # print((sc.SCs[0]))
    #     print(len(sc.SCs))
    #     print(sc.SCs[0].shape)

import os
import numpy as np
import fire


def read_pcd(filepath):
    lidar = []
    with open(filepath, 'r') as f:
        line = f.readline().strip()
        while line:
            linestr = line.split(" ")
            if len(linestr) == 4:
                linestr_convert = list(map(float, linestr))
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)

def convertAll():
    global pl
    ori_path = "/home/qh/oxford1/compact_data/pcd"
    file_list = os.listdir(ori_path)
    des_path = "/media/qh/何国建/DLUTdatasetBin/2019-01-10-11-46-21-radar-oxford-10k/compact_data/bin"
    total = 0
    for file in file_list:
        (filename, extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        pl = read_pcd(velodyne_file)
        pl = pl.reshape(-1, 4).astype(np.float32)
        velodyne_file_new = os.path.join(des_path, filename) + '.bin'
        pl.tofile(velodyne_file_new)
        print(len(pl))
        total+=1
        if total >= 2:
            break


if __name__ == "__main__":
    # convertAll()
    pl = read_pcd("/home/qh/oxford1/velodyne_bin/1547120785695080.pcd")
    pl = pl.reshape(-1, 4).astype(np.float32)
    pl.tofile("/home/qh/oxford1/velodyne_bin/1.bin")

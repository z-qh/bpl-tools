import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os
import time as ttime


# import ros_numpy


def convert(bag_file, save_path, lidar_topic="/os_cloud_node/points"):
    if bag_file is None or lidar_topic is None:
        print("Bag File Not Exist !")
        return
    bag = rosbag.Bag(bag_file)
    bag_clouds = bag.read_messages(lidar_topic)
    bag_count_info = bag.get_message_count(lidar_topic)
    count = 0
    handle_size = bag_count_info
    report_size = handle_size // 50 if handle_size // 50 != 0 else 1
    start_time = ttime.time()
    time_file = open(os.path.join(save_path, "time.txt"), "w")
    for topic, msg, rtime in bag_clouds:
        bin_file_path = os.path.join(save_path, "bin/{:06d}.bin".format(count))
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar), dtype=np.float32)[:, 0:4]
        points = points[(points[:, :3] != 0).any(axis=1)]
        points.tofile(bin_file_path)
        time_file.write("{:f}\n".format(rtime.to_sec()))
        count += 1
        if count % report_size == 0:
            print("Bag -> Bin {:.2f}% Cost: {:.2f}s".format(count / handle_size * 100, ttime.time() - start_time))
            start_time = ttime.time()
    time_file.close()


if __name__ == "__main__":
    convert("/home/qh/YES/jgxy/jgxy1.bag", "/home/qh/YES/jgxy/jgxy1/")
    convert("/home/qh/YES/jgxy/jgxy2.bag", "/home/qh/YES/jgxy/jgxy2/")

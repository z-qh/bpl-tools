import numpy as np
import glob
import sys
import time

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Not Data Found!")
        exit(0)
    bins = sorted(glob.glob(sys.argv[1]))
    handle_size = len(bins)
    report_size = handle_size // 50 if handle_size // 50 != 0 else 1
    start_time = time.time()
    proc_count = 0
    for bin in bins:
        pc = np.fromfile(bin, dtype=np.float32).reshape((-1, 4))
        pc[:, [0, 1, 2]] = -pc[:, [0, 1, 2]]
        pc.tofile(bin)
        proc_count += 1
        if proc_count % report_size == 0:
            print("Proc {:.2f} Cost {:.2f}".format(proc_count/handle_size*100, time.time()-start_time))
            start_time = time.time()
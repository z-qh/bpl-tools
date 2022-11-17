import math
import pickle
import time
import numpy as np
import sys
import random
from global_judge_generate import Vertex, Posi
from make_sc_example import ScanContext
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from mpl_toolkits.axes_grid1.inset_locator import TransformedBbox, BboxPatch, BboxConnector

def mark_inset2(parent_axes, inset_axes, loc1a=1, loc1b=1, loc2a=2, loc2b=2, **kwargs):
    rect = TransformedBbox(inset_axes.viewLim, parent_axes.transData)

    pp = BboxPatch(rect, fill=False, **kwargs)
    parent_axes.add_patch(pp)

    p1 = BboxConnector(inset_axes.bbox, rect, loc1=loc1a, loc2=loc1b, **kwargs)
    inset_axes.add_patch(p1)
    p1.set_clip_on(False)
    p2 = BboxConnector(inset_axes.bbox, rect, loc1=loc2a, loc2=loc2b, **kwargs)
    inset_axes.add_patch(p2)
    p2.set_clip_on(False)

    return pp, p1, p2

if __name__ == "__main__":

    points = np.zeros((2, 0), dtype=np.float32)
    fedges = np.zeros((2, 0), dtype=np.float32)
    nedges = np.zeros((2, 0), dtype=np.float32)
    topo_vertex_dict = pickle.load(open("topo/topo_vertex_dict_0.7.pkl", "rb"))
    count = 0
    for i, key in enumerate(topo_vertex_dict):
        count += 1
        # if count > 100: break
        pc = topo_vertex_dict[key]
        pf = topo_vertex_dict[pc.father]
        if pc.near == -1 or pc.near == pc.father:
            pn = None
        else:
            pn = topo_vertex_dict[pc.near]
        px = pc.position.y
        py = pc.position.x
        points = np.hstack((points, np.array((px, py)).astype(np.float32).reshape(2, 1)))
        e1x = pf.position.y
        e1y = pf.position.x
        fedges = np.hstack((fedges, np.array((px, e1x, py, e1y)).astype(np.float32).reshape(2, 2)))
        if pn is None: continue
        e2x = pn.position.y
        e2y = pn.position.x
        nedges = np.hstack((nedges, np.array((px, e2x, py, e2y)).astype(np.float32).reshape(2, 2)))

    fig, ax = plt.subplots(1, 1, facecolor='white', figsize=(24, 13.5))
    ax.scatter(points[0], points[1], marker="o", s=10, color="black", label='Vertex')
    ax.plot(fedges[0][0:2], fedges[1][0:2], "--", linewidth=1, color="black", alpha=0.6, label='Virtual Edge')
    ax.plot(nedges[0][0:2], nedges[1][0:2], linewidth=5, color="black", alpha=0.6, label='Real Edge')
    for i in range(1, int(fedges.shape[1] / 2)):
        ax.plot(fedges[0][i * 2:i * 2 + 2], fedges[1][i * 2:i * 2 + 2], "--", linewidth=1, color="black", alpha=0.6)
    for i in range(1, int(nedges.shape[1] / 2)):
        ax.plot(nedges[0][i * 2:i * 2 + 2], nedges[1][i * 2:i * 2 + 2], linewidth=5, color="black", alpha=0.6)
    ax.set_aspect(1)
    ax.legend(loc='upper left')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.axis("off")

    # 940
    # ax.set_xlim(850, 950)
    # ax.set_ylim(-100, 50)

    # 局部放大图 1
    # wind_size = 0.2
    # axins = ax.inset_axes((0.01, 0.03, wind_size, wind_size))
    # axins.scatter(points[0], points[1], marker="o", s=36, color="black")
    # for i in range(0, int(fedges.shape[1] / 2)):
    #     axins.plot(fedges[0][i * 2:i * 2 + 2], fedges[1][i * 2:i * 2 + 2], "--", linewidth=1, color="black", alpha=0.6)
    # for i in range(0, int(nedges.shape[1] / 2)):
    #     axins.plot(nedges[0][i * 2:i * 2 + 2], nedges[1][i * 2:i * 2 + 2], linewidth=5, color="black", alpha=0.6)
    # axins.get_xaxis().set_visible(False)
    # axins.get_yaxis().set_visible(False)
    # # axins.axis("off")
    # xc, yc, xlen = 607, -102, 200
    # ylen = xlen / 16 * 9
    # factor = 1 / 8
    # xlim0 = xc - xlen * factor
    # xlim1 = xc + xlen * factor
    # ylim0 = yc - ylen * factor
    # ylim1 = yc + ylen * factor
    # axins.set_xlim(xlim0, xlim1)
    # axins.set_ylim(ylim0, ylim1)
    # mark_inset2(ax, axins, loc1a=1, loc1b=2, loc2a=4, loc2b=4, fc="none", ec='k', lw=1)
    #
    # # 局部放大图 2
    # wind_size = 0.35
    # axins = ax.inset_axes((0.11, 0.63, 0.4, 0.35))
    # axins.scatter(points[0], points[1], marker="o", s=25, color="black")
    # for i in range(0, int(fedges.shape[1] / 2)):
    #     axins.plot(fedges[0][i * 2:i * 2 + 2], fedges[1][i * 2:i * 2 + 2], "--", linewidth=1, color="black", alpha=0.6)
    # for i in range(0, int(nedges.shape[1] / 2)):
    #     axins.plot(nedges[0][i * 2:i * 2 + 2], nedges[1][i * 2:i * 2 + 2], linewidth=5, color="black", alpha=0.6)
    # axins.get_xaxis().set_visible(False)
    # axins.get_yaxis().set_visible(False)
    # # axins.axis("off")
    # xc, yc, xlen = 835, -145, 460
    # ylen = xlen / 16 * 9
    # factor = 1 / 9
    # xlim0 = xc - xlen * factor
    # xlim1 = xc + xlen * factor
    # ylim0 = yc - ylen * factor
    # ylim1 = yc + ylen * factor
    # axins.set_xlim(xlim0, xlim1)
    # axins.set_ylim(ylim0, ylim1)
    # mark_inset2(ax, axins, loc1a=4, loc1b=2, loc2a=1, loc2b=1, fc="none", ec='k', lw=1)
    # #
    # # # 局部放大图 3
    # wind_size = 0.30
    # axins = ax.inset_axes((0.78, 0.03, 0.20, 0.20*16/9))
    # axins.scatter(points[0], points[1], marker="o", s=16, color="black")
    # for i in range(0, int(fedges.shape[1] / 2)):
    #     axins.plot(fedges[0][i * 2:i * 2 + 2], fedges[1][i * 2:i * 2 + 2], "--", linewidth=1, color="black", alpha=0.6)
    # for i in range(0, int(nedges.shape[1] / 2)):
    #     axins.plot(nedges[0][i * 2:i * 2 + 2], nedges[1][i * 2:i * 2 + 2], linewidth=5, color="black", alpha=0.6)
    # axins.get_xaxis().set_visible(False)
    # axins.get_yaxis().set_visible(False)
    # # axins.axis("off")
    # xc, yc, xlen = 892, -20, 300
    # ylen = xlen
    # factor = 1 / 10
    # xlim0 = xc - xlen * factor
    # xlim1 = xc + xlen * factor
    # ylim0 = yc - ylen * factor
    # ylim1 = yc + ylen * factor
    # axins.set_xlim(xlim0, xlim1)
    # axins.set_ylim(ylim0, ylim1)
    # mark_inset2(ax, axins, loc1a=3, loc1b=4, loc2a=2, loc2b=1, fc="none", ec='k', lw=1)

    plt.savefig("t0.6.png", dpi=600)
    plt.show()
    print(123)

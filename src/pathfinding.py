import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathfinder import pathfinder
import random
import time


def run(seed, start, end, neigh_range, sample_n, gridMap, background):
    # time1 = time.time()
    # gridMap = np.load('../res/sampled_sketch.npy')
    # gridMap = np.load('map.npy')
    # time2 = time.time()
    # print("图片加载完毕，耗时{}".format(time2 - time1))
    # maze = cv2.inRange(gridMap, 2.9, 3.1)

    # start = (0, 0)
    # end = (2500, 1000)
    # neigh_range = (200, 250)
    # sample_n = 20
    #     road1 = [(776, 523), (1425, 393), (2930, 122)]
    #     road2 = [(1285, 166), (1425, 393), (1880, 1075), (2020, 1973), (2086, 3737)]
    #     com_line = [(3125, 718), (900, 1700), (1000, 2265), (1166, 3337), (3060, 3142)]
    # forbidden =

    #     finder = pathfinder(maze, neigh_range, sample_n, [road1, road2], [com_line], gridMap)
    print("maze shape:{},{}".format(gridMap.shape[0],gridMap.shape[1]))
    print("类型：起点:{},终点:{}".format(gridMap[start[1]][start[0]],gridMap[end[1]][end[0]]))
    time3 = time.time()
    finder = pathfinder(seed, gridMap, neigh_range, sample_n)
    path = list(finder.astar(start, end))
    time4 = time.time()
    print("寻路完毕,耗时{}".format(time4 - time3))
    # maze_viz = cv2.cvtColor(maze, cv2.COLOR_GRAY2RGB)
    # background = cv2.imread("../res/sampled_map.png")
    # background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
    p1 = path[0]
    for index, p in enumerate(path):
        if index == 0:
            continue
        p2 = p
        cv2.line(background, p1, p2, (255, 0, 0), 40)
        p1 = p
    for p in path:
        cv2.circle(background, p, 10, (0, 0, 0), 40)
    # for p in finder.close_set:
    #     cv2.circle(background, p, 3, (0, 255, 0))
    plt.imshow(background)

    plt.savefig("../output/fig1216_{}_{}_{}.jpg".format(neigh_range[0], neigh_range[1],str(round(time.time()))[-5:]))
    plt.show()
    np.save("../output/path_1216_{}.npy".format(str(round(time.time()))[-5:]), np.array(path))
    time4 = time.time()
    print("算法完毕,总耗时{}".format(time4 - time1))


if __name__ == "__main__":
    seed = 0
    start = (0, 9044)
    end = (10000, 3900)
    neigh_range = (200, 250)
    sample_n = 10
    time1 = time.time()
    gridMap = np.load('../../res/v1/sampled_sketch.npy')
    gridMap.astype(int)
    background = cv2.imread("../../res/v1/sampled_map.png")
    background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
    time2 = time.time()
    print("图片加载完毕，耗时{}".format(time2 - time1))
    run(seed, start, end, neigh_range, sample_n, gridMap, background)

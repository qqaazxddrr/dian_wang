import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathfinder import pathfinder
import random
import time


def main():
    time1 = time.time()
    gridMap = np.load('geotif/sampled_map.npy')
    # gridMap = np.load('map.npy')
    time2 = time.time()
    print("图片加载完毕，耗时{}".format(time2 - time1))
    maze = cv2.inRange(gridMap, 2.9, 3.1)
    start = (0, 9044)
    end = (11690, 50)
    # start = (0, 0)
    # end = (2500, 1000)
    neigh_range = (200, 250)
    sample_n = 20
    #     road1 = [(776, 523), (1425, 393), (2930, 122)]
    #     road2 = [(1285, 166), (1425, 393), (1880, 1075), (2020, 1973), (2086, 3737)]
    #     com_line = [(3125, 718), (900, 1700), (1000, 2265), (1166, 3337), (3060, 3142)]
    # forbidden =

    #     finder = pathfinder(maze, neigh_range, sample_n, [road1, road2], [com_line], gridMap)
    print("maze shape:{},{}".format(maze.shape[0],maze.shape[1]))
    print("start:{},end:{}".format(maze[start[1]][start[0]],maze[end[1]][end[0]]))
    finder = pathfinder(maze, neigh_range, sample_n, gridMap=None)
    path = list(finder.astar(start, end))
    time3 = time.time()
    print("寻路完毕,耗时{}".format(time3 - time2))
    maze_viz = cv2.cvtColor(maze, cv2.COLOR_GRAY2RGB)
    p1 = path[0]
    for index, p in enumerate(path):
        if index == 0:
            continue
        p2 = p
        cv2.line(maze_viz, p1, p2, (255, 50, 0), 20)
        p1 = p
    for p in finder.close_set:
        cv2.circle(maze_viz, p, 5, (0, 255, 0))
    plt.imshow(maze_viz)

    plt.savefig("fig/fig1208_{}_{}.jpg".format(neigh_range[0], neigh_range[1]))
    plt.show()
    np.save("path_1208.npy", np.array(path))
    time4 = time.time()
    print("算法完毕,总耗时{}".format(time4 - time1))


if __name__ == "__main__":
    main()

import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathfinder import pathfinder
import random
import time

def main():
    gridMap = np.load('map.npy')
    maze = cv2.inRange(gridMap, 2.9, 3.1)
    start = (1400, 3500)
    end = (2200, 900)
    neigh_range = (300, 350)
    sample_n = 20
    road1 = [(776, 523), (1425, 393), (2930, 122)]
    road2 = [(1285, 166), (1425, 393), (1880, 1075), (2020, 1973), (2086, 3737)]
    com_line = [(3125, 718), (900, 1700), (1000, 2265), (1166, 3337), (3060, 3142)]
    # forbidden =


    finder = pathfinder(maze, neigh_range, sample_n, [road1, road2], [com_line], gridMap)
    path = list(finder.astar(start, end))
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

    plt.savefig("fig/fig1031_{}_{}.jpg".format(neigh_range[0], neigh_range[1]))
    plt.show()
    np.save("path.npy", np.array(path))


if __name__ == "__main__":
    main()
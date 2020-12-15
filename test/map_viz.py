import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathfinder import pathfinder
import random
from time import sleep
from tqdm import tqdm, trange

road1 = [(776, 523), (1425, 393), (2930, 122)]
road2 = [(1285, 166), (1425, 393), (1880, 1075), (2020, 1973), (2086, 3737)]
com_line = [(3125, 718), (900, 1700), (1000, 2265), (1166, 3337), (3060, 3142)]


def draw_line(pic, line, color, thickness):
    p1 = line[0]
    for index, p in enumerate(line):
        if index == 0:
            continue
        p2 = p
        cv2.line(pic, p1, p2, color, thickness)
        p1 = p

def background_generator():
    gridMap = np.load('map.npy')
    shape = gridMap.shape
    gridMap_viz = np.zeros((shape[0], shape[1], 3), np.uint8)
    for i in tqdm(range(shape[0])):
        for j in range(shape[1]):
            if gridMap[i, j] == 1:
                gridMap_viz[i, j] = [255, 255, 255]
            if gridMap[i, j] == 2:
                gridMap_viz[i, j] = [192, 192, 192]
            if gridMap[i, j] == 3:
                gridMap_viz[i, j] = [176, 224, 230]
    draw_line(gridMap_viz, road1, (255, 0, 0), 10)
    draw_line(gridMap_viz, road2, (255, 0, 0), 10)
    draw_line(gridMap_viz, com_line, (0, 0, 255), 10)
    # path = np.load("path.npy")
    # path = [(p[0], p[1]) for p in path]
    # draw_line(gridMap_viz, path, (0, 255, 0), 20)
    plt.imshow(gridMap_viz)
    # plt.savefig("fig/output.jpg")
    plt.show()
    np.save("background.npy", gridMap_viz)


if __name__ == '__main__':
    # background_generator()
    gridMap_viz = np.load("background.npy")
    draw_line(gridMap_viz, road1, (255, 0, 0), 10)
    draw_line(gridMap_viz, road2, (255, 0, 0), 10)
    draw_line(gridMap_viz, com_line, (0, 0, 255), 10)
    path = np.load("path.npy")
    path = [(p[0], p[1]) for p in path]
    draw_line(gridMap_viz, path, (0, 255, 0), 20)
    plt.imshow(gridMap_viz)
    plt.savefig("fig/output.jpg")
    plt.show()

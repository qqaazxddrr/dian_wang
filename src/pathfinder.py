from iastar import AStar
import numpy as np
import math
import random
from bresenham import bresenham


class1 = [5, 6]
class2 = [2]
class3 = [3, 1]
class4 = [4]


# def neighbors_generator(radius_inner, radius_outer, sample_n):
#     quadrant = []
#     result = set()
#     for x in range(0, radius_outer + 1):
#         for y in range(0, radius_outer + 1):
#             if radius_inner ** 2 <= x ** 2 + y ** 2 <= radius_outer ** 2:
#                 if x % sample_n == 0 and y % sample_n == 0:
#                     quadrant.append((x, y))
#
#     for x, y in quadrant:
#         result.add((x, y))
#         result.add((-x, y))
#         result.add((x, -y))
#         result.add((-x, -y))
#     return list(result)

# def neighbors_generator_v1(radius_inner, radius_outer, sample_n):
#     result=set()
#     for _ in range(sample_n):
#         degree = random.uniform(0,360)/180*np.pi
#         length = random.uniform(radius_inner,radius_outer)
#         x=round(np.cos(degree)*length)
#         y=round(np.sin(degree)*length)
#         result.add(((int(x)),(int(y))))
#     return list(result)


def neighbors_generator_v2(radius_inner, radius_outer, length_part, degree_delta):
    result = set()
    length_delta = int((radius_outer - radius_inner)/length_part)
    lengths = []
    for i in range(length_part):
        lengths.append(i*length_delta+radius_inner)
    # lengths = [radius_inner, radius_outer, int((radius_inner+radius_outer)/2)]
    for degree in range(0,360,degree_delta):
        for length in lengths:
            x = int(np.cos(degree/180*np.pi) * length)
            y = int(np.sin(degree / 180 * np.pi) * length)
            result.add((x,y))
    return list(result)


    return list(result)

class pathfinder(AStar):

    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, ver, gridMap, neigh_range, length_part=10, degree_delta=10, roads=None, com_lines=None, openset_size=800):
        super().__init__(neigh_range, roads, com_lines, gridMap, openset_size)
        self.gridMap = gridMap
        self.height, self.width = np.array(gridMap).shape
        self.neighbors_coordinate = neighbors_generator_v2(neigh_range[0], neigh_range[1], length_part, degree_delta)

    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    # 判断路径中是否存在第四类地块
    def is_forbiddenzoom_in_between(self, n1, n2):
        # --------- version 1--------------#
        route = list(bresenham(n1[0], n1[1], n2[0], n2[1]))
        for p in route:
            if self.gridMap[p[1]][p[0]] in class4:
                return True
        return False

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        neighbors = [(x+neiX, y+neiY) for neiX, neiY in self.neighbors_coordinate]
        # return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.height and 0 <= ny < self.width and self.maze[nx][ny] == 0]
        # return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.width and 0 <= ny < self.height and self.maze[nx][ny] == 0]
        filter1 = [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.width and 0 <= ny < self.height]
        filter2 = [i for i in filter1 if self.gridMap[i[1]][i[0]] in class1 or self.gridMap[i[1]][i[0]] in class2]
        filter3 = [j for j in filter2 if self.is_forbiddenzoom_in_between(node, j)]
        return filter3


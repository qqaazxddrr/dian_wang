from iastar import AStar
import numpy as np
import math


def neighbors_generator(radius_inner, radius_outer, sample_n):
    quadrant = []
    result = set()
    for x in range(0, radius_outer + 1):
        for y in range(0, radius_outer + 1):
            if radius_inner ** 2 <= x ** 2 + y ** 2 <= radius_outer ** 2:
                if x % sample_n == 0 and y % sample_n == 0:
                    quadrant.append((x, y))

    for x, y in quadrant:
        result.add((x, y))
        result.add((-x, y))
        result.add((x, -y))
        result.add((-x, -y))
    return list(result)


class pathfinder(AStar):

    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, gridMap, neigh_range, sample_n, roads=None, com_lines=None):
        super().__init__(neigh_range, roads, com_lines, gridMap)
        self.gridMap = gridMap
        self.height, self.width = np.array(gridMap).shape
        self.neighbors_coordinate = neighbors_generator(neigh_range[0], neigh_range[1], sample_n)

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

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        neighbors = [(x+neiX, y+neiY) for neiX, neiY in self.neighbors_coordinate]
        # return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.height and 0 <= ny < self.width and self.maze[nx][ny] == 0]
        # return [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.width and 0 <= ny < self.height and self.maze[nx][ny] == 0]
        filter1 = [(nx, ny) for nx, ny in neighbors if 0 <= nx < self.width and 0 <= ny < self.height]
        return [i for i in filter1 if self.gridMap[i[1]][i[0]] == 1]
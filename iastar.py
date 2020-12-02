# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify
import math
import numpy as np
from bresenham import bresenham

Infinite = float('inf')


def angle_computing(n1, n2):
    (x1, y1) = n1
    (x2, y2) = n2
    degree = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return degree


def cross(p1, p2, p3):  # 跨立实验
    x1 = p2[0] - p1[0]
    y1 = p2[1] - p1[1]
    x2 = p3[0] - p1[0]
    y2 = p3[1] - p1[1]
    return x1 * y2 - x2 * y1


def is_intersec(p1, p2, p3, p4):  # 判断两线段是否相交

    # 快速排斥，以l1、l2为对角线的矩形必相交，否则两线段不相交
    if (max(p1[0], p2[0]) >= min(p3[0], p4[0])  # 矩形1最右端大于矩形2最左端
            and max(p3[0], p4[0]) >= min(p1[0], p2[0])  # 矩形2最右端大于矩形最左端
            and max(p1[1], p2[1]) >= min(p3[1], p4[1])  # 矩形1最高端大于矩形最低端
            and max(p3[1], p4[1]) >= min(p1[1], p2[1])):  # 矩形2最高端大于矩形最低端

        # 若通过快速排斥则进行跨立实验
        if (cross(p1, p2, p3) * cross(p1, p2, p4) <= 0
                and cross(p3, p4, p1) * cross(p3, p4, p2) <= 0):
            return True
        else:
            return False
    else:
        return False


class AStar:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset', 'father_angle', 'start')

        def __init__(self, data, gscore=Infinite, fscore=Infinite, start=False):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None
            self.father_angle = None
            self.start = start

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def __init__(self, roads, com_lines, gridMap):
        self.close_set = set()
        self.roads = roads
        self.com_lines = com_lines
        self.map = gridMap

    # 判断路径中是否存在第四类地块
    def is_forbiddenzoom_in_between(self, n1, n2):
        route = list(bresenham(n1[0], n1[1], n2[0], n2[1]))
        for p in route:
            if self.map[p[0], p[1]] == 4:
                return True
        return False

    # 判断是否符合道路约束条件，相交且夹角<75度为False，其他情况为True
    def road_condition(self, n1, n2):
        for road in self.roads:
            p1 = road[0]
            for index, p in enumerate(road):
                if index == 0:
                    continue
                p2 = p
                if is_intersec(p1, p2, n1, n2):
                    degree = abs(angle_computing((p2[0] - p1[0], p2[1] - p1[1]), (n2[0] - n1[0], n2[1] - n1[1])))
                    if degree > 90:
                        degree = 180 - degree
                    if degree < 75:
                        return False
                p1 = p2
        return True

    # 判断是否符合通讯线路约束条件，相交且夹角<45度为False，其他情况为True
    def com_condition(self, n1, n2):
        for com_line in self.com_lines:
            p1 = com_line[0]
            for index, p in enumerate(com_line):
                if index == 0:
                    continue
                p2 = p
                if is_intersec(p1, p2, n1, n2):
                    degree = abs(angle_computing((p2[0] - p1[0], p2[1] - p1[1]), (n2[0] - n1[0], n2[1] - n1[1])))
                    if degree > 90:
                        degree = 180 - degree
                    if degree < 45:
                        return False
                p1 = p2
        return True

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a subclass. The second parameter is always the goal."""
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1, n2):
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        raise NotImplementedError

    @abstractmethod
    def neighbors(self, node):
        """For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedError

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reversePath=False):
        if self.is_goal_reached(start, goal):
            return [start]
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal), start=True)
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            self.close_set.add(current.data)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reversePath)
            current.out_openset = True
            current.closed = True
            for neighbor in map(lambda n: searchNodes[n], self.neighbors(current.data)):
                if self.roads is not None:
                    if not self.road_condition(current.data, neighbor.data):
                        continue
                if self.com_lines is not None:
                    if not self.com_condition(current.data, neighbor.data):
                        continue
                if self.is_forbiddenzoom_in_between(current.data, neighbor.data):
                    continue
                if not current.start:
                    degree = angle_computing(neighbor.data, current.data)
                    if abs(degree - current.father_angle) >= 90:
                        continue
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                                   self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                                  self.heuristic_cost_estimate(neighbor.data, goal)
                neighbor.father_angle = angle_computing(neighbor.data, current.data)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor)
                else:
                    # re-add the node in order to re-sort the heap
                    openSet.remove(neighbor)
                    heappush(openSet, neighbor)
        return None


def find_path(start, goal, neighbors_fnct, reversePath=False, heuristic_cost_estimate_fnct=lambda a, b: Infinite,
              distance_between_fnct=lambda a, b: 1.0, is_goal_reached_fnct=lambda a, b: a == b):
    """A non-class version of the path finding algorithm"""

    class FindPath(AStar):

        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            return is_goal_reached_fnct(current, goal)

    return FindPath().astar(start, goal, reversePath)


# __all__ = ['AStar', 'find_path']
if __name__ == "__main__":
    path = list(bresenham(0, 0, 4, 7))
    print(path)

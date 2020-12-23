# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify, nlargest
import math
import numpy as np
from bresenham import bresenham
from tqdm import tqdm
import random

Infinite = float('inf')
class1 = [5, 6]
class2 = [2]
class3 = [3, 1]
class4 = [4]


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
    def __init__(self, neigh_range, roads, com_lines, gridMap, openset_size):
        self.close_set = set()
        self.roads = roads
        self.com_lines = com_lines
        self.map = gridMap
        self.neigh_range = neigh_range
        # self.sample_openset = 10  # 从openset抽样节点的个数，防止一块区域重复采样
        # self.delay = 100        # 延迟一段时间后再去将新加节点与openset中的点做比较
        self.openset_size = openset_size # 设定一个固定大小的openset

    # 判断路径中是否存在第四类地块
    def is_forbiddenzoom_in_between(self, n1, n2):
        # --------- version 1--------------#
        route = list(bresenham(n1[0], n1[1], n2[0], n2[1]))
        for p in route:
            if self.map[p[1]][p[0]] in class4:
                return True
        return False
        # --------- version 2--------------#
        # route = list(bresenham(n1[0], n1[1], n2[0], n2[1]))
        # sample = route[int(len(route)/2)]
        # if self.map[sample[1]][sample[0]] == 4:
        #     return True
        # return False
        # --------- version 3--------------#
        # sample = [int((n1[0] + n2[0])/2), int((n1[1] + n2[1])/2)]
        # if self.map[sample[1]][sample[0]] == 4:
        #     return True
        # return False
        # --------- version 4--------------#
        # n = 10            # 10等分
        # route = []
        # delta_x = int((n2[0] - n1[0]) / n)
        # delta_y = int((n2[1] - n1[1]) / n)
        # for i in range(1,10):
        #     route.append((n1[0]+i*delta_x, n1[1]+i*delta_y))
        # for p in route:
        #     if self.map[p[1]][p[0]] == 4:
        #         return True
        # return False




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
        dis = self.distance_between(current, goal)
        # if self.neigh_range[0] <= dis <= self.neigh_range[1]:
        if dis <= self.neigh_range[1]:
            return True

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return list(reversed(list(_gen()))), list(self.close_set)

    def astar(self, start, goal, reversePath=False):
        process_max = 0
        pbar = tqdm(total=100)
        dis_total = self.distance_between(start, goal)
        if self.is_goal_reached(start, goal):
            return [start], None
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal), start=True)
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            # 进度条显示
            dis_now = self.distance_between(current.data, goal)
            process_now = round((1 - dis_now / dis_total) * 100)
            if process_now > process_max:
                process_max = process_now
                pbar.n = process_now
                pbar.refresh()
                print("目前OpenSet大小为：{}".format(len(openSet)))
            # -------------- #
            self.close_set.add(current.data)
            if self.is_goal_reached(current.data, goal):
                pbar.n = 100
                pbar.refresh()
                pbar.close()
                return self.reconstruct_path(current, reversePath)
            current.out_openset = True
            current.closed = True
            nei_tmp = self.neighbors(current.data)
            for neighbor in map(lambda n: searchNodes[n], nei_tmp):

                # -----------------openset取样-------------------- #
                # if len(openSet)>self.sample_openset and len(openSet)>self.delay:
                #     flag = False
                #     for openset_sample in random.sample(openSet, self.sample_openset):
                #         dis = self.distance_between(current.data, openset_sample.data)
                #         if dis < self.neigh_range[1]:
                #             flag = True
                #             break
                #     if flag:
                #         continue
                # ------------------------------------- #


                if self.roads is not None:
                    if not self.road_condition(current.data, neighbor.data):
                        continue
                if self.com_lines is not None:
                    if not self.com_condition(current.data, neighbor.data):
                        continue
                if self.map is not None:
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

                if len(openSet) > self.openset_size:
                    # heappop(openSet)
                    max_openset = max(openSet)
                    max_openset.out_openset = True
                    max_openset.closed = True
                    self.close_set.add(max_openset.data)
                    openSet.remove(max_openset)
        return None, list(self.close_set)


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

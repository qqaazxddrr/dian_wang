import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cv2
from pathfinder import pathfinder
import random
import time
import math
from multiprocessing import Process, Manager
import os
import argparse
from libtiff import TIFF
from osgeo import osr, ogr
# from tqdm import tqdm
import multiprocessing

DL = 1  # 1 道路
SX = 2  # 2 水系
FW = 3  # 3 房屋(居民用地)
JZYD = 4  # 4 建设用地
NT = 5  # 5 农田
LM = 6  # 6 林木
class1 = [5, 6]
class2 = [2]
class3 = [3, 1]
class4 = [4]

unique_tag = str(round(time.time()))[-5:]


def output_profile():
    if not os.path.exists("output/"):
        os.makedirs("output/")
    output_file = open("output/profile.txt", 'w')
    output_file.write("地图大小：100，100\n")
    output_file.write("电压等级为：3\n")
    output_file.write("起点为：0，0；终点为：100，100\n")
    output_file.write("起点与终点的直线距离为：100\n")
    output_file.write("规划路线总长为：100，共架设：10个塔基\n")
    output_file.write("综合代价为：1000\n")
    output_file.write("跨越水系次数：3\n")
    output_file.write("跨越交通线次数：3\n")
    output_file.close()


def road_extract(layer):
    line_list = []
    layer_shape = layer.GetExtent()
    for i in range(0, layer.GetFeatureCount()):
        feat = layer.GetFeature(i)
        geom = feat.geometry()
        if geom is None:
            continue
        line = []
        if geom.GetGeometryCount() > 0:
            for j in range(0, geom.GetGeometryCount()):
                g = feat.geometry().GetGeometryRef(j)
                for p in range(0, g.GetPointCount()):
                    pt = g.GetPoint(p)
                    #                     new_x,new_y=coordinate_transfer(pt[0],pt[1],layer_shape, sketch_shape)
                    #                     line.append((new_x,new_y))
                    line.append((int(pt[0]), int(layer_shape[3] - int(pt[1]))))
        else:
            for p in range(0, geom.GetPointCount()):
                pt = geom.GetPoint(p)
                #                 new_x,new_y=coordinate_transfer(pt[0],pt[1],layer_shape, sketch_shape)
                #                 line.append((new_x,new_y))
                line.append((int(pt[0]), int(layer_shape[3] - int(pt[1]))))
        line_list.append(line)
    return line_list


def point_generator(gridMap, type):
    X, Y = gridMap.shape
    while True:
        p_x = random.randint(0, X)
        p_y = random.randint(0, Y)
        if gridMap[p_x][p_y] == type:
            return p_x, p_y


def se_generator(gridMap):
    s_x, s_y = point_generator(gridMap, 1)
    e_x, e_y = point_generator(gridMap, 1)
    return s_x, s_y, e_x, e_y


def run(_ver, _return_dict, start, end, neigh_range, gridMap, background, openset_size, length_part, degree_delta,
        roads=None):
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
    # print("maze shape:{},{}".format(gridMap.shape[0], gridMap.shape[1]))
    # print("类型：起点:{},终点:{}".format(gridMap[start[1]][start[0]], gridMap[end[1]][end[0]]))
    time3 = time.time()
    plt.figure()
    finder = pathfinder(_ver, gridMap, neigh_range, openset_size=openset_size, length_part=length_part,
                        degree_delta=degree_delta, roads=roads)
    path, close_list = finder.astar(start, end)
    if path is None:
        # print("查找失败，无解")
        for p in close_list:
            cv2.circle(background, p, 5, (255, 0, 0), 2)
        plt.imshow(background)
        plt.savefig("output/{}/fail_fig_{}_{}_{}_ver{}.png".format(unique_tag, neigh_range[0], neigh_range[1],
                                                                   str(round(time.time()))[-5:], _ver))
        _return_dict[_ver] = 0
        return False
    time4 = time.time()
    print("寻路完毕,耗时{}".format(time4 - time3))
    p1 = path[0]
    for p in close_list:
        cv2.circle(background, p, 10, (0, 0, 255), 5)
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

    plt.savefig(
        "output/{}/fig_{}_{}_{}_ver{}.png".format(unique_tag, neigh_range[0], neigh_range[1],
                                                  str(round(time.time()))[-5:],
                                                  _ver))
    np.save("output/{}/path__{}_ver{}.npy".format(unique_tag, str(round(time.time()))[-5:], _ver), np.array(path))
    _return_dict[_ver] = 1
    return 1


if __name__ == "__main__":

    if not os.path.exists("output/"):
        os.makedirs("output/")
    if not os.path.exists("output/{}/".format(unique_tag)):
        os.makedirs("output/{}/".format(unique_tag))
    parser = argparse.ArgumentParser(description='电力寻路程序')
    parser.add_argument("--gridMap", help="地图的路径", type=str)
    parser.add_argument("--start", nargs="+", help="起点", type=int)
    parser.add_argument("--end", nargs="+", help="终点", type=int)
    parser.add_argument("-v", "--voltage", help="电压等级", type=int)
    parser.add_argument("-b", "--buffer", help="搜索集大小", type=int)
    parser.add_argument("-p", "--precision", help="搜索精确等级", type=int)
    parser.add_argument("-r", "--road", help="道路SHP文件的路径", type=str)
    args = parser.parse_args()
    try:
        start = (args.start[0], args.start[1])
    except:
        print("没有输入起点！")
    try:
        end = (args.end[0], args.end[1])
    except:
        print("没有输入终点！")
    voltage_level = args.voltage
    neigh_range = (500, 600)
    if voltage_level == 35:
        neigh_range = (100, 150)
    elif voltage_level == 110:
        neigh_range = (150, 250)
    elif voltage_level == 220:
        neigh_range = (250, 450)
    elif voltage_level == 330:
        neigh_range = (300, 400)
    elif voltage_level == 500:
        neigh_range = (350, 450)
    elif voltage_level == 750:
        neigh_range = (450, 500)
    elif voltage_level == 1000:
        neigh_range = (500, 600)
    else:
        raise Exception("电压等级输入错误！")
    try:
        openset_size = args.buffer
    except:
        print("请输入合适的搜索集大小！")
    try:
        precision = args.precision
    except:
        print("请输入搜索精确度！")
    if precision == 1:
        length_part = 5
        degree_delta = 90  # 20
    elif precision == 2:
        length_part = 10
        degree_delta = 90  # 40
    elif precision == 3:
        length_part = 10
        degree_delta = 90  # 40
    elif precision == 4:
        length_part = 5
        degree_delta = 45  # 40
    elif precision == 5:
        length_part = 10
        degree_delta = 45  # 80
    elif precision == 6:
        length_part = 20
        degree_delta = 45  # 160
    elif precision == 7:
        length_part = 20
        degree_delta = 30  # 240
    else:
        length_part = 5
        degree_delta = 90  # 20

    print("读取TIFF文件中...")
    try:
        tif = TIFF.open(args.gridMap, mode='r')  # 打開tiff文件進行讀取
    except:
        print("输入的路径有误！")
    im = tif.read_image()
    print("正在分析各类地块...")
    class4 = cv2.inRange(im, 3.9, 4.1)
    class1 = cv2.inRange(im, 4.9, 5.1) + cv2.inRange(im, 5.9, 6.1)
    class2 = cv2.inRange(im, 1.9, 2.1)
    class3 = cv2.inRange(im, 0.9, 1.1) + cv2.inRange(im, 2.9, 3.1)
    print("正在生成各类地块预览图并保存...")
    plt.figure(num='sketch', figsize=(16, 16))

    plt.subplot(2, 2, 1)  # 将窗口分为两行两列四个子图，则可显示四幅图片
    plt.title('class1')  # 第一幅图片标题
    plt.imshow(class1)  # 绘制第一幅图片

    plt.subplot(2, 2, 2)  # 将窗口分为两行两列四个子图，则可显示四幅图片
    plt.title('class2')  # 第一幅图片标题
    plt.imshow(class2)  # 绘制第一幅图片

    plt.subplot(2, 2, 3)  # 将窗口分为两行两列四个子图，则可显示四幅图片
    plt.title('class3')  # 第一幅图片标题
    plt.imshow(class3)  # 绘制第一幅图片

    plt.subplot(2, 2, 4)  # 将窗口分为两行两列四个子图，则可显示四幅图片
    plt.title('class4')  # 第一幅图片标题
    plt.imshow(class4)  # 绘制第一幅图片
    plt.savefig("output/{}/preview_landtype_tag{}.png".format(unique_tag, unique_tag))
    del class1, class2, class3, class4
    print("正在生成背景预览图...")
    plt.imsave("output/{}/background_tag{}.png".format(unique_tag, unique_tag), im)
    background = cv2.imread("output/{}/background_tag{}.png".format(unique_tag, unique_tag))
    background = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)
    np.save("output/{}/sketch_tag{}.npy".format(unique_tag,unique_tag), im)
    print("提取道路信息...")
    driver = ogr.GetDriverByName("ESRI Shapefile")
    filename = args.road
    dataSource = driver.Open(filename, 0)
    try:
        layer = dataSource.GetLayer(0)
    except:
        print("输入的道路文件有误！")
    roads = road_extract(layer)
    print("共有{}条道路".format(len(roads)))

    im.astype(int)

    # processes = []
    # for ver in range(6):
    #     processes.append(Process(target=run, args=(ver, start, end, neigh_range, im, background, openset_size, length_part, degree_delta, roads)))
    # for ver in range(6):
    #     processes[ver].start()
    # for ver in range(6):
    #     processes[ver].join()
    # print('Process will start.')
    # for ver in range(5):
    #     processes[ver].start()
    # for ver in range(5):
    #     processes[ver].join()
    # print('Process end.')
    print("开始跑程序...")
    count = 0
    ver_count = 0
    processes = []
    # pbar = tqdm(total=4)
    while True:
        manager = Manager()
        d = manager.dict()
        for ver in range(ver_count, ver_count + 5):
            p = Process(target=run, args=(
            ver, d, start, end, neigh_range, im, background, openset_size, length_part, degree_delta,
            roads))
            processes.append(p)
            p.start()
            print("载入进程...")
        for i in range(ver_count, ver_count + 5):
            processes[i].join()
        for result in d.values():
            count = count + result
            # if result==1:
                # pbar.update(1)
        ver_count = ver_count + 5
        # print(d.keys())
        # print("count大小：{}".format(count))
        if count > 4:
            # pbar.close()
            break
    print('Process end.')

    print("结束")

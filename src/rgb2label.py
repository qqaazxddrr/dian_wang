import numpy as np
from scipy import misc
from PIL import Image
from libtiff import TIFF
import numpy as np
import cv2
from matplotlib import pyplot as plt
import time
import sys


fp = sys.argv[1]
tif = TIFF.open(fp, mode='r')  # 打開tiff文件進行讀取
im = tif.read_image()  # 讀取圖像並作爲numpy數組返回
row, col = im.shape
sketch = np.zeros([row, col])

# 将原始二维三通道RGB图像转换为带标签的二维单通道图像
time1 = time.time()
for i in range(row):
    for j in range(col):
        tmp = im[i, j].tolist()
        if tmp == [0, 0, 255]:
            sketch[i, j] = 2
            continue
        if tmp == [0, 255, 0]:
            sketch[i, j] = 1
            continue
        if tmp == [85, 107, 47]:
            sketch[i, j] = 1
            continue
        if tmp == [210, 105, 30]:
            sketch[i, j] = 4
            continue
        if tmp == [255, 0, 0]:
            sketch[i, j] = 3
            continue
        if tmp == [255, 255, 0]:
            sketch[i, j] = 3
            continue
    if i % 1000 == 0:
        time2 = time.time()
        print("转换进度 row:{},time:{}".format(i, time2 - time1))
        time1 = time2
    np.save("../res/sketch_label.npy",sketch)



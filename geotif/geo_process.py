import numpy as np
from scipy import misc

import numpy as np
import cv2
from matplotlib import pyplot as plt
import time

fp = 'sketch.npy'

sketch = np.load(fp)
new_row = round(sketch.shape[0] / 3)
new_col = round(sketch.shape[1] / 3)

sampled_sketch = np.zeros([new_row, new_col])

sample_core = np.array([[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2], [2, 0], [2, 1], [2, 2]])

time1 = time.time()
for i in range(new_row):
    for j in range(new_col):
        anchor = np.array([i, j])
        type_list = []
        for unit in sample_core:
            index = anchor + unit
            type_list.append(sketch[index[0], index[1]])
        if 4 in type_list:
            sampled_sketch[anchor[0], anchor[1]] = 4
        elif 3 in type_list:
            sampled_sketch[anchor[0], anchor[1]] = 3
        elif 2 in type_list:
            sampled_sketch[anchor[0], anchor[1]] = 2
        else:
            sampled_sketch[anchor[0], anchor[1]] = 1
        type_list = []
    if i % 1000 == 0:
        time2 = time.time()
        print("row:{},time:{}".format(i, time2 - time1))
        time1 = time2

cv2.imwrite("sampled_sketch_a.png", sampled_sketch)
#!usr/bin/python3

import numpy as np

mtx = [[1108.7654256452881, 0.0, 640.5],
       [0.0, 1108.7654256452881, 360.5],
       [0.0, 0.0, 1.0]]
dist = [0.0, 0.0, 0.0, 0.0, 0.0]

np.save("mtx_sim", mtx)
np.save("dist_sim", dist)

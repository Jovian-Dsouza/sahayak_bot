#!/usr/bin/env python

# height: 480
# width: 640
# distortion_model: "plumb_bob"
# D: [0.0, 0.0, 0.0, 0.0, 0.0]
# K: [554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
# P 3x4 matrix
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w

import numpy as np

point_3d = np.array([(0.0264, -0.0923, 0.5735, 1.0)])
P = np.array([[554.3827128226441, 0.0, 320.5, -38.80678989758509],\
            [0.0, 554.3827128226441, 240.5, 0.0],\
            [0.0, 0.0, 1.0, 0.0]])
point_2d = np.matmul(P, point_3d.T)

x = int(np.asscalar(point_2d[0] / point_2d[2]))
y = int(np.asscalar(point_2d[1] / point_2d[2]))

print(point_2d)
print(x ,y)
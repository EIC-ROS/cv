#!/usr/bin/env python

import numpy as np

R = np.array([[0.7071,-0.7071,0],
             [0.7071,0.7071,0],
             [0,0,1]])

x=1-0.025/(2**0.5)
dx = 0.05
dy = 0.05

position_A = np.array([x,0.025/(2**0.5),0])
position_B = np.array([x,-0.025/(2**0.5),0])

dimension_X = np.array([dx,0,0])
dimension_NY = np.array([0,-dy,0])

print(np.matmul(dimension_X,R))
print(np.matmul(dimension_NY,R))

center1 = position_A + np.matmul(dimension_X,R)
center2 = position_B + np.matmul(dimension_NY,R)

print(center1)
print(center2)
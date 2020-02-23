import pylab as plt
from casadi import *

# def CasADi_optim_weigths():

# This code is implemented in the same philosophy as the main paper.
# Path has no connection with a vehicle model and is build up with quintic splines.
# 200 spline control points are calculated --> paths are refined tot meer dan 1000 samples

N = 200

opti = Opti()

#     Definition of variables
px = opti.variable(N)
py = opti.variable(N)
vx = opti.variable(N)
vy = opti.variable(N)
ax = opti.variable(N)
ay = opti.variable(N)

T = opti.variable()  # Time [s]
dt = T/(N-1)
jx = (ax[0:-1]-ax[1:])/dt
jy = (ay[0:-1]-ay[1:])/dt










import os
from typing import List

import numpy as np
import casadi.casadi as cs


### Define the dynamics here ###

def dynamics_ct(ts, x, u): # δ(state) per ts
    dx = ts * (u[0]*cs.cos(x[2]))
    dy = ts * (u[0]*cs.sin(x[2]))
    dtheta = ts * u[1]
    return cs.vertcat(dx, dy, dtheta)

def dynamics_rk1(ts, x, u): # discretized via Runge-Kutta 1 (Euler method)
    return x + dynamics_ct(ts, x, u)

def dynamics_rk4(ts, x, u): # discretized via Runge-Kutta 4
    k1 = dynamics_ct(ts, x, u)
    k2 = dynamics_ct(ts, x + 0.5*k1, u)
    k3 = dynamics_ct(ts, x + 0.5*k2, u)
    k4 = dynamics_ct(ts, x + k3, u)
    x_next = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    return x_next

### Define the meta cost functions here ###
def dist_point_to_lineseg(point:cs.SX, line_segment:List[cs.SX]):
    '''
    Arguments:
        point        <cs.SX>          - column vector
        line_segment <list of cs.SX>  - two points
    Return:
        distance <value>
    Comments:
        [Ref] https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment
    '''
    (p, s1, s2) = (point, line_segment[0], line_segment[1])
    s2s1 = s2-s1 # line segment
    t_hat = cs.dot(p-s1,s2s1)/(s2s1[0]**2+s2s1[1]**2+1e-16)
    t_star = cs.fmin(cs.fmax(t_hat,0.0),1.0) # limit t
    temp_vec = s1 + t_star*s2s1 - p # vector pointing to closest point
    distance = np.sqrt(temp_vec[0]**2+temp_vec[1]**2)
    return distance

def cost_cte(point:cs.SX, line_segments:List[cs.SX], weight:float=1):
    '''
    Description:
        [Cost] Cross-track-error, penalizes on the deviation from the reference path.
    Arguments:
        point         <cs.SX>         - column vector
        line_segments <list of cs.SX> - from the the start point to the end point
    Comments:
        The 'line_segments' contains segments which are end-to-end.
    '''
    distances_sqrt = cs.SX.ones(1)
    for i in range(len(line_segments)-1):
        distance = dist_point_to_lineseg(point, [line_segments[i], line_segments[i+1]])
        distances_sqrt = cs.horzcat(distances_sqrt, distance**2)
    cost = cs.mmin(distances_sqrt[1:]) * weight
    return cost
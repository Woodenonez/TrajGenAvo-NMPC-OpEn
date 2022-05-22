import os
from typing import List, Union

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
def dist_to_lineseg(point:Union[cs.SX, cs.DM], line_segment:List[Union[cs.SX, cs.DM]]):
    '''
    Arguments:
        line_segment - list of two points
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

def inside_ellipse(point:Union[List, cs.SX, cs.DM], ellipse_param:List[Union[cs.SX, cs.DM]]):
    # Center: (cx, cy), semi-axes: (rx, ry), rotation angle to x axis: ang
    # If inside, return positive value, else return negative value
    x, y = point[0], point[1]
    cx, cy, rx, ry, ang = ellipse_param[0], ellipse_param[1], ellipse_param[2], ellipse_param[3], ellipse_param[4]
    indicator = 1 - ((x-cx)*cs.cos(ang)+(y-cy)*cs.sin(ang))**2/(rx**2) - ((x-cx)*cs.sin(ang)-(y-cy)*cs.cos(ang))**2/(ry)**2
    return indicator

def cost_cte(point:Union[cs.SX, cs.DM], line_segments:List[Union[cs.SX, cs.DM]], weight:float=1):
    '''
    Description:
        [Cost] Cross-track-error, penalizes on the deviation from the reference path.
    Arguments:
        line_segments - from the the start point to the end point
    Comments:
        The 'line_segments' contains segments which are end-to-end.
    '''
    distances_sqrt = cs.SX.ones(1)
    for i in range(len(line_segments)-1):
        distance = dist_to_lineseg(point, [line_segments[i], line_segments[i+1]])
        distances_sqrt = cs.horzcat(distances_sqrt, distance**2)
    cost = cs.mmin(distances_sqrt[1:]) * weight
    return cost

def cost_inside_ellipse(point:Union[List, cs.SX, cs.DM], ellipse_param:List[Union[cs.SX, cs.DM]], narrowness=5, weight:float=1):
    alpha = ellipse_param[5]
    indicator = inside_ellipse(point, ellipse_param) # indicator<0, if outside ellipse
    cost = cs.sum1( weight / (1+cs.exp(-narrowness*indicator-4)) * alpha )
    return cost

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    point = cs.DM([1,0])
    ellipse = cs.DM([0,0,1,1,0])
    indicator = inside_ellipse(point, ellipse)

    print(indicator)

    weight = 1
    narrowness = 5
    x = np.linspace(-5, 5, 100)
    y = weight / (1+np.exp(-narrowness*x-4))

    plt.plot(x, y)
    plt.show()

    import sys
    sys.exit(0)
    
    C  = np.array([2, 1]).reshape(-1,1)
    U0 = np.array([1, 1]).reshape(-1,1)
    U1 = np.array([-1, 1]).reshape(-1,1)
    e0, e1 = 2, 1
    a0 = np.arange(start=-e0, stop=e0, step=0.01)
    a1 = np.concatenate([np.sqrt(1-(a0/e0)**2)*e1, -np.sqrt(1-(a0/e0)**2)*e1])
    a0 = np.tile(a0, 2)

    P = C + a0*U0 + a1*U1

    plt.plot(P[0,:], P[1,:], '.')
    plt.plot(C[0], C[1], 'rx')
    plt.axis('equal')
    plt.show()
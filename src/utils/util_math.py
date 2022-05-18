import os
import math

import numpy as np

def dist_point_to_lineseg(point:np.ndarray, line_segment:np.ndarray) -> float:
    '''
    Arguments:
        point        <ndarray> - column vector
        line_segment <ndarray> - two end points, column vector
    Return:
        distance <value>
    Comments:
        [Ref] https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment
    '''
    (p, s1, s2) = (point, line_segment[:, [0]], line_segment[:, [1]])
    s2s1 = s2-s1 # line segment
    t_hat = np.dot((p-s1).transpose(), s2s1)/(s2s1[0]**2+s2s1[1]**2+1e-16)
    t_star = min(max(t_hat,0.0),1.0) # limit t
    temp_vec = s1 + t_star*s2s1 - p # vector pointing to closest point
    distance = np.sqrt(temp_vec[0]**2+temp_vec[1]**2)
    return distance

if __name__ == '__main__':
    pass
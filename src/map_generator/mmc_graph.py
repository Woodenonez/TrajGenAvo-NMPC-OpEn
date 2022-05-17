import sys, math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.collections import PatchCollection

'''
File info:
    Name    - [mmc_graph]
    Exe     - [Yes]
File description:
    This contains the map of the synthetic multimodal crossing (mmc) data.
File content:
    Graph     <class> - Define the map.
Comments:
    None
'''

class Graph:
    '''
    Description:
        Define a map.
    Arguments:
        None
    Attributes:
        boundary_coordinates <list of tuples> - Each tuple is a vectex of the boundary polygon.
                                              - Defined in counter-clockwise ordering.
        obstacle_list        <list of lists>  - Each sub-list represents an obstacle in the form of a list of tuples.
                                              - Defined in clockwise ordering.
    Functions
        plot_map <vis> - Visualization of the map. Plot directly.
    '''
    def __init__(self):
        self.boundary_coordinates = [(0, 0), (16, 0), (16, 10), (0, 10)]  # in counter-clockwise ordering
        self.obstacle_list = [[(0, 1.5), (0, 1.6), (9, 1.6), (9, 1.5)],
                              [(0, 8.4), (0, 8.5), (9, 8.5), (9, 8.4)],
                              [(11, 1.5), (11, 1.6), (16, 1.6), (16, 1.5)],
                              [(11, 8.4), (11, 8.5), (16, 8.5), (16, 8.4)]] # in clock-wise ordering
        self.crossing_area = [(9, 1.5), (11, 1.5), (11, 8.5), (9, 8.5)]

    def plot_map(self, start=None, end=None):
        boundary = self.boundary_coordinates + [self.boundary_coordinates[0]]
        boundary = np.array(boundary)
        _, ax = plt.subplots()
        ax.plot(boundary[:,0], boundary[:,1], 'k')
        ax.plot([0, 16], [5, 5], c='orange', linestyle='--')
        ax.fill_between([0, 16], [1.6, 1.6], [8.4, 8.4], color='lightgray')
        crossing = patches.Polygon(self.crossing_area, hatch='-', fc='white', ec='gray')
        ax.add_patch(crossing)
        for obs in self.obstacle_list:
            obs_edge = obs + [obs[0]]
            xs, ys = zip(*obs_edge)
            plt.plot(xs,ys,'k')

            obs = np.array(obs)
            poly = patches.Polygon(obs, color='k')
            ax.add_patch(poly)
        if start is not None:
            plt.plot(self.start[0], self.start[1], 'b*')
        if end is not None:
            plt.plot(self.end[0], self.end[1], 'r*')
        plt.show()

if __name__ == '__main__':
    start = (0, 3.3, math.radians(0))
    end = (16, 3.3, math.radians(0))
    graph = Graph(start, end)
    graph.plot_map()
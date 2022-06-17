import os, sys
import math
import matplotlib.pyplot as plt 

import extremitypathfinder.extremitypathfinder as epf
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment, draw_prepared_map

'''
File info:
    Name    - [visibility]
    Date    - [Jan. 01, 2021] -> [Aug. 20, 2021]
    Exe     - [Yes]
File description:
    [Path advisor] Generate the reference path via the visibility graph and A* algorithm.
'''

### Helper functions
def plot_line_segments(line_segments, ax, color:str):
    for i in range(len(line_segments)-1):
        ax.plot([line_segments[i][0], line_segments[i+1][0]], [line_segments[i][1], line_segments[i+1][1]], color)
def plot_polygon(polygon, ax, color='b', label=None):
    plot_line_segments(polygon, ax, color)
    ax.plot([polygon[-1][0], polygon[0][0]], [polygon[-1][1], polygon[0][1]], color, label=label)

def plot_boundaries(boundary_coordinates, ax, color='g', label=None):
    plot_polygon(boundary_coordinates, ax, color=color, label=label)
def plot_obstacles(obstacle_list, ax, color='b', label=None):
    for obs in obstacle_list[:-1]:
        plot_polygon(obs, ax, color=color, label=None)
    plot_polygon(obstacle_list[-1], ax, color=color, label=label)
def plot_path(path, ax, color='k--'):
    plot_line_segments(path, ax, color)
def plot_vertices(vertices, radius,  ax):
    for vert in vertices:
        c = plt.Circle(vert, radius, color = 'r', alpha = 0.7)
        ax.add_artist(c)


class VisibilityPathFinder:
    '''
    Description:
        Generate the reference path via the visibility graph and A* algorithm.
    Arguments:
        inflate_margin <value> - The safe margin used to extend the obstacle shape and shrink the boundary.
        save_plot_path <str>   - The path to save the generated path result (if None, no saving).
    Attributes:
        env  <object>         - The environment object of solving the visibility graph.
        path <list of tuples> - The shortest path from the visibility graph.
        vert <list of tuples> - The closest vertex (of obstacles and boundary) of every path node.
    Functions
        prepare                 <pre> - Prepare the visibility graph including preprocess the map.
        preprocess_obstacle     <pre> - Preprocess an obstacle by extend/shrink its shape.
        preprocess_obstacles    <pre> - Preprocess obstacles by extend/shrink their shape.
        get_ref_path            <get> - Get the (shortest) refenence path.
        get_closest_vert        <get> - Find the closest point to the targeted point from a list of points.
        find_original_vertices  <get> - Find the orginal vertices (of obstacles and boundary) corresponding to the path nodes.
        find_closest_vertices   <get> - Find a set of closest vertices to a given point/position.
        plot_map                <vis> - Plot the map.
    '''
    def __init__(self, graph_map):
        self.print_name = '[Path]'
        self.graph = graph_map

        self.__prepare()

    def __prepare(self):
        self.env = PolygonEnvironment()
        self.env.store(self.graph.processed_boundary_coords, self.graph.processed_obstacle_list) # pass obstacles and boundary to environment
        self.env.prepare() # prepare the visibility graph 

    def get_ref_path_node(self, start_pos, end_pos):
        '''
        Description:
            Generate the initially guessed path based on obstacles and boundaries specified during preparation.
        Arguments:
            start_pos <tuple> - The x,y coordinates.
            end_pos   <tuple> - The x,y coordinates.
        Return:
            Path     <list of tuples> - List of coordinates of the inital path nodes, lying on extremities of the padded obstacles. 
            Vertices <list of tuples> - List of the vertices on the original (unpadded) obstacles corresponding to the vertices in path.
        '''
        self.path, dist = self.env.find_shortest_path(start_pos[:2], end_pos[:2]) # 'dist' are distances of every segments.
        self.vertices   = self.find_original_vertices(self.path)
        return self.path, self.vertices
    
    def get_closest_vert(self, vert, point_list):
        distances = [math.hypot(vert[0]-x[0], vert[1]-x[1]) for x in point_list]
        best_idx = distances.index(min(distances))
        return point_list[best_idx], best_idx
        
    def find_original_vertices(self, path):
        vertices = []
        if not (len(path) > 2):
           print(f'{self.print_name} Path is only one line. No vertices to find.') 
           return vertices
        all_vert = self.graph.obstacle_list + [self.graph.boundary_coords]
        all_vert = [item for sublist in all_vert for item in sublist]
        for vert in path[1:-1]: # dont use start and final positions as contstraints
            closest_vert, _ = self.get_closest_vert(vert, all_vert)
            vertices.append(closest_vert)
        return vertices
    
    def plot_map(self, ax, vert_radius=1):
        plot_boundaries(self.graph.boundary_coords,           ax=ax, color='k', label='Original Boundary')
        plot_boundaries(self.graph.processed_boundary_coords, ax=ax, color='g', label='Padded Boundary')
        plot_obstacles( self.graph.obstacle_list,                  ax=ax, color='r', label='Original Obstacles')
        plot_obstacles( self.graph.processed_obstacle_list,        ax=ax, color='y', label='Padded Obstacles')
        
        plot_path(self.path, ax=ax, color='--k')
        plot_vertices(self.vertices, radius=vert_radius, ax=ax)


if __name__ == '__main__':

    try:
        from map_generator.test_graphs import Graph
    except:
        sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
        from map_generator.test_graphs import Graph
        
    ### Test graph
    graph = Graph(index=11) # right now 0~11

    ### FTD graph
    # start = (9.0, 1.0, math.radians(0))
    # end   = (5.5, 9.0, math.radians(0))
    # graph = Graph(start, end)

    path_ad = PathAdvisor(inflate_margin=0.5, save_plot_path=None)
    path_ad.prepare(graph)
    path, vertices = path_ad.get_ref_path(graph.start[:2], graph.end[:2])

    fig, ax = plt.subplots()
    path_ad.plot_map(ax, vert_radius=0.5)
    plt.legend()
    plt.axis('equal')
    plt.show()
    
    
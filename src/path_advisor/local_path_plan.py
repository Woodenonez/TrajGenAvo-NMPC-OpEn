import os, sys
import math

from typing import List

### Import specific path-finding algorithm here
from path_advisor.visibility import VisibilityPathFinder


'''
File info:
    None
File description:
    (What does this file do?)
File content (important ones):
    ClassA      <class> - (Basic usage).
    ClassB      <class> - (Basic usage).
    function_A  <func>  - (Basic usage).
    function_B  <func>  - (Basic usage).
Comments:
    (Things worthy of attention.)
'''
    

class LocalPathPlanner:
    def __init__(self, graph_map):
        self.path_planner = VisibilityPathFinder(graph_map=graph_map)

    def get_ref_path_node(self, start, end):
        path_node = self.path_planner.get_ref_path_node(start, end)
        if isinstance(path_node, tuple):
            path_node = path_node[0] # if there are multiple outputs, then the first one must be the path
        return path_node

    @staticmethod
    def get_detailed_path(time_step:float, reference_speed:float, current_position:tuple, node_list:List[tuple]) -> List[tuple]:
        '''
        Description:
            Generate the reference trajectory from the reference path.
        Arguments:
            current_position <tuple>         - The x,y coordinates. \\
            node_list        <list of tuple> - A list of reference path nodes.
        Return:
            ref_path <list> - List of x, y coordinates and the heading angles of the detailed reference path.
        '''
        x, y = current_position
        x_next, y_next = node_list[0]
        v_ref = reference_speed
        ts = time_step
        
        ref_path = []
        idx = 0
        traveling = True
        while(traveling):# for n in range(N):
            while(True):
                dist_to_next = math.hypot(x_next-x, y_next-y)
                if dist_to_next < 1e-9:
                    idx = idx+1
                    x_next, y_next = node_list[idx]
                    break
                x_dir = (x_next-x) / dist_to_next
                y_dir = (y_next-y) / dist_to_next
                eta = dist_to_next/v_ref # estimated time of arrival
                if eta > ts: # move to the target node for t
                    x, y = x+x_dir*v_ref*ts, y+y_dir*v_ref*ts
                    break # to append the position
                else: # move to the target node then set a new target
                    x,y = x+x_dir*v_ref*eta, y+y_dir*v_ref*eta
                    idx = idx+1
                    if idx > len(node_list)-1 :
                        traveling = False
                        break
                    else:
                        x_next, y_next = node_list[idx] # set a new target node
            if not dist_to_next < 1e-9:
                ref_path.append((x, y, math.atan2(y_dir,x_dir)))
            
        return ref_path
        

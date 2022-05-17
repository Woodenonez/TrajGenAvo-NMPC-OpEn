import os, sys
import math
import matplotlib.pyplot as plt 

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

    def get_ref_path(self, start, end):
        result = self.path_planner.get_ref_path(start, end)
        if isinstance(result, tuple):
            result = result[0]
        return result

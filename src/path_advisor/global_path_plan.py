
from typing import Union, Tuple, List

'''
This is a dummy class
'''

class GloablPathPlanner:
    def __init__(self, external_path:List[Tuple]=None) -> None:
        self.__scheduled_path(external_path)

    def __scheduled_path(self, external_path:List[Tuple]=None):
        if external_path is None:
            self.global_path = [(), ()]
        else:
            self.global_path = external_path
        self.start = None
        self.next_goal  = self.global_path[0]
        self.final_goal = self.global_path[-1]

    def set_start_point(self, start:Union[list,tuple]):
        self.global_path.insert(0, start)
        self.start = start
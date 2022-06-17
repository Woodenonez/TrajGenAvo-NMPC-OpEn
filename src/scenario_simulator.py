import math


class Simulator:
    def __init__(self, index, inflate_margin) -> None:
        self.idx = index
        self.inflate_margin = inflate_margin
        self.__intro()
        self.load_map_and_obstacles()

    def __intro(self):
        assert(self.idx in [0,1,2]),(f'Index {self.idx} not found!')
        print('='*30)
        print('Index 0 - Test cases.')
        print('Index 1 - Single object, crosswalk.')
        print('Index 2 - Multiple objects, road crossing.')
        print(f'[{self.idx}] is selected.')
        print('='*30)

    def load_map_and_obstacles(self, test_graph_index=11):
        if self.idx == 0:
            from map_generator.test_graphs import Graph
            from obstacle_scanner.test_dynamic_obstacles import ObstacleScanner
            self.graph = Graph(inflate_margin=self.inflate_margin, index=test_graph_index)
            self.scanner = ObstacleScanner(self.graph)
            self.start = self.graph.start
            self.waypoints = [self.graph.end]
        elif self.idx == 1:
            from map_generator.mmc_graph import Graph
            from obstacle_scanner.mmc_dynamic_obstacles import ObstacleScanner
            self.start = (0.5, 3.5, math.radians(0))
            self.waypoints = [(15.5, 3.5, math.radians(0))]
            self.graph = Graph(inflate_margin=self.inflate_margin)
            self.scanner = ObstacleScanner()
        elif self.idx == 2:
            from map_generator.mmc_graph2 import Graph
            from obstacle_scanner.mmc_dynamic_obstacles2 import ObstacleScanner
            self.start = (7, 0.5, math.radians(90))
            self.waypoints = [(7, 11.5, math.radians(90)), (7, 15.5, math.radians(90))]
            self.graph = Graph(inflate_margin=self.inflate_margin)
            self.scanner = ObstacleScanner()
        else:
            raise ModuleNotFoundError
        
        return self.graph, self.scanner

        
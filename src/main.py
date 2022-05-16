import os, math

import matplotlib.pyplot as plt

from map_generator.test_graphs import Graph # choose the correct file
from trajectory_generator import TrajectoryGenerator
from utils.config import Configurator
from pathlib import Path

'''
File info:
    Name    - [main]
    Date    - [Jan. 01, 2021] -> [Aug. 20, 2021]
    Ref     - [Trajectory generation for mobile robotsin a dynamic environment using nonlinear model predictive control, CASE2021]
            - [https://github.com/wljungbergh/mpc-trajectory-generator]
    Exe     - [Yes]
File description:
    The main file for running the trajectory planner
Comments:
                                                           V [MpcModule] V
    [MAP] --graph--> [PathAdvisor] --refernece path--> [TrajGenrator(Config)] <--dynamic obstacles-- [ObstacleScanner]
'''

### Customize
config_fn = 'default.yaml'
init_build = False
show_animation = True
save_animation = False
plot_prediction = True

start = (1, 1.0, math.radians(0))
end = (9.0, 1.0, math.radians(0))
### Choose a map to continue
graph = Graph(start, end, index=11) # for test, index=0~11

### Configure MPC
yaml_fp = os.path.join(Path(__file__).resolve().parents[1], 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()

### Start MPC
traj_gen = TrajectoryGenerator(config, build=init_build)

### Run MPC
start = list(graph.start)
end   = list(graph.end)
xx,xy,uv,uomega,tot_solver_time,overhead_times = traj_gen.run(graph, start, end)

### Plot results (press any key to continue if dynamic)
traj_gen.plot_results(xx,xy,uv,uomega, start, end, animation=show_animation, video=save_animation, plot_prediction=plot_prediction)

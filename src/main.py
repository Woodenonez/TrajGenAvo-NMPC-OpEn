import os
import math
from pathlib import Path

from utils.config import Configurator

from map_generator.mmc_graph import Graph # choose the correct file
from path_advisor.global_path_plan import GloablPathPlanner
from path_advisor.visibility import PathAdvisor
from trajectory_generator import TrajectoryGenerator

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

Branches:
    [main]: Using TCP/IP interface
    [direct_interface]: Using Python binding direct interface to Rust
    [new_demo]: Under construction
'''

### Customize
config_fn = 'default.yaml'
init_build = False
show_animation = True
save_animation = False
plot_prediction = True
start = (0.6, 3.3, math.radians(0))

### Load configuration
yaml_fp = os.path.join(Path(__file__).resolve().parents[1], 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()

### Load map
graph = Graph()

### Global path
gpp = GloablPathPlanner(external_path=[(15.4, 3.3, math.radians(0))])
gpp.set_start_point(start)

start = gpp.start
end   = gpp.final_goal # special case

### Local path
lpp = PathAdvisor(inflate_margin=config.vehicle_width, save_plot_path=None)
lpp.prepare(graph)
path, _ = lpp.get_ref_path(start, end)

### Start MPC
traj_gen = TrajectoryGenerator(config, build=init_build)
### Run MPC
start = list(gpp.start)
end   = list(gpp.final_goal)
xx,xy,uv,uomega,tot_solver_time,overhead_times = traj_gen.run(path, start, end)

### Plot results (press any key to continue in dynamic mode if stuck)
traj_gen.plot_results(xx,xy,uv,uomega, start, end, animation=show_animation, video=save_animation, plot_prediction=plot_prediction)

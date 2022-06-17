import os
import math
from pathlib import Path

from utils.config import Configurator

from path_advisor.global_path_plan import GloablPathPlanner
from path_advisor.local_path_plan import LocalPathPlanner
from trajectory_generator import TrajectoryGenerator

from scenario_simulator import Simulator

from utils import util_plot

'''
File info:
    Date    - [May. 01, 2022] -> [??. ??, 2022]
    Ref     - [Trajectory generation for mobile robotsin a dynamic environment using nonlinear model predictive control, CASE2021]
            - [https://github.com/wljungbergh/mpc-trajectory-generator]
    Exe     - [Yes]
File description:
    The main file for running the trajectory planner
Comments:
    GPP - GlobalPathPlanner; 
    LPP - LocalPathPlanner; 
    TG  - TrajGenrator; 
    OS  - ObstacleScanner; 
    MPC - MpcModule
                                                                                        V [MPC] V
    [GPP] --global path & static obstacles--> [LPP] --refernece path & tunnel width--> [TG(Config)] <--dynamic obstacles-- [OS]
    GPP is assumed given, which takes info of the "map" and "static obstacles" and controller by a [Scheduler].
Branches:
    [main]: Using TCP/IP interface
    [direct_interface]: Using Python binding direct interface to Rust
    [new_demo]: Under construction
'''

### Customize
config_fn = 'default.yaml'
init_build = False
plot_in_loop = True
show_animation = False
save_animation = False
case_index = 2

### Load configuration
yaml_fp = os.path.join(Path(__file__).resolve().parents[1], 'configs', config_fn)
configurator = Configurator(yaml_fp)
config = configurator.configurate()

### Load simulator
sim = Simulator(index=case_index, inflate_margin=config.vehicle_width)

### Global path
gpp = GloablPathPlanner(external_path=sim.waypoints)
gpp.set_start_point(sim.start) # must set the start point

start = gpp.start
end   = gpp.final_goal # special case, otherwise iterate waypoints

### Local path
lpp = LocalPathPlanner(sim.graph)
path_node = lpp.get_ref_path_node(start, end)
ref_path  = lpp.get_detailed_path(config.ts, config.high_speed * config.lin_vel_max, start, path_node)

### Start & run MPC
traj_gen = TrajectoryGenerator(config, obstacle_info=sim.scanner, graph_map=sim.graph, build=init_build, verbose=True)
xx, xy, uv, uomega, cost_list = traj_gen.run(ref_path, list(start), list(end), plot_in_loop=plot_in_loop)

### Plot results (press any key to continue in dynamic mode if stuck)
util_plot.plot_results(sim.graph, config.ts, xx, xy, uv, uomega, cost_list, start, end, animation=show_animation, scanner=sim.scanner, video=save_animation)

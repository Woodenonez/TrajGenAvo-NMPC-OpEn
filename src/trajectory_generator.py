import os, sys
import math
import itertools

import numpy as np
import opengen as og

from obstacle_scanner.mmc_dynamic_obstacles import ObstacleScanner ### choose the right scanner
from mpc.mpc_generator import MpcModule

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import matplotlib.patches as patches
from map_generator.mmc_graph import Graph # choose the correct file

'''
File info:
    Ref     - [Trajectory generation for mobile robotsin a dynamic environment using nonlinear model predictive control, CASE2021]
            - [https://github.com/wljungbergh/mpc-trajectory-generator]
    Exe     - [No]
File description:
    Generate the trajectory by generating/using the defined MPC solver. 
File content:
    TrajectoryGenerator <class> - Build and run the MPC problem. Calculate the trajectory step by step.
Comments:
                                                                                      V [MPC] V
    [GPP] --global path & static obstacles--> [LPP] --refernece path & tube width--> [TG(Config)] <--dynamic obstacles-- [OS]
'''

MAX_RUNNING_TIME_MS = 100_000 # ms

class TrajectoryGenerator:
    '''
    Description:
        Generate a smooth trajectory based on the reference path and obstacle information.
        Use a configuration specified by 'utils/config'
    Arguments:
        config  <dotdict> - A dictionary in the dot form contains all information/parameters needed.
        build   <bool>    - If true, build the MPC module.
        verbose <bool>    - If true, show verbose.
    Attributes:
        __prtname     <str>     - The name to print while running this class.
        config        <dotdict> - As above mentioned.
        scanner       <object>  - The obstacle scanner offering the dynamic obstacle info.
        mpc_generator <object>  - The MPC module.
    Functions
        run <run>  - Run.
    Comments:
        Have fun but may need to modify the dynamic obstacle part (search NOTE).
    '''
    def __init__(self, config, build=False, verbose=False):
        self.__prtname = '[Traj]'
        self.config = config
        self.vb = verbose
        
        self.scanner       = ObstacleScanner()
        self.mpc_generator = MpcModule(self.config)

        if build:
            self.mpc_generator.build()
        
        self.import_solver()

    def import_solver(self, root_dir=''):
        sys.path.append(os.path.join(root_dir, self.config.build_directory, self.config.optimizer_name))
        built_solver = __import__(self.config.optimizer_name)
        self.solver = built_solver.solver()

    def load_tunning_parameter(self, robot_flag=None):
        nparams = 10
        if robot_flag is None:
            parameter_list = [self.config.qpos, self.config.qvel, self.config.qtheta, self.config.lin_vel_penalty, self.config.ang_vel_penalty,
                              self.config.qpN, self.config.qthetaN, self.config.qcte, self.config.lin_acc_penalty, self.config.ang_acc_penalty]
        elif robot_flag=='aligning':
            parameter_list = [0.0] * nparams
            parameter_list[2] = 100
        else:
            raise ModuleNotFoundError(f'No state called {robot_flag}.')
        return parameter_list

    def termination_condition(self, states, final_goal, system_input) -> bool:
        if np.allclose(states[-3:-1], final_goal[0:2], atol=0.05, rtol=0) and abs(system_input[-2]) < 0.05:
            terminated = True
            print(f"{self.__prtname} MPC solution found.")
        else:
            terminated = False
        return terminated

    def run(self, ref_path:list, start:list, end:list, plot_in_loop=False):
        '''
        Description:
            Run the trajectory planner.
        Arguments:
            ref_path  <list of tuple> - Reference path
            start     <list> - The start state [x, y, theta]
            end       <list> - The end state [x, y, theta]
        Return:
            xx             <list> - List of x coordinate.
            xy             <list> - List of y coordinate.
            uv             <list> - List of velocity input.
            uomega         <list> - List of angular velocity input.
        '''

        x_ref, y_ref, theta_ref = np.array(ref_path)[:,0].tolist(), np.array(ref_path)[:,1].tolist(), np.array(ref_path)[:,2].tolist()

        ### Prepare for the loop computing ###
        ts                  = self.config.ts # sampling time
        N_hor               = self.config.N_hor # frequently used: control/prediction horizon
        params_per_dyn_obs  = N_hor*self.config.ndynobs
        base_speed          = self.config.lin_vel_max*self.config.high_speed
        
        system_input = [] # initalize list with selected system inputs/velocities
        states = start.copy() # initialiize states as starting state
        ref_points = [(x, y) for x, y in zip(x_ref, y_ref)]

        # Initialize lists
        refs = [0.0] * (N_hor * self.config.ns)
        dyn_constraints = [0.0] * self.config.Ndynobs * self.config.ndynobs*N_hor
        # Avoid dividing by zero in MPC solver by init x radius and y radius to 1
        dyn_constraints[2::self.config.ndynobs] = [1.0] * self.config.Ndynobs*N_hor
        dyn_constraints[3::self.config.ndynobs] = [1.0] * self.config.Ndynobs*N_hor

        ### Start the loop ###
        kt = 0  # time step, start from 0 (kt*ts is the actual time)
        idx = 0 # index of the current reference trajectory point
        terminated = False
        establish_heading = False
        cost_list = []

        ### Plot in loop XXX
        if plot_in_loop:
            fig = plt.figure(constrained_layout=True)
            gs = GridSpec(3, 4, figure=fig)

            vel_ax = fig.add_subplot(gs[0, :2])
            vel_ax.set_xlabel('Time [s]')
            vel_ax.set_ylabel('Velocity [m/s]')
            vel_ax.grid('on')

            omega_ax = fig.add_subplot(gs[1, :2])
            omega_ax.set_xlabel('Time [s]')
            omega_ax.set_ylabel('Angular velocity [rad/s]')
            omega_ax.grid('on')

            cost_ax = fig.add_subplot(gs[2, :2])
            cost_ax.set_xlabel('Time [s]')
            cost_ax.set_ylabel('Cost')
            cost_ax.grid('on')

            path_ax = fig.add_subplot(gs[:, 2:])
            graph = Graph(self.config.vehicle_width)
            path_ax.plot(start[0], start[1], marker='*', color='g', markersize=15, label='Start')
            path_ax.plot(end[0], end[1], marker='*', color='r', markersize=15, label='End')
            path_ax.arrow(start[0], start[1], math.cos(start[2]), math.sin(start[2]), head_width=0.05, head_length=0.1, fc='k', ec='k')
            path_ax.set_xlabel('X [m]', fontsize=15)
            path_ax.set_ylabel('Y [m]', fontsize=15)
            path_ax.axis('equal')
        ### Plot in loop

        while (not terminated) and kt < MAX_RUNNING_TIME_MS/1000/ts:
            x_cur = states[-self.config.ns:] # set current state as initial state for solver

            ### full_obstacle_list = [[(x, y, rx ,ry, angle, alpha),(),...],[(),(),...]] each sub-list is a mode/obstacle
            full_obstacle_list = self.scanner.get_full_obstacle_list(current_time=(kt*ts), horizon=N_hor, ts=ts)
            for i, dyn_obstacle in enumerate(full_obstacle_list):
                dyn_constraints[i*params_per_dyn_obs:(i+1)*params_per_dyn_obs] = list(itertools.chain(*dyn_obstacle))

            ### XXX
            # if kt == 0: # NOTE May vary for different types of obstacles
            #     full_obstacle_list = self.scanner.get_full_obstacle_list(current_time=(kt*ts), horizon=N_hor, ts=ts)
            #     for i, dyn_obstacle in enumerate(full_obstacle_list):
            #         dyn_constraints[i*params_per_dyn_obs:(i+1)*params_per_dyn_obs] = list(itertools.chain(*dyn_obstacle))
            # else: # Rotate list to the left
            #     dyn_constraints = dyn_constraints[self.config.ndynobs*self.config.num_steps_taken:] + \
            #                         dyn_constraints[:self.config.ndynobs*self.config.num_steps_taken]
            #     current_time = (kt+N_hor-self.config.num_steps_taken)*self.config.ts
            #     full_obstacle_list = self.scanner.get_full_obstacle_list(current_time=current_time, horizon=self.config.num_steps_taken, ts=ts)
            #     for i, dyn_obstacle in enumerate(full_obstacle_list):
            #         # Update last num_steps taken dynobs positions
            #         dyn_constraints[(i+1)*params_per_dyn_obs-self.config.ndynobs*self.config.num_steps_taken:(i+1)*params_per_dyn_obs] = list(
            #             itertools.chain(*dyn_obstacle))
            ### XXX

            # print(len(full_obstacle_list), len(full_obstacle_list[0]), len(full_obstacle_list[1]))
            # print(dyn_constraints)


            ### Get reference states ###
            lb_idx = max(0, idx-1*self.config.num_steps_taken)                  # reduce search space for closest reference point
            ub_idx = min(len(ref_points), idx+5*self.config.num_steps_taken)    # reduce search space for closest reference point

            distances = [math.hypot(x_cur[0]-x[0], x_cur[1]-x[1]) for x in ref_points[lb_idx:ub_idx]]
            idx = distances.index(min(distances))

            idx += lb_idx  # idx in orignal reference trajectory list
            if (idx+N_hor >= len(x_ref)):
                x_finish = end
                tmpx = x_ref[idx:]      + [end[0]]*(N_hor-(len(x_ref)-idx))
                tmpy = y_ref[idx:]      + [end[1]]*(N_hor-(len(y_ref)-idx))
                tmpt = theta_ref[idx:]  + [end[2]]*(N_hor-(len(theta_ref)-idx))
            else:
                x_finish = [x_ref[idx+N_hor], y_ref[idx+N_hor], theta_ref[idx+N_hor]]
                tmpx = x_ref[idx:idx+N_hor]
                tmpy = y_ref[idx:idx+N_hor]
                tmpt = theta_ref[idx:idx+N_hor]
            refs[0::self.config.ns] = tmpx
            refs[1::self.config.ns] = tmpy
            refs[2::self.config.ns] = tmpt

            ### Get reference velocities ###
            dist_to_goal = math.hypot(states[-3]-end[0], states[-2]-end[1])
            if dist_to_goal >= base_speed*self.config.N_hor*self.config.ts:
                vel_ref = [base_speed]*self.config.N_hor
            else:
                speed_ref = dist_to_goal / self.config.N_hor / self.config.ts
                speed_ref = max(speed_ref, self.config.low_speed)
                vel_ref = [speed_ref]*self.config.N_hor

            if establish_heading and abs(theta_ref[idx]-states[-1])<(math.pi/6): # if initial heading established
                establish_heading = False

            if len(system_input):
                last_u = system_input[-self.config.nu:]
            else:
                last_u = [0.0] * self.config.nu

            ### Assemble parameters for solver ###
            parameter_list = self.load_tunning_parameter(robot_flag=('aligning' if establish_heading else None))
            params = x_cur + last_u + x_finish + \
                     parameter_list + \
                     refs + vel_ref + \
                     dyn_constraints

            try:
                exit_status, solver_time, cost, pred_states = self.mpc_generator.run(params, self.solver, self.config.num_steps_taken, system_input, states)
                cost_list.append(cost)
            except RuntimeError as err:
                print("Fatal: Cannot run.")
                if self.vb:
                    print(err)
                return

            if exit_status in self.config.bad_exit_codes and self.vb:
                print(f"{self.__prtname} Bad converge status: {exit_status}")

            ### Plot in loop XXX
            if plot_in_loop:
                vel_ax.plot(kt*self.config.ts, system_input[-2], 'bo')
                omega_ax.plot(kt*self.config.ts, system_input[-1], 'bo')
                cost_ax.plot(kt*self.config.ts, cost, 'bo')

                graph.plot_map(path_ax)
                veh = plt.Circle((states[-3], states[-2]), self.config.vehicle_width/2, color='b', alpha=0.7, label='Robot')
                path_ax.add_artist(veh)
                path_ax.plot(states[-3], states[-2], 'b.')
                pred_line = path_ax.plot(pred_states[::self.config.ns], pred_states[1::self.config.ns], 'm.')
                remove_later = []
                for obstacle_list in full_obstacle_list: # each "obstacle_list" has N_hor predictions
                    for al, pred in enumerate(obstacle_list):
                        x,y,rx,ry,angle,_ = pred
                        pos = (x,y)
                        this_ellipse = patches.Ellipse(pos, rx, ry, angle/(2*math.pi)*360, color='r', alpha=max(8-al,1)/20, label='Obstacle')
                        path_ax.add_patch(this_ellipse)
                        remove_later.append(this_ellipse)

                plt.draw()
                plt.pause(0.01)
                # while not plt.waitforbuttonpress():  # XXX press a button to continue
                #     pass

                pred_line.pop(0).remove()
                for j in range(len(remove_later)): # NOTE: dynamic obstacles (predictions)
                    remove_later[j].remove()
                veh.remove()
            ### Plot in loop

            terminated = self.termination_condition(states, end, system_input)
            kt += self.config.num_steps_taken

        xx     = states[0:len(states):self.config.ns]
        xy     = states[1:len(states):self.config.ns]
        uv     = system_input[0:len(system_input):2]
        uomega = system_input[1:len(system_input):2]

        plt.show()

        return xx, xy, uv, uomega, cost_list



import os, sys, time, math
import itertools
from pathlib import Path

import cv2
import numpy as np
import opengen as og

from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.cm as cm

### Choose the right file for scanner and advisor
from obstacle_scanner.test_dynamic_obstacles import ObstacleScanner
from path_advisor.visibility import PathAdvisor

from mpc.mpc_generator import MpcModule

'''
File info:
    Name    - [trajectory_generator]
    Ref     - [Trajectory generation for mobile robotsin a dynamic environment using nonlinear model predictive control, CASE2021]
            - [https://github.com/wljungbergh/mpc-trajectory-generator]
    Exe     - [No]
File description:
    Assemble the path advisor, obstacle scanner, and MPC module together.
File content:
    TrajectoryGenerator <class> - Build and run the MPC problem. Visualize the results.
Comments:
    [MAP] --graph--> [PathAdvisor] --refernece path--> [TrajGenrator(Config)] <--dynamic obstacles-- [ObstacleScanner]
    Note in plot_dynamic_results, 'plt.waitforbuttonpress()' is used to stop the animation from running automatically. Comment that if no need.
'''

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
        print_name    <str>     - The name to print while running this class.
        config        <dotdict> - As above mentioned.
        time_dict     <dict>    - Keep all the computing time information.
        ppp           <object>  - The path adviser offering the reference path.
        scanner       <object>  - The obstacle scanner offering the dynamic obstacle info.
        mpc_generator <object>  - The MPC module.
    Functions
        plot_results      <vis>  - Visualization of the planned trajectory, only result or with process.
        run               <run>  - Run.
        get_brake_vel_ref <get>  - Get the velocity reference if the robot needs to stop.
        runtime_analysis  <misc> - Print all runtime.
    Comments:
        Have fun but may need to modify the dynamic obstacle part (search NOTE).
    '''
    def __init__(self, config, build=False, verbose=False):
        self.print_name = '[Traj]'
        self.config = config
        self.vb = verbose
        
        self.time_dict = dict() # in ms
        self.loop_time = [] # loop = solve + overhead
        self.solver_times = []
        self.overhead_times = []

        self.ppp           = PathAdvisor(inflate_margin=self.config.vehicle_width, save_plot_path=None)
        self.scanner       = ObstacleScanner()
        self.mpc_generator = MpcModule(self.config)

        if build:
            self.mpc_generator.build()

    def plot_results(self, x_coords, y_coords, vel, omega, start, end, animation=False, video=False, plot_prediction=False):
        if animation:
            self.plot_dynamic_results(x_coords, y_coords, vel, omega, start, end, video, plot_prediction)
        else:
            self.plot_static_results(x_coords, y_coords, vel, omega, start, end)

    def plot_static_results(self, xx, xy, vel, omega, start, end):
        fig = plt.figure(constrained_layout=True)
        gs = GridSpec(2, 4, figure=fig)

        vel_ax = fig.add_subplot(gs[0, :2])
        self.mpc_generator.plot_action(vel_ax, vel)
        vel_ax.set_xlabel('Time [s]')
        vel_ax.set_ylabel('Velocity [m/s]')

        omega_ax = fig.add_subplot(gs[1, :2])
        self.mpc_generator.plot_action(omega_ax, omega)
        omega_ax.set_xlabel('Time [s]')
        omega_ax.set_ylabel('Angular velocity [rad/s]')

        path_ax = fig.add_subplot(gs[:, 2:])
        self.ppp.plot_map(path_ax, vert_radius=self.config.vehicle_width)
        path_ax.plot(xx, xy, c='b', label='Path', marker='o', alpha=0.5)
        path_ax.plot(start[0], start[1], marker='*', color='g', markersize=15)
        path_ax.plot(end[0], end[1], marker='*', color='r', markersize=15)
        path_ax.set_xlabel('X [m]', fontsize=15)
        path_ax.set_ylabel('Y [m]', fontsize=15)

        legend_elems = [Line2D([0], [0], color='k', label='Original Boundary'),
                        Line2D([0], [0], color='g', label='Padded Boundary'),
                        Line2D([0], [0], color='r', label='Original Obstacles'),
                        Line2D([0], [0], color='y', label='Padded Obstacles'),
                        Line2D([0], [0], marker='o', color='b', label='Generated Path', alpha=0.5),
                        Line2D([0], [0], marker='*', color='g', label='Start Position', alpha=0.5),
                        Line2D([0], [0], marker='*', color='r', label='End Position'),
                        ]

        path_ax.legend(handles=legend_elems)
        path_ax.axis('equal')

    def plot_dynamic_results(self, xx, xy, vel, omega, start, end, make_video, plot_prediction):
        if make_video:
            from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
            fig = plt.figure(constrained_layout=True, figsize=(16,9))
        else:
            fig = plt.figure(constrained_layout=True)
        
        gs = GridSpec(2, 4, figure=fig)

        vel_ax = fig.add_subplot(gs[0, :2])
        vel_line, = vel_ax.plot([1], '-o', markersize=4, linewidth=2)
        vel_ax.set_ylim(-self.config.lin_vel_max - 0.1, self.config.lin_vel_max + 0.1)
        vel_ax.set_xlim(0, self.config.ts * len(xx))
        vel_ax.set_xlabel('Time [s]')
        vel_ax.set_ylabel('Velocity [m/s]')
        vel_ax.grid('on')

        omega_ax = fig.add_subplot(gs[1, :2])
        omega_line, = omega_ax.plot([1], '-o', markersize=4, linewidth=2)
        omega_ax.set_ylim(-self.config.ang_vel_max - 0.1, self.config.ang_vel_max + 0.1)
        omega_ax.set_xlim(0, self.config.ts * len(xx))
        omega_ax.set_xlabel('Time [s]')
        omega_ax.set_ylabel('Angular velocity [rad/s]')
        omega_ax.grid('on')

        path_ax = fig.add_subplot(gs[:, 2:])
        path_ax.plot(start[0], start[1], marker='*', color='g', markersize=15, label='Start')
        path_ax.plot(end[0], end[1], marker='*', color='r', markersize=15, label='End')
        path_ax.arrow(start[0], start[1], math.cos(start[2]), math.sin(start[2]), head_width=0.05, head_length=0.1, fc='k', ec='k')

        self.ppp.plot_map(path_ax, vert_radius=self.config.vehicle_width)
        path_line, = path_ax.plot([1], '-ob', alpha=0.7, markersize=5)
        path_ax.set_xlabel('X [m]', fontsize=15)
        path_ax.set_ylabel('Y [m]', fontsize=15)
        path_ax.axis('equal')
        if make_video:
            fig.tight_layout()

        legend_elems = [Line2D([0], [0], color='k', label='Original Boundary'),
                        Line2D([0], [0], color='g', label='Padded Boundary'),
                        Line2D([0], [0], marker='o', color='b', label='Traversed Path', alpha=0.5),
                        Line2D([0], [0], marker='*', color='g', label='Start Position', alpha=0.5),
                        Line2D([0], [0], marker='*', color='r', label='End Position'),
                        patches.Patch(color='b', label='Robot'),
                        patches.Patch(color='r', label='Obstacle'),
                        patches.Patch(color='y', label='Padded obstacle')
                        ]
        path_ax.legend(handles=legend_elems, loc='lower left')

        obs        = [object] * self.scanner.num_obstacles # NOTE: dynamic obstacles
        obs_padded = [object] * self.scanner.num_obstacles # NOTE: dynamic obstacles
        start_idx = 0
        for i in range(start_idx, len(xx)):
            time = np.linspace(0, self.config.ts*i, i)
            omega_line.set_data(time, omega[:i])
            vel_line.set_data(time, vel[:i])
            path_line.set_data(xx[:i], xy[:i])

            veh = plt.Circle((xx[i], xy[i]), self.config.vehicle_width/2, color='b', alpha=0.7, label='Robot')
            path_ax.add_artist(veh)

            ### Plot obstacles # NOTE
            for idx in range(self.scanner.num_obstacles): # NOTE: Maybe different if the obstacle is different
                pos = self.scanner.get_obstacle_info(idx, i*self.config.ts, 'pos')
                x_radius, y_radius = self.scanner.get_obstacle_info(idx, i*self.config.ts, 'radius')
                angle = self.scanner.get_obstacle_info(idx, i*self.config.ts, 'angle')

                obs[idx] = patches.Ellipse(pos, x_radius*2, y_radius*2, angle/(2*math.pi)*360, color='r', label='Obstacle')
                x_rad_pad = x_radius + self.config.vehicle_width/2 + self.config.vehicle_margin
                y_rad_pad = y_radius + self.config.vehicle_width/2 + self.config.vehicle_margin
                obs_padded[idx] = patches.Ellipse(pos, x_rad_pad*2, y_rad_pad*2, angle/(2*math.pi)*360, color='y', alpha=0.7, label='Padded obstacle')
                
                path_ax.add_artist(obs_padded[idx])
                path_ax.add_artist(obs[idx])
            ## Plot predictions # NOTE
            pred = []
            if plot_prediction:
                for j, obstacle in enumerate(self.scanner.get_full_obstacle_list(i*self.config.ts, self.config.N_hor, ts=self.config.ts)):
                    for al, obsya in enumerate(obstacle):
                        x,y,rx,ry,angle = obsya
                        pos = (x,y)
                        this_ellipse = patches.Ellipse(pos, rx*2, ry*2, angle/(2*math.pi)*360, color='r', alpha=max(8-al,1)/20, label='Obstacle')
                        pred.append(this_ellipse)
                        path_ax.add_patch(this_ellipse)

            if make_video:
                canvas = FigureCanvas(fig) # put pixel buffer in numpy array
                canvas.draw()
                mat = np.array(canvas.renderer._renderer)
                mat = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)
                if i == start_idx:
                    video = cv2.VideoWriter('video.mp4', cv2.VideoWriter_fourcc(*'avc1'), 10, (mat.shape[1],mat.shape[0]))
                video.write(mat)
                print(f'{self.print_name} wrote frame {i+1}/{len(xx)}')
            else:
                plt.draw()
                plt.pause(self.config.ts / 10)

                while not plt.waitforbuttonpress():  # XXX press a button to continue
                    pass
            
            veh.remove()
            for j in range(self.scanner.num_obstacles): # NOTE: dynamic obstacles
                obs[j].remove()
                obs_padded[j].remove()
            for j in range(len(pred)): # NOTE: dynamic obstacles (predictions)
                pred[j].remove()

        if make_video:
            video.release()
            cv2.destroyAllWindows()
            
        plt.show()

    def run(self, graph_map:object, start, end):
        '''
        Description:
            Run the trajectory planner.
        Arguments:
            graph_map <object> - A 'Graph' object for the path advisor.
            start     <list> - The start state [x, y, theta]
            end       <list> - The end state [x, y, theta]
        Return:
            xx             <list> - List of x coordinate.
            xy             <list> - List of y coordinate.
            uv             <list> - List of velocity input.
            uomega         <list> - List of angular velocity input.
            solver_times   <list> - List of solver   time per loop/step
            overhead_times <list> - List of overhead time per loop/step
        Comments:
            'x_init'    <list>          [x, y, theta]
            'x_finish'  <list>          [x, y, theta]
            'node_list' <list of tuple> Path nodes
            'time_dict' See details in 'runtime_analysis'
        '''
        t_temp = time.time()  # Used to check run time for specific functions
        mng = og.tcp.OptimizerTcpManager(self.config.build_directory + os.sep + self.config.optimizer_name)
        mng.start()
        mng.ping() # ensure RUST solver is up and runnings
        self.time_dict["launch_optimizer"] = int(1000*(time.time()-t_temp))

        # Initialize tuning parameters to be passed to solver
        parameter_list = [self.config.qp, self.config.qv, self.config.qtheta, self.config.lin_vel_penalty, self.config.ang_vel_penalty,
                          self.config.qpN, self.config.qthetaN, self.config.qcte, self.config.lin_acc_penalty, self.config.ang_acc_penalty]
        # generate costs to establish initial heading
        p_init_c = [0.0*i for i in parameter_list]
        p_init_c[2] = np.max(parameter_list)

        tt = time.time() # total time

        if self.vb:
            print(f"{self.print_name} Initializing the path advisor...")
        t_temp = time.time()
        self.ppp.prepare(graph_map)
        self.time_dict["launch_adviser"] = int(1000*(time.time()-t_temp))

        if self.vb:
            print(f"{self.print_name} Requesting the reference path...")
        t_temp = time.time()
        path, _ = self.ppp.get_ref_path( (start[0], start[1]), (end[0], end[1]) )
        self.time_dict["get_refpath"] = int(1000*(time.time()-t_temp))

        if self.vb:
            print(f"{self.print_name} Requesting the reference trajectory...")
        x_ref, y_ref, theta_ref = self.mpc_generator.gen_ref_trajectory( (start[0], start[1]), path[1:] )
        self.time_dict["get_reftraj"] = int(1000*(time.time()-t_temp)) # including computing reference path

        if self.vb:
            print(f"{self.print_name} Rough reference was succedfully generated.")

        ### Prepare for the loop computing ###
        N_hor               = self.config.N_hor # frequently used: control/prediction horizon
        params_per_dyn_obs  = N_hor*self.config.ndynobs
        base_speed          = self.config.lin_vel_max*self.config.throttle_ratio
        bk_vels, bk_dists   = self.get_brake_vel_ref()
        
        system_input = [] # initalize list with selected system inputs/velocities
        states = start.copy() # initialiize states as starting state
        ref_points = [(x, y) for x, y in zip(x_ref, y_ref)]
        # Initialize lists
        refs = [0.0] * (N_hor * self.config.ns)
        stc_constraints = [0.0] * self.config.Nobs*self.config.nobs
        dyn_constraints = [0.0] * self.config.Ndynobs * self.config.ndynobs*N_hor
        # Avoid dividing by zero in MPC solver by init x radius and y radius to 1
        dyn_constraints[2::self.config.ndynobs] = [1.0] * self.config.Ndynobs*N_hor
        dyn_constraints[3::self.config.ndynobs] = [1.0] * self.config.Ndynobs*N_hor

        ### Start the loop ###
        t = 0   # time step, start from 0
        idx = 0 # index of the current reference trajectory point
        terminal = False
        establish_heading = False
        t_temp = time.time()
        try:
            while (not terminal) and t < 500.0/self.config.ts:
                t_overhead = time.time()
                x_init = states[-self.config.ns:] # set current state as initial state for solver

                if len(self.ppp.obstacle_list):
                    # Create constraints from verticies
                    constraint_origin = self.ppp.find_closest_vertices( (x_init[0], x_init[1]), self.config.Nobs, 0)
                    margin = self.config.vehicle_width/2 + self.config.vehicle_margin
                    stc_constraints = list(itertools.chain( *[(x, y, margin) for x, y in constraint_origin]) )
                    stc_constraints += [0.0] * (self.config.Nobs * self.config.nobs - len(stc_constraints)) # zero-pad to correct length

                if t == 0: # NOTE May vary for different types of obstacles
                    full_obstacle_list = self.scanner.get_full_obstacle_list(current_time=(t*self.config.ts), horizon=N_hor, ts=self.config.ts)
                    for i, dyn_obstacle in enumerate(full_obstacle_list):
                        dyn_constraints[i*params_per_dyn_obs:(i+1)*params_per_dyn_obs] = list(itertools.chain(*dyn_obstacle))
                else: # Rotate list to the left
                    dyn_constraints = dyn_constraints[self.config.ndynobs*self.config.num_steps_taken:] + \
                                      dyn_constraints[:self.config.ndynobs*self.config.num_steps_taken]
                    current_time = (t+N_hor-self.config.num_steps_taken)*self.config.ts
                    full_obstacle_list = self.scanner.get_full_obstacle_list(current_time=current_time, horizon=self.config.num_steps_taken, ts=self.config.ts)
                    for i, dyn_obstacle in enumerate(full_obstacle_list):
                        # Update last num_steps taken dynobs positions
                        dyn_constraints[(i+1)*params_per_dyn_obs-self.config.ndynobs*self.config.num_steps_taken:(i+1)*params_per_dyn_obs] = list(
                            itertools.chain(*dyn_obstacle))

                ### Get reference states ###
                lb_idx = max(0, idx-1*self.config.num_steps_taken)                  # reduce search space for closest reference point
                ub_idx = min(len(ref_points), idx+5*self.config.num_steps_taken)    # reduce search space for closest reference point

                _, idx = self.ppp.get_closest_vert((x_init[0], x_init[1]), ref_points[lb_idx:ub_idx])
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
                if (idx+N_hor) >= len(x_ref)-bk_dists[0]/base_speed: # if any future step will be within braking "zone"
                    num_base_speed = min(len(x_ref)-idx-1, N_hor)
                    vel_ref = [base_speed]*num_base_speed
                    if num_base_speed == 0:
                        dist_to_goal = math.sqrt(
                            (states[-3]-end[0])**2+(states[-2]-end[1])**2)
                        vel_ref = [vel for (vel, dist) in zip(bk_vels, bk_dists) if dist <= dist_to_goal]
                    else:
                        num_brake_vel = min(len(bk_vels), N_hor-num_base_speed)
                        vel_ref += bk_vels[:num_brake_vel]
                    vel_ref += [0.0] * (self.config.N_hor-len(vel_ref)) # Zero pad velocity references
                else:
                    vel_ref = [base_speed]*self.config.N_hor

                if establish_heading and abs(theta_ref[idx]-states[-1])<(math.pi/6): # if initial heading established
                    establish_heading = False

                if len(system_input):
                    last_u = system_input[-self.config.nu:]
                else:
                    last_u = [0.0] * self.config.nu

                ### Assemble parameters for solver ###
                if(not establish_heading):
                    params = x_init + last_u + x_finish + \
                            parameter_list + \
                            refs + vel_ref + \
                            stc_constraints + dyn_constraints
                else:
                    params = x_init + last_u + x_finish + \
                            p_init_c + \
                            refs + vel_ref + \
                            stc_constraints + dyn_constraints

                try:
                    exit_status, solver_time = self.mpc_generator.run(params, mng, self.config.num_steps_taken, system_input, states)
                    self.solver_times.append(solver_time)
                except RuntimeError as err:
                    print("Fatal: Cannot run.")
                    if self.vb:
                        print(err)
                    return

                if exit_status in self.config.bad_exit_codes and self.vb:
                    print(f"{self.print_name} Bad converge status: {exit_status}")

                if np.allclose(states[-3:-1], end[0:2], atol=0.05, rtol=0) and abs(system_input[-2]) < 0.005:
                    terminal = True
                    print(f"{self.print_name} MPC solution found.")

                t += self.config.num_steps_taken
                loop_time = (time.time() - t_overhead)*1000.0

                self.loop_time.append(loop_time)
                self.overhead_times.append(loop_time-solver_time)

        except KeyboardInterrupt:
            if self.vb:
                print(f"{self.print_name} Killing TCP connection to MCP solver...")
            mng.kill()
            ns = self.config.ns
            xx = states[0:len(states):ns]
            xy = states[1:len(states):ns]
            uv = system_input[0:len(system_input):2]
            uomega = system_input[1:len(system_input):2]

            return xx, xy, uv, uomega, self.solver_times, self.overhead_times

        mng.kill()

        total_compute_time = int(1000*(time.time()-tt))   # from preparing to solving
        total_mpc_time     = int(1000*(time.time()-t_temp)) # MPC running time

        self.time_dict["mpc_time"] = total_mpc_time
        self.time_dict["solver_time"] = sum(self.solver_times)
        self.time_dict["average_solver_time"] = np.mean(self.solver_times)
        self.time_dict["average_loop_time"] = np.mean(self.loop_time)
        self.time_dict["total_time"] = total_compute_time
        self.runtime_analysis()

        # Prepare state for plotting
        xx = states[0:len(states):self.config.ns]
        xy = states[1:len(states):self.config.ns]
        uv = system_input[0:len(system_input):2]
        uomega = system_input[1:len(system_input):2]

        return xx, xy, uv, uomega, self.solver_times, self.overhead_times

    def get_brake_vel_ref(self):
        '''
        Description:
            Calculates reference velocities during braking when close to target.
        Return:
            bk_vels  <list> - List of velocities during braking per time step.
            bk_dists <list> - List of remaining braking distances per time step.
        '''
        base_speed = self.config.lin_vel_max*self.config.throttle_ratio
        bk_acc = -base_speed/(self.config.ts*self.config.vel_red_steps) # brake acceleration
        bk_acc = max(self.config.lin_acc_min, bk_acc)
        bk_time = -base_speed/bk_acc
        bk_time_steps = math.ceil(bk_time/self.config.ts)
        # Velocities at each time step for proper braking
        bk_vels = [base_speed - i*base_speed/(bk_time_steps-1) for i in range(bk_time_steps)]
        # Predicted distance to goal at each time step
        bk_dists = [0.0] * len(bk_vels)
        bk_dists[0] = base_speed*bk_time + 0.5*bk_acc*bk_time**2
        for i, vel in enumerate(bk_vels):
            if i < len(bk_dists)-1:
                bk_dists[i+1] = bk_dists[i] - vel*self.config.ts
        return bk_vels, bk_dists

    def runtime_analysis(self, save_path=None):
        '''
        Description:
            Runtime analysis print.
        Arguments:
            save_path <str> - If not None, output to the file.
        Comments:
            'time_dict' VVV
                t0
                launch MPC
                t0
                tt
                    tp
                    prepare reference
                    tp
                    t1
                    run in loop
                        t2
                        (per loop), (solve)
                        overhead = per loop - solver time
                        t2
                    t1 (MPC time)
                tt (total time = MPC time + prepare)
        '''

        if not len(self.time_dict):
            print('There are no time entries run program before runtime analysis...')
            return
            
        str1 = f'Runtime Analysis ({time.strftime("%a, %d %b %Y %H:%M:%S +0100", time.localtime())})'
        str2 = f'Launching optimizer           : {self.time_dict["launch_optimizer"]} ms'

        str3 = f'Prepare visibility graph      : {self.time_dict["launch_adviser"]} ms'
        str4 = f'Generate reference trajectory : {self.time_dict["get_reftraj"]} ms'
        str5 = f'MPC total running time        : {self.time_dict["mpc_time"]} ms'
        str6 = f'MPC total solving time        : {int(self.time_dict["solver_time"])} ms'
        str7 = f'MPC total overhead time       : {int(self.time_dict["mpc_time"]-self.time_dict["solver_time"])} ms'
        str8 = f'MPC average running time      : {int(self.time_dict["average_loop_time"])} ms'
        str9 = f'MPC average solving time      : {int(self.time_dict["average_solver_time"])} ms'
        str10= f'System total running time     : {self.time_dict["total_time"]} ms'
        str_list = [str3,str4,str5,str6,str7,str8,str9,str10]

        print(70*'#')
        print(str1)
        print(70*'#')
        print(str2)
        print(70*'-')
        for s in str_list:
            print(s)
        print(70*'#')

        if save_path is None:
            return

        try:
            file = open(file_name, "a")
            file.write('###############################################\n')
            file.write(str1+'\n')
            file.write('###############################################\n')
            file.write(str2+'\n')
            for s in str_list:
                file.write(s+'\n')
            file.write('###############################################\n\n')
            file.close()
        except OSError:
            print("Runtime analysis was called with an invalid filename")

    def plot_solver_performance(self, plot_type="scatter"):
        '''
        Description:
            Plots all solver times generated in the MPC loop.
        Return:
            plot_type  <str> - Name of desired plot ('scatter','plot', or 'hist').
        '''

        if plot_type == "scatter":
            x_ax = [self.config.ts*i for i in range(len(self.solver_times))]
            plt.scatter(x_ax, self.solver_times, label='solve times')
            plt.legend()
            plt.ylabel('solve time [ms]')
            plt.xlabel('sampled at time [s]')
            plt.show()
        elif plot_type == "plot":
            x_ax = [self.config.ts*i for i in range(len(self.solver_times))]
            plt.plot(x_ax, self.solver_times, label='solve times')
            plt.legend()
            plt.ylabel('solve time [ms]')
            plt.xlabel('sampled at time [s]')
            plt.show()
        elif plot_type == "hist":
            plt.hist(self.solver_times, density=True, bins=60)
            plt.ylabel('Probability')
            plt.xlabel('Solver Time [ms]')
            plt.show()

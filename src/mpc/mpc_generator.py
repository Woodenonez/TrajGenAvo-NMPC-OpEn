import os
import math, time

import numpy as np

import opengen as og
import casadi.casadi as cs

'''
File info:
    Name    - [mpc_generator]
    Date    - [Jan. 01, 2021] -> [Aug. 20, 2021]
    Exe     - [No]
File description:
    The MPC module defines the MPC problem with the cost and constraints.
Comments:
    Adjust MAX_SOVLER_TIME accordingly.
'''

MAX_SOVLER_TIME = 500_000 # ms

def dist_point_to_lineseg(point, line_segment):
    '''
    Arguments:
        point        <ndarray>          - column vector
        line_segment <list of ndarray>  - two points
    Return:
        distance <value>
    Comments:
        [Ref] https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment
    '''
    (p, s1, s2) = (point, line_segment[0], line_segment[1])
    s2s1 = s2-s1 # line segment
    t_hat = cs.dot(p-s1,s2s1)/(s2s1[0]**2+s2s1[1]**2+1e-16)
    t_star = cs.fmin(cs.fmax(t_hat,0.0),1.0) # limit t
    temp_vec = s1 + t_star*s2s1 - p # vector pointing to closest point
    distance = np.sqrt(temp_vec[0]**2+temp_vec[1]**2)
    return distance

def cost_cte(point, line_segments:list, weight=1):
    '''
    Description:
        [Cost] Cross-track-error, penalizes on the deviation from the reference path.
    Arguments:
        point         <ndarray>         - column vector
        line_segments <list of ndarray> - from the the start point to the end point
    Comments:
        The 'line_segments' contains segments which are end-to-end.
    '''
    (p, p_ls) = (point, line_segments)
    distances_sqrt = cs.SX.ones(1)
    for i in range(len(p_ls)-1):
        distance = dist_point_to_lineseg(p, [p_ls[i], p_ls[i+1]])
        distances_sqrt = cs.horzcat(distances_sqrt, distance**2)
    cost = cs.mmin(distances_sqrt[1:]) * weight
    return cost

def cost_vec_sqaure(vector, weight=1):
    return np.sum(np.multiply(vector**2, weight))

### Main class ###
class MpcModule:
    '''
    Description:
        Build the MPC module. Define states, inputs, cost, and constraints.
    Arguments:
        config  <dotdict> - A dictionary in the dot form contains all information/parameters needed.
    Attributes:
        print_name    <str>     - The name to print while running this class.
        config        <dotdict> - As above mentioned.
    Functions
        gen_ref_trajectory <pre>  - Generate the reference trajectory from the reference path.
        build              <pre>  - Build the MPC problem and solver.
        run                <run>  - Run.
        plot_action        <vis>  - Plot an input action.
    '''
    def __init__(self, config):
        self.print_name = '[MPC]'
        self.config = config

    def gen_ref_trajectory(self, current_position:tuple, node_list):
        '''
        Description:
            Generate the reference trajectory from the reference path.
        Arguments:
            current_position <tuple>         - The x,y coordinates.
            node_list        <list of tuple> - A list of reference path nodes.
        Return:
            x_ref       <list> - List of x coordinates  of the reference trajectory per time step. 
            y_ref       <list> - List of y coordinates  of the reference trajectory per time step.
            theta_ref   <list> - List of heading angles of the reference trajectory per time step.
        '''
        x,y = current_position
        x_target, y_target = node_list[0]
        v_ref = self.config.throttle_ratio * self.config.lin_vel_max # self.v_ref
        
        (x_ref, y_ref, theta_ref) = ([],[],[])
        idx = 0
        traveling = True
        while(traveling):# for n in range(N):
            t = self.config.ts
            while(True):
                x_dir = (x_target-x)/math.hypot(x_target-x,y_target-y)
                y_dir = (y_target-y)/math.hypot(x_target-x,y_target-y)
                dist = math.hypot(x_target-x,y_target-y)
                time = dist/v_ref
                if time > t: # move to the target node for t
                    x,y = x+x_dir*v_ref*t, y+y_dir*v_ref*t 
                    break # to append the position
                else: # move to the target node then set a new target
                    x,y = x+x_dir*v_ref*time, y+y_dir*v_ref*time
                    t = t-time # update how much time you have left on the current time step
                    idx = idx+1
                    if idx > len(node_list)-1 :
                        traveling = False
                        break
                    else:
                        x_target, y_target = node_list[idx] # set a new target node
            x_ref.append(x)
            y_ref.append(y)
            theta_ref.append(math.atan2(y_dir,x_dir))
            
        return x_ref, y_ref, theta_ref
        
    def build(self):
        '''
        Description:
            Build the MPC problem and solver, including states, inputs, cost, and constraints.
        Conmments:
            Horizon: N_hor
            Inputs (u): speed, angular speed
            states (s): x, y, theta
            Constraints (z):    1. Initialization, states, and parameters (0~17) ->
                                    x, y, theta, v0, w0; x_goal, y_goal, theta_goal;
                                    qp, qv, qtheta, rv, rw; qN, qthetaN, qCTE, acc_penalty, omega_acc_penalty
                                    (rv, rw are penalties for input)
                                2. Reference path (dim of states * N_hor)
                                3. Speed reference in each step (N_hor)
                                4. Dynamic obstacles (#obs * dim of obs_params * N_hor)
            Reference: Ellipse definition - [https://math.stackexchange.com/questions/426150/what-is-the-general-equation-of-the-ellipse-that-is-not-in-the-origin-and-rotate]
        '''
        print(f'{self.print_name} Building MPC module...')

        u = cs.SX.sym('u', self.config.nu*self.config.N_hor)    # 0. Inputs at every predictive step

        s = cs.SX.sym('s', 2*self.config.ns + self.config.nu)   # 1. States and initial inputs and final goal
        q = cs.SX.sym('q', self.config.nq)                      # 1. Penalty parameters
        r = cs.SX.sym('r', self.config.ns*self.config.N_hor     # 2. Reference path
                         + self.config.N_hor)                   # 3. Speed reference in each step
        o = cs.SX.sym('o', self.config.Ndynobs*self.config.ndynobs*self.config.N_hor) # 4. Dynamic obstacles
        z = cs.vertcat(s,q,r,o)
        
        (x, y, theta, v_init, w_init, x_goal, y_goal, theta_goal) = (s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7])
        (qp, qv, qtheta, rv, rw, qN, qthetaN, qCTE, acc_penalty, w_acc_penalty) = (q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9])
        
        cost = 0
        obstacle_constraints = 0

        for t in range(0, self.config.N_hor): # LOOP OVER TIME STEPS
            
            u_t = u[t*self.config.nu:(t+1)*self.config.nu]  # inputs at time t
            
            # Kinematic model
            x += self.config.ts * (u_t[0] * cs.cos(theta))
            y += self.config.ts * (u_t[0] * cs.sin(theta))
            theta += self.config.ts * u_t[1]

            # Dynamic obstacles
            # (x, y, rx, ry, tilted_angle) for obstacle 0 for N_hor steps, then (x, y, rx, ry, tilted_angle) for obstalce 1 for N_hor steps...
            x_dyn  = o[t*self.config.ndynobs  ::self.config.ndynobs*self.config.N_hor]
            y_dyn  = o[t*self.config.ndynobs+1::self.config.ndynobs*self.config.N_hor]
            rx_dyn = o[t*self.config.ndynobs+2::self.config.ndynobs*self.config.N_hor]
            ry_dyn = o[t*self.config.ndynobs+3::self.config.ndynobs*self.config.N_hor]
            As     = o[t*self.config.ndynobs+4::self.config.ndynobs*self.config.N_hor]

            # ellipse center - x_dyn, y_dyn;  radii - rx_dyn, ry_dyn;  angles of ellipses (positive from x axis) - As
            distance_inside_ellipse = 1 - ((x-x_dyn)*cs.cos(As)+(y-y_dyn)*cs.sin(As))**2/(rx_dyn**2) - ((x-x_dyn)*cs.sin(As)-(y-y_dyn)*cs.cos(As))**2/(ry_dyn)**2
            obstacle_constraints += cs.fmax(0, distance_inside_ellipse)

            # Initialize list with CTE to all line segments
            current_p = cs.vertcat(x,y)
            path_ref = [cs.vertcat(r[i*self.config.ns], r[i*self.config.ns+1]) for i in range(1, self.config.N_hor)]
            cost += cost_cte(current_p, path_ref, weight=qCTE) # [cost] cross track error
            cost += rv*u_t[0]**2 + rw*u_t[1]**2 # [cost] penalize control actions
            cost += qv*(u_t[0]-r[self.config.ns*self.config.N_hor+t])**2 # [cost] deviation from refenrence velocity
            
        cost += qN*((x-x_goal)**2 + (y-y_goal)**2) + qthetaN*(theta-theta_goal)**2 # terminated cost

        # Max speeds 
        umin = [self.config.lin_vel_min, -self.config.ang_vel_max] * self.config.N_hor
        umax = [self.config.lin_vel_max, self.config.ang_vel_max] * self.config.N_hor
        bounds = og.constraints.Rectangle(umin, umax)

        # Acceleration bounds and cost
        v = u[0::2] # velocity
        w = u[1::2] # angular velocity
        acc   = (v-cs.vertcat(v_init, v[0:-1]))/self.config.ts
        w_acc = (w-cs.vertcat(w_init, w[0:-1]))/self.config.ts
        acc_constraints = cs.vertcat(acc, w_acc)
        # Acceleration bounds
        acc_min   = [ self.config.lin_acc_min] * self.config.N_hor 
        w_acc_min = [-self.config.ang_acc_max] * self.config.N_hor
        acc_max   = [ self.config.lin_acc_max] * self.config.N_hor
        w_acc_max = [ self.config.ang_acc_max] * self.config.N_hor
        acc_bounds = og.constraints.Rectangle(acc_min + w_acc_min, acc_max + w_acc_max)
        # Accelerations cost
        cost += cs.mtimes(acc.T, acc)*acc_penalty
        cost += cs.mtimes(w_acc.T, w_acc)*w_acc_penalty

        problem = og.builder.Problem(u, z, cost) \
            .with_penalty_constraints(obstacle_constraints) \
            .with_constraints(bounds) \
            .with_aug_lagrangian_constraints(acc_constraints, acc_bounds)

        build_config = og.config.BuildConfiguration() \
            .with_build_directory(self.config.build_directory) \
            .with_build_mode(self.config.build_type)
        build_config.with_build_python_bindings()
        # build_config.with_tcp_interface_config()

        meta = og.config.OptimizerMeta() \
            .with_optimizer_name(self.config.optimizer_name)

        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(1e-4) \
            .with_max_duration_micros(MAX_SOVLER_TIME)

        builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config) \
            .with_verbosity_level(1)
        builder.build()

        print(f'{self.print_name} MPC module built.')

    def run(self, parameters, solver, take_steps, system_input, states):
        '''
        Description:
            Run the solver for the pre-defined MPC problem.
        Arguments:
            parameters   <list>   - All parameters used by MPC, defined in 'build'.
            take_steps   <int>    - The number of control step taken by the input (normally one).
            system_input <list>   - The overall input action.
            states       <list>   - The overall states.
        Return:
            exit_status <???>   - The exit state of the solver.
            solver_time <value> - Time cost for solving MPC of the current time step
        Comments:
            Here contains the motion model again (can be optimized).
        '''
        solution = solver.run(parameters)
        
        u = solution.solution
        exit_status = solution.exit_status
        solver_time = solution.solve_time_ms
        
        system_input += u[:self.config.nu*take_steps]

        for i in range(take_steps):
            u_v = u[i*self.config.nu]
            u_omega = u[1+i*self.config.nu]
            
            x = states[-3]
            y = states[-2]
            theta = states[-1]

            states += [x + self.config.ts * (u_v * math.cos(theta)), # XXX
                       y + self.config.ts * (u_v * math.sin(theta)), 
                       theta + self.config.ts*u_omega]
        
        return exit_status, solver_time



import os

import numpy as np

import opengen as og
import casadi.casadi as cs

from mpc import mpc_helper as helper

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

    def build(self):
        '''
        Description:
            Build the MPC problem and solver, including states, inputs, cost, and constraints.
        Conmments:
            Horizon: N_hor
            Inputs (u): speed, angular speed
            states (s): x, y, theta, e (e is the tube width / allowable error)
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
        (qpos, qvel, qtheta, rv, rw, qN, qthetaN, qCTE, acc_penalty, w_acc_penalty) = (q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9])
        
        cost = 0
        penalty_constraints = 0

        for kt in range(0, self.config.N_hor): # LOOP OVER TIME STEPS
            
            u_t = u[kt*self.config.nu:(kt+1)*self.config.nu]  # inputs at time t
            
            # Kinematic model
            state_next = helper.dynamics_rk4(self.config.ts, cs.vcat([x,y,theta]), u_t)
            x, y, theta = state_next[0], state_next[1], state_next[2]

            # Dynamic obstacles
            # (alpha, x, y, rx, ry, tilted_angle) for obstacle 0 for N_hor steps, then (x, y, rx, ry, tilted_angle) for obstalce 1 for N_hor steps...
            x_dyn     = o[kt*self.config.ndynobs  ::self.config.ndynobs*self.config.N_hor]
            y_dyn     = o[kt*self.config.ndynobs+1::self.config.ndynobs*self.config.N_hor]
            rx_dyn    = o[kt*self.config.ndynobs+2::self.config.ndynobs*self.config.N_hor]
            ry_dyn    = o[kt*self.config.ndynobs+3::self.config.ndynobs*self.config.N_hor]
            As        = o[kt*self.config.ndynobs+4::self.config.ndynobs*self.config.N_hor]
            alpha_dyn = o[kt*self.config.ndynobs+5::self.config.ndynobs*self.config.N_hor]

            # ellipse center - x_dyn, y_dyn;  radii - rx_dyn, ry_dyn;  angles of ellipses (positive from x axis) - As
            # penalty_constraints += cs.fmax(0, helper.inside_ellipse([x, y], [x_dyn, y_dyn, rx_dyn, ry_dyn, As]))
            cost += helper.cost_inside_ellipse([x, y], [x_dyn, y_dyn, rx_dyn, ry_dyn, As, alpha_dyn], weight=10)

            # Initialize list with CTE to all line segments
            path_ref = [cs.vertcat(r[i*self.config.ns], r[i*self.config.ns+1]) for i in range(1, self.config.N_hor)]
            cost += helper.cost_cte(cs.vertcat(x,y), path_ref, weight=qCTE) # [cost] cross track error
            cost += rv*u_t[0]**2 + rw*u_t[1]**2 # [cost] penalize control actions
            cost += qvel*(u_t[0]-r[self.config.ns*self.config.N_hor+kt])**2 # [cost] deviation from refenrence velocity
            
        cost += qN*((x-x_goal)**2 + (y-y_goal)**2) + qthetaN*(theta-theta_goal)**2 # terminated cost

        # Max speeds 
        umin = [self.config.lin_vel_min, -self.config.ang_vel_max] * self.config.N_hor
        umax = [self.config.lin_vel_max,  self.config.ang_vel_max] * self.config.N_hor
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
            .with_constraints(bounds) \
            .with_aug_lagrangian_constraints(acc_constraints, acc_bounds)
        if penalty_constraints is not 0:
            problem.with_penalty_constraints(penalty_constraints)

        build_config = og.config.BuildConfiguration() \
            .with_build_directory(self.config.build_directory) \
            .with_build_mode(self.config.build_type)
        build_config.with_build_python_bindings()

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
        cost = solution.cost
        
        system_input += u[:self.config.nu*take_steps]
        for i in range(take_steps):
            state_next = helper.dynamics_rk4(self.config.ts, states[-self.config.ns:], u[(i*self.config.nu):(2+i*self.config.nu)])
            states += np.array(state_next).reshape(-1,).tolist()

        pred_states = states[-self.config.ns:]
        for i in range(len(u)//self.config.nu):
            state_next = helper.dynamics_rk4(self.config.ts, pred_states[-self.config.ns:], u[(i*self.config.nu):(2+i*self.config.nu)])
            pred_states += np.array(state_next).reshape(-1,).tolist()
        pred_states = pred_states[self.config.ns:]
        
        return exit_status, solver_time, cost, pred_states



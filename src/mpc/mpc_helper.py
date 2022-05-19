
import casadi.casadi as cs



def dynamics_ct(ts, x, u): # δ(state) per ts
    dx = ts * (u[0]*cs.cos(x[2]))
    dy = ts * (u[0]*cs.sin(x[2]))
    dtheta = ts * u[1]
    return cs.vertcat(dx, dy, dtheta)

def dynamics_rk1(ts, x, u): # discretized via Runge-Kutta 1 (Euler method)
    return x + dynamics_ct(ts, x, u)

def dynamics_rk4(ts, x, u): # discretized via Runge-Kutta 4
    k1 = dynamics_ct(ts, x, u)
    k2 = dynamics_ct(ts, x + 0.5*k1, u)
    k3 = dynamics_ct(ts, x + 0.5*k2, u)
    k4 = dynamics_ct(ts, x + k3, u)
    x_next = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    return x_next
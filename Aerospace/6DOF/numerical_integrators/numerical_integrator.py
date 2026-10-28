import numpy as np

def forward_euler(f, t_s, x, u, h_s, vmod, amod):
    """_summary_

    Forward Euler integrator to approximate solution of differential eqn

    Args:
        f   (func) : function representing RHS of a ODE (x_dot = f(x,t))
        t_s (list) : vector of time points at which numerical solution will be approximated
        x   (list) : numerically approximated solns of f(x,t)
        h_s (float): step size in sec
        vmod(dict) : vehicle (aircraft) model
        amod(dict) : atmosphere model

    Returns:
        t_s (list) : vector of time points at which numerical solution was be approximated
        x   (list) : numerically approximated solns of f(x,t)
    """
    
    # Fwd Euler numerical integration
    dx = np.empty((12, len(t_s)), dtype=float)

    for i in range(1, len(t_s)):
        dx[:, i] = f(t_s[i-1], x[:, i-1], u, amod, vmod)
        x[:, i]  = x[:, i-1] + h_s * dx[:, i] 

    return t_s, x, dx

def rk4(f, ti, yi, u, h_s, **kwargs):
    dx = np.empty((12, 1), dtype=float)

    # 4th order RK
    k1 = f(ti, yi, u, **kwargs)
    k2 = f(ti + h_s/2, yi + k1*h_s/2, u, **kwargs)
    k3 = f(ti + h_s/2, yi + k2*h_s/2, u, **kwargs)
    k4 = f(ti + h_s, yi + k3*h_s, u, **kwargs)

    y = yi + (h_s/6)*(k1 + 2*k2 + 2*k3 + k4)
    dy= k1 #unintegrated dynamics

    return y, dy

def rk4_fixed_ip(f, t_s, x, u, h_s, **kwargs):
    dx = np.empty((12, len(t_s)), dtype=float)

    for i in range(1, len(t_s)):
        ti = t_s[i-1]
        yi = x[:,i-1]

        x[:, i], dx[:, i] = rk4(f, ti, yi, u, h_s, **kwargs)

    return t_s, x, dx

def rk4_timeseries(f, t_s, x, u, h_s, **kwargs):
    dx = np.empty((12, len(t_s)), dtype=float)

    for i in range(1, len(t_s)):
        ti = t_s[i-1]
        yi = x[:,i-1]
        ui = u[i-1]

        x[:, i], dx[:, i] = rk4(f, ti, yi, ui, h_s, **kwargs)
    
    return t_s, x, dx

"""
    Code for linearizing the aircraft into state-space matrices
"""
import numpy as np
from scipy.optimize import least_squares
import ussa1976

if __package__:
    from ..governing_equations.flat_earth_eom import flat_earth_eom
    from ..vehicle_models.multirotor import quadcopter
else:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from governing_equations.flat_earth_eom import flat_earth_eom
    from vehicle_models.multirotor import quadcopter

def system_matrix(f, x_trim, u_trim, epsilon=1e-6, **kwargs):
    n_states = len(x_trim)
    A = np.zeros((x_trim, x_trim))

    for i in range(n_states):
        # perturb state forward
        x_plus = x_trim.copy() # directly equating copies it as reference
        x_plus[i] += epsilon
        f_plus = f(0., x_plus, u_trim, **kwargs)

        # perturb state backward
        x_minus = x_trim.copy()
        x_minus[i] += epsilon
        f_minus = f(0., x_minus, u_trim, **kwargs)

        # Central diff derivative approximation
        A[:, i] = (f_plus - f_minus) / 2 * epsilon

        return A

def main():
    
    
    pass
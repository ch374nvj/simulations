"""
    Code for trimming the aircraft, as a prerequisite to linearization
"""
import numpy as np
from scipy.optimize import least_squares
import ussa1976
from ..governing_equations.flat_earth_eom import flat_earth_eom
from ..vehicle_models.multirotor import quadcopter

def trim_residuals(op_vec, target_vec, **kwargs):
    """
    Computes residuals by taking f{EOM function}'s derivatives 

    Args:
        f (function): The function that needs to be trimmed
        op_vec (list): Optimization vector that is used by cost func to minimize
                        {states(x):[phi, theta, u, v, w], control(u_vec):[rotors or surface deflections]}
        target_vec (list): This vector is the same length as op_vec, consists of target values the op_vec must hopefully reach, eg. u = 10 m/s
        kwargs:
            vmod: a vehicle model (vmod) must be passed as an explicit argument
    """
    
    vmod = kwargs.get('vmod')
    amod = kwargs.get('amod')
    num_rotors = kwargs.get('num_rotors')
    num_surfaces = kwargs.get('num_surfaces')

    if num_rotors is not None:
        u_vec = op_vec[-num_rotors:]
    if num_surfaces is not None:
        u_vec = op_vec[-num_surfaces:]
    else:
        u_vec = op_vec[-4:] #last 4 elements

    # states x = [u, v ,w, p, q, r, phi, theta, psi, x, y, h]
    # note not all states will be used by optimizer to obtain trim
    # And taking only first 6 derivatives for the minimization

    # Unpacking op_vec and repacking it to states vector
    states = np.zeros(12)
    phi, theta, u, v, w = op_vec[:5]
    states[:3] = [u, v, w]
    states[6:8] = [phi, theta]

    # derivatives dx = [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, x_dot, y_dot, h_dot]
    dx = flat_earth_eom(0, states, u_vec, vmod=vmod, amod=amod)

    target_optimization = op_vec - target_vec

    residuals = np.append(dx[:6], target_optimization)

    return residuals


def main():

    atmosphere = ussa1976.compute()
    
    # Essential gravity and atmospheric data
    amod_alt_m       = atmosphere['z'].values
    amod_rho_kgpm3   = atmosphere['rho'].values
    amod_c_mps       = atmosphere['cs'].values
    amod_g_mps2      = ussa1976.core.compute_gravity(amod_alt_m)

    # amod: atmospheric model with gravity
    amod = {
        "alt_m"     : amod_alt_m,
        "rho_kgpm3" : amod_rho_kgpm3,
        "c_mps"     : amod_c_mps,
        "g_mps2"    : amod_g_mps2
    }

    vmod = quadcopter.Quad_X_DJI_Mavic_Pro()
    num_rotors = vmod.num_rotors

    x0 = [0.0] * 5 + [0.0] * num_rotors
    target_vec = x0

    result = least_squares(trim_residuals, x0, args=(target_vec,), kwargs={'amod': amod, 'vmod': vmod})

    if result.success:
        print("trim successful")
        print(f"trim cost: {result.cost}")
        print(result.x)

    else:
        print('Trim failed to converge')

if __name__ == '__main__':
    main()
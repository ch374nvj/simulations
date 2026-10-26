"""
    Code for trimming the aircraft, as a prerequisite to linearization
"""
import numpy as np
from scipy.optimize import least_squares

if __package__:
    from ..governing_equations.flat_earth_eom import flat_earth_eom
    from ..vehicle_models.multirotor import quadcopter
else:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from governing_equations.flat_earth_eom import flat_earth_eom
    from vehicle_models.multirotor import quadcopter

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
    
    num_rotors = kwargs.get('num_rotors')
    num_surfaces = kwargs.get('num_surfaces')

    if num_rotors is not None:
        num_ctrl_vars = num_rotors
    if num_surfaces is not None:
        num_ctrl_vars = num_surfaces
    else:
        num_ctrl_vars = 4
    u_vec = op_vec[-num_ctrl_vars:] #last ctrl elements

    # states x = [u, v ,w, p, q, r, phi, theta, psi, x, y, h]
    # note not all states will be used by optimizer to obtain trim
    # And taking only first 6 derivatives for the minimization

    # Unpacking op_vec and repacking it to states vector
    [u_t, v_t, w_t], [phi_t, theta_t], alt_t = target_vec
    states = np.zeros(12)
    u, v, w, phi, theta, = op_vec[:5]
    states[:3] = [u, v, w]
    states[6:8] = [phi, theta]
    states[-1] = -alt_t

    # derivatives dx = [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, x_dot, y_dot, h_dot]
    dx = flat_earth_eom(0, states, u_vec, **kwargs)

    target_optimization = op_vec[:5] - [u_t, v_t, w_t, phi_t, theta_t]
    # print(f'target_op: {target_optimization} \n op_vec: {op_vec}')

    residuals = np.append(dx[:6], target_optimization)

    return residuals


def main():
    vmod = quadcopter.Quad_X_DJI_Mavic_Pro()
    num_ctrl_vars = vmod.num_rotors
    vehicle_name = type(vmod).__name__

    x0 = [0.0] * 5 + [100] * num_ctrl_vars
    target_vel_mps = [0., 0., 0.]
    target_attitude_rad = [0., 0.]
    target_alt_m = 200
    target_vec = [target_vel_mps, target_attitude_rad, target_alt_m] #u, v, w, phi, theta

    lower_bounds = [
        -20., #u (m/s)
        -10., #v
        -10., #w
        -np.pi/2, #phi (rad)
        -np.pi/2, #theta
        0.0, #w1 (rpm)
        0.0, #w2
        0.0, #w3
        0.0, #w4
    ]

    upper_bounds = [
        20.,
        10.,
        10.,
        np.pi/2,
        np.pi/2,
        12000,
        12000,
        12000,
        12000,
    ]

    result = least_squares(
        trim_residuals, x0, args=(target_vec,), kwargs={'vmod': vmod, 'verbose':0},
        bounds=(lower_bounds, upper_bounds), method='trf',
        )

    if result.success:
        print("trim successful")
        print(f"trim cost: {result.cost}")
        print(f"trim states: {result.x}")
        print('Final residual values (result.fun):', result.fun)
        trim_states = np.zeros(12)
        trim_states[:6] = result.fun[:6]
        trim_states[-1] = -target_alt_m
        trim_control = result.x[-num_ctrl_vars:]
        np.savetxt(f"dataset/{vehicle_name}_trim_ip.csv", trim_control, delimiter=",")
        np.save(f"dataset/{vehicle_name}_trim_ip.npy", trim_control)
        np.savetxt(f"dataset/{vehicle_name}_trim_states.csv", trim_states, delimiter=",")
        np.save(f"dataset/{vehicle_name}_trim_states.npy", trim_states)
        print()
        np.set_printoptions(precision=4, suppress=True)
        print(f'Final Trim states: {trim_states}\n')
        print(f'Final Trim Control: {trim_control}\n')
    
    else:
        print('Trim failed to converge')

    print(f'trim message: {result.message}')

if __name__ == '__main__':
    # Script must be run as module:
    # python3 -m 6DOF.tools.trim
    main()
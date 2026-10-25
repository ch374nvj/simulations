"""
    Code for linearizing the aircraft into state-space matrices
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

def system_matrix(f, x_trim, u_trim, epsilon=1e-6, **kwargs):
    n_states = len(x_trim)
    A = np.zeros((n_states, n_states))

    for i in range(n_states):
        # perturb state forward
        x_plus = x_trim.copy() # directly equating copies it as reference
        x_plus[i] += epsilon
        f_plus = f(0., x_plus, u_trim, **kwargs)

        # perturb state backward
        x_minus = x_trim.copy()
        x_minus[i] -= epsilon
        f_minus = f(0., x_minus, u_trim, **kwargs)

        # Central diff derivative approximation
        A[:, i] = (f_plus - f_minus) / (2 * epsilon)

    return A

def control_matrix(f, x_trim, u_trim, epsilon=1e-6, **kwargs):
    n_states = len(x_trim)
    n_ctrl = len(u_trim)
    B = np.zeros((n_states, n_ctrl))

    for i in range(n_ctrl):
        # perturb ctrl upward
        u_plus = u_trim.copy()
        u_plus[i] += epsilon
        f_plus = f(0., x_trim, u_plus, **kwargs)

        # perturb ctrl downward
        u_minus = u_trim.copy()
        u_minus[i] -= epsilon
        f_minus = f(0., x_trim, u_minus, **kwargs)

        B[:,i] = (f_plus - f_minus) / (2 * epsilon)

    return B

def main():
    vmod = quadcopter.Quad_X_DJI_Mavic_Pro()
    x_trim = np.load("Quad_X_DJI_Mavic_Pro_trim_states.npy")
    u_trim = np.load("Quad_X_DJI_Mavic_Pro_trim_ip.npy")
    epsilon = 1e-1

    A = system_matrix(flat_earth_eom, x_trim, u_trim, epsilon, vmod=vmod, verbose=0)
    B = control_matrix(flat_earth_eom, x_trim, u_trim, epsilon, vmod=vmod, verbose=0)

    # np.set_printoptions(precision=4, suppress=True)

    # print(A)
    # print()
    # print(B)

    # states, x = [u v w p q r phi theta psi x y h]
    lon_index = [4, 7] # u,w,q,theta

    lat_index = [1,3,5,6,8] #v,p,r,phi,theta

    # Longitudinal state-space
    Alon = A[np.ix_(lon_index, lon_index)]
    Blon = B[lon_index, :]

    # Lateral state-space
    Alat = A[np.ix_(lat_index, lat_index)]
    Blat = B[lat_index, :]

    np.savez("DJI_mavic_pro_lon.npz", Alon=Alon, Blon=Blon)
    np.savez("DJI_mavic_pro_lat.npz", Alat=Alat, Blat=Blat)

    print(np.linalg.eigvals(Alon))
    print(np.linalg.eigvals(Alat))

if __name__ == '__main__':
    main()
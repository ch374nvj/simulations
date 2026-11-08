import numpy as np
from tools.state_space import StateSpace
from tools.Interpolators import interp
from vehicle_models.multirotor.quadcopter import Quad_X_DJI_Mavic_Pro
import matplotlib.pyplot as plt

# Set time conditions
t0_s = 0.0
tf_s = 10.
h_s  = 0.01
t_s = np.arange(t0_s, tf_s + h_s, h_s)
nt_s = t_s.size

# loading linear state space plant
with np.load('./dataset/DJI_mavic_pro_AB.npz') as data:
    lin_plant_A = data['A']
    lin_plant_B = data['B']

lin_plant = StateSpace(lin_plant_A, lin_plant_B, np.eye(12), np.zeros((12,4)), h_s)

vmod = Quad_X_DJI_Mavic_Pro()
xtrim = vmod.x_trim
nx = xtrim.size
print(xtrim)

# timeseries control input
time_data = np.array([0., 1., 1.1, 3., 3.1, 5., 5.1, 8., 8.1, 10.])
delta = 100
rpm1 = np.array([0., 0., delta, delta, 0., 0., -delta, -delta, 0., 0.])
rpm2 = np.array([0., 0., -delta, -delta, 0., 0., delta, delta, 0., 0.])
# rpm2 = np.array([0.] * 10)
rpm3 = np.array([0., 0., -delta, -delta, 0., 0., delta, delta, 0., 0.])
rpm4 = np.array([0., 0., delta, delta, 0., 0., -delta, -delta, 0., 0.])

rpm_cmd = np.empty((4, nt_s), dtype=float) 
x = np.empty((nx, nt_s), dtype=float)

for i, t in enumerate(t_s):
    rpm_cmd[0,i] = interp(time_data, rpm1, t)
    rpm_cmd[1,i] = interp(time_data, rpm2, t)
    rpm_cmd[2,i] = interp(time_data, rpm3, t)
    rpm_cmd[3,i] = interp(time_data, rpm4, t)

    x[:,i] = lin_plant.step(rpm_cmd[:, i])

# plotting
simulation_test = 'quad_test_linear'
test_count = 4

fig, axes = plt.subplots(1, 4, figsize=(10,6), layout="constrained")
fig.set_facecolor('black')
fig.suptitle("phi, theta, psi, alt", color="white")

d2r = 57.3

axes[0].plot(t_s, x[6,:]*d2r, label='phi', color='red', linestyle='--')
# axes[0].plot(t_s, rpm_cmd[0,:]*d2r, label='phi_cmd', color='blue')
axes[0].set_xlabel('Time (sec)', color='white')
axes[0].set_ylabel('phi', color='white')
axes[0].grid(True)
axes[0].set_facecolor('black')
axes[0].tick_params(colors='white')
axes[0].legend(facecolor='black', labelcolor='white')    

axes[1].plot(t_s, x[7,:]*d2r, label='theta', color='red', linestyle='--')
# axes[1].plot(t_s, rpm_cmd[1,:]*d2r, label='theta_cmd', color='blue')
axes[1].set_xlabel('Time (sec)', color='white')
axes[1].set_ylabel('theta', color='white')
axes[1].grid(True)
axes[1].set_facecolor('black')
axes[1].tick_params(colors='white')
axes[1].legend(facecolor='black', labelcolor='white')

axes[2].plot(t_s, x[8,:]*d2r, label='psi', color='red', linestyle='--')
# axes[2].plot(t_s, rpm_cmd[2,:]*d2r, label='psi_cmd', color='blue')
axes[2].set_xlabel('Time (sec)', color='white')
axes[2].set_ylabel('psi', color='white')
axes[2].grid(True)
axes[2].set_facecolor('black')
axes[2].tick_params(colors='white')
axes[2].legend(facecolor='black', labelcolor='white')

axes[3].plot(t_s, -x[11,:], label='alt', color='red', linestyle='--')
# axes[3].plot(t_s, rpm_cmd[3,:], label='alt_cmd', color='blue')
axes[3].set_xlabel('Time (sec)', color='white')
axes[3].set_ylabel('alt', color='white')
axes[3].grid(True)
axes[3].set_facecolor('black')
axes[3].tick_params(colors='white')
axes[3].legend(facecolor='black', labelcolor='white')

plt.savefig(f'savefig/linear/{simulation_test}_{test_count}.png')

# Export data
headers = (
"time_s,u_b_mps,v_b_mps,w_b_mps,"
"p_b_radps,q_b_radps,r_b_radps,"
"phi_rad,theta_rad,psi_rad,"
"p1_n_m,p2_n_m,p3_n_m,"
"delta_u1, delta_u2, delta_u3, delta_u4"
)

data = np.column_stack((t_s, x.T, rpm_cmd.T))

np.savetxt(
f"dataset/linear/simulation_output_{simulation_test}_{test_count}.csv",
data,
delimiter=",",
header=headers,
comments="", 
fmt="%.10g",
)

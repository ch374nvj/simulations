# Six Degrees of Freedom (6DOF) Quadcopter Simulation

This repository contains a comprehensive six degrees of freedom (6DOF) simulation of an aircraft. The simulation models the full nonlinear dynamics of the vehicle including aerodynamic forces, rigid body dynamics, and control system integration.

## Overview

The simulation performs the following key tasks:

- **Atmospheric Modeling**: Uses the US Standard Atmosphere 1976 (USSA1976) model for realistic altitude-dependent air density, speed of sound, and gravity
- **Numerical Integration**: Solves the nonlinear equations of motion using 4th-order Runge-Kutta method
- **Visualization**: Generates comprehensive plots showing states, positions, and accelerations over time
- **Data Export**: Saves both plots and time-series data for analysis and validation

## Repository Structure
The sixDOF simulation code is entirely written in python and heavily uses classes and objects (My fav). If it is a little hard to follow the code, I recommend you to open the specific file or folder in github and click on the commit history on the right side (one with the clock symbol). 

```
simulations/Aerospace/sixDOF/
├── main.py                          # Main nonlinear simulation script
├── linear_model_openloop_test.py    # Linear model validation script
├── dataset/                         # Files containing Linearized state-space models, trim states and trim control inputs 
├── governing_equations/              # Files defining EOMs
├── numerical_integrators/           # Numerical integration methods (Contains Euler and RK4 solvers)
├── savefig/                         # Generated output plots
│   ├── nonlinear/                  # Nonlinear simulation plots
│   └── linear/                     # Linear model plots
├── tools/                           # Utility functions and libraries
│   ├── linearization.py/            # To Linearize the model at certain trim condition, and obtain state space matrices
│   ├── profutils.py/                # A code performance profiler (AI generated)
│   ├── state-space.py/              # A generic class that runs state space model simulations using rk4
│   └── trim.py/                     # Code to trim the Hi-Fi 6dof model to a spicific operating condition using least squares and obtain the corresponding trim states and control ip  
├── vehicle_models/                 # Vehicle-specific models
└── readme.md                       # This file
```
## AI usage declaration
At a time when the use of AI LLMs is prevalent and helps speed up the code-writing process, I would like to clarify that no AI tools were used to write the code or develop the libraries, unless explicitly mentioned in the commits or in the inline comments (eg., [commit 595ca7e](https://github.com/ch374nvj/simulations/commit/595ca7eda6007018f67f46e388f70c0edb7e3672)).


As a personal principle, I generally avoid using AI tools for my personal projects.  However, I do use them as learning aids to explore and understand theoretical concepts, clarify doubts, and deepen my understanding of unfamiliar topics.

I would also like to acknowledge that AI was used as an aid in debugging certain issues, such as resolving import errors involving my custom library packages  
(like [commit 93724b1](https://github.com/ch374nvj/simulations/commit/93724b1143c3f5d6e48f3c882d72b8eeb000fe6e))

## Dependencies

```bash
numpy>=1.21.0
matplotlib>=3.3.0
ipympl>=0.9.0
ussa1976>=1.0.0
```

## Installation

1. Ensure Python 3.7+ is installed
2. Install required dependencies:
   ```bash
   pip install numpy matplotlib ipympl ussa1976
   ```

3. The repository structure already includes all necessary subdirectories and files

## Usage

### Running the Main Simulation

The primary simulation is in `main.py`. When executed, it will:

1. Load atmospheric data and trim conditions from the dataset
2. Simulate the quadcopter response to control input perturbations
3. Generate plots showing:
   - Position and velocity states (x, y, z, u, v, w)
   - Angular states (phi, theta, psi, p, q, r)
   - Body accelerations (u_dot, v_dot, w_dot)
4. Save plots to `savefig/` directory
5. Export time-series data to `dataset/` directory

**Note**: For running the simulation of state space, it requires the trim input and states which can be taken from `tools/trim.py` code output or typed manually.

### Running Linear Model Test

The `linear_model_openloop_test.py` script demonstrates the linear state-space model:

1. Loads the linear model matrices (A, B) (from `dataset/DJI_mavic_pro_AB.npz`)
2. Simulates the linear plant using control input perturbations
3. Generates plots showing attitude and altitude responses
4. Exports linear simulation data

### Key Variables and States

**State Vector (12 elements)**:
1. `u`: Body-fixed x-axis velocity
2. `v`: Body-fixed y-axis velocity  
3. `w`: Body-fixed z-axis velocity
4. `p`: Roll rate (rad/s)
5. `q`: Pitch rate (rad/s)
6. `r`: Yaw rate (rad/s)
7. `phi`: Roll angle (rad)
8. `theta`: Pitch angle (rad)
9. `psi`: Yaw angle (rad)
10. `p1`: Position x (m)
11. `p2`: Position y (m)
12. `p3`: Position z (m) [negated for altitude]

**Control Inputs (4 elements)**:
1. `u1`: Rotor 1 speed perturbation
2. `u2`: Rotor 2 speed perturbation
3. `u3`: Rotor 3 speed perturbation
4. `u4`: Rotor 4 speed perturbation


## Output Files

### Generated Plots
- **Nonlinear Simulation**: Multiple PNG files in `savefig/nonlinear/`
- Examples:
  - `quad_test_trim_perturb_yaw_1.png`: States plot
  - `quad_test_trim_perturb_yaw_1_xyz.png`: Position plot
  - `quad_test_trim_perturb_yaw_1_uvw_dot.png`: Acceleration plot
  - `quad_test_linear_4.png`: Linear model plots (in `savefig/linear/`)

## Mathematical Models

### Equations of Motion
- **Reference Frame**: NED (North-East-Down) for position, Body frame for velocity and orientation
- **Coordinate Transformations**: Full 6DOF kinematic and dynamic equations
- **Atmospheric Effects**: Density, viscosity, and compressibility included


## Extensions and Customization

1. **New Vehicle Models**: Add new vehicle models in `vehicle_models/` directory
2. **Numerical Integration Methods**: Swap numerical integrators in `numerical_integrators/` directory

## Contact

For questions or issues, contact me over [LinkedIn](https://linkedin.com/in/ch374n).
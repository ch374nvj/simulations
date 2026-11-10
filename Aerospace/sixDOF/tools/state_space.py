import numpy as np

import sys
from pathlib import Path

# 1. Dynamically find the submodule root folder regardless of execution context
# Adjust .parents[N] depending on how deep this file is nested inside the submodule
root = Path(__file__).resolve().parent.parent  # Points to 'root'

if str(root) not in sys.path:
    sys.path.insert(0, str(root))

from simulations.Aerospace.sixDOF.numerical_integrators.numerical_integrator import rk4

class StateSpace:
    """
        A class to run linear state space models. Takes (A, B, C & D) matrices
    """
    def __init__(self, A, B, C, D, dt) -> None:
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.C = np.asarray(C, dtype=float)
        self.D = np.asarray(D, dtype=float)

        self.dt = dt

        self.n_states = self.A.shape[0]
        self.n_inputs = self.B.shape[1]

        assert self.A.shape == (self.n_states, self.n_states)
        assert self.B.shape[0] == self.n_states

        # State is represented as deviation from trim
        self.x = np.zeros(self.n_states)

    def _derivatives(self, t, x, u):
        return self.A @ x + self.B @ u

    def outputs(self, x, u):
        self._derivatives(0., x, u)
        return self.C @ self.x + self.D @ u

    def step(self, u):
        u = np.asarray(u, dtype=float)

        self.x, _ = rk4(self._derivatives, 0, self.x, u, self.dt)

        return self.x.copy()
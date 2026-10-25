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

def main():
    pass
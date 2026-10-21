"""
    This library holds a subclass of multirotors, namely Quad_plus_* ('+' config quadcopters)
    and Quad_X_* ('x' config quads)

    When initialized in main function, it can be used to get forces and moments of the selected aircraft

    References:
        E. Stingu and F. Lewis, "Design and implementation of a structured flight controller for a 6DoF quadrotor using quaternions" doi: 10.1109/MED.2009.5164715.
"""

from multirotor import MultiRotor

class Quad_X_DJI_Mavic_Pro(MultiRotor):
    """
    Model of DJI Mavic Pro, obtained from https://github.com/souhaiel1/DJI-Mavic-pro-simplified-modelling-and-simulation
    """

    def __init__(self, num_rotors) -> None:
        super().__init__(num_rotors)
        self._l_m = 0.335
        self._weight_kg = 0.743
        self._frontal_area_m2 = 0.0197
        self._motor_rpmpV = 1400
        self._max_voltage_V = 11.4
        self._voltage_V = 0.0
        self._prop_diameter_m = 0.2032
        self._prop_pitch_m = 0.1016

    def _RPM_model(self, V):
        RPM = -2.6931 * V ** 3 + 1400 * V
        return RPM

    def _thrust_model(self, RPM):
        CT = 2e-15 * RPM ** 3 - 4e-11 * RPM ** 2 + 3e-7 * RPM + 0.1013
        RPS = RPM/60
        thrust = CT * self.rho_kgpm3 * (RPS ** 2) *  self._prop_diameter_m 
        return thrust

    def _torque_model(self, RPM):
        pass

    
    
    def F_B_thrust_N(self):
        pass
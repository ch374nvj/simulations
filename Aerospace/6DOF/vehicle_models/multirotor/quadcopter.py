"""
    This library holds a subclass of multirotors, namely Quad_plus_* ('+' config quadcopters)
    and Quad_X_* ('x' config quads)

    When initialized in main function, it can be used to get forces and moments of the selected aircraft

    References:
        E. Stingu and F. Lewis, "Design and implementation of a structured flight controller for a 6DoF quadrotor using quaternions" doi: 10.1109/MED.2009.5164715.
"""
import numpy as np
from .multirotor import MultiRotor

class Quad_X_DJI_Mavic_Pro(MultiRotor):
    """
    Model of DJI Mavic Pro, obtained from https://github.com/souhaiel1/DJI-Mavic-pro-simplified-modelling-and-simulation
    """

    def __init__(self) -> None:
        super().__init__(4)
        self._diagonal_length_m = 0.335
        self._lateral_length_m = 0.237
        self._mass_kg = 0.743
        self._frontal_area_m2 = 0.0197
        self._motor_rpmpV = 1400
        self._max_voltage_V = 11.4
        self._voltage_V = 0.0
        self._prop_diameter_m = 0.2032
        self._prop_pitch_m = 0.1016
        self._Jxx_b_kgm2 = 0.003
        self._Jyy_b_kgm2 = 0.003
        self._Jzz_b_kgm2 = 0.007
        self._Jxz_b_kgm2 = 0.0
        # self._motor_rpm = [0.0] * 4

    # Expose inertia/mass as public attributes expected by the EOM
    # (flat_earth_eom reads vmod.m_kg, vmod.Jxx_b_kgm2, ...)
    @property
    def m_kg(self):
        return self._mass_kg

    @property
    def Jxx_b_kgm2(self):
        return self._Jxx_b_kgm2

    @property
    def Jyy_b_kgm2(self):
        return self._Jyy_b_kgm2

    @property
    def Jzz_b_kgm2(self):
        return self._Jzz_b_kgm2

    @property
    def Jxz_b_kgm2(self):
        return self._Jxz_b_kgm2

    def _RPM_model(self, V):
        RPM = -2.6931 * V ** 3 + 1400 * V
        return RPM

    def _thrust_model(self, RPM):
        CT = 2e-15 * RPM ** 3 - 4e-11 * RPM ** 2 + 3e-7 * RPM + 0.1013
        RPS = RPM/60
        thrust = CT * self.rho_kgpm3 * (RPS ** 2) *  self._prop_diameter_m ** 4
        return thrust

    def _torque_model(self, RPM):
        return 4e-14 * RPM ** 3 + 8e-12 * RPM ** 2 + 3e-6 * RPM

    def _prop_torque(self):
        Q1 = self._torque_model(self.u_vec_rpm[0])
        Q2 = self._torque_model(self.u_vec_rpm[1])
        Q3 = self._torque_model(self.u_vec_rpm[2])
        Q4 = self._torque_model(self.u_vec_rpm[3])

        return np.array([Q1, Q2, Q3, Q4])

    def _thrust(self):
        T1 = self._thrust_model(self.u_vec_rpm[0])
        T2 = self._thrust_model(self.u_vec_rpm[1])
        T3 = self._thrust_model(self.u_vec_rpm[2])
        T4 = self._thrust_model(self.u_vec_rpm[3])

        return np.array([T1, T2, T3, T4])
    
    def moments_body_kgm2ps2(self):
        thrust_vec = self._thrust()
        motor_lateral_distance = self._lateral_length_m/np.sqrt(2)

        l_b_kgm2ps2 = motor_lateral_distance * np.sum(thrust_vec * [-1, -1, 1, 1])
        m_b_kgm2ps2 = motor_lateral_distance * np.sum(thrust_vec * [-1, 1, 1, -1])
        n_b_kgm2ps2 = np.sum(self._prop_torque() * [1.0,-1.0,1.0,-1.0])

        return [l_b_kgm2ps2, m_b_kgm2ps2, n_b_kgm2ps2]
    
    def F_b_thrust_N(self):
        Fx_thrust = 0
        Fy_thrust = 0
        Fz_thrust = -1 * np.sum(self._thrust())

        return [Fx_thrust, Fy_thrust, Fz_thrust]

    def drag_kgmps2(self):
        C_D = 1.0
        return C_D * self.qbar_kgpm2 * self._frontal_area_m2
    
    def side_kgmps2(self):
        return 0

    def lift_kgmps2(self):
        return 0

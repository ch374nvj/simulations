class MultiRotor:
    def __init__(self, num_rotors) -> None:
        self.num_rotors = num_rotors
        self.u_vec_rpm = [0.0] * num_rotors

    def state_update(self, alpha_rad, beta_rad, c_mps, qbar_kgpm2, rho_kgpm3, 
                     vel_b_mps, TAS_mps, p_b_rps, q_b_rps, r_b_rps, u_vec):
        self.alpha_rad = alpha_rad
        self.beta_rad = beta_rad
        self.qbar_kgpm2 = qbar_kgpm2
        self.rho_kgpm3 = rho_kgpm3
        self.TAS_mps = TAS_mps
        self.vel_b_mps = vel_b_mps  
        self.p_b_rps = p_b_rps
        self.q_b_rps = q_b_rps
        self.r_b_rps = r_b_rps
        self.u_vec_rpm = u_vec
        self.M = TAS_mps/c_mps


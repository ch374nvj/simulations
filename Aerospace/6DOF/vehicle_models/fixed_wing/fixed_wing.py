class FixedWing:
    def __init__(self) -> None:
        pass

    def state_update(self, alpha_rad, beta_rad, c_mps, qbar_kgpm2, 
                     TAS_mps, p_b_rps, q_b_rps, r_b_rps, u_rad):
        self.alpha_rad = alpha_rad
        self.beta_rad = beta_rad
        self.alpha_deg = self.alpha_rad * 57.3
        self.beta_deg = self.beta_rad * 57.3
        self.qbar_kgpm2 = qbar_kgpm2
        self.TAS_mps = TAS_mps
        self.p_b_rps = p_b_rps
        self.q_b_rps = q_b_rps
        self.r_b_rps = r_b_rps
        self.p_deg = self.p_b_rps * 57.3
        self.q_deg = self.q_b_rps * 57.3
        self.r_deg = self.r_b_rps * 57.3
        self.u_rad = u_rad
        self.u_deg = u_rad*[1,1,1,1/57.3]*57.3 #Multiplying only surfaces deflections by 57.3
        self.M = TAS_mps/c_mps


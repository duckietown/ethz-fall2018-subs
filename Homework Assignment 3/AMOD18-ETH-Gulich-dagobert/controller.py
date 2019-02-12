class Controller():

    def __init__(self):
    
    # gains of lqr-k-matrix
	self.k1 = 6.3246
	self.k2 = 4.2011

    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       forward linear velocity of Duckiebot [m/s]
    #           omega_out   angular velocity of Duckiebot [rad/s]

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):
        # calculate the errors of the states to allow for reference tracking
        d_err = d_ref - d_est
        phi_err = phi_ref - phi_est
        
        # calculate the command
        omega_lqr = self.k1*d_err + self.k2*phi_err


        # Declaring return values
        omega_out = omega_lqr
        v_out = v_ref

        return (v_out, omega_out)


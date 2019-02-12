import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller
        self.k_P = 5
        self.k_I = 7
        self.k_D = 1
        self.err_sum = 0
        self.prev_err = 0

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

        # Do your calculations here

        # Calculate the output y, used the same as given in template.
        ref =   (6 * d_ref + 1 * phi_ref)
        y =     (6 * d_est + 1 * phi_est)
        err = ref - y

        # Sum of errors for the I-part 
        self.err_sum += err

        # PID-Controller
        C_P = self.k_P * err + 1/self.k_I * self.err_sum + self.k_D * (err - self.prev_err)/dt_last
        omega = C_P

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        
        # Save currrent error for next time step for the D-part
        self.prev_err = err
        

        return (v_out, omega_out)

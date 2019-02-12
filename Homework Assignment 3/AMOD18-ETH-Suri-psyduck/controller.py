import math
import numpy as np

class Controller():

    def __init__(self):
        # Set your parmeters here
        # Gains for controller
        self.k_P = 3.1
        self.k_I = 0.005
        self.k_D = 0.01
        self.sum_err = 0.0
        self.prev_err = 0.0

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
        # Calculate the output y
        ref =   (6 * d_ref + 2.5 * phi_ref)
        y =     (6 * d_est + 2.5 * phi_est)
        err = ref - y
        # Native P-Controller
        self.sum_err += (err + self.prev_err)*t_delay/2
        C_P = self.k_P * err + self.k_I * self.sum_err + self.k_D * (err - self.prev_err)/t_delay
        omega = C_P
        self.prev_err = err
        # Declaring return values
        omega_out = omega
        v_out = v_ref

        return (v_out, omega_out)


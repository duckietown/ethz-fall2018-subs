##################################################
# Please note, this file is untested:
# See the attached file "video.txt" for details.
# 31.10.2018, Victor Klemm
##################################################

import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller
        self.k_P_d = 1
        self.k_P_phi = 1
        self.k_I_d = 0.02
        self.k_I_phi = 0.02
        self.k_D_d = 0.02
        self.k_D_phi = 0.02

        # Values for integration / differentiation
        self.d_int = 0
        self.phi_int = 0
        self.d_prev = 0
        self.phi_prev = 0

        # values for ARW of the integrator
        self.int_bound_d = 1
        self.int_bound_phi = 1

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

        # ARW of the integrator
        if abs(self.d_int) < self.int_bound_d:
            self.d_int += d_est * dt_last
        if abs(self.phi_int) < self.int_bound_phi:
            self.phi_int += phi_est * dt_last

        # PID controller
        P_part = self.k_P_d * (d_ref - d_est) + self.k_P_phi * (phi_ref - phi_est)
        I_part = self.k_I_d * self.d_int + self.k_I_phi * self.phi_int
        D_part = self.k_D_d * (d_est - self.d_prev)/dt_last + self.k_D_phi * (phi_est - self.phi_prev)/dt_last
        u = P_part + I_part + D_part

        # Declaring return values
        omega_out = u
        v_out = v_ref

        # Update previous values for differentiation
        self.d_prev = d_est
        self.phi_prev = phi_est

        return (v_out, omega_out)

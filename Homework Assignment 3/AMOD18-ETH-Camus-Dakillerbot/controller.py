import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller
        self.k_P = 4.0
        self.k_D = 0.01
        self.k_I = 0.05
        self.mode = 1
        self.last_err = 0.0
        self.omegap = 0.0
        self.omegai_sum = 0.0
        self.omegad = 0.0

    def PIDControl(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        # Do your calculations here

        # Calculate the output y
        ref =   (6 * d_ref + 1.5 * phi_ref)
        y =     (6 * d_est + 1.5 * phi_est)
        err = ref - y

        # PID-Controller

        self.omegap = self.k_P * err
        self.omegai_sum = self.omegai_sum + self.k_I * (dt_last / 2.0) * (err + self.last_err)
        self.omegad = self.k_D * (2.0 / dt_last) * (err - self.last_err)
        omega = self.omegad + self.omegai_sum + self.omegap


        # Declaring return values
        omega_out = omega
        v_out = v_ref

        self.last_err = err
        
        return (v_out, omega_out)


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
        if(self.mode == 1):
            return self.PIDControl(d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last)

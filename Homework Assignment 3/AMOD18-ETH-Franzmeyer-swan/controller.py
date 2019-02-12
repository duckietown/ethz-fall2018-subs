import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller

        self.d_old = 0
        self.phi_old = 0

        self.d_err_sum = 0
        self.phi_err_sum = 0

        self.k_P_d = 24
        self.k_P_phi = 6

        self.k_I_d = 80
        self.k_I_phi = 20

        self.k_D_d = 4
        self.k_D_phi = 1

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        # Do your calculations here

        diff_d = (self.d_old - d_est)/dt_last
        diff_phi = (self.phi_old - phi_est)/dt_last

        # Calculate the output y
        err_d = d_ref - d_est
        err_phi = phi_ref - phi_est

        self.d_err_sum += err_d
        self.phi_err_sum += err_phi

        # Native P-Controller
        C_P_d = self.k_P_d * err_d
        C_P_phi = self.k_P_phi * err_phi

        C_I_d = 1/self.k_I_d * self.d_err_sum
        C_I_phi = 1/self.k_I_phi * self.phi_err_sum

        C_D_d = self.k_D_d * diff_d
        C_D_phi = self.k_D_phi * diff_phi

        omega = C_P_d + C_P_phi + C_I_d + C_I_phi + C_D_d + C_D_phi

        # Declaring return values
        omega_out = omega
        v_out = v_ref

        self.d_old = d_est
        self.phi_old = phi_est

        return (v_out, omega_out)

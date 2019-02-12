import math
import numpy as np


class Controller():

    def __init__(self):
        # Gains for controller
        self.k_P = 5.0
        self.k_I = 0.15
	self.k_D = 1.0
	# Current and last tracking error. 
	self.error = 0.0
	self.error_last = 0.0
	self.error_sum = 0.0

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
        # Calculate the output y
        ref =   (6.0 * d_ref + 1 * phi_ref)
	y =     (6.0 * d_est + 1 * phi_est)
        self.error = ref - y
	self.error_sum += self.error
	# PID steering control.
 	C_P = self.k_P*self.error
	C_I = self.k_I*self.error_sum
	C_D = self.k_D*(self.error - self.error_last)/max(0.01, dt_last)
        omega_out = C_P + C_D + C_I
        # Reference velocity control.
        k_vel = 1.0
	if abs(phi_est - phi_ref) < 0.3 and abs(self.error) < 0.3:
		k_vel = 2.0
	v_out = v_ref
	# Update last error. 
	self.error_last = self.error
        return (v_out, omega_out)

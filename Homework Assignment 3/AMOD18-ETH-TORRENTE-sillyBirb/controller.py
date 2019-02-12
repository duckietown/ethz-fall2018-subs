import math
import numpy as np
import rospy

class Controller():

    def __init__(self):

        # Gains for controller
        self.k_P = 1.2
        self.k_I = 0.5
        self.k_D = 0.4

        # Variables for storing integral error and last error
        self.integral_err = 0
        self.last_err = 0


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
        ref =   (6 * d_ref + 1 * phi_ref)
        y =     (6 * d_est + 1 * phi_est)
        err = ref - y

     

        # Calculate integral and differential errors. Keep integral error in small range
        self.integral_err += err
        if (self.integral_err > 1):
            self.integral_err = 1
        if (self.integral_err < -1):
            self.integral_err = -1

        differential_error = (err-self.last_err)/dt_last

        # Update last error for next iteration
        self.last_err = err

        rospy.loginfo("Error: %s", err)
        rospy.loginfo("P: %s  I: %s  D: %s", self.k_P*err, self.k_I*self.integral_err, self.k_D*differential_error)

        # PID Controller
        C_PID = self.k_P * err + self.k_I*self.integral_err + self.k_D*differential_error
        omega = C_PID

        # Declaring return values
        omega_out = omega
        v_out = v_ref

        return (v_out, omega_out)

import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller
        self.k_P = 2
        self.k_I = 0.3

        self.totalErr = 0


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

        distWeight = 1
        headingWeight = 0.1

        windingErrorLimit = 0.5

        minAllowance = -1.57
        maxAllowance = 1.57
        if phi_est>0 && phi_est<1.57
                maxAllowance = 1.57 - phi_est
        elif phi_est<0 && phi_est>-1.57
                minAllowance = -1.57 - phi_est

        err = distWeight*d_est + headingWeight*phi_est
        if err<minAllowance
                err = minAllowance;
        elif err>maxAllowance
                err = maxAllowance;

        #opposite sign to correct the error
        err = -err

        self.totalErr = self.totalErr + err


        ## Calculate the output y
        #ref =   (6 * d_ref + 1 * phi_ref)
        #y =     (6 * d_est + 1 * phi_est)
        #err = ref - y

        # Naive P-Controller
        C_P = self.k_P * err + self.k_I * self.totalErr
        omega = C_P

        # Declaring return values
        omega_out = omega
        v_out = v_ref

        return (v_out, omega_out)


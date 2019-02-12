import math
import numpy as np


class Controller():
    def __init__(self):

        # Max. angular velocity
        self.omega_max = 2

        # Scale for estimate improvements
        self.est_scale = 0  # No dead reckoning

        # Velocity reduction factor for omega_max
        self.vel_red = 0  # No velocity reduction

        # Gains for controller
        self.k_P_d = 30
        self.k_P_phi = 6
        self.k_I_d = 2.4
        self.k_I_phi = 0.4
        self.k_D_d = 6
        self.k_D_phi = 1

        # Integrals
        self.d_I = 0
        self.phi_I = 0

        # Last errors
        self.err_d_last = 0
        self.err_phi_last = 0

        # Last commands
        self.omega_last = 0
        self.v_last = 0

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
    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay,
                         dt_last):

        # Get (more) current estimates with dead reckoning
        if dt_last < t_delay:
            phi_est_curr = phi_est + self.est_scale * (
                self.omega_last * dt_last)
            if self.omega_last < 0.1:
                d_est_curr = d_est + self.est_scale * (
                    self.v_last * math.sin(phi_est) * dt_last)
            else:
                d_est_curr = d_est + self.est_scale * (
                    self.v_last / self.omega_last *
                    (math.cos(phi_est) - math.cos(phi_est_curr)))
        else:
            phi_est_curr = phi_est
            d_est_curr = d_est

        # Calculate current errors
        err_d = d_ref - d_est_curr
        err_phi = phi_ref - phi_est_curr

        # Update integrals with Modified Euler
        self.d_I += (err_d + self.err_d_last) / 2 * dt_last
        self.phi_I = (err_phi + self.err_phi_last) / 2 * dt_last

        # Calculating error derivatives
        d_err_d = (err_d - self.err_d_last) / dt_last
        d_err_phi = (err_phi - self.err_phi_last) / dt_last

        # Storing errors
        self.err_d_last = err_d
        self.err_phi_last = err_phi

        # PID-Controller
        omega = (self.k_P_d * err_d + self.k_P_phi * err_phi +
                 self.k_I_d * self.d_I + self.k_I_phi * self.phi_I +
                 self.k_D_d * d_err_d + self.k_D_phi * d_err_phi)

        # Declaring return values
        omega_out = self.sat(omega, self.omega_max)
        v_out = v_ref * (1 - self.vel_red * omega_out / self.omega_max)

        # Storing commands
        self.omega_last = self.sat(omega_out, self.omega_max)
        self.v_last = v_out

        return (v_out, omega_out)

    def sat(self, u, u_max):
        if abs(u) > u_max:
            return np.sign(u) * u_max
        return u


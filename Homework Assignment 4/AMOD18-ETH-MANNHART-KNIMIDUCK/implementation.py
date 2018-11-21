from duckietown_world import GenericKinematicsSE2,PlatformDynamicsFactory
from duckietown_serialization_ds1 import Serializable
from contracts import check_isinstance
import geometry as geo
import numpy as np

class CarCommands(Serializable):
    '''
        This represents the velocity commands for car
        kinematics.
    '''

    def __init__(self, linear_velocity, steering_angle):
        self.linear_velocity = linear_velocity
        self.steering_angle = steering_angle



class CarParameters(PlatformDynamicsFactory, Serializable):
    '''
        This class represents the parameters of the ideal differential drive dynamics.

        wheel_distance: distance between front and rear wheels

    '''

    def __init__(self, wheel_distance):
        self.wheel_distance = wheel_distance

    def initialize(self, c0, t0=0, seed=None):
        return CarDynamics(self, c0, t0)


class CarDynamics(GenericKinematicsSE2):
    def __init__(self, parameters, c0, t0):
        """
        :param parameters:  instance of CarParameters
        :param c0: initial configuration
        :param t0: initial time
        """
        check_isinstance(parameters, CarParameters)
        self.parameters = parameters
        GenericKinematicsSE2.__init__(self, c0, t0)

    def integrate(self, dt, commands):
        """

        :param dt:
        :param commands: an instance of CarCommands
        :return:
        """
        check_isinstance(commands, CarCommands)

        # Your code comes here!
        linear = [commands.linear_velocity,0]
        angular = commands.linear_velocity/self.parameters.wheel_distance*np.tan(commands.steering_angle)
        # represent this as se(2)
        commands_se2 = geo.se2_from_linear_angular(linear, angular)

        # Call the "integrate" function of GenericKinematicsSE2
        s1 = GenericKinematicsSE2.integrate(self, dt, commands_se2)

        # new state
        c1 = s1.q0, s1.v0
        t1 = s1.t0
        return CarDynamics(self.parameters, c1, t1)

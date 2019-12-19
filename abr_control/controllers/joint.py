import numpy as np

from .controller import Controller


class Joint(Controller):
    """ Implements a joint space controller

    Moves the arm joints to a set of specified target angles

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    kp : float, optional (Default: 1)
        proportional gain term
    kv : float, optional (Default: None)
        derivative gain term, a good starting point is sqrt(kp)
    """

    def __init__(self, robot_config, kp=1, kv=None):
        super(Joint, self).__init__(robot_config)

        self.kp = kp
        self.kv = np.sqrt(self.kp) if kv is None else kv
        self.ZEROS_N_JOINTS = np.zeros(robot_config.N_JOINTS)
        self.q_tilde = np.copy(self.ZEROS_N_JOINTS)

    def generate(self, q, dq, target, target_velocity=None):
        """Generate a joint space control signal

        Parameters
        ----------
        q : float numpy.array
            current joint angles [radians]
        dq : float numpy.array
            current joint velocities [radians/second]
        target : float numpy.array
            desired joint angles [radians]
        target_velocity : float numpy.array, optional (Default: None)
            desired joint velocities [radians/sec]
        """

        if target_velocity is None:
            target_velocity = self.ZEROS_N_JOINTS

        # calculate the direction for each joint to move, wrapping
        # around the -pi to pi limits to find the shortest distance
        self.q_tilde = ((target - q + np.pi) % (np.pi * 2)) - np.pi

        # get the joint space inertia matrix
        M = self.robot_config.M(q)
        u = np.dot(M, (self.kp * self.q_tilde + self.kv * (target_velocity - dq)))
        # account for gravity
        u -= self.robot_config.g(q)

        return u

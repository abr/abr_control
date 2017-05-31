import numpy as np

from . import controller

class Joint(controller.Controller):
    """ Implements a joint space controller

    Moves the arm joints to a set of specified target angles

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
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

    def generate(self, q, dq, target_pos, target_vel=None):
        """Generate a joint space control signal

        Parameters
        ----------
        q : float numpy.array
            current joint angles in radians
        dq : float numpy.array
            current joint velocities in radians/second
        target_pos : float numpy.array
            desired joint angles in radians
        target_vel : float numpy.array, optional (Default: None)
            desired joint velocities in radians/sec
        """

        self.q_tilde = ((target_pos - q + np.pi) % (np.pi * 2)) - np.pi
        if target_vel is None:
            target_vel = self.ZEROS_N_JOINTS
        # TODO: do we want to include vel compensation?
        # get the joint space inertia matrix
        # M = self.robot_config.M(q)
        # get the gravity compensation signal
        g = self.robot_config.g(q)

        # calculated desired joint control signal
        # self.training_signal = np.dot(M, (self.kp * self.q_tilde +
        #                               self.kv * (target_vel - dq)))
        self.training_signal = self.kp * self.q_tilde - self.kv * dq
        u = self.training_signal - g

        return u

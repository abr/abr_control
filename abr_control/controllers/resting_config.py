import numpy as np

from .controller import Controller


class RestingConfig(Controller):
    """ Move the arm towards a set of 'resting state' joint angles

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as number of joints, number of links, mass information etc.
    """

    def __init__(self, robot_config, rest_angles, kp, kv=None):
        super(RestingConfig, self).__init__(robot_config)

        self.rest_angles = np.asarray(rest_angles)
        self.rest_indices = [val is not None for val in rest_angles]
        self.kp = kp
        self.kv = np.sqrt(kp) if kv is None else kv

    def generate(self, q, dq):
        """ Generates the control signal

        q: np.array
          the current joint angles [radians]
        dq: np.array
          the current joint angle velocity [radians/second]
        """

        # account for going across 2*pi line when calculating
        # distance / direction
        q_des = np.zeros(len(q))
        dq_des = np.zeros(len(q))
        q_des[self.rest_indices] = (
            self.rest_angles[self.rest_indices] - q[self.rest_indices] + np.pi
        ) % (np.pi * 2) - np.pi
        dq_des[self.rest_indices] = dq[self.rest_indices]

        # calculate joint space inertia matrix
        M = self.robot_config.M(q=q)

        return np.dot(M, (self.kp * q_des - self.kv * dq_des))

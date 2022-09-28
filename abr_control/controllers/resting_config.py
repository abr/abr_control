# TODO: set up to work with ball joints / quaternion positions

import numpy as np

from .joint import Joint


class RestingConfig(Joint):
    """Move the arm towards a set of 'resting state' joint angles

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as number of joints, number of links, mass information etc.
    """

    def __init__(self, robot_config, rest_angles, **kwargs):
        super().__init__(robot_config, account_for_gravity=False, **kwargs)

        self.rest_angles = np.asarray(rest_angles)
        self.rest_indices = [val is not None for val in rest_angles]
        self.q_tilde = self.q_tilde_angle

    def q_tilde_angle(self, q, target):
        # distance / direction
        q_tilde = np.zeros(len(q))
        q_tilde[self.rest_indices] = (
            self.rest_angles[self.rest_indices] - q[self.rest_indices] + np.pi
        ) % (np.pi * 2) - np.pi
        return q_tilde

    def generate(self, q, dq):
        """Generates the control signal

        q: np.array
          the current joint angles [radians]
        dq: np.array
          the current joint angle velocity [radians/second]
        """

        return super().generate(q, dq, target=self.rest_angles)

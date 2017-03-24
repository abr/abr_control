import numpy as np


class controller:
    """ Implements a floating controller that compensates for
    the effects of gravity on the arm and velocity.
    """

    def __init__(self, robot_config):

        self.robot_config = robot_config

    def control(self, q, dq):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        """

        # calculate the effect of gravity in joint space
        g = self.robot_config.g(q)
        Mq = self.robot_config.Mq(q)
        u = - g - 0.5 * np.dot(Mq, dq)

        return u

import numpy as np

from .controller import Controller


class Floating(Controller):
    """Implements a controller to compensate for gravity

    Only compensates for the effects of gravity on the arm. The arm will
    remain compliant and hold whatever position it is placed in (as long
    as an accurate mass / inertia model is provided)

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dynamic : boolean, optional (Default: False)
        accounts for joint velocity / inertia in controller if True
    """

    def __init__(self, robot_config, dynamic=False):
        super(Floating, self).__init__(robot_config)
        self.dynamic = dynamic

    def generate(self, q, dq=None):
        """ Generates the control signal to compensate for gravity

        Parameters
        ----------
        q : float numpy.array
            the current joint angles [radians]
        dq : float numpy.array
            the current joint velocities [radians/second]
        """

        # calculate the effect of gravity in joint space
        g = self.robot_config.g(q)
        u = -g

        if self.dynamic:
            # compensate for current velocity
            M = self.robot_config.M(q)
            u -= np.dot(M, dq)

        return u

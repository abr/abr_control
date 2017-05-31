from . import controller

class Floating(controller.Controller):
    """Implements a controller to compensate for gravity

    Implements a floating controller that only compensates for
    the effects of gravity on the arm. Have to be able to use
    force control. The arm will remain compliant and will hold
    whatever position it is placed in as long as an accurate
    mass / inertia model is provided

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    dynamic : boolean, optional (Default: False)
        accounts for joint velocity/inertia in controller if True
    """

    def __init__(self, robot_config, dynamic=False):
        super(Floating, self).__init__(robot_config)

    def generate(self, q, dq=None):
        """ Generates the control signal to compensate for gravity

        Parameters
        ----------
        q : float numpy.array
            the current joint angles in radians
        dq : float numpy.array
            the current joint velocities in radians/second
        """

        # calculate the effect of gravity in joint space
        g = self.robot_config.g(q)
        u = -g

        if self.dynamic:
            # compensate for current velocity
            M = self.robot_config.M(q)
            u -= np.dot(M, dq)

        return u

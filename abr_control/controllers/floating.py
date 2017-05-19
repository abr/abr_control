from . import controller

class Floating(controller.Controller):
    """ Implements a floating controller that only compensates for
    the effects of gravity on the arm.
    """

    def __init__(self, robot_config):
        self.robot_config = robot_config
        super(Floating,self).__init__(robot_config=self.robot_config)

    def generate(self, q, dq=None):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        """

        # calculate the effect of gravity in joint space
        g = self.robot_config.g(q)
        u = -g

        return u

class Signal:
    """ Base class for additive control signals

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):

        self.robot_config = robot_config

    def generate(self, q):
        """ Generates the control signal

        q : np.array
          the current joint angles [radians]
        """
        raise NotImplementedError

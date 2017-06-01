class Interface:
    """ Base class for interfaces

    The purpose of interfaces is to abstract away the API
    and overhead of connection / disconnection etc for each
    of the different systems that can be controlled.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        self.robot_config = robot_config

    def connect(self):
        """ All initial setup. """

        raise NotImplementedError

    def disconnect(self):
        """ Any socket closing etc that must be done
        """

        raise NotImplementedError

    def send_forces(self, u):
        """ Applies the set of torques u to the arm.

        u : np.array
            An array of joint torques [Nm]
        """

        raise NotImplementedError

    def get_feedback(self):
        """ Returns a dictionary of the relevant feedback

        Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """

        raise NotImplementedError

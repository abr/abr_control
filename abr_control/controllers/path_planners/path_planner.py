class PathPlanner(object):
    """ Super class for any trajectory planners.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):

        self.robot_config = robot_config

    def generate(self, state, target, n_timesteps):
        """ Generate the path for the system to follow

        Parameters
        ----------
        state : numpy.array
            the current position of the system [meters]
        target : numpy.array
            the target position [radians]
        n_timesteps : int, optional (Default: 200)
            the number of time steps to reach the target
        raise NotImplementedError
        """

    def next(self):
        """ Return the next target point along the path """
        raise NotImplementedError

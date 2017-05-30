class Controller:
    """
    A class defines the base functions for all controllers

    Parameters
    ----------
    robot_config : class instance, required (Default: None)
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        self.robot_config = robot_config

    def generate(self, q):
        """
        Generate the control signal to move individual joints

        q : float numpy.array, required (Default: None)
            joint angles in radians

        Depending on the complexity of the controller,
        pass in robot status variables (velocity, force etc.)
        and generate a control signal to move joints as an
        output
        """
        raise NotImplementedError

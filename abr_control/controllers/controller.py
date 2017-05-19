class Controller:
    """ Class defines the base functions for controllers
    """

    def __init__(self, robot_config):
        """
        robot_config: config class of robot, passes
        in useful information about the robot being
        used, such as: number of joints, number of
        links, mass information...
        """

    def generate(self, q):
        """
        q: joint angles in radians, the minimum info
        needed for control

        Depending on the complexity of the controller,
        pass in robot status variables and generate a
        control signal to move joints as an output
        """

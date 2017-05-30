import numpy as np

class Signal:
    """ Base class for additive control signals
    """

    def __init__(self, robot_config):

        self.robot_config = robot_config

    def generate(self, q):
        """ Generates the control signal

        q np.array: the current joint angles
        """
        raise NotImplementedError

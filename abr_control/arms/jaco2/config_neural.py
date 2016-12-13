import numpy as np

from . import config


class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q': np.array([3.5, 2.09, 2.75, 4.8, .38, 5.83]),
            'dq': np.array([-.03, -.04, -.018, .003, .00, -.0])
            }

        self.scales = {
            'q': np.array([0.78, 1.24, 0.96, .85, 6.27, 6.28]),
            'dq': np.array([0.75, 1.26, 1.48, 2.21, 2.06, 2.9])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

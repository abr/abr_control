import numpy as np
import sympy as sp

import nengo

from . import config


class robot_config(config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q':  np.array([3.06, .968, -.946, .2, 1.1, -.434]),
            'dq':  np.array([1.21, .177, .54, -.64, -0.112, -1.898])
            }

        self.scales = {
            'q':  np.array([2.0, 0.30, 0.9, 1.0, 0.97, 3.0]),
            'dq': np.array([12.47, 2.5, 1.986, 3.374, 10.557, 6.223])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

import numpy as np
import sympy as sp

import nengo

from . import config


class robot_config(config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.means = {
            'q':  np.array([.88, 1.95, .19]),
            'dq':  np.array([.645, 2.76, -1.422])
            }

        self.scales = {
            'q':  np.array([.52, 1.3, .71]),
            'dq': np.array([6.7, 12.37, 6.18])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

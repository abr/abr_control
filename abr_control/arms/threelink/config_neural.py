import numpy as np

from . import config


class ThreeLinkConfigNeural(config.ThreeLinkConfig):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(RobotConfig, self).__init__(**kwargs)

        self.MEANS = {
            'q':  np.array([.88, 1.95, .19]),
            'dq':  np.array([.645, 2.76, -1.422])
            }

        self.SCALES = {
            'q':  np.array([.52, 1.3, .71]),
            'dq': np.array([6.7, 12.37, 6.18])
            }

    def scaledown(self, name, x):
        return (x - self.MEANS[name]) / self.SCALES[name]

    def scaleup(self, name, x):
        return x * self.SCALES[name] + self.MEANS[name]

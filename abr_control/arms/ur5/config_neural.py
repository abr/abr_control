import numpy as np

from . import config


class UR5ConfigNeural(config.UR5Config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(RobotConfig, self).__init__(**kwargs)

        self.MEANS = {
            'q':  np.array([3.06, .968, -.946, .2, 1.1, -.434]),
            'dq':  np.array([1.21, .177, .54, -.64, -0.112, -1.898])
            }

        self.SCALES = {
            'q':  np.array([2.0, 0.30, 0.9, 1.0, 0.97, 3.0]),
            'dq': np.array([12.47, 2.5, 1.986, 3.374, 10.557, 6.223])
            }

    def scaledown(self, name, x):
        return (x - self.MEANS[name]) / self.SCALES[name]

    def scaleup(self, name, x):
        return x * self.SCALES[name] + self.MEANS[name]

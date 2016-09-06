import numpy as np
import sympy as sp

import nengo

from . import config

class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self):

        super(robot_config, self).__init__()

        self.CB = {
            'dimensions': self.num_joints * 3,
            'n_neurons': 5000,
            'neuron_type': nengo.Direct(),
            'radius': np.sqrt(self.num_joints * 3),
            }

        self.CB_adapt = {
            'dimensions': self.num_joints  * 3,
            'n_neurons': 1000,
            'neuron_type': nengo.Direct(),
            }

        import itertools
        encoder_set = [[-1,1]]*15
        encoders = list(itertools.product(*encoder_set))
        self.M1 = {
            'dimensions': self.num_joints * 2 + 3,# + 128,
            'n_neurons': 20000,
            # 'neuron_type': nengo.Direct(),
            'radius': np.sqrt(self.num_joints * 2 + 3) / 2.0,
            'encoders': nengo.dists.Choice(encoders)
            }

        self.means = {
            'q': np.array([-0.86, 1.26, -2.12, 0.94, 2.44, 12.10, ]),
            'dq': np.array([-0.11, 0.67, -0.93, 0.04, 2.66, -3.80, ]),
            }

        self.scales = {
            'q': (np.array([-0.41, 1.32, -1.61, 0.99, 2.32, 14.27, ]) -
                  np.array([-0.79, 0.80, -2.05, 0.83, 1.35, 12.32, ])),
            'dq': (np.array([3.32, 3.78, 0.03, 0.00, 6.88, 0.00, ]) -
                   np.array([0.00, -0.09, -3.19, -2.56, -0.07, -11.23, ])),
            }
        # np.array([-2.53, 1.03, -1.72, 0.73, 2.34, 12.19, ])
        # np.array([2.41, 0.56, 0.32, 0.41, 1.28, 1.31, ])
        # np.array([-0.53, -0.09, 0.06, 0.01, 0.23, 0.04, ])
        # np.array([4.49, 2.01, 2.35, 2.27, 5.94, 5.63, ])

        self.cq = [sp.Symbol('cq%i' % ii) for ii in range(self.num_joints)]
        self.sq = [sp.Symbol('sq%i' % ii) for ii in range(self.num_joints)]

        self.subs_list = {
            sp.cos(self.q[0]): self.cq[0],
            sp.cos(self.q[1]): self.cq[1],
            sp.cos(self.q[1] + self.q[2]): self.cq[2],
            sp.cos(self.q[1] + self.q[2] + self.q[3]): self.cq[3],
            sp.cos(self.q[4]): self.cq[4],

            sp.sin(self.q[0]): self.sq[0],
            sp.sin(self.q[1]): self.sq[1],
            sp.sin(self.q[1] + self.q[2]): self.sq[2],
            sp.sin(self.q[1] + self.q[2] + self.q[3]): self.sq[3],
            sp.sin(self.q[4]): self.sq[4]}

    def format_q(self, q):
        return [
            np.cos(q[0]),
            np.cos(q[1]),
            np.cos(q[1] + q[2]),
            np.cos(q[1] + q[2] + q[3]),
            np.cos(q[4]),
            np.cos(q[5]),

            np.sin(q[0]),
            np.sin(q[1]),
            np.sin(q[1] + q[2]),
            np.sin(q[1] + q[2] + q[3]),
            np.sin(q[4]),
            np.sin(q[5])]

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

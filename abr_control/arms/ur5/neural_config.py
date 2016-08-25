import numpy as np

import nengo

from . import config


class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self):

        super(robot_config, self).__init__()

        n_neurons = 3000
        self.CB = {
            'dimensions': self.num_joints * 2,
            'n_neurons': n_neurons,
            'neuron_type': nengo.Direct(),
            'radius': 5,
            }

        # n_neurons = 2000
        self.M1 = {
            'dimensions': self.num_joints + 3,
            'n_neurons': n_neurons,
            'neuron_type': nengo.Direct(),
            'radius': .25,
            }

        # # n_neurons = 7000
        # self.M1_mult = {
        #     'encoders': nengo.dists.Choice([[1, 1],
        #                                     [-1, 1],
        #                                     [-1, -1],
        #                                     [1, -1]]),
        #     'ens_dimensions': 2,
        #     'n_ensembles': 6,
        #     'n_neurons': n_neurons,
        #     'neuron_type': nengo.Direct(),
        #     'radius': np.sqrt(2),
        #     }

        # n_neurons = 3000
        self.M1_null = {
            'dimensions': self.num_joints,
            'n_neurons': n_neurons,
            'neuron_type': nengo.Direct(),
            'radius': np.sqrt(3),
            }

        self.means = {
            'u': np.array([0, 0, 0]),
            'CB': np.zeros(12),#np.array([.61, 1.86, .416, -0.03, -.012, .021,
                            #0, 0, 0, 0, 0, 0]),
            'M1': np.zeros(9),#np.array([.58, .57, .23, .79, -.76, -.96, 0, 0, 0]),
            'M1_null': np.zeros(6)#np.array([0.61,  1.9,  0.4, 0, 0, 0]),
            # 'M1_mult': np.array([-167., -26., -90., -81., -20., -33.]),
            }

        self.scales = {
            'u':  np.ones(3),#np.array([2., 1., 1.]),
            'CB': np.ones(12),#np.array([.4, .6, .3, 1.0, 1.0, 1.0,
                            #1, 1, 1, 1, 1, 1]),
            'M1': np.ones(9),#np.array([.43, .525, .3, .25, .4, .08, 1.0, 1.0, 1.0]),
            'M1_null': np.ones(6)# np.array([.45, .55, .32, 1.0, 1.0, 1.0]),
            # 'M1_mult': np.array([50., 65., 35., 35., 5., 9.])
            }

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

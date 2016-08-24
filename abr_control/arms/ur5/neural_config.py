'''
Copyright (C) 2016 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np

import nengo

from . import config


class robot_config(config.robot_config):
    """ robot config file for the UR5 arm """

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

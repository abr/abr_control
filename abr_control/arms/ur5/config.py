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

import cloudpickle
import os
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ robot config file for the UR5 arm """

    def __init__(self):

        super(robot_config, self).__init__(num_joints=6, num_links=7,
                                           robot_name='ur5')

        self.joint_names = ['UR5_joint%i' % ii
                            for ii in range(self.num_joints)]

        # for the null space controller, keep arm near these angles
        self.rest_angles = np.array([0, 45, -90, 45, 90, 0])

        # create the inertia matrices for each link of the ur5
        self._M.append(np.diag([1.0, 1.0, 1.0,
                                     0.02, 0.02, 0.02]))  # link0
        self._M.append(np.diag([2.5, 2.5, 2.5,
                                     0.04, 0.04, 0.04]))  # link1
        self._M.append(np.diag([5.7, 5.7, 5.7,
                                     0.06, 0.06, 0.04]))  # link2
        self._M.append(np.diag([3.9, 3.9, 3.9,
                                     0.055, 0.055, 0.04]))  # link3
        self._M.append(np.copy(self._M[1]))  # link4
        self._M.append(np.copy(self._M[1]))  # link5
        self._M.append(np.diag([0.7, 0.7, 0.7,
                                     0.01, 0.01, 0.01]))  # link6

        # segment lengths associated with each joint
        L = np.array([0.0935, 0.13453, 0.4251,
                      0.12, 0.3921, 0.0935, 0.0935, 0.0935])

        # transform matrix from origin to joint 0 reference frame
        # link 0 reference frame is the same as joint 0
        self.T0org = sp.Matrix([[sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
                                [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
                                [0, 0, 1, L[0]],
                                [0, 0, 0, 1]])

        # transform matrix from joint 0 to joint 1 reference frame
        # link 1 reference frame is the same as joint 1
        self.T10 = sp.Matrix([[1, 0, 0, -L[1]],
                              [0, sp.cos(-self.q[1] + sp.pi/2),
                               -sp.sin(-self.q[1] + sp.pi/2), 0],
                              [0, sp.sin(-self.q[1] + sp.pi/2),
                               sp.cos(-self.q[1] + sp.pi/2), 0],
                              [0, 0, 0, 1]])

        # transform matrix from joint 1 to joint 2 reference frame
        self.T21 = sp.Matrix([[1, 0, 0, 0],
                              [0, sp.cos(-self.q[2]),
                               -sp.sin(-self.q[2]), L[2]],
                              [0, sp.sin(-self.q[2]),
                               sp.cos(-self.q[2]), 0],
                              [0, 0, 0, 1]])

        # transform matrix from joint 1  to link 2
        self.Tl21 = sp.Matrix([[1, 0, 0, 0],
                               [0, sp.cos(-self.q[2]),
                                -sp.sin(-self.q[2]), L[2] / 2],
                               [0, sp.sin(-self.q[2]),
                                sp.cos(-self.q[2]), 0],
                               [0, 0, 0, 1]])

        # transform matrix from joint 2 to joint 3
        self.T32 = sp.Matrix([[1, 0, 0, L[3]],
                              [0, sp.cos(-self.q[3] - sp.pi/2),
                               -sp.sin(-self.q[3] - sp.pi/2), L[4]],
                              [0, sp.sin(-self.q[3] - sp.pi/2),
                               sp.cos(-self.q[3] - sp.pi/2), 0],
                              [0, 0, 0, 1]])

        # transform matrix from joint 2 to link 3
        self.Tl32 = sp.Matrix([[1, 0, 0, L[3]],
                               [0, sp.cos(-self.q[3] - sp.pi/2),
                                -sp.sin(-self.q[3] - sp.pi/2), L[4] / 2],
                               [0, sp.sin(-self.q[3] - sp.pi/2),
                                sp.cos(-self.q[3] - sp.pi/2), 0],
                               [0, 0, 0, 1]])

        # transform matrix from joint 3 to joint 4
        self.T43 = sp.Matrix([[sp.sin(-self.q[4] - sp.pi/2),
                               sp.cos(-self.q[4] - sp.pi/2), 0, -L[5]],
                              [sp.cos(-self.q[4] - sp.pi/2),
                               -sp.sin(-self.q[4] - sp.pi/2), 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # transform matrix from joint 4 to joint 5
        self.T54 = sp.Matrix([[1, 0, 0, 0],
                              [0, sp.cos(self.q[5]), -sp.sin(self.q[5]), 0],
                              [0, sp.sin(self.q[5]), sp.cos(self.q[5]), L[6]],
                              [0, 0, 0, 1]])

        # transform matrix from joint 5 to end-effector
        self.TEE5 = sp.Matrix([[0, 0, 0, L[7]],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[0, 0, 10],  # joint 0 rotates around z axis
                              [10, 0, 0],  # joint 1 rotates around x axis
                              [10, 0, 0],  # joint 2 rotates around x axis
                              [10, 0, 0],  # joint 3 rotates around x axis
                              [0, 0, 10],  # joint 4 rotates around z axis
                              [1, 0, 0]]  # joint 5 rotates around x axis

        # # create placeholder for Coriolis and centrifugal term
        # self._C = None

    def _calc_T(self, name, lambdify=True):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        # check to see if we have our transformation saved in file
        if os.path.isfile('%s/%s.T' % (self.config_folder, name)):
            Tx = cloudpickle.load(open('%s/%s.T' % (self.config_folder, name),
                                       'rb'))
        else:
            if name == 'joint0' or name == 'link0':
                T = self.T0org
            elif name == 'joint1' or name == 'link1':
                T = self.T0org * self.T10
            elif name == 'joint2':
                T = self.T0org * self.T10 * self.T21
            elif name == 'link2':
                T = self.T0org * self.T10 * self.Tl21
            elif name == 'joint3':
                T = self.T0org * self.T10 * self.T21 * self.T32
            elif name == 'link3':
                T = self.T0org * self.T10 * self.T21 * self.Tl32
            elif name == 'joint4' or name == 'link4':
                T = self.T0org * self.T10 * self.T21 * self.T32 * self.T43
            elif name == 'joint5' or name == 'link5':
                T = self.T0org * self.T10 * self.T21 * self.T32 * self.T43 * \
                    self.T54
            elif name == 'link6' or name == 'EE':
                T = self.T0org * self.T10 * self.T21 * self.T32 * self.T43 * \
                    self.T54 * self.TEE5
            Tx = T * self.x  # to convert from transform matrix to (x,y,z)

            # save to file
            cloudpickle.dump(Tx, open('%s/%s.T' % (self.config_folder, name),
                                      'wb'))

        if lambdify is False:
            return Tx
        return sp.lambdify(self.q, Tx)

    # def C(self, q, dq):
    #     """ Calculates the Coriolis and centrifugal effects for the ur5
    #
    #     q np.array: joint angles
    #     """
    #     # check for function in dictionary
    #     if self._C is None:
    #         print('Generating Coriolis and centrifugal effects function')
    #         self._C = self.calc_C()
    #     return np.array(self._C(*q, *dq))
    #
    # def calc_C(self, lambdify=True):
    #     """ Uses Sympy to generate the coriolis effects in
    #     joint space for the ur5
    #
    #     lambdify boolean: if True returns a function to calculate
    #                       the Jacobian. If False returns the Sympy
    #                       matrix
    #     """
    #
    #     # check to see if we have our inertia matrix saved in file
    #     if os.path.isfile('ur5_TnJ/C'):
    #         C = cloudpickle.load(open('ur5_TnJ/C', 'rb'))
    #     else:
    #         # from Improvements to the Adaptive Slotine & Li Controller
    #         # by Rudas and Gati
    #
    #         # TODO: Is this the full term or just part of the term?
    #
    #         # C_ij = 1/2 * sum_k dq_k * (-dM_kj/dq_i + dM_ij/dq_k + dM_iz/dq_j)
    #         Mq = self.calc_Mq(lambdify=False)
    #         C = sp.zeros(6)
    #         for ii in range(6):
    #             for jj in range(6):
    #                 for kk in range(self.num_joints):
    #                     C[ii, jj] += (.5 * self.dq[kk] *
    #                                   -Mq[kk, jj].diff(self.q[ii]) +
    #                                   Mq[ii, jj].diff(self.q[kk]) +
    #                                   Mq[ii, kk].diff(self.q[jj]))
    #
    #         # save to file
    #         cloudpickle.dump(Mq, open('ur5_TnJ/C', 'wb'))
    #
    #     if lambdify is False:
    #         return C
    #     return sp.lambdify(self.q + self.dq, C)

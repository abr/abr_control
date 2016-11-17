import cloudpickle
import os
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=6, num_links=7,
                                           robot_name='ur5', **kwargs)

        self.joint_names = ['UR5_joint%i' % ii
                            for ii in range(self.num_joints)]

        # for the null space controller, keep arm near these angles
        self.rest_angles = np.array([None,
                                     np.pi/4.0,
                                     -np.pi/2.0,
                                     np.pi/4.0,
                                     np.pi/2.0,
                                     np.pi/2.0])

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
        self.L = np.array([0.0935, 0.13453, 0.4251,
                           0.12, 0.3921, 0.0935, 0.0935, 0.0935])

        # transform matrix from origin to joint 0 reference frame
        # link 0 reference frame is the same as joint 0
        self.T0org = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, self.L[0]],
            [0, 0, 0, 1]])

        # transform matrix from joint 0 to joint 1 reference frame
        # link 1 reference frame is the same as joint 1
        self.T10 = sp.Matrix([
            [1, 0, 0, -self.L[1]],
            [0, sp.cos(-self.q[1] + sp.pi/2),
             -sp.sin(-self.q[1] + sp.pi/2), 0],
            [0, sp.sin(-self.q[1] + sp.pi/2),
             sp.cos(-self.q[1] + sp.pi/2), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1 to joint 2 reference frame
        self.T21 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[2]),
             -sp.sin(-self.q[2]), self.L[2]],
            [0, sp.sin(-self.q[2]),
             sp.cos(-self.q[2]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1  to link 2
        self.Tl21 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[2]),
             -sp.sin(-self.q[2]), self.L[2] / 2],
            [0, sp.sin(-self.q[2]),
             sp.cos(-self.q[2]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to joint 3
        self.T32 = sp.Matrix([
            [1, 0, 0, self.L[3]],
            [0, sp.cos(-self.q[3] - sp.pi/2),
             -sp.sin(-self.q[3] - sp.pi/2), self.L[4]],
            [0, sp.sin(-self.q[3] - sp.pi/2),
             sp.cos(-self.q[3] - sp.pi/2), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to link 3
        self.Tl32 = sp.Matrix([
            [1, 0, 0, self.L[3]],
            [0, sp.cos(-self.q[3] - sp.pi/2),
             -sp.sin(-self.q[3] - sp.pi/2), self.L[4] / 2],
            [0, sp.sin(-self.q[3] - sp.pi/2),
             sp.cos(-self.q[3] - sp.pi/2), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 3 to joint 4
        self.T43 = sp.Matrix([
            [sp.sin(-self.q[4] - sp.pi/2),
             sp.cos(-self.q[4] - sp.pi/2), 0, -self.L[5]],
            [sp.cos(-self.q[4] - sp.pi/2),
             -sp.sin(-self.q[4] - sp.pi/2), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 4 to joint 5
        self.T54 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[5]), -sp.sin(-self.q[5]), 0],
            [0, sp.sin(-self.q[5]), sp.cos(-self.q[5]), self.L[6]],
            [0, 0, 0, 1]])

        # transform matrix from joint 5 to end-effector
        self.TEE5 = sp.Matrix([
            [1, 0, 0, self.L[7]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[0, 0, 10],  # joint 0 rotates around z axis
                              [10, 0, 0],  # joint 1 rotates around x axis
                              [10, 0, 0],  # joint 2 rotates around x axis
                              [10, 0, 0],  # joint 3 rotates around x axis
                              [0, 0, 10],  # joint 4 rotates around z axis
                              [1, 0, 0]]  # joint 5 rotates around x axis

    def _calc_T(self, name, lambdify=True, regenerate=False):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        if name == 'link0':
            T = np.eye(4)
        elif name == 'joint0' or name == 'link1':
            T = self.T0org
        elif name == 'joint1':
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
            T = (self.T0org * self.T10 * self.T21 * self.T32 * self.T43 *
                    self.T54)
        elif name == 'link6' or name == 'EE':
            T = (self.T0org * self.T10 * self.T21 * self.T32 * self.T43 *
                    self.T54 * self.TEE5)
        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the onelink arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=1, num_links=1,
                                           robot_name='onelink', **kwargs)

        self.joint_names = ['joint0']
        self.rest_angles = np.array([90.0])

        # create the inertia matrices for each link of the ur5
        self._M.append(np.diag([1.0, 1.0, 1.0,
                                0.02, 0.02, 0.02]))  # link0

        # segment lengths associated with each joint
        self.L = np.array([
            [0.0, 0.0, 0.15],
            [0.37, 0.0, 0.0]],
            dtype='float32')

        self.L_com = np.array([
            [0.0, 0.0, .05],
            [0.22, 0.0, 0.0]],
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # transform matrix from origin to joint 0 reference frame
        # link 0 reference frame is the same as joint 0
        self.Torg0 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, self.L[0, 2]],
            [0, 0, 0, 1]])

        # transform to move to end-effector position
        self.T0EEa = sp.Matrix([
            [sp.sin(sp.pi - self.q[0]), sp.cos(sp.pi - self.q[0]), 0,
             self.L[1, 0] * sp.cos(sp.pi - self.q[0])],
            [sp.cos(sp.pi - self.q[0]), -sp.sin(sp.pi - self.q[0]), 0,
             self.L[1, 0] * sp.sin(sp.pi - self.q[0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # transform to match orientation to link 1
        self.T0EEb = sp.Matrix([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]])

        self.T0EE = self.T0EEa * self.T0EEb

        # ---- COM Transform Matrices ----

        self.Tlorg0 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_com[0, 2]],
            [0, 0, 0, 1]])

        self.Tl01 = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0,
             self.L_com[1, 0] * sp.cos(self.q[0])],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0,
             self.L_com[1, 0] * sp.sin(self.q[0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[0, 0, 1]]  # joint 0 rotates around z axis

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if name == 'link0':
            T = self.Tlorg0
        elif name == 'joint0':
            T = self.Torg0
        elif name == 'link1':
            T = self.Torg0 * self.Tl01
        elif name == 'EE':
            T = self.Torg0 * self.T0EE
        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

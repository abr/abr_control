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
        self._M_links.append(np.diag([1.0, 1.0, 1.0,
                                      0.02, 0.02, 0.02]))  # link0

        # the joints don't weigh anything
        self._M_joints = [sp.zeros(6, 6) for ii in range(self.num_joints)]

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
        q0 = self.q[0]
        self.T0EE = sp.Matrix([
            [sp.cos(q0), -sp.sin(q0), 0,
             self.L[1, 0] * sp.cos(q0)],
            [sp.sin(q0), sp.cos(q0), 0,
             self.L[1, 0] * sp.sin(q0)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # ---- COM Transform Matrices ----

        self.Tlorg0 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_com[0, 2]],
            [0, 0, 0, 1]])

        self.Tl01a = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0,
             self.L_com[1, 0] * sp.cos(self.q[0])],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0,
             self.L_com[1, 0] * sp.sin(self.q[0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # transform to match orientation to link 1
        self.Tl01b = sp.Matrix([
            [0, 0, 1,
             self.L_com[1, 0] * sp.cos(self.q[0])],
            [0, 1, 0,
             self.L_com[1, 0] * sp.sin(self.q[0])],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]])
        self.Tl01 = self.Tl01b #self.Tl01a * self.Tl01b

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

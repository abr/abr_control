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
                                     np.pi/2.0], dtype='float32')

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
        self.L = np.array([
            [0.0, 0.0, 0.0935],
            [0.13453, 0.0, 0.0],
            [0.0, 0.4251, 0.0],
            [0.12, 0.3921, 0.0],
            [0.0935, 0.0, 0.0],
            [0.0, 0.0, 0.0935],
            [0.0935, 0.0, 0.0]],
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> joint 0
        # link 0 reference frame is the same as joint 0
        self.Torg0 = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> joint 1
        self.T01 = sp.Matrix([
            [1, 0, 0, -self.L[1, 0]],
            [0, sp.cos(-self.q[1] + sp.pi/2),
             -sp.sin(-self.q[1] + sp.pi/2), 0],
            [0, sp.sin(-self.q[1] + sp.pi/2),
             sp.cos(-self.q[1] + sp.pi/2), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> joint 2
        self.T12 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[2]),
             -sp.sin(-self.q[2]), self.L[2, 1]],
            [0, sp.sin(-self.q[2]),
             sp.cos(-self.q[2]), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> joint 3
        self.T23 = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, sp.cos(-self.q[3] - sp.pi/2),
             -sp.sin(-self.q[3] - sp.pi/2), self.L[3, 1]],
            [0, sp.sin(-self.q[3] - sp.pi/2),
             sp.cos(-self.q[3] - sp.pi/2), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 3 -> joint 4
        self.T34 = sp.Matrix([
            [sp.sin(-self.q[4] - sp.pi/2),
             sp.cos(-self.q[4] - sp.pi/2), 0, -self.L[4, 0]],
            [sp.cos(-self.q[4] - sp.pi/2),
             -sp.sin(-self.q[4] - sp.pi/2), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 4 -> joint 5
        self.T45 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[5]), -sp.sin(-self.q[5]), 0],
            [0, sp.sin(-self.q[5]), sp.cos(-self.q[5]), self.L[5, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 5 -> end-effector
        self.T5EE = sp.Matrix([
            [1, 0, 0, self.L[6, 0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # ---- COM Transform Matrices ----

        # NOTE: link 1 reference frame is the same as joint 1

        # Transform matrix : joint 1 -> link 2
        self.Tl21 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(-self.q[2]),
             -sp.sin(-self.q[2]), self.L[2, 1] / 2],
            [0, sp.sin(-self.q[2]),
             sp.cos(-self.q[2]), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        self.Tl23 = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, sp.cos(-self.q[3] - sp.pi/2),
             -sp.sin(-self.q[3] - sp.pi/2), self.L[3, 1] / 2],
            [0, sp.sin(-self.q[3] - sp.pi/2),
             sp.cos(-self.q[3] - sp.pi/2), 0],
            [0, 0, 0, 1]])

        # NOTE: link 4 reference frame is the same as joint 4

        # NOTE: link 5 reference frame is the same as joint 5

        # NOTE: link 6 reference frame is the same as EE

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[0, 0, 10],  # joint 0 rotates around z axis
                              [10, 0, 0],  # joint 1 rotates around x axis
                              [10, 0, 0],  # joint 2 rotates around x axis
                              [10, 0, 0],  # joint 3 rotates around x axis
                              [0, 0, 10],  # joint 4 rotates around z axis
                              [1, 0, 0]]  # joint 5 rotates around x axis

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if name == 'link0':
            T = sp.eye(4)
        elif name == 'joint0' or name == 'link1':
            T = self.Torg0
        elif name == 'joint1':
            T = self.Torg0 * self.T01
        elif name == 'joint2':
            T = self.Torg0 * self.T01 * self.T12
        elif name == 'link2':
            T = self.Torg0 * self.T01 * self.Tl21
        elif name == 'joint3':
            T = self.Torg0 * self.T01 * self.T12 * self.T23
        elif name == 'link3':
            T = self.Torg0 * self.T01 * self.T12 * self.Tl23
        elif name == 'joint4' or name == 'link4':
            T = self.Torg0 * self.T01 * self.T12 * self.T23 * self.T34
        elif name == 'joint5' or name == 'link5':
            T = (self.Torg0 * self.T01 * self.T12 * self.T23 * self.T34 *
                 self.T45)
        elif name == 'EE' or name == 'link6':
            T = (self.Torg0 * self.T01 * self.T12 * self.T23 * self.T34 *
                 self.T45 * self.T5EE)
        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

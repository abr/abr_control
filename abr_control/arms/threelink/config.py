import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the threelink MapleSim arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=3, num_links=3,
                                           robot_name='threelink', **kwargs)

        # for the null space controller, keep arm near these angles
        self.rest_angles = np.array([np.pi/4.0, np.pi/4.0, np.pi/4.0],
                                    dtype='float32')

        # create the inertia matrices for each link of the threelink
        # TODO: confirm that these are actually the right values
        # self._M.append(np.diag([1.93, 1.93, 1.93,
        #                         0.0141, 0.0141, 0.0141]) * 10)  # link0
        # self._M.append(np.diag([0.27, 0.27, 0.27,
        #                         0.012, 0.012, 0.012]) * 10)  # link1
        # self._M.append(np.diag([0.15, 0.15, 0.15,
        #                         0.001, 0.001, 0.001]) * 10)  # link2
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link0
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link1
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link2

        # segment lengths associated with each joint
        # [x, y, z],  Ignoring lengths < 1e-04

        self.L = np.array([
            [0, 0, 0],
            [2.0, 0, 0],
            [1.2, 0, 0],
            [0.7, 0, 0]],
            dtype='float32')
        self.L_com = self.L / 2.0

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> joint 0
        self.Torg0 = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> joint 1
        self.T01 = sp.Matrix([
            [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, self.L[1, 0]],
            [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> joint 2
        self.T12 = sp.Matrix([
            [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, self.L[2, 0]],
            [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> EE
        self.T2EE = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # ---- COM Transform Matrices ----

        # NOTE: following convention for the rest of the repo,
        # joint 0 is mounted to the end of link 0
        self.Tlorg0 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        self.Tl01 = sp.Matrix([
            [1, 0, 0, self.L_com[1, 0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> link 2
        self.Tl12 = sp.Matrix([
            [1, 0, 0, self.L_com[2, 0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        self.Tl23 = sp.Matrix([
            [1, 0, 0, self.L_com[3, 0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian
        # accounting for which axis is rotated around at each joint
        self.J_orientation = [[0.0, 0.0, 1.0],  # joint 0 rotates around z axis
                              [0.0, 0.0, 1.0],  # joint 1 rotates around z axis
                              [0.0, 0.0, 1.0]]  # joint 2 rotates around z axis

    def _calc_T(self, name):
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        # ---- Joint Transforms ----

        if name == 'joint0':
            T = self.Torg0
        elif name == 'joint1':
            T = self.Torg0 * self.T01
        elif name == 'joint2':
            T = self.Torg0 * self.T01 * self.T12
        elif name == 'EE':
            T = self.Torg0 * self.T01 * self.T12 * self.T2EE

        # ---- COM Transforms ----

        elif name == 'link0':
            T = self.Tlorg0
        elif name == 'link1':
            T = self.Torg0 * self.Tl01
        elif name == 'link2':
            T = self.Torg0 * self.T01 * self.Tl12
        elif name == 'link3':
            T = self.Torg0 * self.T01 * self.T12 * self.Tl23
        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

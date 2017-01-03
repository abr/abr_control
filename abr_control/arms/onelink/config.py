import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the onelink arm """

    def __init__(self):

        super(robot_config, self).__init__(num_joints=1, num_links=1,
                                           robot_name='onelink')

        self.joint_names = ['joint0']
        self.rest_angles = np.array([90.0])

        # create the inertia matrices for each link of the ur5
        self._M.append(np.diag([1.0, 1.0, 1.0,
                                0.02, 0.02, 0.02]))  # link0

        # segment lengths associated with each joint
        self.L = np.array([
            [0.37, 0.0, 0.0]],
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # transform matrix from origin to joint 0 reference frame
        # link 0 reference frame is the same as joint 0
        self.Torg0 = sp.Matrix([
            [sp.cos(self.q[0]), 0, -sp.sin(self.q[0]), 0],
            [0, 1, 0, 0],
            [sp.sin(self.q[0]), 0, sp.cos(self.q[0]), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> EE
        self.T0EE = sp.Matrix([[1, 0, 0, self.L[0, 0]],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

        # ---- COM Transform Matrices ----

        # Transform matrix : joint 0 to link1
        self.Tl01 = sp.Matrix([[1, 0, 0, self.L[0, 0] / 2],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[10, 0, 0]]  # joint 0 rotates around z axis

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if name == 'joint0':
            T = self.Torg0
        elif name == 'link0':
            T = self.Torg0 * self.Tl01
        elif name == 'EE':
            T = self.Torg0 * self.T0EE
        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

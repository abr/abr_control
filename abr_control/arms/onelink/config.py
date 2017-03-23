import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the onelink arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=1, num_links=1,
                                           robot_name='onelink', **kwargs)

        self._T = {}  # dictionary for storing calculated transforms

        self.joint_names = ['joint0']
        self.rest_angles = np.array([np.pi/2.0])

        # create the inertia matrices for each link of the ur5
        self._M_links.append(np.diag([1.0, 1.0, 1.0,
                                      0.02, 0.02, 0.02]))  # link0

        # the joints don't weigh anything
        self._M_joints = [sp.zeros(6, 6) for ii in range(self.num_joints)]

        # segment lengths associated with each joint
        self.L = np.array([
            [0.0, 0.0, 0.05],
            [0.0, 0.0, 0.05],
            [0.22, 0.0, 0.0],
            [0.0, 0.0, .15]],
            dtype='float32')

        # ---- Transform Matrices ----

        # Transform matrix : origin -> link 0
        # account for axes change and offsets
        self.Torgl0 = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, 1, 0, self.L[0, 1]],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : link 0 -> joint 0
        # account for axes change and offsets
        self.Tl0j0 = sp.Matrix([
            [1, 0, 0, self.L[1, 0]],
            [0, 0, -1, self.L[1, 1]],
            [0, 1, 0, self.L[1, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        # account for rotation due to q
        self.Tj0l1a = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for change of axes and offsets
        self.Tj0l1b = sp.Matrix([
            [0, 0, 1, self.L[2, 0]],
            [0, 1, 0, self.L[2, 1]],
            [-1, 0, 0, self.L[2, 2]],
            [0, 0, 1, 0]])
        self.Tj0l1 = self.Tj0l1a * self.Tj0l1b

        # Transform matrix : link 1 -> end-effector
        self.Tl1ee = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, 1, 0, self.L[3, 1]],
            [0, 0, 1, self.L[3, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for angular velocity)
        kz = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * kz]  # joint 0 orientation

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if self._T.get(name, None) is None:
            if name == 'link0':
                self._T[name] = self.Torgl0
            elif name == 'joint0':
                self._T[name] = self._calc_T('link0') * self.Tl0j0
            elif name == 'link1':
                self._T[name] = self._calc_T('joint0') * self.Tj0l1
            elif name == 'EE':
                self._T[name] = self._calc_T('link1') * self.Tl1ee

            else:
                raise Exception('Invalid transformation name: %s' % name)

        return self._T[name]

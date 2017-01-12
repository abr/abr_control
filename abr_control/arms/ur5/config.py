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

        # TODO: automate getting all this information from VREP

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

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = np.array([
            [0.0, 0.0, 1.4650e-02],  # link 0 offset
            [0.0, 0.0, 8.5001e-03],  # joint 0 offset
            [-7.1771e-03, 1.1159e-04, 7.0381e-02],  # link 1 offset
            [-6.3122e-02, -9.5099e-05, -4.3305e-03],  # joint 1 offset
            [2.1255e-01, -9.9446e-04, 6.4234e-02],  # link 2 offset
            [6.4235e-02, 1.1502e-04, 2.1255e-01],  # joint 2 offset
            [1.8677e-01, 6.7934e-04, -5.7847e-02],  # link 3 offset
            [-5.7847e-02, -1.6153e-05, 2.0538e-01],  # joint 3 offset
            [-7.5028e-03, -5.5328e-05, 3.2830e-02],  # link 4 offset
            [-6.8700e-03, 4.5318e-05, 5.3076e-02],  # joint 4 offset
            [3.6091e-03, 5.0090e-05, 4.2340e-02],  # link 5 offset
            [1.0824e-02, -4.5293e-05, 6.8700e-03],  # joint 5 offset
            [0, 0, 7.6645e-02]],  # link 6 offset
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, 1, 0, self.L[0, 1]],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : link 0 -> joint 0
        # no change of axes, account for offsets
        self.Tl0j0 = sp.Matrix([
            [1, 0, 0, self.L[1, 0]],
            [0, 1, 0, self.L[1, 1]],
            [0, 0, 1, self.L[1, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        # account for rotations due to q
        self.Tj0l1a = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # no change of axes, account for offsets
        self.Tj0l1b = sp.Matrix([
            [1, 0, 0, self.L[2, 0]],
            [0, 1, 0, self.L[2, 1]],
            [0, 0, 1, self.L[2, 2]],
            [0, 0, 0, 1]])
        self.Tj0l1 = self.Tj0l1a * self.Tj0l1b

        # Transform matrix : link 1 -> joint 1
        # account for axes rotation and offset
        self.Tl1j1 = sp.Matrix([
            [0, 0, -1, self.L[3, 0]],
            [0, 1, 0, self.L[3, 1]],
            [1, 0, 0, self.L[3, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> link 2
        # account for rotations due to q
        self.Tj1l2a = sp.Matrix([
            [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, 0],
            [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes rotation and offsets
        self.Tj1l2b = sp.Matrix([
            [0, 0, 1, self.L[4, 0]],
            [0, 1, 0, self.L[4, 1]],
            [-1, 0, 0, self.L[4, 2]],
            [0, 0, 0, 1]])
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

        # Transform matrix : link 2 -> joint 2
        # account for axes rotation and offsets
        self.Tl2j2 = sp.Matrix([
            [0, 0, -1, self.L[5, 0]],
            [0, 1, 0, self.L[5, 1]],
            [1, 0, 0, self.L[5, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        # account for rotations due to q
        self.Tj2l3a = sp.Matrix([
            [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
            [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes rotation and offsets
        self.Tj2l3b = sp.Matrix([
            [0, 0, 1, self.L[6, 0]],
            [0, 1, 0, self.L[6, 1]],
            [-1, 0, 0, self.L[6, 2]],
            [0, 0, 0, 1]])
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

        # Transform matrix : link 3 -> joint 3
        # account for axes change and offsets
        self.Tl3j3 = sp.Matrix([
            [0, 0, -1, self.L[7, 0]],
            [0, 1, 0, self.L[7, 1]],
            [1, 0, 0, self.L[7, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 3 -> link 4
        # account for rotations due to q
        self.Tj3l4a = sp.Matrix([
            [sp.cos(self.q[3]), -sp.sin(self.q[3]), 0, 0],
            [sp.sin(self.q[3]), sp.cos(self.q[3]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes and rotation and offsets
        self.Tj3l4b = sp.Matrix([
            [0, 0, 1, self.L[8, 0]],
            [0, 1, 0, self.L[8, 1]],
            [-1, 0, 0, self.L[8, 2]],
            [0, 0, 0, 1]])
        self.Tj3l4 = self.Tj3l4a * self.Tj3l4b

        # Transform matrix: link 4 -> joint 4
        # no axes change, account for offsets
        self.Tl4j4 = sp.Matrix([
            [1, 0, 0, self.L[9, 0]],
            [0, 1, 0, self.L[9, 1]],
            [0, 0, 1, self.L[9, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 4 -> link 5
        # account for rotations due to q
        self.Tj4l5a = sp.Matrix([
            [sp.cos(self.q[4]), -sp.sin(self.q[4]), 0, 0],
            [sp.sin(self.q[4]), sp.cos(self.q[4]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for axes and rotation and offsets
        # no axes change, account for offsets
        self.Tj4l5b = sp.Matrix([
            [1, 0, 0, self.L[10, 0]],
            [0, 1, 0, self.L[10, 1]],
            [0, 0, 1, self.L[10, 2]],
            [0, 0, 0, 1]])
        self.Tj4l5 = self.Tj4l5a * self.Tj4l5b

        # Transform matrix : link 5 -> joint 5
        # account for axes change and offsets
        self.Tl5j5 = sp.Matrix([
            [0, 0, -1, self.L[11, 0]],
            [0, 1, 0, self.L[11, 1]],
            [1, 0, 0, self.L[11, 2]],
            [0, 0, 0, 1]])

        # Transform matrix: joint 5 -> link 6
        # account for rotations due to q
        self.Tj5l6a = sp.Matrix([
            [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
            [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # no axes change, account for offsets
        self.Tj5l6b = sp.Matrix([
            [1, 0, 0, self.L[12, 0]],
            [0, 1, 0, self.L[12, 1]],
            [0, 0, 1, self.L[12, 2]],
            [0, 0, 0, 1]])
        self.Tj5l6 = self.Tj5l6a * self.Tj5l6b

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
            T = self.Torgl0
        elif name == 'joint0':
            T = self.Torgl0 * self.Tl0j0
        elif name == 'link1':
            T = self.Torgl0 * self.Tl0j0 * self.Tj0l1
        elif name == 'joint1':
            T = self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1
        elif name == 'link2':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2)
        elif name == 'joint2':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2)
        elif name == 'link3':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3)
        elif name == 'joint3':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3)
        elif name == 'link4':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                 self.Tj3l4)
        elif name == 'joint4':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                 self.Tj3l4 * self.Tl4j4)
        elif name == 'link5':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                 self.Tj3l4 * self.Tl4j4 * self.Tj4l5)
        elif name == 'joint5':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                 self.Tj3l4 * self.Tl4j4 * self.Tj4l5 * self.Tl5j5)
        elif name == 'link6' or name == 'EE':
            T = (self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                 self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                 self.Tj3l4 * self.Tl4j4 * self.Tj4l5 * self.Tl5j5 *
                 self.Tj5l6)
        else:
            raise Exception('Invalid transformation name: %s' % name)

        print(T)

        return T

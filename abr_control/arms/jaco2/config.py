import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=6, num_links=7,
                                           robot_name='jaco2', **kwargs)

        self.joint_names = ['joint%i' % ii
                            for ii in range(self.num_joints)]

        # for the null space controller, keep arm near these angles
        # TODO: fill in rest angles

        # create the inertia matrices for each link of the ur5
        self._M.append(np.diag([0.5, 0.5, 0.5,
                                0.01, 0.01, 0.01]))  # link0

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = np.array([
            [0.0, 0.0, 7.8369e-02],  # link 0 offset
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 0 offset
            [-2.2084e-05, 1.3257e-04, -7.9887e-02],  # link 1 offset
            [-2.2042e-05, 1.3245e-04, -3.8863e-02],  # joint 1 offset
            [-1.0869e-06, 2.0499e-01, -2.3228e-02],  # link 2 offset
            [-2.3094e-02, -1.0980e-06, 2.0503e-01],  # joint 2 offset
            [-1.5273e-03, -8.3337e-02, -1.2206e-02],  # link 3 offset
            [2.5923e-04, -3.8935e-03, -1.2393e-01],  # joint 3 offset
            [-4.7767e-04, 1.2373e-02, -3.5387e-02],  # link 4 offset
            [-2.3603e-03, -4.8662e-03, 3.7097e-02],  # joint 4 offset
            [-5.3939e-04, 1.2305e-02, -3.5455e-02],  # link 5 offset
            [-1.9534e-03, 5.0298e-03, -3.7176e-02],  # joint 5 offset
            [-3.8644e-05, 7.3135e-05, 3.2783e-07]],  # link 6 offset
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, 1, 0, self.L[0, 1]],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : origin -> joint 0
        # account for change of axes and offsets
        self.Tl0j0 = sp.Matrix([
            [1, 0, 0, self.L[1, 0]],
            [0, -1, 0, self.L[1, 1]],
            [0, 0, -1, self.L[1, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        # account for rotations due to q
        self.Tj0l1a = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # account for change of axes and offsets
        self.Tj0l1b = sp.Matrix([
            [-1, 0, 0, self.L[2, 0]],
            [0, -1, 0, self.L[2, 1]],
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
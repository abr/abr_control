# Config file for the first 2 links of the Jaco 2 in VREP
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the Kinova Jaco^2 V2"""

    def __init__(self, hand_attached=False, **kwargs):

        num_links = 5
        super(robot_config, self).__init__(num_joints=4, num_links=num_links,
                                           robot_name='jaco2', **kwargs)

        self._T = {}  # dictionary for storing calculated transforms

        self.joint_names = ['joint%i' % ii
                            for ii in range(self.num_joints)]

        # Kinova Home Position - straight up
        self.home_position = np.array([250.0, 180.0, 180.0, 270.0], dtype="float32")

        # for the null space controller, keep arm near these angles
        # currently set to the center of the limits
        self.rest_angles = np.array([0.0, 140.0, 140.0, 0.0],
                                     dtype='float32') * np.pi / 180

        # TODO: check if using sp or np diag makes a difference
        # create the inertia matrices for each link of the ur5
        self._M_links = [
            sp.diag(0.5, 0.5, 0.5, 0.02, 0.02, 0.02),  # link0
            sp.diag(0.5, 0.5, 0.5, 0.02, 0.02, 0.02),  # link1
            sp.diag(0.50025, 0.50025, 0.50025, 0.02, 0.02, 0.02),  # link2
            sp.diag(0.50025, 0.50025, 0.50025, 0.02, 0.02, 0.02),  # link3
            sp.diag(0.499, 0.499, 0.499, 0.02, 0.02, 0.02)]  # link4

        # the joints don't weigh anything in VREP
        self._M_joints = [sp.zeros(6, 6) for ii in range(self.num_joints)]

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = np.array([
            [0.0, 0.0, 7.8369e-02],  # link 0 offset
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 0 offset
            [2.1217e-05, 4.8455e-05, -7.9515e-02],  # link 1 offset
            [-2.2042e-05, 1.3245e-04, -3.8863e-02],  # joint 1 offset
            [-1.9519e-03, 2.0902e-01, -2.8839e-02],  # link 2 offset
            [-2.3094e-02, -1.0980e-06, 2.0503e-01],  # joint 2 offset
            [-4.8786e-04, -8.1945e-02, -1.2931e-02],  # link 3 offset
            [2.5923e-04, -3.8935e-03, -1.2393e-01],  # joint 3 offset
            [-4.0053e-04, 1.2581e-02, -3.5270e-02]])  # link 4 offset

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, 1, 0, self.L[0, 1]],
            [0, 0, 1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : link0 -> joint 0
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
            [1, 0, 0, self.L[3, 0]],
            [0, 0, -1, self.L[3, 1]],
            [0, 1, 0, self.L[3, 2]],
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
            [0, -1, 0, self.L[4, 0]],
            [0, 0, 1, self.L[4, 1]],
            [-1, 0, 0, self.L[4, 2]],
            [0, 0, 0, 1]])
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

         # Transform matrix : link 2 -> joint 2
        # account for axes rotation and offsets
        self.Tl2j2 = sp.Matrix([
            [0, 0, 1, self.L[5, 0]],
            [1, 0, 0, self.L[5, 1]],
            [0, 1, 0, self.L[5, 2]],
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
            [0.14262926, -0.98977618, 0, self.L[6, 0]],
            [0, 0, 1, self.L[6, 1]],
            [-0.98977618, -0.14262926, 0, self.L[6, 2]],
            [0, 0, 0, 1]])
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

        # Transform matrix : link 3 -> joint 3
        # account for axes change and offsets
        self.Tl3j3 = sp.Matrix([
            [-0.14262861, -0.98977628, 0, self.L[7, 0]],
            [0.98977628, -0.14262861, 0, self.L[7, 1]],
            [0, 0, 1, self.L[7, 2]],
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
            [0.85536427, -0.51802699, 0, self.L[8, 0]],
            [-0.45991232, -0.75940555,  0.46019982, self.L[8, 1]],
            [-0.23839593, -0.39363848, -0.88781537, self.L[8, 2]],
            [0, 0, 0, 1]])
        self.Tj3l4 = self.Tj3l4a * self.Tj3l4b

        # orientation part of the Jacobian (compensating for orientations)
        kz = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * kz,  # joint 0 angular velocity
            self._calc_T('joint1')[:3, :3] * kz,  # joint 1 angular velocity
            self._calc_T('joint2')[:3, :3] * kz,  # joint 2
            self._calc_T('joint3')[:3, :3] * kz]  # joint 3

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if self._T.get(name, None) is None:
            if name == 'link0':
                self._T[name] = self.Torgl0
            elif name == 'joint0':
                self._T[name] = self.Torgl0 * self.Tl0j0
            elif name == 'link1':
                self._T[name] = self.Torgl0 * self.Tl0j0 * self.Tj0l1
            elif name == 'joint1':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1)
            elif name == 'link2':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2)
            elif name == 'joint2':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2)
            elif name == 'link3':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3)
            elif name == 'joint3':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3)
            elif name == 'link4' or name == 'EE':
                self._T[name] = (
                    self.Torgl0 * self.Tl0j0 * self.Tj0l1 * self.Tl1j1 *
                    self.Tj1l2 * self.Tl2j2 * self.Tj2l3 * self.Tl3j3 *
                    self.Tj3l4)

        return self._T[name]

# Config file for the first 2 links of the Jaco 2 in VREP
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the Kinova Jaco^2 V2"""

    def __init__(self, hand_attached=False, **kwargs):

        num_links = 2
        super(robot_config, self).__init__(num_joints=1, num_links=num_links,
                                           robot_name='jaco2', **kwargs)

        self._T = {}  # dictionary for storing calculated transforms

        self.joint_names = ['joint%i' % ii
                            for ii in range(self.num_joints)]

        # Kinova Home Position - straight up
        self.home_position = np.array([250.0, 180.0], dtype="float32")
        self.home_torques = np.array([-0.138, -0.116], dtype="float32")

        # for the null space controller, keep arm near these angles
        # currently set to the center of the limits
        self.rest_angles = np.array([0.0, 140.0], dtype='float32')

        # TODO: check if using sp or np diag makes a difference
        # create the inertia matrices for each link of the ur5
        self._M_links = [
            sp.diag(0.5, 0.5, 0.5, 0.04, 0.04, 0.04),  # link0
            sp.diag(0.5, 0.5, 0.5, 0.04, 0.04, 0.04)]  # link1

        # the joints don't weigh anything in VREP
        self._M_joints = [sp.zeros(6, 6) for ii in range(self.num_joints)]

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = np.array([
            [0.0, 0.0, 7.8369e-02],  # link 0 offset
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 0 offset
            [2.1217e-05, 4.8455e-05, -7.9515e-02],  # link 1 offset
            [-2.2042e-05, 1.3245e-04, -3.8863e-02]])  # joint 1 offset

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

        # orientation part of the Jacobian (compensating for orientations)
        kz = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * kz]  # joint 0 angular velocity

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """

        if self._T.get(name, None) is None:
            if name == 'link0':
                self._T[name] = self.Torgl0
            elif name == 'joint0':
                self._T[name] = self.Torgl0 * self.Tl0j0
            elif name == 'link1' or name == 'EE':
                self._T[name] = self.Torgl0 * self.Tl0j0 * self.Tj0l1

        return self._T[name]

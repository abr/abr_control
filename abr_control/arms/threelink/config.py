import numpy as np
import sympy as sp

from .. import robot_config


class ThreeLinkConfig(robot_config.RobotConfig):
    """ Robot config file for the threelink MapleSim arm """

    def __init__(self, **kwargs):

        super(RobotConfig, self).__init__(NUM_JOINTS=3, NUM_LINKS=3,
                                           ROBOT_NAME='threelink', **kwargs)

        self._T = {}  # dictionary for storing calculated transforms

        # for the null space controller, keep arm near these angles
        self.REST_ANGLES = np.array([np.pi/4.0, np.pi/4.0, np.pi/4.0],
                                    dtype='float32')

        # create the inertia matrices for each link of the threelink
        # TODO: confirm that these are actually the right values
        # self._M.append(np.diag([1.93, 1.93, 1.93,
        #                         0.0141, 0.0141, 0.0141]) * 10)  # link0
        # self._M.append(np.diag([0.27, 0.27, 0.27,
        #                         0.012, 0.012, 0.012]) * 10)  # link1
        # self._M.append(np.diag([0.15, 0.15, 0.15,
        #                         0.001, 0.001, 0.001]) * 10)  # link2
        self._M_links.append(np.diag([10.0, 10.0, 10.0,
                             0.0, 0.0, 100.0]))  # link0
        self._M_links.append(np.diag([10.0, 10.0, 10.0,
                             0.0, 0.0, 100.0]))  # link1
        self._M_links.append(np.diag([10.0, 10.0, 10.0,
                             0.0, 0.0, 100.0]))  # link2

        # the joints don't weigh anything
        self._M_joints = [sp.zeros(6, 6) for ii in range(self.NUM_JOINTS)]

        # segment lengths associated with each joint
        # [x, y, z],  Ignoring lengths < 1e-04

        self.L = np.array([
            [0, 0, 0],  # from origin to l0 (non-existant)
            [0, 0, 0],  # from l0 to j0
            [1.0, 0, 0],  # from j0 to l1 COM
            [1.0, 0, 0],  # from l1 COM to j1
            [0.6, 0, 0],  # from j1 to l2 COM
            [0.6, 0, 0],  # from l2 COM to j2
            [0.35, 0, 0],  # from j2 to l3 COM
            [0.35, 0, 0]],  # from l3 COM to EE
            dtype='float32')

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
        # account for rotation of q
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
        # no change of axes, account for offsets
        self.Tl1j1 = sp.Matrix([
            [1, 0, 0, self.L[3, 0]],
            [0, 1, 0, self.L[3, 1]],
            [0, 0, 1, self.L[3, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> link 2
        # account for rotation of q
        self.Tj1l2a = sp.Matrix([
            [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, 0],
            [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # no change of axes, account for offsets
        self.Tj1l2b = sp.Matrix([
            [1, 0, 0, self.L[4, 0]],
            [0, 1, 0, self.L[4, 1]],
            [0, 0, 1, self.L[4, 2]],
            [0, 0, 0, 1]])
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

        # Transform matrix : link 2 -> joint 2
        # no change of axes, account for offsets
        self.Tl2j2 = sp.Matrix([
            [1, 0, 0, self.L[5, 0]],
            [0, 1, 0, self.L[5, 1]],
            [0, 0, 1, self.L[5, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        # account for rotation of q
        self.Tj2l3a = sp.Matrix([
            [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
            [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        # no change of axes, account for offsets
        self.Tj2l3b = sp.Matrix([
            [1, 0, 0, self.L[6, 0]],
            [0, 1, 0, self.L[6, 1]],
            [0, 0, 1, self.L[6, 2]],
            [0, 0, 0, 1]])
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

        # Transform matrix : link 3 -> end-effector
        # no change of axes, account for offsets
        self.Tl3ee = sp.Matrix([
            [1, 0, 0, self.L[7, 0]],
            [0, 1, 0, self.L[7, 1]],
            [0, 0, 1, self.L[7, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian
        KZ = sp.Matrix([0, 0, 1])  # all joints rotate around their z axis
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * KZ,  # joint 0 angular velocity
            self._calc_T('joint1')[:3, :3] * KZ,  # joint 1 angular velocity
            self._calc_T('joint2')[:3, :3] * KZ]  # joint 2 angular velocity

    def _calc_T(self, name):
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
            elif name == 'joint1':
                self._T[name] = self._calc_T('link1') * self.Tl1j1
            elif name == 'link2':
                self._T[name] = self._calc_T('joint1') * self.Tj1l2
            elif name == 'joint2':
                self._T[name] = self._calc_T('link2') * self.Tl2j2
            elif name == 'link3':
                self._T[name] = self._calc_T('joint2') * self.Tj2l3
            elif name == 'EE':
                self._T[name] = self._calc_T('link3') * self.Tl3ee

            else:
                raise Exception('Invalid transformation name: %s' % name)

        return self._T[name]

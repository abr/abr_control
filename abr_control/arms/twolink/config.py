import numpy as np
import sympy as sp

from ..base_config import BaseConfig


class Config(BaseConfig):
    """ Robot config file for the twolink Python arm

    Attributes
    ----------
    REST_ANGLES : numpy.array
        the joint angles the arm tries to push towards with the
        null controller
    _M_LINKS : sympy.diag
        inertia matrix of the links
    _M_JOINTS : sympy.diag
        inertia matrix of the joints
    L : numpy.array
        segment lengths of arm [meters]

    Transform Naming Convention: Tpoint1point2
    ex: Tj1l1 tranforms from joint 1 to link 1

    Transforms are broken up into two matrices for simplification
    ex: Tj0l1a and Tj0l1b where the former transform accounts for
    joint rotations and the latter accounts for static rotations
    and translations
    """

    def __init__(self, **kwargs):

        super(Config, self).__init__(
            N_JOINTS=2, N_LINKS=3, ROBOT_NAME='twolink', **kwargs)

        if self.MEANS is None:
            self.MEANS = {  # expected mean of joints angles / velocities
                'q': np.array([.88, 1.95]),
                'dq': np.array([.645, 2.76])
                }

        if self.SCALES is None:
            self.SCALES = {  # expected variance of joint angles / velocities
                'q': np.array([.52, 1.3]),
                'dq': np.array([6.7, 12.37])
                }

        self._T = {}  # dictionary for storing calculated transforms

        # for the null space controller, keep arm near these angles
        self.REST_ANGLES = np.array([np.pi/4.0, np.pi/4.0])

        # create the inertia matrices for each link of the twolink
        self._M_LINKS.append(np.diag(np.zeros(6)))  # non-existent link0
        self._M_LINKS.append(np.diag([1.98, 1.98, 1.98,
                             15, 15, 15]))  # link1
        self._M_LINKS.append(np.diag([1.32, 1.32, 1.32,
                             8, 8, 8]))  # link2

        # the joints don't weigh anything
        self._M_JOINTS = [sp.zeros(6, 6) for ii in range(self.N_JOINTS)]

        # segment lengths associated with each joint
        # [x, y, z],  Ignoring lengths < 1e-04

        self.L = np.array([
            [0, 0, 0],  # from origin to l0 (non-existent)
            [0, 0, 0],  # from l0 to j0
            [1.0, 0, 0],  # from j0 to l1 COM
            [1.0, 0, 0],  # from l1 COM to j1
            [0.6, 0, 0],  # from j1 to l2 COM
            [0.6, 0, 0]])  # from l2 COM to j2

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

        # Transform matrix : link 2 -> end-effector
        # no change of axes, account for offsets
        self.Tl2ee = sp.Matrix([
            [1, 0, 0, self.L[5, 0]],
            [0, 1, 0, self.L[5, 1]],
            [0, 0, 1, self.L[5, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for angular velocity)
        self.J_orientation = [
            self._calc_T('joint0')[:3, :3] * self._KZ,  # joint 0 orientation
            self._calc_T('joint1')[:3, :3] * self._KZ]  # joint 1 orientation

    def _calc_T(self, name):
        """ Uses Sympy to generate the transform for a joint or link

        name : string
            name of the joint, link, or end-effector
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
            elif name == 'EE':
                self._T[name] = self._calc_T('link2') * self.Tl2ee

            else:
                raise Exception('Invalid transformation name: %s' % name)

        return self._T[name]

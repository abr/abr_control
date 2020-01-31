import numpy as np


class TwoJoint:  # pylint: disable=too-many-public-methods
    """ Analytically derived kinematics and dynamics functions for a two joint arm,
    with the first joint rotating around z and the second y. """

    def __init__(self, L0=0.1, L1=0.2, L2=0.4):
        self.N_JOINTS = 2
        self.L = [L0, L1, L2]
        self.m = [0.0, 2.0, 2.0]
        self.I = [
            [0.0, 0.0, 0.0],
            [0.3, 0.2, 0.1],
            [0.1, 0.2, 0.3]
        ]
        self.M_LINKS = []
        for ii in range(3):
            M = np.zeros((6, 6))
            M[:3, :3] = np.eye(3) * self.m[ii]
            M[3:, 3:] = np.diag(self.I[ii])
            self.M_LINKS.append(M)

        self.r10 = np.array([0, 0, .1])  # offset of COM of link 1 from joint 0
        self.r21 = np.array([0, 0, 0.2])  # offset of COM of link 2 from joint 1

        # function puts vector into cross product matrix
        self.tilde = lambda r: np.array([
            [0, -r[2], -r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]])

        # need to generate spatial inertia tensor for CRB algorithm
        def gen_spatial_I(i_diag, c, m):
            I = np.zeros((6, 6))
            I[:3, :3] = np.diag(i_diag) - m * np.dot(self.tilde(c), self.tilde(c))
            I[:3, 3:] = m * self.tilde(c)
            I[3:, :3] = m * self.tilde(c).T
            I[3:, 3:] = m * np.eye(3)
            return I
        I1 = gen_spatial_I(self.I[1], self.r10, self.m[1])
        I2 = gen_spatial_I(self.I[2], self.r21, self.m[2])
        self.I = [I1, I2]

    def R_link0(self, q):
        """ Returns the rotation matrix for the COM of link 0 """
        return np.eye(3)

    def Tx_link0(self, q):
        """ Returns the position of COM of link 0 """
        return np.array([0, 0, self.L[0] / 2.0])

    # def T_inv_link0(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """

    def J_link0(self, q):
        """ Returns the Jacobian of the COM of link 0 """
        return np.array([[0.0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])

    # def dJ_link0(self, q, dq):
    #     """ Returns the derivative of the Jacobian of link 0 """

    # def R_joint0(self, q):
    #     """ Returns rotation matrix for joint 0 """

    def Tx_joint0(self, q):
        """ Returns the position of joint 0 """
        return np.array([0, 0, self.L[0]])

    # def T_inv_joint0(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """
    # def J_joint0(self, q):
    #     """ Returns the Jacobian of joint 0 """
    # def dJ_joint0(self, q, dq):
    #     """ Returns the derivative of the Jacobian of joint 0 """

    def R_link1(self, q):
        """ Returns rotation matrix for the COM of link 1 """
        c0 = np.cos(q[0])
        s0 = np.sin(q[0])
        return np.array([[c0, -s0, 0], [s0, c0, 0], [0, 0, 1]])

    def Tx_link1(self, q):
        """ Returns the position of COM of link 1 """
        return np.array([0, 0, self.L[0] + self.L[1] / 2])

    # def T_inv_link1(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """

    def J_link1(self, q):
        """ Returns the Jacobian of the COM of link 1 """
        return np.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 0]])

    # def dJ_link1(self, q, dq):
    #     """ Returns the derivative of the Jacobian of link 1 """

    # def R_joint1(self, q):
    #     """ Returns rotation matrix for joint 1 """

    def Tx_joint1(self, q):
        """ Returns the position of joint 1 """
        return np.array([0, 0, self.L[0] + self.L[1]])

    # def T_inv_joint1(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """
    # def J_joint1(self, q):
    #     """ Returns the Jacobian of joint 1 """
    # def dJ_joint1(self, q, dq):
    #     """ Returns the derivative of the Jacobian of joint 1 """

    def R_link2(self, q):
        """ Returns rotation matrix for the COM of link 2 """
        c1 = np.cos(q[1])
        s1 = np.sin(q[1])
        R21 = np.array([[c1, 0, s1], [0, 1, 0], [-s1, 0, c1]])
        return np.dot(self.R_link1(q), R21)

    def Tx_link2(self, q):
        """ Returns the position of the COM of link 2 """
        c = np.cos(q)
        s = np.sin(q)
        return np.array(
            [
                c[0] * s[1] * self.L[2] / 2,
                s[0] * s[1] * self.L[2] / 2,
                c[1] * self.L[2] / 2 + self.L[1] + self.L[0],
            ]
        )

    # def T_inv_link2(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """

    def J_link2(self, q):
        """ Returns the Jacobian of the COM of link 2 """
        c = np.cos(q)
        s = np.sin(q)
        return np.array(
            [
                [-s[0] * s[1] * self.L[2] / 2, c[0] * c[1] * self.L[2] / 2],
                [c[0] * s[1] * self.L[2] / 2, s[0] * c[1] * self.L[2] / 2],
                [0, -s[1] * self.L[2] / 2],
                [0, -s[0]],
                [0, c[0]],
                [1, 0],
            ]
        )

    # def dJ_link2(self, q, dq):
    #     """ Returns the derivative of the Jacobian of link 2 """

    def R_EE(self, q):
        """ Returns rotation matrix of the end effector """
        return self.R_link2(q)

    def Tx_EE(self, q):
        """ Returns the position of the end effector """
        c = np.cos(q)
        s = np.sin(q)
        return np.array(
            [
                c[0] * s[1] * self.L[2],
                s[0] * s[1] * self.L[2],
                c[1] * self.L[2] + self.L[1] + self.L[0],
            ]
        )

    # def T_inv_EE(self, q):
    #     """ Returns the inverse transform matrix for the COM of link 0 """

    def J_EE(self, q):
        """ Returns the Jacobian of the end effector """
        c = np.cos(q)
        s = np.sin(q)
        return np.array(
            [
                [-s[0] * s[1] * self.L[2], c[0] * c[1] * self.L[2]],
                [c[0] * s[1] * self.L[2], s[0] * c[1] * self.L[2]],
                [0, -s[1] * self.L[2]],
                [0, -s[0]],
                [0, c[0]],
                [1, 0],
            ]
        )

    # def dJ_EE(self, q, dq):
    #     """ Returns the derivative of the Jacobian of the end effector """

    def M(self, q):
        """ Returns the inertia matrix in joint space calculated using the
        composite rigid body (CRB) algorithm, from Featherstone, 2008, Ch 6 """

        S0 = np.array([0, 0, 1, 0, 0, 0])  # joint 0 motion subspace
        S1 = np.array([0, 1, 0, 0, 0, 0])  # joint 1 motion subspace
        S = [S0, S1]

        H = np.zeros((self.N_JOINTS, self.N_JOINTS))  # initialize inertia matrix

        # Ic is the inertia of subtrees starting at each joint. This changes based on
        # configuration, start with each link's spatial inertia tensor.
        Ic = np.zeros((self.N_JOINTS, 6, 6))
        for ii in range(0, self.N_JOINTS):
            Ic[ii] = self.I[ii]

        # rotation matrix of joint 0 (around z axis)
        R1 = np.array([[np.cos(q[0]), np.sin(q[0]), 0],
                       [-np.sin(q[0]), np.cos(q[0]), 0],
                       [0, 0, 1]])
        # rotation matrix of joint 1 (around y axis)
        R2 = np.array([[np.cos(q[1]), 0, -np.sin(q[1])],
                       [0, 1, 0],
                       [np.sin(q[1]), 0, np.cos(q[1])]])

        def plucker_transform(R, c):
            X = np.zeros((6, 6))
            X[:3, :3] = R
            X[3:, 3:] = R
            X[3:, :3] = -np.dot(R, self.tilde(c))
            return X
        X10 = plucker_transform(R1, self.r10)
        X21 = plucker_transform(R2, self.r21)
        X = [X10, X21]

        for ii in range(self.N_JOINTS-1, -1, -1):
            if ii - 1 > -1:
                Ic[ii-1] += np.dot(X[ii].T, np.dot(Ic[ii], X[ii]))

            F = np.dot(Ic[ii], S[ii])
            H[ii, ii] = np.dot(S[ii].T, F)
            for jj in range(ii, 0, -1):
                F = np.dot(X[jj].T, F)
                kk = jj - 1
                H[ii, kk] = np.dot(F.T, S[kk])
                H[kk, ii] = H[ii, kk].T
        return H

    def g(self, q):
        """ Returns the effects of gravity in joint space """
        g = np.array([0, 9.81 * np.sin(q[1]) * self.L[2]])

        return g

    def C(self, q, dq):
        """ Returns the partial centrifugal and Coriolis effects
        where np.dot(C, dq) is the full term """
        m2 = self.M_LINKS[2][0, 0]
        return (
            m2
            * self.L[1]
            * self.L[2]
            / 2.0
            * np.sin(q[1])
            * np.array([[-dq[1], -dq[1] - dq[0]], [dq[0], 0]])
        )

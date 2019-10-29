import numpy as np


class TwoJoint:  # pylint: disable=too-many-public-methods
    """ A class with equationns analytically derived in (Spong et al, 2004),
    page 209, for testing the symbolic function generation config system. """

    def __init__(self, L0=0.1, L1=0.2, L2=0.4):
        self.L = [L0, L1, L2]
        self.M_LINKS = [np.diag(np.zeros(6))]  # non-existent link0
        self.M_LINKS.append(
            np.diag([2.0, 2.0, 2.0, 0.226891, 0.226891, 0.0151074])
        )  # link1
        self.M_LINKS.append(
            np.diag([2.0, 2.0, 2.0, 0.226891, 0.226891, 0.0151074])
        )  # link2
        # joints don't weigh anything
        self.M_JOINTS = [np.zeros((6, 6)) for ii in range(2)]

        self.MEANS = {"q": np.ones(2), "dq": np.ones(2) * 3}
        self.SCALES = {"q": np.ones(2) * 2, "dq": np.ones(2) * 0.5}

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
        T21 = np.array([[c1, 0, s1], [0, 1, 0], [-s1, 0, c1]])
        return np.dot(self.R_link1(q), T21)

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
        """ Returns the inertia matrix in joint space """
        M1 = np.dot(np.dot(self.J_link1(q).T, self.M_LINKS[1]), self.J_link1(q))
        M2 = np.dot(np.dot(self.J_link2(q).T, self.M_LINKS[2]), self.J_link2(q))
        M = M1 + M2
        return M

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

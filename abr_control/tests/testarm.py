import numpy as np


class TwoJoint():
    """ A class with equationns analytically derived in (Spong et al, 2004),
    page 209, for testing the symbolic function generation config system. """

    def __init__(self):
        self.L = [0.0, 2.0, 1.2]
        self.M_LINKS = [np.diag(np.zeros(6))] # non-existant link0
        self.M_LINKS.append(np.diag([1.98, 1.98, 1.98,
                               2.56, 2.56, 2.56]))  # link1
        self.M_LINKS.append(np.diag([1.32, 1.32, 1.32,
                               0.6336, 0.6336, 0.6336]))  # link2
        # joints don't weigh anything
        self.M_JOINTS = [np.zeros((6, 6)) for ii in range(2)]


    def R_link0(self, q):
        """ Returns the rotation matrix for the COM of link 0 """
        return np.eye(3)

    def Tx_link0(self, q):
        """ Returns the position of COM of link 0 """
        return np.array([
            self.L[0] / 2.0,
            0,
            0])

    def T_inv_link0(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        return np.array([
            [1, 0, 0, -self.L[0] / 2.0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    def J_link0(self, q):
        """ Returns the Jacobian of the COM of link 0 """
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def dJ_link0(self, q, dq):
        """ Returns the derivative of the Jacobian of link 0 """
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def R_joint0(self, q):
        """ Returns rotation matrix for joint 0 """
        return np.eye(3)

    def Tx_joint0(self, q):
        """ Returns the position of joint 0 """
        return np.array([
            self.L[0],
            0,
            0])

    def T_inv_joint0(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        return np.array([
            [1, 0, 0, -self.L[0]],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    def J_joint0(self, q):
        """ Returns the Jacobian of joint 0 """
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def dJ_joint0(self, q, dq):
        """ Returns the derivative of the Jacobian of joint 0 """
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def R_link1(self, q):
        """ Returns rotation matrix for the COM of link 1 """
        return np.array([
            [np.cos(q[0]), -np.sin(q[0]), 0],
            [np.sin(q[0]), np.cos(q[0]), 0],
            [0, 0, 1]])

    def Tx_link1(self, q):
        """ Returns the position of COM of link 1 """
        return np.array([
            self.L[0] + self.L[1] / 2.0 * np.cos(q[0]),
            self.L[1] / 2.0 * np.sin(q[0]),
            0])

    def T_inv_link1(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        c0 = np.cos(q[0])
        s0 = np.sin(q[0])
        return np.array([
            [c0, s0, 0, -s0 * self.L[1] / 2.0 * s0 -
                c0 * (self.L[0] + self.L[1] / 2.0 * c0)],
            [-s0, c0, 0, s0 * (self.L[0] + self.L[1] / 2.0 * c0) -
                c0 * self.L[1] / 2.0 * s0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    def J_link1(self, q):
        """ Returns the Jacobian of the COM of link 1 """
        return np.array([
            [-self.L[1] / 2.0 * np.sin(q[0]), 0],
            [self.L[1] / 2.0 * np.cos(q[0]), 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 0]])

    def dJ_link1(self, q, dq):
        """ Returns the derivative of the Jacobian of link 1 """
        return np.array([
            [-self.L[1] / 2.0 * np.cos(q[0]) * dq[0], 0],
            [-self.L[1] / 2.0 * np.sin(q[0]) * dq[0], 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def R_joint1(self, q):
        """ Returns rotation matrix for joint 1 """
        return np.array([
            [np.cos(q[0]), -np.sin(q[0]), 0],
            [np.sin(q[0]), np.cos(q[0]), 0],
            [0, 0, 1]])

    def Tx_joint1(self, q):
        """ Returns the position of joint 1 """
        return np.array([
            self.L[0] + self.L[1] * np.cos(q[0]),
            self.L[1] * np.sin(q[0]),
            0])

    def T_inv_joint1(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        c0 = np.cos(q[0])
        s0 = np.sin(q[0])
        alpha = self.L[0] + self.L[1] * c0
        beta = self.L[1] * s0
        return np.array([
            [c0, s0, 0, -c0 * alpha - s0 * beta],
            [-s0, c0, 0, s0 * alpha - c0 * beta],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    def J_joint1(self, q):
        """ Returns the Jacobian of joint 1 """
        return np.array([
            [-self.L[1] * np.sin(q[0]), 0],
            [self.L[1] * np.cos(q[0]), 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 0]])

    def dJ_joint1(self, q, dq):
        """ Returns the derivative of the Jacobian of joint 1 """
        return np.array([
            [-self.L[1] * np.cos(q[0]) * dq[0], 0],
            [-self.L[1] * np.sin(q[0]) * dq[0], 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def R_link2(self, q):
        """ Returns rotation matrix for the COM of link 2 """
        c0 = np.cos(q[0])
        c1 = np.cos(q[1])
        s0 = np.sin(q[0])
        s1 = np.sin(q[1])
        return np.array([
            [c0 * c1 - s0 * s1, -c0 * s1 - s0 * c1, 0],
            [s0 * c1 + c0 * s1, -s0 * s1 + c0 * c1, 0],
            [0, 0, 1]])

    def Tx_link2(self, q):
        """ Returns the position of the COM of link 2 """
        return np.array([
            (self.L[0] + self.L[1] * np.cos(q[0]) +
                self.L[2] / 2.0 * np.cos(q[0] + q[1])),
            self.L[1] * np.sin(q[0]) + self.L[2] / 2.0 * np.sin(q[0] + q[1]),
            0])

    def T_inv_link2(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        c0 = np.cos(q[0])
        c1 = np.cos(q[1])
        c01 = np.cos(q[0] + q[1])
        s0 = np.sin(q[0])
        s1 = np.sin(q[1])
        s01 = np.sin(q[0] + q[1])
        gamma = c0 * c1 - s0 * s1
        phi = s0 * c1 + c0 * s1
        eta = self.L[0] + self.L[1] * c0 + self.L[2] / 2.0 * c01
        lamb = self.L[1] * s0 + self.L[2] / 2.0 * s01
        tau = -c0 * s1 - s0 * c1
        theta = -s0 * s1 + c0 * c1
        return np.array([
            [gamma, phi, 0, -gamma * eta - phi * lamb],
            [tau, theta, 0, -tau*eta - theta*lamb],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])


    def J_link2(self, q):
        """ Returns the Jacobian of the COM of link 2 """
        return np.array([
            [-self.L[1] * np.sin(q[0]) - self.L[2] / 2.0 * np.sin(q[0] + q[1]),
                -self.L[2] / 2.0 * np.sin(q[0] + q[1])],
            [self.L[1] * np.cos(q[0]) + self.L[2] / 2.0 * np.cos(q[0] + q[1]),
                self.L[2] / 2.0 * np.cos(q[0] + q[1])],
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 1]])

    def dJ_link2(self, q):
        """ Returns the derivative of the Jacobian of link 2 """
        return np.array([
            [-self.L[1] * np.cos(q[0]) * dq[0] -
                self.L[2] / 2.0 * np.cos(q[0] + q[1]) * dq[0],
                -self.L[2] / 2.0 * np.sin(q[0] + q[1]) * dq[1]],
            [-self.L[1] * np.sin(q[0]) * dq[0] -
                self.L[2] / 2.0 * np.sin(q[0] + q[1]) * dq[0],
                -self.L[2] / 2.0 * np.sin(q[0] + q[1]) * dq[1]],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])

    def R_EE(self, q):
        """ Returns rotation matrix of the end effector """
        c0 = np.cos(q[0])
        c1 = np.cos(q[1])
        s0 = np.sin(q[0])
        s1 = np.sin(q[1])
        return np.array([
            [c0 * c1 - s0 * s1, -c0 * s1 - s0 * c1, 0],
            [s0 * c1 + c0 * s1, -s0 * s1 + c0 * c1, 0],
            [0, 0, 1]])

    def Tx_EE(self, q):
        """ Returns the position of the end effector """
        return np.array([
            (self.L[0] + self.L[1] * np.cos(q[0]) +
                self.L[2] * np.cos(q[0] + q[1])),
            (self.L[0] + self.L[1] * np.sin(q[0]) +
                self.L[2] * np.sin(q[0] + q[1])),
            0])

    def T_inv_EE(self, q):
        """ Returns the inverse transform matrix for the COM of link 0 """
        c0 = np.cos(q[0])
        c1 = np.cos(q[1])
        s0 = np.sin(q[0])
        s1 = np.sin(q[1])
        gamma = c0 * c1 - s0 * s1
        phi = s0 * c1 + c0 * s1
        eta = self.L[0] + self.L[1] * c0 + self.L[2] * np.cos(q[0] + q[1])
        lamb = self.L[1] * s0 + self.L[2] * np.sin(q[0] + q[1])
        tau = -c0 * s1 - s0 * c1
        theta = -s0 * s1 + c0 * c1
        return np.array([
            [gamma, phi, 0, -gamma * eta - phi * lamb],
            [tau, theta, 0, -tau * eta - theta * lamb],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    def J_EE(self, q):
        """ Returns the Jacobian of the end effector """
        return np.array([
            [-self.L[1] * np.sin(q[0]) - self.L[2] *
                np.sin(q[0] + q[1]), -self.L[2] * np.sin(q[0] + q[1])],
            [self.L[1] * np.cos(q[0]) + self.L[2] * np.cos(q[0] + q[1]),
                self.L[2] * np.cos(q[0] + q[1])],
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 1]])

    def dJ_EE(self, q, dq):
        """ Returns the derivative of the Jacobian of the end effector """
        return np.array([
            [-self.L[1] * np.cos(q[0]) * dq[0] -
                self.L[2] / 2.0 * np.cos(q[0] + q[1]) * dq[0],
                -self.L[2] * np.sin(q[0] + q[1]) * dq[1]],
            [-self.L[1] * np.sin(q[0]) * dq[0] -
                self.L[2] / 2.0 * np.sin(q[0] + q[1]) * dq[0],
                -self.L[2] * np.sin(q[0] + q[1]) * dq[1]],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]])


    def M(self, q):
        """ Returns the inertia matrix in joint space """
        m1 = self.M_LINKS[1][0, 0]
        m2 = self.M_LINKS[2][0, 0]
        i1 = self.M_LINKS[1][3, 3]
        i2 = self.M_LINKS[2][3, 3]
        L = self.L
        lc1 = L[1] / 2.0
        lc2 = L[2] / 2.0

        # NOTE: (Spong et al, 2004) incorrectly has the term commented out
        # below included in their equations.
        m11 = (m1*lc1**2 + m2 * (L[1]**2 + lc2**2 + #2*L[1]*lc2**2 +
               2*L[1]*lc2*np.cos(q[1])) + i1 + i2)
        m12 = m21 = m2 * (lc2**2 + L[1] * lc2 * np.cos(q[1])) + i2
        m22 = m2 * lc2**2 + i2
        return np.array([[m11, m12], [m21, m22]])


    def g(self, q):
        """ Returns the effects of gravity in joint space """
        return np.array([0, 0])


    def C(self, q, dq):
        """ Returns the partial centrifugal and Coriolis effects
        where np.dot(C, dq) is the full term """
        m2 = self.M_LINKS[2][0, 0]
        return (m2 * self.L[1] * self.L[2] / 2.0 * np.sin(q[1]) *
                np.array([[-dq[1], -dq[1] - dq[0]], [dq[0], 0]]))

import cloudpickle
import os
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the Kinova Jaco^2 V2"""

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=6, num_links=7,
                                           robot_name='jaco2', **kwargs)

        self.joint_names = ['joint%i' % ii
                            for ii in range(self.num_joints)]
        # Kinova Home Position
        #self.home_position = np.array([275.0, 167.0, 57.0,
        #                      240.0, 82.0, 75.0], dtype="float32")
        # Straight up
        self.home_position = np.array([90.0, 180.0, 180.0,
                              0.0, 0.0, 0.0], dtype="float32")
        self.home_torques = np.array([-0.138, -0.116, 3.339,
                             -0.365, -0.113, 0.061], dtype="float32")

        # for the null space controller, keep arm near these angles
        # currently set to the center of the limits
        self.rest_angles = np.array([0.0, 140.0, 140.0, 0.0, 0.0, 0.0])

        # create the inertia matrices for each link of the kinova jaco2
        self._M.append(np.diag([0.64, 0.64, 0.64,
                                0.01, 0.01, 0.01]))  # link0
        self._M.append(np.diag([0.6, 0.6, 0.6,
                                0.04, 0.04, 0.04]))  # link1
        self._M.append(np.diag([0.57, 0.57, 0.57,
                                0.04, 0.04, 0.04]))  # link2
        self._M.append(np.diag([0.6, 0.6, 0.6,
                                0.04, 0.04, 0.04]))  # link3
        self._M.append(np.diag([0.37, 0.37, 0.37,
                                0.04, 0.04, 0.04]))  # link4
        self._M.append(np.diag([0.37, 0.37, 0.37,
                                0.04, 0.04, 0.04]))  # link5
        self._M.append(np.diag([0.37, 0.37, 0.37,
                                0.04, 0.04, 0.04]))  # link6
        #self._M.append(np.diag([1.05, 1.05, 1.05,
        #                        0.04, 0.04, 0.04]))  # link6 with hand

        # segment lengths associated with each joint [m]
        # [x, y, z],  Ignoring lengths < 1e-04

        #using world frame
        self.L = np.array([
            [-0.0000327, -0.0000173, 0.15675],
            [0.0, 0.0, -0.119],
            [0.0, 0.410, 0.000000014901],
            [0.0, -0.207, -0.00980],
            [0.0, 0.0342, -0.0658],
            [0.0, 0.0343, -0.0659]])

        self.L_com = np.array([
            [-0.0000327, -0.0000173, 0.081581],
            [-0.0000221, 0.00013257, -0.0799],
            [-0.00000109, 0.20499, -0.0232],
            [-0.00153, -0.0833, -0.0122],
            [-0.000478, 0.012373, -0.0354],
            [-0.000539, 0.012305, -0.0355],
            [-0.0000386, 0.000073135, 0.00000032783]]) # EE

        # ---- Joint Transform Matrices ----

        # transform matrix from origin to joint 0 reference frame
        # adding pi offset to have arm point straight up at joint angles = 0
        self.T0org = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, -1, 0, self.L[0, 1]],
            [0, 0, -1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # transform matrix from origin to joint 0 reference frame
        # Transform due to rotation in reference frame
        self.T10a = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])

        # Transform due to q
        self.T10b = sp.Matrix([
            [sp.cos(sp.pi + self.q[0]), 0, sp.sin(sp.pi + self.q[0]), 0],
            [0, 1, 0, self.L[1, 2]],
            [-sp.sin(sp.pi + self.q[0]), 0, sp.cos(sp.pi + self.q[0]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1 to joint 2 reference frame
        self.T21a = sp.Matrix([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]])

        self.T21b = sp.Matrix([
            [sp.cos(sp.pi + self.q[1]), sp.sin(sp.pi + self.q[1]), 0,
             self.L[2, 1]*sp.sin(sp.pi + self.q[1])],
            [-sp.sin(sp.pi + self.q[1]), sp.cos(sp.pi + self.q[1]), 0,
             self.L[2, 1]*sp.cos(sp.pi + self.q[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to joint 3
        self.T32a = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])

        self.T32b = sp.Matrix([
            [sp.cos(sp.pi + self.q[2]), 0, sp.sin(sp.pi + self.q[2]),
             self.L[3, 1]*sp.sin(sp.pi + self.q[2])],
            [0, 1, 0, self.L[3, 2]],
            [-sp.sin(sp.pi + self.q[2]), 0, sp.cos(sp.pi + self.q[2]),
             self.L[3, 1]*sp.cos(sp.pi + self.q[2])],
            [0, 0, 0, 1]])

        # transform matrix from joint 3 to joint 4
        # reference frame is rotated by 55 degrees (0.959931rad)
        # Did transform in reverse order to simplify math
        self.T43a = sp.Matrix([
            [1, 0, 0, 0],
            #[0, sp.cos(0.959931), sp.sin(0.959931), 0],
            #[0, -sp.sin(0.959931), sp.cos(0.959931), 0],
            # NOTE: switching from 55 to 60
            [0, sp.cos(1.047), sp.sin(1.047), 0],
            [0, -sp.sin(1.047), sp.cos(1.047), 0],
            [0, 0, 0, 1]])

        self.T43b = sp.Matrix([
            [sp.cos(sp.pi - self.q[3]), sp.sin(sp.pi - self.q[3]), 0,
             self.L[4, 1]*sp.sin(sp.pi + self.q[3])],
            [-sp.sin(sp.pi - self.q[3]), sp.cos(sp.pi - self.q[3]), 0,
             -self.L[4, 1]*sp.cos(sp.pi + self.q[3])],
            [0, 0, 1, self.L[4, 2]],
            [0, 0, 0, 1]])

        # transform matrix from joint 4 to joint 5
        self.T54a = sp.Matrix([
            [1, 0, 0, 0],
            #[0, sp.cos(0.959931), sp.sin(0.959931), 0],
            #[0, -sp.sin(0.959931), sp.cos(0.959931), 0],
            # NOTE: switching from 55 to 60
            [0, sp.cos(1.047), sp.sin(1.047), 0],
            [0, -sp.sin(1.047), sp.cos(1.047), 0],
            [0, 0, 0, 1]])

        self.T54b = sp.Matrix([
            [sp.cos(sp.pi - self.q[4]), sp.sin(sp.pi - self.q[4]), 0,
             self.L[5, 1]*sp.sin(sp.pi + self.q[4])],
            [-sp.sin(sp.pi - self.q[4]), sp.cos(sp.pi - self.q[4]), 0,
             -self.L[5, 1]*sp.cos(sp.pi + self.q[4])],
            [0, 0, 1, self.L[5, 2]],
            [0, 0, 0, 1]])

        # transform matrix from joint 6 to end-effector
        # might need to flip y and z rotation
        self.TEE5 = sp.Matrix([
            [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
            [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0,  0],
            [0, 0, 1, self.L_com[6, 2]],
            [0, 0, 0, 1]])


        # ---- COM Transform Matrices ----

        # transform matrix from origin  to link 0
        self.Tl0org = sp.Matrix([
            [1, 0, 0, self.L_com[0, 0]],
            [0, 1, 0, self.L_com[0, 1]],
            [0, 0, 1, self.L_com[0, 2]],
            [0, 0, 0, 1]])

        # transform matrix from joint 0  to link 1
        self.Tl10 = sp.Matrix([
            [sp.cos(sp.pi + self.q[0]), 0, sp.sin(sp.pi + self.q[0]), 0],
            [0, 1, 0, self.L_com[1, 2]],
            [-sp.sin(sp.pi + self.q[0]), 0, sp.cos(sp.pi + self.q[0]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1  to link 2
        self.Tl21 = sp.Matrix([
            [sp.cos(sp.pi + self.q[1]), sp.sin(sp.pi + self.q[1]), 0,
             self.L_com[2, 1]*sp.sin(sp.pi + self.q[1])],
            [-sp.sin(sp.pi + self.q[1]), sp.cos(sp.pi + self.q[1]), 0,
             self.L_com[2, 1]*sp.cos(sp.pi + self.q[1]) + self.L_com[2,0]],
            [0, 0, 1, -self.L_com[2, 2]],
            [0, 0, 0, 1]])

        # transform matrix from joint 2  to link 3
        # offset of -0.00153[m] in x, not sure if necessary, but currently added
        self.Tl32 = sp.Matrix([
            [sp.cos(sp.pi + self.q[2]), 0, sp.sin(sp.pi + self.q[2]),
             self.L_com[3, 1]*sp.sin(sp.pi + self.q[2])],
            [0, 1, 0, self.L_com[3, 2]],
            [-sp.sin(sp.pi + self.q[2]), 0, sp.cos(sp.pi + self.q[2]),
             self.L_com[3, 1]*sp.cos(sp.pi + self.q[2])],
            [0, 0, 0, 1]])

        # transform matrix from joint 3  to link 4
        self.Tl43 = sp.Matrix([
            [sp.cos(sp.pi - self.q[3]), sp.sin(sp.pi - self.q[3]), 0,
             self.L_com[4, 1]*sp.sin(sp.pi + self.q[3])],
            [-sp.sin(sp.pi - self.q[3]), sp.cos(sp.pi - self.q[3]), 0,
             -self.L_com[4, 1]*sp.cos(sp.pi + self.q[3])],
            [0, 0, 1, self.L_com[4, 2]],
            [0, 0, 0, 1]])

        # transform matrix from joint 4  to link 5
        self.Tl54 = sp.Matrix([
            [sp.cos(sp.pi - self.q[4]), sp.sin(sp.pi - self.q[4]), 0,
             self.L_com[5, 1]*sp.sin(sp.pi + self.q[4])],
            [-sp.sin(sp.pi - self.q[4]), sp.cos(sp.pi - self.q[4]), 0,
             -self.L_com[5, 1]*sp.cos(sp.pi + self.q[4])],
            [0, 0, 1, self.L_com[5, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        """self.J_orientation = [[0, 1, 0],  # joint 0 rotates around y axis
                              [0, 0, 1],  # joint 1 rotates around z axis
                              [0, 1, 0],  # joint 2 rotates around y axis
                              [0, 0, 1],  # joint 3 rotates around z axis
                              [0, 0, 1],  # joint 4 rotates around z axis
                              [0, 0, 1]]  # joint 5 rotates around z axis"""

        """self.J_orientation = [[0, 10, 0],  # joint 0 rotates around y axis
                              [0, 0, 8],  # joint 1 rotates around z axis
                              [0, 8, 0],  # joint 2 rotates around y axis
                              [0, 8, 8],  # joint 3 rotates around z axis
                              [0, 8, 8],  # joint 4 rotates around z axis
                              [0, 0, 8]]  # joint 5 rotates around z axis"""

        self.J_orientation = [[0, 0, 1],  # joint 0 rotates around y axis
                              [0, 0, 1],  # joint 1 rotates around z axis
                              [0, 0, 1],  # joint 2 rotates around y axis
                              [0, 0, 1],  # joint 3 rotates around z axis
                              [0, 0, 1],  # joint 4 rotates around z axis
                              [0, 0, 1]]  # joint 5 rotates around z axis

    def _calc_T(self, name, lambdify=True): #, regenerate=False):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link
        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        # ---- Joint Transforms ----
        if name == 'joint0':
            T = self.T0org
        elif name == 'joint1':
            T = self.T0org * self.T10a * self.T10b
        elif name == 'joint2':
            T = self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
        elif name == 'joint3':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.T32b)
        elif name == 'joint4':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.T32b * self.T43b * self.T43a)
        elif name == 'joint5':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.T32b * self.T43b * self.T43a * self.T54b
                    * self.T54a)
        elif name == 'EE' or name == 'link6':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.T32b * self.T43b * self.T43a * self.T54b
                    * self.T54a * self.TEE5)

        # ---- COM Transforms ----
        elif name == 'link0':
            T = self.Tl0org
        elif name == 'link1':
            T = self.T0org * self.T10a * self.Tl10
        elif name == 'link2':
            T = self.T0org * self.T10a * self.T10b * self.T21a * self.Tl21
        elif name == 'link3':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.Tl32)
        elif name == 'link4':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    * self.T32a * self.T32b * self.Tl43)
        elif name == 'link5':
            T = (self.T0org * self.T10a * self.T10b * self.T21a * self.T21b
                    *self.T32a * self.T32b * self.T43b * self.T43a * self.Tl54)

        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

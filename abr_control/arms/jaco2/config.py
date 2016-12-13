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
        # Kinova Home Position - straight up
        self.home_position = np.array([250.0, 180.0, 180.0,
                                       270.0, 0.0, 0.0], dtype="float32")
        self.home_torques = np.array([-0.138, -0.116, 3.339,
                                      -0.365, -0.113, 0.061], dtype="float32")

        # for the null space controller, keep arm near these angles
        # currently set to the center of the limits
        self.rest_angles = np.array([0.0, 140.0, 140.0, 0.0, 0.0, 0.0],
                                    dtype='float32')

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
        # self._M.append(np.diag([1.05, 1.05, 1.05,
        #                        0.04, 0.04, 0.04]))  # link6 with hand

        # segment lengths associated with each joint [m]
        # [x, y, z],  Ignoring lengths < 1e-04

        self.L = np.array([
            [-0.0000327, -0.0000173, 0.15675],
            [0.0, 0.0, -0.119],
            [0.0, 0.410, 0.000000014901],
            [0.0, -0.207, -0.00980],
            [0.0, 0.0342, -0.0658],
            [0.0, 0.0343, -0.0659],
            [0.0, 0.06, -0.05]],
            dtype='float32')

        self.L_com = np.array([
            [-0.0000327, -0.0000173, 0.081581],
            [-0.0000221, 0.00013257, -0.0799],
            [-0.00000109, 0.20499, -0.0232],
            [-0.00153, -0.0833, -0.0122],
            [-0.000478, 0.012373, -0.0354],
            [-0.000539, 0.012305, -0.0355],
            [-0.0000386, 0.000073135, 0.00000032783]],  # EE
            dtype='float32')

        # ---- Joint Transform Matrices ----

        # Transform matrix : origin -> joint 0
        # + pi because we're expecting the arm to
        # point straight up at joint angles = 0
        self.TorgO = sp.Matrix([
            [1, 0, 0, self.L[0, 0]],
            [0, -1, 0, self.L[0, 1]],
            [0, 0, -1, self.L[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> joint 1
        # transformation due to rotation in reference frame
        self.T01a = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])
        # transformation due to q
        self.T01b = sp.Matrix([
            [sp.cos(sp.pi + self.q[0]), 0, sp.sin(sp.pi + self.q[0]), 0],
            [0, 1, 0, self.L[1, 2]],
            [-sp.sin(sp.pi + self.q[0]), 0, sp.cos(sp.pi + self.q[0]), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> joint 2
        # transformation due to rotation in reference frame
        self.T12a = sp.Matrix([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]])
        # transformation due to q
        self.T12b = sp.Matrix([
            [sp.cos(sp.pi + self.q[1]), sp.sin(sp.pi + self.q[1]), 0,
             self.L[2, 1]*sp.sin(sp.pi + self.q[1])],
            [-sp.sin(sp.pi + self.q[1]), sp.cos(sp.pi + self.q[1]), 0,
             self.L[2, 1]*sp.cos(sp.pi + self.q[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> joint 3
        # transformation due to rotation in reference frame
        self.T23a = sp.Matrix([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])
        # transformation due to q
        self.T23b = sp.Matrix([
            [sp.cos(sp.pi + self.q[2]), 0, sp.sin(sp.pi + self.q[2]),
             self.L[3, 1]*sp.sin(sp.pi + self.q[2])],
            [0, 1, 0, self.L[3, 2]],
            [-sp.sin(sp.pi + self.q[2]), 0, sp.cos(sp.pi + self.q[2]),
             self.L[3, 1]*sp.cos(sp.pi + self.q[2])],
            [0, 0, 0, 1]])

        # Transform matrix : joint 3 -> joint 4
        # NOTE: reference frame is rotated by 55 degrees (0.959931rad)
        # transformation due to rotation in reference frame
        self.T34a = sp.Matrix([
            [1, 0, 0, 0],
            # NOTE: actual arm is 60 degrees, but VREP might be 55
            [0, sp.cos(1.047), sp.sin(1.047), 0],
            [0, -sp.sin(1.047), sp.cos(1.047), 0],
            [0, 0, 0, 1]])
        # transformation due to q
        self.T34b = sp.Matrix([
            [sp.cos(sp.pi - self.q[3]), sp.sin(sp.pi - self.q[3]), 0,
             self.L[4, 1]*sp.sin(sp.pi + self.q[3])],
            [-sp.sin(sp.pi - self.q[3]), sp.cos(sp.pi - self.q[3]), 0,
             -self.L[4, 1]*sp.cos(sp.pi + self.q[3])],
            [0, 0, 1, self.L[4, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 4 -> joint 5
        # transformation due to rotation in reference frame
        self.T45a = sp.Matrix([
            [1, 0, 0, 0],
            # NOTE: actual arm is 60 degrees, but VREP might be 55
            [0, sp.cos(1.047), sp.sin(1.047), 0],
            [0, -sp.sin(1.047), sp.cos(1.047), 0],
            [0, 0, 0, 1]])
        # transformation due to q
        self.T45b = sp.Matrix([
            [sp.cos(sp.pi - self.q[4]), sp.sin(sp.pi - self.q[4]), 0,
             self.L[5, 1]*sp.sin(sp.pi + self.q[4])],
            [-sp.sin(sp.pi - self.q[4]), sp.cos(sp.pi - self.q[4]), 0,
             -self.L[5, 1]*sp.cos(sp.pi + self.q[4])],
            [0, 0, 1, self.L[5, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 6 -> end-effector
        self.T5EE = sp.Matrix([
            [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
            [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
            [0, 0, 1, self.L_com[6, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : EE -> camera
        self.TEEcamera = sp.Matrix([
            [1, 0, 0, self.L[6, 0]],
            [0, 1, 0, self.L[6, 1]],
            [0, 0, 1, self.L[6, 2]],
            [0, 0, 0, 1]])

        # ---- COM Transform Matrices ----

        # Transform matrix : origin -> link 0
        self.Tlorg0 = sp.Matrix([
            [1, 0, 0, self.L_com[0, 0]],
            [0, 1, 0, self.L_com[0, 1]],
            [0, 0, 1, self.L_com[0, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 0 -> link 1
        self.Tl01 = sp.Matrix([
            [sp.cos(sp.pi + self.q[0]), 0, sp.sin(sp.pi + self.q[0]), 0],
            [0, 1, 0, self.L_com[1, 2]],
            [-sp.sin(sp.pi + self.q[0]), 0, sp.cos(sp.pi + self.q[0]), 0],
            [0, 0, 0, 1]])

        # Transform matrix : joint 1 -> to link 2
        self.Tl12 = sp.Matrix([
            [sp.cos(sp.pi + self.q[1]), sp.sin(sp.pi + self.q[1]), 0,
             self.L_com[2, 1]*sp.sin(sp.pi + self.q[1])],
            [-sp.sin(sp.pi + self.q[1]), sp.cos(sp.pi + self.q[1]), 0,
             self.L_com[2, 1]*sp.cos(sp.pi + self.q[1]) + self.L_com[2, 0]],
            [0, 0, 1, -self.L_com[2, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 2 -> link 3
        # NOTE: offset of -0.00153[m] in x, not sure if necessary
        self.Tl23 = sp.Matrix([
            [sp.cos(sp.pi + self.q[2]), 0, sp.sin(sp.pi + self.q[2]),
             self.L_com[3, 1]*sp.sin(sp.pi + self.q[2])],
            [0, 1, 0, self.L_com[3, 2]],
            [-sp.sin(sp.pi + self.q[2]), 0, sp.cos(sp.pi + self.q[2]),
             self.L_com[3, 1]*sp.cos(sp.pi + self.q[2])],
            [0, 0, 0, 1]])

        # Transform matrix : joint 3 -> link 4
        self.Tl34 = sp.Matrix([
            [sp.cos(sp.pi - self.q[3]), sp.sin(sp.pi - self.q[3]), 0,
             self.L_com[4, 1]*sp.sin(sp.pi + self.q[3])],
            [-sp.sin(sp.pi - self.q[3]), sp.cos(sp.pi - self.q[3]), 0,
             -self.L_com[4, 1]*sp.cos(sp.pi + self.q[3])],
            [0, 0, 1, self.L_com[4, 2]],
            [0, 0, 0, 1]])

        # Transform matrix : joint 4 -> link 5
        self.Tl45 = sp.Matrix([
            [sp.cos(sp.pi - self.q[4]), sp.sin(sp.pi - self.q[4]), 0,
             self.L_com[5, 1]*sp.sin(sp.pi + self.q[4])],
            [-sp.sin(sp.pi - self.q[4]), sp.cos(sp.pi - self.q[4]), 0,
             -self.L_com[5, 1]*sp.cos(sp.pi + self.q[4])],
            [0, 0, 1, self.L_com[5, 2]],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian
        # accounting for which axis is rotated around at each joint
        self.J_orientation = [[0, 0, 1],  # joint 0 rotates around z axis
                              [0, 0, 1],  # joint 1 rotates around z axis
                              [0, 0, 1],  # joint 2 rotates around z axis
                              [0, 0, 1],  # joint 3 rotates around z axis
                              [0, 0, 1],  # joint 4 rotates around z axis
                              [0, 0, 1]]  # joint 5 rotates around z axis

    def _calc_T(self, name):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link
        name string: name of the joint or link, or end-effector
        """

        # ---- Joint Transforms ----

        if name == 'joint0':
            T = self.TorgO
        elif name == 'joint1':
            T = self.TorgO * self.T01a * self.T01b
        elif name == 'joint2':
            T = self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
        elif name == 'joint3':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b)
        elif name == 'joint4':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.T34b * self.T34a)
        elif name == 'joint5':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.T34b * self.T34a * self.T45b
                 * self.T45a)
        elif name == 'EE' or name == 'link6':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.T34b * self.T34a * self.T45b
                 * self.T45a * self.T5EE)
        elif name == 'camera':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.T34b * self.T34a * self.T45b
                 * self.T45a * self.T5EE * self.TEEcamera)

        # ---- COM Transforms ----

        elif name == 'link0':
            T = self.Tlorg0
        elif name == 'link1':
            T = self.TorgO * self.T01a * self.Tl01
        elif name == 'link2':
            T = self.TorgO * self.T01a * self.T01b * self.T12a * self.Tl12
        elif name == 'link3':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.Tl23)
        elif name == 'link4':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.Tl34)
        elif name == 'link5':
            T = (self.TorgO * self.T01a * self.T01b * self.T12a * self.T12b
                 * self.T23a * self.T23b * self.T34b * self.T34a * self.Tl45)

        else:
            raise Exception('Invalid transformation name: %s' % name)

        return T

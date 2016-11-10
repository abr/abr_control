import cloudpickle
import os
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the threelink MapleSim arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(num_joints=3, num_links=3,
                                           robot_name='threelink', **kwargs)

        # for the null space controller, keep arm near these angles
        self.rest_angles = np.array([np.pi/4.0,
                                     np.pi/4.0,
                                     np.pi/4.0])

        # create the inertia matrices for each link of the threelink
        # TODO: confirm that these are actually the right values
        # self._M.append(np.diag([1.93, 1.93, 1.93,
                                # 0.0141, 0.0141, 0.0141]) * 10)  # link0
        # self._M.append(np.diag([0.27, 0.27, 0.27,
                                # 0.012, 0.012, 0.012]) * 10)  # link1
        # self._M.append(np.diag([0.15, 0.15, 0.15,
                                # 0.001, 0.001, 0.001]) * 10)  # link2
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link0
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link0
        self._M.append(np.diag([10.0, 10.0, 10.0,
                                0.0, 0.0, 100.0]))  # link0

        # segment lengths associated with each joint
        # self.L = np.array([0.31, 0.27, 0.15])
        # self.L_links = np.array([0.165, 0.135, 0.075])
        self.L = np.array([2.0, 1.2, 0.7])
        self.L_links = self.L / 2.0

        # transform matrix from origin to joint 0 reference frame
        self.T0org = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 0 to joint 1
        self.T10 = sp.Matrix([
            [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0,
             self.L[0]*sp.cos(self.q[0])],
            [sp.sin(self.q[0]), sp.cos(self.q[0]), 0,
             self.L[0]*sp.sin(self.q[0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1 to joint 2
        self.T21 = sp.Matrix([
            [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0,
             self.L[1]*sp.cos(self.q[1])],
            [sp.sin(self.q[1]), sp.cos(self.q[1]), 0,
             self.L[1]*sp.sin(self.q[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to end-effector
        self.TEE2 = sp.Matrix([
            [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0,
             self.L[2]*sp.cos(self.q[2])],
            [sp.sin(self.q[2]), sp.cos(self.q[2]), 0,
             self.L[2]*sp.sin(self.q[2])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 0 to link 0
        self.Tl00 = sp.Matrix([
            [1, 0, 0, self.L_links[0]*sp.cos(self.q[0])],
            [0, 1, 0, self.L_links[0]*sp.sin(self.q[0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1 to link 1
        self.Tl11 = sp.Matrix([
            [1, 0, 0, self.L_links[1]*sp.cos(self.q[1])],
            [0, 1, 0, self.L_links[1]*sp.sin(self.q[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to link 2
        self.Tl22 = sp.Matrix([
            [1, 0, 0, self.L_links[2]*sp.cos(self.q[2])],
            [0, 1, 0, self.L_links[2]*sp.sin(self.q[2])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[0.0, 0.0, 1.0],  # joint 0 rotates around z axis
                              [0.0, 0.0, 1.0],  # joint 1 rotates around z axis
                              [0.0, 0.0, 1.0]]  # joint 2 rotates around z axis

    def _calc_T(self, name, lambdify=True, regenerate=False):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """
        # check to see if we have our transformation saved in file
        if (regenerate is False and
                os.path.isfile('%s/%s.T' % (self.config_folder, name))):
            Tx = cloudpickle.load(open('%s/%s.T' % (self.config_folder, name),
                                       'rb'))
        else:
            if name == 'joint0':
                T = self.T0org
            elif name == 'link0':
                T = self.T0org * self.Tl00
            elif name == 'joint1':
                T = self.T0org * self.T10
            elif name == 'link1':
                T = self.T0org * self.T10 * self.Tl11
            elif name == 'joint2':
                T = self.T0org * self.T10 * self.T21
            elif name == 'link2':
                T = self.T0org * self.T10 * self.T21 * self.Tl22
            elif name == 'EE':
                T = self.T0org * self.T10 * self.T21 * self.TEE2
            else:
                raise Exception('Invalid transformation name: %s' % name)
            # convert from transform matrix to (x,y,z)
            Tx = sp.simplify(T * self.x)

            # save to file
            cloudpickle.dump(Tx, open('%s/%s.T' % (self.config_folder, name),
                                      'wb'))

        if lambdify is False:
            return Tx
        return sp.lambdify(self.q, Tx)

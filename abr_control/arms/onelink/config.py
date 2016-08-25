import cloudpickle
import os
import numpy as np
import sympy as sp

from .. import robot_config


class robot_config(robot_config.robot_config):
    """ Robot config file for the onelink arm """

    def __init__(self):

        super(robot_config, self).__init__(num_joints=1, num_links=1,
                                           robot_name='onelink')

        self.joint_names = ['shoulder']
        self.rest_angles = [90.0]

        # create the inertia matrices for each link of the ur5
        self._M.append(np.diag([1.0, 1.0, 1.0,
                                0.02, 0.02, 0.02]))  # link0

        # segment lengths associated with each joint
        L = np.array([0.37])

        # transform matrix from origin to joint 0 reference frame
        # link 0 reference frame is the same as joint 0
        self.T0org = sp.Matrix([[sp.cos(self.q[0]), 0, -sp.sin(self.q[0]), 0],
                                [0, 1, 0, 0],
                                [sp.sin(self.q[0]), 0, sp.cos(self.q[0]), 0],
                                [0, 0, 0, 1]])

        # transform matrix from joint 5 to end-effector
        self.Tl00 = sp.Matrix([[0, 0, 0, L[0] / 2],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 1]])

        # transform matrix from joint 5 to end-effector
        self.TEE0 = sp.Matrix([[0, 0, 0, L[0]],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 1]])

        # orientation part of the Jacobian (compensating for orientations)
        self.J_orientation = [[10, 0, 0]]  # joint 0 rotates around z axis

    def _calc_T(self, name, lambdify=True):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        # check to see if we have our transformation saved in file
        if os.path.isfile('%s/%s.T' % (self.config_folder, name)):
            Tx = cloudpickle.load(open('%s/%s.T' % (self.config_folder, name),
                                       'rb'))
        else:
            if name == 'joint0':
                T = self.T0org
            elif name == 'link0':
                T = self.T0org * self.Tl00
            elif name == 'EE':
                T = self.T0org * self.TEE0
            Tx = T * self.x  # to convert from transform matrix to (x,y,z)

            # save to file
            cloudpickle.dump(Tx, open('%s/%s.T' % (self.config_folder, name),
                                      'wb'))

        if lambdify is False:
            return Tx
        return sp.lambdify(self.q, Tx)

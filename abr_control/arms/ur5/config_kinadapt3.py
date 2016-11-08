import cloudpickle
import numpy as np
import os
import sympy as sp

from . import config_neural


class robot_config(config_neural.robot_config):
    """ Robot config file for the UR5 arm for kinematic adaptation.
    All we're doing is replacing the set arm segment lengths, L, with
    variables, and making all of the transforms and Jacobians also
    functions of L.
    """

    def __init__(self, L_init=None, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.adapt = {
            'dimensions': 1,
            'n_neurons': 1000,
            }

        # create variable for an (x, y, z) offset of the tool's end-effector
        self.L_offset_sym = [sp.Symbol('l%i' % ii) for ii in range(3)]
        self.L_offset = np.zeros(3)

        self._T_inv = None

    def J(self, name, q):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Jacobian function
        """
        # check for function in dictionary
        if self._J.get(name, None) is None:
            print('Generating Jacobian function for %s' % name)
            self._J[name] = self._calc_J(
                name=name, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_offset)
        return np.array(self._J[name](*parameters))

    def T(self, name, q):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the T function
        """
        # check for function in dictionary
        if self._T.get(name, None) is None:
            print('Generating transform function for %s' % name)
            # TODO: link0 and joint0 share a transform, but will
            # both have their own transform calculated with this check
            self._T[name] = self._calc_T(
                name, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_offset)
        return self._T[name](*parameters)[:-1].flatten()

    def T_inv(self, q):
        """ Calculates the inverse transform for the end-effector

        q list: set of joint angles to pass in to the T function
        """
        # check for function in dictionary
        if self._T_inv is None:
            print('Generating inverse transform function for end-effector')
            self._T_inv = self._calc_T_inv(
                regenerate=self.regenerate_functions)
        return self._T_inv(*q)

    def _calc_T(self, name, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the transform for a joint or link.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        if name == 'objectEE':
            # highjack this function here and return the one to calculate
            # the end-effector position of the object
            TOBJ5 = sp.Matrix([
                [1, 0, 0, self.L_offset_sym[0]],
                [0, 1, 0, self.L_offset_sym[1]],
                [0, 0, 1, self.L_offset_sym[2]],
                [0, 0, 0, 1]])

            T = (self.T0org * self.T10 * self.T21 * self.T32 * self.T43 *
                 self.T54 * self.TEE5 * TOBJ5)

            Tx = T * self.x
        else:
            # get the transform using the cos and sin matrices defined above
            Tx = super(robot_config, self)._calc_T(name=name, lambdify=False,
                                                   regenerate=regenerate)

        if lambdify is False:
            return Tx
        # return a function of q and self.L_offset
        return sp.lambdify(self.q + self.L_offset_sym, Tx)

    def _calc_T_inv(self, lambdify=True, regenerate=False):
        """ Return the inverse transform matrix, which converts from
        world coordinates into the robot's end-effector reference frame

        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        T = self.T0org * self.T10 * self.T21 * self.T32 * self.T43 * \
            self.T54 * self.TEE5
        rotation_inv = T[:3, :3].T
        translation_inv = -rotation_inv * T[:3, 3]
        T_inv = rotation_inv.row_join(translation_inv).col_join(
            sp.Matrix([[0, 0, 0, 1]]))

        if lambdify is False:
            return T_inv
        # return a function of q and self.L_offset
        return sp.lambdify(self.q, T_inv)

    def _calc_J(self, name, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the Jacobian for a joint or link
        For the neural case we are going to be working with cos(q) and
        sin(q), so need to change the expected input for lambdify.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        print('name in calcJ: ', name)
        # get the transform using the cos and sin matrices defined above
        J = super(robot_config, self)._calc_J(name=name, lambdify=False,
                                              regenerate=regenerate)

        if lambdify is False:
            return J
        # return a function of q and self.L_offset
        return sp.lambdify(self.q + self.L_offset_sym, J)

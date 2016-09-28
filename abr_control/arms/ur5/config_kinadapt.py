import numpy as np
import sympy as sp

import nengo

from . import config

class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm for kinematic adaptation.
    All we're doing is replacing the set arm segment lengths, L, with
    variables, and making all of the transforms and Jacobians also
    functions of L.
    """

    def __init__(self, L_init=None):

        super(robot_config, self).__init__()

        self.adapt = {
            'dimensions': 1,
            'n_neurons': 1000,
            # 'neuron_type': nengo.Direct(),
            }

        self.L = [sp.Symbol('l%i' % ii) for ii in range(self.L.shape[0])]
        # create an estimate of the arm segment lengths
        self.L_hat = L_est if L_init is not None else np.ones(len(self.L))

    def J(self, name, q):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Jacobian function
        """
        # check for function in dictionary
        if self._J.get(name, None) is None:
            print('Generating Jacobian function for %s' % name)
            self._J[name] = self._calc_J(name=name,
                                         regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_hat)
        return np.array(self._J[name](*parameters))

    def Mq(self, q):
        """ Calculates the joint space inertia matrix for the ur5

        q list: set of joint angles to pass in to the Mq function
        """
        # check for function in dictionary
        if self._Mq is None:
            print('Generating inertia matrix function')
            self._Mq = self._calc_Mq(regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_hat)
        return np.array(self._Mq(*parameters))

    def Mq_g(self, q):
        """ Calculates the force of gravity in joint space for the ur5

        q list: set of joint angles to pass in to the Mq_g function
        """
        # check for function in dictionary
        if self._Mq_g is None:
            print('Generating gravity effects function')
            self._Mq_g = self._calc_Mq_g(regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_hat)
        return np.array(self._Mq_g(*parameters)).flatten()

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
            self._T[name] = self._calc_T(name,
                                         regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_hat)
        return self._T[name](*parameters)[:-1].flatten()

    def _calc_T(self, name, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the transform for a joint or link.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """
        # get the transform using the cos and sin matrices defined above
        Tx = super(robot_config, self)._calc_T(name=name, lambdify=False,
                                               regenerate=regenerate)

        if lambdify is False:
            return Tx
        # return a function of cos(q) and sin(q)
        return sp.lambdify(self.q + self.L, Tx)

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
        # get the transform using the cos and sin matrices defined above
        J = super(robot_config, self)._calc_J(name=name, lambdify=False,
                                              regenerate=regenerate)

        if lambdify is False:
            return J
        return sp.lambdify(self.q + self.L, J)

    def _calc_Mq(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the inertia matrix in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """
        # get the transform using the cos and sin matrices defined above
        Mq = super(robot_config, self)._calc_Mq(lambdify=False,
                                                regenerate=regenerate)

        if lambdify is False:
            return Mq
        return sp.lambdify(self.q + self.L, Mq)

    def _calc_Mq_g(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the force of gravity in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """
        # get the transform using the cos and sin matrices defined above
        Mq_g = super(robot_config, self)._calc_Mq_g(lambdify=False,
                                                    regenerate=regenerate)

        if lambdify is False:
            return Mq_g
        return sp.lambdify(self.q + self.L, Mq_g)

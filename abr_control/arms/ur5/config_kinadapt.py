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

    def __init__(self):

        super(robot_config, self).__init__()

        self.adapt = {
            'dimensions': 1,
            'n_neurons': 1000,
            # 'neuron_type': nengo.Direct(),
            }

        self.L = [sp.Symbol('l%i' % ii) for ii in range(self.num_links + 1)]

    def _calc_T(self, name, lambdify=True):
        """ Uses Sympy to generate the transform for a joint or link.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Tx = super(robot_config, self)._calc_T(name=name, lambdify=False)

        if lambdify is False:
            return Tx
        # return a function of cos(q) and sin(q)
        return sp.lambdify(self.q + self.L, Tx)

    def _calc_T(self, name, lambdify=True):
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        T = super(robot_config, self)._calc_T(name=name, lambdify=False)
        print(T)

        if lambdify is False:
            return T
        print((self.q + self.L))
        return sp.lambdify(self.q + self.L, T)


    def _calc_J(self, name, lambdify=True):
        """ Uses Sympy to generate the Jacobian for a joint or link
        For the neural case we are going to be working with cos(q) and
        sin(q), so need to change the expected input for lambdify.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        J = super(robot_config, self)._calc_J(name=name, lambdify=False)

        if lambdify is False:
            return J
        return sp.lambdify(self.q + self.L, J)

    def _calc_Mq(self, lambdify=True):
        """ Uses Sympy to generate the inertia matrix in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Mq = super(robot_config, self)._calc_Mq(lambdify=False)

        if lambdify is False:
            return Mq
        return sp.lambdify(self.q + self.L, Mq)

    def _calc_Mq_g(self, lambdify=True):
        """ Uses Sympy to generate the force of gravity in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Mq_g = super(robot_config, self)._calc_Mq_g(lambdify=False)

        if lambdify is False:
            return Mq_g
        return sp.lambdify(self.q + self.L, Mq_g)

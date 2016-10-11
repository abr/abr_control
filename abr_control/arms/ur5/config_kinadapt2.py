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

        self.L_actual = np.copy(self.L)  # save actual lengths
        self.L = [sp.Symbol('l%i' % ii) for ii in range(L_init.shape[0])]
        # create an estimate of the arm segment lengths
        self.L_hat = L_init if L_init is not None else np.ones(len(self.L))
        self._Y = None

    def J(self, name, q, use_estimate=False):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Jacobian function
        """
        # check for function in dictionary
        if self._J.get(name, None) is None:
            print('Generating Jacobian function for %s' % name)
            self._J[name] = self._calc_J(name=name,
                                         regenerate=self.regenerate_functions)
        parameters = tuple(q)
        if use_estimate is True:
            parameters += tuple(self.L_hat)
        else:
            parameters += tuple(self.L_actual) + (0, 0)
        return np.array(self._J[name](*parameters))

    def Mq(self, q):
        """ Calculates the joint space inertia matrix for the ur5

        q list: set of joint angles to pass in to the Mq function
        """
        # check for function in dictionary
        if self._Mq is None:
            print('Generating inertia matrix function')
            self._Mq = self._calc_Mq(regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_actual) + (0, 0)
        return np.array(self._Mq(*parameters))

    def Mq_g(self, q):
        """ Calculates the force of gravity in joint space for the ur5

        q list: set of joint angles to pass in to the Mq_g function
        """
        # check for function in dictionary
        if self._Mq_g is None:
            print('Generating gravity effects function')
            self._Mq_g = self._calc_Mq_g(regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(self.L_actual) + (0, 0)
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
        parameters = tuple(q)
        if use_estimate is True:
            parameters += tuple(self.L_hat)
        else:
            parameters += tuple(self.L_actual) + (0, 0)
        return self._T[name](*parameters)[:-1].flatten()

    def Y(self, q, dq, name='EE'):
        """ Calculates the basis functions for the end-effector Jacobian
        as a linear function of the arm segment lengths

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the T function
        """
        # check for function
        if self._Y is None:
            print('name in Y: ', name)
            print('Generating basis functions Y for end-effector')
            self._Y = self._calc_Y(name=name,
                                   regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(dq)
        return self._Y(*parameters)

    def _calc_T(self, name, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the transform for a joint or link.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        # transform matrix from joint 5 to end-effector
        self.TOBJ5 = sp.Matrix([
            [0, 0, 0, self.L[7]],
            [0, 0, 0, self.L[8]],
            [0, 0, 0, self.L[9]],
            [0, 0, 0, 1]])

        if name == 'objectEE':
            T = super(robot_config, self)._calc_T(name='link0')
            T = self.T0org * self.T10 * self.T21 * self.T32 * self.T43 * \
                self.T54 * self.TOBJ5
            Tx = T * self.x
        else:
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

        print('name in calcJ: ', name)
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

    def _calc_Y(self, name='EE', lambdify=True, regenerate=False):
        """ Takes the Jacobian for the end-effector and reworks
        it such that all of the self.L terms are pulled out. This
        shuffling of terms is part of kinematic adaptation, such that we
        have basis functions for the learned arm segment parameters.

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        if (regenerate is False and
                os.path.isfile('%s/Y' % self.config_folder)):
            Y = cloudpickle.load(open('%s/Y' % self.config_folder,
                                      'rb'))
        else:
            # get the Jacobians for the end-effector
            J = self._calc_J(name=name, lambdify=False,
                             regenerate=regenerate)[:3]

            Y = sp.Matrix(np.zeros((3, len(self.L))))
            # work through row by row
            for ii, row in enumerate(J):
                row = sp.expand(row)
                # get all the coefficients for each of the L terms
                print('L shape: ', self.L)
                for jj, l in enumerate(self.L):
                    Y[ii, jj] = sp.simplify(row.coeff(l))

            # save to file
            cloudpickle.dump(Y, open('%s/Y' % self.config_folder,
                                     'wb'))

        print('Y : ', Y)
        if lambdify is False:
            return Y
        return sp.lambdify(self.q + self.dq, Y)

import numpy as np
import sympy as sp

import nengo

from . import config

class robot_config(config.robot_config):
    """ Robot config file for the UR5 arm """

    def __init__(self, **kwargs):

        super(robot_config, self).__init__(**kwargs)

        self.CB = {
            'dimensions': self.num_joints * 3,
            'n_neurons': 5000,
            'neuron_type': nengo.Direct(),
            'radius': np.sqrt(self.num_joints * 3),
            }

        self.CB_adapt = {
            'dimensions': self.num_joints  * 3,
            'n_neurons': 1000,
            'neuron_type': nengo.Direct(),
            }

        import itertools
        encoder_set = [[-1,1]]*15
        encoders = list(itertools.product(*encoder_set))
        self.M1 = {
            'dimensions': self.num_joints * 2 + 3,# + 128,
            'n_neurons': 20000,
            # 'neuron_type': nengo.Direct(),
            'radius': np.sqrt(self.num_joints * 2 + 3) / 2.0,
            'encoders': nengo.dists.Choice(encoders)
            }

        self.means = {
            'q': np.array([-0.86, 1.26, -2.12, 0.94, 2.44, 12.10, ]),
            'dq': np.array([-0.11, 0.67, -0.93, 0.04, 2.66, -3.80, ]),
            }

        self.scales = {
            'q': (np.array([-0.41, 1.32, -1.61, 0.99, 2.32, 14.27, ]) -
                  np.array([-0.79, 0.80, -2.05, 0.83, 1.35, 12.32, ])),
            'dq': (np.array([3.32, 3.78, 0.03, 0.00, 6.88, 0.00, ]) -
                   np.array([0.00, -0.09, -3.19, -2.56, -0.07, -11.23, ])),
            }
        # np.array([-2.53, 1.03, -1.72, 0.73, 2.34, 12.19, ])
        # np.array([2.41, 0.56, 0.32, 0.41, 1.28, 1.31, ])
        # np.array([-0.53, -0.09, 0.06, 0.01, 0.23, 0.04, ])
        # np.array([4.49, 2.01, 2.35, 2.27, 5.94, 5.63, ])

        self.cq = [sp.Symbol('cq%i' % ii) for ii in range(self.num_joints)]
        self.sq = [sp.Symbol('sq%i' % ii) for ii in range(self.num_joints)]

        self.subs_list = {
            sp.cos(self.q[0]): self.cq[0],
            sp.cos(self.q[1]): self.cq[1],
            sp.cos(self.q[1] + self.q[2]): self.cq[2],
            sp.cos(self.q[1] + self.q[2] + self.q[3]): self.cq[3],
            sp.cos(self.q[4]): self.cq[4],

            sp.sin(self.q[0]): self.sq[0],
            sp.sin(self.q[1]): self.sq[1],
            sp.sin(self.q[1] + self.q[2]): self.sq[2],
            sp.sin(self.q[1] + self.q[2] + self.q[3]): self.sq[3],
            sp.sin(self.q[4]): self.sq[4]}

    def format_q(self, q):
        return [
            np.cos(q[0]),
            np.cos(q[1]),
            np.cos(q[1] + q[2]),
            np.cos(q[1] + q[2] + q[3]),
            np.cos(q[4]),
            np.cos(q[5]),

            np.sin(q[0]),
            np.sin(q[1]),
            np.sin(q[1] + q[2]),
            np.sin(q[1] + q[2] + q[3]),
            np.sin(q[4]),
            np.sin(q[5])]

    def T(self, name, q, format_q=True):
        """ A wrapper that does formatting of q so
        functions interaction is consistent """
        if format_q is True:
            q = self.format_q(q)
        return super(robot_config, self).T(name, q)

    def _calc_T(self, name, lambdify=True):
        """ Uses Sympy to generate the transform for a joint or link.
        For the neural case we are going to be working with cos(q) and
        sin(q), so need to change the expected input for lambdify.

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Tx = super(robot_config, self)._calc_T(name=name, lambdify=False)

        # substitute in to Tx with cos(q) and sin(q) values
        Tx = Tx.subs(self.subs_list)

        if lambdify is False:
            return Tx
        # return a function of cos(q) and sin(q)
        return sp.lambdify(self.cq + self.sq, Tx)

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

        # substitute in to Tx with cos(q) and sin(q) values
        J = J.subs(self.subs_list)

        if lambdify is False:
            return J
        return sp.lambdify(self.cq + self.sq, J)

    def _calc_Mq(self, lambdify=True):
        """ Uses Sympy to generate the inertia matrix in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Mq = super(robot_config, self)._calc_Mq(lambdify=False)

        # substitute in to Tx with cos(q) and sin(q) values
        Mq = Mq.subs(self.subs_list)

        if lambdify is False:
            return Mq
        return sp.lambdify(self.cq + self.sq, Mq)

    def _calc_Mq_g(self, lambdify=True):
        """ Uses Sympy to generate the force of gravity in
        joint space for the ur5

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # get the transform using the cos and sin matrices defined above
        Mq_g = super(robot_config, self)._calc_Mq_g(lambdify=False)

        # substitute in to Tx with cos(q) and sin(q) values
        Mq_g = Mq_g.subs(self.subs_list)

        if lambdify is False:
            return Mq_g
        return sp.lambdify(self.cq + self.sq, Mq_g)

    def scaledown(self, name, x):
        return (x - self.means[name]) / self.scales[name]

    def scaleup(self, name, x):
        return x * self.scales[name] + self.means[name]

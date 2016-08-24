'''
Copyright (C) 2016 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
import numpy as np
import time

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

from . import osc


class KeepLearningSolver(nengo.solvers.Lstsq):
    """ Loads in weights from a file if they exist,
    otherwise returns random shit. """

    def __init__(self, filename, weights=False):
        super(KeepLearningSolver, self).__init__(weights=weights)
        self.filename = filename

    def __call__(self, A, Y, rng=None, E=None):
        import os
        if os.path.isfile('./%s' % self.filename):
            print('Loading weights from %s' % self.filename)
            tstart = time.time()
            weights = np.load(self.filename)['weights'][-1].T
            info = {'rmses': 'what no stop',
                    'time': time.time() - tstart}
            if (weights.shape[0] != A.shape[1] or
                    weights.shape[1] != Y.shape[1]):
                raise Exception('Stored weights are not ' +
                                'correct shape for connection.')
        else:
            print('No weights file found, ' +
                  'generating with Lstsq solver')
            weights, info = \
                super(KeepLearningSolver, self).__call__(A, Y)

        return weights, info


class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    # # for scaling and biasing input to -1 to 1 range
    # q_bias = np.array([2.66e-03, 1.07, -1.93, 7.37e-01, 2.54, 1.27e+01])
    # q_max_vals = np.array([0.99, 1.63, -1.40, 1.64, 3.368, 14.27])
    # q_min_vals = np.array([-1.74, 0.67, -2.54, -0.68, 1.35, 11.78])
    # q_scale = (q_max_vals - q_min_vals) / 2
    # # for scaling and biasing input to -1 to 1 range
    # dq_bias = np.array([-0.03, 0.20, -0.08, -0.30, 0.51, -0.44])
    # dq_max_vals = np.array([25.44, 7.56, 4.29, 8.62, 15.68, 8.72])
    # dq_min_vals = np.array([-22.74, -3.09, -6.27, -9.65, -13.57, -20.53])
    # dq_scale = (dq_max_vals - dq_min_vals) / 2

    def __init__(self, robot_config, tracking=False, learning_rate=1e-3):
        """
        learning_rate float: controls the speed of neural adaptation
        """

        super(controller, self).__init__(robot_config, tracking)

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        if self.tracking is True:
            self.track_u_adapt = []
            self.track_adapt_input = []

        dim = self.robot_config.num_joints
        nengo_model = nengo.Network()
        with nengo_model:

            def qdq_input(t):
                """ returns q and dq scaled and bias to
                be around -1 to 1 """
                # vals = np.hstack([(self.q - self.q_bias) / self.q_scale,
                #                   (self.dq - self.dq_bias) / self.dq_scale])
                # self.track_adapt_input.append(np.copy(vals))
                # return vals
                return np.hstack([self.q, self.dq])
            qdq_input = nengo.Node(qdq_input, size_out=dim*2)

            def u_input(t):
                """ returns the control signal for training """
                print('self.u: ', self.u)
                return self.u
            u_input = nengo.Node(u_input, size_out=dim)

            def u_adapt_output(t, x):
                """ stores the adaptive output for use in control() """
                self.u_adapt = np.copy(x)
            output = nengo.Node(u_adapt_output, size_in=dim, size_out=0)

            adapt_ens = nengo.Ensemble(n_neurons=500,
                                       dimensions=dim*2,
                                       # TODO: investigate best radius size
                                       radius=np.sqrt(dim*2),
                                       seed=10)

            nengo.Connection(qdq_input, adapt_ens)
            learn_conn = nengo.Connection(
                adapt_ens, output,
                # start with outputting just zero
                function=lambda x: np.zeros(dim),
                learning_rule_type=nengo.PES(learning_rate),)
                # use the weights solver that lets you keep
                # learning from the what's saved to file
                # solver=KeepLearningSolver('weights.npz'))
            nengo.Connection(
                u_input, learn_conn.learning_rule,
                # invert because we're providing error not reward
                transform=-1, synapse=.01)

            self.probe_weights = nengo.Probe(
                learn_conn, 'weights',
                sample_every=.1)  # in seconds

        self.sim = nengo.Simulator(nengo_model)

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        # store local copies to feed in to the adaptive population
        self.q = q
        self.dq = dq
        # generate the osc signal
        self.u = super(controller, self).control(q, dq, target_xyz)
        # run the simulation to generate the adaptive signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        if self.tracking is True:
            self.track_u_adapt.append(np.copy(self.u_adapt))
        self.u += self.u_adapt

        print('u_adapt: ', self.u_adapt)

        # return the sum of the two
        return self.u

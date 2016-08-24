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

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config, tracking=False):  # noqa C901

        self.robot_config = robot_config

        self.kp = 100.0  # proportional gain term
        self.kv = np.sqrt(self.kp)  # derivative gain term

        self.target = np.zeros(3)

        self.tracking = tracking
        if self.tracking is True:
            self.track_u = []
            self.track_u_adapt = []

        dim = self.robot_config.num_joints
        self.model = nengo.Network('REACH', seed=5)
        with self.model:

            # create input nodes
            def get_arm_state(t):
                """ returns q and dq scaled and bias to
                be around -1 to 1 """
                return np.hstack([self.q, self.dq, self.xyz])
            arm_node = nengo.Node(output=get_arm_state, size_out=dim*2 + 3)

            def get_target(t):
                return self.target
            target_node = nengo.Node(output=get_target)

            def set_output(t, x):
                self.u = np.copy(x)
            output_node = nengo.Node(output=set_output, size_in=dim)

            # create neural ensembles
            CB = nengo.Ensemble(**self.robot_config.CB)
            M1 = nengo.Ensemble(**self.robot_config.M1)
            M1_null = nengo.Ensemble(**self.robot_config.M1_null)

            # create summation / output ensembles
            u_relay = nengo.Ensemble(n_neurons=1, dimensions=dim,
                                     neuron_type=nengo.Direct())

            # Connect up M1 ---------------------------------------------------

            def m1_input(t, x):
                return self.robot_config.scaledown('M1', x)
            M1_relay = nengo.Node(output=m1_input,
                                  size_in=9, size_out=dim+3)

            # connect up arm joint angles feedback to M1
            nengo.Connection(arm_node[:dim], M1_relay[:dim], synapse=None)
            # connect up hand xyz feedback to M1
            nengo.Connection(arm_node[dim*2:], M1_relay[dim:], synapse=None,
                             transform=-1)
            # connect up target xyz feedback to M1
            nengo.Connection(target_node, M1_relay[dim:], synapse=None)
            nengo.Connection(M1_relay, M1)

            def gen_Mx(q):
                """ Generate the inertia matrix in operational space """
                Mq = self.robot_config.Mq(q)
                JEE = self.robot_config.J('EE', q)

                # convert the mass compensation into end effector space
                Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
                svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
                # cut off any singular values that could cause control problems
                singularity_thresh = .00025
                for i in range(len(svd_s)):
                    svd_s[i] = 0 if svd_s[i] < singularity_thresh else \
                        1./float(svd_s[i])
                # numpy returns U,S,V.T, so have to transpose both here
                Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

                return Mx

            def gen_u(signal):
                """Generate Jacobian weighted by task-space inertia matrix"""
                # scale things back
                signal = self.robot_config.scaleup('M1', signal)
                q = signal[:dim]
                u = signal[dim:]

                JEE = self.robot_config.J('EE', q)
                Mx = gen_Mx(q)

                u = np.dot(JEE.T, np.dot(Mx, u))
                return u

            nengo.Connection(M1, u_relay, function=gen_u)

            # Set up null control ---------------------------------------------

            print('applying null space control')

            def gen_null_signal(signal):
                """Generate the null space control signal"""

                # calculate our secondary control signal
                q = self.robot_config.scaleup('M1_null', signal[:dim])
                u_null = ((self.robot_config.rest_angles - q) +
                          np.pi) % (np.pi*2) - np.pi

                Mq = self.robot_config.Mq(q=q)
                JEE = self.robot_config.J('EE', q=q)
                Mx = gen_Mx(q=q)

                u_null = np.dot(Mq, 100 * u_null)

                # calculate the null space filter
                Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
                null_filter = np.eye(dim) - np.dot(JEE.T, Jdyn_inv)

                return np.dot(null_filter, u_null).flatten()

            nengo.Connection(arm_node[:dim], M1_null,
                             function=lambda x:
                             self.robot_config.scaledown('M1_null', x))
            nengo.Connection(M1_null, u_relay,
                             function=gen_null_signal)

            # Connect up cerebellum -------------------------------------------

            # connect up arm feedback to Cerebellum
            nengo.Connection(arm_node[:dim*2], CB,
                             function=lambda x:
                             self.robot_config.scaledown('CB', x))

            def gen_Mqdq(signal):
                """Generate inertia compensation signal, np.dot(Mq,dq)"""
                # scale things back
                signal = self.robot_config.scaleup('CB', signal)

                q = signal[:dim]
                dq = signal[dim:dim*2]

                Mq = self.robot_config.Mq(q=q)
                return np.dot(Mq, self.kv * dq).flatten()

            # connect up Cerebellum inertia compensation to summation node
            nengo.Connection(CB, u_relay,
                             function=gen_Mqdq,
                             transform=-1)

            # connect up summation node u_relay to arm
            nengo.Connection(u_relay, output_node, synapse=None)

            # ---------------- set up adaptive bias -------------------

            print('applying adaptive bias...')
            # set up learning, with initial output the zero vector
            # CB_adapt_conn = nengo.Connection(CB, output_node,
            #                                  function=lambda x: np.zeros(dim),
            #                                  learning_rule_type=nengo.PES(
            #                                      learning_rate=1e-3))
            # nengo.Connection(u_relay, CB_adapt_conn.learning_rule,
            #                  transform=-1)

        print('building REACH model...')
        self.sim = nengo.Simulator(self.model, dt=.001)

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        # store local copies to feed in to the adaptive population
        self.q = q
        self.dq = dq
        self.target = target_xyz
        self.xyz = self.robot_config.T('EE', q)
        # run the simulation to generate the control signal
        self.sim.run(time_in_seconds=.005, progress_bar=False)

        if self.tracking is True:
            self.track_u.append(np.copy(self.u))

        # return the sum of the two
        return self.u

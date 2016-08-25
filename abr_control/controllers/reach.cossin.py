""" This version of the REACH uses cos and sin of the joint angles
to keep things in a nice range and address discontinuities. """

import numpy as np

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

from .keeplearningsolver import KeepLearningSolver


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config):  # noqa C901

        self.robot_config = robot_config

        self.kp = 100.0  # proportional gain term
        self.kv = np.sqrt(self.kp)  # derivative gain term

        self.dq = np.zeros(self.robot_config.num_joints)

        self.target = np.zeros(3)

        self.track_input = []

        dim = self.robot_config.num_joints
        self.model = nengo.Network('REACH')#, seed=5)
        self.model.config[nengo.Connection].synapse = None
        with self.model:

            # The reason to use cos(q) and sin(q) is twofold:
            # 1) you get rid of the representation discontinuity
            #    when you wrap around 2*pi
            # 2) you don't go crazy when you move outside 0-2*pi
            #    - NOTE: TODO this could be done with modulo also

            # create input nodes
            def get_feedback(t):
                """ returns q, dq, and target - hand  scaled and
                biased such that each dimension will have a
                range around -1 to 1 """
                self.track_input.append(np.hstack([self.cqsq,
                                                   (self.target - self.xyz) * 5]))
                return np.hstack([
                    self.cqsq,
                    self.robot_config.scaledown('dq', self.dq),
                    (self.target - self.xyz) * 5])

            feedback_node = nengo.Node(output=get_feedback,
                                       size_out=dim*3 + 3)

            def set_output(t, x):
                self.u = np.copy(x)
            output_node = nengo.Node(output=set_output, size_in=dim)

            # TODO: Calculate the symbolic equations for the output u
            # and see what input variables are required, break up the
            # computation into separate populations that only represent
            # the necessary variables instead of this huge 15D space

            # create neural ensembles
            CB = nengo.Ensemble(**self.robot_config.CB)
            CB_adapt = nengo.Ensemble(**self.robot_config.CB_adapt)
            M1 = nengo.Ensemble(**self.robot_config.M1)

            # create relay
            u_relay = nengo.Ensemble(n_neurons=1, dimensions=dim,
                                     neuron_type=nengo.Direct())

            # connect up relay to output
            nengo.Connection(u_relay, output_node, synapse=None)

            # Connect up M1 ---------------------------------------------------

            # connect up arm joint angles feedback to M1
            nengo.Connection(feedback_node[:dim*2], M1[:dim*2])#, synapse=None)
            # connect up hand xyz feedback to M1
            nengo.Connection(feedback_node[dim*3:], M1[dim*2:dim*2+3])#, synapse=None)

            def gen_Mx(cqsq):
                """ Generate the inertia matrix in operational space """
                Mq = self.robot_config.Mq(cqsq)
                JEE = self.robot_config.J('EE', cqsq)

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
                """Generate Jacobian weighted by task-space inertia matrix
                The q that is being passed in here is actually that crazy
                one from above, with cos and sin all over the place """
                cqsq = signal[:dim*2]
                u = signal[dim*2:] / 5.0

                JEE = self.robot_config.J('EE', cqsq)
                Mx = gen_Mx(cqsq)

                u = self.kp * np.dot(JEE.T, np.dot(Mx, u))
                return u

            self.conn_u = nengo.Connection(M1[:dim*2+3], u_relay,
                                           function=gen_u,
                                           synapse=.01)

            # Set up null control ---------------------------------------------

            print('applying null space control')

            def gen_null_signal(signal):
                """Generate the null space control signal"""

                # calculate our secondary control signal
                cqsq = signal[:dim*2]
                q = np.arctan2(cqsq[dim:], cqsq[:dim])
                q_des = (((self.robot_config.rest_angles - q) + np.pi) %
                        (np.pi*2) - np.pi)

                Mq = self.robot_config.Mq(cqsq)
                JEE = self.robot_config.J('EE', cqsq)
                Mx = gen_Mx(cqsq)

                kp = self.kp / 10
                kv = np.sqrt(kp)
                u_null = np.dot(Mq, (kp * q_des - kv * self.dq))

                # calculate the null space filter
                Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
                null_filter = np.eye(dim) - np.dot(JEE.T, Jdyn_inv)

                return np.dot(null_filter, u_null).flatten()

            nengo.Connection(M1, u_relay,
                             function=gen_null_signal,
                             synapse=.01)

            # Connect up cerebellum -------------------------------------------

            # connect up arm feedback to Cerebellum
            nengo.Connection(feedback_node[:dim*3], CB)

            def gen_Mqdq(signal):
                """ Generate inertia compensation signal, np.dot(Mq,dq)
                The q that is being passed in here is actually that crazy
                one from above, with cos and sin all over the place """
                cqsq = signal[:dim*2]
                dq = self.robot_config.scaleup('dq', signal[dim*2:dim*3])

                Mq = self.robot_config.Mq(q=cqsq)
                return np.dot(Mq, self.kv * dq).flatten()

            # connect up CB inertia compensation to relay node
            nengo.Connection(CB, u_relay,
                             function=gen_Mqdq,
                             transform=-1)

            def gen_Mq_g(signal):
                """ Generate the gravity compensation signal
                The q that is being passed in here is actually that crazy
                one from above, with cos and sin all over the place """
                cqsq = signal[:dim*2]
                return self.robot_config.Mq_g(cqsq)

            # connect up CB gravity compensation to arm directly
            # (not to be used as part of training signal for u_adapt)
            nengo.Connection(CB, output_node,
                             function=gen_Mq_g,
                             transform=-1)

            # ---------------- set up adaptive bias -------------------
            print('applying adaptive bias...')
            # set up learning, with initial output the zero vector
            nengo.Connection(feedback_node[:dim*3], CB_adapt,
                             learning_rule_type=nengo.Voja(learning_rate=1e-3))
            CB_adapt_conn = nengo.Connection(CB_adapt, output_node,
                                             function=lambda x: np.zeros(dim),
                                             learning_rule_type=nengo.PES(
                                                 learning_rate=1e-4))
            nengo.Connection(u_relay, CB_adapt_conn.learning_rule,
                             transform=-1)

        print('building REACH model...')
        self.sim = nengo.Simulator(self.model, dt=.001)
        print('building complete...')

        print('error of conn_u: ', self.sim.data[self.conn_u].solver_info)

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        # store local copies to feed in to the adaptive population
        self.cqsq = self.robot_config.format_q(q)
        self.dq = dq
        self.target = target_xyz
        self.xyz = self.robot_config.T('EE', q=q)

        # run the simulation to generate the control signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        # return the sum of the two
        return self.u

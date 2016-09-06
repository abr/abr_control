""" This version of the REACH controller uses q and modulo math to
keep the angles in bounds. """

import itertools
import numpy as np

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

nengo_ocl = None
try:
    import nengo_ocl
except ImportWarning:
    print('Nengo OCL not installed, simulation will be slower.')

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

        dim = self.robot_config.num_joints
        self.model = nengo.Network('REACH', seed=5)
        self.model.config[nengo.Connection].synapse = None
        with self.model:

            # create input nodes
            def get_feedback(t):
                """ returns q, dq, and target - hand_xyz scaled and
                biased such that each dimension will have a range
                around -1 to 1. Also modulo. """
                q = ((self.q + np.pi) % (np.pi*2)) - np.pi
                return np.hstack([self.robot_config.scaledown('q', q),
                                  self.robot_config.scaledown('dq', self.dq),
                                  self.target - self.xyz])
            feedback_node = nengo.Node(output=get_feedback, size_out=dim*2 + 3)

            def set_output(t, x):
                self.u = np.copy(x)
            output_node = nengo.Node(output=set_output, size_in=dim)

            CB = nengo.Ensemble(**self.robot_config.CB)
            # CB_adapt = nengo.Ensemble(**self.robot_config.CB_adapt)
            M1 = nengo.Ensemble(**self.robot_config.M1)

            # create relay
            u_relay = nengo.Ensemble(n_neurons=1, dimensions=dim,
                                     neuron_type=nengo.Direct())

            # connect up relay to output
            nengo.Connection(u_relay, output_node, synapse=None)

            # Connect up M1 ---------------------------------------------------

            # connect up arm joint angles feedback to M1
            nengo.Connection(feedback_node[:dim], M1[:dim])
            # connect up hand xyz feedback to M1
            nengo.Connection(feedback_node[dim*2:], M1[dim:])

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
                q = self.robot_config.scaleup('q', signal[:dim])
                u = signal[dim:]

                JEE = self.robot_config.J('EE', q)
                Mx = gen_Mx(q)

                u = self.kp * np.dot(JEE.T, np.dot(Mx, u))
                return u

            nengo.Connection(M1, u_relay,
                             function=gen_u,
                             synapse=.01)

            # Set up null control ---------------------------------------------

            print('applying null space control')

            def gen_null_signal(signal):
                """Generate the null space control signal"""

                # calculate our secondary control signal
                q = self.robot_config.scaleup('q', signal[:dim])
                q_des = (((self.robot_config.rest_angles - q) + np.pi) %
                         (np.pi*2) - np.pi)

                Mq = self.robot_config.Mq(q=q)
                JEE = self.robot_config.J('EE', q=q)
                Mx = gen_Mx(q=q)

                u_null = np.dot(Mq, (self.kp * q_des - self.kv * self.dq))

                # calculate the null space filter
                Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
                null_filter = np.eye(dim) - np.dot(JEE.T, Jdyn_inv)

                return np.dot(null_filter, u_null).flatten()

            nengo.Connection(M1, u_relay,
                             function=gen_null_signal,
                             synapse=.01)

            # Connect up cerebellum -------------------------------------------

            # connect up arm feedback to Cerebellum
            nengo.Connection(feedback_node[:dim*2], CB)

            def gen_Mqdq(signal):
                """ Generate inertia compensation signal, np.dot(Mq,dq)"""
                # scale things back
                q = self.robot_config.scaleup('q', signal[:dim])
                dq = self.robot_config.scaleup('dq', signal[dim:dim*2])

                Mq = self.robot_config.Mq(q=q)
                return np.dot(Mq, self.kv * dq).flatten()

            # connect up CB inertia compensation to relay node
            nengo.Connection(CB, u_relay,
                             function=gen_Mqdq,
                             transform=-1)

            def gen_Mq_g(signal):
                """ Generate the gravity compensation signal """
                # scale things back
                q = self.robot_config.scaleup('q', signal[:dim])
                return self.robot_config.Mq_g(q)

            # connect up CB gravity compensation to arm directly
            # (not to be used as part of training signal for u_adapt)
            nengo.Connection(CB, output_node,
                             function=gen_Mq_g,
                             transform=-1)

            # # ---------------- set up adaptive bias -------------------
            # print('applying adaptive bias...')
            # # set up learning, with initial output the zero vector
            # nengo.Connection(feedback_node[:dim*2], CB_adapt,
            #                  learning_rule_type=nengo.Voja(learning_rate=1e-3))
            # CB_adapt_conn = nengo.Connection(CB_adapt, output_node,
            #                                  function=lambda x: np.zeros(dim),
            #                                  learning_rule_type=nengo.PES(
            #                                      learning_rate=1e-4))
            # nengo.Connection(u_relay, CB_adapt_conn.learning_rule,
            #                  transform=-1)

        print('building REACH model...')
        if nengo_ocl is not None:
            self.sim = nengo_ocl.Simulator(self.model, dt=.001)
        else:
            self.sim = nengo.Simulator(self.model, dt=.001)
        print('building complete...')

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
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        # return the sum of the two
        return self.u

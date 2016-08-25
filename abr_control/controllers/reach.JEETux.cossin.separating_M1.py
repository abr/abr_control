""" This version of the REACH implements a basic np.dot(JEE.T, ux) control
with each dimension of the resulting torque signal calculated by a separate
population that only represents the dimensions necessary for the calculation.
It's more accurate in this calculation, but without the inertia matrix
incorporated the gain scaling needs to be fixed.

TODO: have kp be a matrix with different gain values for each joint """

import numpy as np
import sympy as sp

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
        self.model = nengo.Network('REACH', seed=5)
        self.model.config[nengo.Connection].synapse = None
        self.model.config[nengo.Ensemble].neuron_type = nengo.Direct()
        with self.model:

            # TODO: The reason that genius past me used cos(q) and sin(q)
            # was twofold.
            # 1) you get rid of insane discontinuity when you wrap around 2*pi
            # 2) you don't go crazy when you move outside 0-2*pi

            # create input nodes
            def get_feedback(t):
                """ returns q, dq, and target - hand  scaled and
                biased such that each dimension will have a
                range around -1 to 1 """
                self.track_input.append(np.hstack(
                    [self.cqsq, (self.target - self.xyz) * 5]))
                return np.hstack([
                    self.cqsq,
                    self.robot_config.scaledown('dq', self.dq),
                    (self.target - self.xyz) * 5])

            feedback_node = nengo.Node(output=get_feedback,
                                       size_out=dim*3 + 3)

            def set_output(t, x):
                self.u = np.copy(x)
            output_node = nengo.Node(output=set_output, size_in=dim)

            # create neural ensembles
            CB = nengo.Ensemble(**self.robot_config.CB)
            CB_adapt = nengo.Ensemble(**self.robot_config.CB_adapt)

            n_neurons = 1000
            # (cq0, cq3, cq4, sq0, sq1, sq2, sq3, sq4, u0)
            M1_j00 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=9,
                                    radius=3)
            # (cq1, cq2, cq3, sq0, sq3, sq4, u0)
            M1_j01 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=7,
                                    radius=np.sqrt(7))
            # (cq2, cq3, sq0, sq3, sq4, u0)
            M1_j02 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=6,
                                    radius=np.sqrt(6))
            # (cq3, sq0, sq3, sq4, u0)
            M1_j03 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=5,
                                    radius=np.sqrt(5))
            # (cq0, cq3, cq4, sq0, sq4, u0)
            M1_j04 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=6,
                                    radius=np.sqrt(6))
            # M1_j05 = 0

            # (cq0, cq3, cq4, sq0, sq1, sq2, sq3, sq4, u0)
            M1_j10 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=9,
                                    radius=3)
            # (cq0, cq1, cq2, cq3, sq3, sq4, u1)
            M1_j11 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=7,
                                    radius=np.sqrt(7))
            # (cq0, cq2, cq3, sq3, sq4, u1)
            M1_j12 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=6,
                                    radius=np.sqrt(6))
            # (cq0, cq3, sq3, sq4, u1)
            M1_j13 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=5,
                                    radius=np.sqrt(5))
            # (cq0, cq3, cq4, sq0, sq4, u1)
            M1_j14 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=6,
                                    radius=np.sqrt(6))
            # M1_j15 = 0

            # M1_j20 = 0

            # (cq3, sq1, sq2, sq3, sq4, u2)
            M1_j21 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=6,
                                    radius=np.sqrt(6))
            # (cq3, sq2, sq3, sq4, u2)
            M1_j22 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=5,
                                    radius=np.sqrt(5))
            # (cq3, sq3, sq4, u2)
            M1_j23 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=4,
                                    radius=np.sqrt(4))
            # (cq4, sq3, u2)
            M1_j24 = nengo.Ensemble(n_neurons=n_neurons,
                                    dimensions=3,
                                    radius=np.sqrt(3))
            # M1_j25 = 0

            # create relay
            u_relay = nengo.Ensemble(n_neurons=1, dimensions=dim,
                                     neuron_type=nengo.Direct())

            # connect up relay to output
            nengo.Connection(u_relay, output_node, synapse=None)

            # Connect up M1 ---------------------------------------------------

            # the calculation of each dimension of np.dot(JEE.T, u_x) has been
            # separated out into different populations based on what's input
            # is required, to increase accuracy.

            JEE = self.robot_config._calc_J('EE', lambdify=False)
            cq = self.robot_config.cq
            sq = self.robot_config.sq
            u = [sp.Symbol('u%i' % ii) for ii in range(3)]

            # TODO: what index is sq0?? !!-------------------------------------

            # (cq0, cq3, cq4, sq0, sq1, sq2, sq3, sq4, u0)
            nengo.Connection(feedback_node[[0, 3, 4, 5, 6, 7, 8, 9, -3]],
                             M1_j00)
            JEE_00_func = sp.lambdify([cq[0], cq[3], cq[4],
                                       sq[0], sq[1], sq[2], sq[3], sq[4],
                                       u[0]],
                                      JEE[0, 0] * self.kp * u[0])
            nengo.Connection(M1_j00, u_relay[0],
                             function=lambda x: JEE_00_func(*x))

            # (cq1, cq2, cq3, sq0, sq3, sq4, u0)
            nengo.Connection(feedback_node[[1, 2, 3, 5, 8, 9, -3]],
                             M1_j01)
            JEE_01_func = sp.lambdify([cq[1], cq[2], cq[3],
                                       sq[0], sq[3], sq[4],
                                       u[0]],
                                      JEE[0, 1] * self.kp * u[0])
            nengo.Connection(M1_j01, u_relay[0],
                             function=lambda x: JEE_01_func(*x))

            # (cq2, cq3, sq0, sq3, sq4, u0)
            nengo.Connection(feedback_node[[2, 3, 5, 8, 9, -3]],
                             M1_j02)
            JEE_02_func = sp.lambdify([cq[2], cq[3],
                                       sq[0], sq[3], sq[4],
                                       u[0]],
                                      JEE[0, 2] * self.kp * u[0])
            nengo.Connection(M1_j02, u_relay[0],
                             function=lambda x: JEE_02_func(*x))

            # (cq3, sq0, sq3, sq4, u0)
            nengo.Connection(feedback_node[[3, 5, 8, 9, -3]],
                             M1_j03)
            JEE_03_func = sp.lambdify([cq[3],
                                       sq[0], sq[3], sq[4],
                                       u[0]],
                                      JEE[0, 3] * self.kp * u[0])
            nengo.Connection(M1_j03, u_relay[0],
                             function=lambda x: JEE_03_func(*x))

            # (cq0, cq3, cq4, sq0, sq4, u0)
            nengo.Connection(feedback_node[[0, 3, 4, 5, 9, -3]],
                             M1_j04)
            JEE_04_func = sp.lambdify([cq[0], cq[3], cq[4],
                                       sq[0], sq[4],
                                       u[0]],
                                      JEE[0, 4] * self.kp * u[0])
            nengo.Connection(M1_j04, u_relay[0],
                             function=lambda x: JEE_04_func(*x))

            # (cq0, cq3, cq4, sq0, sq1, sq2, sq3, sq4, u1)
            nengo.Connection(feedback_node[[0, 3, 4, 5, 6, 7, 8, 9, -2]],
                             M1_j10)
            JEE_10_func = sp.lambdify([cq[0], cq[3], cq[4],
                                       sq[0], sq[1], sq[2], sq[3], sq[4],
                                       u[1]],
                                      JEE[1, 0] * self.kp * u[1])
            nengo.Connection(M1_j10, u_relay[1],
                             function=lambda x: JEE_10_func(*x))

            # (cq0, cq1, cq2, cq3, sq3, sq4, u1)
            nengo.Connection(feedback_node[[0, 1, 2, 3, 7, 8, -2]],
                             M1_j11)
            JEE_11_func = sp.lambdify([cq[0], cq[1], cq[2], cq[3],
                                       sq[3], sq[4],
                                       u[1]],
                                      JEE[1, 1] * self.kp * u[1])
            nengo.Connection(M1_j11, u_relay[1],
                             function=lambda x: JEE_11_func(*x))

            # (cq0, cq2, cq3, sq3, sq4, u1)
            nengo.Connection(feedback_node[[0, 2, 3, 7, 8, -2]],
                             M1_j12)
            JEE_12_func = sp.lambdify([cq[0], cq[2], cq[3],
                                       sq[3], sq[4],
                                       u[1]],
                                      JEE[1, 2] * self.kp * u[1])
            nengo.Connection(M1_j12, u_relay[1],
                             function=lambda x: JEE_12_func(*x))

            # (cq0, cq3, sq3, sq4, u1)
            nengo.Connection(feedback_node[[0, 3, 7, 8, -2]],
                             M1_j13)
            JEE_13_func = sp.lambdify([cq[0], cq[3],
                                       sq[3], sq[4],
                                       u[1]],
                                      JEE[1, 3] * self.kp * u[1])
            nengo.Connection(M1_j13, u_relay[1],
                             function=lambda x: JEE_13_func(*x))

            # (cq0, cq3, cq4, sq0, sq4, u1)
            nengo.Connection(feedback_node[[0, 3, 4, 5, 9, -2]],
                             M1_j14)
            JEE_14_func = sp.lambdify([cq[0], cq[3], cq[4],
                                       sq[0], sq[4],
                                       u[1]],
                                      JEE[1, 4] * self.kp * u[1])
            nengo.Connection(M1_j14, u_relay[1],
                             function=lambda x: JEE_14_func(*x))

            # (cq3, sq1, sq2, sq3, sq4, u2)
            nengo.Connection(feedback_node[[3, 6, 7, 8, 9, -1]],
                             M1_j21)
            JEE_21_func = sp.lambdify([cq[3],
                                       sq[1], sq[2], sq[3], sq[4],
                                       u[2]],
                                      JEE[2, 1] * self.kp * u[2])
            nengo.Connection(M1_j21, u_relay[2],
                             function=lambda x: JEE_21_func(*x))

            # (cq3, sq2, sq3, sq4, u2)
            nengo.Connection(feedback_node[[3, 6, 7, 8, -1]],
                             M1_j22)
            JEE_22_func = sp.lambdify([cq[3],
                                       sq[2], sq[3], sq[4],
                                       u[2]],
                                      JEE[2, 2] * self.kp * u[2])
            nengo.Connection(M1_j22, u_relay[2],
                             function=lambda x: JEE_22_func(*x))

            # (cq3, sq3, sq4, u2)
            nengo.Connection(feedback_node[[3, 6, 7, -1]],
                             M1_j23)
            JEE_23_func = sp.lambdify([cq[3],
                                       sq[3], sq[4],
                                       u[2]],
                                      JEE[2, 3] * self.kp * u[2])
            nengo.Connection(M1_j23, u_relay[2],
                             function=lambda x: JEE_23_func(*x))

            # (cq4, sq3)
            nengo.Connection(feedback_node[[4, 7, -1]],
                             M1_j24)
            JEE_24_func = sp.lambdify([cq[4],
                                       sq[3],
                                       u[2]],
                                      JEE[2, 4] * self.kp * u[2])
            nengo.Connection(M1_j24, u_relay[2],
                             function=lambda x: JEE_24_func(*x))

            # TODO: figure out how effective the null control
            # is with 15D state

            # def gen_Mx(cqsq):
            #     """ Generate the inertia matrix in operational space """
            #     Mq = self.robot_config.Mq(cqsq)
            #     JEE = self.robot_config.J('EE', cqsq)
            #
            #     # convert the mass compensation into end effector space
            #     Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
            #     svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
            #     # cut off any singular values that could cause control problems
            #     singularity_thresh = .00025
            #     for i in range(len(svd_s)):
            #         svd_s[i] = 0 if svd_s[i] < singularity_thresh else \
            #             1./float(svd_s[i])
            #     # numpy returns U,S,V.T, so have to transpose both here
            #     Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))
            #
            #     return Mx
            #
            # def gen_u(signal):
            #     """Generate Jacobian weighted by task-space inertia matrix
            #     The q that is being passed in here is actually that crazy
            #     one from above, with cos and sin all over the place """
            #     cqsq = signal[:dim*2]
            #     u = signal[dim*2:] / 5.0
            #
            #     JEE = self.robot_config.J('EE', cqsq)
            #     Mx = gen_Mx(cqsq)
            #
            #     u = self.kp * np.dot(JEE.T, np.dot(Mx, u))
            #     return u
            #
            # nengo.Connection(M1, u_relay,
            #                  function=gen_u,
            #                  synapse=.01)
            #
            # # Set up null control ---------------------------------------------
            #
            # print('applying null space control')
            #
            # def gen_null_signal(signal):
            #     """Generate the null space control signal"""
            #
            #     # calculate our secondary control signal
            #     cqsq = signal[:dim*2]
            #     q = np.arctan2(cqsq[dim:], cqsq[:dim])
            #     q_des = (((self.robot_config.rest_angles - q) + np.pi) %
            #             (np.pi*2) - np.pi)
            #
            #     Mq = self.robot_config.Mq(cqsq)
            #     JEE = self.robot_config.J('EE', cqsq)
            #     Mx = gen_Mx(cqsq)
            #
            #     kp = self.kp / 10
            #     kv = np.sqrt(kp)
            #     u_null = np.dot(Mq, (kp * q_des - kv * self.dq))
            #
            #     # calculate the null space filter
            #     Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
            #     null_filter = np.eye(dim) - np.dot(JEE.T, Jdyn_inv)
            #
            #     return np.dot(null_filter, u_null).flatten()
            #
            # nengo.Connection(M1, u_relay,
            #                  function=gen_null_signal,
            #                  synapse=.01)

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
